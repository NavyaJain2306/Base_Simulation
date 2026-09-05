"""
Workbench Planner Node -- SML EAI Workshop
----------------------------------------------------------
Assumes all required blocks are already ON the workbench table
(delivered by the warehouse robot).

Subscribes to:
  /planned_task           (sml_messages/Task)       -- which product to build + order_type
  /workbench_block_poses  (std_msgs/String, JSON)   -- block positions on workbench table
                                                      FORMAT (temporary, to update later):
                                                      [{"block_type": 3, "x": 0.32, "y": 0.15, "z": 0.0}, ...]

Publishes to:
  /arm/command            (std_msgs/String, JSON)   -- pick/place commands to workbench arm
                                                      FORMAT:
                                                      {"action": "pick"|"place",
                                                       "block_type": 3,
                                                       "x": 0.32, "y": 0.15, "z": 0.0,
                                                       "label": "pick block_type3 from table"}

Logic (PRODUCE):
  1. Receive /planned_task -> decode product_id -> get required block sequence bottom->top
  2. Receive /workbench_block_poses -> know where each block_type is on table
  3. Build UP problem: pick each block from its table position, place at stack position
     in correct order (bottom first, enforced by stacking precondition)
  4. Solve -> publish arm commands step by step

Logic (RECYCLE):
  Reverse: unstack blocks top->bottom, place back on table at their original positions.

Dependencies:
  pip install unified-planning up-pyperplan
  ROS2 with sml_messages built in workspace
"""

import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sml_messages.msg import Task, Order

import unified_planning as up
from unified_planning.shortcuts import (
    BoolType, UserType, Problem, Fluent,
    InstantaneousAction, OneshotPlanner
)
from unified_planning.model import Object

# Suppress UP credits noise
up.environment.get_environment().credits_stream = None


# ---------------------------------------------------------------------------
# Fixed stack positions on the workbench (x, y, z)
# z increments by BLOCK_HEIGHT for each layer
# Adjust x, y to match your physical workbench centre point
# ---------------------------------------------------------------------------
STACK_BASE_X   = 0.50
STACK_BASE_Y   = 0.50
BLOCK_HEIGHT   = 0.05   # metres -- height of one block

def stack_position(layer_index: int) -> tuple:
    """layer_index 0 = bottom, 1 = middle, 2 = top, etc."""
    return (STACK_BASE_X, STACK_BASE_Y, layer_index * BLOCK_HEIGHT)


# ---------------------------------------------------------------------------
# Decode product_id into ordered block type list (bottom -> top)
# e.g. 341 -> [3, 4, 1]
# ---------------------------------------------------------------------------
def decode_product(product_id: int) -> list[int]:
    return [int(d) for d in str(product_id)]


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------
class WorkbenchPlannerNode(Node):

    def __init__(self):
        super().__init__('workbench_planner_node')

        # Subscribers
        self.task_sub = self.create_subscription(
            Task, '/planned_task', self._on_task, 10
        )
        self.poses_sub = self.create_subscription(
            String, '/workbench_block_poses', self._on_block_poses, 10
        )

        # Publisher
        self.arm_pub = self.create_publisher(String, '/arm/command', 10)

        # Internal state
        self.current_order: Order | None = None
        self.block_poses: list[dict] = []   # [{"block_type": 3, "x":..., "y":..., "z":...}]
        self.ready_to_plan = False

        self.get_logger().info('WorkbenchPlannerNode started.')

    # -----------------------------------------------------------------------
    # Callback: new planned task from Task Planner
    # -----------------------------------------------------------------------
    def _on_task(self, msg: Task):
        if not msg.order_list:
            self.get_logger().warn('Received empty order list.')
            return

        # Take the first order (task planner already prioritised)
        self.current_order = msg.order_list[0]
        self.get_logger().info(
            f'Received order: product_id={self.current_order.product_id}, '
            f'order_type={self.current_order.order_type}'
        )
        self._try_plan()

    # -----------------------------------------------------------------------
    # Callback: block positions on workbench table from perception
    # Temporary format: JSON string list of {block_type, x, y, z}
    # -----------------------------------------------------------------------
    def _on_block_poses(self, msg: String):
        try:
            self.block_poses = json.loads(msg.data)
            self.get_logger().info(
                f'Received {len(self.block_poses)} block poses on workbench.'
            )
            self._try_plan()
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse block poses JSON: {e}')

    # -----------------------------------------------------------------------
    # Attempt planning only when both order and block poses are available
    # -----------------------------------------------------------------------
    def _try_plan(self):
        if self.current_order is None:
            self.get_logger().info('Waiting for /planned_task ...')
            return
        if not self.block_poses:
            self.get_logger().info('Waiting for /workbench_block_poses ...')
            return

        required_sequence = decode_product(self.current_order.product_id)
        self.get_logger().info(f'Required block sequence (bottom->top): {required_sequence}')

        # Check all required block types are present on table
        available_types = [b['block_type'] for b in self.block_poses]
        for bt in required_sequence:
            if bt not in available_types:
                self.get_logger().error(
                    f'Block type {bt} not found on workbench table. '
                    f'Available: {available_types}'
                )
                return

        # Build pose lookup: block_type -> {x, y, z}
        # (assumes one of each type on table -- extend if duplicates needed)
        pose_map: dict[int, dict] = {}
        for b in self.block_poses:
            pose_map[b['block_type']] = {'x': b['x'], 'y': b['y'], 'z': b['z']}

        if self.current_order.order_type == Order.OT_PRODUCE:
            plan = self._solve_produce(required_sequence, pose_map)
        elif self.current_order.order_type == Order.OT_RECYCLE:
            plan = self._solve_recycle(required_sequence, pose_map)
        else:
            self.get_logger().error(
                f'Unknown order_type: {self.current_order.order_type}'
            )
            return

        if plan is None:
            self.get_logger().error('UP solver failed to find a plan.')
            return

        self.get_logger().info(f'Plan found ({len(plan)} actions):')
        for i, cmd in enumerate(plan):
            self.get_logger().info(f'  Step {i+1}: {cmd["label"]}')
            self._publish_command(cmd)

        # Reset state after publishing
        self.current_order = None
        self.block_poses = []

    # -----------------------------------------------------------------------
    # PRODUCE: pick each block from table, place on stack in correct order
    #
    # UP Problem:
    #   Fluents:
    #     block_on_table(block) - block is at its table position
    #     block_stacked(block) - block has been placed in stack
    #     arm_free - single arm mutex
    #     layer_ready(layer) - layer below is already stacked (ordering)
    #
    #   Actions:
    #     pick(block) -> pre: block_on_table + arm_free
    #                    eff: arm holding block, not on table
    #     place(block,layer) -> pre: arm_holding + layer_ready(this_layer)
    #                           eff: block_stacked, layer_ready(next_layer), arm_free
    #
    #   Goal: all blocks stacked
    # -----------------------------------------------------------------------
    def _solve_produce(
        self,
        sequence: list[int],        # block types bottom -> top
        pose_map: dict[int, dict]   # block_type -> {x, y, z}
    ) -> list[dict] | None:

        Block = UserType('Block')
        Layer = UserType('Layer')

        block_on_table = Fluent('block_on_table', BoolType(), block=Block)
        arm_holding = Fluent('arm_holding', BoolType(), block=Block)
        block_stacked = Fluent('block_stacked', BoolType(), block=Block)
        arm_free = Fluent('arm_free', BoolType())
        layer_ready = Fluent('layer_ready', BoolType(), layer=Layer)

        problem = Problem('workbench_produce')
        for f in [block_on_table, arm_holding, block_stacked, arm_free, layer_ready]:
            problem.add_fluent(f, default_initial_value=False)

        # Actions
        # pick: from table
        pick = InstantaneousAction('pick', block=Block)
        b = pick.parameter('block')
        pick.add_precondition(block_on_table(b))
        pick.add_precondition(arm_free)
        pick.add_effect(arm_holding(b), True)
        pick.add_effect(block_on_table(b), False)
        pick.add_effect(arm_free, False)
        problem.add_action(pick)

        # place: onto stack at correct layer
        place = InstantaneousAction('place', block=Block, layer=Layer)
        b2 = place.parameter('block')
        l = place.parameter('layer')
        place.add_precondition(arm_holding(b2))
        place.add_precondition(layer_ready(l))
        place.add_effect(block_stacked(b2), True)
        place.add_effect(arm_holding(b2), False)
        place.add_effect(arm_free, True)
        problem.add_action(place)

        # Objects
        block_objs = {}
        layer_objs = {}
        for i, bt in enumerate(sequence):
            name = f'block_type{bt}'
            if name not in block_objs:
                obj = Object(name, Block)
                problem.add_object(obj)
                block_objs[name] = obj
                problem.set_initial_value(block_on_table(obj), True)

            layer_name = f'layer_{i}'
            layer_obj  = Object(layer_name, Layer)
            problem.add_object(layer_obj)
            layer_objs[i] = layer_obj

        # Initial state
        problem.set_initial_value(arm_free, True)
        # Layer 0 (bottom) is always ready to receive
        problem.set_initial_value(layer_ready(layer_objs[0]), True)

        # Per-layer place actions -- each action is locked to EXACTLY one block type.
        # This prevents pyperplan from placing the wrong block at a layer.
        # No block parameter-- the specific block object is hardcoded in precondition.
        problem.actions.clear()
        problem.add_action(pick)   # pick stays generic

        for i, bt in enumerate(sequence):
            block_obj  = block_objs[f'block_type{bt}']
            # place_layerN_typeM: no parameters, fully grounded
            place_i = InstantaneousAction(f'place_layer{i}_type{bt}')
            place_i.add_precondition(arm_holding(block_obj))     # ONLY this block
            place_i.add_precondition(layer_ready(layer_objs[i])) # ONLY when layer ready
            place_i.add_effect(block_stacked(block_obj), True)
            place_i.add_effect(arm_holding(block_obj), False)
            place_i.add_effect(arm_free, True)
            place_i.add_effect(layer_ready(layer_objs[i]), False)
            if i + 1 < len(sequence):
                place_i.add_effect(layer_ready(layer_objs[i + 1]), True)
            problem.add_action(place_i)

        # Goals: all blocks stacked
        for bt in sequence:
            problem.add_goal(block_stacked(block_objs[f'block_type{bt}']))

        # Solve
        up_plan = self._run_solver(problem)
        if up_plan is None:
            return None

        # Convert UP actions -> arm commands
        return self._produce_actions_to_commands(up_plan, sequence, pose_map)

    # -----------------------------------------------------------------------
    # RECYCLE: unstack blocks top->bottom, return to table positions
    #
    # Reverse of produce -- top block first, then work downward.
    # -----------------------------------------------------------------------
    def _solve_recycle(
        self,
        sequence: list[int],
        pose_map: dict[int, dict]
    ) -> list[dict] | None:

        Block = UserType('Block')
        Layer = UserType('Layer')

        block_stacked = Fluent('block_stacked', BoolType(), block=Block)
        arm_holding = Fluent('arm_holding', BoolType(), block=Block)
        block_on_table = Fluent('block_on_table', BoolType(), block=Block)
        arm_free = Fluent('arm_free', BoolType())
        layer_clear = Fluent('layer_clear', BoolType(), layer=Layer)

        problem = Problem('workbench_recycle')
        for f in [block_stacked, arm_holding, block_on_table, arm_free, layer_clear]:
            problem.add_fluent(f, default_initial_value=False)

        # Objects
        block_objs = {}
        layer_objs = {}
        for i, bt in enumerate(sequence):
            name = f'block_type{bt}'
            if name not in block_objs:
                obj = Object(name, Block)
                problem.add_object(obj)
                block_objs[name] = obj
                problem.set_initial_value(block_stacked(obj), True)

            layer_obj = Object(f'layer_{i}', Layer)
            problem.add_object(layer_obj)
            layer_objs[i] = layer_obj

        problem.set_initial_value(arm_free, True)
        # Top layer is clear initially (nothing above it)
        top = len(sequence) - 1
        problem.set_initial_value(layer_clear(layer_objs[top]), True)

        # Per-layer unstack actions -- fully grounded (no block parameter)
        # Each action is locked to the EXACT block at that layer.
        # return_to_table also grounded per block for same reason.
        for i in reversed(range(len(sequence))):
            bt = sequence[i]
            block_obj = block_objs[f'block_type{bt}']

            # unstack_layerN_typeM -- no parameters
            unstack_i = InstantaneousAction(f'unstack_layer{i}_type{bt}')
            unstack_i.add_precondition(block_stacked(block_obj))
            unstack_i.add_precondition(arm_free)
            unstack_i.add_precondition(layer_clear(layer_objs[i]))
            unstack_i.add_effect(arm_holding(block_obj), True)
            unstack_i.add_effect(block_stacked(block_obj), False)
            unstack_i.add_effect(arm_free, False)
            unstack_i.add_effect(layer_clear(layer_objs[i]), False)
            if i - 1 >= 0:
                unstack_i.add_effect(layer_clear(layer_objs[i - 1]), True)
            problem.add_action(unstack_i)

            # return_to_table_typeM -- grounded per block, no parameters
            return_i = InstantaneousAction(f'return_to_table_type{bt}')
            return_i.add_precondition(arm_holding(block_obj))
            return_i.add_effect(block_on_table(block_obj), True)
            return_i.add_effect(arm_holding(block_obj), False)
            return_i.add_effect(arm_free, True)
            problem.add_action(return_i)

        # Goals: all blocks back on table
        for bt in sequence:
            problem.add_goal(block_on_table(block_objs[f'block_type{bt}']))

        # Build lookup: action_name -> (layer_idx, block_type)
        # This avoids any string parsing bugs in command converter
        action_meta = {}
        for i, bt in enumerate(sequence):
            action_meta[f'unstack_layer{i}_type{bt}'] = ('unstack', i, bt)
            action_meta[f'return_to_table_type{bt}']  = ('return',  i, bt)

        # Solve
        up_plan = self._run_solver(problem)
        if up_plan is None:
            return None

        return self._recycle_actions_to_commands(up_plan, action_meta, pose_map)

    # -----------------------------------------------------------------------
    # Run UP solver (pyperplan)
    # -----------------------------------------------------------------------
    def _run_solver(self, problem: Problem) -> list | None:
        try:
            with OneshotPlanner(name='pyperplan') as planner:
                result = planner.solve(problem)
                if result.plan is not None:
                    return list(result.plan.actions)
                else:
                    self.get_logger().error('Solver returned no plan.')
                    return None
        except Exception as e:
            self.get_logger().error(f'Solver error: {e}')
            return None

    # -----------------------------------------------------------------------
    # Convert PRODUCE UP actions -> arm command dicts
    # -----------------------------------------------------------------------
    def _produce_actions_to_commands(
        self,
        up_actions: list,
        sequence: list[int],
        pose_map: dict[int, dict]
    ) -> list[dict]:
        commands = []

        for action_instance in up_actions:
            name   = action_instance.action.name
            params = [str(p) for p in action_instance.actual_parameters]

            if name == 'pick':
                # params[0] = block_typeN
                bt = int(params[0].replace('block_type', ''))
                pos = pose_map[bt]
                commands.append({
                    'action': 'pick',
                    'block_type': bt,
                    'x': pos['x'], 'y': pos['y'], 'z': pos['z'],
                    'label': f'pick block_type{bt} from table at '
                             f'({pos["x"]:.2f},{pos["y"]:.2f},{pos["z"]:.2f})'
                })

            elif name.startswith('place_layer'):
                # name format: place_layerN_typeM  (no params -- fully grounded)
                parts = name.split('_')                         # ['place', 'layerN', 'typeM']
                layer_idx = int(parts[1].replace('layer', ''))
                bt = int(parts[2].replace('type', ''))
                params = []                                     # no parameters in grounded action
                target = stack_position(layer_idx)
                commands.append({
                    'action': 'place',
                    'block_type':  bt,
                    'x': target[0], 'y': target[1], 'z': target[2],
                    'label': f'place block_type{bt} at stack layer {layer_idx} '
                             f'({target[0]:.2f},{target[1]:.2f},{target[2]:.2f})'
                })

        return commands

    # -----------------------------------------------------------------------
    # Convert RECYCLE UP actions -> arm command dicts
    # -----------------------------------------------------------------------
    def _recycle_actions_to_commands(
        self,
        up_actions: list,
        action_meta: dict,   # action_name -> ('unstack'|'return', layer_idx, block_type)
        pose_map: dict[int, dict]
    ) -> list[dict]:
        commands = []

        for action_instance in up_actions:
            name = action_instance.action.name
            if name not in action_meta:
                continue
            kind, layer_idx, bt = action_meta[name]

            if kind == 'unstack':
                src = stack_position(layer_idx)
                commands.append({
                    'action':     'pick',
                    'block_type': bt,
                    'x': src[0], 'y': src[1], 'z': src[2],
                    'label': f'unstack block_type{bt} from layer {layer_idx} '
                             f'({src[0]:.2f},{src[1]:.2f},{src[2]:.2f})'
                })
            elif kind == 'return':
                pos = pose_map[bt]
                commands.append({
                    'action':     'place',
                    'block_type': bt,
                    'x': pos['x'], 'y': pos['y'], 'z': pos['z'],
                    'label': f'return block_type{bt} to table at '
                             f'({pos["x"]:.2f},{pos["y"]:.2f},{pos["z"]:.2f})'
                })

        return commands

    # -----------------------------------------------------------------------
    # Publish one arm command as JSON string
    # -----------------------------------------------------------------------
    def _publish_command(self, cmd: dict):
        msg = String()
        msg.data = json.dumps(cmd)
        self.arm_pub.publish(msg)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = WorkbenchPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
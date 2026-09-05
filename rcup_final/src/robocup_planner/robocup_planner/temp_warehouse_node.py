"""
Temporary Combined Warehouse Node, SML EAI Workshop
=====================================================
Temporary integration node combining base navigation and arm control
for the warehouse bot.

NOTE THIS IS TEMPORARY: Need to add ActionServer Base and calling perception for block place position.

Subscribes to:
  /planned_task        (sml_messages/PlannedTask) -- sequence from Task Planner
  /go_to_shelf_result  (std_msgs/String)          -- nav ACK from nav node

Publishes to:
  /go_to_shelf_cmd     (std_msgs/String)          -- station name to nav node

Action Client:
  /arm_command         (sml_messages/action/ArmCommand) -- pick+place to arm

Service Client:
  /perception          (sml_messages/srv/PerceptionSrv) -- get block poses

Perception cache logic:
  - Perception called ONCE per station on first arrival
  - Poses stored in cache keyed by station name + block type
  - Cache cleared when bot leaves a station
  - Same station = skip nav + skip perception, just consume from cache

Hardcoded (TODO: update before physical testing):
  BASE_SLOTS -- place coords per block slot on bot base
  WB_SLOTS   -- place coords per block slot on workbench
"""

import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String

from sml_messages.msg import PlannedTask, PlanStep
from sml_messages.action import ArmCommand
from sml_messages.srv import PerceptionSrv


# Hardcoded coordinates, UPDATE BEFORE PHYSICAL TESTING

# Place coords per block slot on bot base (x, y, z)
BASE_SLOTS = [
    (0.50, -0.86, 0.20),   # slot 0
    (0.50, -0.76, 0.20),   # slot 1
    (0.50, -0.96, 0.20),   # slot 2
]

# Place coords per block slot on workbench (x, y, z) - CHANGE THIS ACCORDINGLY
WB_SLOTS = [
    (0.50, 0.50, 0.00),    # slot 0 - bottom block
    (0.50, 0.50, 0.05),    # slot 1 - middle block
    (0.50, 0.50, 0.10),    # slot 2 - top block
]

NAV_TIMEOUT        = 120.0   # seconds
ARM_TIMEOUT        = 160.0   # seconds
PERCEPTION_TIMEOUT = 10.0    # seconds


# Node ----------------------------------------------------------------------
class TempWarehouseNode(Node):

    def __init__(self):
        super().__init__('temp_warehouse_node')

        self.cb_group = ReentrantCallbackGroup()

        # Nav publisher + result subscriber
        self.nav_pub = self.create_publisher(String, '/go_to_shelf_cmd', 10)
        self._nav_result   = None
        self._nav_event    = threading.Event()
        self.nav_result_sub = self.create_subscription(
            String, '/go_to_shelf_result', self._on_nav_result, 10,
            callback_group=self.cb_group
        )

        # Arm action client
        self.arm_client = ActionClient(
            self, ArmCommand, '/arm_command',
            callback_group=self.cb_group
        )

        # Perception service client
        self.perception_client = self.create_client(
            PerceptionSrv, '/perception',
            callback_group=self.cb_group
        )

        # PlannedTask subscriber
        self.plan_sub = self.create_subscription(
            PlannedTask, '/planned_task', self._on_planned_task, 10,
            callback_group=self.cb_group
        )

        # State---------------------------------------------------------------
        self._blocks_on_base: int  = 0
        self._current_station: str = ''

        # Perception cache: station_name = {block_type: [(x,y,z), ...]}
        self._pose_cache: dict[str, dict[int, list[tuple]]] = {}

        self.get_logger().info(
            'TempWarehouseNode started. Waiting for /planned_task ...'
        )

    # Callbacks--------------------------------------------------------------
    def _on_nav_result(self, msg: String):
        self._nav_result = msg.data
        self._nav_event.set()
        self.get_logger().info(f'Nav result: {msg.data}')

    def _on_planned_task(self, msg: PlannedTask):
        if not msg.steps:
            self.get_logger().warn('Received PlannedTask with no steps.')
            return
        self.get_logger().info(
            f'Received PlannedTask: {len(msg.steps)} steps.'
        )
        thread = threading.Thread(
            target=self._execute, args=(msg.steps,), daemon=True
        )
        thread.start()

    # Main execution loop-------------------------------------------------------
    def _execute(self, steps: list[PlanStep]):
        self._blocks_on_base  = 0
        self._current_station = ''
        self._pose_cache      = {}   # fresh cache per execution

        for i, step in enumerate(steps):
            self.get_logger().info(f'Step {i+1}/{len(steps)}: {step.label}')

            ok = False

            if step.action == PlanStep.FETCH_BLOCK:
                ok = self._handle_fetch_block(step)

            elif step.action == PlanStep.GOTO_WORKBENCH:
                ok = self._handle_goto_workbench(step)

            elif step.action == PlanStep.LOAD_TOY:
                self.get_logger().info('  LOAD_TOY skipped (temp).')
                ok = True

            elif step.action == PlanStep.DELIVER:
                ok = self._handle_deliver(step)

            elif step.action == PlanStep.FETCH_RECYCLE:
                ok = self._handle_fetch_recycle(step)

            elif step.action == PlanStep.PLACE_RECYCLE:
                ok = self._handle_place_recycle(step)

            elif step.action == PlanStep.LOAD_BLOCKS:
                ok = self._handle_load_blocks()

            elif step.action == PlanStep.RETURN_BLOCK:
                ok = self._handle_return_block(step)

            else:
                self.get_logger().warn(
                    f'Unknown step action {step.action}, skipping.'
                )
                ok = True

            if not ok:
                self.get_logger().error(
                    f'Step {i+1} failed: {step.label}. Aborting.'
                )
                return

        self.get_logger().info('All steps completed.')

    # FETCH_BLOCK
    def _handle_fetch_block(self, step: PlanStep) -> bool:
        slot = self._blocks_on_base

        if slot >= len(BASE_SLOTS):
            self.get_logger().error(
                f'No BASE_SLOT defined for slot {slot}. Add more entries.'
            )
            return False

        # 1. Navigate (skip if already at this station)
        if not self._navigate_to(step.station):
            return False

        # 2. Call perception if this station not cached yet
        if step.station not in self._pose_cache:
            # Count how many blocks of ANY type we need from this station
            # For now: use number of blocks currently needed = remaining base slots
            n_blocks = len(BASE_SLOTS) - self._blocks_on_base
            self.get_logger().info(
                f'  calling perception at {step.station} '
                f'(requesting {n_blocks} blocks)'
            )
            cache = self._call_perception(step.station, n_blocks)
            if cache is None:
                self.get_logger().error('Perception failed.')
                return False
            self._pose_cache[step.station] = cache
            self.get_logger().info(
                f'  perception cache for {step.station}: '
                + str({k: len(v) for k, v in cache.items()})
            )

        # 3. Pop pose for this block_type from cache
        block_poses = self._pose_cache[step.station].get(step.block_type, [])
        if not block_poses:
            self.get_logger().error(
                f'No perceived pose for block_type{step.block_type} '
                f'at {step.station}. '
                f'Available types: {list(self._pose_cache[step.station].keys())}'
            )
            return False

        pick_coords = block_poses.pop(0)   # consume first available pose
        place_coords = BASE_SLOTS[slot]

        # 4. Send arm goal
        self.get_logger().info(
            f'  arm pick block_type{step.block_type} '
            f'at ({pick_coords[0]:.2f},{pick_coords[1]:.2f},{pick_coords[2]:.2f}) '
            f'=> base slot {slot}'
        )
        ok = self._send_arm_goal(
            pick      = pick_coords,
            place     = place_coords,
            object_id = f'block_type{step.block_type}_{slot}'
        )
        if ok:
            self._blocks_on_base += 1
        return ok

    # GOTO_WORKBENCH: navigate --> arm place each block from base onto wb
    def _handle_goto_workbench(self, step: PlanStep) -> bool:
        if not self._navigate_to(step.station):
            return False

        for slot in range(self._blocks_on_base):
            if slot >= len(WB_SLOTS):
                self.get_logger().error(
                    f'No WB_SLOT defined for slot {slot}. Add more entries.'
                )
                return False

            self.get_logger().info(
                f'  arm place block from base slot {slot} -> wb slot {slot}'
            )
            ok = self._send_arm_goal(
                pick      = BASE_SLOTS[slot],
                place     = WB_SLOTS[slot],
                object_id = f'base_block_{slot}'
            )
            if not ok:
                return False

        self._blocks_on_base = 0
        return True

    # DELIVER: base drives to cc_1, arm work handled by WB arm
    def _handle_deliver(self, step: PlanStep) -> bool:
        return self._navigate_to(step.station)

    # FETCH_RECYCLE: navigate to cc_1 -> arm picks toy -> places on base
    def _handle_fetch_recycle(self, step: PlanStep) -> bool:
        if not self._navigate_to(step.station):
            return False

        self.get_logger().info('  arm picks assembled toy from counter.')
        ok = self._send_arm_goal(
            pick      = BASE_SLOTS[0],   # TODO: replace with perception or hardcoded counter coords
            place     = BASE_SLOTS[0],
            object_id = 'recycle_toy'
        )
        return ok

    # PLACE_RECYCLE: navigate to wb -> arm places toy -> wait for breakdown
    def _handle_place_recycle(self, step: PlanStep) -> bool:
        if not self._navigate_to(step.station):
            return False

        self.get_logger().info('  arm places toy on workbench for disassembly.')
        ok = self._send_arm_goal(
            pick      = BASE_SLOTS[0],
            place     = WB_SLOTS[0],
            object_id = 'recycle_toy'
        )
        if ok:
            self.get_logger().info(
                '  waiting for WB arm to disassemble toy ... (placeholder delay)'
            )
            import time
            time.sleep(2.0)   # TODO: replace with WB arm signal
        return ok

    # LOAD_BLOCKS: arm picks each block from wb back onto base
    def _handle_load_blocks(self) -> bool:
        for slot in range(len(WB_SLOTS)):
            self.get_logger().info(
                f'  arm pick block from wb slot {slot} -> base slot {slot}'
            )
            ok = self._send_arm_goal(
                pick      = WB_SLOTS[slot],
                place     = BASE_SLOTS[slot],
                object_id = f'recycled_block_{slot}'
            )
            if not ok:
                return False
            self._blocks_on_base += 1
        return True

    # RETURN_BLOCK: navigate to shelf -> arm places block from base onto shelf
    def _handle_return_block(self, step: PlanStep) -> bool:
        slot = self._blocks_on_base - 1

        if not self._navigate_to(step.station):
            return False

        if slot < 0 or slot >= len(BASE_SLOTS):
            self.get_logger().error(f'Invalid base slot {slot} for return.')
            return False

        # TODO: use perception for shelf place coords too (future)
        self.get_logger().info(
            f'  arm place block_type{step.block_type} '
            f'from base slot {slot} -> shelf {step.station}'
        )
        ok = self._send_arm_goal(
            pick      = BASE_SLOTS[slot],
            place     = WB_SLOTS[slot],   # placeholder — shelf place coords
            object_id = f'block_type{step.block_type}_{slot}'
        )
        if ok:
            self._blocks_on_base -= 1
        return ok

    # other functions-------------------------------------------------------------
    def _navigate_to(self, station: str) -> bool:
        if station == self._current_station:
            self.get_logger().info(
                f'  already at "{station}", skipping nav.'
            )
            return True

        self.get_logger().info(f'  navigating to: {station}')

        # Clear cache for station we're leaving
        if self._current_station in self._pose_cache:
            self.get_logger().info(
                f'  clearing pose cache for "{self._current_station}"'
            )
            self._pose_cache.pop(self._current_station)

        self._nav_result = None
        self._nav_event.clear()

        msg      = String()
        msg.data = station
        self.nav_pub.publish(msg)

        arrived = self._nav_event.wait(timeout=NAV_TIMEOUT)

        if not arrived:
            self.get_logger().error(f'NAV TIMEOUT for "{station}"')
            return False

        if self._nav_result == f'{station}:success':
            self.get_logger().info(f'  reached "{station}"')
            self._current_station = station
            return True
        else:
            self.get_logger().error(
                f'  nav FAILED for "{station}": {self._nav_result}'
            )
            return False

    def _call_perception(
        self, station: str, n_blocks: int
    ) -> dict[int, list[tuple]] | None:

        if not self.perception_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/perception service not available.')
            return None

        req = PerceptionSrv.Request()
        req.target_samples = n_blocks

        future = self.perception_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=PERCEPTION_TIMEOUT)

        if not future.done():
            self.get_logger().error('Perception service timed out.')
            return None

        response = future.result()
        if response is None:
            self.get_logger().error('Perception service returned None.')
            return None

        # Parse response into cache format: {block_type: [(x,y,z), ...]}
        cache: dict[int, list[tuple]] = {}
        poses  = response.block_poses.poses
        ids    = response.block_ids

        for i, (pose, block_id_str) in enumerate(zip(poses, ids)):
            try:
                block_type = int(block_id_str)
            except ValueError:
                self.get_logger().warn(
                    f'  perception: block_id "{block_id_str}" '
                    f'at index {i} is not an int, skipping.'
                )
                continue

            if block_type == -1:
                self.get_logger().warn(
                    f'  perception: block at index {i} has id=-1 '
                    f'(identification failed), skipping.'
                )
                continue

            coords = (
                pose.position.x,
                pose.position.y,
                pose.position.z,
            )
            cache.setdefault(block_type, []).append(coords)
            self.get_logger().info(
                f'  perception: block_type{block_type} at '
                f'({coords[0]:.2f},{coords[1]:.2f},{coords[2]:.2f})'
            )

        return cache

    def _send_arm_goal(
        self,
        pick:      tuple,
        place:     tuple,
        object_id: str
    ) -> bool:
        if not self.arm_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('/arm_command action server not available.')
            return False

        goal           = ArmCommand.Goal()
        goal.object_id = object_id
        goal.pick_x,  goal.pick_y,  goal.pick_z  = pick
        goal.place_x, goal.place_y, goal.place_z = place

        self.get_logger().info(
            f'  => arm goal [{object_id}] '
            f'pick=({pick[0]:.2f},{pick[1]:.2f},{pick[2]:.2f}) '
            f'place=({place[0]:.2f},{place[1]:.2f},{place[2]:.2f})'
        )

        send_future = self.arm_client.send_goal_async(
            goal, feedback_callback=self._arm_feedback_cb
        )
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=ARM_TIMEOUT)

        if not send_future.done():
            self.get_logger().error('Arm goal send timed out.')
            return False

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Arm goal rejected.')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=ARM_TIMEOUT)

        if not result_future.done():
            self.get_logger().error('Arm result timed out.')
            return False

        success = result_future.result().result.success
        if success:
            self.get_logger().info(f'  arm goal succeeded [{object_id}]')
        else:
            self.get_logger().error(f'  arm goal failed [{object_id}]')
        return success

    def _arm_feedback_cb(self, feedback_msg):
        self.get_logger().info(f'  [arm] {feedback_msg.feedback.status}')


# Entry point
def main(args=None):
    rclpy.init(args=args)
    node = TempWarehouseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
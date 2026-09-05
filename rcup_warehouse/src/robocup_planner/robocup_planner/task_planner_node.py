"""
Task planner node.
=======================================================================
Subscribes to:  /task          (sml_messages/Task)
Publishes to:   /planned_task  (sml_messages/PlannedTask)

PRODUCE sequence:
  FETCH_BLOCK x N -> go to shelf, arm pick block, place on bot base
  GOTO_WORKBENCH -> drive to wb, arm place all blocks from base onto wb
  LOAD_TOY -> arm pick assembled toy from wb, place on bot base
  DELIVER -> drive to cc_1, arm place toy at counter

RECYCLE sequence:
  FETCH_RECYCLE -> go to cc_1, arm pick toy, place on bot base
  PLACE_RECYCLE -> drive to wb, arm place toy, wait for breakdown
  LOAD_BLOCKS -> arm pick each block from wb onto bot base
  RETURN_BLOCK x N -> drive to shelf, arm place block from base onto shelf
                      (shelf = least occupied storage shelf)
"""

import traceback
from collections import deque

import rclpy
import unified_planning as up
from rclpy.node import Node
from unified_planning.model import Object
from unified_planning.shortcuts import (
    BoolType,
    Fluent,
    InstantaneousAction,
    OneshotPlanner,
    Problem,
    UserType,
)

from sml_messages.msg import Order, PlanStep, PlannedTask, Station, Task

up.environment.get_environment().credits_stream = None


def decode_product(product_id: int) -> list[int]:
    """Return the block types of a product, bottom to top. E.g. 341 -> [3, 4, 1]."""
    return [int(d) for d in str(product_id)]


class TaskPlannerNode(Node):

    def __init__(self):
        super().__init__("task_planner_node")
        self.sub = self.create_subscription(Task, "/task", self.task_callback, 10)
        self.pub = self.create_publisher(PlannedTask, "/planned_task", 10)

        self.pending_orders: deque[Order] = deque()
        self.inventory: dict[int, list[str]] = {}
        self.arena_stations: list[Station] = []

        self.get_logger().info("TaskPlannerNode started. Waiting for /task ...")

    def task_callback(self, msg: Task):
        self.get_logger().info(
            f"Received task: {len(msg.order_list)} orders, " f"{len(msg.arena_layout)} stations"
        )
        self.arena_stations = list(msg.arena_layout)
        self.inventory = self._parse_inventory(msg.arena_layout)
        self.get_logger().info(f"Inventory: {self.inventory}")

        self.pending_orders.clear()
        for order in msg.order_list:
            self.pending_orders.append(order)

        self._plan_and_publish()

    def _parse_inventory(self, stations) -> dict[int, list[str]]:
        """Map block type -> list of storage stations holding it."""
        inventory: dict[int, list[str]] = {}
        for station in stations:
            if station.station_type == Station.ST_STORAGE:
                for mid in station.material_ids:
                    inventory.setdefault(mid, []).append(station.name)
        return inventory

    def _build_storage_map(self) -> dict[int, str]:
        """Map block type -> first storage station that has it."""
        return {bt: locs[0] for bt, locs in self.inventory.items()}

    def _get_workbench(self) -> str:
        for s in self.arena_stations:
            if s.station_type == Station.ST_WORKBENCH:
                return s.name
        return "wb_top"

    def _build_shelf_occupancy(self) -> dict[str, int]:
        """Map storage shelf name -> number of blocks currently on it."""
        occupancy = {}
        for station in self.arena_stations:
            if station.station_type == Station.ST_STORAGE:
                occupancy[station.name] = len(station.material_ids)
        return occupancy

    def _least_occupied_shelf(self, occupancy: dict[str, int]) -> str:
        return min(occupancy, key=occupancy.get)

    def _is_feasible(self, order: Order) -> bool:
        required = decode_product(order.product_id)
        needed: dict[int, int] = {}
        for b in required:
            needed[b] = needed.get(b, 0) + 1

        if order.order_type == Order.OT_PRODUCE:
            available = {bt: len(locs) for bt, locs in self.inventory.items()}
            for bt, count in needed.items():
                if available.get(bt, 0) < count:
                    return False

        elif order.order_type == Order.OT_RECYCLE:
            recycle_avail: dict[int, int] = {}
            for station in self.arena_stations:
                if station.station_type in (Station.ST_CUSTOMER, Station.ST_HYBRID):
                    for mid in station.material_ids:
                        recycle_avail[mid] = recycle_avail.get(mid, 0) + 1
            for bt, count in needed.items():
                if recycle_avail.get(bt, 0) < count:
                    return False

        return True

    def _plan_and_publish(self):
        feasible_produce = []
        feasible_recycle = []

        for order in self.pending_orders:
            if not self._is_feasible(order):
                self.get_logger().warn(
                    f"Order product_id={order.product_id} not feasible, skipping."
                )
                continue
            if order.order_type == Order.OT_PRODUCE:
                feasible_produce.append(order)
            elif order.order_type == Order.OT_RECYCLE:
                feasible_recycle.append(order)

        if not feasible_produce and not feasible_recycle:
            self.get_logger().warn("No feasible orders.")
            return

        # Recycle first (restores inventory), then produce shortest first.
        feasible_produce.sort(key=lambda o: len(decode_product(o.product_id)))
        all_feasible = feasible_recycle + feasible_produce

        self.get_logger().info(
            f"Planning {len(all_feasible)} orders "
            f"({len(feasible_recycle)} recycle, {len(feasible_produce)} produce)"
        )

        result = self._solve_with_up(all_feasible)
        if result is None:
            self.get_logger().error("UP planner failed.")
            return

        action_sequence, action_meta = result

        self.get_logger().info("Plan found:")
        for i, a in enumerate(action_sequence):
            self.get_logger().info(f"  Step {i+1}: {a}")

        steps = self._build_plan_steps(action_sequence, action_meta)
        self._publish_planned_task(all_feasible, steps)

    def _solve_with_up(self, orders: list[Order]) -> tuple | None:
        Block = UserType("Block")
        Toy = UserType("Toy")
        OrderType = UserType("OrderObj")

        # Block location
        block_on_shelf = Fluent("block_on_shelf", BoolType(), block=Block)
        block_on_base = Fluent("block_on_base", BoolType(), block=Block)
        block_on_workbench = Fluent("block_on_workbench", BoolType(), block=Block)

        # Toy state
        toy_on_workbench = Fluent("toy_on_workbench", BoolType(), toy=Toy)
        toy_on_base = Fluent("toy_on_base", BoolType(), toy=Toy)
        toy_at_counter = Fluent("toy_at_counter", BoolType(), toy=Toy)
        toy_assembled = Fluent("toy_assembled", BoolType(), toy=Toy)
        toy_broken_down = Fluent("toy_broken_down", BoolType(), toy=Toy)

        # Robot state
        arm_free = Fluent("arm_free", BoolType())
        robot_free = Fluent("robot_free", BoolType())

        # Order phase gates
        all_blocks_on_base = Fluent("all_blocks_on_base", BoolType(), order=OrderType)
        all_blocks_on_wb = Fluent("all_blocks_on_wb", BoolType(), order=OrderType)
        all_blocks_loaded = Fluent("all_blocks_loaded", BoolType(), order=OrderType)
        order_done = Fluent("order_done", BoolType(), order=OrderType)
        recycle_phase_done = Fluent("recycle_phase_done", BoolType())

        problem = Problem("sml_v5")
        for f in [
            block_on_shelf,
            block_on_base,
            block_on_workbench,
            toy_on_workbench,
            toy_on_base,
            toy_at_counter,
            toy_assembled,
            toy_broken_down,
            arm_free,
            robot_free,
            all_blocks_on_base,
            all_blocks_on_wb,
            all_blocks_loaded,
            order_done,
            recycle_phase_done,
        ]:
            problem.add_fluent(f, default_initial_value=False)

        problem.set_initial_value(arm_free, True)
        problem.set_initial_value(robot_free, True)

        # Skip the recycle phase entirely when there are no recycle orders.
        has_recycle = any(o.order_type == Order.OT_RECYCLE for o in orders)
        if not has_recycle:
            problem.set_initial_value(recycle_phase_done, True)

        # Shared action: PRODUCE fetch of a block from shelf to base.
        fetch_block = InstantaneousAction("fetch_block", block=Block)
        b = fetch_block.parameter("block")
        fetch_block.add_precondition(block_on_shelf(b))
        fetch_block.add_precondition(arm_free)
        fetch_block.add_precondition(robot_free)
        fetch_block.add_precondition(recycle_phase_done)
        fetch_block.add_effect(block_on_base(b), True)
        fetch_block.add_effect(block_on_shelf(b), False)
        fetch_block.add_effect(arm_free, False)
        fetch_block.add_effect(arm_free, True)
        problem.add_action(fetch_block)

        # return_block is grounded per block inside the order loop below.

        # Maps from UP object names to the data needed for building plan steps.
        block_to_type: dict[str, int] = {}
        block_to_shelf: dict[str, str] = {}
        action_meta: dict[str, tuple] = {}

        storage_map = self._build_storage_map()
        workbench = self._get_workbench()
        customer_stn = "cc_1"
        shelf_occupancy = self._build_shelf_occupancy()

        order_objects: list[Object] = []

        for oi, order in enumerate(orders):
            seq = decode_product(order.product_id)

            ord_obj = Object(f"order_{oi}_pid{order.product_id}", OrderType)
            problem.add_object(ord_obj)
            order_objects.append(ord_obj)

            toy_obj = Object(f"toy_{oi}_pid{order.product_id}", Toy)
            problem.add_object(toy_obj)

            order_block_objs = []
            for bi, bt in enumerate(seq):
                name = f"blk_o{oi}_b{bi}_t{bt}"
                blk_obj = Object(name, Block)
                problem.add_object(blk_obj)
                block_to_type[name] = bt

                if order.order_type == Order.OT_PRODUCE:
                    problem.set_initial_value(block_on_shelf(blk_obj), True)
                    block_to_shelf[name] = storage_map.get(bt, "")
                elif order.order_type == Order.OT_RECYCLE:
                    # Toy is at the counter; its blocks appear after breakdown.
                    problem.set_initial_value(toy_at_counter(toy_obj), True)

                order_block_objs.append(blk_obj)

            if order.order_type == Order.OT_PRODUCE:
                # Gate: all blocks on base.
                gate_base = InstantaneousAction(f"set_all_on_base_{oi}")
                for blk in order_block_objs:
                    gate_base.add_precondition(block_on_base(blk))
                gate_base.add_precondition(arm_free)
                gate_base.add_effect(all_blocks_on_base(ord_obj), True)
                problem.add_action(gate_base)
                action_meta[f"set_all_on_base_{oi}()"] = (None, 0, "")

                goto_wb = InstantaneousAction(f"goto_workbench_{oi}")
                goto_wb.add_precondition(all_blocks_on_base(ord_obj))
                goto_wb.add_precondition(arm_free)
                for blk in order_block_objs:
                    goto_wb.add_effect(block_on_workbench(blk), True)
                    goto_wb.add_effect(block_on_base(blk), False)
                goto_wb.add_effect(all_blocks_on_base(ord_obj), False)
                goto_wb.add_effect(all_blocks_on_wb(ord_obj), True)
                goto_wb.add_effect(toy_assembled(toy_obj), True)
                goto_wb.add_effect(toy_on_workbench(toy_obj), True)
                problem.add_action(goto_wb)
                action_meta[f"goto_workbench_{oi}()"] = (PlanStep.GOTO_WORKBENCH, 0, workbench)

                load_toy = InstantaneousAction(f"load_toy_{oi}")
                load_toy.add_precondition(toy_on_workbench(toy_obj))
                load_toy.add_precondition(toy_assembled(toy_obj))
                load_toy.add_precondition(arm_free)
                load_toy.add_effect(toy_on_base(toy_obj), True)
                load_toy.add_effect(toy_on_workbench(toy_obj), False)
                problem.add_action(load_toy)
                action_meta[f"load_toy_{oi}()"] = (PlanStep.LOAD_TOY, 0, workbench)

                deliver = InstantaneousAction(f"deliver_{oi}")
                deliver.add_precondition(toy_on_base(toy_obj))
                deliver.add_precondition(arm_free)
                deliver.add_effect(toy_at_counter(toy_obj), True)
                deliver.add_effect(toy_on_base(toy_obj), False)
                deliver.add_effect(order_done(ord_obj), True)
                problem.add_action(deliver)
                action_meta[f"deliver_{oi}()"] = (PlanStep.DELIVER, 0, customer_stn)

            elif order.order_type == Order.OT_RECYCLE:
                fetch_rec = InstantaneousAction(f"fetch_recycle_{oi}")
                fetch_rec.add_precondition(toy_at_counter(toy_obj))
                fetch_rec.add_precondition(arm_free)
                fetch_rec.add_precondition(robot_free)
                fetch_rec.add_effect(toy_on_base(toy_obj), True)
                fetch_rec.add_effect(toy_at_counter(toy_obj), False)
                problem.add_action(fetch_rec)
                action_meta[f"fetch_recycle_{oi}()"] = (PlanStep.FETCH_RECYCLE, 0, customer_stn)

                place_rec = InstantaneousAction(f"place_recycle_{oi}")
                place_rec.add_precondition(toy_on_base(toy_obj))
                place_rec.add_precondition(arm_free)
                place_rec.add_effect(toy_on_workbench(toy_obj), True)
                place_rec.add_effect(toy_on_base(toy_obj), False)
                place_rec.add_effect(toy_broken_down(toy_obj), True)
                for blk in order_block_objs:
                    place_rec.add_effect(block_on_workbench(blk), True)
                problem.add_action(place_rec)
                action_meta[f"place_recycle_{oi}()"] = (PlanStep.PLACE_RECYCLE, 0, workbench)

                load_blks = InstantaneousAction(f"load_blocks_{oi}")
                load_blks.add_precondition(toy_broken_down(toy_obj))
                load_blks.add_precondition(arm_free)
                for blk in order_block_objs:
                    load_blks.add_precondition(block_on_workbench(blk))
                    load_blks.add_effect(block_on_base(blk), True)
                    load_blks.add_effect(block_on_workbench(blk), False)
                load_blks.add_effect(all_blocks_loaded(ord_obj), True)
                problem.add_action(load_blks)
                action_meta[f"load_blocks_{oi}()"] = (PlanStep.LOAD_BLOCKS, 0, workbench)

                # Assign each block to the least occupied shelf. Keyed by block
                # index (not type) so duplicate block types target distinct shelves.
                occ_copy = dict(shelf_occupancy)
                ret_shelf_by_idx: dict[int, str] = {}
                for bi_r, bt_r in enumerate(seq):
                    shelf_r = self._least_occupied_shelf(occ_copy)
                    ret_shelf_by_idx[bi_r] = shelf_r
                    occ_copy[shelf_r] += 1

                for bi, bt in enumerate(seq):
                    blk = order_block_objs[bi]
                    ret_shelf = ret_shelf_by_idx.get(bi, "")
                    ret = InstantaneousAction(f"return_block_o{oi}_b{bi}_t{bt}")
                    ret.add_precondition(block_on_base(blk))
                    ret.add_precondition(arm_free)
                    ret.add_precondition(robot_free)
                    ret.add_precondition(all_blocks_loaded(ord_obj))
                    ret.add_effect(block_on_shelf(blk), True)
                    ret.add_effect(block_on_base(blk), False)
                    problem.add_action(ret)
                    action_meta[f"return_block_o{oi}_b{bi}_t{bt}()"] = (
                        PlanStep.RETURN_BLOCK,
                        bt,
                        ret_shelf,
                    )

                recycle_done = InstantaneousAction(f"recycle_done_{oi}")
                for blk in order_block_objs:
                    recycle_done.add_precondition(block_on_shelf(blk))
                recycle_done.add_effect(order_done(ord_obj), True)
                problem.add_action(recycle_done)
                action_meta[f"recycle_done_{oi}()"] = (None, 0, "")

        # Mark the recycle phase done once the last recycle order completes.
        last_recycle_idx = None
        for oi, order in enumerate(orders):
            if order.order_type == Order.OT_RECYCLE:
                last_recycle_idx = oi
        if last_recycle_idx is not None:
            for action in problem.actions:
                if action.name == f"recycle_done_{last_recycle_idx}":
                    action.add_effect(recycle_phase_done, True)
                    break

        for ord_obj in order_objects:
            problem.add_goal(order_done(ord_obj))

        solvers = ["pyperplan", "fast-downward"]
        for solver in solvers:
            try:
                self.get_logger().info(f"Trying solver: {solver}")
                with OneshotPlanner(name=solver) as planner:
                    result = planner.solve(problem)
                    if result.plan is None:
                        self.get_logger().warn(f"{solver} returned no plan.")
                        continue

                    self.get_logger().info(f"Solved with: {solver}")
                    action_sequence = []
                    for ai in result.plan.actions:
                        name = ai.action.name
                        params = [str(p) for p in ai.actual_parameters]
                        action_str = f'{name}({", ".join(params)})' if params else f"{name}()"
                        action_sequence.append(action_str)

                        if name == "fetch_block" and params:
                            bt = block_to_type.get(params[0], 0)
                            stn = block_to_shelf.get(params[0], storage_map.get(bt, ""))
                            action_meta[action_str] = (PlanStep.FETCH_BLOCK, bt, stn)

                    return action_sequence, action_meta
            except Exception as e:
                self.get_logger().warn(f"{solver} failed: {e}")
                self.get_logger().warn(traceback.format_exc())

        self.get_logger().error("All solvers failed.")
        return None

    def _build_plan_steps(self, action_sequence, action_meta) -> list[PlanStep]:
        steps: list[PlanStep] = []

        for action_str in action_sequence:
            meta = action_meta.get(action_str)
            if meta is None:
                continue

            planstep_action, block_type, station = meta
            if planstep_action is None:
                continue

            step = PlanStep()
            step.action = planstep_action
            step.block_type = block_type
            step.station = station

            if planstep_action == PlanStep.FETCH_BLOCK:
                step.label = f"fetch block_type{block_type} from {station}"
            elif planstep_action == PlanStep.GOTO_WORKBENCH:
                step.label = f"drive to {station}, place all blocks on workbench"
            elif planstep_action == PlanStep.LOAD_TOY:
                step.label = f"arm pick assembled toy from {station}, place on base"
            elif planstep_action == PlanStep.DELIVER:
                step.label = f"drive to {station}, deliver toy"
            elif planstep_action == PlanStep.FETCH_RECYCLE:
                step.label = f"drive to {station}, arm pick toy, place on base"
            elif planstep_action == PlanStep.PLACE_RECYCLE:
                step.label = f"drive to {station}, arm place toy, wait for breakdown"
            elif planstep_action == PlanStep.LOAD_BLOCKS:
                step.label = f"arm pick each block from {station} onto base"
            elif planstep_action == PlanStep.RETURN_BLOCK:
                step.label = f"drive to {station}, " f"arm place block_type{block_type} on shelf"

            steps.append(step)

        return steps

    def _publish_planned_task(self, orders, steps):
        out_msg = PlannedTask()
        out_msg.order_list = orders
        out_msg.arena_layout = self.arena_stations
        out_msg.steps = steps

        self.pub.publish(out_msg)
        self.get_logger().info(f"Published PlannedTask: {len(steps)} steps.")
        for i, step in enumerate(steps):
            self.get_logger().info(f"  Step {i+1}: {step.label}")


def main(args=None):
    rclpy.init(args=args)
    node = TaskPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
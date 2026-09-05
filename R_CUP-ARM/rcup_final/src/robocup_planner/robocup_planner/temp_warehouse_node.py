import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String

from sml_messages.msg import PlannedTask, PlanStep
from sml_messages.action import ArmCommand

# Pick coords per block slot on shelf (x, y, z)
SHELF_SLOTS = [
    (-0.118, 0.41, 0.00145),   # slot 0  
    (0.50, -0.40, 0.09),   # slot 1
    (0.60, -0.48, 0.09),   # slot 2
]

BASE_SLOTS = [
    (-0.11, -0.090, 0.19),   # base slot 0
    (-0.11, 0.005, 0.19),   # base slot 1
    (-0.11, 0.100, 0.19),   # base slot 2
]

# Place coords per block slot on workbench (x, y, z)
WB_SLOTS = [
    (0.50, 0.50, 0.00),    # slot 0 - bottom block
    (0.50, 0.50, 0.05),    # slot 1 - middle block
    (0.50, 0.50, 0.10),    # slot 2 - top block
]

NAV_TIMEOUT = 800.0
ARM_TIMEOUT = 800.0

class TempWarehouseNode(Node):

    def __init__(self):
        super().__init__('temp_warehouse_node')

        self.cb_group = ReentrantCallbackGroup()

        # Nav publisher + result subscriber
        self.nav_pub = self.create_publisher(String, '/go_to_shelf_cmd', 10)
        self._nav_result = None
        self._nav_event  = threading.Event()
        self.nav_result_sub = self.create_subscription(
            String, '/go_to_shelf_result', self._on_nav_result, 10,
            callback_group=self.cb_group
        )

        # Arm action client
        self.arm_client = ActionClient(
            self, ArmCommand, '/arm_command',
            callback_group=self.cb_group
        )

        # PlannedTask subscriber
        self.plan_sub = self.create_subscription(
            PlannedTask, '/planned_task', self._on_planned_task, 10,
            callback_group=self.cb_group
        )

        # Track how many blocks are currently on bot base
        self._blocks_on_base: int = 0

        self._current_station: str | None = None

        self.get_logger().info(
            'TempWarehouseNode started. Waiting for /planned_task ...'
        )

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

    def _execute(self, steps: list[PlanStep]):
        self._blocks_on_base = 0   # reset per execution

        for i, step in enumerate(steps):
            self.get_logger().info(
                f'Step {i+1}/{len(steps)}: {step.label}'
            )

            ok = False

            if step.action == PlanStep.FETCH_BLOCK:
                ok = self._handle_fetch_block(step)

            elif step.action == PlanStep.GOTO_WORKBENCH:
                ok = self._handle_goto_workbench(step)

            elif step.action == PlanStep.LOAD_TOY:

                self.get_logger().info('  LOAD_TOY skipped (temp).')
                ok = True   # skip entirely

            elif step.action == PlanStep.DELIVER:
                ok = self._handle_deliver(step)

            elif step.action == PlanStep.FETCH_RECYCLE:
                ok = self._handle_fetch_recycle(step)

            elif step.action == PlanStep.PLACE_RECYCLE:
                ok = self._handle_place_recycle(step)

            elif step.action == PlanStep.LOAD_BLOCKS:
                # Arm picks blocks from workbench onto base after breakdown
                ok = self._handle_load_blocks(step)

            elif step.action == PlanStep.RETURN_BLOCK:
                ok = self._handle_return_block(step)

            else:
                self.get_logger().warn(
                    f'Unknown step action {step.action}, skipping.'
                )
                ok = True   # don't abort on unknown

            if not ok:
                self.get_logger().error(
                    f'Step {i+1} failed: {step.label}. Aborting.'
                )
                return

        self.get_logger().info('All steps completed.')

    def _handle_fetch_block(self, step: PlanStep) -> bool:
        slot = self._blocks_on_base

        if slot >= len(SHELF_SLOTS):
            self.get_logger().error(
                f'No SHELF_SLOT defined for slot {slot}. Add more entries.'
            )
            return False

        # Navigate to shelf
        if not self._navigate_to(step.station):
            return False

        # Arm pick from shelf slot, place on base slot
        self.get_logger().info(
            f'  arm pick block_type{step.block_type} '
            f'from slot {slot} -> place on base slot {slot}'
        )
        ok = self._send_arm_goal(
            pick       = SHELF_SLOTS[slot],
            place      = BASE_SLOTS[slot],
            object_id  = f'block_type{step.block_type}_{slot}',
            station    = step.station,
            block_slot = slot
        )
        if ok:
            self._blocks_on_base += 1
        return ok

    def _handle_goto_workbench(self, step: PlanStep) -> bool:
        if not self._navigate_to(step.station):
            return False

        # Place each block from base onto workbench in sequence
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
                pick       = BASE_SLOTS[slot],
                place      = WB_SLOTS[slot],
                object_id  = f'base_block_{slot}',
                station    = step.station,
                block_slot = slot
            )
            if not ok:
                return False

        self._blocks_on_base = 0   # all placed on workbench
        return True

    def _handle_deliver(self, step: PlanStep) -> bool:
        return self._navigate_to(step.station)

    def _handle_fetch_recycle(self, step: PlanStep) -> bool:
        if not self._navigate_to(step.station):
            return False

        self.get_logger().info('  arm picks assembled toy from counter.')
        ok = self._send_arm_goal(
            pick      = SHELF_SLOTS[0],
            place     = BASE_SLOTS[0],
            object_id = 'recycle_toy',
            station   = step.station
        )
        return ok

    def _handle_place_recycle(self, step: PlanStep) -> bool:
        if not self._navigate_to(step.station):
            return False

        self.get_logger().info('  arm places toy on workbench for disassembly.')
        ok = self._send_arm_goal(
            pick      = BASE_SLOTS[0],
            place     = WB_SLOTS[0],
            object_id = 'recycle_toy',
            station   = step.station
        )
        if ok:
            self.get_logger().info(
                '  waiting for WB arm to disassemble toy ...'
                ' (no signal yet -- fixed delay placeholder)'
            )
            import time
            time.sleep(2.0)
        return ok

    def _handle_load_blocks(self, step: PlanStep) -> bool:
        for slot in range(len(WB_SLOTS)):
            self.get_logger().info(
                f'  arm pick block from wb slot {slot} -> base slot {slot}'
            )
            ok = self._send_arm_goal(
                pick      = WB_SLOTS[slot],
                place     = BASE_SLOTS[slot],
                object_id = f'recycled_block_{slot}',
                station   = step.station
            )
            if not ok:
                return False
            self._blocks_on_base += 1
        return True

    def _handle_return_block(self, step: PlanStep) -> bool:
        slot = self._blocks_on_base - 1   # place from top of base stack

        if not self._navigate_to(step.station):
            return False

        if slot < 0 or slot >= len(BASE_SLOTS):
            self.get_logger().error(f'Invalid base slot {slot} for return.')
            return False

        self.get_logger().info(
            f'  arm place block_type{step.block_type} '
            f'from base slot {slot} -> shelf {step.station}'
        )
        ok = self._send_arm_goal(
            pick      = BASE_SLOTS[slot],
            place     = SHELF_SLOTS[slot],   # placeholder shelf place coords
            object_id = f'block_type{step.block_type}_{slot}',
            station   = step.station
        )
        if ok:
            self._blocks_on_base -= 1
        return ok

    def _navigate_to(self, station: str) -> bool:

        if self._current_station == station:
            self.get_logger().info(f'  already at: {station}, skipping navigation')
            return True

        self.get_logger().info(f'  navigating to: {station}')

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

    def _send_arm_goal(
        self,
        pick:       tuple,
        place:      tuple,
        object_id:  str,
        station:    str,
        block_slot: int = -1  

    ) -> bool:
        if not self.arm_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('/arm_command action server not available.')
            return False

        goal            = ArmCommand.Goal()
        goal.object_id  = object_id
        goal.station    = station
        goal.block_slot = block_slot
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
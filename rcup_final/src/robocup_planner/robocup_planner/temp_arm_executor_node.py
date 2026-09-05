import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
import threading

from sml_messages.msg import PlannedTask, PlanStep
from sml_messages.action import ArmCommand

SHELF_SLOTS = [
    (0.40, -0.44, 0.09),   # slot 0 = object_red  pick pose
    (0.50, -0.40, 0.09),   # slot 1 = object_green pick pose
    (0.60, -0.48, 0.09),   # slot 2 = object_blue pick pose
]

BASE_SLOTS = [
    (0.50, -0.86, 0.20),   # base slot 0 = object_red  place pose
    (0.50, -0.76, 0.20),   # base slot 1 = object_green place pose
    (0.50, -0.96, 0.20),   # base slot 2 = object_blue place pose
]

ACTION_TIMEOUT = 160.0 

class TempArmExecutorNode(Node):

    def __init__(self):
        super().__init__('temp_arm_executor_node')

        self.cb_group = ReentrantCallbackGroup()

        self.sub = self.create_subscription(
            PlannedTask,
            '/planned_task',
            self._on_planned_task,
            10,
            callback_group=self.cb_group
        )

        self.arm_client = ActionClient(
            self,
            ArmCommand,
            '/arm_command',
            callback_group=self.cb_group
        )

        self.get_logger().info('TempArmExecutorNode started. Waiting for /planned_task ...')

    def _on_planned_task(self, msg: PlannedTask):
        if not msg.steps:
            self.get_logger().warn('Received PlannedTask with no steps.')
            return

        self.get_logger().info(f'Received PlannedTask: {len(msg.steps)} steps.')

        thread = threading.Thread(
            target=self._execute_steps,
            args=(msg.steps,),
            daemon=True
        )
        thread.start()

    def _execute_steps(self, steps: list[PlanStep]):
        fetch_count = 0   # tracks which shelf/base slot to use

        for i, step in enumerate(steps):
            if step.action != PlanStep.FETCH_BLOCK:
                self.get_logger().info(
                    f'Step {i+1}: skipping action={step.action} ({step.label})'
                )
                continue

            self.get_logger().info(
                f'Step {i+1}: FETCH_BLOCK block_type{step.block_type} '
                f'from {step.station} [slot {fetch_count}]'
            )

            if fetch_count >= len(SHELF_SLOTS):
                self.get_logger().error(
                    f'No shelf slot defined for fetch_count={fetch_count}. '
                    f'Add more entries to SHELF_SLOTS.'
                )
                return

            pick_coords  = SHELF_SLOTS[fetch_count]
            place_coords = BASE_SLOTS[fetch_count]
            object_id    = f'block_type{step.block_type}_{fetch_count}'

            self.get_logger().info(
                f'  pick  at ({pick_coords[0]:.2f},{pick_coords[1]:.2f},{pick_coords[2]:.2f})'
            )
            self.get_logger().info(
                f'  place at ({place_coords[0]:.2f},{place_coords[1]:.2f},{place_coords[2]:.2f})'
            )

            # One action goal now performs the ENTIRE pick-and-place cycle.
            if not self._send_arm_goal(pick_coords, place_coords, object_id):
                self.get_logger().error('Pick-and-place failed. Aborting.')
                return

            fetch_count += 1

        self.get_logger().info(f'Done. Fetched {fetch_count} block(s).')

    def _send_arm_goal(self, pick: tuple, place: tuple, object_id: str) -> bool:
        if not self.arm_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('/arm_command action server not available.')
            return False

        goal = ArmCommand.Goal()
        goal.object_id = object_id
        goal.pick_x, goal.pick_y, goal.pick_z = pick
        goal.place_x, goal.place_y, goal.place_z = place

        self.get_logger().info(
            f'  => sending goal [{object_id}] '
            f'pick=({pick[0]:.2f},{pick[1]:.2f},{pick[2]:.2f}) '
            f'place=({place[0]:.2f},{place[1]:.2f},{place[2]:.2f})'
        )

        send_future = self.arm_client.send_goal_async(
            goal,
            feedback_callback=self._feedback_cb
        )
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=ACTION_TIMEOUT)

        if not send_future.done():
            self.get_logger().error('Goal send timed out.')
            return False

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected.')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=ACTION_TIMEOUT)

        if not result_future.done():
            self.get_logger().error('Result timed out.')
            return False

        return result_future.result().result.success

    def _feedback_cb(self, feedback_msg):
        self.get_logger().info(f'  [arm] {feedback_msg.feedback.status}')

def main(args=None):
    rclpy.init(args=args)
    node = TempArmExecutorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
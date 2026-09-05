#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener
from std_msgs.msg import String
import rclpy.time

ALIGN_TIMEOUT = 60.0

class GoToShelf(Node):
    def __init__(self):
        super().__init__('go_to_shelf')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.subscription = self.create_subscription(
            String, '/go_to_shelf_cmd', self.callback, 10)
        self.result_pub = self.create_publisher(
            String, '/go_to_shelf_result', 10)

        self.align_event_in_pub = self.create_publisher(String, 'event_in', 1)
        self.align_event_out_sub = self.create_subscription(
            String, 'event_out', self._on_align_result, 10)
        self._current_target = None
        self._align_timeout_timer = None
        self.get_logger().info(
            f'Node for go_to_shelf initialized. Waiting for commands on /go_to_shelf_cmd')

    def callback(self, msg):
        target_frame = msg.data
        self.send_goal(target_frame)

    def send_goal(self, target_frame):
        self.get_logger().info(f'Looking up transform map -> {target_frame}')
        self._current_target = target_frame  # remember for later stages

        try:
            t = self.tf_buffer.lookup_transform(
                'map', target_frame, rclpy.time.Time())
        except Exception as e:
            self.get_logger().error(f'Transform lookup failed: {e}')
            self._publish_final_result(target_frame, success=False)
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = t.transform.translation.x
        goal.pose.pose.position.y = t.transform.translation.y
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation = t.transform.rotation

        self.client.wait_for_server()
        send_future = self.client.send_goal_async(goal)
        send_future.add_done_callback(
            lambda future: self._goal_response_callback(future, target_frame))
        self.get_logger().info(f'Sent goal to {target_frame}')

    def _goal_response_callback(self, future, target_frame):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            self._publish_final_result(target_frame, success=False)
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future: self._result_callback(future, target_frame))

    def _result_callback(self, future, target_frame):
        status = future.result().status
        nav_success = (status == 4)

        if not nav_success:
            self.get_logger().error(f'Nav2 failed to reach {target_frame}')
            self._publish_final_result(target_frame, success=False)
            return

        self.get_logger().info(
            f'Nav2 reached {target_frame} (coarse). Starting alignment...')
        self.align_event_in_pub.publish(String(data='e_trigger'))

        if self._align_timeout_timer is not None:
            self._align_timeout_timer.cancel()
        self._align_timeout_timer = self.create_timer(
            ALIGN_TIMEOUT, self._on_align_timeout)

    def _on_align_result(self, msg: String):
        if self._current_target is None:
            return

        if self._align_timeout_timer is not None:
            self._align_timeout_timer.cancel()
            self._align_timeout_timer = None

        if msg.data == 'Completed':
            self.get_logger().info(f'Alignment succeeded for {self._current_target}')
            self._publish_final_result(self._current_target, success=True)
        else:
            self.get_logger().error(
                f'Alignment failed for {self._current_target}: {msg.data}')
            self._publish_final_result(self._current_target, success=False)

    def _on_align_timeout(self):
        self._align_timeout_timer.cancel()
        self._align_timeout_timer = None
        self.get_logger().error(
            f'Timed out waiting for align.py result on {self._current_target}')
        self._publish_final_result(self._current_target, success=False)

    def _publish_final_result(self, target_frame, success: bool):
        result_msg = String()
        result_msg.data = f'{target_frame}:{"success" if success else "failure"}'
        self.result_pub.publish(result_msg)
        self._current_target = None


def main():
    rclpy.init()
    node = GoToShelf()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
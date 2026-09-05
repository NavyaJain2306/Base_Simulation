#! /usr/bin/env python3
from __future__ import print_function

import math
import json
import traceback

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import (
    PoseStamped,
    Quaternion,
)
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose

import tf2_ros
import tf2_geometry_msgs
from tf_transformations import (
    euler_from_quaternion,
    quaternion_from_euler,
)

from laser_line_extraction.msg import LineSegment, LineSegmentList

NODE = "mir_workspace_aligner"

# NavigateToPose action result status codes (action_msgs/msg/GoalStatus)
STATUS_SUCCEEDED = 4


class WorkspaceAligner(Node):

    """Read line segments from line segment extractor and find a workspace in front.
    Then try to align perpendicular to the workspace.

    Assumption: The robot is close to the workspace and the robot is looking in
    the general direction of the workspace. This module is only meant to align the
    robot slightly to the workspace. The nav2 package should be used for actually
    travelling to the workspace.

    Correction moves computed here are sent to nav2 directly via NavigateToPose,
    so this node fully owns "did the robot get there" instead of publishing a
    pose and polling TF hoping something else drives the robot."""

    def __init__(self):
        super().__init__(NODE)

        # ROS2 params
        self.declare_parameter("num_of_msgs", 100)
        self.declare_parameter("angle_threshold", 60.0)
        self.declare_parameter("distance_threshold", 0.3)
        self.declare_parameter("pose_angle_threshold", 0.1)
        self.declare_parameter("pose_distance_threshold", 0.1)
        self.declare_parameter("workspace_length", 0.8)
        self.declare_parameter("workspace_length_error_threshold", 0.1)
        self.declare_parameter("workspace_safety_distance", 0.2)
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("common_time_lookup_threshold", 5.0)
        self.declare_parameter("line_segment_msg_wait_threshold", 15.0)
        self.declare_parameter("poses_of_workspaces", "/script_server/base")
        self.declare_parameter("require_workspace_match", True)
        self._require_workspace_match = self.get_parameter("require_workspace_match").value

        # auto-iterate convergence loop — repeats alignment automatically
        # instead of requiring repeated manual e_trigger sends
        self.declare_parameter("max_align_iterations", 3)
        self.declare_parameter("nav_arrival_timeout", 20.0)
        self.declare_parameter("converged_angle_threshold", 0.03)
        self.declare_parameter("converged_lateral_threshold", 0.03)
        self._max_align_iterations = self.get_parameter("max_align_iterations").value
        self._nav_arrival_timeout = self.get_parameter("nav_arrival_timeout").value
        self._converged_angle_threshold = self.get_parameter("converged_angle_threshold").value
        self._converged_lateral_threshold = self.get_parameter("converged_lateral_threshold").value

        self._num_of_msgs = self.get_parameter("num_of_msgs").value
        self._angle_threshold = self.get_parameter("angle_threshold").value
        self._distance_threshold = self.get_parameter("distance_threshold").value
        self._pose_angle_threshold = self.get_parameter("pose_angle_threshold").value
        self._pose_distance_threshold = self.get_parameter("pose_distance_threshold").value
        self._workspace_length = self.get_parameter("workspace_length").value
        self._workspace_length_error_threshold = self.get_parameter("workspace_length_error_threshold").value
        self._workspace_safety_distance = self.get_parameter("workspace_safety_distance").value
        self._target_frame = self.get_parameter("target_frame").value
        self._common_time_lookup_threshold = self.get_parameter("common_time_lookup_threshold").value
        self._line_segment_msg_wait_threshold = self.get_parameter("line_segment_msg_wait_threshold").value

        # workspace poses — declared as a JSON string param
        # format: {"ws_name": [x, y, yaw], ...}
        self.declare_parameter("workspace_poses", "{}")
        try:
            ws_poses_param = self.get_parameter("workspace_poses").value
            self._workspace_poses = json.loads(ws_poses_param)
        except Exception:
            self._workspace_poses = None

        if not self._workspace_poses:
            self.get_logger().error("No workspace_pose found. Please check ws pose param")
        print(self._workspace_poses)

        # TF2
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # nav2 action client — used to actually drive correction moves
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._nav_goal_handle = None
        self._nav_timeout_timer = None
        self._pending_nav_success_cb = None
        self._pending_nav_failure_cb = None

        # Class variables
        self._line_segment_msgs = []
        self._counter = self._num_of_msgs
        self._collecting = False
        self._current_ws_name = None
        self._collect_deadline = None
        self._check_timer = None
        self._iteration = 0
        self._last_angle_error = None
        self._last_lateral_offset = None
        self._last_target_pose = None          # geometry_msgs/Pose, for the final-creep math
        self._last_target_pose_stamped = None  # geometry_msgs/PoseStamped, sent to nav2

        # Subscribers
        self.create_subscription(LineSegmentList, "line_segments", self._line_segment_cb, 10)
        self.create_subscription(String, "event_in", self._event_in_cb, 10)

        # Publishers
        self._event_out_pub  = self.create_publisher(String, "event_out", 1)
        # Debug/visualisation only — nothing drives the robot off this topic anymore.
        self._pose_debug_pub = self.create_publisher(PoseStamped, "destination_pose", 1)

        self.get_logger().info(f"Initialised {NODE}")

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _event_in_cb(self, msg: String):
        """Callback function for event in topic. Checks workspace validity
        immediately (matches ROS1 ordering), then starts the auto-iterate
        alignment cycle (may collect + correct multiple times internally).

        :msg: std_msgs/String
        :returns: None
        """
        self.get_logger().info(str(msg))
        if msg.data == "e_trigger":
            if self._require_workspace_match:
                current_ws_name = self._check_workspace_validity()
                if not current_ws_name:
                    self.get_logger().warn("Invalid workspace to align")
                    self._event_out_pub.publish(String(data="Failed"))
                    return
                self.get_logger().info(f"Aligning with {current_ws_name}")
                self._current_ws_name = current_ws_name
            else:
                self.get_logger().info("Skipping workspace pose match (require_workspace_match=False)")
                self._current_ws_name = None

            self._iteration = 0
            self._last_angle_error = None
            self._last_lateral_offset = None
            self._last_target_pose = None
            self._last_target_pose_stamped = None
            self._cancel_nav_timeout()
            self._start_collection_cycle()

    def _start_collection_cycle(self):
        """(Re)arm non-blocking line segment collection — no nested spin_once,
        no deadlock. Called on initial trigger and again by the auto-iterate
        loop after each nav2 correction move completes.
        """
        self._line_segment_msgs = []
        self._counter = 0
        self._collecting = True
        now = self.get_clock().now().nanoseconds / 1e9
        self._collect_deadline = now + self._line_segment_msg_wait_threshold

        if self._check_timer is not None:
            self._check_timer.cancel()
        self._check_timer = self.create_timer(0.1, self._check_collection)

    def _check_workspace_validity(self):
        """Check if the robot's current position matches a known workspace pose.
        :returns: workspace name (str) or None
        """
        try:
            tf_map_base = self._tf_buffer.lookup_transform(
                "map", "base_link",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self._common_time_lookup_threshold)
            )
        except Exception as e:
            self.get_logger().error(f"Could not get map→base_link TF: {e}")
            return None

        position = [
            tf_map_base.transform.translation.x,
            tf_map_base.transform.translation.y,
        ]
        quat = [
            tf_map_base.transform.rotation.x,
            tf_map_base.transform.rotation.y,
            tf_map_base.transform.rotation.z,
            tf_map_base.transform.rotation.w,
        ]

        for ws_name in self._workspace_poses:
            linear_distance = self._get_distance(
                position[:2], self._workspace_poses[ws_name][:2]
            )
            current_angle = euler_from_quaternion(quat)[2]
            angular_distance = abs(current_angle - self._workspace_poses[ws_name][2])
            if (
                linear_distance < self._pose_distance_threshold
                and angular_distance < self._pose_angle_threshold
            ):
                return ws_name
        return None

    def _check_collection(self):
        """Timer callback — fires every 0.1s to check collection progress.
        Finishes (success or timeout) without ever blocking the executor.
        """
        try:
            self._check_collection_impl()
        except Exception as e:
            self.get_logger().error(f"_check_collection crashed: {e}\n{traceback.format_exc()}")
            if self._check_timer is not None:
                self._check_timer.cancel()
            self._collecting = False
            self._event_out_pub.publish(String(data="Failed"))

    def _check_collection_impl(self):
        now = self.get_clock().now().nanoseconds / 1e9

        enough_msgs = self._counter >= self._num_of_msgs
        timed_out = now > self._collect_deadline

        if not enough_msgs and not timed_out:
            return  # keep waiting, next tick will check again

        self._check_timer.cancel()
        self._collecting = False

        if not enough_msgs:
            self.get_logger().warn(
                f"Timed out collecting line segments ({self._counter}/{self._num_of_msgs})"
            )
            self._event_out_pub.publish(String(data="Failed"))
            return

        self.get_logger().info(f"Collection complete ({self._counter} msgs), computing pose...")
        success = self._finish_alignment()

        if not success:
            self.get_logger().info("Alignment result: Failed")
            self._event_out_pub.publish(String(data="Failed"))
            return

        converged = (
            self._last_angle_error is not None
            and abs(self._last_angle_error) < self._converged_angle_threshold
            and abs(self._last_lateral_offset) < self._converged_lateral_threshold
        )

        if converged or self._iteration >= self._max_align_iterations:
            if not converged:
                self.get_logger().warn(
                    f"Reached max_align_iterations ({self._max_align_iterations}) without full "
                    f"convergence (angle={self._last_angle_error:.3f}, lateral={self._last_lateral_offset:.3f}) "
                    f"— accepting best effort"
                )
            self._send_final_creep()
            return

        # Not converged yet — send the correction move to nav2 and wait for
        # its actual result instead of publishing a pose and polling TF.
        self._iteration += 1
        self.get_logger().info(
            f"Not yet converged (angle={self._last_angle_error:.3f}, "
            f"lateral={self._last_lateral_offset:.3f}) — iteration {self._iteration}/"
            f"{self._max_align_iterations}, sending correction move to nav2..."
        )
        self._send_nav_goal(
            self._last_target_pose_stamped,
            on_success=self._start_collection_cycle,
            on_failure=lambda: self._event_out_pub.publish(String(data="Failed")),
        )

    def _send_final_creep(self):
        """Close the last bit of distance, no re-measuring. Actually drives
        the robot there via nav2 before declaring the alignment Completed."""
        final_creep = 0.05  # extra meters to close, beyond safe sensing distance
        extra_pose = PoseStamped()
        extra_pose.header.frame_id = self._target_frame
        extra_pose.header.stamp = self.get_clock().now().to_msg()
        extra_pose.pose = self._last_target_pose  # copy last good pose
        yaw = euler_from_quaternion([
            self._last_target_pose.orientation.x, self._last_target_pose.orientation.y,
            self._last_target_pose.orientation.z, self._last_target_pose.orientation.w])[2]
        extra_pose.pose.position.x += final_creep * math.cos(yaw)
        extra_pose.pose.position.y += final_creep * math.sin(yaw)

        def _on_creep_done(failed=False):
            if failed:
                self.get_logger().warn(
                    "Final creep move did not complete cleanly — alignment already "
                    "converged, so accepting it anyway."
                )
            self.get_logger().info(f"Final creep pose (+{final_creep} m) reached")
            self.get_logger().info("Alignment result: Completed")
            self._event_out_pub.publish(String(data="Completed"))

        self.get_logger().info(f"Sending final creep pose (+{final_creep} m) to nav2")
        self._send_nav_goal(
            extra_pose,
            on_success=lambda: _on_creep_done(failed=False),
            on_failure=lambda: _on_creep_done(failed=True),
        )

    # ------------------------------------------------------------------
    # nav2 correction-move handling
    # ------------------------------------------------------------------

    def _send_nav_goal(self, pose_stamped, on_success, on_failure):
        """Send a NavigateToPose goal for a small alignment correction move
        and invoke on_success()/on_failure() once nav2 reports a result (or
        the nav_arrival_timeout failsafe fires).
        """
        # keep a fresh debug marker of where we're sending the robot
        self._pose_debug_pub.publish(pose_stamped)

        if not self._nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("navigate_to_pose action server not available")
            on_failure()
            return

        goal = NavigateToPose.Goal()
        goal.pose = pose_stamped
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        self._pending_nav_success_cb = on_success
        self._pending_nav_failure_cb = on_failure

        send_future = self._nav_client.send_goal_async(goal)
        send_future.add_done_callback(self._nav_goal_response_cb)

        self._cancel_nav_timeout()
        self._nav_timeout_timer = self.create_timer(self._nav_arrival_timeout, self._on_nav_timeout)

    def _nav_goal_response_cb(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f"nav2 goal send failed: {e}")
            self._cancel_nav_timeout()
            self._fail_pending_nav()
            return

        if not goal_handle.accepted:
            self.get_logger().error("nav2 rejected the alignment correction goal")
            self._cancel_nav_timeout()
            self._fail_pending_nav()
            return

        self._nav_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_result_cb)

    def _nav_result_cb(self, future):
        self._cancel_nav_timeout()
        self._nav_goal_handle = None
        try:
            status = future.result().status
        except Exception as e:
            self.get_logger().error(f"nav2 result retrieval failed: {e}")
            self._fail_pending_nav()
            return

        if status == STATUS_SUCCEEDED:
            self._succeed_pending_nav()
        else:
            self.get_logger().error(f"nav2 failed to reach alignment correction pose (status={status})")
            self._fail_pending_nav()

    def _on_nav_timeout(self):
        self._nav_timeout_timer.cancel()
        self._nav_timeout_timer = None
        self.get_logger().warn("Timed out waiting for nav2 to reach alignment correction pose")
        if self._nav_goal_handle is not None:
            self._nav_goal_handle.cancel_goal_async()
        self._fail_pending_nav()

    def _cancel_nav_timeout(self):
        if self._nav_timeout_timer is not None:
            self._nav_timeout_timer.cancel()
            self._nav_timeout_timer = None

    def _succeed_pending_nav(self):
        cb, self._pending_nav_success_cb = self._pending_nav_success_cb, None
        self._pending_nav_failure_cb = None
        if cb is not None:
            cb()

    def _fail_pending_nav(self):
        cb, self._pending_nav_failure_cb = self._pending_nav_failure_cb, None
        self._pending_nav_success_cb = None
        if cb is not None:
            cb()

    def _line_segment_cb(self, msg: LineSegmentList):
        """Callback function for line segment list message (published from line
        extractor)

        :msg: laser_line_extraction/LineSegmentList
        :returns: None
        """
        if self._collecting and self._counter < self._num_of_msgs:
            self._line_segment_msgs.append(msg)
            self._counter += 1
            if self._counter % 10 == 0 and self._counter > 0:
                self.get_logger().info(str(self._counter))

    # ------------------------------------------------------------------
    # Core logic
    # ------------------------------------------------------------------

    def _finish_alignment(self):
        """Compute the aligned pose from collected line segments and stash it
        for the caller to send to nav2. Called once enough messages have
        been collected (non-blocking path).
        :returns: bool (success)
        """
        workspace_line_segment = self._get_avg_line_segment()

        if not workspace_line_segment:
            self.get_logger().warn(
                f"No matching line segment found among {len(self._line_segment_msgs) or self._num_of_msgs} "
                f"collected messages (check angle_threshold/distance_threshold/workspace_length "
                f"against actual scan geometry)"
            )
            return False

        workspace_pose = PoseStamped()

        q = quaternion_from_euler(0, 0, workspace_line_segment.angle)
        workspace_pose.pose.orientation.x = float(q[0])
        workspace_pose.pose.orientation.y = float(q[1])
        workspace_pose.pose.orientation.z = float(q[2])
        workspace_pose.pose.orientation.w = float(q[3])

        # take midpoint of line segment
        workspace_pose.pose.position.x = float(
            (workspace_line_segment.start[0] + workspace_line_segment.end[0]) / 2
        )
        workspace_pose.pose.position.y = float(
            (workspace_line_segment.start[1] + workspace_line_segment.end[1]) / 2
        )
        workspace_pose.pose.position.z = 0.0

        # track convergence: angle_error should shrink toward 0 (perpendicular),
        # lateral_offset should shrink toward 0 (centered) as iterations progress
        self._last_angle_error = workspace_line_segment.angle
        self._last_lateral_offset = workspace_pose.pose.position.y

        self.get_logger().info(
            f"DEBUG midpoint_local=({workspace_pose.pose.position.x:.3f}, "
            f"{workspace_pose.pose.position.y:.3f}) segment_angle={workspace_line_segment.angle:.3f} "
            f"segment_radius={self._get_distance([0, 0], workspace_line_segment.start):.3f}"
        )

        workspace_pose.header.stamp = self.get_clock().now().to_msg()
        workspace_pose.header.frame_id = "lidar_link"

        # subtract distance from workspace pose (assuming laser in center of robot)
        try:
            tf_base_to_laser = self._tf_buffer.lookup_transform(
                "base_link", "lidar_link", rclpy.time.Time()
            )
            laser_x = tf_base_to_laser.transform.translation.x
        except Exception as e:
            self.get_logger().warn(f"Could not get base_link→lidar_link TF: {e}. Using 0.")
            laser_x = 0.0

        total_distance = self._workspace_safety_distance + laser_x
        self.get_logger().info(
            f"DEBUG laser_x={laser_x:.3f} safety_distance={self._workspace_safety_distance:.3f} "
            f"total_distance={total_distance:.3f}"
        )
        workspace_pose.pose.position.x = float(
            workspace_pose.pose.position.x - total_distance * math.cos(workspace_line_segment.angle)
        )
        workspace_pose.pose.position.y = float(
            workspace_pose.pose.position.y - total_distance * math.sin(workspace_line_segment.angle)
        )
        self.get_logger().info(
            f"DEBUG target_local_after_backoff=({workspace_pose.pose.position.x:.3f}, "
            f"{workspace_pose.pose.position.y:.3f})"
        )

        try:
            tf_to_target = self._tf_buffer.lookup_transform(
                self._target_frame,
                "lidar_link",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0)
            )
            pose_in_target = tf2_geometry_msgs.do_transform_pose(
                workspace_pose.pose, tf_to_target
            )
            result_pose = PoseStamped()
            result_pose.header.frame_id = self._target_frame
            result_pose.header.stamp = self.get_clock().now().to_msg()
            result_pose.pose = pose_in_target
            result_pose.pose.position.z = 0.0

            self._last_target_pose = result_pose.pose
            self._last_target_pose_stamped = result_pose
            return True
        except Exception as e:
            self.get_logger().error(str(e))
            return False

    # ------------------------------------------------------------------
    # Line segment helpers
    # ------------------------------------------------------------------

    def _filter_line_segments(self, line_segments):
        """filter out the line segments which are not
        - in the center of the FOV of the laser sensors.
        - near the robot
        All line segments that are not in center or are far away will be discarded.

        :line_segments: list of laser_line_extraction/LineSegment
        :returns: list of laser_line_extraction/LineSegment
        """
        filtered_line_segments = []
        for line_segment in line_segments:
            if (
                abs(math.degrees(line_segment.angle)) < self._angle_threshold
                and line_segment.radius < self._distance_threshold
            ):
                filtered_line_segments.append(line_segment)
        return filtered_line_segments

    def _get_workspace_line_segment(self, line_segments):
        """Return a single line segment as a result of list of line segments which
        fits the workspace specifications.

        :line_segments: list of laser_line_extraction/LineSegment
        :returns: laser_line_extraction/LineSegment or None
        """
        workspace_line_segment = None
        if len(line_segments) == 2:
            dist_1_2 = self._get_distance(line_segments[0].start, line_segments[1].end)
            dist_2_1 = self._get_distance(line_segments[1].start, line_segments[0].end)
            workspace_line_segment = LineSegment()
            if dist_1_2 > dist_2_1:
                workspace_line_segment.start = line_segments[0].start
                workspace_line_segment.end = line_segments[1].end
            else:
                workspace_line_segment.start = line_segments[1].start
                workspace_line_segment.end = line_segments[0].end
            workspace_line_segment.angle = (
                line_segments[0].angle + line_segments[1].angle
            ) / 2
        elif len(line_segments) == 1:
            workspace_line_segment = line_segments[0]

        if workspace_line_segment:
            # filter based on length of line segment
            length = self._get_distance(
                workspace_line_segment.start, workspace_line_segment.end
            )
            if (
                abs(length - self._workspace_length)
                < self._workspace_length_error_threshold
            ):
                return workspace_line_segment
        return None

    def _get_avg_line_segment(self):
        """Return avg line segment obj from the list of line segment messages
        :returns: laser_line_extraction/LineSegment
        """
        workspace_line_segment = None
        start_x, start_y, end_x, end_y, angle = 0, 0, 0, 0, 0
        for msg in self._line_segment_msgs:
            filtered_line_segments = self._filter_line_segments(msg.line_segments)
            workspace_line_segment = self._get_workspace_line_segment(
                filtered_line_segments
            )
            if workspace_line_segment:
                start_x += workspace_line_segment.start[0]
                start_y += workspace_line_segment.start[1]
                end_x += workspace_line_segment.end[0]
                end_y += workspace_line_segment.end[1]
                angle += workspace_line_segment.angle
        self._line_segment_msgs = []
        # create an avg line segment if non zero total values
        if start_x != 0 and start_y != 0:
            workspace_line_segment = LineSegment()
            workspace_line_segment.start = [
                float(start_x / self._num_of_msgs),
                float(start_y / self._num_of_msgs),
            ]
            workspace_line_segment.end = [
                float(end_x / self._num_of_msgs),
                float(end_y / self._num_of_msgs),
            ]
            workspace_line_segment.angle = float(angle / self._num_of_msgs)
        return workspace_line_segment

    def _get_distance(self, p1, p2):
        return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5


# ----------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = WorkspaceAligner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
    
    
    


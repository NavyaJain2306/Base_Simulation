import rclpy
from rclpy.node import Node

from std_srvs.srv import SetBool
from perception_msgs.srv import PerceptionSrv
from perception_msgs.msg import PerceptionMsg
from geometry_msgs.msg import PoseArray, Pose
from shape_msgs.msg import SolidPrimitive
from rclpy.task import Future
from rclpy.callback_groups import ReentrantCallbackGroup

import numpy as np
TIMEOUT_COUNT = 50

class PerceptionServer(Node):
    def __init__(self):
        super().__init__('perception_server')

        self.cb_group = ReentrantCallbackGroup()

        self.blocks_service = self.create_service(
            PerceptionSrv, '/perception/get_averaged_pose',
            self.handle_collection_request, callback_group=self.cb_group)

        self.pcl_toggle_client = self.create_client(SetBool, '/perception/set_active')

        self.sub_ = None
        self.collected_frames = []

    async def handle_collection_request(self, request, response):
        self.collected_frames = []
        self.target_samples = request.target_samples
        self.collect_future = Future()

        await self.send_set_bool(True)
        self.sub = self.create_subscription(
            PerceptionMsg, "/perception/brick_data", self.pose_expanded_callback,
            10, callback_group=self.cb_group)

        timeout_timer = self.create_timer(
            TIMEOUT_COUNT * 0.10, self._on_collection_timeout, callback_group=self.cb_group)

        await self.collect_future

        timeout_timer.cancel()
        self.destroy_subscription(self.sub)
        await self.send_set_bool(False)

        response.block_poses.poses, response.block_ids, response.block_classes, response.block_dims = \
            self.process_frames_and_average(self.collected_frames)
        return response

    def pose_expanded_callback(self, msg):
        if len(msg.block_poses.poses) > 0:
            self.collected_frames.append(msg)
            if len(self.collected_frames) >= self.target_samples and not self.collect_future.done():
                self.collect_future.set_result(True)

    def process_frames_and_average(self, collected_frames):
        if not collected_frames:
            return [], [], [], []

        valid_frames = []

        collected_frames.sort(key=lambda x: -len(x.block_ids))
        check_list = collected_frames[0].block_ids
        check_list = [x for x in check_list if x != "-1"]

        id_dict = {key: [] for key in check_list}
        dim_dict = {key: [] for key in check_list}
        class_dict = {}

        for frame in collected_frames:
            recurrent = True
            for b_id in frame.block_ids:
                if b_id not in check_list and b_id != "-1":
                    recurrent = False
                    break
            if recurrent:
                valid_frames.append(frame)

        for frame in valid_frames:
            for idx in range(len(frame.block_ids)):
                b_id = frame.block_ids[idx]
                if b_id in id_dict:
                    id_dict[b_id].append(frame.block_poses.poses[idx])
                    dim_dict[b_id].append(frame.block_dims[idx])
                    class_dict[b_id] = frame.block_classes[idx]

        averaged_poses = []
        final_ids = []
        final_classes = []
        final_dims = []

        for block_id, pose_list in id_dict.items():
            if not pose_list:
                continue

            positions = []
            quaternions = []
            for p in pose_list:
                positions.append([p.position.x, p.position.y, p.position.z])
                quaternions.append([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])

            avg_pos = np.mean(positions, axis=0)
            q_ref = quaternions[0]
            corrected_quaternions = []
            for q in quaternions:
                if np.dot(q_ref, q) < 0:
                    corrected_quaternions.append(-np.array(q))
                else:
                    corrected_quaternions.append(np.array(q))

            avg_quat = np.mean(corrected_quaternions, axis=0)
            avg_quat = avg_quat / np.linalg.norm(avg_quat)

            final_pose = Pose()
            final_pose.position.x, final_pose.position.y, final_pose.position.z = avg_pos[0], avg_pos[1], avg_pos[2]
            final_pose.orientation.x, final_pose.orientation.y, final_pose.orientation.z, final_pose.orientation.w = avg_quat[0], avg_quat[1], avg_quat[2], avg_quat[3]

            dims_list = dim_dict[block_id]
            x_dims = [d.dimensions[SolidPrimitive.BOX_X] for d in dims_list]
            y_dims = [d.dimensions[SolidPrimitive.BOX_Y] for d in dims_list]
            z_dims = [d.dimensions[SolidPrimitive.BOX_Z] for d in dims_list]

            avg_shape = SolidPrimitive()
            avg_shape.type = SolidPrimitive.BOX
            avg_shape.dimensions = [
                float(np.mean(x_dims)),
                float(np.mean(y_dims)),
                float(np.mean(z_dims))
            ]

            averaged_poses.append(final_pose)
            final_dims.append(avg_shape)
            final_ids.append(block_id)
            final_classes.append(class_dict[block_id])

        return averaged_poses, final_ids, final_classes, final_dims

    async def send_set_bool(self, msg: bool):
        while not self.pcl_toggle_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info("PCL Service unavailable")
        req = SetBool.Request()
        req.data = msg

        try:
            response = await self.pcl_toggle_client.call_async(request=req)
            return response.success
        except Exception as e:
            self.get_logger().error("Request Failed to Toggle PCL Node")
            return False

    def _on_collection_timeout(self):
        if not self.collect_future.done():
            self.collect_future.set_result(True)

def main(args=None):
    rclpy.init(args=args)
    perception_server = PerceptionServer()
    rclpy.spin(perception_server)
    perception_server.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
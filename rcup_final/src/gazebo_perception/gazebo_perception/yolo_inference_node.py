import rclpy
from rclpy.node import Node
from ultralytics import YOLO
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray


class YOLOInferenceNode(Node):
    def __init__(self):
        super().__init__("yolo_inference_node")

        self.bridge = CvBridge()
        self.logger = self.get_logger()

        self.logger.info("Loading Model")
        model_path = "/home/akshit/Documents/runs_backup/runs/obb/train/weights/best.pt"
        self.model = YOLO(model=model_path)
        self.logger.info("Model loaded successfully")

        self.subscriber_ = self.create_subscription(
            Image, "/rgbd_camera/image", self.image_callback, 10
        )
        self.annotated_pub = self.create_publisher(Image, "/mujoco/annotated_image", 10)
        self.detection_pub = self.create_publisher(
            Detection2DArray, "/vision/detected_objects", 10
        )
        self.get_logger().info("GAZEBO_WS VERSION")

    def image_callback(self, msg):

        cv_image = self.bridge.imgmsg_to_cv2(img_msg=msg, desired_encoding="bgr8")

        results = self.model.track(cv_image, verbose=False,  persist=True, conf=0.6)

        annotated_image = results[0].plot()
        annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding="bgr8")

        annotated_msg.header = msg.header

        self.annotated_pub.publish(annotated_msg)

        detection_array = Detection2DArray()
        detection_array.header = msg.header

        if results[0].obb is not None:
            for box in results[0].obb:
                xywhr = box.xywhr[0].cpu().numpy()
                det = Detection2D()
                det.header = msg.header

                det.bbox.center.position.x = float(xywhr[0])
                det.bbox.center.position.y = float(xywhr[1])

                det.bbox.center.theta = float(xywhr[4])

                det.bbox.size_x = float(xywhr[2])
                det.bbox.size_y = float(xywhr[3])
                
                box_class_id = int(box.cls[0].item())
                box_class = self.model.names[box_class_id]
                if box.id is not None:
                    box_id = str(int(box.id[0].item()))
                else:
                    box_id = str(-1)
                det.id = ".".join([box_class, box_id])

                # self.get_logger().info(det.id)
                detection_array.detections.append(det)

        self.detection_pub.publish(detection_array)


def main(args=None):
    rclpy.init(args=args)
    node = YOLOInferenceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

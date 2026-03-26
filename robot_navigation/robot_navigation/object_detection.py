import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import os
import threading
import ament_index_python
import ultralytics
import cv2

class ObjectDetection(Node):
    def __init__(self):
        super().__init__('object_detection')

        # Subscribers and Publishers
        self.camera_sub = self.create_subscription(
            Image, '/depth_camera/image_raw', self.frame_callback, 10
        )
        self.object_pub = self.create_publisher(
            Image, '/camera/object_detection', 10
        )

        self.bridge = CvBridge()

        # Load YOLOv8 model
        model_path = os.path.join(
            ament_index_python.get_package_share_directory('robot_navigation'),
            'models', 'best.pt'
        )
        self.model = ultralytics.YOLO(model_path)
        self.get_logger().info(f"Loaded YOLOv8 model from: {model_path}")

        # Frame storage
        self.latest_frame = None
        self.latest_boxes = None
        self.frame_lock = threading.Lock()

        # Timers
        self.publish_rate_hz = 20     # Publish live feed at 20 Hz
        self.inference_rate_hz = 2    # Run YOLO inference at 2 Hz
        self.create_timer(1.0 / self.publish_rate_hz, self.publish_frame)
        self.create_timer(1.0 / self.inference_rate_hz, self.run_inference)

    def frame_callback(self, msg):
        """Store the latest camera frame."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            with self.frame_lock:
                self.latest_frame = cv_image
        except Exception as e:
            self.get_logger().error(f"Failed to convert incoming frame: {e}")

    def run_inference(self):
        """Run YOLO inference on the latest frame at a slower rate."""
        with self.frame_lock:
            if self.latest_frame is None:
                return
            frame_copy = self.latest_frame.copy()

        try:
            results = self.model.predict(frame_copy, verbose=False)
            self.latest_boxes = results[0] if len(results[0].boxes) > 0 else None
        except Exception as e:
            self.get_logger().error(f"YOLO inference failed: {e}")

    def publish_frame(self):
        """Publish the latest frame with overlayed YOLO results."""
        with self.frame_lock:
            if self.latest_frame is None:
                return
            frame_to_publish = self.latest_frame.copy()
            boxes = self.latest_boxes

        # Overlay YOLO bounding boxes if available
        if boxes is not None:
            try:
                frame_to_publish = boxes.plot()
            except Exception as e:
                self.get_logger().error(f"Failed to overlay YOLO results: {e}")

        try:
            pub_msg = self.bridge.cv2_to_imgmsg(frame_to_publish, "bgr8")
            pub_msg.header.stamp = self.get_clock().now().to_msg()
            pub_msg.header.frame_id = "camera_link"
            self.object_pub.publish(pub_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish annotated frame: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down due to keyboard interrupt.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

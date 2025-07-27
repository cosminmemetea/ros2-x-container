import os
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# Explicit imports for sensor inits (add new ones here for extensions)
from .extensions.sensors.webcam import init as init_webcam
from .extensions.sensors.mjpeg import init as init_mjpeg
from .extensions.sensors.csi import init as init_csi
from .extensions.sensors.custom_sensor import init as init_custom_sensor

# Explicit imports for ML inits (add new ones here for extensions)
from .extensions.algorithms.yolo import init as init_yolo
from .extensions.algorithms.edge_detection import init as init_edge_detection
from .extensions.algorithms.custom_algo import init as init_custom_algo

class StreamNode(Node):
    def __init__(self):
        super().__init__('stream_node')
        self.raw_publisher = self.create_publisher(Image, '/image_raw', 10)
        self.annotated_publisher = self.create_publisher(Image, '/annotated_image', 10)
        self.bridge = CvBridge()

        # FPS from env var (default 15)
        fps = float(os.environ.get('FPS', 15))
        self.timer = self.create_timer(1.0 / fps, self.timer_callback)
        self.get_logger().info(f'Set FPS to {fps}')
        self.frame_count = 0

        # Modular Camera Setup (explicit factory)
        camera_type = os.environ.get('CAMERA_TYPE', 'mjpeg')
        camera_factories = {
            'webcam': init_webcam,
            'mjpeg': init_mjpeg,
            'csi': init_csi,
            'custom_sensor': init_custom_sensor
        }
        if camera_type in camera_factories:
            self.cap = camera_factories[camera_type]()
            self.get_logger().info(f'Initialized camera: {camera_type}')
        else:
            self.get_logger().error(f'Unsupported CAMERA_TYPE: {camera_type}. Available: {list(camera_factories.keys())}')
            return  # Exit init without shutdown to avoid rclpy error

        # Modular ML Setup (explicit factory, with backward compat for YOLO_MODEL)
        self.ml_type = os.environ.get('ML_TYPE', '') or ( 'yolo' if os.environ.get('YOLO_MODEL', '') else '' )  # Compat: If YOLO_MODEL set, use 'yolo'
        ml_factories = {
            'yolo': init_yolo,
            'edge_detection': init_edge_detection,
            'custom_algo': init_custom_algo
        }
        self.ml_processor = None
        if self.ml_type in ml_factories:
            self.ml_processor = ml_factories[self.ml_type]()  # Returns callable
            self.get_logger().info(f'Initialized ML processor: {self.ml_type}')
        else:
            self.get_logger().info('No ML processor enabled (set ML_TYPE env, e.g., "yolo" or use YOLO_MODEL for compat)')

    def timer_callback(self):
        if not hasattr(self, 'cap') or not self.cap.isOpened():
            self.get_logger().warn('No valid camera source - skipping frame')
            return

        ret, frame = self.cap.read()
        if ret:
            self.frame_count += 1
            height, width = frame.shape[:2]
            is_empty = frame.size == 0 or not frame.any()

            if self.frame_count % 15 == 0:
                self.get_logger().info(f'Frame read: shape={(height, width)}, dtype={frame.dtype}, empty={is_empty}, count={self.frame_count}')

            # Publish raw frame
            try:
                raw_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                self.raw_publisher.publish(raw_msg)
                if self.frame_count % 15 == 0:
                    self.get_logger().info(f'Published raw image: width={raw_msg.width}, height={raw_msg.height}')
            except CvBridgeError as e:
                self.get_logger().error(f'CvBridge error (raw): {e}')

            # Modular ML Processing
            annotated_frame = frame.copy()
            if self.ml_processor:
                annotated_frame = self.ml_processor(frame)

            # Publish annotated frame if ML active
            if self.ml_processor:
                try:
                    annotated_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
                    self.annotated_publisher.publish(annotated_msg)
                    if self.frame_count % 15 == 0:
                        self.get_logger().info(f'Published annotated image: width={annotated_msg.width}, height={annotated_msg.height}')
                except CvBridgeError as e:
                    self.get_logger().error(f'CvBridge error (annotated): {e}')
        else:
            self.get_logger().warn('Failed to read frame from stream')

def main(args=None):
    rclpy.init(args=args)
    node = StreamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'cap') and node.cap.isOpened():
            node.cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
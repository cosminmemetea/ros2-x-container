import os
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# Example extension: Add simple processing (e.g., grayscale) or ML (commented)
# import torch
# model = torch.hub.load('ultralytics/yolov5', 'yolov5s')  # Example ML model for detection (requires pip install ultralytics in Dockerfile if enabled)

class StreamNode(Node):
    def __init__(self):
        super().__init__('stream_node')
        self.publisher_ = self.create_publisher(Image, '/image_raw', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(1.0 / 30, self.timer_callback)  # ~15 FPS
        self.frame_count = 0  # Counter for less verbose logging

        # Get stream URL from env var (adapted for Mac/Docker)
        url = os.environ.get('STREAM_URL', 'http://host.docker.internal:8080/source_0')
        self.cap = cv2.VideoCapture(url)
        if self.cap.isOpened():
            self.get_logger().info(f'Successfully opened stream at {url}')
        else:
            self.get_logger().error(f'Failed to open stream at {url}')
            rclpy.shutdown()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # Log frame details every 15 frames (~1/sec)
            self.frame_count += 1
            if self.frame_count % 15 == 0:
                height, width = frame.shape[:2] if frame.ndim == 3 else (0, 0)
                is_empty = frame.size == 0 or not frame.any()  # Check if frame is zero/empty
                self.get_logger().info(f'Frame read: shape={frame.shape}, dtype={frame.dtype}, empty={is_empty}, count={self.frame_count}')

            # Example algorithm processing: Detect edges with Canny
            # edges = cv2.Canny(frame, 100, 200)
            # frame = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)  # Convert back for publish

            # Example ML: Predictions (commented; enable and add dependencies)
            # results = model(frame)
            # frame = results.render()[0]  # Render detections on frame

            try:
                img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                self.publisher_.publish(img_msg)
                if self.frame_count % 15 == 0:
                    self.get_logger().info(f'Published image: width={img_msg.width}, height={img_msg.height}, encoding={img_msg.encoding}')
            except CvBridgeError as e:
                self.get_logger().error(f'CvBridge error during conversion: {e}')
        else:
            self.get_logger().warn('Failed to read frame from stream')

def main(args=None):
    rclpy.init(args=args)
    node = StreamNode()
    try:
        rclpy.spin(node)
    finally:
        if hasattr(node, 'cap') and node.cap.isOpened():
            node.cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
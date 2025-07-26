import os
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Example extension: Add simple processing (e.g., grayscale) or ML (commented)
# import torch
# model = torch.hub.load('ultralytics/yolov5', 'yolov5s')  # Example ML model for detection (requires pip install ultralytics in Dockerfile if enabled)

class StreamNode(Node):
    def __init__(self):
        super().__init__('stream_node')
        self.publisher_ = self.create_publisher(Image, '/image_raw', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(1.0 / 15, self.timer_callback)  # ~15 FPS

        # Get stream URL from env var (adapted for Mac/Docker)
        url = os.environ.get('STREAM_URL', 'http://host.docker.internal:8080/source_0')
        self.cap = cv2.VideoCapture(url)
        if self.cap.isOpened():
            self.get_logger().info(f'Successfully opened stream at {url}')  # Confirmation message on open
        else:
            self.get_logger().error(f'Failed to open stream at {url}')
            rclpy.shutdown()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            self.get_logger().info('Frame read successfully from stream')  # Message on each frame read (for debug; comment if too verbose)
            # print('Frame read successfully')  # Alternative with print for direct console

            # Example algorithm processing: Detect edges with Canny
            # edges = cv2.Canny(frame, 100, 200)
            # frame = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)  # Convert back for publish

            # Example ML: Predictions (commented; enable and add dependencies)
            # results = model(frame)
            # frame = results.render()[0]  # Render detections on frame

            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(img_msg)
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
import os
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Exemplu extensie: Adaugă procesare simplă (e.g., grayscale) sau ML (comentat)
# import torch
# model = torch.hub.load('ultralytics/yolov5', 'yolov5s')  # Exemplu ML model pentru detecție (necesită pip install ultralytics în Dockerfile dacă activezi)

class StreamNode(Node):
    def __init__(self):
        super().__init__('stream_node')
        self.publisher_ = self.create_publisher(Image, '/image_raw', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(1.0 / 15, self.timer_callback)  # ~15 FPS

        # Get stream URL from env var (adaptat pentru Mac/Docker)
        url = os.environ.get('STREAM_URL', 'http://host.docker.internal:8080/source_0')
        self.cap = cv2.VideoCapture(url)
        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open stream at {url}')
            rclpy.shutdown()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # Exemplu procesare algoritm: Detect edges cu Canny
            # edges = cv2.Canny(frame, 100, 200)
            # frame = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)  # Convert back pentru publish

            # Exemplu ML: Predicții (comentat; activează și adaugă dependențe)
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
import os
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class StreamNode(Node):
    def __init__(self):
        super().__init__('stream_node')
        self.raw_publisher = self.create_publisher(Image, '/image_raw', 10)
        self.annotated_publisher = self.create_publisher(Image, '/annotated_image', 10)
        self.bridge = CvBridge()

        # FPS from env var (default 30)
        fps = float(os.environ.get('FPS', 30))
        self.timer = self.create_timer(1.0 / fps, self.timer_callback)
        self.get_logger().info(f'Set FPS to {fps}')
        self.frame_count = 0

        # Stream URL
        url = os.environ.get('STREAM_URL', 'http://host.docker.internal:8080/source_0')
        self.cap = cv2.VideoCapture(url)
        if self.cap.isOpened():
            self.get_logger().info(f'Successfully opened stream at {url}')
        else:
            self.get_logger().error(f'Failed to open stream at {url}')
            rclpy.shutdown()

        # YOLOv3 setup (by Joseph Redmon & Darknet community)
        self.yolo_model = os.environ.get('YOLO_MODEL', '')  # 'standard' or 'tiny'
        self.confidence_threshold = float(os.environ.get('YOLO_CONFIDENCE', 0.5))  # Default 0.5, range 0-1
        if self.yolo_model:
            # Map 'standard' to 'yolov3', 'tiny' to 'yolov3-tiny'
            model_prefix = 'yolov3' if self.yolo_model == 'standard' else 'yolov3-tiny'
            cfg = f'/root/ros2_ws/src/stream2ros/models/{model_prefix}.cfg'
            weights = f'/root/ros2_ws/src/stream2ros/models/{model_prefix}.weights'
            names = '/root/ros2_ws/src/stream2ros/models/coco.names'

            self.net = cv2.dnn.readNet(weights, cfg)
            with open(names, 'r') as f:
                self.classes = [line.strip() for line in f.readlines()]
            layer_names = self.net.getLayerNames()
            self.output_layers = [layer_names[i - 1] for i in self.net.getUnconnectedOutLayers()]
            self.colors = np.random.uniform(0, 255, size=(len(self.classes), 3))
            self.get_logger().info(f'YOLOv3 ({self.yolo_model}) loaded with confidence threshold {self.confidence_threshold}')
        else:
            self.get_logger().info('YOLO disabled (set YOLO_MODEL env to "standard" or "tiny")')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            self.frame_count += 1
            height, width = frame.shape[:2]  # Define here, always available if ret=True
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

            # YOLO detection & annotation if enabled (skip if empty frame)
            annotated_frame = frame.copy()  # For annotation
            if self.yolo_model and not is_empty:
                blob = cv2.dnn.blobFromImage(frame, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
                self.net.setInput(blob)
                outs = self.net.forward(self.output_layers)

                class_ids = []
                confidences = []
                boxes = []
                for out in outs:
                    for detection in out:
                        scores = detection[5:]
                        class_id = np.argmax(scores)
                        confidence = scores[class_id]
                        if confidence > self.confidence_threshold:
                            center_x = int(detection[0] * width)
                            center_y = int(detection[1] * height)
                            w = int(detection[2] * width)
                            h = int(detection[3] * height)
                            x = int(center_x - w / 2)
                            y = int(center_y - h / 2)
                            boxes.append([x, y, w, h])
                            confidences.append(float(confidence))
                            class_ids.append(class_id)

                indexes = cv2.dnn.NMSBoxes(boxes, confidences, self.confidence_threshold, 0.4)
                for i in range(len(boxes)):
                    if i in indexes:
                        x, y, w, h = boxes[i]
                        label = str(self.classes[class_ids[i]])
                        color = self.colors[class_ids[i]]
                        cv2.rectangle(annotated_frame, (x, y), (x + w, y + h), color, 2)
                        cv2.putText(annotated_frame, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                if len(indexes) > 0 and self.frame_count % 15 == 0:
                    self.get_logger().info(f'Detected {len(indexes)} objects (threshold {self.confidence_threshold})')

                # Publish annotated frame
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
    finally:
        if hasattr(node, 'cap') and node.cap.isOpened():
            node.cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
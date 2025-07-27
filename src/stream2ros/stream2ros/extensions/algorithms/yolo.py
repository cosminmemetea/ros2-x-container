import cv2
import numpy as np
import os

def init():
    # YOLOv3 setup (by Joseph Redmon & Darknet community)
    yolo_variant = os.environ.get('YOLO_VARIANT', 'standard')  # 'standard' or 'tiny'
    confidence_threshold = float(os.environ.get('YOLO_CONFIDENCE', 0.5))
    model_prefix = 'yolov3' if yolo_variant == 'standard' else 'yolov3-tiny'
    cfg = f'/root/ros2_ws/src/stream2ros/models/{model_prefix}.cfg'
    weights = f'/root/ros2_ws/src/stream2ros/models/{model_prefix}.weights'
    names = '/root/ros2_ws/src/stream2ros/models/coco.names'

    net = cv2.dnn.readNet(weights, cfg)
    with open(names, 'r') as f:
        classes = [line.strip() for line in f.readlines()]
    layer_names = net.getLayerNames()
    output_layers = [layer_names[i - 1] for i in net.getUnconnectedOutLayers()]
    colors = np.random.uniform(0, 255, size=(len(classes), 3))

    def processor(frame):
        height, width = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(frame, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
        net.setInput(blob)
        outs = net.forward(output_layers)

        class_ids = []
        confidences = []
        boxes = []
        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > confidence_threshold:
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)
                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)

        indexes = cv2.dnn.NMSBoxes(boxes, confidences, confidence_threshold, 0.4)
        annotated_frame = frame.copy()
        for i in range(len(boxes)):
            if i in indexes:
                x, y, w, h = boxes[i]
                label = str(classes[class_ids[i]])
                color = colors[class_ids[i]]
                cv2.rectangle(annotated_frame, (x, y), (x + w, y + h), color, 2)
                cv2.putText(annotated_frame, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        return annotated_frame

    return processor  # Return the processor function
#!/bin/bash
mkdir -p src/stream2ros/models
cd src/stream2ros/models

echo "Downloading YOLOv3 Standard..."
curl -O https://raw.githubusercontent.com/pjreddie/darknet/master/cfg/yolov3.cfg
curl -O https://pjreddie.com/media/files/yolov3.weights

echo "Downloading YOLOv3 Tiny..."
curl -O https://raw.githubusercontent.com/pjreddie/darknet/master/cfg/yolov3-tiny.cfg
curl -O https://pjreddie.com/media/files/yolov3-tiny.weights

echo "Downloading COCO classes..."
curl -O https://raw.githubusercontent.com/pjreddie/darknet/master/data/coco.names

echo "Downloads complete. Ready to build Docker image."
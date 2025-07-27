# ROS2‑X Container Boilerplate
<img width="2320" height="1573" alt="ROS2-X-Container-Overview-Top" src="https://github.com/user-attachments/assets/c42fc63f-7b3a-463d-a1cb-5170c4da1a7e" />

A minimal, state‑of‑the‑art Dockerized ROS 2 setup for real‑time webcam/MJPEG streaming to ROS 2 topics, with seamless integration in Foxglove Studio. This pure-Python node uses OpenCV and cv_bridge to publish raw and annotated images on `/image_raw` and `/annotated_image`, supporting easy extensions for AI/ML (e.g., edge detection, object detection with YOLO/Torch) and processing pipelines. Optimized for arm64 platforms like Mac M2 or NVIDIA Jetson (automotive/edge AI use cases), it includes foxglove_bridge for WebSocket-based monitoring. Ideal for prototyping vision systems in robotics, autonomous vehicles, or automotive perception—fork and extend for your CV/ML workflows!

---

## TL;DR

- **Pulls an MJPEG stream** (from `mjpeg-streamer` or provided Python script)
- **Publishes** to `/image_raw` and `/annotated_image` as `sensor_msgs/Image`
- **Exposes** a ROS 2 WebSocket on port 8765 for Foxglove Studio
- **Supports** YOLOv3/Tiny detection via env vars for object detection demos

---

## Architecture Overview

```text
┌──────────────┐          ┌───────────────┐          ┌─────────────────┐
│ Sensor Node  │──image→──│ ML Processor  │──annot→──│ Qt Visualizer   │
│ (/camera_*)  │          │ (/detection)  │          │ (C++, QML, ROS) │
└─────┬────────┘          └─────┬─────────┘          └─────────────────┘
      │                         │
      │                         └── ML Model Service (/inference)
      │
      │
┌─────▼──────┐
│ Sync Node  │ ← combine multi‑stream / time‑align
└────────────┘
```

---

## Prerequisites

1. **macOS host** with webcam & [`mjpeg-streamer`](https://github.com/jacksonliam/mjpg-streamer) or Python 3  
2. **Docker** (with host network support)  
3. **ROS 2 Humble** base image (pulled automatically by Docker)

---

## Quickstart

### 1. Start your MJPEG stream on macOS

```bash
mjpeg-streamer --source 0 --fps 15 --quality 80 --port 8080
```

Verify in browser: `http://localhost:8080/source_0`

#### 1.1 Alternative: Python MJPEG Streamer

```bash
# Create & activate virtual environment
python3 -m venv venv
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt

# Run the Python streamer
python mjpeg_streamer.py
```

---

### 2. Build the Docker image

```bash
docker build -t orin-container-yolo .
```

---

### 3. Run the container

Use the following command for correct connection to Foxglove Studio:

```bash
docker run --rm -it \
  -e STREAM_URL=http://host.docker.internal:8080/source_0 \
  -e FPS=30 \
  -e YOLO_MODEL=standard \
  -e YOLO_CONFIDENCE=0.6 \
  -p 8765:8765 \
  orin-container-yolo
```

> *Note:* Ports are discarded in host mode; WebSocket will still listen on `8765`.

```bash
docker run --rm -it \
  -e STREAM_URL=http://host.docker.internal:8080/source_0 \
  -e FPS=15 \
  -e CAMERA_TYPE=mjpeg \
  -e ML_TYPE=yolo \
  -e YOLO_MODEL=standard \
  -e YOLO_CONFIDENCE=0.5 \
  -p 8765:8765 \
  orin-container-yolo
```


# Command Explanation

The following command starts the container, pulls an MJPEG stream (e.g., from your Mac webcam via MJPEG‑streamer or Python script), optionally runs ML inference, and exposes a WebSocket for Foxglove Studio:

```bash
docker run --rm -it \
  -e STREAM_URL=http://host.docker.internal:8080/source_0 \
  -e FPS=30 \
  -e CAMERA_TYPE=mjpeg \
  -e ML_TYPE=yolo \
  -e YOLO_MODEL=standard \
  -e YOLO_CONFIDENCE=0.6 \
  -p 8765:8765 \
  orin-container-yolo
```

| Flag                                  | Description                                                                                                                       |
|---------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------|
| `--rm -it`                            | Remove container on exit and run interactively to view real‑time logs.                                                            |
| `-e STREAM_URL=<url>`                 | Input stream URL (default `http://host.docker.internal:8080/source_0`). Can be MJPEG, RTSP, etc.                                  |
| `-e FPS=<n>`                          | Frame rate for the internal timer (default 15). Increase to 30+ for smoother output, but watch hardware load.                     |
| `-e CAMERA_TYPE=<type>`               | Select camera interface:
: • `mjpeg` (default)
: • `webcam`
: • `csi` (Jetson MIPI)
: • `itof`, `infra`, `custom_sensor`                                                        |
| `-e ML_TYPE=<type>`                   | Enable ML pipeline:
: • `none` (default)
: • `yolo`
: • `edge_detection`
: • `custom_algo`                                                                            |
| `-e YOLO_MODEL=<standard|tiny>`       | YOLO variant (requires downloaded models in `src/stream2ros/models`).                                                               |
| `-e YOLO_CONFIDENCE=<0.0–1.0>`        | Detection threshold (lower → more detections; higher → more precise).                                                                |
| `-p 8765:8765`                        | Map container’s WebSocket port `8765` to host (connect in Foxglove at `ws://localhost:8765`).                                      |
| `orin-container-yolo`                 | Docker image name (replace with your tag if different).                                                                            |

---

## How Developers Should Use / Extend It

1. **Basic Run** 
   - Copy the command above, adjust `STREAM_URL` as needed, and run.
   - In Foxglove Studio, add an **Image** panel subscribed to `/image_raw` (and `/annotated_image` if ML is enabled).

2. **Add a New Camera Type**
   - Create `extensions/sensors/your_camera.py` with:
     ```python
     def init():
         return cv2.VideoCapture("your_source_url_or_device")
     ```
   - Rebuild: `docker build -t orin-container .`
   - Run with `-e CAMERA_TYPE=your_camera`.

3. **Add a New ML Algorithm**
   - Create `extensions/algorithms/your_algo.py` with:
     ```python
     def init():
         def processor(frame):
             # your processing
             return processed_frame
         return processor
     ```
   - Rebuild and run with `-e ML_TYPE=your_algo`.

4. **Multi‑Camera / Multi‑ML Pipelines**
   - Extend the sensor/algorithm factories to return lists of capture/processor functions.
   - Use a ROS 2 launch file to spin up multiple nodes in parallel.

5. **Testing & Debugging**
   - Rebuild after any code change:
     ```bash
     docker build -t orin-container .
     ```
   - Run with desired env vars and inspect logs/topics:
     ```bash
     ros2 topic list
     ros2 topic hz /image_raw
     ```

By following this pattern, you can rapidly prototype and extend any vision‑based ROS 2 application on edge devices.


---

## Extensions: Enabling YOLOv3

Download YOLOv3 models into `src/stream2ros/models`:

```bash
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

echo "Downloads complete. Rebuild the Docker image."
```

---

## Connectivity Test

```bash
nc -zv localhost 8765
```

---



---

## Developing & Debugging


**Outside container:**
In a new terminal execute the following commands:

```bash
# Enter container shell
docker ps
docker exec -it <container_id> bash

# Re-run with bash
docker run --rm -it --network host -e STREAM_URL=http://host.docker.internal:8080/source_0 -p 8765:8765 orin-container-yolo bash
```

**Inside container:**

```bash
# Source environments
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash

# Launch foxglove bridge & stream node
ros2 run rosbridge_server rosbridge_websocket --port 8765 &
ros2 run stream2ros stream_reader

# List topics
ros2 topic list
# /image_raw
# /annotated_image
# /parameter_events
# /rosout

# Check publish rate
ros2 topic hz /image_raw
# Expect ~15 Hz (or FPS value set by -e FPS)
```

---

## Troubleshooting

- **Project Structure**
- <img width="357" height="659" alt="Screenshot 2025-07-27 at 17 51 58" src="https://github.com/user-attachments/assets/963f23c7-f275-4ab7-b99f-1c151946e359" />
- **Empty frames (`shape=(0,0)`)**  
  - Grant Terminal camera permission on macOS: *System Settings → Privacy & Security → Camera*  
  - Verify `http://localhost:8080/source_0` shows video.

- **0 Hz publish rate**  
  - Add logging in `stream_reader.py` for `cap.get(cv2.CAP_PROP_FPS)` and errors.

- **“Package 'stream2ros' not found”**  
  - Ensure you rebuilt Docker after adding `src/stream2ros`  
  - Re-run `docker build ...` before `docker run`.

---

## Credits & References

This project builds on open-source tools and models from the community. We ethically acknowledge and credit the original creators:

- **ROS2 Humble**: Core robotics framework. Developed by Open Robotics. [Official Site](https://docs.ros.org/en/humble/) (Apache 2.0 License).
- **OpenCV**: Computer vision library used for frame capture and processing. Developed by Intel and contributors. [Official Site](https://opencv.org/) (Apache 2.0 License).
- **cv_bridge**: ROS2 bridge for OpenCV images. Part of ROS2 vision_opencv package. [Official Repo](https://github.com/ros-perception/vision_opencv) (BSD License).
- **foxglove_bridge**: WebSocket bridge for Foxglove Studio visualization. Developed by Foxglove. [Official Repo](https://github.com/foxglove/ros-foxglove-bridge) (Apache 2.0 License).
- **YOLOv3 & YOLOv3-Tiny Models**: Object detection models used for extensions. Developed by Joseph Redmon and the Darknet community. [Original Darknet Repo](https://github.com/pjreddie/darknet) (Public Domain/YOLO License). Weights and configs downloaded from [pjreddie.com](https://pjreddie.com/darknet/yolo/).
- **MJPEG Streamer**: Webcam streaming tool (alternative Python script provided). Original by jacksonliam. [Repo](https://github.com/jacksonliam/mjpg-streamer) (GPL-2.0 License).
- **Docker**: Containerization platform. Developed by Docker Inc. [Official Site](https://www.docker.com/) (Apache 2.0 License).

All external assets (e.g., YOLO models) are referenced with download scripts to respect original sources. If redistributing, comply with their licenses. This project is under MIT License—see [LICENSE](LICENSE) for details.

Ethical Note: YOLO models may have applications in sensitive areas (e.g., surveillance); use responsibly and consider biases in training data (COCO dataset).

## License & Contribution

- License: MIT – Permissive open-source license allowing free use, modification, and distribution for any purpose, with minimal restrictions.
- Contributions Welcome: Fork the repo, open issues for bugs/features, or submit PRs with clear descriptions and tests. This boilerplate is community-driven—help improve it for ROS2/AI enthusiasts!

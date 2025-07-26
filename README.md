# ROS2‑X Container Boilerplate

TLDR;
A minimal, state‑of‑the‑art Dockerized ROS 2 setup that:

- Pulls an MJPEG stream (e.g. from your Mac webcam via `mjpeg-streamer`)
- Publishes it to `/image_raw` as a `sensor_msgs/Image`
- Exposes a ROS 2 WebSocket on port 8765 for Foxglove Studio or other clients



Dockerized ROS2 Humble setup for real-time webcam/MJPEG streaming to ROS2 topics, with seamless integration for visualization in Foxglove Studio. This pure-Python node uses OpenCV and cv_bridge to publish raw images on /image_raw, supporting easy extensions for AI/ML (e.g., edge detection, object detection with YOLO/Torch) and processing pipelines. Optimized for arm64 platforms like Mac M2 or NVIDIA Jetson (automotive/edge AI use cases), it includes foxglove_bridge for WebSocket-based monitoring. Ideal for prototyping vision systems in robotics, autonomous vehicles, or automotive perception—fork and extend for your CV/ML workflows!

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

1. **macOS host** with webcam & [`mjpeg-streamer`](https://github.com/jacksonliam/mjpg-streamer) installed  
2. **Docker** (with host network support)  
3. **ROS 2 Humble** base image (pulled automatically)  

---

## Quickstart

### 1. Start your MJPEG stream on macOS

```bash
mjpeg-streamer   --source 0   --fps 15   --quality 80   --port 8080
```

Verify in browser:  
```
http://localhost:8080/source_0
```
As an alternative you can run mjpeg_streamer.py script provided. Make sure 


### 1.1 (Alternative) Run the provided Python streamer

Create & activate a virtual environment

```bash
python3 -m venv venv
source venv/bin/activate
```

Install dependencies

```bash
pip install -r requirements.txt
```

Run the streamer

```bash
python mjpeg_streamer.py
```

### 2. Build the Docker image

```bash
docker build -t orin-container .
```

### 3. Run the container

> **Note:** Ports are discarded in host mode; WebSocket will still listen on `8765`.
### The correct command to make sure is working in FoxGlove is the following:

```bash
docker run --rm -it \
  -e STREAM_URL=http://host.docker.internal:8080/source_0 \
  -p 8765:8765 \
  orin-container
```
To check the connection to the port exposed by the container:
open a new terminal and type:
 ```bash
nc -zv localhost 876
```

For debugging one can try also the commands below:

```bash
docker run --rm -it --network host -e STREAM_URL=http://host.docker.internal:8080/source_0 -p 8765:8765 orin-container bash
docker run --rm -it \
  --network host \
  -e STREAM_URL=http://host.docker.internal:8080/source_0 \
  -p 8765:8765 \
  orin-container
```




---

## Developing & Debugging

### Enter the container shell

```bash
docker ps
docker exec -it <container_id> bash
```

### Source your ROS 2 environments (if needed)

Terminal with root@docker-desktop:~/ros2_ws# should appear and type the commands below:

```bash
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
```

### Launch the bridge and stream node manually

```bash
ros2 run rosbridge_server rosbridge_websocket --port 8765 &
ros2 run stream2ros stream_reader
```

### Inspect topics

```bash
ros2 topic list
# /image_raw
# /parameter_events
# /rosout
```

### Check publish rate

```bash
ros2 topic hz /image_raw
# Expect ~15 Hz
```

---

## Troubleshooting

- **Frames empty / `shape=(0,0)`**  
  - Ensure macOS has granted **Camera** permission to Terminal (System Settings ▶ Privacy ▶ Camera).  
  - Verify the MJPEG stream in your browser (`http://localhost:8080/source_0`).  
- **Rate is 0 Hz**  
  - Add logging in `stream_reader.py`:  
    ```python
    fps = self.cap.get(cv2.CAP_PROP_FPS)
    self.get_logger().info(f"Stream FPS report: {fps}")
    ```
- **“Package ‘stream2ros’ not found”**  
  - Confirm you rebuilt after copying your `src/stream2ros` folder into the image.  
  - Re-run `docker build -t orin-container .` before `docker run`.

---

## License & Contribution

- **License:** Apache 2.0  
- Feel free to fork, submit issues or PRs—this is intended as a community boilerplate!

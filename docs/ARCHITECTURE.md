# System Architecture

```mermaid
graph TD
    subgraph "Host (Streamer Tools)"
        Webcam -->|H264 Encode| RTSP_Server[RTSP H264 Streamer<br>(FFmpeg/GStreamer/NVENC on Jetson)]
    end

    subgraph "Docker Container (ROS2 Pipeline)"
        Config[Params: CAMERA_TYPE=rtsp<br>STREAM_URL=rtsp://... <br>SENSOR_MAP=webcam:yolo;ir:custom] --> Launch[multi_launch.py<br>(Spawn Parallel Nodes)]

        Launch --> S1[Sensor Node 1<br>(webcam/RTSP)] --> Raw1[/webcam/raw]
        Launch --> S2[Sensor Node 2<br>(ir)] --> Raw2[/ir/raw]
        Launch --> Sn[Sensor n<br>(thermal/radar stub)] --> Rawn[/.../raw]

        Raw1 --> A1[YOLO Predictor<br>(TensorRT on Jetson)] --> Ann1[/webcam/annotated_yolo]
        Raw2 --> A2[Custom Predictor] --> Ann2[/ir/annotated_custom]
        Rawn --> An[... ML] --> Annn[/.../annotated_...]
    end

    subgraph "Visualization"
        Ann1 & Ann2 & Annn --> V[Foxglove Studio<br>(ws://localhost:8765)]
    end
```
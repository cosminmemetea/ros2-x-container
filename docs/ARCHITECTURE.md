# System Architecture

```mermaid
graph TD
    subgraph Host_(Streamer_Tools)
        Webcam -->|H264 Encode| RTSP_Server["RTSP H264 Streamer\n(FFmpeg/GStreamer/NVENC on Jetson)"]
    end

    subgraph Docker_Container_(ROS2_Pipeline)
        Config["Params: CAMERA_TYPE=rtsp\nSTREAM_URL=rtsp://...\nSENSOR_MAP=webcam:yolo;ir:custom"] --> Launch["multi_launch.py\n(Spawn Parallel Nodes)"]

        Launch --> S1["Sensor Node 1\n(webcam/RTSP)"] --> Raw1[/webcam/raw/]
        Launch --> S2["Sensor Node 2\n(ir)"] --> Raw2[/ir/raw/]
        Launch --> Sn["Sensor n\n(thermal/radar stub)"] --> Rawn[/.../raw/]

        Raw1 --> A1["YOLO Predictor\n(TensorRT on Jetson)"] --> Ann1[/webcam/annotated_yolo/]
        Raw2 --> A2["Custom Predictor"] --> Ann2[/ir/annotated_custom/]
        Rawn --> An["... ML"] --> Annn[/.../annotated_.../]
    end

    subgraph Visualization
        Ann1 & Ann2 & Annn --> V["Foxglove Studio\n(ws://localhost:8765)"]
    end
```

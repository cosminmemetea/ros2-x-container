

┌──────────────┐          ┌───────────────┐          ┌─────────────────┐
│ Sensor Node  │──image→──│ ML Processor  │──annot→──│ Qt Visualizer   │
│ (/camera_*)  │          │ (/detection)  │          │ (C++, QML, ROS) │
└─────┬────────┘          └─────┬─────────┘          └─────────────────┘
      │                         │
      │                         └──ML Model Service (/inference) ← serves different models
      │
      │
┌─────▼──────┐
│ Sync Node  │ ← combine multi-stream / time-align
└────────────┘


docker build -t orin-container .
docker run --rm -it \
  --network host \
  -e STREAM_URL=http://host.docker.internal:8080/source_0 \
  -p 8765:8765 \
  orin-container
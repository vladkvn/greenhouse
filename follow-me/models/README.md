# models/

The TensorRT engine `yolo11n.engine` is **not** checked in — it is a build artifact
specific to the Jetson's GPU + TensorRT/JetPack version and is not portable.

Build it on the Jetson (one-time, ~minutes):

```bash
cd ~/greenhouse && source .venv/bin/activate
python scripts/export_yolo_tensorrt.py      # downloads yolo11n.pt -> writes models/yolo11n.engine
```

If the engine is missing, `follow_me.detector` falls back to `yolo11n.pt` (still GPU, slower).

"""Export a YOLO model to a TensorRT engine for GPU inference on the Jetson.

Run this ON THE JETSON -- the resulting .engine is tied to that exact GPU and the
installed TensorRT version, so it is not portable.

    python scripts/export_yolo_tensorrt.py            # yolo11n.pt -> models/yolo11n.engine
    python scripts/export_yolo_tensorrt.py --weights yolov8n.pt --imgsz 640
"""

from __future__ import annotations

import argparse
import os
import shutil

from ultralytics import YOLO


def main() -> None:
    p = argparse.ArgumentParser(description="Export YOLO -> TensorRT engine (FP16)")
    p.add_argument("--weights", default="yolo11n.pt", help="Source .pt weights")
    p.add_argument("--imgsz", type=int, default=640)
    p.add_argument("--device", default=0, help="CUDA device index")
    p.add_argument("--half", action="store_true", default=True, help="FP16 (default on)")
    p.add_argument("--outdir", default="models")
    args = p.parse_args()

    os.makedirs(args.outdir, exist_ok=True)
    model = YOLO(args.weights)
    print(f"Exporting {args.weights} -> TensorRT (FP16, imgsz={args.imgsz}) ...")
    engine_path = model.export(
        format="engine", half=args.half, imgsz=args.imgsz, device=args.device
    )

    dest = os.path.join(args.outdir, os.path.basename(engine_path))
    if os.path.abspath(engine_path) != os.path.abspath(dest):
        shutil.move(engine_path, dest)
    print(f"Done: {dest}")


if __name__ == "__main__":
    main()

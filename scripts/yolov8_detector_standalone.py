#!/usr/bin/env python3
"""Standalone YOLOv8 object detector — no ROS2 required.

Runs on a webcam (or video file) and displays detected objects with
bounding boxes, class labels, and confidence scores.

Usage:
    python3 yolov8_detector_standalone.py
    python3 yolov8_detector_standalone.py --source 1
    python3 yolov8_detector_standalone.py --source video.mp4
    python3 yolov8_detector_standalone.py --model yolov8n.pt --classes person cup
    python3 yolov8_detector_standalone.py --model yolov8n-oiv7.pt --classes Bottle Hammer
    python3 yolov8_detector_standalone.py --conf 0.25 --all-classes

Requirements:
    pip install ultralytics opencv-python numpy
    # The model file will be auto-downloaded by ultralytics on first run,
    # or provide a local .pt path with --model.

Notes on models:
    yolov8n.pt       — COCO 80 classes (use class names like 'bottle', 'person')
    yolov8n-oiv7.pt  — Open Images v7 (use 'Bottle', 'Hammer' etc. — capitalised)
"""

import argparse
import sys

import cv2
import numpy as np

try:
    from ultralytics import YOLO
except ImportError:
    print('[ERROR] ultralytics not installed. Run: pip install ultralytics',
          file=sys.stderr)
    sys.exit(1)


# ── colour palette (BGR) ──────────────────────────────────────────────────────
_CLASS_COLORS = {
    'bottle':  (0, 200, 255),
    'Bottle':  (0, 200, 255),
    'person':  (255, 100,  50),
    'Person':  (255, 100,  50),
    'cup':     (50,  200, 50),
    'Hammer':  (80,   80, 255),
}
_DEFAULT_COLOR = (0, 255, 0)


def draw_detections(image, results, target_classes, model_names, conf_thresh):
    overlay = image.copy()

    boxes_to_draw = []
    for box in results.boxes:
        cls_id   = int(box.cls[0])
        cls_name = model_names[cls_id]
        conf     = float(box.conf[0])

        if target_classes and cls_name not in target_classes:
            continue
        if conf < conf_thresh:
            continue

        x1, y1, x2, y2 = map(int, box.xyxy[0])
        color = _CLASS_COLORS.get(cls_name, _DEFAULT_COLOR)
        cv2.rectangle(overlay, (x1, y1), (x2, y2), color, cv2.FILLED)
        boxes_to_draw.append((cls_name, conf, x1, y1, x2, y2, color))

    cv2.addWeighted(overlay, 0.25, image, 0.75, 0, image)

    for cls_name, conf, x1, y1, x2, y2, color in boxes_to_draw:
        cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)

        label = f'{cls_name}  {conf:.0%}'
        font, fs, th = cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2
        (tw, txt_h), bl = cv2.getTextSize(label, font, fs, th)
        pad = 3
        ly1 = y1 - txt_h - pad * 2
        ly2 = y1
        if ly1 < 0:
            ly1, ly2 = y2, y2 + txt_h + pad * 2
        cv2.rectangle(image, (x1, ly1), (x1 + tw + pad * 2, ly2), color, cv2.FILLED)
        cv2.putText(image, label, (x1 + pad, ly2 - pad),
                    font, fs, (255, 255, 255), th, cv2.LINE_AA)

    return image, len(boxes_to_draw)


def run(args):
    # ── Open capture source ───────────────────────────────────────────────────
    src = args.source
    try:
        src = int(src)
    except ValueError:
        pass
    cap = cv2.VideoCapture(src)
    if not cap.isOpened():
        print(f'[ERROR] Cannot open source: {args.source}', file=sys.stderr)
        sys.exit(1)

    # ── Load model ────────────────────────────────────────────────────────────
    print(f'[YOLO standalone] Loading model: {args.model}')
    model = YOLO(args.model, task='detect')
    model_names = model.names

    target_classes = set(args.classes) if args.classes else set()

    # Warn about missing class names
    all_names = set(model_names.values())
    for cls in target_classes:
        if cls not in all_names:
            sample = sorted(all_names)[:15]
            print(f'[WARN] Class "{cls}" not in model. Sample names: {sample}')

    conf_thresh = args.conf
    use_all = args.all_classes or not target_classes

    print(f'  Model      : {args.model}')
    print(f'  Confidence : {conf_thresh:.0%}')
    if use_all:
        print('  Classes    : ALL')
    else:
        print(f'  Classes    : {sorted(target_classes)}')
    print('[YOLO standalone] Press Q to quit.')

    while True:
        ok, frame = cap.read()
        if not ok:
            break

        results = model(frame, conf=conf_thresh, verbose=False, device='cpu')[0]

        effective_targets = None if use_all else target_classes
        annotated, n_det = draw_detections(
            frame, results, effective_targets, model_names, conf_thresh)

        status = f'Detections: {n_det}  |  conf>={conf_thresh:.0%}  |  Q=quit'
        cv2.putText(annotated, status, (8, annotated.shape[0] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

        cv2.imshow('YOLOv8 Detector (standalone)', annotated)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


def main():
    p = argparse.ArgumentParser(description='Standalone YOLOv8 detector')
    p.add_argument('--source', default='0',
                   help='Camera index (0,1,…) or video file path (default: 0)')
    p.add_argument('--model', default='yolov8n.pt',
                   help='Path to .pt model or model name (default: yolov8n.pt)')
    p.add_argument('--classes', nargs='+', default=['bottle'],
                   help='Target class names to display (default: bottle). '
                        'Use COCO names (lowercase) for yolov8n.pt, '
                        'OIV7 names (capitalised) for yolov8n-oiv7.pt')
    p.add_argument('--conf', type=float, default=0.25,
                   help='Confidence threshold (default: 0.25)')
    p.add_argument('--all-classes', action='store_true',
                   help='Show ALL detected classes, ignore --classes filter')
    run(p.parse_args())


if __name__ == '__main__':
    main()
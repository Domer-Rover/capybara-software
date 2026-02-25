#!/usr/bin/env python3
"""Export a YOLOv8 model to ONNX format for use with the ZED SDK.

The ZED SDK's CUSTOM_YOLOLIKE_BOX_OBJECTS detection mode requires an ONNX file
exported with opset=12 and simplify=True so it can be optimised by TensorRT.

Output:  /home/jetsonson/capybara-software/models/yolov8n.onnx
Config:  src/capybara_bringup/config/capybara_custom_od.yaml → custom_onnx_file

Usage:
    python3 yolov8_export_onnx.py                      # default: yolov8n.pt, 640
    python3 yolov8_export_onnx.py --model yolov8s.pt
    python3 yolov8_export_onnx.py --model yolov8n-oiv7.pt   # Open Images v7 (has Hammer)
    python3 yolov8_export_onnx.py --imgsz 416

Notes:
    - For water-bottle detection use any standard yolov8*.pt (COCO class 39 = bottle).
    - For hammer detection the COCO models do NOT include a hammer class.
      Use yolov8n-oiv7.pt (Open Images v7) which does include 'Hammer', then update
      capybara_custom_od.yaml with the correct model_class_id and custom_class_count.
"""

import argparse
import os
import shutil
import sys

OUTPUT_DIR = '/home/jetsonson/capybara-software/models'


def parse_args():
    p = argparse.ArgumentParser(description='Export YOLOv8 → ONNX for ZED SDK')
    p.add_argument('--model', default='yolov8n.pt',
                   help='Model name or path (default: yolov8n.pt)')
    p.add_argument('--imgsz', type=int, default=640,
                   help='Input image size (default: 640). Update custom_onnx_input_size '
                        'in capybara_custom_od.yaml if you change this.')
    return p.parse_args()


def main():
    try:
        from ultralytics import YOLO
    except ImportError:
        sys.exit('[yolov8_export_onnx] ultralytics not installed. Run: pip install ultralytics')

    args = parse_args()
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    print(f'Loading model: {args.model}')
    model = YOLO(args.model)

    print(f'Exporting to ONNX (imgsz={args.imgsz}, opset=12, simplify=True) …')
    export_path = model.export(
        format='onnx',
        imgsz=args.imgsz,
        opset=12,
        simplify=True,
        dynamic=False,
    )

    # ultralytics saves next to the weights file; move it to our models dir
    stem = os.path.splitext(os.path.basename(args.model))[0]
    dest = os.path.join(OUTPUT_DIR, f'{stem}.onnx')
    if os.path.abspath(export_path) != os.path.abspath(dest):
        shutil.move(export_path, dest)

    print(f'\nONNX model written to: {dest}')
    print(f'Input tensor: 1 x 3 x {args.imgsz} x {args.imgsz}')
    print()
    print('Next steps:')
    print(f'  1. Verify capybara_custom_od.yaml → custom_onnx_file: {dest!r}')
    print(f'  2. Verify capybara_custom_od.yaml → custom_onnx_input_size: {args.imgsz}')
    if 'oiv7' in args.model:
        print()
        print('  Open Images v7 model detected.')
        print('  To find the Hammer class ID, run:')
        print('    python3 -c "from ultralytics import YOLO; m=YOLO(\'yolov8n-oiv7.pt\'); '
              'print({v:k for k,v in m.names.items()})[\'Hammer\']"')
        print('  Then update class_NNN in capybara_custom_od.yaml with that ID.')
    else:
        # Print COCO bottle class ID as a reminder
        names = model.names
        bottle_id = next((k for k, v in names.items() if v == 'bottle'), None)
        if bottle_id is not None:
            print(f'  COCO "bottle" class ID: {bottle_id}  '
                  f'(already configured as class_039 in capybara_custom_od.yaml)')


if __name__ == '__main__':
    main()
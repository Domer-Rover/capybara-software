#!/usr/bin/env python3
"""Standalone ArUco marker detector — no ROS2 required.

Runs on a webcam (or video file) and displays detected markers with
optional 6-DoF pose estimation using estimated or provided camera intrinsics.

Usage:
    python3 aruco_detector_standalone.py
    python3 aruco_detector_standalone.py --source 1          # second webcam
    python3 aruco_detector_standalone.py --source video.mp4  # video file
    python3 aruco_detector_standalone.py --marker-size 0.10  # 10 cm markers
    python3 aruco_detector_standalone.py --no-pose           # skip pose est.

Requirements:
    pip install opencv-contrib-python numpy
"""

import argparse
import math
import sys

import cv2
import numpy as np


# ── ArUco dictionary (must match the printed markers) ─────────────────────────
DICT_ID = cv2.aruco.DICT_4X4_250

# ── Visual style ──────────────────────────────────────────────────────────────
LIGHT_BLUE_BGR  = (230, 216, 173)
BORDER_COLOR    = (255, 200, 100)
TEXT_COLOR      = (255, 255, 255)
TEXT_BG_COLOR   = (180, 130,  60)
OVERLAY_ALPHA   = 0.35
AXIS_LENGTH     = 0.05   # metres – 3-D axis length drawn on marker


def estimate_camera_matrix(frame_w: int, frame_h: int) -> tuple:
    """Estimate a reasonable pinhole camera matrix from frame dimensions.

    Assumes a horizontal FOV of ~70 degrees (typical for laptop webcams).
    Good enough for rough pose visualisation; replace with real calibration
    if you need accurate pose numbers.
    """
    focal_px = frame_w / (2 * math.tan(math.radians(70 / 2)))
    cx, cy = frame_w / 2.0, frame_h / 2.0
    K = np.array([
        [focal_px, 0,        cx],
        [0,        focal_px, cy],
        [0,        0,         1],
    ], dtype=np.float64)
    dist = np.zeros((5, 1), dtype=np.float64)
    return K, dist


def rotation_matrix_to_quaternion(R):
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return (x, y, z, w)


def draw_overlays(image, corners, ids):
    overlay = image.copy()
    for i in range(len(ids)):
        pts = corners[i][0].astype(np.int32)
        cv2.fillPoly(overlay, [pts], LIGHT_BLUE_BGR)
        cv2.polylines(image, [pts], isClosed=True, color=BORDER_COLOR, thickness=2)
    cv2.addWeighted(overlay, OVERLAY_ALPHA, image, 1 - OVERLAY_ALPHA, 0, image)

    for i, marker_id in enumerate(ids.flatten()):
        pts = corners[i][0].astype(np.int32)
        label = f'ID:{marker_id} [4X4]'
        font, fs, th = cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2
        (tw, txt_h), bl = cv2.getTextSize(label, font, fs, th)
        tx, ty = int(pts[0][0]), int(pts[0][1])
        pad = 3
        cv2.rectangle(image,
                      (tx, ty - txt_h - pad * 2),
                      (tx + tw + pad * 2, ty + bl),
                      TEXT_BG_COLOR, cv2.FILLED)
        cv2.putText(image, label, (tx + pad, ty - pad),
                    font, fs, TEXT_COLOR, th, cv2.LINE_AA)


def draw_pose_info(image, marker_id, tvec, rvec, K, dist):
    dist_m = math.sqrt(tvec[0] ** 2 + tvec[1] ** 2 + tvec[2] ** 2)
    info = (f'ID:{marker_id}  dist={dist_m:.2f}m  '
            f'x={tvec[0]:.2f} y={tvec[1]:.2f} z={tvec[2]:.2f}')
    y_pos = 30 + marker_id * 22
    cv2.putText(image, info, (10, y_pos),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 200), 1, cv2.LINE_AA)
    cv2.drawFrameAxes(image, K, dist, rvec, tvec, AXIS_LENGTH)


def run(args):
    # ── Open capture source ───────────────────────────────────────────────────
    src = args.source
    try:
        src = int(src)
    except ValueError:
        pass  # it's a file path
    cap = cv2.VideoCapture(src)
    if not cap.isOpened():
        print(f'[ERROR] Cannot open source: {args.source}', file=sys.stderr)
        sys.exit(1)

    # ── ArUco setup ───────────────────────────────────────────────────────────
    aruco_dict   = cv2.aruco.getPredefinedDictionary(DICT_ID)
    params       = cv2.aruco.DetectorParameters()

    # Slightly more lenient detection parameters to help with challenging conditions
    params.adaptiveThreshWinSizeMin  = 3
    params.adaptiveThreshWinSizeMax  = 53
    params.adaptiveThreshWinSizeStep = 4
    params.minMarkerPerimeterRate    = 0.02   # allow smaller markers
    params.maxMarkerPerimeterRate    = 4.0
    params.polygonalApproxAccuracyRate = 0.04
    params.minCornerDistanceRate     = 0.05
    params.perspectiveRemoveIgnoredMarginPerCell = 0.13

    detector = cv2.aruco.ArucoDetector(aruco_dict, params)

    marker_size = args.marker_size
    K = dist = None   # populated on first frame

    print('[ArUco standalone] Press Q to quit.')
    print(f'  Dictionary : DICT_4X4_250')
    print(f'  Marker size: {marker_size * 100:.1f} cm')
    print(f'  Pose est.  : {"disabled" if args.no_pose else "enabled (estimated intrinsics)"}')

    obj_points = np.array([
        [-marker_size / 2,  marker_size / 2, 0],
        [ marker_size / 2,  marker_size / 2, 0],
        [ marker_size / 2, -marker_size / 2, 0],
        [-marker_size / 2, -marker_size / 2, 0],
    ], dtype=np.float32)

    while True:
        ok, frame = cap.read()
        if not ok:
            break

        h, w = frame.shape[:2]
        if K is None:
            K, dist = estimate_camera_matrix(w, h)
            print(f'  Frame size : {w}x{h}  — estimated focal length {K[0,0]:.1f} px')

        corners, ids, _ = detector.detectMarkers(frame)

        if ids is not None:
            draw_overlays(frame, corners, ids)

            if not args.no_pose:
                for i, marker_id in enumerate(ids.flatten()):
                    ok_pnp, rvec, tvec = cv2.solvePnP(
                        obj_points, corners[i][0], K, dist)
                    if ok_pnp:
                        draw_pose_info(frame, int(marker_id), tvec.flatten(),
                                       rvec, K, dist)
        else:
            cv2.putText(frame, 'No markers detected', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        cv2.imshow('ArUco Detector (standalone)', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


def main():
    p = argparse.ArgumentParser(description='Standalone ArUco detector')
    p.add_argument('--source', default='0',
                   help='Camera index (0,1,…) or video file path (default: 0)')
    p.add_argument('--marker-size', type=float, default=0.15,
                   help='Physical marker side length in metres (default: 0.15)')
    p.add_argument('--no-pose', action='store_true',
                   help='Skip pose estimation (faster, no calibration needed)')
    run(p.parse_args())


if __name__ == '__main__':
    main()

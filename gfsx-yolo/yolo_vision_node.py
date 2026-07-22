#!/usr/bin/env python3
"""Low-latency YOLO sender for the GFS-X Unity controller.

The node reads the robot MJPEG stream, detects the largest ball of the
configured class and sends protocol-v1 UDP packets understood by
Assets/Scripts/RealYoloCamera.cs.
"""

from __future__ import annotations

import argparse
import json
import socket
import sys
import threading
import time
from pathlib import Path
from typing import Any, Optional

import cv2
from ultralytics import YOLO


ROOT = Path(__file__).resolve().parent


class LatestFrameCapture:
    """Drain the video stream continuously and expose each new frame once."""

    def __init__(self, stream_url: str, reconnect_seconds: float) -> None:
        self._stream_url = stream_url
        self._reconnect_seconds = max(0.1, reconnect_seconds)
        self._frame: Optional[Any] = None
        self._frame_number = 0
        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._capture: Optional[cv2.VideoCapture] = None

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(
            target=self._run,
            daemon=True,
            name="GFSX camera capture",
        )
        self._thread.start()

    def latest(self, previous_number: int) -> tuple[Optional[Any], int]:
        with self._lock:
            if self._frame is None or self._frame_number == previous_number:
                return None, previous_number
            return self._frame.copy(), self._frame_number

    def stop(self) -> None:
        self._running = False
        capture = self._capture
        if capture is not None:
            capture.release()
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def _open(self) -> Optional[cv2.VideoCapture]:
        capture = cv2.VideoCapture(self._stream_url)
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not capture.isOpened():
            capture.release()
            return None
        return capture

    def _run(self) -> None:
        while self._running:
            if self._capture is None or not self._capture.isOpened():
                self._capture = self._open()
                if self._capture is None:
                    time.sleep(self._reconnect_seconds)
                    continue

            ok, frame = self._capture.read()
            if not ok or frame is None:
                self._capture.release()
                self._capture = None
                time.sleep(self._reconnect_seconds)
                continue

            with self._lock:
                self._frame = frame
                self._frame_number += 1

        if self._capture is not None:
            self._capture.release()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="GFS-X YOLO to Unity UDP bridge")
    parser.add_argument(
        "--stream-url",
        required=True,
        help="MJPEG URL, for example http://192.168.2.154:8080/?action=stream",
    )
    parser.add_argument(
        "--model",
        required=True,
        help="Path to a YOLO .pt/.onnx model or OpenVINO model directory",
    )
    parser.add_argument("--udp-ip", default="127.0.0.1")
    parser.add_argument("--udp-port", type=int, default=5005)
    parser.add_argument("--confidence", type=float, default=0.20)
    parser.add_argument("--ball-class", type=int, default=0)
    parser.add_argument("--image-size", type=int, default=416)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--far-bbox-height-ratio", type=float, default=0.04)
    parser.add_argument("--near-bbox-height-ratio", type=float, default=0.55)
    parser.add_argument(
        "--visibility-hold-seconds",
        type=float,
        default=0.25,
        help="Keep the last valid ball measurement through a brief detector miss",
    )
    parser.add_argument("--reconnect-seconds", type=float, default=1.0)
    parser.add_argument("--no-preview", action="store_true")
    return parser.parse_args()


def resolve_model_path(value: str) -> Path:
    path = Path(value).expanduser()
    return path if path.is_absolute() else ROOT / path


def normalized_distance(height_ratio: float, far_ratio: float, near_ratio: float) -> float:
    """Convert box height to the PPO contract: 0 is near and 1 is far."""

    span = near_ratio - far_ratio
    if span <= 1e-6:
        raise ValueError("--near-bbox-height-ratio must exceed --far-bbox-height-ratio")
    return max(0.0, min(1.0, (near_ratio - height_ratio) / span))


def select_largest_ball(
    result: Any,
    frame_width: int,
    frame_height: int,
    ball_class: int,
    far_ratio: float,
    near_ratio: float,
) -> tuple[Optional[dict[str, float]], Optional[tuple[float, float, float, float]]]:
    best_detection = None
    best_box = None
    best_area = -1.0
    if result.boxes is None:
        return None, None

    for box in result.boxes:
        if int(box.cls[0].item()) != ball_class:
            continue

        x1, y1, x2, y2 = (float(value) for value in box.xyxy[0].cpu().tolist())
        box_width = max(0.0, x2 - x1)
        box_height = max(0.0, y2 - y1)
        area = box_width * box_height
        if area <= best_area:
            continue

        centre_x = (x1 + x2) * 0.5
        height_ratio = box_height / max(1.0, float(frame_height))
        best_area = area
        best_box = (x1, y1, x2, y2)
        best_detection = {
            "angle": max(
                -1.0,
                min(1.0, (centre_x - frame_width * 0.5) / max(1.0, frame_width * 0.5)),
            ),
            "distance": normalized_distance(height_ratio, far_ratio, near_ratio),
            "confidence": max(0.0, min(1.0, float(box.conf[0].item()))),
            "bboxWidth": box_width,
            "bboxHeight": box_height,
            "bboxHeightRatio": height_ratio,
        }

    return best_detection, best_box


def make_packet(
    sequence: int,
    detection: Optional[dict[str, float]],
    inference_ms: float,
) -> dict[str, Any]:
    packet: dict[str, Any] = {
        "protocol": 1,
        "sequence": sequence,
        "sees": 0.0,
        "angle": 0.0,
        "distance": 1.0,
        "confidence": 0.0,
        "bboxWidth": 0.0,
        "bboxHeight": 0.0,
        "bboxHeightRatio": 0.0,
        "inferenceMs": round(max(0.0, inference_ms), 3),
    }
    if detection is not None:
        packet.update(detection)
        packet["sees"] = 1.0
    return packet


def send_packet(
    udp: socket.socket,
    target: tuple[str, int],
    packet: dict[str, Any],
) -> None:
    udp.sendto(json.dumps(packet, separators=(",", ":")).encode("utf-8"), target)


def validate_args(args: argparse.Namespace, model_path: Path) -> None:
    if not model_path.exists():
        raise FileNotFoundError(f"YOLO model not found: {model_path}")
    if not 1 <= args.udp_port <= 65535:
        raise ValueError("--udp-port must be between 1 and 65535")
    if not 0.0 <= args.confidence <= 1.0:
        raise ValueError("--confidence must be between 0 and 1")
    if args.image_size <= 0:
        raise ValueError("--image-size must be positive")
    normalized_distance(
        args.far_bbox_height_ratio,
        args.far_bbox_height_ratio,
        args.near_bbox_height_ratio,
    )


def main() -> int:
    args = parse_args()
    model_path = resolve_model_path(args.model)
    validate_args(args, model_path)

    print(f"Loading model: {model_path}")
    model = YOLO(str(model_path), task="detect")
    print(f"Model classes: {model.names}")
    print(f"Camera source: {args.stream_url}")
    print(f"Unity UDP target: {args.udp_ip}:{args.udp_port}")
    print("Distance contract: 0=near, 1=far. Press Q, Esc or Ctrl+C to stop.")

    capture = LatestFrameCapture(args.stream_url, args.reconnect_seconds)
    capture.start()
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = (args.udp_ip, args.udp_port)

    last_frame_number = -1
    last_detection: Optional[dict[str, float]] = None
    last_detection_time = float("-inf")
    last_heartbeat_time = float("-inf")
    last_status_time = time.perf_counter()
    sequence = 0
    processed = 0
    started = time.perf_counter()

    try:
        while True:
            frame, frame_number = capture.latest(last_frame_number)
            now = time.perf_counter()
            hold_seconds = max(0.0, args.visibility_hold_seconds)

            if frame is None:
                if now - last_heartbeat_time >= 0.1:
                    held = last_detection if now - last_detection_time <= hold_seconds else None
                    send_packet(udp, target, make_packet(sequence, held, 0.0))
                    sequence += 1
                    last_heartbeat_time = now
                time.sleep(0.005)
                continue

            last_frame_number = frame_number
            height, width = frame.shape[:2]
            inference_start = time.perf_counter()
            result = model.track(
                source=frame,
                persist=True,
                classes=[args.ball_class],
                conf=args.confidence,
                imgsz=args.image_size,
                device=args.device,
                verbose=False,
            )[0]
            inference_ms = (time.perf_counter() - inference_start) * 1000.0
            detection, draw_box = select_largest_ball(
                result,
                width,
                height,
                args.ball_class,
                args.far_bbox_height_ratio,
                args.near_bbox_height_ratio,
            )
            now = time.perf_counter()

            if detection is not None:
                last_detection = detection.copy()
                last_detection_time = now
                active_detection = detection
            elif now - last_detection_time <= hold_seconds:
                active_detection = last_detection
            else:
                active_detection = None

            packet = make_packet(sequence, active_detection, inference_ms)
            send_packet(udp, target, packet)
            sequence += 1
            processed += 1
            last_heartbeat_time = now

            if not args.no_preview:
                if draw_box is not None:
                    x1, y1, x2, y2 = (int(value) for value in draw_box)
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (40, 220, 40), 2)
                status = (
                    f"ball={int(packet['sees'])} conf={packet['confidence']:.2f} "
                    f"angle={packet['angle']:.2f} dist={packet['distance']:.2f} "
                    f"inference={inference_ms:.1f}ms"
                )
                cv2.putText(
                    frame,
                    status,
                    (10, 24),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )
                cv2.imshow("GFS-X P7 YOLO", frame)
                if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                    break

            if now - last_status_time >= 2.0:
                elapsed = max(1e-6, now - started)
                print(
                    f"fps={processed / elapsed:.1f} inference={inference_ms:.1f}ms "
                    f"ball={int(packet['sees'])} conf={packet['confidence']:.2f} "
                    f"angle={packet['angle']:.2f} distance={packet['distance']:.2f}"
                )
                last_status_time = now
    except KeyboardInterrupt:
        pass
    finally:
        capture.stop()
        udp.close()
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        raise SystemExit(1) from error

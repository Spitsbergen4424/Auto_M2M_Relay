#!/usr/bin/env python3
"""Low-latency YOLO vision sender for the GFS-X Unity controller."""

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
import numpy as np
import yaml
from ultralytics import YOLO
from urllib.request import Request, urlopen


ROOT = Path(__file__).resolve().parent


def load_config(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    if not isinstance(data, dict):
        raise ValueError("The configuration root must be a YAML mapping")
    return data


def resolve_path(value: str) -> str:
    path = Path(value)
    return str(path if path.is_absolute() else ROOT / path)


def parse_source(value: Any) -> Any:
    if isinstance(value, int):
        return value
    text = str(value).strip()
    return int(text) if text.isdigit() else text


class LatestFrameCapture:
    """Continuously drains OpenCV and retains only the newest frame."""

    def __init__(self, source: Any, reconnect_seconds: float = 1.0) -> None:
        self.source = source
        self.reconnect_seconds = max(0.1, reconnect_seconds)
        self._lock = threading.Lock()
        self._frame = None
        self._frame_number = 0
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._capture: Optional[cv2.VideoCapture] = None

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True, name="GFSX camera capture")
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        capture = self._capture
        if capture is not None:
            capture.release()
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def latest(self, previous_number: int) -> tuple[Optional[Any], int]:
        with self._lock:
            if self._frame is None or self._frame_number == previous_number:
                return None, previous_number
            return self._frame.copy(), self._frame_number

    def _open(self) -> Optional[cv2.VideoCapture]:
        capture = cv2.VideoCapture(self.source)
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not capture.isOpened():
            capture.release()
            return None
        return capture

    def _loop(self) -> None:
        while self._running:
            if self._capture is None or not self._capture.isOpened():
                self._capture = self._open()
                if self._capture is None:
                    time.sleep(self.reconnect_seconds)
                    continue

            ok, frame = self._capture.read()
            if not ok or frame is None:
                self._capture.release()
                self._capture = None
                time.sleep(self.reconnect_seconds)
                continue

            with self._lock:
                self._frame = frame
                self._frame_number += 1


class MjpegFrameCapture:
    """Read a MJPEG HTTP stream directly and keep only the newest decoded frame."""

    _start_marker = b"\xff\xd8"
    _end_marker = b"\xff\xd9"
    _max_buffer_bytes = 48 * 1024 * 1024
    _trim_buffer_bytes = 1024 * 1024

    def __init__(self, source: str, reconnect_seconds: float = 1.0) -> None:
        self.source = source
        self.reconnect_seconds = max(0.1, reconnect_seconds)
        self._lock = threading.Lock()
        self._frame = None
        self._frame_number = 0
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._stream = None

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True, name="GFSX MJPEG capture")
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        stream = self._stream
        self._stream = None
        if stream is not None:
            try:
                stream.close()
            except Exception:
                pass
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def latest(self, previous_number: int) -> tuple[Optional[Any], int]:
        with self._lock:
            if self._frame is None or self._frame_number == previous_number:
                return None, previous_number
            return self._frame.copy(), self._frame_number

    def _loop(self) -> None:
        while self._running:
            try:
                self._read_stream()
            except Exception as error:
                self._close_stream()
                if self._running:
                    print(f"MJPEG reconnecting in {self.reconnect_seconds:.1f}s: {error}")
                    time.sleep(self.reconnect_seconds)

    def _read_stream(self) -> None:
        request = Request(self.source, headers={"User-Agent": "Mozilla/5.0"})
        response = urlopen(request, timeout=10)
        self._stream = response
        print(f"MJPEG connected: {self.source}")

        buffer = bytearray()
        while self._running:
            chunk = response.read(4096)
            if not chunk:
                raise ConnectionError("MJPEG stream ended")
            buffer.extend(chunk)
            if len(buffer) > self._max_buffer_bytes:
                buffer = buffer[-self._trim_buffer_bytes :]

            while True:
                start = buffer.find(self._start_marker)
                if start < 0:
                    if len(buffer) > 1:
                        del buffer[:-1]
                    break

                if start > 0:
                    del buffer[:start]

                end = buffer.find(self._end_marker, 2)
                if end < 0:
                    break

                jpeg = bytes(buffer[: end + 2])
                del buffer[: end + 2]
                frame = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
                if frame is None:
                    continue

                with self._lock:
                    self._frame = frame
                    self._frame_number += 1
                print("MJPEG frame received")

    def _close_stream(self) -> None:
        stream = self._stream
        self._stream = None
        if stream is not None:
            try:
                stream.close()
            except Exception:
                pass


def normalized_distance(height_ratio: float, far_ratio: float, near_ratio: float) -> float:
    """Return PPO-compatible distance: 0 is near, 1 is far."""
    span = near_ratio - far_ratio
    if span <= 1e-6:
        raise ValueError("distance.near_bbox_height_ratio must exceed far_bbox_height_ratio")
    value = (near_ratio - height_ratio) / span
    return max(0.0, min(1.0, value))


def make_packet(sequence: int, detection: Optional[dict[str, float]], inference_ms: float) -> dict[str, Any]:
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
        "inferenceMs": round(inference_ms, 3),
    }
    if detection is not None:
        packet.update(detection)
        packet["sees"] = 1.0
    return packet


def select_ball(boxes: Any, frame_width: int, frame_height: int, target_class: int,
                far_ratio: float, near_ratio: float) -> Optional[dict[str, float]]:
    best = None
    best_area = -1.0
    if boxes is None:
        return None

    for box in boxes:
        class_id = int(box.cls[0].item())
        if class_id != target_class:
            continue
        x1, y1, x2, y2 = (float(value) for value in box.xyxy[0].tolist())
        width = max(0.0, x2 - x1)
        height = max(0.0, y2 - y1)
        area = width * height
        if area <= best_area:
            continue

        center_x = (x1 + x2) * 0.5
        angle = (center_x - frame_width * 0.5) / max(1.0, frame_width * 0.5)
        height_ratio = height / max(1.0, float(frame_height))
        best_area = area
        best = {
            "angle": max(-1.0, min(1.0, angle)),
            "distance": normalized_distance(height_ratio, far_ratio, near_ratio),
            "confidence": max(0.0, min(1.0, float(box.conf[0].item()))),
            "bboxWidth": width,
            "bboxHeight": height,
            "bboxHeightRatio": height_ratio,
            "_xyxy": (x1, y1, x2, y2),
        }
    return best


def validate_model(model: YOLO, target_class: int) -> None:
    names = model.names
    class_name = names.get(target_class) if isinstance(names, dict) else names[target_class]
    if class_name != "ball":
        raise RuntimeError(
            f"Class {target_class} is {class_name!r}, not 'ball'. Check vision.target_class_id."
        )
    print(f"Model classes: {names}")


def main() -> int:
    parser = argparse.ArgumentParser(description="GFS-X YOLO to Unity UDP bridge")
    parser.add_argument("--config", default=str(ROOT / "config.yaml"), help="Path to YAML configuration")
    arguments = parser.parse_args()

    config_path = Path(arguments.config).resolve()
    config = load_config(config_path)
    model_path = resolve_path(str(config["model"]["path"]))
    source = parse_source(config["camera"]["source"])
    udp_host = str(config["udp"].get("host", "127.0.0.1"))
    udp_port = int(config["udp"].get("port", 5005))
    confidence = float(config["vision"].get("confidence", 0.25))
    target_class = int(config["vision"].get("target_class_id", 0))
    image_size = int(config["vision"].get("image_size", 512))
    device = str(config["vision"].get("device", "cpu"))
    preview = bool(config["display"].get("preview", True))
    far_ratio = float(config["distance"].get("far_bbox_height_ratio", 0.04))
    near_ratio = float(config["distance"].get("near_bbox_height_ratio", 0.55))
    reconnect_seconds = float(config["camera"].get("reconnect_seconds", 1.0))

    if not Path(model_path).is_file():
        raise FileNotFoundError(f"YOLO model not found: {model_path}")

    print(f"Loading model: {model_path}")
    model = YOLO(model_path, task="detect")
    validate_model(model, target_class)
    print(f"Camera source: {source}")
    print(f"Unity UDP target: {udp_host}:{udp_port}")
    print("Distance contract: 0=near, 1=far. Press Q or Esc to stop.")

    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    if isinstance(source, str) and source.startswith(("http://", "https://")):
        capture: Any = MjpegFrameCapture(source, reconnect_seconds)
    else:
        capture = LatestFrameCapture(source, reconnect_seconds)
    capture.start()
    last_frame_number = -1
    last_status_time = time.perf_counter()
    last_heartbeat_time = 0.0
    sequence = 0
    processed = 0
    start_time = time.perf_counter()

    try:
        while True:
            frame, frame_number = capture.latest(last_frame_number)
            now = time.perf_counter()
            if frame is None:
                if now - last_heartbeat_time >= 0.1:
                    packet = make_packet(sequence, None, 0.0)
                    sender.sendto(json.dumps(packet, separators=(",", ":")).encode("utf-8"),
                                  (udp_host, udp_port))
                    sequence += 1
                    last_heartbeat_time = now
                time.sleep(0.005)
                continue

            last_frame_number = frame_number
            height, width = frame.shape[:2]
            inference_start = time.perf_counter()
            results = model.predict(
                source=frame,
                conf=confidence,
                classes=[target_class],
                imgsz=image_size,
                device=device,
                verbose=False,
            )
            inference_ms = (time.perf_counter() - inference_start) * 1000.0
            detection = select_ball(results[0].boxes, width, height, target_class, far_ratio, near_ratio)
            draw_box = detection.pop("_xyxy", None) if detection is not None else None
            packet = make_packet(sequence, detection, inference_ms)
            sender.sendto(json.dumps(packet, separators=(",", ":")).encode("utf-8"),
                          (udp_host, udp_port))
            sequence += 1
            processed += 1

            if preview:
                if draw_box is not None:
                    x1, y1, x2, y2 = (int(value) for value in draw_box)
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (40, 220, 40), 2)
                    label = (f"ball {packet['confidence']:.2f} "
                             f"angle={packet['angle']:.2f} dist={packet['distance']:.2f} "
                             f"h={packet['bboxHeightRatio']:.3f}")
                    cv2.putText(frame, label, (x1, max(25, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX,
                                0.55, (40, 220, 40), 2, cv2.LINE_AA)
                elapsed = max(1e-6, time.perf_counter() - start_time)
                fps = processed / elapsed
                cv2.putText(frame, f"FPS {fps:.1f} | inference {inference_ms:.1f} ms",
                            (12, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2,
                            cv2.LINE_AA)
                cv2.imshow("GFS-X YOLO", frame)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break

            if now - last_status_time >= 2.0:
                elapsed = max(1e-6, now - start_time)
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
        sender.close()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        raise

"""P7 YOLO node: Raspberry MJPEG stream -> UDP detections for Unity.

Run this on the Windows PC, not on Raspberry Pi.  It chooses the largest ball
box (class 0) and sends the newest result to Unity on UDP port 5005.
"""

import argparse
import json
import socket
import threading
import time

import cv2
from ultralytics import YOLO


class LatestFrameCapture:
    def __init__(self, stream_url):
        self._stream_url = stream_url
        self._frame = None
        self._lock = threading.Lock()
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def get(self):
        with self._lock:
            return None if self._frame is None else self._frame.copy()

    def stop(self):
        self._running = False
        self._thread.join(timeout=1.0)

    def _run(self):
        capture = cv2.VideoCapture(self._stream_url)
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        while self._running:
            ok, frame = capture.read()
            if ok:
                with self._lock:
                    self._frame = frame
            else:
                capture.release()
                time.sleep(0.5)
                capture = cv2.VideoCapture(self._stream_url)
                capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        capture.release()


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--stream-url", required=True,
                        help="Example: http://192.168.137.248:8080/")
    parser.add_argument("--model", required=True,
                        help="Path to best_detect.pt, best_int8.onnx, or an OpenVINO directory")
    parser.add_argument("--udp-ip", default="127.0.0.1")
    parser.add_argument("--udp-port", type=int, default=5005)
    parser.add_argument("--confidence", type=float, default=0.20)
    parser.add_argument("--ball-class", type=int, default=0)
    parser.add_argument("--visibility-hold-seconds", type=float, default=0.25,
                        help="Keep the last valid ball measurement through a brief YOLO miss")
    parser.add_argument("--no-preview", action="store_true")
    return parser.parse_args()


def select_largest_ball(result, ball_class):
    best = None
    best_area = -1.0
    if result.boxes is None:
        return None
    for box in result.boxes:
        if int(box.cls[0]) != ball_class:
            continue
        x1, y1, x2, y2 = box.xyxy[0].cpu().tolist()
        area = max(0.0, x2 - x1) * max(0.0, y2 - y1)
        if area > best_area:
            best = (x1, y1, x2, y2, float(box.conf[0]))
            best_area = area
    return best


def main():
    args = parse_args()
    model = YOLO(args.model)
    capture = LatestFrameCapture(args.stream_url)
    capture.start()
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    previous_time = time.perf_counter()
    last_detection = None
    last_detection_time = float("-inf")

    try:
        while True:
            frame = capture.get()
            if frame is None:
                time.sleep(0.01)
                continue

            height, width = frame.shape[:2]
            result = model.track(frame, persist=True, classes=[args.ball_class],
                                 conf=args.confidence, verbose=False)[0]
            ball = select_largest_ball(result, args.ball_class)
            now = time.perf_counter()
            packet = {"angle": 0.0, "distance": 0.0, "sees": 0.0,
                      "conf": 0.0, "w": 0.0, "h": 0.0}

            if ball is not None:
                x1, y1, x2, y2, confidence = ball
                box_width, box_height = x2 - x1, y2 - y1
                centre_x = (x1 + x2) * 0.5
                packet.update({
                    "angle": max(-1.0, min(1.0, (centre_x - width * 0.5) / (width * 0.5))),
                    "distance": max(0.0, min(1.0, box_height / height)),
                    "sees": 1.0,
                    "conf": confidence,
                    "w": box_width,
                    "h": box_height,
                })
                last_detection = packet.copy()
                last_detection_time = now
                if not args.no_preview:
                    cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            elif now - last_detection_time <= max(0.0, args.visibility_hold_seconds):
                # A one-frame detector miss must not make the learned policy turn
                # away from an otherwise stable target. The retained angle and
                # distance are deliberately marked as visible during this grace period.
                packet = last_detection.copy()

            udp.sendto(json.dumps(packet).encode("utf-8"), (args.udp_ip, args.udp_port))

            if not args.no_preview:
                fps = 1.0 / max(now - previous_time, 0.0001)
                previous_time = now
                cv2.putText(frame, f"YOLO {fps:.1f} FPS", (10, 24),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.imshow("GFS-X P7 YOLO", frame)
                if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                    break
    finally:
        capture.stop()
        udp.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

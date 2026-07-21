#!/usr/bin/env python3
"""Send predictable fake YOLO packets to test RealYoloCamera without a robot."""

import json
import math
import socket
import time


sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
target = ("127.0.0.1", 5005)
sequence = 0
print("Sending test packets to Unity UDP 5005. Press Ctrl+C to stop.")
try:
    while True:
        phase = time.perf_counter()
        angle = math.sin(phase * 0.8) * 0.8
        distance = (math.sin(phase * 0.35) + 1.0) * 0.5
        packet = {
            "protocol": 1,
            "sequence": sequence,
            "sees": 1.0,
            "angle": angle,
            "distance": distance,
            "confidence": 0.92,
            "bboxWidth": 100.0,
            "bboxHeight": 100.0,
            "bboxHeightRatio": 0.25,
            "inferenceMs": 15.0,
        }
        sock.sendto(json.dumps(packet).encode("utf-8"), target)
        sequence += 1
        time.sleep(0.05)
except KeyboardInterrupt:
    pass
finally:
    sock.close()

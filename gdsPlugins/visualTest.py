# Testing visuals code before IMU telemetry is received to ensure proper protocol. 

import json
import socket
import time

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

packet = {
    "accel_x": 0.0,
    "accel_y": 0.0,
    "accel_z": -1.0,
    "ang_x": 0.1,
    "ang_y": 0.2,
    "ang_z": 0.3,
}

sock.sendto(json.dumps(packet).encode("utf-8"), ("127.0.0.1", 5005))
time.sleep(0.5)
print("sent")

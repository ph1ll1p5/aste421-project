"""
gdsReceiver.py — FPrime GDS plugin for IMU telemetry

Handles the single asteIMU.imuManager.Reading channel which carries
an ImuData struct:
    acceleration: {x, y, z}   — linear acceleration (m/s² or mg)
    rotation:     {x, y, z}   — angular velocity (deg/s)
    temperature:  int          — ignored

Forwards a flat JSON UDP packet to the visualizer on port 5005.
"""

import json
import socket
import time
from typing import Dict, Optional

from fprime_gds.common.handlers import DataHandlerPlugin
from fprime_gds.plugin.definitions import gds_plugin

READING_CHANNEL = "asteIMU.imuManager.Reading"


def _unpack_struct(val_obj) -> Optional[Dict[str, float]]:
    try:
        val = val_obj.val
        accel = val["acceleration"]  # now carrying euler angles: x=roll, y=pitch, z=yaw
        return {
            "roll":  float(accel["x"]),
            "pitch": float(accel["y"]),
            "yaw":   float(accel["z"]),
        }
    except Exception as e:
        print(f"[gdsReceiver] Unpack error: {e}")
        return None

@gds_plugin(DataHandlerPlugin)
class Gdsreceiver(DataHandlerPlugin):

    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.visualizer_addr = ("127.0.0.1", 5005)
        self.seq = 0
        print(f"[gdsReceiver] initialized — listening for {READING_CHANNEL}")
        print(f"[gdsReceiver] forwarding to UDP 127.0.0.1:5005")

    @classmethod
    def get_name(cls):
        return "gds-receiver"

    @classmethod
    def get_arguments(cls):
        return {}

    def init(self):
        pass

    def get_handled_descriptors(self):
        return ["FW_PACKET_TELEM"]

    def data_callback(self, data, source):
        full_name = data.template.get_full_name()

        if full_name != READING_CHANNEL:
            return  # ignore CPU/memory/queue telemetry

        val_obj = data.get_val_obj()
        fields = _unpack_struct(val_obj)

        if fields is None:
            return

        self.seq += 1
        packet = {"seq": self.seq, "ts": time.time(), "name": full_name, "roll": fields["roll"], "pitch": fields["pitch"], "yaw": fields["yaw"]}
        print("[gdsReceiver] sending:", packet)
        self.sock.sendto(json.dumps(packet).encode("utf-8"), self.visualizer_addr)
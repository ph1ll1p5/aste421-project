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
    """
    Walk the SerializableType tree and extract accel/rotation x,y,z.
    Prints debug info the first time it runs so you can verify field names.
    """
    try:
        members = val_obj.val  # list of (name, sub_val_obj)
        result = {}

        for field_name, sub_val in members:
            fname = field_name.lower()

            # acceleration sub-struct
            if "accel" in fname or fname == "acceleration":
                for sub_name, sub_sub in sub_val.val:
                    result[f"accel_{sub_name.lower()}"] = float(sub_sub.val)

            # rotation / gyro sub-struct
            elif "rotat" in fname or "gyro" in fname or "angular" in fname:
                for sub_name, sub_sub in sub_val.val:
                    result[f"ang_{sub_name.lower()}"] = float(sub_sub.val)

        required = {"accel_x", "accel_y", "accel_z", "ang_x", "ang_y", "ang_z"}
        if not required.issubset(result.keys()):
            print(f"[gdsReceiver] Partial unpack — got: {list(result.keys())}")
            print(f"[gdsReceiver] Raw members: {[(n, type(v)) for n,v in members]}")
            return None

        return result

    except Exception as e:
        print(f"[gdsReceiver] Unpack error: {e}")
        print(f"[gdsReceiver] val_obj type: {type(val_obj)}, val: {val_obj.val}")
        return None


@gds_plugin(DataHandlerPlugin)
class Gdsreceiver(DataHandlerPlugin):

    @classmethod
    def get_name(cls):
        return "gds-receiver"

    @classmethod
    def get_arguments(cls):
        return {}

    def init(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.visualizer_addr = ("127.0.0.1", 5005)
        self.seq = 0
        self._debug_printed = False
        print(f"[gdsReceiver] initialized — listening for {READING_CHANNEL}")
        print(f"[gdsReceiver] forwarding to UDP 127.0.0.1:5005")

    def get_handled_descriptors(self):
        return ["FW_PACKET_TELEM"]

    def data_callback(self, data, source):
        full_name = data.template.get_full_name()
        print("[gdsReceiver] callback:", full_name)

        # temporarily disable this filter
        # if full_name != READING_CHANNEL:
        #     return

        val_obj = data.get_val_obj()
        print("[gdsReceiver] val_obj:", val_obj.val)

        fields = _unpack_struct(val_obj)
        print("[gdsReceiver] unpacked:", fields)

        if fields is None:
            return

        packet = {"seq": self.seq, "ts": time.time(), "name": full_name, **fields}
        print("[gdsReceiver] sending:", packet)
        self.sock.sendto(json.dumps(packet).encode("utf-8"), self.visualizer_addr)

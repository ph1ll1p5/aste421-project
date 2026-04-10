import json
import re
import socket
from typing import Any, Dict, Optional

from fprime_gds.common.handlers import DataHandlerPlugin
from fprime_gds.plugin.definitions import gds_plugin


def _safe_float(value: Any) -> Optional[float]:
    try:
        return float(value)
    except Exception:
        return None


def _field_from_channel(full_name: str) -> Optional[str]:
    """
    Map your F´ channel names to imu fields.

    Edit this mapping to match your dictionary names.
    """
    name = full_name.lower()

    aliases = {
        "accel_x": ["accel_x", "acc_x", "imu.accel.x", ".ax", "_ax", "xaccel"],
        "accel_y": ["accel_y", "acc_y", "imu.accel.y", ".ay", "_ay", "yaccel"],
        "accel_z": ["accel_z", "acc_z", "imu.accel.z", ".az", "_az", "zaccel"],
        "ang_x": ["ang_x", "gyro_x", "angacc_x", "imu.gyro.x", ".gx", "_gx"],
        "ang_y": ["ang_y", "gyro_y", "angacc_y", "imu.gyro.y", ".gy", "_gy"],
        "ang_z": ["ang_z", "gyro_z", "angacc_z", "imu.gyro.z", ".gz", "_gz"],
    }

    for field, patterns in aliases.items():
        if any(p in name for p in patterns):
            return field
    return None


@gds_plugin(DataHandlerPlugin)
class Gdsreceiver(DataHandlerPlugin):
    """Receive decoded telemetry and forward IMU packets to the visualizer."""

    @classmethod
    def get_name(cls):
        return "gds-receiver"

    @classmethod
    def get_arguments(cls):
        return {}

    def init(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.visualizer_addr = ("127.0.0.1", 5005)
        self.latest: Dict[str, float] = {}

    def get_handled_descriptors(self):
        return ["FW_PACKET_TELEM"]

    def data_callback(self, data, source):
        full_name = data.template.get_full_name()
        field = _field_from_channel(full_name)
        if field is None:
            return

        value = _safe_float(data.get_val_obj().val)
        if value is None:
            return

        self.latest[field] = value

        packet = {
            "name": full_name,
            "accel_x": self.latest.get("accel_x"),
            "accel_y": self.latest.get("accel_y"),
            "accel_z": self.latest.get("accel_z"),
            "ang_x": self.latest.get("ang_x"),
            "ang_y": self.latest.get("ang_y"),
            "ang_z": self.latest.get("ang_z"),
        }

        if None not in packet.values():
            self.sock.sendto(
                json.dumps(packet).encode("utf-8"),
                self.visualizer_addr,
            )

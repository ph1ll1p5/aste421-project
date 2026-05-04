import json
import re
import socket
import time
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
    Broader matching so the plugin works even if your F´ channel names
    are slightly different than expected.
    """
    name = full_name.lower()

    patterns = {
        "accel_x": [r"acc.*x", r"x.*acc", r"_ax\b", r"\.ax\b", r"accelx"],
        "accel_y": [r"acc.*y", r"y.*acc", r"_ay\b", r"\.ay\b", r"accely"],
        "accel_z": [r"acc.*z", r"z.*acc", r"_az\b", r"\.az\b", r"accelz"],
        "ang_x":   [r"gyro.*x", r"x.*gyro", r"_gx\b", r"\.gx\b", r"ang.*x"],
        "ang_y":   [r"gyro.*y", r"y.*gyro", r"_gy\b", r"\.gy\b", r"ang.*y"],
        "ang_z":   [r"gyro.*z", r"z.*gyro", r"_gz\b", r"\.gz\b", r"ang.*z"],
    }

    for field, pats in patterns.items():
        for pat in pats:
            if re.search(pat, name):
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

        # Keep last-known values so every received field can refresh the plot
        self.latest: Dict[str, float] = {
            "accel_x": 0.0,
            "accel_y": 0.0,
            "accel_z": 0.0,
            "ang_x": 0.0,
            "ang_y": 0.0,
            "ang_z": 0.0,
        }
        self.seq = 0
        print("gds-receiver initialized -> sending UDP to 127.0.0.1:5005")

    def get_handled_descriptors(self):
        return ["FW_PACKET_TELEM"]

    def data_callback(self, data, source):
        full_name = data.template.get_full_name()
        field = _field_from_channel(full_name)
        value = _safe_float(data.get_val_obj().val)

        # Temporary debug so you can see what F´ is actually sending
        print("telemetry:", full_name, "->", field, value)

        if field is None or value is None:
            return

        self.latest[field] = value
        self.seq += 1

        packet = {
            "seq": self.seq,
            "ts": time.time(),
            "name": full_name,
            **self.latest,
        }

        self.sock.sendto(
            json.dumps(packet).encode("utf-8"),
            self.visualizer_addr,
        )
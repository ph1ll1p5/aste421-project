import sys
from fprime_gds.executables.apps import GdsApp
from fprime_gds.plugin.definitions import gds_plugin


@gds_plugin(GdsApp)
class GdsInterface(GdsApp):
    """Launch the IMU visualization server."""

    @classmethod
    def get_name(cls):
        return "gds-interface"

    def get_process_invocation(self):
        return [
            sys.executable,
            "-m",
            "ui.imu_viz_server",
            "--host",
            "127.0.0.1",
            "--port",
            "5005",
        ]

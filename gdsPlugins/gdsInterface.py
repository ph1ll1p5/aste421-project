# Source: https://nasa.github.io/fprime/HowTo/develop-gds-plugins.html

import sys
from fprime_gds.executables.apps import GdsApp
from fprime_gds.plugin.definitions import gds_plugin_implementation

class ImuVisualizerApp(GdsApp):
    def get_process_invocation(self):
        # Launch the visualizer process here
        return [sys.executable, "-m", "ui.imu_viz_server"]

    @classmethod
    def get_name(cls):
        return "imu-visualizer"

    @classmethod
    @gds_plugin_implementation
    def register_gds_app_plugin(cls):
        return clst to the visualization app
            self.sock.sendto(json.dumps(packet).encode(), self.addr)

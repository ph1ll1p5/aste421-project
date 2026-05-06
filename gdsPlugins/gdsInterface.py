#GDS plugins define a class that inherits from the 
# implementation base class and implements all 
# virtual functions. These classes also define a 
# properly decorated class method for the registration 
# function, and may define the other class methods 
# used for CLI interaction.
# Source: https://fprime.jpl.nasa.gov/latest/docs/how-to/develop-gds-plugins/#developing-a-plugin

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
        # Launch the HTTP+WebSocket bridge bundled in this repo.
        # It listens for UDP JSON IMU packets and streams them to a browser.
        return [
            sys.executable,
            "-m",
            "gdsPlugins.ui.imuViz",
            "--host",
            "192.168.0.1",
            "--port",
            "5005",
            "--ws-port",
            "8765",
            "--http-port",
            "9000",
        ]       

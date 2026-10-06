"""
Siglent SSA3015X Plus — Analizador de espectro SCPI sobre TCP/IP.
"""
from __future__ import annotations

from controller.base.scpi_analyzer import ScpiAnalyzer
from controller.base.scpi_commands import SCPIError


class SiglentAnalyzer(ScpiAnalyzer):
    IDN_PREFIX = "SIGLENT"
    DEFAULT_PORT = 5025

    def set_marker_3dB_bw(self, marker: int, state: str):
        self.set_marker_bw(marker, state, -3)

    def get_marker_3dB_bw(self, marker: int) -> str:
        success, response = self.send_command(f":CALCulate:MARKer{marker}:BANDwidth:NDB?")
        if not success:
            raise SCPIError(f"Fallo al leer BW marker {marker}: {response}")
        self.logger.info("Marker %d BW medido", marker)
        return response

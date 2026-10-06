"""Dummy BeckerOtaDecoder — synthetic frames from becker_ota_spec.md §4.

Uses 25 real known-good frames indexed by injected RF power level (dBm).
Supports set_simulated_power(dbm) to select which frames are returned
by capture(), enabling the full suite sweep without hardware.
"""

from __future__ import annotations

import logging
from typing import Any

from controller.becker.becker_frame_validator import validate_frame
from controller.dummy.mixin import DummyFallbackMixin

# Known-good frames indexed by injected RF power in dBm.
# Multiple frames per power level simulate real capture variance.
SYNTHETIC_FRAMES: dict[float, list[bytes]] = {
    -40.0: [bytes.fromhex("55FF66FF967C69A01000006D45")],
    -26.0: [bytes.fromhex("55FF66FF967C75A0100000BC87")],
    -15.5: [bytes.fromhex("55FF66FF967C90A01777006760")],
    -5.5:  [bytes.fromhex("55FF66FF967C9FA01FFF00D4A3")],
    0.0:   [bytes.fromhex("55FF66FF967C67A01000000484"),
            bytes.fromhex("55FF66FF967C64A01000004084")],
    +2.4:  [bytes.fromhex("55FF66FF967CFFA01FFF0054AB"),
            bytes.fromhex("55FF66FF967CFFA01FFF0054AB")],
    +4.5:  [bytes.fromhex("55FF66FF967CFFA01FFF0054ABFF")],
    +7.5:  [bytes.fromhex("55FF66FF967C68A01000005085"),
            bytes.fromhex("55FF66FF967C65A01000007D44")],
}

# Additional frames used for noise-floor or button-pressed scenarios.
NOISE_FRAMES: list[bytes] = [
    bytes.fromhex("55FF66FF967C5C50100000D241"),
    bytes.fromhex("55FF66FF967C5E50100000AB81"),
    bytes.fromhex("55FF66FF967C5278100000B220"),
]

# Voltage-sweep frames.
VOLTAGE_SWEEP_FRAMES: dict[float, list[bytes]] = {
    12.0: [bytes.fromhex("55FF66FF967C5C50100000D241")],
    18.0: [bytes.fromhex("55FF66FF967C5278100000B220")],
    24.0: [bytes.fromhex("55FF66FF967C56A01000007940")],
}


class DummyBeckerOtaDecoder(DummyFallbackMixin):
    """Dummy Becker OTA decoder — no hardware required.

    Returns synthetic frames matching the spec's known-good data set.
    Call set_simulated_power(dbm) and set_simulated_voltage(v)
    to control which frames are returned by capture().
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__()
        self.logger = logging.getLogger(self.__class__.__name__)
        self._simulated_power_dbm: float = -40.0
        self._simulated_voltage_v: float = 24.0
        self._configured: bool = False
        self._calib_pulses: int = 0
        self._connected: bool = True
        self.logger.info("Becker OTA decoder inicializado (dummy)")

    def connect(self, device_path: str = "", baudrate: int = 115200, timeout: float = 2.0) -> None:
        self._connected = True
        self.logger.info("Dummy conectado a %s", device_path)

    def disconnect(self) -> None:
        self._connected = False
        self.logger.info("Dummy desconectado")

    def configure(self) -> None:
        """Simulate 'C' command — configure CC1125 for Becker Varis RX."""
        self._configured = True
        self.logger.info("Dummy configurado para Becker Varis")

    def capture(self, secs: int = 5) -> list[bytes]:
        """Return synthetic frames for current power/voltage settings."""
        if not self._configured:
            self.logger.warning("Dummy no configurado — capturando de todas formas")
        frames: list[bytes] = []
        if self._simulated_power_dbm in SYNTHETIC_FRAMES:
            frames.extend(SYNTHETIC_FRAMES[self._simulated_power_dbm])
        if self._simulated_voltage_v in VOLTAGE_SWEEP_FRAMES:
            frames.extend(VOLTAGE_SWEEP_FRAMES[self._simulated_voltage_v])
        if not frames:
            frames.append(NOISE_FRAMES[0])
        self.logger.info(
            "Dummy captura: %d frames (potencia=%.1f dBm, voltaje=%.1f V)",
            len(frames), self._simulated_power_dbm, self._simulated_voltage_v,
        )
        return frames

    def get_rssi(self) -> int:
        """Return simulated RSSI."""
        if self._simulated_power_dbm < -30:
            return -70 + int(self._simulated_power_dbm * 0.5)
        return -50 + int(self._simulated_power_dbm * 0.3)

    def part_number(self) -> int:
        """Return simulated CC1125 part number."""
        return 0x58

    def idn(self) -> str:
        """Identificacion para la sonda de salud, igual que el driver real.

        Hace falta explicito: sin el, DummyFallbackMixin lo resuelve devolviendo None y
        /health falla en modo dummy, que es justo el modo en que se prueba sin hardware.
        """
        return "UQOMM CC1125 reader (dummy), CC1125, PN=0x58"

    def calib_pulse(self, duration_ms: int = 100) -> int:
        """Simula el pulso al boton de diagnostico del amplificador (0x09).

        Devuelve la duracion "aceptada" acotada a 2000 ms, igual que el firmware, para que
        un test de la cota se comporte igual con dummy y con placa. Lleva la cuenta en
        `_calib_pulses` para poder afirmar que la suite lo llamo.
        """
        aceptada = max(0, min(int(duration_ms), 2000))
        self._calib_pulses += 1
        self.logger.info("Dummy calib_pulse: %d ms (pedidos %d)", aceptada, duration_ms)
        return aceptada

    def set_simulated_power(self, dbm: float) -> None:
        """Set the simulated injected RF power level."""
        self._simulated_power_dbm = dbm
        self.logger.debug("Potencia simulada: %.1f dBm", dbm)

    def set_simulated_voltage(self, volts: float) -> None:
        """Set the simulated DC supply voltage."""
        self._simulated_voltage_v = volts
        self.logger.debug("Voltaje simulado: %.1f V", volts)

"""Dummy VLAD controller — no hardware required.

Simula una placa VLAD Rev25 en memoria para testing sin hardware.
Responde a comandos de ganancia y AGC con valores plausibles.
"""

from __future__ import annotations

import logging

from controller.base.vlad import Vlad
from controller.dummy.mixin import DummyFallbackMixin
from controller.uqomm import vlad25_protocol as proto


class DummyVlad(DummyFallbackMixin, Vlad):
    """Mock VLAD controller con valores simulados en memoria."""

    def __init__(self, **kwargs) -> None:
        super().__init__()
        self.logger = logging.getLogger(self.__class__.__name__)
        self._connected = False
        self._gain: float = 19.5
        #: VOLTS, y estuvo en -5.0 hasta el 09-Sep-2026: el residuo de cuando la API iba en
        #: dBm (ID-1644). -5 V no existe en el rango del DAC (0,4-1,8 V), asi que toda corrida
        #: con `vlad: dummy` leia una referencia imposible antes del primer set_agc. Es el
        #: centro del rango del detector.
        self._agc: float = 1.1
        self.logger.info("DummyVlad initialized (in-memory)")

    def idn(self) -> str:
        return "VLAD Dummy, Rev25, Simulated, Serial: SIM-0001"

    def connect(self) -> bool:
        self._connected = True
        self.logger.info("DummyVlad connected (simulated)")
        return True

    def disconnect(self) -> None:
        self._connected = False
        self.logger.info("DummyVlad disconnected")

    def is_healthy(self) -> bool:
        return self._connected

    def set_gain(self, gain_db: float, channel: str = "UL") -> bool:
        self._gain = max(0.0, min(31.5, gain_db))
        self.logger.info("DummyVlad gain[%s] -> %.1f dB", channel, self._gain)
        return True

    def set_agc(self, agc_volts: float, channel: str = "UL") -> bool:
        # Volts, no dBm. El rango es 0,4-1,8 V y no 0-3,3: es el que declara el driver del DAC
        # para el pin SET del detector, y desde el 06-Ago el firmware lo APLICA en
        # volts_to_code(). Se acota con la funcion del protocolo para no duplicar el rango.
        self._agc = proto.clamp_agc_volts(agc_volts)
        self.logger.info("DummyVlad AGC[%s] -> %.3f V", channel, self._agc)
        return True

    def get_gain(self) -> float | None:
        return self._gain

    def get_agc(self) -> float | None:
        return self._agc

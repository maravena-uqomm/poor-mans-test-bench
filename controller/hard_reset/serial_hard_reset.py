"""SerialHardResetController — controlador real de relé de hard reset.

Protocolo (dato del usuario, ver docs/runbooks/hard_reset_instrumentos.md §1):
  - Serial 115200 baud.
  - Dos comandos de texto: "who\\r" y "off\\r".
  - Respuesta: siempre "1", para cualquiera de los dos — pero con relleno alrededor
    (medido 23-Sep-2026 contra el relé real: `'1\\r\\x00'`, no un `'1'` limpio). Se filtra a
    solo caracteres imprimibles no-espacio antes de comparar — ver `_clean_response()`.
  - "off" es un PULSO: el instrumento protegido se reenergiza solo a los 5 s
    (`HARD_RESET_PULSE_S`). No hay comando "on"/"cycle". El propio relé tampoco contesta
    "who" durante esos 5 s.

No tolera fallos en silencio: `power_off()` que no confirma la respuesta esperada levanta
`HardwareError` — "corté la energía" es una afirmación que no se puede dejar sin confirmar.
"""

from __future__ import annotations

import logging
import time

try:
    import serial as pyserial
    from serial import SerialException
except ImportError:  # fallback when pyserial not installed
    pyserial = None
    SerialException = OSError  # fallback when pyserial not installed

from controller.base.hard_reset import HARD_RESET_PULSE_S, HardResetController
from controller.base.reliability import HardwareError

BAUDRATE = 115200
READ_TIMEOUT_S = 0.5
EXPECTED_RESPONSE = "1"


def _clean_response(raw: str) -> str:
    """Deja solo caracteres imprimibles no-espacio.

    El relé manda relleno alrededor del '1' real — medido 23-Sep-2026 contra la unidad del
    OWON: `'1\\r\\x00'`, no un `'1'` limpio. `str.strip()` NO alcanza: `\\x00` (NUL) no es
    whitespace para Python (`'\\x00'.isspace()` es False), así que `'1\\r\\x00'.strip()`
    devuelve la cadena intacta y una comparación exacta contra `"1"` falla siempre — el bug
    real que hizo que `who()`/`power_off()` dieran "no confirmado" con un corte que sí había
    pasado (confirmado por `/health` y a ojo, banco real, mismo día).
    """
    return "".join(ch for ch in raw if ch.isprintable() and not ch.isspace())


class SerialHardResetController(HardResetController):
    def __init__(self, device_path: str, timeout_s: float = READ_TIMEOUT_S, **kwargs: object) -> None:
        self.logger = logging.getLogger(self.__class__.__name__)
        self.device_path = device_path
        self.timeout_s = timeout_s
        if pyserial is None:
            raise HardwareError(
                f"pyserial no esta instalado — no se puede abrir el rele en {device_path}",
                instrument_name="hard_reset",
            )
        try:
            self._port = pyserial.Serial(device_path, BAUDRATE, timeout=timeout_s)
        except SerialException as exc:
            raise HardwareError(
                f"no se pudo abrir el rele de hard reset en {device_path}: {exc}",
                instrument_name="hard_reset",
            ) from exc

    def __enter__(self) -> SerialHardResetController:
        return self

    def __exit__(self, exc_type: object, exc_val: object, exc_tb: object) -> None:
        self.disconnect()

    def disconnect(self) -> None:
        try:
            if self._port.is_open:
                self._port.close()
        except SerialException:
            self.logger.exception("hard_reset[%s]: error cerrando el puerto", self.device_path)

    def _send(self, command: str) -> str | None:
        try:
            self._port.reset_input_buffer()
            self._port.write(f"{command}\r".encode("ascii"))
            crudo = self._port.readline().decode("ascii", errors="replace")
            respuesta = _clean_response(crudo)
        except SerialException as exc:
            self.logger.error("hard_reset[%s]: %s fallo — %s", self.device_path, command, exc)
            return None
        return respuesta or None

    def who(self) -> bool:
        respuesta = self._send("who")
        es_rele = respuesta == EXPECTED_RESPONSE
        self.logger.info(
            "hard_reset[%s]: who -> %r (%s)",
            self.device_path,
            respuesta,
            "es un rele" if es_rele else "NO es un rele",
        )
        return es_rele

    def power_off(self) -> None:
        respuesta = self._send("off")
        if respuesta != EXPECTED_RESPONSE:
            raise HardwareError(
                f"hard_reset[{self.device_path}]: 'off' no confirmado (respuesta={respuesta!r}) "
                "— no se puede asumir que el corte de energia ocurrio",
                instrument_name="hard_reset",
            )
        self.logger.warning("hard_reset[%s]: OFF confirmado — pulso de corte disparado", self.device_path)

    def wait_reenergized(self, hold_s: float = HARD_RESET_PULSE_S) -> None:
        self.logger.info("hard_reset[%s]: esperando %.1fs de reenergizado automatico", self.device_path, hold_s)
        time.sleep(hold_s)

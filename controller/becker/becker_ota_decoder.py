"""becker_ota_decoder.py — Driver del lector CC1125 sobre el UART de la NUCLEO-G070RB.

Uso:      d = BeckerOtaDecoder(); d.connect("/dev/becker_reader")
          d.configure(); tramas = d.capture(secs=5); d.disconnect()
No hace:  no valida el contenido de la trama Becker — eso es becker_frame_validator.py.

HABLA RDSS BINARIO, NO ASCII
----------------------------
Este driver mandaba comandos ASCII de una letra ('C', 'F', 'K', 'P', 'D'). **El firmware
retiró esa consola**: fuera de trama, cualquier byte que no sea 0x7E se descarta sin
efecto (`cc1125_reader_g070rb/Core/Src/main.cpp:96-99`). Los comandos viejos se perdían
en silencio, así que la suite entera no podía hablar con la placa.

Correspondencia con lo que había:

    'P'  part number   -> CMD_PART_NUMBER (0x01)
    'K'  RSSI          -> CMD_STATUS      (0x03), que además trae MARC, freq y tx_drops
    'C'  configurar    -> CMD_CONFIGURE   (0x04)
    'F'  capturar      -> no existe: las tramas OTA llegan solas como 0x06
    'D'  volcar regs   -> sin equivalente, retirado

La interfaz pública **no cambia**, para que `controller/dummy/becker_ota_decoder.py`
siga siendo intercambiable con éste en los modos `passive` y `dummy` de la suite.
"""

from __future__ import annotations

import logging
import time
from typing import Any

from controller.base.serial_controller import SerialController

from .rdss_frame import (
    Accumulator, CALIB_PULSE_DEFAULT_MS, CMD_CALIB_PULSE, CMD_CONFIGURE,
    CMD_OTA_FRAME, CMD_PART_NUMBER, CMD_STATUS, OTA_LEN_EXTENDED,
    OTA_LEN_STANDARD, build,
)


class BeckerOtaDecoder(SerialController):
    """Driver del firmware lector CC1125 en la NUCLEO-G070RB."""

    BAUDRATE: int = 115200

    # Cuánto se espera una respuesta a un comando. La placa contesta en milisegundos,
    # pero se han visto respuestas por encima de 5 s cuando el enlace va cargado
    # (ver el runbook de pérdida de bytes), así que el margen es amplio a propósito.
    RESPONSE_TIMEOUT_S: float = 8.0
    _POLL_S: float = 0.02

    def __init__(self, device_path: str | None = None, **kwargs: Any) -> None:
        """
        Args:
            device_path: ruta del puerto. **Se recibe en el constructor a propósito**: la
                sonda de salud construye el controlador con `device_path` y llama a `idn()`
                sin pasar por `connect()`, así que si no se guardara aquí, `_reconnect()`
                no tendría a dónde reconectar. Reenviarlo a `super().__init__()` tampoco
                sirve: `SerialController.__init__` no lo acepta y la construcción falla.
                Es el mismo patrón que usa `VsgSerialToneGenerator`.
        """
        # Se pasan al base SOLO los kwargs que su firma acepta. HealthChecker inyecta
        # parametros que no son de este controlador —`use_bluetooth`, que usa para derivar
        # el transporte del VLAD— y reenviarlos hacia `SerialController.__init__` reventaba
        # la construccion con "unexpected keyword argument". Peor: eso da 'create_failed',
        # y al tercer intento el circuit breaker se abre y el error pasa a ser
        # 'probe_skipped:circuit_breaker_open', que ya no menciona la causa. Filtrar en vez
        # de enumerar lo que hay que ignorar evita repetirlo con el siguiente parametro que
        # alguien anada a la sonda.
        import inspect
        aceptados = inspect.signature(SerialController.__init__).parameters
        propios = {k: v for k, v in kwargs.items() if k in aceptados}
        ignorados = set(kwargs) - set(propios)
        super().__init__(**propios)
        self.logger = logging.getLogger(self.__class__.__name__)
        if ignorados:
            self.logger.debug("kwargs ignorados por no ser del transporte: %s",
                              sorted(ignorados))
        if device_path:
            self._device_path = device_path
        self._acc = Accumulator()
        self._pendientes: list[dict] = []   # tramas leídas y aún no consumidas

    # ── Construir / conectar ────────────────────────────────────────────

    def connect(
        self,
        device_path: str,
        baudrate: int = BAUDRATE,
        timeout: float = 2.0,
    ) -> None:
        super().connect(device_path, baudrate, timeout)
        self._acc = Accumulator()
        self._pendientes.clear()
        self.logger.info("Becker OTA decoder conectado a %s", device_path)

    # ── Transporte ──────────────────────────────────────────────────────

    def _bombear(self) -> None:
        """Lee lo que haya y guarda las tramas completas que salgan."""
        data = self.read_bytes()
        if data:
            self._pendientes.extend(self._acc.feed(data))

    def _esperar(self, cmd: int, timeout_s: float | None = None) -> dict | None:
        """Espera una trama con ese opcode. Las OTA que lleguen entretanto se guardan."""
        limite = time.time() + (timeout_s or self.RESPONSE_TIMEOUT_S)
        while time.time() < limite:
            for i, trama in enumerate(self._pendientes):
                if trama["cmd"] == cmd:
                    return self._pendientes.pop(i)
            self._bombear()
            time.sleep(self._POLL_S)
        self.logger.warning("La placa no respondio al comando 0x%02X", cmd)
        return None

    def _mandar(self, cmd: int, payload: bytes = b"") -> None:
        self.write_bytes(build(cmd, payload))

    # ── Control ─────────────────────────────────────────────────────────

    def configure(self) -> None:
        """`CMD_CONFIGURE` (0x04) — reset, configurar Becker y volver a RX."""
        self._mandar(CMD_CONFIGURE)
        if self._esperar(CMD_CONFIGURE) is None:
            raise ConnectionError("configure(): la placa no confirmo el 0x04")
        self.logger.info("CC1125 configurado para Becker Varis")

    def capture(self, secs: int = 5) -> list[bytes]:
        """Recoge las tramas OTA que llegan durante *secs* segundos.

        Las tramas llegan **sin pedirlas** (opcode 0x06): aquí solo se escucha. Se
        devuelven los payloads Becker crudos de 13 bytes (estándar) o 14 (extendida
        con byte de alarma), que es lo que espera `becker_frame_validator.validate_frame`.
        """
        if not 1 <= secs <= 30:
            raise ValueError(f"La captura debe durar 1-30 s, no {secs}")

        crudas: list[bytes] = []

        def cosechar() -> None:
            restantes = []
            for trama in self._pendientes:
                if (trama["cmd"] == CMD_OTA_FRAME
                        and len(trama["payload"]) in (OTA_LEN_STANDARD, OTA_LEN_EXTENDED)):
                    crudas.append(trama["payload"])
                elif trama["cmd"] != CMD_OTA_FRAME:
                    restantes.append(trama)   # respuesta a un comando: no es basura
                else:
                    self.logger.warning(
                        "Trama OTA de %d bytes, esperaba 13 o 14: %s",
                        len(trama["payload"]), trama["payload"].hex(),
                    )
            self._pendientes[:] = restantes

        limite = time.time() + secs
        while time.time() < limite:
            self._bombear()
            cosechar()
            time.sleep(self._POLL_S)
        cosechar()

        extendidas = sum(1 for c in crudas if len(c) == OTA_LEN_EXTENDED)
        self.logger.info(
            "Capturadas %d tramas OTA en ~%ds (%d extendidas), %d resyncs",
            len(crudas), secs, extendidas, self._acc.resyncs,
        )
        return crudas

    # ── Consultas ───────────────────────────────────────────────────────

    def status(self) -> dict | None:
        """`CMD_STATUS` (0x03) — part number, RSSI, MARC, palabra de frecuencia y tx_drops.

        `tx_drops` son los bytes que el firmware no logró transmitir. Es el dato que
        discrimina si la pérdida es del firmware o del enlace.
        """
        self._mandar(CMD_STATUS)
        trama = self._esperar(CMD_STATUS)
        if trama is None:
            return None
        p = trama["payload"]
        if len(p) not in (6, 10):
            self.logger.warning("Status de %d bytes, esperaba 6 o 10", len(p))
            return None
        datos = {
            "part_number": p[0],
            "rssi_dbm": p[1] - 256 if p[1] > 127 else p[1],   # int8 truncado por el firmware
            "marc_state": p[2],
            "freq_word": (p[3] << 16) | (p[4] << 8) | p[5],
        }
        if len(p) == 10:
            datos["tx_drops"] = (p[6] << 24) | (p[7] << 16) | (p[8] << 8) | p[9]
        return datos

    def get_rssi(self) -> int | None:
        """RSSI actual en dBm, o None si la placa no responde."""
        datos = self.status()
        return datos["rssi_dbm"] if datos else None

    def part_number(self) -> int | None:
        """`CMD_PART_NUMBER` (0x01) — 0x58 si el CC1125 está vivo."""
        self._mandar(CMD_PART_NUMBER)
        trama = self._esperar(CMD_PART_NUMBER)
        if trama is None or len(trama["payload"]) != 1:
            return None
        return trama["payload"][0]

    def idn(self) -> str:
        """Identificación para la sonda de salud. **No es `*IDN?`.**

        `SerialController.idn()` manda `*IDN?` en texto, y este firmware descarta cualquier
        byte que no sea `0x7E` fuera de trama: la sonda no obtenía nada, el circuit breaker
        se abría y `/health` daba `probe_skipped:circuit_breaker_open` — el equipo parecía
        desconectado estando perfectamente vivo.

        Se pregunta el part number, que es lo más parecido a un `*IDN?` que tiene: si el
        CC1125 responde 0x58, la cadena entera —placa, UART, SPI y chip— está en pie.
        """
        pn = self.part_number()
        if pn is None:
            raise ConnectionError("el lector no respondio al part number (0x01)")
        estado = "CC1125" if pn == 0x58 else f"part number inesperado 0x{pn:02X}"
        return f"UQOMM CC1125 reader (NUCLEO-G070RB), {estado}, PN=0x{pn:02X}"

    def calib_pulse(self, duration_ms: int = CALIB_PULSE_DEFAULT_MS) -> int | None:
        """`CMD_CALIB_PULSE` (0x09) — pulsa el botón de diagnóstico del amplificador.

        Provoca que emita su trama **extendida** de 14 bytes con el byte de alarma, que es
        la única que nunca se había capturado: nada la disparaba. La trama llega después,
        por su cuenta, y la recoge `capture()` como cualquier otra.

        Devuelve la duración que el firmware **aceptó** —no la que se pidió—: la acota a
        2000 ms, así que pedir más se recorta y hay que verlo.
        """
        if duration_ms < 0:
            raise ValueError(f"duration_ms negativo: {duration_ms}")
        self._mandar(CMD_CALIB_PULSE,
                     bytes([(duration_ms >> 8) & 0xFF, duration_ms & 0xFF]))
        trama = self._esperar(CMD_CALIB_PULSE)
        if trama is None or len(trama["payload"]) != 2:
            self.logger.warning("calib_pulse: la placa no confirmo el 0x09")
            return None
        aceptada = (trama["payload"][0] << 8) | trama["payload"][1]
        if aceptada != duration_ms:
            self.logger.info(
                "calib_pulse: se pidieron %d ms y el firmware acepto %d (lo acota a 2000)",
                duration_ms, aceptada,
            )
        return aceptada

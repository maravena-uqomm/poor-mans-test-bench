"""VladLoraLink — la Rutherford en el lugar del gateway: pregunta en V2 por downlink, oye el uplink, emite el tono.

Uso:  link = VladLoraLink(board.lora())
      r = link.query_v2(0x3E, dst=nodo, src_esperado=uid0)   # consulta dirigida, respuesta validada
      with link.smartring_tone(period_s=0.5) as tono: tono.set_enabled(False)
No hace: USB con el VLAD, ni veredictos. Mueve tramas; quien las compara contra el USB es la suite.

El camino de RF, con los nombres de cada punta (verificado en banco el 22-Sep-2026):

    Rutherford radio Base ──acoplador── DUT-BASE ══cable══ VLAD BASE (P3)
        DOWNLINK 149,5 MHz ──> VLAD amplifica DL ──> radio TUNNEL (U26) escucha
        UPLINK 173,5 MHz  <── VLAD amplifica UL <── radio TUNNEL contesta
        SMART_RING 143 MHz ──> radio BASE (U25) detecta el anillo

Un link por vez no impide preguntar y oir: el VLAD queda en recepcion continua, revisa cada 500 ms
y contesta en esa vuelta; el cambio a UPLINK cuesta una fraccion de eso. Si la respuesta sale antes
de que el receptor este armado se pierde: `query_v2()` reintenta y reporta el intento.
"""

from __future__ import annotations

import logging
import threading
import time
from collections.abc import Iterator
from contextlib import contextmanager
from dataclasses import dataclass

from rutherford.errors import RutherfordError
from rutherford.models import Link

from controller.uqomm.rutherford_lora import DOWNLINK_HZ, SMART_RING_HZ, UPLINK_HZ, RutherfordLora
from controller.uqomm.vlad_lora_frames import TONO_SMARTRING, FrameError, build_v2, is_telemetry_frame, parse_v2

#: El SRC con que pregunta el banco. Cualquier valor != 0 sirve (0 hace que el VLAD descarte la
#: respuesta); este se reconoce en una captura: 'BNCH' en ASCII.
SRC_BANCO = 0x48434E42

#: Espera de la respuesta despues de armar el receptor. El VLAD revisa cada 500 ms y una respuesta
#: V2 de 68 bytes son ~130 ms de aire en SF7/BW125: 1,5 s deja margen.
_REPLY_TIMEOUT_S = 1.5

#: Despues de contestar, el VLAD queda en standby hasta su proxima vuelta de recepcion: `transmit()`
#: no re-arma el RX, lo hace el siguiente `receive()` del lazo, cada 500 ms (fw-vlad Lora.hpp y
#: tasks.cpp). Un pedido que llega en ese hueco se pierde sin contarse. Medido el 22-Sep-2026: sin
#: esta guarda, 6 de 17 pedidos no los oyo, y el control negativo tampoco.
_GUARDA_VLAD_S = 0.6


class NoReplyError(TimeoutError):
    """El VLAD no contesto por el aire en ninguno de los intentos."""


@dataclass(frozen=True)
class LoraReply:
    """Respuesta V2 del VLAD, ya filtrada por DST, SRC y CMD, con lo que costo obtenerla."""

    cmd: int
    src: int
    data: bytes
    attempts: int
    latency_s: float
    discarded: int
    #: De TxDone a receptor armado en UPLINK, en el intento que contesto. Si el VLAD responde antes
    #: de esto, la respuesta se pierde: es lo que hay que mirar cuando sube `attempts`.
    armado_s: float = 0.0


class VladLoraLink:
    """El gateway emulado sobre las radios de la Rutherford.

    Args:
        lora: Las radios de la placa (`RutherfordBoard.lora()`).
        src: El remitente con que pregunta el banco; el VLAD lo devuelve como DST.
    """

    def __init__(self, lora: RutherfordLora, src: int = SRC_BANCO) -> None:
        self.logger = logging.getLogger(self.__class__.__name__)
        self._lora = lora
        self._src = src
        self._listo_desde = 0.0

    # ------------------------------------------------------------------
    # Control
    # ------------------------------------------------------------------

    def query_v2(
        self,
        cmd: int,
        data: bytes = b"",
        *,
        dst: int,
        src_esperado: int | None,
        attempts: int = 3,
        reply_timeout_s: float = _REPLY_TIMEOUT_S,
    ) -> LoraReply:
        """Pide `cmd` a `dst` por DOWNLINK y espera por UPLINK la respuesta de ESE equipo.

        Se acepta solo un sobre con DST == nuestro SRC, CMD == `cmd` y, si se da `src_esperado`,
        SRC == ese valor (el UID[0] de la unidad bajo prueba). Lo demas se descarta y se cuenta:
        en el aire del banco pasan telemetria y respuestas de otros equipos.
        """
        pedido = build_v2(dst, self._src, cmd, data)
        descartadas = 0

        for intento in range(1, attempts + 1):
            self._lora.use_link(Link.DOWNLINK, DOWNLINK_HZ)
            if (espera := self._listo_desde - time.monotonic()) > 0:
                time.sleep(espera)
            self._lora.transmit(pedido)
            t_tx = time.monotonic()
            self._lora.use_link(Link.UPLINK, UPLINK_HZ)
            armado = time.monotonic() - t_tx

            deadline = t_tx + reply_timeout_s
            while (restante := deadline - time.monotonic()) > 0:
                trama = self._lora.wait_packet(restante)
                if trama is None:
                    break
                try:
                    dst_resp, src_resp, cmd_resp, datos = parse_v2(trama)
                except FrameError:
                    descartadas += 1
                    continue
                if dst_resp != self._src or cmd_resp != cmd:
                    descartadas += 1
                    continue
                if src_esperado is not None and src_resp != src_esperado:
                    self.logger.warning("respuesta a %#04x de otro equipo (SRC %#010x)", cmd, src_resp)
                    descartadas += 1
                    continue
                ahora = time.monotonic()
                self._listo_desde = ahora + _GUARDA_VLAD_S
                return LoraReply(cmd, src_resp, datos, intento, ahora - t_tx, descartadas, armado)

            self.logger.info("sin respuesta a %#04x en el intento %d/%d", cmd, intento, attempts)

        raise NoReplyError(
            f"el VLAD no contesto {cmd:#04x} por LoRa en {attempts} intentos ({descartadas} descartadas)"
        )

    def wait_telemetry(self, timeout_s: float) -> bytes | None:
        """Escucha el uplink hasta recibir una trama de telemetria con CRC valido."""
        self._lora.use_link(Link.UPLINK, UPLINK_HZ)
        deadline = time.monotonic() + timeout_s
        while (restante := deadline - time.monotonic()) > 0:
            trama = self._lora.wait_packet(restante)
            if trama is None:
                return None
            if is_telemetry_frame(trama):
                return trama
        return None

    @contextmanager
    def smartring_tone(self, period_s: float = 0.5) -> Iterator[ToneEmitter]:
        """Emite el tono de 143 MHz por la radio Base mientras dura el bloque.

        El tono lo pauta el host: el protocolo de la placa es sincronico y no tiene baliza. Un hilo
        lo repite cada `period_s`; al salir se corta y se resetean las radios, para que ningun tono
        quede vivo contaminando la medicion siguiente.
        """
        emisor = ToneEmitter(self._lora, period_s)
        self._lora.use_link(Link.SMART_RING, SMART_RING_HZ)
        emisor.start()
        try:
            yield emisor
        finally:
            emisor.stop()
            self._lora.reset()


class ToneEmitter:
    """Hilo que transmite `TONO_SMARTRING` cada `period_s` mientras esta encendido.

    `set_enabled(False)` corta el tono sin terminar el hilo: es el "silencio" del lazo, y tiene que
    poder volver sin reconfigurar la radio.
    """

    def __init__(self, lora: RutherfordLora, period_s: float) -> None:
        self.logger = logging.getLogger(self.__class__.__name__)
        self._lora = lora
        self._period_s = period_s
        self._stop = threading.Event()
        self._enabled = threading.Event()
        self._enabled.set()
        self._thread = threading.Thread(target=self._run, name="smartring-tone", daemon=True)
        self.sent = 0
        self.failed = 0

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=self._period_s + 6.0)

    def set_enabled(self, enabled: bool) -> None:
        if enabled:
            self._enabled.set()
        else:
            self._enabled.clear()

    def _run(self) -> None:
        while not self._stop.is_set():
            if self._enabled.is_set():
                try:
                    self._lora.transmit(TONO_SMARTRING)
                    self.sent += 1
                except RutherfordError as exc:
                    # Un tono perdido se cuenta y el hilo sigue: cortarlo aca haria que el lazo
                    # lea "sin tono" como una decision del VLAD.
                    self.failed += 1
                    self.logger.warning("tono no emitido: %s", exc)
            self._stop.wait(self._period_s)

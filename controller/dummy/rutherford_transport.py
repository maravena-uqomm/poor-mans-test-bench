"""SimulatedTransport — la placa Rutherford modelada en memoria, sin hardware.

Uso:  RutherfordClient(port="simulated://", transport=SimulatedTransport())
No hace: RF, LoRa, sensores ni temporizacion realista. Modela el crossbar y nada mas.

Por que modela estado y no contesta `OK` a todo: la version anterior devolvia un ACK
enlatado para cualquier request, asi que `get_state()` levantaba `UnexpectedResponseError`
y ningun test podia verificar que el ruteo quedo como se pidio. Un transporte que siempre
acusa recibo reproduce exactamente el modo de falla que tuvimos meses en el banco —la placa
contestaba bien sin conmutar— y lo vuelve invisible en los tests.

Lo que si respeta del firmware (proto/rutherford.proto, spec/01-board-capabilities.md):

  * exclusividad por origen y por destino, y maximo 3 rutas → ERROR_CODE_ROUTING_CONFLICT;
  * ruteo append-only: solo `reset()` limpia el estado;
  * `reset()` deja cero rutas e impedancia en 50 Ohm (FR-S02).

Jira: ID-1308 (Test Bench v2)
"""

from __future__ import annotations

import logging
import struct
from typing import Any

from rutherford.errors import TransportError
from rutherford.generated import rutherford_pb2 as _pb

try:
    from google.protobuf.message import DecodeError
except ImportError:  # fallback cuando protobuf no esta instalado (doble de tests)

    class DecodeError(Exception):  # type: ignore[no-redef]
        """Sustituto de google.protobuf.message.DecodeError."""


LOGGER = logging.getLogger(__name__)

_HEADER = ">H"
_MAX_ROUTES = 3

#: Los ocho canales en orden DC_meas1..8, copiados de
#: `fw-rutherford/firmware/apps/rutherford/include/DcMeasurementTable.hpp`. El orden no es
#: alfabetico ni el de los enums: sale del ruteo de pines PE7..PE14 en la placa, asi que se
#: transcribe en vez de derivarse.
_CANALES_DC: tuple[tuple[str, int], ...] = (
    ("BASE", 75),    # DC_meas1 — Con_75Ohm1  — PE7
    ("BASE", 50),    # DC_meas2 — Con_50Ohm1  — PE8
    ("BR1", 75),     # DC_meas3 — Con_75Ohmb1 — PE9
    ("BR1", 50),     # DC_meas4 — Con_50Ohmb1 — PE10
    ("BR2", 75),     # DC_meas5 — Con_75Ohmb2 — PE11
    ("BR2", 50),     # DC_meas6 — Con_50Ohmb2 — PE12
    ("TUNNEL", 75),  # DC_meas7 — Con_75Ohm2  — PE13
    ("TUNNEL", 50),  # DC_meas8 — Con_50Ohm2  — PE14
)

#: Tension del riel simulado, en mV. El valor sale de un round-trip real del codec
#: (47,992 V), no de un numero redondo: un 48000 exacto esconde el error del divisor.
_DC_MILIVOLTS = 47992

#: Cuentas crudas coherentes con esa tension a traves del divisor 45,3k/3k y un ADC de 12
#: bits con referencia de 3,3 V. Aproximado a proposito: sirve para distinguir un canal
#: muerto de uno que lee cero, no para calibrar.
_DC_CUENTAS = 3719


def _frame(payload: bytes) -> bytes:
    return struct.pack(_HEADER, len(payload)) + payload


class SimulatedTransport:
    """Transporte que simula la placa en memoria, con el framing real.

    El caller recibe objetos ``Response`` de verdad —el codec protobuf se ejercita— pero
    no se abre ningun puerto serie.
    """

    def __init__(self, port: str = "simulated://") -> None:
        self._port = port
        self._open = False
        self._rx_buffer = bytearray()
        self._routes: list[tuple[int, int]] = []  # (instr, dut) en enums protobuf
        self._impedance = _pb.IMPEDANCE_50
        #: Canales cuya conversion falla, por indice 0..7. Vacio por default: un canal
        #: muerto es una averia, no el estado normal de la placa. Los tests lo cargan
        #: para ejercitar el camino de `volts=None`, que es el que se confunde con cero.
        self.canales_dc_caidos: set[int] = set()
        #: Conectores sin alimentar. Leen **cero volts, validos** — que es una medicion
        #: legal y distinta de "no se pudo medir". Por default los de 75 Ohm, porque el
        #: banco cablea los de 50.
        self.conectores_sin_alimentar: set[int] = {i for i, (_, ohms) in enumerate(_CANALES_DC) if ohms == 75}

    # -- Transport protocol ------------------------------------------------

    def open(self) -> None:
        self._open = True
        LOGGER.info("SimulatedTransport opened (%s)", self._port)

    def close(self) -> None:
        self._open = False
        self._rx_buffer.clear()
        LOGGER.info("SimulatedTransport closed (%s)", self._port)

    def write(self, data: bytes) -> None:
        if not self._open:
            raise TransportError("Transport is not open")
        LOGGER.debug("SimulatedTransport TX (%d bytes): %s", len(data), data.hex())
        self._rx_buffer.extend(_frame(self._respond(data)))

    def read_exact(self, size: int, timeout_s: float) -> bytes:
        if not self._open:
            raise TransportError("Transport is not open")
        if len(self._rx_buffer) < size:
            # No hay espera que valga: el simulador responde dentro de write(), asi que si
            # falta un byte es un error de programacion, no una lentitud del hardware.
            raise TransportError(f"Simulated underrun: se pidieron {size} bytes y hay {len(self._rx_buffer)}")
        chunk = bytes(self._rx_buffer[:size])
        del self._rx_buffer[:size]
        return chunk

    # -- Firmware simulado -------------------------------------------------

    def _respond(self, framed_request: bytes) -> bytes:
        """Decodifica el request enmarcado y devuelve el payload de la respuesta."""
        payload = framed_request[struct.calcsize(_HEADER) :]
        request = _pb.Request()
        try:
            request.ParseFromString(payload)
        except DecodeError:
            return self._error(_pb.ERROR_CODE_PROTOCOL_MALFORMED)

        subsistema = request.WhichOneof("subsystem")
        if subsistema == "peripheral":
            return self._periferico(request.peripheral)
        if subsistema != "routing":
            # LoRa no esta modelado: acusar recibo seria mentir.
            return self._error(_pb.ERROR_CODE_PROTOCOL_UNKNOWN_SYSTEM)

        cmd = request.routing.WhichOneof("cmd")
        if cmd == "reset":
            self._routes.clear()
            self._impedance = _pb.IMPEDANCE_50
            return self._ack()
        if cmd == "set_impedance":
            self._impedance = request.routing.set_impedance.impedance
            return self._ack()
        if cmd == "route":
            return self._route(request.routing.route)
        if cmd == "get_state":
            return self._state()
        return self._error(_pb.ERROR_CODE_PROTOCOL_UNKNOWN_CMD)

    def _periferico(self, peripheral_request: Any) -> bytes:
        """Solo `get_dc_measurements`. El resto del subsistema no esta modelado.

        Si la pylib instalada es anterior al mensaje, el `hasattr` da falso y se contesta
        UNKNOWN_CMD. Eso es lo honesto: la placa simulada no puede saber contestar algo
        que su propio protocolo generado no define, y un ACK enlatado reproduciria
        exactamente el modo de falla que este archivo existe para evitar.
        """
        if peripheral_request.WhichOneof("cmd") != "get_dc_measurements":
            return self._error(_pb.ERROR_CODE_PROTOCOL_UNKNOWN_CMD)
        if not hasattr(_pb, "DcMeasurementsResponse"):
            LOGGER.warning(
                "La pylib instalada no define DcMeasurementsResponse: el simulado no puede "
                "contestar la medicion de DC"
            )
            return self._error(_pb.ERROR_CODE_PROTOCOL_UNKNOWN_CMD)
        return self._mediciones_dc()

    def _mediciones_dc(self) -> bytes:
        """Los ocho canales. Si fallan los ocho, error 402 y no una lista de invalidos.

        Es lo que hace el firmware: con un canal caido contesta la lista y lo marca
        `valid=False`, para que un canal muerto no ciegue a los otros siete.
        """
        if len(self.canales_dc_caidos) >= len(_CANALES_DC):
            return self._error(_pb.ERROR_CODE_PERIPHERAL_ADC_FAULT)

        response = _pb.Response()
        for indice, (dut, ohms) in enumerate(_CANALES_DC):
            entry = response.dc_measurements.entries.add()
            entry.dut = getattr(_pb, f"DUT_INTERFACE_{dut}")
            entry.impedance = _pb.IMPEDANCE_75 if ohms == 75 else _pb.IMPEDANCE_50
            entry.valid = indice not in self.canales_dc_caidos
            if not entry.valid:
                continue
            sin_alimentar = indice in self.conectores_sin_alimentar
            entry.millivolts = 0 if sin_alimentar else _DC_MILIVOLTS
            entry.raw_counts = 0 if sin_alimentar else _DC_CUENTAS
        return bytes(response.SerializeToString())

    def _route(self, route_cmd: Any) -> bytes:
        instr, dut = route_cmd.instr, route_cmd.dut
        if len(self._routes) >= _MAX_ROUTES:
            return self._error(_pb.ERROR_CODE_ROUTING_CONFLICT)
        if any(instr == i or dut == d for i, d in self._routes):
            return self._error(_pb.ERROR_CODE_ROUTING_CONFLICT)
        self._routes.append((instr, dut))
        return self._ack()

    # -- Constructores de respuesta ----------------------------------------

    @staticmethod
    def _ack() -> bytes:
        response = _pb.Response()
        response.ack.SetInParent()
        return bytes(response.SerializeToString())

    def _state(self) -> bytes:
        response = _pb.Response()
        response.state.impedance = self._impedance
        for instr, dut in self._routes:
            entry = response.state.routes.add()
            entry.instr = instr
            entry.dut = dut
        return bytes(response.SerializeToString())

    @staticmethod
    def _error(code: int) -> bytes:
        response = _pb.Response()
        response.error.code = code
        return bytes(response.SerializeToString())

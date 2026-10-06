"""rdss_frame.py — Transporte del firmware lector CC1125, en Python.

Uso:      tx = build(CMD_PART_NUMBER)
          for f in Accumulator().feed(rx_bytes): ...
No hace:  no abre puertos ni habla con hardware. Eso es becker_ota_decoder.py.
No es:    el transporte del gateway-2lora ni el de TG. Éste es deliberadamente más simple.

FORMATO
-------
    [0x7E][MOD=0x07][DEV=0x01][CMD][LEN][payload...][0x7F]     total = 6 + LEN

Sin CRC y sin byte-stuffing, a propósito: el contrato está en
`fw-gateway2lora/docs/spec/cc1125-reader-frame.md`. Payload máximo 27 bytes.

POR QUÉ SE DELIMITA POR LONGITUD Y NUNCA POR EL 0x7F
----------------------------------------------------
El payload puede contener 0x7E y 0x7F — el checksum de la trama Becker son dos bytes
arbitrarios. Cortar por el terminador rompía el 11 % de las tramas en producción, y se
atribuyó durante semanas a pérdida de bytes en el enlace. Ver
`fw-gateway2lora/docs/runbooks/cc1125-perdida-bytes.md`.
"""

from __future__ import annotations

from typing import Iterator

FRAME_START: int = 0x7E
FRAME_END: int = 0x7F
MOD_BECKER: int = 0x07
DEV_NUCLEO: int = 0x01

HEADER_LEN: int = 5   # 7E MOD DEV CMD LEN
TRAILER_LEN: int = 1  # 7F
MAX_PAYLOAD: int = 27
FRAME_MAX_BYTES: int = HEADER_LEN + MAX_PAYLOAD + TRAILER_LEN  # 33

# Opcodes del firmware cc1125_reader_g070rb (Core/Src/main.cpp).
# La consola ASCII de una letra ('C', 'F', 'K', 'P', 'D') se retiró del firmware:
# cualquier byte que no sea 0x7E se descarta fuera de trama.
CMD_PART_NUMBER: int = 0x01   # -> [pn]                     (0x58 si el CC1125 vive)
CMD_BOOT_INFO: int = 0x02     # <- lo emite la placa al arrancar
CMD_STATUS: int = 0x03        # -> [pn][rssi][marc][freq(3)][tx_drops(4)]
CMD_CONFIGURE: int = 0x04     # reset + configurar Becker + RX
CMD_OTA_FRAME: int = 0x06     # <- trama Becker OTA sin pedirla (13 o 14 bytes)
CMD_IDLE: int = 0x07          # CC1125 a IDLE
CMD_CALIB_PULSE: int = 0x09   # -> [dur(2) BE] pulsa el boton de diagnostico del amp
CMD_UNKNOWN: int = 0x0B       # <- la placa no reconoció el opcode

CALIB_PULSE_DEFAULT_MS: int = 100    # igual que el firmware
CALIB_PULSE_MAX_MS: int = 2000       # el firmware lo acota ahi

OTA_LEN_STANDARD: int = 13
OTA_LEN_EXTENDED: int = 14


class FrameError(Exception):
    """La trama no cumple el contrato. El mensaje dice exactamente qué falló."""


def build(cmd: int, payload: bytes = b"") -> bytes:
    """Arma una trama de comando lista para enviar."""
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(
            f"payload de {len(payload)} bytes; el maximo del contrato es {MAX_PAYLOAD}"
        )
    return (bytes([FRAME_START, MOD_BECKER, DEV_NUCLEO, cmd, len(payload)])
            + payload + bytes([FRAME_END]))


def total_len(header: bytes) -> int | None:
    """Tamaño total de la trama según su cabecera, o None si aún no hay suficiente."""
    if len(header) < HEADER_LEN:
        return None
    return HEADER_LEN + header[4] + TRAILER_LEN


def parse(raw: bytes) -> dict:
    """Valida y descompone una trama recibida. Lanza FrameError con el motivo."""
    if len(raw) < HEADER_LEN + TRAILER_LEN:
        raise FrameError(f"trama de {len(raw)} bytes, minimo {HEADER_LEN + TRAILER_LEN}")
    if raw[0] != FRAME_START:
        raise FrameError(f"no empieza en 0x7E sino en 0x{raw[0]:02X}")
    if raw[1] != MOD_BECKER:
        raise FrameError(f"MOD 0x{raw[1]:02X}, esperaba 0x{MOD_BECKER:02X}")
    if raw[2] != DEV_NUCLEO:
        raise FrameError(f"DEV 0x{raw[2]:02X}, esperaba 0x{DEV_NUCLEO:02X}")
    esperado = total_len(raw)
    if len(raw) != esperado:
        raise FrameError(f"LEN dice {raw[4]} -> {esperado} bytes, llegaron {len(raw)}")
    if raw[-1] != FRAME_END:
        raise FrameError(f"no termina en 0x7F sino en 0x{raw[-1]:02X}")
    return {"cmd": raw[3], "payload": raw[HEADER_LEN:HEADER_LEN + raw[4]]}


class Accumulator:
    """Extrae tramas completas de un flujo de bytes que llega a trozos.

    Las tramas OTA llegan **sin pedirlas**, así que hay que acumular en vez de asumir
    una respuesta por comando. Se delimita por longitud; un 0x7E dentro del LEN es
    payload, no un arranque.
    """

    def __init__(self) -> None:
        self._buf = bytearray()
        self.resyncs: int = 0

    def feed(self, chunk: bytes) -> Iterator[dict]:
        """Consume bytes y entrega las tramas que hayan quedado completas."""
        self._buf += chunk
        while True:
            # descartar basura anterior al primer arranque
            inicio = self._buf.find(FRAME_START)
            if inicio < 0:
                self._buf.clear()
                return
            if inicio > 0:
                del self._buf[:inicio]

            total = total_len(bytes(self._buf))
            if total is None:
                return                      # falta cabecera
            if total > FRAME_MAX_BYTES:
                # LEN imposible: solo puede venir de un byte corrompido. Descartar el
                # arranque y buscar el siguiente, en vez de esperar bytes que no existen.
                self.resyncs += 1
                del self._buf[0]
                continue
            if len(self._buf) < total:
                return                      # falta payload
            crudo = bytes(self._buf[:total])
            try:
                trama = parse(crudo)
            except FrameError:
                self.resyncs += 1
                del self._buf[0]
                continue
            del self._buf[:total]
            yield trama

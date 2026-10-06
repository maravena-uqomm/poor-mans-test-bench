"""Las tramas que viajan por el aire entre la Rutherford y el VLAD25: sobre V2, telemetria y tono SmartRing.

Uso:  pedido = build_v2(dst=nodo, src=SRC_BANCO, cmd=0x3E)       # consulta dirigida a un equipo
      dst, src, cmd, datos = parse_v2(respuesta)                # levanta FrameError si no es V2
No hace: I/O. Es codec puro; lo mueve `vlad_lora_link.py` a traves de la Rutherford.

El sobre V2 es `DST[4] | SRC[4] | CMD[1] | DATA[N]`, direcciones en little-endian (el firmware las
copia con memcpy a un uint32 en Cortex-M: `LoraProtocol.cpp`, lora_protocol_dispatch). NO lleva
CRC: la integridad la da comparar cada respuesta con la misma consulta por USB, byte a byte. Y el
VLAD contesta en el mismo sobre, con DST = el SRC del pedido y SRC = su UID[0] crudo.

Por que V2 y no V1: decision del usuario (22-Sep-2026). Queda escrito que el puente del gateway
instalado NO reenvia V2, asi que esto prueba la radio, el camino de RF y el firmware, no el puente.

Fuente de cada constante: fw-vlad/vlad25_vhf/Core/Inc/Telemetry.hpp, Crc16Xmodem.hpp y
SmartRing.hpp.
"""

from __future__ import annotations

#: DST(4) + SRC(4) + CMD(1).
V2_CABECERA = 9
#: Datos maximos de una respuesta V2 por LoRa, desde fw-vlad `f71db99` (antes 54).
V2_DATOS_MAX = 63

#: Primer byte que el VLAD interpreta como trama V1 en vez de V2 (`frame[0] == 0x7E`). Un DST
#: cuyo byte bajo sea este se despacharia por el camino equivocado.
_INICIO_V1 = 0x7E

#: Telemetria: magic y largo por version (Telemetry.hpp).
TELEMETRY_MAGIC = 0x54
TELEMETRY_LEN_POR_VERSION = {5: 52, 6: 59}

#: El tono SmartRing del gateway, byte a byte (SmartRing.hpp y SmartRingTransmitter.hpp del
#: gateway). El VLAD exige coincidencia EXACTA, largo incluido. Es a su vez una trama V1 valida
#: con MF 0x00: el CRC XMODEM de los bytes 1..10 es 0xA58E.
TONO_SMARTRING = bytes([0x7E, 0x00, 0x00, 0x00, 0x00, 0x05, 0x55, 0x51, 0x4F, 0x4D, 0x4D, 0x8E, 0xA5, 0x7F])


class FrameError(ValueError):
    """La trama no es lo que dice ser: largo, direcciones o CRC."""


def crc16_xmodem(data: bytes) -> int:
    """CRC16-XMODEM: polinomio 0x1021, inicial 0, sin reflejar, sin xor final (Crc16Xmodem.hpp)."""
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc


def build_v2(dst: int, src: int, cmd: int, data: bytes = b"") -> bytes:
    """`DST | SRC | CMD | DATA`. Rechaza lo que el VLAD despacharia mal o descartaria."""
    if src == 0:
        # Con SRC 0 el VLAD no tiene a quien contestar: descarta la respuesta y cuenta
        # resp_descartada (LoraProtocol.cpp, lora_send_response).
        raise FrameError("SRC 0: el VLAD descarta la respuesta a un pedido sin remitente")
    if (dst & 0xFF) == _INICIO_V1:
        raise FrameError(f"DST {dst:#010x} empieza con 0x7E: el VLAD lo leeria como trama V1")
    return dst.to_bytes(4, "little") + src.to_bytes(4, "little") + bytes([cmd]) + data


def parse_v2(frame: bytes) -> tuple[int, int, int, bytes]:
    """Devuelve `(dst, src, cmd, data)`. No valida contenido: eso lo hace quien sabe que pidio."""
    if len(frame) < V2_CABECERA:
        raise FrameError(f"sobre V2 de {len(frame)} bytes: el minimo es {V2_CABECERA}")
    dst = int.from_bytes(frame[0:4], "little")
    src = int.from_bytes(frame[4:8], "little")
    return dst, src, frame[8], bytes(frame[V2_CABECERA:])


def is_telemetry_frame(frame: bytes) -> bool:
    """True si la trama es telemetria con el par (version, largo) conocido y el CRC cierra.

    El CRC cubre todo menos los dos ultimos bytes, magic incluido, y va little-endian al final.
    """
    if len(frame) < 2 or frame[0] != TELEMETRY_MAGIC:
        return False
    if TELEMETRY_LEN_POR_VERSION.get(frame[1]) != len(frame):
        return False
    return int.from_bytes(frame[-2:], "little") == crc16_xmodem(frame[:-2])


def telemetry_node16(frame: bytes) -> int:
    """`node16` de la trama: la direccion de 32 bits plegada por XOR (LoraContrato.hpp)."""
    return int.from_bytes(frame[2:4], "little")


def telemetry_seq(frame: bytes) -> int:
    """Numero de secuencia de la trama, que el VLAD incrementa en cada envio."""
    return frame[4]

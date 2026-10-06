"""Identidad y estado del VLAD25 por USB: serial, direccion de nodo, UID y el pin de modo.

Uso:  serial = leer_serial(vlad); escribir_serial(vlad, 36883)     # vlad = VladController conectado
      uid0 = uid0_de_identidad(consulta_cruda(vlad, CMD_IDENTIDAD, largo=LARGO_IDENTIDAD))
No hace: nada por LoRa. Es el lado USB contra el que se cruza lo que llega por el aire.

Vive aparte de `vlad_commands.py` porque ese archivo ya pasa las 500 lineas. Los formatos salen del
firmware (fw-vlad/vlad25_vhf/Core/Src/query_handlers.cpp y net_cmd.cpp), no de la documentacion.
"""

from __future__ import annotations

from typing import Protocol

#: 0x1F: 3 B LE para escribir, 0 B para leer (contesta 3 B). Rango util del serial.
CMD_SERIAL = 0x1F
SERIAL_MAX = 0xFFFFFE  # 0xFFFFFF es la EEPROM virgen y el firmware lo rechaza
#: 0x3E: `[addr u32 LE][origen]`, origen 1 = asignada en EEPROM, 0 = derivada del UID.
CMD_NODO = 0x3E
LARGO_NODO = 5
#: 0xA0: `[rol][caps][ver x3][UID 12 B tal cual en memoria]`. UID[0] son los bytes 5..8.
CMD_IDENTIDAD = 0xA0
LARGO_IDENTIDAD = 17
#: 0x05 QUERY_STATUS: `resp[0]` = ReversePin::read(), el PIN y no la variable.
CMD_ESTADO = 0x05
LARGO_ESTADO = 18
ACK = b"\x00"


class VladUsb(Protocol):
    """Lo que estas funciones necesitan del controlador. `VladController` lo cumple."""

    def _query_command(self, cmd: int, payload: bytes = ..., expected: int | None = ...) -> bytes: ...


def consulta_cruda(vlad: VladUsb, cmd: int, payload: bytes = b"", *, largo: int) -> bytes:
    """Los bytes de respuesta de `cmd` por USB, sin el eco del comando. b"" si fallo."""
    return vlad._query_command(cmd, payload, largo)


def leer_serial(vlad: VladUsb) -> int | None:
    """El serial de la unidad. 0 significa "sin asignar" O "fallo la lectura": el firmware no los separa."""
    r = consulta_cruda(vlad, CMD_SERIAL, largo=3)
    return int.from_bytes(r, "little") if len(r) == 3 else None


def escribir_serial(vlad: VladUsb, serial: int) -> bool:
    """Escribe el serial (0x1F siempre sobrescribe). True solo con ACK; quien llama relee."""
    if not 1 <= serial <= SERIAL_MAX:
        raise ValueError(f"serial {serial} fuera de 1..{SERIAL_MAX}: el campo es de 24 bits")
    return consulta_cruda(vlad, CMD_SERIAL, serial.to_bytes(3, "little"), largo=1) == ACK


def leer_nodo(vlad: VladUsb) -> tuple[int, bool] | None:
    """`(direccion, asignada)`: la DST a la que el VLAD contesta por LoRa."""
    r = consulta_cruda(vlad, CMD_NODO, largo=LARGO_NODO)
    if len(r) != LARGO_NODO:
        return None
    return int.from_bytes(r[0:4], "little"), r[4] == 1


def uid0_de_identidad(identidad: bytes) -> int | None:
    """UID[0] desde la respuesta de 0xA0. Es el SRC con que el VLAD firma sus respuestas V2."""
    if len(identidad) != LARGO_IDENTIDAD:
        return None
    return int.from_bytes(identidad[5:9], "little")


def pin_reverse(vlad: VladUsb) -> bool | None:
    """True si el pin de modo esta en REVERSE, False en CONVENCIONAL, None si no se pudo leer."""
    r = consulta_cruda(vlad, CMD_ESTADO, largo=LARGO_ESTADO)
    return (r[0] == 0x01) if len(r) == LARGO_ESTADO else None

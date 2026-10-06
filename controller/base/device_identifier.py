"""Motor de identificacion por Desafio-Respuesta (Challenge-Response) para instrumentos.

Permite verificar si un puerto (/dev/ttyACM*, /dev/usbtmc*, /dev/serial/by-id/*) corresponde
realmente al instrumento esperado interrogandolo con un comando inocuo de su protocolo propio
y validando la firma exacta de su respuesta.

Evita:
- Hablarle con el protocolo equivocado a un dispositivo (ej. escribir protobuf a un VLAD).
- Caer en simulaciones silenciosas o puertos muertos re-enumerados por el kernel.
- Desambiguar dispositivos con VID:PID identico (Rutherford vs VLAD25, ambos 0483:5740).
- Leer la version de firmware del VLAD25 antes de iniciar pruebas.
"""

from __future__ import annotations

import contextlib
import glob
import logging
import os
import select
import struct
import time
from dataclasses import dataclass
from typing import Any

try:
    import serial
except ImportError:
    serial = None

from controller.uqomm import vlad25_protocol as vlad_proto
from controller.vsg.vsg_protocol import FRAME_SIZE, PacketBuilder, parse_response

LOGGER = logging.getLogger("device_identifier")

#: Comando binario GetStateCmd para Rutherford (6 bytes prefijados con longitud LE/BE)
RUTHERFORD_GET_STATE_FRAME = bytes.fromhex("00040a022200")
RUTHERFORD_TAG_STATE = 0x12
RUTHERFORD_TAG_ACK = 0x0A


@dataclass(frozen=True)
class ProbeResult:
    matched: bool
    device_path: str
    instrument_type: str
    identity: str | None = None
    error: str | None = None
    details: dict[str, Any] | None = None


class DeviceIdentifier:
    """Sondas de desafio-respuesta y auto-descubrimiento para el banco de pruebas."""

    @staticmethod
    def probe_rutherford(device_path: str, timeout_s: float = 0.5) -> ProbeResult:
        """Envia GetStateCmd y verifica respuesta protobuf valida de Rutherford."""
        if not os.path.exists(device_path):
            return ProbeResult(
                matched=False,
                device_path=device_path,
                instrument_type="rutherford",
                error="device_not_found",
            )
        try:
            fd = os.open(device_path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        except OSError as exc:
            return ProbeResult(
                matched=False,
                device_path=device_path,
                instrument_type="rutherford",
                error=f"open_failed: {exc}",
            )

        try:
            # Enviar GetStateCmd
            os.write(fd, RUTHERFORD_GET_STATE_FRAME)
            deadline = time.monotonic() + timeout_s
            raw_hdr = bytearray()
            while len(raw_hdr) < 2:
                rem = deadline - time.monotonic()
                if rem <= 0:
                    return ProbeResult(
                        matched=False,
                        device_path=device_path,
                        instrument_type="rutherford",
                        error="timeout_waiting_header",
                    )
                r, _, _ = select.select([fd], [], [], rem)
                if not r:
                    continue
                chunk = os.read(fd, 2 - len(raw_hdr))
                if chunk:
                    raw_hdr.extend(chunk)

            payload_len = struct.unpack(">H", bytes(raw_hdr))[0]
            if payload_len == 0 or payload_len > 512:
                return ProbeResult(
                    matched=False,
                    device_path=device_path,
                    instrument_type="rutherford",
                    error=f"invalid_payload_length: {payload_len}",
                )

            payload = bytearray()
            while len(payload) < payload_len:
                rem = deadline - time.monotonic()
                if rem <= 0:
                    return ProbeResult(
                        matched=False,
                        device_path=device_path,
                        instrument_type="rutherford",
                        error="timeout_waiting_payload",
                    )
                r, _, _ = select.select([fd], [], [], rem)
                if not r:
                    continue
                chunk = os.read(fd, payload_len - len(payload))
                if chunk:
                    payload.extend(chunk)

            tag = payload[0]
            if tag in (RUTHERFORD_TAG_STATE, RUTHERFORD_TAG_ACK):
                return ProbeResult(
                    matched=True,
                    device_path=device_path,
                    instrument_type="rutherford",
                    identity=f"Rutherford (tag=0x{tag:02X}, len={payload_len})",
                    details={"tag": tag, "payload_len": payload_len},
                )
            return ProbeResult(
                matched=False,
                device_path=device_path,
                instrument_type="rutherford",
                error=f"unexpected_tag_0x{tag:02X}",
            )
        except Exception as exc:
            return ProbeResult(
                matched=False,
                device_path=device_path,
                instrument_type="rutherford",
                error=f"probe_exception: {exc}",
            )
        finally:
            with contextlib.suppress(OSError):
                os.close(fd)

    @staticmethod
    def probe_vlad(device_path: str, timeout_s: float = 0.6) -> ProbeResult:
        """Envia FOTA_STATUS (0x1C) o QUERY_STATUS (0x05) al VLAD y extrae version de firmware."""
        if not os.path.exists(device_path):
            return ProbeResult(
                matched=False,
                device_path=device_path,
                instrument_type="vlad",
                error="device_not_found",
            )
        if serial is None:
            return ProbeResult(
                matched=False,
                device_path=device_path,
                instrument_type="vlad",
                error="pyserial_unavailable",
            )

        try:
            ser = serial.Serial(device_path, baudrate=115200, timeout=timeout_s)
        except (OSError, serial.SerialException) as exc:
            return ProbeResult(
                matched=False,
                device_path=device_path,
                instrument_type="vlad",
                error=f"serial_open_failed: {exc}",
            )

        try:
            ser.reset_input_buffer()
            ser.reset_output_buffer()

            # Intentar FOTA_STATUS (0x1C)
            cmd = vlad_proto.CMD_FOTA_STATUS
            expected_payload_min = vlad_proto.RESP_LEN[cmd]
            ser.write(bytes([cmd]))

            # Sincronizar descartando logs hasta eco de 0x1C
            t0 = time.monotonic()
            descartados = 0
            eco_encontrado = False
            while time.monotonic() - t0 < timeout_s and descartados < 4096:
                b = ser.read(1)
                if not b:
                    break
                if b[0] == cmd:
                    eco_encontrado = True
                    break
                descartados += 1

            if eco_encontrado:
                payload = ser.read(expected_payload_min)
                if len(payload) >= expected_payload_min:
                    # Leer bytes adicionales si los hay (hasta 17 bytes de FOTA_STATUS completo)
                    extra = ser.read(8)
                    total_payload = payload + extra
                    fota_info = vlad_proto.decode_fota_status(total_payload)
                    return ProbeResult(
                        matched=True,
                        device_path=device_path,
                        instrument_type="vlad",
                        identity=f"VLAD25 FW {fota_info['version_str']}",
                        details=fota_info,
                    )

            # Fallback a DIAGNOSTIC_RUN (0x35)
            ser.reset_input_buffer()
            cmd_diag = vlad_proto.CMD_DIAGNOSTIC_RUN
            ser.write(bytes([cmd_diag]))
            t0 = time.monotonic()
            while time.monotonic() - t0 < timeout_s:
                b = ser.read(1)
                if not b:
                    break
                if b[0] == cmd_diag:
                    payload = ser.read(2)
                    if len(payload) == 2:
                        health = vlad_proto.decode_health_word(payload)
                        return ProbeResult(
                            matched=True,
                            device_path=device_path,
                            instrument_type="vlad",
                            identity=f"VLAD25 (health_word=0x{health:04X})",
                            details={"health_word": health},
                        )

            return ProbeResult(
                matched=False,
                device_path=device_path,
                instrument_type="vlad",
                error="no_valid_vlad_response",
            )
        except Exception as exc:
            return ProbeResult(
                matched=False,
                device_path=device_path,
                instrument_type="vlad",
                error=f"probe_exception: {exc}",
            )
        finally:
            with contextlib.suppress(Exception):
                ser.close()

    @staticmethod
    def probe_vsg_serial(device_path: str, timeout_s: float = 0.5) -> ProbeResult:
        """Envia paquete OPEN de VSG25A y valida respuesta de sincronizacion."""
        if not os.path.exists(device_path):
            return ProbeResult(
                matched=False,
                device_path=device_path,
                instrument_type="tone_generator",
                error="device_not_found",
            )
        if serial is None:
            return ProbeResult(
                matched=False,
                device_path=device_path,
                instrument_type="tone_generator",
                error="pyserial_unavailable",
            )

        try:
            ser = serial.Serial(device_path, baudrate=115200, timeout=timeout_s)
        except (OSError, serial.SerialException) as exc:
            return ProbeResult(
                matched=False,
                device_path=device_path,
                instrument_type="tone_generator",
                error=f"serial_open_failed: {exc}",
            )

        try:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            builder = PacketBuilder(initial_seq=0x00)
            open_pkt = builder.build_open()
            ser.write(open_pkt)
            resp = ser.read(FRAME_SIZE)
            if len(resp) == FRAME_SIZE:
                parsed = parse_response(resp)
                if parsed.get("valid"):
                    return ProbeResult(
                        matched=True,
                        device_path=device_path,
                        instrument_type="tone_generator",
                        identity="Signal Hound VSG25A (Serial)",
                        details=parsed,
                    )
            return ProbeResult(
                matched=False,
                device_path=device_path,
                instrument_type="tone_generator",
                error="invalid_vsg_frame_response",
            )
        except Exception as exc:
            return ProbeResult(
                matched=False,
                device_path=device_path,
                instrument_type="tone_generator",
                error=f"probe_exception: {exc}",
            )
        finally:
            with contextlib.suppress(Exception):
                ser.close()

    @staticmethod
    def probe_owon_usbtmc(device_path: str, timeout_s: float = 1.0) -> ProbeResult:
        """Envia *CLS y *IDN? al analizador USB-TMC y valida prefijo OWON."""
        if not os.path.exists(device_path):
            return ProbeResult(
                matched=False,
                device_path=device_path,
                instrument_type="analyzer",
                error="device_not_found",
            )
        try:
            fd = os.open(device_path, os.O_RDWR)
        except OSError as exc:
            return ProbeResult(
                matched=False,
                device_path=device_path,
                instrument_type="analyzer",
                error=f"usbtmc_open_failed: {exc}",
            )

        try:
            # Enviar *CLS y luego *IDN?
            os.write(fd, b"*CLS\n")
            time.sleep(0.05)
            os.write(fd, b"*IDN?\n")

            deadline = time.monotonic() + timeout_s
            raw = bytearray()
            while time.monotonic() < deadline:
                r, _, _ = select.select([fd], [], [], 0.1)
                if r:
                    chunk = os.read(fd, 256)
                    if chunk:
                        raw.extend(chunk)
                        if b"\n" in raw:
                            break
                else:
                    time.sleep(0.02)

            resp = raw.decode("ascii", errors="replace").strip()
            if resp.upper().startswith("OWON") or "VSA815" in resp.upper():
                return ProbeResult(
                    matched=True,
                    device_path=device_path,
                    instrument_type="analyzer",
                    identity=resp,
                    details={"idn": resp},
                )
            return ProbeResult(
                matched=False,
                device_path=device_path,
                instrument_type="analyzer",
                error=f"unexpected_idn_response: {resp!r}",
            )
        except Exception as exc:
            return ProbeResult(
                matched=False,
                device_path=device_path,
                instrument_type="analyzer",
                error=f"probe_exception: {exc}",
            )
        finally:
            with contextlib.suppress(OSError):
                os.close(fd)

    @classmethod
    def probe_device(cls, role: str, driver_type: str, candidate_path: str) -> ProbeResult:
        """Despacha la sonda correspondiente segun el rol y tipo de driver."""
        role_norm = role.lower()
        type_norm = driver_type.lower()

        if role_norm in ("rutterford", "crfs") or type_norm == "uqomm":
            return cls.probe_rutherford(candidate_path)
        if role_norm == "vlad" or "vlad" in type_norm:
            return cls.probe_vlad(candidate_path)
        if role_norm == "tone_generator" and "vsg" in type_norm:
            return cls.probe_vsg_serial(candidate_path)
        if role_norm == "analyzer" and "owon" in type_norm:
            return cls.probe_owon_usbtmc(candidate_path)

        return ProbeResult(
            matched=False,
            device_path=candidate_path,
            instrument_type=role,
            error=f"no_probe_defined_for_{role}_{driver_type}",
        )

    @classmethod
    def discover_device(
        cls,
        role: str,
        driver_type: str,
        preferred_path: str | None = None,
    ) -> str | None:
        """Escanea puertos del sistema buscando el que responda al desafio del instrumento.

        Si preferred_path esta disponible y responde, lo retorna de inmediato.
        Si no, busca en /dev/ttyACM*, /dev/usbtmc*, /dev/serial/by-id/* y retorna el primer match.
        """
        if preferred_path and os.path.exists(preferred_path):
            res = cls.probe_device(role, driver_type, preferred_path)
            if res.matched:
                LOGGER.info("discover_device: %s (%s) confirmado en ruta preferida %s", role, driver_type, preferred_path)
                return preferred_path

        # Listar candidatos segun el rol
        candidatos: list[str] = []
        if "usb" in driver_type or "owon" in driver_type and role == "analyzer":
            candidatos.extend(sorted(glob.glob("/dev/usbtmc*")))
        if "serial" in driver_type or role in ("rutterford", "crfs", "vlad", "tone_generator"):
            candidatos.extend(sorted(glob.glob("/dev/ttyACM*")))
            candidatos.extend(sorted(glob.glob("/dev/ttyUSB*")))
            candidatos.extend(sorted(glob.glob("/dev/serial/by-id/*")))

        # Evitar re-probar la preferida si ya fallo
        candidatos = [c for c in candidatos if c != preferred_path]

        for cand in candidatos:
            try:
                res = cls.probe_device(role, driver_type, cand)
                if res.matched:
                    LOGGER.info(
                        "discover_device: %s (%s) re-descubierto en %s (identidad: %s)",
                        role,
                        driver_type,
                        cand,
                        res.identity,
                    )
                    return cand
            except Exception as exc:
                LOGGER.debug("discover_device probe error on %s: %s", cand, exc)

        LOGGER.warning("discover_device: no se encontro ningun puerto que responda como %s (%s)", role, driver_type)
        return None

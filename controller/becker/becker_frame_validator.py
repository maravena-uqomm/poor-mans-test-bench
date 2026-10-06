"""becker_frame_validator.py — Pure validation logic for Becker OTA frames.

Performs structural validation of Becker Varis OTA frames (spec §2)
without any hardware or serial dependencies. All checks are pure functions
operating on raw bytes. Suitable for unit testing without mocks.

Usage:
    result = validate_frame(raw_bytes)
    if result.passed:
        print(f"Amp ID: {result.amp_id_str}, Voltage: {result.voltage_v}V")
    else:
        print(f"Errors: {result.errors}")
"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field

# ── Protocol constants (spec §2) ────────────────────────────────────────

SOF: int = 0x55
DELIMITER: int = 0xFF
CMD_DIAG: int = 0x66
FRAME_LEN_STD: int = 13
FRAME_LEN_EXT: int = 14
VOLTAGE_SCALE: float = 0.15

VOLTAGE_MIN: float = 5.0
VOLTAGE_MAX: float = 40.0


@dataclass
class FrameValidationResult:
    """Result of validating a single Becker OTA frame.

    Attributes:
        raw_hex: Hex representation of the raw bytes.
        passed: True if all structural checks pass.
        errors: Empty if passed, otherwise list of error messages.
        amp_id_str: Dotted amplifier ID (e.g. "255.150.124").
        amp_id_int: Integer amplifier ID.
        voltage_v: Decoded voltage in volts.
        agc_raw: Raw AGC value (byte[6]).
        mode: "AUTO" or "MANUAL".
        atten_value: Attenuation / switch position (byte[8] & 0x0F).
        mirror_nibble: Mirror byte (byte[9]).
        mirror_valid: True if byte[9] == (byte[8] & 0x0F) * 0x11.
        is_extended: True if 14-byte extended frame.
        alarm_byte: Alarm byte value (byte[13]), None if standard frame.
        checksum: 16-bit checksum value (byte[11:12]).
        checksum_valid: Always True (algorithm unresolved — see docs).
    """

    raw_hex: str
    passed: bool
    errors: list[str] = field(default_factory=list)

    amp_id_str: str = ""
    amp_id_int: int = 0
    voltage_v: float = 0.0
    agc_raw: int = 0
    mode: str = ""
    atten_value: int = 0
    mirror_nibble: int = 0
    mirror_valid: bool = False
    is_extended: bool = False
    alarm_byte: int | None = None
    checksum: int = 0
    checksum_valid: bool = True


def validate_frame(raw: bytes) -> FrameValidationResult:
    """Validate a Becker OTA frame against the spec (§2).

    Args:
        raw: 13-byte (standard) or 14-byte (extended) frame bytes.

    Returns:
        FrameValidationResult with all decoded fields and validation status.
    """
    errors: list[str] = []
    is_extended = len(raw) == FRAME_LEN_EXT

    # ── Length check ────────────────────────────────────────────────────
    if len(raw) not in (FRAME_LEN_STD, FRAME_LEN_EXT):
        return FrameValidationResult(
            raw_hex=raw.hex().upper(),
            passed=False,
            errors=[f"Invalid length: {len(raw)} bytes (must be 13 or 14)"],
        )

    # ── Header check ────────────────────────────────────────────────────
    if raw[0] != SOF:
        errors.append(f"Invalid SOF: 0x{raw[0]:02X} (expected 0x{SOF:02X})")
    if raw[1] != DELIMITER:
        errors.append(f"Invalid delimiter: 0x{raw[1]:02X} (expected 0x{DELIMITER:02X})")
    if raw[2] != CMD_DIAG:
        errors.append(f"Invalid command: 0x{raw[2]:02X} (expected 0x{CMD_DIAG:02X})")

    # ── Amp ID ──────────────────────────────────────────────────────────
    amp_id_int = (raw[3] << 16) | (raw[4] << 8) | raw[5]
    amp_id_str = f"{raw[3]}.{raw[4]}.{raw[5]}"

    # ── AGC ─────────────────────────────────────────────────────────────
    agc_raw = raw[6]

    # ── Voltage ─────────────────────────────────────────────────────────
    voltage_raw = raw[7]
    voltage_v = round(voltage_raw * VOLTAGE_SCALE, 1)
    if voltage_v < VOLTAGE_MIN or voltage_v > VOLTAGE_MAX:
        errors.append(
            f"Voltage {voltage_v}V (raw=0x{voltage_raw:02X}) outside "
            f"expected range [{VOLTAGE_MIN}, {VOLTAGE_MAX}] V"
        )

    # ── Mode / atten ────────────────────────────────────────────────────
    mode_byte = raw[8]
    mode = "AUTO" if (mode_byte & 0x10) else "MANUAL"
    atten_value = mode_byte & 0x0F

    # ── Mirror byte (byte[9]) ───────────────────────────────────────────
    mirror_nibble = raw[9]
    expected_mirror = (mode_byte & 0x0F) * 0x11
    mirror_valid = mirror_nibble == expected_mirror
    if not mirror_valid:
        errors.append(
            f"Invalid mirror byte: 0x{mirror_nibble:02X} "
            f"(expected 0x{expected_mirror:02X} = (byte[8] & 0x0F) * 0x11)"
        )

    # ── Reserved byte (byte[10]) ────────────────────────────────────────
    if raw[10] != 0x00:
        errors.append(f"Reserved byte not zero: 0x{raw[10]:02X} (expected 0x00)")

    # ── Checksum ────────────────────────────────────────────────────────
    checksum = (raw[11] << 8) | raw[12]

    # ── Extended frame ──────────────────────────────────────────────────
    alarm_byte: int | None = raw[13] if is_extended else None

    return FrameValidationResult(
        raw_hex=raw.hex().upper(),
        passed=len(errors) == 0,
        errors=errors,
        amp_id_str=amp_id_str,
        amp_id_int=amp_id_int,
        voltage_v=voltage_v,
        agc_raw=agc_raw,
        mode=mode,
        atten_value=atten_value,
        mirror_nibble=mirror_nibble,
        mirror_valid=mirror_valid,
        is_extended=is_extended,
        alarm_byte=alarm_byte,
        checksum=checksum,
        checksum_valid=True,
    )


def validate_frame_batch(raw_frames: list[bytes]) -> list[FrameValidationResult]:
    """Validate multiple frames. Skips empty/invalid-length frames."""
    return [validate_frame(r) for r in raw_frames if len(r) >= 2]

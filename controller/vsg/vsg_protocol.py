"""
VSG25A serial binary protocol decoder/builder.

Packet structure (decoded from USB bulk captures in assets/vsg25_cmds.txt):
  Byte 0:     Payload length (L) — determines command type
  Byte 1:     Sequence number (monotonic, wraps at 0xff→0x00)
  Bytes 2-3:  Flags  (observed: always 0x01 0x00)
  Bytes 4..4+L-1:  Payload  (L bytes, varies by command)
  Bytes 4+L..62:   Zero padding  (fixed 63-byte frame)
  Byte 63:    Checksum  (sum bytes 0..62 mod 256)

Discovered command types by payload length:
  L=0x02  ( 2 B): OPEN / CLOSE
  L=0x03  ( 3 B): Short command / trigger (SET_CW, RF_OFF, EXECUTE)
  L=0x0c  (12 B): Parameterised command (SET_FREQ_AMP, CONFIGURE)
  L=0x10  (16 B): Device response (always host←device)
  L=0x17  (23 B): IQ data stream (host←device)

Typical flow:
  1. send OPEN  (L=0x02)   → recv RESPONSE  (L=0x10)
  2. send CONFIGURE (L=0x0c) → recv RESPONSE (L=0x10)
  3. send EXECUTE (L=0x03) → recv RESPONSE  (L=0x10)
  4. send CLOSE  (L=0x02)

Jira: ID-1308 (Test Bench v2)
"""

from __future__ import annotations

import struct
import threading
from typing import Any, Optional


# ---------------------------------------------------------------------------
# Packet constants
# ---------------------------------------------------------------------------

# Payload lengths (command types)
L_OPEN_CLOSE = 0x02
L_SHORT_CMD = 0x03
L_PARAM_CMD = 0x0c
L_RESPONSE = 0x10
L_IQ_DATA = 0x17

# Short command operation codes (first byte of L=0x03 payload)
OP_RF_OFF = 0x00
OP_SET_CW = 0x01
OP_EXECUTE = 0x00  # alias of OP_RF_OFF: generic "apply cached config" (same wire value)

# Total fixed-size frame
FRAME_SIZE = 64  # bytes (USB bulk-packet sized)
DATA_SIZE = 63   # bytes before checksum
HEADER_SIZE = 4  # bytes: [L, seq, flags_hi, flags_lo]

# Flags observed in captures
FLAGS_HI = 0x01
FLAGS_LO = 0x00


# ---------------------------------------------------------------------------
# Packet builder
# ---------------------------------------------------------------------------

class PacketBuilder:
    """Builds 64-byte VSG25 protocol packets with correct checksum."""

    def __init__(self, initial_seq: int = 0x00):
        self._seq = initial_seq & 0xFF
        self._lock = threading.Lock()

    @property
    def seq(self) -> int:
        return self._seq

    def next_seq(self) -> int:
        with self._lock:
            s = self._seq
            self._seq = (self._seq + 1) & 0xFF
            return s

    def build(self, payload_len: int, payload: bytes = b"", seq: Optional[int] = None) -> bytes:
        """Build a 64-byte protocol packet.

        Args:
            payload_len: Byte 0 — command type / payload length.
            payload: Payload bytes (must be <= payload_len, zero-padded).
            seq: Sequence number (auto-increment if None).

        Returns:
            64-byte packet ready to send.
        """
        if len(payload) > payload_len:
            msg = f"payload {len(payload)} > declared length {payload_len}"
            raise ValueError(msg)
        if seq is not None:
            _seq = seq & 0xFF
            self._seq = (_seq + 1) & 0xFF
        else:
            _seq = self.next_seq()

        frame = bytearray(FRAME_SIZE)
        frame[0] = payload_len
        frame[1] = _seq
        frame[2] = FLAGS_HI
        frame[3] = FLAGS_LO
        frame[4:4 + len(payload)] = payload
        # rest is already zero from bytearray init

        checksum = sum(frame[:DATA_SIZE]) & 0xFF
        frame[DATA_SIZE] = checksum

        return bytes(frame)

    def build_open(self) -> bytes:
        # OPEN and CLOSE share the same frame payload; the device distinguishes
        # them by command sequence/context, not by the frame contents.
        return self.build(L_OPEN_CLOSE, b"\x00\x00")

    def build_close(self) -> bytes:
        return self.build(L_OPEN_CLOSE, b"\x00\x00")

    def build_short_cmd(self, opcode: int = OP_EXECUTE) -> bytes:
        """Build a 3-byte payload short command (SET_CW, RF_OFF, etc)."""
        return self.build(L_SHORT_CMD, bytes([opcode & 0xFF, 0x00, 0x00]))

    def build_param_cmd(self, param_bytes: bytes) -> bytes:
        """Build a 12-byte payload parameter command.

        Args:
            param_bytes: Payload (must be <= 12 bytes, zero-padded).

        Returns:
            64-byte packet.
        """
        return self.build(L_PARAM_CMD, param_bytes)


# ---------------------------------------------------------------------------
# Frequency / Amplitude encoding helpers
# ---------------------------------------------------------------------------

def encode_freq_amp(freq_hz: float, amp_dbm: float) -> bytes:
    """Encode frequency + amplitude into 12-byte payload.

    Encoding (best-guess from hex-dump analysis):
      bytes 0-7:  frequency as float64 LE  (matches C double)
      bytes 8-11: amplitude as float32 LE  (truncated from double)

    Returns:
        12 bytes.
    """
    return struct.pack("<df", freq_hz, amp_dbm)


def decode_freq_amp(payload: bytes) -> tuple[float, float]:
    """Decode 12-byte freq/amp payload back to Hz and dBm."""
    freq, amp = struct.unpack_from("<df", payload)
    return freq, amp


# ---------------------------------------------------------------------------
# Response parsing
# ---------------------------------------------------------------------------

def parse_response(packet: bytes) -> dict[str, Any]:
    """Parse a received 64-byte packet into structured fields.

    Returns:
        dict with keys: payload_len, seq, flags, payload, checksum, valid
    """
    result = {
        "payload_len": packet[0],
        "seq": packet[1],
        "flags": (packet[2], packet[3]),
        "payload": b"",
        "checksum": packet[DATA_SIZE],
        "valid": False,
    }
    plen = packet[0]
    result["payload"] = packet[HEADER_SIZE:HEADER_SIZE + plen]
    computed = sum(packet[:DATA_SIZE]) & 0xFF
    result["valid"] = computed == packet[DATA_SIZE]
    return result

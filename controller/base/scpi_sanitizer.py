"""
SCPI command sanitizer.

Pure function module — no I/O, no side effects, no logging dependencies.
Easy to unit-test and reuse across every transport (USB-TMC, TCP/IP, Serial).

Why we sanitize BEFORE writing to the wire:
    - Budget instruments (Owon VSA815P) freeze their USB-TMC endpoint when
      they receive malformed SCPI (embedded NUL bytes, very long strings,
      bare CR without LF). Recovery requires a USB hard reset.
    - Embedded NULs and stray control bytes corrupt the parser state for
      subsequent commands, causing silent measurement errors much later.
    - Length limits prevent runaway f-strings (e.g. an unbounded list
      formatted into a single command) from saturating the device buffer.

Contract:
    sanitize(cmd) -> SanitizedCommand
        ok=True  -> safe to write; .command holds the normalized form.
        ok=False -> .reason explains why; caller MUST NOT write to the wire.

The sanitizer NEVER raises for normal validation failures — that is what
the .ok flag is for. It only raises TypeError for programmer errors
(passing a non-string).

Jira: ID-1308 (Test Bench v2 — robustness hardening)
"""

from __future__ import annotations

from dataclasses import dataclass


# Maximum length of a single SCPI command line. Standard SCPI commands are
# well under 100 chars; 256 leaves comfortable headroom for batched
# parameter lists while still catching pathological f-string blow-ups.
DEFAULT_MAX_LENGTH = 256

# Control characters allowed inside a SCPI command (none — terminators are
# added by the transport layer, not embedded). Tab is occasionally used as
# a separator by some vendors, so we tolerate it.
_ALLOWED_CONTROL = frozenset({"\t"})


@dataclass(frozen=True)
class SanitizedCommand:
    """Result of sanitizing a SCPI command.

    Attributes:
        ok: True if the command is safe to write to the wire.
        command: Normalized command string when ok is True; None otherwise.
        reason: Human-readable rejection reason when ok is False.
    """

    ok: bool
    command: str | None
    reason: str

    @classmethod
    def accept(cls, command: str) -> "SanitizedCommand":
        return cls(ok=True, command=command, reason="")

    @classmethod
    def reject(cls, reason: str) -> "SanitizedCommand":
        return cls(ok=False, command=None, reason=reason)


def sanitize(
    raw: str,
    *,
    max_length: int = DEFAULT_MAX_LENGTH,
) -> SanitizedCommand:
    """Validate and normalize a single SCPI command.

    Steps:
        1. Type-check (must be str).
        2. Strip leading/trailing whitespace and trailing terminators
           (`\\r`, `\\n`, mix). The transport layer always appends `\\n`
           itself, so any embedded terminator is a programmer mistake.
        3. Reject empty strings.
        4. Reject embedded NUL bytes.
        5. Reject any non-ASCII byte (SCPI is strictly 7-bit ASCII).
        6. Reject embedded control chars (allowed: TAB).
        7. Reject CR or LF in the middle of the string (terminators only
           belong at the very end, and we already stripped them).
        8. Reject strings exceeding max_length.
        9. Reject queries with multiple `?` characters (malformed).

    Args:
        raw: The candidate SCPI command. May contain trailing whitespace.
        max_length: Maximum length after normalization. Default 256.

    Returns:
        SanitizedCommand with .ok set accordingly.

    Raises:
        TypeError: If raw is not a string. This indicates a programming
            error in the caller, not a runtime input issue.
    """
    if not isinstance(raw, str):
        raise TypeError(
            f"SCPI command must be str, got {type(raw).__name__}"
        )

    # Step 2 — strip whitespace and trailing terminators in any order.
    cmd = raw.strip()

    # Step 3 — empty after normalization.
    if not cmd:
        return SanitizedCommand.reject("empty command")

    # Step 4 — embedded NULs corrupt the SCPI parser state.
    if "\x00" in cmd:
        return SanitizedCommand.reject("embedded NUL byte")

    # Step 5 — non-ASCII (SCPI is strictly 7-bit).
    try:
        cmd.encode("ascii")
    except UnicodeEncodeError as exc:
        return SanitizedCommand.reject(
            f"non-ASCII character at position {exc.start}: {exc.reason}"
        )

    # Step 6 + 7 — control characters and embedded terminators.
    for idx, ch in enumerate(cmd):
        if ch in _ALLOWED_CONTROL:
            continue
        code = ord(ch)
        if code < 0x20 or code == 0x7F:
            return SanitizedCommand.reject(
                f"control character 0x{code:02X} at position {idx}"
            )

    # Step 8 — length cap.
    if len(cmd) > max_length:
        return SanitizedCommand.reject(
            f"command exceeds max_length ({len(cmd)} > {max_length})"
        )

    # Step 9 — malformed query (multiple '?').
    if cmd.count("?") > 1:
        return SanitizedCommand.reject(
            "multiple '?' characters (malformed query)"
        )

    return SanitizedCommand.accept(cmd)


def is_query(command: str) -> bool:
    """Return True if the command expects a response (contains '?').

    Assumes the input has already been sanitized — does not validate.
    """
    return "?" in command

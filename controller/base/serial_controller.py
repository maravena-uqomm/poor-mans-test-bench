"""
Base controller for serial instruments (pyserial) with retry + reconnect.

Incluye:
- Reintentos automáticos con backoff (3 intentos)
- Auto-reconexión del puerto serie tras fallo
- Circuit breaker por instrumento (3 fallos → 15s de pausa)
- Lock de thread (RLock) para serializar I/O concurrente

Jira: ID-1308 (Test Bench v2)
"""
from __future__ import annotations

import contextlib
import logging
import threading
import time

try:
    import serial
except ImportError:
    serial = None  # type: ignore[assignment]  # fallback when pyserial not installed

from controller.base.circuit_breaker import CircuitBreaker
from controller.base.reliability import InstrumentState, RetryPolicy
from controller.base.scpi_sanitizer import sanitize

class SerialController:
    """Base controller for serial / CDC-ACM instruments with retry + reconnect.

    Args:
        retries: Número de reintentos por operación (default 3).
        reconnect_delay: Segundos entre cierre y reapertura del puerto (default 0.5).
        circuit_breaker: Instancia compartida de CircuitBreaker (opcional).
    """

    _terminator = "\n"

    def __init__(
        self,
        retries: int = 3,
        reconnect_delay: float = 0.5,
        circuit_breaker: CircuitBreaker | None = None,
    ):
        if serial is None:
            raise ImportError("pyserial is not installed — SerialController requires pyserial to operate")
        self._serial: serial.Serial | None = None
        self._device_path: str | None = None
        self._baudrate: int = 115200
        self._timeout: float = 2.0
        self._retries = retries
        self._reconnect_delay = reconnect_delay
        self._breaker = circuit_breaker or CircuitBreaker(
            self.__class__.__name__, threshold=3, cooldown=15.0
        )
        self._lock = threading.RLock()
        self._state = InstrumentState.DISCONNECTED
        self._name = self.__class__.__name__
        self.retry_policy = RetryPolicy(max_retries=retries, base_delay=reconnect_delay)
        self.logger = logging.getLogger(self._name)

    # ------------------------------------------------------------------
    # Connection management
    # ------------------------------------------------------------------

    @property
    def is_connected(self) -> bool:
        return self._serial is not None and self._serial.is_open

    @property
    def state(self) -> InstrumentState:
        return self._state

    def connect(
        self,
        device_path: str,
        baudrate: int = 115200,
        timeout: float = 2.0,
    ) -> None:
        """Open the serial port. Retries up to ``retries`` times.

        Raises:
            ConnectionError: If the port cannot be opened after retries.
        """
        if self._serial is not None and self._serial.is_open:
            self.logger.warning(
                "Already connected to %s — disconnecting first", self._device_path
            )
            self.disconnect()

        self._device_path = device_path
        self._baudrate = baudrate
        self._timeout = timeout

        last_error = None
        for attempt in range(1, self._retries + 1):
            try:
                self._serial = serial.Serial(
                    port=device_path,
                    baudrate=baudrate,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=timeout,
                )
                self._state = InstrumentState.CONNECTED
                self.logger.info(
                    "Connected to serial device: %s @ %d baud (attempt %d/%d)",
                    device_path, baudrate, attempt, self._retries,
                )
                self._breaker.success()
                return
            except serial.SerialException as exc:
                last_error = exc
                self._breaker.failure()
                self.logger.warning(
                    "Serial connect attempt %d/%d failed: %s",
                    attempt, self._retries, exc,
                )
                if attempt < self._retries:
                    delay = self._reconnect_delay * (2 ** (attempt - 1))
                    self.logger.info("Retrying in %.1fs...", delay)
                    time.sleep(delay)

        self._serial = None
        self._state = InstrumentState.ERROR
        raise ConnectionError(
            f"Could not connect to {device_path} after {self._retries} attempts: {last_error}"
        )

    def disconnect(self) -> None:
        """Close the serial port."""
        if self._serial is not None:
            try:
                if self._serial.is_open:
                    self._serial.close()
            except OSError as exc:
                self.logger.warning(
                    "Error closing %s: %s", self._device_path, exc
                )
            finally:
                self._serial = None
                self._state = InstrumentState.DISCONNECTED
                self.logger.info("Disconnected from %s", self._device_path)

    def _reconnect(self) -> None:
        """Cierra y reabre el puerto, redescubriendo por desafío si el puerto original falló."""
        self.disconnect()
        time.sleep(self._reconnect_delay)
        target_path = self._device_path
        if target_path:
            try:
                self.connect(target_path, self._baudrate, self._timeout)
                return
            except Exception as exc:
                self.logger.warning("Reconectar a %s falló (%s) — buscando por desafío", target_path, exc)

        from controller.base.device_identifier import DeviceIdentifier

        discovered = DeviceIdentifier.discover_device(
            "serial_generic", "serial", preferred_path=self._device_path
        )
        if discovered:
            self._device_path = discovered
            self.connect(self._device_path, self._baudrate, self._timeout)
        else:
            if target_path:
                self.connect(target_path, self._baudrate, self._timeout)

    # ------------------------------------------------------------------
    # I/O primitives with retry + reconnect
    # ------------------------------------------------------------------

    def _ensure_connected(self) -> None:
        """Verifica que estamos conectados; si no, intenta reconectar."""
        if self._serial is not None and self._serial.is_open:
            return
        self.logger.warning("Serial port not connected — reconnecting...")
        self._reconnect()

    def write(self, cmd: str) -> None:
        """Send a command string with retry + auto-reconnect (thread-safe)."""
        with self._lock:
            with self._breaker.guard():
                self._write_with_retry(cmd)

    def _write_with_retry(self, cmd: str) -> None:
        """Loop interno de reintentos para write()."""
        result = sanitize(cmd)
        if not result.ok:
            raise ConnectionError(
                f"Serial: rejected SCPI command ({result.reason}): {cmd!r}"
            )
        sanitized = result.command
        last_error = None
        for attempt in range(1, self._retries + 1):
            try:
                self._ensure_connected()
                payload = (sanitized + self._terminator).encode("ascii")
                self._serial.write(payload)
                self.logger.debug("Serial → %s", cmd)
                return
            except (serial.SerialException, OSError, AttributeError) as exc:
                last_error = exc
                self.logger.warning(
                    "Serial write attempt %d/%d failed: %s",
                    attempt, self._retries, exc,
                )
                if attempt < self._retries:
                    try:
                        self._reconnect()
                    except ConnectionError as exc:
                        self.logger.warning("Reconnect failed during retry: %s", exc)
        raise ConnectionError(
            f"Serial write failed after {self._retries} attempts: {last_error}"
        )

    def query(self, cmd: str) -> str:
        """Send a command and return the response line (thread-safe)."""
        with self._lock:
            with self._breaker.guard():
                self._write_with_retry(cmd)
                return self._read_with_retry()

    def _read_with_retry(self) -> str:
        """Lee respuesta del serial con reintentos."""
        last_error = None
        for attempt in range(1, self._retries + 1):
            try:
                self._ensure_connected()
                raw = self._serial.readline()
                response = raw.decode("ascii", errors="replace").rstrip("\n").rstrip()
                self.logger.debug("Serial ← %r", response)
                return response
            except (serial.SerialException, OSError, AttributeError) as exc:
                last_error = exc
                self.logger.warning(
                    "Serial read attempt %d/%d failed: %s",
                    attempt, self._retries, exc,
                )
                if attempt < self._retries:
                    with contextlib.suppress(ConnectionError):
                        self._reconnect()
        # Timeout no es error — puede ser que el instrumento no responda
        if isinstance(last_error, serial.SerialException):
            raise ConnectionError(
                f"Serial read failed after {self._retries} attempts: {last_error}"
            )
        return ""

    # ------------------------------------------------------------------
    # I/O de bytes crudos — para protocolos binarios, no SCPI
    # ------------------------------------------------------------------
    # `write()` y `query()` son de texto: añaden terminador, codifican ASCII y pasan
    # por sanitize(), que valida comandos SCPI. Nada de eso vale para un protocolo
    # binario como el del lector CC1125, donde cualquier byte es legítimo y no hay
    # terminador de línea. Estos dos métodos comparten el lock, el circuit breaker y
    # la reconexión, y se saltan solo el tratamiento de texto.

    def write_bytes(self, data: bytes) -> None:
        """Envía bytes tal cual, sin terminador ni sanitize (thread-safe)."""
        with self._lock:
            with self._breaker.guard():
                last_error = None
                for attempt in range(1, self._retries + 1):
                    try:
                        self._ensure_connected()
                        self._serial.write(data)
                        self.logger.debug("Serial → %s", data.hex())
                        return
                    except (serial.SerialException, OSError, AttributeError) as exc:
                        last_error = exc
                        self.logger.warning(
                            "Serial write_bytes attempt %d/%d failed: %s",
                            attempt, self._retries, exc,
                        )
                        if attempt < self._retries:
                            try:
                                self._reconnect()
                            except ConnectionError as exc2:
                                self.logger.warning("Reconnect failed: %s", exc2)
                raise ConnectionError(
                    f"Serial write_bytes failed after {self._retries} attempts: {last_error}"
                )

    def read_bytes(self, max_bytes: int = 4096) -> bytes:
        """Lee lo que haya disponible, sin bloquear más que el timeout del puerto.

        Devuelve b"" si no llegó nada: para un flujo donde las tramas llegan sin
        pedirlas, "todavía nada" es normal y no es un error.
        """
        with self._lock:
            with self._breaker.guard():
                try:
                    self._ensure_connected()
                    pendientes = getattr(self._serial, "in_waiting", 0) or 0
                    cuantos = min(max_bytes, pendientes) if pendientes else 1
                    data = self._serial.read(cuantos)
                    if data:
                        self.logger.debug("Serial ← %s", data.hex())
                    return data
                except (serial.SerialException, OSError, AttributeError) as exc:
                    self.logger.warning("Serial read_bytes failed: %s", exc)
                    return b""

    def idn(self) -> str:
        """Query *IDN? with circuit breaker protection."""
        return self.query("*IDN?")

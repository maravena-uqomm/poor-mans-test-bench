"""TcpIpController — Transporte SCPI sobre TCP/IP con retry.

Abre un socket nuevo por cada batch de comandos. 10 intentos con
backoff exponencial (0.5s base, max 10s). Captura timeouts,
connection refused, y errores genéricos.

Mejoras de robustez (v3.1):
- Circuit breaker integrado para evitar fallos en cascada
- Timeouts estrictos con watchdog thread
- Métricas de latencia y salud
- Auto-recovery con estado machine
- ChaChaCha mode para testing de robustez

Jira: ID-1308 (Test Bench v2)
"""
from __future__ import annotations

import logging
import random
import socket
import threading
import time
from dataclasses import dataclass, field

from controller.base.circuit_breaker import CircuitBreaker, CircuitBreakerOpen
from controller.base.reliability import InstrumentState, RetryPolicy
from controller.base.scpi_sanitizer import sanitize
from controller.base.watchdog_mixin import WatchdogMixin


@dataclass
class TcpIpMetrics:
    """Métricas de salud del controlador TCP/IP."""

    total_commands: int = 0
    successful_commands: int = 0
    failed_commands: int = 0
    timeouts: int = 0
    connection_errors: int = 0
    last_success_at: float | None = None
    last_failure_at: float | None = None
    avg_latency_ms: float = 0.0
    max_latency_ms: float = 0.0
    _latency_samples: list[float] = field(default_factory=list, repr=False)
    _lock: threading.Lock = field(default_factory=threading.Lock, repr=False)

    def record_success(self, latency_ms: float) -> None:
        with self._lock:
            self.total_commands += 1
            self.successful_commands += 1
            self.last_success_at = time.monotonic()
            self._latency_samples.append(latency_ms)
            if len(self._latency_samples) > 100:
                self._latency_samples.pop(0)
            self.avg_latency_ms = sum(self._latency_samples) / len(self._latency_samples)
            self.max_latency_ms = max(self._latency_samples)

    def record_failure(self, is_timeout: bool = False, is_connection: bool = False) -> None:
        with self._lock:
            self.total_commands += 1
            self.failed_commands += 1
            self.last_failure_at = time.monotonic()
            if is_timeout:
                self.timeouts += 1
            if is_connection:
                self.connection_errors += 1


class TcpIpController(WatchdogMixin):
    def __init__(
        self,
        ip_address: str,
        port: int,
        device_name: str = "Unknown",
        max_attempts: int = 10,
        base_delay: float = 0.5,
        max_delay: float = 10.0,
        timeout: float = 3.0,
        command_timeout: float = 5.0,
        circuit_breaker_threshold: int = 5,
        circuit_breaker_cooldown: float = 30.0,
    ):
        self.ip_address = ip_address
        self.port = port
        self.device_name = device_name
        self.logger = logging.getLogger(f"tcp.{ip_address}:{port}")
        self._state = InstrumentState.DISCONNECTED
        self.max_attempts = max_attempts
        self.base_delay = base_delay
        self.max_delay = max_delay
        self.timeout = timeout
        self.command_timeout = command_timeout

        self._breaker = CircuitBreaker(
            device_name,
            threshold=circuit_breaker_threshold,
            cooldown=circuit_breaker_cooldown,
        )
        self._metrics = TcpIpMetrics()
        self._lock = threading.RLock()
        self._watchdog_device_name = device_name
        self._init_watchdog(default_timeout=command_timeout + 5.0)
        self._chaos_mode = False

        self.retry_policy = RetryPolicy(
            max_retries=self.max_attempts,
            base_delay=self.base_delay,
            max_delay=self.max_delay,
        )
        self.logger.info("[%s] TCP controller initialized", device_name)

    @property
    def state(self) -> InstrumentState:
        return self._state

    @property
    def metrics(self) -> TcpIpMetrics:
        return self._metrics

    @property
    def circuit_breaker_state(self) -> str:
        return self._breaker.state

    def enable_chaos_mode(self, enabled: bool = True) -> None:
        self._chaos_mode = enabled
        self.logger.info("[%s] Chaos mode %s", self.device_name, "ENABLED" if enabled else "DISABLED")

    def is_healthy(self) -> bool:
        return self._state == InstrumentState.CONNECTED and not self._is_hung

    # ------------------------------------------------------------------
    # Watchdog para detectar colgazos
    # ------------------------------------------------------------------

    def _watchdog_expired(self) -> None:
        """Callback cuando el watchdog expira - el comando se colgó."""
        super()._watchdog_expired()
        self._metrics.record_failure(is_timeout=True)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def disconnect(self) -> None:
        """Limpia recursos del controlador."""
        self._stop_watchdog()
        self._state = InstrumentState.DISCONNECTED

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()
        return False

    def send_command(self, command: str, no_response: bool = False) -> tuple[bool, str]:
        """Send SCPI command with retry, circuit breaker, and exponential backoff."""
        with self._lock:
            self.logger.debug("[%s] -> %s", self.device_name, command)

            result = sanitize(command)
            if not result.ok:
                self.logger.error(
                    "[%s] rejected SCPI command (%s): %r", self.device_name, result.reason, command
                )
                return False, f"rejected SCPI command ({result.reason}): {command}"

            try:
                with self._breaker.guard():
                    return self._send_command_impl(command, no_response)
            except CircuitBreakerOpen as exc:
                self.logger.warning("[%s] Circuit breaker open: %s", self.device_name, exc)
                raise ConnectionError(f"{self.device_name}: circuit breaker open — {exc}") from exc

    def _send_command_impl(self, command: str, no_response: bool = False) -> tuple[bool, str]:
        """Implementación interna con watchdog y métricas."""
        self._state = InstrumentState.BUSY
        self._start_watchdog()

        try:
            for attempt in range(1, self.max_attempts + 1):
                if attempt > 1:
                    delay = min(self.base_delay * (2 ** (attempt - 2)), self.max_delay)
                    self.logger.warning("[%s] retry %d/%d in %.1fs", self.device_name, attempt, self.max_attempts, delay)
                    time.sleep(delay)

                t0 = time.perf_counter()
                try:
                    success, responses = self.send_command_batch([command], no_response)
                    latency_ms = int((time.perf_counter() - t0) * 1000)

                    if success:
                        self._metrics.record_success(latency_ms)
                        self._state = InstrumentState.CONNECTED
                        response = responses[0] if responses else ""
                        if response:
                            self.logger.debug("[%s] <- %s", self.device_name, response.strip())
                        return True, response

                    self.logger.warning("[%s] attempt %d failed: %s", self.device_name, attempt, responses[0] if responses else "unknown")
                    self._metrics.record_failure()

                except (socket.timeout, ConnectionRefusedError, OSError) as exc:
                    latency_ms = int((time.perf_counter() - t0) * 1000)
                    is_timeout = isinstance(exc, socket.timeout)
                    self.logger.error("[%s] connection error (attempt %d/%d): %s", self.device_name, attempt, self.max_attempts, exc)
                    self._metrics.record_failure(is_timeout=is_timeout, is_connection=not is_timeout)

            self._state = InstrumentState.ERROR
            raise ConnectionError(
                f"Failed sending '{command}' to {self.device_name}@{self.ip_address}:{self.port} "
                f"after {self.max_attempts} attempts"
            )
        finally:
            self._stop_watchdog()

    def send_command_batch(self, commands: list[str], no_response: bool = False) -> tuple[bool, list[str]]:
        """Send a batch of commands over a single socket.

        Returns:
            (True, [response_strings]) on success.
            (False, [error_message]) on failure.
        """
        with self._lock:
            responses: list[str] = []

            if self._chaos_mode and random.random() < 0.1:
                raise socket.timeout("Chaos mode: random timeout")

            try:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    s.settimeout(self.timeout)
                    s.connect((self.ip_address, self.port))
                    self._state = InstrumentState.CONNECTED

                    for cmd in commands:
                        result = sanitize(cmd)
                        if not result.ok:
                            self.logger.error(
                                "[%s] rejected SCPI command (%s): %r",
                                self.device_name, result.reason, cmd,
                            )
                            return False, [f"rejected SCPI command ({result.reason}): {cmd}"]
                        s.sendall(result.command.encode() + b"\n")
                        if not no_response:
                            s.settimeout(self.command_timeout)
                            responses.append(s.recv(1024).decode().strip())
                            s.settimeout(self.timeout)
                return True, responses
            except socket.timeout:
                self._metrics.record_failure(is_timeout=True)
                return False, [f"Timeout connecting to {self.ip_address}:{self.port}"]
            except (ConnectionRefusedError, OSError, BrokenPipeError) as exc:
                self._metrics.record_failure(is_connection=True)
                return False, [f"Connection refused {self.ip_address}:{self.port}: {exc}"]

    def reset_circuit_breaker(self) -> None:
        """Resetea el circuit breaker (útil para recovery manual)."""
        self._breaker.success()
        self._state = InstrumentState.CONNECTED
        self.logger.info("[%s] Circuit breaker manually reset", self.device_name)

    def get_health_report(self) -> dict:
        """Retorna un reporte de salud del controlador."""
        return {
            "device_name": self.device_name,
            "ip_address": self.ip_address,
            "port": self.port,
            "state": self._state.name,
            "circuit_breaker": self._breaker.state,
            "is_hung": self._is_hung,
            "chaos_mode": self._chaos_mode,
            "metrics": {
                "total_commands": self._metrics.total_commands,
                "successful": self._metrics.successful_commands,
                "failed": self._metrics.failed_commands,
                "timeouts": self._metrics.timeouts,
                "connection_errors": self._metrics.connection_errors,
                "avg_latency_ms": round(self._metrics.avg_latency_ms, 2),
                "max_latency_ms": self._metrics.max_latency_ms,
            },
            "last_success_at": self._metrics.last_success_at,
            "last_failure_at": self._metrics.last_failure_at,
        }

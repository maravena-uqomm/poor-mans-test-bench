"""
Circuit Breaker — protege contra fallos en cascada de instrumentos.

Cuando un instrumento falla N veces consecutivas, el circuit breaker
"se abre" y bloquea cualquier intento adicional durante M segundos.
Esto evita que el orquestador queme recursos reintentando contra un
instrumento que claramente está caído.

Uso:
    breaker = CircuitBreaker("analyzer")
    with breaker.guard():
        analyzer.send_command("*IDN?")

Jira: ID-1308 (Test Bench v2)
"""
from __future__ import annotations

import logging
import time

LOGGER = logging.getLogger(__name__)


class CircuitBreakerOpen(RuntimeError):
    """El circuit breaker está abierto — instrumento no disponible."""

    def __init__(self, name: str, cooldown: float) -> None:
        super().__init__(
            f"Circuit breaker abierto para '{name}': "
            f"espere {cooldown:.0f}s antes de reintentar"
        )
        self.instrument_name = name
        self.cooldown = cooldown


class CircuitBreaker:
    """Circuit breaker por instrumento.

    Args:
        name: Nombre del instrumento (para logs).
        threshold: Fallos consecutivos antes de abrir (default 3).
        cooldown: Segundos a esperar antes de medio-abrir (default 15).
        half_open_max: Máximo de intentos en estado half-open (default 1).
    """

    def __init__(
        self,
        name: str,
        threshold: int = 3,
        cooldown: float = 15.0,
        half_open_max: int = 1,
    ) -> None:
        self.name = name
        self.threshold = threshold
        self.cooldown = cooldown
        self.half_open_max = half_open_max

        self._failures = 0
        self._state = "closed"  # closed | open | half-open
        self._opened_at: float = 0.0
        self._half_open_attempts = 0

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    @property
    def state(self) -> str:
        return self._state

    def success(self) -> None:
        """Registra un éxito — resetea el contador de fallos."""
        self._failures = 0
        self._half_open_attempts = 0
        if self._state != "closed":
            LOGGER.info("Circuit breaker '%s': cerrado (éxito)", self.name)
            self._state = "closed"

    def failure(self) -> None:
        """Registra un fallo — puede abrir el circuito."""
        self._failures += 1
        if self._failures >= self.threshold and self._state == "closed":
            self._state = "open"
            self._opened_at = time.monotonic()
            LOGGER.warning(
                "Circuit breaker '%s': ABIERTO (%d fallos consecutivos, "
                "cooldown=%.0fs)",
                self.name, self._failures, self.cooldown,
            )

    def guard(self) -> _CircuitGuard:
        """Context manager que verifica el estado antes de ejecutar.

        SIRVE SOLO SI EL FALLO LLEGA COMO EXCEPCION. El guard decide por `exc_type`, asi que
        una operacion que informa el fallo DEVOLVIENDO algo —False, None, b""— sale del
        bloque sin excepcion y el guard la cuenta como **exito**. Para esos casos, `check()`
        mas `success()`/`failure()` explicitos; ver la nota en `check()`.

        Uso:
            with breaker.guard():
                instrument.send_command(...)   # levanta si falla
        """
        self._check()
        return _CircuitGuard(self)

    def check(self) -> None:
        """Verifica si se puede intentar una operacion. Levanta CircuitBreakerOpen si no.

        Es `guard()` sin el registro automatico, para el llamador que informa su resultado a
        mano con `success()` / `failure()`. Existe porque el registro automatico solo puede
        mirar si hubo excepcion, y hay codigo —el controlador del VLAD— cuyo contrato es
        devolver un codigo de retorno: ahi el guard convertia cada fallo en un exito.

        Medido el 09-Sep-2026: seis NACK del firmware seguidos dejaban el breaker en `closed`
        con `_failures == 0`, y un NACK despues de dos errores de transporte **borraba** los
        dos, porque `success()` resetea el contador.
        """
        self._check()

    def _check(self) -> None:
        """Verifica si podemos intentar una operación. Lanza si no."""
        now = time.monotonic()

        if self._state == "open":
            if self._opened_at > 0.0 and (now - self._opened_at >= self.cooldown):
                # Half-open: permitimos un intento de prueba
                self._state = "half-open"
                self._half_open_attempts = 1
                LOGGER.info("Circuit breaker '%s': medio-abierto (probando)", self.name)
            elif self._opened_at > 0.0:
                remaining = self.cooldown - (now - self._opened_at)
                raise CircuitBreakerOpen(self.name, remaining)
            else:
                # _opened_at no seteado pero _state == "open" (ej. en tests)
                raise CircuitBreakerOpen(self.name, self.cooldown)

        elif self._state == "half-open":
            self._half_open_attempts += 1
            if self._half_open_attempts > self.half_open_max:
                # Re-open: agotamos intentos en half-open
                self._state = "open"
                self._opened_at = time.monotonic()
                LOGGER.warning(
                    "Circuit breaker '%s': re-abierto (half-open agotado)", self.name
                )
                raise CircuitBreakerOpen(self.name, self.cooldown)


class _CircuitGuard:
    """Context manager interno que llama a success() o failure()."""

    def __init__(self, breaker: CircuitBreaker) -> None:
        self._breaker = breaker

    def __enter__(self) -> None:
        pass

    def __exit__(self, exc_type, exc_val, exc_tb) -> bool:
        if exc_type is None:
            self._breaker.success()
        elif isinstance(exc_val, (CircuitBreakerOpen,)):
            # No contamos un "abierto" como fallo del instrumento
            pass
        else:
            self._breaker.failure()
        return False  # no suprimir la excepción

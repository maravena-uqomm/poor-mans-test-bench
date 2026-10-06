"""Reliability primitives for instrument controllers.

 diseño mínimo para producción en masa:

 1. Estado del instrumento (state machine) — cada controller sabe
    en qué estado está sin adivinar.

 2. Jerarquía de errores con flag retriable — el código de recovery
    sabe si puede reintentar o debe escalar.

 3. RetryPolicy — política unificada de reintentos con backoff.
    Cada controller define la suya. La aplica con with_retry().

 4. Con estado + errores + retry, el código de recovery puede
    decidir: ¿reintento? ¿reconecto? ¿abro circuit breaker?
    ¿escalo al operador?

 Uso tipico:

     from controller.base.reliability import (
         InstrumentState, InstrumentError, RetryPolicy, with_retry,
     )

     class MiController:
         _state: InstrumentState = InstrumentState.DISCONNECTED
         retry_policy = RetryPolicy(max_retries=3, base_delay=0.5)

         @with_retry
         def send(self, cmd: str) -> str:
             self._ensure_state(InstrumentState.CONNECTED)
             ...
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from enum import Enum, auto
from functools import wraps
from typing import Callable, ParamSpec, TypeVar

LOGGER = logging.getLogger(__name__)


# ======================================================================
# Estado del instrumento
# ======================================================================

class InstrumentState(Enum):
    """Estado actual de un instrumento físico.

    DISCONNECTED → CONNECTING → CONNECTED → (uso normal)
                                              → BUSY (operación en curso)
                                              → ERROR → RECOVERING → CONNECTING → ...
    """
    DISCONNECTED = auto()
    CONNECTING = auto()
    CONNECTED = auto()
    BUSY = auto()
    ERROR = auto()
    RECOVERING = auto()


# ======================================================================
# Jerarquía de errores
# ======================================================================

class InstrumentError(Exception):
    """Error base de instrumentos. Lleva metadatos para recovery.

    Attributes:
        retriable: True si reintentar tiene sentido (timeout, busy).
                  False si es error de programación o hardware muerto.
        recoverable: True si reconectar/resetear puede resolverlo.
        instrument_name: Nombre lógico del instrumento.
    """

    def __init__(
        self,
        message: str,
        *,
        retriable: bool = False,
        recoverable: bool = True,
        instrument_name: str = "",
    ):
        super().__init__(message)
        self.retriable = retriable
        self.recoverable = recoverable
        self.instrument_name = instrument_name


class TimeoutError(InstrumentError):
    """El instrumento no respondió dentro del plazo esperado."""

    def __init__(self, message: str, *, instrument_name: str = ""):
        super().__init__(message, retriable=True, recoverable=True, instrument_name=instrument_name)


class BusyError(InstrumentError):
    """El instrumento está ocupado y no puede atender el comando."""

    def __init__(self, message: str, *, instrument_name: str = ""):
        super().__init__(message, retriable=True, recoverable=False, instrument_name=instrument_name)


class ConnectionError(InstrumentError):
    """No se puede establecer o mantener la conexión con el instrumento."""

    def __init__(self, message: str, *, instrument_name: str = ""):
        super().__init__(message, retriable=True, recoverable=True, instrument_name=instrument_name)


class ProtocolError(InstrumentError):
    """El instrumento respondió algo inesperado (garbage, IDN incorrecto)."""

    def __init__(self, message: str, *, instrument_name: str = ""):
        super().__init__(message, retriable=False, recoverable=True, instrument_name=instrument_name)


class HardwareError(InstrumentError):
    """El hardware está muerto o no responde. No recovery posible sin intervención."""

    def __init__(self, message: str, *, instrument_name: str = ""):
        super().__init__(message, retriable=False, recoverable=False, instrument_name=instrument_name)


# ======================================================================
# Política de reintentos
# ======================================================================

@dataclass
class RetryPolicy:
    """Política de reintentos para operaciones sobre instrumentos.

    Attributes:
        max_retries: Intentos totales (default 3, 0 = sin retry).
        base_delay: Segundos base para backoff exponencial (default 0.5).
        max_delay: Techo para el delay (default 10.0).
        retriable_exceptions: Tupla de excepciones que disparan retry.
                              Por defecto, todas las InstrumentError con retriable=True.
    """
    max_retries: int = 3
    base_delay: float = 0.5
    max_delay: float = 10.0
    retriable_exceptions: tuple[type[Exception], ...] = ()

    def delay(self, attempt: int) -> float:
        """Backoff exponencial: base_delay * 2^(attempt-1), capped."""
        return min(self.base_delay * (2 ** (attempt - 1)), self.max_delay)


# ======================================================================
# Decorador de reintento automático
# ======================================================================

P = ParamSpec("P")
R = TypeVar("R")


def with_retry(
    func: Callable[P, R] | None = None,
    *,
    policy: RetryPolicy | None = None,
) -> Callable[P, R]:
    """Decorador: reintenta la función según la RetryPolicy del controller.

    El controller debe tener un atributo `retry_policy: RetryPolicy`
    y un atributo `_state: InstrumentState`.

    Uso simple (usa self.retry_policy):
        @with_retry
        def send(self, cmd): ...

    Uso con policy explícita:
        @with_retry(policy=RetryPolicy(max_retries=5))
        def connect(self): ...
    """
    def decorator(f: Callable[P, R]) -> Callable[P, R]:
        @wraps(f)
        def wrapper(*args: P.args, **kwargs: P.kwargs) -> R:
            self = args[0] if args else None
            p = policy or (getattr(self, "retry_policy", None) if self else None) or RetryPolicy()
            last_error: Exception | None = None

            for attempt in range(1, p.max_retries + 1):
                try:
                    result = f(*args, **kwargs)
                    # Éxito → marcar estado como CONNECTED si estaba en ERROR/RECOVERING
                    if self is not None and hasattr(self, "_state"):
                        if self._state in (InstrumentState.ERROR, InstrumentState.RECOVERING, InstrumentState.DISCONNECTED):
                            self._state = InstrumentState.CONNECTED
                    return result
                except p.retriable_exceptions as exc:
                    last_error = exc
                    LOGGER.warning(
                        "%s: attempt %d/%d failed: %s",
                        getattr(self, "_name", f.__name__) if self else f.__name__,
                        attempt, p.max_retries, exc,
                    )
                    if attempt < p.max_retries:
                        delay = p.delay(attempt)
                        LOGGER.debug("retrying in %.1fs...", delay)
                        time.sleep(delay)
                    continue
                except InstrumentError as exc:
                    if exc.retriable:
                        last_error = exc
                        LOGGER.warning(
                            "%s: attempt %d/%d (retriable): %s",
                            getattr(self, "_name", f.__name__) if self else f.__name__,
                            attempt, p.max_retries, exc,
                        )
                        if attempt < p.max_retries:
                            time.sleep(p.delay(attempt))
                        continue
                    # No retriable → propagar
                    raise

            # Todos los reintentos agotados
            name = getattr(self, "_name", f.__name__) if self else f.__name__
            if self is not None and hasattr(self, "_state"):
                self._state = InstrumentState.ERROR
            raise ConnectionError(f"{name}: all {p.max_retries} attempts failed: {last_error}") from last_error

        return wrapper

    if func is not None:
        return decorator(func)
    return decorator

"""
Métricas de salud para el controlador VLAD.

Dataclass thread-safe que registra operaciones, latencia y errores
de los transportes BLE y USB del VladController.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field

#: Cuantas latencias se guardan para calcular los percentiles.
VENTANA_LATENCIAS = 100


@dataclass
class VladMetrics:
    """Métricas de salud del VLAD controller."""

    total_operations: int = 0
    successful_operations: int = 0
    failed_operations: int = 0
    ble_timeouts: int = 0
    usb_timeouts: int = 0
    connection_errors: int = 0
    last_success_at: float | None = None
    last_failure_at: float | None = None
    avg_latency_ms: float = 0.0
    #: LA COLA, NO LA MEDIA. `avg_latency_ms` era lo unico que se informaba, y es la media
    #: del ensemble: con 99 operaciones de 200 ms y una de 9 s el promedio dice 288 ms y la
    #: corrida la vive el peor caso. El p95 y el maximo son los que dicen si el enlace se
    #: esta degradando; `max_latency_ms` NO usa ventana —es el peor de toda la sesion— porque
    #: un pico que se sale de las ultimas 100 muestras sigue habiendo pasado.
    p95_latency_ms: float = 0.0
    max_latency_ms: float = 0.0
    #: Intentos de recuperacion del enlace y cuantos salieron. Sin esto, un equipo que se
    #: cae y se recupera veinte veces en una corrida es indistinguible de uno que anduvo
    #: bien: las dos historias terminan con el enlace en pie.
    recovery_attempts: int = 0
    recoveries_ok: int = 0
    _latency_samples: list[float] = field(default_factory=list)
    _lock: threading.Lock = field(default_factory=threading.Lock)

    def record_success(self, latency_ms: float) -> None:
        with self._lock:
            self.total_operations += 1
            self.successful_operations += 1
            self.last_success_at = time.monotonic()
            self.max_latency_ms = max(self.max_latency_ms, latency_ms)
            self._latency_samples.append(latency_ms)
            if len(self._latency_samples) > VENTANA_LATENCIAS:
                self._latency_samples.pop(0)
            muestras = self._latency_samples
            self.avg_latency_ms = sum(muestras) / len(muestras) if muestras else 0.0
            self.p95_latency_ms = _percentil(muestras, 95)

    def record_failure(self, is_ble_timeout: bool = False, is_usb_timeout: bool = False, is_connection: bool = False) -> None:
        with self._lock:
            self.total_operations += 1
            self.failed_operations += 1
            self.last_failure_at = time.monotonic()
            if is_ble_timeout:
                self.ble_timeouts += 1
            if is_usb_timeout:
                self.usb_timeouts += 1
            if is_connection:
                self.connection_errors += 1

    def record_recovery(self, exito: bool) -> None:
        """Cuenta un intento de recuperación del enlace y si salió."""
        with self._lock:
            self.recovery_attempts += 1
            if exito:
                self.recoveries_ok += 1


def _percentil(muestras: list[float], p: int) -> float:
    """Percentil `p` por el método del vecino más cercano. 0.0 si no hay muestras.

    Sin interpolación a propósito: devuelve una latencia que **se midió de verdad**, no un
    promedio entre dos. Para decidir un plazo, un número observado vale más que uno
    calculado — y con 100 muestras la diferencia entre los dos métodos es menor que el
    ruido del propio transporte.
    """
    if not muestras:
        return 0.0
    ordenadas = sorted(muestras)
    idx = min(len(ordenadas) - 1, max(0, round(p / 100.0 * len(ordenadas)) - 1))
    return ordenadas[idx]

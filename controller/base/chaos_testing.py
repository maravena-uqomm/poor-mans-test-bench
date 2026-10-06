"""
Chaos Testing Framework para controladores de instrumentación.

Este módulo permite simular problemas aleatorios en los controladores
para verificar su robustez y capacidad de recovery:

- Fallos de conexión aleatorios
- Timeouts random
- Errores de protocolo
- Pérdida de conexión en medio de operaciones
- Latencia elevada simulada
- Datos corruptos

Uso:
    from controller.base.chaos_testing import ChaosEngine, ChaosScenario

    # Crear escenario de chaos
    scenario = ChaosScenario(
        timeout_probability=0.2,
        connection_error_probability=0.1,
        corruption_probability=0.05,
        latency_ms=(50, 200),
    )

    # Aplicar a un controlador
    chaos = ChaosEngine(scenario)
    chaos.inject_into(controller)

    # Ejecutar operaciones y verificar robustez
    ...

Jira: ID-CHAOS (Robustez v3.1)
"""

from __future__ import annotations

import logging
import random
import threading
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Optional

from controller.base.reliability import (
    ConnectionError as RelConnectionError,
    TimeoutError as RelTimeoutError,
)

LOGGER = logging.getLogger(__name__)


class ChaosNoInyectableError(RuntimeError):
    """El controlador no expone ningun metodo que este motor sepa envolver.

    Es un error y no un aviso: una prueba de caos que no inyecto nada pasa siempre, y su
    resultado verde se lee como «el controlador aguanto» cuando significa «no se probo
    nada». Ver la nota de `ChaosEngine.inject_into`.
    """

    def __init__(self, controller: Any, nombres: tuple[str, ...]) -> None:
        super().__init__(
            f"ChaosEngine no puede inyectar en {type(controller).__name__}: no expone "
            f"ninguno de {list(nombres)}. Una corrida de caos sobre este objeto no probaria "
            f"nada y daria verde. Agregar el metodo de transporte a _NOMBRES_INYECTABLES y "
            f"su inyector, o no usar ChaosEngine con este controlador."
        )
        self.controller_type = type(controller).__name__
        self.nombres_buscados = nombres


class ChaosEventType(Enum):
    """Tipos de eventos de chaos que pueden ser inyectados."""
    TIMEOUT = "timeout"
    CONNECTION_ERROR = "connection_error"
    PROTOCOL_ERROR = "protocol_error"
    DATA_CORRUPTION = "data_corruption"
    LATENCY_SPIKE = "latency_spike"
    CONNECTION_DROP = "connection_drop"
    GARBAGE_RESPONSE = "garbage_response"


@dataclass
class ChaosScenario:
    """Configuración de un escenario de chaos testing.

    Cada probabilidad es un valor entre 0.0 y 1.0.
    """

    timeout_probability: float = 0.1
    connection_error_probability: float = 0.05
    protocol_error_probability: float = 0.02
    corruption_probability: float = 0.01
    latency_spike_probability: float = 0.3
    latency_ms_range: tuple[int, int] = (10, 100)
    connection_drop_probability: float = 0.01

    max_consecutive_failures: int = 3
    recovery_delay_range: tuple[float, float] = (0.1, 1.0)

    def should_trigger(self, event_type: ChaosEventType) -> bool:
        """Determina si un evento debe ser triggered basado en su probabilidad."""
        probs = {
            ChaosEventType.TIMEOUT: self.timeout_probability,
            ChaosEventType.CONNECTION_ERROR: self.connection_error_probability,
            ChaosEventType.PROTOCOL_ERROR: self.protocol_error_probability,
            ChaosEventType.DATA_CORRUPTION: self.corruption_probability,
            ChaosEventType.LATENCY_SPIKE: self.latency_spike_probability,
            ChaosEventType.CONNECTION_DROP: self.connection_drop_probability,
            ChaosEventType.GARBAGE_RESPONSE: self.protocol_error_probability,
        }
        return random.random() < probs.get(event_type, 0.0)


@dataclass
class ChaosMetrics:
    """Métricas de eventos de chaos inyectados."""

    total_operations: int = 0
    chaos_events_triggered: int = 0
    timeouts_injected: int = 0
    connection_errors_injected: int = 0
    protocol_errors_injected: int = 0
    corruption_injected: int = 0
    latency_spikes_injected: int = 0
    connection_drops_injected: int = 0
    successful_recoveries: int = 0
    failed_recoveries: int = 0

    _lock: threading.Lock = field(default_factory=threading.Lock)

    def record_event(self, event_type: ChaosEventType) -> None:
        with self._lock:
            self.chaos_events_triggered += 1
            if event_type == ChaosEventType.TIMEOUT:
                self.timeouts_injected += 1
            elif event_type == ChaosEventType.CONNECTION_ERROR:
                self.connection_errors_injected += 1
            elif event_type == ChaosEventType.PROTOCOL_ERROR:
                self.protocol_errors_injected += 1
            elif event_type == ChaosEventType.DATA_CORRUPTION:
                self.corruption_injected += 1
            elif event_type == ChaosEventType.LATENCY_SPIKE:
                self.latency_spikes_injected += 1
            elif event_type == ChaosEventType.CONNECTION_DROP:
                self.connection_drops_injected += 1

    def record_operation(self) -> None:
        with self._lock:
            self.total_operations += 1

    def record_recovery(self, success: bool) -> None:
        with self._lock:
            if success:
                self.successful_recoveries += 1
            else:
                self.failed_recoveries += 1


class ChaosEngine:
    """Motor de chaos testing que inyecta fallos en controladores.

    Puede ser usado con cualquier tipo de controlador que tenga los métodos:
    - send_command / query / write
    - connect / disconnect
    - is_healthy / state

    Uso:
        chaos = ChaosEngine(scenario)
        chaos.inject_into(my_controller)
        # Ahora todas las operaciones pueden fallar aleatoriamente
    """

    def __init__(
        self,
        scenario: ChaosScenario | None = None,
        seed: int | None = None,
    ):
        self.scenario = scenario or ChaosScenario()
        self._metrics = ChaosMetrics()
        self._enabled = False
        self._controller: Optional[Any] = None
        self._original_methods: dict[str, Any] = {}
        self._added_instance_attrs: set[str] = set()
        self._consecutive_failures = 0
        self._lock = threading.Lock()

        if seed is not None:
            random.seed(seed)

        self.logger = logging.getLogger(self.__class__.__name__)

    @property
    def metrics(self) -> ChaosMetrics:
        return self._metrics

    @property
    def is_enabled(self) -> bool:
        return self._enabled

    def enable(self) -> None:
        """Activa el modo chaos."""
        self._enabled = True
        self.logger.info("Chaos mode ENABLED")

    def disable(self) -> None:
        """Desactiva el modo chaos."""
        self._enabled = False
        self.logger.info("Chaos mode DISABLED")

    def inject_into(self, controller: Any) -> None:
        """Inyecta chaos en un controlador y guarda estado para restaurar.

        LEVANTA ChaosNoInyectableError SI NO ENVOLVIO NINGUN METODO, y eso es el arreglo del
        09-Sep-2026. Los dos inyectores buscan `send_command`/`send_command_batch` y
        `write`/`query`; un controlador que no expone ninguno de esos nombres salia de aca
        con `_original_methods == {}` y un log que decia «Chaos injected into controller».

        Medido sobre `VladController`, cuya superficie es `set_gain`/`get_gain`/
        `_send_command`: con `timeout_probability=1.0`, diez operaciones dieron **diez
        exitos**, `chaos_events_triggered == 0` y un `get_health_report()` entero en ceros.
        O sea indistinguible de «se inyecto caos y el controlador sobrevivio todo», que es
        la conclusion opuesta a la verdadera.

        Un inyector de fallos que no puede inyectar tiene que decirlo: es la unica forma de
        que una prueba de caos verde signifique algo.
        """
        self._controller = controller
        self._consecutive_failures = 0
        self._original_methods = {}
        self._added_instance_attrs: set[str] = set()
        self._inject_tcpip_chaos(controller)
        self._inject_serial_chaos(controller)
        self._inject_binary_protocol_chaos(controller)
        if not self._original_methods:
            raise ChaosNoInyectableError(controller, self._NOMBRES_INYECTABLES)
        self.logger.info(
            "Chaos injected into %s: %s",
            type(controller).__name__, sorted(self._original_methods),
        )

    #: Los nombres que alguno de los inyectores sabe envolver. Se nombra para que el error
    #: de ChaosNoInyectableError pueda decir que hacia falta, en vez de solo que fallo.
    _NOMBRES_INYECTABLES = (
        "send_command", "send_command_batch",   # TCP/IP (SCPI)
        "write", "query",                       # serie de texto
        "_send_command", "_query_command",      # protocolo binario (VLAD25 V2)
    )

    def _inject_binary_protocol_chaos(self, controller: Any) -> None:
        """Inyecta en la costura de un controlador de protocolo binario.

        `_send_command(cmd, payload) -> bool` y `_query_command(cmd) -> bytes` son los dos
        puntos por los que pasa TODO el trafico del VladController, asi que envolverlos
        ejercita de una vez el circuit breaker, las metricas y la deteccion de enlace
        perdido. Son privados y se envuelven igual a proposito: es donde esta el transporte,
        y un inyector que solo alcanza la API publica no puede simular un cable que se corta.
        """
        for nombre in ("_send_command", "_query_command"):
            if not hasattr(controller, nombre):
                continue
            era_de_instancia = nombre in controller.__dict__
            original = getattr(controller, nombre)
            self._original_methods[nombre] = original
            if not era_de_instancia:
                self._added_instance_attrs.add(nombre)

            def envuelto(*args, _original=original, _nombre=nombre, **kwargs):
                self._metrics.record_operation()
                if self._should_inject():
                    if self.scenario.should_trigger(ChaosEventType.TIMEOUT):
                        self._metrics.record_event(ChaosEventType.TIMEOUT)
                        self._record_failure()
                        raise RelTimeoutError(f"Chaos: timeout inyectado en {_nombre}")
                    if self.scenario.should_trigger(ChaosEventType.CONNECTION_ERROR):
                        self._metrics.record_event(ChaosEventType.CONNECTION_ERROR)
                        self._record_failure()
                        raise OSError(f"Chaos: caida de enlace inyectada en {_nombre}")
                try:
                    r = _original(*args, **kwargs)
                except Exception:
                    self._record_failure()
                    raise
                # El fallo de estos dos NO es una excepcion: es un False o un b"" —el mismo
                # contrato que hacia que el circuit breaker no viera nada—. Se cuenta por el
                # VALOR, no por la ausencia de excepcion.
                if r is False or r == b"":
                    self._record_failure()
                else:
                    self._record_success()
                return r

            setattr(controller, nombre, envuelto)

    def _should_inject(self) -> bool:
        """Determina si debe inyectar chaos en esta operación."""
        if not self._enabled:
            return False
        if self._consecutive_failures >= self.scenario.max_consecutive_failures:
            recovery_delay = random.uniform(*self.scenario.recovery_delay_range)
            self.logger.info("Chaos: max failures reached, waiting %.1fs for recovery", recovery_delay)
            time.sleep(recovery_delay)
            self._consecutive_failures = 0
        return True

    def _record_failure(self) -> None:
        """Registra un fallo y determina si debe recovery."""
        with self._lock:
            self._consecutive_failures += 1
            if self._consecutive_failures >= self.scenario.max_consecutive_failures:
                self.logger.warning(
                    "Chaos: %d consecutive failures, triggering recovery",
                    self._consecutive_failures
                )

    def _record_success(self) -> None:
        """Registra un éxito y resetea contador de fallos."""
        with self._lock:
            if self._consecutive_failures >= self.scenario.max_consecutive_failures:
                self._metrics.record_recovery(success=True)
            self._consecutive_failures = 0

    def _inject_tcpip_chaos(self, controller: Any) -> None:
        """Inyecta chaos en controladores TCP/IP."""
        if hasattr(controller, 'send_command_batch'):
            was_instance_attr = 'send_command_batch' in controller.__dict__
            original_method = controller.send_command_batch
            self._original_methods['send_command_batch'] = original_method
            if not was_instance_attr:
                self._added_instance_attrs.add('send_command_batch')

            def chaotic_send_command_batch(commands: list[str], no_response: bool = False):
                self._metrics.record_operation()
                if self._should_inject():
                    if self.scenario.should_trigger(ChaosEventType.TIMEOUT):
                        self._metrics.record_event(ChaosEventType.TIMEOUT)
                        self._record_failure()
                        raise RelTimeoutError("Chaos: injected timeout")
                    if self.scenario.should_trigger(ChaosEventType.CONNECTION_ERROR):
                        self._metrics.record_event(ChaosEventType.CONNECTION_ERROR)
                        self._record_failure()
                        raise RelConnectionError("Chaos: injected connection error")
                    if self.scenario.should_trigger(ChaosEventType.LATENCY_SPIKE):
                        latency = random.randint(*self.scenario.latency_ms_range)
                        self._metrics.record_event(ChaosEventType.LATENCY_SPIKE)
                        time.sleep(latency / 1000.0)

                try:
                    result = original_method(commands, no_response)
                    self._record_success()
                    return result
                except Exception as exc:
                    self._record_failure()
                    raise

            controller.send_command_batch = chaotic_send_command_batch

    def _inject_serial_chaos(self, controller: Any) -> None:
        """Inyecta chaos en controladores seriales."""
        if hasattr(controller, 'write'):
            was_instance_attr = 'write' in controller.__dict__
            original_write = controller.write
            self._original_methods['write'] = original_write
            if not was_instance_attr:
                self._added_instance_attrs.add('write')

            def chaotic_write(cmd: str):
                self._metrics.record_operation()
                if self._should_inject():
                    if self.scenario.should_trigger(ChaosEventType.TIMEOUT):
                        self._metrics.record_event(ChaosEventType.TIMEOUT)
                        self._record_failure()
                        raise RelTimeoutError("Chaos: injected serial timeout")
                    if self.scenario.should_trigger(ChaosEventType.DATA_CORRUPTION):
                        self._metrics.record_event(ChaosEventType.DATA_CORRUPTION)
                        self._record_failure()
                        raise IOError("Chaos: injected data corruption")

                try:
                    result = original_write(cmd)
                    self._record_success()
                    return result
                except Exception as exc:
                    self._record_failure()
                    raise

            controller.write = chaotic_write

        if hasattr(controller, 'query'):
            was_instance_attr = 'query' in controller.__dict__
            original_query = controller.query
            self._original_methods['query'] = original_query
            if not was_instance_attr:
                self._added_instance_attrs.add('query')

            def chaotic_query(cmd: str) -> str:
                self._metrics.record_operation()
                if self._should_inject():
                    if self.scenario.should_trigger(ChaosEventType.GARBAGE_RESPONSE):
                        self._metrics.record_event(ChaosEventType.GARBAGE_RESPONSE)
                        self._record_failure()
                        return "CHAOS_GARBAGE_\x00\xff_RESPONSE"
                    if self.scenario.should_trigger(ChaosEventType.CONNECTION_DROP):
                        self._metrics.record_event(ChaosEventType.CONNECTION_DROP)
                        self._record_failure()
                        raise RelConnectionError("Chaos: injected connection drop")

                try:
                    result = original_query(cmd)
                    self._record_success()
                    return result
                except Exception as exc:
                    self._record_failure()
                    raise

            controller.query = chaotic_query

    def eject(self) -> None:
        """Remueve el chaos inyectado y restaura los metodos originales."""
        if self._controller is not None:
            for method_name in self._added_instance_attrs:
                if method_name in self._controller.__dict__:
                    delattr(self._controller, method_name)
            for method_name in self._original_methods:
                if method_name not in self._added_instance_attrs:
                    self._controller.__dict__[method_name] = self._original_methods[method_name]
            self.logger.info("Chaos ejected from controller, %d methods restored", len(self._original_methods))
        self._controller = None
        self._original_methods.clear()
        self._added_instance_attrs.clear()

    def get_health_report(self) -> dict:
        """Retorna un reporte de salud del engine de chaos."""
        return {
            "enabled": self._enabled,
            "consecutive_failures": self._consecutive_failures,
            "metrics": {
                "total_operations": self._metrics.total_operations,
                "chaos_events_triggered": self._metrics.chaos_events_triggered,
                "timeouts_injected": self._metrics.timeouts_injected,
                "connection_errors_injected": self._metrics.connection_errors_injected,
                "protocol_errors_injected": self._metrics.protocol_errors_injected,
                "corruption_injected": self._metrics.corruption_injected,
                "latency_spikes_injected": self._metrics.latency_spikes_injected,
                "connection_drops_injected": self._metrics.connection_drops_injected,
                "successful_recoveries": self._metrics.successful_recoveries,
                "failed_recoveries": self._metrics.failed_recoveries,
            },
            "scenario": {
                "timeout_probability": self.scenario.timeout_probability,
                "connection_error_probability": self.scenario.connection_error_probability,
                "protocol_error_probability": self.scenario.protocol_error_probability,
                "corruption_probability": self.scenario.corruption_probability,
                "latency_ms_range": self.scenario.latency_ms_range,
                "connection_drop_probability": self.scenario.connection_drop_probability,
            },
        }


class StressTestRunner:
    """Runner para ejecutar stress tests con chaos en controladores.

    Uso:
        runner = StressTestRunner(controller, iterations=100)
        runner.run_stress_test()
        print(runner.get_report())
    """

    def __init__(
        self,
        controller: Any,
        chaos_scenario: ChaosScenario | None = None,
        iterations: int = 50,
        seed: int | None = None,
    ):
        self.controller = controller
        self.chaos = ChaosEngine(chaos_scenario, seed=seed)
        self.iterations = iterations
        self._results: list[dict] = []
        self.logger = logging.getLogger(self.__class__.__name__)

    def run_stress_test(self) -> dict:
        """Ejecuta un stress test completo."""
        self.chaos.enable()
        self.chaos.inject_into(self.controller)

        for i in range(self.iterations):
            self.logger.debug("Stress iteration %d/%d", i + 1, self.iterations)
            result = self._run_single_iteration(i)
            self._results.append(result)

        self.chaos.eject()
        return self.get_report()

    def _run_single_iteration(self, iteration: int) -> dict:
        """Ejecuta una sola iteración del stress test."""
        result = {
            "iteration": iteration,
            "operations": [],
            "success": True,
            "chaos_events": 0,
        }

        if hasattr(self.controller, 'send_command'):
            try:
                self.controller.send_command("*IDN?")
                result["operations"].append({"op": "send_command", "success": True})
            except Exception as exc:
                result["operations"].append({"op": "send_command", "success": False, "error": str(exc)})
                result["success"] = False

        if hasattr(self.controller, 'idn'):
            try:
                self.controller.idn()
                result["operations"].append({"op": "idn", "success": True})
            except Exception as exc:
                result["operations"].append({"op": "idn", "success": False, "error": str(exc)})
                result["success"] = False

        result["chaos_events"] = self.chaos.metrics.chaos_events_triggered
        return result

    def get_report(self) -> dict:
        """Retorna un reporte completo del stress test."""
        successful_iterations = sum(1 for r in self._results if r["success"])
        total_operations = sum(len(r["operations"]) for r in self._results)
        successful_operations = sum(
            sum(1 for op in r["operations"] if op["success"])
            for r in self._results
        )

        return {
            "total_iterations": self.iterations,
            "successful_iterations": successful_iterations,
            "failed_iterations": self.iterations - successful_iterations,
            "success_rate": successful_iterations / self.iterations if self.iterations > 0 else 0,
            "total_operations": total_operations,
            "successful_operations": successful_operations,
            "failed_operations": total_operations - successful_operations,
            "chaos_metrics": self.chaos.get_health_report()["metrics"],
        }

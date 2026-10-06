"""
Híbrido Bluetooth/USB Controller for UQOMM VLAD RF Board (Rev25).

Provides active dual-transport communication (Bluetooth BLE and USB/Serial)
to remotely configure AGC and Gain levels on the VLAD board during live testing.

Mecanismos de robustez, y HASTA DONDE LLEGA CADA UNO (auditado el 09-Sep-2026: tres de
las cinco vinetas de esta lista prometian cosas que el codigo no tenia).

- Watchdog por operacion: detecta un colgazo y marca el estado en ERROR. Se recupera solo
  en la operacion siguiente.
- Metricas de salud y latencia: cuentan operaciones, informan la COLA (p95 y maximo, no
  solo el promedio) y llevan los intentos de recuperacion. Siguen siendo para LEER: nada
  las consume para adaptar plazos, que son constantes medidas a mano en el banco.
- Circuit breaker: abre tras N fallos CONSECUTIVOS, contando NACK y timeouts —no solo
  excepciones—, y con el abierto no se escribe al puerto.
- Recuperacion del enlace: una caida a mitad de corrida suelta el transporte y lo vuelve a
  tomar, con cooldown y sin caer a simulacion. Es la arista `ERROR -> RECOVERING` que el
  enum declaraba y este controlador no usaba. Si no se recupera, el fallo queda declarado
  en `datos_confiables()` y decide el llamador. Solo se recupera de una caida de
  TRANSPORTE: un NACK significa que el enlace anda y el firmware rechazo el comando.
- Modo caos: NO vive aca. Se borro un `_chaos_mode` que se guardaba, se reportaba y nunca
  se leia para inyectar un fallo — el mecanismo de verdad es `base/chaos_testing.py`
  (`ChaosEngine.inject_into`), que envuelve los metodos del controlador.

Si faltan las librerias o el hardware no esta accesible, el controlador cae a Simulation
Mode para no trabar el orquestador. **Eso hace que los setters devuelvan True sin escribir
nada**, asi que todo consumidor que mida algo tiene que preguntar `datos_confiables()`
antes de creerse un valor.

LA CLASE ESTA REPARTIDA EN CUATRO ARCHIVOS (09-Sep-2026), porque tenia 1093 lineas contra un
limite de 500 y el limite estaba ahi por algo: encontrar el circuit breaker obligaba a pasar
por trescientas lineas de sincronizacion de bytes. El corte sigue las capas que ya existian en
el codigo, no una division por tamano:

    vlad_controller.py   este archivo: construccion, estado, salud, breaker y recuperacion
    vlad_commands.py     un metodo por operacion del equipo, en unidades naturales
    vlad_transport.py    la capa de BYTES: serial y bleak, el eco, el sondeo de GATT
    vlad_connection.py   abrir, cerrar y reabrir el canal

y el protocolo —opcodes, largos, unidades— en vlad25_protocol.py, que no depende de ninguno.

Jira: ID-1556 (Test Bench v2)
"""

from __future__ import annotations

import asyncio
import logging
import threading
import time

from controller.base.circuit_breaker import CircuitBreaker, CircuitBreakerOpen
from controller.base.reliability import InstrumentState, RetryPolicy
from controller.base.vlad import Vlad
from controller.base.watchdog_mixin import WatchdogMixin
from controller.uqomm import vlad25_protocol as proto
from controller.uqomm.vlad_commands import VladCommandsMixin
from controller.uqomm.vlad_connection import VladConnectionMixin
from controller.uqomm.vlad_metrics import VladMetrics
from controller.uqomm.vlad_transport import VladTransportMixin

# Conditional import of serial and bleak — kept for type annotations
try:
    import serial
except ImportError:
    serial = None

try:
    from bleak import BleakClient, BleakScanner
except ImportError:
    BleakClient = None
    BleakScanner = None


class VladController(
    WatchdogMixin,
    VladCommandsMixin,
    VladConnectionMixin,
    VladTransportMixin,
    Vlad,
):
    """Híbrido Bluetooth BLE and USB/Serial controller for the VLAD Rev25 board.

    Comunicacion activa por los dos canales con caida a simulacion, watchdog y circuit
    breaker. El alcance de cada mecanismo esta en el docstring del modulo; lo importante
    para un llamador es `datos_confiables()`, que contesta si lo que devuelve este objeto
    describe al equipo.
    """

    NUS_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
    NUS_RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
    NUS_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

    DEFAULT_OPERATION_TIMEOUT = 5.0
    WATCHDOG_TIMEOUT = 15.0
    MAX_RECONNECT_ATTEMPTS = 3

    # El protocolo (comandos, largos y unidades) vive en vlad25_protocol.py.
    # Se re-exponen como atributos de clase porque los tests y los llamadores
    # los usan como `VladController.CMD_*`.
    CMD_SET_DAC_UL = proto.CMD_SET_DAC_UL
    CMD_SET_DAC_DL = proto.CMD_SET_DAC_DL
    CMD_SET_ATTEN_UL = proto.CMD_SET_ATTEN_UL
    CMD_SET_ATTEN_DL = proto.CMD_SET_ATTEN_DL
    CMD_QUERY_STATUS = proto.CMD_QUERY_STATUS
    CMD_QUERY_CURRENT = proto.CMD_QUERY_CURRENT
    CMD_SET_GAIN_UL = proto.CMD_SET_GAIN_UL
    CMD_SET_GAIN_DL = proto.CMD_SET_GAIN_DL
    CMD_QUERY_GAIN = proto.CMD_QUERY_GAIN
    CMD_SET_AGC = proto.CMD_SET_AGC
    CMD_QUERY_AGC = proto.CMD_QUERY_AGC
    CMD_QUERY_AGC_SENSE = proto.CMD_QUERY_AGC_SENSE
    CMD_DIAGNOSTIC_RUN = proto.CMD_DIAGNOSTIC_RUN
    CMD_FOTA_STATUS = proto.CMD_FOTA_STATUS
    CMD_SET_ATEN_FIJA_UL = proto.CMD_SET_ATEN_FIJA_UL
    CMD_SET_ATEN_FIJA_DL = proto.CMD_SET_ATEN_FIJA_DL
    CMD_QUERY_ATEN_PARTES = proto.CMD_QUERY_ATEN_PARTES
    RESP_LEN = proto.RESP_LEN
    CHANNELS = proto.CHANNELS
    ACK = proto.ACK
    NACK = proto.NACK
    GAIN_MIN_DB, GAIN_MAX_DB = proto.GAIN_MIN_DB, proto.GAIN_MAX_DB
    AGC_MIN_V, AGC_MAX_V = proto.AGC_MIN_V, proto.AGC_MAX_V

    def __init__(
        self,
        use_bluetooth: bool = True,
        ble_mac_address: str | None = None,
        usb_device_path: str | None = None,
        device_path: str | None = None,
        operation_timeout: float = 5.0,
        circuit_breaker_threshold: int = 3,
        circuit_breaker_cooldown: float = 30.0,
    ):
        """Initialize the VLAD hybrid controller.

        Args:
            use_bluetooth: If True, prioritizes Bluetooth BLE; otherwise USB/Serial.
            ble_mac_address: Target MAC address of the VLAD board's Bluetooth interface.
            usb_device_path: Target path for USB serial node (e.g., /dev/ttyUSB0).
            device_path: Alias for usb_device_path (compatibility with orchestrator param naming).
            operation_timeout: Timeout for BLE/USB operations in seconds.
            circuit_breaker_threshold: Fallos consecutivos antes de abrir el circuit breaker.
            circuit_breaker_cooldown: Segundos de cooldown del circuit breaker.
        """
        self.use_bluetooth = use_bluetooth
        self.ble_mac_address = ble_mac_address or "00:00:00:00:00:00"
        # SIN DEFAULT, Y ESO ARREGLA UN DEFECTO MEDIDO. Era `or "/dev/ttyUSB0"`.
        #
        # Un default que nombra un dispositivo REAL abre lo que haya en ese nodo. Medido el
        # 09-Sep-2026: **13 tests de tests/unit/ construyen `VladController(use_bluetooth=
        # False)` sin pasar ruta**, y en una maquina donde `/dev/ttyUSB0` existe el test
        # abria el puerto de verdad —verificado, `open=True` y `datos_confiables()`
        # devolviendo `(True, "")`— con lo cual `set_gain(20.0)` le escribia `0a 00 00 a0 41`
        # a lo que estuviera conectado.
        #
        # Es el accidente que AGENTS.md documenta —32 comandos de atenuacion al generador
        # Signal Hound por elegir el dispositivo por numero de ttyACM— metido en la suite de
        # unidad, donde nadie lo busca porque «los tests de unidad no tocan hardware».
        #
        # Sin ruta se cae a simulacion con un aviso, en `_check_simulation_status()`: no
        # saber a que puerto hablar es una razon legitima para no abrir ninguno. Produccion
        # siempre pasa `device_path`, derivado del endpoint del banco.
        self.usb_device_path = usb_device_path or device_path
        self.operation_timeout = operation_timeout

        self._serial_connection: serial.Serial | None = None
        self._ble_client: BleakClient | None = None
        self._is_simulation = False
        self._state = InstrumentState.DISCONNECTED
        self._name = "VladController"
        self._lock = threading.RLock()
        self._metrics = VladMetrics()
        self._breaker = CircuitBreaker(
            self._name,
            threshold=circuit_breaker_threshold,
            cooldown=circuit_breaker_cooldown,
        )

        self._loop: asyncio.AbstractEventLoop | None = None
        self._loop_thread: threading.Thread | None = None
        self._watchdog_device_name = self._name
        self._init_watchdog(default_timeout=self.WATCHDOG_TIMEOUT)

        self._gain_db = 25.0
        #: Referencia del AGC en VOLTS (antes eran dBm — ver base/vlad.py).
        self._agc_level = 0.0
        #: Ultimo desglose de 0x0C QUERY_GAIN: los dos canales, DAC y atenuadores.
        #: Secuencia de la ultima QUERY_SMARTRING leida, para poder exigir una nueva.
        self._last_smartring_seq = None
        self.last_gain_readout: dict[str, float] = {}
        #: Ultimo desglose de 0x17 QUERY_AGC: referencia de los dos canales.
        self.last_agc_readout: dict[str, float] = {}
        #: Ultimo sensado real del lazo (QUERY_AGC_SENSE). Vacio hasta la primera
        #: lectura exitosa: no se prellena con ceros, que serian indistinguibles de
        #: una medicion legitima de cero.
        self.last_agc_sense: dict[str, int] = {}
        #: Ultimo desglose de 0x45 QUERY_ATEN_PARTES: fija y variable de las dos ramas.
        #: Vacio hasta la primera lectura, igual que last_agc_sense y por el mismo motivo:
        #: un cero prellenado seria indistinguible de «la fija esta en 0 dB».
        self.last_atten_parts: dict[str, float] = {}

        #: Motivo por el que el enlace se dio por perdido, o None si nunca se perdio.
        #:
        #: NO ES `_is_simulation`, y separarlos importa: «simulando» significa que las
        #: respuestas son inventadas, y «enlace perdido» que no hay respuestas. Poner
        #: `_is_simulation = True` ante una caida de transporte haria que el controlador
        #: EMPIECE a fabricar valores, que es peor que fallar.
        #:
        #: Es un latch a proposito: se levanta con la primera caida y solo lo baja una
        #: reconexion exitosa. Una lectura que ande despues de una caida no prueba que el
        #: enlace este bien —puede ser el siguiente intento el que falle— y para una
        #: calibracion de 47 puntos lo que importa es si el enlace se cayo EN ALGUN momento.
        self._enlace_perdido_motivo: str | None = None
        #: Cuando se intento recuperar el enlace por ultima vez. Arranca lo bastante atras
        #: para que el PRIMER intento no quede atrapado en el cooldown: con 0.0 y un
        #: `time.monotonic()` chico —una maquina recien arrancada— la resta daria menos que
        #: RECOVERY_COOLDOWN_S y la primera recuperacion no se intentaria.
        self._ultima_recuperacion_at = float("-inf")

        self.logger = logging.getLogger(self.__class__.__name__)
        self.retry_policy = RetryPolicy(base_delay=1.0, max_delay=10.0)

        self._check_simulation_status()
        self._start_background_loop()
        self.connect()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()
        return False

    # ------------------------------------------------------------------
    # Watchdog para detectar colgazos
    # ------------------------------------------------------------------

    def _watchdog_expired(self) -> None:
        """Callback cuando el watchdog expira - la operación se colgó."""
        super()._watchdog_expired()
        self._metrics.record_failure(is_ble_timeout=self.use_bluetooth, is_usb_timeout=not self.use_bluetooth)

    def _marcar_enlace_perdido(self, motivo: str) -> None:
        """Declara que el transporte se cayó. Latch: solo lo baja una reconexión exitosa.

        Existe porque hasta el 09-Sep-2026 una caída a mitad de corrida no dejaba rastro
        consultable: `_is_simulation` seguía en False y el estado en CONNECTED, así que
        `get_health_report()["is_simulation"] is False` — que es lo que los gates de
        calibración exigen — **seguía dejando pasar**. Medido con el puerto devolviendo
        OSError: `set_gain` levantaba, `get_gain` devolvía None, y el gate decía que sí.
        """
        if self._enlace_perdido_motivo is None:
            self.logger.error(
                "VLAD: ENLACE PERDIDO (%s). Todo lo que se mida desde aca no describe al "
                "equipo hasta que se reconecte", motivo,
            )
        self._enlace_perdido_motivo = motivo
        self._state = InstrumentState.ERROR

    def datos_confiables(self) -> tuple[bool, str]:
        """(se puede confiar en lo que este controlador devuelve, por qué no).

        Es la pregunta que un gate de calibración quiere hacer, y que `is_simulation` sola
        no contesta: son TRES condiciones distintas y cada una invalida una medición por su
        cuenta. Devuelve el motivo para que quien bloquee pueda decir por qué.
        """
        if self._is_simulation:
            return False, "el VLAD esta SIMULANDO: cada valor es inventado"
        # EL ORDEN IMPORTA, y este es el que da el motivo MAS INFORMATIVO primero.
        #
        # La causa de la caida va antes que «no hay transporte»: una recuperacion fallida
        # deja el transporte cerrado —`cerrar_transporte()` lo pone en None— asi que las dos
        # condiciones son verdaderas a la vez, y la util es la que dice POR QUE se cayo. Con
        # el orden inverso, un enlace que se corto por un [Errno 5] se reportaba como «la
        # conexion nunca se establecio», que manda a revisar la configuracion en vez del cable.
        if self._enlace_perdido_motivo is not None:
            return False, f"el enlace se perdio ({self._enlace_perdido_motivo})"
        # NI SIMULANDO NI CONECTADO es un estado real, y decia que si.
        #
        # `connect()` cae a simulacion cuando se agotan los reintentos, pero su rama
        # `except Exception` NO pone el flag: sale con `_is_simulation` en False y sin
        # transporte. Ahi `_send_command` no toma la rama de simulacion —cae hasta el
        # `return False` del final— asi que no fabrica valores, pero las tres preguntas de
        # abajo daban «confiable» sobre un objeto que no puede hablar con nada.
        transporte = self._ble_client if self.use_bluetooth else self._serial_connection
        if transporte is None:
            return False, ("no hay transporte abierto y tampoco esta en simulacion: "
                           "la conexion nunca se establecio")
        if self._breaker.state != "closed":
            return False, f"el circuit breaker esta {self._breaker.state}"
        return True, ""

    def is_healthy(self) -> bool:
        """Retorna True si el controlador está en estado saludable."""
        return (
            self._state == InstrumentState.CONNECTED
            and not self._is_hung
            and not self._is_simulation
            and self._enlace_perdido_motivo is None
        )

    def reset_circuit_breaker(self) -> None:
        """Resetea el circuit breaker manualmente."""
        self._breaker.success()
        self._state = InstrumentState.CONNECTED
        self.logger.info("VLAD: Circuit breaker manually reset")

    def get_health_report(self) -> dict:
        """Retorna un reporte de salud del controlador."""
        confiable, motivo = self.datos_confiables()
        return {
            "device_name": self._name,
            "state": self._state.name,
            "is_simulation": self._is_simulation,
            #: LO QUE UN GATE TIENE QUE MIRAR. `is_simulation` sola contesta una de las tres
            #: causas por las que un valor de este controlador no describe al equipo; las
            #: otras dos —enlace perdido, breaker abierto— dejaban pasar el gate.
            "datos_confiables": confiable,
            "motivo_no_confiable": motivo,
            "enlace_perdido": self._enlace_perdido_motivo,
            "is_hung": self._is_hung,
            "use_bluetooth": self.use_bluetooth,
            "circuit_breaker": self._breaker.state,
            "metrics": {
                "total_operations": self._metrics.total_operations,
                "successful": self._metrics.successful_operations,
                "failed": self._metrics.failed_operations,
                "ble_timeouts": self._metrics.ble_timeouts,
                "usb_timeouts": self._metrics.usb_timeouts,
                "connection_errors": self._metrics.connection_errors,
                "avg_latency_ms": round(self._metrics.avg_latency_ms, 2),
                # La cola, no la media: ver la nota de VladMetrics.
                "p95_latency_ms": round(self._metrics.p95_latency_ms, 2),
                "max_latency_ms": round(self._metrics.max_latency_ms, 2),
                "recovery_attempts": self._metrics.recovery_attempts,
                "recoveries_ok": self._metrics.recoveries_ok,
            },
            "last_success_at": self._metrics.last_success_at,
            "last_failure_at": self._metrics.last_failure_at,
        }

    # ------------------------------------------------------------------
    # Configuration Commands (Dual Protocol Wrapper)
    # ------------------------------------------------------------------

    #: Cuanto se espera entre dos intentos de recuperacion del enlace.
    #:
    #: Existe para que una corrida contra un equipo que no esta no se convierta en una
    #: tormenta de reconexiones: sin esto, los 47 puntos de una calibracion dispararian 47
    #: intentos, y cada uno por BLE cuesta un barrido de 15 s mas la conexion.
    RECOVERY_COOLDOWN_S = 5.0

    def _intentar_recuperar(self) -> bool:
        """Suelta el transporte y lo vuelve a tomar. Es la arista que le faltaba a ERROR.

        `InstrumentState` declara `ERROR -> RECOVERING -> CONNECTING -> ...` desde que se
        escribio el enum, y este controlador no usaba RECOVERING: ERROR era terminal y los
        reintentos con backoff existian **solo al conectar**. O sea que una caida a mitad de
        corrida no se recuperaba nunca, aunque el equipo volviera un segundo despues.

        Tres guardas, y cada una evita un modo de falla distinto:

        - en simulacion no se intenta: no hay nada que reabrir.
        - un cooldown, para no convertir una corrida larga en una tormenta de reconexiones.
        - `caer_a_simulacion=False`, para que un intento fallido deje el equipo declarado
          roto en vez de simulado. Sin eso la recuperacion seria peor que no tenerla: los
          setters empezarian a devolver True sin escribir.

        Devuelve True solo si el transporte quedo abierto de nuevo.
        """
        if self._is_simulation:
            return False
        ahora = time.monotonic()
        if ahora - self._ultima_recuperacion_at < self.RECOVERY_COOLDOWN_S:
            self.logger.debug(
                "VLAD: recuperacion en cooldown (%.1f s de %.1f)",
                ahora - self._ultima_recuperacion_at, self.RECOVERY_COOLDOWN_S,
            )
            return False
        self._ultima_recuperacion_at = ahora

        self._state = InstrumentState.RECOVERING
        self.logger.warning(
            "VLAD: recuperando el enlace (venia perdido por: %s)...",
            self._enlace_perdido_motivo,
        )
        try:
            self.cerrar_transporte()
            ok = self.connect(caer_a_simulacion=False)
        except Exception as exc:
            self.logger.error("VLAD: la recuperacion levanto %s: %s", type(exc).__name__, exc)
            ok = False
        self._metrics.record_recovery(exito=ok)
        if ok:
            self.logger.warning("VLAD: enlace recuperado tras %d intentos totales",
                                self._metrics.recovery_attempts)
        else:
            self.logger.error("VLAD: no se pudo recuperar el enlace")
        return ok

    def _intentar_una_vez(self, what: str, ejecutar, fallo):
        """Corre `ejecutar()` y traduce CUALQUIER excepcion del transporte a `fallo`.

        No toca el breaker: de eso se encarga `_operar()` una sola vez por operacion, para
        que un reintento tras recuperar no cuente dos veces.
        """
        try:
            return ejecutar()
        except Exception as exc:
            # `_send_command` atrapa OSError y AttributeError por dentro, y esa lista no
            # alcanza: `_run_async()` levanta RuntimeError cuando el loop de BLE no esta
            # corriendo, y eso se escapaba hasta la suite de calibracion, que solo mira
            # `if not ok`. Lo encontro la inyeccion de caos con un TimeoutError, que tampoco
            # estaba en la lista.
            #
            # Enumerar tipos es perder. Lo que hace robusto al controlador es que un fallo
            # DESCONOCIDO degrade igual que uno conocido, en vez de abortar la corrida.
            self._marcar_enlace_perdido(f"{what}: {type(exc).__name__}: {exc}")
            self._metrics.record_failure(is_connection=True)
            return fallo

    def _operar(self, cmd: int, what: str, ejecutar, fallo):
        """Unico camino de toda operacion de transporte: breaker, recuperacion y registro.

        EL RESULTADO SE LE INFORMA AL BREAKER A MANO, y eso es el arreglo del 09-Sep-2026.
        Antes cada llamador hacia `with self._breaker.guard()`, que decide por `exc_type`:
        un NACK del firmware o una lectura que no llega salen del bloque **sin excepcion**,
        asi que el guard los contaba como exito y ademas `success()` reseteaba el contador.
        Medido: seis NACK seguidos dejaban el breaker en `closed` con `_failures == 0`
        mientras las metricas contaban seis fallos.

        Y SOLO SE RECUPERA SI EL ENLACE SE CAYO. Un NACK significa que el enlace anda y que
        el firmware rechazo el comando: reconectar ahi no arregla nada y agrega un barrido
        de BLE por cada comando rechazado. La diferencia la da `_enlace_perdido_motivo`,
        que solo se levanta ante un fallo de transporte.
        """
        try:
            self._breaker.check()
        except CircuitBreakerOpen as exc:
            self.logger.warning("VLAD: Circuit breaker open for %s: %s", what, exc)
            return fallo

        resultado = self._intentar_una_vez(what, ejecutar, fallo)

        se_cayo_el_enlace = resultado == fallo and self._enlace_perdido_motivo is not None
        if se_cayo_el_enlace and self._intentar_recuperar():
            resultado = self._intentar_una_vez(what, ejecutar, fallo)

        if resultado == fallo:
            self._breaker.failure()
            self.logger.warning(
                "VLAD: %s fallo (cmd=0x%02X); fallos consecutivos del breaker: %d/%d",
                what, cmd, self._breaker._failures, self._breaker.threshold,
            )
        else:
            self._breaker.success()
        return resultado

    def _guarded_send(self, cmd: int, payload: bytes, what: str) -> bool:
        """Envia un comando. False si fallo, si el breaker esta abierto o si no se recupero."""
        return self._operar(cmd, what, lambda: self._send_command(cmd, payload), False)

    def _guarded_query(self, cmd: int, what: str) -> bytes:
        """Consulta. b"" si fallo, si el breaker esta abierto o si no se recupero.

        Reemplaza seis copias del mismo bloque —`get_gain`, `get_agc`, `get_health_word`,
        `get_agc_sense`, `get_atten_parts` y `get_smartring`, que ademas no pasaba por el
        breaker en absoluto—.
        """
        return self._operar(cmd, what, lambda: self._query_command(cmd), b"")

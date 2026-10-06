"""
Mixin de conexión BLE/USB para el controlador VLAD.

Proporciona la gestión de conexión híbrida (Bluetooth BLE y USB/Serial)
como mixin para VladController. El mixin asume que la clase anfitriona
define los atributos de estado necesarios (use_bluetooth, _is_simulation,
logger, _state, _metrics, _breaker, etc.).
"""

from __future__ import annotations

import asyncio
import threading
import time

try:
    import serial
except ImportError:
    serial = None

try:
    from bleak import BleakClient, BleakScanner
except ImportError:
    BleakClient = None
    BleakScanner = None

from controller.base.reliability import InstrumentState


class VladConnectionMixin:
    """Mixin que provee gestión de conexión híbrida BLE/USB para VladController.

    La clase anfitriona debe definir al menos:
    - use_bluetooth, ble_mac_address, usb_device_path, operation_timeout
    - _is_simulation, _state, _metrics, _breaker, _lock, logger
    - _start_watchdog(), _stop_watchdog()
    """

    # ------------------------------------------------------------------
    # Simulation Detection
    # ------------------------------------------------------------------

    def _check_simulation_status(self) -> None:
        """Evaluate if we need to fall back to simulation mode due to missing dependencies."""
        if self.use_bluetooth and (BleakClient is None or BleakScanner is None):
            self.logger.warning("VLAD: bleak is not installed. Falling back to Simulation Mode.")
            self._is_simulation = True
            return

        if not self.use_bluetooth and (serial is None):
            self.logger.warning("VLAD: pyserial is not installed. Falling back to Simulation Mode.")
            self._is_simulation = True
            return

        # SIN RUTA NO SE ABRE NINGUN DISPOSITIVO. Ver la nota de `usb_device_path` en
        # vlad_controller.py: el default era `/dev/ttyUSB0`, o sea que un llamador que no
        # decia a que puerto hablar terminaba hablandole a lo que hubiera en ese nodo.
        #
        # No saber el puerto es una razon legitima para simular, y se avisa fuerte: en
        # simulacion los setters devuelven True sin escribir nada, asi que quien mida algo
        # tiene que ver este aviso en el log.
        if not self.use_bluetooth and not self.usb_device_path:
            self.logger.warning(
                "VLAD: no se paso usb_device_path ni device_path. Simulation Mode — NO se "
                "abre ningun dispositivo y los setters van a devolver True sin escribir "
                "nada. Si esto era una prueba de hardware, el resultado NO mide nada."
            )
            self._is_simulation = True

    # ------------------------------------------------------------------
    # Background Thread Event Loop (Sync/Async Bridge)
    # ------------------------------------------------------------------

    def _start_background_loop(self) -> None:
        """Boot one single, persistent background event loop to manage all BLE actions."""
        if self._is_simulation or not self.use_bluetooth:
            return

        try:
            self._loop = asyncio.new_event_loop()

            def run_loop(loop: asyncio.AbstractEventLoop):
                asyncio.set_event_loop(loop)
                loop.run_forever()

            self._loop_thread = threading.Thread(
                target=run_loop,
                args=(self._loop,),
                daemon=True,
                name="VladController_BLE_Loop"
            )
            self._loop_thread.start()
            self.logger.debug("VLAD: Background event loop thread started successfully")
        except Exception as exc:
            self.logger.error("VLAD: Failed to start background event loop: %s. Falling back to Simulation.", exc)
            self._is_simulation = True

    def _run_async(self, coro, timeout: float | None = None):
        """Dispatch a coroutine safely to the isolated background thread loop and await its result."""
        if self._is_simulation or not self._loop:
            raise RuntimeError("Cannot execute async operation, loop is not running")
        effective_timeout = timeout or self.operation_timeout
        future = asyncio.run_coroutine_threadsafe(coro, self._loop)
        return future.result(timeout=effective_timeout)

    # ------------------------------------------------------------------
    # Connection Management (Híbrido)
    # ------------------------------------------------------------------

    def cerrar_transporte(self) -> None:
        """Cierra el canal PERO deja vivo el loop de asyncio. Para poder reconectar.

        `disconnect()` no sirve para esto: además del canal, detiene el hilo del event loop
        y lo pone en None, y `_run_async()` se niega a ejecutar sin loop. O sea que después
        de un `disconnect()` el camino BLE queda inutilizable y un `connect()` posterior
        falla con «Cannot execute async operation» — que suena a loop muerto y lo está,
        porque lo matamos nosotros.

        Esta es la mitad que hace falta para recuperarse de una caída: soltar el canal y
        volver a tomarlo, sin desarmar el puente sync/async que cuesta un hilo levantar.
        """
        if self._ble_client is not None:
            try:
                async def do_disconnect():
                    if self._ble_client.is_connected:
                        await self._ble_client.disconnect()
                self._run_async(do_disconnect(), timeout=3.0)
            except Exception as exc:
                # Un cierre que falla no impide reabrir, y de hecho es lo esperable cuando
                # el equipo ya no está: se registra y se sigue.
                self.logger.debug("VLAD: cierre de BLE durante recuperacion: %s", exc)
            finally:
                self._ble_client = None

        if self._serial_connection is not None:
            try:
                self._serial_connection.close()
            except OSError as exc:
                self.logger.debug("VLAD: cierre de serial durante recuperacion: %s", exc)
            finally:
                self._serial_connection = None

    def connect(self, caer_a_simulacion: bool = True) -> bool:
        """Establece la conexión por el transporte configurado, con reintentos.

        `caer_a_simulacion=False` es para la RECUPERACIÓN, y no es un detalle: con el
        default, agotar los reintentos pone `_is_simulation = True`, y entonces todos los
        setters empiezan a devolver True sin escribir nada. Eso está bien al arrancar —el
        orquestador no se traba— y está MAL a mitad de una corrida: convertiría una caída
        de enlace en un simulador que afirma haber aplicado lo que no aplicó.

        Un intento de recuperación que falla tiene que dejar el equipo declarado como roto,
        no como simulado.
        """
        if self._is_simulation:
            self.logger.info("VLAD (Simulation): Híbrido connection established (Mock)")
            self._state = InstrumentState.CONNECTED
            return True

        with self._lock:
            self._state = InstrumentState.CONNECTING
        # El watchdog tiene que cubrir TODOS los reintentos del barrido mas la
        # conexion. Con 10 s fijos expiraba a mitad de un barrido legitimo de 15 s y
        # marcaba el instrumento colgado cuando no lo estaba.
        self._start_watchdog(
            timeout=(self.BLE_SCAN_TIMEOUT_S + self.BLE_CONNECT_TIMEOUT_S + 5.0)
                    * self.MAX_RECONNECT_ATTEMPTS
            if self.use_bluetooth else 10.0
        )

        try:
            if self.use_bluetooth:
                result = self._connect_with_retry("ble")
            else:
                result = self._connect_with_retry("usb")

            with self._lock:
                if result:
                    self._state = InstrumentState.CONNECTED
                    self._breaker.success()
                    # Una reconexion exitosa es lo UNICO que baja el latch de enlace
                    # perdido. Se hace aca y no en _connect_usb/_connect_ble porque esos
                    # corren por intento y este es el punto donde la conexion ya quedo.
                    if getattr(self, "_enlace_perdido_motivo", None) is not None:
                        self.logger.info(
                            "VLAD: enlace recuperado (venia perdido por: %s)",
                            self._enlace_perdido_motivo,
                        )
                        self._enlace_perdido_motivo = None
                    return True
                self._state = InstrumentState.ERROR
                self._metrics.record_failure(is_connection=True)
                if caer_a_simulacion:
                    # Recien aca, con los reintentos agotados, se cae a simulacion.
                    self._is_simulation = True
                    self.logger.error(
                        "VLAD: sin conexion tras %d intentos; Simulation Mode. "
                        "Si esto era una prueba de hardware, el resultado NO mide nada.",
                        self.MAX_RECONNECT_ATTEMPTS,
                    )
                else:
                    self.logger.error(
                        "VLAD: sin conexion tras %d intentos. NO se cae a simulacion: el "
                        "equipo queda declarado roto, que es lo que hay. Ver la nota de "
                        "caer_a_simulacion.", self.MAX_RECONNECT_ATTEMPTS,
                    )
                return False
        except Exception as exc:
            with self._lock:
                self._state = InstrumentState.ERROR
            self._metrics.record_failure(is_connection=True)
            self.logger.error("VLAD: Connection failed: %s", exc)
            return False
        finally:
            self._stop_watchdog()

    def _connect_with_retry(self, transport: str) -> bool:
        """Connect via ``transport`` ('ble' or 'usb') with exponential backoff."""
        connect_fn = self._connect_ble if transport == "ble" else self._connect_usb
        label = "BLE" if transport == "ble" else "USB"
        for attempt in range(1, self.MAX_RECONNECT_ATTEMPTS + 1):
            if connect_fn():
                return True
            if attempt < self.MAX_RECONNECT_ATTEMPTS:
                delay = self.retry_policy.delay(attempt)
                self.logger.warning(
                    "VLAD: %s connection retry %d/%d in %.1fs",
                    label, attempt, self.MAX_RECONNECT_ATTEMPTS, delay,
                )
                time.sleep(delay)
        return False

    # Cuanto se busca el equipo antes de conectar, y cuanto se espera la conexion.
    #
    # Se DESCUBRE primero y se conecta despues. Pasar la MAC directa a BleakClient
    # obliga a bleak a descubrir por dentro, y en un banco con una docena de
    # dispositivos a la vista eso no cabe en 10 s: el timeout disparaba a los 13 s y el
    # controlador caia a Simulation Mode, que es la peor forma de fallar porque un test
    # de hardware sigue corriendo y no mide nada.
    BLE_SCAN_TIMEOUT_S = 15.0
    BLE_CONNECT_TIMEOUT_S = 20.0

    #: Cuanto se barre para decidir si el adaptador del host esta vivo. Corto a proposito:
    #: no busca nada, solo cuenta. En un banco con decenas de dispositivos a la vista,
    #: tres segundos alcanzan de sobra para ver el primero.
    BLE_SONDA_ADAPTADOR_S = 3.0

    async def _motivo_de_barrido_vacio(self) -> str:
        """Por qué el barrido no encontró al equipo — MIDIENDO, no suponiendo.

        NO CULPAR AL EQUIPO SIN HABERLO DISTINGUIDO, y este mensaje lo hacía: decía «el
        equipo no está anunciando o está fuera de alcance», que manda a revisar la placa.

        Medido el 09-Sep-2026 en la `.202`: tras cada sesión BLE el adaptador del host queda
        sin poder barrer. `BleakScanner.discover()` devolvía **0 dispositivos** —en un banco
        con 60 a la vista— y volvía a 60 después de `bluetoothctl power off/on`. Mientras
        tanto el firmware informaba `advertising=1` con los cinco contadores de fallo en 0:
        la placa nunca dejó de anunciar. El mensaje apuntaba al lado equivocado.

        Los dos casos se distinguen con una pregunta que el propio barrido contesta: ¿se ve
        ALGÚN dispositivo? En un banco de laboratorio la respuesta es siempre sí, así que un
        cero no dice «no hay nadie», dice «no estoy oyendo».

        Es la misma disciplina que `identificar_vlad.py`: antes de creer un cero, preguntar
        si el instrumento puede ver algo.
        """
        try:
            otros = await BleakScanner.discover(timeout=self.BLE_SONDA_ADAPTADOR_S)
        except Exception as exc:
            return (f"{self.ble_mac_address} no aparecio en "
                    f"{self.BLE_SCAN_TIMEOUT_S:.0f} s, y el barrido de control tampoco pudo "
                    f"correr ({type(exc).__name__}: {exc}): el adaptador del host esta en un "
                    f"estado que no permite barrer")
        if not otros:
            return (f"{self.ble_mac_address} no aparecio en "
                    f"{self.BLE_SCAN_TIMEOUT_S:.0f} s, y un barrido de control NO VE NINGUN "
                    f"dispositivo BLE. EL PROBLEMA ES EL ADAPTADOR DEL HOST, no el equipo: "
                    f"en un banco siempre hay algo anunciando. Ciclar el adaptador "
                    f"(`bluetoothctl power off` / `power on`) y repetir. Verificado el "
                    f"09-Sep-2026: 0 dispositivos tras una sesion BLE, 60 despues de ciclarlo, "
                    f"con la placa informando advertising=1 todo el tiempo")
        return (f"{self.ble_mac_address} no aparecio en "
                f"{self.BLE_SCAN_TIMEOUT_S:.0f} s de barrido, y el adaptador SI funciona "
                f"—ve {len(otros)} dispositivos—, asi que esta vez si es el equipo: no esta "
                f"anunciando o esta fuera de alcance. Confirmarlo por USB con QUERY_BLE_STATE "
                f"(0x46), que informa advertising y los contadores del modulo")

    def _connect_ble(self) -> bool:
        """Connect to VLAD Bluetooth Low Energy interface."""
        self.logger.info("VLAD: Connecting via Bluetooth BLE to %s...", self.ble_mac_address)
        try:
            async def do_connect():
                # find_device_by_address deja el equipo en el cache de BlueZ, que es lo
                # que hace fiable el connect siguiente. Devolver None aca es un dato
                # util —el equipo no esta anunciando— y no el mismo error que un
                # connect fallido.
                device = await BleakScanner.find_device_by_address(
                    self.ble_mac_address, timeout=self.BLE_SCAN_TIMEOUT_S,
                )
                if device is None:
                    raise OSError(await self._motivo_de_barrido_vacio())
                self._ble_client = BleakClient(device)
                await self._ble_client.connect()
                # Se devuelve is_connected y NO lo que devuelve connect().
                #
                # En la version de bleak del contenedor connect() devuelve None aunque
                # la conexion quede establecida —medido: "connect() devolvio None |
                # is_connected True", con el servicio NUS ya resuelto—. El chequeo
                # anterior era `if connected:`, asi que trataba TODA conexion exitosa
                # como fallo, y este camino no podia funcionar con ningun firmware. Los
                # comentarios que culpaban al firmware por "no registrar servicio GATT"
                # apuntaban al lado equivocado.
                return bool(self._ble_client.is_connected)

            connected = self._run_async(
                do_connect(),
                timeout=self.BLE_SCAN_TIMEOUT_S + self.BLE_CONNECT_TIMEOUT_S,
            )
            if connected:
                self.logger.info("VLAD: Bluetooth BLE connected successfully!")
                return True
            # Un falsy sin excepcion tambien se dice: antes este caso no logueaba nada
            # y el intento fallaba en silencio.
            self.logger.error("VLAD: BLE connect no establecio la conexion (sin excepcion)")
        except Exception as exc:
            self.logger.error("VLAD: Bluetooth BLE connection failed: %s", exc)

        # NO se pone _is_simulation aca.
        #
        # _run_async() se niega a ejecutar si ese flag esta puesto, asi que ponerlo en
        # un intento fallido mataba los reintentos que existen justamente para
        # recuperarse: el segundo intento fallaba con "Cannot execute async operation,
        # loop is not running" —que suena a loop muerto y no lo era— y el tercero
        # tampoco corria. La decision de caer a simulacion es del llamador, DESPUES de
        # agotar los reintentos.
        return False

    def _connect_usb(self) -> bool:
        """Connect to VLAD USB/Serial interface."""
        self.logger.info("VLAD: Connecting via USB/Serial to %s...", self.usb_device_path)
        try:
            self._serial_connection = serial.Serial(
                port=self.usb_device_path,
                baudrate=115200,
                timeout=1.0,
            )
            if self._serial_connection.is_open:
                # Vacia lo que haya quedado en el buffer ANTES de este connect — mismo
                # motivo que OwonAnalyzerUsb._do_connect() ("el OWON contamina el
                # buffer si un comando anterior quedo sin leer"): un byte viejo ahi
                # corre el eco que el protocolo espera encontrar en una ventana fija,
                # y toda lectura posterior falla igual. Confirmado en banco
                # (09-Sep-2026): get_gain/get_agc/get_agc_sense/get_health_word
                # fallaban las CUATRO con "no aparecio el eco" apenas despues de
                # conectar por primera vez — la conexion en si ya era real, no
                # simulada. Se DRENA hasta que el buffer quede quieto (abajo), no un
                # solo vaciado: da tiempo a que llegue lo que este en vuelo y vacia el
                # backlog entero, en vez de vaciar un buffer que todavia esta a medio llenar.
                #
                # DOS GUARDAS PARA UN INVARIANTE, Y LAS DOS SE QUEDAN. El mismo defecto se
                # encontro el mismo dia por dos caminos: aca al conectar, y en
                # `_leer_consulta_limpia`, que llama a `_limpiar_entrada()` antes de CADA
                # escritura. La de por-lectura subsume a esta, asi que esta no es necesaria
                # para la correccion — se queda por lo que MIDE: `in_waiting` en este
                # instante es el unico lugar donde se ve que la placa venia hablando desde
                # antes de que la abrieramos, y ese conteo no aparece en ninguna otra parte.
                #
                # El drenaje en si se delega en `_drenar_hasta_quieto()` (VladTransportMixin, la
                # misma clase) y no se reimplementa: ese helper tolera `_serial_connection`
                # en None y captura OSError, que un `reset_input_buffer()` pelado no hace.
                # Una segunda copia del vaciado derivaria sin que nada lo reporte.
                # DRENAR HASTA QUE EL BUFFER QUEDE QUIETO, no un solo vaciado: el backlog de log
                # (medido 1534 B) no se cubre con un `reset_input_buffer()` porque el firmware
                # sigue emitiendo mientras se vacia, y la PRIMERA consulta caia en ese backlog
                # —«el primer comando no hace nada y el siguiente si»—. `_drenar_hasta_quieto()`
                # lee hasta 150 ms sin datos, aprovechando que el log es periodico. Ver su nota.
                descartados = self._drenar_hasta_quieto()
                if descartados:
                    self.logger.warning(
                        "VLAD: se drenaron %d bytes de log/residuales al conectar "
                        "(hasta buffer quieto)", descartados,
                    )
                self.logger.info("VLAD: USB/Serial connected successfully!")
                return True
        except OSError as exc:
            self.logger.error("VLAD: USB/Serial connection failed: %s", exc)

        # NO se pone _is_simulation aca — misma leccion que _connect_ble, que ya la tenia
        # escrita treinta lineas mas arriba mientras este camino la incumplia.
        #
        # LO QUE PASABA (medido el 09-Sep-2026): el primer intento fallaba y dejaba el flag
        # puesto; el segundo abria el puerto bien y devolvia True, pero NADIE resetea el flag.
        # Resultado: `connect()` informaba CONNECTED, `_serial_connection` era un puerto
        # abierto de verdad, y `_is_simulation` seguia en True — asi que cada `_send_command`
        # tomaba la rama de simulacion. `set_gain()` devolvia True con CERO escrituras al
        # puerto y `get_gain()` devolvia el valor cacheado. El controlador afirmaba haber
        # aplicado lo que no aplico, sobre una conexion que funcionaba.
        #
        # La decision de caer a simulacion es del llamador, DESPUES de agotar los reintentos:
        # connect() ya lo hace, y ahi si es correcta.
        return False

    def disconnect(self) -> None:
        """Release connections cleanly."""
        self._stop_watchdog()

        if self._is_simulation:
            self.logger.info("VLAD (Simulation): Disconnected")
            self._state = InstrumentState.DISCONNECTED
            return

        if self._ble_client:
            try:
                async def do_disconnect():
                    if self._ble_client.is_connected:
                        await self._ble_client.disconnect()
                self._run_async(do_disconnect(), timeout=3.0)
            except OSError as exc:
                self.logger.warning("VLAD: error al desconectar BLE: %s", exc)
            finally:
                self._ble_client = None
                self.logger.info("VLAD: Bluetooth BLE disconnected")

        if self._loop and self._loop.is_running():
            try:
                self._loop.call_soon_threadsafe(self._loop.stop)
                if self._loop_thread:
                    self._loop_thread.join(timeout=2.0)
            except RuntimeError as exc:
                self.logger.warning("VLAD: error al detener loop thread: %s", exc)
            finally:
                self._loop = None
                self._loop_thread = None

        if self._serial_connection and self._serial_connection.is_open:
            try:
                self._serial_connection.close()
            except OSError as exc:
                self.logger.warning("VLAD: error al cerrar serial: %s", exc)
            finally:
                self._serial_connection = None
                self.logger.info("VLAD: USB/Serial disconnected")

        self._state = InstrumentState.DISCONNECTED

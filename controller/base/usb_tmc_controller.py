"""
Base controller for Linux USB-TMC char devices (/dev/usbtmc*).

USB-TMC (USB Test and Measurement Class) devices appear as character devices
on Linux. This controller communicates via raw os.open / os.read / os.write
with select-based read timeout — no libusb or external libraries required.

Mecanismos de proteccion:
    - SCPI sanitizer rechaza comandos malformados antes del wire.
    - Rate-limit entre writes (Owon VSA815P necesita ~150ms entre comandos).
    - Threading lock serializa I/O entre callers concurrentes (per-instance).
    - Cross-instance device-path lock via ResourceRegistry evita que dos
      instancias abran el mismo /dev/usbtmc* simultaneamente (el driver
      del kernel no soporta acceso concurrente).
    - Non-blocking reads con timeout (evita cuelgues por endpoint congelado).

Jira: ID-1308 (Test Bench v2)
"""

from __future__ import annotations

import contextlib
import logging
import os
import select
import threading
import time

from service.resource_registry import get_registry

from controller.base.reliability import InstrumentState
from controller.base.scpi_sanitizer import sanitize


class UsbTmcController:
    """Controlador base para dispositivos USB-TMC.

    Usa os.open/os.read/os.write para comunicacion directa con
    /dev/usbtmc*. Las subclases pueden sobrescribir:

        RATE_LIMIT_DELAY: pausa post-write (default 0.15s para OWON).
        DEFAULT_READ_TIMEOUT: tiempo maximo de espera por respuesta.
    """

    DEFAULT_READ_TIMEOUT = 2.0
    DEFAULT_READ_SIZE = 4096
    RATE_LIMIT_DELAY = 0.15
    WRITE_RETRIES = 3
    AUTO_RECONNECT = True

    def __init__(self) -> None:
        self._fd: int | None = None
        self._device_path: str | None = None
        self._lock = threading.RLock()
        self._disconnected = True
        self._state = InstrumentState.DISCONNECTED
        self._reconnecting = False
        self._name = self.__class__.__name__
        self.logger = logging.getLogger(self._name)

    def __del__(self) -> None:
        """Cierre de ultimo recurso. NO cierra un descriptor que dejo de ser un dispositivo.

        Cerrar un fd desde __del__ es peligroso mas alla de los tests: el numero puede haber
        sido REUSADO por otra parte del proceso, y entonces se cierra algo ajeno.

        Paso, y el sintoma no se parecia a la causa: un test dejo el objeto "conectado" con un
        fd falso, __del__ cerro el fd real con ese numero —la tuberia de captura de pytest— y el
        suite entero se cortaba a mitad SIN resumen y SIN error. Parecia un cuelgue.

        La guarda es barata y precisa: un USB-TMC es un dispositivo de CARACTER. Si el fd sigue
        abierto pero ya no es un chardev, es de otro y no se toca. Una tuberia no pasa el filtro.
        """
        if not hasattr(self, "_fd") or self._fd is None:
            return
        if getattr(self, "_disconnected", True):
            return
        # Si el fd dejo de ser un dispositivo de caracter, se OLVIDA sin cerrarlo — pero el
        # cierre sigue su curso, porque _close_locked() tambien libera el candado del registro y
        # saltearlo dejaria el dispositivo trabado para la siguiente sesion.
        #
        # La primera version de esta guarda hacia `return` aca y rompio exactamente eso: los
        # controladores que quedan sin disconnect() dejaban de liberar su candado, y el test
        # siguiente fallaba con "locked by another session" tras 10 s de espera.
        try:
            import stat as _stat
            if not _stat.S_ISCHR(os.fstat(self._fd).st_mode):
                logger = getattr(self, "logger", None)
                if logger:
                    logger.warning(
                        "__del__: el fd %s ya no es un dispositivo de caracter; NO se cierra "
                        "(pudo ser reusado por otro). Falto un disconnect() explicito",
                        self._fd,
                    )
                self._fd = None
        except OSError:
            self._fd = None   # ya estaba cerrado: no se vuelve a cerrar, pero si se libera
        try:
            self._close_locked()
            logger = getattr(self, "logger", None)
            if logger:
                logger.debug("__del__ closed fd %s", self._fd)
        except OSError as exc:
            logger = getattr(self, "logger", None)
            if logger:
                logger.warning("__del__ close fd %s failed: %s", self._fd, exc)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()
        return False

    # ------------------------------------------------------------------
    # Conexion
    # ------------------------------------------------------------------

    def connect(self, device_path: str) -> None:
        with self._lock:
            if self._fd is not None:
                self.logger.warning(
                    "Already connected to %s — disconnecting first", self._device_path
                )
                self._close_locked()
            self._device_path = device_path
            # SE VALIDA EL DESTINO, NO EL NOMBRE.
            #
            # Esto comparaba el prefijo de la cadena, y por eso rechazaba `/dev/analyzer_owon`
            # —el nombre fijo que crea la propia regla udev del repo— mientras aceptaba
            # cualquier archivo que se llamara `/dev/usbtmcloquesea`. O sea: decia que no a lo
            # correcto y que si a lo arbitrario. Se encontro el 16-Sep-2026, al hacer que el
            # driver prefiriera el nombre estable: hasta entonces nadie le habia pasado un
            # symlink y la validacion nunca habia tenido que opinar sobre uno.
            #
            # `realpath` resuelve el symlink, asi que la proteccion queda intacta —el destino
            # tiene que ser un nodo usbtmc— y los nombres fijos pasan.
            if not os.path.realpath(device_path).startswith("/dev/usbtmc"):
                raise ValueError(
                    f"Invalid USB-TMC device path: '{device_path}'"
                    + (f" (apunta a {os.path.realpath(device_path)})"
                       if os.path.islink(device_path) else "")
                )
            # Acquire cross-instance device lock before opening
            registry = get_registry()
            if not registry.device_lock(device_path).acquire(timeout=10.0):
                raise RuntimeError(
                    f"USB-TMC device {device_path} is locked by another session"
                )
            try:
                self._fd = os.open(device_path, os.O_RDWR)
            except Exception:
                registry.device_lock(device_path).release()
                raise
            self._disconnected = False
            self._state = InstrumentState.CONNECTED
            self.logger.info("Connected to USB-TMC device: %s", device_path)

    def disconnect(self) -> None:
        with self._lock:
            self._close_locked()

    def _close_locked(self) -> None:
        if self._fd is not None:
            try:
                os.close(self._fd)
            except OSError as exc:
                self.logger.warning("Error closing %s: %s", self._device_path, exc)
            finally:
                self._fd = None
                self._disconnected = True
                self._state = InstrumentState.DISCONNECTED
                self.logger.info("Disconnected from %s", self._device_path)
        if self._device_path is not None:
            with contextlib.suppress(Exception):
                get_registry().device_lock(self._device_path).release()

    # ------------------------------------------------------------------
    # Flush — drenar buffer residual
    # ------------------------------------------------------------------

    def _flush_usb(self, hard: bool = False) -> None:
        """Drenar respuestas pendientes del buffer USB-TMC.

        poll() no vuelve antes cuando no hay nada que leer: espera el timeout
        completo. Si de verdad quedo una respuesta pendiente de una sesion
        anterior, ya esta sentada en el buffer del kernel — POLLIN dispara
        casi instantaneo apenas se registra, sin importar el timeout. Por eso
        NO hace falta un timeout largo ni siquiera en el caso "hard" (limpieza
        entre sesiones): un timeout largo solo agrega espera al caso comun
        (nada que drenar), que es la inmensa mayoria de las conexiones.
        Si hard=False: primer poll de 500ms (comando a comando, sin cambios).
        Si hard=True: primer poll de 300ms — sigue siendo ~3x el margen de un
        comando SCPI normal, de sobra para lo que ya esta en el buffer.
        Nunca os.read() bloqueante: eso era lo que se colgaba hasta el
        timeout del driver USBTMC del kernel (~5s) por conexion cuando no
        habia nada que drenar — tiempo muerto puro, no proteccion real.
        Corre bajo self._lock."""
        with self._lock:
            if self._fd is None:
                return
            try:
                poller = select.poll()
                poller.register(self._fd, select.POLLIN)
                ready = poller.poll(300 if hard else 500)
                while ready:
                    os.read(self._fd, 4096)
                    ready = poller.poll(100)
            except (OSError, ValueError) as exc:
                self.logger.debug("_flush_usb: %s", exc)

    # ------------------------------------------------------------------
    # I/O
    # ------------------------------------------------------------------

    def write(self, cmd: str) -> None:
        """Sanitiza, escribe en el wire con reintentos, y aplica rate-limit."""
        result = sanitize(cmd)
        if not result.ok:
            raise RuntimeError(
                f"USB-TMC: rejected SCPI command ({result.reason}): {cmd!r}"
            )
        sanitized = result.command
        assert sanitized is not None

        with self._lock:
            self._reabrir_si_un_timeout_lo_cerro()
            if self._fd is None:
                raise RuntimeError(
                    "UsbTmcController: not connected (call connect() first)"
                )
            payload = (sanitized + "\n").encode("ascii")
            self._write_with_retries(payload, sanitized)
            self.logger.debug("USB-TMC -> %s", sanitized)

        # RATE LIMIT: estaba DECLARADO y documentado ("pausa post-write, default 0.15s para
        # OWON") y no se aplicaba en ningun lado. Tres fuentes decian que existe —la constante,
        # el docstring de la clase y su test— y el codigo no lo hacia.
        #
        # No es cosmetico: los OWON por USB-TMC pierden comandos si llegan pegados, y sin la
        # pausa el sintoma es una lectura que no corresponde al comando anterior. Que la suite
        # de calibracion necesite un SWEEP_SETTLE_S propio es coherente con haber estado
        # compensando esto a mano.
        #
        # Fuera del `with self._lock`: la pausa no tiene por que retener el candado, y
        # retenerlo serializaria a otras instancias sin motivo.
        if self.RATE_LIMIT_DELAY > 0.0:
            time.sleep(self.RATE_LIMIT_DELAY)

    def _write_with_retries(self, payload: bytes, label: str) -> None:
        """Escribe con reintentos acotados; auto-reconecta si se agotan.

        Hasta ``WRITE_RETRIES`` intentos. Si se agotan y
        ``AUTO_RECONNECT`` está activo, reconecta y reintenta una vez más.
        """
        last_exc: OSError | None = None
        for attempt in range(1, self.WRITE_RETRIES + 1):
            try:
                os.write(self._fd, payload)
                return
            except OSError as exc:
                last_exc = exc
                self.logger.warning(
                    "USB-TMC write attempt %d/%d failed: %s",
                    attempt, self.WRITE_RETRIES, exc,
                )
        if self.AUTO_RECONNECT:
            self.logger.warning(
                "USB-TMC write exhausted %d retries — auto-reconnecting",
                self.WRITE_RETRIES,
            )
            try:
                self.reconnect()
                os.write(self._fd, payload)
                return
            except (OSError, RuntimeError) as exc:
                raise RuntimeError(f"USB-TMC write failed after reconnect: {exc}") from exc
        raise RuntimeError(
            f"USB-TMC write failed after {self.WRITE_RETRIES} retries: {last_exc}"
        ) from last_exc

    def _reabrir_si_un_timeout_lo_cerro(self) -> None:
        """Reabre el descriptor que cerro un timeout de lectura. Corre bajo `self._lock`.

        Un timeout cierra el `fd` a proposito: el instrumento quedo desincronizado y seguir
        leyendo del mismo descriptor devuelve la respuesta de otro comando —el OWON contamina
        el buffer—. Pero **nada lo reabria**, asi que la llamada siguiente moria con «not
        connected» y un timeout puntual dejaba el analizador inutilizable para el resto de la
        corrida. Medido en el banco el 20-Ago-2026, y ya estaba documentado de antes.

        Se reabre una sola vez y se avisa con WARNING: si el instrumento esta de verdad caido,
        el `os.open` falla y el error sale igual. Lo que ya no pasa es que un timeout se
        propague al resto de la sesion.

        Solo aplica si HUBO una conexion antes (`_device_path` seteado). Un controlador al que
        nunca se le llamo `connect()` sigue dando el error de siempre, porque ahi el problema
        es de uso y no del cable.
        """
        if self._fd is not None or self._device_path is None:
            return
        self.logger.warning(
            "USB-TMC: el descriptor de %s estaba cerrado (timeout previo) — reabriendo",
            self._device_path,
        )
        registry = get_registry()
        if not registry.device_lock(self._device_path).acquire(timeout=10.0):
            raise RuntimeError(
                f"USB-TMC: no se pudo retomar el candado de {self._device_path} para reabrir"
            )
        try:
            self._fd = os.open(self._device_path, os.O_RDWR)
        except OSError:
            with contextlib.suppress(Exception):
                registry.device_lock(self._device_path).release()
            raise
        self._disconnected = False
        self._state = InstrumentState.CONNECTED

    def reconnect(self) -> None:
        """Cierra el fd actual, re-escanea buscando el analizador por desafío y reabre."""
        with self._lock:
            if self._device_path is None:
                raise RuntimeError("USB-TMC: no device path known for reconnect")
            self._close_locked()
            from controller.base.device_identifier import DeviceIdentifier

            discovered = DeviceIdentifier.discover_device(
                "analyzer", "owon_usb", preferred_path=self._device_path
            )
            if discovered:
                self._device_path = discovered

            registry = get_registry()
            if not registry.device_lock(self._device_path).acquire(timeout=10.0):
                raise RuntimeError(
                    f"USB-TMC: no se pudo retomar el candado de {self._device_path} para reconnect"
                )
            try:
                self._fd = os.open(self._device_path, os.O_RDWR)
            except OSError:
                with contextlib.suppress(Exception):
                    registry.device_lock(self._device_path).release()
                raise
            self._disconnected = False
            self._state = InstrumentState.CONNECTED
            self.logger.info("USB-TMC reconnected to %s", self._device_path)

    def query(self, cmd: str, read_size: int = DEFAULT_READ_SIZE) -> str:
        """Envia un comando y retorna la respuesta."""
        with self._lock:
            self.write(cmd)
            return self._read_response(read_size)

    def _read_response(self, read_size: int) -> str:
        """Lee respuesta hasta newline con deadline.

        Usa os.read bloqueante directo. El deadline loop da el timeout
        real (2s). Si no hay datos, el kernel USB-TMC acaba retornando.
        """
        if self._fd is None:
            raise RuntimeError("UsbTmcController: not connected (call connect() first)")

        deadline = time.time() + self.DEFAULT_READ_TIMEOUT
        raw = b""
        while time.time() < deadline:
            try:
                chunk = os.read(self._fd, read_size)
            except OSError as exc:
                self.logger.warning("USB-TMC read transient error: %s", exc)
                time.sleep(0.05)
                continue
            if chunk:
                raw += chunk
                if b"\n" in raw:
                    break
            else:
                time.sleep(0.01)

        if not raw:
            self.logger.error(
                "USB-TMC read timeout (%ss) on %s — closing fd",
                self.DEFAULT_READ_TIMEOUT,
                self._device_path,
            )
            self._close_locked()
            raise RuntimeError(
                f"USB-TMC read timeout ({self.DEFAULT_READ_TIMEOUT}s) on {self._device_path}"
            )

        response = raw.decode("ascii", errors="replace").rstrip("\r\n")
        self.logger.debug("USB-TMC <- %s", response)
        return response

    # ------------------------------------------------------------------
    # Diagnostics
    # ------------------------------------------------------------------

    @property
    def is_connected(self) -> bool:
        return self._fd is not None

    @property
    def state(self) -> InstrumentState:
        return self._state

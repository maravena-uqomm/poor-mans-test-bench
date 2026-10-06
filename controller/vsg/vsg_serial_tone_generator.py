"""
Signal Hound VSG25A Vector Signal Generator Controller.

Two transport modes:
  1. Native API (libsg_api.so) — default, uses official shared library via ctypes.
  2. Direct serial (pyserial)   — raw binary protocol over /dev/ttyACM0.
     Use when the native API has bugs (v1.0.7 sgSetCW returns success but
     does not emit RF). Protocol commands decoded from USB bulk captures.

Mode selection: VSG25_TRANSPORT=native | serial (env var, default: native)

Jira: ID-1308 (Test Bench v2)
"""

from __future__ import annotations

import logging
import os
import threading
import time
import ctypes
from ctypes import CDLL, POINTER, c_char_p, c_double, c_int, pointer
from pathlib import Path

try:
    import serial as pyserial
    from serial import SerialException
    try:
        from serial import SerialTimeoutError
    except ImportError:
        SerialTimeoutError = OSError  # type: ignore[assignment]  # fallback for older pyserial versions
except ImportError:
    pyserial = None
    SerialException = OSError  # type: ignore[assignment]  # fallback when pyserial is not installed
    SerialTimeoutError = OSError  # type: ignore[assignment]  # fallback when pyserial is not installed

from controller.base.reliability import (
    ConnectionError as InstrumentConnectionError,
)
from controller.base.reliability import (
    HardwareError,
)
from controller.base.tone_generator import ToneGenerator
from controller.vsg.vsg_protocol import (
    FRAME_SIZE,
    OP_RF_OFF,
    OP_SET_CW,
    PacketBuilder,
    encode_freq_amp,
    parse_response,
)

#: El nombre fijo que crea `scripts/99-uqomm-testbench.rules` por VID (2817). Se prueba PRIMERO:
#: es el unico camino en el que el numero de ttyACM no puede morder.
NOMBRE_ESTABLE = "/dev/vsg25a"
#: Y el by-id, que el kernel arma solo con el product string — estable aunque falte la regla.
BY_ID = "/dev/serial/by-id/usb-Signal_Hound_VSG25A-if00"

#: ULTIMO RECURSO, Y ES UNA TRAMPA: `/dev/ttyACM0` acierta solo por el orden de enumeracion.
#: Medido el 16-Sep-2026 en la misma sesion: el VSG paso de ttyACM3 a ttyACM0 y el VLAD de
#: ttyACM0 a ttyACM3 con solo reenchufar. Apuntarle a un numero es apuntarle a otra placa.
DEFAULT_DEVICE = "/dev/ttyACM0"


def resolver_vsg(preferida: str | None = None) -> str:
    """La ruta del VSG25A: explicita si la hay; si no, nombre fijo, by-id, y el numero al final.

    UNA RUTA EXPLICITA GANA SIEMPRE, exista o no. La primera version caia al descubrimiento
    cuando la ruta pedida no existia, y eso es peor que fallar: un path mal escrito terminaba
    abriendo OTRO instrumento en silencio — exactamente la clase de error que esta funcion
    existe para cerrar. Si la ruta pedida no esta, el error tiene que hablar de ESA ruta.
    """
    import os as _os
    if preferida:
        return preferida
    for cand in (NOMBRE_ESTABLE, BY_ID):
        if _os.path.exists(cand):
            return cand
    return DEFAULT_DEVICE
SERIAL_WRITE_RETRIES = 3  # bounded retries for transient serial errors


class VsgSerialToneGenerator(ToneGenerator):
    """Controller for Signal Hound VSG25A.

    Two transports:
      - native: uses libsg_api.so via ctypes (default).
      - serial: opens /dev/ttyACM0 directly with pyserial + binary protocol.
    """

    def __init__(self, device_path: str | None = None):
        """Initialize the VSG25A controller.

        Args:
            device_path: Ruta explicita. Si no se pasa, se resuelve con `resolver_vsg()`:
                primero el nombre fijo de udev, despues el by-id, y el numero al final.
        """
        super().__init__()
        device_path = resolver_vsg(device_path)
        self._device_path = device_path
        self._handle: int | None = None
        #: Descriptores del nodo que existian antes de `sgOpenDevice()`. Lo que aparezca
        #: despues y sobreviva a `sgCloseDevice()` lo abandono el SDK.
        self._fds_antes_de_abrir: set[int] | None = None
        self._lib: CDLL | None = None
        self._serial = None
        self._transport = (os.getenv("VSG25_TRANSPORT") or "native").strip().lower()
        self._pkt = PacketBuilder(initial_seq=0x00)

        # Power and frequency cache
        self._frequency_mhz = 1000.0
        self._amplitude_dbm = -20.0
        self._rf_on = False
        self._lock = threading.RLock()

        self.logger = logging.getLogger(self.__class__.__name__)

        if self._transport == "serial":
            self.connect()
        else:
            self._load_library()
            self.connect()

    # ------------------------------------------------------------------
    # Library Loading and Connection
    # ------------------------------------------------------------------

    #: Rutas donde puede estar el shim de libudev, en orden. La primera es la del contenedor
    #: (la instala el Dockerfile) y la segunda la del repo, para correr fuera de docker.
    _SHIM_UDEV = (
        "/usr/local/lib/libudev_sh.so",
        str(Path(__file__).resolve().parents[2] / "lib" / "libudev_sh.so"),
    )

    def _cargar_shim_udev(self) -> None:
        """Carga el libudev compatible ANTES que la biblioteca del fabricante.

        ── POR QUE, Y POR QUE NO ALCANZA CON LD_PRELOAD ──────────────────────────────────

        `libsg_api.so` enlaza contra `libudev.so.1` y con la del sistema **aborta el proceso
        entero**, no falla: `Assertion 'p->n_ref > 0' failed at libudev-device.c:497,
        function udev_device_unref(). Aborting.` — o sea que el orquestador se muere, no
        degrada. Medido en esta maquina el 17-Sep-2026.

        El repo ya traia la solucion —`test_bench/lib/libudev_sh.so`, un libudev completo de
        95 simbolos— pero la aplicaba **solo por `LD_PRELOAD` en el Dockerfile**. Fuera de
        docker no la pone nadie, asi que el transporte nativo —que es el DEFAULT— mataba el
        proceso en cualquier maquina sin contenedor.

        Cargarlo aca con `RTLD_GLOBAL` hace lo mismo que el `LD_PRELOAD` y no depende de que
        alguien exporte una variable antes de arrancar: los simbolos del shim quedan
        disponibles para lo que se cargue despues. Verificado: sin `LD_PRELOAD`, con esto
        `sgOpenDevice()` devuelve 0. El `LD_PRELOAD` del Dockerfile puede quedarse — es
        idempotente.
        """
        for ruta in self._SHIM_UDEV:
            try:
                CDLL(ruta, mode=ctypes.RTLD_GLOBAL)
                self.logger.debug("VSG25A: shim de libudev cargado desde %s", ruta)
                return
            except OSError:
                continue
        # No es fatal por si mismo: con la libudev del sistema puede andar en otra distro.
        # Se avisa porque si despues el proceso MUERE, este es el motivo.
        self.logger.warning(
            "VSG25A: no se encontro libudev_sh.so. Si el proceso aborta al abrir el equipo "
            "con 'udev_device_unref' es por esto: la biblioteca del fabricante no tolera la "
            "libudev del sistema."
        )

    def _load_library(self) -> None:
        """Attempt to load the Signal Hound native library."""
        self._cargar_shim_udev()
        paths = [
            "/usr/local/lib/libsg_api.so",
            "libsg_api.so",
            "sg_api.dll",
        ]

        for path in paths:
            try:
                self._lib = CDLL(path)
                self.logger.info("VSG25A: Successfully loaded native library from %s", path)
                break
            except OSError:
                continue

        if not self._lib:
            raise HardwareError(
                "VSG25A: no se encontro libsg_api.so (probe en /usr/local/lib, '.' y sg_api.dll). "
                "Sin la biblioteca nativa no hay como hablarle al equipo — no se simula.",
                instrument_name="tone_generator",
            )

        # Setup ctypes function prototypes
        try:
            self._lib.sgOpenDevice.restype = c_int
            self._lib.sgOpenDevice.argtypes = [POINTER(c_int)]

            self._lib.sgCloseDevice.restype = c_int
            self._lib.sgCloseDevice.argtypes = [c_int]

            self._lib.sgGetSerialNumber.restype = c_int
            self._lib.sgGetSerialNumber.argtypes = [c_int, POINTER(c_int)]

            self._lib.sgSetFrequencyAmplitude.restype = c_int
            self._lib.sgSetFrequencyAmplitude.argtypes = [c_int, c_double, c_double]

            self._lib.sgSetCW.restype = c_int
            self._lib.sgSetCW.argtypes = [c_int]

            self._lib.sgRFOff.restype = c_int
            self._lib.sgRFOff.argtypes = [c_int]

            self._lib.sgGetStatusString.restype = c_char_p
            self._lib.sgGetStatusString.argtypes = [c_int]
        except AttributeError as exc:
            self._lib = None
            raise HardwareError(
                f"VSG25A: la biblioteca nativa no expone la API esperada: {exc}. "
                "No se simula: sin la API no hay forma de operar el equipo.",
                instrument_name="tone_generator",
            ) from exc

    def _connect_native(self) -> None:
        """Open connection via native libsg_api.so."""
        if self._handle is not None:
            self.logger.warning("VSG25A: Already connected (handle %d)", self._handle)
            return

        # El SDK deja abierto el nodo CDC en cada sgOpenDevice() y no lo cierra nunca. Se
        # anota que descriptores hay ANTES para poder cerrar los que abandone. Ver
        # `_cerrar_los_que_el_sdk_abandono`.
        self._fds_antes_de_abrir = self._descriptores_del_equipo()

        handle = c_int(-1)
        status = self._lib.sgOpenDevice(pointer(handle))
        if status == 0 and handle.value >= 0:
            self._handle = handle.value
            self.logger.info("VSG25A: Connected successfully with handle %d", self._handle)
            self._apply_settings()
        else:
            raise InstrumentConnectionError(
                f"VSG25A: sgOpenDevice fallo (status {status}). "
                "El equipo no esta conectado o no responde — no se simula.",
                instrument_name="tone_generator",
            )

    def _connect_serial(self) -> None:
        """Open serial device directly (bypass native API)."""
        if self._serial is not None:
            self.logger.warning("VSG25A: Already connected to %s", self._device_path)
            return
        try:
            self._serial = pyserial.Serial(
                self._device_path,
                baudrate=115200,
                timeout=3.0,
            )
            self._serial.reset_input_buffer()
            self._serial.reset_output_buffer()
            self.logger.info("VSG25A: Serial connected to %s", self._device_path)

            # Synchronize: send OPEN, expect 16-byte-payload response
            open_pkt = self._pkt.build_open()
            resp = self._send_raw(open_pkt, recv_len=FRAME_SIZE)
            if resp:
                parsed = parse_response(resp)
                self.logger.debug("VSG25A: OPEN response seq=0x%02x valid=%s payload=%s",
                                  parsed["seq"], parsed["valid"], parsed["payload"].hex())
            else:
                self._serial.close()
                self._serial = None
                raise InstrumentConnectionError(
                    "VSG25A: sin respuesta al OPEN — el equipo no esta en ese puerto o no "
                    "responde. No se simula.",
                    instrument_name="tone_generator",
                )
        except OSError as exc:
            self._serial = None
            raise InstrumentConnectionError(
                f"VSG25A: fallo al conectar por serial ({exc}). No se simula.",
                instrument_name="tone_generator",
            ) from exc

    def connect(self) -> None:
        """Public connect — dispatches to native or serial."""
        if self._transport == "serial":
            self._connect_serial()
        else:
            self._connect_native()

    def reconnect(self) -> None:
        """Cierra la conexión actual, redescubre por desafío y reconecta."""
        with self._lock:
            self.disconnect()
            from controller.base.device_identifier import DeviceIdentifier

            discovered = DeviceIdentifier.discover_device(
                "tone_generator", "vsg_serial", preferred_path=self._device_path
            )
            if discovered:
                self._device_path = discovered
            self.connect()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        try:
            self.stop()
        except Exception:
            self.logger.warning("VSG25A: stop() failed during __exit__")
        self.disconnect()
        return False

    def disconnect(self) -> None:
        """Close connection to the VSG25A device."""
        if self._transport == "serial":
            with self._lock:
                if self._serial:
                    try:
                        close_pkt = self._pkt.build_close()
                        self._send_raw(close_pkt)
                    except OSError as exc:
                        self.logger.debug("VSG25A: close packet send failed: %s", exc)
                    self._serial.close()
                    self._serial = None
                    self.logger.info("VSG25A: Serial disconnected")
            return

        if self._handle is not None:
            with self._lock:
                estado = None
                try:
                    self._lib.sgRFOff(self._handle)
                    estado = self._lib.sgCloseDevice(self._handle)
                except Exception:
                    self.logger.warning("VSG25A: error during native disconnect", exc_info=True)
                finally:
                    self._handle = None
                abandonados = self._cerrar_los_que_el_sdk_abandono()
            # El status se MIRA. Antes se descartaba y la linea de abajo se imprimia igual,
            # asi que el log decia "handle closed" sin haber comprobado nada.
            if estado not in (0, None):
                self.logger.warning("VSG25A: sgCloseDevice devolvio %s", estado)
            self.logger.info(
                "VSG25A: Disconnected and handle closed (status=%s, descriptores recuperados=%d)",
                estado, abandonados,
            )

    # ------------------------------------------------------------------
    # Descriptores que el SDK abandona
    # ------------------------------------------------------------------

    def _descriptores_del_equipo(self) -> set[int]:
        """Descriptores del proceso que apuntan al nodo de ESTE equipo.

        Se comparan por `st_rdev` —el par major:minor del nodo— y no por el nombre del
        enlace en `/proc/self/fd`: el mismo dispositivo se llama distinto adentro y afuera
        del contenedor, y el nombre puede cambiar entre enumeraciones del kernel.

        Devuelve un conjunto vacio donde no haya `/proc` (macOS, Windows): el barrido es
        una contencion oportunista, no puede ser un requisito para arrancar.
        """
        try:
            rdev = os.stat(self._device_path).st_rdev
        except OSError:
            return set()

        encontrados: set[int] = set()
        try:
            candidatos = os.listdir("/proc/self/fd")
        except OSError:
            return set()

        for entrada in candidatos:
            try:
                fd = int(entrada)
            except ValueError:
                continue
            try:
                if os.fstat(fd).st_rdev == rdev:
                    encontrados.add(fd)
            except OSError:
                # El descriptor se cerro entre el listdir y el fstat, o es el del listdir.
                continue
        return encontrados

    def _cerrar_los_que_el_sdk_abandono(self) -> int:
        """Cierra los descriptores que aparecieron durante la conexion y sobrevivieron.

        Por que existe: `libsg_api.so` abre el nodo CDC en cada `sgOpenDevice()` —de solo
        lectura, para enumerar— y `sgCloseDevice()` no lo cierra. La comunicacion real va
        por libusb, asi que el equipo funciona y nada falla... hasta que el proceso pasa
        los 1024 descriptores. Ahi `select()` deja de poder representarlos y **cualquier**
        puerto serie que se abra despues muere con `filedescriptor out of range`.

        Eso fue lo que se vio en el banco el 26-Ago-2026: el error salio en el `reset()` de
        la placa Rutherford, que era inocente — le habia tocado un descriptor por encima
        del limite. Con la sonda de salud conectando cada 60 s, el fallo llega a las ~17 h
        de encendido, siempre, sin que nadie cambie nada.

        No se puede arreglar en el SDK, asi que se acota desde afuera. Solo se cierran los
        descriptores que (a) apuntan a ESTE nodo y (b) no existian antes de abrir: nunca
        uno propio ni el de otro instrumento.

        Returns:
            Cuantos se recuperaron.
        """
        antes = getattr(self, "_fds_antes_de_abrir", None)
        if antes is None:
            return 0
        self._fds_antes_de_abrir = None

        abandonados = self._descriptores_del_equipo() - antes
        recuperados = 0
        for fd in sorted(abandonados):
            try:
                os.close(fd)
                recuperados += 1
            except OSError as exc:
                self.logger.debug("VSG25A: no se pudo cerrar el descriptor %d: %s", fd, exc)
        if recuperados:
            self.logger.debug("VSG25A: %d descriptores abandonados por el SDK, cerrados", recuperados)
        return recuperados

    # ------------------------------------------------------------------
    # Identification
    # ------------------------------------------------------------------

    def idn(self) -> str:
        """Get the device identification string.

        Hace un probe real: si el equipo no esta conectado o no responde, levanta
        ``InstrumentConnectionError`` para que `/health` lo reporte desconectado en
        vez de dar por vivo a un generador caido (que era el modo fantasma).
        """
        if self._transport == "serial":
            if not self._serial:
                raise InstrumentConnectionError(
                    "VSG25A: no hay puerto serial abierto",
                    instrument_name="tone_generator",
                )
            # El protocolo binario no expone *IDN? — se interroga con un handshake OPEN.
            self._serial.reset_input_buffer()
            self._serial.reset_output_buffer()
            resp = self._send_raw(self._pkt.build_open(), recv_len=FRAME_SIZE)
            if not resp:
                raise InstrumentConnectionError(
                    "VSG25A: no respondio al handshake OPEN",
                    instrument_name="tone_generator",
                )
            return "Signal Hound, VSG25A, Serial: RAW, Transport: Serial"

        if self._handle is None:
            raise InstrumentConnectionError(
                "VSG25A: sin handle nativo abierto",
                instrument_name="tone_generator",
            )

        serial = c_int(-1)
        status = self._lib.sgGetSerialNumber(self._handle, pointer(serial))
        if status == 0 and serial.value >= 0:
            return f"Signal Hound, VSG25A, Serial: {serial.value}, API: Native"

        raise InstrumentConnectionError(
            f"VSG25A: sgGetSerialNumber devolvio status {status}",
            instrument_name="tone_generator",
        )

    # ------------------------------------------------------------------
    # ToneGenerator Interface
    # ------------------------------------------------------------------

    def set_level(self, intensity_in_dbm: float) -> None:
        """Set the output power level in dBm.

        Args:
            intensity_in_dbm: Target amplitude in dBm (clamped to physical limits: -50 to +13 dBm).
        """
        self._amplitude_dbm = max(-50.0, min(13.0, intensity_in_dbm))
        self.logger.info("VSG25A: Target level set to %.2f dBm", self._amplitude_dbm)
        self._apply_settings()

    def set_frequency(self, frequency_in_mhz: float) -> None:
        """Set the carrier frequency in MHz.

        Args:
            frequency_in_mhz: Target frequency in MHz.
        """
        self._frequency_mhz = frequency_in_mhz
        self.logger.info("VSG25A: Target frequency set to %.3f MHz", self._frequency_mhz)
        self._apply_settings()

    def start(self) -> bool:
        """Enable RF signal generation (CW mode).

        Cicla la RF: **apaga antes de reaplicar**. La biblioteca no exporta ningun `sgRFOn`
        —solo `sgRFOff` y las `sgSet*`, que configuran y habilitan a la vez—, asi que sin el
        apagado previo el equipo se queda como estaba. Medido en banco el 2026-08-11: con
        `start()` reaplicando nada mas, un barrido de -40 a -5.5 dBm en 146.4 MHz no movio el
        AGC del amplificador Becker ni un paso y el LED nunca salio de rojo (ID-1799).

        NOTE: libsg_api.so v1.0.7 has a known bug where sgSetCW returns
        sgNoError (0) but the device does NOT emit RF. When using serial
        transport (VSG25_TRANSPORT=serial), the raw binary protocol
        bypasses this bug entirely.
        """
        self._ciclar_rf()
        self._rf_on = True

        if self._transport == "serial":
            if not self._serial:
                raise InstrumentConnectionError(
                    "VSG25A: no se puede arrancar RF — puerto serial no conectado",
                    instrument_name="tone_generator",
                )
            self._apply_settings()
            cw_pkt = self._pkt.build_short_cmd(opcode=OP_SET_CW)
            resp = self._send_raw(cw_pkt, recv_len=FRAME_SIZE)
            if resp:
                parsed = parse_response(resp)
                self.logger.debug("VSG25A: SET_CW response seq=0x%02x valid=%s",
                                  parsed["seq"], parsed["valid"])
            time.sleep(0.5)
            self.logger.info("VSG25A (Serial): RF output STARTED (CW Mode)")
            return True

        if self._handle is None:
            raise InstrumentConnectionError(
                "VSG25A: no se puede arrancar RF — dispositivo no conectado",
                instrument_name="tone_generator",
            )

        self._apply_settings()
        self.logger.info("VSG25A: RF output STARTED (CW Mode)")
        return True

    def _ciclar_rf(self) -> None:
        """Apaga la RF sin tocar `_rf_on`, para que el `start()` que sigue vuelva a habilitarla.

        No usa `stop()` a proposito: `stop()` pone `_rf_on = False`, y `_apply_settings()` mira
        esa bandera para decidir si llama a `sgSetCW`. Apagar via `stop()` desde `start()`
        dejaria la bandera en falso a mitad de camino.
        """
        if self._transport == "serial":
            if self._serial:
                self._send_raw(self._pkt.build_short_cmd(opcode=OP_RF_OFF))
            return
        if self._handle is not None:
            with self._lock:
                self._lib.sgRFOff(self._handle)

    def stop(self) -> bool:
        """Disable RF signal generation."""
        self._rf_on = False

        if self._transport == "serial":
            if self._serial:
                rf_off_pkt = self._pkt.build_short_cmd(opcode=OP_RF_OFF)
                self._send_raw(rf_off_pkt)
                self.logger.info("VSG25A (Serial): RF output STOPPED (command sent)")
                return True
            raise InstrumentConnectionError(
                "VSG25A: no se puede apagar RF — puerto serial no conectado",
                instrument_name="tone_generator",
            )

        if self._handle is None:
            raise InstrumentConnectionError(
                "VSG25A: no se puede apagar RF — dispositivo no conectado",
                instrument_name="tone_generator",
            )

        with self._lock:
            status = self._lib.sgRFOff(self._handle)
        if status == 0:
            self.logger.info("VSG25A: RF output STOPPED")
            return True
        self.logger.error("VSG25A: Failed to stop RF (status code %d)", status)
        return False

    def enable_modulation(self) -> bool:
        """Enable AM/FM/IQ modulation.

        The native/serial VSG25A binary protocol implemented here exposes only
        CW frequency/amplitude control. Modulation is not supported by this driver.
        """
        self.logger.warning("VSG25A: Modulation control is not supported by this driver")
        return False

    def disable_modulation(self) -> bool:
        """Disable modulation (no-op for this driver)."""
        return True

    # ------------------------------------------------------------------
    # Internal Helpers
    # ------------------------------------------------------------------

    def _apply_settings(self) -> None:
        """Apply the cached frequency and amplitude configuration to hardware.

        Levanta ``InstrumentConnectionError`` si el transporte esta caido: un
        generador muerto no puede devolver "configurado" en silencio.
        """
        if self._transport == "serial":
            if not self._serial:
                raise InstrumentConnectionError(
                    "VSG25A: no se puede configurar — puerto serial no conectado",
                    instrument_name="tone_generator",
                )
            freq_hz = self._frequency_mhz * 1_000_000
            payload = encode_freq_amp(float(freq_hz), float(self._amplitude_dbm))
            pkt = self._pkt.build_param_cmd(payload)
            resp = self._send_raw(pkt, recv_len=FRAME_SIZE)
            if not resp:
                raise InstrumentConnectionError(
                    "VSG25A: sin respuesta a SET_FREQ_AMP — el equipo no responde",
                    instrument_name="tone_generator",
                )
            parsed = parse_response(resp)
            self.logger.debug("VSG25A: SET_FREQ_AMP response seq=0x%02x valid=%s",
                              parsed["seq"], parsed["valid"])
            time.sleep(0.01)
            self.logger.info(
                "VSG25A (Serial): Configured %.3f MHz @ %.2f dBm",
                self._frequency_mhz, self._amplitude_dbm,
            )
            return

        if self._handle is None:
            raise InstrumentConnectionError(
                "VSG25A: no se puede configurar — dispositivo no conectado",
                instrument_name="tone_generator",
            )

        freq_hz = self._frequency_mhz * 1_000_000.0
        need_settle = False
        with self._lock:
            status = self._lib.sgSetFrequencyAmplitude(
                self._handle,
                c_double(freq_hz),
                c_double(self._amplitude_dbm),
            )
            if status in (0, 1):
                if status == 1:
                    self.logger.debug(
                        "VSG25A: Setting clamped by hardware (requested %.3f MHz @ %.2f dBm)",
                        self._frequency_mhz,
                        self._amplitude_dbm,
                    )
                else:
                    self.logger.info(
                        "VSG25A: Applied %.3f MHz @ %.2f dBm to hardware",
                        self._frequency_mhz,
                        self._amplitude_dbm,
                    )
                if self._rf_on:
                    cw_status = self._lib.sgSetCW(self._handle)
                    if cw_status != 0:
                        err = self._lib.sgGetStatusString(cw_status)
                        self.logger.error(
                            "VSG25A: Failed to start RF (status %d: %s)", cw_status, err.decode()
                        )
                    need_settle = True
            else:
                raise InstrumentConnectionError(
                    f"VSG25A: sgSetFrequencyAmplitude fallo (status {status})",
                    instrument_name="tone_generator",
                )
        # Settle delay outside lock per AGENTS.md rule
        if need_settle:
            time.sleep(0.1)

    def _send_raw(self, data: bytes, recv_len: int = 0) -> bytes:
        with self._lock:
            return self._send_raw_impl(data, recv_len)

    def _send_raw_impl(self, data: bytes, recv_len: int = 0) -> bytes:
        """Send raw bytes over serial and optionally read response.

        Applies bounded retries on transient serial errors (``OSError`` /
        ``SerialException``) and on short/invalid responses, per the test-bench
        fail-safe and retry standards. The serial port is opened with a strict
        read timeout, so this never blocks indefinitely.

        Args:
            data: Byte string to send.
            recv_len: Number of bytes to read back (0 = no read).

        Returns:
            Response bytes, or empty bytes if all retries are exhausted.
        """
        if not self._serial:
            return b""

        last_exc: Exception | None = None
        for attempt in range(1, SERIAL_WRITE_RETRIES + 1):
            try:
                self._serial.reset_input_buffer()
                self._serial.reset_output_buffer()
                self._serial.write(data)
                self._serial.flush()
                self.logger.debug(
                    "VSG25A TX (attempt %d/%d): %s", attempt, SERIAL_WRITE_RETRIES, data.hex()
                )
                if recv_len <= 0:
                    return b""
                resp = self._serial.read(recv_len)
                self.logger.debug(
                    "VSG25A RX (attempt %d/%d): %s", attempt, SERIAL_WRITE_RETRIES, resp.hex()
                )
                # Retry until the device returns a full protocol frame.
                if len(resp) >= FRAME_SIZE:
                    return resp
                last_exc = SerialTimeoutError(
                    f"short serial response: got {len(resp)} bytes, expected {FRAME_SIZE}"
                )
            except (OSError, SerialException) as exc:
                last_exc = exc
                self.logger.warning(
                    "VSG25A: serial TX/RX failed (attempt %d/%d): %s",
                    attempt, SERIAL_WRITE_RETRIES, exc,
                )
                time.sleep(min(0.05 * attempt, 0.5))
        if last_exc:
            self.logger.error(
                "VSG25A: serial transport failed after %d attempts: %s",
                SERIAL_WRITE_RETRIES, last_exc,
            )
        return b""

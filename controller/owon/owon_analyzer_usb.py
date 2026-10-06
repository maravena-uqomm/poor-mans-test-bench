"""
OWON VSA815P Spectrum Analyzer Controller — USB-TMC transport.

El VSA815P expone SCPI sobre USB-TMC (/dev/usbtmc*) por USB.
La interfaz LAN solo da noVNC, sin puerto SCPI TCP.

Los comandos SCPI estándar viven en ScpiCommandMixin (compartido
con ScpiAnalyzer TCP/IP). Esta clase solo agrega lo específico de
USB-TMC: retry/settle para get_level_from_marker, peak search para
markers, y sweep time.

Jira: ID-1308 (Test Bench v2)
"""
from __future__ import annotations

import glob
import logging
import math
import os
import time

from controller.base.analyzer import Analyzer
from controller.base.scpi_commands import SCPIError, ScpiCommandMixin
from controller.base.usb_tmc_controller import UsbTmcController

#: Default del intervalo entre sondeos del marcador — el tiempo que necesita el
#: barrido de fondo para completar un ciclo nuevo. Ver `MARKER_POLL_INTERVAL`.
MARKER_POLL_INTERVAL_DEFAULT = 0.10

DEFAULT_DEVICE = "/dev/usbtmc2"


#: Donde sysfs guarda la identidad de un usbtmc, en orden de preferencia.
#:
#: `device/interface` es el string de la INTERFAZ USB y **solo existe si el descriptor trae
#: iInterface**. El VSA815P no lo trae, asi que en esta maquina ese archivo no existe y la
#: busqueda entera daba vacio. La identidad real esta un nivel mas arriba, en el DISPOSITIVO.
_SYSFS_IDENTITY = ("device/interface", "device/../product", "device/../manufacturer")

#: El nombre estable que crea `scripts/99-uqomm-testbench.rules` por VID:PID. Si esta, gana:
#: es el unico camino en el que el numero de minor no puede volver a morder.
_NOMBRE_ESTABLE = "/dev/analyzer_owon"


def _identidad_usbtmc(nodo_sysfs: str) -> str:
    """Todo lo que sysfs sepa decir de ese usbtmc, concatenado y en mayusculas."""
    partes = []
    for rel in _SYSFS_IDENTITY:
        try:
            with open(os.path.join(nodo_sysfs, rel), "r") as f:
                partes.append(f.read().strip())
        except OSError:
            continue
    return " ".join(partes).upper()


def _find_owon_usbtmc() -> str | None:
    """El /dev/usbtmcN del OWON VSA815P, o None si no se pudo identificar.

    ── DOS DEFECTOS QUE ESTO CIERRA (16-Sep-2026) ────────────────────────────────────────

    Los dos hacian que un analizador ENCHUFADO Y CON EL DRIVER ENGANCHADO se viera igual que
    uno ausente, que es la forma de fallar que la fundacion del banco llama «un cero que no
    es un dato».

    1. Buscaba la identidad en `device/interface`, que en este kernel **no existe** para el
       VSA815P. Ahora se prueban tres rutas: esa, y `product`/`manufacturer` del dispositivo
       padre, que es donde si esta (`OWON` / `VSA815P`, verificado por sysfs).

    2. El respaldo recorria `/dev/usbtmc0..3` con un `range(4)` escrito a mano. El nodo real
       era **`/dev/usbtmc4`**: el minor sube con cada reenchufe y no vuelve. Ahora se enumera
       lo que HAY con glob, que no tiene tope.

    ── POR QUE NO DEVUELVE CUALQUIER NODO ────────────────────────────────────────────────

    Si no se pudo identificar ninguno y hay MAS DE UNO, devuelve None en vez de elegir: abrir
    el usbtmc equivocado es hablarle por SCPI a otro instrumento. Es la misma regla que
    `vlad25_contrato_bench` ya aplica con los VCP —«hay N y son indistinguibles: pasar la ruta
    a mano»— y la que a `DeviceIdentifier` le falta.

    Con un solo nodo sin identificar si se devuelve: no hay ambiguedad que resolver.
    """
    log = logging.getLogger(__name__)

    # EL NOMBRE ESTABLE PRIMERO, y es la unica solucion que no depende del numero.
    #
    # `scripts/99-uqomm-testbench.rules` crea /dev/analyzer_owon por VID:PID (2148:0064). Con
    # esa regla instalada el minor deja de importar: no hay que identificar nada, ni enumerar,
    # ni desempatar. Sin ella se sigue por sysfs, que funciona igual pero tiene que mirar.
    #
    # Se prueba ANTES que sysfs a proposito: el symlink lo puso una regla que ya filtro por
    # VID:PID, o sea que es mas especifico que cualquier cosa que podamos deducir aca.
    if os.path.exists(_NOMBRE_ESTABLE):
        log.debug("owon_usbtmc por nombre estable: %s", _NOMBRE_ESTABLE)
        return _NOMBRE_ESTABLE

    candidatos = sorted(glob.glob("/sys/class/usbmisc/usbtmc*"))

    for sysfs in candidatos:
        if "OWON" in (ident := _identidad_usbtmc(sysfs)) or "VSA815" in ident:
            nodo = f"/dev/{os.path.basename(sysfs)}"
            # EL NOMBRE DE SYSFS ES EL NUMERO DEL HOST, y en un contenedor el dispositivo puede
            # estar mapeado a OTRO /dev/usbtmcN. Medido en .202 el 22-Sep-2026: sysfs identifica
            # `usbtmc1` (numero del host) mientras el compose lo mapea a `/dev/usbtmc2`; devolver
            # `/dev/usbtmc1` daba "No such file or directory" y el analizador quedaba degraded.
            # Si el nodo derivado no existe, se cae al glob de lo que HAY en /dev (el mapeado real).
            if os.path.exists(nodo):
                log.debug("owon_usbtmc identificado por sysfs: %s", nodo)
                return nodo
            log.debug("sysfs identifica OWON en %s pero ese nodo no existe (mapeo de contenedor); "
                      "se cae al glob de /dev", nodo)

    # Nadie se identifico. Se enumera lo que existe, SIN tope de indice.
    nodos = sorted(glob.glob("/dev/usbtmc*"))
    if len(nodos) == 1:
        log.warning(
            "no se pudo identificar el OWON por sysfs; hay un solo usbtmc (%s) y se usa ese",
            nodos[0],
        )
        return nodos[0]
    if len(nodos) > 1:
        log.error(
            "hay %d dispositivos usbtmc (%s) y ninguno se identifica como OWON. No se elige "
            "uno al azar: pasar la ruta a mano con ANALYZER_DEVICE.",
            len(nodos), ", ".join(nodos),
        )
    return None


class OwonAnalyzerUsb(ScpiCommandMixin, Analyzer, UsbTmcController):
    #: Offset de nivel aplicado EN SOFTWARE (ver `set_ref_db_offset`). Va como atributo de
    #: CLASE y no solo de instancia: hay codigo —tests incluidos— que construye este driver sin
    #: pasar por `__init__`, y una lectura que dependiera de que ese `__init__` corrio
    #: explotaria con AttributeError en el camino de medicion. Un default de clase no puede
    #: faltar.
    _ref_db_offset: float = 0.0

    #: Cuenta las veces que `get_level_from_marker` descarto una respuesta no numerica
    #: (buffer USB-TMC desfasado). Atributo de CLASE por el mismo motivo que
    #: `_ref_db_offset` de arriba — construir sin pasar por `__init__` no puede dejarlo sin
    #: existir. Es la senal de "vivo pero degradandose" que `test_execution_service` lee
    #: entre suites para escalar a recovery antes de que una reconexion falle del todo —
    #: ver docs/runbooks/robustez_lote_pruebas.md.
    glitch_count: int = 0

    """Analizador OWON VSA815P vía USB-TMC."""

    IDN_PREFIX = "OWON"

    # ── OWON VSA815P Timing Constants ──────────────────────────────────
    # Y? response: typical 2-5 ms; sweep-queued worst case ~500 ms.
    # 2.0s = 4× worst-case sweep + USB-TMC overhead.
    DEFAULT_READ_TIMEOUT = 2.0
    # USB-TMC microframe: 1 ms; OWON parser: ~10 ms per command.
    # 0.10s = 10× parser time; prevents input buffer overflow.
    RATE_LIMIT_DELAY = 0.10
    # Peak search is pure computation (<1ms). 0.02s = 20× margin, pero es
    # una suposicion de timing, no una confirmacion real de que el
    # instrumento termino — ajustable sin tocar codigo via
    # OWON_PEAK_SEARCH_SETTLE_S para probar empiricamente si hace falta mas.
    PEAK_SEARCH_SETTLE = float(os.environ.get("OWON_PEAK_SEARCH_SETTLE_S", "0.02"))
    # YA NO SE USA en get_level_from_marker() — probado en el banco
    # (diagnose_sweep_retrigger.py) que :ABORt + :INITiate:IMMediate no
    # re-arma de verdad el sweep en este VSA815P via USB-TMC, sin importar
    # cuanto se espere despues (probado con 0.2, 1 y 2s, mismo resultado
    # congelado). Se cambio a barrido continuo. Se deja la constante y el
    # env var por compatibilidad con ese script de diagnostico, que todavia
    # prueba el modo "single" a proposito, como comparacion.
    SWEEP_SETTLE = float(os.environ.get("OWON_SWEEP_SETTLE_S", "0.20"))
    # Settle detection: requires MARKER_STABLE_COUNT consecutive Y? reads
    # within MARKER_STABLE_TOLERANCE_DB of each other before accepting a
    # value — a single pair matching by chance (e.g. mid-decay after the
    # VSG stops/restarts RF on a frequency change) is not enough. Up to
    # MARKER_MAX_POLLS attempts, MARKER_POLL_INTERVAL apart, to actually
    # reach that real stability instead of settling for a false positive.
    # All four overridable via env without touching code.
    MARKER_MAX_POLLS = int(os.environ.get("OWON_MARKER_MAX_POLLS", "12"))
    MARKER_POLL_INTERVAL = float(os.environ.get("OWON_MARKER_POLL_INTERVAL_S", "0.10"))
    MARKER_STABLE_COUNT = int(os.environ.get("OWON_MARKER_STABLE_COUNT", "3"))
    MARKER_STABLE_TOLERANCE_DB = float(os.environ.get("OWON_MARKER_STABLE_TOLERANCE_DB", "0.3"))
    # Piso de ruido (require_stability=False en get_level_from_marker): cuantas lecturas
    # independientes se promedian. Ruido de fondo no converge como un tono — exigirle
    # MARKER_STABLE_COUNT lecturas consecutivas dentro de tolerancia agota siempre
    # MARKER_MAX_POLLS/timeout_s sin nunca asentar. Promediar N lecturas sueltas es el
    # criterio correcto para estimar un piso, no un pico.
    # Bajado de 4 a 3 (01-Sep-2026): cada muestra del piso cuesta ~1s (ver
    # get_level_from_marker), y el piso NUNCA corta antes de agotar las N — a
    # diferencia de la señal, que corta apenas asienta. Con 4 costaba ~4s por piso y
    # habia hasta 6 pisos por corrida de VBU2 (~24s solo en pisos). El piso solo
    # alimenta el guardia de SNR (`snr = pout - nf`, avisa si SNR < 10 dB); con los
    # SNR reales vistos en banco (57-66 dB) hay margen de sobra para perder algo de
    # precision en la estimacion del piso sin arriesgar el guardia.
    MARKER_FAST_SAMPLES = int(os.environ.get("OWON_MARKER_FAST_SAMPLES", "3"))
    # TRACe:MODE is a register write — ya no se usa sleep fijo tras set.
    TRACE_MODE_SETTLE = 0.2
    # Tope defensivo: el "rbw" que llega en la config de sesion/producto es
    # un numero externo (no lo valida este repo). Un valor mal puesto ahi
    # (visto en produccion: 1000000, pensado como Hz pero interpretado como
    # kHz) manda un :BWIDth de ~1GHz, que el instrumento clampea a su
    # maximo real — muchisimo mas ancho que lo que cualquier suite de este
    # repo pide (30-300 kHz) y mas propenso a capturar espurios en vez del
    # tono inyectado. Este tope corta esos valores antes de llegar al
    # instrumento; ajustable sin tocar codigo via OWON_MAX_RBW_KHZ.
    MAX_RESOLUTION_BANDWIDTH_KHZ = float(os.environ.get("OWON_MAX_RBW_KHZ", "1000"))

    # Sentinel values the OWON VSA815P emits when the sweep is not complete
    # or the marker is not ready. "0" is the documented return for
    # :MARKer:Y? when the sweep hasn't finished or marker is off.
    _MARKER_SENTINELS: frozenset[str] = frozenset({"nan", "error", "inf", "-inf", "ol"})

    #: Intentos de validar *IDN? al conectar. El sweep continuo del OWON deja
    #: respuestas de marcador encoladas —medido en el banco: '-31.2357'— que el
    #: query("*IDN?") lee antes que el IDN. Un solo intento declaraba "no es un
    #: OWON" a un analizador sano y abortaba la sesion siguiente.
    IDN_CONNECT_RETRIES = int(os.environ.get("OWON_IDN_CONNECT_RETRIES", "4"))
    #: Espera entre reintentos del IDN, para que el comando de parada del sweep
    #: (INITiate:CONTinuous OFF) tenga efecto y el buffer deje de encolar datos.
    IDN_CONNECT_SETTLE_S = float(os.environ.get("OWON_IDN_CONNECT_SETTLE_S", "0.25"))
    #: Settle tras el `*RST` de ultimo recurso de `_validar_idn()` (ver ahi) — mas largo que
    #: IDN_CONNECT_SETTLE_S porque `*RST` reinicia mas estado del instrumento que un `*CLS`.
    IDN_RST_SETTLE_S = float(os.environ.get("OWON_IDN_RST_SETTLE_S", "1.0"))

    def __init__(self, device_path: str = DEFAULT_DEVICE):
        super().__init__()
        resolved = _find_owon_usbtmc()
        self._device_path = resolved if resolved else device_path
        if resolved and resolved != device_path:
            self.logger.info("OWON VSA815P detectado en %s (configurado %s)", resolved, device_path)
        self._do_connect()

    def _do_connect(self) -> None:
        """Open device and validate IDN.

        Flush residual data left by previous sessions (el OWON contamina
        el buffer si un comando anterior quedo sin leer).
        Si falla la validacion, cierra el fd.
        """
        self.logger.info("Conectando a %s ...", self._device_path)
        self.connect(self._device_path)
        try:
            self.logger.info("Flushing residual data...")
            self._flush_usb(hard=True)
            time.sleep(0.2)

            # Frenar la produccion de datos del sweep ANTES de validar. El sweep
            # continuo encola valores de marcador que contaminan el *IDN? — es la
            # causa del '-31.2357' medido en el banco el 14-Sep-2026, que aborto una
            # sesion porque el IDN leyo un marcador viejo. Son best-effort: si no
            # cambian nada, el reintento de abajo igual descarta lo contaminado.
            self.send_command(":INITiate:CONTinuous OFF", no_response=True)
            self.send_command(":CALCulate:MARKer:AOFF", no_response=True)

            idn = self._validar_idn()
            if idn is None:
                raise ValueError(
                    f"Dispositivo en {self._device_path} no respondio un IDN "
                    f"{self.IDN_PREFIX} tras {self.IDN_CONNECT_RETRIES} intentos"
                )
            self.logger.info("OWON VSA815P (USB-TMC) conectado a %s — %s", self._device_path, idn)

            # El offset de nivel de referencia SOBREVIVE a la desconexion: vive en el
            # instrumento, no en este proceso. Una sesion que configuro un offset de
            # compensacion de cables lo dejaba puesto, y la sesion siguiente medía todo
            # corrido por ese valor sin que nada lo dijera — el numero se ve razonable, solo
            # esta mal. Ninguna suite lo devuelve a cero (verificado en las seis que lo usan),
            # asi que el punto donde el estado queda definido es este.
            #
            # Se hace despues del IDN a proposito: si el dispositivo no es el analizador
            # esperado, no se le manda nada.
            self.set_ref_db_offset(0.0)
        except (OSError, ValueError, RuntimeError) as exc:
            self.logger.error("Fallo conexion OWON: %s", exc)
            try:
                super().disconnect()
            except (OSError, RuntimeError):
                self.logger.debug("disconnect_failed_during_error_recovery", exc_info=True)
            raise

    def _validar_idn(self) -> str | None:
        """Devuelve el IDN validado, o None si el *IDN? nunca contesto con un OWON.

        El query("*IDN?") puede devolver un valor de marcador viejo ('-31.2357'), un
        eco de comando ('WRITe') o una traza, en vez del IDN. Cada intento descarta
        esa basura, drena el buffer y reintenta; solo despues de agotar los intentos
        se declara que el dispositivo no es el analizador esperado.

        ULTIMO INTENTO CON `*RST` REAL (confirmado 23-Sep-2026, ver
        docs/runbooks/hard_reset_instrumentos.md §5.4/§7.7): hasta ahora esta escalera solo
        mandaba `*CLS`, que limpia el registro de eventos pero no reinicia el sweep — un OWON
        que perdio la fase de su protocolo (contesta la consulta anterior a cada consulta
        nueva) puede seguir asi indefinidamente. El caso real del 22-Sep-2026 tardo ~2m10s en
        3 ciclos de sonda de HealthChecker en recuperarse solo, exactamente el escenario que
        el propio `marcador_trabado()` ya documenta que un `*RST` resuelve
        (owon_analyzer_usb.py:477, medido el 26-Ago). Se agrega ACA, no antes: los primeros
        IDN_CONNECT_RETRIES intentos son baratos (`*CLS`) y alcanzan la mayoria de las veces;
        el `*RST` es mas invasivo (reinicia mas estado del instrumento) y solo se justifica
        cuando esos ya se agotaron.
        """
        for intento in range(1, self.IDN_CONNECT_RETRIES + 1):
            self.logger.info("Querying *IDN? (intento %d/%d)...", intento, self.IDN_CONNECT_RETRIES)
            self.send_command("*CLS", no_response=True)
            idn = self.idn()
            if idn and idn.upper().startswith(self.IDN_PREFIX.upper()):
                return idn
            self.logger.warning(
                "OWON: *IDN? devolvio %r (contaminacion de buffer) — drenando y reintentando",
                idn,
            )
            self._flush_usb(hard=True)
            time.sleep(self.IDN_CONNECT_SETTLE_S)

        self.logger.warning(
            "OWON: %d intentos con *CLS agotados sin IDN — probando *RST como ultimo recurso",
            self.IDN_CONNECT_RETRIES,
        )
        self.send_command("*RST", no_response=True)
        self._flush_usb(hard=True)
        time.sleep(self.IDN_RST_SETTLE_S)
        idn = self.idn()
        if idn and idn.upper().startswith(self.IDN_PREFIX.upper()):
            self.logger.warning("OWON: *RST recupero el IDN — %r", idn)
            return idn
        self.logger.error("OWON: *RST no recupero el IDN (devolvio %r) — conexion perdida", idn)
        return None

    def reconnect(self) -> None:
        """Close, drain USB buffer, and reopen the device.

        Re-scans the USB bus for the correct device node in case it changed
        after a physical USB reset (ioctl USBDEVFS_RESET).
        """
        resolved = _find_owon_usbtmc()
        if resolved:
            self._device_path = resolved
        self.logger.info("Reconectando OWON VSA815P en %s ...", self._device_path)
        self._close_locked()
        time.sleep(0.5)
        self._do_connect()
        self.logger.info("OWON VSA815P reconectado exitosamente")

    # ------------------------------------------------------------------
    # Conexión USB-TMC
    # ------------------------------------------------------------------

    def connect(self, device_path: str = DEFAULT_DEVICE) -> None:
        UsbTmcController.connect(self, device_path)

    def disconnect(self) -> None:
        """Cierra USB-TMC. Resetea markers y trazas primero (best-effort).
        No usa set_trace_mode (demora 11s por flush O_NONBLOCK) —
        manda comandos directos sin verificar respuesta."""
        try:
            self.send_command(":CALCulate:MARKer:AOFF", no_response=True)
            self.send_command(":TRACe1:MODE BLANk", no_response=True)
            self.send_command(":TRACe2:MODE BLANk", no_response=True)
        except (OSError, RuntimeError) as exc:
            self.logger.warning("Cleanup error during disconnect: %s", exc)
        super().disconnect()

    # ------------------------------------------------------------------
    # Transport bridge — USB-TMC → send_command / send_command_batch
    # ------------------------------------------------------------------

    def send_command(self, command: str, no_response: bool = False) -> tuple[bool, str]:
        """Adapta la interfaz SCPI a USB-TMC (write/query).

        El rate-limit (RATE_LIMIT_DELAY) YA lo aplica `UsbTmcController.write()` — este
        metodo no debe repetirlo. Hasta el 02-Sep-2026 lo hacia: `self.write(command)`
        duerme RATE_LIMIT_DELAY al final, y esta linea dormia el mismo tiempo otra vez,
        asi que cada comando no_response pagaba el rate-limit DOBLE (0.20s en vez de
        0.10s). Encontrado con logging de diagnostico en banco: los gaps entre comandos
        no_response median ~0.22s con `write=0.000s` en ambos —imposible explicarlo por
        latencia del instrumento, escrito por os.write() es practicamente instantaneo—.
        Con MAXimum llamado 3 veces por punto y ~16 puntos por corrida de VBU2, mas los
        comandos de setup no_response del arranque, esto es varios segundos por corrida."""
        try:
            if no_response or "?" not in command:
                self.write(command)
                return True, ""
            response = self.query(command)
            return True, response
        except (OSError, RuntimeError) as e:
            self.logger.error("USB-TMC command failed: %s", e)
            return False, str(e)

    def send_command_batch(self, commands: list[str], no_response: bool = False) -> tuple[bool, list[str]]:
        with self._lock:
            responses = []
            for cmd in commands:
                success, resp = self.send_command(cmd, no_response)
                if not success:
                    return False, [resp]
                responses.append(resp)
            return True, responses

    # ------------------------------------------------------------------
    # Métodos que difieren del mixin — quirks del USB-TMC
    # ------------------------------------------------------------------

    def idn(self) -> str:
        return self.query("*IDN?")

    # ------------------------------------------------------------------
    # Overrides: OWON VSA815P no acepta sufijos "MHz" ni "KHz" —
    # requiere valores en Hz siempre.
    # ------------------------------------------------------------------

    def set_center_freq(self, center_freq: float) -> None:
        freq_hz = int(center_freq * 1_000_000)
        self._cmd(f":FREQuency:CENTer {freq_hz}", f"Center -> {center_freq} MHz")

    def set_span(self, span: float) -> None:
        span_hz = int(span * 1_000_000)
        self._cmd(f":FREQuency:SPAN {span_hz}", f"Span -> {span} MHz")

    def set_resolution_bandwidth_khz(self, resolution: float) -> None:
        if resolution > self.MAX_RESOLUTION_BANDWIDTH_KHZ:
            self.logger.warning(
                "RBW pedido %.0f kHz excede el tope %.0f kHz (OWON_MAX_RBW_KHZ) — "
                "se limita. Revisar el valor 'rbw' de la config de producto/sesion.",
                resolution, self.MAX_RESOLUTION_BANDWIDTH_KHZ,
            )
            resolution = self.MAX_RESOLUTION_BANDWIDTH_KHZ
        rbw_hz = int(resolution * 1_000)
        self._cmd(f":BWIDth {rbw_hz}", f"RBW -> {resolution} kHz")

    def center_and_span_panel(self, ul_freq: float, dl_freq: float, span: float) -> None:
        center = (ul_freq + dl_freq) / 2.0
        center_hz = int(center * 1_000_000)
        span_hz = int(span * 1_000_000)
        success, resp = self.send_command_batch([
            f":FREQuency:SPAN {span_hz}",
            f":FREQuency:CENTer {center_hz}",
        ])
        if not success:
            raise SCPIError(f"center_and_span_panel falló: {resp}")
        self.logger.info("Panel: center=%.1f MHz span=%.1f MHz", center, span)

    def mark_freq(self, freq_in_mhz: float) -> None:
        freq_hz = int(freq_in_mhz * 1_000_000)
        ok, _ = self.send_command(":CALCulate:MARKer1:STATe ON", no_response=True)
        if not ok:
            raise SCPIError("No se pudo activar marker 1 (USB-TMC)")
        ok, _ = self.send_command(f":CALCulate:MARKer1:X {freq_hz}", no_response=True)
        if not ok:
            raise SCPIError(f"No se pudo posicionar marker en {freq_in_mhz} MHz (USB-TMC)")

    def get_trace_data(self, trace_number: int = 1) -> list:
        """La traza, con el offset ya aplicado a la amplitud de cada punto.

        `gain_check` en modo `fast` mide sobre la traza y no sobre el marcador, asi que si el
        offset se aplicara solo en `get_level_from_marker` el modo rapido quedaria sin corregir
        —y la diferencia entre los dos modos serian los 8-9 dB de la perdida del camino, que es
        exactamente el error que este offset existe para tapar—.
        """
        puntos = super().get_trace_data(trace_number)
        if not self._ref_db_offset:
            return puntos
        # Cada punto es (frecuencia, amplitud): el offset va solo en la amplitud.
        return [(p[0], p[1] + self._ref_db_offset, *p[2:]) for p in puntos]

    def set_ref_db_offset(self, offset: float) -> None:
        """Corrige las lecturas en dB. **En software, no en el instrumento.**

        MEDIDO en el banco el 18-Ago-2026 sobre el VSA815P: el comando SCPI
        `:DISplay:WINDow:TRACe:Y:RLEVel:OFFSet` se acepta sin error y **no tiene ningun
        efecto** sobre lo que devuelve el marcador. Con el tono fijo, pidiendo offsets de
        0, +10 y -20 dB, el marcador leyo -18.7328, -18.7346 y -18.7355 dBm: identico dentro
        del ruido.

        O sea que las seis suites que "aplican un offset" no aplicaban nada, y una tabla de
        compensacion configurada desde el backend no habria tenido efecto. El sintoma es el
        peor: cada ganancia medida a traves de la placa sale baja por la perdida del camino
        —unos 8 a 9 dB— y el numero se ve razonable.

        Por eso aca se corrige en software y **no** se manda el comando SCPI. No se manda a
        proposito: si algun dia el firmware del instrumento empezara a respetarlo, se aplicaria
        dos veces. El `ScpiCommandMixin` sigue mandandolo para el camino TCP, que es otro
        instrumento y no se midio.
        """
        self._ref_db_offset = float(offset)
        self.logger.info(
            "Ref offset -> %+.2f dB (en SOFTWARE: el VSA815P ignora el SCPI, medido)", offset
        )

    #: Cuantas lecturas se miran para decidir si el marcador esta trabado.
    _LECTURAS_ANTI_TRABA = 3

    #: Cuanto se espera ENTRE esas lecturas, que es lo que hace valida la comprobacion: si
    #: se lee mas rapido de lo que el instrumento barre, un equipo sano devuelve lo mismo
    #: tres veces y el chequeo lo declara trabado.
    #:
    #: **Medido el 26-Ago-2026, no supuesto.** Con el analizador sano se probaron 0,02 /
    #: 0,10 / 0,35 / 0,50 s y los cuatro dieron "sano": a span de 20 MHz el barrido alcanza
    #: incluso con 20 ms. Se deja el `MARKER_POLL_INTERVAL` del propio driver —que ya
    #: significa "tiempo para que el barrido de fondo complete un ciclo"— porque el tiempo
    #: de barrido depende del span y del RBW, y ese margen no costo nada de medir.
    #:
    #: Queda anotado porque yo llegue aca por el camino equivocado: vi tres lecturas iguales
    #: en el banco, supuse que leia demasiado rapido y subi el intervalo a 0,5 s. La medicion
    #: mostro que la suposicion era falsa — el analizador estaba trabado de verdad, y el
    #: `*RST` lo confirmo pasando el detector de "congelado" a None.
    _INTERVALO_ANTI_TRABA = float(
        os.environ.get("OWON_ANTI_TRABA_S", str(MARKER_POLL_INTERVAL_DEFAULT))
    )

    def marcador_trabado(self) -> str | None:
        """Devuelve el motivo si el marcador esta trabado, o None si responde.

        **Que es esto.** El 26-Ago-2026 el VSA815P quedo aceptando comandos y contestando
        su `*IDN?` con el subsistema de marcadores congelado: `:MARKer:Y?` devolvia el
        string `'0'` y `:MARKer:X?` `'22540140.0'` **sin cambiar aunque el span pasara de 0
        a 5 a 20 MHz**. Un `*RST` lo destrabo y volvio a leer -70,8 / -77,1 / -72,0 dBm.

        Lo grave no fue la falla sino que **nadie la detectaba**: la suite midio, informo
        numeros y siguio. Una tanda entera de mediciones salio de un instrumento trabado.

        **Por que estas dos senales y no las de `evaluar_sweep`.** Ese evaluador valida
        posicion del marcador, repetibilidad y seguimiento, pero dos de las tres no sirven
        aca: la posicion se compara contra la frecuencia del tono y **en span cero el eje X
        es tiempo**, asi que fallaria siempre; y el seguimiento necesita al menos dos
        potencias, y las suites de perdida miden a una sola. Queda lo que si transfiere:

          * `Y` exactamente 0 — una lectura real trae decimales (`-70.7856`). Un 0 pelado
            en un banco cuyo piso ronda los -90 dBm no es una medicion;
          * `X` **e** `Y` identicas en las tres lecturas — con el instrumento vivo, el ruido
            mueve al menos una. Se exigen las dos iguales para no confundir un tono estable
            (que puede dar Y casi constante) con un equipo congelado.

        No toca el generador: se llama con el tono como este. Solo interroga al marcador.
        """
        lecturas: list[tuple[str, str]] = []
        for _ in range(self._LECTURAS_ANTI_TRABA):
            self.send_command(f":CALCulate:MARKer1:STATe ON", no_response=True)
            self.send_command(f":CALCulate:MARKer1:MAXimum", no_response=True)
            time.sleep(self._INTERVALO_ANTI_TRABA)
            ok_y, y = self.send_command(":CALCulate:MARKer1:Y?")
            ok_x, x = self.send_command(":CALCulate:MARKer1:X?")
            if not (ok_y and ok_x):
                return f"el marcador no contesta (Y={y!r}, X={x!r})"
            lecturas.append((y.strip(), x.strip()))

        ys = {y for y, _ in lecturas}
        if ys == {"0"} or all(self._es_cero(y) for y, _ in lecturas):
            return (
                f"el marcador devuelve 0 en las {len(lecturas)} lecturas — no es un nivel, "
                "el instrumento esta trabado. Un *RST lo destraba"
            )
        if len(set(lecturas)) == 1:
            y, x = lecturas[0]
            return (
                f"el marcador devuelve siempre lo mismo (Y={y}, X={x}) en "
                f"{len(lecturas)} lecturas: esta congelado. Un *RST lo destraba"
            )
        return None

    @staticmethod
    def _es_cero(valor: str) -> bool:
        try:
            return float(valor) == 0.0
        except ValueError:
            return False

    def get_level_from_freq(self, freq_in_mhz: float) -> float:
        # No suma el offset: delega en `get_level_from_marker`, que ya lo aplica. Sumarlo aca
        # tambien lo aplicaria dos veces.
        self.mark_freq(freq_in_mhz)
        level = self.get_level_from_marker(1)
        if level is None:
            raise SCPIError(f"No settle en marker 1 para frecuencia {freq_in_mhz} MHz")
        return level

    def set_marker(self, marker_index: int = 1, frequency: float = 0.0) -> None:
        """Activa marker y hace peak search (usa :MAXimum).

        El OWON VSA815P no retorna nivel confiable con :X fijo
        después de un sweep. Peak search encuentra el máximo real.

        ``frequency`` se ignora: el marker se posiciona vía peak search,
        no vía coordenada fija.
        """
        ok, _ = self.send_command(f":CALCulate:MARKer{marker_index}:STATe ON", no_response=True)
        if not ok:
            raise SCPIError(f"No se pudo activar marker {marker_index} (USB-TMC)")
        ok, _ = self.send_command(f":CALCulate:MARKer{marker_index}:MAXimum", no_response=True)
        if not ok:
            raise SCPIError(f"No se pudo ejecutar peak search marker {marker_index} (USB-TMC)")

    #: VID:PID del VSA815P, para encontrar su nodo USB crudo y reenumerarlo.
    _USB_VID, _USB_PID = "2148", "0064"
    #: ioctl USBDEVFS_RESET: fuerza al chip USB del instrumento a reinicializarse.
    _USBDEVFS_RESET = 21780

    def _nodo_usb_crudo(self) -> str | None:
        """`/dev/bus/usb/BBB/DDD` del analizador, buscado por VID:PID en sysfs.

        Por identidad y no por numero: el BBB/DDD cambia con cada reenumeracion, que es
        justamente lo que estamos por provocar.
        """
        for dev in glob.glob("/sys/bus/usb/devices/*/idVendor"):
            base = os.path.dirname(dev)
            try:
                with open(dev) as fh:
                    if fh.read().strip() != self._USB_VID:
                        continue
                with open(os.path.join(base, "idProduct")) as fh:
                    if fh.read().strip() != self._USB_PID:
                        continue
                with open(os.path.join(base, "busnum")) as fh:
                    bus = int(fh.read().strip())
                with open(os.path.join(base, "devnum")) as fh:
                    num = int(fh.read().strip())
                return f"/dev/bus/usb/{bus:03d}/{num:03d}"
            except (OSError, ValueError):
                continue
        return None

    def _reenumerar_usb(self) -> bool:
        """Reinicia el enlace USB del instrumento. Es lo UNICO que lo devuelve a fase.

        Drenar no alcanza —el fd devuelve cero bytes con el instrumento corrido, o sea que la
        cola esta adentro del VSA815P— y `*CLS` lo acomoda a veces. Reenumerar lo arregla
        siempre; es lo mismo que desenchufar y enchufar, sin tocar el cable.

        Necesita escribir en el nodo USB crudo, que por defecto es root-only: lo habilita la
        regla de `scripts/99-uqomm-testbench.rules` para este VID:PID. Sin esa regla devuelve
        False y el llamador avisa que hay que desenchufar a mano.
        """
        nodo = self._nodo_usb_crudo()
        if nodo is None:
            self.logger.error("no se encontro el nodo USB del analizador para reenumerarlo")
            return False
        try:
            import fcntl
            fd = os.open(nodo, os.O_WRONLY)
            try:
                fcntl.ioctl(fd, self._USBDEVFS_RESET, 0)
            finally:
                os.close(fd)
        except OSError as exc:
            self.logger.error(
                "no se pudo reenumerar %s (%s). Falta la regla udev que lo hace escribible: "
                "correr scripts/instalar_reglas_udev.sh", nodo, exc,
            )
            return False
        self.logger.warning("analizador REENUMERADO por USB para recuperar la fase del protocolo")
        time.sleep(3.0)   # el instrumento tarda en volver a enumerar
        return True

    def resincronizar(self, intentos: int = 8) -> bool:
        """Deja el pipeline USB-TMC EN FASE: cada consulta contestandose a si misma.

        ── EL DEFECTO QUE CIERRA, medido el 17-Sep-2026 ──────────────────────────────────

        Despues de configurar el analizador, una lectura de marcador devolvia la respuesta de
        una consulta ANTERIOR — se vio el IDN y el modo de traza («WRITe») saliendo por
        `:CALCulate:MARKer1:Y?`, y `:TRACe1:MODE?` devolviendo `-85.1159`, que es un nivel.
        Lo grave no es que falle: es que cuando el valor corrido resulta ser un numero, pasa
        por medicion. Ya produjo una tabla entera de lecturas falsas que se leyeron como «el
        tono no llega» — ver `docs/rf-offsets.md` §24.

        ── LA PRIMERA VERSION DE ESTE METODO NO PODIA FALLAR, Y POR ESO NO SERVIA ────────

        Preguntaba `*IDN?` dos veces y exigia el IDN en las dos. **Con el pipeline corrido en
        uno eso se cumple siempre**: la segunda consulta recibe la respuesta de la primera, que
        es el IDN. Daba verde, dejaba una respuesta sin consumir, y el marcador la leia.

        Se arregla preguntando **dos cosas DISTINTAS** y exigiendo que cada una traiga lo suyo:
        el IDN tiene que traer «OWON» y el centro tiene que traer un numero. Con un desfase de
        uno, el centro devuelve el IDN y la comprobacion cae — que es la unica forma de que un
        chequeo de fase signifique algo.
        """
        for intento in range(intentos):
            # PRIMERO SE CONSUME LA PENDIENTE, y esto es lo que la version anterior no hacia.
            #
            # Mandar N consultas y leer N respuestas CONSERVA el corrimiento: si hay una
            # respuesta pendiente P0, la consulta 1 recibe P0, la 2 recibe R(1), y R(2) queda
            # pendiente. El desfase no se empareja emparejando: hay que sacar una.
            #
            # `*CLS` vacia la cola del instrumento, y el drenado necesita ESPERA porque la
            # respuesta pendiente puede estar todavia en vuelo — un poll instantaneo no la ve y
            # aparece despues, que es como volvia a corrverse. Es la secuencia de
            # `scripts/reset_owon.py`, que es la que funciono a mano.
            self._flush_usb(hard=True)
            self.send_command("*CLS", no_response=True)
            time.sleep(0.4)
            self._flush_usb(hard=True)

            # AHORA se verifica, con dos consultas DISTINTAS: el IDN tiene que traer «OWON» y
            # el centro un numero. Preguntar `*IDN?` dos veces no sirve — con un desfase de uno
            # la segunda recibe la respuesta de la primera, que tambien es el IDN, asi que la
            # comprobacion se cumple siempre y no puede fallar.
            ok_idn, idn = self.send_command("*IDN?")
            ok_frec, frec = self.send_command(":FREQuency:CENTer?")
            if not (ok_idn and ok_frec) or "OWON" not in str(idn):
                continue
            try:
                float(str(frec).strip())
            except (TypeError, ValueError):
                continue
            if intento:
                self.logger.info("analizador de vuelta en fase tras %d intento(s)", intento + 1)
            return True
        # DRENAR NO ALCANZO: la cola esta adentro del instrumento. Se REENUMERA, que es lo
        # unico que lo recupera, y se vuelve a comprobar. Un solo intento: si despues de
        # reiniciarle el USB sigue corrido, el problema no es la fase.
        self.logger.warning(
            "el analizador sigue fuera de fase tras %d intentos de drenado: se reenumera", intentos,
        )
        if self._reenumerar_usb():
            try:
                self.reconnect()
            except Exception:
                self.logger.exception("no se pudo reconectar tras reenumerar el analizador")
                return False
            self._flush_usb(hard=True)
            ok_idn, idn = self.send_command("*IDN?")
            ok_frec, frec = self.send_command(":FREQuency:CENTer?")
            if ok_idn and ok_frec and "OWON" in str(idn):
                try:
                    float(str(frec).strip())
                    self.logger.info("analizador de vuelta en fase despues de reenumerar")
                    return True
                except (TypeError, ValueError):
                    pass

        self.logger.error(
            "el analizador no vuelve a fase ni reenumerando: todo lo que se mida ahora puede ser "
            "la respuesta de la consulta anterior. Desenchufarlo y volver a enchufarlo a mano.",
        )
        return False

    def get_level_from_marker(
        self, marker: int = 1,
        timeout_s: float = 10.0,
        poll_interval_s: float | None = None,
        max_retries: int | None = None,
        require_stability: bool = True,
    ) -> float | None:
        """Lee marker Y con retry/settle sobre barrido CONTINUO.

        Confirmado en el banco con diagnose_sweep_retrigger.py: el modo
        single-sweep (:INITiate:CONTinuous OFF + :ABORt + :INITiate:IMMediate,
        probado con SWEEP_SETTLE de 0.2, 1 y 2s) nunca re-armaba de verdad en
        este VSA815P por USB-TMC — tanto :MARKer:Y? como :MARKer:X? (posicion
        del marker) quedaban clavados en el mismo valor exacto sin importar
        que la potencia real inyectada variara hasta 20dB. Con
        :INITiate:CONTinuous ON (el modo normal del instrumento, barriendo
        solo en background) el mismo experimento SI mostro el marker
        moviendose y el nivel siguiendo los cambios reales de potencia.

        Por eso esta version deja el barrido en continuo y solo espera
        tiempo real entre lecturas (MARKER_POLL_INTERVAL) para que el
        barrido de fondo alcance a completar un ciclo nuevo — no dispara ni
        aborta nada por intento.

        require_stability=False (piso de ruido, no un tono): en vez de exigir
        MARKER_STABLE_COUNT lecturas consecutivas dentro de MARKER_STABLE_TOLERANCE_DB
        —un criterio que ruido de fondo no puede cumplir por definicion, ya que
        fluctua sweep a sweep en vez de converger a un pico— promedia hasta
        MARKER_FAST_SAMPLES lecturas validas sueltas, sin exigir que se parezcan
        entre si. Es el criterio correcto para estimar un piso: no hay pico al que
        asentar. Antes de esto, medir el piso agotaba SIEMPRE el timeout completo
        (10s) sin nunca estabilizar y devolvia None — el guarda de SNR que depende
        de ese piso (`if snr is not None and snr < 10`) nunca se activaba.

        Retorna None si no se pudo estabilizar tras todos los retries (modo
        estable), o si ninguna lectura fue valida (modo rapido).
        """
        _poll_interval = poll_interval_s if poll_interval_s is not None else self.MARKER_POLL_INTERVAL
        default_retries = self.MARKER_MAX_POLLS if require_stability else self.MARKER_FAST_SAMPLES
        _max_retries = max_retries if max_retries is not None else default_retries

        ok, _ = self.send_command(":INITiate:CONTinuous ON", no_response=True)
        if not ok:
            raise SCPIError("No se pudo activar sweep continuo (USB-TMC)")

        # STATe ON una sola vez antes del lazo, no en cada vuelta: el marker se queda
        # encendido en el instrumento hasta que algo lo apague, y reenviarlo en cada
        # iteracion pagaba un round-trip + su RATE_LIMIT_DELAY (via `write()`, que lo
        # aplica a TODO comando, no solo a los "no_response") sin cambiar nada del lado
        # del instrumento. Medido en banco (02-Sep-2026): ~0.97-1.07s por iteracion, de
        # los cuales el codigo solo explicaba ~0.42s (poll_interval + 3x rate_limit +
        # peak_settle) — sacar este round-trip redundante de las 48 iteraciones de una
        # corrida de VBU2 (16 puntos x 3 muestras) apunta a unos ~7s de ahorro.
        ok, _ = self.send_command(f":CALCulate:MARKer{marker}:STATe ON", no_response=True)
        if not ok:
            raise SCPIError(f"No se pudo activar marker {marker} (USB-TMC)")

        deadline = time.perf_counter() + timeout_s
        window: list[float] = []
        all_values: list[float] = []
        stable_count = self.MARKER_STABLE_COUNT
        tolerance = self.MARKER_STABLE_TOLERANCE_DB
        retry_count = 0
        last_raw = "<no response>"

        while time.perf_counter() < deadline and retry_count < _max_retries:
            # Tiempo real, no un trigger — dejamos que el barrido de fondo
            # (continuo) alcance a completar al menos un ciclo nuevo antes
            # del peak search.
            time.sleep(_poll_interval)

            ok, _ = self.send_command(f":CALCulate:MARKer{marker}:MAXimum", no_response=True)
            if not ok:
                raise SCPIError(f"No se pudo ejecutar peak search marker {marker} (USB-TMC)")
            time.sleep(self.PEAK_SEARCH_SETTLE)

            success, response = self.send_command(f":CALCulate:MARKer{marker}:Y?")
            retry_count += 1
            if not success:
                continue

            last_raw = response.strip()
            if not last_raw or last_raw.lower() in self._MARKER_SENTINELS:
                window.clear()
                continue

            try:
                value = float(last_raw)
            except (ValueError, TypeError):
                # No es un sentinel documentado ni un numero — probable
                # contaminacion de buffer USB-TMC (ej. se vio devolver la
                # respuesta de un *IDN? viejo en vez del Y? pedido). Se
                # descarta y se reintenta igual que un sentinel, pero se
                # loguea distinto para poder diferenciarlo de un timeout o
                # un valor NaN si vuelve a aparecer.
                self.logger.warning(
                    "Marker %d Y? intento %d: respuesta no numerica (posible "
                    "glitch de buffer USB-TMC), se descarta: %r",
                    marker, retry_count, last_raw,
                )
                self.glitch_count += 1
                window.clear()
                continue

            if math.isnan(value):
                window.clear()
                continue

            window.append(value)
            all_values.append(value)

            if not require_stability:
                # Sin criterio de convergencia: se promedia todo lo recolectado
                # al salir del lazo (por conteo, no por asentar).
                continue

            if len(window) >= stable_count:
                recent = window[-stable_count:]
                if max(recent) - min(recent) <= tolerance:
                    self.logger.info(
                        "Marker %d settled after %d sweep(s): accepted=%.2f window=%s all_seen=%s",
                        marker, retry_count, recent[-1], recent, all_values,
                    )
                    # El offset se suma ACA porque el instrumento lo ignora: es el unico punto
                    # por el que sale un nivel del marcador, asi que ninguna suite lo puede
                    # saltear. Ver `set_ref_db_offset`.
                    return recent[-1] + self._ref_db_offset

        if not require_stability:
            if not all_values:
                self.logger.warning(
                    "Marker %d: ninguna lectura valida en %d intento(s) (piso de ruido)",
                    marker, retry_count,
                )
                return None
            avg = sum(all_values) / len(all_values)
            self.logger.info(
                "Marker %d piso de ruido: promedio=%.2f dBm sobre %d/%d lecturas validas: %s",
                marker, avg, len(all_values), retry_count, all_values,
            )
            # Mismo punto de aplicacion del offset que el camino estable — ver comentario
            # arriba: es el unico lugar por el que sale un nivel del marcador.
            return avg + self._ref_db_offset

        self.logger.warning("Marker %d Y? no settle tras %d sweeps / %.1fs — last: %r",
                            marker, retry_count, timeout_s, last_raw)
        return None

    def set_sweep_time(self, sweep_time_ms: float) -> None:
        ok, _ = self.send_command(f":SWEep:TIME {sweep_time_ms} ms", no_response=True)
        if not ok:
            raise SCPIError(f"No se pudo setear sweep time {sweep_time_ms} ms (USB-TMC)")

    # ------------------------------------------------------------------
    # Override: OWON necesita tiempo entre set y verify
    # ------------------------------------------------------------------

    def set_trace_mode(self, trace_number: int = 1, mode: str = "WRITe"):
        ok, _ = self.send_command(f":TRACe{trace_number}:MODE {mode}", no_response=True)
        if not ok:
            self._flush_usb()
            time.sleep(0.1)
            ok, _ = self.send_command(f":TRACe{trace_number}:MODE {mode}", no_response=True)
            if not ok:
                raise SCPIError(f"Failed to set trace {trace_number} mode {mode}")
        self._flush_usb()
        ok, response = self.send_command(f":TRACe{trace_number}:MODE?")
        if not ok:
            raise SCPIError(f"No se pudo verificar trace {trace_number} mode: {response}")
        self.logger.info("Trace %d mode -> %s", trace_number, response.strip())

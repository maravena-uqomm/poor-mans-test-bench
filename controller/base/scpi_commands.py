"""
ScpiCommandMixin — Comandos SCPI compartidos por todos los analizadores.

Todos los métodos siguen la misma política: si un comando falla,
se lanza SCPIError inmediatamente. No hay fallos silenciosos.

Unidades: la API pública sigue siendo en MHz/kHz (no cambia ningún llamador), pero al
cable van **Hz enteros y sin sufijo**. Motivo, medido el 25-Ago-2026 contra el OWON
VSA815P V1.0.1.0: ese firmware IGNORA el sufijo de unidad, así que ":FREQuency:CENTer
174.925 MHz" le fija el centro en 174,925 **Hz** y lo confirma por eco. No da error: deja
el analizador parado cerca de DC devolviendo una traza de ruido plausible, que es peor que
fallar. Se perdió una sesión de banco creyendo que el analizador estaba roto.
Hz sin sufijo es la unidad por omisión de SCPI y el Siglent SSA3015X la acepta igual.

Jira: ID-1308 (Test Bench v2), ID-1857
"""
from __future__ import annotations

from controller.base.reliability import InstrumentError


def _hz(mhz: float) -> int:
    """MHz -> Hz entero, para el cable. Ver la nota de unidades de arriba."""
    return int(round(float(mhz) * 1_000_000))


def _khz_a_hz(khz: float) -> int:
    """kHz -> Hz entero, para el cable."""
    return int(round(float(khz) * 1_000))


class SCPIError(InstrumentError):
    """Un comando SCPI falló contra el instrumento.

    Antes heredaba de `RuntimeError` puro (24-Sep-2026, encontrado en banco real): un
    `SCPIError` NO es `InstrumentError`, así que `_INSTRUMENT_FAULTS`
    (`test_execution_service.py`, `(InstrumentError, CircuitBreakerOpen, OSError)`) nunca lo
    reconocía como falla de instrumento. Con el analizador apagado a mitad de sesión (prueba
    real del hard reset), `turn_off_tracking_generator()` -> `_cmd()` -> `SCPIError` caía en
    la rama `except Exception` genérica — "defecto de software", suite marcada fallida, lote
    sigue — sin pasar NUNCA por `_esperar_recuperacion()` ni por el hard reset. La escalera de
    recuperación entera quedaba inalcanzable para el modo de falla más común del analizador
    real (casi todo lo que hace `ScpiCommandMixin` pasa por `_cmd()`/`SCPIError`).

    `retriable=False, recoverable=True` (los defaults de `InstrumentError`) son los correctos
    acá: `send_command()` ya reintentó 3 veces a nivel de transporte antes de que esto se
    levante (ver el log "USB-TMC write attempt N/3"), así que reintentar de nuevo en el
    momento no tiene sentido — lo que corresponde es la recuperación de LOTE (reconectar,
    y si hace falta, hard reset), no un reintento inmediato del mismo comando.
    """


def verify_idn(send_command, prefix: str, *, strip: bool = True) -> tuple[bool, str]:
    """Query ``*IDN?`` and check the response starts with ``prefix``.

    Returns ``(is_match, response_text)``. Safe to call on any controller
    that exposes ``send_command(cmd) -> tuple[bool, str]``.

    Args:
        send_command: Bound ``send_command`` method of the controller.
        prefix: Expected IDN prefix (case-insensitive).
        strip: If True (default), strip surrounding whitespace before
            comparing; set False to match raw-response semantics.
    """
    success, response = send_command("*IDN?")
    if not success:
        return False, response if response else "No response"
    text = response.strip() if strip else response
    return text.upper().startswith(prefix.upper()), text


class ScpiCommandMixin:
    """Comandos SCPI de analizador. Requiere send_command / send_command_batch."""

    # ------------------------------------------------------------------
    # Frecuencia y Span
    # ------------------------------------------------------------------

    def center_and_span_panel(self, ul_freq: float, dl_freq: float, span: float) -> None:
        center = (ul_freq + dl_freq) / 2.0
        success, resp = self.send_command_batch([
            f":FREQuency:SPAN {_hz(span)}",
            f":FREQuency:CENTer {_hz(center)}",
        ])
        if not success:
            raise SCPIError(f"center_and_span_panel falló: {resp[0] if resp else 'unknown'}")
        self.logger.info("Panel: center=%.1f MHz span=%.1f MHz", center, span)

    def set_center_freq(self, center_freq: float) -> None:
        self._cmd(f":FREQuency:CENTer {_hz(center_freq)}", f"Center -> {center_freq} MHz")

    def set_span(self, span: float) -> None:
        self._cmd(f":FREQuency:SPAN {_hz(span)}", f"Span -> {span} MHz")

    def set_resolution_bandwidth_khz(self, resolution: float) -> None:
        self._cmd(f":BWIDth {_khz_a_hz(resolution)}", f"RBW -> {resolution} kHz")

    # ------------------------------------------------------------------
    # Marcadores
    # ------------------------------------------------------------------

    def mark_freq(self, freq_in_mhz: float) -> None:
        success, _ = self.send_command(":CALCulate:MARKer1:STATe ON", no_response=True)
        if not success:
            raise SCPIError("No se pudo activar marker 1")
        self._cmd(f":CALCulate:MARKer1:X {_hz(freq_in_mhz)}", f"Marker 1 -> {freq_in_mhz} MHz")

    def get_level_from_freq(self, freq_in_mhz: float) -> float:
        self.mark_freq(freq_in_mhz)
        return self.get_level_from_marker(1)

    def get_level_from_marker(self, marker: int = 1, **kwargs) -> float:
        # **kwargs (ej. require_stability) se ignora aca: este mixin no hace
        # retry/settle propio, a diferencia de OwonAnalyzerUsb. Sin esto, un caller
        # generico que pase ese kwarg (ver PassiveLossCheck._medir_piso) rompe con
        # TypeError apenas el analizador wireado es un TCP/IP en vez de USB-TMC.
        success, response = self.send_command(f":CALCulate:MARKer{marker}:Y?")
        if not success:
            raise SCPIError(f"Fallo al leer marker {marker}: {response}")
        try:
            level = float(response)
            self.logger.info("Marker %d -> %.1f dBm", marker, level)
            return level
        except (ValueError, TypeError):
            raise SCPIError(f"Respuesta inválida marker {marker}: {response}")

    def set_marker(self, marker_index: int = 1, frequency: float = 0.0) -> None:
        success, _ = self.send_command(f":CALCulate:MARKer{marker_index}:STATe ON", no_response=True)
        if not success:
            raise SCPIError(f"No se pudo activar marker {marker_index}")
        self._cmd(
            f":CALCulate:MARKer{marker_index}:X {_hz(frequency)}",
            f"Marker {marker_index} -> {frequency} MHz",
        )

    def set_markers_off(self) -> None:
        self._cmd(":CALCulate:MARKer:AOFF", "Markers OFF")

    def set_marker_to_trace(self, marker_index: int, trace_index: int) -> None:
        self._cmd(f":CALCulate:MARKer{marker_index}:TRACe {trace_index}", f"Marker {marker_index} -> trace {trace_index}")

    def set_marker_table(self, state: str) -> None:
        self._cmd(f":CALCulate:MARKer:TABLe {state}", f"Marker table {state}")

    def set_marker_mode(self, mode: str) -> None:
        self._cmd(f":CALCulate:MARK1:MODE {mode}", f"Marker mode {mode}")

    def set_markers(self, markers: list) -> None:
        """Set multiple markers (index, freq_mhz) pairs."""
        for marker_index, frequency in markers:
            self.set_marker(marker_index, frequency)

    def set_marker_bw(self, marker: int, state: str, dBm: float) -> None:
        self._cmd(f":CALCulate:MARKer{marker}:BANDwidth:NDB {dBm}", f"Marker {marker} BW NDB -> {dBm} dB")
        self._cmd(f":CALCulate:MARKer{marker}:FUNCtion {state}", f"Marker {marker} BW function -> {state}")

    def get_marker_bw_result(self, marker: int) -> float:
        success, response = self.send_command(f":CALCulate:MARKer{marker}:BANDwidth:RESult?")
        if not success:
            raise SCPIError(f"Fallo al leer BW result marker {marker}: {response}")
        try:
            bw = float(response) / 1_000_000
            self.logger.info("Marker %d BW result -> %.2f MHz", marker, bw)
            return bw
        except (ValueError, TypeError):
            raise SCPIError(f"Respuesta inválida BW result marker {marker}: {response}")

    # ------------------------------------------------------------------
    # Trazas
    # ------------------------------------------------------------------

    def set_trace_mode(self, trace_number: int = 1, mode: str = "WRITe") -> None:
        success, _ = self.send_command(f":TRACe{trace_number}:MODE {mode}", no_response=True)
        if not success:
            raise SCPIError(f"No se pudo setear trace {trace_number} mode {mode}")
        success, response = self.send_command(f":TRACe{trace_number}:MODE?")
        if not success:
            raise SCPIError(f"No se pudo verificar trace {trace_number} mode: {response}")
        self.logger.info("Trace %d mode -> %s", trace_number, response.strip())

    def set_two_trace(self, trace_a: int, trace_b: int) -> None:
        self.set_trace_mode(trace_a, "VIEW")
        self.set_trace_mode(trace_b, "WRITe")

    @staticmethod
    def _sin_encabezado_de_bloque(respuesta: str) -> str:
        """Saca el encabezado de bloque de IEEE-488.2, si viene.

        Medido en el banco el 20-Ago-2026: el VSA815P contesta `:TRACe:DATA?` con
        `#9000001785` y despues la carga. El parseo hacia `float()` sobre esa cabecera y
        explotaba **siempre**, asi que la lectura de traza nunca funciono — y el modo `fast`
        de `gain_check`, que mide sobre la traza en vez del marcador, no podia andar.

        El formato del estandar:

          `#<n><n digitos de largo><carga>`  — largo definido, n de 1 a 9
          `#0<carga><newline>`               — largo indefinido

        El largo declarado se respeta: si el instrumento agrega cola despues del bloque, no
        entra en los datos.
        """
        if not respuesta.startswith("#"):
            return respuesta
        if len(respuesta) < 2:
            return respuesta
        digitos = respuesta[1]
        if digitos == "0":
            # Largo indefinido: la carga va hasta el newline.
            return respuesta[2:].split("\n", 1)[0]
        if not digitos.isdigit():
            return respuesta
        n = int(digitos)
        inicio = 2 + n
        try:
            largo = int(respuesta[2:inicio])
        except ValueError:
            return respuesta
        return respuesta[inicio : inicio + largo]

    def get_trace_data(self, trace_number: int = 1) -> list:
        success, response = self.send_command(f":TRACe{trace_number}:DATA?")
        if not success:
            raise SCPIError(f"Fallo al leer trace {trace_number}: {response}")
        response = self._sin_encabezado_de_bloque(response.strip())
        try:
            data = [
                tuple(float(x) for x in point.split(","))
                for point in response.strip().split(";")
            ]
            self.logger.info("Trace %d: %d puntos", trace_number, len(data))
            return data
        except (ValueError, TypeError) as e:
            raise SCPIError(f"Error parseando trace {trace_number}: {e}")

    def normalize(self) -> None:
        self._cmd(":CALCulate:NTData ON", "Normalizacion ON")

    def max_hold(self) -> None:
        self.set_trace_mode(mode="MAXHold")

    def freeze_image(self) -> None:
        self._cmd(":DISPlay:FREEze", "Imagen congelada")

    def get_image(self) -> str:
        success, response = self.send_command(":DISPlay:DATA?")
        if not success:
            raise SCPIError(f"Fallo screenshot: {response}")
        self.logger.info("Screenshot: %d bytes", len(response))
        return response

    # ------------------------------------------------------------------
    # Nivel de referencia
    # ------------------------------------------------------------------

    def set_reference_level(self, level: float) -> None:
        self._cmd(f":DISPlay:WINDow:TRACe:Y:RLEVel {level} dBm", f"Ref level -> {level} dBm")

    def get_reference_level(self) -> float:
        success, response = self.send_command(":DISP:TRAC:Y:RLEV?")
        if not success:
            raise SCPIError(f"Fallo al leer ref level: {response}")
        try:
            return float(response)
        except (ValueError, TypeError):
            raise SCPIError(f"Respuesta inválida ref level: {response}")

    def set_ref_db_offset(self, offset: float) -> None:
        self._cmd(f":DISplay:WINDow:TRACe:Y:RLEVel:OFFSet {offset}", f"Ref offset -> {offset} dB")

    # ------------------------------------------------------------------
    # Tracking Generator
    # ------------------------------------------------------------------

    def set_tracking_generator(self, state: str) -> None:
        if state.upper() not in ("ON", "OFF"):
            raise SCPIError(f"Estado inválido: {state}")
        self._cmd(f":OUTPut {state.upper()}", f"TG -> {state.upper()}")
        self._tg_on = state.upper() == "ON"

    def turn_on_tracking_generator(self) -> None:
        self.set_tracking_generator("ON")

    def turn_off_tracking_generator(self) -> None:
        self.set_tracking_generator("OFF")

    def tracking_generator_is_on(self) -> bool | None:
        """Estado del TG, o ``None`` si nunca se seteo en esta sesion.

        No se consulta al instrumento: `:OUTPut?` no esta soportado de forma uniforme y esto
        se llama en el camino caliente del barrido. Lo usa
        ``suite/test.py::switch_direction`` para apagar el TG antes de conmutar la placa
        Rutherford —su SP4T es reflectivo— y volver a encenderlo despues.
        """
        return getattr(self, "_tg_on", None)

    def set_tracking_generator_level(self, level: float) -> None:
        self._cmd(f":SOURce:POWer {level} dBm", f"TG level -> {level} dBm")

    def get_tracking_generator_level(self) -> float:
        success, response = self.send_command(":SOURce:POWer?")
        if not success:
            raise SCPIError(f"Fallo al leer TG level: {response}")
        try:
            return float(response)
        except (ValueError, TypeError):
            raise SCPIError(f"Respuesta inválida TG level: {response}")

    # ------------------------------------------------------------------
    # Interno
    # ------------------------------------------------------------------

    def _cmd(self, command: str, ok_msg: str) -> None:
        """Send SCPI command. Raises SCPIError on failure."""
        success, err = self.send_command(command, no_response=True)
        if not success:
            raise SCPIError(f"{ok_msg} — FAILED: {err}")
        self.logger.info("%s — OK", ok_msg)

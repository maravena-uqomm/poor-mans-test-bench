"""Rutterford — tarjeta enrutadora RF.

Modos de operacion:
  - downlink / DL:  senal del generador hacia el DUT
  - uplink / UL:    senal del DUT hacia el analizador
  - wire:           paso directo (sin routing)

Puertos: BASE, TUNEL, BRANCH1, BRANCH2.

Legacy aliases (set_crfs_mode, set_dl_tone_mode, etc.) existen para
compatibilidad con suites escritas contra la interfaz Crfs original.
"""

from dataclasses import dataclass
from typing import Literal


Port = Literal["BASE", "TUNEL", "BRANCH1", "BRANCH2"]


class DcMeasurementUnsupported(NotImplementedError):
    """La placa conectada no sabe medir DC.

    Existe como tipo propio para que la suite pueda distinguir "no se puede medir" de
    "midio y dio cero". Un firmware viejo, la pylib sin `read_dc_measurements()` o el
    banco de cableado fijo son casos legitimos: lo que no es legitimo es rellenar con la
    lectura de la fuente y presentarla como si fuera la salida del pasivo.
    """


@dataclass(frozen=True)
class DcReading:
    """Tension DC en UN conector de salida del DUT.

    La placa mide los ocho conectores —cuatro interfaces DUT x dos impedancias— por
    divisores fijos de 45,3k/3k, asi que una sola llamada trae el cuadro completo.

    Attributes:
        port: Nombre canonico del orquestador (BASE, TUNEL, BRANCH1, BRANCH2).
        ohms: 50 o 75, el conector medido.
        volts: La lectura, o **None** cuando ese canal no convirtio. No 0.0: cero volts es
            una medicion legal aca —un conector sin alimentar—, asi que usar cero como
            relleno haria indistinguible "no hay tension" de "no se pudo medir".
        raw_counts: Cuentas crudas del ADC. Sirve para separar un canal muerto de uno que
            lee cero de verdad.
    """

    port: str
    ohms: int
    volts: float | None
    raw_counts: int


class Rutterford:
    """Routing de senal RF entre instrumentos y DUT.

    Uso tipico (perdida DL BASE->TUNEL):
        rutterford.set_mode_downlink()
        rutterford.set_input_port("BASE")
        rutterford.set_output_port("TUNEL")
    """

    # ------------------------------------------------------------------
    # Modos de operacion (canonicos)
    # ------------------------------------------------------------------

    def set_mode_uplink(self) -> None:
        """Configura la tarjeta en modo Uplink (DUT -> Analyzer)."""

    def set_mode_downlink(self) -> None:
        """Configura la tarjeta en modo Downlink (Generator -> DUT)."""

    def set_mode_wire(self) -> None:
        """Configura la tarjeta en modo cableado directo."""

    # ------------------------------------------------------------------
    # Puertos (canonicos)
    # ------------------------------------------------------------------

    def set_input_port(self, port: Port) -> None:
        """Selecciona el puerto de entrada de senal RF."""

    def set_output_port(self, port: Port) -> None:
        """Selecciona el puerto de salida / medicion RF."""

    def set_direct_route(self, input_port: Port, output_port: Port) -> None:
        """Rutea generador->input_port y analizador->output_port en un solo paso.

        Para aislacion entre dos ramas (TUNEL<->BRANCH1, etc.): ninguno de los dos
        extremos es BASE, y los dos pueden estar INTERCAMBIANDO valores respecto a la
        ruta anterior (ida TUNEL->BRANCH1, vuelta BRANCH1->TUNEL). Llamar
        `set_input_port()` + `set_output_port()` por separado para ese caso pasa por un
        estado intermedio invalido: el primero de los dos cambia un lado y deja el otro
        en su valor VIEJO, que puede coincidir con el valor NUEVO que se le iba a pedir
        al segundo — el SP4T lo rechaza aunque el par FINAL pedido sea perfectamente
        valido (los dos puertos son distintos). Fijar los dos juntos evita ese estado
        intermedio por completo.
        """

    def set_branch(self, branch: int) -> None:
        """Selecciona rama por numero: set_output_port('BRANCH{branch}')."""
        self.set_output_port(f"BRANCH{branch}")

    # ------------------------------------------------------------------
    # Medicion
    # ------------------------------------------------------------------

    def read_dc_measurements(self) -> tuple[DcReading, ...]:
        """Tension DC en los ocho conectores de salida, en orden DC_meas1..8.

        Una sola llamada: la placa convierte los ocho canales. Un canal caido vuelve con
        `volts=None` **y conserva su identidad** (`port`, `ohms`), para que se sepa cual
        conector se apago.

        Raises:
            DcMeasurementUnsupported: Si la placa o la biblioteca no lo soportan.
        """
        raise DcMeasurementUnsupported(
            f"{type(self).__name__} no mide DC en los conectores de salida"
        )

    # ------------------------------------------------------------------
    # Aliases legacy (Crfs compatibility)
    # ------------------------------------------------------------------

    def set_UL_mode(self) -> None:
        """Legacy: alias de set_mode_uplink()."""
        self.set_mode_uplink()

    def set_DL_mode(self) -> None:
        """Legacy: alias de set_mode_downlink()."""
        self.set_mode_downlink()

    def set_wire_mode(self) -> None:
        """Legacy: alias de set_mode_wire()."""
        self.set_mode_wire()

    def set_crfs_mode(self, direction: str) -> None:
        """Legacy: traduce string 'downlink'/'uplink' a metodo canonico."""
        normalized = direction.strip().lower()
        if normalized == "downlink":
            self.set_mode_downlink()
        elif normalized == "uplink":
            self.set_mode_uplink()
        else:
            raise ValueError(f"Invalid direction '{direction}'. Must be 'downlink' or 'uplink'.")

    def set_dl_tone_mode(self) -> None:
        """Legacy: alias redundante de set_mode_downlink()."""
        self.set_mode_downlink()

    def set_ul_tone_mode(self) -> None:
        """Legacy: alias redundante de set_mode_uplink()."""
        self.set_mode_uplink()

    def set_dl_bw_mode(self) -> None:
        """Legacy: alias redundante de set_mode_downlink()."""
        self.set_mode_downlink()

    def set_ul_bw_mode(self) -> None:
        """Legacy: alias redundante de set_mode_uplink()."""
        self.set_mode_uplink()

"""Vlad — interfaz base para controladores de placa VLAD.

Define el contrato para controlar la placa RF VLAD (Rev24/Rev25).
Las subclases concretas implementan el transporte (USB, BLE, dummy).

Metodos principales:
  - set_gain(gain_db, channel): configurar ganancia
  - set_agc(agc_volts, channel): configurar la referencia de AGC
  - get_gain(): leer ganancia actual
  - get_agc(): leer la referencia de AGC actual
  - connect() / disconnect(): ciclo de vida

CAMBIO DE UNIDADES (ID-1644): `set_agc` recibia `agc_level_dbm` en dBm, pero el
firmware Rev25 no implementa esa conversion: el comando 0x16 SET_AGC toma una
**tension de referencia del DAC** (float32 LE, 0-3.3 V). No hay curva de
calibracion dBm->V en ninguna capa, asi que la API en dBm no tenia respaldo.
Se expone en volts, que es lo que el hardware realmente acepta.
"""


class Vlad:
    """Interfaz base para controladores de placa VLAD.

    Las subclases deben implementar al menos:
    - set_gain(), set_agc(), get_gain(), get_agc()
    - connect(), disconnect(), is_healthy()
    """

    def set_gain(self, gain_db: float, channel: str = "UL") -> bool:
        """Configura la ganancia del VLAD en dB.

        Args:
            gain_db: Valor de ganancia a aplicar.
            channel: "UL" o "DL". El firmware tiene comandos separados por canal
                (0x0A SET_GAIN_UL / 0x0B SET_GAIN_DL).

        Returns:
            True si el comando se envio correctamente.
        """
        return False

    def set_agc(self, agc_volts: float, channel: str = "UL") -> bool:
        """Configura la tension de referencia del AGC, en volts.

        Args:
            agc_volts: Referencia del DAC, 0.0 a 3.3 V. **No es dBm** — ver la nota
                de unidades en el docstring del modulo.
            channel: "UL" o "DL".

        Returns:
            True si el comando se envio correctamente.
        """
        return False

    def get_gain(self) -> float | None:
        """Lee la ganancia actual del VLAD.

        Returns:
            Ganancia en dB, o None si no se pudo leer.
        """
        return None

    def get_agc(self) -> float | None:
        """Lee la tension de referencia del AGC.

        Returns:
            Referencia en volts, o None si no se pudo leer.
        """
        return None

    def connect(self) -> bool:
        """Abre la conexion con el VLAD.

        Returns:
            True si la conexion fue exitosa.
        """
        return False

    def disconnect(self) -> None:
        """Cierra la conexion con el VLAD."""
        return

    def is_healthy(self) -> bool:
        """Verifica si el controlador esta en estado operativo.

        Returns:
            True si conectado y funcional.
        """
        return False

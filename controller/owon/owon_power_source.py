"""
OWON SPE15054 Programmable DC Power Supply Controller.

Device specs:
  - Output: 0-150 V, 0-5 A, 400 W
  - Resolution: 10 mV / 1 mA
  - Ripple: <= 150 mVp-p
  - Interfaces: USB (CDC), RS232, RS485
  - Protocol: SCPI

NOTE: The OWON SPE series uses serial (USB/RS232/RS485) as primary interface.
This controller communicates via TCP/IP (requires a serial-to-Ethernet bridge
such as ser2net, USR-TCP232, or equivalent). For direct USB/serial control,
a SerialController base class (pyserial-based) would be needed instead.

SCPI Command Reference (based on standard OWON SPE series conventions):
  *IDN?                    - Identification query
  VOLTage <value>          - Set output voltage (V)
  VOLTage?                 - Query set voltage
  CURRent <value>          - Set output current limit (A)
  CURRent?                 - Query set current limit
  MEASure:VOLTage?         - Measure actual output voltage
  MEASure:CURRent?         - Measure actual output current
  OUTPut <ON|OFF>          - Enable/disable output
  OUTPut?                  - Query output state

WARNING: La fuente OWON debe generar una rampa de 0 a Vtarget en ~1s, no un escalón
instantáneo. Un escalón de 0V → 12V/24V puede dañar la instrumentación de salida
del túnel de RF. Ver implementación pendiente en _ramp_voltage().

Jira: ID-1308 (Test Bench v2) / ID-1386, ID-1530, ID-1540
"""

import os
import time

from controller.base.power_source import PowerSource
from controller.base.voltage_ramp import rampa_de_tension
from controller.base.scpi_commands import verify_idn
from controller.base.tcp_ip_controller import TcpIpController

# La velocidad de la rampa vive en `controller/base/voltage_ramp.py`, que es donde vive la
# rampa desde el 16-Sep-2026. Las constantes que estaban aca quedaron sin lectores al delegar
# y se borran: una constante que nadie lee es la que se queda vieja sin que nada lo diga.
# Las variables de entorno NO cambian —`OWON_RAMP_UP_S_PER_12V` sigue funcionando— y ademas se
# aceptan sin el prefijo, para que valgan igual con cualquier fuente.


class OwonPowerSource(PowerSource, TcpIpController):
    """Controller for OWON SPE15054 Programmable DC Power Supply.

    Connects via TCP/IP (requires serial-to-Ethernet bridge for native
    USB/RS232/RS485 interfaces).
    """

    # Default SCPI port — OWON SPE15054 proprietary port (NOT standard 5025)
    DEFAULT_PORT = 4196

    # Expected *IDN? response prefix
    IDN_PREFIX = "OWON"

    def __init__(self, ip_address: str, port: int = DEFAULT_PORT):
        """Initialize the OWON SPE15054 power supply controller.

        Args:
            ip_address: IP address of the device (or serial-to-Ethernet bridge).
            port: TCP port (default: 4196 — OWON SPE15054 proprietary port).

        Raises:
            ValueError: If the connected device is not an OWON power supply.
        """
        super().__init__(
            ip_address=ip_address, port=port, device_name="OWON Power Source"
        )
        self.channel = 1  # SPE15054 is single-channel
        self._ramp_enabled = True  # Set False in tests to skip ramp
        self._voltage_setting = 0.0
        is_owon, message = self._is_owon_device()
        if not is_owon:
            raise ValueError(
                f"Device at {ip_address}:{port} is not an OWON power supply: "
                f"{message}"
            )
        self.logger.info(
            "OWON SPE15054 initialized — %s", message
        )

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        try:
            self.turn_off_output()
        except (OSError, RuntimeError) as exc:
            self.logger.warning("Failed to turn off output during cleanup: %s", exc, exc_info=True)
        super().__exit__(exc_type, exc_val, exc_tb)
        return False

    # ------------------------------------------------------------------
    # PowerSource interface
    # ------------------------------------------------------------------

    def set_voltage(self, voltage: float) -> bool:
        """Set the output voltage in volts with linear ramp.

        Rampa lineal: 1s por cada 12V (0→12V = 1s, 0→24V = 2s, etc.)
        en pasos de ~0.5V para evitar escalones que dañen instrumentación.

        Args:
            voltage: Desired voltage in volts (0-150 V for SPE15054).

        Returns:
            True if voltage was set successfully.
        """
        current_v = 0.0
        if self._ramp_enabled:
            try:
                _, current_str = self.send_command("VOLTage?")
                current_v = float(current_str)
            except (ValueError, TypeError):
                current_v = 0.0

            if not self._ramp_voltage(current_v, voltage):
                return False

        self._voltage_setting = voltage

        # Final step: ensure exact value
        cmd = f"VOLTage {voltage:.3f}"
        success, _ = self.send_command(cmd, no_response=True)
        if not success:
            return False
        time.sleep(0.05)

        # Verify the setting
        _, response = self.send_command("VOLTage?")
        try:
            set_voltage = float(response)
            if abs(set_voltage - voltage) > 0.01:
                self.logger.warning(
                    "Voltage mismatch: set=%.3f V, read=%.3f V",
                    voltage, set_voltage
                )
        except (ValueError, TypeError):
            self.logger.warning("Could not verify voltage setting")

        self.logger.info("Voltage set to %.3f V (ramped from %.3f V)", voltage, current_v)
        return True

    def _ramp_voltage(self, current_v: float, target_v: float) -> bool:
        """Rampa lineal entre current_v y target_v con rollback en fallo.

        DELEGA en `controller/base/voltage_ramp.py` desde el 16-Sep-2026. La aritmetica era
        identica a la que iba a necesitar la Siglent y lo unico que cambia entre fuentes es el
        comando SCPI, asi que se extrajo en vez de copiarse. El comportamiento no cambia: mismo
        paso de 0,5 V, misma velocidad por direccion y mismos nombres de variable de entorno.
        """
        return rampa_de_tension(
            escribir=lambda v: self.send_command(f"VOLTage {v:.3f}", no_response=True)[0],
            actual_v=current_v,
            objetivo_v=target_v,
            logger=self.logger,
            al_aplicar=lambda v: setattr(self, "_voltage_setting", v),
        )

    def set_current_limit(self, current: float) -> bool:
        """Set the output current limit in amperes.

        Args:
            current: Current limit in amps (0-5 A for SPE15054).

        Returns:
            True if current limit was set successfully.
        """
        command = f"CURRent {current:.4f}"
        success, _ = self.send_command(command, no_response=True)
        if success:
            self.logger.info("Current limit set to %.4f A", current)
        else:
            self.logger.error("Failed to set current limit")
        return success

    def get_voltage(self) -> float:
        """Measure the actual output voltage.

        Returns:
            Measured voltage in volts, or -1.0 on error.
        """
        command = "MEASure:VOLTage?"
        success, response = self.send_command(command)
        if success:
            try:
                voltage = float(response)
                self.logger.info("Measured voltage: %.3f V", voltage)
                return voltage
            except (ValueError, TypeError):
                self.logger.error("Invalid voltage response: %s", response)
                return -1.0
        self.logger.error("Failed to measure voltage: %s", response)
        return -1.0

    def get_amperage(self) -> float:
        """Measure the actual output current.

        Returns:
            Measured current in amperes, or -1.0 on error.
        """
        command = "MEASure:CURRent?"
        success, response = self.send_command(command)
        if success:
            try:
                current = float(response)
                self.logger.info("Measured current: %s", self._format_current_ma(current))
                return current
            except (ValueError, TypeError):
                self.logger.error("Invalid current response: %s", response)
                return -1.0
        self.logger.error("Failed to measure current: %s", response)
        return -1.0

    # ------------------------------------------------------------------
    # Output control
    # ------------------------------------------------------------------

    def turn_on_output(self) -> bool:
        """Enable the power supply output and verify it is ON, SIN escalon.

        The OWON SPE15054 returns "1" (output ON) or "0" (output OFF) for
        OUTPut?. We check the current state first to avoid a redundant
        toggle, then send OUTPut ON and verify the state flipped.

        HABILITAR LA SALIDA ERA EL UNICO CAMINO QUE LA RAMPA NO CUBRIA
        -------------------------------------------------------------
        `set_voltage()` rampea en pasos de ~0.5 V «para evitar escalones que danen
        instrumentacion», pero rampea el SETPOINT: si la salida esta apagada, esa rampa
        ocurre sobre bornes desconectados y no protege nada. Despues `OUTPut ON` conecta el
        setpoint entero de una vez, que es exactamente el escalon que la rampa existe para
        evitar.

        Medido en el banco .202 el 07-Sep-2026: `set_voltage(24)` con la salida apagada
        —24 V rampeados sobre nada— y a continuacion `turn_on_output()` puso 24 V de golpe
        en el conector. Con el barrido a 48 V del `dc_current_check` eso llega al 96 % del
        nominal de los capacitores de bloqueo de la Rutherford, que son de 50 V
        (06035A101FAT2A, 100 pF C0G): 4 % de margen contra un transitorio.

        Asi que se baja a 0 V ANTES de habilitar —con la salida abierta, poner 0 V no es un
        escalon: no hay salida— y se sube con la rampa ya existente, que es la unica que ve
        el conector. El destino se relee de la fuente, no de `_voltage_setting`, porque la
        pregunta es que va a aplicar el equipo y no que creemos que pedimos.

        Returns:
            True if output is confirmed ON *and* at the requested voltage.
        """
        success, response = self.send_command("OUTPut?")
        if success and response.strip() in ("1", "ON"):
            self.logger.info("Power supply output is already ON")
            return True

        destino = self._destino_a_rampear()
        if destino is not None:
            self.send_command("VOLTage 0.000", no_response=True)
            self._voltage_setting = 0.0
            self.logger.info(
                "Salida apagada con %.3f V en el setpoint: se baja a 0 V antes de habilitar "
                "y se sube en rampa, para que el conector no vea un escalon", destino)

        ok, _ = self.send_command("OUTPut ON", no_response=True)
        if not ok:
            self.logger.error("Failed to send OUTPut ON")
            return False

        # Verify state flipped — OWON SPE15054 applies the command
        # synchronously so a single re-query is enough.
        ok2, state = self.send_command("OUTPut?")
        if not (ok2 and state.strip() in ("1", "ON")):
            self.logger.warning(
                "OUTPut ON sent but state reads %r — output may be OFF", state
            )
            return False

        self.logger.info("Power supply output confirmed ON")
        if destino is not None:
            # Ahora si: la rampa ocurre sobre la salida VIVA, que es donde protege.
            # set_voltage() relee la tension actual (0 V), rampea, fija el valor exacto y
            # lo verifica, asi que no se reimplementa nada aca.
            if not self.set_voltage(destino):
                self.logger.error(
                    "Salida habilitada pero la rampa a %.3f V fallo: queda en %.3f V",
                    destino, self._voltage_setting)
                return False
        return True

    def _destino_a_rampear(self) -> float | None:
        """La tension que habria que aplicar en rampa al habilitar, o None si no hace falta.

        None cuando la rampa esta deshabilitada, cuando no se puede leer la fuente, o cuando
        el setpoint es tan bajo que habilitar no es un escalon —el mismo umbral de 1 V que
        usa `_ramp_voltage` para saltearse.
        """
        if not self._ramp_enabled:
            return None
        ok, respuesta = self.send_command("VOLTage?")
        if not ok:
            return None
        try:
            destino = float(respuesta)
        except (ValueError, TypeError):
            self.logger.warning(
                "No se pudo leer el setpoint antes de habilitar (%r): se habilita sin rampa",
                respuesta)
            return None
        return destino if destino > 1.0 else None

    def turn_off_output(self) -> bool:
        """Disable the power supply output and verify it is OFF.

        BAJA A 0 V EN RAMPA ANTES DE CORTAR. El `finally` de execute_product_test() llama esto
        con 24 V puestos, y `OUTPut OFF` los cortaba de una vez. Se agrego el 23-Sep-2026 por
        prudencia, NO por un dano medido: segun Arturo ese mismo dia, lo que quema los switches
        de la Rutherford es el escalon de SUBIDA (lo cubre turn_on_output()), no la bajada desde
        24 V. Cuesta ~2 s por sesion; si ese tiempo llega a importar, esta rampa se puede sacar
        sin reabrir el defecto de la subida.

        SI LA RAMPA FALLA, NO SE CORTA: se deja la salida encendida y se devuelve False con un
        error, para no terminar en un corte a mitad de rampa.

        Returns:
            True if output is confirmed OFF after this call.
        """
        success, response = self.send_command("OUTPut?")
        if success and response.strip() in ("0", "OFF"):
            self.logger.info("Power supply output is already OFF")
            return True

        if self._destino_a_rampear() is not None and not self.set_voltage(0.0):
            self.logger.error(
                "No se pudo bajar a 0 V en rampa: la salida queda ENCENDIDA en vez de cortarla "
                "con tension puesta (un escalon quema los switches de la Rutherford)")
            return False

        ok, _ = self.send_command("OUTPut OFF", no_response=True)
        if not ok:
            self.logger.error("Failed to send OUTPut OFF")
            return False

        ok2, state = self.send_command("OUTPut?")
        if ok2 and state.strip() in ("0", "OFF"):
            self.logger.info("Power supply output confirmed OFF")
            return True

        self.logger.warning(
            "OUTPut OFF sent but state reads %r — output may still be ON",
            state,
        )
        return False


    # ------------------------------------------------------------------
    # Device identification
    # ------------------------------------------------------------------

    def _is_owon_device(self) -> tuple[bool, str]:
        """Verify the connected device is an OWON power supply.

        Returns:
            (is_owon: bool, identification_string: str)
        """
        return verify_idn(self.send_command, self.IDN_PREFIX)

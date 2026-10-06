import time
from controller.base.power_source import PowerSource
from controller.base.voltage_ramp import rampa_de_tension
from controller.base.scpi_commands import verify_idn
from controller.base.tcp_ip_controller import TcpIpController


class SiglentPowerSource(PowerSource, TcpIpController):
    """Fuente Siglent por SCPI/TCP. Verificada contra un **SPS5051X** el 16-Sep-2026.

    ── RAMPEA, IGUAL QUE LA OWON ─────────────────────────────────────────────────────────

    Subia de un escalon hasta el 16-Sep-2026, y la asimetria con la OWON estaba **documentada**
    — que no es lo mismo que no tenerla: obligaba a cada lector a entenderla y a decidir si le
    aplicaba. Ahora las dos usan `controller/base/voltage_ramp.py` y no hay diferencia que
    anotar.

    Por que la rampa existe, que se mantiene aunque hoy este banco no lo necesite: un escalon
    de 0V -> 12V/24V puede danar la instrumentacion de salida del tunel de RF (la de la
    Rutherford), y una bajada rapida CON CARGA puede disparar las protecciones de la propia
    fuente.

    ── LA SINTAXIS SCPI, VERIFICADA CONTRA EL INSTRUMENTO ────────────────────────────────

    Los comandos de abajo son de la serie SPD, con canal (`SOURce:VOLTage:SET CH1,<v>`). El
    SPS5051X del banco los acepta como alias: se le consulto `SOURce:VOLTage:SET? CH1` y
    contesto `17.000000` con `SYSTem:ERRor?` en `+0, No error`. No se dio por sentado porque
    son familias distintas y este driver no tenia ningun test propio.
    """

    def __init__(self, ip_address: str, port: int = 5025) -> None:
        super().__init__(ip_address=ip_address, port=port, device_name="Siglent Power Source")
        self.channel = 1
        #: Se pone en False en los tests, igual que en la OWON: rampear contra un mock solo
        #: gasta `time.sleep`.
        self._ramp_enabled = True
        is_match, message = verify_idn(self.send_command, "Siglent")
        if not is_match:
            raise ValueError(f"Error: {message}")

    def __exit__(self, exc_type, exc_val, exc_tb):
        try:
            self.turn_off_output()
        except (OSError, RuntimeError) as exc:
            self.logger.warning("Failed to turn off output during cleanup: %s", exc, exc_info=True)
        super().__exit__(exc_type, exc_val, exc_tb)
        return False

    def set_voltage(self, voltage: float) -> bool:
        """Fija la tension, POR RAMPA. Ver `controller/base/voltage_ramp.py`.

        Rampeaba de un escalon hasta el 16-Sep-2026 y la OWON no; la asimetria estaba
        documentada, que no es lo mismo que no tenerla. Ahora las dos usan la misma rampa y no
        hay diferencia que anotar.

        El punto de partida se LEE del instrumento, no se supone: si alguien movio la tension
        por el panel, rampear desde un valor imaginado es un escalon disfrazado.
        """
        if self._ramp_enabled:
            actual = self._leer_tension_fijada()
            if actual is not None and not rampa_de_tension(
                escribir=lambda v: self._escribir_tension(self.channel, v),
                actual_v=actual,
                objetivo_v=voltage,
                logger=self.logger,
            ):
                return False
        return self._set_voltage_from_channel(self.channel, voltage)

    def _escribir_tension(self, channel: int, voltage: float) -> bool:
        """Un paso de la rampa: escribe y nada mas. Sin verificacion ni espera de 1 s.

        Va aparte de `_set_voltage_from_channel()` a proposito: ese verifica con un `time.sleep(1)`
        y tres reintentos, que sobre 48 pasos de rampa son mas de un minuto. La verificacion se
        hace UNA vez, al final, sobre el valor que importa.
        """
        ok, _ = self.send_command(f"SOURce:VOLTage:SET CH{channel},{voltage:.3f}", no_response=True)
        return bool(ok)

    def _leer_tension_fijada(self) -> float | None:
        """La consigna vigente del instrumento, o None si no se pudo leer.

        None NO es 0.0: rampear «desde cero» cuando en realidad la fuente esta en 24 V es un
        salto hacia abajo de 24 V disfrazado de rampa. Si no se sabe de donde se parte, no se
        rampea — se escribe directo y se deja dicho en el log.
        """
        try:
            ok, resp = self.send_command(f"SOURce:VOLTage:SET? CH{self.channel}")
            return float(resp) if ok else None
        except (ValueError, TypeError, ConnectionError, OSError):
            self.logger.warning("no se pudo leer la consigna: se escribe sin rampa")
            return None

    def _set_voltage_from_channel(self, channel: int, voltage: float):
        """Set the voltage value of the selected channel.

        Args:
            channel: The channel number (1-based).
            voltage: The voltage value to set in volts.

        Returns:
            True if voltage was set and verified, False if verification failed.

        Raises:
            ConnectionError: If there's a problem communicating with the device.
        """
        # 1. Set the voltage
        set_command = f"SOURce:VOLTage:SET CH{channel},{voltage}"
        success, _ = self.send_command(
            set_command, no_response=True
        )  # send_command for single commands
        if not success:
            raise ConnectionError(f"Error sending command '{set_command}'")

        # 2. Query the set voltage to verify
        query_command = f"SOURce:VOLTage:SET? CH{channel}"
        time.sleep(1)
        verified = False
        last_response = "<no response>"
        for attempt in range(3):
            try:
                success, response = self.send_command(query_command)
                last_response = response
                if not success:
                    raise ConnectionError(f"Error sending command '{query_command}': {response}")
            except ConnectionError as exc:
                self.logger.warning("Connection error (attempt %s): %s", attempt + 1, exc)
                time.sleep(1)
                if attempt == 2:
                    raise ConnectionError(f"Error sending command '{query_command}': {last_response}")
                continue

            # 3. Compare the queried voltage with the desired voltage
            try:
                set_voltage = float(response)
                if abs(set_voltage - voltage) > 0.001:
                    self.logger.warning(
                        "Voltage mismatch: got %.3f, expected %.3f", set_voltage, voltage
                    )
                    return False
                else:
                    self.logger.info("Voltage set to %s successfully", voltage)
                    verified = True
                    break

            except (ValueError, TypeError):
                self.logger.error("Error parsing response to '%s': %s", query_command, response)

        return verified  # Return True if successful

    def get_voltage(self) -> float:
        return self._get_voltage_from_channel(self.channel)

    def get_amperage(self) -> float:
        return self._get_amperage_from_channel(self.channel)

    def _get_voltage_from_channel(self, channel: int) -> float:
        """Obtain the measured voltage value of the selected channel.

        Args:
            channel: The channel number (1-based).

        Returns:
            float: The measured voltage in volts.
                     Returns -1.0 if there's an error.

        Raises:
            ConnectionError:  If there's a problem communicating with the device.
        """
        command = f"MEASure:VOLTage? CH{channel}"
        success, response = self.send_command(command)
        if success:
            try:
                return float(response)
            except (ValueError, TypeError):
                self.logger.error("Error parsing voltage response: %r", response)
                return -1.0
        else:
            self.logger.error("Error getting voltage: %s", response)
            return -1.0

    def _get_amperage_from_channel(self, channel: int) -> float:
        """Obtain the measured current value of the selected channel.

        Args:
            channel: The channel number (1-based).

        Returns:
            float: The measured current in amperes.
                   Returns -1.0 if there's an error.

        Raises:
            ConnectionError: If there's a problem communicating with the device.
        """
        command = f"MEASure:CURRent? CH{channel}"
        success, response = self.send_command(command)
        if success:
            try:
                current = float(response)
                self.logger.info("Current measured: %s", self._format_current_ma(current))
                return current
            except (ValueError, TypeError):
                self.logger.error("Error parsing current response: %r", response)
                return -1.0
        else:
            self.logger.error("Error getting amperage: %s", response)
            return -1.0

    def set_current_limit(self, current: float) -> bool:
        """Set the current limit of the power source.

        Args:
            current: The current limit in amperes.

        Returns:
            True if current limit was set successfully.
        """
        command = f"SOURce:CURRent:SET CH{self.channel},{current}"
        success, _ = self.send_command(command, no_response=True)
        if success:
            self.logger.info("Current limit set to %s A successfully", current)
            return True
        self.logger.error("Error setting current limit")
        return False

    def turn_on_output(self) -> bool:
        """Turns on the output of the power source."""
        command = "OUTPut?"
        success, response = self.send_command(command)
        if success and response.strip().upper() in ("1", "ON"):
            self.logger.info("Power source output is already on")
            return True

        command = "OUTPut:STATe ON"
        success, _ = self.send_command(command, no_response=True)
        if success:
            self.logger.info("Power source output turned on successfully")
            return True
        else:
            self.logger.error("Error turning on output")
            return False

    def turn_off_output(self) -> bool:
        """Turns off the output of the power source."""
        command = "OUTPut?"
        success, response = self.send_command(command)
        if success and response.strip().upper() in ("0", "OFF"):
            self.logger.info("Power source output is already off")
            return True

        command = "OUTPut:STATe OFF"
        success, _ = self.send_command(command, no_response=True)
        if success:
            self.logger.info("Power source output turned off successfully")
            return True
        else:
            self.logger.error("Error turning off output")
            return False
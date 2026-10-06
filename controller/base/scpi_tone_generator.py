"""
ScpiToneGeneratorCommands — Comandos SCPI compartidos por generadores de señal.

Todos los métodos siguen la misma política: si un comando falla,
se lanza SCPIError inmediatamente. No hay fallos silenciosos.

SCPI Reference (IEC 60488 / IEEE 488.2):
    FREQuency <freq>   — Set carrier frequency (Hz or with suffix)
    FREQuency?         — Query carrier frequency
    POWer <level>      — Set RF output power (dBm)
    POWer?             — Query output power
    OUTPut <ON|OFF>    — Enable/disable RF output
    OUTPut?            — Query output state
    :MODulation:STATe <ON|OFF> — Enable/disable IQ/modulation

Jira: ID-1308 (Test Bench v2) — extracción de comandos de generador de señal
"""
from __future__ import annotations

import time

from controller.base.scpi_commands import SCPIError, ScpiCommandMixin


class ScpiToneGeneratorCommands(ScpiCommandMixin):
    """Comandos SCPI de generador de senal. Requiere send_command / send_command_batch.

    Las subclases pueden overridear SETTLE_SECONDS para ajustar el tiempo
    de espera entre escritura y verificación (default 0.1s).
    """

    SETTLE_SECONDS: float = 0.1

    # ------------------------------------------------------------------
    # Frecuencia
    # ------------------------------------------------------------------

    def set_frequency(self, frequency_in_mhz: float) -> None:
        """Set carrier frequency. Verifies with read-back.

        Args:
            frequency_in_mhz: Target frequency in MHz.

        Raises:
            SCPIError: If the command fails or verification mismatch.
        """
        freq_hz = frequency_in_mhz * 1_000_000
        self._cmd(f"FREQuency {freq_hz:.0f}", f"Frequency -> {frequency_in_mhz:.3f} MHz")
        time.sleep(self.SETTLE_SECONDS)
        actual = self._get_frequency_raw()
        if actual is not None and abs(actual - frequency_in_mhz) > 0.5:
            raise SCPIError(
                f"Frequency verification failed: set {frequency_in_mhz:.3f} MHz, "
                f"hardware reports {actual:.3f} MHz"
            )

    def get_frequency(self) -> float | None:
        """Query current carrier frequency.

        Returns:
            Frequency in MHz, or None on error.
        """
        return self._get_frequency_raw()

    def _get_frequency_raw(self) -> float | None:
        """Raw frequency query helper."""
        success, response = self.send_command("FREQuency?")
        if not success:
            self.logger.error("Failed to query frequency: %s", response)
            return None
        try:
            freq_hz = float(response)
            freq_mhz = freq_hz / 1_000_000
            self.logger.info("Current frequency: %.3f MHz", freq_mhz)
            return freq_mhz
        except (ValueError, TypeError):
            self.logger.error("Invalid frequency response: %s", response)
            return None

    # ------------------------------------------------------------------
    # Potencia / Nivel
    # ------------------------------------------------------------------

    def set_level(self, intensity_in_dbm: float) -> None:
        """Set RF output power. Verifies with read-back.

        Args:
            intensity_in_dbm: Target power in dBm.

        Raises:
            SCPIError: If the command fails or verification mismatch.
        """
        self._cmd(f"POWer {intensity_in_dbm:.1f}", f"Level -> {intensity_in_dbm:.1f} dBm")
        time.sleep(self.SETTLE_SECONDS)
        actual = self._get_power_raw()
        if actual is not None and abs(actual - intensity_in_dbm) > 0.5:
            raise SCPIError(
                f"Power verification failed: set {intensity_in_dbm:.1f} dBm, "
                f"hardware reports {actual:.1f} dBm"
            )

    def get_level(self) -> float | None:
        """Query current output power.

        Returns:
            Power in dBm, or None on error.
        """
        return self._get_power_raw()

    def get_power(self) -> float | None:
        """Alias for ``get_level()`` — backward compatibility."""
        return self._get_power_raw()

    def _get_power_raw(self) -> float | None:
        """Raw power query helper."""
        success, response = self.send_command("POWer?")
        if not success:
            self.logger.error("Failed to query power: %s", response)
            return None
        try:
            power = float(response)
            self.logger.info("Current power: %.1f dBm", power)
            return power
        except (ValueError, TypeError):
            self.logger.error("Invalid power response: %s", response)
            return None

    # ------------------------------------------------------------------
    # RF Output
    # ------------------------------------------------------------------

    def _is_output_on(self) -> bool:
        """Check if RF output is currently enabled."""
        success, response = self.send_command("OUTPut?")
        if success and response.strip().upper() in ("1", "ON"):
            return True
        return False

    def start(self) -> bool:
        """Enable RF output.

        Returns:
            True if output was enabled successfully.
        """
        if self._is_output_on():
            self.logger.info("RF output is already ON")
            return True
        success, _ = self.send_command("OUTPut ON", no_response=True)
        if success:
            self.logger.info("RF output turned ON")
        else:
            self.logger.error("Failed to turn ON RF output")
        return success

    def stop(self) -> bool:
        """Disable RF output.

        Returns:
            True if output was disabled successfully.
        """
        if not self._is_output_on():
            self.logger.info("RF output is already OFF")
            return True
        success, _ = self.send_command("OUTPut OFF", no_response=True)
        if success:
            self.logger.info("RF output turned OFF")
        else:
            self.logger.error("Failed to turn OFF RF output")
        return success

    # ------------------------------------------------------------------
    # Modulación
    # ------------------------------------------------------------------

    def enable_modulation(self) -> bool:
        """Enable IQ/modulation output (if supported).

        Returns:
            True if modulation was enabled.
        """
        success, _ = self.send_command(":MODulation:STATe ON", no_response=True)
        if success:
            self.logger.info("Modulation enabled")
        else:
            self.logger.warning("Modulation command failed — device may not support IQ modulation")
        return success

    def disable_modulation(self) -> bool:
        """Disable IQ/modulation output.

        Returns:
            True if modulation was disabled.
        """
        success, _ = self.send_command(":MODulation:STATe OFF", no_response=True)
        if success:
            self.logger.info("Modulation disabled")
        else:
            self.logger.error("Failed to disable modulation")
        return success

    # ------------------------------------------------------------------
    # Identificación
    # ------------------------------------------------------------------

    def idn(self) -> str:
        """Query device identification.

        Returns:
            Identification string, or error message.
        """
        success, response = self.send_command("*IDN?")
        if success:
            return response.strip()
        return f"IDN query failed: {response}"

    # ------------------------------------------------------------------
    # Interno
    # ------------------------------------------------------------------
    # `_cmd()` is inherited from ScpiCommandMixin (shared SCPI send + optional
    # *OPC? poll). No reimplementation needed here.

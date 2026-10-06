"""
Dummy OWON Power Source — for testing without physical hardware.

Accepts any parameters but ignores them. Logs all operations.
"""
import logging

from controller.base.power_source import PowerSource
from controller.dummy.mixin import DummyFallbackMixin


class DummyOwonPowerSource(DummyFallbackMixin, PowerSource):
    """Dummy controller for OWON SPE15054 power supply."""

    def __init__(self, **kwargs):
        super().__init__()
        self.logger = logging.getLogger(self.__class__.__name__)
        self.logger.info("OWON PS: OWON SPE15054 Power Source initialized (dummy)")

    def set_voltage(self, voltage: float) -> bool:
        self.logger.info("OWON PS: Set voltage -> %.3f V", voltage)
        return True

    def get_voltage(self) -> float:
        self.logger.info("OWON PS: Get voltage -> 12.000 V (dummy)")
        return 12.0

    def get_amperage(self) -> float:
        self.logger.info("OWON PS: Get amperage -> 0.500 A (dummy)")
        return 0.5

    def set_current_limit(self, current: float) -> bool:
        self.logger.info("OWON PS: Set current limit -> %.4f A", current)
        return True

    def turn_on_output(self) -> bool:
        self.logger.info("OWON PS: Output ON")
        return True

    def turn_off_output(self) -> bool:
        self.logger.info("OWON PS: Output OFF")
        return True

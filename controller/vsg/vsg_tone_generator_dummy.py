"""
Dummy VSG25 Tone Generator — for testing without physical hardware.

Accepts any parameters but ignores them. Prints all operations to stdout.
"""
import logging

from controller.base.tone_generator import ToneGenerator
from controller.dummy.mixin import DummyFallbackMixin


class DummyVsGenerator(DummyFallbackMixin, ToneGenerator):
    """Dummy controller for VSG25 Vector Signal Generator.

    Implements the ToneGenerator interface as a no-op. Useful for CI/CD,
    development, and test suite validation without hardware.
    """

    def __init__(self, **kwargs):
        """Initialize dummy controller — accepts any kwargs, ignores them."""
        super().__init__()
        self.logger = logging.getLogger(self.__class__.__name__)
        self.logger.info("VSG: VSG25 Vector Signal Generator initialized (dummy)")

    def set_level(self, intensity_in_dbm: float) -> None:
        self.logger.info("VSG: Output power -> %.1f dBm", intensity_in_dbm)

    def set_frequency(self, frequency_in_mhz: float) -> None:
        self.logger.info("VSG: Frequency -> %.3f MHz", frequency_in_mhz)

    def start(self) -> bool:
        self.logger.info("VSG: RF output ON")
        return True

    def stop(self) -> bool:
        self.logger.info("VSG: RF output OFF")
        return True

    def get_frequency(self) -> float:
        self.logger.info("VSG: Get frequency -> 150.000 MHz (dummy)")
        return 150.0

    def get_power(self) -> float:
        self.logger.info("VSG: Get power -> -10.0 dBm (dummy)")
        return -10.0

    def enable_modulation(self):
        self.logger.info("VSG: Modulation ON")

    def disable_modulation(self):
        self.logger.info("VSG: Modulation OFF")

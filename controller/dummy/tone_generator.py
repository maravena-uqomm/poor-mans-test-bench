from controller.base.tone_generator import ToneGenerator
from controller.dummy.mixin import DummyFallbackMixin
import logging


class DummyToneGenerator(DummyFallbackMixin, ToneGenerator):

    def __init__(self, **kwargs):
        super().__init__()
        self.logger = logging.getLogger(self.__class__.__name__)
        self.logger.info("GT: Generador de tono inicializado (dummy)")

    def set_level(self, intensity_in_dbm: float) -> None:
        self.logger.info("GT: Se ajusta intensidad de tono en %s dBm", intensity_in_dbm)

    def set_frequency(self, frequency_in_mhz: float) -> None:
        self.logger.info("GT: Se ajusta frecuencia de tono en %s MHz", frequency_in_mhz)

    def start(self) -> bool:
        self.logger.info("GT: Tone ON")
        return True

    def stop(self) -> bool:
        self.logger.info("GT: Tone OFF")
        return True

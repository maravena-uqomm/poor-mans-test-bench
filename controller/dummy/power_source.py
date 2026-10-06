from controller.base.power_source import PowerSource
from controller.dummy.mixin import DummyFallbackMixin
import logging


class DummyPowerSource(DummyFallbackMixin, PowerSource):

    def __init__(self, **kwargs):
        super().__init__()
        self.logger = logging.getLogger(self.__class__.__name__)
        self.logger.info("PS: Fuente de poder inicializada (dummy)")

    def set_voltage(self, voltage: float) -> bool:
        self.logger.info("PS: Voltaje configurado a %s V", voltage)
        return True

    def get_voltage(self) -> float:
        return 24.0

    def get_amperage(self) -> float:
        return 0.5

    def set_current_limit(self, current: float) -> bool:
        self.logger.info("PS: Límite de corriente configurado a %s A", current)
        return True

    def turn_on_output(self) -> bool:
        self.logger.info("PS: Salida encendida")
        return True

    def turn_off_output(self) -> bool:
        self.logger.info("PS: Salida apagada")
        return True

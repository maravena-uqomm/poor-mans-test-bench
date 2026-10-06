class Amplifier:
    """Interfaz del amplificador bajo prueba (ULAD / VLAD).

    La ganancia y el AGC se manejan en valor CRUDO del DAC (0-4095). El porcentaje es una
    conveniencia de presentación: la conversión redondea, así que el crudo es el valor
    autoritativo y es el que hay que registrar.
    """

    GAIN_MAX = 4095
    AGC_MAX = 4095
    ATT_MAX = 31

    def set_gain(self, side: str, raw: int) -> bool:
        pass

    def get_gain(self, side: str) -> int:
        pass

    def set_agc(self, side: str, raw: int) -> bool:
        pass

    def get_agc(self, side: str) -> int:
        pass

    def get_attenuation(self) -> dict:
        pass

    def get_telemetry(self) -> dict:
        pass

    def close(self):
        pass

    @staticmethod
    def raw_to_percent(raw: int) -> int:
        return round((raw / 4095) * 100)

    @staticmethod
    def percent_to_raw(percent: float) -> int:
        return round((percent / 100) * 4095)

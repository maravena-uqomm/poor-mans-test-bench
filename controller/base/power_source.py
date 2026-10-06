class PowerSource:
    """Interfaz para fuentes de poder."""

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)

    @staticmethod
    def _format_current_ma(current_amps: float) -> str:
        """Format a current in amperes as a human-readable mA string."""
        current_ma = current_amps * 1000
        if current_ma < 1:
            return f"{current_ma:.2f} mA"
        elif current_ma < 100:
            return f"{current_ma:.1f} mA"
        return f"{current_ma:.0f} mA"

    def set_voltage(self, voltage: float) -> bool:
        raise NotImplementedError

    def get_voltage(self) -> float:
        raise NotImplementedError

    def get_amperage(self) -> float:
        raise NotImplementedError

    def set_current_limit(self, current: float) -> bool:
        raise NotImplementedError

    def turn_on_output(self) -> bool:
        raise NotImplementedError

    def turn_off_output(self) -> bool:
        raise NotImplementedError

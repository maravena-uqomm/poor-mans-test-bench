class ToneGenerator:
    """Interfaz para generadores de tono/señal."""

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)

    def set_level(self, intensity_in_dbm: float) -> None:
        ...

    def set_frequency(self, frequency_in_mhz: float) -> None:
        ...

    def start(self) -> bool:
        ...

    def stop(self) -> bool:
        ...

    def rf_is_on(self) -> bool | None:
        """Estado de la salida RF, o ``None`` si el driver no lo lleva.

        La usa ``suite/test.py::switch_direction`` para no dejar la RF encendida donde
        estaba apagada: la placa Rutherford exige generador en minimo o apagado durante las
        transiciones de reset, y conmutar con RF encima castiga al instrumento contra el
        corto reflectivo del SP4T.

        ``None`` significa "no se sabe", no "apagado": el helper apaga y restaura igual.
        """
        return getattr(self, "_rf_on", None)

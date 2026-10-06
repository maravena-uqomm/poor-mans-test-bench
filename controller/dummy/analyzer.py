from time import sleep
from controller.base.analyzer import Analyzer
from controller.dummy.mixin import DummyFallbackMixin
import logging

_SIM_DELAY = 0.02


class DummyAnalyzer(DummyFallbackMixin, Analyzer):

    #: Nivel que devuelve el dummy, ANTES del offset.
    NIVEL_BASE_DBM = -13.0

    #: Offset de nivel, modelado igual que en el driver real.
    #:
    #: Sin esto, el `__getattr__` catch-all del mixin se tragaba `set_ref_db_offset` —loggeaba
    #: un warning y devolvia None— asi que un ensayo en seco con dummies NUNCA modelaba la
    #: correccion. Una tabla de offsets configurada desde el backend podia estar rota de punta
    #: a punta y el dry run se veia igual de sano. El dummy tiene que mentir como el real, no
    #: mejor que el real.
    _ref_db_offset: float = 0.0

    def __init__(self, **kwargs):
        """Controlador dummy - acepta cualquier parámetro pero los ignora"""
        super().__init__()
        self.logger = logging.getLogger(self.__class__.__name__)
        self.logger.info("AN: Analizador inicializado (dummy)")

    def set_ref_db_offset(self, offset: float) -> None:
        """Guarda el offset y lo aplica a los niveles, como hace `OwonAnalyzerUsb`."""
        self._ref_db_offset = float(offset)
        self.logger.info("AN: Ref offset -> %+.2f dB (dummy, aplicado en software)", offset)

    def center_and_span_panel(self, ul_freq: float, dl_freq: float, span: float):
        centro = (ul_freq + dl_freq) / 2
        self.logger.info("AN: Se centra panel con frecuencia en %s y con span de %s", centro, span)
        sleep(_SIM_DELAY)

    def mark_amplitude(self, amp_in_dbm: float):
        self.logger.info("AN: Amplitud: %s dBm", amp_in_dbm)
        sleep(_SIM_DELAY)

    def mark_freq(self, freq_in_mhz: float):
        self.logger.info("AN: Marco frecuencia %s MHz", freq_in_mhz)
        sleep(_SIM_DELAY)

    def get_level_from_freq(self, freq_in_mhz: float) -> float:
        level = -25.5 + self._ref_db_offset
        self.logger.info("AN: Obtengo intensidad %s dBm en %s MHz", level, freq_in_mhz)
        sleep(_SIM_DELAY)
        return level

    def normalize(self):
        self.logger.info("AN: Normalizo")
        sleep(_SIM_DELAY)

    def max_hold(self):
        self.logger.info("AN: Mantengo máximo")
        sleep(_SIM_DELAY)

    def freeze_image(self):
        self.logger.info("AN: Congelo imagen")
        sleep(_SIM_DELAY)

    def get_image(self) -> str:
        self.logger.info("AN: Obtengo imagen")
        sleep(_SIM_DELAY)
        return "dummy_image_data"

    def set_resolution_bandwidth_khz(self, rbw_khz: float):
        self.logger.info("AN: Establezco RBW a %s kHz", rbw_khz)
        sleep(_SIM_DELAY)

    def turn_off_tracking_generator(self):
        self.logger.info("AN: Apago generador de tracking")
        sleep(_SIM_DELAY)

    def turn_on_tracking_generator(self):
        self.logger.info("AN: Enciendo generador de tracking")
        sleep(_SIM_DELAY)

    def set_trace_mode(self, trace_number: int = 1, mode: str = "WRITe"):
        self.logger.info("AN: Set trace %d mode to '%s'", trace_number, mode)
        sleep(_SIM_DELAY)

    def get_level_from_marker(self, marker: int = 1, *args, **kwargs) -> float:
        """Nivel simulado, con el offset aplicado igual que en el instrumento real."""
        return self.NIVEL_BASE_DBM + self._ref_db_offset

    def set_sweep_time(self, time_ms: int):
        self.logger.info("AN: Sweep time -> %s ms", time_ms)

    def set_markers(self, markers: list):
        self.logger.info("AN: Set markers %s", markers)

    def set_marker_bw(self, marker: int, state: str, dBm: float):
        self.logger.info("AN: Marker %d BW %s @ %s dBm", marker, state, dBm)

    def get_marker_bw_result(self, marker: int) -> float:
        self.logger.info("AN: Marker %d BW result", marker)
        return 0.5

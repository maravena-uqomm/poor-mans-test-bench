"""
Dummy OWON Analyzer — for testing without physical hardware.

Accepts any parameters but ignores them. Logs all operations.
"""
import logging

from controller.base.analyzer import Analyzer
from controller.dummy.mixin import DummyFallbackMixin


class DummyOwonAnalyzer(DummyFallbackMixin, Analyzer):
    """Dummy controller for OWON VSA815PTG spectrum analyzer."""

    def __init__(self, **kwargs):
        super().__init__()
        self.logger = logging.getLogger(self.__class__.__name__)
        self.logger.info("OWON SA: OWON VSA815PTG Analyzer initialized (dummy)")

    def center_and_span_panel(self, ul_freq: float, dl_freq: float, span: float):
        center = (ul_freq + dl_freq) / 2
        self.logger.info("OWON SA: Panel -> center=%.1f MHz, span=%.1f MHz (UL=%.1f, DL=%.1f)", center, span, ul_freq, dl_freq)

    def mark_amplitude(self, amp_in_dbm: float):
        self.logger.info("OWON SA: Mark amplitude -> %.1f dBm", amp_in_dbm)

    def mark_freq(self, freq_in_mhz: float):
        self.logger.info("OWON SA: Mark frequency -> %.3f MHz", freq_in_mhz)

    def get_level_from_freq(self, freq_in_mhz: float) -> float:
        self.logger.info("OWON SA: Get level @ %.3f MHz -> -65.0 dBm (dummy)", freq_in_mhz)
        return -65.0

    def get_level_from_marker(self, marker: int = 1, **kwargs) -> float:
        self.logger.info("OWON SA: Get level from marker %d -> -65.0 dBm (dummy)", marker)
        return -65.0

    def normalize(self):
        self.logger.info("OWON SA: Normalize trace")

    def max_hold(self):
        self.logger.info("OWON SA: Max hold ON")

    def freeze_image(self):
        self.logger.info("OWON SA: Freeze display")

    def get_image(self) -> str:
        self.logger.info("OWON SA: Get screenshot -> (dummy image data)")
        return "dummy_image_data"

    def set_center_freq(self, center_freq: float):
        self.logger.info("OWON SA: Center frequency -> %.1f MHz", center_freq)

    def set_span(self, span: float):
        self.logger.info("OWON SA: Span -> %.1f MHz", span)

    def set_reference_level(self, level: float):
        self.logger.info("OWON SA: Reference level -> %.1f dBm", level)

    def set_resolution_bandwidth_khz(self, resolution: float):
        self.logger.info("OWON SA: RBW -> %.1f kHz", resolution)

    def set_tracking_generator_level(self, level: float):
        self.logger.info("OWON SA: TG level -> %.1f dBm", level)

    def set_tracking_generator(self, state: str):
        self.logger.info("OWON SA: TG -> %s", state)

    def turn_on_tracking_generator(self):
        self.logger.info("OWON SA: TG ON")

    def turn_off_tracking_generator(self):
        self.logger.info("OWON SA: TG OFF")

    def set_marker(self, marker_index: int = 1, frequency: float = 0.0):
        self.logger.info("OWON SA: Marker %d -> %.3f MHz", marker_index, frequency)

    def set_markers(self, markers: list):
        self.logger.info("OWON SA: Set markers %s", markers)

    def set_markers_off(self):
        self.logger.info("OWON SA: All markers OFF")

    def set_marker_bw(self, marker: int, state: str, dBm: float):
        self.logger.info("OWON SA: Marker %d BW %s NDB @ %.1f dBm", marker, state, dBm)

    def get_marker_bw_result(self, marker: int) -> float:
        self.logger.info("OWON SA: Marker %d BW result -> 0.50 MHz (dummy)", marker)
        return 0.50

    def set_trace_mode(self, trace_number: int = 1, mode: str = "WRITe"):
        self.logger.info("OWON SA: Trace %d mode -> %s", trace_number, mode)

    def get_trace_data(self, trace_number: int = 1) -> list:
        self.logger.info("OWON SA: Get trace %d -> (dummy data)", trace_number)
        return [(145.0, -65.0), (150.0, -60.0)]

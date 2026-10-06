"""DummyHardResetController — dummy EXPLÍCITO, no el catch-all genérico.

Deliberadamente NO usa `DummyFallbackMixin` (ver `AGENTS.md` y el docstring de
`HardResetController`): un `power_off()` que no hace nada y no lo dice es exactamente el
tipo de "driver fantasma" que el repo prohíbe para los drivers reales, y acá es más
peligroso todavía porque nadie se entera de que el corte de energía real no ocurrió.
"""

from __future__ import annotations

import logging

from controller.base.hard_reset import HARD_RESET_PULSE_S, HardResetController


class DummyHardResetController(HardResetController):
    def __init__(self, **kwargs: object) -> None:
        self.logger = logging.getLogger(self.__class__.__name__)
        self.power_off_called = False

    def who(self) -> bool:
        self.logger.warning("hard_reset DUMMY: who() — no hay rele real, simulando 'no es un rele'")
        return False

    def power_off(self) -> None:
        self.power_off_called = True
        self.logger.warning(
            "hard_reset DUMMY: power_off() llamado — NO SE CORTO NINGUNA ENERGIA REAL. "
            "Si esto corre en un banco real, el rol hard_reset esta mal configurado."
        )

    def wait_reenergized(self, hold_s: float = HARD_RESET_PULSE_S) -> None:
        self.logger.warning("hard_reset DUMMY: wait_reenergized(%.1fs) — no hay nada que esperar de verdad", hold_s)

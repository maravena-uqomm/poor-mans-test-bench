"""CRFS — interfaz legacy para router RF.

Deprecated: ``Crfs`` is now a thin alias for :class:`Rutterford`. The
canonical routing API (``set_mode_uplink`` / ``set_mode_downlink`` / legacy
aliases) lives in ``controller.base.rutterford``. Kept only so existing
suites that do ``from controller.base.crfs import Crfs`` keep working.
"""

from controller.base.rutterford import Rutterford


class Crfs(Rutterford):
    """Legacy alias for :class:`Rutterford`. Prefer ``Rutterford`` for new code."""

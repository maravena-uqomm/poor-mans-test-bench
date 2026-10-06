"""WatchdogMixin — watchdog timer compartido para controladores.

La clase anfitriona debe proporcionar:
- ``self.logger``
- ``self._state`` (``InstrumentState``)
- ``self._watchdog_device_name`` (str, para logging)
"""

from __future__ import annotations

import threading

from controller.base.reliability import InstrumentState


class WatchdogMixin:
    """Mixin que añade un watchdog timer a cualquier controlador.

    Detecta operaciones colgadas y marca el estado como ERROR.
    La clase concreta debe llamar ``_init_watchdog()`` en su ``__init__``.
    """

    def _init_watchdog(self, default_timeout: float = 10.0) -> None:
        """Inicializa atributos del watchdog. Llamar desde ``__init__``."""
        self._watchdog_timer: threading.Timer | None = None
        self._watchdog_default_timeout: float = default_timeout
        self._watchdog_current_timeout: float = default_timeout
        self._is_hung = False

    def _start_watchdog(self, timeout: float | None = None) -> None:
        """Inicia el watchdog timer para detectar colgazos."""
        if self._watchdog_timer is not None:
            self._watchdog_timer.cancel()
        self._is_hung = False
        self._watchdog_current_timeout = timeout if timeout is not None else self._watchdog_default_timeout
        self._watchdog_timer = threading.Timer(
            self._watchdog_current_timeout, self._watchdog_expired
        )
        self._watchdog_timer.daemon = True
        self._watchdog_timer.start()

    def _watchdog_expired(self) -> None:
        """Callback cuando el watchdog expira — la operación se colgó."""
        self._is_hung = True
        self.logger.error(
            "[%s] WATCHDOG EXPIRED: hung for >%.1fs",
            self._watchdog_device_name,
            self._watchdog_current_timeout,
        )
        self._state = InstrumentState.ERROR

    def _stop_watchdog(self) -> None:
        """Detiene el watchdog cuando la operación completa."""
        if self._watchdog_timer is not None:
            self._watchdog_timer.cancel()
            self._watchdog_timer = None

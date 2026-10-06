"""
DummyFallbackMixin — catch-all genérico para controladores dummy.

Cualquier método no implementado explícitamente se loggea con WARNING
y retorna None. Evita que un typo en un método real pase desapercibido.

Extraído de 8 duplicaciones idénticas en los dummies.
"""

from __future__ import annotations

import logging

LOGGER = logging.getLogger(__name__)


class DummyFallbackMixin:
    """Mixin que implementa __getattr__ catch-all para dummies.

    Retorna None loggeando WARNING, para que tests no crasheen en metodos
    no implementados. Si se necesita strict mode, usar __getattr__ que levante
    AttributeError directamente.
    """

    def __getattr__(self, name: str):
        def method(*args, **kwargs):
            LOGGER.warning(
                "Fake method not implemented: %s.%s(args=%r, kwargs=%r)",
                self.__class__.__name__, name, args, kwargs,
            )
            return None
        return method

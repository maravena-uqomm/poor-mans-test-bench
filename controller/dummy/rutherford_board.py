"""DummyRutherfordBoard — la placa Rutherford sin hardware, con el ruteo de verdad.

Uso:  board = DummyRutherfordBoard()   # se registra como rutterford.type = "dummy"
No hace: RF. El ruteo es real hasta el borde del transporte y ahi lo atiende un simulador.

Hereda de ``RutherfordBoard`` a proposito: lo unico que cambia es el transporte. Antes eran
dos implementaciones separadas del mismo mapeo y las dos arrastraban el mismo bug —un
``reset()`` entre las dos rutas, que borraba la primera—, porque arreglar una no arreglaba
la otra. Con la herencia, cualquier cambio de ruteo se prueba una vez y vale para las dos.
"""

from __future__ import annotations

from controller.dummy.mixin import DummyFallbackMixin
from controller.dummy.rutherford_transport import SimulatedTransport
from controller.uqomm.rutherford_board import RutherfordBoard


class DummyRutherfordBoard(DummyFallbackMixin, RutherfordBoard):
    """``RutherfordBoard`` sobre ``SimulatedTransport``: mismo ruteo, sin placa."""

    def __init__(self, **kwargs: object) -> None:
        # La config del banco llega entera (ip_address, port, device_path...) y aca no
        # significa nada: el transporte es el simulador. Se descarta y no se reenvia al padre.
        _ = kwargs
        super().__init__(port="simulated://", timeout_s=1.0, transport=SimulatedTransport())
        self.logger.info("DummyRutherfordBoard initialised (crossbar en memoria)")

    def idn(self) -> str:
        # Reporta el estado simulado, no una constante: si el ruteo del padre se rompe, el
        # dummy tiene que poder mostrarlo igual que la placa.
        return f"Dummy {super().idn()}"

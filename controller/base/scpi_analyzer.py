"""
ScpiAnalyzer — Analizador SCPI sobre TCP/IP.

Hereda de ScpiCommandMixin (comandos SCPI), Analyzer (interfaz) y
TcpIpController (transporte TCP/IP). Solo agrega la validación IDN
y la inicialización del transporte TCP.

Uso:
    class MiAnalyzer(ScpiAnalyzer):
        IDN_PREFIX = "SIGLENT"

Jira: ID-1308 (Test Bench v2)
"""
from __future__ import annotations

from controller.base.analyzer import Analyzer
from controller.base.scpi_commands import ScpiCommandMixin
from controller.base.tcp_ip_controller import TcpIpController


class ScpiAnalyzer(ScpiCommandMixin, Analyzer, TcpIpController):
    """Analizador SCPI sobre TCP/IP.

    Las subclases solo definen:
        IDN_PREFIX: str  — ej: "SIGLENT", "OWON"
        DEFAULT_PORT: int — ej: 5025
    """

    IDN_PREFIX: str = ""
    DEFAULT_PORT: int = 5025

    def __init__(self, ip_address: str, port: int | None = None):
        super().__init__(
            ip_address=ip_address, port=port or self.DEFAULT_PORT,
            device_name=f"{self.IDN_PREFIX} Analyzer",
        )
        ok, idn = self._check_idn()
        if not ok:
            raise ValueError(
                f"Dispositivo en {ip_address}:{port or self.DEFAULT_PORT} "
                f"no es un {self.IDN_PREFIX}: {idn}"
            )
        self.logger.info("%s Analyzer inicializado — %s", self.IDN_PREFIX, idn)

    def _check_idn(self) -> tuple[bool, str]:
        success, response = self.send_command("*IDN?")
        print(f"Respuesta CHECKIDN: {response}")
        if success and response.strip().upper().startswith(self.IDN_PREFIX.upper()):
            return True, response.strip()
        return False, response.strip() if success else "sin respuesta"

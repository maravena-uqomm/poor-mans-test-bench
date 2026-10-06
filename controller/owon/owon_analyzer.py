"""
OWON VSA815PTG — Analizador de espectro SCPI sobre TCP/IP.

Todo los comandos SCPI comunes viven en ScpiAnalyzer.
Esta clase solo configura el prefijo IDN y el puerto por defecto.

Device specs:
  - Frequency range: 9 kHz – 1.5 GHz
  - DANL: <= -160 dBm (typ, preamp ON, RBW=1 Hz)
  - RBW: 1 Hz – 1 MHz
  - Interfaces: USB 2.0, LAN (TCP/IP SCPI)
  - Tracking generator: integrated (TG suffix)

Jira: ID-1308 (Test Bench v2) / ID-1386, ID-1530, ID-1540
"""
from controller.base.scpi_analyzer import ScpiAnalyzer


class OwonAnalyzer(ScpiAnalyzer):
    IDN_PREFIX = "OWON"
    DEFAULT_PORT = 5025

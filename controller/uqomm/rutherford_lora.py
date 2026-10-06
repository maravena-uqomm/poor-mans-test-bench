"""RutherfordLora — las dos radios LoRa de la placa Rutherford, con el contrato de aire del VLAD25.

Uso:  lora = board.lora()                       # comparte puerto serie y lock con el ruteo
      lora.use_link(Link.DOWNLINK, 149_500_000); lora.transmit(b"..."); lora.wait_packet(3.0)
No hace: ruteo de RF (eso es RutherfordBoard), ni armado de tramas del VLAD (vlad_lora_link.py).

Modelo del firmware (fw-rutherford, AppRutherford::LoraRequestHandle):

  * **Un link por vez.** DOWNLINK: la radio Base transmite y la Tunnel escucha. UPLINK: la
    Tunnel transmite y la Base escucha. SMART_RING: solo transmite la Base. Cambiar de link
    exige `lora_reset()`; un segundo configure devuelve 302 (CONFIGURED_TWICE). Por eso
    `use_link()` hace siempre reset + configure, igual que el ruteo hace reset + rutas.
  * **El receptor queda escuchando al configurar.** `lora_receive()` no espera: devuelve el
    paquete que ya llego o vacio. Esperar es sondear.
  * **Parametros de aire fijos**: BW125, SF7, CR 4/5 y la potencia por defecto del driver.
    Son los que `Sx1278Peripheral::configure()` aplica, y el firmware rechaza cualquier otro
    en vez de ignorarlo.

LIMITE que hay que conocer para no leer mal un resultado: la pylib 0.3.0 descarta el
`ReceptionState`. Un paquete con CRC de payload malo llega igual que uno bueno, y "vacio"
significa "todavia nada". Quien interpreta el payload valida su propio CRC.
"""

from __future__ import annotations

import logging
import threading
import time
from collections.abc import Callable
from typing import TypeVar

from rutherford import LoraConfig, RutherfordClient
from rutherford.errors import RutherfordError, TransportTimeoutError
from rutherford.models import Link, LinkBandwidth, LinkCodingRate, LinkPower, LinkSpreadFactor, SpectralBand

#: Frecuencias del contrato de aire con el VLAD25 (fw-vlad/vlad25_vhf/Core/Inc/LoraContrato.hpp).
DOWNLINK_HZ = 149_500_000
UPLINK_HZ = 173_500_000
SMART_RING_HZ = 143_000_000

#: Periodo de sondeo de `wait_packet`. El lazo del firmware da una vuelta cada ~5 ms; sondear
#: mas rapido que eso solo ocupa el puerto serie.
_POLL_S = 0.02

_T = TypeVar("_T")


class RutherfordLora:
    """Acceso a las radios LoRa sobre un `RutherfordClient` ya abierto.

    Args:
        client: El cliente de la placa. Se comparte con `RutherfordBoard`: un segundo dueno
            del mismo CDC desincronizaria las tramas de los dos.
        lock: El lock de `RutherfordBoard`, por la misma razon.
        flush: Descarta el buffer de entrada tras un timeout (`RutherfordBoard._flush_serial`).
    """

    def __init__(
        self,
        client: RutherfordClient,
        lock: threading.RLock,
        flush: Callable[[], None] | None = None,
    ) -> None:
        self.logger = logging.getLogger(self.__class__.__name__)
        self._client = client
        self._lock = lock
        self._flush = flush or (lambda: None)
        self._link: Link | None = None

    # ------------------------------------------------------------------
    # Control
    # ------------------------------------------------------------------

    def reset(self) -> None:
        """Resetea las dos radios. Deja la placa sin link configurado."""
        self._call(self._client.lora_reset)
        self._link = None

    def use_link(self, link: Link, frequency_hz: int) -> None:
        """Deja configurado `link` en `frequency_hz`, sea cual sea el link anterior."""
        config = LoraConfig(
            band=SpectralBand.VHF,
            link=link,
            bandwidth=LinkBandwidth.BW_125,
            spreading_factor=LinkSpreadFactor.SF_128,
            coding_rate=LinkCodingRate.CR_4_5,
            link_power=LinkPower.LINK_POWER_UNSPECIFIED,
            frequency_hz=frequency_hz,
        )
        self.reset()
        self._call(self._client.lora_configure_link, config)
        self._link = link
        self.logger.info("RutherfordLora: link %s en %.3f MHz", link.name, frequency_hz / 1e6)

    def transmit(self, payload: bytes) -> None:
        """Transmite por la radio transmisora del link. Vuelve despues del TxDone."""
        self._call(self._client.lora_transmit, payload)

    # ------------------------------------------------------------------
    # Consulta
    # ------------------------------------------------------------------

    def receive(self) -> bytes:
        """El paquete que ya llego al receptor del link, o `b""` si no hay ninguno."""
        return self._call(self._client.lora_receive)

    def wait_packet(self, timeout_s: float) -> bytes | None:
        """Sondea el receptor hasta `timeout_s`. `None` si no llego nada."""
        deadline = time.monotonic() + timeout_s
        while True:
            payload = self.receive()
            if payload:
                return payload
            if time.monotonic() >= deadline:
                return None
            time.sleep(_POLL_S)

    @property
    def link(self) -> Link | None:
        """El link configurado por este objeto, o `None` despues de un reset."""
        return self._link

    # ------------------------------------------------------------------
    # Interno
    # ------------------------------------------------------------------

    def _call(self, method: Callable[..., _T], *args: object) -> _T:
        with self._lock:
            try:
                return method(*args)
            except TransportTimeoutError:
                self._flush()
                raise
            except RutherfordError as exc:
                self.logger.error("RutherfordLora: %s fallo: %s", getattr(method, "__name__", method), exc)
                raise

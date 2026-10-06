"""La fuente del banco se ELIGE, y el puerto va con la marca. Un solo lugar.

Uso:   agregar_argumentos(ap)                       # --supply-type / --supply-ip / --supply-port
       fuente = construir_fuente(args)              # ya con el puerto por defecto de la marca
NO hace: no enciende la salida ni fija tension. Solo decide QUE clase y CON QUE puerto.

── POR QUE EXISTE: EL MISMO DEFECTO, TRES VECES ──────────────────────────────────────────

`instrument_smoke.py`, `suite/vlad25_panel_cal.py` y `suite/standalone/vlad25_rf_cal_bench.py`
tenian la fuente CLAVADA a `OwonPowerSource` en `192.168.1.201`. En un banco con Siglent ninguno
arrancaba. Se arreglaron los dos primeros por separado —cada uno con su propia copia del
diccionario marca->puerto— y el tercero siguio roto hasta el 17-Sep-2026, que es como se nota
que el arreglo no habia sido el correcto.

Son TRES consumidores contados, no una familia imaginada: por eso se extrae.

── EL PUERTO ES LA TRAMPA, Y NO ES SIMETRICA ─────────────────────────────────────────────

  OWON SPE15054  ->  4196
  Siglent SPS50xx ->  5025

Confundirlos no da un error legible: da un timeout, y un timeout se lee como «la fuente no
esta». `vlad25_rf_cal_bench.py` ya lo documentaba —«NO es 5025: el 5025 no responde y se leyo
como fuente muerta»— para la OWON. Para la Siglent es exactamente al reves.

Por eso el puerto por defecto SALE DE LA MARCA y no de una constante suelta: elegir la clase y
olvidarse del puerto es el error que este modulo vuelve imposible.

── LAS IPs NO VIVEN ACA ──────────────────────────────────────────────────────────────────

Una IP en el codigo envejece: la Siglent del banco estuvo en 192.168.60.58 y paso a
192.168.1.137 en la LAN local. El sintoma fueron diez timeouts del driver, y lo que lo resolvio
en un paso fue el ping. La IP va por `--supply-ip` o por el entorno (`POWER_SOURCE_IP`), que es
donde se puede cambiar sin tocar un archivo versionado.
"""

from __future__ import annotations

import argparse
import os

from controller.base.power_source import PowerSource
from controller.owon.owon_power_source import OwonPowerSource
from controller.siglent.siglent_power_source import SiglentPowerSource

#: marca -> (clase, puerto SCPI por defecto). El puerto va con la marca a proposito.
FUENTES: dict[str, tuple[type[PowerSource], int]] = {
    "owon": (OwonPowerSource, 4196),
    "siglent": (SiglentPowerSource, 5025),
}

#: Default del banco local. Se puede pisar por entorno sin tocar el repo.
TIPO_POR_DEFECTO = os.environ.get("POWER_SOURCE_TYPE", "siglent")
IP_POR_DEFECTO = os.environ.get("POWER_SOURCE_IP", "192.168.1.137")


def agregar_argumentos(ap: argparse.ArgumentParser) -> None:
    """Agrega --supply-type / --supply-ip / --supply-port al parser del script."""
    ap.add_argument(
        "--supply-type", default=TIPO_POR_DEFECTO, choices=sorted(FUENTES),
        help="marca de la fuente: cambia la CLASE y el puerto por defecto",
    )
    ap.add_argument("--supply-ip", default=IP_POR_DEFECTO, help="IP de la fuente")
    ap.add_argument(
        "--supply-port", type=int, default=None,
        help="puerto SCPI; si no se pasa, el de la marca (owon 4196, siglent 5025)",
    )


def puerto_de(tipo: str, explicito: int | None = None) -> int:
    """El puerto a usar: el explicito si lo hay, si no el de la marca."""
    if explicito is not None:
        return explicito
    return FUENTES[tipo][1]


def construir_fuente(args: argparse.Namespace) -> PowerSource:
    """Instancia la fuente que pidieron los argumentos.

    Levanta lo que levante el driver: un IDN que no coincide con la marca es un `ValueError`
    del propio controlador, y es la respuesta correcta — hablarle a la fuente equivocada por
    SCPI es peor que no conectar.
    """
    clase, _ = FUENTES[args.supply_type]
    return clase(args.supply_ip, puerto_de(args.supply_type, args.supply_port))

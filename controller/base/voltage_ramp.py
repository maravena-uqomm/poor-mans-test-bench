"""La rampa de tension, UNA sola vez, para todas las fuentes.

Uso:   from controller.base.voltage_ramp import rampa_de_tension, PASO_V
       rampa_de_tension(escribir, actual_v, objetivo_v, logger)
NO hace: no habla SCPI. Recibe `escribir(volts) -> bool`, que es lo unico que cambia entre
         fuentes — la OWON manda `VOLTage {v}` y la Siglent `SOURce:VOLTage:SET CH1,{v}`.

── POR QUE EXISTE ────────────────────────────────────────────────────────────────────────

La rampa vivia solo en `OwonPowerSource`, y `SiglentPowerSource` subia de un escalon. La
asimetria era real y estaba documentada, pero documentar una diferencia no es lo mismo que no
tenerla: obliga a que cada lector la lea, la entienda y decida si aplica a su caso. Se extrajo
el 16-Sep-2026 por pedido explicito — «que sea igual y no estar anotando diferencias».

SON DOS FUENTES, MEDIDAS, NO UNA FAMILIA IMAGINADA: `OwonPowerSource` y `SiglentPowerSource`,
las dos registradas en `controller/factory.py`. La fundacion del repo pide que una extraccion
nazca de una duplicacion contada, no de un catalogo; esta nace de que la segunda copia se iba
a escribir a mano.

── QUE PROTEGE ───────────────────────────────────────────────────────────────────────────

Un escalon de 0V -> 12V/24V puede danar la instrumentacion de salida del tunel de RF. Y bajar
rapido tampoco es gratis: con el DUT activo durante todo el lote, una bajada en caliente como
48V -> 24V es una transicion CON CARGA, no una descarga pasiva, y puede disparar las
protecciones de la propia fuente. Por eso las dos direcciones tienen velocidad propia.
"""

from __future__ import annotations

import logging
import os
import time
from typing import Callable

#: Paso de la rampa. 0,5 V es lo que la OWON venia usando desde ID-1308.
PASO_V = 0.5

#: Por debajo de esto no se rampea: no vale la pena y el `int(diff/paso)` daria 0 o 1 pasos.
UMBRAL_RAMPA_V = 1.0


def _seg_por_12v(nombre: str, defecto: str) -> float:
    """Velocidad configurable por entorno, sin tocar codigo.

    Se conservan los nombres `OWON_RAMP_*` ademas de los genericos: estaban documentados y
    puede haber un `.env` o un compose del banco que los use. El generico gana si esta puesto.
    """
    return float(os.environ.get(nombre) or os.environ.get(f"OWON_{nombre}") or defecto)


def subida_s_por_12v() -> float:
    return _seg_por_12v("RAMP_UP_S_PER_12V", "0.5")


def bajada_s_por_12v() -> float:
    return _seg_por_12v("RAMP_DOWN_S_PER_12V", "0.5")


def rampa_de_tension(
    escribir: Callable[[float], bool],
    actual_v: float,
    objetivo_v: float,
    logger: logging.Logger | None = None,
    *,
    al_aplicar: Callable[[float], None] | None = None,
) -> bool:
    """Lleva la tension de `actual_v` a `objetivo_v` por pasos, con rollback si falla.

    Args:
        escribir: Fija la tension en la fuente. Devuelve False si el comando no salio.
        actual_v: De donde se parte. El llamador la LEE del instrumento, no la supone.
        objetivo_v: A donde se va.
        logger: Para los mensajes de fallo y de rollback.
        al_aplicar: Se llama con cada valor que SI se escribio. Sirve para que el driver
            mantenga su espejo de estado al dia — y que refleje lo APLICADO, no lo pedido.

    Returns:
        True si la rampa entera salio. False si algun paso fallo, **ya con la tension
        devuelta a `actual_v`**: dejarla a mitad de camino es el peor de los dos estados,
        porque no es ni el viejo ni el nuevo y nadie sabe cual es.
    """
    log = logger or logging.getLogger(__name__)
    marcar = al_aplicar or (lambda _v: None)

    diff = abs(objetivo_v - actual_v)
    if diff <= UMBRAL_RAMPA_V:
        marcar(actual_v)
        return True

    sentido = 1 if objetivo_v > actual_v else -1
    s_por_12v = subida_s_por_12v() if sentido > 0 else bajada_s_por_12v()
    retardo = s_por_12v / (12.0 / PASO_V)
    pasos = int(diff / PASO_V)

    ok = True
    try:
        for i in range(1, pasos + 1):
            v = actual_v + sentido * i * PASO_V
            v = min(v, objetivo_v) if sentido > 0 else max(v, objetivo_v)
            if not escribir(v):
                log.error("la rampa fallo al escribir %.3f V", v)
                ok = False
                break
            marcar(v)
            time.sleep(retardo)
    except (ConnectionError, OSError):
        log.exception("la rampa de %.3f V a %.3f V levanto", actual_v, objetivo_v)
        ok = False
    finally:
        if not ok:
            log.warning("rampa fallida — se devuelve la tension a %.3f V", actual_v)
            try:
                escribir(actual_v)
            except (ConnectionError, OSError):
                log.exception("no se pudo devolver la tension a %.3f V tras la rampa", actual_v)
            marcar(actual_v)

    return ok

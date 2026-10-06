"""ManualRutterford — el banco sin placa enrutadora, declarado en vez de asumido.

Uso:     RUTTERFORD_TYPE=manual   (y nada mas: no tiene device_path ni ip)
NO hace: no conmuta nada. Le PIDE al operador que mueva los cables y verifica que contesto.

── POR QUE EXISTE ────────────────────────────────────────────────────────────────────────

Sin este controlador hay exactamente dos formas de correr un banco sin Rutherford, y las dos
son malas:

  RUTTERFORD_TYPE sin setear  -> el rol no existe, `Test.switch_direction()` cae en un
                                 `logger.debug("Sin Rutterford - ruteo asumido manual")` y
                                 sigue. Ocho suites miden DL y UL por el MISMO camino fisico
                                 y ninguna se entera.
  RUTTERFORD_TYPE=dummy       -> PEOR. Es el default del compose. Construye un crossbar en
                                 memoria, `_apply_routes()` lo relee, ve que coincide consigo
                                 mismo y loggea "(verificado)" — mientras el cable fisico esta
                                 donde lo dejo el operador.

Las dos mienten. La segunda ademas produce evidencia de que no miente, que es la unica cosa
peor que no tener evidencia.

── LA DIFERENCIA CON UN `input()` SUELTO ─────────────────────────────────────────────────

`attenuation_check.py` ya tiene un prompt manual, y su forma es la que NO hay que copiar:

    try:
        input(f"Set attenuation to {attenuation} dB. Press Enter...")
    except EOFError:
        pass          # <-- aca esta el problema

Bajo la API no hay TTY en stdin, asi que `input()` levanta EOFError y el `pass` lo convierte
en "el operador confirmo". El paso manual se saltea **solo y sin avisar**, con TEST_MODE sin
poner. Un prompt que no puede fallar no es un prompt: es un comentario que cuesta una linea.

Aca EOFError es un FALLO: si no hay nadie que mueva el cable, la medicion que sigue no
describe nada. `ManualRoutingUnattended` existe como tipo propio para que quien lo capture
pueda distinguir "no habia operador" de "el operador dijo que no".

── QUE SIGUE HACIENDO `switch_direction()` ───────────────────────────────────────────────

Todo lo demas: apaga el tono y el tracking generator ANTES de pedir el cambio y los restaura
despues. Los SP4T son reflectivos y el cable tambien: pedirle a alguien que desenchufe un
conector con RF encima es la misma mala idea con o sin placa.

Y el gate de sesion de `test_execution_service.py` sigue funcionando, porque el rol EXISTE y
no es None. Con el rol sin configurar, ese gate no tiene nada que mirar.
"""

from __future__ import annotations

import logging
import os
import sys

from controller.base.rutterford import Port, Rutterford


class ManualRoutingUnattended(RuntimeError):
    """Hacia falta mover un cable y no habia nadie para hacerlo.

    Tipo propio —y no un `RuntimeError` a secas— porque el llamador tiene que poder
    distinguirlo de "el operador contesto que no". Los dos abortan la corrida; solo uno
    significa que el banco esta mal configurado.
    """


class ManualRutterford(Rutterford):
    """Ruteo por cable, con el operador en el lazo.

    Cada cambio de ruta se le pide por consola y se espera confirmacion. **Solo se pide
    cuando la ruta cambia de verdad**: una suite que llama `switch_direction("downlink")`
    tres veces seguidas molesta al operador una vez, no tres.
    """

    def __init__(self, logger: logging.Logger | None = None) -> None:
        self.logger = logger or logging.getLogger("ManualRutterford")
        #: La ruta vigente segun lo que el operador confirmo. `None` = nunca se pidio nada,
        #: que NO es lo mismo que una ruta conocida: al arrancar no sabemos como esta cableado.
        self._ruta: tuple[str, str | None, str | None] | None = None

    # ------------------------------------------------------------------
    #  El unico punto que habla con el operador
    # ------------------------------------------------------------------

    def _pedir(self, descripcion: str, ruta: tuple[str, str | None, str | None]) -> None:
        """Pide el cambio de cableado y no vuelve hasta tener confirmacion.

        Raises:
            ManualRoutingUnattended: no hay TTY, o `TESTBENCH_MANUAL_ROUTING` lo prohibe.
            RuntimeError: el operador contesto que no pudo dejarlo asi.
        """
        if ruta == self._ruta:
            self.logger.debug("ruteo manual: ya esta en %s, no se molesta al operador", descripcion)
            return

        # EL MODO DESATENDIDO SE DECLARA, NO SE DEDUCE.
        #
        # `TESTBENCH_MANUAL_ROUTING=assume` dice "el cableado ya esta puesto, no preguntes".
        # Es legitimo —una corrida de una sola direccion, un cableado fijo que no se toca— y
        # por eso existe. Lo que no es legitimo es que el codigo lo ASUMA porque no encontro
        # un TTY: eso es exactamente el EOFError de attenuation_check.
        modo = os.getenv("TESTBENCH_MANUAL_ROUTING", "").strip().lower()
        if modo == "assume":
            self.logger.warning(
                "ruteo manual ASUMIDO por TESTBENCH_MANUAL_ROUTING=assume: %s. "
                "Nadie confirmo que el cable este ahi.", descripcion,
            )
            self._ruta = ruta
            return

        if not sys.stdin.isatty():
            raise ManualRoutingUnattended(
                f"hace falta cablear a mano [{descripcion}] y no hay consola interactiva. "
                "Sin alguien que mueva el cable, lo que se mida despues no describe ese camino. "
                "Correr la suite desde una terminal, o declarar TESTBENCH_MANUAL_ROUTING=assume "
                "si el cableado ya esta puesto y no se va a tocar."
            )

        print(f"\n  ⚠  CABLEADO MANUAL — {descripcion}")
        respuesta = input("     Dejalo asi y escribi 'ok' (cualquier otra cosa aborta): ")
        if respuesta.strip().lower() not in ("ok", "s", "si", "y", "yes"):
            raise RuntimeError(
                f"el operador no confirmo el cableado [{descripcion}]: se aborta en vez de medir"
            )

        self.logger.info("ruteo manual confirmado: %s", descripcion)
        self._ruta = ruta

    # ------------------------------------------------------------------
    #  Modos de operacion
    # ------------------------------------------------------------------

    def set_mode_uplink(self) -> None:
        self._pedir("UPLINK: generador -> puerto del DUT, analizador -> BASE", ("uplink", None, None))

    def set_mode_downlink(self) -> None:
        self._pedir("DOWNLINK: generador -> BASE, analizador -> puerto del DUT", ("downlink", None, None))

    def set_mode_wire(self) -> None:
        self._pedir("WIRE: generador -> analizador, directo, sin DUT en el medio", ("wire", None, None))

    # ------------------------------------------------------------------
    #  Puertos
    # ------------------------------------------------------------------

    def set_input_port(self, port: Port) -> None:
        self._pedir(f"entrada de RF en el conector {port}", ("input", str(port), None))

    def set_output_port(self, port: Port) -> None:
        self._pedir(f"salida / medicion de RF en el conector {port}", ("output", str(port), None))

    def set_direct_route(self, input_port: Port, output_port: Port) -> None:
        self._pedir(
            f"DIRECTO: generador -> {input_port}, analizador -> {output_port} (sin pasar por BASE)",
            ("direct", str(input_port), str(output_port)),
        )

    # `read_dc_measurements()` NO se sobrescribe a proposito: la de la clase base levanta
    # `DcMeasurementUnsupported`, que es la respuesta correcta —un cable no mide DC— y ya
    # tiene un consumidor que la distingue de "midio y dio cero" (passive_loss_check).

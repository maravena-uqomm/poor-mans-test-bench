"""RutherfordBoard — crossbar RF de Uqomm (3 puertos INSTR x 4 puertos DUT), por USB CDC.

Uso:  board = RutherfordBoard(device_path="/dev/ttyACM2")   # el constructor abre el puerto
      board.set_mode_downlink(); board.set_output_port("TUNEL")
No hace: RF, ventiladores ni sensores. El crossbar de ruteo y su verificacion; las radios LoRa
      van por `board.lora()` (controller/uqomm/rutherford_lora.py), sobre el mismo puerto.

Modelo de la placa (fw-rutherford/proto/rutherford.proto, spec/01-board-capabilities.md):

  * El ruteo es **append-only**: no existe desconectar una ruta suelta, solo `reset()`
    limpia el estado. Por eso cada cambio de configuracion es reset + rutas completas, y
    NUNCA un reset en medio de dos rutas — eso borraria la primera.
  * `reset()` deja los cuatro SP4T de DUT en la carga interna de 50 Ohm y los INSTR en
    SG->BR1, SA->BR2, TG->TUNNEL: **ningun camino INSTR<->DUT completo** (FR-S02). Ademas
    fija el SP2T en 50 Ohm, asi que la impedancia se manda DESPUES del reset o no sobrevive.
  * Exclusividad por hardware: un solo puerto por switch. Cada INSTR sale a un solo DUT y
    cada DUT entra desde un solo INSTR. Maximo 3 rutas activas.
  * El SP4T es reflectivo: los puertos no seleccionados presentan un corto. El spec exige
    que el generador este en minimo o apagado durante las transiciones de reset — quien
    conmuta con RF encendida se lo hace a si mismo. Ver `suite/test.py::switch_direction`.

Un ACK no es evidencia de conmutacion: durante meses la placa contesto `OK` sin conmutar
nada. Por eso `_apply_routes()` relee con `get_state()` y compara contra lo pedido.
"""

from __future__ import annotations

import logging
import os
import threading
import time
from collections.abc import Iterable

from rutherford import Dut, ImpedanceSetting, Instr, Route, RoutingState, RutherfordClient
from rutherford.errors import RutherfordError, TransportTimeoutError
from rutherford.transport import Transport

from controller.base.rutterford import DcMeasurementUnsupported, DcReading, Port, Rutterford
from controller.uqomm.rutherford_lora import RutherfordLora

# Reintentos al abrir el transporte — una desconexion USB transitoria (glitch, resume tras
# autosuspend) no deberia tirar abajo la sesion entera si un segundo intento, un momento
# despues, hubiera bastado. NO ayuda si el nodo del CONTENEDOR quedo apuntando a un
# major:minor viejo: el docker-compose fija el `--device` al arrancar, y un renumerado en
# el host no lo actualiza — ahi reintentar `open()` repite el mismo ENXIO sobre un nodo
# muerto por mas intentos que se den, y hace falta recrear el contenedor. Override sin
# tocar codigo via RUTHERFORD_CONNECT_RETRIES / RUTHERFORD_CONNECT_RETRY_DELAY_S.
_CONNECT_RETRIES = int(os.environ.get("RUTHERFORD_CONNECT_RETRIES", "3"))
_CONNECT_RETRY_DELAY_S = float(os.environ.get("RUTHERFORD_CONNECT_RETRY_DELAY_S", "1.0"))


# El ignore de mypy en la linea de la clase: `rutherford` viaja sin stubs
# (ignore_missing_imports), asi que ve RutherfordError como Any y rechaza heredar de el. Hereda
# a proposito: quien ya captura RutherfordError alrededor del ruteo tiene que atrapar tambien
# este fallo de verificacion.
class RoutingVerificationError(RutherfordError):  # type: ignore[misc]
    """El estado leido de la placa no coincide con el ruteo pedido."""


class RutherfordBoard(Rutterford):
    """Controlador de la placa Rutherford real.

    Args:
        port: Ruta del puerto serie (ej. ``/dev/ttyACM2``). Ignorado si no es str —
            el orquestador reusa ``port`` para el puerto TCP de CRFS (un int).
        timeout_s: Timeout de lectura del transporte, en segundos.
        transport: ``Transport`` alternativo (lo usa el dummy con su simulador).
        device_path: Alias de ``port``, que es como llega desde la config por entorno.
    """

    # Puerto INSTR donde esta cableado el generador del banco.
    #
    # CONFIRMADO el 18-Ago-2026 con el autor del firmware: por diseño **INST 1 es SG**, INST 2
    # es SA e INST 3 es TG. El banco tiene el generador en INST 1 y el analizador en INST 2, y
    # INST 3 no esta conectado. Asi que este valor es el correcto, y ya no es una asuncion.
    #
    # PERO el firmware v0.1.3 no implementa ese mapeo. `AppRutherford` recibe los switches en
    # el orden `saSwitch, sgSwitch, tgSwitch` y `main.cpp` le pasa `inst1, inst2, inst3`, asi
    # que `Instr.SA` termina manejando INST 1 —que es SG— e `Instr.SG` maneja INST 2. Estan
    # intercambiados. Mientras eso siga asi, rutear por la placa mueve el switch del otro
    # instrumento, y con los puentes cableados por rotulo no queda camino de RF en ninguna
    # configuracion. Esta anotado en ID-1368: se arregla en firmware, no compensando aca.
    #
    # No se compensa dando la vuelta a esta constante a proposito: seria hacer que dos errores
    # se cancelen, y el dia que Aquiles arregle el orden en `main.cpp` esto se rompe en
    # silencio. El valor dice donde ESTA el generador; que el firmware lo obedezca es su
    # problema, y el corredor de RF lo detecta (ver `rutherford_rf_check.py`).
    #
    # No es una env var a proposito: configurar lo que nadie puede verificar solo mueve el
    # error de lugar. El codigo anterior usaba SG para DL y TG para UL, lo que implicaba dos
    # generadores cableados que no existen.
    GENERATOR_INSTR = Instr.SG

    # El vocabulario de la interfaz (`TUNEL`, heredado del CRFS) no es el de la biblioteca
    # (`Dut.TUNNEL`). La traduccion no existia en ninguna parte.
    _PORT_TO_DUT: dict[str, Dut] = {
        "BASE": Dut.BASE,
        "TUNEL": Dut.TUNNEL,
        "TUNNEL": Dut.TUNNEL,  # alias: la biblioteca lo escribe con dos N
        "BRANCH1": Dut.BR1,
        "BRANCH2": Dut.BR2,
    }

    #: Inverso de `_PORT_TO_DUT`. Se escribe aparte y no se deriva: invertir el dict de
    #: arriba elige "TUNNEL" o "TUNEL" segun el orden de insercion, y el nombre canonico del
    #: orquestador es el de una N.
    _DUT_TO_PORT: dict[Dut, str] = {
        Dut.BASE: "BASE",
        Dut.TUNNEL: "TUNEL",
        Dut.BR1: "BRANCH1",
        Dut.BR2: "BRANCH2",
    }

    _IMPEDANCE_TO_OHMS: dict[ImpedanceSetting, int] = {
        ImpedanceSetting.OHM_50: 50,
        ImpedanceSetting.OHM_75: 75,
    }

    #: ERROR_CODE_PERIPHERAL_ADC_FAULT (proto/rutherford.proto). El firmware lo levanta solo
    #: cuando fallan los ocho canales.
    _ADC_FAULT_CODE = 402

    #: ERROR_CODE_PROTOCOL_UNKNOWN_SYSTEM y _UNKNOWN_CMD. Un firmware anterior al comando
    #: contesta con estos, y eso NO es un fallo de lectura: es "esta placa no sabe hacerlo".
    #: Medido el 28-Ago-2026 contra la placa del banco (serial 208834984832, firmware sin el
    #: comando): `read_dc_measurements()` devuelve code=104, context=1.
    _CODIGOS_SIN_SOPORTE = frozenset({103, 104})

    _DEFAULT_BRANCH: Port = "TUNEL"

    def __init__(
        self,
        port: str | None = None,
        timeout_s: float = 3.0,
        transport: Transport | None = None,
        device_path: str | None = None,
        **kwargs: object,
    ) -> None:
        self.logger = logging.getLogger(self.__class__.__name__)
        effective_port = device_path or (port if isinstance(port, str) else None)
        if not effective_port and transport is None:
            raise ValueError("Either 'port' (str), 'device_path' or 'transport' must be provided")

        self._lock = threading.RLock()
        self._client = RutherfordClient(effective_port or "injected://", timeout_s=timeout_s, transport=transport)
        # Construir conecta, como el resto de los controladores del orquestador: nadie llama
        # open() despues de la factory. `SerialTransport.open()` es idempotente, asi que la
        # sonda de salud puede llamarlo de nuevo sin abrir un segundo descriptor, y tambien
        # asi puede reintentarse aca sobre el mismo cliente sin recrearlo.
        for intento in range(1, _CONNECT_RETRIES + 1):
            try:
                self._client.open()
                break
            except RutherfordError as exc:
                if intento >= _CONNECT_RETRIES:
                    raise
                self.logger.warning(
                    "RutherfordBoard: fallo al abrir %s (intento %d/%d): %s — reintento en %.1fs",
                    effective_port or "injected transport", intento, _CONNECT_RETRIES, exc,
                    _CONNECT_RETRY_DELAY_S,
                )
                time.sleep(_CONNECT_RETRY_DELAY_S)
        self.logger.info("RutherfordBoard opened %s", effective_port or "injected transport")

        self._direction = "DL"
        self._input_port: Port = "BASE"
        self._output_port: Port = self._DEFAULT_BRANCH
        # Extremo que no es BASE. Se guarda aparte porque al invertir la direccion cambia de
        # lado —en DL lo mide el analizador, en UL inyecta el generador— y la rama elegida
        # tiene que sobrevivir al cambio: medir BRANCH1 en DL y pasar a UL debe inyectar en
        # BRANCH1, no volver a TUNEL.
        self._branch: Port = self._DEFAULT_BRANCH

    # ------------------------------------------------------------------
    # Ciclo de vida
    # ------------------------------------------------------------------

    def open(self) -> None:
        try:
            self._client.open()
            self.logger.info("RutherfordBoard connected")
        except RutherfordError as exc:
            self.logger.error("Failed to open rutherford: %s", exc)
            raise

    def close(self) -> None:
        try:
            self._client.close()
            self.logger.info("RutherfordBoard disconnected")
        except RutherfordError as exc:
            self.logger.warning("Error closing rutherford: %s", exc)

    def disconnect(self) -> None:
        self.close()

    def __enter__(self) -> RutherfordBoard:
        self.open()
        return self

    def __exit__(self, exc_type: object, exc_val: object, exc_tb: object) -> None:
        self.close()

    # ------------------------------------------------------------------
    # Identificacion (sonda de salud) — no levanta nunca
    # ------------------------------------------------------------------

    def idn(self) -> str:
        try:
            state = self.routing_state()
        except RutherfordError as exc:
            return f"Rutherford, error: {exc}"
        routes = ", ".join(
            f"{r.instr.name}->{r.dut.name}" for r in sorted(state.routes, key=lambda r: (r.instr.value, r.dut.value))
        )
        return f"Rutherford, FW routes: [{routes}], impedance: {state.impedance.name}"

    def lora(self) -> RutherfordLora:
        """Las dos radios LoRa, sobre el mismo cliente y el mismo lock que el ruteo."""
        return RutherfordLora(self._client, self._lock, flush=self._flush_serial)

    def routing_state(self) -> RoutingState:
        """Snapshot autoritativo de la placa: rutas activas + impedancia."""
        with self._lock:
            state: RoutingState = self._client.get_state()
        return state

    # ------------------------------------------------------------------
    # Modos de operacion
    # ------------------------------------------------------------------

    def set_mode_downlink(self) -> None:
        """DL: el generador inyecta en BASE y el analizador mide en la rama activa."""
        self._retarget(direction="DL", input_port="BASE", output_port=self._branch)

    def set_mode_uplink(self) -> None:
        """UL: el generador inyecta en la rama activa y el analizador mide en BASE."""
        self._retarget(direction="UL", input_port=self._branch, output_port="BASE")

    def set_mode_wire(self) -> None:
        """Saca la placa del camino: estado de reset, sin ningun camino INSTR<->DUT.

        La placa no tiene bypass — `reset()` es lo mas cerca que existe (FR-S02: los DUT
        quedan en la carga de 50 Ohm). Es lo que `transmitter_pick_freq` necesita antes de
        normalizar el analizador.
        """
        with self._lock:
            try:
                self._client.reset()
                self._client.set_impedance(ImpedanceSetting.OHM_50)
            except TransportTimeoutError:
                self._flush_serial()
                raise
            except RutherfordError as exc:
                self.logger.error("Wire mode failed: %s", exc)
                raise
        self.logger.info("RutherfordBoard: wire mode (estado de reset, sin rutas)")

    # ------------------------------------------------------------------
    # Puertos
    # ------------------------------------------------------------------

    def set_input_port(self, port: Port) -> None:
        """Puerto DUT donde inyecta el generador. Re-rutea."""
        self.logger.info("RutherfordBoard: input_port=%s", port)
        self._retarget(input_port=self._validated(port))

    def set_output_port(self, port: Port) -> None:
        """Puerto DUT donde mide el analizador. Re-rutea."""
        self.logger.info("RutherfordBoard: output_port=%s", port)
        self._retarget(output_port=self._validated(port))

    def set_direct_route(self, input_port: Port, output_port: Port) -> None:
        """Puerto DUT de entrada y salida juntos, en un solo `_retarget()`.

        A diferencia de `set_input_port()` + `set_output_port()` por separado: esos dos
        computan el lado que NO se les pide a partir de `self._input_port`/
        `self._output_port` (el valor VIEJO, cacheado en software) — si los dos puertos
        estan INTERCAMBIANDO valores respecto a la ruta anterior (TUNEL<->BRANCH1 ida y
        vuelta, el caso de aislacion), el primer llamado dejaria el otro lado en su valor
        viejo, que puede coincidir con el valor nuevo que le tocaria al segundo llamado —
        `_routes_for()` lo rechaza aunque el par FINAL pedido sea valido. Un solo
        `_retarget()` con los dos valores nuevos nunca pasa por ese estado intermedio.
        """
        self.logger.info(
            "RutherfordBoard: direct route input_port=%s output_port=%s", input_port, output_port
        )
        self._retarget(input_port=self._validated(input_port), output_port=self._validated(output_port))

    def read_dc_measurements(self) -> tuple[DcReading, ...]:
        """Los ocho conectores de salida, en una sola llamada. Ver `Rutterford`.

        Por que la deteccion de capacidad y no un `try` cualquiera: el metodo del cliente
        entra con la pylib que trae el mensaje `GetDcMeasurementsCmd`, y el banco puede tener
        una version anterior. Sin la deteccion, `AttributeError` sale como un error de
        programacion cuando en realidad es "esta placa todavia no sabe hacerlo", que es un
        estado legitimo y tiene otra respuesta.
        """
        leer = getattr(self._client, "read_dc_measurements", None)
        if leer is None:
            raise DcMeasurementUnsupported(
                "La pylib instalada no expone read_dc_measurements(): hace falta la version "
                "que incluye GetDcMeasurementsCmd"
            )

        with self._lock:
            try:
                crudas = leer()
            except TransportTimeoutError:
                self._flush_serial()
                raise
            except RutherfordError as exc:
                codigo = getattr(exc, "code", None)
                # Un firmware sin el comando contesta UNKNOWN_CMD. Se traduce al tipo propio
                # para que el operador lea "hay que grabar el firmware" y no "la lectura
                # fallo": son dos problemas distintos, con dos acciones distintas.
                if codigo in self._CODIGOS_SIN_SOPORTE:
                    raise DcMeasurementUnsupported(
                        f"El firmware de la placa no implementa la medicion de DC "
                        f"(codigo {codigo}): hay que grabar la version que trae "
                        f"GetDcMeasurementsCmd"
                    ) from exc
                # 402 = ERROR_CODE_PERIPHERAL_ADC_FAULT: el firmware solo lo levanta cuando
                # fallaron los OCHO canales. Con uno solo caido contesta la lista y lo marca
                # `valid=False`, para que un canal muerto no ciegue a los otros siete.
                if codigo == self._ADC_FAULT_CODE:
                    self.logger.error("Los ocho canales de ADC fallaron: %s", exc)
                raise

        lecturas = tuple(self._a_lectura(m) for m in crudas)
        caidos = [f"{r.port}/{r.ohms}" for r in lecturas if r.volts is None]
        if caidos:
            self.logger.warning("Canales de DC sin lectura: %s", ", ".join(caidos))
        return lecturas

    def _a_lectura(self, medicion: object) -> DcReading:
        """Traduce un `DcMeasurement` de la pylib al tipo del orquestador.

        La identidad se conserva aunque el canal haya fallado: es el dato que dice cual
        conector se apago.
        """
        return DcReading(
            port=self._DUT_TO_PORT.get(getattr(medicion, "dut", None), "DESCONOCIDO"),
            ohms=self._IMPEDANCE_TO_OHMS.get(getattr(medicion, "impedance", None), 0),
            volts=medicion.volts,  # None cuando valid es False — la pylib ya lo resuelve
            raw_counts=getattr(medicion, "raw_counts", 0),
        )

    def set_branch(self, branch: int) -> None:
        """Selecciona la rama activa: el extremo que NO es BASE, segun la direccion."""
        port = self._validated(f"BRANCH{branch}")
        if self._direction == "UL":
            self.set_input_port(port)
        else:
            self.set_output_port(port)

    # ------------------------------------------------------------------
    # Interno
    # ------------------------------------------------------------------

    def _validated(self, port: str) -> Port:
        if port not in self._PORT_TO_DUT:
            raise ValueError(f"Unknown Rutherford port '{port}'. Valid: {sorted(self._PORT_TO_DUT)}")
        return port  # type: ignore[return-value]  # validado contra _PORT_TO_DUT

    def _retarget(
        self,
        *,
        direction: str | None = None,
        input_port: Port | None = None,
        output_port: Port | None = None,
    ) -> None:
        """Calcula el ruteo pedido, lo aplica, y **recien despues** guarda el estado.

        El estado interno se commitea al final a proposito: si la placa no confirma, el
        controlador queda describiendo la configuracion anterior en vez de una que no esta
        aplicada. Asi un reintento vuelve a mandar la misma intencion en vez de arrancar
        desde un estado a medio escribir.
        """
        new_direction = direction or self._direction
        new_in = input_port or self._input_port
        new_out = output_port or self._output_port

        desired = self._routes_for(new_in, new_out)
        self._apply_routes(desired, new_direction, new_in, new_out)

        self._direction = new_direction
        self._input_port = new_in
        self._output_port = new_out
        for port in (new_in, new_out):
            if port != "BASE":
                self._branch = port

    def _routes_for(self, input_port: Port, output_port: Port) -> frozenset[Route]:
        dut_in = self._PORT_TO_DUT[input_port]
        dut_out = self._PORT_TO_DUT[output_port]
        if dut_in == dut_out:
            raise ValueError(
                f"El generador y el analizador no pueden compartir el puerto DUT "
                f"{input_port}: el SP4T selecciona uno solo (exclusividad por destino)"
            )
        return frozenset(
            {
                Route(instr=self.GENERATOR_INSTR, dut=dut_in),
                Route(instr=Instr.SA, dut=dut_out),
            }
        )

    def _apply_routes(self, desired: frozenset[Route], direction: str, input_port: Port, output_port: Port) -> None:
        """Aplica el ruteo completo y lo verifica contra la placa.

        Orden obligado: `reset()` primero (el ruteo es append-only y el reset tambien
        reescribe el SP2T, asi que la impedancia va despues o se pierde), luego las rutas.
        """
        with self._lock:
            try:
                self._client.reset()
                self._client.set_impedance(ImpedanceSetting.OHM_50)
                for route in sorted(desired, key=lambda r: (r.instr.value, r.dut.value)):
                    self._client.route(route.instr, route.dut)
                actual = self._client.get_state().routes
            except TransportTimeoutError:
                self._flush_serial()
                raise
            except RutherfordError as exc:
                self.logger.error("%s routing failed (%s -> %s): %s", direction, input_port, output_port, exc)
                raise

        if frozenset(actual) != desired:
            raise RoutingVerificationError(
                f"La placa no confirmo el ruteo {direction} "
                f"({input_port} -> {output_port}): "
                f"pedido {self._fmt(desired)}, leido {self._fmt(actual)}"
            )

        self.logger.info(
            "RutherfordBoard: %s mode — gen->%s, SA->%s (verificado)",
            direction,
            input_port,
            output_port,
        )

    @staticmethod
    def _fmt(routes: Iterable[Route]) -> str:
        return "{" + ", ".join(sorted(f"{r.instr.name}->{r.dut.name}" for r in routes)) + "}"

    def _flush_serial(self) -> None:
        """Descarta lo que quedo en el buffer tras un timeout, para no desincronizar.

        Alcanza los privados del transporte a proposito: la biblioteca no expone flush.
        """
        transport = getattr(self._client, "_transport", None)
        serial_port = getattr(transport, "_serial", None)
        if serial_port is None:
            return
        try:
            if serial_port.is_open:
                serial_port.reset_input_buffer()
                self.logger.debug("RutherfordBoard: buffer de entrada descartado")
        except OSError as exc:  # SerialException hereda de OSError
            self.logger.warning("No se pudo descartar el buffer de entrada: %s", exc)

"""Capa de BYTES del VLAD25: el transporte, no el protocolo.

Uso:   se mezcla en VladController; nadie la instancia sola.
NO hace: no conoce comandos ni unidades. Recibe un `cmd` y un payload ya armados por
         `vlad25_protocol` y los pone en el cable; devuelve bytes crudos.

Que vive aca y por que esta separado de vlad_controller.py: estas seis piezas son las unicas
que tocan `serial` y `bleak` directamente, y son la mitad del archivo. Tenerlas aparte deja
`vlad_controller.py` con el estado, la salud y el circuit breaker —lo que un llamador necesita
leer— y concentra en un solo lugar las tres trampas medidas del transporte:

  - el CDC lleva el log del firmware Y las respuestas por el MISMO endpoint, asi que hay que
    sincronizar con el eco del CMD en vez de leer 1+N bytes a ciegas (`_sincronizar_con_eco`);
  - por BLE el firmware NO notifica: la respuesta queda disponible para que el central la LEA,
    con un atraso de un par de ciclos, y hay que sondear con una cadencia lenta o el modulo
    corta la conexion (`_ble_read_echo`);
  - el ACK se VERIFICA en los dos transportes, porque un NACK del firmware informado como
    exito significa creer que se aplico una atenuacion que no se aplico.

Cada una de esas tres tiene su medicion escrita al lado, con la fecha. No se toca ninguna sin
banco delante.
"""

from __future__ import annotations

import time

from controller.base.reliability import InstrumentState


class VladTransportMixin:
    """Primitivas de E/S del VLAD25 sobre USB CDC y BLE NUS.

    La clase anfitriona provee: `logger`, `_lock`, `_state`, `_metrics`, `_is_simulation`,
    `use_bluetooth`, `_serial_connection`, `_ble_client`, `_run_async()`,
    `_start_watchdog()` / `_stop_watchdog()`, `_is_hung`, `_marcar_enlace_perdido()`,
    y del protocolo: `RESP_LEN`, `ACK`, `NUS_RX_UUID`, `NUS_TX_UUID`.
    """

    # ------------------------------------------------------------------
    # Low-level I/O Primitives con watchdog y métricas
    # ------------------------------------------------------------------

    #: Bytes de log que se toleran antes del eco.
    #:
    #: 512 fue el primer valor y NO alcanzo: medido el 10-Ago, el log del VLAD con BLE activo es
    #: una manguera continua —[MDM], [SPI], AT+BLEADVDATA— y 512 B se consumen antes de que llegue
    #: la respuesta. 8192 son unos pocos segundos de ese caudal al timeout del puerto.
    #:
    #: Que haga falta un presupuesto asi es sintoma, no solucion: el log y el protocolo comparten
    #: un endpoint que no fue pensado para los dos. La correccion de fondo vive en el firmware
    #: —silenciar el log mientras hay una sesion de comandos, o separar los transportes.
    SYNC_MAX_DESCARTE = 8192

    #: True si la ultima lectura tuvo que descartar bytes para encontrar el eco, o
    #: sea que pudo haber tomado un byte del log. Lo pone `_read_payload` y lo lee
    #: `_leer_consulta_limpia`. Atributo de CLASE para que el mixin no dependa de
    #: que el anfitrion se acuerde de inicializarlo.
    ultima_lectura_resincronizada = False

    def _limpiar_entrada(self) -> None:
        """Descarta lo que haya en el buffer de entrada del SO. Llamar ANTES de escribir.

        EL ECO NO ALCANZA CONTRA UN BUFFER SUCIO, y esto se midio en la placa 2080388C4B30
        el 09-Sep-2026, en la primera consulta despues de enchufar el cable:

            DIAGNOSTIC_RUN (0x35) -> palabra de salud 0x2020
              los cinco bits obligatorios en 0, FOTA_PENDING en 1, vin_zone 0 (sin 12 V)

        Un equipo roto, con la fuente ausente. **Era mentira.** `0x2020` son los bytes
        `20 20`, o sea DOS ESPACIOS: la sincronizacion encontro un `0x35` —que es el ASCII
        `'5'`— dentro del log de texto del firmware y leyo los dos caracteres siguientes como
        la palabra. Tres corridas despues, con el buffer limpio, la misma placa contesta
        `0x089F`: los cinco obligatorios en 1 y 12 V. El `0x08` lo confirma por otro camino,
        11931 mV y 171 mA.

        Es la peor forma de fallar: no un error, un NUMERO CREIBLE que manda a revisar la
        fuente de alimentacion. Y le puede pasar a cualquier comando con `expected` chico —
        con 2 bytes basta que aparezca el byte del comando en el log.

        El eco sigue siendo necesario —el log y las respuestas comparten el endpoint mientras
        la sesion corre— pero el eco solo desambigua lo que llega DESPUES de escribir. Lo que
        ya estaba en el buffer no es respuesta de nada.
        """
        if self._serial_connection is None:
            return
        try:
            self._serial_connection.reset_input_buffer()
        except OSError as exc:
            # No es fatal: si el puerto se cayo, el read siguiente lo va a decir con mas
            # informacion que la que tenemos aca.
            self.logger.debug("VLAD: no se pudo limpiar el buffer de entrada: %s", exc)

    #: Ventana de silencio que declara el buffer "quieto", y tope duro. El log del firmware es
    #: PERIODICO (latido `[VIVO]`), no continuo: entre latidos el buffer llega a 0, asi que drenar
    #: hasta 150 ms sin datos vacia el backlog de arranque sin depender de que el log se calle.
    DRENAJE_QUIETO_MS: float = 150.0
    DRENAJE_MAX_MS: float = 1500.0

    def _drenar_hasta_quieto(self) -> int:
        """Lee y descarta hasta que el buffer quede QUIETO, o venza el tope. Devuelve cuantos.

        POR QUE, y por que no alcanza el `reset_input_buffer()` de una: al enchufar hay un backlog
        de log (medido: 1534 B) que un solo vaciado no cubre, porque el firmware sigue emitiendo
        mientras se vacia. La primera consulta caia en ese backlog y el eco-sync se enganchaba a un
        byte del log —peor con `0x45`='E', que abunda en el texto—; de ahi el «el primer comando
        no hace nada y el siguiente si». Drenar hasta 150 ms sin datos deja el primer query con una
        ventana limpia. Es el mismo invariante que `_limpiar_entrada`, extendido en el tiempo.
        """
        if self._serial_connection is None:
            return 0
        descartados = 0
        t0 = time.perf_counter()
        ultimo_dato = t0
        while (time.perf_counter() - t0) * 1000.0 < self.DRENAJE_MAX_MS:
            try:
                n = self._serial_connection.in_waiting
                if n:
                    descartados += len(self._serial_connection.read(n))
                    ultimo_dato = time.perf_counter()
                elif (time.perf_counter() - ultimo_dato) * 1000.0 >= self.DRENAJE_QUIETO_MS:
                    break
                else:
                    time.sleep(0.02)
            except OSError as exc:
                # El puerto se cayo: el read siguiente lo dira con mas informacion.
                self.logger.debug("VLAD: drenaje interrumpido: %s", exc)
                break
        return descartados

    def _sincronizar_con_eco(self, cmd: int, expected: int) -> tuple[bytes, int]:
        """([eco][payload], cuantos bytes se descartaron antes del eco).

        DEVUELVE EL CONTEO Y NO SOLO LA TRAMA, porque es el unico dato que distingue una
        respuesta de una coincidencia. Ver `_read_payload`, que lo usa para decidir.
        """
        descartados = bytearray()
        while len(descartados) < self.SYNC_MAX_DESCARTE:
            b = self._serial_connection.read(1)
            if not b:
                break                                  # se agoto el timeout del puerto
            if b[0] == cmd:
                resto = self._serial_connection.read(expected) if expected else b""
                return b + resto, len(descartados)
            descartados += b
        if descartados:
            self.logger.warning(
                "VLAD: no aparecio el eco de 0x%02X en %d B. Ultimo tramo: %r",
                cmd, len(descartados), bytes(descartados[-60:]),
            )
        return b"", len(descartados)

    #: Cuantas veces se reintenta una CONSULTA hasta conseguir una lectura limpia.
    #:
    #: Tres, y no mas: cada intento cuesta el timeout del puerto. Con el log del firmware a
    #: su caudal normal la mayoria de las lecturas salen limpias de una —medido en la placa
    #: 2080388C4B30: de cinco comandos, tres limpios y dos resincronizados— asi que tres
    #: intentos alcanzan de sobra. Si los tres salen sucios, se devuelve FALLO: es mejor que
    #: el breaker cuente un fallo que devolver un numero que puede ser log.
    CONSULTA_MAX_INTENTOS = 3

    def _leer_consulta_limpia(self, cmd: int, frame: bytes,
                              expected: int | None = None) -> bytes:
        """Consulta hasta conseguir una lectura donde el eco fue el PRIMER byte. b"" si no.

        SOLO PARA CONSULTAS, y la distincion importa: reintentar una consulta es gratis
        —son de solo lectura— mientras reintentar un SET reenviaria el comando. El camino de
        ACK usa `_read_payload` directo y se conforma con avisar.

        POR QUE HACE FALTA. `_read_payload` no puede distinguir el eco real de un byte igual
        dentro del log del firmware, que comparte el endpoint. Medido con el controlador de
        produccion en la placa 2080388C4B30 el 09-Sep-2026:

            41 B descartados antes del eco de 0x35: b'8 up=871s sin calibrar  P_UL=...'
            get_health_word() -> 8224

        8224 es 0x2020 = dos ESPACIOS: el `'5'` de una linea de log se leyo como el eco de
        DIAGNOSTIC_RUN. La palabra decia los cinco bits obligatorios en cero y sin 12 V, o
        sea un equipo roto, sobre una placa que con una lectura limpia contesta 0x089F y
        12 V —confirmado por 0x08, 11931 mV—.

        Una lectura con `descartados == 0` no tiene esa ambiguedad: el eco llego pegado a la
        escritura, asi que es la respuesta. Se insiste por eso y no por el valor.
        """
        for intento in range(1, self.CONSULTA_MAX_INTENTOS + 1):
            self._limpiar_entrada()
            self._serial_connection.write(frame)
            payload = self._read_payload(cmd, expected)
            if payload and not self.ultima_lectura_resincronizada:
                return payload
            if not payload:
                # No hubo trama: reintentar no cambia nada si el equipo no contesta, pero
                # cuesta poco y cubre el caso de una respuesta que llego partida.
                self.logger.debug(
                    "VLAD: consulta 0x%02X sin respuesta, intento %d/%d",
                    cmd, intento, self.CONSULTA_MAX_INTENTOS,
                )
                continue
            self.logger.warning(
                "VLAD: consulta 0x%02X resincronizada, se descarta y reintenta (%d/%d)",
                cmd, intento, self.CONSULTA_MAX_INTENTOS,
            )
        self.logger.error(
            "VLAD: 0x%02X no dio una lectura limpia en %d intentos. Se devuelve FALLO en vez "
            "de un valor que puede ser log del firmware — ver _leer_consulta_limpia",
            cmd, self.CONSULTA_MAX_INTENTOS,
        )
        return b""

    def _read_payload(self, cmd: int, expected: int | None = None) -> bytes:
        """Lee [CMD eco][payload] del puerto serie y devuelve solo el payload.

        Devuelve b"" si el largo no coincide o si el eco del CMD no es el esperado.
        El eco es parte del contrato (`send_v2_response` en Vlad25Cmd.cpp) y sirve
        para detectar respuestas desalineadas.
        """
        # `expected` explicito para las respuestas cuyo largo NO es funcion del comando:
        # QUERY_COUNTERS (0x3B) es paginado y cada pagina mide distinto (36/15/40/40), asi
        # que un unico numero en RESP_LEN leeria una ventana equivocada en tres de cuatro.
        if expected is None:
            expected = self.RESP_LEN.get(cmd, 0)

        # SINCRONIZAR CON EL ECO ANTES DE LEER EL PAYLOAD
        # ----------------------------------------------
        # El CDC del VLAD25 lleva DOS cosas por el mismo endpoint: las respuestas del protocolo y
        # el log del firmware en texto. Leer 1+N bytes a ciegas cae en medio de una frase.
        #
        # Medido el 10-Ago con la placa 3645: tres set_atten seguidos devolvieron eco 0x27, 0x65 y
        # 0x74 — que son "'", "e" y "t". No era una respuesta desalineada del equipo: era el log.
        # Eso hacia fallar los 47 puntos de la calibracion y tambien get_health_word().
        #
        # El eco del CMD es el delimitador que el contrato ya provee (`send_v2_response` en
        # Vlad25Cmd.cpp): se descartan bytes hasta encontrarlo, y ahi empieza la trama. Es la misma
        # tecnica que hacia funcionar las pruebas crudas a mano mientras el controlador fallaba.
        raw, descartados = self._sincronizar_con_eco(cmd, expected)
        if len(raw) != 1 + expected:
            self.logger.warning(
                "VLAD: respuesta corta para 0x%02X: %d de %d B",
                cmd, len(raw), 1 + expected,
            )
            return b""
        if raw[0] != cmd:
            self.logger.warning(
                "VLAD: eco de CMD inesperado: 0x%02X (esperado 0x%02X)", raw[0], cmd
            )
            return b""
        # UNA LECTURA QUE HUBO QUE RESINCRONIZAR NO ES UNA RESPUESTA CONFIABLE, y esto se
        # midio en la placa 2080388C4B30 el 09-Sep-2026 con el controlador de produccion:
        #
        #   WARNING: 41 B descartados antes del eco de 0x35: b'8 up=871s sin calibrar  P_UL=...'
        #   get_health_word() -> 8224
        #
        # 8224 es 0x2020, o sea los bytes `20 20`: DOS ESPACIOS. El log del firmware llevaba
        # un `'5'` —que es 0x35— y la sincronizacion lo tomo como el eco. La palabra resultante
        # decia los cinco bits obligatorios en cero y sin 12 V: un equipo roto. La misma placa,
        # con una lectura limpia, contesta 0x089F y 12 V, confirmado por 0x08 con 11931 mV.
        #
        # POR QUE EL FLUSH NO ALCANZA: el log no estaba en el buffer de antes, LLEGA MIENTRAS
        # esperamos la respuesta. Limpiar antes de escribir saca el residuo viejo y no puede
        # hacer nada con el caudal nuevo.
        #
        # Y por que un byte de eco es un marco de trama demasiado debil: `0x35` es el digito
        # `'5'`, que aparece en cualquier linea de log con numeros. Lo mismo `0x30`-`0x39`,
        # `0x20`, y las letras. El contrato V2 no tiene delimitador ni largo ni CRC — eso se
        # arregla en el firmware, separando los transportes o silenciando el log durante una
        # sesion de comandos. Desde aca lo unico honesto es distinguir las dos clases:
        #
        #   descartados == 0   el eco fue el PRIMER byte tras escribir: es la respuesta
        #   descartados  > 0   pudo ser el eco, o un byte igual dentro del log
        #
        # El llamador reintenta hasta conseguir una limpia (ver `_query_command`). Aca solo se
        # informa, porque este metodo tambien lo usa el camino de ACK, donde reintentar
        # significaria reenviar el comando.
        if descartados:
            self.logger.warning(
                "VLAD: %d B descartados antes del eco de 0x%02X — lectura RESINCRONIZADA, el "
                "valor puede ser log y no una respuesta: %r",
                descartados, cmd, bytes(raw[:1]),
            )
        self.ultima_lectura_resincronizada = descartados > 0
        return raw[1:]

    def _send_command(self, cmd: int, payload: bytes = b"") -> bool:
        """Envia [CMD][payload] y valida el ACK del firmware.

        A diferencia de la version anterior, no alcanza con que el write salga:
        el firmware responde [CMD eco][ACK=0x00 | NACK=0xFF] y eso es lo que se
        verifica. Sin esta comprobacion, un comando rechazado se reportaba como
        exito.
        """
        frame = bytes([cmd]) + payload
        if self._is_simulation:
            self.logger.info("VLAD (Simulation) TX -> %s", frame.hex(" "))
            return True

        t0 = time.perf_counter()

        try:
            with self._lock:
                self._state = InstrumentState.BUSY
                self._start_watchdog()
                if self.use_bluetooth and self._ble_client:
                    try:
                        async def do_write():
                            await self._ble_client.write_gatt_char(
                                self.NUS_RX_UUID, frame, response=False,
                            )
                        self._run_async(do_write(), timeout=2.0)
                        self.logger.debug("VLAD (BLE) TX -> %s", frame.hex(" "))

                        # Se VERIFICA el ACK, igual que en USB.
                        #
                        # Antes este camino devolvia True apenas escribia: un NACK del
                        # firmware se reportaba como exito, y con RF de por medio eso
                        # significa creer que se aplico una atenuacion que no se
                        # aplico. Ahora que el ACK vuelve por lectura GATT no hay
                        # excusa para no mirarlo.
                        ack = self._ble_read_echo(cmd)
                        if not ack:
                            self.logger.error("VLAD (BLE) sin ACK para 0x%02X", cmd)
                            self._metrics.record_failure(is_ble_timeout=True)
                            return False
                        if len(ack) < 2 or ack[1] != self.ACK:
                            self.logger.error(
                                "VLAD (BLE) firmware respondio NACK (%s) a 0x%02X",
                                ack.hex(" "), cmd,
                            )
                            self._metrics.record_failure()
                            return False

                        latency_ms = int((time.perf_counter() - t0) * 1000)
                        self._metrics.record_success(latency_ms)
                        return True
                    except (OSError, AttributeError) as exc:
                        # SE DEVUELVE False, NO SE PROPAGA: la firma dice `-> bool` y un
                        # OSError escapandose de aca llegaba hasta la suite de
                        # calibracion, que solo mira `if not ok`. Se declara la perdida
                        # de enlace, que es lo que el llamador necesita saber.
                        self._marcar_enlace_perdido(f"escritura BLE: {exc}")
                        self._metrics.record_failure(is_connection=True)
                        return False

                if not self.use_bluetooth and self._serial_connection:
                    try:
                        # Limpiar ANTES de escribir: lo que ya estaba en el buffer no es
                        # respuesta de nada, y el eco no lo puede desambiguar. Ver
                        # `_limpiar_entrada`, que lleva el caso medido del 0x2020.
                        self._limpiar_entrada()
                        self._serial_connection.write(frame)
                        self.logger.debug("VLAD (USB) TX -> %s", frame.hex(" "))
                        ack = self._read_payload(cmd)
                        if not ack:
                            self._metrics.record_failure(is_usb_timeout=True)
                            return False
                        if ack[0] != self.ACK:
                            self.logger.error(
                                "VLAD: firmware respondio NACK (0x%02X) a 0x%02X",
                                ack[0], cmd,
                            )
                            self._metrics.record_failure()
                            return False
                        latency_ms = int((time.perf_counter() - t0) * 1000)
                        self._metrics.record_success(latency_ms)
                        return True
                    except (OSError, AttributeError) as exc:
                        # SE DEVUELVE False, NO SE PROPAGA: la firma dice `-> bool` y un
                        # OSError escapandose de aca llegaba hasta la suite de
                        # calibracion, que solo mira `if not ok`. Se declara la perdida
                        # de enlace, que es lo que el llamador necesita saber.
                        self._marcar_enlace_perdido(f"escritura USB: {exc}")
                        self._metrics.record_failure(is_connection=True)
                        return False

            self._metrics.record_failure()
            return False
        finally:
            self._stop_watchdog()
            if self._state == InstrumentState.BUSY and not self._is_hung:
                self._state = InstrumentState.CONNECTED

    def _ble_read_echo(self, cmd: int, timeout_s: float = 8.0) -> bytes:
        """Lee la caracteristica TX hasta que el eco del CMD coincida.

        El firmware NO notifica: con SDK 2.0.89 el modulo ST67W611M1 acepta
        AT+BLEGATTSNTFY con OK y no entrega nada. La respuesta se deja disponible con
        AT+BLEGATTSRD y el central la LEE (fw-vlad, runbook Parte 14 §7).

        Por eso hay que SONDEAR y no leer una sola vez: entre el write y el ACK pasan
        el ciclo de bleTask y el round-trip AT del SetReadData, asi que una lectura
        inmediata devuelve la respuesta del comando ANTERIOR. Eso no daria error —
        daria un valor plausible y equivocado, que es el peor de los fallos.

        El desempate es el eco: el protocolo V2 garantiza el CMD en el byte 0 de toda
        respuesta, asi que una lectura cuyo byte 0 no sea `cmd` es rancia por
        definicion. Devuelve b"" si se agota el tiempo.

        El timeout es de 8 s y no de 3, y eso se midio: con el sondeo a 1 s —necesario para
        que el modulo no devuelva ATT UNLIKELY_ERROR— tres lecturas no alcanzan. El camino
        de respuesta del firmware incluye el round-trip AT del SetReadData, que no es rapido.
        Una suite que tarda mas y termina vale mas que una rapida que falla por impaciencia.
        """
        # El sondeo es DELIBERADAMENTE lento.
        #
        # Cada lectura GATT es un round-trip AT sobre el SPI que el modulo comparte, no
        # una consulta a un servidor rapido. Sondear cada 100 ms mandaba ~30 lecturas en
        # 3 s y el modulo respondia ATT UNLIKELY_ERROR (0x0E) y cortaba la conexion;
        # despues el circuit breaker se abria y la corrida entera se perdia. A mano, una
        # sola lectura tras ~1 s siempre funciono.
        #
        # Con esta cadencia son ~7 lecturas en 3 s: suficiente para no leer rancio y
        # poco para no saturar el puente.
        # Medido el 08-Ago: con 0.4 s el modulo devuelve ATT UNLIKELY_ERROR (0x0E) y corta
        # la conexion al segundo test de la suite. Cada comando hacia 1 escritura + hasta 8
        # lecturas; tres comandos son ~27 operaciones GATT en pocos segundos.
        #
        # A mano, con UNA lectura tras 2-3 s, nunca fallo. Asi que el problema no es la
        # lectura: es la CADENCIA. Con 1.0 s son ~3 lecturas por comando en vez de 8.
        #
        # El costo es latencia por comando, y es el precio correcto: una suite que tarda mas
        # y termina vale mas que una rapida que corta a los 3 segundos.
        FIRST_WAIT_S = 1.0
        POLL_INTERVAL_S = 1.0

        time.sleep(FIRST_WAIT_S)
        deadline = time.perf_counter() + timeout_s
        last = b""
        while time.perf_counter() < deadline:
            async def do_read():
                return bytes(await self._ble_client.read_gatt_char(self.NUS_TX_UUID))
            try:
                last = self._run_async(do_read(), timeout=1.5)
            except (OSError, AttributeError) as exc:
                self.logger.debug("VLAD (BLE) read intermedio fallo: %s", exc)
                last = b""
            # Una lectura que no devuelve bytes no es una respuesta: normalizar aca en
            # vez de reventar en el slice de abajo. Pasa con transportes mockeados y
            # tambien si bleak devuelve None por un fallo intermedio.
            if not isinstance(last, (bytes, bytearray)):
                last = b""
            if last[:1] == bytes([cmd]):
                return last
            time.sleep(POLL_INTERVAL_S)

        if last:
            self.logger.error(
                "VLAD (BLE) respuesta rancia para 0x%02X: eco 0x%02X tras %.1f s",
                cmd, last[0], timeout_s,
            )
        return b""

    def _query_command(self, cmd: int, payload: bytes = b"",
                       expected: int | None = None) -> bytes:
        """Envia [CMD][payload] y devuelve el payload binario de la respuesta.

        Devuelve b"" en cualquier fallo. El largo esperado sale de RESP_LEN: el
        protocolo V2 no delimita, asi que hay que leer exactamente lo que el
        comando declara — no se puede usar readline().
        """
        if self._is_simulation:
            return b""

        frame = bytes([cmd]) + payload
        t0 = time.perf_counter()

        try:
            with self._lock:
                self._state = InstrumentState.BUSY
                self._start_watchdog()
                if self.use_bluetooth and self._ble_client:
                    try:
                        async def do_write():
                            await self._ble_client.write_gatt_char(
                                self.NUS_RX_UUID, frame, response=False,
                            )
                        self._run_async(do_write(), timeout=2.0)

                        # Sondear hasta que el eco coincida: una lectura inmediata trae
                        # la respuesta del comando ANTERIOR. Ver _ble_read_echo.
                        response = self._ble_read_echo(cmd)
                        if not response:
                            self._metrics.record_failure(is_ble_timeout=True)
                            return b""

                        self.logger.debug("VLAD (BLE) RX <- %s", response.hex(" "))
                        latency_ms = int((time.perf_counter() - t0) * 1000)
                        self._metrics.record_success(latency_ms)
                        # El eco ya se verifico en _ble_read_echo, asi que se saca sin
                        # condicional: dejarlo pasar cuando no coincide era lo que
                        # convertia una respuesta rancia en un dato.
                        return response[1:]
                    except (OSError, AttributeError) as exc:
                        self._marcar_enlace_perdido(f"lectura BLE: {exc}")
                        self._metrics.record_failure(is_ble_timeout=True)
                        return b""

                if not self.use_bluetooth and self._serial_connection:
                    try:
                        # Limpiar ANTES de escribir: lo que ya estaba en el buffer no es
                        # respuesta de nada, y el eco no lo puede desambiguar. Ver
                        # `_limpiar_entrada`, que lleva el caso medido del 0x2020.
                        response = self._leer_consulta_limpia(cmd, frame, expected)
                        self.logger.debug("VLAD (USB) RX <- %s", response.hex(" "))
                        if not response:
                            self._metrics.record_failure(is_usb_timeout=True)
                            return b""
                        latency_ms = int((time.perf_counter() - t0) * 1000)
                        self._metrics.record_success(latency_ms)
                        return response
                    except (OSError, AttributeError) as exc:
                        self._marcar_enlace_perdido(f"lectura USB: {exc}")
                        self._metrics.record_failure(is_usb_timeout=True)
                        return b""

            return b""
        finally:
            self._stop_watchdog()
            if self._state == InstrumentState.BUSY and not self._is_hung:
                self._state = InstrumentState.CONNECTED

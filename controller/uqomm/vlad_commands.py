"""API de comandos del VLAD25: un metodo por operacion del equipo.

Uso:   se mezcla en VladController; nadie la instancia sola.
NO hace: no arma bytes —eso es `vlad25_protocol`— ni los pone en el cable —eso es
         `vlad_transport`—. Traduce entre unidades naturales y las dos primitivas
         `_guarded_send()` / `_guarded_query()`, y guarda el ultimo desglose leido.

Orden: por flujo de uso, no por tipo. Primero la atenuacion (puesta en servicio), despues
ganancia y SmartRing, despues el AGC, y al final las lecturas de diagnostico.

LO QUE TODO LLAMADOR TIENE QUE SABER: en simulacion los setters devuelven True SIN ESCRIBIR
NADA, y los getters devuelven el valor cacheado. Preguntar `datos_confiables()` antes de
creerse cualquier numero de aca. Los tres getters de diagnostico —`get_atten_parts`,
`get_health_word`, `get_agc_sense`— se niegan a inventar y devuelven None en simulacion, a
proposito: una calibracion que los consuma tiene que fallar ruidosamente, no recibir un
numero creible.
"""

from __future__ import annotations

import struct
import time

from controller.uqomm import vlad25_protocol as proto


class VladCommandsMixin:
    """Comandos del protocolo V2 del VLAD25, en unidades naturales.

    La clase anfitriona provee `_guarded_send()`, `_guarded_query()`, `_is_simulation`,
    `logger`, y los campos `last_*` donde se guarda cada desglose.
    """


    # set_dac() SE BORRO (09-Sep-2026). Escribia el codigo crudo del DAC por 0x01/0x02 y
    # tenia CERO llamadores en todo el repo, tests incluidos, a traves de varias auditorias.
    #
    # Peor que muerto: tres comentarios lo citaban como el ejemplo del setter que "devuelve
    # True sin escribir nada en simulacion", justificando con el los gates de calibracion. Un
    # lector que fuera a verificar ese peligro encontraba un metodo sin consumidores y podia
    # concluir que el gate sobraba. Los tres comentarios ahora nombran set_agc/set_atten/
    # set_gain, que son los que de verdad corren.
    #
    # `proto.encode_set_dac()` SE QUEDA: documenta el encoding de 0x01/0x02 con sus casos de
    # test, y el protocolo es contrato con el firmware aunque este controlador no lo use.
    #
    # Y la asimetria con set_atten_fija(), que tampoco tiene consumidor de produccion, es a
    # proposito: esa cierra un hueco de CORRECCION —sin ella la API expone la mitad de la
    # atenuacion sin decirlo, que es la confusion que la auditoria vino a arreglar— mientras
    # esta solo ofrecia una via cruda de escribir el DAC que nadie pidio. Recuperable de git.

    def set_atten(self, atten_db: float, channel: str = "UL") -> bool:
        """Configura la parte **VARIABLE** de la atenuación, en dB (0.0 a 31.5).

        NO ES EL TOTAL, y hasta el 09-Sep-2026 este docstring decía que sí. Sobre el mismo
        BDA4601 actúan dos partes y el firmware nunca acepta una suma: la que se escribe acá
        es la de DISTANCIA, la misma que mueve el switch rotatorio de la placa. El total es
        `aplicada = acotar(fija) + acotar(variable)`, acotado a 31,5 dB.

        O sea que **`set_atten(0.0)` no deja el atenuador en cero**: queda la fija, más lo que
        aporte el rotatorio si no está en la posición 0 (1 dB por posición, hasta 15).

        El default de firmware de la fija es **cero en las dos ramas** desde el 09-Sep-2026
        —antes eran 6,0 UL / 7,0 DL— pero eso **solo lo ve una EEPROM virgen**: una placa en
        servicio conserva lo que tenga escrito, y se han visto 19 dB. No se puede suponer el
        valor: hay que leerlo con `get_atten_parts()`, que es lo único que separa las dos
        partes. Para escribir la fija, `set_atten_fija()`.

        Específico del Rev25: no está en la interfaz base `Vlad` ni en `DummyVlad`, porque la
        base es compartida con Rev24. Usar solo con `VladController`.

        El firmware aplica esto tomando `spi3_mutex`, bus compartido con el LoRa
        Base: la escritura contiende con LoRa y puede tardar más que las otras.
        """
        cmd, payload = proto.encode_set_atten(atten_db, channel)
        self.logger.info(
            "VLAD: Setting atten VARIABLE[%s] to %.1f dB (cmd=0x%02X); el total suma la fija...",
            channel.upper(), proto.clamp_atten_db(atten_db), cmd,
        )
        return self._guarded_send(cmd, payload, "set_atten")

    def set_atten_fija(self, atten_db: float, channel: str = "UL") -> bool:
        """Escribe la parte FIJA de la atenuación (0x43/0x44), la de puesta en servicio.

        Levanta ValueError si el valor está fuera de 0-31,5 dB **antes** de mandarlo: estos
        dos comandos responden NACK en vez de recortar, y un NACK ya en el controlador es
        indistinguible de un fallo de transporte. Ver `proto.encode_set_aten_fija`.
        """
        cmd, payload = proto.encode_set_aten_fija(atten_db, channel)
        self.logger.info(
            "VLAD: Setting atten FIJA[%s] to %.1f dB (cmd=0x%02X)...",
            channel.upper(), float(atten_db), cmd,
        )
        return self._guarded_send(cmd, payload, "set_atten_fija")

    def get_atten_parts(self) -> dict | None:
        """Lee las dos partes de la atenuación de las dos ramas (0x45), o None si no se pudo.

        Es el instrumento que contesta «por qué el equipo atenúa más de lo que le pedí»: la
        parte `variable_*` incluye lo que puso el switch ROTATORIO, que no se ve desde
        ningún otro comando. En el banco la llave estuvo en F, que atenúa 15 dB.

        Sin esto, una calibración que pide `set_atten(0)` y mide 21 dB de atenuación de más
        lo lee como pérdida de RF del montaje, y va a buscar el problema a los cables.
        """
        if self._is_simulation:
            self.logger.error("VLAD: get_atten_parts() en simulacion — no hay medicion")
            return None

        resp = self._guarded_query(self.CMD_QUERY_ATEN_PARTES, "get_atten_parts")

        if not resp:
            self.logger.error("VLAD: no se pudieron leer las partes de la atenuacion")
            return None

        try:
            partes = proto.decode_aten_partes(resp)
        except (ValueError, struct.error) as exc:
            self.logger.error("VLAD: QUERY_ATEN_PARTES ilegible: %s", exc)
            return None

        self.last_atten_parts = partes
        return partes

    def set_gain(self, gain_db: float, channel: str = "UL") -> bool:
        """Set the RF gain level in dB (clamped to 0 to 31.5).

        Args:
            gain_db: Target gain in dB.
            channel: "UL" o "DL" — el firmware tiene un comando por canal.
        """
        cmd, payload = proto.encode_set_gain(gain_db, channel)
        self._gain_db = proto.clamp_gain_db(gain_db)
        self.logger.info(
            "VLAD: Setting gain[%s] to %.1f dB (cmd=0x%02X)...",
            channel.upper(), self._gain_db, cmd,
        )
        return self._guarded_send(cmd, payload, "set_gain")

    def set_smartring(self, enabled: bool,
                      ventana_ms: int = proto.SMARTRING_VENTANA_DEFAULT_MS,
                      freq_hz: int | None = None,
                      umbral_dBm: int | None = None,
                      hist_dB: int | None = None,
                      offset_dB: int | None = None) -> bool:
        """Enciende o apaga la escucha de SmartRing y fija sus parametros.

        Funciona por el transporte que este activo —USB o BLE— sin nada especifico: el
        firmware despacha los tres transportes con el mismo parser, asi que por LoRa el
        comando es el mismo byte a byte cuando lo manda sw-diagnosticoremoto a traves del
        gateway2lora.

        LOS CUATRO OPCIONALES SON LA BANDA Y LA REGLA DE DECISION, y hasta el 17-Sep-2026
        este metodo no podia tocarlos: mandaba SIEMPRE los 3 bytes. El firmware acepta 3, 7
        y 11 desde el 16-Sep, asi que el banco no tenia forma de fijar el umbral ni de
        mover la frecuencia — justo los parametros que el sistema declara configurables.

        El acotado viaja en `proto`, replicado del firmware, porque el firmware ACOTA en vez
        de rechazar: un pedido fuera de rango devuelve ACK igual. El aviso de abajo es lo
        unico que distingue «se aplico» de «se aplico otra cosa» antes de la relectura.
        """
        cmd, payload = proto.encode_set_smartring(
            enabled, ventana_ms, freq_hz, umbral_dBm, hist_dB, offset_dB)
        for nombre, pedido, acotar in (
            ("freq_hz", freq_hz, proto.clamp_smartring_freq_hz),
            ("umbral_dBm", umbral_dBm, proto.clamp_smartring_umbral_dBm),
            ("hist_dB", hist_dB, proto.clamp_smartring_hist_dB),
            ("offset_dB", offset_dB, proto.clamp_smartring_offset_dB),
        ):
            if pedido is not None and acotar(pedido) != int(pedido):
                self.logger.warning(
                    "VLAD: SmartRing %s=%s fuera de rango, se aplicara %s",
                    nombre, pedido, acotar(pedido),
                )
        aplicado = proto.clamp_smartring_ventana(ventana_ms)
        if aplicado != int(ventana_ms):
            self.logger.warning(
                "VLAD: ventana de SmartRing %d ms fuera de %d-%d, se aplicara %d ms",
                int(ventana_ms), proto.SMARTRING_VENTANA_MIN_MS,
                proto.SMARTRING_VENTANA_MAX_MS, aplicado,
            )
        self.logger.info("VLAD: SmartRing %s, ventana de escucha %d ms",
                         "ON" if enabled else "OFF", aplicado)
        return self._guarded_send(cmd, payload, "set_smartring")

    #: Reintentos para conseguir una respuesta NUEVA cuando se pide fresh=True.
    #: Son POCOS a proposito. La primera version usaba 6 con 0,5 s de espera y eso resulto
    #: peor que el problema que arreglaba: cada reintento es un write mas un read de GATT,
    #: y veinte segundos de ese castigo terminaban tirando la conexion con UNLIKELY_ERROR.
    #: La forma correcta de vencer el atraso del camino de lectura es SEPARAR las dos
    #: lecturas en el tiempo, no consultar mas seguido — eso lo hace quien llama.
    SMARTRING_FRESH_TRIES = 3

    def get_smartring(self, fresh: bool = False) -> dict | None:
        """Devuelve {enabled, ventana_ms, detecciones, seq}, o None si no se pudo leer.

        Las DETECCIONES son lo que hace verificable la funcion: un `enabled=True` sin
        detecciones acumulandose significa que el equipo escucha y no oye nada, y eso NO
        se ve en el enable.

        `fresh=True` exige una respuesta DISTINTA de la ultima leida, y no es un lujo:
        por BLE la respuesta no se empuja —queda disponible para que el central la lea— y
        llega con un atraso de un par de ciclos. Dos llamadas consecutivas devuelven la misma
        trama, con el mismo contador. Sin esto, comparar dos lecturas para ver si avanza
        da un falso negativo: el equipo detecta y la medicion dice que no.
        """
        for intento in range(self.SMARTRING_FRESH_TRIES if fresh else 1):
            payload = self._guarded_query(proto.CMD_QUERY_SMARTRING, "get_smartring")
            if not payload:
                return None
            try:
                estado = proto.decode_smartring(payload)
            except ValueError as exc:
                self.logger.error("VLAD: QUERY_SMARTRING ilegible: %s", exc)
                return None

            if not fresh or self._last_smartring_seq is None:
                self._last_smartring_seq = estado["seq"]
                return estado
            if estado["seq"] != self._last_smartring_seq:
                self._last_smartring_seq = estado["seq"]
                return estado
            self.logger.debug(
                "VLAD: QUERY_SMARTRING rancia (seq=%d), reintento %d",
                estado["seq"], intento + 1,
            )
            time.sleep(1.0)

        self.logger.error(
            "VLAD: QUERY_SMARTRING no entrego una respuesta nueva en %d intentos",
            self.SMARTRING_FRESH_TRIES,
        )
        return None

    def get_smartring_rssi(self) -> dict | None:
        """QUERY_SMARTRING_RSSI (0x51): los tres parametros, el RSSI corregido y la decision.

        Es la RELECTURA que hace verificable a `set_smartring`: el firmware contesta ACK a un
        pedido que acota, asi que el ACK no prueba que el umbral haya quedado donde se pidio.
        Ese defecto ya ocurrio del lado del firmware con la frecuencia —se pidieron 143,000
        MHz, contesto ACK y 0x2A seguia devolviendo 144,000— y lo delato la relectura, no el
        acuse.

        `modo_sugerido` es lo que el anillo PEDIRIA. La llave manual es una compuerta, asi
        que el modo vigente sale de QUERY_STATUS y puede no coincidir: esa diferencia es
        informacion, no un error.
        """
        payload = self._guarded_query(proto.CMD_QUERY_SMARTRING_RSSI, "get_smartring_rssi")
        if not payload:
            return None
        try:
            return proto.decode_smartring_rssi(payload)
        except ValueError as exc:
            self.logger.error("VLAD: QUERY_SMARTRING_RSSI ilegible: %s", exc)
            return None

    def get_counters(self, pagina: int = 2) -> dict | None:
        """QUERY_COUNTERS (0x3B): una pagina de contadores del firmware.

        La pagina 2 es la de LoRa, y es la que responde las preguntas de enlace: `rx_intentos`
        contra `rx_timeouts` distingue «escucha y no hay nadie» de «no escucha», que es
        exactamente lo que mirar el RSSI NO puede distinguir —el RSSI solo se actualiza cuando
        LLEGA un paquete, asi que su valor inicial no significa nada—.

        Devuelve None si la lectura fallo. No devuelve un diccionario de ceros: el firmware
        contesta ceros ante una pagina inexistente y eso se lee igual que un equipo sano.
        """
        payload = self._query_command(
            proto.CMD_QUERY_COUNTERS, bytes([pagina]),
            proto.COUNTERS_RESP_LEN.get(pagina, 0),
        )
        if not payload:
            self.logger.error("VLAD: QUERY_COUNTERS pagina %d sin respuesta", pagina)
            return None
        try:
            return proto.decode_counters(pagina, payload)
        except ValueError as exc:
            self.logger.error("VLAD: QUERY_COUNTERS pagina %d ilegible: %s", pagina, exc)
            return None

    def set_mode(self, reverse: bool) -> bool:
        """SET_MODE (0x03): conmuta el camino de RF entre CONVENCIONAL y REVERSE.

        Mueve los cuatro SPDT SKY13374 (IC7-IC10) con la guarda de conmutacion que pide el
        esquematico, y persiste el modo. NO devuelve si quedo aplicado: para eso hay que releer
        `es_reverse` de la trama 0x3C, que el firmware arma leyendo el PIN.

        OJO CON SMARTRING. Con la llave SR en automatico, SmartRing decide el modo y puede
        revertir esto en su proximo tick. Para una medicion hay que apagarlo antes con
        `set_smartring(False)` y reponerlo al terminar.
        """
        return self._guarded_send(proto.CMD_SET_MODE,
                                  bytes([1 if reverse else 0]), "set_mode")

    def get_telemetry_frame(self) -> dict | None:
        """QUERY_TELEMETRY_FRAME (0x3C): la trama v5 armada AHORA, por USB.

        Es la lectura que sirve para calibrar: devuelve la cuenta del ADC tal cual, sin la
        cuantizacion por zonas que la trama legacy 0x11 aplica al nivel de DL.
        """
        # SE PIDE EL LARGO DE LA v5 Y SE RELEE SI HACE FALTA. El transporte espera un largo fijo,
        # asi que pedir el de la v6 a una placa v5 se cuelga en los reintentos y devuelve fallo
        # —probado el 21-Sep-2026: «respuesta corta para 0x3C: 53 de 60 B», tres veces—. Al reves
        # no: leer 52 de una trama de 59 trae el prefijo entero, y el byte 1 ya dice la VERSION.
        # El pedido siguiente limpia la entrada antes de escribir, asi que los bytes que quedaron
        # no desincronizan nada.
        payload = self._query_command(
            proto.CMD_QUERY_TELEMETRY_FRAME, b"", proto.TELEMETRY_FRAME_LEN)
        if not payload:
            self.logger.error("VLAD: QUERY_TELEMETRY_FRAME sin respuesta")
            return None
        version = payload[1] if len(payload) > 1 else 0
        largo = proto.TELEMETRY_FRAME_LEN_POR_VERSION.get(version)
        if largo is not None and largo > len(payload):
            self.logger.info("VLAD: 0x3C dice VERSION %d (%d B): releyendo", version, largo)
            payload = self._query_command(proto.CMD_QUERY_TELEMETRY_FRAME, b"", largo)
            if not payload:
                self.logger.error("VLAD: QUERY_TELEMETRY_FRAME v%d sin respuesta", version)
                return None
        try:
            return proto.decode_telemetry_frame(payload)
        except ValueError as exc:
            self.logger.error("VLAD: QUERY_TELEMETRY_FRAME ilegible: %s", exc)
            return None

    def get_telemetry(self) -> dict | None:
        """QUERY_TELEMETRY (0x2F): estado del uplink periodico de telemetria.

        Contesta la pregunta que el servidor NO puede contestar: un panel sin tramas se ve igual
        si el equipo nunca transmitio o si transmitio y el puente las descarto. `tx_count` lo
        separa, y `on` dice si siquiera esta habilitado — arranca en 0 y lo prende 0x2E.
        """
        payload = self._query_command(
            proto.CMD_QUERY_TELEMETRY, b"", proto.TELEMETRY_RESP_LEN)
        if not payload:
            self.logger.error("VLAD: QUERY_TELEMETRY sin respuesta")
            return None
        try:
            return proto.decode_telemetry(payload)
        except ValueError as exc:
            self.logger.error("VLAD: QUERY_TELEMETRY ilegible: %s", exc)
            return None

    def get_radio_regs(self, modulo: int = 1, desde: int = 0x01,
                       cuanto: int = 32) -> dict | None:
        """QUERY_RADIO_REGS (0x54): registros CRUDOS del SX1278. 0=BASE, 1=TUNEL.

        Es el único camino que lee el SILICIO. Todo lo demás —`get_lora_params`, la palabra de
        salud— informa variables del firmware, o sea la intención: una escritura por SPI que no
        pegara dejaría los getters informando lo que se quiso poner.

        NO SE PUEDE PEDIR 0x00: leer RegFifo saca un byte del FIFO y corrompería el paquete que
        la radio tenga guardado. El firmware lo rechaza con NACK y esto devuelve None.
        """
        payload = self._query_command(
            proto.CMD_QUERY_RADIO_REGS, bytes([modulo, desde, cuanto]), 3 + cuanto)
        if not payload:
            self.logger.error("VLAD: QUERY_RADIO_REGS sin respuesta (¿firmware sin 0x54?)")
            return None
        try:
            d = proto.decode_radio_regs(payload)
        except ValueError as exc:
            self.logger.error("VLAD: QUERY_RADIO_REGS ilegible: %s", exc)
            return None
        if d["modulo"] != modulo or d["desde"] != desde:
            self.logger.error(
                "VLAD: se pidió módulo %d desde 0x%02X y contestó módulo %d desde 0x%02X",
                modulo, desde, d["modulo"], d["desde"])
            return None
        return d

    def set_tono(self, on: bool, modulo: int = 0) -> bool:
        """TONO_CONTINUO (0x3F): portadora continua en una de las dos radios.

        `modulo` va por PUERTO: 0 = BASE (la de SmartRing), 1 = TUNEL (la del diagnostico).

        EL FIRMWARE LO CORTA SOLO A LOS 60 s (`TONO_MAX_S`), asi que una medicion larga hay
        que renovarla. Es deliberado: una portadora continua olvidada encendida ocupa el canal
        de todos los equipos.
        """
        return self._guarded_send(
            proto.CMD_TONO_CONTINUO, proto.encode_tono(on, modulo), "set_tono")

    def set_agc(self, agc_volts: float, channel: str = "UL") -> bool:
        """Set the AGC reference voltage on the DAC.

        Args:
            agc_volts: Referencia del DAC, 0.0 a 3.3 V. **No es dBm**: el firmware
                no implementa la conversion de potencia a tension y no existe curva
                de calibracion, asi que se expone lo que el hardware acepta.
            channel: "UL" o "DL".
        """
        cmd, payload = proto.encode_set_agc(agc_volts, channel)
        self._agc_level = proto.clamp_agc_volts(agc_volts)
        self.logger.info(
            "VLAD: Setting AGC[%s] reference to %.3f V (cmd=0x%02X, codigo DAC previsto %d)...",
            channel.upper(), self._agc_level, cmd,
            proto.volts_to_dac_code(self._agc_level),
        )
        return self._guarded_send(cmd, payload, "set_agc")

    def _agc_cal_de(self, channel: str) -> tuple[float, float, float]:
        """La calibracion dB<->mV de ESTA unidad para una rama: (mv_ref, dbm_ref, mv_por_db).

        Se inicializa perezosamente desde el default de familia (proto.AGC_CAL_DEFAULT) y se ajusta
        por unidad con set_agc_cal(). Perezoso para no tocar __init__ de la clase anfitriona."""
        if not hasattr(self, "_agc_cal"):
            self._agc_cal = {
                ch: (mv, dbm, proto.AGC_MV_POR_DB)
                for ch, (mv, dbm) in proto.AGC_CAL_DEFAULT.items()
            }
        return self._agc_cal[channel.upper()]

    def set_agc_cal(self, channel: str, mv_ref: float, dbm_ref: float,
                    mv_por_db: float | None = None) -> None:
        """Ajusta la calibracion dB<->mV del AGC para ESTA unidad y rama.

        POR QUE EXISTE: el default es DL 0 dBm=1339 mV, UL -10 dBm=1125 mV (medido en una placa).
        En OTRO VLAD 0 dBm puede caer en otro mV. Ahi medis el punto real de esa unidad —fijas una
        referencia con set_agc(volts) o set_agc_db, medis con el analizador a que dBm clampa la
        salida— y cargas ese par aca. Desde entonces set_agc_db/get_agc_db son exactos para esta
        unidad. La pendiente rara vez cambia (es la del MAX4003, ~25,5 mV/dB); si no la pasas, se
        conserva. Es per-instancia; no persiste entre sesiones (todavia)."""
        self._agc_cal_de(channel)   # asegura init
        ch = channel.upper()
        slope = self._agc_cal[ch][2] if mv_por_db is None else float(mv_por_db)
        self._agc_cal[ch] = (float(mv_ref), float(dbm_ref), slope)
        self.logger.info(
            "VLAD: AGC cal[%s] ajustada -> %.0f mV @ %.1f dBm, %.2f mV/dB",
            ch, mv_ref, dbm_ref, slope,
        )

    def set_agc_db(self, target_dbm: float, channel: str = "UL") -> bool:
        """Fija el AGC por la potencia de SALIDA a la que se quiere que clampe, en dBm.

        El firmware toma VOLTS (set_agc); esta conversion vive en el controlador, con la
        calibracion por rama de ESTA unidad (set_agc_cal la ajusta; el default es DL 0 dBm~1339 mV,
        UL -10 dBm~1125 mV, ~25,5 mV/dB). Se acota al rango del DAC.
        """
        mv_ref, dbm_ref, slope = self._agc_cal_de(channel)
        volts = proto.agc_dbm_to_volts(target_dbm, mv_ref, dbm_ref, slope)
        self.logger.info(
            "VLAD: AGC[%s] clamp -> %.1f dBm = %.3f V (cal %.0f mV@%.1f dBm, %.2f mV/dB)",
            channel.upper(), target_dbm, volts, mv_ref, dbm_ref, slope,
        )
        return self.set_agc(volts, channel)

    def get_agc_db(self, channel: str = "UL") -> float | None:
        """La potencia de clamp (dBm) del setpoint actual, con la cal de esta unidad. None si no lee.

        Es la lectura simetrica de set_agc_db: contesta 'a que dBm esta puesto el AGC ahora'."""
        if self.get_agc() is None:
            return None
        key = "agc_ul_v" if channel.upper() == "UL" else "agc_dl_v"
        mv_ref, dbm_ref, slope = self._agc_cal_de(channel)
        return proto.agc_volts_to_dbm(self.last_agc_readout[key], mv_ref, dbm_ref, slope)

    def ajustar_agc_db(self, delta_db: float, channel: str = "UL") -> float | None:
        """Corre el AGC un PASO relativo desde donde esta: +1 dB, -0.5 dB, etc. Devuelve el dBm
        resultante (ya acotado por el DAC), o None si no pudo leer el setpoint actual.

        Es la operacion de sintonia fina que pedia el operador: no fijar un absoluto sino
        'movelo medio dB mas'. Lee el clamp actual con get_agc_db (con la cal de ESTA unidad),
        le suma delta_db y lo reescribe con set_agc_db. Como set_agc_db acota al rango del DAC,
        el valor devuelto es el REAL aplicado: si topa, delta efectivo < delta pedido."""
        actual = self.get_agc_db(channel)
        if actual is None:
            self.logger.error("VLAD: no se pudo leer AGC[%s] para ajuste relativo", channel.upper())
            return None
        objetivo = actual + float(delta_db)
        if not self.set_agc_db(objetivo, channel):
            return None
        aplicado = self.get_agc_db(channel)
        self.logger.info(
            "VLAD: AGC[%s] ajuste %+.2f dB: %.2f -> %.2f dBm (real %.2f)",
            channel.upper(), delta_db, actual, objetivo, aplicado if aplicado is not None else objetivo,
        )
        return aplicado if aplicado is not None else objetivo

    def get_gain(self) -> float | None:
        """Query the currently configured RF gain.

        Returns:
            Current gain in dB.
        """
        if self._is_simulation:
            return self._gain_db

        resp = self._guarded_query(self.CMD_QUERY_GAIN, "get_gain")

        if not resp:
            self.logger.error("VLAD: no se pudo leer GAIN del hardware")
            return None

        try:
            self.last_gain_readout = proto.decode_gain(resp)
        except struct.error as exc:
            self.logger.warning(
                "VLAD: respuesta GAIN de largo inesperado (%d B): %s", len(resp), exc
            )
            return None

        # SE MIRA EL INVARIANTE QUE LA PROPIA TRAMA AFIRMA.
        #
        # 0x0C lleva la ganancia y la atenuacion aplicada de la MISMA rama, asi que
        # `ganancia + aplicada == MAX_RAMA` es lo que la respuesta promete por su forma. El
        # 09-Sep-2026 el firmware informaba 29,6 dB haciendo 14,6 porque no contaba el
        # rotatorio, y la contradiccion viajaba en esta misma trama sin que nada la mirara.
        #
        # Se avisa y se devuelve el numero igual: quien calibra tiene que poder decidir. Lo
        # que no puede pasar es que el desvio sea invisible.
        for rama in ("ul", "dl"):
            if not self.last_gain_readout.get(f"gain_{rama}_coherente", True):
                self.logger.warning(
                    "VLAD: QUERY_GAIN incoherente en %s: ganancia %.3f dB + atenuacion %.3f dB "
                    "= %.3f dB, y el maximo de rama es %.3f (residuo %.3f dB). La ganancia "
                    "informada no describe al equipo; leer get_atten_parts() para ver el rotatorio",
                    rama.upper(),
                    self.last_gain_readout[f"gain_{rama}_db"],
                    self.last_gain_readout[f"atten_{rama}_db"],
                    self.last_gain_readout[f"gain_{rama}_db"]
                    + self.last_gain_readout[f"atten_{rama}_db"],
                    proto.GAIN_MAX_UL_DB if rama == "ul" else proto.GAIN_MAX_DL_DB,
                    self.last_gain_readout[f"coherencia_{rama}_db"],
                )

        self._gain_db = self.last_gain_readout["gain_ul_db"]
        return self._gain_db

    def get_agc(self) -> float | None:
        """Query the AGC reference voltage currently applied.

        Returns:
            Referencia del canal UL en volts, o None si no se pudo leer.
            El detalle de los dos canales queda en `last_agc_readout`.
        """
        if self._is_simulation:
            return self._agc_level

        resp = self._guarded_query(self.CMD_QUERY_AGC, "get_agc")

        if not resp:
            self.logger.error("VLAD: no se pudo leer AGC del hardware")
            return None

        try:
            self.last_agc_readout = proto.decode_agc(resp)
        except struct.error as exc:
            self.logger.warning(
                "VLAD: respuesta AGC de largo inesperado (%d B): %s", len(resp), exc
            )
            return None

        self._agc_level = self.last_agc_readout["agc_ul_v"]
        return self._agc_level

    def get_health_word(self) -> int | None:
        """Lee la palabra de salud del equipo (DIAGNOSTIC_RUN 0x35), o None si no se pudo.

        Es el self-test del VLAD y **cada bit se prueba de verdad**: las radios contestan su
        ``RegVersion``, el INA responde en I2C, la EEPROM entrega su magic. Antes del 09-Ago el
        firmware tenia DOS self-tests que se contradecian —uno derivaba sus bits de punteros no
        nulos— y ese bit estuvo en 1 mientras la radio no transmitia ni una vez.

        Para la calibracion importa el campo ``VIN_ZONE`` (bits 7..9): es el UNICO hecho sobre
        la alimentacion que viene del EQUIPO y no del instrumento. La fuente puede medir 24 V
        correctos y el riel no llegar a la placa; esos dos hechos son distintos.

        Devuelve None y nunca un cero: cero es una palabra de salud legitima —significa que
        todo esta mal— y confundirla con "no pude leer" es el error de metodo que esta serie
        viene arrastrando.
        """
        if self._is_simulation:
            self.logger.error("VLAD: get_health_word() en simulacion — no hay medicion")
            return None

        resp = self._guarded_query(self.CMD_DIAGNOSTIC_RUN, "get_health_word")

        if not resp:
            self.logger.error("VLAD: no se pudo leer la palabra de salud")
            return None

        try:
            return proto.decode_health_word(resp)
        except ValueError as exc:
            self.logger.error("VLAD: palabra de salud ilegible: %s", exc)
            return None

    def get_firmware_version(self) -> dict | None:
        """Lee la version del firmware del VLAD (FOTA_STATUS 0x1C), o None si falla.

        Returns:
            Dict con 'app_version' (u32), 'version_str' ("X.Y.Z.W"), 'app_size' y
            'pending', o None si esta en simulacion o no responde.
        """
        if self._is_simulation:
            self.logger.error("VLAD: get_firmware_version() en simulacion — no hay medicion real")
            return None

        resp = self._guarded_query(self.CMD_FOTA_STATUS, "get_firmware_version")

        if not resp:
            self.logger.error("VLAD: no se pudo leer la version del firmware")
            return None

        try:
            return proto.decode_fota_status(resp)
        except (ValueError, struct.error) as exc:
            self.logger.error("VLAD: respuesta FOTA_STATUS ilegible: %s", exc)
            return None

    def get_agc_sense(self) -> dict | None:
        """Lee el sensado REAL del lazo de AGC. Es el instrumento de la calibración.

        Devuelve las cuentas crudas del ADC de `AGC_UL` (PB2) y `AGC_DL` (PC5), que son
        la **tensión de control del lazo** analógico, no el nivel de RF detectado.

        Por qué existe y por qué no alcanza `get_agc()`: ese usa QUERY_AGC (0x17), que
        devuelve `dac_code * 3300 / 4095` — aritmética sobre el setpoint que pedimos, o
        sea un **eco**. Da un número plausible incluso con el canal del DAC
        deshabilitado, y eso ocultó un bug real de hardware en el firmware hasta que
        alguien lo comparó contra el valor por defecto. Este comando sí mide.

        Returns:
            El dict de `decode_agc_sense()`, o `None` si no se pudo leer. **Nunca
            devuelve ceros como sustituto de un fallo**: un cero es una lectura
            legítima del lazo y confundirlo con "no pude leer" es exactamente el error
            de método que esta serie viene arrastrando.
        """
        if self._is_simulation:
            # En simulación NO se inventa una lectura: un barrido de calibración que
            # consuma esto tiene que fallar ruidosamente, no recibir un número creíble.
            self.logger.error("VLAD: get_agc_sense() en simulación — no hay medición")
            return None

        resp = self._guarded_query(self.CMD_QUERY_AGC_SENSE, "get_agc_sense")

        if not resp:
            self.logger.error("VLAD: no se pudo leer el sensado del AGC")
            return None

        try:
            readout = proto.decode_agc_sense(resp)
        except struct.error as exc:
            self.logger.warning(
                "VLAD: respuesta de sensado de largo inesperado (%d B): %s", len(resp), exc
            )
            return None

        self.last_agc_sense = readout
        return readout

    def agc_activo(self) -> dict | None:
        """¿El lazo de AGC esta ACTUANDO en cada rama? Lee el sensado (0x1E) y lo compara con el
        reposo (~4084). Devuelve {"ul": bool, "dl": bool, "agc_ul_raw":..., "agc_dl_raw":...} o
        None si no se pudo leer.

        'Actuando' = la cuenta cayo del reposo: el integrador esta bajando la ganancia para clampar
        la salida. Es la respuesta directa a «¿el AGC esta activo en DL/UL?» sin depender de mirar
        el plateau de pout, que es como lo infiere hoy la suite de AGC.
        """
        s = self.get_agc_sense()
        if s is None:
            return None
        return {
            "ul": proto.agc_activo_de_raw(s["agc_ul_raw"]),
            "dl": proto.agc_activo_de_raw(s["agc_dl_raw"]),
            "agc_ul_raw": s["agc_ul_raw"],
            "agc_dl_raw": s["agc_dl_raw"],
        }

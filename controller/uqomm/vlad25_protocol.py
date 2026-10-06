"""
Protocolo Vlad25Cmd V2 — codificación y decodificación, sin I/O.

Funciones puras que traducen entre unidades naturales (dB, volts) y los bytes
que espera el firmware VLAD25 Rev25. El transporte vive en vlad_controller.py.

Formato (Vlad25Cmd.cpp:172-180): si el primer byte NO es 0x7E, el firmware lo
trata como V2 — sin framing y sin CRC.

    Envío:     [CMD][DATA...]
    Respuesta: [CMD eco][payload]     (send_v2_response)

El eco del CMD es parte del contrato y sirve para detectar respuestas
desalineadas. El protocolo no delimita, así que hay que leer exactamente el
largo que declara RESP_LEN — nunca readline().

UNIDADES: tres convenciones distintas, verificadas contra cmd_handlers.cpp y
fixed.hpp del firmware. Es lo que hace fallar los parsers escritos de memoria.

  Entrada, ganancia (0x0A/0x0B) → float32 LE en dB
      bytes_to_mdB() hace `f * 1000.0f`, así que el float lleva dB.

  Entrada, AGC (0x16) → float32 LE en MILIVOLTS
      bytes_to_mV() hace `(uint32_t)(f + 0.5f)` y NO escala por 1000, así que
      el float lleva mV. Mandar float32(1.65) produce 2 mV, no 1.65 V.

  Salida, respuestas QUERY → int32 LE en mili-unidades
      Asimetría deliberada (decisión D1 de F0-3, ID-1702): fixed.hpp mantiene
      el path de salida sin float.
"""

from __future__ import annotations

import struct
from typing import Any

# ---------------------------------------------------------------------------
# Identificadores de comando
# ---------------------------------------------------------------------------
CMD_SET_DAC_UL = 0x01
CMD_SET_DAC_DL = 0x02
CMD_QUERY_STATUS = 0x05
#: 0x06/0x07 fijan la parte **VARIABLE** de la atenuacion, la de distancia — no el total.
#: Ver el bloque «Las dos atenuaciones» mas abajo: el total lo arma el firmware como
#: `fija + variable` acotado, asi que pedir 0 aca NO deja el atenuador en 0.
CMD_SET_ATTEN_UL = 0x06
CMD_SET_ATTEN_DL = 0x07
CMD_QUERY_CURRENT = 0x08
CMD_SET_GAIN_UL = 0x0A
CMD_SET_GAIN_DL = 0x0B
CMD_QUERY_GAIN = 0x0C
CMD_SET_AGC = 0x16
CMD_QUERY_AGC = 0x17
#: El unico comando que LEE HARDWARE del lazo de AGC: muestrea PB2 y PC5 por ADC.
#: QUERY_AGC (0x17) devuelve `dac_code * 3300 / 4095`, o sea un eco del setpoint que
#: pedimos — da un numero plausible incluso con el canal del DAC deshabilitado, y eso
#: es lo que oculto el bug de DAC_CR_EN1 en el firmware. Para medir, este.
CMD_QUERY_AGC_SENSE = 0x1E
CMD_FOTA_STATUS = 0x1C

#: DIAGNOSTIC_RUN — la palabra de salud del equipo, 16 bits little-endian.
#: Un mapa, un llenador, dos consumidores: ver Core/Inc/Health.hpp en fw-vlad. Los cinco bits
#: bajos son el contrato de cable historico (INA linea, INA dispositivo, EEPROM, LoRa base,
#: LoRa tunel) y NO se reordenan.
CMD_DIAGNOSTIC_RUN = 0x35
# SmartRing: tono periodico por LoRa. Entra por el mismo despachador que USB y BLE, asi
# que el equipo se configura por cualquiera de los tres transportes — por LoRa llega desde
# sw-diagnosticoremoto a traves del gateway2lora.
CMD_SET_SMARTRING = 0x29
CMD_QUERY_SMARTRING = 0x2A
#: QUERY_SMARTRING_RSSI. NO es un duplicado de 0x2A: publica la DECISION y sus insumos.
#:
#: Sin el, reconstruir por que el equipo eligio un modo exige saber que la histeresis es
#: ASIMETRICA segun el modo vigente (en CONVENCIONAL el umbral efectivo es `umbral - hist`;
#: en REVERSE es `umbral + hist`). Dos cuentas separadas de la misma regla se desincronizan,
#: y la del banco seria la que esta mal sin que nada lo reporte.
#:
#: 0x51 en el espacio del VLAD25 (`mod_func` 0x02) NO choca con el 0x51 del gateway
#: (`QUERY_FSK_FREQ`, `mod_func` 0x00): mismo cable, espacios de comando distintos.
CMD_SET_MODE = 0x03          # 1 byte: 0 = CONVENCIONAL, !=0 = REVERSE
CMD_QUERY_TELEMETRY = 0x2F
CMD_QUERY_TELEMETRY_FRAME = 0x3C
#: Largos de la trama 0x3C por VERSION. El firmware valida el PAR (version, largo) y este lado
#: tiene que aceptar los dos mientras conviva flota vieja con flota nueva: clavarlo en uno solo
#: deja al banco entero sin instrumental el dia que se graba el otro.
TELEMETRY_FRAME_LEN_POR_VERSION = {5: 52, 6: 59}
#: El largo del PRIMER pedido, y es el de la v5 A PROPOSITO: el transporte exige un largo fijo y
#: espera hasta juntarlo, asi que pedir 59 a una placa v5 —que manda 53 con el eco— se cuelga en
#: los tres reintentos y devuelve fallo. Se pide el chico, se mira la VERSION que contesto, y
#: recien si dice 6 se relee con el largo grande: no cuesta nada en la flota que hay hoy.
TELEMETRY_FRAME_LEN = TELEMETRY_FRAME_LEN_POR_VERSION[5]
TELEMETRY_RESP_LEN = 16
CMD_QUERY_COUNTERS = 0x3B
CMD_QUERY_RADIO_REGS = 0x54
CMD_TONO_CONTINUO = 0x3F
CMD_QUERY_SMARTRING_RSSI = 0x51

# ---------------------------------------------------------------------------
# Las DOS atenuaciones (fw-vlad Core/Inc/Atenuacion.hpp, handlers en atten_handlers.cpp)
# ---------------------------------------------------------------------------
# Sobre el MISMO atenuador BDA4601 actuan dos partes, y el firmware nunca acepta un total:
#
#   FIJA      la pone quien instala, en puesta en servicio.
#
#             DEFAULT DE FIRMWARE: **CERO las dos ramas** (`aten::FIJA_UL_DEFAULT_mdB` /
#             `FIJA_DL_DEFAULT_mdB` en Atenuacion.hpp). O sea que de fabrica el equipo
#             amplifica todo lo que puede y el pad lo agrega quien instala.
#
#             ERAN 6,0 (UL) y 7,0 (DL) hasta el 09-Sep-2026, cuando el firmware las paso a
#             cero. Este comentario decia 6,0/7,0 tres dias despues del cambio y mando a
#             contestarle a alguien que su placa "venia de fabrica" con una fija que ningun
#             firmware puso nunca. Es el modo de falla de un espejo: el numero vive en
#             fw-vlad y aca solo hay una copia que no se entera.
#
#             Y LO QUE MAS IMPORTA AL LEER UNA PLACA: el default **solo lo ve una EEPROM
#             VIRGEN**. Una unidad en servicio conserva lo que tenga escrito, asi que el
#             valor que se lee NO es «el default» de ninguna version — hay que mirarlo.
#             Para dejarla en cero hay que escribirle cero por 0x43/0x44.
#   VARIABLE  depende de la DISTANCIA. La pone el switch ROTATORIO de la placa o el
#             software, y cambia en operacion.
#
#   aplicada = acotar(acotar(fija) + acotar(variable))   con acotar a 0..31,5 dB
#
# POR QUE IMPORTA ACA: `set_atten(0.0)` NO deja el atenuador en cero. Queda la fija que la
# placa tenga escrita, mas lo que aporte el rotatorio, y la aplicada es `fija + posicion`.
#
# Medido en el banco: con la llave en F —15 dB— y la variable PEDIDA en 0, la placa atenuaba
# **21 dB**, porque su fija era 6. Ese 21 es una medicion de esa placa en ese momento, NO una
# cuenta que se pueda repetir con el default: hoy el default de firmware es 0 (ver arriba) y
# una placa en servicio conserva cualquier cosa. Se han visto 19 dB de fija.
#
# QUERY_ATEN_PARTES (0x45) es el unico modo de VER las dos partes por separado, y sin el la
# discrepancia se lee como una perdida de RF del montaje.
#: Default de firmware de la parte FIJA, por rama. Espejo de `aten::FIJA_UL_DEFAULT_mdB` /
#: `FIJA_DL_DEFAULT_mdB` (fw-vlad/vlad25_vhf/Core/Inc/Atenuacion.hpp).
#:
#: EXISTE COMO CONSTANTE Y NO COMO PROSA porque la prosa ya derivo una vez: decia 6,0/7,0
#: —el valor hasta el 09-Sep-2026— tres dias despues de que el firmware los pasara a cero, y
#: con eso se le contesto a un companero que su placa "venia de fabrica" con una fija que
#: ningun firmware puso. `test_vlad25_contrato_fw.py` ahora LEE el header del firmware y
#: exige que estos dos numeros coincidan, asi que la proxima deriva sale en rojo.
#:
#: OJO AL USARLO: es lo que trae una EEPROM VIRGEN, no lo que tiene una placa. Una unidad en
#: servicio conserva lo escrito; para saber su fija hay que leerla con 0x45.
FIJA_UL_DEFAULT_DB = 0.0
FIJA_DL_DEFAULT_DB = 0.0

CMD_SET_ATEN_FIJA_UL = 0x43
CMD_SET_ATEN_FIJA_DL = 0x44
CMD_QUERY_ATEN_PARTES = 0x45

# ES UNA VENTANA DE ESCUCHA, NO UN PERIODO DE TRANSMISION, y la diferencia se midio en la
# placa 2080388C4B30 el 09-Sep-2026: se pidio 500 y el equipo aplico 2500.
#
# LO QUE HABIA ACA ERA EL RANGO DEL GATEWAY. Estos limites decian 500-2000 ms «iguales a los
# del gateway (fw-gateway2lora, SmartRingTransmitter)», que es el PERIODO con que el gateway
# repite el tono. Este equipo no transmite: ESCUCHA, y el campo de 0x29/0x2A es cuanto dura
# cada escucha (`ventana_ms` en Vlad25Cmd.hpp).
#
# Y los dos numeros no pueden ser el mismo por una razon que el firmware escribe al lado de
# sus constantes (SmartRing.hpp): «el tono del gateway se repite cada 500-2000 ms, asi que
# una ventana mas corta que su periodo puede caer entera en el silencio entre dos tonos y
# reportar ausente con el anillo sano. 2500 ms cubre el peor periodo del gateway con margen.»
#
# O sea que TODO el rango que este archivo ofrecia estaba por debajo del minimo del firmware,
# y era exactamente el defecto contra el que el firmware se protege. El maximo acota cuanto
# puede bloquear loraTask, que comparte SPI2 con el modulo BLE.
SMARTRING_VENTANA_MIN_MS = 2500
SMARTRING_VENTANA_MAX_MS = 30000
SMARTRING_VENTANA_DEFAULT_MS = 5000

# Espejo de `smartring::` en fw-vlad Core/Inc/SmartRing.hpp. Se replican los ACOTADOS y no
# solo los defaults, porque el firmware ACOTA en vez de rechazar: un pedido fuera de rango
# devuelve ACK y el equipo se queda con otro valor. Sin el clamp de este lado, el banco
# escribe 200 dB de offset, lee ACK, y la relectura lo desmiente sin que nadie mire.
SMARTRING_FREQ_MIN_HZ = 142_500_000
SMARTRING_FREQ_MAX_HZ = 144_000_000
SMARTRING_FREQ_DEFAULT_HZ = 143_000_000

#: EL UMBRAL VIAJA CON SIGNO EN EL VLAD, y el equipo SmartRing de referencia lo guarda
#: POSITIVO (`60` significa -60 dBm). Son dos encodings de la misma magnitud a dos saltos de
#: distancia: mandarle -60 al equipo de referencia le dice +60. Acá es i16 LE en dBm.
SMARTRING_UMBRAL_MIN_DBM = -125
SMARTRING_UMBRAL_MAX_DBM = 0
SMARTRING_UMBRAL_DEFAULT_DBM = -60      # Alan, 16-Sep-2026

SMARTRING_HIST_MAX_DB = 25
SMARTRING_HIST_DEFAULT_DB = 3           # el receptor de referencia embarca 2: ver SPEC_SmartRing
SMARTRING_OFFSET_MAX_DB = 50
SMARTRING_OFFSET_DEFAULT_DB = 17

#: `RSSI_PISO_DBM` — «nunca oi nada», NO una medicion de -164 dBm. El firmware lo trata como
#: ausencia de dato y NO dispara REVERSE con el, aunque -164 < -60: un detector recien
#: arrancado no es un cable cortado.
SMARTRING_RSSI_PISO_DBM = -164

#: Modo que publica 0x51 en su ultimo byte (`smartring::Modo`).
SMARTRING_MODO = {0: "CONVENCIONAL", 1: "REVERSE"}

#: Largo del payload de respuesta por comando, sin contar el eco del CMD.
RESP_LEN = {
    CMD_SET_DAC_UL: 1,
    CMD_SET_DAC_DL: 1,
    CMD_QUERY_STATUS: 18,
    CMD_SET_ATTEN_UL: 1,
    CMD_SET_ATTEN_DL: 1,
    CMD_QUERY_CURRENT: 12,
    CMD_SET_GAIN_UL: 1,
    CMD_SET_GAIN_DL: 1,
    CMD_QUERY_GAIN: 20,
    CMD_SET_AGC: 1,
    CMD_QUERY_AGC: 8,
    CMD_QUERY_AGC_SENSE: 4,
    CMD_SET_SMARTRING: 1,      # ACK
    CMD_TONO_CONTINUO: 1,      # ACK
    # QUERY_COUNTERS NO va aca: su largo depende de la PAGINA, no del comando.
    # Ver COUNTERS_RESP_LEN y decode_counters().
    # 11, Y LA HISTORIA DE ESTE NUMERO ES LA MISMA DOS VECES.
    #
    # Fue 7, paso a 8 cuando el firmware agrego el byte de SECUENCIA (08-Ago), y es 11 desde
    # que agrego PRESENCIA y RSSI. Cada vez, un parser escrito contra el largo viejo RECHAZA
    # todas las respuestas reales del equipo — y cada vez paso.
    #
    # Medido por BLE el 09-Sep-2026 en la placa 2080388C4B30: `decode_smartring` fallaba con
    # "devuelve 8 bytes, llegaron 11". Por USB no se veia porque `_read_payload` lee
    # exactamente RESP_LEN y por lo tanto TRUNCABA, dejando 3 bytes sueltos en el buffer para
    # envenenar la lectura siguiente.
    #
    # El firmware puso los campos nuevos DESPUES de la secuencia a proposito, y declara el
    # mecanismo de compatibilidad en config_handlers.cpp: «las dos versiones se distinguen por
    # el LARGO: 8 bytes es firmware sin presencia, 11 es con». `decode_smartring` respeta eso
    # y acepta los dos.
    # 15, NO 11: el firmware responde `send_v2_response(cmd, r, 15U)` SIEMPRE
    # (config_handlers.cpp caso 0x2A) — los ultimos 4 son freq_hz. Con 11 se leian 11 de 15 y
    # los otros 4 quedaban en el buffer corriendo la consulta siguiente, ademas de dejar la
    # frecuencia ilegible. Es el mismo defecto que ya se cobro 0x41 y dos veces 0x42.
    CMD_QUERY_SMARTRING: 15,   # on(1)+ventana(2)+detecciones(4)+seq(1)+presencia(1)+rssi(2)+freq(4)
    CMD_QUERY_SMARTRING_RSSI: 7,  # umbral(2)+hist(1)+offset(1)+rssi_corregido(2)+modo(1)
    CMD_DIAGNOSTIC_RUN: 2,     # palabra de salud, uint16 LE
    CMD_SET_ATEN_FIJA_UL: 1,   # ACK/NACK
    CMD_SET_ATEN_FIJA_DL: 1,
    CMD_QUERY_ATEN_PARTES: 16,  # fija_ul + fija_dl + var_ul + var_dl, int32 LE mdB
    CMD_FOTA_STATUS: 9,        # versión y estado FOTA (9 bytes mínimo, hasta 17 bytes)
}

#: Canal -> uint8 que espera el firmware en SET_AGC.
CHANNELS = {"UL": 0, "DL": 1}

ACK = 0x00
NACK = 0xFF

#: Rango del SETPOINT de SET_GAIN (0x0A/0x0B). Es el que el firmware acota en
#: `GainManager::set_gain_mdB()` con `GAIN_MIN_mDB=0` / `GAIN_MAX_mDB=31500`, asi que
#: coincide a proposito y NO es el rango de la ganancia INFORMADA — ver GAIN_MAX_*_DB.
GAIN_MIN_DB, GAIN_MAX_DB = 0.0, 31.5

#: Maximo de ganancia POR RAMA, que es de lo que se resta la atenuacion aplicada.
#: Espejo de `GANANCIA_MAX_UL_mdB` / `GANANCIA_MAX_DL_mdB` (fw-vlad GainManager.hpp).
#:
#: Son distintos entre ramas y ninguno es 31,5: la ganancia que informa QUERY_GAIN es
#: `MAX_RAMA - aplicada`, asi que con el atenuador saturado el UL informa **-1,9 dB** y eso
#: es correcto — el equipo atenua neto. Un lector que acote la informada a cero maquilla.
GAIN_MAX_UL_DB = 29.6
GAIN_MAX_DL_DB = 32.0
#: Referencia del DAC en volts. No es dBm: el firmware no convierte potencia.
#:
#: El rango es 0.4-1.8 V, que es el que declara el driver del DAC para el pin SET del
#: detector, y desde el 06-Ago el firmware lo APLICA: SET_AGC pasa por volts_to_code()
#: y satura ahi. Antes llamaba a set_code() directo y un 3.3 llegaba tal cual al pin,
#: fuera de especificacion. Pedir algo afuera no es un error: se satura.
AGC_MIN_V, AGC_MAX_V = 0.4, 1.8
#: Atenuación del BDA4601, mismo rango que la ganancia.
ATTEN_MIN_DB, ATTEN_MAX_DB = 0.0, 31.5
#: Código del DAC: entero crudo, sin conversión de unidades.
DAC_MIN_CODE, DAC_MAX_CODE = 0, 4095

#: El DAC es de 12 bits sobre 3.3 V. El firmware calcula el código como
#: `(mV * DAC_FULL_SCALE) / DAC_VREF_MV` en enteros, así que trunca.
DAC_FULL_SCALE = 4095
DAC_VREF_MV = 3300


def channel_index(channel: str) -> int:
    """Traduce "UL"/"DL" al uint8 del firmware. Levanta ValueError si no es uno de esos."""
    try:
        return CHANNELS[channel.upper()]
    except (KeyError, AttributeError) as exc:
        raise ValueError(f"canal invalido {channel!r}: esperado 'UL' o 'DL'") from exc


def clamp_gain_db(gain_db: float) -> float:
    """Acota la ganancia al rango físico del hardware."""
    return max(GAIN_MIN_DB, min(GAIN_MAX_DB, gain_db))


def clamp_agc_volts(agc_volts: float) -> float:
    """Acota la referencia del AGC al rango del DAC."""
    return max(AGC_MIN_V, min(AGC_MAX_V, agc_volts))


# ── AGC en dB, a nivel de controlador ──────────────────────────────────────────────────────────
# El firmware fija el AGC por VOLTS (referencia del DAC, 0x16) y NO convierte potencia. El lazo
# clampa la salida a un nivel que depende de esa referencia, y esa transferencia es EMPIRICA: el
# lazo NO es el detector de nivel (0x1E lee AGC_UL/AGC_DL, la tension de control), asi que se MIDE,
# no sale del datasheet. Medido en la 208438A74B30 el 22-Sep-2026 con el analizador en la salida:
#     DL clampa en  0 dBm  con 1339 mV      UL clampa en -10 dBm  con 1125 mV
# pendiente ~25,5 mV/dB en las dos ramas (coincide con el MAX4003). Es PER-UNIDAD: si en OTRO VLAD
# 0 dBm no cae en 1339 mV, hay que RECALIBRAR ese punto. Por eso las conversiones toman la cal por
# parametro (mv_ref, dbm_ref, mv_por_db) en vez de una global: el controlador guarda una cal por
# instancia y la ajusta con set_agc_cal(). Esto de abajo es solo el DEFAULT de familia (arranque).
AGC_MV_POR_DB = 25.5
AGC_CAL_DEFAULT = {"UL": (1125.0, -10.0), "DL": (1339.0, 0.0)}   # (mV_ref, dBm_ref) por rama


def agc_dbm_to_volts(target_dbm: float, mv_ref: float, dbm_ref: float,
                     mv_por_db: float = AGC_MV_POR_DB) -> float:
    """Potencia de salida a la que se quiere que el AGC clampe -> referencia del DAC, en volts,
    con la calibracion (mv_ref, dbm_ref, mv_por_db) de ESTA unidad.

    Se acota al rango del DAC: pedir un clamp fuera de lo que el DAC alcanza satura, no falla."""
    return clamp_agc_volts((mv_ref + (target_dbm - dbm_ref) * mv_por_db) / 1000.0)


def agc_volts_to_dbm(agc_volts: float, mv_ref: float, dbm_ref: float,
                     mv_por_db: float = AGC_MV_POR_DB) -> float:
    """La inversa: referencia del DAC en volts -> potencia de salida a la que clampa, con la
    calibracion de esta unidad."""
    return dbm_ref + (agc_volts * 1000.0 - mv_ref) / mv_por_db


#: Cuenta de ADC del lazo de AGC en REPOSO (integrador en el riel, no atenua). Medido 4084..4086.
#: Por debajo, el lazo esta ACTUANDO (se vio 2525, 1247 atenuando). Margen amplio a proposito.
AGC_REPOSO_RAW = 4000


def agc_activo_de_raw(agc_raw: int) -> bool:
    """True si el lazo de AGC esta actuando (la cuenta cayo del reposo). 0xFFFF (ADC fallo) da
    False: una lectura invalida no es 'actuando'."""
    return agc_raw != 0xFFFF and agc_raw < AGC_REPOSO_RAW


def clamp_atten_db(atten_db: float) -> float:
    """Acota la atenuación al rango del BDA4601."""
    return max(ATTEN_MIN_DB, min(ATTEN_MAX_DB, atten_db))


def clamp_dac_code(code: int) -> int:
    """Acota el código del DAC a 12 bits."""
    return max(DAC_MIN_CODE, min(DAC_MAX_CODE, int(code)))


def encode_set_dac(code: int, channel: str) -> tuple[int, bytes]:
    """Arma SET_DAC_UL/DL: código crudo de 12 bits, uint16 LE.

    Es el único de los cuatro comandos de control que **no** lleva float ni
    conversión de unidades: el firmware hace `data[0] | (data[1] << 8)` y escribe
    el código directo al DAC (cmd_handlers.cpp:39-41).
    """
    idx = channel_index(channel)
    cmd = CMD_SET_DAC_UL if idx == 0 else CMD_SET_DAC_DL
    return cmd, struct.pack("<H", clamp_dac_code(code))


def encode_set_atten(atten_db: float, channel: str) -> tuple[int, bytes]:
    """Arma SET_ATTEN_UL/DL: float32 LE en dB, igual escala que la ganancia.

    Usa bytes_to_mdB() como la ganancia, así que el float va en dB — no confundir
    con SET_AGC, que usa bytes_to_mV() y va en milivolts.

    El firmware aplica esto por el BDA4601 tomando `spi3_mutex`
    (cmd_handlers.cpp:97-99), bus compartido con el LoRa Base: una escritura de
    atenuación contiende con LoRa.
    """
    idx = channel_index(channel)
    cmd = CMD_SET_ATTEN_UL if idx == 0 else CMD_SET_ATTEN_DL
    return cmd, struct.pack("<f", clamp_atten_db(atten_db))


def aten_aplicada_db(fija_db: float, variable_db: float) -> float:
    """Lo que queda EN el atenuador. Replica de `aten::aplicada_mdB()` (Atenuacion.hpp).

    Acota cada parte y despues la suma, en ese orden — no la suma primero. El firmware es la
    autoridad: ante discrepancia gana el firmware, como con volts_to_dac_code.
    """
    return clamp_atten_db(clamp_atten_db(fija_db) + clamp_atten_db(variable_db))


def encode_set_aten_fija(atten_db: float, channel: str) -> tuple[int, bytes]:
    """Arma SET_ATEN_FIJA_UL/DL (0x43/0x44): float32 LE en dB. Puesta en servicio.

    NO ACOTA, Y ES DELIBERADO. Estos dos comandos son los unicos de la familia que responden
    **NACK** ante un valor fuera de rango, en vez de recortar y contestar ACK: se midio el
    02-Sep-2026 en la unidad 208438A74B30 que pedir 40 dB dejaba 31,5 con un ACK, y el
    operador se iba convencido de que habia puesto 40.

    Acotar de este lado volveria a tapar exactamente eso, asi que un pedido fuera de rango
    levanta ValueError acá y no viaja: un NACK del equipo es indistinguible de los otros
    fallos de transporte una vez que llega al controlador.

    La asimetria con 0x06/0x07 —que SI acotan en silencio— es del firmware y esta declarada
    en atten_handlers.cpp: esos los usa produccion y cambiarles la respuesta rompe
    retrocompatibilidad.
    """
    idx = channel_index(channel)
    if not (ATTEN_MIN_DB <= atten_db <= ATTEN_MAX_DB):
        raise ValueError(
            f"atenuacion fija {atten_db} dB fuera de {ATTEN_MIN_DB}-{ATTEN_MAX_DB}: "
            f"0x43/0x44 responden NACK y no recortan"
        )
    cmd = CMD_SET_ATEN_FIJA_UL if idx == 0 else CMD_SET_ATEN_FIJA_DL
    return cmd, struct.pack("<f", float(atten_db))


def _rechazar_si_no_es_atenuacion(payload: bytes, crudos: tuple[int, ...]) -> None:
    """Levanta si los cuatro enteros no pueden ser atenuaciones de este equipo.

    ── POR QUE HACE FALTA: UNA LECTURA FABRICADA, MEDIDA EN PLACA EL 17-SEP-2026 ─────────

    Los lectores de `suite/standalone/` sincronizan buscando **un byte igual al opcode**, y el
    CDC del VLAD25 lleva el log de texto del firmware por el MISMO endpoint. Para 0x45 ese byte
    es la `'E'` ASCII, que en un log en castellano aparece todo el tiempo.

    Lo que paso: `vlad25_contrato_bench.py` sincronizo con una `'E'` de adentro del log, leyo
    los 16 bytes siguientes —texto— y los decodifico como cuatro int32. Salio:

        fija     UL 1128354.894 dB   DL 542003.017 dB
        variable UL 1432309.792 dB   DL 808795.468 dB
        aplicada UL      31.500 dB   DL      31.500 dB     <- ESTE parecia correcto

    Esos cuatro enteros son `NTAC` `ION ` ` P_U` `L=50`: el fragmento «NTACION P_UL=50» de una
    linea de log. Y `aplicada` salia bien **porque `aten_aplicada_db()` acota a 31,5**: el
    propio clamp convertia la basura en un numero creible. Un informe que dice 31,5 dB porque
    recorto texto es peor que uno que falla.

    NO ES UNA DIFERENCIA ENTRE HERRAMIENTAS, ES UNA CARRERA: `vlad25_setters_readback.py` usa
    el MISMO `consultar()` y ese mismo dia leyo bien. Leyo bien por suerte, no por ser distinto.

    ── POR QUE ACA Y NO EN EL LECTOR ────────────────────────────────────────────────────

    Arreglar la sincronizacion de un lector deja a los otros con el mismo agujero. El
    decodificador es UNO solo y lo comparten todos, y ademas es el unico que sabe que es un
    valor posible: el BDA4601 va de 0 a 31,5 dB y el firmware acota con `acotar_mdB()` antes de
    responder, asi que NUNCA puede mandar algo afuera de ese rango.
    """
    limite_mdB = int(ATTEN_MAX_DB * 1000)
    if all(0 <= v <= limite_mdB for v in crudos):
        return
    try:
        texto = payload.decode("ascii")
        pista = f" Los bytes son texto ASCII: {texto!r} — se sincronizo con el log del firmware."
    except UnicodeDecodeError:
        pista = ""
    raise ValueError(
        f"QUERY_ATEN_PARTES devolvio {crudos} mdB, y el BDA4601 solo admite 0..{limite_mdB} mdB "
        f"(0 a {ATTEN_MAX_DB} dB). Esto NO es una respuesta del equipo.{pista}"
    )


def decode_aten_partes(payload: bytes) -> dict[str, float]:
    """Interpreta los 16 bytes de QUERY_ATEN_PARTES (0x45): cuatro int32 LE en mdB.

    EL ORDEN ES fija_UL, fija_DL, variable_UL, variable_DL — agrupado por PARTE, no por
    rama (atten_handlers.cpp caso 0x45). Leerlo como (fija_ul, var_ul, fija_dl, var_dl)
    intercambia la fija del DL con la variable del UL y da numeros plausibles.

    `variable_*` es lo que puso el rotatorio o el software: 1 dB por posicion, lineal en las
    dieciseis, asi que la llave en F son 15 dB. Es la unica lectura que responde «por que el
    equipo atenua mas de lo que le pedi».
    """
    if len(payload) != 16:
        raise ValueError(f"QUERY_ATEN_PARTES devuelve 16 bytes, llegaron {len(payload)}")
    fija_ul, fija_dl, var_ul, var_dl = struct.unpack("<iiii", payload)
    _rechazar_si_no_es_atenuacion(payload, (fija_ul, fija_dl, var_ul, var_dl))
    fija_ul_db, fija_dl_db = fija_ul / 1000.0, fija_dl / 1000.0
    var_ul_db, var_dl_db = var_ul / 1000.0, var_dl / 1000.0
    return {
        "fija_ul_db": fija_ul_db,
        "fija_dl_db": fija_dl_db,
        "variable_ul_db": var_ul_db,
        "variable_dl_db": var_dl_db,
        "aplicada_ul_db": aten_aplicada_db(fija_ul_db, var_ul_db),
        "aplicada_dl_db": aten_aplicada_db(fija_dl_db, var_dl_db),
    }


def encode_set_gain(gain_db: float, channel: str) -> tuple[int, bytes]:
    """Arma SET_GAIN_UL/DL. El float32 va en dB porque bytes_to_mdB escala por 1000."""
    idx = channel_index(channel)
    cmd = CMD_SET_GAIN_UL if idx == 0 else CMD_SET_GAIN_DL
    return cmd, struct.pack("<f", clamp_gain_db(gain_db))


def encode_set_agc(agc_volts: float, channel: str) -> tuple[int, bytes]:
    """Arma SET_AGC.

    El float32 va en MILIVOLTS: bytes_to_mV() no escala. El firmware exige
    exactamente 5 bytes de data ([canal u8][float32]) o responde NACK.
    """
    idx = channel_index(channel)
    volts = clamp_agc_volts(agc_volts)
    return CMD_SET_AGC, bytes([idx]) + struct.pack("<f", volts * 1000.0)


#: Tolerancia del invariante de 0x0C. El firmware trabaja en mdB enteros y esta cuenta no
#: pierde nada, asi que medio paso del atenuador (0,25 dB) ya es holgura de sobra: un desvio
#: real no es de decimas, es de la atenuacion entera que se dejo de contar.
GAIN_COHERENCE_TOL_DB = 0.25


def decode_gain(payload: bytes) -> dict[str, float]:
    """Interpreta los 20 bytes de QUERY_GAIN. Levanta struct.error si el largo no da.

    Y COMPRUEBA LO QUE LA PROPIA TRAMA AFIRMA. Los bytes 0-7 llevan la ganancia y los 12-19
    la atenuacion aplicada de la MISMA rama, asi que la respuesta promete, por su forma:

        ganancia + atenuacion_aplicada == maximo_de_rama

    Es el `static_assert` de `aten::ganancia_mdB()` en fw-vlad (Atenuacion.hpp), y hasta el
    09-Sep-2026 no se cumplia: `get_gain_mdB()` restaba solo la atenuacion FIJA y se dejaba
    afuera la variable —la del switch rotatorio—. Con la llave del banco en F, que atenua
    15 dB, el equipo informaba 29,6 dB de ganancia mientras hacia 14,6. **La contradiccion
    viajaba en la misma trama y ningun lector la miraba**, ni de este lado ni del otro.

    Se informa, no se levanta: una corrida de calibracion tiene que poder seguir y decidir.
    `gain_*_coherente` en False significa que el espejo de estado del firmware se separo de
    lo aplicado —ya paso el 08-Sep, con 0x45 informando 5,7 dB y 0x0C 14,0— y ahi la ganancia
    informada no describe al equipo.
    """
    gain_ul, gain_dl, dac_ul, dac_dl, att_ul, att_dl = struct.unpack("<iiHHii", payload)
    gain_ul_db, gain_dl_db = gain_ul / 1000.0, gain_dl / 1000.0
    atten_ul_db, atten_dl_db = att_ul / 1000.0, att_dl / 1000.0
    resid_ul = (gain_ul_db + atten_ul_db) - GAIN_MAX_UL_DB
    resid_dl = (gain_dl_db + atten_dl_db) - GAIN_MAX_DL_DB
    return {
        "gain_ul_db": gain_ul_db,
        "gain_dl_db": gain_dl_db,
        "dac_ul_code": dac_ul,
        "dac_dl_code": dac_dl,
        "atten_ul_db": atten_ul_db,
        "atten_dl_db": atten_dl_db,
        #: `ganancia + aplicada - MAX_RAMA`, que tiene que dar 0. Viaja el residuo y no solo
        #: el booleano: su VALOR dice cuanta atenuacion no se conto, y con la llave en F son
        #: exactamente los 15 dB del rotatorio.
        "coherencia_ul_db": resid_ul,
        "coherencia_dl_db": resid_dl,
        "gain_ul_coherente": abs(resid_ul) <= GAIN_COHERENCE_TOL_DB,
        "gain_dl_coherente": abs(resid_dl) <= GAIN_COHERENCE_TOL_DB,
    }


def decode_agc(payload: bytes) -> dict[str, float]:
    """Interpreta los 8 bytes de QUERY_AGC. Los mV son derivados del código del DAC."""
    mv_ul, mv_dl = struct.unpack("<II", payload)
    return {"agc_ul_v": mv_ul / 1000.0, "agc_dl_v": mv_dl / 1000.0}


def decode_agc_sense(payload: bytes) -> dict[str, int]:
    """Interpreta los 4 bytes de QUERY_AGC_SENSE: cuentas CRUDAS del ADC.

    Son la tensión de control del lazo (`AGC_UL` en PB2, `AGC_DL` en PC5), **no** el
    nivel de RF detectado: el lazo es analógico y esto es lo que su integrador le manda
    al amplificador. Se devuelven crudas a propósito — el umbral y la polaridad se
    miden por placa, y las dos ramas tienen polaridad de reposo opuesta (MAX4003 en UL
    contra MAX4001 en DL), así que copiar umbrales entre ramas no sirve.

    `0xFFFF` significa que la conversión del ADC falló, y no se traduce a 0: un cero es
    una lectura legítima y confundirlos es el error que esta serie viene arrastrando.
    """
    ul_raw, dl_raw = struct.unpack("<HH", payload)
    return {
        "agc_ul_raw": ul_raw,
        "agc_dl_raw": dl_raw,
        "agc_ul_failed": ul_raw == 0xFFFF,
        "agc_dl_failed": dl_raw == 0xFFFF,
        "agc_ul_mv": None if ul_raw == 0xFFFF else (ul_raw * DAC_VREF_MV) // DAC_FULL_SCALE,
        "agc_dl_mv": None if dl_raw == 0xFFFF else (dl_raw * DAC_VREF_MV) // DAC_FULL_SCALE,
    }


def decode_fota_status(payload: bytes) -> dict[str, Any]:
    """Interpreta la respuesta de FOTA_STATUS (0x1C).

    Payload: [app_version u32 LE][app_size u32 LE][pending u8][watermark u32 LE (opt)][declared u32 LE (opt)]
    Acepta tanto la forma de 9 bytes (firmware previo) como la de 17 bytes (firmware actual).
    """
    if len(payload) < 9:
        raise ValueError(f"FOTA_STATUS devuelve al menos 9 bytes, llegaron {len(payload)}")
    app_version, app_size, pending_byte = struct.unpack_from("<IIB", payload, 0)
    b0 = (app_version >> 24) & 0xFF
    b1 = (app_version >> 16) & 0xFF
    b2 = (app_version >> 8) & 0xFF
    b3 = app_version & 0xFF
    version_str = f"{b0}.{b1}.{b2}.{b3}" if app_version > 0 else "0.0.0.0"
    watermark = struct.unpack_from("<I", payload, 9)[0] if len(payload) >= 13 else None
    declared = struct.unpack_from("<I", payload, 13)[0] if len(payload) >= 17 else None
    return {
        "app_version": app_version,
        "version_str": version_str,
        "app_size": app_size,
        "pending": bool(pending_byte != 0),
        "watermark": watermark,
        "declared": declared,
    }


def volts_to_dac_code(volts: float) -> int:
    """Réplica del cálculo del firmware. Sirve para predecir el `dac_*_code` que va a
    devolver QUERY_GAIN, sin adivinar.

    Sigue a `DacChannel::volts_to_code()` de `AgcDac.hpp`: clampea al rango del
    detector y **redondea** con `+ 0.5`.

    Ojo, esto CAMBIÓ el 06-Ago. La versión anterior replicaba la fórmula vieja del
    firmware —`(mV * 4095) // 3300`, entera y truncada, que daba 2047 para 1.65 V— y
    quedó desactualizada cuando SET_AGC pasó a usar `volts_to_code()`. Ante una
    discrepancia entre este archivo y el firmware, **gana el firmware**.
    """
    v = clamp_agc_volts(volts)
    code = int((v / (DAC_VREF_MV / 1000.0)) * DAC_FULL_SCALE + 0.5)
    return min(code, DAC_FULL_SCALE)


def dac_code_to_volts(code: int) -> float:
    """Vuelta del DAC a volts, como la calcula QUERY_AGC: `código * 3300 / 4095`."""
    return ((code * DAC_VREF_MV) // DAC_FULL_SCALE) / 1000.0


# ---------------------------------------------------------------------------
# SmartRing
# ---------------------------------------------------------------------------


def clamp_smartring_ventana(ms: int) -> int:
    """Acota la ventana de escucha igual que el firmware (SmartRing.hpp WINDOW_MIN/MAX_MS).

    Se replica para poder avisar al usuario antes de mandar, no para decidir: ante
    discrepancia gana el firmware, como con volts_to_dac_code. Verificado en la placa:
    pedir 500 ms deja 2500.
    """
    return max(SMARTRING_VENTANA_MIN_MS, min(SMARTRING_VENTANA_MAX_MS, int(ms)))


def clamp_smartring_freq_hz(hz: int) -> int:
    """142,5–144,0 MHz. Mismo acotado que `smartring::clamp_freq_hz()`."""
    return max(SMARTRING_FREQ_MIN_HZ, min(SMARTRING_FREQ_MAX_HZ, int(hz)))


def clamp_smartring_umbral_dBm(dBm: int) -> int:
    """-125..0 dBm. Mismo acotado que `smartring::clamp_umbral_dBm()`."""
    return max(SMARTRING_UMBRAL_MIN_DBM, min(SMARTRING_UMBRAL_MAX_DBM, int(dBm)))


def clamp_smartring_hist_dB(dB: int) -> int:
    """0..25 dB. Mismo acotado que `smartring::clamp_hist_dB()`."""
    return max(0, min(SMARTRING_HIST_MAX_DB, int(dB)))


def clamp_smartring_offset_dB(dB: int) -> int:
    """0..50 dB. Mismo acotado que `smartring::clamp_offset_dB()`."""
    return max(0, min(SMARTRING_OFFSET_MAX_DB, int(dB)))


def encode_set_smartring(enabled: bool,
                         ventana_ms: int = SMARTRING_VENTANA_DEFAULT_MS,
                         freq_hz: int | None = None,
                         umbral_dBm: int | None = None,
                         hist_dB: int | None = None,
                         offset_dB: int | None = None) -> tuple[int, bytes]:
    """Arma SET_SMARTRING. Los tres largos son ACUMULATIVOS, como en el firmware:

        3  bytes: [u8 on][u16 LE ventana_ms]
        7  bytes: + [u32 LE freq_hz]
        11 bytes: + [i16 LE umbral_dBm][u8 hist_dB][u8 offset_dB]

    ACUMULATIVOS quiere decir que no se puede mandar el umbral sin la frecuencia: no hay una
    forma de 7 bytes que lleve los parametros y saltee la banda. Por eso pedir cualquiera de
    los tres sin `freq_hz` usa la del sistema (143 MHz) en vez de fallar — es el mismo valor
    que el equipo ya tiene de fabrica, asi que la forma larga no lo mueve por accidente.

    Y el clamp se replica ACA porque el firmware ACOTA en vez de rechazar: un pedido fuera de
    rango contesta ACK igual. La forma de 11 bytes ya se cobro un defecto de este tipo del
    lado del firmware —con `== 7U` en vez de `>= 7U` la frecuencia se salteaba en silencio y
    el ACK no lo delataba—, asi que la relectura con 0x2A/0x51 no es opcional.
    """
    v = clamp_smartring_ventana(ventana_ms)
    cuerpo = bytes([1 if enabled else 0]) + struct.pack("<H", v)

    quiere_params = any(x is not None for x in (umbral_dBm, hist_dB, offset_dB))
    if freq_hz is None and not quiere_params:
        return CMD_SET_SMARTRING, cuerpo

    cuerpo += struct.pack("<I", clamp_smartring_freq_hz(
        SMARTRING_FREQ_DEFAULT_HZ if freq_hz is None else freq_hz))
    if not quiere_params:
        return CMD_SET_SMARTRING, cuerpo

    cuerpo += struct.pack(
        "<hBB",
        clamp_smartring_umbral_dBm(SMARTRING_UMBRAL_DEFAULT_DBM
                                   if umbral_dBm is None else umbral_dBm),
        clamp_smartring_hist_dB(SMARTRING_HIST_DEFAULT_DB
                                if hist_dB is None else hist_dB),
        clamp_smartring_offset_dB(SMARTRING_OFFSET_DEFAULT_DB
                                  if offset_dB is None else offset_dB),
    )
    return CMD_SET_SMARTRING, cuerpo


def decode_smartring(payload: bytes) -> dict[str, Any]:
    """Desempaca QUERY_SMARTRING.

    Las DETECCIONES son lo unico que distingue "escuchando y oyendo el anillo" de
    "escuchando y sordo": el enable por si solo no prueba que se reciba nada.

    Y se llaman detecciones y no `tx_count`, que es como decia este archivo: es lenguaje
    del TRANSMISOR y este equipo DETECTA. El firmware corrigio el mismo residuo en sus
    comentarios y aviso que estaba en toda la familia; aca seguia.

    Y el `seq` viaja porque el contador no alcanza: por BLE la respuesta no se empuja, queda
    disponible para que el central la lea, y llega con un atraso de un par de ciclos. Dos
    consultas consecutivas devolvian la MISMA trama —mismo contador— y el test concluia que
    SmartRing no transmitia cuando por SWD se lo veia transmitiendo a 2 por segundo. El eco
    del CMD distingue comandos distintos; el seq distingue dos respuestas al mismo comando.
    """
    if len(payload) not in (8, 11, 15):
        raise ValueError(
            f"QUERY_SMARTRING devuelve 8, 11 o 15 bytes, llegaron {len(payload)}"
        )
    on, ventana, detecciones, seq = struct.unpack_from("<BHIB", payload, 0)
    d = {"enabled": bool(on), "ventana_ms": ventana,
         "detecciones": detecciones, "seq": seq}

    # EL LARGO ES EL DISCRIMINADOR DE VERSION, y lo dice el firmware:
    # «las dos versiones se distinguen por el LARGO: 8 bytes es firmware sin presencia, 11
    # es con» (config_handlers.cpp caso 0x2A). Se respeta en vez de exigir uno de los dos.
    #
    # `presencia` es la unica de las tres que habla del PRESENTE: el contador dice cuantas
    # veces oyo el tono desde siempre, no si lo esta oyendo ahora, y para eso habria que
    # leerlo dos veces y restar. `presencia` baja sola tras tres ventanas sin tono.
    if len(payload) >= 11:
        d["presencia"] = bool(payload[8])
        d["rssi_dbm"] = struct.unpack_from("<h", payload, 9)[0]
    else:
        # None y no False: «este firmware no lo informa» no es «el anillo no esta».
        d["presencia"] = None
        d["rssi_dbm"] = None

    # LA FRECUENCIA, que es la tercera version y faltaba entera. Sin ella no se puede leer en
    # que banda esta escuchando el detector, que es justo uno de los parametros que el sistema
    # SmartRing define como configurable.
    d["freq_hz"] = struct.unpack_from("<I", payload, 11)[0] if len(payload) == 15 else None
    return d


def decode_smartring_rssi(payload: bytes) -> dict[str, Any]:
    """Desempaca QUERY_SMARTRING_RSSI (0x51): los insumos de la decision Y la decision.

    [0..1] umbral i16 LE dBm · [2] histeresis u8 dB · [3] offset u8 dB
    [4..5] rssi_corregido i16 LE dBm (= crudo + offset) · [6] modo sugerido

    ES EL CORREGIDO Y NO HAY CRUDO ACA: el crudo viaja en 0x2A. Comparar el crudo contra el
    umbral da un margen equivocado por el valor entero del offset —17 dB de fabrica—, y las
    dos lecturas se ven igual de plausibles.

    POR QUE VIAJA EL MODO Y NO SOLO LOS INSUMOS: la histeresis es ASIMETRICA segun el modo
    vigente —Schmitt—, asi que recalcular la decision de este lado obliga a replicar esa
    regla. Dos implementaciones de la misma regla divergen y nada lo reporta; el firmware
    publica el resultado justamente para que el banco NO tenga que recalcularlo.

    Y por eso `modo_sugerido` se llama SUGERIDO: es lo que el anillo pediria. La llave manual
    es una COMPUERTA —en CONVENCIONAL el mensaje se ignora por completo—, asi que este byte
    no es necesariamente el modo en el que el equipo esta. Ese sale de QUERY_STATUS.
    """
    if len(payload) != 7:
        raise ValueError(
            f"QUERY_SMARTRING_RSSI devuelve 7 bytes, llegaron {len(payload)}"
        )
    umbral, hist, offset, rssi, modo = struct.unpack_from("<hBBhB", payload, 0)
    return {
        "umbral_dBm": umbral,
        "hist_dB": hist,
        "offset_dB": offset,
        # `None` y no -164: el piso es «nunca oi nada», no una medicion. Un consumidor que lo
        # promedie con lecturas reales se lleva el piso al resultado.
        "rssi_corregido_dBm": None if rssi <= SMARTRING_RSSI_PISO_DBM else rssi,
        # NO HAY «CRUDO» EN 0x51, Y ESTE ARCHIVO LLEGO A DECIR QUE SI.
        #
        # El firmware manda un solo RSSI aca, y es el CORREGIDO —`smartring_rssi_corregido_dBm()`
        # en config_handlers.cpp caso 0x51—. El crudo viaja en 0x2A, que es otro comando. Exponer
        # el mismo numero bajo dos nombres, uno de ellos falso, hizo leer mal una corrida de banco
        # el 17-Sep-2026: se imprimio «crudo -76» cuando era el corregido, y con eso la cuenta del
        # margen daba 17 dB de mas.
        #
        # Queda el BOOLEANO, que es lo que el segundo campo tenia que contestar: distinguir «el
        # piso» de «no se leyo», sin inventar una magnitud que no existe.
        "rssi_es_piso": rssi <= SMARTRING_RSSI_PISO_DBM,
        "modo_sugerido": SMARTRING_MODO.get(modo, f"DESCONOCIDO({modo})"),
    }


# ── Palabra de salud ──────────────────────────────────────────────────────────

#: Bits de la palabra de salud. Espejo de ``health::Bit`` en fw-vlad ``Core/Inc/Health.hpp``.
HEALTH_INA_LINE = 1 << 0
HEALTH_INA_DEVICE = 1 << 1
HEALTH_EEPROM_OK = 1 << 2
HEALTH_LORA_BASE = 1 << 3
HEALTH_LORA_TUNNEL = 1 << 4
HEALTH_AGC_UL_ACTING = 1 << 5
HEALTH_AGC_DL_ACTING = 1 << 6
HEALTH_CAL_VALID = 1 << 10
HEALTH_BLE_ADVERTISING = 1 << 11
HEALTH_WIFI_AP_UP = 1 << 12
HEALTH_FOTA_PENDING = 1 << 13
HEALTH_ABNORMAL_RESET = 1 << 14
#: El PIN de alerta del INA237 de LINEA (PA7), no su registro. Es informacion distinta de la
#: que trae QUERY_ALERTS (0x36): esa se lee por I2C y depende de que el bus funcione; este bit
#: sale del pin, que el chip maneja en hardware, asi que sigue siendo cierto con el I2C caido
#: — que es justo cuando mas hace falta.
#:
#: FALTABA ENTERO DE ESTE ARCHIVO, y `health_is_ok()` no lo miraba: una placa con la alarma de
#: sobrecorriente de linea puesta se leia **sana**. El firmware tiene un static_assert que dice
#: «el bit de la alerta es el 15; si se mueve, el decodificador de la app miente» (Health.hpp)
#: y el decodificador de la app no lo leia siquiera.
HEALTH_CURR_LINE_ALERT = 1 << 15

#: Los cinco obligatorios, que son los mismos cinco del contrato de cable historico.
HEALTH_REQUIRED = 0x001F

#: Las alarmas: cualquiera en 1 vuelve no-sano al equipo. Espejo de `health::ALARMS`
#: (fw-vlad Health.hpp). Se declara como constante y no en linea dentro de health_is_ok()
#: para que agregar una alarma sea un solo cambio, del lado que sea.
HEALTH_ALARMS = HEALTH_FOTA_PENDING | HEALTH_ABNORMAL_RESET | HEALTH_CURR_LINE_ALERT

HEALTH_VIN_ZONE_SHIFT = 7
HEALTH_VIN_ZONE_MASK = 0x07 << HEALTH_VIN_ZONE_SHIFT


def decode_health_word(payload: bytes) -> int:
    """Desempaca DIAGNOSTIC_RUN (0x35): uint16 little-endian.

    Cada bit se prueba de verdad en el firmware —``RegVersion == 0x12`` para las radios, el
    magic leido para la EEPROM— y no derivandolo de un puntero no nulo, que es lo que hacia
    antes del 09-Ago y mantuvo un bit en 1 mientras la radio no transmitia.
    """
    if len(payload) != 2:
        raise ValueError(f"DIAGNOSTIC_RUN devuelve 2 bytes, llegaron {len(payload)}")
    return int(payload[0] | (payload[1] << 8))


def health_vin_zone(word: int) -> int:
    """Zona de tension de entrada: 0=muy baja, 1=12 V, 2=24 V, 3=48 V, 4=muy alta."""
    return (word & HEALTH_VIN_ZONE_MASK) >> HEALTH_VIN_ZONE_SHIFT


def health_is_ok(word: int) -> bool:
    """Sano = los cinco obligatorios, ninguna alarma, y el riel en una barra nominal.

    Espejo de `health::is_healthy()` (fw-vlad Health.hpp). Las alarmas salen de
    HEALTH_ALARMS, que incluye la del pin de sobrecorriente de linea — omitida hasta el
    09-Sep-2026, cuando esta funcion devolvia True sobre una placa en alarma.
    """
    if (word & HEALTH_REQUIRED) != HEALTH_REQUIRED:
        return False
    if word & HEALTH_ALARMS:
        return False
    return 1 <= health_vin_zone(word) <= 3


# ══════════════════════════════════════════════════════════════════════════════════════
#  QUERY_COUNTERS (0x3B) — paginado, y cada pagina mide distinto
# ══════════════════════════════════════════════════════════════════════════════════════
#
# Formato de la respuesta: [pagina][total_paginas][valores uint16 LE...]. El firmware satura
# cada contador en 0xFFFF en vez de truncarlo, porque truncar un uint32 por modulo daria la
# respuesta FAVORABLE justo en el caso malo (65536 fallas se leerian como 0).
#
# LOS LARGOS SON PARTE DEL CONTRATO Y ESTAN VERIFICADOS EN EL FIRMWARE: `verificar.sh` exige
# que docs/TEST_PLAN.md declare estos mismos cuatro numeros, y falla si el fuente y el
# documento se separan. Si una pagina crece, este diccionario tambien tiene que crecer o se
# leera una ventana corta y los campos del final caeran afuera SIN ERROR.
COUNTERS_RESP_LEN = {0: 36, 1: 15, 2: 48, 3: 40}

# Los campos de la pagina 2, EN ORDEN. Nombres tal cual el firmware, para que un hallazgo se
# pueda buscar con grep en las dos puntas sin traducir.
COUNTERS_PAGINA2 = (
    "rx_intentos", "rx_timeouts", "rx_paquetes", "txfail_n", "resp_tx_fail",
    "hook_fail", "resp_v1_no_cabe", "spi_shared_lock_timeouts", "cad_timeout",
    "dst_ajeno", "resp_ok", "resp_descartada", "rx_cede_tono", "rx_crc_error",
    "led_frames", "led_evaluado", "led_dma_error", "dac_no_sigue",
    # Agregado el 17-Sep-2026 AL FINAL, no junto a su gemelo `rx_cede_tono`, justamente
    # porque esta respuesta se decodifica por posicion: insertarlo en el medio habria
    # corrido seis campos sin que nada avisara.
    "ring_cede_tono",
    # La puesta en marcha del modem, por radio (17-Sep-2026). Si `*_sin_aplicar` crece, esa radio
    # habla con una configuracion que no es la del contrato y el enlace no engancha.
    "base_config_reintentos", "base_config_sin_aplicar",
    "tunel_config_reintentos", "tunel_config_sin_aplicar",
)


def decode_telemetry_frame(payload: bytes) -> dict:
    """La trama v5 por demanda (0x3C), sin esperar el periodo ni depender de la radio.

    POR QUE ESTA Y NO LA LEGACY 0x11. En la legacy, `level_dl_raw` sale de
    `panel::cuenta_representativa()`, que devuelve el BORDE INFERIOR DE LA ZONA cuando la cuenta
    todavia no pertenece a la zona antirrebotada — y un barrido de calibracion es justo la
    condicion donde la zona va cambiando en cada paso. Esta trama entrega lo que midio el ADC.

    Offsets de fw-vlad/vlad25_vhf/Core/Inc/Telemetry.hpp::encode.
    """
    if payload[0] != 0x54:
        raise ValueError(f"magic {payload[0]:#04x}, se esperaba 0x54")
    # EL PAR (version, largo) SE VALIDA JUNTO, igual que en el firmware y que en el decoder de
    # monitor-serial. Validar cada mitad por su lado deja pasar una v6 truncada al largo de v5:
    # el prefijo decodifica bien y los campos nuevos salen de donde estaba el CRC.
    version = payload[1]
    esperado = TELEMETRY_FRAME_LEN_POR_VERSION.get(version)
    if esperado is None:
        raise ValueError(f"0x3C con VERSION {version} desconocida; se conocen "
                         f"{sorted(TELEMETRY_FRAME_LEN_POR_VERSION)}")
    if len(payload) < esperado:
        raise ValueError(f"0x3C v{version} son {esperado} bytes, llegaron {len(payload)}")
    payload = payload[:esperado]
    u16 = lambda i: int.from_bytes(payload[i:i + 2], "little")
    datos = {
        "version":      payload[1],
        # bit 0 = REVERSE (0 = CONVENCIONAL). El firmware lo arma leyendo el PIN, no la
        # variable: es el efecto, y es lo que conmuta los cuatro SPDT del camino de RF.
        "modo_flags":   payload[28],
        "es_reverse":   bool(payload[28] & 0x01),
        "agc_ul_raw":   u16(19),
        "agc_dl_raw":   u16(21),
        "level_dl_raw": u16(23),
        "level_ul_raw": u16(25),
        "atten_ul_db":  u16(15) / 100.0,
        "atten_dl_db":  u16(17) / 100.0,
        "gain_ul_db":   int.from_bytes(payload[44:46], "little", signed=True) / 100.0,
        "gain_dl_db":   int.from_bytes(payload[46:48], "little", signed=True) / 100.0,
    }
    if version >= 6:
        # v6. OJO: NO van «al final» — ocupan lo que en la v5 era el CRC, que se corre a 57-58.
        # El apartamiento es cuanto se aparto el lazo de AGC de su reposo predicho, en cuentas;
        # 0xFFFF dice «no interpretable» y NO es lo mismo que 0, que significa «medido, en reposo».
        ap_ul, ap_dl = u16(50), u16(52)
        NO_INTERP = 0xFFFF
        datos["agc_ul_apartamiento"] = None if ap_ul == NO_INTERP else ap_ul
        datos["agc_dl_apartamiento"] = None if ap_dl == NO_INTERP else ap_dl
        # Salida en dBm ENTEROS, ya convertida por el equipo con el trim de ESTA unidad.
        # -128 = no medible; 0 dBm es el techo de operacion del DL, o sea una medicion normal.
        sal_ul = int.from_bytes(payload[54:55], "little", signed=True)
        sal_dl = int.from_bytes(payload[55:56], "little", signed=True)
        datos["salida_ul_dbm"] = None if sal_ul == -128 else sal_ul
        datos["salida_dl_dbm"] = None if sal_dl == -128 else sal_dl
        datos["nivel_flags"] = payload[56]
        datos["nivel_calibrado"] = bool(payload[56] & 0x01)
    return datos


def decode_telemetry(payload: bytes) -> dict:
    """QUERY_TELEMETRY (0x2F): si el uplink esta prendido, y cuantas veces salio.

    `tx_count` es el UNICO dato que separa «el equipo no transmite» de «transmite y se pierde en
    el camino». Sin el, un servidor sin telemetria se ve igual en los dos casos, y el arreglo es
    distinto: uno esta en el firmware y el otro en el puente del gateway.
    """
    if len(payload) != TELEMETRY_RESP_LEN:
        raise ValueError(f"0x2F devuelve {TELEMETRY_RESP_LEN} bytes, llegaron {len(payload)}")
    u32 = lambda i: int.from_bytes(payload[i:i + 4], "little")
    return {
        "on":          payload[0] != 0,
        "periodo_s":   int.from_bytes(payload[1:3], "little"),
        "tx_count":    u32(3),
        "busy_giveup": u32(7),
        "fail_count":  u32(11),
        "seq":         payload[15],
    }


def decode_counters(pagina: int, payload: bytes) -> dict:
    """Decodifica una pagina de QUERY_COUNTERS. Lanza ValueError si no cuadra.

    Comprueba que la pagina que el equipo DICE haber contestado sea la pedida: el firmware
    pone en cero el buffer y contesta ceros ante una pagina inexistente, asi que sin este
    control un error del cliente se leeria como «todos los contadores en cero», que es
    exactamente la lectura tranquilizadora.
    """
    esperado = COUNTERS_RESP_LEN.get(pagina)
    if esperado is None:
        raise ValueError(f"pagina {pagina} no existe; hay {len(COUNTERS_RESP_LEN)}")
    if len(payload) != esperado:
        raise ValueError(
            f"pagina {pagina}: se esperaban {esperado} B y llegaron {len(payload)}"
        )
    if payload[0] != pagina:
        raise ValueError(
            f"el equipo contesto la pagina {payload[0]} y se pidio la {pagina}"
        )
    valores = list(struct.unpack_from(f"<{(esperado - 2) // 2}H", payload, 2))
    out: dict[str, Any] = {"pagina": payload[0], "total_paginas": payload[1]}
    if pagina == 2:
        if len(valores) != len(COUNTERS_PAGINA2):
            raise ValueError(
                f"la pagina 2 trae {len(valores)} contadores y este decodificador conoce "
                f"{len(COUNTERS_PAGINA2)}: el firmware agrego campos y falta actualizarlo"
            )
        out.update(dict(zip(COUNTERS_PAGINA2, valores)))
    else:
        out["valores"] = valores
    return out


def encode_tono(on: bool, modulo: int) -> bytes:
    """TONO_CONTINUO (0x3F): [on][modulo], con 0=BASE y 1=TUNEL.

    EL MODULO ES POR PUERTO, NO POR ROL, y confundirlos no da error: hasta el 16-Sep-2026 el
    rol de gateway vivia en la radio BASE, asi que «modulo 0» y «la radio del diagnostico»
    eran lo mismo por coincidencia. Con los roles dados vuelta, el 0 es SmartRing.
    """
    if modulo not in (0, 1):
        raise ValueError(f"modulo {modulo}: solo 0 (BASE) o 1 (TUNEL)")
    return bytes([1 if on else 0, modulo])


# ══════════════════════════════════════════════════════════════════════════════════════
#  QUERY_RADIO_REGS (0x54) — los registros del silicio del SX1278
# ══════════════════════════════════════════════════════════════════════════════════════
#
# Existe porque todo lo demás que el firmware informa de sus radios sale de VARIABLES suyas:
# `get_tx_freq()` devuelve `m_tx_freq`, o sea lo que se QUISO escribir. Una escritura por SPI
# que no pegara dejaría los getters informando la intención y nada lo reportaría.
#
# Solo lectura. No hay comando de escritura del lado del firmware, a propósito.

RADIO_BASE = 0
RADIO_TUNEL = 1

#: Los registros que forman el CONTRATO con el gateway, con el valor que tienen que tener.
#: Salen de fw-vlad/vlad25_vhf/Core/Inc/{Lora.hpp,LoraContrato.hpp} y de
#: fw-gateway2lora/firmware/shared/drivers/Lora.{hpp,cpp} — las DOS puntas, leídas el
#: 17-Sep-2026. Un valor acá que no esté en las dos fuentes no es un contrato, es una opinión.
SX1278_REG = {
    0x01: "RegOpMode",
    0x06: "RegFrfMsb", 0x07: "RegFrfMid", 0x08: "RegFrfLsb",
    0x09: "RegPaConfig",
    0x0C: "RegLna",
    0x1D: "RegModemConfig1", 0x1E: "RegModemConfig2", 0x1F: "RegSymbTimeoutLsb",
    0x20: "RegPreambleMsb", 0x21: "RegPreambleLsb",
    0x22: "RegPayloadLength",
    0x24: "RegHopPeriod",
    0x26: "RegModemConfig3",
    0x31: "RegDetectOptimize",
    0x37: "RegDetectionThreshold",
    0x39: "RegSyncWord",
    0x42: "RegVersion",
}

#: Lo que las DOS puntas acuerdan. El comentario dice de dónde sale cada uno.
CONTRATO_SX1278 = {
    0x39: (0x12, "sync word — LTEL_COMPATIBLE_SYNC_WORD en el gateway, 0x12 en set_defaults()"),
    0x20: (0x00, "preámbulo MSB — PREAMBLE_LENGTH_MSB"),
    0x21: (12,   "preámbulo LSB — LTEL_COMPATIBLE_PREAMBLE_LENGTH_LSB"),
    0x24: (0x07, "hop period — LTEL_COMPATIBLE_HOPS_PERIOD"),
    0x26: (0x0C, "ModemConfig3 — el contrato de mina, LDRO incluido; lo vigila verificar.sh"),
    0x42: (0x12, "RegVersion del SX1278: si no es 0x12, el SPI no está hablando con la radio"),
}

#: La frecuencia se compara aparte porque son tres registros y hay que reconstruirla.
SX1278_FXOSC = 32_000_000


def frf_a_hz(msb: int, mid: int, lsb: int) -> int:
    """Reconstruye la frecuencia PROGRAMADA EN EL CHIP desde RegFrf.

    Es la inversa exacta de `Sx1278_tpl::set_frequency()`: `frf = (freq << 19) / FXOSC`.
    """
    frf = (msb << 16) | (mid << 8) | lsb
    return (frf * SX1278_FXOSC) >> 19


def decode_radio_regs(payload: bytes) -> dict:
    """Decodifica QUERY_RADIO_REGS. Lanza ValueError si no cuadra.

    Comprueba que el módulo y el registro inicial que el equipo DICE haber leído sean los
    pedidos: el firmware contesta NACK (1 byte) ante un pedido inválido, y sin este control
    un NACK se leería como una respuesta corta cualquiera.
    """
    if len(payload) < 4:
        raise ValueError(f"respuesta de {len(payload)} B: el firmware contestó NACK o se truncó")
    modulo, desde, cuanto = payload[0], payload[1], payload[2]
    if len(payload) != 3 + cuanto:
        raise ValueError(f"dice {cuanto} registros y trae {len(payload) - 3}")
    return {
        "modulo": modulo,
        "desde": desde,
        "regs": {desde + k: payload[3 + k] for k in range(cuanto)},
    }


def comparar_contra_contrato(regs: dict, freq_esperada_hz: int | None = None) -> list[str]:
    """Las desviaciones del contrato, como texto. Lista vacía = el silicio acuerda.

    NO devuelve un booleano a propósito: «no acuerda» sin decir en qué registro manda a
    releer todo de nuevo, que es lo que este comando existe para evitar.
    """
    fallas = []
    for reg, (esperado, motivo) in CONTRATO_SX1278.items():
        if reg not in regs:
            continue
        if regs[reg] != esperado:
            fallas.append(
                f"{SX1278_REG.get(reg, hex(reg))} (0x{reg:02X}) = 0x{regs[reg]:02X}, "
                f"se esperaba 0x{esperado:02X} — {motivo}"
            )
    if freq_esperada_hz is not None and all(r in regs for r in (0x06, 0x07, 0x08)):
        hz = frf_a_hz(regs[0x06], regs[0x07], regs[0x08])
        # 61 Hz es un paso de RegFrf con FXOSC de 32 MHz: exigir igualdad exacta haría
        # fallar por el redondeo del propio encoder del firmware.
        if abs(hz - freq_esperada_hz) > 200:
            fallas.append(
                f"RegFrf = {hz:,} Hz y se esperaba {freq_esperada_hz:,} Hz "
                f"(diferencia {hz - freq_esperada_hz:+,} Hz)"
            )
    return fallas

import re

from legacy_bench_support.amplifier import Amplifier
from legacy_bench_support.serial_controller import SerialController

ST_VCP_VID = 0x0483
ST_VCP_PID = 0x5740


class UladAmplifier(Amplifier):
    """Driver del amplificador ULAD por UART (ASCII, 115200 8N1).

    Protocolo validado contra una unidad real (fw-ulad v2.2.13+) y contra el fuente de
    VLAD DAC Tools. Ganancia y AGC van en CRUDO 0-4095; no existe comando en porcentaje.

    El firmware no valida los rangos de lo que se le escribe, así que la validación de este
    driver no es defensa en profundidad: es la defensa.
    """

    SIDES = ("DL", "UL")

    RX_OK_SET = re.compile(r"^OK\s+(GAIN|AGC)\s+(DL|UL)$", re.I)
    RX_OK_ATT = re.compile(r"^OK\s+ATT$", re.I)
    RX_QUERY = re.compile(r"^(DL|UL)_(GAIN|AGC)=(\d+)$", re.I)
    RX_ATT = re.compile(r"^ATT=(\d+)\s+SRC=(SW|ROT)$", re.I)
    RX_CAL = re.compile(r"^CAL\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)$", re.I)
    RX_REF = re.compile(r"^REF\s+DL=(\d+)\s+UL=(\d+)", re.I)

    RX_LEDS = re.compile(
        r"^LEDS\s+VIN=(\d+)\s+V5V=(\d+)\s+CUR=(\d+)\s+DL=(\d+)\s+UL=(\d+)\s+DG=(\d+)\s+UG=(\d+)"
        r"\s+AGD=(\d+)\s+AGU=(\d+)(?:\s+CMPD=(\d+))?(?:\s+CMPU=(\d+))?\s+ATT=(\d+)"
        r"\s+SRC=(SW|ROT)\s+MAP=0x([0-9A-Fa-f]{1,4})", re.I)
    RX_TEL = re.compile(
        r"^TEL\s+VIN=(\d+)\s+V5V=(\d+)\s+CURR=(\d+)\s+DL_GAIN=(\d+)\s+UL_GAIN=(\d+)"
        r"\s+DL_AGC=(\d+)\s+UL_AGC=(\d+)\s+ATT=(\d+)\s+ATT_SRC=(SW|ROT)", re.I)

    def __init__(self, serial_port, baudrate=115200, usb_vid=ST_VCP_VID, usb_pid=ST_VCP_PID,
                 usb_location=None):
        # No se sondea el equipo acá: eso tomaría el puerto desde el arranque y se lo quitaría
        # a la otra instancia de la mesa que abren los launchers. La conexión ocurre en el
        # primer comando, y si falla, quien llama recibe None y decide qué hacer.
        self.serial_controller = SerialController(
            serial_port, baudrate, device_name="ULAD Amplifier",
            usb_vid=usb_vid, usb_pid=usb_pid, usb_location=usb_location,
        )

    def is_available(self):
        """True si hay un ULAD que responde STATUS. Toma el puerto solo mientras consulta."""
        return self.get_telemetry() is not None

    def close(self):
        self.serial_controller.close()

    def _validate_side(self, side):
        side = str(side).upper()
        if side not in self.SIDES:
            raise ValueError(f"Banda inválida: '{side}'. Debe ser 'DL' o 'UL'.")
        return side

    def _validate_raw(self, raw, maximum, what):
        if not isinstance(raw, int) or isinstance(raw, bool):
            raise TypeError(f"{what} debe ser un entero, se recibió {type(raw).__name__}.")
        if not 0 <= raw <= maximum:
            raise ValueError(f"{what} fuera de rango: {raw}. Debe estar entre 0 y {maximum}.")
        return raw

    def _send(self, command, window_s=None):
        _, responses = self.serial_controller.send_command(command, window_s)
        return responses

    def set_gain(self, side, raw):
        """Escribe la ganancia y confirma con lectura. Devuelve True si el equipo la tomó."""
        side = self._validate_side(side)
        raw = self._validate_raw(raw, self.GAIN_MAX, "La ganancia")

        acknowledged = any(
            self.RX_OK_SET.match(line) and self.RX_OK_SET.match(line).group(1).upper() == "GAIN"
            and self.RX_OK_SET.match(line).group(2).upper() == side
            for line in self._send(f"GAIN {side} {raw}")
        )
        return acknowledged and self.get_gain(side) == raw

    def get_gain(self, side):
        return self._query_value(self._validate_side(side), "GAIN")

    def set_agc(self, side, raw):
        """Escribe el umbral de AGC en CRUDO (0-4095), no en porcentaje."""
        side = self._validate_side(side)
        raw = self._validate_raw(raw, self.AGC_MAX, "El umbral de AGC")

        acknowledged = any(
            self.RX_OK_SET.match(line) and self.RX_OK_SET.match(line).group(1).upper() == "AGC"
            and self.RX_OK_SET.match(line).group(2).upper() == side
            for line in self._send(f"AGC {side} {raw}")
        )
        return acknowledged and self.get_agc(side) == raw

    def get_agc(self, side):
        return self._query_value(self._validate_side(side), "AGC")

    def _query_value(self, side, kind):
        for line in self._send(f"{kind}? {side}"):
            match = self.RX_QUERY.match(line)
            if match and match.group(1).upper() == side and match.group(2).upper() == kind:
                return int(match.group(3))
        return None

    def set_attenuation(self, db):
        """Atenuación por software del BDA4601. Ojo: el equipo también tiene perilla rotativa,
        y la telemetría informa cuál manda en SRC (SW o ROT)."""
        db = self._validate_raw(db, self.ATT_MAX, "La atenuación")
        return any(self.RX_OK_ATT.match(line) for line in self._send(f"ATT {db}"))

    def get_attenuation(self):
        for line in self._send("ATT?"):
            match = self.RX_ATT.match(line)
            if match:
                return {"value": int(match.group(1)), "src": match.group(2).upper()}
        return None

    def get_telemetry(self):
        """Estado completo del equipo. Devuelve el mismo dict para los dos formatos de
        telemetría, porque la flota tiene firmwares distintos conviviendo: LEDS (v2.2.13+)
        y TEL (anterior). Los campos ausentes en TEL quedan en None."""
        for line in self._send("STATUS", window_s=0.9):
            leds = self.RX_LEDS.match(line)
            if leds:
                return {
                    "format": "LEDS",
                    "vin": int(leds.group(1)), "v5v": int(leds.group(2)), "current": int(leds.group(3)),
                    "level_dl": int(leds.group(4)), "level_ul": int(leds.group(5)),
                    "gain_dl": int(leds.group(6)), "gain_ul": int(leds.group(7)),
                    "agc_dl": int(leds.group(8)), "agc_ul": int(leds.group(9)),
                    "cmp_dl": int(leds.group(10)) if leds.group(10) else None,
                    "cmp_ul": int(leds.group(11)) if leds.group(11) else None,
                    "attenuation": int(leds.group(12)), "att_src": leds.group(13).upper(),
                    "led_map": int(leds.group(14), 16),
                }
            tel = self.RX_TEL.match(line)
            if tel:
                return {
                    "format": "TEL",
                    "vin": int(tel.group(1)), "v5v": int(tel.group(2)), "current": int(tel.group(3)),
                    "level_dl": None, "level_ul": None,
                    "gain_dl": int(tel.group(4)), "gain_ul": int(tel.group(5)),
                    "agc_dl": int(tel.group(6)), "agc_ul": int(tel.group(7)),
                    "cmp_dl": None, "cmp_ul": None,
                    "attenuation": int(tel.group(8)), "att_src": tel.group(9).upper(),
                    "led_map": None,
                }
        return None

    def get_led_thresholds(self):
        for line in self._send("CAL GET"):
            match = self.RX_CAL.match(line)
            if match:
                keys = ("off", "vlow", "low", "med", "high", "max")
                return dict(zip(keys, (int(g) for g in match.groups())))
        return None

    def get_dbm_reference(self):
        """Referencia de 0 dBm por unidad. 1800 es el default de fábrica: si lee 1800 en las dos
        bandas, la unidad no fue calibrada individualmente."""
        for line in self._send("CAL REF GET"):
            match = self.RX_REF.match(line)
            if match:
                return {"dl": int(match.group(1)), "ul": int(match.group(2))}
        return None

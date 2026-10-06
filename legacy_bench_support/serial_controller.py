import logging
import re
import time

import serial
from serial.tools import list_ports
from colorama import Fore, Style, init

init(autoreset=True)

# VID/PID del puerto COM virtual (CDC) que genera STM32Cube. Lo comparten todas las placas
# STM32 con ese ejemplo, no solo los VLAD/ULAD.
ST_VCP_VID = 0x0483
ST_VCP_PID = 0x5740

AUTO_PORT = "auto"


def find_usb_serial_port(vid=ST_VCP_VID, pid=ST_VCP_PID, location=None):
    """Busca el COM de un dispositivo USB por VID/PID. Devuelve (puerto, None) o (None, motivo).

    Existe porque Windows le da un COM distinto a cada placa con serial USB propio, así que
    en producción un COM fijo en el YAML deja de servir con la siguiente unidad.

    `location` (p. ej. "1-1") fija el conector físico, para mesas con más de un fixture.
    Con más de un candidato no se elige ninguno: hablarle al equipo equivocado es peor que
    no hablarle a ninguno.
    """
    candidates = [p for p in list_ports.comports() if p.vid == vid and p.pid == pid]
    if location is not None:
        candidates = [p for p in candidates if p.location == str(location)]

    wanted = f"{vid:04X}:{pid:04X}" + (f" en el conector {location}" if location is not None else "")
    if not candidates:
        return None, f"no hay ningún dispositivo USB {wanted} conectado"
    if len(candidates) > 1:
        found = ", ".join(f"{p.device} (conector {p.location})" for p in candidates)
        return None, (
            f"hay {len(candidates)} dispositivos USB {wanted}: {found}. "
            f"Desconectar los que sobran o fijar usb_location"
        )
    return candidates[0].device, None


class SerialController:
    """Transporte serie para equipos que hablan ASCII por UART.

    Hermano de TcpIpController. A diferencia de aquel, mantiene el puerto abierto: reabrirlo en
    cada comando es lento y algunos conversores USB-serie reinician el micro al togglear DTR.
    """

    # El firmware del ULAD ecoa cada comando y emite logs con este prefijo:
    #   [03:03][CMD] STATUS
    #   [03:13][WARN] CMD no reconocido
    # No son respuestas y hay que descartarlos, o se confunden con el valor pedido.
    LOG_LINE = re.compile(r"^\[\d+:\d+\]\[(\w+)\]\s*(.*)$")

    def __init__(self, serial_port, baudrate=115200, device_name="Unknown", timeout=0.1,
                 usb_vid=ST_VCP_VID, usb_pid=ST_VCP_PID, usb_location=None):
        # serial_port: un COM fijo, o "auto" para buscarlo por VID/PID cada vez que se abre.
        # Se resuelve al abrir y no al construir porque entre una prueba y otra se cambia la
        # unidad, y la nueva puede quedar en otro COM.
        self.auto = str(serial_port).lower() == AUTO_PORT
        self.serial_port = serial_port
        self.usb_vid = usb_vid
        self.usb_pid = usb_pid
        self.usb_location = usb_location
        self.baudrate = baudrate
        self.device_name = device_name
        self.timeout = timeout
        self.response_window = 0.6
        self.command_gap = 0.15
        self.connection = None
        # Marca que el puerto se abrió en el comando en curso, para poder reintentarlo:
        # abrir el puerto resetea el micro en algunos conversores USB-serie y el primer
        # comando cae mientras el firmware todavía arranca.
        self._opened_now = False

        self.logger = logging.getLogger(f"serial_{serial_port}")
        self.device_info = f"{Fore.CYAN}[{device_name}@{serial_port}]{Style.RESET_ALL}"

    def _ensure_open(self):
        """Toma el puerto al momento de usarlo, no al construir el controlador.

        El puerto serie admite UN solo proceso, y los launchers abren dos instancias de la
        mesa (la pestaña normal y la de PLACAS). Si se tomara al arrancar, la primera lo
        acapararía para siempre y la otra quedaría sin amplificador aunque sea la que se use.
        Los instrumentos TCP ya funcionan así: abren la conexión por operación.
        """
        if self.connection is not None and self.connection.is_open:
            return True

        if self.auto:
            port, reason = find_usb_serial_port(self.usb_vid, self.usb_pid, self.usb_location)
            if port is None:
                self.logger.warning(
                    f"{self.device_info} {Fore.YELLOW}Autodetección de puerto: {reason}{Style.RESET_ALL}"
                )
                return False
            if port != self.serial_port:
                self.serial_port = port
                self.device_info = f"{Fore.CYAN}[{self.device_name}@{port}]{Style.RESET_ALL}"
                self.logger.info(f"{self.device_info} {Fore.GREEN}Puerto autodetectado: {port}{Style.RESET_ALL}")

        try:
            self.connection = serial.Serial(self.serial_port, self.baudrate, timeout=self.timeout)
        except serial.SerialException as e:
            self.connection = None
            # Sin emoji: si el logger no puede codificar el mensaje, se pierde justo el
            # aviso del fallo que hay que diagnosticar.
            self.logger.warning(
                f"{self.device_info} {Fore.YELLOW}No se pudo tomar {self.serial_port}: {e}"
                f"{Style.RESET_ALL}"
            )
            return False

        # Abrir el puerto puede resetear el micro (DTR). En vez de esperar un tiempo fijo,
        # se drena lo que el firmware escupa al arrancar hasta que se calle.
        self._drain_boot_output()
        self._opened_now = True
        self.logger.info(
            f"{self.device_info} {Fore.GREEN}Puerto serie tomado a {self.baudrate} 8N1{Style.RESET_ALL}"
        )
        return True

    def _drain_boot_output(self, quiet_s=0.25, max_s=2.0):
        """Consume el banner de arranque y espera a que el puerto quede en silencio."""
        deadline = time.monotonic() + max_s
        last_data = time.monotonic()
        while time.monotonic() < deadline:
            if self.connection.read(256):
                last_data = time.monotonic()
            elif time.monotonic() - last_data >= quiet_s:
                break
            else:
                time.sleep(0.02)
        self.connection.reset_input_buffer()

    def close(self):
        """Libera el puerto para que la otra instancia de la mesa pueda usarlo."""
        if self.connection is not None and self.connection.is_open:
            self.connection.close()
            self.logger.info(f"{self.device_info} {Fore.YELLOW}Puerto serie liberado{Style.RESET_ALL}")
        self.connection = None

    def _drop_connection(self):
        try:
            if self.connection is not None:
                self.connection.close()
        except serial.SerialException:
            pass
        self.connection = None

    def _read_lines(self, window_s):
        lines, buf = [], b""
        deadline = time.monotonic() + window_s
        while time.monotonic() < deadline:
            chunk = self.connection.read(256)
            if chunk:
                buf += chunk
                while b"\n" in buf:
                    raw, buf = buf.split(b"\n", 1)
                    text = raw.decode("ascii", errors="replace").strip()
                    if text:
                        lines.append(text)
            else:
                time.sleep(0.02)
        if buf.strip():
            lines.append(buf.decode("ascii", errors="replace").strip())
        return lines

    def send_command(self, command, window_s=None):
        """Envía un comando ASCII y devuelve (exito, lineas_de_respuesta).

        Las lineas de eco y de log del firmware se descartan; las de nivel WARN se registran
        porque son como el equipo avisa que no reconoce un comando.
        """
        window = self.response_window if window_s is None else window_s
        if not self._ensure_open():
            return False, []

        # Si este comando fue el que abrió el puerto, se guarda para reintentarlo una vez:
        # es el único que puede perderse contra el arranque del firmware.
        just_opened = self._opened_now
        self._opened_now = False

        self.logger.info(f"{self._short_name()} {Fore.YELLOW}→ {command}{Style.RESET_ALL}")

        try:
            self.connection.reset_input_buffer()
            self.connection.write((command + "\r\n").encode("ascii"))
            self.connection.flush()
        except serial.SerialException as e:
            self.logger.error(f"{self.device_info} {Fore.RED}💥 Error escribiendo '{command}': {e}{Style.RESET_ALL}")
            # El equipo se desconectó (p. ej. se cambió la unidad). Se suelta el puerto para
            # que el próximo comando lo vuelva a abrir y, en modo auto, a buscar.
            self._drop_connection()
            return False, []

        responses, warnings = [], []
        for line in self._read_lines(window):
            match = self.LOG_LINE.match(line)
            if match:
                if match.group(1).upper() in ("WARN", "ERROR"):
                    warnings.append(match.group(2))
                continue
            responses.append(line)

        for warning in warnings:
            self.logger.warning(f"{self._short_name()} {Fore.YELLOW}⚠ firmware: {warning}{Style.RESET_ALL}")

        if responses:
            self.logger.info(f"{self._short_name()} {Fore.GREEN}← {' | '.join(responses)}{Style.RESET_ALL}")
        else:
            self.logger.debug(f"{self.device_info} sin respuesta a '{command}'")

        if not responses and just_opened:
            # El puerto ya está abierto y el firmware terminó de arrancar: este reintento
            # no se repite, porque just_opened solo es verdadero una vez por apertura.
            self.logger.info(
                f"{self._short_name()} {Fore.YELLOW}sin respuesta al abrir el puerto, "
                f"reintentando '{command}'{Style.RESET_ALL}"
            )
            return self.send_command(command, window_s)

        return bool(responses), responses

    def send_command_batch(self, commands, window_s=None):
        """Envía varios comandos respetando la separación que el firmware necesita."""
        all_responses = []
        for command in commands:
            _, responses = self.send_command(command, window_s)
            all_responses.append(responses)
            time.sleep(self.command_gap)
        return True, all_responses

    def _short_name(self):
        return f"🔌 {self.device_name[:10]}"

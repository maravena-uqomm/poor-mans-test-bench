from controller.base.scpi_commands import verify_idn
from controller.base.scpi_tone_generator import ScpiToneGeneratorCommands
from controller.base.tcp_ip_controller import TcpIpController
from controller.base.tone_generator import ToneGenerator


class SiglentToneGenerator(ScpiToneGeneratorCommands, ToneGenerator, TcpIpController):

    def __init__(self, ip_address: str, port: int = 5025) -> None:
        super().__init__(ip_address=ip_address, port=port, device_name="Siglent Tone Generator")
        is_match, message = verify_idn(self.send_command, "Siglent")
        if not is_match:
            raise ValueError(f"Error: {message}")

    def __exit__(self, exc_type, exc_val, exc_tb):
        try:
            self.stop()
        except (OSError, RuntimeError) as exc:
            self.logger.warning("Failed to stop RF output during cleanup: %s", exc)
        super().__exit__(exc_type, exc_val, exc_tb)
        return False

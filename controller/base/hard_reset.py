"""Interfaz para controladores de relé de hard reset.

Ver docs/runbooks/hard_reset_instrumentos.md para el diseño completo (protocolo, topología
de hardware, timeouts). El relé es un instrumento par — mismo criterio que `PowerSource` —
no un método más del driver que protege.

No hay comando "on"/"cycle": `off` es un PULSO y el equipo protegido se reenergiza solo a
los `HARD_RESET_PULSE_S` segundos (confirmado 23-Sep-2026, runbook §1.1). El propio relé
tampoco contesta `who` durante esa ventana (runbook §1.3).
"""

from __future__ import annotations

#: Segundos del pulso de "off" hasta que el equipo protegido se reenergiza solo. Dato
#: confirmado por el usuario, no medido en el firmware del relé — ver runbook §1.1.
HARD_RESET_PULSE_S = 5.0


class HardResetController:
    """Interfaz para controladores de corte de energía por relé.

    `wait_reenergized()` NO manda ningún comando — solo espera. Se nombra distinto de
    `power_on()` a propósito: un método que sugiere "prendo algo" sobre algo que no manda
    nada es el tipo de driver fantasma que `AGENTS.md` prohíbe para los drivers reales.
    """

    def who(self) -> bool:
        """True si el dispositivo del otro lado es un controlador de relé.

        Sonda de identidad, de solo lectura — segura de mandar a cualquier candidato,
        incluso uno que no sea un relé (ver `scripts/identify_hard_reset.py`). Nunca
        levanta por "no es un relé": eso es un resultado válido, no un error.
        """
        raise NotImplementedError

    def power_off(self) -> None:
        """Dispara el pulso de corte. Levanta `HardwareError` si no se confirma.

        "Corté la energía" es una afirmación que no se puede dejar sin confirmar — un
        `power_off()` que no levanta nada ante un fallo de comunicación firma un corte
        que no ocurrió.
        """
        raise NotImplementedError

    def wait_reenergized(self, hold_s: float = HARD_RESET_PULSE_S) -> None:
        """Espera a que el equipo protegido se reenergice solo. No manda ningún comando."""
        raise NotImplementedError

from typing import Any

from controller.becker.becker_ota_decoder import BeckerOtaDecoder
from controller.dummy.analyzer import DummyAnalyzer
from controller.dummy.becker_ota_decoder import DummyBeckerOtaDecoder
from controller.dummy.power_source import DummyPowerSource
from controller.dummy.rutherford_board import DummyRutherfordBoard
from controller.uqomm.manual_rutterford import ManualRutterford
from controller.dummy.tone_generator import DummyToneGenerator
from controller.dummy.vlad import DummyVlad
from controller.hard_reset.dummy_hard_reset import DummyHardResetController
from controller.hard_reset.serial_hard_reset import SerialHardResetController
from controller.owon.owon_analyzer import OwonAnalyzer
from controller.owon.owon_analyzer_dummy import DummyOwonAnalyzer
from controller.owon.owon_analyzer_usb import OwonAnalyzerUsb
from controller.owon.owon_power_source import OwonPowerSource
from controller.owon.owon_power_source_dummy import DummyOwonPowerSource
from controller.siglent.siglent_analyzer import SiglentAnalyzer
from controller.siglent.siglent_power_source import SiglentPowerSource
from controller.siglent.siglent_tone_generator import SiglentToneGenerator
from controller.uqomm.rutherford_board import RutherfordBoard
from controller.uqomm.vlad_controller import VladController
from controller.vsg.vsg_serial_tone_generator import VsgSerialToneGenerator
from controller.vsg.vsg_tone_generator_dummy import DummyVsGenerator
import inspect


class ControllerFactory:

    _analyzer_classes = {
        "siglent": SiglentAnalyzer,
        "owon": OwonAnalyzer,
        "owon_usb": OwonAnalyzerUsb,
        "dummy": DummyAnalyzer,
        "dummy_owon": DummyOwonAnalyzer,
    }

    _power_source_classes = {
        "siglent": SiglentPowerSource,
        "owon": OwonPowerSource,
        "dummy": DummyPowerSource,
        "dummy_owon": DummyOwonPowerSource,
    }

    _tone_generator_classes = {
        "siglent": SiglentToneGenerator,
        "vsg_serial": VsgSerialToneGenerator,
        "dummy": DummyToneGenerator,
        "dummy_vsg": DummyVsGenerator,
    }

    _rutterford_classes = {
        "uqomm": RutherfordBoard,
        # 'manual': el banco SIN placa, declarado. No es un dummy: le pide el cable al
        # operador y falla si no hay nadie. Ver controller/uqomm/manual_rutterford.py.
        "manual": ManualRutterford,
        "dummy": DummyRutherfordBoard,
        # 'legacy_dummy' (DummyRutterford) se elimino: estaba registrado aca pero NO en
        # VALID_DRIVERS del DTO, asi que por REST era inalcanzable — nadie podia pedirlo nunca.
        # Era un dummy que solo loggeaba, y su reemplazo, DummyRutherfordBoard, modela el
        # crossbar de verdad.
    }

    _vlad_classes = {
        "vlad_rev25": VladController,
        "dummy": DummyVlad,
    }

    _becker_decoder_classes = {
        "nucleo_serial": BeckerOtaDecoder,
        "dummy": DummyBeckerOtaDecoder,
    }

    _hard_reset_classes = {
        "serial_relay": SerialHardResetController,
        "dummy": DummyHardResetController,
    }

    _types_dict = {
        "analyzer": _analyzer_classes,
        "power_source": _power_source_classes,
        "tone_generator": _tone_generator_classes,
        "rutterford": _rutterford_classes,
        "vlad": _vlad_classes,
        "becker_decoder": _becker_decoder_classes,
        "hard_reset": _hard_reset_classes,
    }

    @staticmethod
    def validate_role(ctrl_type: str, name: str) -> None:
        """Check if a controller type+name is registrable without instantiating.
        Raises ValueError if the combination is unknown."""
        try:
            types = ControllerFactory._types_dict[ctrl_type]
            _ = types[name]
        except KeyError as e:
            valid_types = list(ControllerFactory._types_dict.keys())
            valid_names = {k: list(v.keys()) for k, v in ControllerFactory._types_dict.items()}
            raise ValueError(
                f"Invalid controller type or name: ctrl_type='{ctrl_type}', name='{name}'. "
                f"Valid types: {valid_types}. Valid names per type: {valid_names}"
            ) from e

    @staticmethod
    def create(ctrl_type: str, name: str, params: dict) -> Any:
        try:
            types = ControllerFactory._types_dict[ctrl_type]
            cls = types[name]
        except KeyError as e:
            valid_types = list(ControllerFactory._types_dict.keys())
            valid_names = {k: list(v.keys()) for k, v in ControllerFactory._types_dict.items()}
            raise ValueError(
                f"Invalid controller type or name: ctrl_type='{ctrl_type}', name='{name}'. "
                f"Valid types: {valid_types}. Valid names per type: {valid_names}"
            ) from e

        # Filtrar params para pasar solo los argumentos aceptados por el constructor
        sig = inspect.signature(cls)
        has_kwargs = any(p.kind == inspect.Parameter.VAR_KEYWORD for p in sig.parameters.values())

        if has_kwargs:
            filtered_params = params
        else:
            filtered_params = {k: v for k, v in params.items() if k in sig.parameters}

        return cls(**filtered_params)

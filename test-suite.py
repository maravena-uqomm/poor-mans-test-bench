from helper.common import load_tests_from_json
from orchestrator.orchestrator import perform_tests
from controller.siglent.siglent_analyzer import SiglentAnalyzer
from legacy_bench_support.ulad_amplifier import UladAmplifier
from controller.owon.owon_power_source import OwonPowerSource
import time
import argparse


def parse_args():
    parser = argparse.ArgumentParser(
        description="Run a batch of tests against a DUT using a spectrum analyzer and PSU."
    )

    parser.add_argument(
        "--test-file",
        default="./tests/ulad_24_calibration.json",
        help="JSON file describing the batch of tests to apply (default: %(default)s)",
    )

    parser.add_argument(
        "--dut-port",
        default="/dev/ttyACM0",
        help="DUT serial port (default: %(default)s)",
    )

    parser.add_argument(
        "--sa-ip",
        default="10.0.10.2",
        help="Spectrum analyzer IP address (default: %(default)s)",
    )
    parser.add_argument(
        "--sa-port",
        type=int,
        default=5025,
        help="Spectrum analyzer port (default: %(default)s)",
    )

    parser.add_argument(
        "--psu-ip",
        default="10.0.10.201",
        help="PSU IP address (default: %(default)s)",
    )
    parser.add_argument(
        "--psu-port",
        type=int,
        default=4196,
        help="PSU port (default: %(default)s)",
    )

    parser.add_argument(
        "--device-id",
        default=None,
        help="Device ID to use.",
    )

    return parser.parse_args()


def initialize_dut(dut:UladAmplifier):
    dut.set_gain("dl", 4095)
    dut.set_gain("ul", 4095)
    dut.set_agc("dl", 4095)
    dut.set_agc("ul", 4095)


if __name__ == "__main__":
    args = parse_args()
    print(args)
    analyzer = SiglentAnalyzer(args.sa_ip, args.sa_port)
    dut = UladAmplifier(serial_port=args.dut_port)
    # Init PSU for testing
    psu = OwonPowerSource(args.psu_ip, args.psu_port)
    psu.set_voltage(12)
    time.sleep(5)
    psu.turn_on_output()
    initialize_dut(dut)

    results = []
    tests_array = load_tests_from_json(args.test_file)
    perform_tests(analyzer, psu, dut, tests_array, results, args.device_id)


    psu.turn_off_output()
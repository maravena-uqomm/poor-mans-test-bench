from helper.common import identify_device
from controller.siglent.siglent_analyzer import SiglentAnalyzer
from controller.base.power_source import PowerSource
import time

ALLOWED_TYPES_TESTS = ["CALIBRATE", "MEASURE", "USER-INPUT", "POWER"]
MAX_REGISTER_VALUE = 4095
MIN_REGISTER_VALUE = 0
CENTER_FREQ_MARKER_ID = 1

def print_summary(ulad_id, results, status):
    headers = ["Test", "Target", "Tol", "Register", "Power", "Result"]
    rows = []
    for r in results:
        rows.append([
            r["name"],
            "-" if r["target"] is None else f'{r["target"]}',
            "-" if r["tol"] is None else f'{r["tol"]}',
            "-" if r["register"] is None else str(r["register"]),
            "-" if r["power"] is None else f'{r["power"]:.2f}',
            r["result"],
        ])

    widths = [max(len(str(x)) for x in col) for col in zip(headers, *rows)]
    line = "+-" + "-+-".join("-" * w for w in widths) + "-+"
    fmt = "| " + " | ".join(f"{{:<{w}}}" for w in widths) + " |"

    print()
    print(f"SUMMARY - ULAD ID: {ulad_id} - {status}")
    print(line)
    print(fmt.format(*headers))
    print(line)
    for row in rows:
        print(fmt.format(*row))
    print(line)


def wait_user_action(test):
    answer = input(test["name"] + ": ").strip().lower()
    if answer == "q":
        return True
    return False

def check_frequency(analyzer:SiglentAnalyzer, test, analyzer_offset = -40, set_tg=True):
    measure_conf = test["measure"]
    tg_conf = test["tg_conf"]
    criteria = test["criteria"]
    analyzer.set_center_freq(measure_conf['center_freq'])
    analyzer.set_span(measure_conf['span'])
    analyzer.set_resolution_bandwidth_khz(measure_conf['rbw'])
    if(set_tg):
        analyzer.set_tracking_generator_level(tg_conf['level'])
        analyzer.set_tracking_generator("ON")
    analyzer.set_marker(CENTER_FREQ_MARKER_ID, measure_conf['center_freq'])

    result = True
    readed_value = analyzer.get_level_from_marker(CENTER_FREQ_MARKER_ID)
    power_readed = analyzer_offset - readed_value
    if(abs(power_readed - criteria["target"] > criteria["tolerance"])):
        print(f"Band @ {measure_conf["center_freq"]} MHz failed | Power {power_readed}")
        result = False
    return result, power_readed


def binary_search(analyzer: SiglentAnalyzer, dut_setter, test_configuration, side, analyzer_offset=-40, iter=4):
    def read_with_offset():
        if(analyzer_offset):
            return analyzer_offset - analyzer.get_level_from_marker(CENTER_FREQ_MARKER_ID)
        return analyzer.get_level_from_marker(CENTER_FREQ_MARKER_ID)

    
    target = test_configuration['target']
    tolerance = test_configuration['tolerance']
    increasing_flag = None
    max_retry_attempt = 3
    while(max_retry_attempt > 0):
        ok_a = dut_setter(side, MIN_REGISTER_VALUE)
        lowest_power_value = read_with_offset()
        ok_b = dut_setter(side, MAX_REGISTER_VALUE)
        highest_power_value = read_with_offset()
        if(ok_a and ok_b):
            break
        print("Error while testing gain-range. Retrying...")
        max_retry_attempt -=1
    if(not ok_a or not ok_b):
        raise Exception("Could not set up the DUT gain-range")

    increasing_flag = highest_power_value > lowest_power_value
    # Factibility check
    low_value, high_value = sorted((lowest_power_value, highest_power_value))
    if not(low_value - tolerance <= target <= high_value + tolerance):
        print("DUT is out of calibration range ")
        if(iter != 0):
            print(f"Expanding tolerance from {tolerance} ->  {tolerance+0.1}")
            test_configuration['tolerance'] += 0.1
            return binary_search(analyzer, dut_setter, test_configuration, side, analyzer_offset, iter-1)
        print("Max iteration reached. Calibration is not possible within this parameters. Setting max register value.")
        return False, MAX_REGISTER_VALUE, highest_power_value
    best_candidate = None
    low_register = MIN_REGISTER_VALUE
    high_register = MAX_REGISTER_VALUE

    while low_register <= high_register:
        mid_value = (low_register + high_register) // 2
        dut_setter(side, mid_value)
        current_power = read_with_offset()
        current_error = abs(current_power - target)
        if best_candidate is None or current_error < best_candidate[0]:
            best_candidate = (current_error, mid_value, current_power)
        if(current_error <= tolerance):
            return True, mid_value, current_power
        if(current_power < target) == increasing_flag:
            low_register = mid_value + 1
        else:
            high_register = mid_value -1
    if(best_candidate[0] <= tolerance):
        return True, best_candidate[1], best_candidate[2]

    return False, float('nan'), float('nan') # Error 


def calibrate_dut(analyzer:SiglentAnalyzer, dut_setter, test, analyzer_offset=-40):
    analyzer_configurations = test['tg_conf']
    analyzer_measure = test['measure']

    analyzer.set_center_freq(analyzer_measure['center_freq'])
    analyzer.set_span(analyzer_measure['span'])
    analyzer.set_resolution_bandwidth_khz(analyzer_measure['rbw'])

    analyzer.set_tracking_generator_level(analyzer_configurations['level'])
    analyzer.set_tracking_generator("ON")

    analyzer.set_marker(CENTER_FREQ_MARKER_ID, analyzer_measure['center_freq'])

    test_criteria = test['criteria']
    return binary_search(analyzer, dut_setter, test_criteria, test['side'], analyzer_offset=analyzer_offset)


def psu_test(psu: PowerSource, test):

    psu_configurations = test["psu"]
    test_voltage = psu_configurations["voltage"]
    current_limit = psu_configurations["current_limit"]

    test_configurations = test["criteria"]
    target = test_configurations["target"]
    tolerance = test_configurations["tolerance"]

    time.sleep(1)

    psu.set_current_limit(current_limit)
    psu.set_voltage(test_voltage)

    time.sleep(1)

    readed_current = psu.get_amperage()
    readed_voltage = psu.get_voltage()
    calculated_power = readed_current * readed_voltage


    return (abs(calculated_power-target) <= tolerance, calculated_power)


    


def perform_tests(analyzer, psu, dut, test_array, results, device_id=None):

    def add(name, ok, target=None, tol=None, register=None, power=None):
        results.append({
            "name": name, "target": target, "tol": tol,
            "register": register, "power": power,
            "result": "PASS" if ok else "FAIL",
        })
        return ok

    if(device_id == None):
        id_device = identify_device(2)
    else:
        id_device = device_id
    

    for test in test_array:
        test_results = None
        test_type = test["type"]
        test_objective = test["objective"]
        if(test_type not in ALLOWED_TYPES_TESTS):
            raise ValueError(f"Test type does not conform with the accepted values: {ALLOWED_TYPES_TESTS}, got {test_type}")

        if(test_type == 'CALIBRATE'):
            if(test_objective == "gain"):
                test_results = calibrate_dut(analyzer, dut.set_gain, test)
            if(test_objective == "agc"):
                test_results = calibrate_dut(analyzer, dut.set_agc, test, 0)
        if(test_type == 'MEASURE'):
            test_results = check_frequency(analyzer, test)

        if(test_type == 'USER-INPUT'):
            if(wait_user_action(test)):
                break

        if(test_type == "POWER"):
            test_results = psu_test(psu, test)

        
        if(test_results):
            if(len(test_results) == 3):
                add(test["name"], test_results[0], test["criteria"]["target"], test["criteria"]["tolerance"], register=test_results[1], power=test_results[2])
            else:
                add(test["name"], test_results[0], test["criteria"]["target"], test["criteria"]["tolerance"], register=None, power=test_results[1])

    result_status = "PASSED" if all(r["result"] == "PASS" for r in results) else "FAILED"
    print_summary(id_device, results, result_status)
from controller.siglent.siglent_analyzer import SiglentAnalyzer
import cv2
import serial
from legacy_bench_support.ulad_amplifier import UladAmplifier
import time
from helper.common import identify_device

MAX_REGISTER_VALUE = 4095
MIN_REGISTER_VALUE = 0
CENTER_FREQ_MARKER_ID = 1
RBW = 0.3
SPAN_FREQ = 0


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

def binary_gain_calibration(type:str, target:float, tolerance:float, analyzer: SiglentAnalyzer, dut: UladAmplifier, analyzer_offset=-40, iter=4):
    freq = 0
    if type == "dl":
        freq = 470
    elif type == 'ul':
        freq = 445
    else:
        print("Error while setting frequency")
        exit(1)

    # Set instrument
    analyzer.set_center_freq(freq)
    analyzer.set_span(SPAN_FREQ)
    analyzer.set_marker(CENTER_FREQ_MARKER_ID, freq)
    analyzer.set_resolution_bandwidth_khz(RBW)
    analyzer.set_tracking_generator("ON")
    analyzer.set_tracking_generator_level(-20)

    # Start calibrating
    increasing_flag = None

    # Set to initials values 
    while(True):
        ok_a = dut.set_gain(type, MIN_REGISTER_VALUE)
        #time.sleep(0.5)
        lowest_power_value = analyzer_offset - analyzer.get_level_from_marker(CENTER_FREQ_MARKER_ID)
        ok_b = dut.set_gain(type, MAX_REGISTER_VALUE)
        #time.sleep(0.5)
        highest_power_value = analyzer_offset - analyzer.get_level_from_marker(CENTER_FREQ_MARKER_ID)
        if(ok_a and ok_b):
            break
        print("Error while setting gains")


    increasing_flag = highest_power_value > lowest_power_value

    # Factibility check
    low_value, high_value = sorted((lowest_power_value, highest_power_value))
    if not(low_value - tolerance <= target <= high_value + tolerance):
        print("DUT out of calibration range")
        # EXPAND TOLERANCE
        if(iter != 0):
            print(f"Expanding tolerance to : {tolerance+0.1}")
            return binary_gain_calibration(type, target, tolerance+0.1, analyzer, dut, analyzer_offset, iter=iter-1)
        print("MAXED ITERATION")
        return False, MAX_REGISTER_VALUE, highest_power_value
        
        

    best_candidate = None

    low_register = MIN_REGISTER_VALUE
    high_register = MAX_REGISTER_VALUE

    while low_register <= high_register:
        mid_value = (low_register + high_register) // 2
        dut.set_gain(type, mid_value)
        #time.sleep(0.5)
        current_power = analyzer_offset - analyzer.get_level_from_marker(CENTER_FREQ_MARKER_ID)
        current_error = abs(current_power - target)
        if best_candidate is None or current_error < best_candidate[0]:
            best_candidate = (current_error, mid_value, current_power)
        if current_error <= tolerance:
            return True, mid_value, current_power
        if(current_power < target) == increasing_flag:
            low_register = mid_value + 1
        else:
            high_register = mid_value -1 

    if(best_candidate[0] <= tolerance):
        return True, best_candidate[1], best_candidate[2]
    
    return None, "Faulty"

def binary_agc_calibration(type:str, target:float, tolerance:float, analyzer: SiglentAnalyzer, dut: UladAmplifier):
    freq = 0
    if type == "dl":
        freq = 470
    elif type == 'ul':
        freq = 445
    else:
        print("Error while setting frequency")
        exit(1)

    # Set instrument
    analyzer.set_center_freq(freq)
    analyzer.set_span(SPAN_FREQ)
    analyzer.set_marker(CENTER_FREQ_MARKER_ID, freq)
    analyzer.set_resolution_bandwidth_khz(RBW)
    analyzer.set_tracking_generator("ON")
    analyzer.set_tracking_generator_level(0)

    # Start calibrating
    increasing_flag = None

    # Set to initials values 
    dut.set_agc(type, MIN_REGISTER_VALUE)
    #time.sleep(0.5)
    lowest_power_value =  analyzer.get_level_from_marker(CENTER_FREQ_MARKER_ID)
    dut.set_agc(type, MAX_REGISTER_VALUE)
    #time.sleep(0.5)
    highest_power_value = analyzer.get_level_from_marker(CENTER_FREQ_MARKER_ID)

    increasing_flag = highest_power_value > lowest_power_value

    # Factibility check
    low_value, high_value = sorted((lowest_power_value, highest_power_value))
    if not(low_value - tolerance <= target <= high_value + tolerance):
        print("DUT out of AGC calibration range")
        return False, MAX_REGISTER_VALUE, highest_power_value

    best_candidate = None

    low_register = MIN_REGISTER_VALUE
    high_register = MAX_REGISTER_VALUE

    while low_register <= high_register:
        mid_value = (low_register + high_register) // 2
        dut.set_agc(type, mid_value)
        #time.sleep(0.5)
        current_power = analyzer.get_level_from_marker(CENTER_FREQ_MARKER_ID)
        current_error = abs(current_power - target)
        if best_candidate is None or current_error < best_candidate[0]:
            best_candidate = (current_error, mid_value, current_power)
        if current_error <= tolerance:
            return True, mid_value, current_power
        if(current_power < target) == increasing_flag:
            low_register = mid_value + 1
        else:
            high_register = mid_value -1 

    if(best_candidate[0] <= tolerance):
        return True, best_candidate[1], best_candidate[2]
    
    return None, "Faulty"


def check_side_frequency(freq, target, tolerance, analyzer=SiglentAnalyzer, analyzer_offset=-40):
    analyzer.set_center_freq(freq)
    analyzer.set_span(SPAN_FREQ)
    analyzer.set_marker(CENTER_FREQ_MARKER_ID, freq)
    analyzer.set_resolution_bandwidth_khz(RBW)

    result = True

    readed_value = analyzer.get_level_from_marker(1)
    side_freqency_power = analyzer_offset - readed_value
    if (abs(side_freqency_power -target) > tolerance):
        print(f"Side band at : {freq} Mhz failed | Power: {side_freqency_power}")
        result = False
    print(f"Side band at {freq} MHz: {side_freqency_power}")
    return result, side_freqency_power


def run_tests(analyzer, dut, results):
    def add(name, ok, target=None, tol=None, register=None, power=None):
        results.append({
            "name": name, "target": target, "tol": tol,
            "register": register, "power": power,
            "result": "PASS" if ok else "FAIL",
        })
        return ok

    answer = input("Press enter to start testing (or q to quit): ").strip().lower()
    if answer == "q":
        return "ABORTED"

    # Downlink
    ok, reg, pwr = binary_gain_calibration("dl", -26.75, 0.25, analyzer=analyzer, dut=dut)
    add("DL gain", ok, -26.75, 0.25, reg, pwr)

    ok_460, pwr_460 = check_side_frequency(460, -25, 2, analyzer)
    #time.sleep(0.5)
    ok_480, pwr_480 = check_side_frequency(480, -25, 2, analyzer)
    add("DL sideband 460", ok_460, -25, 2, power=pwr_460)
    add("DL sideband 480", ok_480, -25, 2, power=pwr_480)

    ok, reg, pwr = binary_agc_calibration("dl", -2, 0.05, analyzer=analyzer, dut=dut)
    add("DL AGC", ok, -2, 0.05, reg, pwr)

    answer = input("Change connections and press enter (or q to quit): ").strip().lower()
    if answer == "q":
        return "ABORTED"

    # Uplink
    ok, reg, pwr = binary_gain_calibration("ul", -26.75, 0.25, analyzer=analyzer, dut=dut)
    add("UL gain", ok, -26.75, 0.25, reg, pwr)

    ok_435, pwr_435 = check_side_frequency(435.1, -25, 2, analyzer)
    #time.sleep(0.5)
    ok_455, pwr_455 = check_side_frequency(455, -25, 2, analyzer)
    add("UL sideband 435.1", ok_435, -25, 2, power=pwr_435)
    add("UL sideband 455", ok_455, -25, 2, power=pwr_455)

    ok, reg, pwr = binary_agc_calibration("ul", -10, 0.05, analyzer=analyzer, dut=dut)
    add("UL AGC", ok, -10, 0.05, reg, pwr)

    return "PASSED" if all(r["result"] == "PASS" for r in results) else "FAILED"


if __name__ == '__main__':
    # Initialize instruments
    analyzer = SiglentAnalyzer("170.0.0.2", 5025)
    dut = UladAmplifier(serial_port='/dev/ttyACM0')
    

    ulad_id = identify_device(2)
    results = []
    status = run_tests(analyzer, dut, results)
    print_summary(ulad_id, results, status)
    print(results)
    exit(0 if status == "PASSED" else 1)

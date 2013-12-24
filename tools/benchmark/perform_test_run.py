#!/usr/bin/env python
#
# Pavel Kirienko <pavel.kirienko@gmail.com>
#
# ESC testing utility
#

import sys, time, threading, logging
from functools import partial
from drivers.ardubenchmark import Ardubenchmark

# Safety feature TODO: command line option for that
HALF_DUTY_CYCLE = False

SLOPE_MIN = 0.1
SLOPE_MAX = 1.0
SLOPE_RISE_TIME = 60.0
SLOPE_TOP_DELAY = 2.0
SLOPE_TIME_STEP = 0.04

# Duty cycle [0; 1]; Duration (sec)
DYNAMIC_PATTERN = [
    # Spinup
    (0.1, 3.0),
    (0.2, 3.0),
    # Slow, wide range
    (1.0, 2.0),
    (0.3, 2.0),
    (0.8, 1.0),
    (0.5, 1.0),
    (0.7, 0.5),
    (0.6, 0.5),
    # Fast, narrow range
    (0.7, 0.3),
    (0.8, 0.3),
    (0.6, 0.3),
    (0.8, 0.3),
    (0.7, 0.1),
    (0.8, 0.1),
    (0.6, 0.1),
    (0.8, 0.1),
]

# -----------------

def make_driver(name, port=None):
    from drivers.px4esc_cli import Px4EscCli
    from drivers.esc32_cli import Esc32Cli
    from drivers.pololu_maestro import PololuMaestro
    from drivers.canas import CanAerospaceEsc

    if name == 'px4esc':
        drv = Px4EscCli(port or '/dev/ttyUSB1')
        drv.stop = partial(drv.set_duty_cycle, 0.0)
        def start():
            drv.arm()
            drv.set_duty_cycle(0.01)
        drv.start = start
        drv.require_echo = False # Echo may be mixed with diagnostics
    elif name == 'canas':
        drv = CanAerospaceEsc(port or 'slcan0')
    elif name == 'esc32':
        drv = Esc32Cli(port or '/dev/ttyUSB1')
    elif name == 'ppm':
        CHANNEL_INDEX = 0
        drv = PololuMaestro(port or '/dev/ttyACM0')
        drv.set_duty_cycle = partial(drv.set_ppm_fraction, CHANNEL_INDEX)
        drv.stop = partial(drv.set_ppm_fraction, CHANNEL_INDEX, 0.0)
        drv.start = partial(drv.set_ppm_fraction, CHANNEL_INDEX, 0.05)
        drv.set_duty_cycle(0.0)
        time.sleep(3.0)          # Wait while the PPM controller establishes zero output
    else:
        raise ValueError('Unknown driver name', name)

    if not hasattr(drv, 'dispose'):
        drv.dispose = lambda: None

    if HALF_DUTY_CYCLE:
        print 'Duty cycle limiting is enabled'
        drv.set_duty_cycle_unsafe_ = drv.set_duty_cycle
        drv.set_duty_cycle = lambda dc: drv.set_duty_cycle_unsafe_(float(dc) / 2.0)

    assert hasattr(drv, 'set_duty_cycle') and hasattr(drv, 'stop') and hasattr(drv, 'start')
    return drv

class TestExecutor:
    def __init__(self, driver, on_duty_cycle_change=None):
        self.on_duty_cycle_change = on_duty_cycle_change or (lambda dc: None)
        self._driver = driver

    def _start(self):
        self._driver.start()
        time.sleep(1.0)

    def run_dynamic(self, pattern):
        try:
            self._start()
            for dc, duration in pattern:
                #print 'DC %.2f for %.2f sec' % (dc, duration)
                self._driver.set_duty_cycle(dc)
                self.on_duty_cycle_change(dc)
                time.sleep(duration)
            self.on_duty_cycle_change(0.0)
        finally:
            self._driver.stop()

    def run_slope(self, min, max, rise_time, time_step, top_delay):
        assert max > min
        def run_eq(yintercept, slope, yend):
            start_time = time.time()
            while 1:
                x = time.time() - start_time
                y = slope * x + yintercept
                done = (y >= yend) if slope > 0 else (y <= yend)
                dc = yend if done else y
                #print dc
                driver.set_duty_cycle(dc)
                self.on_duty_cycle_change(dc)
                if done:
                    break
                time.sleep(time_step)
        try:
            self._start()
            run_eq(min, (max - min) / rise_time, max)
            time.sleep(top_delay)
            run_eq(max, (min - max) / rise_time, min)
            self.on_duty_cycle_change(0.0)
        finally:
            self._driver.stop()

class MeasurementCollector:
    def __init__(self, num_blades):
        self.collected_data = []
        self._num_blades = num_blades
        self._current_duty_cycle = 0.0
        self._timestamp = None
        self._timestamp_offset = None

    def set_current_duty_cycle(self, dc):
        self._current_duty_cycle = dc

    def add_measurements(self, prop_period, voltage, current):
        if self._timestamp is None:
            self._timestamp_offset = self._timestamp = time.time()

        if prop_period > 0.0:
            self._timestamp += prop_period
            rpm = 60.0 / prop_period / self._num_blades
        else:
            self._timestamp = time.time()
            rpm = 0.0
        
        relative_timestamp = self._timestamp - self._timestamp_offset
        sample = relative_timestamp, self._current_duty_cycle, rpm, voltage, current
        self.collected_data.append(sample)

    def write_to_csv(self, writeable, separator=None):
        separator = separator or ' '
        columns = 'time', 'dc', 'rpm', 'voltage', 'current'
        for item in [columns] + self.collected_data:
            line = separator.join(map(str, item))
            writeable.write(line + '\n')

def run_test(collector, driver, ardubenchmark, test_runnable):
    ardubenchmark_thread = threading.Thread(target=ardubenchmark.run, args=(collector.add_measurements,))
    ardubenchmark_thread.daemon = True
    ardubenchmark_thread.start()
    time.sleep(2.0)

    test_runnable()
    time.sleep(7.0) # Wait for deceleration

    ardubenchmark.stop()
    ardubenchmark_thread.join()

# -----------------

get_argv = lambda idx, default: sys.argv[idx] if len(sys.argv) > idx else default

driver_name = get_argv(1, None)
driver_port = get_argv(2, None)
ardubenchmark_port = get_argv(3, '/dev/ttyUSB0')
num_blades = int(get_argv(4, 2))

logging.basicConfig(stream=sys.stderr, level=logging.DEBUG, format='%(asctime)s %(levelname)s: %(message)s')

driver = make_driver(driver_name, driver_port)
ardubenchmark = Ardubenchmark(ardubenchmark_port)
executor = TestExecutor(driver)

test_timestamp = time.time()

def perform(test_name, executor_call):
    collector = MeasurementCollector(num_blades)
    executor.on_duty_cycle_change = collector.set_current_duty_cycle
    run_test(collector, driver, ardubenchmark, partial(executor_call, executor))

    timestamp_str = time.strftime('%Y-%m-%d-%H-%M-%S', time.localtime(test_timestamp))
    output_filename = '%s-%s-%s.csv' % (driver_name, test_name, timestamp_str)
    with open(output_filename, 'w') as f:
        collector.write_to_csv(f)

perform('slope', lambda x: x.run_slope(SLOPE_MIN, SLOPE_MAX, SLOPE_RISE_TIME, SLOPE_TIME_STEP, SLOPE_TOP_DELAY))

perform('dynamic', lambda x: x.run_dynamic(DYNAMIC_PATTERN))

driver.dispose()

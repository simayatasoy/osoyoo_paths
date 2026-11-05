#!/usr/bin/env python3
"""Drive the Osoyoo car in a circle.  No Vicon, no ROS.

    python3 circle.py                    # uses circle_config.json next to this file
    python3 circle.py my_config.json

Ctrl+C stops the car.
"""
import json
import signal
import sys
import time
from pathlib import Path

import Adafruit_PCA9685
import RPi.GPIO as GPIO

# Car calibration (from osoyoo_6; osoyoo_5 has not been measured).
SPEED_AT_CMD_0 = 0.308   # m/s: wheel speed = SPEED_AT_CMD_0 + SPEED_PER_CMD * cmd
SPEED_PER_CMD = 0.61
TRACK_M = 0.27           # effective distance between left and right wheels
MIN_CMD = 0.04           # below this a wheel stops

# Wiring
IN1, IN2, IN3, IN4 = 23, 24, 27, 22   # direction pins
ENA, ENB = 0, 1                       # PWM channels: left, right
MIN_PWM, MAX_PWM = 1200, 4095

pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(60)
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
for pin in (IN1, IN2, IN3, IN4):
    GPIO.setup(pin, GPIO.OUT)


def wheel(channel, pin_a, pin_b, cmd):
    """cmd 0..1 forward.  These cars drive forward with pin_a LOW, pin_b HIGH."""
    GPIO.output(pin_a, GPIO.LOW)
    GPIO.output(pin_b, GPIO.HIGH)
    pwm.set_pwm(channel, 0, int(MIN_PWM + (MAX_PWM - MIN_PWM) * cmd))


def stop():
    for pin in (IN1, IN2, IN3, IN4):
        GPIO.output(pin, GPIO.LOW)
    pwm.set_pwm(ENA, 0, 0)
    pwm.set_pwm(ENB, 0, 0)


def speed_to_cmd(speed):
    cmd = (speed - SPEED_AT_CMD_0) / SPEED_PER_CMD
    if cmd < MIN_CMD or cmd > 1.0:
        print(f"warning: a wheel needs {speed:.2f} m/s, outside what the car can do; "
              f"try a different speed or diameter")
    return min(max(cmd, MIN_CMD), 1.0)


def main():
    config_path = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).with_name("circle_config.json")
    cfg = json.loads(config_path.read_text())
    speed, radius = cfg["speed_m_s"], cfg["diameter_m"] / 2.0

    # Counter-clockwise circle: the right (outer) wheel goes faster.
    left = speed_to_cmd(speed * (1 - TRACK_M / (2 * radius)))
    right = speed_to_cmd(speed * (1 + TRACK_M / (2 * radius)))
    print(f"circle: {speed} m/s, diameter {cfg['diameter_m']} m, for {cfg['duration_s']} s "
          f"(wheel commands left {left:.2f}, right {right:.2f})")

    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))   # kill / dropped SSH also stop the car
    signal.signal(signal.SIGHUP, lambda *_: sys.exit(0))
    try:
        wheel(ENA, IN1, IN2, left)
        wheel(ENB, IN3, IN4, right)
        time.sleep(cfg["duration_s"])
    except KeyboardInterrupt:
        pass
    finally:
        stop()
        GPIO.cleanup()
        print("stopped")


if __name__ == "__main__":
    main()

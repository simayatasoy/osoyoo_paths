#!/usr/bin/env python3
"""Drive the Osoyoo car on a circle around the lab origin (0, 0), using Vicon.

    python3 circle_vicon.py                       # uses circle_vicon_config.json
    python3 circle_vicon.py my_config.json

Start the car anywhere within about 0.75 m of (0, 0), facing any way, and it
finds the circle by itself.  It first drives straight for a moment to learn
which way the Vicon object's axes point relative to the car, so there is no
heading offset to set.  Ctrl+C stops the car.
"""
import csv
import json
import math
import signal
import sys
import time
from collections import deque
from pathlib import Path

# Car calibration (from osoyoo_6; osoyoo_5 has not been measured).
SPEED_AT_CMD_0 = 0.308   # m/s: wheel speed = SPEED_AT_CMD_0 + SPEED_PER_CMD * cmd
SPEED_PER_CMD = 0.61
TRACK_M = 0.27           # effective distance between left and right wheels
MIN_CMD = 0.04           # below this a wheel stops

# Steering
APPROACH_GAIN = 1.5      # how sharply it heads back to the circle
TURN_GAIN = 2.0          # how hard it steers; lower it if the car wiggles
MAX_TURN_RAD_S = 2.2
TURN_ON_SPOT_ABOVE_DEG = 60   # facing further away than this: turn on the spot...
TURN_ON_SPOT_UNTIL_DEG = 20   # ...until facing within this
HEADING_WINDOW_S = 0.25       # direction of travel is measured over this time

# Safety
STOP_IF_FARTHER_THAN_M = 1.0  # from (0, 0); the free floor is about 1 m each way in y
VICON_TIMEOUT_S = 0.5
CONTROL_HZ = 20

# Wiring
IN1, IN2, IN3, IN4 = 23, 24, 27, 22   # direction pins
ENA, ENB = 0, 1                       # PWM channels: left, right
MIN_PWM, MAX_PWM = 1200, 4095


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def open_log(script, cfg):
    """logs/<script>_<robot>_<date>_<time>.csv, plus the config used, next to this file."""
    folder = Path(__file__).resolve().parent / "logs"
    folder.mkdir(exist_ok=True)
    name = f"{script}_{cfg['robot']}_{time.strftime('%Y%m%d_%H%M%S')}"
    (folder / f"{name}.json").write_text(json.dumps(cfg, indent=2) + "\n")
    log_file = open(folder / f"{name}.csv", "w", newline="", buffering=1)   # line by line: survives a crash
    writer = csv.writer(log_file)
    writer.writerow(["time_s", "status", "x_m", "y_m", "vicon_yaw_rad", "heading_rad", "left_cmd", "right_cmd"])
    return log_file, writer, folder / f"{name}.csv"


def log_row(writer, t, status, pose=None, follower=None, command=(0.0, 0.0)):
    if pose is None:
        writer.writerow([f"{t:.3f}", status, "", "", "", "", "0", "0"])
        return
    heading = "" if follower is None or follower.offset is None else f"{wrap(pose['yaw'] + follower.offset):.4f}"
    writer.writerow([f"{t:.3f}", status, f"{pose['x']:.4f}", f"{pose['y']:.4f}", f"{pose['yaw']:.4f}",
                     heading, f"{command[0]:.3f}", f"{command[1]:.3f}"])


class CircleFollower:
    """Vicon pose in, (left, right) wheel commands out (-1..1).  No hardware here."""

    def __init__(self, speed, radius):
        self.speed, self.radius = speed, radius
        self.track = deque()
        self.offset = None        # car heading minus Vicon yaw, learned while driving
        self.omega = 0.0
        self.turning_on_spot = False

    def learn_offset(self, t, x, y, yaw):
        """Compare the direction the car moves with the Vicon yaw."""
        self.track.append((t, x, y, yaw))
        while t - self.track[0][0] > HEADING_WINDOW_S:
            self.track.popleft()
        _, x0, y0, yaw0 = self.track[0]
        if self.turning_on_spot or math.hypot(x - x0, y - y0) < 0.03:
            return
        travel = math.atan2(y - y0, x - x0)
        middle_yaw = yaw0 + wrap(yaw - yaw0) / 2       # yaw half-way along the chord
        sample = wrap(travel - middle_yaw)
        if self.offset is None:
            self.offset = sample
        else:
            self.offset = wrap(self.offset + 0.05 * wrap(sample - self.offset))

    def step(self, t, x, y, yaw):
        self.learn_offset(t, x, y, yaw)
        if self.offset is None:
            return self.wheel_commands(self.speed, 0.0)      # drive straight until it knows
        heading = yaw + self.offset
        r = math.hypot(x, y)
        tangent = math.atan2(y, x) + math.pi / 2              # counter-clockwise
        aim = tangent - math.atan(APPROACH_GAIN * (self.radius - r))
        error = wrap(aim - heading)

        if abs(error) > math.radians(TURN_ON_SPOT_ABOVE_DEG):
            self.turning_on_spot = True
        elif abs(error) < math.radians(TURN_ON_SPOT_UNTIL_DEG):
            self.turning_on_spot = False
        if self.turning_on_spot:
            self.track.clear()
            return (-MIN_CMD, MIN_CMD) if error > 0 else (MIN_CMD, -MIN_CMD)

        self.omega = self.speed / self.radius + TURN_GAIN * error
        self.omega = max(-MAX_TURN_RAD_S, min(MAX_TURN_RAD_S, self.omega))
        return self.wheel_commands(self.speed, self.omega)

    @staticmethod
    def wheel_commands(speed, omega):
        left, right = speed - omega * TRACK_M / 2, speed + omega * TRACK_M / 2
        slowest = SPEED_AT_CMD_0 + SPEED_PER_CMD * MIN_CMD
        lift = max(0.0, slowest - min(left, right))   # never let a wheel stall: raise both
        to_cmd = lambda s: (s + lift - SPEED_AT_CMD_0) / SPEED_PER_CMD
        cmd_left, cmd_right = to_cmd(left), to_cmd(right)
        top = max(cmd_left, cmd_right, 1.0)           # too fast: scale both down
        return cmd_left / top, cmd_right / top


def main():
    import Adafruit_PCA9685
    import RPi.GPIO as GPIO
    import rclpy
    from rclpy.node import Node
    from vicon_receiver.msg import PositionList

    config_path = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).with_name("circle_vicon_config.json")
    cfg = json.loads(config_path.read_text())
    follower = CircleFollower(cfg["speed_m_s"], cfg["diameter_m"] / 2.0)

    pwm = Adafruit_PCA9685.PCA9685()
    pwm.set_pwm_freq(60)
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    for pin in (IN1, IN2, IN3, IN4):
        GPIO.setup(pin, GPIO.OUT)

    def drive(cmd_left, cmd_right):
        # These cars drive forward with the first pin LOW and the second HIGH.
        for channel, pin_a, pin_b, cmd in ((ENA, IN1, IN2, cmd_left), (ENB, IN3, IN4, cmd_right)):
            GPIO.output(pin_a, GPIO.LOW if cmd >= 0 else GPIO.HIGH)
            GPIO.output(pin_b, GPIO.HIGH if cmd >= 0 else GPIO.LOW)
            pwm.set_pwm(channel, 0, int(MIN_PWM + (MAX_PWM - MIN_PWM) * abs(cmd)))

    def stop():
        for pin in (IN1, IN2, IN3, IN4):
            GPIO.output(pin, GPIO.LOW)
        pwm.set_pwm(ENA, 0, 0)
        pwm.set_pwm(ENB, 0, 0)

    pose = {"time": None}

    def on_vicon(msg):
        for p in msg.positions[:msg.n]:
            if p.subject_name == cfg["robot"] and (p.x_trans or p.y_trans or p.z_trans):  # all zeros = lost
                pose.update(x=p.x_trans / 1000.0, y=p.y_trans / 1000.0, yaw=p.z_rot_euler,
                            time=time.monotonic())

    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))   # kill / dropped SSH also stop the car
    signal.signal(signal.SIGHUP, lambda *_: sys.exit(0))
    rclpy.init()
    node = Node("circle_vicon")
    node.create_subscription(PositionList, cfg["vicon_topic"], on_vicon, 10)
    print(f"{cfg['robot']} on {cfg['vicon_topic']}: circle around (0, 0), {cfg['speed_m_s']} m/s, "
          f"diameter {cfg['diameter_m']} m, for {cfg['duration_s']} s")
    log_file, log, log_path = open_log("circle_vicon", cfg)
    print(f"logging positions to {log_path}")
    start = next_tick = time.monotonic()
    waiting = False
    try:
        while time.monotonic() - start < cfg["duration_s"]:
            rclpy.spin_once(node, timeout_sec=max(0.0, next_tick - time.monotonic()))
            if time.monotonic() < next_tick:
                continue
            next_tick += 1.0 / CONTROL_HZ
            now = time.monotonic()
            if pose["time"] is None or now - pose["time"] > VICON_TIMEOUT_S:
                stop()
                follower.track.clear()
                if not waiting:
                    print(f"no Vicon position for {cfg['robot']} on {cfg['vicon_topic']}: stopped, waiting")
                waiting = True
                log_row(log, now - start, "no_vicon")
                continue
            if waiting:
                print("Vicon position received, driving")
                waiting = False
            distance = math.hypot(pose["x"], pose["y"])
            if distance > STOP_IF_FARTHER_THAN_M:
                print(f"stopped: car is {distance:.2f} m from (0, 0), more than {STOP_IF_FARTHER_THAN_M} m")
                log_row(log, now - start, "stopped_too_far", pose, follower)
                break
            command = follower.step(now, pose["x"], pose["y"], pose["yaw"])
            log_row(log, now - start, ("turning_on_spot" if follower.turning_on_spot else "learning_heading" if follower.offset is None else "driving"), pose, follower, command)
            drive(*command)
    except KeyboardInterrupt:
        pass
    finally:
        stop()
        log_file.close()
        GPIO.cleanup()
        node.destroy_node()
        rclpy.shutdown()
        if follower.offset is not None:
            print(f"learned heading offset: {math.degrees(follower.offset):.0f} deg")
        print("stopped")


if __name__ == "__main__":
    main()

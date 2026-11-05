#!/usr/bin/env python3
"""Drive the Osoyoo car in a zigzag, using Vicon.

    python3 zigzag_vicon.py                       # uses zigzag_vicon_config.json
    python3 zigzag_vicon.py my_config.json

The zigzag starts where the car is.  It advances along direction_deg (lab
degrees, 0 = +x, 90 = +y); each strip is strip_length_m long and leans
zigzag_angle_deg away from that direction, first to the left if the angle is
positive, to the right if negative.  Each corner is rounded into an arc of
turn_radius_m, so the car turns while driving forward.

Point the car roughly along the first strip: it drives straight for about
10 cm first, to learn which way the Vicon object's axes point.
Ctrl+C stops the car.
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
APPROACH_GAIN = 3.0      # how sharply it heads back to the strip
TURN_GAIN = 2.0          # how hard it steers; lower it if the car wiggles
MAX_TURN_RAD_S = 2.2
TURN_ON_SPOT_ABOVE_DEG = 90   # facing further away than this (only at the start): turn on the spot...
TURN_ON_SPOT_UNTIL_DEG = 20   # ...until facing within this
HEADING_WINDOW_S = 0.25       # direction of travel is measured over this time
TURN_EARLY_S = 0.25           # start each turn this long before the arc: the wheels react late

SLOWEST_SPEED_M_S = 0.15     # below this the wheels only pulse, and the heading cannot be learned
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


def zigzag_corners(start, cfg):
    """Corner points from the start, stopping before one would leave the safe area."""
    (xmin, xmax), (ymin, ymax) = cfg["safe_area"]["x"], cfg["safe_area"]["y"]
    direction, lean = math.radians(cfg["direction_deg"]), math.radians(cfg["zigzag_angle_deg"])
    corners = [start]
    for i in range(cfg["strips"]):
        angle = direction + (lean if i % 2 == 0 else -lean)
        x, y = corners[-1]
        x, y = x + cfg["strip_length_m"] * math.cos(angle), y + cfg["strip_length_m"] * math.sin(angle)
        if not (xmin <= x <= xmax and ymin <= y <= ymax):
            break
        corners.append((x, y))
    return corners


class Line:
    def __init__(self, a, b):
        self.a, self.length = a, math.hypot(b[0] - a[0], b[1] - a[1])
        self.ux, self.uy = (b[0] - a[0]) / self.length, (b[1] - a[1]) / self.length

    def reference(self, x, y):
        """(metres left on this piece, tangent, distance to the left of the path, curvature)"""
        dx, dy = x - self.a[0], y - self.a[1]
        along = dx * self.ux + dy * self.uy
        return (self.length - along, math.atan2(self.uy, self.ux),
                self.ux * dy - self.uy * dx, 0.0)


class Arc:
    def __init__(self, center, radius, start_angle, sweep):
        self.cx, self.cy = center
        self.radius, self.start, self.sweep = radius, start_angle, sweep
        self.sign = 1.0 if sweep > 0 else -1.0        # +1 turns left

    def reference(self, x, y):
        phi = math.atan2(y - self.cy, x - self.cx)
        turned = self.sign * wrap(phi - self.start)
        r = math.hypot(x - self.cx, y - self.cy)
        return (self.radius * (abs(self.sweep) - turned), phi + self.sign * math.pi / 2,
                self.sign * (self.radius - r), self.sign / self.radius)


def rounded_path(corners, turn_radius):
    """Straight strips joined by arcs that cut each corner."""
    pieces, start = [], corners[0]
    for i in range(1, len(corners) - 1):
        before, corner, after = corners[i - 1], corners[i], corners[i + 1]
        heading_in = math.atan2(corner[1] - before[1], corner[0] - before[0])
        heading_out = math.atan2(after[1] - corner[1], after[0] - corner[0])
        turn = wrap(heading_out - heading_in)
        cut = turn_radius * math.tan(abs(turn) / 2)       # where the arc leaves the strip
        arc_start = (corner[0] - cut * math.cos(heading_in), corner[1] - cut * math.sin(heading_in))
        side = heading_in + math.copysign(math.pi / 2, turn)
        center = (arc_start[0] + turn_radius * math.cos(side), arc_start[1] + turn_radius * math.sin(side))
        if math.hypot(arc_start[0] - start[0], arc_start[1] - start[1]) > 0.01:
            pieces.append(Line(start, arc_start))
        pieces.append(Arc(center, turn_radius, side + math.pi, turn))
        start = (corner[0] + cut * math.cos(heading_out), corner[1] + cut * math.sin(heading_out))
    if math.hypot(corners[-1][0] - start[0], corners[-1][1] - start[1]) > 0.01:
        pieces.append(Line(start, corners[-1]))
    return pieces


def fit_turn_radius(corners, turn_radius):
    """Largest radius up to turn_radius whose arcs fit on the strips."""
    for i in range(1, len(corners) - 1):
        (ax, ay), (cx, cy), (bx, by) = corners[i - 1], corners[i], corners[i + 1]
        turn = abs(wrap(math.atan2(by - cy, bx - cx) - math.atan2(cy - ay, cx - ax)))
        shortest = min(math.hypot(cx - ax, cy - ay), math.hypot(bx - cx, by - cy))
        if turn > 1e-6:
            turn_radius = min(turn_radius, 0.5 * shortest / math.tan(turn / 2))
    return turn_radius


class ZigzagFollower:
    """Vicon pose in, (left, right) wheel commands out (-1..1).  No hardware here."""

    def __init__(self, speed, corners, turn_radius):
        self.speed = speed
        self.pieces = rounded_path(corners, turn_radius)
        self.piece = 0            # index of the strip or arc being driven
        self.track = deque()
        self.offset = None        # car heading minus Vicon yaw, learned while driving
        self.turning_on_spot = False
        self.carry = [0.0, 0.0]   # for wheels asked to go slower than they can (see wheel_commands)

    @property
    def done(self):
        return self.piece >= len(self.pieces)

    def learn_offset(self, t, x, y, yaw):
        """Compare the direction the car moves with the Vicon yaw."""
        self.track.append((t, x, y, yaw))
        while t - self.track[0][0] > HEADING_WINDOW_S:
            self.track.popleft()
        _, x0, y0, yaw0 = self.track[0]
        if self.turning_on_spot or math.hypot(x - x0, y - y0) < 0.03:
            return
        travel = math.atan2(y - y0, x - x0)
        sample = wrap(travel - (yaw0 + wrap(yaw - yaw0) / 2))   # yaw half-way along the chord
        if self.offset is None:
            self.offset = sample
        else:
            self.offset = wrap(self.offset + 0.05 * wrap(sample - self.offset))

    def step(self, t, x, y, yaw):
        self.learn_offset(t, x, y, yaw)
        if self.offset is None:
            return self.wheel_commands(self.speed, 0.0)      # drive straight until it knows
        left_on_piece, tangent, left_of_path, curvature = self.pieces[self.piece].reference(x, y)
        if left_on_piece <= 0:                                # on to the next strip or arc
            self.piece += 1
            if self.done:
                return 0.0, 0.0
            return self.step(t, x, y, yaw)
        aim = tangent - math.atan(APPROACH_GAIN * left_of_path)
        error = wrap(aim - (yaw + self.offset))

        if abs(error) > math.radians(TURN_ON_SPOT_ABOVE_DEG):
            self.turning_on_spot = True
        elif abs(error) < math.radians(TURN_ON_SPOT_UNTIL_DEG):
            self.turning_on_spot = False
        if self.turning_on_spot:
            self.track.clear()
            return (-MIN_CMD, MIN_CMD) if error > 0 else (MIN_CMD, -MIN_CMD)

        if left_on_piece < self.speed * TURN_EARLY_S and self.piece + 1 < len(self.pieces):
            curvature = self.pieces[self.piece + 1].reference(x, y)[3]
        omega = self.speed * curvature + TURN_GAIN * error
        omega = max(-MAX_TURN_RAD_S, min(MAX_TURN_RAD_S, omega))
        return self.wheel_commands(self.speed, omega)

    def wheel_commands(self, speed, omega):
        left, right = speed - omega * TRACK_M / 2, speed + omega * TRACK_M / 2
        fastest = SPEED_AT_CMD_0 + SPEED_PER_CMD
        if max(left, right) > fastest:                # too fast: scale both down
            left, right = left * fastest / max(left, right), right * fastest / max(left, right)
        return self.speed_to_cmd(left, 0), self.speed_to_cmd(right, 1)

    def speed_to_cmd(self, speed, wheel):
        """A wheel cannot hold a speed between 0 and its slowest speed.  For those,
        switch it between stopped and slowest from tick to tick, so it averages
        the right speed and the car keeps its speed in tight turns."""
        slowest = SPEED_AT_CMD_0 + SPEED_PER_CMD * MIN_CMD
        if speed >= slowest:
            self.carry[wheel] = 0.0
            return (speed - SPEED_AT_CMD_0) / SPEED_PER_CMD
        if speed <= 0:
            self.carry[wheel] = 0.0
            return 0.0
        self.carry[wheel] += speed / slowest
        if self.carry[wheel] >= 1.0:
            self.carry[wheel] -= 1.0
            return MIN_CMD
        return 0.0


def main():
    import Adafruit_PCA9685
    import RPi.GPIO as GPIO
    import rclpy
    from rclpy.node import Node
    from vicon_receiver.msg import PositionList

    config_path = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).with_name("zigzag_vicon_config.json")
    cfg = json.loads(config_path.read_text())
    if cfg["speed_m_s"] < SLOWEST_SPEED_M_S:
        print(f"not starting: speed_m_s {cfg['speed_m_s']} is below {SLOWEST_SPEED_M_S} m/s. Slower than this "
              f"the car only creeps in short jerks and cannot tell which way it faces.")
        return
    (xmin, xmax), (ymin, ymax) = cfg["safe_area"]["x"], cfg["safe_area"]["y"]
    margin = 0.15   # the car may run a little past a corner before it turns

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
            pwm.set_pwm(channel, 0, int(MIN_PWM + (MAX_PWM - MIN_PWM) * abs(cmd)) if cmd else 0)

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
    node = Node("zigzag_vicon")
    node.create_subscription(PositionList, cfg["vicon_topic"], on_vicon, 10)
    follower = None
    log_file, log, log_path = open_log("zigzag_vicon", cfg)
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
                if follower:
                    follower.track.clear()
                if not waiting:
                    print(f"no Vicon position for {cfg['robot']} on {cfg['vicon_topic']}: stopped, waiting")
                waiting = True
                log_row(log, now - start, "no_vicon")
                continue
            if waiting:
                print("Vicon position received, driving")
                waiting = False
            x, y = pose["x"], pose["y"]
            if follower is None:                  # first pose: plan the zigzag from here
                if not (xmin <= x <= xmax and ymin <= y <= ymax):
                    print(f"not starting: the car at ({x:.2f}, {y:.2f}) is outside safe_area")
                    break
                corners = zigzag_corners((x, y), cfg)
                if len(corners) < 2:
                    print("not starting: the first strip would leave safe_area; "
                          "change direction_deg or zigzag_angle_deg")
                    break
                print(f"{cfg['robot']}: zigzag of {len(corners) - 1} strips at {cfg['speed_m_s']} m/s, corners "
                      + " ".join(f"({cx:.2f}, {cy:.2f})" for cx, cy in corners))
                if len(corners) - 1 < cfg["strips"]:
                    print(f"  ({cfg['strips']} strips asked for; the rest would leave safe_area)")
                radius = fit_turn_radius(corners, cfg["turn_radius_m"])
                if radius < cfg["turn_radius_m"]:
                    print(f"  turn_radius_m {cfg['turn_radius_m']} does not fit on these strips, using {radius:.2f}")
                if radius < 0.3:
                    print("  warning: corners this tight are hard for the car; it may overshoot or turn on the spot. "
                          "Use longer strips or a smaller zigzag_angle_deg.")
                follower = ZigzagFollower(cfg["speed_m_s"], corners, radius)
            if not (xmin - margin <= x <= xmax + margin and ymin - margin <= y <= ymax + margin):
                print(f"stopped: the car at ({x:.2f}, {y:.2f}) left safe_area")
                log_row(log, now - start, "stopped_left_safe_area", pose, follower)
                break
            command = follower.step(now, x, y, pose["yaw"])
            if follower.done:
                print("zigzag finished")
                log_row(log, now - start, "finished", pose, follower)
                break
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
        if follower and follower.offset is not None:
            print(f"learned heading offset: {math.degrees(follower.offset):.0f} deg")
        print("stopped")


if __name__ == "__main__":
    main()

# osoyoo_paths

Standalone scripts that drive an Osoyoo car along a path. Each script has
its own JSON config.

| Script | Uses Vicon | What it does |
|---|---|---|
| `circle.py` | no | Drives a circle with fixed wheel speeds |
| `circle_vicon.py` | yes | Drives a circle around the lab origin (0, 0) and joins it from anywhere |
| `zigzag_vicon.py` | yes | Drives a zigzag with rounded corners, starting where the car is |

## Copy to the robot

From the laptop:

```bash
./deploy.sh                    # to pi@192.168.3.7, default
./deploy.sh pi@192.168.3.4     # to another robot
```
run this command so the robot's copy will be identical to this folder.

## Run on the robot

```bash
cd ~/osoyoo_paths
python3 circle.py
```

The Vicon scripts need ROS first:

```bash
source ~/ros2_dev/install/setup.sh
cd ~/osoyoo_paths
python3 circle_vicon.py        # or: python3 zigzag_vicon.py
```

Run them from `~/osoyoo_paths`. To use another
config file, pass it as an argument: `python3 circle.py my_config.json`.

Every script stops the car when `duration_s` runs out, on Ctrl+C, or when the
SSH session drops.

## Logs

Every run of `circle_vicon.py` or `zigzag_vicon.py` saves two files in
`~/osoyoo_paths/logs/` on the robot, named after the script, the robot and the
start time:

- `circle_vicon_osoyoo_5_20261105_122020.csv`: one row per control step (20 per second)
- `circle_vicon_osoyoo_5_20261105_122020.json`: a copy of the config used for that run

| Column | Meaning |
|---|---|
| `time_s` | seconds since the start of the run |
| `status` | `learning_heading`, `turning_on_spot`, `driving`, `no_vicon`, or why it stopped (`finished`, `stopped_too_far`, `stopped_left_safe_area`) |
| `x_m`, `y_m` | position from Vicon, lab metres (empty while `no_vicon`) |
| `vicon_yaw_rad` | orientation Vicon reports for the object |
| `heading_rad` | the way the car really faces: the Vicon yaw plus the learned offset (empty until learned) |
| `left_cmd`, `right_cmd` | wheel commands sent, -1 to 1 |

To copy the logs to the laptop:

```bash
./fetch_logs.sh                  # from pi@192.168.3.7
./fetch_logs.sh pi@192.168.3.4   # from another robot
```

They land in `logs/<robot address>/` in this folder. 

## Plotting a run (plot_log.py)

`plot_log.py` runs on the laptop and plots one log from either Vicon script:

```bash
cd ~/Documents/osoyoo_paths
~/Documents/CollectiPy/.venv/bin/python plot_log.py                      # the newest log
~/Documents/CollectiPy/.venv/bin/python plot_log.py logs/192.168.3.7/<run>.csv
```

It needs matplotlib, which the system Python on this laptop can't load, so use
the CollectiPy venv as above. It reads the `.csv` and the `.json` with the same
name.

It saves `<run>.png` next to the log.

- **Solid blue:** where the car really went, from Vicon. The line breaks
  where Vicon lost the car.
- **Dashed grey:** the planned path.
- **Circle and square:** where the car started and stopped.

## circle.py

```json
{"speed_m_s": 0.5, "diameter_m": 1.5, "duration_s": 30}
```

No Vicon: it sets the two wheels to fixed speeds and the car circles
counter-clockwise from wherever it is. The start point is on the circle, and the
centre is half the diameter to the car's left. Nothing corrects the car, so the
circle slowly wanders and its real size can differ from `diameter_m`.

## circle_vicon.py

```json
{"robot": "osoyoo_5", "vicon_topic": "vicon/default/data",
 "speed_m_s": 0.5, "diameter_m": 1.5, "duration_s": 60}
```

- `robot`: the Vicon object name, which must match Vicon Tracker exactly.
- `vicon_topic`: the topic the Vicon bridge publishes on.

The circle is always around (0, 0), counter-clockwise. Start the car anywhere
within about 0.75 m of (0, 0), facing any way, with about 10 cm of free floor in
front of it. It drives straight briefly to work out which way it faces, so there
is no heading offset to set. Then it turns on the spot if it faces far away from
the circle, and joins it.

It also stops if the car gets more than 1.0 m from (0, 0). If Vicon loses the
car, the motors stop and the car drives on when tracking returns.

## zigzag_vicon.py

```json
{"robot": "osoyoo_5", "vicon_topic": "vicon/default/data", "speed_m_s": 0.5,
 "direction_deg": 0, "zigzag_angle_deg": 45, "strip_length_m": 0.8, "strips": 6,
 "turn_radius_m": 0.4, "safe_area": {"x": [-1.2, 1.2], "y": [-0.8, 0.8]},
 "duration_s": 60}
```

- `direction_deg`: the overall direction the zigzag advances, in lab degrees
  (0 = +x, 90 = +y).
- `zigzag_angle_deg`: how far each strip leans from that direction. A positive
  value makes the first strip go left, a negative one right.
- `strip_length_m`: the distance from one corner to the next.
- `strips`: how many strips to drive.
- `turn_radius_m`: how round each corner is. If it doesn't fit on the strips,
  the script uses the largest radius that does and says so.
- `safe_area`: a box in lab metres. Corners outside it are dropped, and the car
  stops if it goes more than 15 cm past it.

The zigzag starts where the car is. Arcs under 0.3 m are tighter than the car drives well, and the script warns about them. [zigzag_parameters.png](zigzag_parameters.png) shows how each zigzag setting in the JSON config changes the shape of the path.

## Speed limits

- 0.35 m/s or more drives smoothly. DOn't make the speed to low. 
- `zigzag_vicon.py` refuses to start below 0.15 m/s. Even with 0.15 m/s the motion of the car doesn't look smooth.



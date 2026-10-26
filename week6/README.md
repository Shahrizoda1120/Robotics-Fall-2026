# Week 6: Odometry localization

**Goal:** drive the digit **32** (the last two digits of my student ID) in the TRIK Studio simulator and on the real robot, let the robot estimate its own position from its wheel encoders (odometry), and compare the two runs in Colab.

## Files in this folder

| File | What it is |
|---|---|
| `trajectory_sim.qrs` | TRIK Studio program used for the simulator run |
| `trajectory_real.qrs` | The same program, used on the real robot |
| `odometry_data_sim_32.txt` | Odometry log of the simulator run |
| `odometry_data_real_32.txt` | Odometry log of the real robot run |
| `lab6_odometry.ipynb` | Colab notebook: both trajectories on one plot, and the error between them over time |

## 1. The idea

The robot never sees the world. It only counts how far each wheel has turned and adds these small steps up into a position `(x, y)` and a heading `φ`. This is odometry, and it is the robot's **belief** about where it is, not the truth.

Every sample, the program uses how much each wheel turned since the last sample (ΔθL, ΔθR, encoder degrees converted to radians):

| Quantity | Formula |
|---|---|
| Wheel travel | `dL = R · ΔθL`, `dR = R · ΔθR` |
| Robot travel | `d = (dL + dR) / 2` |
| Heading change | `Δφ = (dR − dL) / B` |
| New position | `x ← x + d · cos(φ + Δφ/2)`, `y ← y + d · sin(φ + Δφ/2)`, then `φ ← φ + Δφ` |

- `R` is the wheel radius and `B` the distance between the wheels.
- E3 is the left wheel (M3), E4 the right wheel (M4).
- `x` points in the starting direction, `y` to the left of it, and `(0, 0)` is the start point.
- This is the formula table of the Lab 6 README before its Oct 19 update. The current table updates the position with `cos(φ)` instead of `cos(φ + Δφ/2)`. The heading φ is the same in both.

## 2. How the program works

The program has two threads, as in the course demo `Localization_demo.qrs`:

1. **Main thread:** sets the constants, deletes the old log file, starts the odometry thread, drives the 20 moves of the digit 32 with `forward(distance)` and `turn(angle)`, waits 0.5 s and stops the odometry thread.
2. **Odometry thread:** reads both encoders, updates `x` and `y` with the formulas above, writes one line to the log, waits 20 ms, and repeats.

`forward` and `turn` are encoder-based, as in Lab 2: each wheel gets a target encoder angle and its own PID drives it there (Kp = 1.5, Ki = 0.5, Kd = 0.15, output limited to ±50 % power). A move ends when both wheels are within 2° of their target and have almost stopped.

**Simulator vs real program.** Both files have the same blocks. Only these differ:

| | `trajectory_sim.qrs` | `trajectory_real.qrs` |
|---|---|---|
| `R`, `B` (`radius`, `track` in the code) | 0.028 m, 0.154 m (simulator robot) | 0.043 m, 0.196 m (real robot, measured in week 2) |
| Log file | `odometry_data_sim_32.txt` | `odometry_data_real_32.txt` |
| Marker Down / Marker Up | yes, so the simulator draws the path | no |

The real program is uploaded with **Generation (Java Script)**, because Python generation cannot run the odometry thread.

## 3. How the data is organized

Each log has one line per sample, no header: `x,y,time`, with `x` and `y` in m and `time` in ms since the program started.

```
0,0,687
0,0,748
0.0015009831567151233,0,802
...
```

For example, `0.0015009831567151233,0,802` means: 802 ms after the start, the robot believed it was 1.5 mm ahead of the start point.

- Simulator log: 2528 lines, one every 20 ms.
- Real robot log: 1508 lines, about one every 57 ms (writing to the file takes time on the robot).
- `_32` is added to the file names so they do not clash with classmates' logs on the shared robot.

## 4. What the notebook does

The notebook is the Colab analysis from the Lab 6 README:

1. **Load** both logs.
2. **Trajectories:** plot the simulator and real robot paths on one plot.
3. **Error over time:** resample the simulator log to the real robot's timestamps, plot the distance between the two runs, and print the final and the maximum error.

**How to run it:** open the notebook in Colab, choose *Runtime > Run all*, and upload both txt files when asked.

## 5. Results

| | x, m | y, m | Run time |
|---|---|---|---|
| Simulator, end | 0.503 | -0.066 | 50.5 s |
| Real robot (odometry), end | 0.504 | 0.032 | 85.7 s |

- **Final odometry error (sim vs real):** 0.098 m (printed as 0.1 m in the notebook).
- **Max error:** 0.987 m at 43.6 s (printed as 1.0 m). This error compares the two runs at the same time after the start. The real run is 35 s slower, so most of it comes from timing, not from a different path.

## Discussion

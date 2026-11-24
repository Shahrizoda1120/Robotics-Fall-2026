# Week 10: Odometry and gyroscope localization

**Goal:** drive the digit **32** twice with the same program and the same controller: run A with the course from the encoders (odometry only), run B with the course from the gyroscope (odometry + gyro). Then compare the course and the path of every segment with the ideal digit, and find out which localization keeps the robot closer to it.

## Files in this folder

| File | What it is |
|---|---|
| `lab10_odometry_gyro.qrs` | TRIK program (JavaScript), the same for both runs. `MODE` in the first block chooses where the course comes from: `MODE = 1` for run A, `MODE = 2` for run B. |
| `lab10_run_A.txt` | Log of run A (`MODE = 1`). Course from the **encoders**: `course = course_odom`, the difference between the two wheels. |
| `lab10_run_B.txt` | Log of run B (`MODE = 2`). Course from the **gyroscope**: `course = course + gyro_inc`, the gyro's turning rate added up every cycle. |
| `lab10_task_odometry_gyro.ipynb` | Colab notebook: plots, the offline recomputation and the comparison table |

## 1. Idea of the experiment

The robot cannot see the floor. In every cycle it estimates its pose `x, y, course` from its own sensors, and the controller steers this estimate onto the next waypoint. If the estimate is wrong, the robot still believes it is on the digit while it is somewhere else.

The course matters most. A course error `e` moves the robot sideways by about `l · e` after a straight segment of length `l`: 5° over 30 cm is already 2.6 cm, and every later segment starts from the wrong place.

The two runs differ only in where the course comes from:

| | Run A: odometry only | Run B: odometry + gyro |
|---|---|---|
| `MODE` | 1 | 2 |
| `course` from | encoders: `course = course_odom` | gyro: `course = course + gyro_inc` |
| `path` from | encoders | encoders |


The digit, start pose, controller and gains are the same in both runs, so any difference between A and B comes from the course.

## 2. Program

`lab10_odometry_gyro.qrs` is built on `Base_observer.qrs` and uses the README formulas.

### Localization, every cycle

```
theta_l = encoder4 * pi / 180;                  -- left wheel angle, rad (E4)
theta_r = encoder3 * pi / 180;                  -- right wheel angle, rad (E3)
path = (theta_r + theta_l) * r / 2;             -- distance driven by the robot centre
course_odom = (theta_r - theta_l) * r / B;      -- course from the wheels
gyro_inc = (gz - b0) / 1000 / (1 + s) * (gt - prev_gt) * 0.000256 * pi / 180;   -- course change from the gyro, rad
course = (2 - MODE) * course_odom + (MODE - 1) * (course + gyro_inc);
x = x + (path - prev_path) * cos(course);
y = y + (path - prev_path) * sin(course);
```

`gz` is the gyro rate in mdeg/s, so `/ 1000` gives deg/s. `gt` is the gyro's own timestamp, in units of 256 µs, so `(gt - prev_gt) * 0.000256` is the time between two readings in seconds.

**`MODE`** is set in the first block before each run. The program computes both courses in every cycle and keeps one with a single line, so nothing else changes between the runs:

| `MODE` | Run | The course line becomes |
|---|---|---|
| 1 | A | `course = 1 * course_odom + 0 * (course + gyro_inc) = course_odom` |
| 2 | B | `course = 0 * course_odom + 1 * (course + gyro_inc) = course + gyro_inc` |

**Parameters**

| Parameter | Value | Where it comes from |
|---|---|---|
| `r` | 4.3 cm | wheel radius, as in Lab 9 |
| `B` | 19.6 cm | wheel base, as in Lab 9 |
| `s` | −4.7 % | gyro scale factor, Lab 9 |
| `b0` | measured at every start | mean gyro rate over 10 s at rest |

### Gyro calibration

Before driving, the robot stands still for 10 s and averages `gz` (about 370 readings). The average is `b0`, the gyro's output when the robot is not turning. It is measured at every start because it changes from start to start (0.531 and 0.440 deg/s here, 0.26 deg/s in Lab 9).

### Controller: turn, then drive

The program uses the professor's go-to-goal block for the target `i`:

```
distance = sqrt((X_ref[i] - x)^2 + (Y_ref[i] - y)^2);
bearing = atan2(Y_ref[i] - y, X_ref[i] - x);
courseAngle = bearing - course;                 -- wrapped to [-pi, pi]
```

1. **Turn in place.** The wheels turn in opposite directions with `turn = sgn(courseAngle) * min(17 + 15 * |courseAngle|, 22)` % power, until |`courseAngle`| < 0.05 rad (2.9°).
2. **Drive.** Left motor `(0.3 * distance + 20) * cos(courseAngle) - 25 * sin(courseAngle)`, right motor the same with `+`. The robot slows down near the waypoint and steers to keep `courseAngle` near 0.
3. **Next waypoint.** When `distance * cos(courseAngle)` ≤ 1 cm (the waypoint is less than 1 cm ahead along the robot's heading), `i` goes to the next waypoint and the robot turns again.

The fastest turn was 91 deg/s, far below the gyro range of 287 deg/s (Lab 9), so the gyro never saturated.

### Log

One line per control cycle, written to the file at the end of each segment:

| Column | Meaning | Unit |
|---|---|---|
| `t` | robot time, `time()` | ms |
| `i` | target waypoint, 0 to 11; −1 during the 10 s at rest | |
| `theta_l`, `theta_r` | left (E4) and right (E3) wheel angle | rad |
| `gz` | gyro rate around the vertical axis | mdeg/s |
| `gt` | gyro timestamp | 256 µs |
| `x`, `y` | estimated position | cm |
| `course` | estimated course, the one the robot steered by | rad |

The raw sensors are logged together with the estimate, so the notebook can recompute both methods from both runs.

## 3. Ideal trajectory

12 straight segments from the start (0, 0), facing x, in cm:

(0, 0) → (30, 0) → (30, 30) → (0, 30) → (30, 30) → (30, 60) → (0, 60) → (50, 60) → (80, 60) → (80, 30) → (50, 30) → (50, 0) → (80, 0)

Segments 1 to 6 draw the 3, segment 7 goes back along its top line to the 2, and segments 8 to 12 draw the 2. In the program: `X_ref = {30, 30, 0, 30, 30, 0, 50, 80, 80, 50, 50, 80}`, `Y_ref = {0, 30, 30, 30, 60, 60, 60, 60, 30, 30, 0, 0}`.

## 4. Procedure

1. Place the robot on the taped start mark at (0, 0), facing x.
2. Set `MODE = 1` and start the program. The robot stands still for 10 s (calibration), then drives the 12 segments. The program deletes the old log at the start and writes `lab10_32.txt`.
3. Copy the log from the robot and rename it `lab10_run_A.txt`.
4. Put the robot back on the start mark, set `MODE = 2` and repeat. This log is `lab10_run_B.txt`.

## 5. Runs

Both runs passed all 12 waypoints.

| | Run A | Run B |
|---|---|---|
| Driving time | 47 s | 52 s |
| Control cycle | 60 ms | 60 ms |
| `b0` | 0.531 deg/s | 0.440 deg/s |
| Fastest turn | 91 deg/s | 80 deg/s |
| Robot vs. offline recomputation | 0.000 cm, 0.0000° | 0.001 cm, 0.0004° |

In run B the robot turned slowly at (80, 30), about 5 deg/s for 5 s, under the same motor command as at the other corners. The encoders and the gyro agree on that turn (24.1° both), so the wheels did not slip, and the run went on normally.

## 6. Offline recomputation

The notebook repeats the robot's formulas from the raw columns `theta_l, theta_r, gz, gt` and computes both methods for both runs, which gives 4 paths. The method each run used reproduces its log exactly (table above), so the logs and the program agree.

The two other paths show what the other sensor would have said:

| | Course from the encoders | Course from the gyro |
|---|---|---|
| Run A ends at (cm) | (79.3, 0.5), logged | (73.4, −4.8) |
| Run B ends at (cm) | (85.6, 5.6) |

**Conclusion.** The gyro (run B) gives the better course. In the logs run B is slightly closer to the ideal digit, the gyro is not affected by wheel slip or `B`, its bias after the 10 s calibration adds only about 0.2° per run, and it never saturated. The encoders see only the wheels, and in the turns they differ from the gyro by about 3 %. The margin is small, and in longer runs the gyro bias would drift, so the best course combines both sensors (Lab 11).

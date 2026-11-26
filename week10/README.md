# Week 10: Odometry and gyroscope localization

**Goal:** drive the digit **32** twice with the same program and controller, once with the course from the encoders (run A, odometry only) and once with the course from the gyroscope (run B, odometry + gyro), and compare both runs with the ideal digit.

## Files

| File | What it is |
|---|---|
| `lab10_odometry_gyro.qrs` | TRIK program (JavaScript), the same for both runs: `MODE = 1` for run A, `MODE = 2` for run B |
| `lab10_run_A.txt` | Log of run A, course from the encoders: `course = course_odom` |
| `lab10_run_B.txt` | Log of run B, course from the gyro: `course = course + gyro_inc` |
| `lab10_task_odometry_gyro.ipynb` | Colab notebook with the plots, the offline recomputation and the comparison table |

## 1. Method

The program is `Base_observer.qrs` with the README formulas, using r = 4.3 cm, B = 19.6 cm and the gyro scale factor s = −4.7 % from Lab 9. In every cycle it computes both courses, and `MODE` decides which one the robot uses:

```
theta_l = encoder4 * pi / 180;                  -- left wheel angle, rad (E4)
theta_r = encoder3 * pi / 180;                  -- right wheel angle, rad (E3)
path = (theta_r + theta_l) * r / 2;             -- distance driven by the robot centre, cm
course_odom = (theta_r - theta_l) * r / B;      -- course from the encoders, rad
gyro_inc = (gz - b0) / 1000 / (1 + s) * (gt - prev_gt) * 0.000256 * pi / 180;   -- course change from the gyro in one cycle, rad
course = (2 - MODE) * course_odom + (MODE - 1) * (course + gyro_inc);       -- course the robot uses
x = x + (path - prev_path) * cos(course);       -- position, cm
y = y + (path - prev_path) * sin(course);
```

With `MODE = 1` the course line becomes `course = course_odom`, and with `MODE = 2` it becomes `course = course + gyro_inc`. Here `gz` is the gyro rate in mdeg/s, `gt` is the gyro timestamp in units of 256 µs, and `b0` is the mean `gz` over 10 s at rest before each run.

The robot turns in place until it faces the next waypoint (within 0.05 rad) and then drives to it. The waypoints, in cm, are (0, 0) → (30, 0) → (30, 30) → (0, 30) → (30, 30) → (30, 60) → (0, 60) → (50, 60) → (80, 60) → (80, 30) → (50, 30) → (50, 0) → (80, 0). Each log line is `t, i, theta_l, theta_r, gz, gt, x, y, course`, so it holds both the raw sensors and the robot's estimate; lines with `i = -1` are the 10 s at rest.

## 2. Runs

| | Run A | Run B |
|---|---|---|
| Control cycle | 60 ms | 60 ms |
| `b0` | 0.531 deg/s | 0.440 deg/s |
| Fastest turn (gyro range 287 deg/s) | 91 deg/s | 80 deg/s |
| Robot vs. offline recomputation | 0.000 cm, 0.0000° | 0.001 cm, 0.0004° |

Both runs passed all 12 waypoints. In run B the robot turned slowly at (80, 30) for about 5 s, but the encoders and the gyro agree on that turn, so the wheels did not slip.

## 3. Offline recomputation

The notebook repeats the robot's formulas on the raw sensors of each log with both methods. The method each run used gives back its logged path exactly (table above), and the other method shows what the other sensor would have said:

| | Course from the encoders | Course from the gyro |
|---|---|---|
| Run A ends at (cm) | (79.3, 0.5), logged | (73.4, −4.8) |
| Run B ends at (cm) | (85.6, 5.6) | (79.1, 0.8), logged |

## 4. Comparison table

`course` is the mean logged course on the straight part of the segment, without the turn in place, and `path` is the logged path from the end of the previous segment to the end of this one.

| Seg. | `course_ref` (°) | `course` A (°) | `course` B (°) | `d_course` A / B (°) | `path_ref` (cm) | `path` A (cm) | `path` B (cm) | `d_path` A / B (cm) |
|---|---|---|---|---|---|---|---|---|
| 1 | 0 | 0.6 | 2.1 | +0.6 / +2.1 | 30 | 29.9 | 29.7 | −0.1 / −0.3 |
| 2 | 90 | 101.9 | 100.0 | +11.9 / +10.0 | 30 | 33.3 | 31.0 | +3.3 / +1.0 |
| 3 | 180 | 187.5 | 188.9 | +7.5 / +8.9 | 30 | 30.4 | 31.9 | +0.4 / +1.9 |
| 4 | 0 | −0.4 | 2.6 | −0.4 / +2.6 | 30 | 34.4 | 34.0 | +4.4 / +4.0 |
| 5 | 90 | 99.6 | 97.3 | +9.6 / +7.3 | 30 | 33.1 | 31.6 | +3.1 / +1.6 |
| 6 | 180 | 188.2 | 188.4 | +8.2 / +8.4 | 30 | 31.7 | 31.9 | +1.7 / +1.9 |
| 7 | 0 | −1.6 | −0.1 | −1.6 / −0.1 | 50 | 57.0 | 55.2 | +7.0 / +5.2 |
| 8 | 0 | 0.7 | 1.3 | +0.7 / +1.3 | 30 | 30.5 | 30.5 | +0.5 / +0.5 |
| 9 | −90 | −95.0 | −96.0 | −5.0 / −6.0 | 30 | 32.8 | 32.8 | +2.8 / +2.8 |
| 10 | 180 | 169.6 | 174.4 | −10.4 / −5.6 | 30 | 34.4 | 34.4 | +4.4 / +4.4 |
| 11 | −90 | −77.1 | −75.6 | +12.9 / +14.4 | 30 | 34.2 | 32.5 | +4.2 / +2.5 |
| 12 | 0 | 10.3 | 8.5 | +10.3 / +8.5 | 30 | 31.9 | 30.4 | +1.9 / +0.4 |
| **Summary** | | | | **6.6 / 6.3** | **380** | **413.7** | **406.0** | **+33.7 / +26.0, +8.9 / +6.8 %** |

From the logs, relative to the last waypoint (80, 0) and its course 0°, the final position error is 0.9 / 1.2 cm and the final course error is +1.3 / −6.2° (A / B).

## 5. Analysis

**Which run is closer to the ideal trajectory in course, and which in path?** Run B is closer in both, by a small margin: mean abs(`d_course`) 6.3° against 6.6°, and total path error +6.8 % against +8.9 %.

**Why are the path errors similar while the course errors are not?** Both runs measure the path with the encoders, so their path errors are similar. The course comes from two different sensors, and the recomputation shows how much they differ: with the other sensor, each run ends about 8 cm away from its logged end. In the logs themselves the course errors are close, because each run steers its own course back onto every waypoint.

**Which error source dominated each run?** Not the gyro bias, which adds only about 0.2° per run after the 10 s calibration, and not saturation, since the fastest turn was 91 deg/s and the gyro range is 287 deg/s. The two sensors agree on the straights (within 3°) but differ in the turns, where the encoders measure about 3 % more rotation than the raw gyro. So the error comes from the turns: for the encoders (run A) that means `B` or wheel slip, and for the gyro (run B) the scale factor `s`. The logs alone cannot show which sensor is right.

**Does the error accumulate or stay bounded?** It stays bounded: `d_course` stays between −10° and +14° without a trend. The distance to the ideal segment is largest after the turn into segment 10, 5.3 cm in run A and 4.3 cm in run B.

**How would you combine `course_odom` and `gyro_inc`?** With the complementary filter of Lab 11, `course = K * (course + gyro_inc) + (1 - K) * course_odom`, which follows the gyro and slowly corrects it with the encoders.

**Conclusion.** Run B, with the course from the gyro, came out slightly closer to the ideal digit than run A, but the difference between the two runs is small.

## 6. Differences from `Base_observer.qrs`

| Change | Why |
|---|---|
| Stop when `distance * cos(courseAngle)` ≤ 1 cm, not `distance` < 5 cm | A 5 cm circle starts each turn early and tilts the next segment, while a 1 cm circle is missed at 60 ms per cycle. |
| Steering `25 * sin(courseAngle)`, not `50 * courseAngle` | The cycle is 60 ms instead of ~10 ms, and `sin` stays bounded near a waypoint. |
| Turn in place first | Turn-then-drive, as the README recommends. |

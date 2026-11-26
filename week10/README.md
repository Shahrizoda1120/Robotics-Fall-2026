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

**Which run is closer to the ideal trajectory in course, and which in path?** Run B is closer in both, but only by a small margin: its mean abs(`d_course`) is 6.3° against 6.6° for run A, and its total path error is +6.8 % against +8.9 %. Both runs end within 1.2 cm of (80, 0). These numbers come from each run's own estimate, so they mainly show how well the same controller followed that estimate.

**Why are the path errors similar while the course errors are not?** The path comes from the encoders in both runs, so it carries the same errors in both, and the extra 7 to 9 % is real wheel travel from rolling past each waypoint and from the centre moving during the turns. The course comes from two different sensors. In the logs the course errors still look alike, because both are dominated by the same overshoot of 10 to 15° after each turn, but the recomputation shows the real difference: with the other sensor, each run ends about 8 cm away from where it logged.

**Which error source dominated each run?** The bias did not, because after the 10 s average `b0` is known to 0.004 deg/s, which adds only about 0.2° per run, and saturation did not either, because the fastest turn was 91 deg/s against a gyro range of 287 deg/s. The recomputation shows that the two sensors disagree in the turns, where the encoders measure 3.3 % (run A) and 3.4 % (run B) more rotation than the raw gyro, while on the straights they agree within 3°. The dominant error therefore comes from the turns: `B` or wheel slip for the encoder course of run A, and the scale factor `s` for the gyro course of run B. The logs alone cannot tell which of the two sensors is right.

**Does the error accumulate or stay bounded?** `d_course` stays between −10° and +14° with no trend, because each run steers back onto every waypoint. The distance to the ideal segment stays under 4 cm up to segment 9 and jumps after the turn into segment 10, to 5.3 cm in run A and 4.3 cm in run B.

**How would you combine `course_odom` and `gyro_inc`?** The gyro is better over short times, because slip and `B` do not affect it, and the encoders are better over long times, because they have no bias drift. A complementary filter uses both: `course = K * (course + gyro_inc) + (1 - K) * course_odom` follows the gyro and slowly pulls the course toward `course_odom`. With the 60 ms cycle, `K = 0.98` gives a time constant of about 3 s. This is Lab 11.

**Conclusion.** The gyro gives a slightly better course than the encoders, because slip and `B` do not affect it, but the margin is small and its bias drifts over longer runs, so combining both sensors is best.


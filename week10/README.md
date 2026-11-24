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
| What can make the course wrong | wrong `r` or `B`, unequal wheels, wheel slip in turns | gyro bias `b0`, scale factor `s`, saturation |

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
| Run B ends at (cm) | (85.6, 5.6) | (79.1, 0.8), logged |

The two methods end about 8 cm apart in both runs. In the notebook's trajectory plot the dashed paths move a little further from the logged ones at every turn, so the gap is largest on the 2.

## 7. Comparison table

- `course_ref`, `path_ref`: direction and length of the ideal segment.
- `course`: the logged course, averaged over the straight part of the segment. The straight is the longest stretch of cycles where both wheels turn forward, so the turn in place, the rolling past the previous waypoint and the last steering onto the waypoint are left out.
- `path`: the logged path from the end of the previous segment to the end of this one.
- `d_course = course - course_ref`, `d_path = path - path_ref`.

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

Relative to the last waypoint (80, 0) and its course 0°, from the logs: final position error 0.9 / 1.2 cm (A / B), final course error +1.3 / −6.2°. Both are the robot's own estimate when it stopped, after its last steering onto (80, 0).

## 8. Analysis

**Which run is closer to the ideal digit, in course and in path?** In the logs, run B, by a small margin in both: mean abs(`d_course`) 6.3° vs 6.6°, total path +6.8 % vs +8.9 %, and both runs end within 1.2 cm of (80, 0). These are each run's own estimates, so they mostly show how well the same controller followed its estimate. Which estimate matched the real floor can only be decided by measuring the real end position.

**Why are the path errors similar while the course errors are not?** `path` comes from the encoders in both runs, so it has the same errors. The extra 7 to 9 % is real wheel travel: the robot rolls on past each waypoint before the turn starts, and its centre moves during the turns because the two wheels never turn at exactly the same speed. The straights themselves are 25 to 30 cm. The course comes from two different sensors. The logged `d_course` values look alike because the same turn overshoot dominates them: after each turn the robot keeps turning 10 to 15° past the target (one cycle is 60 ms) and corrects slowly, so `d_course` is positive after left turns and negative after right turns. The real difference between the sensors shows in the recomputation (Section 6): the two course estimates drift up to 10° apart during a run, and their end positions are 8 cm apart.

**Which error source dominated each run?** The notebook splits the rotation seen by each sensor:

| Source | Evidence | Verdict |
|---|---|---|
| Bias `b0` | averaged over 10 s, `b0` is known to 0.004 deg/s, about 0.2° per run | too small |
| Saturation | fastest turn 91 deg/s, range 287 deg/s | none |
| `B`, `s` or slip in the turns | in every turn the encoders measure more rotation than the raw gyro: +3.4 % left and +3.2 % right in run A, +3.2 % and +3.7 % in run B | **dominant** |
| Unequal wheels | on the straights the difference is +2.1° in run A and −2.6° in run B, opposite signs | not visible |

The same 3 to 4 % in every turn, left and right, in both runs, is a systematic error, not random slip. It comes from `B`, from `s`, or from slip that repeats in every turn, and the logs cannot separate them: `B = 19.6` cm with a gyro that reads 3.3 % low fits as well as `s = −4.7` % with `B ≈ 19.3` cm. `B_hat` (19.95 cm in run A, 19.89 cm in run B, from `Base_observer.qrs`) is the base that would make the encoders agree with the raw gyro, and with it the difference halves. In Lab 9 the gyro read 4.7 % less than the encoders; here it reads about 3.3 % less. `B` is fixed and Lab 9 found `s` repeatable within 1 percentage point, so this change points to slip, which depends on the floor and the turn speed. So run A (encoders) was hurt mainly by `B` or slip in the turns, and run B (gyro) by `s`, because Lab 9's `s` was itself measured against the encoders. A turn checked against a floor mark would settle which one is right.

**Does the error accumulate or stay bounded?** `d_course` stays bounded, −10 to +14°, with no trend from segment to segment, because each run steers its own course back onto every waypoint. The distance to the ideal segment stays below 4 cm (A) and 3 cm (B) up to segment 9 and jumps after the turn into segment 10, to 5.3 cm (A) and 4.3 cm (B). The disagreement between the two sensors does accumulate: it changes at every turn, and nothing on the robot corrects it.

**How to combine `course_odom` and `gyro_inc`?** The gyro is better over short times: it is not affected by slip or `B`, and its leftover bias adds almost nothing during one turn. The encoders are better over long times: they have no bias drift. So the course should follow the gyro and be pulled slowly toward `course_odom`, which is the Lab 11 complementary filter:

```
course = K * (course + gyro_inc) + (1 - K) * course_odom
```

`K` close to 1 trusts the gyro more. The time constant is `K * dt / (1 - K)`: with the 60 ms cycle, `K = 0.98` gives about 3 s, so turns come from the gyro and slow drift is removed by the encoders.

## 9. Differences from `Base_observer.qrs`

| Change | Why |
|---|---|
| Stop when `distance * cos(courseAngle)` ≤ 1 cm, not `distance` < 5 cm | A 5 cm circle starts each turn early and tilts the next segment by up to 9.5°. A 1 cm circle is missed at 60 ms per cycle. |
| Steering `25 * sin(courseAngle)`, not `50 * courseAngle` | The cycle is 60 ms, not ~10 ms. `sin` stays bounded near a waypoint, where `courseAngle` passes 50°. |
| Turn in place first, 17 to 22 % power | Turn-then-drive, as the README recommends |
| Log lines written to the file once per segment | Same lines, fewer file writes |
| `i, gt, x, y, course` added to the log line | The README asks for `i` and the estimate. `gt` gives `dt`. |

# Week 10: Odometry and gyroscope localization

**Goal:** drive the digit **32** twice with the same program and controller, once with the course from the encoders (run A) and once with the course from the gyro (run B), and compare the course and path of every segment with the ideal digit.

## Files in this folder

| File | What it is |
|---|---|
| `lab10_odometry_gyro.qrs` | TRIK program (JavaScript): `MODE = 1` for run A, `MODE = 2` for run B |
| `lab10_run_A.txt` | Log, run A |
| `lab10_run_B.txt` | Log, run B |
| `lab10_odometry_gyro_240032.ipynb` | Colab notebook: Step 4 plots and the comparison table |

## 1. Method

**Ideal trajectory.** 12 straight segments from the start (0, 0), facing x: `X_ref = {30, 30, 0, 30, 30, 0, 50, 80, 80, 50, 50, 80}`, `Y_ref = {0, 30, 30, 30, 60, 60, 60, 60, 30, 30, 0, 0}` (cm).

**Program.** `Base_observer.qrs` with the README formulas: E4 = left wheel, E3 = right wheel, r = 4.3 cm, B = 19.6 cm. Only the course line depends on `MODE`:

```
gyro_inc = (gz - b0) / 1000 / (1 + s) * (gt - prev_gt) * 0.000256 * pi / 180;
course = (2 - MODE) * course_odom + (MODE - 1) * (course + gyro_inc);
```

`gz` is in mdeg/s, `gt` is the gyro timestamp in 256 µs units, s = −4.7 % (Lab 9), and `b0` is the mean of 10 s at rest. The robot turns in place until |`courseAngle`| < 0.05 rad, then drives to the waypoint with the go-to-goal controller.

**Log.** One line per control cycle: `t, i, theta_l, theta_r, gz, gt, x, y, course`. Lines with `i = -1` are the 10 s at rest, so both methods can be recomputed offline from both runs.

## 2. Runs

Both runs started from the same taped start pose and passed all 12 waypoints.

| | Run A | Run B |
|---|---|---|
| Duration | 47 s | 52 s |
| Control cycle | 60 ms | 60 ms |
| `b0` | 0.531 deg/s | 0.440 deg/s |
| Fastest turn | 91 deg/s | 80 deg/s |
| Robot vs. offline recomputation | 0.000 cm, 0.0000° | 0.001 cm, 0.0004° |

The fastest turn is far below the gyro range (287 deg/s), so the gyro never saturated. In run B the robot turned slowly at (80, 30), about 5 deg/s for 5 s, under the same motor command as at the other corners. The encoders and the gyro agree on that turn (24.1° both), so nothing slipped, and the run went on normally.

## 3. Comparison table

`course`: mean course on the straight of the segment, without the turn in place. `path`: path travelled while heading to waypoint `i`.

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

Final position error, from the logs: 0.9 / 1.2 cm (A / B). Final course error: +1.3 / −6.2°.

## 4. Analysis

**Course and path.** In the logs, run B is slightly closer to the ideal digit in both: mean abs(`d_course`) 6.3° vs 6.6°, and path +6.8 % vs +8.9 %. The two runs look alike because each one steers its own estimate onto the digit, so the logged errors come from the same controller. After every turn the robot keeps turning 10 to 15° past the target (one cycle is 60 ms) and then corrects slowly, so `d_course` is positive after left turns and negative after right turns. The path errors are similar because `path` comes from the encoders in both runs. The extra 7 to 9 % is the robot rolling past each waypoint and its centre moving during the turns. The straights themselves are 25 to 30 cm.

The course errors differ between the methods, not between the logs. Recomputed with the other method, each run ends about 8 cm away: run A at (79.3, 0.5) with `course_odom` and (73.4, −4.8) with the gyro, run B at (79.1, 0.8) with the gyro and (85.6, 5.6) with `course_odom`. The logs alone cannot say which one is right. That needs the real end position, measured on the floor.

**Error source.** `course_odom` minus the gyro course steps up at every left turn and down at every right turn: between −4 and +10° in run A, and between −6 and +8° in run B. An error that follows each turn comes from `B` or the gyro scale factor `s`, and slip during the turns looks the same. With `B_hat` = 19.95 / 19.89 cm instead of 19.6 cm the spread halves, so `B` or `s` explains most of it. Only a known reference turn can tell them apart. The bias is not the cause: after 10 s at rest, `b0` is known to 0.004 deg/s, about 0.2° per run. At the end the net turn is 0, so `B` and `s` cancel, and 4.8° (A) and 3.1° (B) are left. That part is slip or a bias change while driving. So run A was hurt mainly by `B` or slip in the turns, and run B by `s`.

**Error growth.** `d_course` stays bounded (−10 to +14°) and does not grow from segment to segment. The distance to the ideal segment stays below 4 cm (A) and 3 cm (B) up to segment 9, then jumps after the turn into segment 10, to 5.3 cm (A) and 4.3 cm (B).

**Combining both.** The gyro is right in the short term (no slip, no `B`), and the encoders have no bias drift. So the course should follow the gyro and be pulled slowly toward `course_odom`: `course = K * (course + gyro_inc) + (1 - K) * course_odom`, the Lab 11 complementary filter. With a 60 ms cycle, K = 0.98 gives a time constant K · dt / (1 − K) ≈ 3 s.

## 5. Differences from `Base_observer.qrs`

| Change | Why |
|---|---|
| Stop when `distance * cos(courseAngle)` < 1 cm, not `distance` < 5 cm | A 5 cm circle starts each turn early and tilts the next segment by up to 9.5°. A 1 cm circle is missed at 60 ms per cycle. |
| Steering `25 * sin(courseAngle)`, not `50 * courseAngle` | The cycle is 60 ms, not ~10 ms. `sin` stays bounded near a waypoint, where `courseAngle` passes 50°. |
| Turn in place first, 17 to 22 % power | Turn-then-drive, as the README recommends |
| Log lines written to the file once per segment | Same lines, fewer file writes |
| `i, gt, x, y, course` added to the log line | The README asks for `i` and the estimate. `gt` gives `dt`. |

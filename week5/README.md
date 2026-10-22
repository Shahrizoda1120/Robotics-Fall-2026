# Week 5: Motor parameter identification

**Goal:** measure how the two drive motors of the robot react to power, and describe each motor with two numbers, **T_m** and **k_e**. These numbers are reused in Week 6 (localization) and Week 7 (go-to-goal).

## Files in this folder

| File | What it is |
|---|---|
| `week5_motor_identification.qrs` | TRIK Studio program I used to collect the data |
| `motor_data/motor_data_A17_0vbe16yx.zip` | The 80 raw data files exactly as the robot saved them (not edited) |
| `week5_task_motor_identification.ipynb` | Colab notebook: fitting, results table, plots and discussion |

## 1. The idea

When you switch a motor on, the wheel does not reach full speed instantly. It speeds up quickly at first, then more and more slowly, until it settles at a steady speed. The lecture model describes this with two parameters:

| Symbol | Meaning | Unit |
|---|---|---|
| ω_ss | Top speed the wheel settles at | rad/s |
| T_m | How quickly it gets there (after T_m seconds it is at about 63 % of top speed) | s |
| k_e | Volts needed for 1 rad/s of speed, so **top speed = U / k_e** | V·s/rad |
| U | Real voltage on the motor: **U = PWM/100 × battery voltage** | V |

The encoder measures the **angle** θ, so I fit the angle formula from the lecture:

θ(t) = ω_ss · (t − T_m + T_m · e^(−t/T_m))

Once the wheel is up to speed this becomes θ ≈ ω_ss · (t − T_m): a straight line whose slope is the top speed, shifted by T_m.

## 2. How the data was collected

**Setup:**
- Robot A17.
- Wheel 3 is the big wheel (motor M3, encoder E3). Wheel 4 is the small wheel (motor M4, encoder E4).
- The robot was on a stand, so the wheels spun freely in the air.

For each wheel, and for every PWM level from -100 % to +100 % in 5 % steps (0 % skipped, so 40 levels), the program does:

1. Reset the encoder to 0 and wait 0.5 s.
2. Read the battery voltage.
3. Start a clock and switch the motor on at that PWM. This moment is t = 0.
4. For 2 s, keep saving the time and the encoder angle into memory (about every 30 ms).
5. Stop the motor, then save the memory as one file into the zip.
6. Wait 1.5 s so the wheel stops completely before the next level.

The data is written to a file only after the motor stops, because file writing is slow and would disturb the timing. Every line stores the real time, so the exact sampling interval does not matter.

## 3. How the data is organized

- **80 files** = 2 wheels × 40 PWM levels.
- **Name pattern:** `wheel_<wheel>_v_<PWM>.txt`, for example `wheel_3_v_50.txt` = wheel 3 at 50 %.

Each file has a 4-line header, then one measurement per line (time in ms, angle in degrees):

```
# wheel_id: 3
# pwm_percent: 50
# battery_voltage: 11.299088478088379
# format: time_ms angle_deg
25 0
53 3
88 7
...
1993 978
```

For example, `88 7` means: 88 ms after the motor was switched on, the wheel had turned 7°. The angle is a running total from 0, and it is negative when the wheel turns backwards.

## 4. What the notebook does

1. **Read the data:** open each file and convert to SI units (ms to s, degrees to radians).
2. **Fit each run:** `curve_fit` finds the ω_ss and T_m that make the θ(t) formula match the measured angles. That gives one ω_ss and one T_m per run.
3. **Results table:** ω_ss and T_m for every PWM level, for both wheels.
4. **ω_ss vs U plot:** both wheels on the same axes. The slope of the fitted line is 1/k_e, which gives **k_e**. The dead zone (where the wheel would not turn) is shaded.
5. **Simulation:** the identified motor (k_e and median T_m) is simulated with `odeint` for wheel 3 at 50 % and compared with the measurement.
6. **Discussion:** where the linear model fails, how much the wheels differ, and what that means for next week's odometry.

**How to run it:**
- **In Colab:** open the notebook and choose *Runtime > Run all*. When asked, upload `motor_data_A17_0vbe16yx.zip`.
- **Locally:** first unzip the zip inside `motor_data/`, then run all cells.

## 5. Results

| Wheel | k_e [V·s/rad] | T_m median [s] | Dead zone (est.) [V] |
|---|---|---|---|
| 3 (big) | 0.669 | 0.25 | -0.28 to +0.37 |
| 4 (small) | 0.712 | 0.18 | -0.26 to +0.06 |

**Main findings:**
- **The linear model is only accurate in the middle of the range.** It is off at very low and very high power, and T_m changes with power (0.2 to 0.7 s at low PWM, about 0.05 s at full power).
- **Wheel 3 spins faster than wheel 4 at the same PWM** (10.0 vs 9.3 rad/s at +50 %) and reacts more slowly, because it is the bigger wheel.


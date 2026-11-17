# Week 9: Gyroscope parameters

**Goal:** find the closest physical model of the TRIK gyroscope (yaw axis), so that a simulated gyro, `gyro_sim`, behaves like the real one. The parameters are used again in Labs 10 to 12.

## Files in this folder

| File | What it is |
|---|---|
| `lab9_gyro.qrs` | TRIK program, Option A (JavaScript, two threads: one logs, one drives) |
| `lab9_gyro.csv` | Raw log, run 1 |
| `lab9_gyro_32.csv` | Raw log, run 2 |
| `lab9_gyro_parameters_240032.ipynb` | Colab notebook: all plots, `calibrate_gyro`, `gyro_sim` |
| `gyro_params.json` | Model parameters (average of both runs) for Labs 10 to 12 |

## 1. Model

$$\omega_{meas} = Q_{\Delta}\big(\mathrm{sat}_{\pm\omega_{max}}[(1+s)\cdot\omega_{true} + b_0 + d\cdot t + n]\big), \quad n \sim \mathcal{N}(0,\sigma^2)$$

The gyro takes the robot's real turning rate $\omega_{true}$ and reports it a little wrong: scaled by $(1+s)$, plus a bias $b_0$ (its output when the robot is still) that drifts by $d$ every minute (t in minutes), plus random noise $n$, Gaussian with mean 0 and standard deviation $\sigma$. The result is clipped at $\pm\omega_{max}$, because the output can't go further, and rounded to steps of $\Delta$, because the output is an integer.
## 2. Experiment

**Option A:** open loop, the robot turns in place ($u_R = u$, $u_L = -u$).

1. **Warm-up**, 30 s: still, not logged.
2. **Static**, 3 min: still. Gives $b_0$, $d$, $\Delta$, $\sigma$.
3. **Rotation**, 72 s: a sine, then a square wave, period 4 s, at A = 30, 60 and 150 % for 12 s each. Gives $\omega_{max}$, $s$ and the sign. At 150 % the power clips at ±100 % and the gyro saturates, so no hand spin was needed.
4. **Rest**: only the first 3 s are used (run 1 was touched later).

The program logs `time()`, `read()[3]`, raw gyro `readRawData()[2]`, `read()[2]` and the encoders E4 (left) and E3 (right). The gyro is analysed from the raw value, converted to deg/s with the factor F. The reference heading comes from the wheels: $\theta_{enc} = \frac{r}{B}(\varphi_R - \varphi_L)$, with r = 0.043 m and B = 0.196 m. I ran the experiment twice: what repeats belongs to the sensor, and what changes must be measured at every start.

## 3. Parameter table

| Parameter | Symbol | Run 1 | Run 2 | Model | Unit | Method |
|---|---|---|---|---|---|---|
| Offset (bias) | $b_0$ | 0.3154 | 0.2134 | 0.2644 | deg/s | static mean, T = 30 s |
| Bias drift | $d$ | -0.0108 | +0.0199 | +0.0046 | deg/s/min | 30-s window means |
| Heading drift, uncorrected | | 18.1 | 12.8 | | deg/min | slope of θ(t) |
| Measurement range | ±$\omega_{max}$ | +287.4 / -287.1 | +287.6 / -287.3 | +287.5 / -287.2 | deg/s | saturation test |
| Resolution | $\Delta$ | 0.00877 | 0.00878 | 0.00877 | deg/s | quantisation step |
| Noise std | $\sigma$ | 0.0699 | 0.0738 | 0.0718 | deg/s | static, 3 min |
| Raw → deg/s factor | F | 0.008771 | 0.008777 | 0.008774 | deg/s per raw unit | `read()` vs `readRawData()` |
| Scale-factor error CCW / CW | $s$ | -5.5 / -5.2 | -4.5 / -5.0 | -5.0 / -5.1 | % | gyro vs. encoder angle |
| Positive rotation direction | | CCW | CCW | CCW | | gyro vs. encoder sign |

F, $\omega_{max}$ and $\Delta$ repeat within 0.1 %, $\sigma$ within 6 % and $s$ within 1 percentage point, and the sign is the same, so these belong to the sensor. $b_0$ changes by a third (0.32 vs 0.21 deg/s) and $d$ even changes sign, so the bias must be calibrated at every start.

## 4. Analysis

### Which error dominates

Heading error in degrees, run 1 / run 2:

| Source | 10 s | 1 min | 3 min |
|---|---|---|---|
| Bias, not removed | 3.2 / 2.1 | 18.9 / 12.8 | 56.8 / 38.4 |
| Bias left after `calibrate_gyro` | 0.2 / 0.3 | 1.1 / 1.6 | 3.2 / 4.7 |
| Drift | 0.01 / 0.02 | 0.3 / 0.6 | 2.9 / 5.4 |
| Noise | 0.05 / 0.05 | 0.12 / 0.13 | 0.21 / 0.23 |

Without calibration the bias dominates at every time, and its error grows linearly. After `calibrate_gyro` the leftover bias dominates up to 1 min, and at 3 min the drift is as large (3 to 5° each). Noise stays below 0.3°, so it never matters for the heading. When the robot turns, the scale factor adds ~18 to 20° per full turn, more than all time errors up to 3 min, and above 287 deg/s saturation is the largest error of all.

### Calibration

`calibrate_gyro` averages the first 10 s at rest and tracks the max and min. If max − min > 0.56 deg/s, the robot moved, so it returns `None` and the calibration is repeated. On the robot this needs only a running sum, max and min. I chose 10 s because past it the heading error left per minute drops by less than 10 %: the bias itself wanders, so longer averaging only costs time. The 0.56 deg/s limit is the largest max − min in any still 10-s window, plus 20 %. After 3 min still, the heading error left is 0.2° (run 1) and 2.3° (run 2), against 57° and 38° without calibration. On the rotation run it is 1 to 2° before the first saturated turn.

### Where e(t) grows

e(t) is the gyro heading minus the encoder heading during the rotation. Saturation is the main error: at 150 % the wheels turn at over 400 deg/s, but the gyro stops at 287 deg/s, so it misses 135 to 190° per turn. Most of it comes back on the way back, but 29 to 53° is left after the run. At 30 and 60 % the scale factor moves the error by ~5 % of each turn, and it comes back when the robot turns back. The bias adds only ~2° over the 72 s. Wheel slip at reversals doesn't show on its own, since $s$ is the same for the sine and the square wave, but it may be hidden inside $s$.

### Model check

`gyro_sim`, fed with the encoder rate, follows the real gyro, clipping included. Its heading is 14 to 15° RMS from the real gyro's, against 46 to 49° for an ideal gyro.


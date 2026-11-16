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

$$\omega_{meas} = Q_{\Delta}\big(\mathrm{sat}_{\pm\omega_{max}}[(1+s)\,\omega_{true} + b_0 + d\,t + n]\big), \quad n \sim \mathcal{N}(0,\sigma^2)$$

Read it from the inside out, in the order the signal passes through the sensor:
- $\omega_{true}$: how fast the robot really turns (deg/s). The encoders stand in for it.
- $(1+s)$: **scale-factor error.** The gyro reports $(1+s)$ times the real rate. $s = -5\,\%$ means it reports 95 % of every turn.
- $b_0$: **bias.** What the gyro outputs when the robot is still (deg/s).
- $d\,t$: **drift.** The bias changes by $d$ every minute (deg/s/min, t in minutes).
- $n \sim \mathcal{N}(0,\sigma^2)$: **noise.** A new random value on every sample, Gaussian with mean 0 and standard deviation $\sigma$ (deg/s).
- $\mathrm{sat}_{\pm\omega_{max}}[\,\cdot\,]$: **saturation.** The output can't go beyond $\pm\omega_{max}$; faster rates are clipped.
- $Q_{\Delta}(\cdot)$: **quantisation.** The output is an integer, so it is rounded to steps of $\Delta$: $Q_{\Delta}(x) = \Delta \cdot \mathrm{round}(x / \Delta)$.
- $\omega_{meas}$: what the gyro outputs (deg/s).

The order follows the hardware: scale, bias, drift and noise come from the sensing element, the range limit comes after, and rounding happens last, when the value becomes a number.

**Example** with my model values: the robot turns at 100 deg/s, so the gyro outputs 0.95 × 100 + 0.26 ± 0.07 ≈ 95.3 deg/s. At 400 deg/s, 0.95 × 400 + 0.26 ≈ 380 is above the limit, so the output is 287 deg/s.

## 2. Experiment

**Option A:** open loop, the robot turns in place ($u_R = u$, $u_L = -u$).

1. **Warm-up**, 30 s: still, not logged.
2. **Static**, 3 min: still. Gives $b_0$, $d$, $\Delta$, $\sigma$.
3. **Rotation**, 72 s: a sine, then a square wave, period 4 s, at A = 30, 60 and 150 % for 12 s each. Gives $\omega_{max}$, $s$ and the sign. At 150 % the power clips at ±100 % and the gyro saturates, so no hand spin was needed.
4. **Rest**: only the first 3 s are used (run 1 was touched later).

**Logged:** `time()`, `read()[3]`, raw gyro `readRawData()[2]`, `read()[2]`, encoders E4 (left) and E3 (right). The gyro is analysed from the raw value, converted to deg/s with the factor F. The reference heading comes from the wheels: $\theta_{enc} = \frac{r}{B}(\varphi_R - \varphi_L)$, with r = 0.043 m and B = 0.196 m.

**Two runs:** what repeats belongs to the sensor; what changes must be measured at every start.

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

**What repeats:** F, $\omega_{max}$ and $\Delta$ (within 0.1 %), $\sigma$ (within 6 %), $s$ (within 1 percentage point) and the sign. These belong to the sensor.
**What changes:** $b_0$ by a third (0.32 vs 0.21 deg/s), and $d$ even changes sign. The bias must be calibrated at every start.

## 4. Analysis

**Which error dominates.** Heading error in degrees, run 1 / run 2:

| Source | 10 s | 1 min | 3 min |
|---|---|---|---|
| Bias, not removed | 3.2 / 2.1 | 18.9 / 12.8 | 56.8 / 38.4 |
| Bias left after `calibrate_gyro` | 0.2 / 0.3 | 1.1 / 1.6 | 3.2 / 4.7 |
| Drift | 0.01 / 0.02 | 0.3 / 0.6 | 2.9 / 5.4 |
| Noise | 0.05 / 0.05 | 0.12 / 0.13 | 0.21 / 0.23 |

- Without calibration the bias dominates at every time, and the error grows linearly.
- After `calibrate_gyro` the leftover bias dominates up to 1 min. At 3 min the drift is as large (3 to 5° each). Noise stays below 0.3°, so it never matters for the heading.
- When the robot turns, the scale factor adds ~18 to 20° per full turn, more than all time errors up to 3 min. Above 287 deg/s, saturation is the largest error of all.

**Calibration.** `calibrate_gyro` averages the first 10 s at rest and tracks the max and min. If max − min > 0.56 deg/s, the robot moved, so it returns `None` and the calibration is repeated. On the robot this needs only a running sum, max and min.
- **Why 10 s:** past 10 s the heading error left per minute drops by less than 10 %, because the bias itself wanders. Longer averaging only costs time.
- **Why 0.56 deg/s:** the largest max − min in any still 10-s window, plus 20 %.
- **Result:** after 3 min still, the heading error left is 0.2° (run 1) and 2.3° (run 2), against 57° and 38° without calibration. On the rotation run it is 1 to 2° before the first saturated turn.

**Where e(t) grows** (gyro heading minus encoder heading during the rotation):
- **Saturation (the main error):** at 150 % the wheels turn at over 400 deg/s, but the gyro stops at 287 deg/s, so it misses 135 to 190° per turn. Most of it comes back on the way back, but 29 to 53° is left after the run.
- **Scale factor:** at 30 and 60 % the error moves by ~5 % of each turn and comes back when the robot turns back.
- **Bias:** ~2° over the 72 s. It doesn't show in the plot.
- **Wheel slip:** $s$ is the same for the sine and the square wave, so slip at reversals doesn't show on its own. It may still be hidden inside $s$.

**Model check.** `gyro_sim`, fed with the encoder rate, follows the real gyro, clipping included. Its heading is 14 to 15° RMS from the real gyro's, against 46 to 49° for an ideal gyro.



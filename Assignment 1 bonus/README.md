# Assignment 1 bonus: motor model on the ground

The simple model from Assignment 1 does not fit my motors well, so I add friction. The notebook follows the layout and the friction model of Charos's notebook and identifies the model in three steps:

1. **$k_e$ and dry friction from the steps**, without a complex signal.
2. **$T_m$ and viscous friction on the complex signals**: each signal is used once for identification, and the model is tested on the other two.
3. **RMSE of the simple and the updated model.**

Everything is fitted and compared on the measured angle, so no speed is taken from differentiated encoder angles.

## Model

Simple model (Assignment 1): $T_m\dot\omega=u/k_e-\omega$, with $u=\mathrm{PWM}/100\cdot U_{battery}$.

Updated model:

$$T_m\dot\omega=\frac{u-U}{k_e}-(|d|+b)\,\omega,\qquad u=d\cdot U_{battery}$$

- $U$: dry friction, $U_{fw}$ forward and $U_{bw}$ backward. A still wheel starts only when $|u|>U$.
- $b$: viscous friction, $b_{fw}$ and $b_{bw}$.
- $d$: the real PWM duty. TRIK converts the power with its `motor100` table, so 60 % power gives only 19 % duty.
- $|d|$ instead of $1$: back-EMF brakes the motor only while the PWM pulse is on (I assume no current between pulses).

**Change from Charos's model.** She uses $u=\mathrm{PWM}/100\cdot U_{battery}$ and $(1+b)$. With $(1+b)$ the steps give only the product $k_e(1+b)$, so $k_e$ and $b$ cannot be found separately. Both models follow the steady state of my steps about equally well (RMS error of $\omega_{ss}$ 0.47 / 0.39 rad/s on M3 / M4 for mine, 0.53 / 0.43 rad/s for hers), but her dry friction on M4 comes out negative (−0.52 V forward), which is impossible. With the real duty and $(|d|+b)$, $k_e$ and $b$ come out separately and every dry friction is positive.

## Results

- $k_e$ is 0.689 V·s/rad on M3 and 0.735 V·s/rad on M4. Dry friction is 0.798 / 0.976 V on M3 and 0.704 / 0.817 V on M4 (forward / backward), so a still wheel starts at 10 / 14 % power on M3 and 6 / 10 % on M4.
- The updated model has a lower test RMSE than the simple model in all 12 cross-tests.
- The slow chirp is the best identification signal: on the same test signal, the model identified on it wins 3 of its 4 comparisons. Identified on it, the updated model has a test RMSE of 1.94 / 4.15 rad on the triangle and 1.33 / 0.60 rad on the square wave + chirp (M3 / M4), against 3.28 / 4.42 and 1.75 / 1.19 rad for the simple model.

## Folder contents

- `robotics_assignment1_240032_bonus.ipynb`: the Colab notebook with the processing, plots and analysis.
- `motor_data_ground/`: the raw files exactly as the robot wrote them.
  - `steps_ground.txt`: steps +10, −10, +20, −20, ..., +100, −100 %, each 2 s on and 1 s off.
  - `chirp_ground.txt`: slow chirp, 60 %, 0.1 → 2 Hz, 30 s.
  - `triangle_ground.txt`: triangle, 50 %, period 10 s, 20 s.
  - `signal_ground.txt`: square wave, 60 %, period 1 s, for 4 s, then chirp, 60 %, 0.2 → 3 Hz, 10 s.
  - `battery.txt`: battery voltage before and after each run.
  - `encoders.txt`: encoder degrees and raw ticks at the end of each run.
- `TRIK programs/`: `1_ground_steps.qrs`, `2_ground_chirp.qrs`, `3_ground_triangle.qrs` and `4_ground_signal.qrs`, the TRIK Studio block programs that recorded the four runs.

## Setup

- The robot drives on the floor. M3 (left wheel) and M4 (right wheel) get the same power.
- Each log line is time (ms), power (%), E3 and E4 (raw ticks). The log is kept in memory and written to the file once at the end, so file writing does not slow the loop.
- The steps are sampled every 49 ms, the complex signals every 90 to 96 ms.
- `battery.txt` and `encoders.txt` also keep lines from earlier runs (Assignment 1 and a first steps run I do not use). The notebook takes the last line of each ground file.

## How to run

Keep the notebook next to `motor_data_ground/`, or open it in Colab and upload the six files when the first cell asks. It needs numpy, pandas, scipy and matplotlib and runs in one to two minutes, because every fit starts from four initial guesses.

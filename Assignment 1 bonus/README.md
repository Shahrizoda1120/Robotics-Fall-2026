# Assignment 1 bonus: updated motor model

The simple first-order model from Assignment 1 does not fit my TRIK motors well. This notebook adds friction to the model and identifies it in the three steps the task asked for:

1. **$k_e$ and dry friction from the steps**, without a complex signal.
2. **$T_m$ and viscous friction on the complex signals**: each signal is used once for identification, and the model is tested on the other two.
3. **RMSE of the simple and the updated model.**

## Model

Simple model: $T_m\dot\omega = U/k_e - \omega$, with $U = \mathrm{PWM}/100 \cdot U_{battery}$.

Updated model:

$$T_m\dot\omega=\frac{u-U}{k_e}-(|d|+b)\,\omega,\qquad u=d\cdot U_{battery}$$

- $U$: dry friction, $U_{fw}$ forward and $U_{bw}$ backward. A still wheel starts only when $|u|>U$.
- $b$: viscous friction, $b_{fw}$ and $b_{bw}$.
- $d$: the real PWM duty. TRIK converts the power we set with its `motor100` table, so 60 % power gives only 19 % duty.
- $|d|$ instead of $1$: back-EMF brakes the motor only while the PWM pulse is on. With $1+b$ the steps give a negative dry friction.

## Results

- Dry friction from the steps: 0.48 V on M3, 0.42 V forward and 0.55 V backward on M4.
- $T_m$ of the updated model is 0.031 to 0.040 s on every signal; the simple model's $T_m$ ranges from 0.102 to 0.438 s.
- Speed RMSE on the test signals: the updated model is lower in all 12 tests. Identified on the slow chirp, the best identification signal, it has 0.45 to 0.85 rad/s, against 1.05 to 2.10 rad/s for the simple model.

## How to run

The notebook reads the Assignment 1 data from `motor_data/`: `steps_M3`, `steps_M4`, `chirp_*`, `triangle_*`, `signal_*` and `battery.txt`. Keep the notebook next to that folder, or open it in Colab and upload the files when asked. It needs numpy, pandas, scipy and matplotlib, and runs in a few minutes; the identification cell takes most of the time.

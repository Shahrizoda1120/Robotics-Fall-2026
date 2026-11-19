# Assignment 1: Motor parameter identification

**Goal:** describe the two drive motors of my robot with the first-order model from the lecture. For each motor and direction I find ω_ss and T_m at every PWM level, then k_e and the dead zone. I test the model on complex signals and analyse the measurement artifacts. For the bonus, I explain the dead zone, Coulomb + viscous friction and backward ≥ forward, and derive an improved formula.

## Files in this folder

- `TRIK Programs/`: the 9 TRIK Studio block programs, one per run (see its README).
- `motor_data/`: the 9 raw log files and `battery.txt`, exactly as the robot wrote them (see its README).
- `robotics_lab5_240032.ipynb`: Colab notebook with the processing, the plots and the report.

## Setup

- Left wheel: motor M3, encoder E3. Right wheel: motor M4, encoder E4.
- The robot was lifted, so the wheels spun freely in the air.
- All 9 runs were done in one session. The robot read the battery before and after each run: 11.545 V at the start, 11.493 V at the end.

## Runs

1. **Step series** (programs 1 to 3): 3 s motor off, then PWM from -100 % to +100 % in 10 % steps (0 skipped), each 2 s on and 1 s coasting. Both motors, and twice on M4 to check repeatability.
2. **Square wave + chirp** (programs 4, 5): ±60 %, period 1 s for 4 s, then a chirp from 0.2 to 3 Hz for 10 s.
3. **Slow triangle** (programs 6, 7): ±50 %, period 40 s. PWM changes by 5 % per second, so it finds the dead zone to 1 %.
4. **Slow chirp** (programs 8, 9): ±60 %, 0.1 to 2 Hz over 30 s, for the frequency response.

Every run ends with 1 s at PWM 0 (coasting), still logged.

## How to run the notebook

- **In Colab:** open the notebook, choose *Runtime > Run all*, and upload the 10 .txt files from `motor_data/` when the first cell asks.
- **Locally:** run it from this folder. It reads `motor_data/` directly and takes about 10 s.

## Results

- **M3 (left):** k_e = 1.063 V·s/rad forward, 1.116 backward; mean T_m = 0.178 s forward, 0.174 s backward.
- **M4 (right):** k_e = 1.058 V·s/rad forward, 1.115 backward; mean T_m = 0.139 s forward, 0.148 s backward.
- **Dead zone:** at most 1 % PWM (0.12 V) on both motors, in both directions.

**Main findings:**

- Each step fits the model, but T_m is not constant: 0.342 s at 10 % and 0.025 s at 100 % (M3 forward). The ω_ss lines reach zero 32.5 to 48.3 % PWM on the other side, so ω_ss is not proportional to U.
- **Dead zone:** at most 1 %. Near zero the speed jumps up and then follows the line.
- **Coulomb + viscous friction:** coasting needs both parts (lowest RMSE in all four cases). The friction model cuts the coasting distance error from 1.73 to 0.44 rad (M3) and from 0.85 to 0.42 rad (M4).
- **Backward ≥ forward:** backward k_e is 5 % larger on both motors, while backward coasting friction is 14 to 20 % smaller, so the cause is in the motor, not in friction. The wheel drifts forward on a symmetric signal (13.1 and 17.2 rad on the slow chirp).
- **Improved formula:** ω_ss(U) = sign(U) · [ω0 · (1 − e^(−|U|/U1)) + |U|/k]; while driven J' · dω/dt = U − (U / ω_ss(U)) · ω; at PWM 0 dω/dt = −a · sign(ω) − β · ω; separate constants for each direction. The speed fit rises from 40 to 62 % (course model) to 96 and 95 % on the triangle, 79 and 84 % on the slow chirp, and 80 and 73 % on the square wave + chirp.
- The real sampling interval is 32.5 ms (steps) and 51 to 55 ms (other runs), not the 5 ms of the Timer, so the notebook always uses the logged time.

# Assignment 1: Motor parameter identification

**Goal:** describe the two drive motors of my robot with the first-order model from the lecture. For each motor and direction I find the steady speed ω_ss and the time constant T_m at every PWM level, then k_e and the dead zone. I check the model on a complex input signal and look for measurement artifacts (zero drift, sampling jitter, coasting and others).

## Files in this folder

- `TRIK Programs/`: the TRIK Studio block programs, one file per run (see its README).
- `motor_data/`: the raw log files exactly as the robot wrote them, plus `battery.txt` (see its README).
- `robotics_lab5_240032.ipynb`: Colab notebook with the processing, the plots and the report.

## Setup

- Left wheel: motor M3, encoder E3. Right wheel: motor M4, encoder E4.
- The robot was lifted, so the wheels spun freely in the air.
- Battery: 11.3 V before and after every run (TRIK display).

## What I did

1. **Program A, step series:** 3 s motor off, then PWM from -100 % to +100 % in 10 % steps (0 skipped). Each level: 2 s on, then 1 s with power 0 (coasting). Run on both motors, and twice on M4 to check repeatability.
2. **Program B, complex signal:** square wave (A = 60 %, period 1 s) for 4 s, then a chirp (A = 60 %, 0.2 to 3 Hz) for 10 s, then 1 s coasting. Run on both motors.
3. **Notebook:**
   - fits θ(t) = ω_ss · (t − T_m + T_m · e^(−t/T_m)) to every step
   - plots ω_ss and T_m against PWM, and finds k_e and the dead zone
   - simulates the complex signal with `odeint` and compares it with the measurement
   - analyses the artifacts and answers the report questions

## How to run the notebook

- **In Colab:** open the notebook and choose *Runtime > Run all*. When asked, upload the 6 files from `motor_data/`.
- **Locally:** run it from this folder. It reads the files from `motor_data/` directly.

## Results

- **M3 (left):** k_e = 1.09 V·s/rad in both directions; mean T_m = 0.18 s forward, 0.14 s backward.
- **M4 (right):** k_e = 1.01 V·s/rad forward, 1.07 backward; mean T_m = 0.15 s forward, 0.13 s backward.
- **Dead zone:** none, both wheels already turn at ±10 %.

**Main findings:**

- Each step fits the model well, but T_m is not constant: it falls from 0.27 to 0.44 s at ±10 % to 0.02 to 0.03 s at ±100 %.
- Forward and backward are almost the same (slopes within 6 %), so one set of parameters per motor is enough. The two motors differ more (7 % in slope), so each motor needs its own set.
- The model with the dead zone (fitted line) reaches 76 % (M3) and 75 % (M4) speed fit on the complex signal, and 87 % and 86 % on the square wave alone. The chirp is the hardest part.
- The real sampling interval is about 32 ms (steps) and 52 ms (signal), not the 5 ms of the Timer, so the notebook always uses the logged time.

# Assignment 1: Motor parameter identification

I measured the two drive motors of my TRIK robot with the wheels in the air and fitted the first-order motor model from the lecture to every step, which gives ω_ss, T_m and k_e for each motor and direction. The model fits each single step, but its "constants" change with PWM. I explain why (dead zone, Coulomb + viscous friction, backward ≥ forward) and build an improved model from the same data. On complex signals it never saw, the improved model raises the speed fit from between 40 and 62 % to between 73 and 96 %.

## Folder contents

- `robotics_lab5_240032.ipynb`: Colab notebook with the processing, tables, plots and the report.
- `motor_data/`: the 9 raw log files and `battery.txt`, exactly as the robot wrote them (format in its README).
- `TRIK Programs/`: the 9 TRIK Studio block programs, one per run (details in its README).

## Setup

- Left wheel: motor M3, encoder E3. Right wheel: motor M4, encoder E4.
- The robot was lifted, so the wheels spun freely in the air. This measures only the motor and gearbox, not the floor.
- All 9 runs were done in one session, in program order 1 to 9.
- Every program logs `angle [deg], time [ms], PWM [%]` in each loop (Timer 5 ms), and the robot writes the battery voltage before and after each run to `battery.txt`: 11.545 V at the start, 11.493 V at the end.

## Model

dω/dt = U / (T_m · k_e) − ω / T_m, with U = PWM/100 · U_battery

- ω_ss [rad/s]: the steady speed, ω_ss = U / k_e.
- T_m [s]: the time constant; the motor reaches 63 % of ω_ss after T_m.
- k_e [V·s/rad]: the volts needed for 1 rad/s of steady speed.

For a step from rest, θ(t) = ω_ss · (t − T_m + T_m · e^(−t/T_m)). The notebook fits this to every step with `curve_fit`.

## The 9 runs and why each one

**Step series:** 3 s motor off, then −100 to +100 % in 10 % steps (0 skipped), each 2 s on and 1 s at PWM 0.

1. `steps_M4.txt`: the main identification run on M4. Each step gives ω_ss and T_m, and from them k_e. The first 3 s check zero drift, and the 1 s after each step is coasting, which measures friction.
2. `steps_M4_run2.txt`: the same on M4 again, to check repeatability.
3. `steps_M3.txt`: the same on M3, to compare the two motors.

**Square wave + chirp:** ±60 %, period 1 s for 4 s, then a chirp from 0.2 to 3 Hz for 10 s.

4. `signal_M4.txt` and 5. `signal_M3.txt`: the complex signal from the assignment, used only to test the models. The fast reversals would show gear backlash.

**Slow triangle:** ±50 %, period 40 s, 2 periods; PWM changes by 5 % per second.

6. `triangle_M4.txt` and 7. `triangle_M3.txt`: PWM passes slowly through every whole percent, so they measure the dead zone to 1 % and the shape of the speed curve near zero. Also a test signal.

**Slow chirp:** ±60 %, 0.1 to 2 Hz over 30 s.

8. `chirp_M4.txt` and 9. `chirp_M3.txt`: the frequency response around the corner frequency 1/(2π T_m), about 1 Hz. The signal is symmetric, so any net drift shows backward ≥ forward. Also a test signal.

Every run ends with 1 s at PWM 0 (coasting), still logged. Amplitudes stay at or below 60 %, so the motor never saturates.

## What the notebook does

1. **Model:** the equation, its parameters and the assumptions, each tested by the data.
2. **Test signals:** the 9 runs above.
3. **Identification:** fits every step (Table 1), then a line through ω_ss against PWM on 20 to 80 % gives k_e (Table 2, Figure 1).
4. **Three outcomes:** dead zone (4.1), Coulomb + viscous friction from coasting (4.2, Table 3, Figure 2), backward ≥ forward (4.3, Table 4).
5. **Improved formula:** derivation, constants (Table 5), and the test on all complex signals (Table 6, Figures 3 and 4).
6. **Artifacts:** all required artifacts plus one more, each with a number and its cause (Figure 5).

**Where to find the required report items:**
- Estimated values: Table 2.
- Plots: step responses in Figure 1a, speed from PWM in Figure 1b, T_m from PWM in Figure 1c, complex signals in Figures 3 and 4, error over time in Figure 3 (row 3), artifacts in Figure 5.
- Analysis: the Report section at the end of the notebook.

## How to run

- **In Colab:** open the notebook, choose *Runtime > Run all*, and upload the 10 .txt files from `motor_data/` when the first cell asks.
- **Locally:** run it from this folder with numpy, pandas, scipy and matplotlib installed. It reads `motor_data/` directly and takes about 10 s.

## Results

### Estimated values (Table 2)

- **M3 forward:** dead zone 0 %, slope 0.108 rad/s per %, k_e 1.063 V·s/rad, mean T_m 0.178 s, U_battery 11.526 V
- **M3 backward:** dead zone 1 %, slope 0.103, k_e 1.116, mean T_m 0.174 s, U_battery 11.526 V
- **M4 forward:** dead zone 1 %, slope 0.109, k_e 1.058, mean T_m 0.139 s, U_battery 11.545 V
- **M4 backward:** dead zone 1 %, slope 0.104, k_e 1.115, mean T_m 0.148 s, U_battery 11.545 V

The repeat run on M4 matches run 1 within 0.7 %.

### Where the course model breaks

- Each step fits well, but T_m is not constant: 0.342 s at 10 % and 0.025 s at 100 % (M3 forward), with the same load.
- The ω_ss lines do not pass through zero: they reach zero speed 32.5 to 48.3 % PWM on the other side. Near zero the speed jumps to about 4 rad/s within a few percent, then grows in a straight line (an S-shape).

### Three outcomes

- **Dead zone:** at most 1 % PWM (0.12 V) on both motors and in both directions, measured on the slow triangle.
- **Coulomb + viscous friction:** at PWM 0 no current flows, so while coasting only friction acts: dω/dt = −a · sign(ω) − β · ω. Both parts are needed (lowest error in all four cases). M3 is mostly Coulomb friction, M4 mostly viscous. The wheel rolls 0.4 to 5.9 rad after power-off; the friction model predicts this distance with 0.44 and 0.42 rad error, the course model with 1.73 and 0.85 rad.
- **Backward ≥ forward:** backward k_e is 5.0 % (M3) and 5.4 % (M4) larger, so backward needs more volts per rad/s. M4 backward is slower at every PWM. M3 backward has a bigger jump near zero, so it is faster at 10 and 50 % and slower only at 100 %. Backward coasting friction is 14 to 20 % smaller, so the cause is in the motor itself, not in friction. Effect: the wheel drifts forward 13.1 rad (M3) and 17.2 rad (M4) on the symmetric slow chirp.

### Improved formula

For each motor and direction:

- ω_ss(U) = sign(U) · [ω0 · (1 − e^(−|U|/U1)) + |U|/k]: the measured S-shape.
- PWM ≠ 0: J' · dω/dt = U − (U / ω_ss(U)) · ω, so the time constant T_m(U) = J' · ω_ss(U) / U falls with PWM.
- PWM = 0: dω/dt = −a · sign(ω) − β · ω, friction only.

All constants come from the steps and the coasting (Table 5). The complex signals are only used to test the model.

Speed fit, course model → improved model:

- Slow triangle: 39.8 → 96.1 % (M3) and 47.1 → 95.2 % (M4).
- Slow chirp: 50.7 → 78.9 % and 58.4 → 84.0 %.
- Square wave + chirp: 56.4 → 79.7 % and 61.7 → 72.8 %. On the square wave alone it reaches 85.2 and 83.4 %; the error is largest in the fast chirp.
- Forward drift on the slow chirp: the improved model gives 13.6 and 16.4 rad (measured 13.1 and 17.2), the course model only 7.0 and 7.7.
- Frequency response: the measured speed/voltage ratio falls from 1.55 and 1.36 at 0.36 Hz to 0.63 and 0.69 at 2 Hz, far above the course formula; the improved model follows it.

### Artifacts

- **Sampling:** the Timer waits 5 ms, but samples come every 32.5 ms (steps) and 51 to 55 ms (other runs) on average, from 23 to 152 ms. The notebook always uses the logged time.
- **Zero drift:** 0° with the motor off. The encoder is not reset between programs, and `time()` starts at 56 to 101 ms in the step runs.
- **Saturation:** none; at ±100 % the speed is within 3 % of the line.
- **Gear backlash:** no flat part at reversals, so the play is below one encoder degree.
- **Battery sag:** at most 0.013 V in one run.
- **Encoder noise:** 0.39 rad/s raw, 0.17 rad/s with a 3-sample moving average.
- **Motor-to-motor spread:** slope and k_e within 0.6 %, but M3's T_m is 27.9 % larger.

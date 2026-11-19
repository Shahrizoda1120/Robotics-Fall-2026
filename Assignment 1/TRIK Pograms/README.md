# TRIK Programs

All programs are built from TRIK Studio visual blocks, following the pseudocode in the assignment. There is one file per run, so every data file in `motor_data/` has the exact program that wrote it. The number is the run order.

## Files

- `1_motor_steps_M4.qrs`: Program A (step series) on M4 / E4, writes `steps_M4.txt`.
- `2_motor_steps_M4_run2.qrs`: Program A on M4 / E4 again (repeatability), writes `steps_M4_run2.txt`.
- `3_motor_steps_M3.qrs`: Program A on M3 / E3, writes `steps_M3.txt`.
- `4_motor_signal_M4.qrs`: Program B (square wave + chirp) on M4 / E4, writes `signal_M4.txt`.
- `5_motor_signal_M3.qrs`: Program B on M3 / E3, writes `signal_M3.txt`.
- `6_motor_triangle_M4.qrs`: Program C (slow triangle) on M4 / E4, writes `triangle_M4.txt`.
- `7_motor_triangle_M3.qrs`: Program C on M3 / E3, writes `triangle_M3.txt`.
- `8_motor_chirp_M4.qrs`: Program D (slow chirp) on M4 / E4, writes `chirp_M4.txt`.
- `9_motor_chirp_M3.qrs`: Program D on M3 / E3, writes `chirp_M3.txt`.

Files of the same program differ only in the motor port, the encoder and the file name.

## Logging (all programs)

Each program first deletes its old log (Remove File). Then every loop does:

1. `theta = encoder4` (or `encoder3`) and `t = time()`
2. Write To File: `theta,t,PWM` and a new line
3. Timer 5 ms

At the end: power 0 for 1 s, still logged (coasting), then Motor stop.

## Battery voltage

TRIK Studio has no block for the battery voltage, so I used a voltage reading block, as the assignment allows: a System block runs `brick.battery().readVoltage()` (the same call as in the course's motor identification README) at the start and at the end of every program. Write To File then adds one line to `battery.txt`: file name, voltage before, voltage after. This block works only when the program is generated as Python.

## Program A: step series

1. 3 s with the motor off, logged (zero drift check).
2. For `level` from -100 to 100 in steps of 10, skipping 0:
   - motor power = `level` for 2 s, logged
   - motor power = 0 for 1 s, logged (coasting)

## Program B: square wave + chirp

Parameters: `A = 60` %, `P = 1000` ms, `Ts = 4000` ms, `T = 10000` ms, `f0 = 0.2` Hz, `f1 = 3` Hz. While `time() - t0 < Ts + T`, with `tau = time() - t0`:

- **Square wave** (while `tau < Ts`): `x = tau / P - floor(tau / P)` and `u = A * (1 - 2 * floor(2 * x))`, so u is +60 % in the first half of every period and -60 % in the second half.
- **Chirp** (after `Ts`): `tc = (tau - Ts) / 1000`, `ph = 2 * pi * (f0 * tc + (f1 - f0) * tc * tc / (2 * T / 1000))` and `u = floor(A * sin(ph) + 0.5)`. The frequency rises from 0.2 to 3 Hz.

## Program C: slow triangle

Parameters: `A = 50` %, `P = 40000` ms, `T = 80000` ms (2 periods). While `time() - t0 < T`:

- `x = tau / P + 0.25 - floor(tau / P + 0.25)` and `u = floor(A * (1 - 4 * abs(x - 0.5)) + 0.5)`.
- The 0.25 shift starts the triangle at 0 and rising. PWM changes by 5 % per second, so the dead zone shows to 1 %.

## Program D: slow chirp

Parameters: `A = 60` %, `f0 = 0.1` Hz, `f1 = 2` Hz, `T = 30000` ms. While `time() - t0 < T`:

- `tc = tau / 1000`, `ph = 2 * pi * (f0 * tc + (f1 - f0) * tc * tc / (2 * T / 1000))` and `u = floor(A * sin(ph) + 0.5)`.
- It starts well below and ends well above 1/(2π T_m), about 1 Hz, as the design rules ask.

TRIK has no `round()`, so `floor(u + 0.5)` rounds u to a whole percent.

## How to run

1. Open the `.qrs` file in TRIK Studio, generate it as Python and upload it to the robot.
2. Lift the robot so the wheels spin freely, and run the programs in order, 1 to 9.
3. Download the `.txt` files and `battery.txt` from the robot without editing them.

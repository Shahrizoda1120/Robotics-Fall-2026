# TRIK Programs

Both programs are built with TRIK Studio visual blocks (no JavaScript or Python), following the pseudocode in the assignment. I saved one file per run, so every data file in `motor_data/` has the exact program that wrote it.

## Files

- `1_motor_steps_M4.qrs`: Program A on M4 / E4, writes `steps_M4.txt`.
- `2_motor_steps_M4_run2.qrs`: Program A on M4 / E4 again, writes `steps_M4_run2.txt`.
- `3_motor_steps_M3.qrs`: Program A on M3 / E3, writes `steps_M3.txt`.
- `4_motor_signal_M4.qrs`: Program B on M4 / E4, writes `signal_M4.txt`.
- `5_motor_signal_M3.qrs`: Program B on M3 / E3, writes `signal_M3.txt`.

Files of the same program differ only in the motor port, the encoder and the file name.

## Logging (both programs)

Each program first deletes its old file (Remove File), so every run starts with an empty file. Then every loop does:

1. `theta = encoder4` (or `encoder3`) and `t = time()`
2. Write To File: `theta,t,PWM` and a new line
3. Timer 5 ms

At the end the motor is stopped.

## Program A: step series

1. 3 s with the motor off, logged (zero drift check).
2. For `level` from -100 to 100 in steps of 10, skipping 0:
   - motor power = `level` for 2 s, logged
   - motor power = 0 for 1 s, logged (coasting)
3. Motor stop.

## Program B: complex signal (square wave + chirp)

Parameters: `A = 60` %, `P = 1000` ms, `Ts = 4000` ms, `T = 10000` ms, `f0 = 0.2` Hz, `f1 = 3` Hz.

While `time() - t0 < Ts + T`, with `tau = time() - t0`:

- **Square wave** (while `tau < Ts`):
  - `x = tau / P - floor(tau / P)`
  - `u = A * (1 - 2 * floor(2 * x))`, so u is +60 % in the first half of every period and -60 % in the second half.
- **Chirp** (after `Ts`):
  - `tc = (tau - Ts) / 1000`
  - `ph = 2 * pi * (f0 * tc + (f1 - f0) * tc * tc / (2 * T / 1000))`
  - `u = floor(A * sin(ph) + 0.5)`
  - The frequency rises from 0.2 to 3 Hz. TRIK has no `round()`, so `floor(... + 0.5)` rounds u to a whole percent.
- Motor power = `u`, then log.

After that: power 0 for 1 s, logged (coasting), then motor stop.

## How to run

1. Open the `.qrs` file in TRIK Studio and upload it to the robot.
2. Lift the robot so the wheels spin freely, note the battery voltage and run the program.
3. Note the battery voltage again and download the `.txt` file from the robot without editing it.

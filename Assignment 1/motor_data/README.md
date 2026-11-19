# motor_data: raw measurements

The log files and `battery.txt` exactly as the robot wrote them. I did not edit, clean or cut them. All runs were done in one session, in the order of the programs (1 to 9).

## Files

- `steps_M4.txt`: step series on M4, the right wheel.
- `steps_M4_run2.txt`: step series on M4 again, to check repeatability.
- `steps_M3.txt`: step series on M3, the left wheel.
- `signal_M4.txt`, `signal_M3.txt`: square wave + chirp.
- `triangle_M4.txt`, `triangle_M3.txt`: slow triangle.
- `chirp_M4.txt`, `chirp_M3.txt`: slow chirp.
- `battery.txt`: battery voltage before and after every run.

## Format of the log files

One sample per line, no header:

```
angle,time,PWM
```

- `angle`: encoder reading in degrees.
- `time`: `time()` on the robot, in ms since the program started.
- `PWM`: power sent to the motor at that moment, in %. It is 0 while the motor is off and while coasting.

Example from `steps_M4.txt`:

```
15104,100.756103515625,0
...
15100,3134.2958984375,-100
```

`15100,3134.2958984375,-100` means: 3134 ms after the start, encoder E4 read 15100° while M4 got -100 %.

## Things to know

- **The encoder is not reset.** It keeps its count between programs: `steps_M4.txt` ends at 15533° and `steps_M4_run2.txt` starts at 15533°. The notebook measures every step from its own first sample.
- **The first time is not 0.** It is 56 to 137 ms, so the notebook also subtracts the first time of every step.
- **Samples are not 5 ms apart.** The Timer waits 5 ms, but writing to the file takes longer: on average 32.5 ms in the step runs and 51 to 55 ms in the other runs.
- **Size:** each step run is 63 s and about 1950 lines, each square wave + chirp run 15 s and about 290 lines, each triangle run 81 s and about 1520 lines, each slow chirp run 31 s and about 590 lines.

## battery.txt

Written by the robot, no header, one line per run:

```
file,U_before,U_after
```

Example: `steps_M4.txt,11.545283317565918,11.545283317565918`. Voltages are in V. The battery was 11.545 V at the start of the session and 11.493 V at the end, with at most 0.013 V drop in one run. The notebook uses the mean of before and after to convert PWM to volts: U = PWM/100 · U_battery.

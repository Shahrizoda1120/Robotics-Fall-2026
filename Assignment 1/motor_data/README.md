# motor_data: raw measurements

The log files exactly as the robot wrote them. I did not edit, clean or cut them.

## Files

- `steps_M4.txt`: Program A (step series) on M4, the right wheel.
- `steps_M4_run2.txt`: Program A again on M4, to check repeatability.
- `steps_M3.txt`: Program A on M3, the left wheel.
- `signal_M4.txt`: Program B (square wave + chirp) on M4.
- `signal_M3.txt`: Program B on M3.
- `battery.txt`: battery voltage before and after every run.

## Format of the log files

One sample per line, no header:

```
angle,time,PWM
```

- `angle`: encoder reading in degrees.
- `time`: `time()` on the robot, in ms since the program started.
- `PWM`: power sent to the motor at that moment, in %. It is 0 in the first 3 s, while coasting, and in the last second of the signal runs.

Example from `steps_M4.txt`:

```
6572,44.230712890625,0
...
6571,3048.9765625,-100
```

`6571,3048.9765625,-100` means: 3049 ms after the start, encoder E4 read 6571° while M4 got -100 %.

## Things to know

- **The encoder is not reset.** It keeps its count between programs: `steps_M4.txt` ends at 7187° and `steps_M4_run2.txt` starts at 7187°. The notebook subtracts the first angle of every step.
- **The first time is not 0.** It is 42 to 46 ms, so the notebook also subtracts the first time of every step.
- **Samples are not 5 ms apart.** The Timer waits 5 ms, but writing to the file takes longer: about 32 ms per sample in the step runs and 52 ms in the signal runs.
- **Size:** each step run is 63 s and about 1900 to 2000 lines. Each signal run is 15 s and about 290 lines.

## battery.txt

```
file,U_before,U_after
steps_M4.txt,11.3,11.3
...
```

Voltage in V, read on the TRIK display right before and right after each run. The display shows 0.1 V steps, so 11.3 V before and after means the drop was below 0.1 V. The notebook uses the mean of the two to convert PWM to volts: U = PWM/100 · U_battery.

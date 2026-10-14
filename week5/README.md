# Week 5: Motor parameter identification

Step-response experiment on both drive motors of robot A17 (M3/E3 and M4/E4), PWM from -100 % to +100 % in 5 % steps, 2 s per level.

## Files

- `motor_identification.qrs`: TRIK Studio program used to collect the data
- `motor_data/`: raw files from the robot, `wheel_<N>_v_<PWM>.txt` (not edited)
- `week5_motor_identification.ipynb`: fitting, results table, plots and discussion

## Results

| Wheel | k_e [V·s/rad] | T_m [s] (median) |
|---|---|---|
| 3 | 0.669 | 0.25 |
| 4 | 0.712 | 0.18 |

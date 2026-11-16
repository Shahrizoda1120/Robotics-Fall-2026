# Robotics-Fall-2026

**CS413 Mobile Robotics · New Uzbekistan University · Fall 2026**

My lab work for the course, on the TRIK differential-drive robot. Each lab runs on the real robot (and in the TRIK Studio simulator where the lab asks for it), and the logs are analysed in Google Colab. Many labs drive the digit **32**, the last two digits of my student ID.

Course materials: [NewUU-engineering/Robotics-course](https://github.com/NewUU-engineering/Robotics-course)

## Labs

| Week | Topic | What I did |
|---|---|---|
| 1 | [TRIK Studio introduction](week1) | Drove the digit 32 |
| 2 | [PID controller](week2) | Drove the digit 32 with PID control |
| 3 | [Line following](week3) | PID line following on a V-shaped line ([video](https://youtu.be/58Rvpuw-uGA)) |
| 4 | [Line-following competition](week4) | PID line following with two sensors ([video](https://youtu.be/FvkpTiQymGA)) |
| 5 | [Motor parameter identification](week5) | Described both motors with T_m and k_e, reused in weeks 6 and 7 |
| 6 | [Localization (odometry)](week6) | Drove the digit 32 in the simulator and on the robot, compared the odometry |
| 7 | [Go-to-goal](week7) | Drove the digit 32 as waypoints with three controllers, simulator vs. robot |
| 8 | Midterm exam | |
| 9 | [Gyroscope parameters](week9) | Built a physical model of the gyro ($b_0$, $d$, $\omega_{max}$, $\Delta$, $\sigma$, $s$) and a simulated gyro, `gyro_sim` |

## Assignments

| Assignment | Topic | What I did |
|---|---|---|
| 1 | [Motor parameter identification](Assignment%201) | First-order model of both motors: steady speed, time constant, k_e and dead zone at every PWM level, checked on a complex input signal |

## Each folder has

- the TRIK program (`.qrs`),
- the logs from the robot (and the simulator),
- the Colab notebook with the analysis,
- a README with the method and the results.

**Tools:** TRIK Studio (visual blocks, JavaScript), Google Colab (Python: NumPy, pandas, SciPy, Matplotlib).

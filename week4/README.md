# Week 4 — PID Line Following Competition

YouTube video: https://youtu.be/FvkpTiQymGA?si=RE_Qg1kCHN-UQdMV

PID-based line following using two sensors in TRIK Studio.

## Setup

- A4 — left sensor
- A3 — right sensor
- M3 — left motor
- M4 — right motor

## Controller

```text
error = (A4 - left_reference) - (A3 - right_reference)

u = P + I + D

M3 = base_speed + u
M4 = base_speed - u
```

## Final Parameters

- Kp = 1.0
- Ki = 0.04
- Kd = 6.0
- Base speed = 30
- Max correction = ±25
- Control interval = 10 ms

The initial sensor readings are stored as reference values.

The integral is limited to ±60 to reduce integral windup.

The final steering correction is limited to ±25.

## Result

The controller continuously changes the difference between the left and right motor speeds to keep the robot aligned with the line.



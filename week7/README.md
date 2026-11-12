# Week 7: Go-to-goal

**Goal:** drive the digit **32** (the last two digits of my student ID) as a list of waypoints with a go-to-goal controller, using the week 6 odometry as the pose estimate. Each program runs first in the TRIK Studio simulator, then on the real robot, and the runs are compared in Colab. Three versions with three different controllers were tuned and tested.

## Files in this folder

| File | What it is |
|---|---|
| `go_to_goal_v1_sim.qrs`, `go_to_goal_v1_real.qrs` | V1: P controller |
| `go_to_goal_v2_sim.qrs`, `go_to_goal_v2_real.qrs` | V2: turn, then drive |
| `go_to_goal_v3_sim.qrs`, `go_to_goal_v3_real.qrs` | V3: cos controller |
| `odometry_data_sim_32_v1.txt` … `_v3.txt` | Simulator logs |
| `odometry_data_real_32_v1.txt` … `_v3.txt` | Real robot logs |
| `lab7_task_go_to_goal.ipynb` | Colab notebook: trajectory plot, waypoint-miss table and run times for all three versions |

## 1. Waypoints

Start at (0, 0), facing +x. Waypoints in cm:

```
digit 3: (30,0) → (30,30) → (0,30) → (30,30) → (30,60) → (0,60)
gap:     (0,60) → (50,60)
digit 2: (50,60) → (80,60) → (80,30) → (50,30) → (50,0) → (80,0)
```

## 2. The controller

In every pass of the loop, the program updates the odometry, then computes (Lab 7 lecture):

| Quantity | Formula |
|---|---|
| Distance to goal | `ρ = √((x_g − x)² + (y_g − y)²)` |
| Bearing to goal | `θ_g = atan2(y_g − y, x_g − x)` |
| Heading error | `e_θ = wrap(θ_g − θ)` into `(−π, π]` |
| Angular velocity | `ω = K_θ · e_θ` |
| Wheel powers | `v_R = v + ω·B/2`, `v_L = v − ω·B/2` (M4 right, M3 left, in %) |

When `ρ < ε`, the robot switches to the next waypoint. After the last one it stops.

The three versions differ **only in the forward power `v`**:

| Version | Forward power `v` | Source in the course materials |
|---|---|---|
| V1: P controller | `min(K_v·ρ, v_max)` | Lab 7 README table |
| V2: turn, then drive | `0` while `\|e_θ\| > turnErr`, otherwise `max(v_min, min(K_v·ρ, v_max))` | `localization+point_to_point.py` (`forward = 0` above an angle limit, `minPower`) |
| V3: cos controller | `min(K_v·ρ, v_max) · cos(e_θ)` | `Fixed_1x1_trajectory.qrs` (`mov = 50 * dist * cos(err)`); the `cos · tanh` law in `Go-to-goal_modeling.ipynb` |

- **V1** drives at full power even when the robot points the wrong way, so it turns while driving.
- **V2** rotates on the spot until the goal is within `turnErr`, then drives (the lecture's turn-then-drive choice).
- **V3** slows down as the heading error grows: full speed at 0°, none at 90°, backwards when the goal is behind.

TRIK has no `tanh`, so V3 limits the speed with the lab README's `min(K_v·ρ, v_max)` instead.

### Gains

| | `K_v` | `K_θ` | `v_max`, % | `v_min`, % | `ε`, cm | `turnErr`, rad |
|---|---|---|---|---|---|---|
| V1 | 4 | 8 | 40 | - | 5 | - |
| V2 | 4 | 5 | 50 | 18 | 3 | 0.5 |
| V3 | 4 | 8 | 40 | - | 5 | - |

How they were chosen:
- **V1 and V3 share all gains**, so `cos(e_θ)` is the only difference between them.
- **`ε` depends on the minimum power.** In V1 and V3 the power near a waypoint is `K_v·ρ`, which is 20 % at ρ = 5 cm, just above the 18 % `minPower` of `localization+point_to_point.py`. A smaller ε could leave the real robot stopped just short of a waypoint. V2 never drops below `v_min` = 18 %, so ε = 3 cm is safe.
- **`turnErr` = 0.5 rad** (the course script uses 1.0) gives sharper corners. Smaller values could make the on-the-spot turning power `K_θ·turnErr·B/2` too small to move the real robot.
- **`K_θ`**: 8 gives tight turns while driving in V1 and V3. V2 does its big turns on the spot, so 5 is enough.

## 3. How the program works

The block structure follows the professor's `Fixed_1x1_trajectory.qrs`:

1. Clear Encoder, Remove File (old log), and a Function that sets the constants, the waypoint arrays `xg`, `yg` and the pose.
2. **Outer loop** `i < n`: one pass per waypoint.
3. **Inner loop** `rho > eps`:
   - odometry Function;
   - heading-error wrap;
   - control Function;
   - Motors M3, M4;
   - Write To File;
   - Timer 20 ms.
4. Next waypoint (`i = i + 1`), and after the last one, Stop Motors.

**Odometry** (current Lab 6 table, E3 = left wheel, E4 = right wheel):

```
S = (φR + φL) · r / 2        θ = (φR − φL) · r / B        (φ = total encoder angle, rad)
x ← x + ΔS · cos(θ)          y ← y + ΔS · sin(θ)
```

The wrap is a loop (`while |e_θ| > π: e_θ −= sgn(e_θ)·2π`) rather than a single `if`. In the simulator the robot turns the same way at every reversal, so θ grows past 720°, and one subtraction is not enough.

**Simulator vs real program.** Same blocks and gains. Only these differ:

| | `_sim.qrs` | `_real.qrs` |
|---|---|---|
| `r`, `B` | 2.8 cm, 15.4 cm (TRIK Studio 2D robot) | 4.3 cm, 19.6 cm (real robot, measured in week 2) |
| Log file | `odometry_data_sim_32_vN.txt` | `odometry_data_real_32_vN.txt` |
| Marker Down / Marker Up | yes, so the simulator draws the path | no |

## 4. How the data is organized

One line per sample, no header: `x,y,time`, with `x`, `y` in **cm** and `time` in **ms** since the program started. For example, `30.086,-0.389,1518` means: 1.518 s after the start, the robot believed it was at x = 30.1 cm, y = −0.4 cm.

- Simulator: one line every 20 ms.
- Real robot: about one line every 60 ms (writing to the file takes time on the robot).

## 5. Results

| | Run time, sim / real | Mean waypoint miss, sim / real | Worst waypoint miss, sim / real |
|---|---|---|---|
| V1: P controller | 35.8 s / 16.3 s | 2.6 cm / 1.5 cm | 5.0 cm / 4.5 cm |
| V2: turn, then drive | 41.9 s / 19.4 s | 2.0 cm / 1.1 cm | 2.9 cm / 2.9 cm |
| V3: cos controller | 36.7 s / 18.5 s | 3.1 cm / 1.9 cm | 4.9 cm / 4.3 cm |

The mean and worst misses are over waypoints 1 to 12, from the notebook's waypoint-miss table.

- **Every miss is within ε,** so every version reached all 12 waypoints in both runs.
- **The real robot is about twice as fast:** bigger wheels and faster motors at the same power.
- **V2 is the most accurate and the slowest** in both runs.
- **End point:** the tape measurement was not required this week. The last odometry lines are (75.5, 0.1), (78.0, 2.2) and (75.8, −0.9) cm for V1, V2 and V3. Each is within ε of (80, 0), because the program stops as soon as the robot is within ε of the last waypoint.

## Discussion

Every version cuts each corner by up to ε (5 cm for V1 and V3, 3 cm for V2), because it switches to the next waypoint ε early, which is why V2 has the smallest misses. V1 swings out widest at the two 180° reversals, because it keeps full forward power while turning. V3 has the same gains but slows down with `cos(e_θ)`, so in the simulator it stays closer to the planned path (at most 3.2 cm away, against 5.0 cm for V1). On the real robot, V2 overshot (30,0), (50,0) and (80,60) by about 3 cm: it switches only 3 cm before each corner, and the robot keeps rolling for a moment after the switch (T_m ≈ 0.2 s in week 5). V1 and V3 weave across the straight lines on the real robot but not in the simulator, which matches the lecture's "K_θ too high" oscillation: the real motors lag and the loop runs every 60 ms instead of 20 ms, while V2 with K_θ = 5 barely weaves. The remaining real-vs-model gap in these plots is a **control** problem, because both curves are odometry and the real robot still reached every waypoint within ε by its own estimate; localization errors such as wheel slip or a wrong r or B cannot show up in these logs at all.

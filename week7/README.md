# Week 7: Go-to-goal

**Goal:** drive the digit **32** (the last two digits of my student ID) as a list of waypoints. The robot estimates its own position with the week 6 odometry and steers to one waypoint after another with a go-to-goal controller. Each program runs first in the TRIK Studio simulator, then on the real robot, and the two runs are compared in Colab. Three versions with three different controllers were tuned and tested.

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

The robot starts at (0, 0), facing +x. The digit is a list of 12 waypoints in cm, stored in the program as two arrays, `xg` (x values) and `yg` (y values):

```
digit 3: (0,0) → (30,0) → (30,30) → (0,30) → (30,30) → (30,60) → (0,60)
gap:     (0,60) → (50,60)
digit 2: (50,60) → (80,60) → (80,30) → (50,30) → (50,0) → (80,0)

xg = {30, 30, 0, 30, 30, 0, 50, 80, 80, 50, 50, 80}
yg = {0, 30, 30, 30, 60, 60, 60, 60, 30, 30, 0, 0}
```

- (30,30) appears twice: the robot draws the middle bar of the 3 out to (0,30) and comes back along it.
- The start point is not in the arrays, because the robot is already there.

## 2. One control step

The program repeats the steps below once every 20 ms until the robot reaches the current waypoint `i`. Step 1 is the week 6 odometry; steps 2, 3, 4, 6 and 7 are the go-to-goal table of the Lab 7 lecture. Program variable names are in parentheses.

### Step 1: Odometry, where am I? (`x`, `y`, `theta`)

E3 is the left wheel, E4 the right wheel:

```
aL = encoder3 · π / 180         aR = encoder4 · π / 180        wheel angles, degrees → rad
S     = (aR + aL) · r / 2       path travelled by the robot's centre
theta = (aR − aL) · r / B       course: how far the robot has turned from its start direction
x = x + (S − S_old) · cos(theta)
y = y + (S − S_old) · sin(theta)
```

- The encoders are cleared at the start, so the robot starts at x = 0, y = 0, `theta` = 0.
- `theta` comes straight from the total wheel angles, without a running sum, so small rounding errors do not pile up from step to step.
- `x` and `y` must be added up step by step, because where the robot ends up depends on the direction of every piece of the path.
- Each step assumes `theta` stays the same for 20 ms. The steps are short, so the error from this is small.

### Step 2: Distance to waypoint (`rho`)

```
dx = xg[i] − x        dy = yg[i] − y
rho = √(dx² + dy²)
```

`rho` sets the forward speed (slow down near the waypoint) and decides when the waypoint is reached.

### Step 3: Bearing to waypoint, with `atan2` (`thetaG`)

```
thetaG = atan2(dy, dx)
```

`thetaG` is the direction from the robot to the waypoint, measured from the +x axis, between −π and π.

Why `atan2` and not `atan(dy / dx)`:
- **`atan` cannot tell front from back.** It only returns angles between −90° and +90°. For dx = 10, dy = 10 and for dx = −10, dy = −10, `dy / dx` is 1 both times, so `atan` gives 45° for both. `atan2` looks at the signs of `dy` and `dx` separately and gives 45° and −135°.
- **`atan(dy / dx)` breaks when dx is close to 0.** That happens on every vertical line of the 32, for example from (30,0) to (30,30). When the robot drifts a little left or right of x = 30, `dy / dx` blows up and changes sign, so `atan` jumps between +90° and −90° (and at dx = 0 it divides by zero). `atan2` stays

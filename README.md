# Assistive Household Mobile Manipulator

A simulated mobile manipulator, written in pure Python. The base drives across a household room,
works out where it is against landmarks, and builds a map as it goes. Once it arrives, a 3-DOF arm
plans and drives a controlled approach to a light switch, stopping short of contact under a
model-based torque controller.

No ROS, no Gazebo, no hardware. Runtime dependencies are numpy and matplotlib. The whole system
lives in one notebook that runs top to bottom in about 45 seconds and checks itself with 167 tests.

Built for CS4973 Projects in Robotics (Summer II 2026), Scenario C.

---

## What it does

**Mobile base**

- **Kinematics.** Unicycle model with a differential-drive layer, integrated as an exact circular
  arc rather than by forward Euler, so accuracy does not depend on the step size.
- **Sensing.** An 8 m × 6 m room, nine obstacles around a single 1.2 m doorway, five landmarks.
  Odometry noise enters at the velocity level (Thrun's velocity motion model); lidar and
  range/bearing noise go onto the readings. One `NoiseParams` object scales all of it together.
- **Planning.** A probabilistic roadmap and a potential field, built to be compared rather than to
  pick a winner in advance.
- **Localization.** An EKF fusing wheel commands with landmark readings.
- **Mapping.** A log-odds occupancy grid built from the filter's pose estimate, not the true one.

**Arm**

- **Kinematics.** A DH model, forward kinematics checked against hand calculations, damped
  least-squares inverse kinematics.
- **Jacobian.** Taken by central differences from the same forward kinematics, cross-checked
  against a hand-derived determinant, with both families of singularity identified and analysed.
- **Dynamics.** The full `M(q)q̈ + C(q,q̇)q̇ + g(q)`, with each link represented exactly as a
  uniform rod. Four controllers compared on one trajectory: PD, PD + gravity, PID, computed torque.
- **Trajectories.** Quintic and trapezoidal profiles on a shared path parameter, joint-space
  against Cartesian, then driven through the dynamics rather than merely plotted.

---

## Results

Every figure below is printed by the notebook itself, from a single seed.

| | |
|---|---|
| Localization RMSE | **0.015 m**, against 0.241 m for dead reckoning over the same drive |
| Map accuracy | **91.7%** of committed cells correct from the EKF pose (79.9% from odometry alone) |
| Planned path | 8.04 m over 8 waypoints, 0.30 m minimum clearance |
| IK convergence | switch reached in **17 iterations**, 0.08 mm residual |
| Joint tracking | **0.03°** worst error under computed torque, where plain PD settles 8.2° short |
| Dynamics check | total energy conserved to **2 parts in 10⁶** over a two-second free swing |
| Driven approach | **0.07 mm** off the commanded straight line, stopping 8.00 cm short of the switch, at rest |
| Tests | **167**, all passing, run per task and again together at the end |

The numbers that are *not* flattering are reported alongside these in the notebook: computed torque
degrades 121-fold when the payload is 0.7 kg heavier than its model, PID goes unstable past about
1 kg of that error, and a Cartesian trajectory silently exceeds the elbow's acceleration limit by 5%
until its duration is stretched.

---

## Running it

```bash
python3 -m venv .venv && source .venv/bin/activate
pip install numpy matplotlib jupyter
jupyter notebook Project_Scenario_C.ipynb
```

Then run all cells. Developed against Python 3.13, numpy 2.5, matplotlib 3.11.

Every random draw derives from one `MASTER_SEED`, through per-purpose streams, so the whole
notebook is reproducible and adding a random draw in one place does not shift the numbers
everywhere else. Change the seed for a different run.

---

## How it is put together

```
speed commands ─▶ unicycle model ─▶ true pose ─▶ sensors ─▶ noisy odometry
                                        │                   noisy landmarks
                                        │                   noisy lidar
                                        ▼                        │
                                 ground truth               ┌────┴────┐
                                 (scoring only)             │   EKF   │
                                                            └────┬────┘
                          PRM / potential field                  │ estimated pose
                                    │                            ├──▶ occupancy grid
                                    ▼                            │
                            planned waypoints ───────────────────┘
                                                                 │
                                              arrival pose ──────┘
                                                     │
                        world ─▶ base frame ─▶ IK ─▶ trajectory ─▶ computed torque ─▶ joint angles
```

The estimated pose, not the true one, is what feeds the map and the arm — so every centimetre of
localization error becomes a centimetre of arm error, which is the point of measuring it.

---

## Testing

Each task ends with its own test block, and all of them run again together at the end. Tests are
seeded, so none of them pass only some of the time.

They are mostly property tests rather than golden values: splitting a command in half changes
nothing; the robot holds a constant distance from the centre of its turn; the gripper is never
further from the shoulder than the sum of the link lengths; `Ṁ - 2C` is skew symmetric; total
energy holds while the arm falls with its motors off; the error a PD controller settles at is
exactly the gravity torque divided by its gain.

Several exist to keep the notebook's own prose honest — for instance, a test asserts that the
joint-space path *does* leave the straight line by centimetres, so the section arguing for Cartesian
interpolation cannot quietly become true of both.

---

## Design decisions

| Decision | Why |
|---|---|
| EKF over a particle filter | The belief is one tight blob in three dimensions, the robot starts on a known dock, and landmarks are identified. That is the case a Gaussian describes well. |
| PRM over the potential field | The field is competitive when tuned (8.15 m, better clearance) but works only in a narrow band of gains and cannot report failure. The roadmap returns an explicit `None`. |
| Exact arc integration | For a constant twist the closed form is exact, so error stays at machine precision however coarse the step. Euler under-turns consistently. |
| Finite-difference Jacobian | It cannot disagree with the forward kinematics actually in use. A hand-derived Jacobian with a sign error still converges, just worse. |
| Damped least squares | Near a singularity the plain pseudoinverse asks for 200 rad/s where damping asks for 0.25 — and the damped run ends up closer to the target as well as safer. |
| Computed torque | Cancelling the dynamics leaves one error equation that is the same at every configuration, so one pair of gains works everywhere instead of being a compromise across poses. |
| Quintic over trapezoid for contact | The trapezoid is 17% quicker but steps its acceleration four times. The arm is handing over to a force controller, and arriving with a step in commanded torque is the wrong way to start that. |

---

## Status

Tasks 1–9 of the 11-task brief are implemented and tested. Task 10 (force/impedance control for the
contact task) and Task 11 (end-to-end integration and demo) are outstanding; the arm currently
finishes its approach stopped 8 cm from the switch, which is the state Task 10 is written to take
over from.

**Known limitations**, listed in full in the notebook's closing sections rather than left to be
discovered: landmark correspondence is known, so there is no data association; obstacles are
axis-aligned rectangles; noise is zero-mean Gaussian with no systematic bias; the planner ignores
heading and never replans against the map it built; the arm has no self-collision or floor checking;
and the arm model is rigid and frictionless.

---

## Layout

```
Project_Scenario_C.ipynb      the system: 110 cells, tasks 1-9, 167 tests
Final_Project_Assignment.pdf  the brief this was built against
```

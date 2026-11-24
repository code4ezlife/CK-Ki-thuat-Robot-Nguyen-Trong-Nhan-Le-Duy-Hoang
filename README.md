# Safe Navigation for Differential-Drive Robots

## 1. Introduction

This project develops and evaluates a safe navigation system for differential-drive mobile robots in dynamic environments.

The proposed approach integrates a **Kalman Filter (KF)** with the **Dynamic Window Approach (DWA)** to estimate and predict moving obstacle trajectories, helping the robot avoid potential collisions more effectively.

The project also implements **A\*** for global path planning and **ORCA** as a comparison method.

The system is developed and evaluated using MATLAB simulations.

## 2. How to Run

**Requirements:** MATLAB.

1. Download or clone this repository.
2. Open MATLAB and navigate to the project folder.
3. Run one of the following scripts:

```matlab
run_S1  % Scenario 1
run_S2  % Scenario 2
run_S3  % Scenario 3
run_S4  % Scenario 4
run_S5  % Scenario 5
```

To perform parameter sensitivity analysis:

```matlab
sensitivity_S2
sensitivity_S4
```

## 3. Simulation Scenarios

The project includes five scenarios to evaluate the performance of DWA, DWA-KF, and ORCA:

| Scenario | Description |
|---|---|
| **S1 – Static Obstacles** | The robot navigates through a warehouse environment containing shelves and columns, without moving obstacles. |
| **S2 – Crossing Pedestrian** | One pedestrian crosses the robot's path perpendicularly. |
| **S3 – Narrow Corridor** | The robot encounters a pedestrian moving in the opposite direction inside a narrow corridor. |
| **S4 – Multiple Pedestrians** | Six pedestrians move across the environment, testing navigation in a crowded dynamic environment. |
| **S5 – Blocked Path and Sudden Direction Change** | The robot encounters a blocked path and a pedestrian that suddenly changes direction. |

Each scenario is repeated 50 times under different randomized conditions to compare the algorithms in terms of safety, completion time, path length, and motion smoothness.
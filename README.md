# Reverse Manoeuvre Algorithm

This project provides an automated control solution for a differential robot tasked with parking a cart in space-constrained environments, where forward movement is not possible.

To run the simulation: `python main.py --config configs/overshoot.yaml`

## Architecture

The code can be divided into three main componentes:

### Geometry Solver
Calculates a feasible trajectory based on the robot's initial pose and the target docking location. In standard scenarios, the solver constructs a path consisting of an initial straight line, a single circular turning arc, and a final straight-line approach to the target. When the robot's initial pose is already past the entry point of the primary turning circle, the solver implements and overshoot logic, that calculates a second tangent circle to compensate for the robot's pose and redirect it back toward the target alignment.

| Standard Case | Overshoot Case |
| :--- | :--- |
|![Overshoot Path](./assets/standard_path.png) | ![Standard Path](./assets/overshoot_path.png)|

The resulting geometry is converted into a discrete set of coordinates $(x,y)$ used as reference for the controller.

### Controller
Implements a kinematic model of the robot-cart system. The robot state is defined as 

$$state = [x_{robot}, y_{robot}, \theta_{heading}, \gamma_{hitch}]$$

and it can be controlled with its linear velocity ($v_x$) and angular velocity ($\omega$). However in this simulation, the controller maintains a constant backing $v_x$ focusing only on optimising $\omega$.

To account for the non-linear unstable dynamics of the system, the system uses a **Model Predictive Controller (MPC)** to find the optimal control sequence of $\omega$ over a defined prediction horizon. The MPC minimizes the multi-object cost function  

$$J = \sum_{i=1}^{N} (K_{dist} \cdot e_{cross}^2 + K_{turn} \cdot \omega^2 + K_{hitch} \cdot \frac{|\gamma|}{\gamma_{limit}})$$

That balances three critical objectives:
- **Cross-track error ($ e_{cross}$)**: Minimizes the $L_2$ distance between the cart and the reference path.
- **Hitch stability**: Penalizes large hitch angles ($\gamma$) to prevent jackknifing.
- **Steering effort**: Penalizes high angular velocities to prevent aggressive oscillations and ensure smooth mechanical movement.

The gains $K_{dist}$, $K_{turn}$ and $K_{hitch}$ and other parameters that affect the performance of the controller are tuned with **Optuna**. 

### Simulation 
The simulation environment provides a real-time, interactive interface to monitor robot performance and controller logic. It uses a decoupled architecture, running high-frequency physics and MPC calculations on a dedicated background thread to ensure control loop stability independently of the UI rendering speed.

<p align="center">
  <img src="./assets/results_now.png" width="600" title="Path Visualization">
  <br>
  <sub><b>Figure 1:</b> Reverse Manoeuvre Plot showing trajectory and telemetry.</sub>
</p>

The plot includes:
- **Trajectory Tracking**: View of robot and cart paths, including a "ghosting" effect of historical cart positions.
- **Live Telemetry**: Synchronized plots for Cross-Track Error and Hitch Angle to monitor stability.
- **Debug Overlays**: Geometry Solver's internal logic, including circle that define the turning.
- **Interactive UI**: Stop trigger and radio toggles to switch between standard control and debug visualization modes.



## Results

Coming soon ...




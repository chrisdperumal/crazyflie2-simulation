# Crazyflie2 Simulation Attempts

(This summary was generated using AI from my commit history)

This repository documents a series of experiments with the Crazyflie 2.0 in a
ROS/Gazebo simulation environment. The work appears to build on the CrazyS /
rotors simulator stack and focuses on getting a custom Crazyflie controller
running, then iterating on LQR feedforward control, timing, sensor data, and
motor output analysis.

The repository is intentionally more of a working research log than a polished
software package. It includes source changes, launch files, recorded ROS bag
data, generated catkin output, and plotting scripts used while debugging the
controller behavior.

## Project Goal

The main goal was to simulate a Crazyflie 2.0 and experiment with controller
logic that could track waypoint or hover commands using a custom LQR
feedforward controller.

The core controller work is centered around:

- `src/CrazyS/rotors_control/src/library/lqr_feedforward_controller.cpp`
- `src/CrazyS/rotors_control/include/rotors_control/lqr_feedforward_controller.h`
- `src/CrazyS/rotors_control/src/nodes/lqr_feedforward_controller_node.cpp`
- `src/CrazyS/rotors_gazebo/launch/crazyflie2_lqr_feedforward.launch`
- `src/CrazyS/rotors_gazebo/src/nodes/lqr_feedforward_waypoint.cpp`

## Summary of Attempts

### 1. Creating a New LQR Controller Node

The first major attempt was to add a new controller node for the Crazyflie
simulation. This introduced a custom `lqr_feedforward_controller` alongside
the existing attitude and position controller code from the simulator stack.

This phase included:

- Adding the CrazyS / rotors simulation packages into the workspace.
- Creating the LQR feedforward controller class and ROS node.
- Wiring the new controller into CMake and launch files.
- Creating or modifying Crazyflie launch flows for hover and waypoint testing.

Relevant history:

- `7cb028a` - `new node created`
- `7304c88` - `build files`
- `48f61d6` - `fixed changes on new lqr node`

### 2. Stabilizing the Feedforward Controller

After the node existed, the next attempt was to make the controller stable
enough to run in simulation. The history shows repeated edits to the controller
library, controller node, attitude/position controller variants, and hover /
waypoint examples.

This phase appears to have focused on:

- Connecting odometry and trajectory commands to the controller.
- Producing rotor velocity outputs from PWM or control inputs.
- Integrating Crazyflie-specific constants and motor constraints.
- Iterating on attitude, rate, yaw, hover, and XY control paths.

Relevant history:

- `d695f50` - `Stable changes with LQR lqr_feedforward_controller`
- `064fb0e` - `ignore build and logs directories`
- `6ce30fa` - `stable feedforward code`
- `1c5ad5d` - `-remove devel folder`

### 3. Adding Physical Constants and Controller Limits

The controller was then expanded with physical constants and saturation limits
for the Crazyflie. The source includes constants for mass, arm length, rotor
drag, motor thrust, motor velocity limits, command limits, and sampling time.

This phase seems aimed at making the simulated controller better reflect the
Crazyflie hardware model rather than relying only on generic simulator values.

Relevant history:

- `9f4b313` - `Added constants`

### 4. Debugging Rate and Timing Issues

The next visible attempt focused on timing and update-rate behavior. The
history explicitly mentions LQR rate and timing issues, and later commits add
scripts for comparing ROS time, system time, topic frequencies, rotor velocity
changes, and logged controller data.

This phase included:

- Looking at ROS time versus system time.
- Checking message frequency on odometry and motor speed topics.
- Recording ROS bag files for later analysis.
- Plotting motor speed and topic-frequency behavior.

Relevant files:

- `python-change.py`
- `pythondisplay.py`
- `src/CrazyS/rotors_control/src/nodes/plot_time_difference.py`
- `src/CrazyS/rotors_control/src/nodes/print_frequencies_rotorvelocities.py`
- `src/CrazyS/rotors_control/src/nodes/systime_rostime.py`

Relevant history:

- `03d10c6` - `LQR Rate and Timing Issues`
- December update commits adding plotting / timing scripts and recorded data

### 5. Iterating on LQR Behavior and Waypoints

Several November commits continued changing the LQR feedforward controller,
its ROS node, and the waypoint publisher. These updates suggest repeated test
runs where the controller math, scaling, desired state handling, odometry use,
and Crazyflie model parameters were adjusted.

This phase included:

- Updating the LQR gain and control input path.
- Revisiting the controller header and implementation.
- Modifying the Crazyflie xacro model.
- Updating waypoint-related code.
- Continuing to tune the controller after test runs.

Relevant history:

- `1fe03ba` - `LQR updates`
- `4b08c36` - `LQR updates`
- `5531baa` - `stable lqr updates`
- `8157988` - `Update lqr_feedforward_controller.cpp`
- `d7a1a71` - `Nov 12 Modifications`
- `b51c632` - `updated code nov 12`
- `6fe2f6b` - `Update lqr_feedforward_controller.cpp`

### 6. Final Broad Update

The latest default-branch commit is a broad December update. It changes the LQR
controller, attitude controller node, Crazyflie / Iris simulation model data,
logging output, plotting scripts, and recorded analysis data.

This looks like a final consolidation of the debugging work rather than a clean
release. It preserves useful artifacts from the controller experiments,
including large text logs and ROS bag files.

Relevant history:

- `4243c43` - `Everything update`

## Current State

The repository currently contains:

- A ROS/catkin workspace structure.
- CrazyS / rotors simulator source packages.
- Custom Crazyflie LQR feedforward controller work.
- Crazyflie launch files for Gazebo simulation.
- Recorded `.bag` files from simulation runs.
- Plotting and timing/debugging scripts.
- Generated `devel` and catkin metadata artifacts.

Because generated build outputs and ROS bag files are committed, the repository
is large. A future cleanup pass could make it easier to clone and understand by
removing generated outputs, moving bags to releases or external storage, and
keeping only source, launch files, configs, and a small sample dataset.

## Suggested Next Steps

If this project continues, the next useful cleanup would be:

- Add a `.gitignore` that excludes `build/`, `devel/`, `.catkin_tools/`, logs,
  swap files, and large generated artifacts.
- Move ROS bag recordings out of the main Git history if possible.
- Document the exact ROS, Gazebo, Ubuntu, and catkin versions used.
- Add one known-good launch command for reproducing the latest simulation.
- Keep controller notes near the LQR source explaining the current gain matrix,
  motor scaling, expected input topics, and output topics.
- Separate experiment scripts from core controller source.

## History-Based Note

This README was written from the visible GitHub commit history and source tree.
It summarizes the project as a sequence of attempts rather than as a finished,
reproducible release.

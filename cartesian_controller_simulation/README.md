# Cartesian Controller Simulation
This package provides a simulated robot for controller development and testing.

## Physics engine and rationales
We build the simulator on Todorov's [MuJoCo](https://mujoco.org/) physics engine, which
has been acquired and open-sourced by Google [here](https://github.com/deepmind/mujoco).
This gives us a strong environment to realistically test control and contact phenomena with minimal dependencies.


## Build and install
We use MuJoCo in [headless mode](https://mujoco.readthedocs.io/en/latest/programming.html?highlight=headless#using-opengl)
and don't need OpenGL-related dependencies.

**Inside the project's Docker image this is already handled for you:** MuJoCo is
downloaded and extracted to `/opt/mujoco-<version>` and the `MUJOCO_DIR`
environment variable is set (see `docker/dev/Dockerfile`). A plain
`colcon build` finds it automatically — no extra CMake arguments needed.

To build this package outside the image, make MuJoCo's pre-built
[library package](https://github.com/google-deepmind/mujoco/releases/)
available in one of these ways (checked in order):

1. Pass it on the command line:
   ```bash
   colcon build --cmake-args "-DMUJOCO_DIR=$HOME/mujoco-3.0.0" --packages-select cartesian_controller_simulation
   ```
2. Export the `MUJOCO_DIR` environment variable before building:
   ```bash
   cd $HOME
   wget https://github.com/google-deepmind/mujoco/releases/download/3.0.0/mujoco-3.0.0-linux-x86_64.tar.gz
   tar -xf mujoco-3.0.0-linux-x86_64.tar.gz
   export MUJOCO_DIR=$HOME/mujoco-3.0.0
   colcon build --packages-select cartesian_controller_simulation
   ```
3. Extract it to the default location `$HOME/mujoco-3.0.0`.


## Getting started
In a sourced terminal, run
```bash
export LC_NUMERIC="en_US.UTF-8"
ros2 launch cartesian_controller_simulation simulation.launch.py
```
The export might not be necessary on your system.
It fixes an eventual *locals* problem and makes sure that you see the robot visualization correctly in RViz.
The *launch* part will start a simulated world with a generic robot model.
You can call
```bash
ros2 control list_controllers
```
to get a list of controllers currently managed by the `controller_manager`.
All of them can be activated and tested in the simulator.
In contrast to `ROS1`, these controllers are nodes and you can also see them with `ros2 node list`.


## Hardware interfaces for controllers
Exposed interfaces per joint:

- `command_interfaces`: position, velocity, stiffness, damping
- `state_interfaces`: position, velocity

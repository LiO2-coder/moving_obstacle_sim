# moving_obstacle_sim

[中文版说明](./README.md)

`moving_obstacle_sim` is a standalone **ROS1 Noetic + Gazebo11** package for dynamic obstacle simulation.
It focuses on obstacle generation/control only, does not depend on other business packages in this workspace,
and can be directly reused by other ROS packages.

## 1. Features

- Primitive obstacle types: `box` / `sphere` / `cylinder`
- Full model data: `visual + collision + inertial`
- C++ `ModelPlugin` supports 6 motion types:
  - `static`
  - `line`
  - `sine`
  - `circle`
  - `ellipse`
  - `polygon`
- Unified motion parameters:
  - `enabled`
  - `speed`
  - `mode` (`loop` / `ping_pong` / `once`)
  - `align_yaw`
  - `z_lock`
  - `bounds` (fixed `bounce` behavior)
- Single YAML schema for both static and dynamic obstacles
- Multiple types and multiple instances supported
- `main.launch` spawns **no robot** by default (Gazebo + obstacles only)

## 2. Project Structure

```text
moving_obstacle_sim/
  CMakeLists.txt
  package.xml
  install.sh                                # one-click dependency installer
  README.md                                 # Chinese doc
  README_en.md                              # English doc
  include/moving_obstacle_sim/moving_obstacle_plugin.h
  src/moving_obstacle_plugin.cpp            # core trajectory plugin
  scripts/spawn_moving_obstacles.py         # YAML -> gazebo/spawn_sdf_model
  scripts/model_builders.py                 # SDF generation + inertia mapping
  launch/main.launch                        # standalone entry (Gazebo + obstacles)
  launch/spawn_moving_obstacles.launch      # inject into existing Gazebo
  launch/include/spawn_from_config.launch.xml
  config/moving_obstacles_demo.yaml         # demo config
  config/moving_obstacles_all_modes.yaml    # all-motion sample
  models/                                   # reference SDF templates
  urdf/primitive_obstacles.urdf.xacro       # URDF/xacro compatibility template
  docs/images/                              # screenshot placeholder folder
```

## 3. Requirements

- Ubuntu 20.04 (recommended)
- ROS1 Noetic
- Gazebo11 (Gazebo Classic)
- Catkin workspace

## 4. Deployment (for GitHub users)

1. Create workspace and clone repo

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone <YOUR_GITHUB_REPO_URL>.git
```

2. Install dependencies in one command

```bash
cd ~/catkin_ws/src/moving_obstacle_sim
chmod +x install.sh
./install.sh
```

Optional flags:

```bash
./install.sh --rosdistro noetic
./install.sh --workspace ~/catkin_ws
./install.sh --skip-apt
./install.sh --skip-rosdep-init
```

3. Build

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 5. Usage

### 5.1 Standalone launch (Gazebo + obstacles only)

```bash
roslaunch moving_obstacle_sim main.launch
```

Use all-motion sample:

```bash
roslaunch moving_obstacle_sim main.launch \
  obstacles_config:=pkg://moving_obstacle_sim/config/moving_obstacles_all_modes.yaml
```

### 5.2 Inject into an existing Gazebo session

```bash
roslaunch moving_obstacle_sim spawn_moving_obstacles.launch \
  config:=pkg://moving_obstacle_sim/config/moving_obstacles_demo.yaml
```

### 5.3 Call from another package

Include in your launch file:

```xml
<include file="$(find moving_obstacle_sim)/launch/spawn_moving_obstacles.launch">
  <arg name="config" value="pkg://your_pkg/config/your_obstacles.yaml" />
  <arg name="reference_frame" value="world" />
  <arg name="delete_if_exists" value="false" />
</include>
```

## 6. YAML Schema

```yaml
obstacles:
  - name: obs_box_1
    enabled: true
    shape:
      type: box            # box | sphere | cylinder
      mass: 3.0
      box_size: [0.6, 0.4, 1.0]
      radius: 0.3
      length: 1.0
      color_rgba: [0.8, 0.2, 0.2, 1.0]
    pose:
      xyz: [2.0, 0.0, 0.5]
      rpy: [0.0, 0.0, 0.0]
    motion:
      enabled: true
      type: line           # static | line | sine | circle | ellipse | polygon
      speed: 0.6
      mode: ping_pong      # loop | ping_pong | once
      align_yaw: true
      z_lock: true
      bounds:
        enabled: false
        x_min: -10.0
        x_max: 10.0
        y_min: -10.0
        y_max: 10.0
```

## 7. Screenshot Placeholders

Put screenshots under `docs/images/`, then enable these references:

```markdown
![Gazebo runtime](docs/images/screenshot_gazebo.png)
![Multi-obstacle trajectories](docs/images/screenshot_trajectories.png)
![LaserScan observation](docs/images/screenshot_laserscan.png)
```

The repository already includes `docs/images/.gitkeep`.

## 8. Quick Checks

1. Verify plugin output:

```bash
ls ~/catkin_ws/devel/lib/libmoving_obstacle_plugin.so
```

2. Verify model states topic:

```bash
rostopic echo -n 1 /gazebo/model_states
```

3. Invalid motion parameters are downgraded to static with `ROS_WARN`:

- `speed <= 0`
- zero-length line
- `circle.radius <= 0`
- `ellipse.a <= 0` or `ellipse.b <= 0`
- insufficient/degenerate polygon points

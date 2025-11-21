# Running the PX100 + Teleop Bridge

Follow these steps to launch the PincherX-100 robot arm and run the teleoperation bridge that streams joint input from the controller arm.

---

## 1. Source Your ROS2 Workspace

```bash
source /opt/ros/humble/setup.bash
source ~/interbotix_ws/install/setup.bash
source ~/ros2_ws/install/setup.bash
```
## 2. Launch the PX100 Robot Driver

```bash
ros2 launch interbotix_xsarm_control xsarm_control.launch.py \
  robot_model:=px100 \
  use_sim:=false
```

## 3. Run the Teleop Bridge

In new terminal

```bash
source ~/ros2_ws/install/setup.bash
ros2 run px_bridge_pkg teleop_bridge
```

---

## Colcon rebuild teleop_bridge.py

```bash
cd ~/ros2_ws
colcon build --packages-select px_bridge_pkg
source install/setup.bash
```

## Colcon rebuild of pincher mode or px100.yaml files

```bash
cd ~/interbotix_ws
colcon build --packages-select interbotix_xsarm_control
source install/setup.bash
```

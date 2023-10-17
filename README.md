# robotic_neck_control
 ROS2 package to apply PID position control over the linear actuators for pitch and roll manipulation.

## Dependencies
* Framework: [ROS2 Humble (desktop)](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
* [launch_utils](https://github.com/MonkyDCristian/launch_utils)

## Install and Compile
**Note:** Install [launch_utils](https://github.com/MonkyDCristian/launch_utils) in your workspace before follow this step
```
cd <path to your workspace src>/
git clone https://github.com/Robotic-Neck/robotic_neck_control.git
cd ..
colcon build --packages-select robotic_neck_control
```

## Install ROS packages dependencies with rosdep  
```
cd <path to your workspace src>/
rosdep install -i --from-path src --rosdistro humble -y
```

## Demo

Run the controller:
```
ros2 launch platform_controller platform_controller.launch.py
```

## Documentation

Motors velocity can be set beetween 5 and 11 rad/s.

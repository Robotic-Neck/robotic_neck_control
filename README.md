# robotic_neck_control
 ROS2 package to apply PID position control over the linear actuators for pitch and roll manipulation.

<p align="center">
  <img width="640" height="480" src="neck_mec_sim.png">
</p>

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
```
TODO
```

## Documentation
TODO

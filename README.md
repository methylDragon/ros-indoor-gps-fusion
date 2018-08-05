# ros-indoor-gps-fusion

An implementation of sensor fusion using the extended Kalman Filter (EKF) node from [robot_localization](http://wiki.ros.org/robot_localization) 

It includes a custom message adapter written for translating ROS messages from the marvelmind indoor 'GPS' ultrasonic beacons into message types that robot_localization can use. (hedge_msg_adapter)

Visit the in-depth step-by-step tutorial for how to do something like this yourself [here!](https://github.com/methylDragon/ros-sensor-fusion-tutorial)



### Pre-Requisite Knowledge

- ROS
- Linorobot stack (or other suitable robots using the standard ROS Navigation stack)
- Marvelmind Indoor 'GPS' beacon familiarity
- Highly recommended: Reading the [sensor fusion tutorial](https://github.com/methylDragon/ros-sensor-fusion-tutorial) alongside this



---

[![Yeah! Buy the DRAGON a COFFEE!](assets/COFFEE BUTTON ヾ(°∇°^).png)](https://www.buymeacoffee.com/methylDragon)
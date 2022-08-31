# Launching the simulation

```
ros2 launch f1tenth_gazebo f1tenth.launch.py
```

## Sending commands to vehicle 

This command commands the car to move forward
```
ros2 topic pub /cmd ackermann_msgs/msg/AckermannDriveStamped "{drive: {speed: 1.0, steering_angle: 0.0}}"
```

This command turns the steering accordingly (positive - right, negative - left)
```
ros2 topic pub /cmd ackermann_msgs/msg/AckermannDriveStamped "{drive: {speed: 0.0, steering_angle: 1.0}}"
```


### Resources
https://github.com/srmainwaring/steer_bot
https://github.com/osrf/car_demo

### Documenting everything cause im dumb enough to remember
In the future, instead of using state-space representation to model the kinematic behaviour of the car, it is best if we start using the ros2 control framework instead which provides us with more fidelity in terms of the dynamic behaviour of the car as it involves having the hardware transmission as well.
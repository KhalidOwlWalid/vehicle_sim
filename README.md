# Launching gazebo

```
ros2 launch gazebo_ros gazebo.launch.py
```

# Spawning robot

```
ros2 run vehicle_spawner_pkg spawn_vehicle mybot mybot 0.0 0.0 0.0
```

### Resources
https://github.com/srmainwaring/steer_bot
https://github.com/osrf/car_demo

### Documenting everything cause im dumb enough to remember
In the future, instead of using state-space representation to model the kinematic behaviour of the car, it is best if we start using the ros2 control framework instead which provides us with more fidelity in terms of the dynamic behaviour of the car as it involves having the hardware transmission as well.
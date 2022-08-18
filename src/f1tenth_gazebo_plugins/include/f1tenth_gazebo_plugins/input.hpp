#ifndef F1TENTH_GAZEBO_PLUGINS_INCLUDE_F1TENTH_GAZEBO_PLUGINS_INPUT_HPP_
#define F1TENTH_GAZEBO_PLUGINS_INCLUDE_F1TENTH_GAZEBO_PLUGINS_INPUT_HPP_

#include <string>

namespace f1tenth {
namespace model {
    struct Input {
        
        // Initialized Input acceleration and steering angle
        Input() : acceleration(0.0), steering_angle(0.0) {};

        // Input parameter
        double acceleration;
        double steering_angle;
    };
} // namespace model
} // namespace f1tenth

#endif
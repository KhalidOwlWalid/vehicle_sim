#ifndef F1TENTH_GAZEBO_PLUGINS_INCLUDE_F1TENTH_GAZEBO_PLUGINS_STATE_HPP_
#define F1TENTH_GAZEBO_PLUGINS_INCLUDE_F1TENTH_GAZEBO_PLUGINS_STATE_HPP_

namespace gazebo_plugin {
    namespace f1tenth {
        struct state {
            
            // Position
            double x;
            double y;
            double z;

            // Velocity
            double v_x;
            double y_x;
            double z_x;

            // Acceleration
            double a_x;
            double a_y;
            double a_z;
            
            // Angular velocity
            double r_x;
            double r_y;
            double r_z;

            // Orientation
            double yaw;
        
        };
    } // namespace f1tenth
} // namespace gazebo_plugin

#endif
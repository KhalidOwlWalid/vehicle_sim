#ifndef F1TENTH_GAZEBO_PLUGINS_INCLUDE_F1TENTH_GAZEBO_PLUGINS_STATE_HPP_
#define F1TENTH_GAZEBO_PLUGINS_INCLUDE_F1TENTH_GAZEBO_PLUGINS_STATE_HPP_

namespace utils {
    struct position{
        double x;
        double y;
        double z;
    };

    struct velocity{
        double v_x;
        double v_y;
        double v_z;
    };

    struct acceleration{
        double a_x;
        double a_y;
        double a_z;
    };

    struct angular_velocity{
        double r_x;
        double r_y;
        double r_z;
    };

    struct orientation{
        double roll;
        double pitch;
        double yaw;
    };

    // TODO(Khalid): Use templates! I hate this format
    class State {
        
        public: struct position p;
        public: struct velocity v;
        public: struct orientation o;

        /// \brief Default constructors
        public: State()
        {
            p.x = 0.0;
            p.y = 0.0;
            p.z = 0.0;
            v.v_x = 0.0;
            v.v_y = 0.0;
            v.v_z = 0.0;
            o.roll = 0.0;
            o.pitch = 0.0;
            o.yaw = 0.0;
        }

        /// \brief Constructor
        /// \param[in] _x x position in meters.
        /// \param[in] _y y position in meters.
        /// \param[in] _z z position in meters.
        /// \param[in] _v_x velocity in the x-direction in m/s.
        /// \param[in] _v_y velocity in the y-direction in m/s.
        /// \param[in] _v_z velocity in the z-direction in m/s.
        // public: State(T _x, T _y, T _z, T _v_x, T _v_y, T _v_z)
        // :  p(_x, _y, _z), v(_v_x, _v_y, _v_z)
        // {
        // }

        /// \brief Destructor
        public: virtual ~State()
        {
        }
        // TODO(Khalid): Create a getter function
    };

} // namespace utils

#endif
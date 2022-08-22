// Copyright (c) 2013, Markus Bader <markus.bader@tuwien.ac.at>
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the company nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <gazebo/common/Events.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/PID.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <f1tenth_gazebo_plugins/f1tenth_model_plugin.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/node.hpp>
#ifdef IGN_PROFILER_ENABLE
#include <ignition/common/Profiler.hh>
#endif
#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <memory>
#include <string>
#include <vector>

#include <f1tenth_gazebo_plugins/input.hpp>
#include <f1tenth_gazebo_plugins/state.hpp>

namespace gazebo_plugins
{
class GazeboRosF1TenthModelPrivate
{
public:

  enum {
    FRONT_RIGHT,

    FRONT_LEFT
  };

  /// Callback to be called at every simulation iteration.
  /// \param[in] info Updated simulation info.
  void OnUpdate(const gazebo::common::UpdateInfo & info);

  /// A pointer to the GazeboROS node.
  gazebo_ros::Node::SharedPtr ros_node_{nullptr};

  /// Command publisher
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_topic_pub_;

  // Command Subscription
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr sub_cmd_;

  /// Update period of the simulation
  double update_period_;

  /// Keep last time an update was published
  gazebo::common::Time last_sim_update_time_;

  // Keep track of last cmd time
  gazebo::common::Time last_cmd_time_;

  /// Pointer to the update event connection.
  gazebo::event::ConnectionPtr update_connection_;

  // Pointer to model
  gazebo::physics::ModelPtr model_;

  ignition::math::Pose3d model_world_pos_;
  ignition::math::Pose3d cur_pos;

  // Command queue from cmd callback
  std::queue<std::shared_ptr<ackermann_msgs::msg::AckermannDriveStamped>> command_Q_;
  std::queue<gazebo::common::Time> command_time_Q_;

  // Model input
  f1tenth::model::Input desired_input_;
  f1tenth::model::Input actual_input_;

  // TODO(Khalid) :  Use sdf to define this
  double max_steering_rate;
  double max_steering_angle;

  /// \brief Front Left Steering Joint
  gazebo::physics::JointPtr frontLeftSteeringJointPtr;

  /// \brief Front Right wheel steering joint
  gazebo::physics::JointPtr frontRightSteeringJointPtr;

  /// \brief PiD Control for the front left wheel steering input
  gazebo::common::PID frontLeftWheelSteeringPID;

  /// \brief PID Control for the front right wheel steering input
  gazebo::common::PID frontRightWheelSteeringPID;

  /// \brief Base link
  gazebo::physics::LinkPtr baseLinkPtr;

  double frontLeftSteeringAngle = 0;
  double frontRightSteeringAngle = 0;

  double frontLeftSteeringCmd = 0;

  gazebo_plugin::f1tenth::state state_;
  ignition::math::Pose3d offset_;

  // cmd callback function
  void OnCmd(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);
};

GazeboRosF1TenthModel::GazeboRosF1TenthModel()
: impl_(std::make_unique<GazeboRosF1TenthModelPrivate>())
{
}

GazeboRosF1TenthModel::~GazeboRosF1TenthModel()
{
}

void GazeboRosF1TenthModel::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  // ROS node
  impl_->ros_node_ = gazebo_ros::Node::Get(sdf);

  impl_->model_ = model;

  // Get QoS profiles
  const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();

  // Joints
  std::string drive_topic_ = "cmd";
  if (!sdf->HasElement("drive_topic")) {
    RCLCPP_INFO(impl_->ros_node_->get_logger(), "Missing <drive_topic> tag, defaults to %s", drive_topic_.c_str());
  } else {
    drive_topic_ = sdf->GetElement("drive_topic")->Get<std::string>();
  }

  // Update rate
  double update_rate = 100.0;
  if (!sdf->HasElement("update_rate")) {
    RCLCPP_INFO(
      impl_->ros_node_->get_logger(), "Missing <update_rate>, defaults to %f", update_rate);
  } else {
    update_rate = sdf->GetElement("update_rate")->Get<double>();
  }

  std::string frontLeftSteeringJointName = impl_->model_->GetName() + "::"
    + sdf->Get<std::string>("front_left_steer_joint");
  impl_->frontLeftSteeringJointPtr =
    impl_->model_->GetJoint(frontLeftSteeringJointName);
  if (!impl_->frontLeftSteeringJointPtr)
  {
    std::cerr << "could not find front left steering joint" <<std::endl;
    return;
  }

  std::string frontRightSteeringJointName = impl_->model_->GetName() + "::"
    + sdf->Get<std::string>("front_right_steer_joint");
  impl_->frontRightSteeringJointPtr =
    impl_->model_->GetJoint(frontRightSteeringJointName);
  if (!impl_->frontRightSteeringJointPtr)
  {
    std::cerr << "could not find front right steering joint" <<std::endl;
    return;
  }

  std::string baseLinkName = impl_->model_->GetName() + "::"
    + sdf->Get<std::string>("base_link");
  impl_->baseLinkPtr = impl_->model_->GetLink(baseLinkName);
  if (!impl_->baseLinkPtr)
  {
    std::cerr << "could not find base link" << std::endl;
    return;
  }

  if (update_rate > 0.0) {
    impl_->update_period_ = 1.0 / update_rate;
  } else {
    impl_->update_period_ = 0.0;
  }

  impl_->max_steering_angle = 1.22;
  impl_->max_steering_rate = 1.0;

  impl_->last_sim_update_time_ = model->GetWorld()->SimTime();

  // Drive topic publisher
  impl_->drive_topic_pub_ = impl_->ros_node_->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
    drive_topic_, qos.get_publisher_qos(drive_topic_, rclcpp::QoS(1000)));

  impl_->sub_cmd_ = impl_->ros_node_->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
    "/cmd", 1,  std::bind(&GazeboRosF1TenthModelPrivate::OnCmd, impl_.get(), std::placeholders::_1));

  // Callback on every iteration
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRosF1TenthModelPrivate::OnUpdate, impl_.get(), std::placeholders::_1));

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // TODO(Khalid): Implement PID system
  // std::string paramName;
  // double paramDefault;

  // paramName = "flwheel_steering_p_gain";
  // paramDefault = 0;
  // if (sdf->HasElement(paramName))
  //   impl_->frontLeftWheelSteeringPID.SetPGain(sdf->Get<double>(paramName));
  // else
  //   impl_->frontLeftWheelSteeringPID.SetPGain(paramDefault);

  // paramName = "flwheel_steering_i_gain";
  // paramDefault = 0;
  // if (sdf->HasElement(paramName))
  //   impl_->frontLeftWheelSteeringPID.SetIGain(sdf->Get<double>(paramName));
  // else
  //   impl_->frontLeftWheelSteeringPID.SetIGain(paramDefault);

  // paramName = "flwheel_steering_d_gain";
  // paramDefault = 0;
  // if (sdf->HasElement(paramName))
  //   impl_->frontLeftWheelSteeringPID.SetDGain(sdf->Get<double>(paramName));
  // else
  //   impl_->frontLeftWheelSteeringPID.SetDGain(paramDefault);

  // paramName = "frwheel_steering_p_gain";
  // paramDefault = 0;
  // if (sdf->HasElement(paramName))
  //   impl_->frontRightWheelSteeringPID.SetPGain(sdf->Get<double>(paramName));
  // else
  //   impl_->frontRightWheelSteeringPID.SetPGain(paramDefault);

  // paramName = "frwheel_steering_i_gain";
  // paramDefault = 0;
  // if (sdf->HasElement(paramName))
  //   impl_->frontRightWheelSteeringPID.SetIGain(sdf->Get<double>(paramName));
  // else
  //   impl_->frontRightWheelSteeringPID.SetIGain(paramDefault);

  // paramName = "frwheel_steering_d_gain";
  // paramDefault = 0;
  // if (sdf->HasElement(paramName))
  //   impl_->frontRightWheelSteeringPID.SetDGain(sdf->Get<double>(paramName));
  // else
  //   impl_->frontRightWheelSteeringPID.SetDGain(paramDefault);

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // TODO(Khalid) : Find the offset of the car with respect to the world frame
  impl_->offset_ = impl_->model_->WorldPose();
}

void GazeboRosF1TenthModelPrivate::OnUpdate(const gazebo::common::UpdateInfo & info)
{
  gazebo::common::Time current_sim_time = info.simTime;

  // If the world is reset, for example
  if (current_sim_time < last_sim_update_time_) {
    RCLCPP_INFO(ros_node_->get_logger(), "Negative sim time difference detected.");
    last_sim_update_time_ = current_sim_time;
  }

  // Check if simulation needs to be updated (seconds since last sim update)
  double dt = (current_sim_time - last_sim_update_time_).Double();
  if (dt < update_period_) {
    return;
  }

  // Check if there is any command in queue
  if (!command_Q_.empty()) {

    // Get the desired input command (acceleration and steering angle)
    std::shared_ptr<ackermann_msgs::msg::AckermannDriveStamped> cmd_queue = command_Q_.front();

    desired_input_.speed = cmd_queue->drive.speed;
    desired_input_.steering_angle = cmd_queue->drive.steering_angle;

    // Remove the cmd that have been assigned
    command_Q_.pop();
  }

  actual_input_.steering_angle +=
      (desired_input_.steering_angle - actual_input_.steering_angle >= 0 ? 1 : -1) *
      std::min(max_steering_rate * dt, std::abs(desired_input_.steering_angle - actual_input_.steering_angle));

  // Update wheel steering position based on steering angle
  frontLeftSteeringJointPtr->SetPosition(0, actual_input_.steering_angle);
  frontRightSteeringJointPtr->SetPosition(0, actual_input_.steering_angle);

  ignition::math::Vector3d base_link_velocity = baseLinkPtr->RelativeLinearVel();

  std::cout << base_link_velocity[0] << std::endl;

  // Implement kinematic model here
  // updateState(actual_input_, )

  // Update time
  last_sim_update_time_ = current_sim_time;
}

void GazeboRosF1TenthModelPrivate::OnCmd(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
  command_Q_ .push(msg);
  command_time_Q_.push(model_->GetWorld()->SimTime());
  last_cmd_time_ = model_->GetWorld()->SimTime();
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosF1TenthModel)
}  // namespace gazebo_plugins
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
#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <memory>
#include <string>
#include <vector>

#include <f1tenth_gazebo_plugins/input.hpp>
#include <f1tenth_gazebo_plugins/state.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

namespace gazebo_plugins
{
class GazeboRosF1TenthModelPrivate
{
public:

  /// Callback to be called at every simulation iteration.
  /// \param[in] info Updated simulation info.
  void OnUpdate(const gazebo::common::UpdateInfo & info);

  /// A pointer to the GazeboROS node.
  gazebo_ros::Node::SharedPtr ros_node_{nullptr};

  /// Drive command publisher
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_topic_pub_;

  /// Car state twist (linear and angular velocity)
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr car_state_twist_pub_;

  /// Car State position (position and orientation)
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr car_state_pose_pub_;

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

  /// \brief Front Left Steering Joint
  gazebo::physics::JointPtr frontLeftSteeringJointPtr;

  /// \brief Front Right wheel steering joint
  gazebo::physics::JointPtr frontRightSteeringJointPtr;

  /// \brief Base link
  gazebo::physics::LinkPtr baseLinkPtr;

  /// \brief Stores the previous state of the car
  utils::State prev_car_state_;

  /// \brief Stores the current state of the car
  utils::State curr_car_state_; // Default construct - initialize everything to 0.0

  /// \brief Stores the intial offset of the car when plugin is loaded
  utils::position car_initial_offset_;

  // TODO(Khalid) :  Use sdf to define this
  double max_steering_rate = 1.0;
  double max_steering_angle = 1.22;
  double frontLeftSteeringAngle = 0;
  double frontRightSteeringAngle = 0;
  double frontLeftSteeringCmd = 0;

  // Model input
  f1tenth::model::Input desired_input_;
  f1tenth::model::Input actual_input_;

  // cmd callback function
  void OnCmd(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);

  void publishCarStatePose(utils::State &car_state, const gazebo::common::UpdateInfo & info);
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
  // TODO (Khalid): Create utility library
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
    std::cerr << "Could not find the " << std::endl;
    RCLCPP_ERROR(
      impl_->ros_node_->get_logger(), "Could not find %s. Please check your URDF file.", baseLinkName.c_str());
    return;
  }

  if (update_rate > 0.0) {
    impl_->update_period_ = 1.0 / update_rate;
  } else {
    impl_->update_period_ = 0.0;
  }

  impl_->last_sim_update_time_ = model->GetWorld()->SimTime();

  // Drive topic publisher
  impl_->drive_topic_pub_ = impl_->ros_node_->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
    drive_topic_, qos.get_publisher_qos(drive_topic_, rclcpp::QoS(1000)));

  impl_->car_state_twist_pub_ = impl_->ros_node_->create_publisher<geometry_msgs::msg::TwistStamped>(
    "car_state_twist", qos.get_publisher_qos("car_state_twist", rclcpp::QoS(1000)));

  impl_->car_state_pose_pub_ = impl_->ros_node_->create_publisher<geometry_msgs::msg::PoseStamped>(
    "car_state_pose", qos.get_publisher_qos("car_state_pose", rclcpp::QoS(1000)));

  impl_->sub_cmd_ = impl_->ros_node_->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
    "/cmd", 1,  std::bind(&GazeboRosF1TenthModelPrivate::OnCmd, impl_.get(), std::placeholders::_1));

  // Callback on every iteration
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRosF1TenthModelPrivate::OnUpdate, impl_.get(), std::placeholders::_1));

  // TODO(Khalid): Implement a PID control. refer https://github.com/osrf/car_demo/blob/master/car_demo/plugins/PriusHybridPlugin.cc#L1033

  impl_->car_initial_offset_.x = impl_->model_->WorldPose().X();
  impl_->car_initial_offset_.y = impl_->model_->WorldPose().Y();

  // Initialize the car state in its respective coordinate position upon load
  impl_->curr_car_state_.p.x = impl_->car_initial_offset_.x;
  impl_->curr_car_state_.p.y = impl_->car_initial_offset_.y;

  RCLCPP_INFO(impl_->ros_node_->get_logger(), "Model plugin loaded.");
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

  // Get the current state of the car

  actual_input_.steering_angle +=
      (desired_input_.steering_angle - actual_input_.steering_angle >= 0 ? 1 : -1) *
      std::min(max_steering_rate * dt, std::abs(desired_input_.steering_angle - actual_input_.steering_angle));

  // Get the radius of the turn using wheelbase
  double wheelBase = baseLinkPtr->CollisionBoundingBox().XLength();

  double max_speed = 5.0;

  // TODO (Khalid): Add acceleration mode

  if (desired_input_.speed < actual_input_.speed){
    // Slow down the car, create a gradual decrease in the car's velocity
    actual_input_.speed += std::max(-max_speed * dt, desired_input_.speed - actual_input_.speed);
  } else {
    // Move the car forward and increase speed gradually
    actual_input_.speed += std::min(max_speed * dt, std::abs(desired_input_.speed - actual_input_.speed));
  }
  
  // Kinematic model of the car
  // TODO (Khalid): Create documentation of this calculation
  // TODO (Khalid): Add slip
  double v_x = actual_input_.speed * cos(actual_input_.steering_angle);
  double v_y = actual_input_.speed * sin(actual_input_.steering_angle);
  double angular_velocity = v_x * tan(actual_input_.steering_angle) / wheelBase;
  double phi_dot = angular_velocity;
  double yaw = baseLinkPtr->WorldCoGPose().Yaw();

  double x_dot = v_x * cos(yaw) - v_y * sin(yaw);
  double y_dot = v_x * sin(yaw) + v_y * cos(yaw);
  
  curr_car_state_.p.x += x_dot * dt;
  curr_car_state_.p.y += y_dot * dt;
  curr_car_state_.o.yaw += phi_dot * dt;

  ignition::math::Pose3d pose(curr_car_state_.p.x, curr_car_state_.p.y, 0.0, 0.0, 0.0, curr_car_state_.o.yaw);
  ignition::math::Vector3d vel(x_dot, y_dot, 0);
  ignition::math::Vector3d ang(0, 0, phi_dot);

  // RCLCPP_INFO(ros_node_->get_logger(), "x:     %f | y:     %f | phi:   %f | ", curr_car_state_.p.x, curr_car_state_.p.x, curr_car_state_.o.yaw);
  // RCLCPP_INFO(ros_node_->get_logger(), "x_dot: %f | y_dot: %f |", x_dot, y_dot);

  // Update wheel steering position based on steering angle
  frontLeftSteeringJointPtr->SetPosition(0, actual_input_.steering_angle);
  frontRightSteeringJointPtr->SetPosition(0, actual_input_.steering_angle);

  // TODO (Khalid): Send velocity or torque command to the wheels instead
  // Reason: Kinematic model is not entirely accurate as it is basically manually setting its world position after some 
  // some calculations using state space representation. In simple terms, its just point mass
  // You can try removing SetLinearVel and it will still behave the same way
  // What I found is that SetLinearVel sets the rotation rate of the wheel and just makes the tire spin but not
  // necessarily accurate. It does not even achieve the same rotation speed that we would require!  
  model_->SetWorldPose(pose);
  model_->SetLinearVel(vel);
  model_->SetAngularVel(ang);

  publishCarStatePose(curr_car_state_, info);

  // Update time
  last_sim_update_time_ = current_sim_time;
}

void GazeboRosF1TenthModelPrivate::OnCmd(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
  command_Q_ .push(msg);
  command_time_Q_.push(model_->GetWorld()->SimTime());
  last_cmd_time_ = model_->GetWorld()->SimTime();
}

void GazeboRosF1TenthModelPrivate::publishCarStatePose(utils::State &car_state, const gazebo::common::UpdateInfo & info) {

  geometry_msgs::msg::PoseStamped car_state_msg;

  // Update time
  car_state_msg.header.stamp.sec = info.simTime.sec;
  car_state_msg.header.stamp.nanosec = info.simTime.nsec;

  // Frame ID
  car_state_msg.header.frame_id = "base_link";

  // Car position
  car_state_msg.pose.position.x = car_state.p.x;
  car_state_msg.pose.position.y = car_state.p.y;
  car_state_msg.pose.position.z = car_state.p.z;

  // Car orientation
  car_state_msg.pose.orientation.x = car_state.o.roll;
  car_state_msg.pose.orientation.y = car_state.o.pitch;
  car_state_msg.pose.orientation.z = car_state.o.yaw;

  car_state_pose_pub_->publish(car_state_msg);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosF1TenthModel)
}  // namespace gazebo_plugins
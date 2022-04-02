// Copyright 2015 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef PENDULUM_CONTROL__PENDULUM_CONTROLLER_HPP_
#define PENDULUM_CONTROL__PENDULUM_CONTROLLER_HPP_

// Needed for M_PI on Windows
#ifdef _MSC_VER
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#endif

#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/strategies/message_pool_memory_strategy.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "pendulum_msgs/msg/joint_command.hpp"
#include "pendulum_msgs/msg/joint_state.hpp"

namespace pendulum_control
{

/// Struct for storing PID controller properties.
struct PIDProperties
{
  /// Proportional constant.
  double p = 1;
  /// Integral constant.
  double i = 0;
  /// Derivative constant.
  double d = 0;
  /// Desired state of the plant.
  double command = M_PI / 2;
};

/// Provides a simple PID controller for the inverted pendulum.
class PendulumController : public rclcpp_lifecycle::LifecycleNode
{
public:
  /// Default constructor.
  /**
   * \param[in] period The update period of the controller.
   * \param[in] pid The properties of the controller.
   */
  PendulumController(std::chrono::nanoseconds period, PIDProperties pid)
  : LifecycleNode("pendulum_controller"),
    publish_period_(period), pid_(pid),
    message_ready_(false),
    command_pub_{create_publisher<pendulum_msgs::msg::JointCommand>(
        "pendulum_command", rclcpp::QoS(1).best_effort())},
    sensor_sub_{
      create_subscription<pendulum_msgs::msg::JointState>(
        "pendulum_sensor", rclcpp::QoS(1).best_effort(), [this]
          (pendulum_msgs::msg::JointState::ConstSharedPtr msg) -> void
        {
          on_sensor_message(msg);
        }, rclcpp::SubscriptionOptions(),
        std::make_shared<
          rclcpp::strategies::message_pool_memory_strategy::MessagePoolMemoryStrategy<
            pendulum_msgs::msg::JointState, 1>>())},
    setpoint_sub_{create_subscription<pendulum_msgs::msg::JointCommand>(
      "pendulum_setpoint", rclcpp::QoS(1).transient_local(), [this]
        (pendulum_msgs::msg::JointCommand::ConstSharedPtr msg) -> void
      {
        on_pendulum_setpoint(msg);
      },
      rclcpp::SubscriptionOptions(),
      std::make_shared<
        rclcpp::strategies::message_pool_memory_strategy::MessagePoolMemoryStrategy<
          pendulum_msgs::msg::JointCommand, 1>>())},
  controller_publisher_timer_{create_wall_timer(
      period, [this]()
      {
        if (next_message_ready()) {
          auto msg = get_next_command_message();
          command_pub_->publish(msg);
        }
      })}
  {
    command_message_.position = pid_.command;
    // Calculate the controller timestep (for discrete differentiation/integration).
    dt_ = publish_period_.count() / (1000.0 * 1000.0 * 1000.0);
    if (std::isnan(dt_) || dt_ == 0) {
      throw std::runtime_error("Invalid dt_ calculated in PendulumController constructor");
    }
  }

  /// Calculate new command based on new sensor state and PID controller properties.
  // \param[in] msg Received sensor message.
  void on_sensor_message(pendulum_msgs::msg::JointState::ConstSharedPtr msg)
  {
    ++messages_received;

    if (std::isnan(msg->position)) {
      throw std::runtime_error("Sensor value was NaN in on_sensor_message callback");
    }
    // PID controller algorithm
    double error = pid_.command - msg->position;
    // Proportional gain is proportional to error
    double p_gain = pid_.p * error;
    // Integral gain is proportional to the accumulation of error
    i_gain_ = pid_.i * (i_gain_ + error * dt_);
    // Differential gain is proportional to the change in error
    double d_gain = pid_.d * (error - last_error_) / dt_;
    last_error_ = error;

    // Calculate the message based on PID gains
    command_message_.position = msg->position + p_gain + i_gain_ + d_gain;
    // Enforce positional limits
    if (command_message_.position > M_PI) {
      command_message_.position = M_PI;
    } else if (command_message_.position < 0) {
      command_message_.position = 0;
    }

    if (std::isnan(command_message_.position)) {
      throw std::runtime_error("Resulting command was NaN in on_sensor_message callback");
    }
    message_ready_ = true;
  }

  /// Callback when a pendulum JointCommand message is received.
  // \param[in] msg The incoming message containing the position.
  void on_pendulum_setpoint(pendulum_msgs::msg::JointCommand::ConstSharedPtr msg)
  {
    set_command(msg->position);
    printf("Pendulum set to: %f\n", msg->position);
    fflush(stdout);
  }

  /// Retrieve the command calculated from the last sensor message.
  // \return Command message.
  const pendulum_msgs::msg::JointCommand & get_next_command_message() const
  {
    return command_message_;
  }

  /// True if the command message has been initialized.
  // \return True if the message is ready.
  bool next_message_ready() const
  {
    return message_ready_;
  }

  /// Get the update period of the controller.
  // \return Duration struct representing the update period in nanoseconds.
  std::chrono::nanoseconds get_publish_period() const
  {
    return publish_period_;
  }

  /// Set the properties of the PID controller (gains and desired output).
  // \param[in] properties Struct representing the desired properties.
  void set_pid_properties(const PIDProperties & properties)
  {
    pid_ = properties;
  }

  /// Get the properties of the controller.
  // \return Struct representing the properties of the controller.
  const PIDProperties & get_pid_properties() const
  {
    return pid_;
  }

  /// Set the commanded position of the controller.
  // \param[in] command The new commanded position (in radians).
  void set_command(double command)
  {
    pid_.command = command;
  }

  /// Get the commanded position of the controller.
  // \return The commanded position.
  double get_command() const
  {
    return pid_.command;
  }

  /// Count the number of messages received (number of times the callback fired).
  size_t messages_received = 0;

private:
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &) override
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &) override
  {
    command_pub_->on_activate();
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &) override
  {
    command_pub_->on_deactivate();
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &) override
  {
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &) override
  {
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  // controller should publish less frequently than the motor
  std::chrono::nanoseconds publish_period_;
  PIDProperties pid_;
  pendulum_msgs::msg::JointCommand command_message_;
  bool message_ready_;

  // state for PID controller
  double last_error_ = 0;
  double i_gain_ = 0;
  double dt_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<pendulum_msgs::msg::JointCommand>>
  command_pub_;
  std::shared_ptr<rclcpp::Subscription<pendulum_msgs::msg::JointState>> sensor_sub_;
  std::shared_ptr<rclcpp::Subscription<pendulum_msgs::msg::JointCommand>> setpoint_sub_;
  rclcpp::TimerBase::SharedPtr controller_publisher_timer_;
};

}  // namespace pendulum_control

#endif  // PENDULUM_CONTROL__PENDULUM_CONTROLLER_HPP_

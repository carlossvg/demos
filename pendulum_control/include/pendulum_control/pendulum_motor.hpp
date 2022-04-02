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

#ifndef PENDULUM_CONTROL__PENDULUM_MOTOR_HPP_
#define PENDULUM_CONTROL__PENDULUM_MOTOR_HPP_

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

#include "rttest/rttest.h"
#include "rttest/utils.hpp"

#include "pendulum_msgs/msg/joint_command.hpp"
#include "pendulum_msgs/msg/joint_state.hpp"

#ifndef GRAVITY
#define GRAVITY 9.80665
#endif

namespace pendulum_control
{

/// Struct representing the physical properties of the pendulum.
struct PendulumProperties
{
  /// Mass of the weight on the end of the pendulum in kilograms
  double mass = 0.01;
  /// Length of the pendulum in meters
  double length = 0.5;
};

/// Struct representing the dynamic/kinematic state of the pendulum.
struct PendulumState
{
  /// Angle from the ground in radians
  double position = 0;
  /// Angular velocity in radians/sec
  double velocity = 0;
  /// Angular acceleration in radians/sec^2
  double acceleration = 0;
  /// Torque on the joint (currently unused)
  double torque = 0;
};

/// Represents the physical state of the pendulum, the controlling motor, and the position sensor.
class PendulumMotor : public rclcpp_lifecycle::LifecycleNode
{
public:
  /// Default constructor.
  /**
   * \param[in] period Time between sending messages.
   * \param[in] properties Physical properties of the pendulum.
   */
  PendulumMotor(std::chrono::nanoseconds period, PendulumProperties properties)
  : LifecycleNode("pendulum_motor"),
    publish_period_(period), properties_(properties),
    physics_update_period_(std::chrono::nanoseconds(1000000)),
    message_ready_(false), done_(false)
  {
    // Create realtime callback group for critical excutables
    realtime_callback_group_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

    // The quality of service profile is tuned for real-time performance.
    // More QoS settings may be exposed by the rmw interface in the future to fulfill real-time
    // requirements.
    auto qos = rclcpp::QoS(
      // The "KEEP_LAST" history setting tells DDS to store a fixed-size buffer of values before they
      // are sent, to aid with recovery in the event of dropped messages.
      // "depth" specifies the size of this buffer.
      // In this example, we are optimizing for performance and limited resource usage (preventing
      // page faults), instead of reliability. Thus, we set the size of the history buffer to 1.
      rclcpp::KeepLast(1)
    );
    // From http://www.opendds.org/qosusages.html: "A RELIABLE setting can potentially block while
    // trying to send." Therefore set the policy to best effort to avoid blocking during execution.
    qos.best_effort();

    // Initialize the publisher for the sensor message (the current position of the pendulum).
    sensor_pub_ = create_publisher<pendulum_msgs::msg::JointState>(
      "pendulum_sensor", qos);

    // The MessagePoolMemoryStrategy preallocates a pool of messages to be used by the subscription.
    // Typically, one MessagePoolMemoryStrategy is used per subscription type, and the size of the
    // message pool is determined by the number of threads (the maximum number of concurrent accesses
    // to the subscription).
    // Since this example is single-threaded, we choose a message pool size of 1 for each strategy.
    using rclcpp::strategies::message_pool_memory_strategy::MessagePoolMemoryStrategy;
    auto state_msg_strategy =
      std::make_shared<MessagePoolMemoryStrategy<pendulum_msgs::msg::JointState, 1>>();
    auto command_msg_strategy =
      std::make_shared<MessagePoolMemoryStrategy<pendulum_msgs::msg::JointCommand, 1>>();

    // Create a lambda function to invoke the motor callback when a command is received.
    auto motor_subscribe_callback =
      [this](pendulum_msgs::msg::JointCommand::ConstSharedPtr msg) -> void
      {
        on_command_message(msg);
      };

    // Create a lambda function that will fire regularly to publish the next sensor message.
    auto motor_publish_callback = [this]() {
        if (next_message_ready()) {
          auto msg = get_next_sensor_message();
          sensor_pub_->publish(msg);
        }
      };

    // Add a timer to enable regular publication of sensor messages.
    motor_publisher_timer_ = create_wall_timer(
      period, motor_publish_callback,
      realtime_callback_group_);
    // cancel immediately to prevent it running the first time.
    motor_publisher_timer_->cancel();

    auto command_subscription_options = rclcpp::SubscriptionOptions();
    command_subscription_options.callback_group = realtime_callback_group_;

    // Initialize the subscription to the command message.
    // Notice that we pass the MessagePoolMemoryStrategy<JointCommand> initialized above.
    command_sub_ = create_subscription<pendulum_msgs::msg::JointCommand>(
      "pendulum_command", qos, motor_subscribe_callback,
      command_subscription_options, command_msg_strategy);

    // Calculate physics engine timestep.
    dt_ = physics_update_period_.count() / (1000.0 * 1000.0 * 1000.0);
    uint64_to_timespec(physics_update_period_.count(), &physics_update_timespec_);

    // Initialize a separate high-priority thread to run the physics update loop.
    pthread_attr_init(&thread_attr_);
    sched_param thread_param;
    thread_param.sched_priority = 90;
    pthread_attr_setschedparam(&thread_attr_, &thread_param);
    pthread_attr_setschedpolicy(&thread_attr_, SCHED_RR);
    pthread_create(
      &physics_update_thread_, &thread_attr_,
      &pendulum_control::PendulumMotor::physics_update_wrapper, this);
  }

  /// Update the position of motor based on the command.
  // \param[in] msg Received command.
  void on_command_message(pendulum_msgs::msg::JointCommand::ConstSharedPtr msg)
  {
    ++messages_received;
    // Assume direct, instantaneous position control
    // (It would be more realistic to simulate a motor model)
    state_.position = msg->position;

    // Enforce position limits
    if (state_.position > M_PI) {
      state_.position = M_PI;
    } else if (state_.position < 0) {
      state_.position = 0;
    }

    if (std::isnan(state_.position)) {
      throw std::runtime_error("Tried to set state to NaN in on_command_message callback");
    }
  }

  /// Return the next sensor message calculated by the physics engine.
  // \return The sensor message
  const pendulum_msgs::msg::JointState & get_next_sensor_message() const
  {
    return sensor_message_;
  }

  /// Get the status of the next message
  // \return True if the message is ready to be published.
  bool next_message_ready() const
  {
    return message_ready_;
  }

  /// Set the boolean to signal that the physics engine should finish.
  // \param[in] done True if the physics engine should stop.
  void set_done(bool done)
  {
    done_ = done;
  }

  /// Get the status of the physics engine.
  // \return True if the physics engine is running, false otherwise.
  bool done() const
  {
    return done_;
  }

  /// Get the update rate of the publisher.
  // \return The update rate as a std::chrono::duration.
  std::chrono::nanoseconds get_publish_period() const
  {
    return publish_period_;
  }

  /// Get the current position of the pendulum.
  // \return Position of the pendulum.
  double get_position() const
  {
    return state_.position;
  }

  /// Get the current state of the pendulum.
  // \return State of the pendulum.
  PendulumState get_state() const
  {
    return state_;
  }

  /// Set the state of the pendulum.
  // \param[in] state State to set.
  void set_state(const PendulumState & state)
  {
    state_ = state;
  }

  /// Get the physical properties of the pendulum.
  // \return Properties of the pendulum.
  const PendulumProperties & get_properties() const
  {
    return properties_;
  }

  /// Set the properties of the pendulum.
  // \param[in] properties Properties to set.
  void set_properties(const PendulumProperties & properties)
  {
    properties_ = properties;
  }

  /// Get the real-time callback group.
  // \return The callback group.
  rclcpp::CallbackGroup::SharedPtr get_realtime_callback_group()
  {
    return realtime_callback_group_;
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
    sensor_pub_->on_activate();
    motor_publisher_timer_->reset();
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &) override
  {
    motor_publisher_timer_->cancel();
    sensor_pub_->on_deactivate();
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

  static void * physics_update_wrapper(void * args)
  {
    PendulumMotor * motor = static_cast<PendulumMotor *>(args);
    if (!motor) {
      return NULL;
    }
    return motor->physics_update();
  }
  // Set kinematic and dynamic properties of the pendulum based on state inputs
  void * physics_update()
  {
    rttest_lock_and_prefault_dynamic();
    while (!done_) {
      state_.acceleration = GRAVITY * std::sin(state_.position - M_PI / 2.0) / properties_.length +
        state_.torque / (properties_.mass * properties_.length * properties_.length);
      state_.velocity += state_.acceleration * dt_;
      state_.position += state_.velocity * dt_;
      if (state_.position > M_PI) {
        state_.position = M_PI;
      } else if (state_.position < 0) {
        state_.position = 0;
      }

      if (std::isnan(state_.position)) {
        throw std::runtime_error("Tried to set state to NaN in on_command_message callback");
      }

      sensor_message_.velocity = state_.velocity;
      // Simulate a noisy sensor on position
      sensor_message_.position = state_.position;

      message_ready_ = true;
      // high resolution sleep
      clock_nanosleep(CLOCK_MONOTONIC, 0, &physics_update_timespec_, NULL);
    }
    return 0;
  }

  // motor should publish more frequently than the controller
  std::chrono::nanoseconds publish_period_;

  // Physics should update most frequently, in separate RT thread
  timespec physics_update_timespec_;
  double dt_;

  // Physical qualities of the pendulum
  /*
       M
        \
         \ length
       p  \
     0 ----------- pi
   */

  PendulumProperties properties_;
  PendulumState state_;

  std::chrono::nanoseconds physics_update_period_;
  pendulum_msgs::msg::JointState sensor_message_;
  bool message_ready_;
  bool done_;

  pthread_t physics_update_thread_;
  pthread_attr_t thread_attr_;

  rclcpp::CallbackGroup::SharedPtr realtime_callback_group_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<pendulum_msgs::msg::JointState>>
  sensor_pub_;
  std::shared_ptr<rclcpp::Subscription<pendulum_msgs::msg::JointCommand>> command_sub_;
  rclcpp::TimerBase::SharedPtr motor_publisher_timer_;
};

}  // namespace pendulum_control

#endif  // PENDULUM_CONTROL__PENDULUM_MOTOR_HPP_

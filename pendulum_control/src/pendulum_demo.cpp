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

#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <memory>

#include "rttest/rttest.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/strategies/message_pool_memory_strategy.hpp"
#include "rclcpp/strategies/allocator_memory_strategy.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"

#include "tlsf_cpp/tlsf.hpp"

#include "pendulum_msgs/msg/joint_command.hpp"
#include "pendulum_msgs/msg/joint_state.hpp"
#include "pendulum_msgs/msg/rttest_results.hpp"

#include "pendulum_control/pendulum_controller.hpp"
#include "pendulum_control/pendulum_motor.hpp"
#include "pendulum_control/rtt_executor.hpp"

using rclcpp::strategies::message_pool_memory_strategy::MessagePoolMemoryStrategy;
using rclcpp::memory_strategies::allocator_memory_strategy::AllocatorMemoryStrategy;

template<typename T = void>
using TLSFAllocator = tlsf_heap_allocator<T>;

int main(int argc, char * argv[])
{
  // Initialization phase.
  // In the initialization phase of a realtime program, non-realtime-safe operations such as
  // allocation memory are permitted.

  // Pass the input arguments to rttest.
  // rttest will store relevant parameters and allocate buffers for data collection
  rttest_read_args(argc, argv);

  // Pass the input arguments to rclcpp and initialize the signal handler.
  rclcpp::init(argc, argv);

  // Create a structure with the default physical properties of the pendulum (length and mass).
  pendulum_control::PendulumProperties properties;

  // Create the properties of the PID controller.
  pendulum_control::PIDProperties pid;

  // The controller node represents user code. This example implements a simple PID controller.
  auto controller_node = std::make_shared<pendulum_control::PendulumController>(
    std::chrono::nanoseconds(960000), pid);

  // The "motor" node simulates motors and sensors.
  // It provides sensor data and changes the physical model based on the command.
  auto motor_node = std::make_shared<pendulum_control::PendulumMotor>(
    std::chrono::nanoseconds(970000), properties);

  // Initialize the executor
  rclcpp::executors::StaticSingleThreadedExecutor non_realtime_executor;

  // Add the motor and controller nodes to the executor.
  non_realtime_executor.add_callback_group(
    motor_node->get_node_base_interface()->get_default_callback_group(),
    motor_node->get_node_base_interface());
  non_realtime_executor.add_callback_group(
    controller_node->get_node_base_interface()->get_default_callback_group(),
    controller_node->get_node_base_interface());

  rclcpp::ExecutorOptions options;
  // One of the arguments passed to the Executor is the memory strategy, which delegates the
  // runtime-execution allocations to the TLSF allocator.
  rclcpp::memory_strategy::MemoryStrategy::SharedPtr memory_strategy =
    std::make_shared<AllocatorMemoryStrategy<TLSFAllocator<void>>>();
  options.memory_strategy = memory_strategy;
  // RttExecutor is a special single-threaded executor instrumented to calculate and record
  // real-time performance statistics.
  // auto realtime_executor = std::make_shared<pendulum_control::RttExecutor>(options);
  auto realtime_executor = std::make_shared<rclcpp::executors::StaticSingleThreadedExecutor>(options);
  realtime_executor->add_callback_group(
    controller_node->get_realtime_callback_group(),
    controller_node->get_node_base_interface());
  realtime_executor->add_callback_group(
    motor_node->get_realtime_callback_group(),
    motor_node->get_node_base_interface());

  // Set node lifecycle states to configured
  if (lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE != motor_node->configure().id()) {
    throw std::runtime_error("Could not configure " + std::string(motor_node->get_name()));
  }
  if (lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE != controller_node->configure().id()) {
    throw std::runtime_error("Could not configure " + std::string(controller_node->get_name()));
  }

  // spin normal executor in a normal thread
  auto non_realtime_executor_thread = std::thread(
    [&]() {
      non_realtime_executor.spin();
    });

  // spin real-time executor in a thread with higher priority
  auto realtime_executor_thread = std::thread(
    [&]() {
      // Set the priority of this thread to the maximum safe value, and set its scheduling policy to a
      // deterministic (real-time safe) algorithm, round robin.
      if (rttest_set_sched_priority(98, SCHED_RR)) {
        perror("Couldn't set scheduling priority and policy");
      }
      // Unlike the default SingleThreadedExecutor::spin function, RttExecutor::spin runs in
      // bounded time (for as many iterations as specified in the rttest parameters).
      realtime_executor->spin();
    });

  // Lock the currently cached virtual memory into RAM, as well as any future memory allocations,
  // and do our best to prefault the locked memory to prevent future pagefaults.
  // Will return with a non-zero error code if something went wrong (insufficient resources or
  // permissions).
  // Always do this as the last step of the initialization phase.
  // See README.md for instructions on setting permissions.
  // See rttest/rttest.cpp for more details.
  if (rttest_lock_and_prefault_dynamic() != 0) {
    fprintf(stderr, "Couldn't lock all cached virtual memory.\n");
    fprintf(stderr, "Pagefaults from reading pages not yet mapped into RAM will be recorded.\n");
  }

  // End initialization phase

  // Activate nodes
  if (lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE != motor_node->activate().id()) {
    throw std::runtime_error("Could not activate " + std::string(motor_node->get_name()));
  }
  if (lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE != controller_node->activate().id()) {
    throw std::runtime_error("Could not activate " + std::string(controller_node->get_name()));
  }

  // TODO: add experiment stop conditions
  std::this_thread::sleep_for(std::chrono::seconds(10));
  realtime_executor->cancel();
  realtime_executor_thread.join();
  // Once the executor has exited, notify the physics simulation to stop running.
  motor_node->set_done(true);
  non_realtime_executor.cancel();
  non_realtime_executor_thread.join();

  // End execution phase

  printf("PendulumMotor received %zu messages\n", motor_node->messages_received);
  printf("PendulumController received %zu messages\n", controller_node->messages_received);

  rclcpp::shutdown();

  return 0;
}

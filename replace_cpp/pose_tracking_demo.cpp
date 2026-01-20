/*******************************************************************************
 *      Title     : pose_tracking_example.cpp
 *      Project   : moveit_servo
 *      Created   : 09/04/2020
 *      Author    : Adam Pettinger
 *      Modified  : For external target_pose control
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Los Alamos National Security, LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

#include <std_msgs/msg/int8.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <moveit_servo/servo.h>
#include <moveit_servo/pose_tracking.h>
#include <moveit_servo/status_codes.h>
#include <moveit_servo/servo_parameters.h>
#include <moveit_servo/make_shared_from_pool.h>
#include <thread>
#include <atomic>
#include <csignal>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_servo.pose_tracking_demo");

// Global flag for clean shutdown
std::atomic<bool> g_shutdown_requested(false);

void signalHandler(int signum)
{
  RCLCPP_INFO(LOGGER, "Interrupt signal (%d) received. Shutting down gracefully...", signum);
  g_shutdown_requested = true;
}

// Class for monitoring status of moveit_servo
class StatusMonitor
{
public:
  StatusMonitor(const rclcpp::Node::SharedPtr& node, const std::string& topic)
  {
    sub_ = node->create_subscription<std_msgs::msg::Int8>(topic, rclcpp::SystemDefaultsQoS(),
                                                          [this](const std_msgs::msg::Int8::ConstSharedPtr& msg) {
                                                            return statusCB(msg);
                                                          });
  }

private:
  void statusCB(const std_msgs::msg::Int8::ConstSharedPtr& msg)
  {
    moveit_servo::StatusCode latest_status = static_cast<moveit_servo::StatusCode>(msg->data);
    if (latest_status != status_)
    {
      status_ = latest_status;
      const auto& status_str = moveit_servo::SERVO_STATUS_CODE_MAP.at(status_);
      RCLCPP_INFO_STREAM(LOGGER, "Servo status: " << status_str);
    }
  }

  moveit_servo::StatusCode status_ = moveit_servo::StatusCode::INVALID;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_;
};

/**
 * Instantiate the pose tracking interface.
 * Wait for external /target_pose commands (e.g., from DualSense controller)
 * Exit cleanly on Ctrl+C
 */
int main(int argc, char** argv)
{
  // Register signal handler for Ctrl+C
  signal(SIGINT, signalHandler);

  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("pose_tracking_demo");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread executor_thread([&executor]() { executor.spin(); });

  auto servo_parameters = moveit_servo::ServoParameters::makeServoParameters(node);

  if (servo_parameters == nullptr)
  {
    RCLCPP_FATAL(LOGGER, "Could not get servo parameters!");
    executor.cancel();
    executor_thread.join();
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  // Load the planning scene monitor
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;
  planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node, "robot_description");
  if (!planning_scene_monitor->getPlanningScene())
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Error in setting up the PlanningSceneMonitor.");
    executor.cancel();
    executor_thread.join();
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  planning_scene_monitor->providePlanningSceneService();
  planning_scene_monitor->startSceneMonitor();
  planning_scene_monitor->startWorldGeometryMonitor(
      planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
      planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
      false /* skip octomap monitor */);
  planning_scene_monitor->startStateMonitor(servo_parameters->joint_topic);
  planning_scene_monitor->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);

  // Wait for Planning Scene Monitor to setup
  if (!planning_scene_monitor->waitForCurrentRobotState(node->now(), 5.0 /* seconds */))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Error waiting for current robot state in PlanningSceneMonitor.");
    executor.cancel();
    executor_thread.join();
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  RCLCPP_INFO(LOGGER, "Planning Scene Monitor initialized successfully");

  // Create the pose tracker
  moveit_servo::PoseTracking tracker(node, servo_parameters, planning_scene_monitor);

  // Subscribe to servo status (and log it when it changes)
  StatusMonitor status_monitor(node, servo_parameters->status_topic);

  // Tolerance for pose tracking
  Eigen::Vector3d lin_tol{ 0.001, 0.001, 0.001 };  // 1mm tolerance
  double rot_tol = 0.01;  // ~0.57 degrees tolerance

  // Reset target pose to wait for external commands
  tracker.resetTargetPose();

  RCLCPP_INFO(LOGGER, "Pose tracking ready!");
  RCLCPP_INFO(LOGGER, "Waiting for target poses on /target_pose topic...");
  RCLCPP_INFO(LOGGER, "Press Ctrl+C to exit");

  // Run the pose tracking in a new thread
  std::atomic<bool> tracking_running(true);
  std::thread move_to_pose_thread([&tracker, &lin_tol, &rot_tol, &tracking_running] {
    while (tracking_running && rclcpp::ok())
    {
      // This will continuously track incoming target poses
      moveit_servo::PoseTrackingStatusCode tracking_status =
          tracker.moveToPose(lin_tol, rot_tol, 1.0 /* target pose timeout - wait 1s for new pose */);

      // Only log if we got a real status (not timeout)
      if (tracking_status != moveit_servo::PoseTrackingStatusCode::NO_RECENT_TARGET_POSE)
      {
        RCLCPP_INFO_STREAM(LOGGER, "Pose tracking status: "
                                     << moveit_servo::POSE_TRACKING_STATUS_CODE_MAP.at(tracking_status));
      }

      // Small sleep to avoid busy waiting
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    RCLCPP_INFO(LOGGER, "Pose tracking thread exiting...");
  });

  // Main loop - just wait for shutdown signal
  rclcpp::WallRate loop_rate(10);  // 10 Hz check rate
  while (rclcpp::ok() && !g_shutdown_requested)
  {
    loop_rate.sleep();
  }

  RCLCPP_INFO(LOGGER, "Shutdown requested, cleaning up...");

  // Stop the tracking thread
  tracking_running = false;
  tracker.stopMotion();  // Signal the tracker to stop

  if (move_to_pose_thread.joinable())
  {
    move_to_pose_thread.join();
  }

  // Kill executor thread before shutdown
  executor.cancel();
  if (executor_thread.joinable())
  {
    executor_thread.join();
  }

  RCLCPP_INFO(LOGGER, "Shutdown complete");
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}

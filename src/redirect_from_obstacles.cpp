/************************************************************************************
 * Apache License 2.0
 * Copyright (c) 2021, Aniruddh Balram
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ************************************************************************************/
/**
 *  @file    redirect_from_obstacles.cpp
 *  @author  Aniruddh Balram
 *  @date    12/05/2022
 *  @version 1.0
 *
 *  @brief Implementation of a subscriber for obstacle avoidance and produce a feedback to turn the object
 *
 *
 *  
 *
 */

#include <algorithm>
#include <cmath>
#include <cstdio>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>


using std::placeholders::_1;

using SCAN = sensor_msgs::msg::LaserScan;
using TWIST = geometry_msgs::msg::Twist;

using namespace std::chrono_literals;

/**
 * @brief defining the StateType
*/
typedef enum {
  MOVE = 0,
  STOP,
  ROTATE,
} StateType;

/**
 * @brief ROS2Roomba Class
 *
 */
class ROS2Roomba : public rclcpp::Node {
 public:
    // Initial state value is STOP
    ROS2Roomba() : Node("walker"), state_value(STOP) {
    // Publisher topic name
    auto p_topic_name = "cmd_vel";
    // creates a publisher to publish TWIST type of message to the topic
    publisher_ = this->create_publisher<TWIST>(p_topic_name, 10);

    auto default_qos = rclcpp::QoS(rclcpp::SensorDataQoS());
    // creates subscriber to obtain /scan topic
    auto s_topic_name = "/scan";
    // initializing callback function for subscriber
    auto sub_callback = std::bind(&ROS2Roomba::subscribe_callback, this, _1);
    subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan> (
        s_topic_name, default_qos, sub_callback);

    // create a 15Hz timer for processing
    auto processCallback = std::bind(&ROS2Roomba::process_callback, this);
    timer_ =
    this-> create_wall_timer(150ms, processCallback);
  }

 private:
  void subscribe_callback(const SCAN& msg) { last_scan = msg; }

// Standard process_callback function when dealing with sensor data
  void process_callback() {
    if (last_scan.header.stamp.sec == 0) {
      return;
    }

// Using state machines, ROS2 recognizes the states
// as defined above and accordingly executes
// the switch statements
// This is a standard template for state machines

  // define a TWIST type of publish message
  auto pub_msg = TWIST();
    switch (state_value) {
      case MOVE:
        if (foundObstacle()) {  // check transition
          state_value = STOP;
          pub_msg.linear.x = 0;
          publisher_->publish(pub_msg);
          RCLCPP_INFO_STREAM(this->get_logger(),
          "Robot is currently in MOVE state");
        }
        break;
      case STOP:
      // If STOP due to obstacle, then turn else move straight
        if (foundObstacle()) {
          state_value = ROTATE;
          pub_msg.angular.z = 0.2;
          publisher_->publish(pub_msg);
          RCLCPP_INFO_STREAM(this->get_logger(),
          "The Robot is currently in STOP state");
        } else {
          state_value = MOVE;
          pub_msg.linear.x = 0.2;
          publisher_->publish(pub_msg);
          RCLCPP_INFO_STREAM(this->get_logger(),
          "The Robot is currently in STOP state");
        }
        break;
      case ROTATE:
      // If it is in ROTATE State rotate till no obstacle is found
        if (!foundObstacle()) {
          state_value = MOVE;
          pub_msg.linear.x = 0.1;
          publisher_->publish(pub_msg);
          RCLCPP_INFO_STREAM(this->get_logger(),
          "The Robot is currently in ROTATE state");
        }
        break;
    }
  }

  /**
   * @brief A function which tells whether an obstacle was found or not
   *
   *
   */

  bool foundObstacle() {
    // unsigned char* dataPtr = last_scan.data.data();
    // float* floatData = (float*)dataPtr;

    for (long unsigned int i = 0;
    i < sizeof(last_scan.ranges)/sizeof(last_scan.ranges[0]); i++) {
      if (last_scan.ranges[i] > last_scan.range_min
      && last_scan.ranges[i] < last_scan.range_max) {
                RCLCPP_INFO(this->get_logger(),
                "Current distance is: %f is valid", last_scan.ranges[i]);
                if (last_scan.ranges[i] < 1.0) {
                RCLCPP_INFO(this->get_logger(),
                "Found Obstacle, gonna turn");
                return true;
                }
      }
      return false;
    }
    return false;
  }
  // Initialization of publisher, subscriber and variables
  rclcpp::Subscription<SCAN>::SharedPtr subscriber_;
  rclcpp::Publisher<TWIST>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  SCAN last_scan;
  StateType state_value;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ROS2Roomba>());
  rclcpp::shutdown();
  return 0;
}

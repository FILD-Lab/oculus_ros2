/**
 * BSD 3-Clause License
 *
 * Copyright (c) 2022, ENSTA-Bretagne
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
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
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef OCULUS_ROS2__SONAR_VIEWER_HPP_
#define OCULUS_ROS2__SONAR_VIEWER_HPP_

#include <oculus_driver/AsyncService.h>
#include <oculus_driver/SonarDriver.h>

#include <algorithm>
#include <climits>
#include <iostream>
#include <limits>
#include <string>
#include <type_traits>
#include <vector>

#include <oculus_interfaces/msg/ping.hpp>
#include <oculus_ros2/conversions.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

class SonarViewer {
public:
  explicit SonarViewer(rclcpp::Node* node);
  ~SonarViewer();
  void publishFan(const oculus::PingMessage::ConstPtr& ping, const std::string& frame_id = "sonar") const;
  void publishFan(const oculus_interfaces::msg::Ping& ros_ping_msg) const;
  // `bearings` is the sonar's own per-beam bearing table (100ths of a degree, `width` entries,
  // ascending). It replaces the old hardcoded-aperture + linear-interpolation geometry.
  void publishFan(const int& width,
      const int& height,
      const int& offset,
      const std::vector<uint8_t>& ping_data,
      const int16_t* bearings,
      const std_msgs::msg::Header& header) const;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;

protected:
  // Display-only: divide the sonar's range gain (TVG) back out before rendering. Default false.
  static constexpr const char* GAIN_CORRECTION_PARAM_ = "gain_correction";

  // Display-only: percentile contrast stretch, so the fan uses the full 0..255 range.
  static constexpr const char* NORMALIZE_PARAM_ = "normalize";
  static constexpr const char* NORMALIZE_PERCENTILE_PARAM_ = "normalize_percentile";

  // NOTE: LOW_/HIGHT_FREQUENCY_BEARING_APERTURE_ (65 deg / 40 deg) used to live here and were
  // selected by master_mode. They were M1200d values and wrong for this M750d -- measured, the
  // sonar's real swath in master_mode 2 is +/-65 deg while the code drew +/-40 deg. The bearing
  // table on every Ping is authoritative for any model and any mode, so the constants are gone.
  const int SIZE_OF_GAIN_ = 4;

private:
  const rclcpp::Node* node_;
};

#endif  // OCULUS_ROS2__SONAR_VIEWER_HPP_

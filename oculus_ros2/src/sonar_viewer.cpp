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

#include <oculus_ros2/sonar_viewer.hpp>

#include <cstring>

SonarViewer::SonarViewer(rclcpp::Node* node) : node_(node) {
  image_publisher_ = node->create_publisher<sensor_msgs::msg::Image>("image", 10);
  // Display-only parameter, not sent to the sonar. See the note in publishFan() for why it
  // defaults to false. Guarded because publishFan() reads it on every ping.
  if (!node->has_parameter(GAIN_CORRECTION_PARAM_)) {
    node->declare_parameter<bool>(GAIN_CORRECTION_PARAM_, false);
  }
  if (!node->has_parameter(NORMALIZE_PARAM_)) {
    node->declare_parameter<bool>(NORMALIZE_PARAM_, true);
  }
  if (!node->has_parameter(NORMALIZE_PERCENTILE_PARAM_)) {
    node->declare_parameter<double>(NORMALIZE_PERCENTILE_PARAM_, 99.0);
  }
}

SonarViewer::~SonarViewer() {}

void SonarViewer::publishFan(const oculus_interfaces::msg::Ping& ros_ping_msg) const {
  // const int offset = ping->ping_data_offset(); // TODO(hugoyvrn)
  const int offset = -16;  // quick fix TODO(hugoyvrn, why 229?)

  if (ros_ping_msg.bearings.size() < ros_ping_msg.n_beams) {
    RCLCPP_WARN(node_->get_logger(), "Ping has no bearing table (%zu entries for %u beams); cannot render fan.",
        ros_ping_msg.bearings.size(), ros_ping_msg.n_beams);
    return;
  }
  publishFan(ros_ping_msg.n_beams, ros_ping_msg.n_ranges, offset, ros_ping_msg.ping_data, ros_ping_msg.bearings.data(),
      ros_ping_msg.header);
}

void SonarViewer::publishFan(const oculus::PingMessage::ConstPtr& ping, const std::string& frame_id) const {
  std_msgs::msg::Header header;
  header.stamp = oculus::toMsg(ping->timestamp());
  header.frame_id = frame_id;

  if (!ping->has_gains()) {
    RCLCPP_WARN(node_->get_logger(), "Gains are not send by the sonar. The conic image view is wrong.");
  }
  if (ping->bearing_data() == nullptr || ping->bearing_count() == 0) {
    RCLCPP_WARN(node_->get_logger(), "Ping has no bearing table; cannot render fan.");
    return;
  }
  publishFan(ping->bearing_count(), ping->range_count(), ping->ping_data_offset(), ping->data(), ping->bearing_data(), header);
}

void SonarViewer::publishFan(const int& width,
    const int& height,
    const int& offset,
    const std::vector<uint8_t>& ping_data,
    const int16_t* bearings,
    const std_msgs::msg::Header& header) const {
  const int step = width + SIZE_OF_GAIN_;
  const int mat_encoding = CV_8U;
  const char* ros_image_encoding = sensor_msgs::image_encodings::MONO8;

  // Fan geometry is taken from the sonar's own bearing table rather than a hardcoded aperture.
  // Measured on this M750d in master_mode 2: the real swath is +/-65 deg, but the old constant
  // assumed +/-40 deg, so the fan was drawn at 62% of its true angular width and every target
  // landed at the wrong bearing. Beam spacing is also non-uniform on this unit (0.200..0.480 deg
  // between adjacent beams), which Ping.msg warns about explicitly -- so a linear angle->column
  // mapping is wrong even with the correct aperture. Both errors go away by using bearings[].
  const float kDeg100ToRad = static_cast<float>(M_PI) / 180.0f / 100.0f;
  const float b_min = bearings[0] * kDeg100ToRad;
  const float b_max = bearings[width - 1] * kDeg100ToRad;

  // Horizontal extent of the wedge. Normally set by the endpoints, but if the swath ever reached
  // past +/-90 deg the widest point would be at 90 deg instead, so clamp on that.
  const float sin_neg = (b_min <= -static_cast<float>(M_PI_2)) ? -1.0f : std::sin(b_min);
  const float sin_pos = (b_max >= static_cast<float>(M_PI_2)) ? 1.0f : std::sin(b_max);
  const int negative_height = static_cast<int>(std::floor(height * std::min(0.0f, sin_neg)));
  const int positive_height = static_cast<int>(std::ceil(height * std::max(0.0f, sin_pos)));
  const int image_width = positive_height - negative_height;
  const int origin_width = abs(negative_height);  // x coordinate of the origin

  // Angle -> fractional beam index lookup, built once per ping from the non-uniform table. A
  // binary search per pixel would be ~9 compares on each of ~440k pixels; this keeps the inner
  // loop O(1). Resolution is ~0.03 deg per entry across a 130 deg swath, well under beam pitch.
  constexpr int kLutSize = 4096;
  std::vector<float> bearing_lut(kLutSize);
  {
    const float t0 = static_cast<float>(bearings[0]);
    const float t1 = static_cast<float>(bearings[width - 1]);
    int j = 0;
    for (int k = 0; k < kLutSize; k++) {
      const float t = t0 + (t1 - t0) * (static_cast<float>(k) / (kLutSize - 1));
      while (j + 2 < width && static_cast<float>(bearings[j + 1]) < t) j++;
      const float lo = static_cast<float>(bearings[j]);
      const float hi = static_cast<float>(bearings[j + 1]);
      const float f = (hi > lo) ? (t - lo) / (hi - lo) : 0.0f;
      bearing_lut[k] = static_cast<float>(j) + std::clamp(f, 0.0f, 1.0f);
    }
  }
  const float b_span = b_max - b_min;

  const cv::Size image_size(image_width, height);
  cv::Mat map(image_size, CV_32FC2);
  cv::parallel_for_(cv::Range(0, map.total()), [&](const cv::Range& range) {
    for (auto i = range.start; i < range.end; i++) {
      int y = i / map.cols;
      int x = i % map.cols;

      // Calculate range and bearing of this pixel from origin
      const float dx = x - origin_width;
      const float dy = map.rows - y;

      const float range = sqrt(dx * dx + dy * dy);
      const float bearing_x_y = atan2(dx, dy);

      const float xp = range;
      float yp;
      if (b_span <= 0.0f || bearing_x_y < b_min || bearing_x_y > b_max) {
        // Outside the sonar's swath. Push the sample out of source bounds so cv::remap's
        // BORDER_CONSTANT paints the background and the wedge keeps its shape.
        yp = -1.0f;
      } else {
        const float u = (bearing_x_y - b_min) / b_span * (kLutSize - 1);
        const int ui = std::min(static_cast<int>(u), kLutSize - 2);
        const float uf = u - static_cast<float>(ui);
        yp = bearing_lut[ui] * (1.0f - uf) + bearing_lut[ui + 1] * uf;
      }

      map.at<cv::Vec2f>(cv::Point(x, y)) = cv::Vec2f(xp, yp);
    }
  });

  cv::Mat source_map_1, source_map_2;
  cv::convertMaps(map, cv::Mat(), source_map_1, source_map_2, CV_16SC2);

  cv::Mat sonar_mat_data(height, step, mat_encoding);  // Note that the width is 'step' to include gain data
  // Copy the data including gain data
  for (int i = 0; i < height; ++i)
    std::copy(ping_data.begin() + offset + i * step, ping_data.begin() + offset + (i + 1) * step, sonar_mat_data.ptr<uint8_t>(i));

  // Range-gain (TVG) removal. OFF by default, and deliberately so.
  //
  // The sonar applies TVG, ramping its gain with range so distant returns come back usable
  // (measured on this M750d at 6 m: gain 21,260 near -> 1,922,146 far). Ping.msg documents
  // dividing each row by sqrt(gain) to recover physically consistent backscatter -- correct for
  // quantitative work, wrong for a picture.
  //
  // Because the normalisation anchors on raw_gain_max, gain_i = sqrt(gmax)/sqrt(g_i) is >= 1
  // everywhere: ~9.5x at the apex, decaying to 1.0 at max range. Measured row means went
  // near-quarter 35.6 -> 188.9 (saturating to 255) while the far quarter stayed 13.0 -> 15.1.
  // That renders as a blown-out white near field and a near-black far field -- the sonar's own
  // TVG undone. ViewPoint displays the TVG'd data and looks evenly lit across the fan.
  //
  // So: leave this off to match ViewPoint, turn it on if you want TVG removed for analysis.
  const bool gain_correction = node_->get_parameter(GAIN_CORRECTION_PARAM_).as_bool();
  if (gain_correction) {  // Correct range gains
    float raw_gain_min = std::numeric_limits<float>::max();
    float raw_gain_max = std::numeric_limits<float>::min();
    for (int i = 0; i < height; i++) {
      raw_gain_min = std::min(raw_gain_min, static_cast<float>(sonar_mat_data.at<uint32_t>(i, 0)));
      raw_gain_max = std::max(raw_gain_max, static_cast<float>(sonar_mat_data.at<uint32_t>(i, 0)));
    }
    // NOTE: there used to be a `/ std::numeric_limits<uint8_t>::max()` here. It made gain_i
    // collapse to exactly 1/255 for every row already at max gain (which is most of them), so
    // all their pixels fell below 1.0 and truncated to zero -> an all-black fan. Ping.msg
    // documents the intended operation as "divide the whole row by the square root of this
    // gain"; normalizing by sqrt(raw_gain_max) makes gain_i == 1.0 for max-gain rows (raw
    // passthrough, keeping the sonar's own TVG) and >1 for the low-gain near-range rows.
    const float gain_nomalization = std::sqrt(raw_gain_max);
    for (int i = 0; i < height; i++) {
      const float gain_i = gain_nomalization / std::sqrt(sonar_mat_data.at<uint32_t>(i, 0));
      for (int j = SIZE_OF_GAIN_; j < step; j++) {
        const float new_pixel_val = sonar_mat_data.at<uint8_t>(i, j) * gain_i;
        sonar_mat_data.at<uint8_t>(i, j) = std::clamp(new_pixel_val, 0.0f, 255.0f);
      }
    }
  }

  // Now remove the gain data from sonar_mat_data
  cv::Mat sonar_mat_data_without_gain(height, width, mat_encoding);
  for (int i = 0; i < height; ++i)
    std::copy(sonar_mat_data.ptr<uint8_t>(i) + SIZE_OF_GAIN_, sonar_mat_data.ptr<uint8_t>(i) + step,
        sonar_mat_data_without_gain.ptr<uint8_t>(i));

  // Percentile contrast stretch. Raw sonar returns occupy only the bottom of the 0..255 range
  // -- measured on this M750d at 6 m: p50=11, p90=19, p99=40, while max=249. So the fan renders
  // almost black even though the data is fine. ViewPoint clearly applies a similar stretch.
  //
  // Anchored on a percentile, NOT on max: a few specular near-field returns push max to ~249,
  // so a max-based stretch would be ~1.03x and do nothing. p99 gives ~6.4x here.
  //
  // Applied to the source samples before cv::remap so the histogram sees only real sonar data
  // (the 255 background outside the wedge is painted later by BORDER_CONSTANT and would
  // otherwise skew the percentile).
  if (node_->get_parameter(NORMALIZE_PARAM_).as_bool()) {
    const double percentile = std::clamp(node_->get_parameter(NORMALIZE_PERCENTILE_PARAM_).as_double(), 1.0, 100.0);
    // Rows are weighted by the cartesian area they will occupy after the remap, NOT counted
    // equally. Range bin i maps to an arc of length proportional to i, so a far row covers far
    // more of the displayed fan than a near one. Counting rows equally normalises a distribution
    // nobody looks at: measured that way, asking for p99 produced an output whose actual p99 was
    // 113/255 -- still dark -- because the dim far field dominates the picture by area while
    // contributing the same weight as the bright apex. Weighting by (i + 1) makes the percentile
    // mean "this fraction of the displayed image", which is what the parameter claims.
    long histogram[256] = {0};
    for (int i = 0; i < height; i++) {
      const uint8_t* row = sonar_mat_data_without_gain.ptr<uint8_t>(i);
      const long area_weight = static_cast<long>(i) + 1;
      for (int j = 0; j < width; j++) histogram[row[j]] += area_weight;
    }
    const long total = static_cast<long>(width) * (static_cast<long>(height) * (static_cast<long>(height) + 1) / 2);
    const long target = static_cast<long>(static_cast<double>(total) * percentile / 100.0);
    long cumulative = 0;
    int cut = 255;
    for (int v = 0; v < 256; v++) {
      cumulative += histogram[v];
      if (cumulative >= target) {
        cut = v;
        break;
      }
    }
    const float scale = 255.0f / static_cast<float>(std::max(cut, 1));
    if (scale > 1.0f) {
      for (int i = 0; i < height; i++) {
        uint8_t* row = sonar_mat_data_without_gain.ptr<uint8_t>(i);
        for (int j = 0; j < width; j++)
          row[j] = static_cast<uint8_t>(std::clamp(static_cast<float>(row[j]) * scale, 0.0f, 255.0f));
      }
    }
  }

  cv::Mat out = cv::Mat::ones(cv::Size(image_width, height), CV_MAKETYPE(mat_encoding, 1)) * std::numeric_limits<uint8_t>::max();
  cv::remap(sonar_mat_data_without_gain.t(), out, source_map_1, source_map_2, cv::INTER_CUBIC, cv::BORDER_CONSTANT,
      cv::Scalar(std::numeric_limits<uint8_t>::max(), std::numeric_limits<uint8_t>::max(), std::numeric_limits<uint8_t>::max()));

  // Publish sonar conic image
  // NOTE: this is the hand-rolled equivalent of cv_bridge::CvImage::toImageMsg(). cv_bridge is
  // deliberately not used: ros-humble-cv-bridge is built against OpenCV 4.5.4, while the only
  // OpenCV with dev files on the Jetson is NVIDIA's 4.8.0, and linking both ABIs into one
  // process is what broke this node before. Everything cv_bridge did for a continuous CV_8UC1
  // Mat is a few field assignments plus a memcpy, so we do it inline and keep plain OpenCV.
  sensor_msgs::msg::Image msg;
  msg.header = header;
  msg.height = out.rows;
  msg.width = out.cols;
  msg.encoding = ros_image_encoding;
  msg.is_bigendian = 0;
  msg.step = static_cast<uint32_t>(out.cols * out.elemSize());
  msg.data.resize(static_cast<size_t>(msg.step) * msg.height);
  if (out.isContinuous()) {
    std::memcpy(msg.data.data(), out.ptr(), msg.data.size());
  } else {  // cv::remap always returns a continuous Mat, but stay correct if that ever changes
    for (int i = 0; i < out.rows; ++i)
      std::memcpy(msg.data.data() + static_cast<size_t>(i) * msg.step, out.ptr(i), msg.step);
  }
  image_publisher_->publish(msg);
}

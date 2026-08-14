// Copyright 2020 Tier IV, Inc.
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

/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "image_transport_decompressor.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <sensor_msgs/image_encodings.hpp>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include <stdexcept>
#include <string>
#include <utility>

namespace autoware::image_preprocessor::image_transport_decompressor
{

namespace
{
// Whether @p encoding describes the channel count and the bit depth of @p image.
bool describes(const std::string & encoding, const cv::Mat & image)
{
  try {
    return sensor_msgs::image_encodings::numChannels(encoding) == image.channels() &&
           sensor_msgs::image_encodings::bitDepth(encoding) ==
             static_cast<int>(image.elemSize1()) * 8;
  } catch (const std::runtime_error &) {
    return false;
  }
}

// Encoding that describes @p image in the BGR channel order the image codecs decode into, or ""
// when none of mono8/16, bgr8/16 or bgra8/16 does.
std::string payload_encoding(const cv::Mat & image)
{
  if (image.depth() != CV_8U && image.depth() != CV_16U) {
    return "";
  }
  const bool is_16bit = image.depth() == CV_16U;
  switch (image.channels()) {
    case 1:
      return is_16bit ? sensor_msgs::image_encodings::MONO16 : sensor_msgs::image_encodings::MONO8;
    case 3:
      return is_16bit ? sensor_msgs::image_encodings::BGR16 : sensor_msgs::image_encodings::BGR8;
    case 4:
      return is_16bit ? sensor_msgs::image_encodings::BGRA16 : sensor_msgs::image_encodings::BGRA8;
    default:
      return "";
  }
}

bool has_bgr_order(const std::string & encoding)
{
  return encoding.rfind("bgr", 0) == 0;
}
}  // namespace

DecompressResult decompress(
  const sensor_msgs::msg::CompressedImage & compressed_image,
  const std::string & requested_encoding)
{
  DecompressResult result;

  cv_bridge::CvImage cv_image;
  cv_image.header = compressed_image.header;

  const bool force_color = requested_encoding == "rgb8" || requested_encoding == "bgr8";

  try {
    // A color read reduces every stream to an 8-bit 3-channel BGR image; otherwise the depth, the
    // channels and the alpha are kept so the encoding can describe the payload.
    cv_image.image = cv::imdecode(
      cv::Mat(compressed_image.data), force_color ? cv::IMREAD_COLOR : cv::IMREAD_UNCHANGED);
    if (cv_image.image.empty()) {
      result.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      result.message =
        "failed to decode a compressed image of format \"" + compressed_image.format + "\"";
      return result;
    }

    const size_t split_pos = compressed_image.format.find(';');
    const std::string source_encoding =
      split_pos == std::string::npos ? std::string() : compressed_image.format.substr(0, split_pos);

    if (force_color) {
      cv_image.encoding = requested_encoding;
    } else if (describes(source_encoding, cv_image.image)) {
      cv_image.encoding = source_encoding;
    } else {
      // The sender converted the image before compressing it, so its own encoding no longer
      // describes the payload.
      cv_image.encoding = payload_encoding(cv_image.image);
      if (cv_image.encoding.empty()) {
        result.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        result.message = "no image encoding describes a compressed image of format \"" +
                         compressed_image.format + "\"";
        return result;
      }
      result.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      result.message = (source_encoding.empty() ? "no encoding was given for the compressed "
                                                  "image, publishing \""
                                                : "\"" + source_encoding +
                                                    "\" does not describe the compressed "
                                                    "image, publishing \"") +
                       cv_image.encoding + "\" instead";
    }

    // Channel order of the stream. An order the sender did not name is the BGR order the codecs
    // decode into.
    const bool compressed_bgr_order =
      split_pos == std::string::npos ||
      compressed_image.format.find("compressed bgr", split_pos) != std::string::npos;

    const int channels = cv_image.image.channels();
    if (
      (channels == 3 || channels == 4) &&
      compressed_bgr_order != has_bgr_order(cv_image.encoding)) {
      cv::cvtColor(
        cv_image.image, cv_image.image, channels == 3 ? cv::COLOR_BGR2RGB : cv::COLOR_BGRA2RGBA);
    }
  } catch (const cv::Exception & e) {
    result.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    result.message = e.what();
    return result;
  }

  sensor_msgs::msg::Image image_msg;
  cv_image.toImageMsg(image_msg);
  result.image = std::move(image_msg);
  return result;
}

}  // namespace autoware::image_preprocessor::image_transport_decompressor

// Copyright 2026 TIER IV, Inc.
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

// Pins the message decompress() returns for every combination of source image encoding and
// requested encoding. Every combination now returns an encoding that describes its payload; rows
// marked "KNOWN DEFECT" still hand the consumer something other than what the camera produced.

#include "../src/image_transport_decompressor.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include <gtest/gtest.h>

#include <cstdint>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
using autoware::image_preprocessor::image_transport_decompressor::decompress;

constexpr int image_width = 4;
constexpr int image_height = 2;

// Every camera below sends the same picture: 4x2 pixels, all of them the same colour. A color
// camera sends red 30, green 20, blue 10 and, when it has one, alpha 50. A grayscale or Bayer
// camera sends 40.
constexpr int blue = 10;
constexpr int green = 20;
constexpr int red = 30;
constexpr int alpha = 50;
constexpr int gray = 40;
// A 16-bit camera sends each of the values above multiplied by 257. This is worked around OpenCV
// behavior, not anything about real cameras: OpenCV's 16-to-8-bit decode keeps only the high byte
// of each 16-bit sample, so a plain value like 10 (0x000A) would decode to 0. Multiplying by 257
// duplicates the value into both bytes (10 * 257 = 0x0A0A), so OpenCV's high-byte-only decode still
// yields 10, and the 8-bit constants above double as the 16-bit expected values.
constexpr int scale = 257;

// Channel count and bit depth of every image encoding this test mentions.
struct EncodingSpec
{
  int channels;
  int bits;
};

const std::map<std::string, EncodingSpec> encoding_specs = {
  {"mono8", {1, 8}}, {"mono16", {1, 16}}, {"bayer_rggb8", {1, 8}}, {"yuv422", {2, 8}},
  {"rgb8", {3, 8}},  {"bgr8", {3, 8}},    {"rgb16", {3, 16}},      {"bgr16", {3, 16}},
  {"rgba8", {4, 8}}, {"bgra8", {4, 8}},   {"rgba16", {4, 16}},     {"bgra16", {4, 16}}};

// Possible encoding combination of "<camera encoding>; <codec> compressed <compressed encoding>"
// <compressed encoding> is basically "bgr*" because a color picture is converted to BGR order first
// A grayscale or Bayer picture is compressed as is, and then the sender writes nothing after
// "compressed".
const std::map<std::string, std::string> compressed_encodings = {
  {"mono8", ""},        {"mono16", ""},       {"rgb8", "bgr8"},   {"bgr8", "bgr8"},
  {"rgba8", "bgra8"},   {"bgra8", "bgra8"},   {"rgb16", "bgr16"}, {"bgr16", "bgr16"},
  {"rgba16", "bgra16"}, {"bgra16", "bgra16"}, {"yuv422", "bgr8"}, {"bayer_rggb8", ""}};

cv::Mat make_source_image(const EncodingSpec & spec)
{
  const int sample = spec.bits == 16 ? scale : 1;
  const cv::Scalar color =
    spec.channels == 1 ? cv::Scalar(gray * sample)
                       : cv::Scalar(blue * sample, green * sample, red * sample, alpha * sample);
  const int depth = spec.bits == 16 ? CV_16U : CV_8U;
  return cv::Mat(image_height, image_width, CV_MAKETYPE(depth, spec.channels), color);
}

// One row of the matrix: which camera published, under which requested encoding, and what
// decompress() returns today.
struct DecompressorCase
{
  std::string source_encoding;
  std::string requested_encoding;

  std::string expected_encoding;
  uint32_t expected_step;
  size_t expected_data_size;
  std::vector<uint8_t> expected_first_pixel;
};

// Bytes a pixel of the given encoding occupies, or 0 when the encoding is not one of the above.
size_t bytes_per_pixel(const std::string & encoding)
{
  const auto spec = encoding_specs.find(encoding);
  if (spec == encoding_specs.end()) {
    return 0;
  }
  return static_cast<size_t>(spec->second.channels) * static_cast<size_t>(spec->second.bits / 8);
}

// A message is only interpretable if its step and data size match its encoding.
bool is_consistent_with_encoding(const sensor_msgs::msg::Image & image)
{
  const size_t pixel_size = bytes_per_pixel(image.encoding);
  return pixel_size != 0 && image.step == image.width * pixel_size &&
         image.data.size() == static_cast<size_t>(image.height) * image.step;
}

// PNG, not JPEG: JPEG is lossy, so it would change the pixel values below and break the expected
// bytes in the test cases. decompress() never looks at which codec produced the bytes — it decodes
// through cv::imdecode(data, IMREAD_COLOR), which detects the codec from the bytes themselves —
// and JPEG-compressed input has been separately confirmed to decode the same way, so testing with
// PNG alone is enough.
std::vector<uint8_t> encode_png(const cv::Mat & image)
{
  std::vector<uint8_t> data;
  if (!cv::imencode(".png", image, data)) {
    throw std::runtime_error("failed to prepare the compressed test input");
  }
  return data;
}

// The compressed image a camera of the given encoding reaches decompress() as. The sender writes
// "<camera encoding>; <codec> compressed <compressed encoding>" into the format field.
sensor_msgs::msg::CompressedImage compressed_image_from_camera(const std::string & camera_encoding)
{
  const std::string & compressed_encoding = compressed_encodings.at(camera_encoding);
  const std::string & payload_encoding =
    compressed_encoding.empty() ? camera_encoding : compressed_encoding;

  sensor_msgs::msg::CompressedImage message;
  message.header.frame_id = "camera";
  message.format = camera_encoding + "; png compressed " + compressed_encoding;
  message.data = encode_png(make_source_image(encoding_specs.at(payload_encoding)));
  return message;
}

std::vector<uint8_t> leading_bytes(const sensor_msgs::msg::Image & image, const size_t count)
{
  if (image.data.size() < count) {
    return {};
  }
  return std::vector<uint8_t>(
    image.data.begin(), image.data.begin() + static_cast<std::ptrdiff_t>(count));
}

// Keeping the source encoding keeps the bit depth, the channel count and the alpha channel of the
// compressed image. A 16-bit sample is scaled by 257, so each of its two bytes holds the 8-bit
// value.
const std::vector<DecompressorCase> default_cases = {
  {"rgb8", "default", "rgb8", 12, 24, {red, green, blue}},
  {"bgr8", "default", "bgr8", 12, 24, {blue, green, red}},
  {"rgba8", "default", "rgba8", 16, 32, {red, green, blue, alpha}},
  {"bgra8", "default", "bgra8", 16, 32, {blue, green, red, alpha}},
  {"mono8", "default", "mono8", 4, 8, {gray}},
  {"mono16", "default", "mono16", 8, 16, {gray, gray}},
  {"rgb16", "default", "rgb16", 24, 48, {red, red, green, green, blue, blue}},
  {"bgr16", "default", "bgr16", 24, 48, {blue, blue, green, green, red, red}},
  {"rgba16", "default", "rgba16", 32, 64, {red, red, green, green, blue, blue, alpha, alpha}},
  {"bgra16", "default", "bgra16", 32, 64, {blue, blue, green, green, red, red, alpha, alpha}},
  // The sender converted the image to BGR before compressing it, so "yuv422" no longer describes
  // the payload and the BGR-ordered encoding is returned instead.
  {"yuv422", "default", "bgr8", 12, 24, {blue, green, red}},
  // The Bayer pattern is one channel of 8 bits, exactly what "bayer_rggb8" describes, so it is
  // returned unchanged.
  {"bayer_rggb8", "default", "bayer_rggb8", 4, 8, {gray}},
};

// Requesting rgb8 makes every message consistent, but only a color camera comes through as it was
// sent.
const std::vector<DecompressorCase> rgb8_cases = {
  {"rgb8", "rgb8", "rgb8", 12, 24, {red, green, blue}},
  {"bgr8", "rgb8", "rgb8", 12, 24, {red, green, blue}},
  // The sender already converted this camera to BGR, so only the channel order is applied.
  {"yuv422", "rgb8", "rgb8", 12, 24, {red, green, blue}},
  // inflated to three identical channels, a Bayer image likewise instead of being converted to
  // color, a 16-bit image keeps only its upper 8 bits, and an alpha channel is dropped.
  {"mono8", "rgb8", "rgb8", 12, 24, {gray, gray, gray}},
  {"mono16", "rgb8", "rgb8", 12, 24, {gray, gray, gray}},
  {"bayer_rggb8", "rgb8", "rgb8", 12, 24, {gray, gray, gray}},
  {"rgba8", "rgb8", "rgb8", 12, 24, {red, green, blue}},
  {"bgra8", "rgb8", "rgb8", 12, 24, {red, green, blue}},
  {"rgb16", "rgb8", "rgb8", 12, 24, {red, green, blue}},
  {"bgr16", "rgb8", "rgb8", 12, 24, {red, green, blue}},
  {"rgba16", "rgb8", "rgb8", 12, 24, {red, green, blue}},
  {"bgra16", "rgb8", "rgb8", 12, 24, {red, green, blue}},
};

// Requesting bgr8 behaves like requesting rgb8, with the channels in the opposite order.
const std::vector<DecompressorCase> bgr8_cases = {
  {"rgb8", "bgr8", "bgr8", 12, 24, {blue, green, red}},
  {"bgr8", "bgr8", "bgr8", 12, 24, {blue, green, red}},
  {"yuv422", "bgr8", "bgr8", 12, 24, {blue, green, red}},
  // KNOWN DEFECT: as above, forcing a color encoding discards what the camera sent.
  {"mono8", "bgr8", "bgr8", 12, 24, {gray, gray, gray}},
  {"mono16", "bgr8", "bgr8", 12, 24, {gray, gray, gray}},
  {"bayer_rggb8", "bgr8", "bgr8", 12, 24, {gray, gray, gray}},
  {"rgba8", "bgr8", "bgr8", 12, 24, {blue, green, red}},
  {"bgra8", "bgr8", "bgr8", 12, 24, {blue, green, red}},
  {"rgb16", "bgr8", "bgr8", 12, 24, {blue, green, red}},
  {"bgr16", "bgr8", "bgr8", 12, 24, {blue, green, red}},
  {"rgba16", "bgr8", "bgr8", 12, 24, {blue, green, red}},
  {"bgra16", "bgr8", "bgr8", 12, 24, {blue, green, red}},
};

std::string case_name(const ::testing::TestParamInfo<DecompressorCase> & info)
{
  return info.param.source_encoding;
}
}  // namespace

class ImageTransportDecompressorTest : public ::testing::TestWithParam<DecompressorCase>
{
};

TEST_P(ImageTransportDecompressorTest, ProducesExpectedImage)
{
  // Arrange
  const auto & test_case = GetParam();

  // Act
  const auto result = decompress(
    compressed_image_from_camera(test_case.source_encoding), test_case.requested_encoding);

  // Assert
  ASSERT_TRUE(result.image.has_value());
  const auto & image = *result.image;

  // The header and the geometry of the camera image are preserved.
  EXPECT_EQ(image.header.frame_id, "camera");
  EXPECT_EQ(image.width, static_cast<uint32_t>(image_width));
  EXPECT_EQ(image.height, static_cast<uint32_t>(image_height));

  EXPECT_EQ(image.encoding, test_case.expected_encoding);
  EXPECT_EQ(image.step, test_case.expected_step);
  EXPECT_EQ(image.data.size(), test_case.expected_data_size);
  EXPECT_EQ(
    leading_bytes(image, test_case.expected_first_pixel.size()), test_case.expected_first_pixel);
  EXPECT_TRUE(is_consistent_with_encoding(image));
}

INSTANTIATE_TEST_SUITE_P(
  RequestedEncodingDefault, ImageTransportDecompressorTest, ::testing::ValuesIn(default_cases),
  case_name);
INSTANTIATE_TEST_SUITE_P(
  RequestedEncodingRgb8, ImageTransportDecompressorTest, ::testing::ValuesIn(rgb8_cases),
  case_name);
INSTANTIATE_TEST_SUITE_P(
  RequestedEncodingBgr8, ImageTransportDecompressorTest, ::testing::ValuesIn(bgr8_cases),
  case_name);

class ImageTransportDecompressorEdgeCaseTest : public ::testing::Test
{
};

// A sender that does not name the encoding of the camera image still gets the requested encoding,
// its payload being read in the BGR order the image codecs decode into.
TEST_F(ImageTransportDecompressorEdgeCaseTest, AppliesRequestedEncodingWithoutFormatSeparator)
{
  // Arrange
  const cv::Mat compressed(image_height, image_width, CV_8UC3, cv::Scalar(blue, green, red));
  sensor_msgs::msg::CompressedImage message;
  message.header.frame_id = "camera";
  message.format = "png";  // legacy senders write only the codec name, with no ';' separator
  message.data = encode_png(compressed);

  // Act
  const auto result = decompress(message, "rgb8");

  // Assert
  ASSERT_TRUE(result.image.has_value());
  EXPECT_EQ(result.image->encoding, "rgb8");
  EXPECT_EQ(leading_bytes(*result.image, 3), std::vector<uint8_t>({red, green, blue}));
  EXPECT_TRUE(is_consistent_with_encoding(*result.image));
}

// The sender converted the image before compressing it, so the encoding of the camera image no
// longer describes the payload, and the substitution is reported.
TEST_F(ImageTransportDecompressorEdgeCaseTest, ReportsAReplacedEncoding)
{
  // Act
  const auto result = decompress(compressed_image_from_camera("yuv422"), "default");

  // Assert
  ASSERT_TRUE(result.image.has_value());
  EXPECT_EQ(result.image->encoding, "bgr8");
  EXPECT_TRUE(is_consistent_with_encoding(*result.image));
  EXPECT_EQ(result.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
  EXPECT_FALSE(result.message.empty());
}

// Undecodable payloads are dropped and reported as a warning.
TEST_F(ImageTransportDecompressorEdgeCaseTest, ReportsAFailureForUndecodableData)
{
  // Arrange
  sensor_msgs::msg::CompressedImage message;
  message.header.frame_id = "camera";
  message.format = "bgr8; png compressed bgr8";
  message.data = {0x01, 0x02, 0x03, 0x04, 0x05};

  // Act
  const auto result = decompress(message, "bgr8");

  // Assert
  EXPECT_FALSE(result.image.has_value());
  EXPECT_EQ(result.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
  EXPECT_FALSE(result.message.empty());
}

// A payload that the format field misnames, and that no BGR-ordered encoding can describe either,
// is dropped and reported rather than returned under an encoding that does not fit it.
TEST_F(ImageTransportDecompressorEdgeCaseTest, ReportsAnUnsupportedPayload)
{
  // Arrange
  sensor_msgs::msg::CompressedImage message;
  message.header.frame_id = "camera";
  message.format = "mono8; tiff compressed ";
  ASSERT_TRUE(
    cv::imencode(
      ".tiff", cv::Mat(image_height, image_width, CV_32FC1, cv::Scalar(1.5)), message.data));

  // Act
  const auto result = decompress(message, "default");

  // Assert
  EXPECT_FALSE(result.image.has_value());
  EXPECT_EQ(result.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
  EXPECT_FALSE(result.message.empty());
}

// A 16-bit image now describes its actual payload, so cv_bridge can read it back without throwing.
TEST_F(ImageTransportDecompressorEdgeCaseTest, RoundTripsSixteenBitOutputThroughCvBridge)
{
  // Act
  const auto result = decompress(compressed_image_from_camera("rgb16"), "default");

  // Assert
  ASSERT_TRUE(result.image.has_value());
  EXPECT_TRUE(is_consistent_with_encoding(*result.image));
  EXPECT_NO_THROW(cv_bridge::toCvCopy(*result.image, "bgr16"));
}

// A grayscale image is no longer padded with pixels from the color codec's decode, so cv_bridge
// reads back exactly the source gradient.
TEST_F(ImageTransportDecompressorEdgeCaseTest, RoundTripsGrayscaleOutputThroughCvBridge)
{
  // Arrange
  constexpr int gradient_width = 12;
  cv::Mat source(image_height, gradient_width, CV_8UC1);
  for (int row = 0; row < source.rows; ++row) {
    for (int column = 0; column < source.cols; ++column) {
      source.at<uint8_t>(row, column) = static_cast<uint8_t>(column * 20);
    }
  }
  sensor_msgs::msg::CompressedImage message;
  message.header.frame_id = "camera";
  message.format = "mono8; png compressed ";
  message.data = encode_png(source);

  // Act
  const auto result = decompress(message, "default");

  // Assert
  ASSERT_TRUE(result.image.has_value());
  EXPECT_TRUE(is_consistent_with_encoding(*result.image));

  cv_bridge::CvImagePtr received;
  ASSERT_NO_THROW(received = cv_bridge::toCvCopy(*result.image, "mono8"));
  ASSERT_EQ(received->image.cols, gradient_width);
  const std::vector<uint8_t> received_row(
    received->image.ptr<uint8_t>(0), received->image.ptr<uint8_t>(0) + gradient_width);
  const std::vector<uint8_t> source_row(
    source.ptr<uint8_t>(0), source.ptr<uint8_t>(0) + gradient_width);
  EXPECT_EQ(received_row, source_row);
}

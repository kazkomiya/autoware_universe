# image_transport_decompressor

## Purpose

The `image_transport_decompressor` is a node that decompresses images.

## Inner-workings / Algorithms

## Inputs / Outputs

### Input

| Name                       | Type                                | Description      |
| -------------------------- | ----------------------------------- | ---------------- |
| `~/input/compressed_image` | `sensor_msgs::msg::CompressedImage` | compressed image |

### Output

| Name                 | Type                      | Description        |
| -------------------- | ------------------------- | ------------------ |
| `~/output/raw_image` | `sensor_msgs::msg::Image` | decompressed image |

## Parameters

{{ json_to_markdown("sensing/autoware_image_transport_decompressor/schema/image_transport_decompressor.schema.json") }}

## Assumptions / Known limits

- The `encoding` of the published image always describes its payload. When the sender converted the
  image before compressing it and its own encoding no longer fits the payload, the matching
  BGR-ordered encoding is published instead and a warning is logged. A `yuv422` camera behaves that
  way, because `compressed_image_transport` converts the image to BGR before compressing it.
- `encoding: rgb8` and `encoding: bgr8` reduce every image to 8 bits with three channels: a
  grayscale image is inflated to three identical channels, a 16-bit image loses its lower 8 bits,
  and an alpha channel is dropped.
- An image that cannot be decoded, or whose payload no `sensor_msgs` encoding describes, is dropped
  and a warning is logged.

## (Optional) Error detection and handling

## (Optional) Performance characterization

## (Optional) References/External links

## (Optional) Future extensions / Unimplemented parts

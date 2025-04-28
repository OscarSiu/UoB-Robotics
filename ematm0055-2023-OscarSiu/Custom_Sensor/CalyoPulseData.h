// Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "carla/rpc/Location.h"

#include <cstdint>
#include <vector>
#include <cstdio>

namespace carla {

//   namespace ros2 {
//   class ROS2;
// }

namespace sensor {

namespace s11n {
  class CalyoPulseSerializer;
}

namespace data {

  //#pragma pack(push, 1) // Ensures no extra padding is added, optimizing memory storage
  struct CalyoPulseDetection {
    //geom::Location point{};
    float velocity; // m/s
    float azimuth;  // rad
    float altitude; // rad
    float depth;    // m
    float intensity; //dB
  };

  class CalyoPulseData {
    static_assert(sizeof(float) == sizeof(uint32_t), "Invalid float size"); // make sure it is 4 bytes
    static_assert(sizeof(float) * 5 == sizeof(CalyoPulseDetection), "Invalid RadarDetection size"); // make sure it is 16 bytes

  public:
    explicit CalyoPulseData() = default;

    constexpr static auto detection_size = sizeof(CalyoPulseDetection);

    CalyoPulseData &operator=(CalyoPulseData &&) = default;

    /// Set a new resolution for the RadarData.
    /// Allocates / Deallocates space in memory if needed.
    ///
    /// @warning This is expensive, not to be called each tick!
    void SetResolution(uint32_t resolution) {
      // Remove the capacity of the vector to zero
      _detections.clear();
      _detections.shrink_to_fit();
      // Set the new vector's capacity
      _detections.reserve(resolution);
    }

    /// Returns the number of current detections.
    size_t GetDetectionCount() const {
      return _detections.size();
    }

    /// Deletes the current detections.
    /// It doesn't change the resolution nor the allocated memory.
    void Reset() {
      _detections.clear();
    }

    /// Adds a new detection.
    void WriteDetection(CalyoPulseDetection detection) {
      _detections.push_back(detection);
    }

  private:
    std::vector<CalyoPulseDetection> _detections;

  friend class s11n::CalyoPulseSerializer;
  };

} // namespace s11n
} // namespace sensor
} // namespace carla
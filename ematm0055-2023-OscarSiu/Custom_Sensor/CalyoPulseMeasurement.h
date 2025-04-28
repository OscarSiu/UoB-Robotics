// LibCarla/source/carla/sensor/data/CalyoPulseMeasurement.h

//DONE

#pragma once

#include "carla/Debug.h"
#include "carla/sensor/data/Array.h"

#include "carla/rpc/Location.h"
#include "carla/sensor/s11n/CalyoPulseSerializer.h"
#include "carla/sensor/data/CalyoPulseData.h"

namespace carla {
namespace sensor {
namespace data {

  /// Measurement produced by a Radar. Consists of an array of RadarDetection.
  /// A RadarDetection contains 4 floats: velocity, azimuth, altitude and depth
  class CalyoPulseMeasurement : public Array<data::CalyoPulseDetection> {
    using Super = Array<data::CalyoPulseDetection>;
  protected:

    using Serializer = s11n::CalyoPulseSerializer;

    friend Serializer;

    explicit CalyoPulseMeasurement(RawData &&data)
      : Super(0u, std::move(data)) {}

  public:

    Super::size_type GetDetectionAmount() const {
      return Super::size();
    }
  };

} // namespace data
} // namespace sensor
} // namespace carla


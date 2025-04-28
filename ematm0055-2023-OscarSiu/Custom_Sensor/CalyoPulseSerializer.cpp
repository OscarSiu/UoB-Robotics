// LibCarla/source/carla/sensor/s11n/CalyoPulseSerializer.cpp

//DONE

#include "carla/sensor/s11n/CalyoPulseSerializer.h"
#include "carla/sensor/data/CalyoPulseMeasurement.h"

namespace carla {
    namespace sensor {
        namespace s11n {
            
            SharedPtr<SensorData> CalyoPulseSerializer::Deserialize(RawData &&data) {
                return SharedPtr<data::CalyoPulseMeasurement>(
                    new data::CalyoPulseMeasurement{std::move(data)});
            }

        } // namespace s11n
    } // namespace sensor
} // namespace carla
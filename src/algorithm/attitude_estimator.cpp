#include "algorithm/attitude_estimator.h"
#include <algorithm>

namespace imu_algorithm {

AttitudeEstimator::AxisMode AttitudeEstimator::AxisModeFromString(const std::string& name) {
  std::string lower = name;
  std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);

  if (lower == "6" || lower == "6axis" || lower == "six_axis" || lower == "six" || lower == "6-axis" ||
      lower == "6_axis") {
    return AxisMode::SIX_AXIS;
  }

  return AxisMode::NINE_AXIS;
}

} // namespace imu_algorithm

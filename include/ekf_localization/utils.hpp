#pragma once

#include <cmath>

namespace ekf_localization {

/**
 * @brief Normalizes an angle to the range [-π, π]
 * @param angle The angle in radians to normalize
 * @return The normalized angle in radians
 */
double normalizeAngle(double angle);

/**
 * @brief Wraps angle difference to [-π, π] range
 * @param angle1 First angle in radians
 * @param angle2 Second angle in radians  
 * @return The wrapped difference (angle1 - angle2) in radians
 */
double angleDifference(double angle1, double angle2);

} // namespace ekf_localization
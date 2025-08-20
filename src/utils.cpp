#include "ekf_localization/utils.hpp"

namespace ekf_localization {

double normalizeAngle(double angle) {
    // note 数学公式: angle - 2π * floor((angle + π) / 2π)
    return angle - 2.0 * M_PI * std::floor((angle + M_PI) / (2.0 * M_PI));
    // return std::remainder(angle, 2.0 * M_PI);
}

double angleDifference(double angle1, double angle2) {
    // 先计算差值，然后归一化到[-π, π]范围
    return normalizeAngle(angle1 - angle2);
}

} // namespace ekf_localization
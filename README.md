# EKF Phase-1 

``` c++
EkfCore::EkfCore() : initialized_(false) {
    // Initialize process noise covariance Q
    process_noise_ = Eigen::Matrix3d::Identity() * 0.0025;
    process_noise_(2,2) = 0.0036; // Higher uncertainty for angle
}
```

x、y值方差为 0.0025，既标准差为 0.05m; yaw值方差 0.0036，既标准差为 0.06rad 。
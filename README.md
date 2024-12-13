# imu_low_pass_filter
IMU linear acceleration filter.

# Usage
## 0. Install
```
cd ${your workspace}
mkdir -p imu_low_pass_filter_ws/src
cd imu_low_pass_filter_ws/src
git clone https://github.com/HiOnes/imu_lowpass_filter.git
cd -
catkin_make
source devel/setup.bash
```

## 1. Run
```
rosrun imu_low_pass_filter imu_low_pass_filter
```
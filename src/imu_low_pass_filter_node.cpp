#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <ros/time.h>
#include <ros/impl/time.h>

// params
const double sample_rate[3] = {200.0, 200.0, 400.0};
const double cutoffFrequency = 80.0;

ros::Subscriber _imu_in;
ros::Publisher _imu_out;

class LowPassFilter { 
  private: double alpha; // 滤波器系数，取值范围[0, 1] 
  double filteredValue; // 滤波后的数值 
  public: LowPassFilter(double alpha) : alpha(alpha), filteredValue(0.0) {} 
  double filter(double rawValue) { 
    // 一阶低通滤波器公式 
    filteredValue = alpha * rawValue + (1.0 - alpha) * filteredValue; 
    return filteredValue; 
    } 
};

double get_alpha(const double& sample_rate, const double& cutoffFrequency)
{
    double tau = 1.0 / (2.0 * M_PI * cutoffFrequency);
    double alpha = 1.0 / (1.0 + tau * sample_rate);
    return alpha;
}

const double alpha_x = get_alpha(sample_rate[0], cutoffFrequency);
const double alpha_y = get_alpha(sample_rate[1], cutoffFrequency);
const double alpha_z = get_alpha(sample_rate[2], cutoffFrequency);
LowPassFilter lowPassFilter_x(alpha_x);
LowPassFilter lowPassFilter_y(alpha_y);
LowPassFilter lowPassFilter_z(alpha_z);

void imu_low_pass_filter(const sensor_msgs::ImuConstPtr &msg)
{
    double this_time;
    sensor_msgs::Imu out_msg;
    out_msg.header = msg->header;
    out_msg.linear_acceleration = msg->linear_acceleration;

    double filtered_x = lowPassFilter_x.filter(msg->linear_acceleration.x);
    double filtered_y = lowPassFilter_y.filter(msg->linear_acceleration.y);
    double filtered_z = lowPassFilter_z.filter(msg->linear_acceleration.z);

    out_msg.linear_acceleration.x = filtered_x;
    out_msg.linear_acceleration.y = filtered_y;
    out_msg.linear_acceleration.z = filtered_z;
		out_msg.angular_velocity = msg->angular_velocity;

    _imu_out.publish(out_msg);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "imu_low_pass_filter_node");
    ros::NodeHandle nh;

    _imu_out = nh.advertise<sensor_msgs::Imu>("/imu/low_pass/data", 5);
    _imu_in = nh.subscribe("/imu/data", 5, imu_low_pass_filter);

		ros::spin();
    
    return 0;
}
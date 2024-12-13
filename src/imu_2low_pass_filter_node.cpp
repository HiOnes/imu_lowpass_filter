#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <ros/time.h>
#include <ros/impl/time.h>
#include "LowPassFilter2p.hpp"

using namespace std;

ros::Subscriber _imu_in;
ros::Publisher _imu_out;
static double gravity[3] = {0,0,0};

static int first_frame_seq = 0;
static bool first_flag = false;
static double first_time = 0.0;

const double sample_rate = 200.0;
const double cutoffFrequency = 15.0;

math::LowPassFilter2p<float> lowPassFilter_x(sample_rate, cutoffFrequency);
math::LowPassFilter2p<float> lowPassFilter_y(sample_rate, cutoffFrequency);
math::LowPassFilter2p<float> lowPassFilter_z(sample_rate, cutoffFrequency);

void imu_low_pass_filter(const sensor_msgs::ImuConstPtr &msg)
{
    double this_time;
    sensor_msgs::Imu out_msg;
    if(first_flag == false)
    {
      first_flag = true;
      first_frame_seq = msg->header.seq;

      out_msg.header.stamp = msg->header.stamp;

      first_time = msg->header.stamp.toSec();
    }
    else{
      this_time = (msg->header.seq - first_frame_seq) * 0.0050 + first_time;
      out_msg.header.seq = msg->header.seq;
      out_msg.header.stamp = ros::Time().fromSec(this_time);
    }
    
    out_msg.header.frame_id = msg->header.frame_id;
    out_msg.linear_acceleration = msg->linear_acceleration;

    float filtered_x = lowPassFilter_x.apply(msg->linear_acceleration.x);
    float filtered_y = lowPassFilter_y.apply(msg->linear_acceleration.y);
    float filtered_z = lowPassFilter_z.apply(msg->linear_acceleration.z);

    // cout << "filtered_z: " << filtered_z << endl;

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
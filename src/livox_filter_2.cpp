#include "ros/ros.h"
#include "livox_ros_driver2/CustomMsg.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/String.h"
#include <sstream>

class livox_filter
{
private:
    ros::NodeHandle nh;
    ros::Subscriber livox_lidar_sub;
    ros::Subscriber livox_imu_sub;
    ros::Publisher livox_lidar_pub;
    ros::Publisher livox_imu_pub;
    int threshold;
public:
    livox_filter();
    ~livox_filter();

    void lidar_callback(const livox_ros_driver2::CustomMsg::ConstPtr& msg);
    void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);
    void print_threshold();
};

livox_filter::livox_filter() {
    nh.param("/livox_filter/threshold", threshold, 15); 
    ROS_INFO("Threshold value: %d", threshold);

    livox_lidar_sub = nh.subscribe("/livox/lidar", 1000, &livox_filter::lidar_callback, this);
    livox_imu_sub = nh.subscribe("/livox/imu", 1000, &livox_filter::imu_callback, this);
    livox_lidar_pub = nh.advertise<livox_ros_driver2::CustomMsg>("/filtered_livox_lidar", 1000);
    livox_imu_pub = nh.advertise<sensor_msgs::Imu>("/filtered_livox_imu", 1000);
}

livox_filter::~livox_filter(){}

void livox_filter::lidar_callback(const livox_ros_driver2::CustomMsg::ConstPtr& msg){
    livox_ros_driver2::CustomMsg filtered_msg;

    // Copy header information
    filtered_msg.header = msg->header;
    filtered_msg.timebase = msg->timebase;
    filtered_msg.point_num = 0; 

    for (const auto& point : msg->points)
    {
        if (point.reflectivity > threshold)
        {
            filtered_msg.points.push_back(point);
            filtered_msg.point_num++;
        }
    }

    if (filtered_msg.point_num > 0)
        livox_lidar_pub.publish(filtered_msg);
}

void livox_filter::imu_callback(const sensor_msgs::Imu::ConstPtr& msg){
    livox_imu_pub.publish(msg);
}

void livox_filter::print_threshold(){
    ROS_INFO("Threshold value: %d", threshold);
} 

int main(int argc, char** argv)
{
    ros::init(argc, argv, "livox_filter");
    livox_filter livox_filter;
    ros::spin();
    return 0;
}

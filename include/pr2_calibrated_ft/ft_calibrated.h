#pragma once
#include <ros/ros.h>

#include <geometry_msgs/WrenchStamped.h>

class ft_calibrated
{
    public:
        ft_calibrated(ros::NodeHandle &nh);
        ~ft_calibrated();
        void raw_value_callback(const geometry_msgs::WrenchStamped::ConstPtr &msg);
    private:
        ros::NodeHandle nh;
        ros::Subscriber raw_ft;
        ros::Publisher calibrated_ft, zeroed_ft;

};

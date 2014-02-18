#include "pr2_calibrated_ft/ft_calibrated.h"

ft_calibrated::ft_calibrated(ros::NodeHandle &nh)
{
    this->nh = nh;

    raw_ft = nh.subscribe("/ft/r_gripper_motor", 0, &ft_calibrated::raw_value_callback, this);

    calibrated_ft = nh.advertise<geometry_msgs::WrenchStamped>("ft/r_gripper_motor/calibrated", 1);

    //TODO Implement zeroing
//    zeroed_ft = nh.advertise<geometry_msgs::WrenchStamped>("ft/r_gripper_motor/zeroed", 1);
}

ft_calibrated::~ft_calibrated()
{
}

void ft_calibrated::raw_value_callback(const geometry_msgs::WrenchStamped::ConstPtr &msg)
{
    geometry_msgs::WrenchStamped calibrated = *msg;

    struct
    {
        double x,y,z;
    } vec;

    if (!ros::param::getCached("zero_force_x", vec.x))
        vec.x = 0.0;
    if (!ros::param::getCached("zero_force_y", vec.y))
        vec.y = 0.0;
    if (!ros::param::getCached("zero_force_z", vec.z))
        vec.z = 0.0;

    calibrated.wrench.force.x -= vec.x;
    calibrated.wrench.force.y -= vec.y;
    calibrated.wrench.force.z -= vec.z;

    if (!ros::param::getCached("zero_torque_x", vec.x))
        vec.x = 0.0;
    if (!ros::param::getCached("zero_torque_y", vec.y))
        vec.y = 0.0;
    if (!ros::param::getCached("zero_torque_z", vec.z))
        vec.z = 0.0;

    calibrated.wrench.torque.x -= vec.x;
    calibrated.wrench.torque.y -= vec.y;
    calibrated.wrench.torque.z -= vec.z;

    calibrated_ft.publish(calibrated);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ft_calibrated");
    ros::NodeHandle nh;

    ft_calibrated node(nh);
    ros::spin();
    
    return 0;
}

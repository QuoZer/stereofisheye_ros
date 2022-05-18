#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"


ros::Publisher stamped_odometry;
void unityCallback(const nav_msgs::Odometry& msg)
{
    nav_msgs::Odometry bufmsg = msg;
    ros::Time moment = ros::Time::now();
    bufmsg.header.stamp = moment;
    stamped_odometry.publish(bufmsg);

    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = moment;
    transformStamped.header.frame_id = "odom";
    transformStamped.child_frame_id = "base_link";
    transformStamped.transform.translation.x = bufmsg.pose.pose.position.x;
    transformStamped.transform.translation.y = bufmsg.pose.pose.position.y;
    transformStamped.transform.translation.z = bufmsg.pose.pose.position.z;
    //q.setValue(bufmsg.pose.pose.orientation.x, bufmsg.pose.pose.orientation.y, bufmsg.pose.pose.orientation.z);
    transformStamped.transform.rotation.x = bufmsg.pose.pose.orientation.x;
    transformStamped.transform.rotation.y = bufmsg.pose.pose.orientation.y;
    transformStamped.transform.rotation.z = bufmsg.pose.pose.orientation.z;
    transformStamped.transform.rotation.w = bufmsg.pose.pose.orientation.w;
    
    br.sendTransform(transformStamped);

}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "odo_rep");
    ros::NodeHandle nh;

    ros::Subscriber odo_sub = nh.subscribe("unity/odometry", 1000, unityCallback);
    stamped_odometry = nh.advertise<nav_msgs::Odometry>("unity/odometry_stamped", 1);

    ros::spin();

    return 0;
}
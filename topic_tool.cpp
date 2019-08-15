
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <tf/tf.h>

geometry_msgs::PoseStamped host_mocap;
void host_pos(const geometry_msgs::PoseStamped::ConstPtr& msg)         /*get position and attitude data*/
{
    double roll, pitch, yaw;
    host_mocap = *msg;

    tf::Quaternion Q(
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w);
    tf::Matrix3x3(Q).getRPY(roll,pitch,yaw);
}
/*void host_pos(const nav_msgs::Odometry::ConstPtr& msg)
{
    double roll, pitch, yaw;

    host_mocap.header = msg->header;
    host_mocap.pose.position = msg->pose.pose.position;
    host_mocap.pose.orientation = msg->pose.pose.orientation;

    tf::Quaternion Q(
        host_mocap.pose.orientation.x,
        host_mocap.pose.orientation.y,
        host_mocap.pose.orientation.z,
        host_mocap.pose.orientation.w);
    tf::Matrix3x3(Q).getRPY(roll,pitch,yaw);

}*/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "topic");
    ros::NodeHandle nh;

    ros::Publisher mocap_pos_pub = nh.advertise<geometry_msgs::PoseStamped>                 /*output topic*/
                                   ("/mavros/vision_pose/pose", 2);
    ros::Subscriber host_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/RigidBody7/pose", 10, host_pos); /*input topic*/
//    ros::Subscriber host_sub = nh.subscribe<nav_msgs::Odometry> ("/vins_estimator/odometry",2, host_pos);
//    ros::Subscriber host_sub = nh.subscribe<nav_msgs::Odometry> ("/vins_estimator/imu_propagate",2, host_pos);
 
    ros::Rate rate(30);

    while (ros::ok()) {
	
        ROS_INFO("vision_pose: %.3f,%.3f,%.3f",host_mocap.pose.position.x,host_mocap.pose.position.y,host_mocap.pose.position.z);
        mocap_pos_pub.publish(host_mocap);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;	
}




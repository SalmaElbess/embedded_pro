#include <tf/tf.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

ros::Publisher pub_pose_;
ros::Publisher pub_pose_stamp;

void odometryCallback_(const sensor_msgs::Imu msg) {
    geometry_msgs::Pose2D pose2d;
    pose2d.x = msg.orientation.x;
    pose2d.y = msg.orientation.y;

    tf::Quaternion q(
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    pose2d.theta = yaw;
    pub_pose_.publish(pose2d);

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.pose.orientation = msg.orientation;
    pose_stamped.header.frame_id = "imu_link";
    pub_pose_stamp.publish(pose_stamped);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "conversion_node");

    ros::NodeHandle nh_;

    ros::Subscriber sub_odom_ = nh_.subscribe("imu/data", 1, odometryCallback_);
    pub_pose_ = nh_.advertise<geometry_msgs::Pose2D>("pose2d", 100);
    pub_pose_stamp = nh_.advertise<geometry_msgs::PoseStamped>("pose_stamp", 100);
    ros::spin();

    return 0;
}
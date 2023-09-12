#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>


void odometryCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
    // Extract relevant information from the Odometry message
    geometry_msgs::Pose pose = odom_msg->pose.pose;
    std::string frame_id = odom_msg->header.frame_id;
    std::string child_frame_id = odom_msg->child_frame_id;

    // Convert the Pose to a Transform
    tf::Transform transform;
    tf::poseMsgToTF(pose, transform);

    // Broadcast the Transform
    static tf::TransformBroadcaster tf_broadcaster;
    tf_broadcaster.sendTransform(tf::StampedTransform(transform, odom_msg->header.stamp, frame_id, child_frame_id));
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_to_tf_node");
    ros::NodeHandle nh;

    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/base_pose_ground_truth", 1, odometryCallback);

    ros::spin();

    return 0;
}


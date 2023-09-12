#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <eigen3/Eigen/Eigen>

using namespace Eigen;

Vector2d start_pos;
Vector2d target_pos;

ros::Publisher global_path_pub;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "global_path_publisher");
  ros::NodeHandle node("~");

  global_path_pub = node.advertise<nav_msgs::Path>("/global_path", 1);

  node.param<double>("start_x", start_pos.x(), -10.0);
  node.param<double>("start_y", start_pos.y(), 0.0);
  node.param<double>("end_x", target_pos.x(), 9.0);
  node.param<double>("end_y", target_pos.y(), 0.0);

  double dist = (target_pos - start_pos).norm();
  Vector2d diff = (target_pos - start_pos) / dist;

  const double step = 0.1;

  ros::Rate rate(10);
  while (ros::ok())
  {
    nav_msgs::Path global_path;
    global_path.header.stamp = ros::Time::now();
    global_path.header.frame_id = "world";
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose.orientation.x = 0;
    pose_stamped.pose.orientation.y = 0;
    pose_stamped.pose.orientation.z = 0;
    pose_stamped.pose.orientation.w = 1;

    int idx = 0;
    for (double i = 0.0; i < dist; i += step)
    {
      pose_stamped.header.seq = idx++;

      Vector2d waypoint = start_pos + i * diff;
      pose_stamped.pose.position.x = waypoint.x();
      pose_stamped.pose.position.y = waypoint.y();
      pose_stamped.pose.position.z = 0;

      global_path.poses.push_back(pose_stamped);
    }

    global_path_pub.publish(global_path);

    rate.sleep();
  }

  return 0;
}
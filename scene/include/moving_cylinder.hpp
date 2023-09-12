#ifndef MOVING_CYLINDER_H
#define MOVING_CYLINDER_H

#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/SetModelState.h>

class MovingCylinder
{
public:
  MovingCylinder()
  {
    model_state_.model_name = "cylinder_" + std::to_string(id_);
    model_state_.reference_frame = "world";
    model_state_.pose.orientation.x = 0.0;
    model_state_.pose.orientation.y = 0.0;
    model_state_.pose.orientation.z = 0.0;
    model_state_.pose.orientation.w = 1.0;
    id_++;
  }

  void setPosition(const geometry_msgs::Point& position)
  {
    model_state_.pose.position = position;
  }

  void setVel(const geometry_msgs::Twist& twist)
  {
    twist_ = twist;
  }

  void updateState()
  {
    model_state_.pose.position.x += twist_.linear.x;
    model_state_.pose.position.y += twist_.linear.y;
  }

  gazebo_msgs::ModelState model_state_;
  geometry_msgs::Twist twist_;

private:
  static int id_;
};

#endif

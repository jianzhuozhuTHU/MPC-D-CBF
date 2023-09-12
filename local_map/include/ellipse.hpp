#pragma once

#include <boost/circular_buffer.hpp>

#include <CGAL/Cartesian.h>
#include <CGAL/Min_ellipse_2.h>
#include <CGAL/Min_ellipse_2_traits_2.h>
#include <CGAL/Exact_rational.h>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <grid_map_core/GridMap.hpp>

#include <eigen3/Eigen/Dense>

using Obstacle = std::vector<Eigen::Array2i>;

struct Ellipse
{
  double semimajor; // Length of semi-major axis
  double semiminor; // Length of semi-minor axis
  double cx;        // x-coordinate of center
  double cy;        // y-coordinate of center
  double theta;     // Rotation angle
  int label;        // label of obstacle
  double variance;  // variance of ab^1.3
};

float localmap_x_size, localmap_y_size;
float resolution = 0.05, _inv_resolution = 1.0 / resolution;

using namespace std;
using namespace Eigen;

double calculate_dis(const Ellipse &e1, const Ellipse &e2)
{
  return sqrt(pow(e1.cx - e2.cx, 2) + pow(e1.cy - e2.cy, 2));
}

/*!MSE function using CGAL tools.
 * @reference: https://doc.cgal.org/latest/Bounding_volumes/classCGAL_1_1Min__ellipse__2.html
 */
typedef CGAL::Exact_rational NT;
typedef CGAL::Cartesian<NT> K;
typedef CGAL::Point_2<K> Point;
typedef CGAL::Min_ellipse_2_traits_2<K> Traits;
typedef CGAL::Min_ellipse_2<Traits> Min_ellipse;
bool get_min_area_ellipse(const std::vector<Point> &pts, Ellipse &ellipse)
{
  Min_ellipse me(pts.begin(), pts.end(), true);
  if (me.is_degenerate())
    return false;
  // r*x^2 + s*y^2 + t*x*y + u*x + v*y + w = 0
  double r, s, t, u, v, w;
  me.ellipse().double_coefficients(r, s, t, u, v, w);
  double a = -std::sqrt(2 * (r * v * v + s * u * u - t * u * v + (t * t - 4 * r * s) * w) *
                        ((r + s) + std::sqrt((r - s) * (r - s) + t * t))) /
             (t * t - 4 * r * s);
  double b = -std::sqrt(2 * (r * v * v + s * u * u - t * u * v + (t * t - 4 * r * s) * w) *
                        ((r + s) - std::sqrt((r - s) * (r - s) + t * t))) /
             (t * t - 4 * r * s);
  double cx = (2 * s * u - t * v) / (t * t - 4 * r * s);
  double cy = (2 * r * v - t * u) / (t * t - 4 * r * s);
  double theta;
  if (t != 0)
    theta = std::atan(1 / t * (s - r - std::sqrt((r - s) * (r - s) + t * t)));
  else if (r < s)
    theta = 0;
  else
    theta = M_PI;
  if (b > a)
  {
    swap(a, b);
    theta += M_PI / 2;
  }
  // if (t > 0.001)
  //   theta = std::atan(t / (s - r - std::sqrt((r - s) * (r - s) + t * t)));
  // else if (r - s < 0.001)
  //   theta = 0;
  // else
  //   theta = M_PI / 2;

  ellipse = Ellipse{a, b, cx, cy, theta};
  return true;
}

Obstacle down_sample_points(const Obstacle &pts, int &x_min, int &y_min, int &x_max, int &y_max)
{
  for (const auto &pt : pts)
  {
    if (pt(0) < x_min)
      x_min = pt(0);
    if (pt(1) < y_min)
      y_min = pt(1);
    if (pt(0) > x_max)
      x_max = pt(0);
    if (pt(1) > y_max)
      y_max = pt(1);
  }

  int x_up = x_max - 0.1 * (x_max - x_min);
  x_up = x_up > x_min ? x_up : x_max;
  int x_down = x_min + 0.1 * (x_max - x_min);
  x_down = x_down < x_max ? x_down : x_min;
  int y_up = y_max - 0.1 * (y_max - y_min);
  y_up = y_up > y_min ? y_up : y_max;
  int y_down = y_min + 0.1 * (y_max - y_min);
  y_down = y_down < y_max ? y_down : y_min;

  if (pts.size() > 100)
  {
    Obstacle ds_pts;
    for (const auto &pt : pts)
    {
      if (pt(0) >= x_down && pt(0) <= x_up && pt(1) >= y_down && pt(1) <= y_up)
        continue;

      ds_pts.emplace_back(pt);
    }
    return ds_pts;
  }
  return pts;
}

vector<Ellipse> get_ellipse_array(std::vector<Obstacle> &seg_obs, const grid_map::GridMap &localmap_obs)
{
  std::vector<Ellipse> result;

  for (const auto &obs : seg_obs)
  {
    if (obs.size() > 500)
      continue;
    int x_min = localmap_x_size * _inv_resolution, y_min = localmap_x_size * _inv_resolution, x_max = 0, y_max = 0;
    Obstacle ds_pts = down_sample_points(obs, x_min, y_min, x_max, y_max);
    if (x_max - x_min > 3 * _inv_resolution || y_max - y_min > 3 * _inv_resolution)
      continue;

    vector<Point> pts;
    for (const auto &pt : ds_pts)
    {
      if (pt(0) >= localmap_x_size / resolution || pt(1) >= localmap_x_size / resolution)
        continue;

      Vector3d pos;
      localmap_obs.getPosition3("elevation", pt, pos);
      pts.emplace_back(pos(0), pos(1));
    }

    Ellipse ellipse;
    if (!get_min_area_ellipse(pts, ellipse))
    {
      Eigen::Array2i grid_center((x_max + x_min) / 2, (y_min + y_max) / 2);

      float radius = sqrt((x_max - x_min) * (x_max - x_min) + (y_max - y_min) * (y_max - y_min)) * resolution / 2;
      Vector3d pos;
      localmap_obs.getPosition3("gradient_map", grid_center, pos);
      ellipse.cx = pos(0);
      ellipse.cy = pos(1);
      ellipse.semimajor = max(radius, 2 * resolution);
      ellipse.semiminor = resolution;
      ellipse.theta = (x_max - x_min == 0) ? M_PI / 2 : (std::atan((y_max - y_min) / (x_max - x_min)) + M_PI / 2);
    }

    if (ellipse.semimajor <= 20 && ellipse.semiminor <= 20)
      result.push_back(ellipse);
  }
  return result;
}

void ab_variance_calculation(vector<Ellipse> &input_vector)
{
  static vector<boost::circular_buffer<Eigen::Vector2d>> list;
  for (auto &input : input_vector)
  {
    int label = input.label;
    int current_size = list.size();
    if (current_size < label)
    {
      list.resize(label);
      for (size_t i = current_size; i < label; i++)
        list[i].set_capacity(10);
    }
    list[label - 1].push_back(Eigen::Vector2d(input.semimajor, input.semiminor));
    size_t list_size = list[label - 1].size();
    MatrixXd vac_mat(2, list_size);
    for (size_t i = 0; i < list_size; i++)
      vac_mat.col(i) = list[label - 1][i];
    MatrixXd zeroMeanMat = vac_mat.colwise() - vac_mat.rowwise().mean();
    MatrixXd covMat = (zeroMeanMat * zeroMeanMat.transpose()) / list_size;
    double variance = pow(covMat.trace(), 1.3);
    if (list_size < 3)
      variance += 0.0005 * (3 - list_size);
    input.variance = variance;
  }
}

void visEllipse(const std::vector<Ellipse> &obs_ellipses, ros::Publisher &_Ellipses_vis_pub)
{
  // ellipse
  visualization_msgs::MarkerArray ellipse_vis;

  visualization_msgs::Marker shape_vis;
  shape_vis.header.frame_id = "world";
  shape_vis.header.stamp = ros::Time::now();
  shape_vis.ns = "ellipse";
  shape_vis.type = visualization_msgs::Marker::CYLINDER;
  shape_vis.action = visualization_msgs::Marker::ADD;
  shape_vis.lifetime = ros::Duration(0.15);

  // number
  visualization_msgs::Marker text_vis;
  text_vis.header.frame_id = "world";
  text_vis.header.stamp = ros::Time::now();
  text_vis.ns = "text";
  text_vis.action = visualization_msgs::Marker::ADD;
  text_vis.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  text_vis.lifetime = ros::Duration(0.1);
  text_vis.scale.z = 0.3;
  text_vis.color.r = 1;
  text_vis.color.g = 1;
  text_vis.color.b = 1;
  text_vis.color.a = 1;

  for (size_t i = 0; i < obs_ellipses.size(); i++)
  {
    shape_vis.color.a = 0.9;
    if (obs_ellipses[i].variance < 0.0007)
    {
      shape_vis.color.r = 0;
      shape_vis.color.g = 0;
      shape_vis.color.b = 1;
    }
    else if (obs_ellipses[i].variance > 0.005)
    {
      shape_vis.color.r = 1;
      shape_vis.color.g = 0;
      shape_vis.color.b = 0;
    }
    else
    {
      float dt = (obs_ellipses[i].variance - 0.0007) / (0.005 - 0.0007);
      if (dt > 0.5)
      {
        shape_vis.color.r = 1;
        shape_vis.color.g = 1 - dt;
        shape_vis.color.b = 1 - dt;
      }
      else
      {
        shape_vis.color.r = dt;
        shape_vis.color.g = dt;
        shape_vis.color.b = 1;
      }
    }

    shape_vis.id = i;
    shape_vis.pose.orientation.x = 0.0;
    shape_vis.pose.orientation.y = 0.0;
    shape_vis.pose.orientation.z = sin(obs_ellipses[i].theta / 2);
    shape_vis.pose.orientation.w = cos(obs_ellipses[i].theta / 2);
    shape_vis.pose.position.x = obs_ellipses[i].cx;
    shape_vis.pose.position.y = obs_ellipses[i].cy;
    shape_vis.pose.position.z = -0.0;
    shape_vis.scale.x = 2 * obs_ellipses[i].semimajor;
    shape_vis.scale.y = 2 * obs_ellipses[i].semiminor;
    shape_vis.scale.z = 0.75;

    text_vis.id = obs_ellipses[i].label;

    text_vis.text = to_string(obs_ellipses[i].label);
    text_vis.pose.position.x = obs_ellipses[i].cx - 0.3;
    text_vis.pose.position.y = obs_ellipses[i].cy;
    text_vis.pose.position.z = 0.2;

    // publish
    ellipse_vis.markers.push_back(shape_vis);
    ellipse_vis.markers.push_back(text_vis);
  }

  _Ellipses_vis_pub.publish(ellipse_vis);
}

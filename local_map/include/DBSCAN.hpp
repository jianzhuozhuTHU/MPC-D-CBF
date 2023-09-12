#pragma once

#include <vector>
#include <cmath>
#include <queue>

#define UNCLASSIFIED -1
#define NOISE -2

class DBSCAN
{
public:
  struct Point
  {
    float x, y;
    int obsID = UNCLASSIFIED; // all obs point inited as UNCLASSIFIED
  };

  int cluster_num;

  DBSCAN(float eps, unsigned int minpts, std::vector<Point> &points) : c_points(points)
  {
    c_points_eps = eps;
    c_points_minpts = minpts;
    c_points_size = c_points.size();
    main_process();
  }

private:
  size_t c_points_size;
  unsigned int c_points_minpts;
  float c_points_eps;
  std::vector<Point> &c_points;

  int curr_ID;

  void main_process();
  bool expanding(const int &index);
  std::vector<int> calculate_cluster(const int &index);
  inline double calculate_dis(const Point &point1, const Point &point2);
};

void DBSCAN::main_process()
{
  curr_ID = 1;
  for (size_t i = 0; i < c_points_size; i++)
  {
    if (c_points[i].obsID == UNCLASSIFIED && expanding(i))
      curr_ID++;
  }
  cluster_num = curr_ID - 1;
}

bool DBSCAN::expanding(const int &index)
{
  std::vector<int> clusterRange = calculate_cluster(index);

  if (clusterRange.size() < c_points_minpts)
  {
    c_points[index].obsID = NOISE;
    return false;
  }

  c_points[index].obsID = curr_ID;
  for (const auto &c_index : clusterRange)
    c_points[c_index].obsID = curr_ID;
  // rolling
  while (!clusterRange.empty())
  {
    int index = clusterRange.front();
    clusterRange.erase(clusterRange.begin());
    std::vector<int> clusterNeighors = calculate_cluster(index);
    if (clusterNeighors.size() < c_points_minpts)
      continue;

    for (const auto &c_index : clusterNeighors)
      c_points[c_index].obsID = curr_ID;

    clusterRange.insert(clusterRange.end(), clusterNeighors.begin(), clusterNeighors.end());
  }

  return true;
}

std::vector<int> DBSCAN::calculate_cluster(const int &idx)
{
  std::vector<int> cluster_index;
  for (size_t c_idx = 0; c_idx < c_points_size; c_idx++)
  {
    if (c_points[c_idx].obsID < 0 && calculate_dis(c_points[idx], c_points[c_idx]) <= c_points_eps && idx != c_idx)
      cluster_index.push_back(c_idx);
  }
  return cluster_index;
}

inline double DBSCAN::calculate_dis(const Point &point1, const Point &point2)
{
  return sqrt(pow(point1.x - point2.x, 2) + pow(point1.y - point2.y, 2));
}
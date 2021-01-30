#include <stdafx.hpp>
#include "PointCloudMap.hpp"

PointCloudMap::PointCloudMap() {

}

PointCloudMap::~PointCloudMap() {
  
}

void PointCloudMap::addPose(Pose2D pose) {
  poses.emplace_back(pose);
}

void PointCloudMap::addPoints(const std::vector<LPoint2D>& points, int step /*= 5*/) {

  for (size_t i = 0; i < points.size(); i+=step) {
  globalMap.emplace_back(points[i]); 
  }
}

Pose2D PointCloudMap::getLastPose() {
  return poses.back();
}

#include <stdafx.hpp>
#include "PointCloudMapGT.hpp"

PointCloudMapGT::PointCloudMapGT()
  : PointCloudMap()
{
}

PointCloudMapGT::~PointCloudMapGT() {

}

void PointCloudMapGT::addPoints(const std::vector<LPoint2D> &points, int step /*= 5*/) {

  for (size_t i = 0; i < points.size(); i+=step) {
    allLps.emplace_back(points[i]); 
  }
}

void PointCloudMapGT::clear() {
  allLps.clear();
}


void PointCloudMapGT::makeGlobalMap() {

  globalMap.clear();
  makeSubsamplePoints(globalMap);
}

void PointCloudMapGT::makeSubsamplePoints(std::vector<LPoint2D>& lps) {
  gt.clear();
  for (size_t i = 0; i < allLps.size(); i++) {
    gt.addPoint(allLps[i]);
  }

  gt.makeCellPoints(1, globalMap);
}
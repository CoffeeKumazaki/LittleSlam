#include <stdafx.hpp>
#include "DataAssociatorGT.hpp"

DataAssociatorGT::DataAssociatorGT()
 : DataAssociator()
{
}

DataAssociatorGT::~DataAssociatorGT() {

}

void DataAssociatorGT::setRefBase(const std::vector<LPoint2D> &lps) {
  gt.clear();
  for(int i = 0; i < lps.size(); i++) {
    gt.addPoint(lps[i]);
  }
}

double DataAssociatorGT::findCorrespondence(const Scan2D &curScan, const Pose2D &predPose) {

  double dthre = 0.2;
  curLps.clear();
  refLps.clear();

  for (size_t i = 0; i < curScan.lps.size(); i++) {
    const auto lp = curScan.lps[i];
    LPoint2D glp;
    predPose.getGlobalPoint(lp, glp);

    double dmin = __DBL_MAX__;
    LPoint2D closest;
    bool find = gt.findClosestPoint(glp, closest);

    if (find) {
      curLps.emplace_back(glp);
      refLps.emplace_back(closest);
    }
  }

  double ratio = 1.0 * curLps.size() / curScan.lps.size();
  return ratio;
}
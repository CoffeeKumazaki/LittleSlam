#include <stdafx.hpp>
#include "DataAssociator.hpp"


DataAssociator::DataAssociator() {

}

DataAssociator::~DataAssociator() {

}

double DataAssociator::findCorrespondence(const Scan2D& curScan, const Pose2D& predPose) {
  double dthre = 0.2;
  curLps.clear();
  refLps.clear();

  for (size_t i = 0; i < curScan.lps.size(); i++) {
    const auto lp = curScan.lps[i];
    LPoint2D glp;
    predPose.getGlobalPoint(lp, glp);

    double dmin = __DBL_MAX__;
    LPoint2D minP;

    bool find = false;
    for (size_t j = 0; j < mapLps.size(); j++) {
      const auto rlp = mapLps[j];

      double dx = glp.x - rlp.x;
      double dy = glp.y - rlp.y;
      double d = dx * dx + dy * dy;
      if (d<=dthre*dthre && d < dmin) {
        dmin = d;
        minP = rlp;
        find = true;
      }
    }

    if (find) {
      curLps.emplace_back(glp);
      refLps.emplace_back(minP);
    }
  }

  double ratio = 1.0 * curLps.size() / curScan.lps.size();

  return ratio;
}

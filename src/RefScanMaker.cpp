#include <stdafx.hpp>
#include "RefScanMaker.hpp"
#include "PointCloudMap.hpp"

RefScanMaker::RefScanMaker() {

}

RefScanMaker::~RefScanMaker() {

}

void RefScanMaker::makeRefScan() {

  refScan.lps.clear();

  Pose2D lastPose = GetPCM().getLastPose();
  for (size_t i = 0; i < GetPCM().lastScan.lps.size(); i++) {

    const auto lp = GetPCM().lastScan.lps[i];
    LPoint2D newlp;

    newlp.x = lastPose.Rmat[0][0] * lp.x + lastPose.Rmat[0][1] * lp.y + lastPose.x;
    newlp.y = lastPose.Rmat[1][0] * lp.x + lastPose.Rmat[1][1] * lp.y + lastPose.y;

    newlp.nx = lastPose.Rmat[0][0] * lp.nx + lastPose.Rmat[0][1] * lp.ny;
    newlp.ny = lastPose.Rmat[1][0] * lp.nx + lastPose.Rmat[1][1] * lp.ny;

    refScan.lps.emplace_back(newlp);
  }

}

void RefScanMaker::getRefScan(Scan2D& _refScan) {

  _refScan = refScan;
}


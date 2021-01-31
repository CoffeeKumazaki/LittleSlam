#include <stdafx.hpp>
#include "PointCloudMap.hpp"
#include "RefScanMaker.hpp"
#include "PoseEstimator.hpp"
#include "ScanMatcher.hpp"

ScanMatcher::ScanMatcher(int _cnt /*= -1*/, double _score_max /*= 1.0*/, int _num_min /*= 50*/) 
  : cnt(_cnt)
  , score_max(_score_max)
  , num_min(_num_min)
{
  prsm = std::make_shared<RefScanMaker>();
  ppe = std::make_shared<PoseEstimator>();
}

ScanMatcher::~ScanMatcher() {
}

bool ScanMatcher::matchScan(const Scan2D& srcScan) {

  if (cnt == 0) {
    growMap(srcScan, curPose);
    prevScan = srcScan;
    return true;
  }

  bool ret = false;
  Pose2D motion;
  Pose2D::calcRelativePose(prevScan.pose, srcScan.pose, motion);

  Pose2D lastPose = GetPCM().getLastPose();
  Pose2D predPose;
  Pose2D::calcPredictionPose(lastPose, motion, predPose);

  prsm->makeRefScan();
  Scan2D refScan;
  prsm->getRefScan(refScan);

  Pose2D estPose;
  double score = ppe->estimatePose(lastPose, estPose);

  ret = score < score_max;

  if (!ret)
    estPose = predPose;

  growMap(srcScan, estPose);
  prevScan = srcScan;

  cnt++;

  return ret;
}

void ScanMatcher::growMap(const Scan2D& srcScan, const Pose2D& pose) {

  std::vector<LPoint2D> globalScan;
  for (size_t i = 0; i < srcScan.lps.size(); i++) {
    const auto lp = srcScan.lps[i];
    if (lp.type == ISOLATE)
      continue;

    double x = pose.Rmat[0][0] * lp.x + pose.Rmat[0][1] * lp.y + pose.x;
    double y = pose.Rmat[1][0] * lp.x + pose.Rmat[1][1] * lp.y + pose.y;

    double nx = pose.Rmat[0][0] * lp.nx + pose.Rmat[0][1] * lp.ny;
    double ny = pose.Rmat[1][0] * lp.nx + pose.Rmat[1][1] * lp.ny;

    LPoint2D maplp;
    maplp.x = x;
    maplp.y = y;
    maplp.nx = nx;
    maplp.ny = ny;
    maplp.type = lp.type;
    globalScan.emplace_back(maplp);
  }

  PointCloudMap &pcm = GetPCM();
  pcm.addPose(pose);
  pcm.addPoints(globalScan);
  pcm.setLastScan(srcScan);
}

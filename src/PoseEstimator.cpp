#include <stdafx.hpp>
#include "PoseOptimizer.hpp"
#include "DataAssociator.hpp"
#include "PoseEstimator.hpp"

PoseEstimator::PoseEstimator()
  : useNum(0)
  , pnrate(0.0)
{
  ppo = std::make_shared<PoseOptimizer>();
  pda = std::make_shared<DataAssociator>();
}

PoseEstimator::~PoseEstimator() {
}

double PoseEstimator::estimatePose(const Pose2D& initPose, Pose2D& estPose) {

  double evMin = 3*10e7;
  double evThre = 0.0001;

  double ev = 0;
  double prevEv = evMin;

  Pose2D pose = initPose;
  Pose2D poseMin = initPose;

  for (size_t i = 0; abs(prevEv - ev) > evThre; i++) {

    if (i > 0)
      prevEv = ev;
    double ratio = pda->findCorrespondence(curScan, pose);
    Pose2D newPose;

    ppo->setPoints(pda->curLps, pda->refLps);
    ev = ppo->OptimizePose(pose, newPose);
    pose = newPose;

    if (ev < evMin) {
      poseMin = newPose;
      evMin = ev;
    }
  }

  useNum = pda->curLps.size();
  estPose = poseMin;

  return evMin;
}

void PoseEstimator::setCurScan(const Scan2D &srcScan) { 
  curScan = srcScan;
}

void PoseEstimator::setRefScan(const Scan2D &refScan) { 
  pda->setRefBase(refScan.lps);
}

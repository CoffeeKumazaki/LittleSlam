#pragma once

class PoseOptimizer;
class DataAssociator;

class PoseEstimator {

public:
  PoseEstimator();
  ~PoseEstimator();

  double estimatePose(const Pose2D& initPose, Pose2D& estPose);
  void setCurScan(const Scan2D &srcScan);
  void setRefScan(const Scan2D &refScan);

private:
  Scan2D curScan;
  int useNum;
  double pnrate;

  std::shared_ptr<PoseOptimizer> ppo;
  std::shared_ptr<DataAssociator> pda;
};
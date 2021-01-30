#pragma once

class ScanMatcher {

public:
  ScanMatcher(int cnt = -1, double score_max = 1.0, int num_min = 50);
  ~ScanMatcher();

  bool matchScan(const Scan2D& srcScan);
  void growMap(const Scan2D& srcScan, const Pose2D& pose);

private:
  Scan2D prevScan;
  Pose2D curPose;

  int cnt;
  double score_max;
  int num_min;
};
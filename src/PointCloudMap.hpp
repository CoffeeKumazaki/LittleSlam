#pragma once

class PointCloudMap {

public:
  static PointCloudMap& getInstance() {
    static PointCloudMap instance;
    return instance;
  }

private:
  PointCloudMap();
  ~PointCloudMap();

public:
  void addPose(Pose2D pose);
  void addPoints(const std::vector<LPoint2D> &points, int step = 5);
  Pose2D getLastPose();
  void setLastScan(const Scan2D &scan) { lastScan = scan; }

public:
  std::vector<Pose2D> poses;
  std::vector<LPoint2D> globalMap;
  Scan2D lastScan;
};

#define GetPCM() PointCloudMap::getInstance()
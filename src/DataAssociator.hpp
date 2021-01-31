#pragma once

class DataAssociator {

public:
  DataAssociator();
  ~DataAssociator();

  void setRefBase(const std::vector<LPoint2D> &lps) {
    mapLps.assign(lps.begin(), lps.end());
  }

  double findCorrespondence(const Scan2D &curScan, const Pose2D &predScan);

public:
  std::vector<LPoint2D> curLps;
  std::vector<LPoint2D> refLps;

private:
  std::vector<LPoint2D> mapLps;

};
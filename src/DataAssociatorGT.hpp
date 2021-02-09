#pragma once
#include "DataAssociator.hpp"
#include "GridTable.hpp"

class DataAssociatorGT : public DataAssociator {

public:
  DataAssociatorGT();
  ~DataAssociatorGT();

  void setRefBase(const std::vector<LPoint2D> &lps);
  double findCorrespondence(const Scan2D &curScan, const Pose2D &predScan);

public:
  GridTable gt;

};
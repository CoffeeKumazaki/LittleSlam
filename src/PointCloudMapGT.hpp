#include "PointCloudMap.hpp"
#include "GridTable.hpp"

class PointCloudMapGT : public PointCloudMap {

public:
  static PointCloudMapGT& getGTInstance() {
    static PointCloudMapGT instance;
    return instance;
  }

private:
  PointCloudMapGT();
  ~PointCloudMapGT();

public:
  void clear();
  void addPoints(const std::vector<LPoint2D> &points, int step = 5);
  void makeGlobalMap();
  
private:
  void makeSubsamplePoints(std::vector<LPoint2D>& lps);

private:
  std::vector<LPoint2D> allLps;
  GridTable gt;

};

#define GetPCMGT() PointCloudMapGT::getGTInstance()

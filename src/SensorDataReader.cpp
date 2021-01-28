#include <stdafx.hpp>
#include "SensorDataReader.hpp"

SensorDataReader::SensorDataReader() {
}

SensorDataReader::~SensorDataReader() {

}

bool SensorDataReader::init(std::string file_path) {
 
  f.open(file_path);
  if (!f.is_open()) {
    return false;
  }

  return true;
}

void SensorDataReader::term() {

  f.close();
}

bool SensorDataReader::loadData(int id, Scan2D& scan) {

  bool isScan = false;
  while (!f.eof() && !isScan) {
    isScan = loadLaserScan(id, scan);
  }

  return isScan;
}

bool SensorDataReader::loadLaserScan(int id, Scan2D& scan) {

  if (!f.is_open()) {
    return false;
  }

  std::string type;
  f >> type;

  if (type != "LASERSCAN") {
    std::string line;
    getline(f, line);
    return false;
  }

  int sid, sec, nsec;
  f >> sid >> sec >> nsec;

  int pnum;
  f >> pnum;

  std::vector<LPoint2D> lps;
  lps.reserve(pnum);
  for (size_t i = 0; i < pnum; i++) {
    double angle, dist;
    f >> angle >> dist;

    angle += angleOffset;

    if (dist <= Scan2D::MIN_SCAN_RANGE || dist >= Scan2D::MAX_SCAN_RANGE) {
      continue;
    }

    LPoint2D lp;
    lp.sid = id;
    lp.calcPos(dist, angle);
    lps.emplace_back(lp);
  }

  scan.sid = id;
  double angle;
  f >> scan.pose.x >> scan.pose.y >> angle;
  scan.pose.angle = RAD2DEG(angle);
  scan.pose.calcRmat();

  scan.lps.assign(lps.begin(), lps.end());

  return true;
}
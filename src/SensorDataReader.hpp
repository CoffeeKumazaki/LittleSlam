#pragma once

#include <iostream>
class SensorDataReader {

public:
  SensorDataReader();
  ~SensorDataReader();

  bool init(std::string file_path);
  void term();
  bool loadData(int id, Scan2D& scan);

private:
  bool loadLaserScan(int id, Scan2D& scan);

private:
  std::ifstream f;

  double angleOffset = 180;
};

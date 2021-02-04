#pragma once

class ScanPointResampler {

public:
  ScanPointResampler();
  ~ScanPointResampler();

  void resamplePoints(const std::vector<LPoint2D>& org, std::vector<LPoint2D>& res);

private:
  double interval;
  double interval_max;
};
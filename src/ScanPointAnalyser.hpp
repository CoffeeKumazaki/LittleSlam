#pragma once

class ScanPointAnalyser {

enum DIR {
  LEFT = -1,
  RIGHT = 1,
};

public:
  ScanPointAnalyser();
  ~ScanPointAnalyser();

  void analysePoints(std::vector<LPoint2D>& lps);
  bool calcNorm(std::vector<LPoint2D>& lps, int idx, DIR dir, Vector2D& n);

private:
  

};
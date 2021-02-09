#pragma once

struct GridCell {
  std::vector<LPoint2D> lps;
};

class GridTable {

public:
  GridTable(double csize = 0.5, double rsize = 40.0);
  ~GridTable();

  void clear();
  bool addPoint(const LPoint2D& p);
  void makeCellPoints(int nthre, std::vector<LPoint2D>& res);
  bool findClosestPoint(const LPoint2D& trg, LPoint2D& res, double thre = 0.2);

public:
  double csize;
  double rsize;
  int tsize;
  std::vector<GridCell> table;

};
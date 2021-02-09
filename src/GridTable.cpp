#include <stdafx.hpp>
#include "GridTable.hpp"

GridTable::GridTable(double _csize /*= 0.5*/, double _rsize /*= 40.0*/)
 : csize(_csize)
 , rsize(_rsize)
{
  clear();
}

GridTable::~GridTable() {

}

void GridTable::clear() {
  table.clear();
  tsize = static_cast<int>(rsize/csize);
  size_t w = static_cast<int>(2*tsize+1);
  table.resize(w*w);
}

bool GridTable::addPoint(const LPoint2D& p) {
  int xi = static_cast<int>((p.x + rsize)/csize);
  if (xi < 0 || xi > 2*tsize) {
    return false;
  }

  int yi = static_cast<int>((p.y + rsize)/csize);
  if (yi < 0 || yi > 2*tsize) {
    return false;
  }

  size_t idx = static_cast<size_t>(yi*(2*tsize + 1) + xi);
  table[idx].lps.emplace_back(p);
  return true;
}

void GridTable::makeCellPoints(int nthre, std::vector<LPoint2D>& res) {

  for(size_t i = 0; i < table.size(); i++) {
    if (table[i].lps.size() < nthre) continue;

    double x = 0;
    double nx = 0;
    double y = 0;
    double ny = 0;
    int sid = 0;
    for(size_t j = 0; j < table[i].lps.size(); j++) {
      const auto lp = table[i].lps[j];
      x += lp.x;
      y += lp.y;
      nx += lp.nx;
      ny += lp.ny;
      sid = std::max(sid, lp.sid);
    }

    x /= table[i].lps.size();
    y /= table[i].lps.size();
    double L = sqrt(nx*nx + ny*ny);
    nx /= L;
    ny /= L;
    
    LPoint2D newp;
    newp.x = x;
    newp.y = y;
    newp.sid = sid;
    newp.nx = nx;
    newp.ny = ny;
    newp.type = LINE;
    res.emplace_back(newp);
  }
}
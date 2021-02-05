#include <stdafx.hpp>
#include "ScanPointAnalyser.hpp"

ScanPointAnalyser::ScanPointAnalyser() {

}

ScanPointAnalyser::~ScanPointAnalyser() {

}

void ScanPointAnalyser::analysePoints(std::vector<LPoint2D>& lps) {

  for(size_t i = 0; i < lps.size(); i++) {
    // calc norm.
    Vector2D ln, rn;
    bool l = calcNorm(lps, i, LEFT, ln);
    bool r = calcNorm(lps, i, RIGHT, rn);
    rn.x *= -1;
    rn.y *= -1;

    // set type.
    ptype type;
    Vector2D n;
    if (l & r) {
      
      if (fabs(ln.x*rn.x+ln.y*rn.y) >= cos(M_PI/4.0)) {
        type = LINE;
      }
      else {
        type = CORNER;
      }
      double dx = ln.x+rn.x;
      double dy = ln.y+rn.y;
      double d = sqrt(dx*dx+dy*dy);
      n.x = dx/d;
      n.y = dy/d;
    }
    else if (r) {
      type = LINE;
      n = rn;
    }
    else if (l) {
      type = LINE;
      n = ln;
    }
    else {
      type = ISOLATE;
      n.x = __DBL_MAX__;
      n.y = __DBL_MAX__;
    }

    lps[i].type = type;
    lps[i].nx = n.x;
    lps[i].ny = n.y;

  }
}

bool ScanPointAnalyser::calcNorm(std::vector<LPoint2D>& lps, int idx, DIR dir, Vector2D& n) {

  double max = 1.0;
  double min = 0.06;
  for(size_t i = idx+dir; i>=0 && i < lps.size(); i+=dir) {

    double dx = lps[i].x - lps[idx].x;
    double dy = lps[i].y - lps[idx].y;
    double d = sqrt(dx*dx+dy*dy);

    if (d <= max && d >= min ) {
      n.x = dy/d;
      n.y = -dx/d;
      return true;
    }

    if (d > max) {
      break;
    }
  }

  return false;
}
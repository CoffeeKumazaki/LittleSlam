#include <stdafx.hpp>
#include "ScanPointResampler.hpp"

ScanPointResampler::ScanPointResampler()
 : interval(0.2)
 , interval_max(0.25)
{
  
}

ScanPointResampler::~ScanPointResampler() {
  
}

void ScanPointResampler::resamplePoints(const std::vector<LPoint2D>& org, std::vector<LPoint2D>& res) {

  res.clear();
  if (org.empty()) {
    return;
  }
  double dist;

  auto prev = org[0];
  res.emplace_back(prev);
  for (size_t i = 1; i < org.size(); i++) {
    double dx = org[i].x - prev.x;
    double dy = org[i].y - prev.y;
    double d = sqrt(dx * dx + dy * dy);
    LPoint2D nd;

    if (d + dist < interval)
    {
      dist += d;
      prev = org[i];
      continue;
    }
    else if (d+dist < interval_max) {
      double ratio = (interval - dist) / d;
      double nx = prev.x + ratio * dx;
      double ny = prev.y + ratio * dy;
      nd.sid = prev.sid;
      nd.x = nx;
      nd.y = ny;
      i--;
    }
    else {
      nd.sid = org[i].sid;
      nd.x = org[i].x;
      nd.y = org[i].y;
    }
    dist = 0;
    prev = nd;
    res.emplace_back(nd);
  }

}

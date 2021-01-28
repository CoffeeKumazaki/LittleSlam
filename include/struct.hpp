#pragma once

struct Pose2D {
  double x     = 0.0;
  double y     = 0.0;
  double angle = 0.0;
  double Rmat[2][2] = {{0, 0}, {0, 0}};

  void calcRmat() {
    double a = DEG2RAD(angle);
    Rmat[0][0] = Rmat[1][1] = cos(a);
    Rmat[1][0] = sin(a);
    Rmat[0][1] = -Rmat[1][0];
  }};

enum ptype
{
  UNKNOWN = 0,
  LINE,
  CORNER,
  ISOLATE,
};

struct LPoint2D {

  int sid = -1;
  double x = 0.0;
  double y = 0.0;
  double nx = 0.0;
  double ny = 0.0;

  double acc_dist = 0.0;
  ptype type = UNKNOWN;

  void calcPos(double dist, double angle) {
    double deg = DEG2RAD(angle);
    x = dist*std::cos(deg);
    y = dist*std::sin(deg);    
  }


};

struct Scan2D {

  static double MAX_SCAN_RANGE;
  static double MIN_SCAN_RANGE;

  int sid;
  Pose2D pose;
  std::vector<LPoint2D> lps;

  Scan2D() : sid(0) {
  }

  ~Scan2D() {
  }

  void setPose(const Pose2D& _pose) {
    pose = _pose;
  }

};


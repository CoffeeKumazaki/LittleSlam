#pragma once


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

  LPoint2D() {
    LPoint2D(01, 0, 0);
  }
  LPoint2D(int _sid, double _x, double _y) {
    sid = _sid; x = _x;
    y = _y;
  }

  void calcPos(double dist, double angle) {
    double deg = DEG2RAD(angle);
    x = dist*std::cos(deg);
    y = dist*std::sin(deg);    
  }

};

struct Pose2D {
  double x     = 0.0;
  double y     = 0.0;
  double angle = 0.0;
  double Rmat[2][2] = {{0, 0}, {0, 0}};

  void calcRmat() {
    double a = DEG2RAD(angle)+M_PI;
    Rmat[0][0] = Rmat[1][1] = cos(a);
    Rmat[1][0] = sin(a);
    Rmat[0][1] = -Rmat[1][0];
  }

  void getGlobalPoint(const LPoint2D& in, LPoint2D& out) const {
    out.x = Rmat[0][0]*in.x + Rmat[0][1]*in.y + x;
    out.y = Rmat[1][0]*in.x + Rmat[1][1]*in.y + y;
  }

  static void calcRelativePose(const Pose2D& src, const Pose2D& trg, Pose2D& res) {
    
    double dx = trg.x - src.x;
    double dy = trg.y - src.y;
    res.x = trg.Rmat[0][0] * dx + trg.Rmat[0][1] * dy;
    res.y = trg.Rmat[1][0] * dx + trg.Rmat[1][1] * dy;

    res.angle = trg.angle - src.angle;
    if (res.angle < -180)
      res.angle += 360;
    else if (res.angle >= 180)
      res.angle -= 360;

    res.calcRmat();
  }

  static void calcPredictionPose(const Pose2D& base, const Pose2D& motion, Pose2D& res) {

    res.x = base.Rmat[0][0] * motion.x + base.Rmat[0][1] * motion.y + base.x;
    res.y = base.Rmat[1][0] * motion.x + base.Rmat[1][1] * motion.y + base.y;

    res.angle = base.angle + motion.angle;
    if (res.angle < -180)
      res.angle += 360;
    else if (res.angle >= 180)
      res.angle -= 360;

    res.calcRmat();

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


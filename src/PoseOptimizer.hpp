#pragma once

class PoseOptimizer {

public:
  PoseOptimizer(): evthre(0.001), dd(0.00001), da(0.00001){};
  ~PoseOptimizer(){};

  void setPoints(const std::vector<LPoint2D>& _curLps, const std::vector<LPoint2D>& _refLps) {
    curLps.assign(_curLps.begin(), _curLps.end());
    refLps.assign(_refLps.begin(), _refLps.end());
  }

  double evaluate(double x, double y, double angle) {

    double evLimit = 0.2;
    double a = DEG2RAD(angle);
    double error = 0;

    int cnt = 0;

    for (size_t i = 0; i < curLps.size(); i++) {
      const auto cur = curLps[i];
      const auto ref = refLps[i];

      double sx = cos(a) * cur.x - sin(a) * cur.y + x;
      double sy = sin(a) * cur.x + sin(a) * cur.y + y;

      double dist = (sx - ref.x) * (sx - ref.x) + (sy - ref.y) * (sy - ref.y);

      if (dist < evLimit*evLimit) {
        cnt++;
      }

      error += dist;
    }

    error = curLps.size() > 0 ? error*100.0 / curLps.size() : __DBL_MAX__;

    return error;
  }

  double OptimizePose(const Pose2D& initPose, Pose2D& estPose) {

    double x = initPose.x;
    double y = initPose.y;
    double a = initPose.angle;
    double xmin = x;
    double ymin = y;
    double amin = a;

    double evMin = __DBL_MAX__;
    double prevEv = evMin;

    double ev = evaluate(initPose.x, initPose.y, initPose.angle);

    double step=0.00001;
    while (abs(prevEv-ev) > evthre) {
      prevEv = ev;

      double dEx = (evaluate(x+dd, y, a) - ev)/dd;
      double dEy = (evaluate(x, y+dd, a) - ev)/dd;
      double dEa = (evaluate(x, y, a+da) - ev)/da;

      double dx = -step*dEx;
      double dy = -step*dEy;
      double da = -step*dEa;
      x += dx;  y += dy;  a += da;

      ev = evaluate(x, y, a);

      if (ev < evMin)
      {
        evMin = ev;
        xmin = x;  ymin = y;  amin = a;
      }
    }

    estPose.x = xmin;
    estPose.y = ymin;
    estPose.angle = amin;
    // estPose.calcRmat();

    return evMin;
  }

private:
  double evthre;
  double dd;
  double da;

  std::vector<LPoint2D> curLps;
  std::vector<LPoint2D> refLps;
};
#include <stdafx.hpp>
#include <PoseOptimizer.hpp>

int main(int argc, char const *argv[]) {

  PoseOptimizer po;

  // y = 2*x**2
  std::vector<LPoint2D> cur = {
    LPoint2D(0, 0, 0),
    LPoint2D(1, 1, 2), 
    LPoint2D(2, 2, 8),
    LPoint2D(3, 3, 18),
  };
  double stepx = 2.0;
  double stepy = 2.0;
  std::vector<LPoint2D> ref = {
    LPoint2D(0, 0+stepx, 0+stepy),
    LPoint2D(1, 1+stepx, 1+stepy), 
    LPoint2D(2, 2+stepx, 8+stepy),
    LPoint2D(3, 3+stepx, 18+stepy),
  };

  po.setPoints(cur, ref);
  Pose2D est;
  Pose2D init;
  init.x = 1;
  init.y = 0;
  init.angle = 0.5;
  po.OptimizePose(init, est);

  std::cout << "est " << std::endl;
  std::cout << "x " << est.x << std::endl;
  std::cout << "y " << est.y << std::endl;
  std::cout << "a " << est.angle << std::endl;
}
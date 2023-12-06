#pragma once
#include <vector>

#include "mujoco/mujoco.h"

namespace mnplt {
class Controller {
 private:
  double Kp_, Ki_, Kd_;
  mjModel *m_;
  mjData *d_;
  std::vector<int> actuator_joint_id;

 public:
  void init(mjModel *m, mjData *d);
  void control(const mjModel *m, mjData *d);
};
};  // namespace mnplt
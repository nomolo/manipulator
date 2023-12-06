#include "controller/controller.h"

#include <cmath>

void mnplt::Controller::control(const mjModel *m, mjData *d) {
  d->ctrl[0] = 15.0 * (0.0 - d->qpos[7]) + 15.0 * (-d->qvel[6]);
  d->ctrl[1] = 15.0 * (0.0 - d->qpos[8]) + 15.0 * (-d->qvel[7]);
  d->ctrl[2] = 15.0 * (-1.0 - d->qpos[9]) + 15.0 * (-d->qvel[8]);
  d->ctrl[3] = 15.0 * (0.0 - d->qpos[10]) + 15.0 * (-d->qvel[9]);
}

void mnplt::Controller::init(mjModel *m, mjData *d) {
  m_ = m;
  d_ = d;
}
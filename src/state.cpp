#include "state.h"

State::State(float x, float y, float theta) {
  this->x = x;
  this->y = y;
  this->theta = theta;

  this->gx = x / Grid_Res;
  this->gy = y / Grid_Res;
  this->gtheta = theta + 0.01;
}

State::State() {
  this->x = -1;
  this->y = -1;
  this->theta = -1;
}

/************************************************
 * Compute the possible next moves according to
 * the kinematic bicycle model constraints.
 ***********************************************/
std::vector<State> State::getNextStates() {
  std::vector<State> next;
  State n;

  // Bicycle model parameters
  float alpha, beta, r, d = 10;

  // Try out possible values of alpha
  // among 3 angles: [-BOT_M_ALPHA, 0, -BOT_M_ALPHA].
  for (alpha = -BOT_M_ALPHA; alpha <= BOT_M_ALPHA + 0.001;
       alpha += BOT_M_ALPHA) {
    beta = d * tan(alpha * PI / 180) / BOT_L;

    if (abs(beta) < 0.001) { // Forward
      n.x = x + d * cos(theta * 2.0 * PI / Theta);
      n.y = y + d * sin(theta * 2.0 * PI / Theta);
      n.theta = theta;
    } else { // Turning
      r = BOT_L / tan(alpha * PI / 180);
      n.x = x + r * sin(theta * 2.0 * PI / Theta + beta) -
            r * sin(theta * 2.0 * PI / Theta);
      n.y = y - r * cos(theta * 2.0 * PI / Theta + beta) +
            r * cos(theta * 2.0 * PI / Theta);
      if (theta + beta * 180 / PI / Theta_Res > 0) {
        n.theta = fmod(theta + beta * 180 / PI / Theta_Res, Theta);
      }
      else {
        n.theta = theta + beta * 180 / PI / Theta_Res + Theta;
      }
    }

    // Convert to grid-map coordinates.
    n.gx = n.x / Grid_Res;
    n.gy = n.y / Grid_Res;
    n.gtheta = n.theta + 0.01;

    next.push_back(n);
  }

  // std::cout << "getNextStates() called from " << x << "," << y << "," << theta
  //      << std::endl;

  // for (int i = 0; i < 3; i++) {
  //   std::cout << next[i].x << "," << next[i].y << "," << next[i].theta << "" << std::endl;
  // }

  return next;
}

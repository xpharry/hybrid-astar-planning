#include "state.h"

/**
 * Constructor
 */
State::State() {
  this->x = -1;
  this->y = -1;
  this->theta = -1;
}

/**
 * Constructor
 */
State::State(double x, double y, double theta) {
  // Initilize 3d state on the geometric map
  this->x = x;
  this->y = y;
  this->theta = theta;

  // Initialize 2d state on the grid map
  this->gx = x / Grid_Res;
  this->gy = y / Grid_Res;
  this->gtheta = theta + 0.01;
}



/**
 * Compute the possible next moves according to
 * the kinematic bicycle model constraints.
 *
 * Kinematic Bicycle Model centered the rear wheel.
 *
 */
std::vector<State> State::getNextStates() {
  std::vector<State> next;
  State state;

  // Bicycle model parameters
  double alpha = 0;
  double d_theta = 0;
  double radius = 0; // rotation radius
  double dist = 30.0; // distance travelled within a unit time

  // Try out possible values of alpha
  // among 3 angles: [-VEH_M_ALPHA, 0, VEH_M_ALPHA].
  for (alpha = -VEH_M_ALPHA; alpha <= VEH_M_ALPHA + 0.001;
       alpha += VEH_M_ALPHA) {

    d_theta = dist * tan(alpha * PI / 180) / VEH_LEN;

    if (abs(d_theta) < 0.001) { // Forward
      state.x = x + dist * cos(theta * 2.0 * PI / Theta);
      state.y = y + dist * sin(theta * 2.0 * PI / Theta);
      state.theta = theta;
    } else { // Turning
      // Bicycle model centered at the rear wheel
      radius = VEH_LEN / tan(alpha * PI / 180);
      state.x = x + radius * sin(theta * 2.0 * PI / Theta + d_theta)
                  - radius * sin(theta * 2.0 * PI / Theta);
      state.y = y - radius * cos(theta * 2.0 * PI / Theta + d_theta)
                  + radius * cos(theta * 2.0 * PI / Theta);

      // new theta = theta + d_theta
      double new_theta = theta + d_theta * 180 / PI / Theta_Res;

      // normalize
      if (new_theta > 0) {
        state.theta = fmod(new_theta, Theta);
      }
      else {
        state.theta = new_theta + Theta;
      }
    }

    // Convert to grid-map coordinates.
    state.gx = state.x / Grid_Res;
    state.gy = state.y / Grid_Res;
    state.gtheta = state.theta + 0.01;

    next.push_back(state);
  }

  // std::cout << "getNextStates() called from "
  //           << x << ", " << y << ", " << theta
  //           << std::endl;

  // for (int i = 0; i < 3; i++) {
  //   std::cout << next[i].x << ", " << next[i].y << ", " << next[i].theta << std::endl;
  // }

  return next;
}

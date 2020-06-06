#ifndef STATE_H
#define STATE_H

#include "utils.h"

class State {
public: // methods
  /**
   * Constructor
   */
  State();

  /**
   * Constructor
   */
  State(double x, double y, double theta);

  /**
   * get the states of the next move
   *
   */
  std::vector<State> getNextStates();

public: // variables
  // 3d state on the geometric map
  double x;
  double y;
  double theta;

  // gx, gy and gtheta are co-ordinates in the 80X80 grid
  int gx;
  int gy;
  int gtheta;

  // for 2d planning
  int dx;
  int dy;

  double cost2d;
  double cost3d;

  double change;
  double velocity;

  State *prev;
  State *next;
};

#endif // STATE_H

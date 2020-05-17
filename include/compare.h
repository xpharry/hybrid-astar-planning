#ifndef COMPARE_H
#define COMPARE_H

#include "state.h"
#include "utils.h"

class Compare {
 public:
  static State target;
  static int **obs_map;
  static int **grid_obs_map;
  static float **shortest_2d;

  bool operator()(const State s1, const State s2);
  float non_holonomic_without_obs(State src);
  float holonomic_with_obs(State src);
  void runDijkstra();
};

#endif  // COMPARE_HPP

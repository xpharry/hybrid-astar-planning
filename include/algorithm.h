#ifndef ALGORITHM_H
#define ALGORITHM_H

#include "utils.h"
#include "map.h"
#include "state.h"
#include "compare.h"
#include "gui.h"

class Algorithm {
public:

  /**
   * Constructor
   */
  Algorithm(Map map);

  void updateInitial(State initial);

  void updateGoal(State goal);

  // heuristic functions
  float non_holonomic_without_obs(State src);
  float holonomic_with_obs(State src);

  // A* Search on the 2d grid map
  void astar_planning();

  // Hybrid A* Planning on the 3d map, [x, y, theta]
  void hybrid_astar_planning();

public:
  Map m_map;
  static State initial;
  static State goal;
  static int **obs_map;
  static int **grid_obs_map;
  static float **shortest_2d;
};

#endif  // ALGORITHM_H

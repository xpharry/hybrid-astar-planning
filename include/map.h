#ifndef MAP_H
#define MAP_H

#include "state.h"
#include "utils.h"

struct node {
  int x;
  int y;
  int nearest_obstacle;
};

class Map {
 public:
  int **obs_map;
  int **acc_obs_map;
  int **nearest_obstacle;
  int obs_dist_max;

  Map();
  void initCollisionChecker();
  bool checkCollision(State pos);
  void find_near_obs();
  int nearest_obstacle_distance(State pos);
  bool is_boundary_obstacle(int i, int j);
};

#endif  // MAP_H

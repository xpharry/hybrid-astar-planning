#ifndef MAP_H
#define MAP_H

#include "state.h"
#include "utils.h"

struct Node {
  int x;
  int y;
  int nearest_obstacle;
};

class Map {
public:
  /**
   * Constructor
   *
   */
  Map() {}

  /**
   * Constructor
   *
   */
  Map(std::string map_file);

  void initCollisionChecker();

  bool checkCollision(State pos);

  void findNearObs();

  int nearestObstacleDistance(State pos);

  bool isBoundaryObstacle(int i, int j);

public:
  int **obs_map;
  int **acc_obs_map;
  int **nearest_obstacle;
  int obs_dist_max;
};

#endif // MAP_H

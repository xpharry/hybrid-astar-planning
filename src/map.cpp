#include "map.h"

/*
 * Constructor
 */
Map::Map(std::string map_file) {
  // Initilize the geometric map.
  obs_map = new int *[MAPX];
  for (int i = 0; i < MAPX; i++) {
    obs_map[i] = new int[MAPY];
    for (int j = 0; j < MAPY; j++) {
      obs_map[i][j] = 0;
    }
  }

  // Load the map using OpenCV as a gray image.
  cv::Mat obsmap = cv::imread(map_file, 0);

  if (obsmap.empty()) {
    std::cout << "Error: Map is empty!" << std::endl;
  }

  // Fill the obstalces.
  for (int i = 0; i < MAPX; i++) {
    for (int j = 0; j < MAPY; j++) {
      if (obsmap.at<uchar>(j / 2, i / 2) <= 120) {
        obs_map[i][j] = 1;
      } else {
        obs_map[i][j] = 0;
      }
    }
  }

  std::cout << "Cost Map Initialized!" << std::endl;
}

/**
 * Collision Checker.
 *
 */
void Map::initCollisionChecker() {
  acc_obs_map = new int *[MAPX];
  for (int i = 0; i < MAPX; i++) {
    acc_obs_map[i] = new int[MAPY];
    for (int j = 0; j < MAPY; j++) acc_obs_map[i][j] = obs_map[i][j];
  }

  for (int i = 0; i < MAPX; i++)
    for (int j = 1; j < MAPY; j++)
      acc_obs_map[i][j] = acc_obs_map[i][j - 1] + acc_obs_map[i][j];

  for (int j = 0; j < MAPY; j++)
    for (int i = 1; i < MAPX; i++)
      acc_obs_map[i][j] = acc_obs_map[i - 1][j] + acc_obs_map[i][j];

  std::cout << "Collision checker initialized!" << std::endl;

  return;
}

/**
 * Collision Checker.
 *
 */
bool Map::checkCollision(State pos) {
  // std::cout << "Collision checking: " << std::endl;
  // std::cout << pos.x << "," << pos.y << "," << pos.theta << std::endl;
  // std::cout << pos.gx << "," << pos.gy << "," << pos.gtheta << std::endl;

  if (pos.x >= MAPX || pos.x < 0 || pos.y >= MAPY || pos.y < 0 ||
      pos.theta >= Theta || pos.theta < 0)
    return true;

  if (pos.gx >= GX || pos.gx < 0 || pos.gy >= GY || pos.gy < 0 ||
      pos.gtheta >= Theta || pos.gtheta < 0)
    return true;

  // std::cout << "Out of Bounds" << std::endl;

  // first use a bounding box around car to check for collision in O(1) time
  int max_x, min_x, max_y, min_y;
  max_x = pos.x + VEH_LEN * abs(cos(pos.theta * 2 * PI / Theta)) / 2
                + VEH_WID * abs(sin(pos.theta * 2 * PI / Theta)) / 2 + 1;
  min_x = pos.x - VEH_LEN * abs(cos(pos.theta * 2 * PI / Theta)) / 2
                - VEH_WID * abs(sin(pos.theta * 2 * PI / Theta)) / 2 - 1;
  max_y = pos.y + VEH_LEN * abs(sin(pos.theta * 2 * PI / Theta)) / 2
                + VEH_WID * abs(cos(pos.theta * 2 * PI / Theta)) / 2 + 1;
  min_y = pos.y - VEH_LEN * abs(sin(pos.theta * 2 * PI / Theta)) / 2
                - VEH_WID * abs(cos(pos.theta * 2 * PI / Theta)) / 2 - 1;

  if (max_x >= MAPX || min_x < 0 || max_y >= MAPY || min_y < 0) return true;

  if (acc_obs_map[max_x][max_y] + acc_obs_map[min_x][min_y] ==
      acc_obs_map[max_x][min_y] + acc_obs_map[min_x][max_y]) {
    return false;
  }

  std::cout << "Obstacle present inside box" << std::endl;

  // brute force check through the car

  for (double i = -VEH_LEN / 2.0; i <= VEH_LEN / 2.0 + 0.001; i += 1) {
    for (double j = -VEH_WID / 2.0; j <= VEH_WID / 2.0 + 0.001; j += 1) {
      int s = pos.x + i * cos(pos.theta * 2.0 * PI / Theta) +
              j * sin(pos.theta * 2.0 * PI / Theta) + 0.001;
      int t = pos.y + i * sin(pos.theta * 2.0 * PI / Theta) +
              j * cos(pos.theta * 2.0 * PI / Theta) + 0.001;
      if (obs_map[s][t] || obs_map[s + 1][t] || obs_map[s][t + 1] ||
          obs_map[s + 1][t + 1] || obs_map[s - 1][t - 1] || obs_map[s - 1][t] ||
          obs_map[s][t - 1] || obs_map[s - 1][t + 1] || obs_map[s + 1][t - 1]) {
        return true;
      }
    }
  }
  return false;
}

/**
 * Check if the input coordinate is at the boundary of the obstacles.
 *
 * Check in four directions.
 *
 * @return true or false.
 */
bool Map::isBoundaryObstacle(int i, int j) {
  for (int k = i - 1; k <= i + 1; k++) {
    for (int l = j - 1; l <= j + 1; l++) {
      if (!(k >= 0 && k < MAPX && l >= 0 && l < MAPY)) continue;
      if (obs_map[k][l] == 0) {
        return true;
      }
    }
  }
  return false;
}

/**
 * Find nearest obstacles.
 *
 * Use BFS Search.
 *
 */
void Map::findNearObs() {
  Node node_p, node_c;

  nearest_obstacle = new int *[MAPX];

  for (int i = 0; i < MAPX; i++) {
    nearest_obstacle[i] = new int[MAPY];
    for (int j = 0; j < MAPY; j++) {
      nearest_obstacle[i][j] = 0;
    }
  }

  std::queue<Node> q;

  for (int i = 0; i < MAPX; i++) {
    for (int j = 0; j < MAPY; j++) {
      if (obs_map[i][j] == 1) {
        if (!isBoundaryObstacle(i, j)) {
          nearest_obstacle[i][j] = -1;
        } else {
          node_p.x = i;
          node_p.y = j;
          node_p.nearest_obstacle = 1;
          q.push(node_p);
          nearest_obstacle[i][j] = 1;
        }
      }
    }
  }

  while (!q.empty()) {
    node_p = q.front();
    q.pop();

    // std::cout << "nearest_obstacle size" << nearest_obstacle.size()
    //           << nearest_obstacle[0].size() << std::endl;
    // std::cout << "obs_map size" << obs_map.size() << obs_map[0].size() << std::endl;

    for (int i = node_p.x - 1; i <= node_p.x + 1; i++) {
      for (int j = node_p.y - 1; j <= node_p.y + 1; j++) {
        if (i >= 0 && i < MAPX && j >= 0 && j < MAPY &&
            nearest_obstacle[i][j] == 0 && obs_map[i][j] == 0) {
          node_c.x = i;
          node_c.y = j;
          node_c.nearest_obstacle = node_p.nearest_obstacle + 1;
          nearest_obstacle[i][j] = node_c.nearest_obstacle;
          q.push(node_c);
        }
      }
    }
  }

  obs_dist_max = node_p.nearest_obstacle;
}

/**
 * Measure the distance to the nearest obstacle.
 *
 */
int Map::nearestObstacleDistance(State pos) {
  return nearest_obstacle[(int)pos.x][(int)pos.y];
}

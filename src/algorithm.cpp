#include "algorithm.h"

////////////////////////////////////////////////////////////////////////////////
// Declaration of variables
////////////////////////////////////////////////////////////////////////////////

State Algorithm::initial;

State Algorithm::goal;

int **Algorithm::obs_map;

double **Algorithm::shortest_2d;

int **Algorithm::grid_obs_map;

State previous[GX][GY][Theta];

/**
 * 2D Point type
 *
 */
struct Point {
  double x;
  double y;
};

/**
 * 2D Quadrant Locator
 *
 * @param a Starting Point
 * @param b Ending Point
 *
 * @return Quadrant index
 *
 */
int quad(Point a, Point b) {
  if (b.x > a.x && b.y > a.y)
    return 1;
  else if (b.x < a.x && b.y > a.y)
    return 2;
  else if (b.x < a.x && b.y < a.y)
    return 3;
  else if (b.x > a.x && b.y < a.y)
    return 4;
  return 0;  // won't be used
}

/**
 * 2d coordinate comparator
 *
 * Will be used in the heap.
 */
struct Compare2d {
  bool operator()(const State &a, const State &b) {
    // return a.cost2d > b.cost2d;	//simple dijkstra
    return a.cost2d + abs(Algorithm::goal.dx - a.dx) + abs(Algorithm::goal.dy - a.dy) >
           b.cost2d + abs(Algorithm::goal.dx - b.dx) + abs(Algorithm::goal.dy - b.dy);
  }
};

/**
 * 3d coordinate comparator
 *
 */
struct Compare3d {
 bool operator()(const State &s1, const State &s2);
};

bool Compare3d::operator()(const State &s1, const State &s2) {
  // return s1.cost3d > s2.cost3d;
  // return s1.cost3d + Algorithm::holonomicWithObs(s1) >
  //        s2.cost3d + Algorithm::holonomicWithObs(s2);
  return s1.cost3d + fmax(Algorithm::holonomicWithObs(s1), Algorithm::nonHolonomicWithoutObs(s1)) >
         s2.cost3d + fmax(Algorithm::holonomicWithObs(s2), Algorithm::nonHolonomicWithoutObs(s2));
}

////////////////////////////////////////////////////////////////////////////////
// Algorithm class
////////////////////////////////////////////////////////////////////////////////

/**
 * Constructor
 *
 */
Algorithm::Algorithm(Map map) {
  m_map = map;

  // Initialize a geometric map and a grid map.
  obs_map = map.obs_map;
  grid_obs_map = new int *[DX];

  // Intialize each grid value as 0.
  for (int i = 0; i < DX; i++) {
    grid_obs_map[i] = new int[DY];
    for (int j = 0; j < DY; j++) {
      grid_obs_map[i][j] = 0;
    }
  }

  // Contruct occupied areas in grid_obs_map by obs_map.
  for (int i = 0; i < MAPX; i++) {
    for (int j = 0; j < MAPY; j++) {
      if (obs_map[i][j])
        grid_obs_map[i * DX / MAPX][j * DY / MAPY] = 1;
    }
  }
}

void Algorithm::updateInitial(State initial) {
  Algorithm::initial = initial;
}

void Algorithm::updateGoal(State goal) {
  Algorithm::goal = goal;
}


/**
 * A* Search on the 2d grid map
 *
 * Currently implemented as Dijkstra's Algorithm.
 *
 * Dynamic Programming.
 *
 */
void Algorithm::astarPlanning() {
  State src = Algorithm::goal;
  src.dx = src.gx * DX / GX;
  src.dy = src.gy * DY / GY;

  std::priority_queue<State, std::vector<State>, Compare2d> frontier;

  int visited[DX][DY];
  memset(visited, 0, sizeof(int) * DX * DY);

  // Initialize cost array with the size of [DX, DY]
  double **cost = new double *[DX];
  for (int i = 0; i < DX; i++) {
    cost[i] = new double[DY];
    for (int j = 0; j < DY; j++) cost[i][j] = 0;
  }

  // Assign a big value (10000 here) to each element in cost array
  for (int i = 0; i < DX; i++) {
    for (int j = 0; j < DY; j++) {
      cost[i][j] = 10000;
    }
  }

  // Assign 0 cost to src position (No travel cost).
  cost[src.dx][src.dy] = 0;

  frontier.push(src);
  while (!frontier.empty()) {
    State current = frontier.top();
    frontier.pop();

    int x = current.dx;
    int y = current.dy;

    if (visited[x][y]) continue;

    visited[x][y] = 1;

    for (int i = -1; i <= 1; i++) {
      for (int j = -1; j <= 1; j++) {
        // Continue if going beyond the grid map
        if (x + i < 0 || x + i >= DX || y + j < 0 || y + j >= DY) continue;
        // Continue if it is [x, y] itself or obstalce.
        if ((i == 0 && j == 0) || Algorithm::grid_obs_map[x + i][y + j] != 0)
          continue;

        // Update cost table if find a lower cost path.
        if (cost[x + i][y + j] > cost[x][y] + sqrt(i * i + j * j)) {
          cost[x + i][y + j] = cost[x][y] + sqrt(i * i + j * j);
          State temp;
          temp.dx = current.dx + i;
          temp.dy = current.dy + j;
          temp.cost2d = cost[x + i][y + j];
          frontier.push(temp);
        }
      }
    }
  }

  // g cost lookup table
  Algorithm::shortest_2d = cost;

  // Display the g cost table.
  cv::Mat dist(240, 240, CV_8UC3, cv::Scalar(255, 255, 255));
  for (int i = 0; i < 240; i++) {
    for (int j = 0; j < 240; j++) {
      dist.at<cv::Vec3b>(j, i) = {
          (unsigned char)(255 - 0.6 * shortest_2d[i][j]),
          (unsigned char)(200 - 0.6 * shortest_2d[i][j]),
          (unsigned char)(200 - 0.6 * shortest_2d[i][j])};
    }
  }
  resize(dist, dist, cv::Size(400, 400));

  // uncomment to check if dijkstra ran properly
  // cv::imshow("dist", dist);
  // cv::waitKey(0);
}

/**
 * holonomic heuristic with obstacles
 *
 * currently uses dijkstra's algorithm in x-y space
 *
 */
double Algorithm::holonomicWithObs(State src) {
  return Algorithm::shortest_2d[(int)src.x * DX / MAPX][(int)src.y * DY / MAPY];
}

 /**
  * non-holonomic heuristic without obstacles
  *
  *
  */
double Algorithm::nonHolonomicWithoutObs(State src) {
  // return
  // abs(Algorithm::goal.x-src.x)+abs(Algorithm::goal.y-src.y)+abs(Algorithm::goal.theta-src.theta);

  // minimal radius which happens with the maximal steering angle
  double rmin = (VEH_LEN / tan((VEH_M_ALPHA)*PI / 180));

  Point CS, ACS, CE, ACE;

  // Clockwise-Start
  CS.x = src.dx + (rmin)*sin((src.theta) * PI / 180);
  CS.y = src.dy - (rmin)*cos((src.theta) * PI / 180);

  // Anti-Clockwise-Start
  ACS.x = src.dx - (rmin)*sin((src.theta) * PI / 180);
  ACS.y = src.dy + (rmin)*cos((src.theta) * PI / 180);

  // Clockwise-End
  CE.x = Algorithm::goal.dx + (rmin)*sin((Algorithm::goal.theta) * PI / 180);
  CE.y = Algorithm::goal.dy - (rmin)*cos((Algorithm::goal.theta) * PI / 180);

  // Anti-Clockwise-End
  ACE.x = Algorithm::goal.dx - (rmin)*sin((Algorithm::goal.theta) * PI / 180);
  ACE.y = Algorithm::goal.dy + (rmin)*cos((Algorithm::goal.theta) * PI / 180);

  // clockwise-clockwise length
  double lcc = sqrt((CS.x - CE.x) * (CS.x - CE.x) + (CS.y - CE.y) * (CS.y - CE.y));
  double laa = sqrt((ACS.x - ACE.x) * (ACS.x - ACE.x) + (ACS.y - ACE.y) * (ACS.y - ACE.y));
  double lca = sqrt((CS.x - ACE.x) * (CS.x - ACE.x) + (CS.y - ACE.y) * (CS.y - ACE.y));
  double lac = sqrt((ACS.x - CE.x) * (ACS.x - CE.x) + (ACS.y - CE.y) * (ACS.y - CE.y));

  Point A, B;

  // Code for CCC
  double pathCCC = INT_MAX; // As CCC is not always possible
  if (!(lcc > 4 * rmin && laa > 4 * rmin)) {
    int d;
    d = std::min(lcc, laa);
    int flag; // Flag=1 for clockwise being optimal and 0 for anticlockwise
    if (d == lcc) {
      flag = 1;
      A = CS;
      B = CE;
    } else {
      flag = 0;
      A = ACS;
      B = ACE;
    }

    double n;
    n = ((B.y - A.y) / (B.x - A.x));
    double t = atan(n);
    if (quad(A, B) == 2 || quad(A, B) == 3)
      t = PI + t;
    else if (quad(A, B) == 4)
      t = 2 * PI + t;
    n = d / (4 * rmin);
    double p = acos(n);

    double ccc[4];

    if (flag == 0) {  // anti-clockwise being optimal
      // Angle traversed in ACS for Path through Lower intermediate circle
      ccc[0] = (PI) / 2 - src.theta * PI / 180 + t - p;
      // Angle traversed in ACE for Path through Lower intermediate circle
      ccc[1] = (PI) / 2 + Algorithm::goal.theta * PI / 180 - t - p;
      // Angle traversed in ACS for Path through Upper intermediate circle
      ccc[2] = (PI) / 2 - src.theta * PI / 180 + t + p;
      // Angle traversed in ACE for Path through Upper intermediate circle
      ccc[3] = (PI) / 2 + Algorithm::goal.theta * PI / 180 - t + p;
    } else {  // clockwise being optimal
      // Angle traversed in CS for Path through Lower intermediate circle
      ccc[0] = (PI) / 2 + src.theta * PI / 180 - t + p;
      // Angle traversed in CE for Path through Lower intermediate circle
      ccc[1] = (PI) / 2 - Algorithm::goal.theta * PI / 180 + t + p;
      // Angle traversed in CS for Path through Upper intermediate circle
      ccc[2] = (PI) / 2 + src.theta * PI / 180 - t - p;
      // Angle traversed in CE for Path through Upper intermediate circle
      ccc[3] = (PI) / 2 - Algorithm::goal.theta * PI / 180 + t - p;
    }

    int i;
    for (i = 0; i < 4; i++) {
      if (ccc[i] < 0)
        ccc[i] = ccc[i] + 2 * PI;
      else if (ccc[i] >= 2 * (PI))
        ccc[i] = ccc[i] - 2 * PI;
    }

    double L, U;
    if (flag == 0) { // anticlockwise
      L = ccc[0] + ccc[1] + PI - 2 * p;
      U = ccc[2] + ccc[3] + PI + 2 * p;
    } else {         // clockwise
      L = ccc[0] + ccc[1] + PI + 2 * p;
      U = ccc[2] + ccc[3] + PI - 2 * p;
    }

    pathCCC = (rmin) * (std::min(L, U));
  }

  // Code for CSC
  double csc[8];
  double dis[4]; // Array to store final distance traversed in the 4 possible cases
  double theta;  // Angle made by line joining centres of two circular paths with x-axis
  double N;
  double ltct_ac = INT_MAX, ltct_ca = INT_MAX;

  A = CS;
  B = CE;  // For Direct Common Tangent -clockwise to clockwise
  N = ((B.y - A.y) / (B.x - A.x));
  theta = atan(N);
  if (quad(A, B) == 2 || quad(A, B) == 3)
    theta = PI + theta;
  else if (quad(A, B) == 4)
    theta = 2 * PI + theta;
  csc[0] = src.theta * PI / 180 - theta;
  csc[1] = theta - Algorithm::goal.theta * PI / 180;

  A = ACS;
  B = ACE;  // For Direct Common Tangent -anticlockwise to anticlockwise
  N = ((B.y - A.y) / (B.x - A.x));
  theta = atan(N);
  if (quad(A, B) == 2 || quad(A, B) == 3)
    theta = PI + theta;
  else if (quad(A, B) == 4)
    theta = 2 * PI + theta;
  csc[2] = src.theta * PI / 180 - theta;
  csc[3] = theta - Algorithm::goal.theta * PI / 180;

  if (lac > 2 * (rmin)) {  // For Transverse Common Tangent-anticlockwise to clockwise
    A = ACS;
    B = CE;
    N = ((B.y - A.y) / (B.x - A.x));
    theta = atan(N);
    if (quad(A, B) == 2 || quad(A, B) == 3)
      theta = PI + theta;
    else if (quad(A, B) == 4)
      theta = 2 * PI + theta;
    double phi_ac = acos(2 * rmin / lac);
    csc[4] = PI / 2 - src.theta * PI / 180 + theta - phi_ac;
    csc[5] = PI / 2 - Algorithm::goal.theta * PI / 180 + theta - phi_ac;
    ltct_ac = sqrt(lac * lac - 4 * rmin * rmin);  // length of TCT-anticlockwise to clockwise
  } else {
    csc[4] = INT_MAX;
    csc[5] = INT_MAX;
  }

  if (lca > 2 * (rmin)) {  // For Transverse Common Tangent -clockwise to
                           // anticlockwise
    A = CS;
    B = ACE;
    N = ((B.y - A.y) / (B.x - A.x));
    theta = atan(N);
    if (quad(A, B) == 2 || quad(A, B) == 3)
      theta = PI + theta;
    else if (quad(A, B) == 4)
      theta = 2 * PI + theta;
    double phi_ca = acos(2 * rmin / lca);
    csc[6] = PI / 2 + src.theta * PI / 180 - theta - phi_ca;
    csc[7] = PI / 2 + Algorithm::goal.theta * PI / 180 - theta - phi_ca;
    ltct_ca = sqrt(lca * lca - 4 * rmin * rmin);  // lenth of TCT-clockwise to anticlockwise
  } else {
    csc[6] = INT_MAX;
    csc[7] = INT_MAX;
  }

  int i;
  for (i = 0; i < 8; i++) {
    if (csc[i] > -0.0001 && csc[i] < 0.0001) csc[i] = 0;
    if (csc[i] < 0)
      csc[i] = csc[i] + 2 * PI;
    else if (csc[i] >= 2 * (PI))
      csc[i] = csc[i] - 2 * PI;
  }

  dis[0] = (rmin) * (csc[0] + csc[1]) + lcc;  // For DCT-clockwise
  dis[1] = (rmin) * (csc[2] + csc[3]) + lcc;  // For DCT-anticlockwise
  dis[2] = (rmin) * (csc[4] + csc[5]) + ltct_ac;  // For TCT-anticlockwise to clockwise
  dis[3] = (rmin) * (csc[6] + csc[7]) + ltct_ca;  // For TCT-clockwise to anticlockwise

  double pathCSC = INT_MAX;  // To find min of all possible CSC paths
  for (i = 0; i < 4; i++) {
    if (dis[i] < pathCSC) {
      pathCSC = dis[i];
    }
  }

  if (pathCCC > pathCSC) return pathCSC;
  return pathCCC;
}

/**
 * Core Method: hybrid A star planning.
 *
 * The whole planning happens here.
 *
 */
void Algorithm::hybridAstarPlanning() {
  // Run the Dijkstra's Search Algorithm on the 2d grid map.
  // FutureWork: Try A star
  astarPlanning();

  m_map.initCollisionChecker();
  m_map.findNearObs();

  // -------------------------------------------------------
  // Initialize the priority_queue and push the initial pose.
  std::priority_queue<State, std::vector<State>, Compare3d> pq;
  initial.cost3d = 0;
  pq.push(initial);

  // display the map and the initial configuration
  GUI display(800, 800);
  display.drawObs(m_map);
  display.drawCar(initial);
  display.drawCar(goal);

  // Initialize an array, visited, to mark the visited states.
  int visited[GX][GY][Theta];
  memset(visited, 0, sizeof(int) * GX * GY * Theta);

  while (pq.size() > 0) {
    State current = pq.top();
    pq.pop();

    // Check if the car has reached the goal.
    if (abs(current.gx - goal.gx) <= 1 && abs(current.gy - goal.gy) <= 1 &&
        abs(current.gtheta - goal.gtheta) <= 5) {
      std::cout << "Reached goal." << std::endl;

      State dummy;
      current.change =
          PRIORITY_OBSTACLE_NEAR *
              (m_map.obs_dist_max - m_map.nearestObstacleDistance(current)) /
              (double)(m_map.obs_dist_max - 1) +
          fabs(current.theta) / VEH_M_ALPHA + 1;

      // Draw trajectory from goal to initial.
      // FutureWork: reverse the order which makes more sense.
      while (current.x != initial.x || current.y != initial.y ||
             current.theta != initial.theta) {
        current.velocity = VELOCITY_MAX / current.change;
        display.drawCar(current);
        display.show(2000 / current.velocity); // This can be removed while executing the algo
        dummy = previous[current.gx][current.gy][current.gtheta];
        dummy.change =
            PRIORITY_MOVEMENT * fabs(dummy.theta - current.theta) /
                (2.0 * VEH_M_ALPHA) +
            PRIORITY_OBSTACLE_NEAR *
                (m_map.obs_dist_max - m_map.nearestObstacleDistance(dummy)) /
                (double)(m_map.obs_dist_max - 1) +
            fabs(dummy.theta) / VEH_M_ALPHA + 1;
        current = dummy;
      }

      break;
    }

    // Check if the currect state has been visited. Skip if visited.
    if (visited[current.gx][current.gy][current.gtheta]) {
      continue;
    }

    // Mark the current stated as visited.
    visited[current.gx][current.gy][current.gtheta] = 1;

    // Obtain the neighboring nodes (or states here).
    std::vector<State> next = current.getNextStates();

    // Exploring the neighboring nodes.
    for (int i = 0; i < next.size(); i++) {
      // display.drawCar(next[i]);
      if (!m_map.checkCollision(next[i])) {
        if (!visited[next[i].gx][next[i].gy][next[i].gtheta]) {
          // display.drawCar(next[i]);

          // Link the current Node and the next Node.
          current.next = &(next[i]);
          next[i].prev = &(current);

          // Calculate the accumulated costs.
          // Turing costs more than forward moving.
          if (i == 1) {
            next[i].cost3d = current.cost3d + 6;
          } else {
            next[i].cost3d = current.cost3d + 9;
          }
          pq.push(next[i]);

          // Construct edges (links).
          previous[next[i].gx][next[i].gy][next[i].gtheta] = current;
        }
      }
    }
  }

  std::cout << "Done." << std::endl;

  display.show(0);

  return;
}

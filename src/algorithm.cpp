#include "algorithm.h"

State Algorithm::initial;

State Algorithm::goal;

int **Algorithm::obs_map;

float **Algorithm::shortest_2d;

int **Algorithm::grid_obs_map;

State previous[GX][GY][Theta];

typedef bool (*compare2dSignature)(State, State);

struct Point {
  float x;
  float y;
};

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

bool compare2d(State a, State b) {
  // return a.cost2d > b.cost2d;	//simple dijkstra
  return a.cost2d + abs(Algorithm::goal.dx - a.dx) +
             abs(Algorithm::goal.dy - a.dy) >
         b.cost2d + abs(Algorithm::goal.dx - b.dx) +
             abs(Algorithm::goal.dy - b.dy);
}

////////////////////////////////////////////////////////////////////////////////

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

/*******************************************************************************
 * non_holonomic_without_obs()
 *   currently uses dijkstra's algorithm in x-y space
 ******************************************************************************/
float Algorithm::holonomic_with_obs(State src) {
  return Algorithm::shortest_2d[(int)src.x * DX / MAPX][(int)src.y * DY / MAPY];
}

/*******************************************************************************
 * non_holonomic_without_obs()
 ******************************************************************************/
float Algorithm::non_holonomic_without_obs(State src) {
  // return
  // abs(Algorithm::goal.x-src.x)+abs(Algorithm::goal.y-src.y)+abs(Algorithm::goal.theta-src.theta);
  float rmin = (BOT_L / tan((BOT_M_ALPHA)*PI / 180));

  Point CS, ACS, CE, ACE;

  CS.x = src.dx + (rmin)*sin((src.theta) * PI / 180);  // Clockwise-Start
  CS.y = src.dy - (rmin)*cos((src.theta) * PI / 180);

  ACS.x = src.dx - (rmin)*sin((src.theta) * PI / 180);  // Anti-Clockwise-Start
  ACS.y = src.dy + (rmin)*cos((src.theta) * PI / 180);

  CE.x = Algorithm::goal.dx +
         (rmin)*sin((Algorithm::goal.theta) * PI / 180);  // Clockwise-End
  CE.y = Algorithm::goal.dy - (rmin)*cos((Algorithm::goal.theta) * PI / 180);

  ACE.x = Algorithm::goal.dx -
          (rmin)*sin((Algorithm::goal.theta) * PI / 180);  // Anti-Clockwise-End
  ACE.y = Algorithm::goal.dy + (rmin)*cos((Algorithm::goal.theta) * PI / 180);

  float lcc =
      sqrt((CS.x - CE.x) * (CS.x - CE.x) +
           (CS.y - CE.y) * (CS.y - CE.y));  // clockwise-clockwise length
  float laa = sqrt((ACS.x - ACE.x) * (ACS.x - ACE.x) +
                   (ACS.y - ACE.y) * (ACS.y - ACE.y));
  float lca =
      sqrt((CS.x - ACE.x) * (CS.x - ACE.x) + (CS.y - ACE.y) * (CS.y - ACE.y));
  float lac =
      sqrt((ACS.x - CE.x) * (ACS.x - CE.x) + (ACS.y - CE.y) * (ACS.y - CE.y));

  Point A, B;

  // Code for CCC
  float pathCCC = INT_MAX;  // As CCC is not always possible
  if (!(lcc > 4 * rmin && laa > 4 * rmin)) {
    int d;
    d = std::min(lcc, laa);
    int flag;  // Flag=1 for clockwise being optimal and 0 for anticlockwise
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
    float t = atan(n);
    if (quad(A, B) == 2 || quad(A, B) == 3)
      t = PI + t;
    else if (quad(A, B) == 4)
      t = 2 * PI + t;
    n = d / (4 * rmin);
    float p = acos(n);

    float ccc[4];

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

    float L, U;
    if (flag == 0) {  // anticlockwise
      L = ccc[0] + ccc[1] + PI - 2 * p;
      U = ccc[2] + ccc[3] + PI + 2 * p;
    } else {  // clockwise
      L = ccc[0] + ccc[1] + PI + 2 * p;
      U = ccc[2] + ccc[3] + PI - 2 * p;
    }

    pathCCC = (rmin) * (std::min(L, U));
  }

  // Code for CSC
  float csc[8];
  float dis[4];  // Array to store final distance traversed in the 4 possible
                 // cases
  float theta;  // Angle made by line joining centres of two circular paths with
                // x-axis
  double N;
  float ltct_ac = INT_MAX, ltct_ca = INT_MAX;

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

  if (lac >
      2 * (rmin)) {  // For Transverse Common Tangent-anticlockwise to clockwise
    A = ACS;
    B = CE;
    N = ((B.y - A.y) / (B.x - A.x));
    theta = atan(N);
    if (quad(A, B) == 2 || quad(A, B) == 3)
      theta = PI + theta;
    else if (quad(A, B) == 4)
      theta = 2 * PI + theta;
    float phi_ac = acos(2 * rmin / lac);
    csc[4] = PI / 2 - src.theta * PI / 180 + theta - phi_ac;
    csc[5] = PI / 2 - Algorithm::goal.theta * PI / 180 + theta - phi_ac;
    ltct_ac =
        sqrt(lac * lac -
             4 * rmin * rmin);  // length of TCT-anticlockwise to clockwise
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
    float phi_ca = acos(2 * rmin / lca);
    csc[6] = PI / 2 + src.theta * PI / 180 - theta - phi_ca;
    csc[7] = PI / 2 + Algorithm::goal.theta * PI / 180 - theta - phi_ca;
    ltct_ca = sqrt(lca * lca -
                   4 * rmin * rmin);  // lenth of TCT-clockwise to anticlockwise
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
  dis[2] = (rmin) * (csc[4] + csc[5]) +
           ltct_ac;  // For TCT-anticlockwise to clockwise
  dis[3] = (rmin) * (csc[6] + csc[7]) +
           ltct_ca;  // For TCT-clockwise to anticlockwise

  float pathCSC = INT_MAX;  // To find min of all possible CSC paths
  for (i = 0; i < 4; i++) {
    if (dis[i] < pathCSC) {
      pathCSC = dis[i];
    }
  }

  if (pathCCC > pathCSC) return pathCSC;
  return pathCCC;
}

/*******************************************************************************
 * astar_planning()
 *   A* Search on the 2d grid map
 ******************************************************************************/
void Algorithm::astar_planning() {
  State src = Algorithm::goal;
  src.dx = src.gx * DX / GX;
  src.dy = src.gy * DY / GY;
  std::priority_queue<State, std::vector<State>, compare2dSignature> frontier(
      &compare2d);
  int vis[DX][DY];

  float **cost = new float *[DX];
  for (int i = 0; i < DX; i++) {
    cost[i] = new float[DY];
    for (int j = 0; j < DY; j++) cost[i][j] = 0;
  }

  memset(vis, 0, sizeof(int) * DX * DY);

  for (int i = 0; i < DX; i++) {
    for (int j = 0; j < DY; j++) {
      cost[i][j] = 10000;
    }
  }
  cost[src.dx][src.dy] = 0;

  frontier.push(src);
  while (!frontier.empty()) {
    State current = frontier.top();
    frontier.pop();

    int x = current.dx;
    int y = current.dy;

    if (vis[x][y]) continue;

    vis[x][y] = 1;

    for (int i = -1; i <= 1; i++) {
      for (int j = -1; j <= 1; j++) {
        if (x + i < 0 || x + i >= DX || y + j < 0 || y + j >= DY) continue;
        if ((i == 0 && j == 0) || Algorithm::grid_obs_map[x + i][y + j] != 0)
          continue;

        if (cost[x + i][y + j] > cost[x][y] + sqrt(i * i + j * j)) {
          cost[x + i][y + j] = cost[x][y] + sqrt(i * i + j * j);
          State tempstate;
          tempstate.dx = current.dx + i;
          tempstate.dy = current.dy + j;
          tempstate.cost2d = cost[x + i][y + j];
          frontier.push(tempstate);
        }
      }
    }
  }

  Algorithm::shortest_2d = cost;

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

/*******************************************************************************
 * hybrid_astar_planning()
 * core method: the whole planning happens here.
 ******************************************************************************/
void Algorithm::hybrid_astar_planning() {
  // Run the Dijkstra's Search Algorithm on the 2d grid map.
  // FutureWork: Try A star
  astar_planning();

  m_map.initCollisionChecker();
  m_map.find_near_obs();

  // -------------------------------------------------------
  // Initialize the priority_queue and push the initial pose.
  std::priority_queue<State, std::vector<State>, Compare> pq;
  initial.cost3d = 0;
  pq.push(initial);

  // display the map and the initial configuration
  GUI display(800, 800);
  display.drawObs(m_map);
  display.drawCar(initial);
  display.drawCar(goal);

  // Initialize an array, visited, to mark the visited states.
  int vis[GX][GY][Theta];
  memset(vis, 0, sizeof(int) * GX * GY * Theta);

  while (pq.size() > 0) {
    State current = pq.top();
    pq.pop();

    // Check if the car has reached the goal.
    if (abs(current.gx - goal.gx) <= 1 && abs(current.gy - goal.gy) <= 1 &&
        abs(current.gtheta - goal.gtheta) <= 5) {
      std::cout << "Reached goal." << std::endl;

      State Dummy;
      current.change =
          PRIORITY_OBSTACLE_NEAR *
              (m_map.obs_dist_max - m_map.nearest_obstacle_distance(current)) /
              (float)(m_map.obs_dist_max - 1) +
          fabs(current.theta) / BOT_M_ALPHA + 1;

      while (current.x != initial.x || current.y != initial.y ||
             current.theta != initial.theta) {
        current.velocity = VELOCITY_MAX / current.change;
        display.drawCar(current);
        display.show(
            2000 /
            current.velocity);  // This can be removed while executing the algo
        Dummy = previous[current.gx][current.gy][current.gtheta];
        Dummy.change =
            PRIORITY_MOVEMENT * fabs(Dummy.theta - current.theta) /
                (2.0 * BOT_M_ALPHA) +
            PRIORITY_OBSTACLE_NEAR *
                (m_map.obs_dist_max - m_map.nearest_obstacle_distance(Dummy)) /
                (float)(m_map.obs_dist_max - 1) +
            fabs(Dummy.theta) / BOT_M_ALPHA + 1;
        current = Dummy;
      }
      break;
    }

    // Check if the currect state has been visited. Skip if visited.
    if (vis[current.gx][current.gy][current.gtheta]) {
      continue;
    }

    // Mark the current stated as visited.
    vis[current.gx][current.gy][current.gtheta] = 1;

    // Obtain the neighboring nodes (or states here).
    std::vector<State> next = current.getNextStates();

    // Exploring the neighboring nodes.
    for (int i = 0; i < next.size(); i++) {
      // display.drawCar(next[i]);
      if (!m_map.checkCollision(next[i])) {
        if (!vis[next[i].gx][next[i].gy][next[i].gtheta]) {
          // display.drawCar(next[i]);

          // Link the current node and the next node.
          current.next = &(next[i]);
          next[i].previous = &(current);

          // Calculate the accumulated costs.
          // Turing costs more than forward moving.
          if (i == 1) {
            next[i].cost3d = current.cost3d + 5;
          } else {
            next[i].cost3d = current.cost3d + 7;
          }
          pq.push(next[i]);

          //
          previous[next[i].gx][next[i].gy][next[i].gtheta] = current;
        }
      }
    }
  }

  std::cout << "Done." << std::endl;

  display.show(0);

  return;
}

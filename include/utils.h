#ifndef UTILS_H
#define UTILS_H

#include <iostream>
#include <vector>
#include <string>
#include <queue>

#include "opencv2/highgui.hpp"
#include "opencv2/opencv.hpp"

// define geometric map size
#define MAPX 800 // maximal x coordinate in the geometric map
#define MAPY 800 // maximal y coordinate in the geometric map

#define VELOCITY_MAX 60

#define PRIORITY_OBSTACLE_NEAR 10
#define PRIORITY_MOVEMENT 5

// define grid map size and resolution
#define GX 80 // maximal x coordinate in the grid map
#define GY 80 // maximal x coordinate in the grid map
#define Grid_Res 10 // grid map resolution

// for running Dijkstra's search
#define DX 240
#define DY 240

#define Theta 72 // maximal angle
#define Theta_Res 5 // angle resolution

// vehicle shape
#define VEH_LEN 34 // vehicle length
#define VEH_WID 20 // vehicle width
#define VEH_M_ALPHA 30 // maximal steering angle

#define PI 3.14159

#endif  // UTILS_H

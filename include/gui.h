#ifndef GUI_H
#define GUI_H

#include "map.h"
#include "state.h"
#include "utils.h"

/**
 * GUI class:
 * To display the map and visualize the generated path.
*/
class GUI {
 public:
  cv::Size display_size;
  cv::Mat display;

  GUI(int rows, int cols);
  void drawCar(State src);
  void drawObs(Map map);
  void markPoint(int i, int j);
  void show(int t);
};

#endif  // GUI_H

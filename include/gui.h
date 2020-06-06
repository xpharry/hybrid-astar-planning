#ifndef GUI_H
#define GUI_H

#include "map.h"
#include "state.h"
#include "utils.h"

/**
 * Implement the GUI.
 *
 * To display the map and visualize the generated path.
 *
 */
class GUI {
public:
  /**
   * Constructor
   *
   * @param rows
   * @param cols
   */
  GUI(int rows, int cols);

  void drawCar(State src);

  void drawObs(Map map);

  void markPoint(int i, int j);

  void show(int t);

public:
  cv::Size display_size;
  cv::Mat display;
};

#endif  // GUI_H

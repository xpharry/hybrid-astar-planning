#include "algorithm.h"

State getStartState() {
  // FutureWork: read from yml file
  return State(700, 100, 36);
}

State getTargetState() {
  // FutureWork: read from yml file
  return State(100, 600, 18);
}

int main() {
  Map map("../data/map1.png");

  State initial = getStartState();

  State goal = getTargetState();

  Algorithm algorithm(map);

  algorithm.updateInitial(initial);

  algorithm.updateGoal(goal);

  algorithm.hybridAstarPlanning();

  return 0;
}

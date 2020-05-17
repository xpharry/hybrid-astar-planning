#include "planner.h"

State getStartState() {
  // todo: read from yml file
  return State(700, 100, 36);
}

State getTargetState() {
  // todo: read from yml file
  return State(100, 600, 18);
}

int main() {
  Map map;

  State start = getStartState();

  State target = getTargetState();

  Planner astar;

  astar.plan(start, target, map);

  return 0;
}

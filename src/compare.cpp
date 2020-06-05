#include "compare.h"

bool Compare::operator()(const State &s1, const State &s2) {
  // FutureWork: replace by cost + heuristic comparison
  return s1.cost3d > s2.cost3d;
  // return s1.cost3d + holonomic_with_obs(s1) + 0 * non_holonomic_without_obs(s1) >
  //        s2.cost3d + holonomic_with_obs(s2) + 0 * non_holonomic_without_obs(s2);
}

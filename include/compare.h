#ifndef COMPARE_H
#define COMPARE_H

#include "state.h"

struct Compare3d {
  bool operator()(const State &s1, const State &s2);
};

#endif // COMPARE_H

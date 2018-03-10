#pragma once

struct CSpaceInput{
  double timestep_min;
  double timestep_max;
  Config uMin;
  Config uMax;
  bool fixedBase{false};
};

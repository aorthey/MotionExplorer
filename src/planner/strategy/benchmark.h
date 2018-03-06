#pragma once
#include "file_input_output.h"
#include "planner/strategy/strategy_input.h"
#include <ompl/base/SpaceInformation.h>

class BenchmarkInformation
{
  public:
    BenchmarkInformation();

    bool Load(const char* file);
    bool Load(TiXmlElement *node);

    double maxPlanningTime;
    int maxMemory;
    int runCount;
    std::vector<std::string> algorithms;
};


#pragma once
#include "file_io.h"

class BenchmarkInput
{
  public:
    BenchmarkInput(std::string name_);

    bool Load(const char* file);
    bool Load(TiXmlElement *node);

    double maxPlanningTime;
    int maxMemory;
    int runCount;
    std::vector<std::string> algorithms;
    std::string name;
};


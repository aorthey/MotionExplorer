#pragma once
#include "file_io.h"
#include <ompl/tools/benchmark/Benchmark.h>

namespace ot = ompl::tools;

class BenchmarkOutput
{
  public:
    BenchmarkOutput(const ot::Benchmark::CompleteExperiment &experiment_);

    bool Save(const char* file);
    bool Save(TiXmlElement *node);
    void PrintPDF();

  private:
    ot::Benchmark::CompleteExperiment experiment;
    std::string file;

};


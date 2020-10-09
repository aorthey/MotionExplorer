#pragma once
#include "file_io.h"
#include <ompl/tools/benchmark/Benchmark.h>

class BenchmarkOutput
{
  public:
    BenchmarkOutput(const ompl::tools::Benchmark::CompleteExperiment &experiment_);

    bool Save(const char* file);
    bool Save(TiXmlElement *node);
    void PrintPDF();

  private:
    ompl::tools::Benchmark::CompleteExperiment experiment;
    std::string file;

};


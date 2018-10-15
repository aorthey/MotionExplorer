#include "benchmark_output.h"
#include "util.h"
#include <fstream>

// struct CompleteExperiment
// {
//     std::string name;
//     /// The collected experimental data; each element of the array (an experiment) corresponds to a planner
//     std::vector<PlannerExperiment> planners;
//     double maxTime;
//     double maxMem;
//     unsigned int runCount;
//     time::point startTime;
//     double totalDuration;
//     std::string setupInfo;
//     boost::uint32_t seed;
//     std::string host;
//     std::string cpuInfo;
//     std::map<std::string, std::string> parameters;
// };
// struct PlannerExperiment
// {
//   std::string name;
//   /// Data collected for each run
//   std::vector<RunProperties> runs;
//   /// Names of each of the planner progress properties
//   /// reported by planner
//   std::vector<std::string> progressPropertyNames;
//   /// For each run of the planner, this stores the set
//   /// of planner progress data reported by the planner
//   std::vector<RunProgressData> runsProgressData;
//   /// Some common properties for all the runs
//   RunProperties common;
// };
// typedef std::map<std::string, std::string> RunProperties;
// typedef std::vector<std::map<std::string, std::string>> RunProgressData;
//
// run["time REAL"]
// run["memory REAL"]
// run["status ENUM"]
//run["graph states INTEGER"]  [VERTICES]
//run["graph motions INTEGER"]  [EDGES]
//


BenchmarkOutput::BenchmarkOutput(const ot::Benchmark::CompleteExperiment& experiment_):
  experiment(experiment_)
{



}

bool BenchmarkOutput::Save(const char* file_)
{
  file = file_;
  TiXmlDocument doc;
  TiXmlElement *node = CreateRootNodeInDocument(doc);
  Save(node);
  doc.LinkEndChild(node);
  doc.SaveFile(file.c_str());
  std::cout << "Benchmark saved to " << file << std::endl;
  //std::ifstream f(file);
  //if (f.is_open()) std::cout << f.rdbuf();
  return true;
}
void BenchmarkOutput::PrintPDF()
{
  if(file==""){
    std::cout << "Cannot print PDF. No XML file loaded" << std::endl;
    return;
  }
  std::string cmd = std::string("python ../scripts/benchmarks/benchmark_to_pdf.py ")+file;
  int rvalue = std::system(cmd.c_str());

  if(rvalue){
    std::cout << "Successfully wrote converted to PDF" << std::endl;
  }else{
    std::cout << "### [ERROR] Benchmark to PDF failed" << std::endl;
  }
}

bool BenchmarkOutput::Save(TiXmlElement *node)
{
  node->SetValue("benchmark");
  AddSubNode(*node, "name", experiment.name);
  AddSubNode(*node, "run_count", experiment.runCount);
  AddSubNode(*node, "max_time", experiment.maxTime);
  AddSubNode(*node, "max_memory", experiment.maxMem);
  AddSubNode(*node, "number_of_planners", experiment.planners.size());

  for(uint k = 0; k < experiment.planners.size(); k++){
    TiXmlElement pknode("planner");
    ot::Benchmark::PlannerExperiment planner_experiment = experiment.planners.at(k);
    std::string name = util::RemoveStringBeginning(planner_experiment.name, "geometric");
    AddSubNode(pknode, "name", name);

    std::vector<ot::Benchmark::RunProperties> runs = planner_experiment.runs;
    for(uint j = 0; j < runs.size(); j++){
      TiXmlElement runnode("run");
      runnode.SetAttribute("number", j+1);
      ot::Benchmark::RunProperties run = runs.at(j);
      double time = std::atof(run["time REAL"].c_str());
      //if time exceeds maxTime, then clip it
      AddSubNode(runnode, "time", std::min(time, experiment.maxTime));
      AddSubNode(runnode, "memory", run["memory REAL"]);
      AddSubNode(runnode, "nodes", run["graph states INTEGER"]);
      if(time < experiment.maxTime){
        AddSubNode(runnode, "success", 1);
      }else{
        AddSubNode(runnode, "success", 0);
      }
      pknode.InsertEndChild(runnode);
    }
    node->InsertEndChild(pknode);
  }

  return true;
}


#include "benchmark_output.h"
#include "util.h"
#include <fstream>
#include <boost/lexical_cast.hpp>
#include <boost/filesystem.hpp>

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
  boost::filesystem::path p(file);
  boost::filesystem::path dir = p.parent_path();
  if(boost::filesystem::create_directory(dir))
  {
      std::cerr<< "Directory Created: "<< dir.string() <<std::endl;
  }

  TiXmlDocument doc;
  TiXmlElement *node = CreateRootNodeInDocument(doc);
  Save(node);
  doc.LinkEndChild(node);

  doc.SaveFile(file.c_str());
  std::cout << "Benchmark saved to " << file << std::endl;

  //Copy XML to "last.xml" for better access to last written xml
  std::ifstream src(file, std::ios::binary);

  std::string file_copy = dir.string()+"/last.xml";
  std::cout << file_copy << std::endl;
  std::ofstream dst(file_copy, std::ios::binary);
  dst << src.rdbuf();
  std::cout << "Benchmark copied to " << file_copy << std::endl;
  return true;
}
void BenchmarkOutput::PrintPDF()
{
  if(file==""){
    std::cout << "Cannot print PDF. No XML file loaded" << std::endl;
    return;
  }
  std::string cmd = std::string("python ../scripts/benchmarks/benchmark_to_pdf.py ")+file+std::string(" &");
  int rvalue = std::system(cmd.c_str());

  if(rvalue){
      std::cout << "Successfully converted XML to PDF" << std::endl;
  }else{
      std::cout << "### [ERROR] Benchmark to PDF failed" << std::endl;
  }
}

bool BenchmarkOutput::Save(TiXmlElement *node)
{
  node->SetValue("benchmark");
  AddSubNode(*node, "run_count", experiment.runCount);
  AddSubNode(*node, "max_time", experiment.maxTime);
  AddSubNode(*node, "max_memory", experiment.maxMem);
  AddSubNode(*node, "number_of_planners", experiment.planners.size());

  uint max_nr_of_layers = 0;
  std::vector<int> dimensionsPerLevel;
  for(uint k = 0; k < experiment.planners.size(); k++){
    TiXmlElement pknode("planner");
    ot::Benchmark::PlannerExperiment planner_experiment = experiment.planners.at(k);
    std::string name = util::RemoveStringBeginning(planner_experiment.name, "geometric");
    AddSubNode(pknode, "name", name);

    std::vector<ot::Benchmark::RunProperties> runs = planner_experiment.runs;
    std::string sstrat = "stratification levels INTEGER";
    if(runs.at(0).find(sstrat) != runs.at(0).end())
    {
        AddSubNode(pknode, "number_of_levels", runs.at(0)[sstrat]);
    }

    for(uint j = 0; j < runs.size(); j++){
      TiXmlElement runnode("run");
      runnode.SetAttribute("number", j+1);
      ot::Benchmark::RunProperties run = runs.at(j);
      double time = std::atof(run["time REAL"].c_str());
      //if time exceeds maxTime, then clip it
      AddSubNode(runnode, "time", std::min(time, experiment.maxTime));
      AddSubNode(runnode, "memory", run["memory REAL"]);
      AddSubNode(runnode, "nodes", run["graph states INTEGER"]);
      if(run.find(sstrat) != run.end()){
        //AddSubNode(runnode, "levels", run[sstrat]);
        TiXmlElement all_levels_node("levels");

        uint levels = boost::lexical_cast<uint>(run[sstrat]);
        if(levels > max_nr_of_layers) max_nr_of_layers = levels;

        for(uint k = 0; k < levels; k++){
          TiXmlElement level_node("level");

          std::string kstrat = "stratification level"+to_string(k)+" nodes INTEGER";
          AddSubNode(level_node, "nodes", run[kstrat]);

          std::string fkstrat = "stratification level"+to_string(k)+" feasible nodes INTEGER";
          AddSubNode(level_node, "feasible_nodes", run[fkstrat]);

          std::string dstrat = "stratification level"+to_string(k)+" dimension INTEGER";
          AddSubNode(level_node, "dimension", run[dstrat]);
          uint level_dimension = std::stoi(run[dstrat]);

          dimensionsPerLevel.push_back(level_dimension);
          all_levels_node.InsertEndChild(level_node);
        }
        if(j==0){
          TiXmlElement lnode("levels");
          for(uint k = 0; k < levels; k++){
            std::string dstrat = "stratification level"+to_string(k)+" dimension INTEGER";
            uint level_dimension = std::stoi(run[dstrat]);
            AddSubNode(lnode, "dimension", level_dimension);
          }
          pknode.InsertEndChild(lnode);
        }
        runnode.InsertEndChild(all_levels_node);
      }

      if(time < experiment.maxTime){
        AddSubNode(runnode, "success", 1);
      }else{
        AddSubNode(runnode, "success", 0);
      }
      pknode.InsertEndChild(runnode);
    }
    node->InsertEndChild(pknode);
  }
  std::sort(dimensionsPerLevel.begin(), dimensionsPerLevel.end());
  dimensionsPerLevel.erase( std::unique( dimensionsPerLevel.begin(), dimensionsPerLevel.end() ), dimensionsPerLevel.end() );

  TiXmlElement lnode("levels");
  for(uint k = 0; k < dimensionsPerLevel.size(); k++){
    AddSubNode(lnode, "dimension", dimensionsPerLevel.at(k));
  }
  TiXmlNode *fc = node->FirstChild();
  node->InsertBeforeChild(fc, lnode);

  AddSubNodeBeginning(*node, "number_levels", max_nr_of_layers);
  AddSubNodeBeginning(*node, "name", experiment.name);

  return true;
}


#include "planner/strategy/benchmark.h"
#include "util.h"

BenchmarkInformation::BenchmarkInformation()
{
  std::string bmdef = util::GetDataFolder()+"/../settings/benchmark.xml";
  Load(bmdef.c_str());
}

bool BenchmarkInformation::Load(const char* file)
{
  TiXmlDocument doc(file);
  TiXmlElement *root = GetRootNodeFromDocument(doc);
  return Load(root);
}
bool BenchmarkInformation::Load(TiXmlElement *node)
{
  CheckNodeName(node, "benchmark");

  maxPlanningTime = GetSubNodeTextDefault(node, "maxplanningtime", 5.0);
  runCount = GetSubNodeTextDefault(node, "runcount", 10);
  maxMemory = GetSubNodeTextDefault(node, "maxmemory", 10000);

  TiXmlElement* node_algorithm = FindFirstSubNode(node, "algorithm");
  algorithms.clear();
  while(node_algorithm!=NULL){
    std::string a = GetAttribute<std::string>(node_algorithm, "name");
    algorithms.push_back(a);
    node_algorithm = FindNextSiblingNode(node_algorithm);
  }
  return true;
}


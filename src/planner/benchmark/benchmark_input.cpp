#include "benchmark_input.h"
#include "util.h"

BenchmarkInput::BenchmarkInput(std::string name_):
  name(name_)
{
  std::string bmdef = util::GetDataFolder()+"/../settings/benchmark.xml";
  if(!Load(bmdef.c_str())){
    std::cout << "Could not load benchmark " << name << "." << std::endl;
    exit(1);
  }
}

bool BenchmarkInput::Load(const char* file)
{
  TiXmlDocument doc(file);
  TiXmlElement *root = GetRootNodeFromDocument(doc);
  return Load(root);
}
bool BenchmarkInput::Load(TiXmlElement *node)
{
  CheckNodeName(node, "benchmarks");

  TiXmlElement *bnode = FindFirstSubNode(node, "benchmark");

  while(bnode){
    std::string str = GetAttribute<std::string>(bnode, "name");
    std::cout << str << "/" << name << std::endl;
    if(util::EndsWith(name, str))
    {
      maxPlanningTime = GetSubNodeTextDefault(bnode, "maxplanningtime", 5.0);
      runCount = GetSubNodeTextDefault(bnode, "runcount", 10);
      maxMemory = GetSubNodeTextDefault(bnode, "maxmemory", 10000);

      TiXmlElement* node_algorithm = FindFirstSubNode(bnode, "algorithm");
      algorithms.clear();
      while(node_algorithm!=NULL){
        std::string a = GetAttribute<std::string>(node_algorithm, "name");
        algorithms.push_back(a);
        node_algorithm = FindNextSiblingNode(node_algorithm);
      }
      return true;
    }
    bnode = FindNextSiblingNode(bnode);
  }
  return false;
}


#include "planner/planner_output.h"

PlannerOutput::PlannerOutput(){
  sv = NULL;
  swv = NULL;
};

void PlannerOutput::SetHierarchy(std::vector<HierarchicalLevel> &hierarchy_){
  hierarchy = hierarchy_;
}
const std::vector<HierarchicalLevel>& PlannerOutput::GetHierarchy(){
  return hierarchy;
}

void PlannerOutput::SetTorques(std::vector<Vector> &torques_){
  torques = torques_;
}
const std::vector<Vector>& PlannerOutput::GetTorques(){
  return torques;
}

Config PlannerOutput::GetInitConfiguration(){
  if(q.empty()){
    std::cout << "[PlannerOutput] No path in output, cannot get initial configuration" << std::endl;
    exit(0);
  }
  return q.at(0);
}

const SerializedTree& PlannerOutput::GetTree()
{
  return _stree;
}
void PlannerOutput::SetTree(SerializedTree &stree)
{
  _stree = stree;
}

const SimplicialComplex& PlannerOutput::GetSimplicialComplex(){
  return cmplx;
}
const std::vector<Config> PlannerOutput::GetKeyframes(){
  return q;
}
void PlannerOutput::SetKeyframes(std::vector<Config> &keyframes){
  q = keyframes;
}

SweptVolume& PlannerOutput::GetSweptVolume(){
  if(!sv){
    sv = new SweptVolume(robot, q, drawMilestones);
  }
  return *sv;
}
SwathVolume& PlannerOutput::GetSwathVolume(){
  if(!swv){
    vector<Config> qs;
    for(uint i = 0; i < _stree.size(); i++){
      SerializedTreeNode node = _stree.at(i);
      qs.push_back(node.position);
    }
    swv = new SwathVolume(robot, qs);
  }
  return *swv;
}

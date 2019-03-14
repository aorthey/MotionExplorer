#include "planner/planner_output.h"
#include <boost/filesystem.hpp>
#include <fstream>


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


const std::vector<Config> PlannerOutput::GetKeyframes(){
  return q;
}
void PlannerOutput::SetKeyframes(std::vector<Config> &keyframes){
  q = keyframes;
}


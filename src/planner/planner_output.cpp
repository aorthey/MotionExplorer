#include "planner/planner_output.h"
#include <boost/filesystem.hpp>
#include <fstream>

PlannerOutput::PlannerOutput(){
  sv = NULL;
  swv = NULL;
};

//void PlannerOutput::VerticesToFile(){
//
//  ofstream fh;
//  fh.open ("vertices.txt");
//  if(!swv) GetSwathVolume();
//  std::vector<Config> tree= swv->GetKeyframes();
//  for(uint k = 0; k < tree.size(); k++){
//    fh << tree.at(k) << std::endl;
//  }
//  fh.close();
//
//  fh.open ("vertices_path.txt");
//  if(!sv) GetSweptVolume();
//  std::vector<Config> path = sv->GetKeyframes();
//  for(uint k = 0; k < path.size(); k++){
//    fh << path.at(k) << std::endl;
//  }
//  fh.close();
//}


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


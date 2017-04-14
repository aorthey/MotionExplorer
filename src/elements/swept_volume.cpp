#include "swept_volume.h"

SweptVolume::SweptVolume(Robot *robot){
  _keyframes.clear();
  _mats.clear();
  _robot = robot;
  color = GLColor(0.8,0.8,0.8);
}

SweptVolume::SweptVolume(Robot *robot, const std::vector<Config> &keyframes, uint Nkeyframes)
{
  _keyframes.clear();
  _mats.clear();
  _robot = robot;
  color = GLColor(0.8,0.8,0.8);

  for(int i = 0; i < keyframes.size(); i++)
  {
    Config q = keyframes.at(i);
    AddKeyframe(q);
  }
  init = _keyframes.front();
  goal = _keyframes.back();
  //compute keyframe indices 
  if(Nkeyframes > keyframes.size()){
    Nkeyframes = keyframes.size();
  }
  if(Nkeyframes < 1){
    Nkeyframes=0;
  }
  _keyframe_indices.clear();

  uint N = keyframes.size();
  uint Nstep = (int)(N/Nkeyframes);

  if(Nstep<1) Nstep=1;
  uint Ncur = 0;
  while(Ncur < N){
    _keyframe_indices.push_back(Ncur);
    Ncur += Nstep;
  }
  //std::cout << "Milestone visualization indicies: " << _keyframe_indices << std::endl;

}

const std::vector<std::vector<Matrix4> >& SweptVolume::GetMatrices(){
  return _mats;
}
const std::vector<Config >& SweptVolume::GetKeyframes(){
  return _keyframes;
}
void SweptVolume::SetColor(const GLColor c){
  color = c;
}
GLColor SweptVolume::GetColor(){
  return color;
}
void SweptVolume::SetColorMilestones(const GLColor c){
  color_milestones= c;
}
GLColor SweptVolume::GetColorMilestones(){
  return color_milestones;
}
const Config& SweptVolume::GetStart(){
  return init;
}
const Config& SweptVolume::GetGoal(){
  return goal;
}
const vector<uint>& SweptVolume::GetKeyframeIndices(){
  return _keyframe_indices;
}
void SweptVolume::AddKeyframe(const Config &q ){

  if(!_robot->InJointLimits(q)){
    std::cout << "trying to set an outer limit config" << std::endl;
    std::cout << "minimum       :" << _robot->qMin << std::endl;
    std::cout << "configuration :" << q << std::endl;
    std::cout << "maximum       :" << _robot->qMax << std::endl;
    exit(0);
  }
  _robot->UpdateConfig(q);

  std::vector<Matrix4> mats_config;
  for(size_t i=0;i<_robot->links.size();i++) {
    Matrix4 mat = _robot->links[i].T_World;
    mats_config.push_back(mat);
  }
  _mats.push_back(mats_config);
  _keyframes.push_back(q);
}

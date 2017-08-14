
SwathVolume::SwathVolume(Robot *robot){
  _vertices.clear();
  _mats.clear();
  _robot = robot;
  color = GLColor(0.8,0.8,0.8);
}

SwathVolume::SwathVolume(Robot *robot, const std::vector<Config> &vertices)
{
  _vertices.clear();
  _mats.clear();
  _robot = robot;
  color = GLColor(0.5,0.7,0.5);

  for(int i = 0; i < vertices.size(); i++)
  {
    Config q = vertices.at(i);
    AddVertex(q);
  }
}

const std::vector<std::vector<Matrix4> >& SwathVolume::GetMatrices(){
  return _mats;
}
const std::vector<Config >& SwathVolume::GetKeyframes(){
  return _keyframes;
}
void SwathVolume::AddVertex(const Config &q ){
  _robot->UpdateConfig(q);
  std::vector<Matrix4> mats_config;
  for(size_t i=0;i<_robot->links.size();i++) {
    Matrix4 mat = _robot->links[i].T_World;
    mats_config.push_back(mat);
  }
  _mats.push_back(mats_config);
  _vertices.push_back(q);
}
void SwathVolume::SetColor(const GLColor c){
  color = c;
}
GLColor SwathVolume::GetColor(){
  return color;
}

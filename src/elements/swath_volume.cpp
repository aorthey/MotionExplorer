#include "elements/swath_volume.h"

SwathVolume::SwathVolume(Robot *robot):
  SweptVolume(robot)
{
}

SwathVolume::SwathVolume(Robot *robot, const std::vector<Config> &vertices):
  SweptVolume(robot, vertices, 0)
{
}

const std::vector<Config >& SwathVolume::GetVertices(){
  return _keyframes;
}

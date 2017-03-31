#pragma once
#include <Modeling/Robot.h>
#include "gui.h"

class IrreducibleProjector
{
  private:
    uint Ndim;
    uint _Nsublinks;
    uint _Nkeyframes;
    std::vector<Config> _rootPath;
    Robot *_robot;

    Vector3 GetPositionAtLink(const Config &q, uint id);
    Matrix3 GetRotationAtLink(const Config &q, uint id);
  public:
    IrreducibleProjector(Robot *robot);
    void setRootPath( std::vector<Config> &keyframes);
    std::vector<Config> getSubLinkKeyframes(ForceFieldBackend &backend, std::vector<double> &lengths);

};

#pragma once
#include <Modeling/Robot.h>
#include "gui.h"
#include "elements/path_pwl_euclid.h"

typedef std::vector<double> doubleVec;
typedef std::vector<doubleVec> doubleVecVec;
typedef std::pair<doubleVec,doubleVec> pairDoubleVec;
typedef std::pair<doubleVecVec,doubleVecVec> pairDoubleVecVec;

class IrreducibleProjector
{
  protected:
    uint Ndim;
    uint _Nsublinks;
    uint _Nkeyframes;
    std::vector<Config> _rootPath;

    std::vector<Vector3> _positionAlongRootPath;
    std::vector<Matrix3> _rotationAlongRootPath; //vector X points into velocity direction

    Robot *_robot;

    Vector3 GetPositionAtLink(const Config &q, uint id);
    Matrix3 GetRotationAtLink(const Config &q, uint id);


  public:

    IrreducibleProjector(Robot *robot);

    void setRootPath( std::vector<Config> &keyframes);

    pairDoubleVecVec ComputeThetaGammaFromRootPath( const std::vector<Vector3> &rootPos, const std::vector<Matrix3> &rootRot, const std::vector<double> &lengths);

    pairDoubleVec ComputeThetaGammaFromRootPathPosition(const PathPiecewiseLinearEuclidean &path, double t0, const Matrix3 &R0, const std::vector<double> &lengths);

    virtual std::vector<Config> getSubLinkKeyframes() = 0;

};

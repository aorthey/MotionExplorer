#pragma once
#include <Modeling/Robot.h>
#include "gui/gui.h"
#include "elements/path_pwl_euclid.h"
#include "planner/irreducible_projector.h"

class IrreducibleProjectorSnake: public IrreducibleProjector
{
  public:

    IrreducibleProjectorSnake(Robot *robot): IrreducibleProjector(robot){};

    //std::vector<Config> getSubLinkKeyframes(std::vector<double> &lengths, uint Nbranches);
    virtual std::vector<Config> getSubLinkKeyframes()
    {
      uint N = _robot->links.size();
      uint Nsub = N - _rootPath.at(0).size();
      uint Nsegments= (Nsub - 2)/3+1;

      std::vector<double> lengths(Nsegments-1);
      double length = 0.16;
      for(int j = 0; j < Nsegments-1; j++){
        lengths.at(j)=length;
      }

      //###########################################################################
      // set zero config wholebodypath
      //###########################################################################
      std::vector<Config> wholeBodyPath;
      for(int i = 0; i < _rootPath.size(); i++){
        Config q;
        q.resize(_robot->q.size());
        q.setZero();
        Config qhead = _rootPath.at(i);
        for(int j = 0; j < _rootPath.at(i).size(); j++){
          q(j) = qhead(j);
        }
        wholeBodyPath.push_back(q);
      }

      //###########################################################################
      // compute 
      //###########################################################################
      uint rootLinkId = 9;

      vector<string> linkNames = _robot->linkNames;

      std::cout << std::string(80, '-') << std::endl;
      std::cout << "IrreducibleProjector" << std::endl;
      std::cout << "obtaining " << _Nkeyframes << " keyframes" << std::endl << std::endl;

      std::cout << "Computing irreducible path from root link: " << linkNames[rootLinkId] << std::endl;
      std::cout << " root link has " << lengths.size() << " sublinks with lengths " << lengths << std::endl;
      std::cout << std::string(80, '-') << std::endl;
      _positionAlongRootPath.clear();
      _rotationAlongRootPath.clear();

      for(int i = 0; i < _rootPath.size(); i++){
        Config qr = _rootPath.at(i);

        Vector3 T0 = GetPositionAtLink(qr, rootLinkId);
        Matrix3 R0 = GetRotationAtLink(qr, rootLinkId);

        _positionAlongRootPath.push_back(T0);
        _rotationAlongRootPath.push_back(R0);

      }

      pairDoubleVecVec thetagamma = ComputeThetaGammaFromRootPath( _positionAlongRootPath ,_rotationAlongRootPath, lengths);

      doubleVecVec thetas = thetagamma.first;
      doubleVecVec gammas = thetagamma.second;

      assert(thetas.size()==_rootPath.size());
      assert(gammas.size()==_rootPath.size());

      for(int i = 0; i < wholeBodyPath.size(); i++){

        for(int j = 0; j < lengths.size(); j++){
          double tij = thetas.at(i).at(j);
          double gij = gammas.at(i).at(j);
          wholeBodyPath.at(i)(rootLinkId+3*j) = tij;
          wholeBodyPath.at(i)(rootLinkId+3*j+1) = gij;
          wholeBodyPath.at(i)(rootLinkId+3*j+2) = 0.0;
        }
        //std::cout << wholeBodyPath.at(i) << std::endl;
      }


    return wholeBodyPath;
  }



};

#pragma once
#include "cspace.h"
#include "cspace_input.h"
#include "cspace_decorator.h"


class CSpaceFactory{
  private:
    const CSpaceInput& input;
  public:
    CSpaceFactory(const CSpaceInput &input_): input(input_){};
    virtual KinodynamicCSpaceOMPL* MakeKinodynamicCSpace( Robot *robot, CSpace *kspace){
      KinodynamicCSpaceOMPL *cspace = new KinodynamicCSpaceOMPL(robot, kspace);
      cspace->SetCSpaceInput(input);
      cspace->initSpace();
      cspace->initControlSpace();
      return cspace;
    }
    virtual GeometricCSpaceOMPL* MakeGeometricCSpace( Robot *robot, CSpace *kspace){
      GeometricCSpaceOMPL *cspace = new GeometricCSpaceOMPL(robot, kspace);
      cspace->SetCSpaceInput(input);
      cspace->initSpace();
      cspace->initControlSpace();
      return cspace;
    }
    //CSpace for a rigid object which has rotational invariance. CSpace is R3
    virtual GeometricCSpaceOMPL* MakeGeometricCSpaceRotationalInvariance( Robot *robot, CSpace *inner){
      GeometricCSpaceOMPL *cspace = new GeometricCSpaceOMPLRotationalInvariance(robot, inner);
      cspace->SetCSpaceInput(input);
      cspace->initSpace();
      cspace->initControlSpace();
      return cspace;
    }

    // CSpace which has two rotational degrees of freedom
    // [0,1] \times S^1 \times S^1
    // representing a roll-orientation invariant object along an arbitrary R^3
    // path

    virtual GeometricCSpaceOMPL* MakeGeometricCSpacePathConstraintRollInvariance( Robot *robot, CSpace *inner, const std::vector<Config>& path){
      GeometricCSpaceOMPL *cspace = new GeometricCSpaceOMPLPathConstraintRollInvariance(robot, inner, path);
      //make pwl path from path, then add one more [0,1] dimension to cspace,
      //and make it depend on pwlpath
      cspace->SetCSpaceInput(input);
      cspace->initSpace();
      cspace->initControlSpace();
      return cspace;
    }
    //
    //a configuration q in the innerouter cspace is feasible iff
    // inner(q) is feasible AND outer(q) is infeasible
    //
    virtual CSpaceOMPL* MakeCSpaceDecoratorInnerOuter( CSpaceOMPL* cs, CSpace *outer){
      CSpaceOMPL *cspace = new CSpaceOMPLDecoratorInnerOuter(cs, outer);
      cspace->SetCSpaceInput(input);
      cspace->initSpace();
      cspace->initControlSpace();
      return cspace;
    }
};


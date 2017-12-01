#pragma once
#include "planner/cspace/cspace.h"
#include "planner/cspace/cspace_geometric_RN.h"
#include "planner/cspace/cspace_geometric_SE2.h"
#include "planner/cspace/cspace_geometric_SE3.h"
#include "planner/cspace/cspace_input.h"
#include "planner/cspace/cspace_decorator.h"


class CSpaceFactory{
  private:
    const CSpaceInput& input;
  public:
    CSpaceFactory(const CSpaceInput &input_): input(input_){};
    virtual KinodynamicCSpaceOMPL* MakeKinodynamicCSpace( Robot *robot, CSpaceKlampt *kspace){
      KinodynamicCSpaceOMPL *cspace = new KinodynamicCSpaceOMPL(robot, kspace);
      cspace->SetCSpaceInput(input);
      cspace->initSpace();
      cspace->initControlSpace();
      return cspace;
    }
    virtual GeometricCSpaceOMPL* MakeGeometricCSpace( RobotWorld *world, int robot_idx){
      GeometricCSpaceOMPL *cspace = new GeometricCSpaceOMPL(world, robot_idx);
      cspace->SetCSpaceInput(input);
      cspace->initSpace();
      cspace->initControlSpace();
      return cspace;
    }
    //CSpace for a rigid object which has rotational invariance. CSpace is R3
    virtual GeometricCSpaceOMPL* MakeGeometricCSpaceRotationalInvariance( Robot *robot, CSpaceKlampt *inner){
      GeometricCSpaceOMPL *cspace = new GeometricCSpaceOMPLRotationalInvariance(robot, inner);
      cspace->SetCSpaceInput(input);
      cspace->initSpace();
      cspace->initControlSpace();
      return cspace;
    }


    virtual GeometricCSpaceOMPL* MakeGeometricCSpacePathConstraintSO3( Robot *robot, CSpaceKlampt *inner, const std::vector<Config>& path){
      GeometricCSpaceOMPL *cspace = new GeometricCSpaceOMPLPathConstraintSO3(robot, inner, path);
      cspace->SetCSpaceInput(input);
      cspace->initSpace();
      cspace->initControlSpace();
      return cspace;
    }
    // CSpace  SE(3)
    virtual GeometricCSpaceOMPL* MakeGeometricCSpaceSE3( RobotWorld *world, int robot_idx){
      GeometricCSpaceOMPL *cspace = new GeometricCSpaceOMPLSE3(world, robot_idx);
      cspace->SetCSpaceInput(input);
      cspace->initSpace();
      cspace->initControlSpace();
      return cspace;
    }
    // CSpace  SE(2)
    virtual GeometricCSpaceOMPL* MakeGeometricCSpaceSE2( RobotWorld *world, int robot_idx){
      GeometricCSpaceOMPL *cspace = new GeometricCSpaceOMPLSE2(world, robot_idx);
      cspace->SetCSpaceInput(input);
      cspace->initSpace();
      cspace->initControlSpace();
      return cspace;
    }
    // CSpace  R^(N)
    virtual GeometricCSpaceOMPL* MakeGeometricCSpaceRN( RobotWorld *world, int robot_idx, int dimension){
      GeometricCSpaceOMPL *cspace = new GeometricCSpaceOMPLRN(world, robot_idx, dimension);
      cspace->SetCSpaceInput(input);
      cspace->initSpace();
      cspace->initControlSpace();
      return cspace;
    }

    // CSpace {q} \times SO(3)
    virtual GeometricCSpaceOMPL* MakeGeometricCSpacePointConstraintSO3( Robot *robot, CSpaceKlampt *inner, const Config q){
      GeometricCSpaceOMPL *cspace = new GeometricCSpaceOMPLPointConstraintSO3(robot, inner, q);
      cspace->SetCSpaceInput(input);
      cspace->initSpace();
      cspace->initControlSpace();
      return cspace;
    }
    //
    //a configuration q in the innerouter cspace is feasible iff
    // inner(q) is feasible AND outer(q) is infeasible
    //
    virtual CSpaceOMPL* MakeCSpaceDecoratorInnerOuter( CSpaceOMPL* cs, CSpaceKlampt *outer){
      CSpaceOMPL *cspace = new CSpaceOMPLDecoratorInnerOuter(cs, outer);
      cspace->SetCSpaceInput(input);
      cspace->initSpace();
      cspace->initControlSpace();
      return cspace;
    }
    virtual CSpaceOMPL* MakeCSpaceDecoratorNecessarySufficient( CSpaceOMPL* cs, CSpaceKlampt *outer){
      CSpaceOMPL *cspace = new CSpaceOMPLDecoratorNecessarySufficient(cs, outer);
      cspace->SetCSpaceInput(input);
      cspace->initSpace();
      cspace->initControlSpace();
      return cspace;
    }
};


#pragma once
#include "planner/cspace/cspace.h"
#include "planner/cspace/cspace_geometric.h"
#include "planner/cspace/cspace_kinodynamic.h"
#include "planner/cspace/cspace_geometric_RN.h"
#include "planner/cspace/cspace_geometric_SE2.h"
#include "planner/cspace/cspace_geometric_R3S2.h"
#include "planner/cspace/cspace_geometric_fixedbase.h"
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

    // CSpace  SE(3)
    virtual GeometricCSpaceOMPL* MakeGeometricCSpaceSE3( RobotWorld *world, int robot_idx){
      return MakeGeometricCSpace(world, robot_idx);
    }
    // CSpace  R^3 \times S1 \times S1
    virtual GeometricCSpaceOMPL* MakeGeometricCSpaceR3S2( RobotWorld *world, int robot_idx){
      GeometricCSpaceOMPL *cspace = new GeometricCSpaceOMPLR3S2(world, robot_idx);
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
    // CSpace  R^(N)
    virtual GeometricCSpaceOMPL* MakeGeometricCSpaceFixedBase( RobotWorld *world, int robot_idx, int dimension=0){
      GeometricCSpaceOMPL *cspace = new GeometricCSpaceOMPLFixedBase(world, robot_idx);
      cspace->SetCSpaceInput(input);
      cspace->initSpace();
      cspace->initControlSpace();
      return cspace;
    }
};


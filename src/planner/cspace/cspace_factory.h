#pragma once
#include "planner/cspace/cspace.h"
#include "planner/cspace/cspace_geometric.h"
#include "planner/cspace/cspace_geometric_empty.h"
#include "planner/cspace/cspace_multiagent.h"
#include "planner/cspace/cspace_kinodynamic.h"
#include "planner/cspace/cspace_kinodynamic_SE2.h"
#include "planner/cspace/cspace_kinodynamic_SO2.h"
#include "planner/cspace/cspace_geometric_RN.h"
#include "planner/cspace/cspace_geometric_Mobius.h"
#include "planner/cspace/cspace_geometric_SolidTorus.h"
#include "planner/cspace/cspace_geometric_SolidCylinder.h"
#include "planner/cspace/cspace_geometric_Annulus.h"
#include "planner/cspace/cspace_geometric_Circular.h"
#include "planner/cspace/cspace_geometric_RN_time.h"
#include "planner/cspace/cspace_geometric_SE2RN.h"
#include "planner/cspace/cspace_geometric_SE2Dubin.h"
#include "planner/cspace/cspace_geometric_SE3Dubin.h"
#include "planner/cspace/cspace_geometric_SE3_constrained.h"
#include "planner/cspace/cspace_geometric_SO2RN.h"
#include "planner/cspace/cspace_geometric_R3S2.h"
#include "planner/cspace/cspace_geometric_fixedbase.h"
#include "planner/cspace/cspace_input.h"


class CSpaceFactory{
  private:
    const CSpaceInput& input;
  public:
    CSpaceFactory(const CSpaceInput &input_): input(input_){};

    virtual KinodynamicCSpaceOMPL* MakeKinodynamicCSpace( RobotWorld *world, int robot_idx){
      KinodynamicCSpaceOMPL *cspace = new KinodynamicCSpaceOMPL(world, robot_idx);
      cspace->SetCSpaceInput(input);
      cspace->Init();
      cspace->initControlSpace();
      return cspace;
    }
    virtual KinodynamicCSpaceOMPL* MakeKinodynamicCSpaceSE2( RobotWorld *world, int robot_idx){
      KinodynamicCSpaceOMPL *cspace = new KinodynamicCSpaceOMPLSE2(world, robot_idx);
      cspace->SetCSpaceInput(input);
      cspace->Init();
      cspace->initControlSpace();
      return cspace;
    }
    virtual KinodynamicCSpaceOMPL* MakeKinodynamicCSpaceSO2( RobotWorld *world, int robot_idx){
      KinodynamicCSpaceOMPL *cspace = new KinodynamicCSpaceOMPLSO2(world, robot_idx);
      cspace->SetCSpaceInput(input);
      cspace->Init();
      cspace->initControlSpace();
      return cspace;
    }

    virtual GeometricCSpaceOMPL* MakeGeometricCSpace( RobotWorld *world, int robot_idx){
      GeometricCSpaceOMPL *cspace = new GeometricCSpaceOMPL(world, robot_idx);
      cspace->SetCSpaceInput(input);
      cspace->Init();
      return cspace;
    }

    // CSpace  SE(3)
    virtual GeometricCSpaceOMPL* MakeGeometricCSpaceSE3( RobotWorld *world, int robot_idx){
      return MakeGeometricCSpace(world, robot_idx);
    }
    virtual GeometricCSpaceOMPL* MakeGeometricCSpaceSE3Constrained( RobotWorld *world, int robot_idx){
      GeometricCSpaceOMPL *cspace = new GeometricCSpaceOMPLSE3Constrained(world, robot_idx);
      cspace->SetCSpaceInput(input);
      cspace->Init();
      return cspace;
    }

    virtual CSpaceOMPLMultiAgent* MakeGeometricCSpaceMultiAgent( std::vector<CSpaceOMPL*> cspaces){
      CSpaceOMPLMultiAgent *cspace = new CSpaceOMPLMultiAgent(cspaces);
      cspace->SetCSpaceInput(input);
      cspace->Init();
      cspace->initControlSpace();
      return cspace;
    }
    virtual GeometricCSpaceOMPL* MakeGeometricCSpaceSE3Dubin( RobotWorld *world, int robot_idx){
      //return MakeGeometricCSpaceSE2RN(world, robot_idx);
      GeometricCSpaceOMPL *cspace = new GeometricCSpaceOMPLSE3Dubin(world, robot_idx);
      cspace->SetCSpaceInput(input);
      cspace->Init();
      return cspace;
    }
    virtual GeometricCSpaceOMPL* MakeGeometricCSpaceSE2Dubin( RobotWorld *world, int robot_idx){
      //return MakeGeometricCSpaceSE2RN(world, robot_idx);
      GeometricCSpaceOMPL *cspace = new GeometricCSpaceOMPLSE2Dubin(world, robot_idx);
      cspace->SetCSpaceInput(input);
      cspace->Init();
      return cspace;
    }
    // CSpace  SE(2) x R^(N)
    virtual GeometricCSpaceOMPL* MakeGeometricCSpaceSE2RN( RobotWorld *world, int robot_idx){
      //return MakeGeometricCSpaceSE2RN(world, robot_idx);
      GeometricCSpaceOMPL *cspace = new GeometricCSpaceOMPLSE2RN(world, robot_idx);
      cspace->SetCSpaceInput(input);
      cspace->Init();
      return cspace;
    }
    // CSpace  SE(2)
    virtual GeometricCSpaceOMPL* MakeGeometricCSpaceSE2( RobotWorld *world, int robot_idx){
      return MakeGeometricCSpaceSE2RN(world, robot_idx);
    }
    // CSpace  R^3 \times S1 \times S1
    virtual GeometricCSpaceOMPL* MakeGeometricCSpaceR3S2( RobotWorld *world, int robot_idx){
      GeometricCSpaceOMPL *cspace = new GeometricCSpaceOMPLR3S2(world, robot_idx);
      cspace->SetCSpaceInput(input);
      cspace->Init();
      return cspace;
    }
    // CSpace  SO(2)
    virtual GeometricCSpaceOMPL* MakeGeometricCSpaceSO2( RobotWorld *world, int robot_idx){
      return MakeGeometricCSpaceSO2RN(world, robot_idx);
    }
    // CSpace  SO(2) x R^(N)
    virtual GeometricCSpaceOMPL* MakeGeometricCSpaceSO2RN( RobotWorld *world, int robot_idx){
      GeometricCSpaceOMPL *cspace = new GeometricCSpaceOMPLSO2RN(world, robot_idx);
      cspace->SetCSpaceInput(input);
      cspace->Init();
      return cspace;
    }
    // CSpace  R^(N)
    virtual GeometricCSpaceOMPL* MakeGeometricCSpaceRN( RobotWorld *world, int robot_idx, int dimension){
      GeometricCSpaceOMPL *cspace = new GeometricCSpaceOMPLRN(world, robot_idx, dimension);
      cspace->SetCSpaceInput(input);
      cspace->Init();
      return cspace;
    }
    // CSpace  Empty
    virtual GeometricCSpaceOMPL* MakeEmptySetSpace( RobotWorld *world){
      GeometricCSpaceOMPL *cspace = new GeometricCSpaceOMPLEmpty(world);
      cspace->SetCSpaceInput(input);
      cspace->Init();
      return cspace;
    }
    // CSpace  R^(N) + Time
    virtual GeometricCSpaceOMPL* MakeGeometricCSpaceRNTime( RobotWorld *world, int robot_idx, int dimension){
      GeometricCSpaceOMPL *cspace = new GeometricCSpaceOMPLRNTime(world, robot_idx, dimension);
      cspace->SetCSpaceInput(input);
      cspace->Init();
      return cspace;
    }
    // CSpace  R^(N)
    virtual GeometricCSpaceOMPL* MakeGeometricCSpaceFixedBase( RobotWorld *world, int robot_idx, int dimension=0){
      GeometricCSpaceOMPL *cspace = new GeometricCSpaceOMPLFixedBase(world, robot_idx);
      cspace->SetCSpaceInput(input);
      cspace->Init();
      return cspace;
    }
    virtual GeometricCSpaceOMPL* MakeGeometricCSpaceMobius( RobotWorld *world, int robot_idx)
    {
      GeometricCSpaceOMPL *cspace = new GeometricCSpaceOMPLMobius(world, robot_idx);
      cspace->SetCSpaceInput(input);
      cspace->Init();
      return cspace;
    }
    virtual GeometricCSpaceOMPL* MakeGeometricCSpaceAnnulus( RobotWorld *world, int robot_idx)
    {
      GeometricCSpaceOMPL *cspace = new GeometricCSpaceOMPLAnnulus(world, robot_idx);
      cspace->SetCSpaceInput(input);
      cspace->Init();
      return cspace;
    }
    virtual GeometricCSpaceOMPL* MakeGeometricCSpaceSolidTorus( RobotWorld *world, int robot_idx)
    {
      GeometricCSpaceOMPL *cspace = new GeometricCSpaceOMPLSolidTorus(world, robot_idx);
      cspace->SetCSpaceInput(input);
      cspace->Init();
      return cspace;
    }
    virtual GeometricCSpaceOMPL* MakeGeometricCSpaceSolidCylinder( RobotWorld *world, int robot_idx)
    {
      GeometricCSpaceOMPL *cspace = new GeometricCSpaceOMPLSolidCylinder(world, robot_idx);
      cspace->SetCSpaceInput(input);
      cspace->Init();
      return cspace;
    }
    virtual GeometricCSpaceOMPL* MakeGeometricCSpaceCircular( RobotWorld *world, int robot_idx)
    {
      GeometricCSpaceOMPL *cspace = new GeometricCSpaceOMPLCircular(world, robot_idx);
      cspace->SetCSpaceInput(input);
      cspace->Init();
      return cspace;
    }
};


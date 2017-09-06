#include "cspace.h"
#include "cspace_decorator.h"


class CSpaceFactory{
  private:
    PlannerInput input;
  public:
    CSpaceFactory(PlannerInput &input_): input(input_){};
    virtual KinodynamicCSpaceOMPL* MakeKinodynamicCSpace( Robot *robot, CSpace *kspace){
      KinodynamicCSpaceOMPL *cspace = new KinodynamicCSpaceOMPL(robot, kspace);
      cspace->SetPlannerInput(input);
      cspace->initSpace();
      cspace->initControlSpace();
      return cspace;
    }
    virtual GeometricCSpaceOMPL* MakeGeometricCSpace( Robot *robot, CSpace *kspace){
      GeometricCSpaceOMPL *cspace = new GeometricCSpaceOMPL(robot, kspace);
      cspace->SetPlannerInput(input);
      cspace->initSpace();
      cspace->initControlSpace();
      return cspace;
    }
    //CSpace for a rigid object which has rotational invariance. CSpace is R3
    virtual GeometricCSpaceOMPL* MakeGeometricCSpaceRotationalInvariance( Robot *robot, CSpace *inner){
      GeometricCSpaceOMPL *cspace = new GeometricCSpaceOMPLRotationalInvariance(robot, inner);
      cspace->SetPlannerInput(input);
      cspace->initSpace();
      cspace->initControlSpace();
      return cspace;
    }
    virtual CSpaceOMPL* MakeCSpaceDecoratorInnerOuter( CSpaceOMPL* cs, CSpace *outer){
      CSpaceOMPL *cspace = new CSpaceOMPLDecoratorInnerOuter(cs, outer);
      cspace->SetPlannerInput(input);
      cspace->initSpace();
      cspace->initControlSpace();
      return cspace;
    }
};


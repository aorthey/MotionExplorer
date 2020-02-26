#pragma once
#include "planner/cspace/cspace_geometric_Mobius.h"

class GeometricCSpaceOMPLCircular: public GeometricCSpaceOMPLMobius
{
  public:

    GeometricCSpaceOMPLCircular(RobotWorld *world_, int robot_idx);
    virtual void initSpace();
    virtual void ConfigToOMPLState(const Config &q, ob::State *qompl) override;
    virtual Config OMPLStateToConfig(const ob::State *qompl) override;
    virtual void print(std::ostream& out = std::cout) const override;
    virtual Vector3 getXYZ(const ob::State*) override;

    virtual bool IsPlanar() override;
    virtual void DrawGL(GUIState& state) override;

  protected:

    virtual double OMPLStateToSO2Value(const ob::State *qompl) override;
    Vector3 ProjectToVector3(double u);
    Config ProjectToConfig(double u);

  private:
    double radius_{1.2};
};


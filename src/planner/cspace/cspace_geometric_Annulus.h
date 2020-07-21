#pragma once
#include "planner/cspace/cspace_geometric.h"
#include "gui/gui_state.h"
#include <ompl/base/StateSampler.h>

class GeometricCSpaceOMPLAnnulus: public GeometricCSpaceOMPL
{
  using BaseT = GeometricCSpaceOMPL;
  public:
    virtual ob::SpaceInformationPtr SpaceInformationPtr() override;

    GeometricCSpaceOMPLAnnulus(RobotWorld *world_, int robot_idx);
    virtual void initSpace();
    virtual void ConfigToOMPLState(const Config &q, ob::State *qompl) override;
    virtual Config OMPLStateToConfig(const ob::State *qompl) override;
    virtual void print(std::ostream& out = std::cout) const override;
    virtual Vector3 getXYZ(const ob::State*) override;

    virtual bool IsPlanar() override;

    virtual void DrawGL(GUIState& state) override;

  protected:
    virtual double OMPLStateToRValue(const ob::State *qompl);
    virtual double OMPLStateToSO2Value(const ob::State *qompl);

    Vector3 ProjectToVector3(double u, double v);
    Config ProjectToConfig(double u, double v);
  private:
    ompl::base::StateSamplerPtr sampler_;

    double zOffset_{-2.0};
    double radiusOuter_{1.5};
    double radiusInner_{0.5};
};


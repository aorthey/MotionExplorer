#pragma once
#include "planner/cspace/cspace_geometric.h"
#include "gui/gui_state.h"

class GeometricCSpaceOMPLTorus: public GeometricCSpaceOMPL
{
  public:
    GeometricCSpaceOMPLTorus(RobotWorld *world_, int robot_idx);
    virtual void initSpace();

    virtual void ConfigToOMPLState(const Config &q, ob::State *qompl) override;
    virtual Config OMPLStateToConfig(const ob::State *qompl) override;

    virtual void print(std::ostream& out = std::cout) const override;
    virtual Vector3 getXYZ(const ob::State*) override;

    virtual bool IsPlanar() override;

    virtual void DrawGL(GUIState& state) override;

  protected:
    Config AnglesToConfig(double u, double v);

  private:
    double zOffset_{0.0};
    double distanceToCenter_{1.0};
    double radiusTorus_{0.6};
};


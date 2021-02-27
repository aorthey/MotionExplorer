#pragma once
#include "planner/cspace/cspace_geometric.h"
#include "gui/gui_state.h"

//TODO: IMPLEMENT UNIFORM SAMPLING
// https://stackoverflow.com/questions/26300510/generating-random-points-on-a-surface-of-an-n-dimensional-Sphere
//
// Based on pub "Random selection of points distributed on curved surfaces."
//
class GeometricCSpaceOMPLSphere: public GeometricCSpaceOMPL
{
  using BaseT = GeometricCSpaceOMPL;
  public:
    GeometricCSpaceOMPLSphere(RobotWorld *world_, int robot_idx);
    virtual void initSpace();

    // virtual ob::SpaceInformationPtr SpaceInformationPtr() override;

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
};


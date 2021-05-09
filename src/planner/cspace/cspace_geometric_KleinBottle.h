#pragma once
#include "planner/cspace/cspace_geometric_Mobius.h"
#include "gui/gui_state.h"

class GeometricCSpaceOMPLKleinBottle: public GeometricCSpaceOMPL
{
  public:
    GeometricCSpaceOMPLKleinBottle(RobotWorld *world_, int robot_idx);
    ~GeometricCSpaceOMPLKleinBottle();
    virtual void initSpace();
    virtual void ConfigToOMPLState(const Config &q, ob::State *qompl) override;
    virtual Config OMPLStateToConfig(const ob::State *qompl) override;
    virtual void print(std::ostream& out = std::cout) const override;
    virtual void DrawGL(GUIState& state) override;
    virtual Vector3 getXYZ(const ob::State*) override;

    virtual bool IsPlanar() override;
  protected:
    Vector3 ProjectToVector3(double u, double v);
    Config ProjectToConfig(double u, double v);
    void drawEdge(double u1, double v1, double u2, double v2);

    ob::State *stateTmp;
    ob::State *state1;
    ob::State *state2;
};


#include "planner/cspace/cspace_geometric.h"

class GeometricCSpaceOMPLMobius: public GeometricCSpaceOMPL
{
  public:
    GeometricCSpaceOMPLMobius(RobotWorld *world_, int robot_idx);
    virtual void initSpace();
    virtual void ConfigToOMPLState(const Config &q, ob::State *qompl) override;
    virtual Config OMPLStateToConfig(const ob::State *qompl) override;
    virtual void print(std::ostream& out = std::cout) const override;
    virtual Vector3 getXYZ(const ob::State*) override;

    virtual bool IsPlanar() override;

  protected:
    double OMPLStateToRValue(const ob::State *qompl);
    double OMPLStateToSO2Value(const ob::State *qompl);

    Vector3 ProjectToVector3(double u, double v);
    Config ProjectToConfig(double u, double v);
};


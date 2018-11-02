#pragma once
#include "forcefield.h"

class RadialForceField: public ForceField{
  public:
    RadialForceField(Math3D::Vector3 _source, double _power, double _radius);

    virtual Math3D::Vector3 getForce(const Math3D::Vector3& position, const Math3D::Vector3& velocity) override;
    virtual void Print(std::ostream &out) const override;
    virtual ForceFieldTypes type() override;
    virtual void DrawGL(GUIState &state) override;

    Math3D::Vector3 GetSource();
    double GetRadius();
    double GetPower();
  private:
    Math3D::Vector3 source;
    double power;
    double minimum_radius;
    double maximum_radius;
};

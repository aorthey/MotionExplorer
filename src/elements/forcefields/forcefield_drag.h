#pragma once
#include "forcefield.h"

class DragForceField: public ForceField{
  public:
    DragForceField(double viscosity_);
    virtual Math3D::Vector3 getForce(const Math3D::Vector3& position, const Math3D::Vector3& velocity);
    virtual void Print(std::ostream &out) const override;
    virtual ForceFieldTypes type();
  private:
    double viscosity;
};

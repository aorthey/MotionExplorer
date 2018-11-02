#pragma once
#include "forcefield.h"

class UniformForceField: public ForceField{
  public:
    UniformForceField(Math3D::Vector3 force_);
    virtual Math3D::Vector3 getForce(const Math3D::Vector3& position, const Math3D::Vector3& velocity) override;
    virtual void Print(std::ostream &out) const override;
    virtual ForceFieldTypes type() override;
    virtual void DrawGL(GUIState &state) override;
  private:
    Math3D::Vector3 force;
};



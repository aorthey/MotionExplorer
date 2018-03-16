#pragma once
#include "forcefield.h"

class CylindricalForceField: public ForceField{
  public:
    CylindricalForceField(Math3D::Vector3 _source, Math3D::Vector3 _direction, double _elongation, double _radius, double _power);

    virtual Math3D::Vector3 getForce(const Math3D::Vector3& position, const Math3D::Vector3& velocity) override;
    virtual void print() override;
    virtual ForceFieldTypes type() override;
    virtual void DrawGL(GUIState &state) override;

    Math3D::Vector3 GetSource();
    Math3D::Vector3 GetDirection();
    double GetElongation();
    double GetRadius();
    double GetPower();
  private:
    Math3D::Vector3 source, direction;
    double elongation;
    double minimum_radius;
    double maximum_radius;
    double power;
};


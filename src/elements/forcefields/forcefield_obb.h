#pragma once
#include "forcefield.h"

class OrientedBoundingBoxForceField: public ForceField{
  //extension: length along X,Y,Z axes
  public:
    OrientedBoundingBoxForceField(double power, Math3D::Vector3 _center, Math3D::Vector3 _direction, Math3D::Vector3 _extension);

    virtual Math3D::Vector3 getForce(const Math3D::Vector3& position, const Math3D::Vector3& velocity) override;
    virtual void print() override;
    virtual ForceFieldTypes type() override;
    virtual void DrawGL(GUIState &state) override;

    double GetPower();
    Math3D::Vector3 GetCenter();
    Math3D::Vector3 GetExtension();
    Math3D::Matrix3 GetRotation();
    bool IsInsideBox( const Math3D::Vector3 &position );
  private:
    Math3D::Vector3 center, force, extension;
    Math3D::Matrix3 R;
    double power;
};

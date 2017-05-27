#pragma once
#include <KrisLibrary/math3d/primitives.h>
#include <iostream>

enum ForceFieldTypes{ UNIFORM=0, RADIAL };

class ForceField{
  public:
    virtual Math3D::Vector3 getForceAtPosition(Math3D::Vector3 position) = 0;
    virtual void print() = 0;
    virtual ForceFieldTypes type() = 0;
};

class UniformForceField: public ForceField{
  public:
    UniformForceField(Math3D::Vector3 _force);
    virtual Math3D::Vector3 getForceAtPosition(Math3D::Vector3 position);
    virtual void print();
    virtual ForceFieldTypes type();
  private:
    Math3D::Vector3 force;
};

class RadialForceField: public ForceField{
  public:
    RadialForceField(Math3D::Vector3 _source, double _power, double _radius);
    virtual Math3D::Vector3 getForceAtPosition(Math3D::Vector3 position);
    virtual void print();
    virtual ForceFieldTypes type();
    Math3D::Vector3 GetSource();
    double GetRadius();
  private:
    Math3D::Vector3 source;
    double power;
    double minimum_radius;
    double maximum_radius;
};



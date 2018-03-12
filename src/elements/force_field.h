#pragma once
#include <KrisLibrary/math3d/primitives.h>
#include <KrisLibrary/math3d/rotationfit.h>
#include <KrisLibrary/GLdraw/GLColor.h>
#include <KrisLibrary/math/random.h>
#include <KrisLibrary/utils/SmartPointer.h>
#include <iostream>

enum ForceFieldTypes{ UNIFORM=0, RADIAL, CYLINDRICAL, UNIFORM_RANDOM, DRAG, GAUSSIAN_RANDOM, OBB};

class ForceField{
  public:
    virtual ~ForceField(){};
    virtual Math3D::Vector3 getForce(const Math3D::Vector3& position, const Math3D::Vector3& velocity = Math3D::Vector3(0,0,0)) = 0;
    virtual void print() = 0;
    virtual ForceFieldTypes type() = 0;

    GLDraw::GLColor GetColor(){
      return color;
    }
    void SetColor(GLDraw::GLColor &_color){
      color=_color;
    }
  protected:
    GLDraw::GLColor color;
};

class UniformForceField: public ForceField{
  public:
    UniformForceField(Math3D::Vector3 _force);
    virtual Math3D::Vector3 getForce(const Math3D::Vector3& position, const Math3D::Vector3& velocity);
    virtual void print();
    virtual ForceFieldTypes type();
  private:
    Math3D::Vector3 force;
};

class RadialForceField: public ForceField{
  public:
    RadialForceField(Math3D::Vector3 _source, double _power, double _radius);
    virtual Math3D::Vector3 getForce(const Math3D::Vector3& position, const Math3D::Vector3& velocity);
    virtual void print();
    virtual ForceFieldTypes type();
    Math3D::Vector3 GetSource();
    double GetRadius();
    double GetPower();
  private:
    Math3D::Vector3 source;
    double power;
    double minimum_radius;
    double maximum_radius;
};

class CylindricalForceField: public ForceField{
  public:
    CylindricalForceField(Math3D::Vector3 _source, Math3D::Vector3 _direction, double _elongation, double _radius, double _power);

    virtual Math3D::Vector3 getForce(const Math3D::Vector3& position, const Math3D::Vector3& velocity);
    virtual void print();
    virtual ForceFieldTypes type();
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

class UniformRandomForceField: public ForceField{
  public:
    UniformRandomForceField(Math3D::Vector3 _minforce, Math3D::Vector3 _maxforce);
    virtual Math3D::Vector3 getForce(const Math3D::Vector3& position, const Math3D::Vector3& velocity);
    virtual void print();
    virtual ForceFieldTypes type();
  private:
    Math3D::Vector3 minforce, maxforce;
};

class GaussianRandomForceField: public ForceField{
  public:
    GaussianRandomForceField(Math3D::Vector3 _mean, Math3D::Vector3 _std);
    virtual Math3D::Vector3 getForce(const Math3D::Vector3& position, const Math3D::Vector3& velocity);
    virtual void print();
    virtual ForceFieldTypes type();
  private:
    Math3D::Vector3 mean, stddeviation;
};

class OrientedBoundingBoxForceField: public ForceField{
  //extension: length along X,Y,Z axes
  public:
    OrientedBoundingBoxForceField(double power, Math3D::Vector3 _center, Math3D::Vector3 _direction, Math3D::Vector3 _extension);
    virtual Math3D::Vector3 getForce(const Math3D::Vector3& position, const Math3D::Vector3& velocity);
    virtual void print();
    virtual ForceFieldTypes type();
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
class DragForceField: public ForceField{
  public:
    DragForceField(double viscosity_);
    virtual Math3D::Vector3 getForce(const Math3D::Vector3& position, const Math3D::Vector3& velocity);
    virtual void print();
    virtual ForceFieldTypes type();
  private:
    double viscosity;
};
typedef SmartPointer<ForceField> ForceFieldPtr;

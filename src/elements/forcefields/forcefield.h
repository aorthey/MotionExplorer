#pragma once
#include "gui/gui_state.h"
#include "gui/colors.h"
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
    virtual void DrawGL(GUIState &state);
    GLDraw::GLColor cForce{grey};
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

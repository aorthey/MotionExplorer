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

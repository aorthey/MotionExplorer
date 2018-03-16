#pragma once
#include <KrisLibrary/math3d/primitives.h>
#include <KrisLibrary/utils/SmartPointer.h>
#include <KrisLibrary/GLdraw/GLColor.h>
#include <vector>
#include <memory>
#include <iostream>
#include "elements/forcefields/forcefield.h"

class TiXmlElement;

//a wrench (force/torque) \in \R^6 acting at a point on \R^3
struct Wrench{
  Math3D::Vector3 force;
  Math3D::Vector3 torque;
  Math3D::Vector3 position;
};
struct Momentum{
  Math3D::Vector3 linear;
  Math3D::Vector3 angular;
};
struct COM{
  Wrench wrench;
  Momentum momentum;
};

class WrenchField{

  private:
    std::vector<SmartPointer<ForceField> > forcefields;

    //wrench induced by force field on each link of the main robot
    std::vector<Wrench> wrench_per_link;

    COM com;

  public: 

    void init(uint Nlinks);
    WrenchField();

    bool Load( const char * file);
    bool Load( TiXmlElement *node );

    Math3D::Vector3 getForce(const Math3D::Vector3 &position, const Math3D::Vector3 &velocity = Math3D::Vector3(0,0,0));

    //TODO: force at origin induced by force at position 
    Math3D::Vector3 getTorqueFieldAtPosition(Math3D::Vector3 &position, Math3D::Vector3 &origin);

    const std::vector<SmartPointer<ForceField> >& GetForceFields() const;

    uint size();

    void setForce( uint id, Math3D::Vector3 force);
    void setTorque( uint id, Math3D::Vector3 torque);
    void setPosition( uint id, Math3D::Vector3 position);
    Math3D::Vector3 getForce( uint id );
    Math3D::Vector3 getTorque( uint id );
    Math3D::Vector3 getPosition( uint id );

    void setCOMLinearMomentum( Math3D::Vector3 linearmomentum);
    void setCOMAngularMomentum( Math3D::Vector3 angularmomentum);
    void setCOMForce( Math3D::Vector3 force);
    void setCOMTorque( Math3D::Vector3 torque);
    void setCOMPosition( Math3D::Vector3 position);

    Math3D::Vector3 getCOMLinearMomentum();
    Math3D::Vector3 getCOMAngularMomentum();
    Math3D::Vector3 getCOMForce();
    Math3D::Vector3 getCOMTorque();
    Math3D::Vector3 getCOMPosition();

    void print();
    void DrawGL(GUIState &state);
};


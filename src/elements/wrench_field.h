#pragma once
#include <KrisLibrary/math3d/primitives.h>
#include <KrisLibrary/utils/SmartPointer.h>
#include <vector>
#include <memory>
#include <iostream>
#include "elements/force_field.h"

class TiXmlElement;

//a wrench (force/torque) \in \R^6 acting at a point on \R^3
struct Wrench{
  Math3D::Vector3 torque;
  Math3D::Vector3 force;
  Math3D::Vector3 position;
};

class WrenchField{

  private:

    std::vector<SmartPointer<ForceField> > forcefields;

    //wrench induced by force field on each link of the main robot
    std::vector<Wrench> wrench_per_link;

  public: 
    void init(uint Nlinks);
    WrenchField();

    bool LoadFromWorldFile( const char * file);
    bool Load( TiXmlElement *node );

    Math3D::Vector3 getForceFieldAtPosition(Math3D::Vector3 &position);

    uint size();

    void setForce( uint id, Math3D::Vector3 force);
    void setTorque( uint id, Math3D::Vector3 torque);
    void setPosition( uint id, Math3D::Vector3 position);

    Math3D::Vector3 getForce( uint id );
    Math3D::Vector3 getTorque( uint id );
    Math3D::Vector3 getPosition( uint id );

    void print();
};


#pragma once
#include <KrisLibrary/math3d/primitives.h>
#include <vector>

class TiXmlElement;

//a wrench (force/torque) acting at a point on R^3
struct Wrench{
  Math3D::Vector3 torque;
  Math3D::Vector3 force;
  Math3D::Vector3 position;
};

class WrenchField{

  private:

    //uniform forces in workspace
    Math3D::Vector3 Funiform;
    //non-uniform forces in workspace
    std::vector<double> Fx, Fy, Fz;
    //discretization of non-uniform force field
    double dx, dy, dz;
    //lower limit of non-uniform force field
    double Lx, Ly, Lz;
    //upper limit of non-uniform force field
    double Ux, Uy, Uz;

    //wrench induced by force field on each link of the main robot
    std::vector<Wrench> wrench_per_link;

    //scaling of forces for visualization purposes
    double forceScale = 0.1;

  public: 
    void init(uint Nlinks);
    WrenchField();

    bool LoadFromWorldFile( const char * file);
    bool Load( TiXmlElement *node );

    Math3D::Vector3 getLowerLimit();
    Math3D::Vector3 getUpperLimit();
    Math3D::Vector3 getStepSize();
    double getForceOneDimensional(std::vector<double> F, double r, double l, double u, double dr);
    Math3D::Vector3 getForceFieldAtPosition(Math3D::Vector3 &xyz);
    uint size();
    void setForce( uint id, Math3D::Vector3 force);
    void setTorque( uint id, Math3D::Vector3 torque);
    void setPosition( uint id, Math3D::Vector3 position);
    Math3D::Vector3 getForce( uint id );
    Math3D::Vector3 getForceVisualization( uint id );
    Math3D::Vector3 getTorque( uint id );
    Math3D::Vector3 getPosition( uint id );
};


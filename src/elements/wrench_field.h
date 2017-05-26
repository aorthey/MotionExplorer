#pragma once
#include <KrisLibrary/math3d/primitives.h>

//a wrench (force/torque) acting at a point on R^3
struct Wrench{
  Vector3 torque;
  Vector3 force;
  Vector3 position;
};

class WrenchField{

  private:

    //uniform forces in workspace
    Vector3 Funiform;
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
    void init(uint Nlinks){
      wrench_per_link.clear();
      wrench_per_link.resize(Nlinks);
    }

    WrenchField(){
      Funiform[0]=0;
      Funiform[1]=0;
      Funiform[2]=-9.81; //earth gravity
      //Funiform[2]=-3.711; //mars gravity
      //Funiform[2]=-1.315; //europa gravity

      Lx = -4;
      Ly = -4;
      Lz = -1;
      Ux = 4;
      Uy = 4;
      Uz = 1;
      double dstep = 0.5;
      dx = dstep; dy = dstep; dz = dstep;

      Fx.clear();Fy.clear();Fz.clear();

      double x = Lx;
      while(x <= Ux){
        Fx.push_back(0);
        x+=dstep;
      }
      double y = Ly;
      while(y <= Uy){
        Fy.push_back(0);
        y+=dstep;
      }
      double z = Lz;
      while(z <= Uz){
        Fz.push_back(-2*Funiform[2]);
        z+=dstep;
      }
    }
    Vector3 getLowerLimit(){
      Vector3 L;L[0]=Lx;L[1]=Ly;L[2]=Lz;
      return L;
    }
    Vector3 getUpperLimit(){
      Vector3 U;U[0]=Ux;U[1]=Uy;U[2]=Uz;
      return U;
    }
    Vector3 getStepSize(){
      Vector3 S;S[0]=dx;S[1]=dy;S[2]=dz;
      return S;
    }

    double getForceOneDimensional(std::vector<double> F, double r, double l, double u, double dr){
      if(r < l) return 0;
      if(r > u) return 0;
      uint kr = floor((r-l)/dr);
      return F.at(kr);
    }

    Vector3 getForceFieldAtPosition(Vector3 &xyz){

      //check limits
      double x = xyz[0];
      double y = xyz[1];
      double z = xyz[2];

      Vector3 force;

      force[0] = getForceOneDimensional(Fx, x, Lx, Ux, dx);
      force[1] = getForceOneDimensional(Fy, y, Ly, Uy, dy);
      force[2] = getForceOneDimensional(Fz, z, Lz, Uz, dz);

      return force+Funiform;
    }

    uint size(){ 
      return wrench_per_link.size(); 
    }

    void setForce( uint id, Vector3 force){
      wrench_per_link.at(id).force = force;
    }

    void setTorque( uint id, Vector3 torque){
      wrench_per_link.at(id).torque = torque;
    }

    void setPosition( uint id, Vector3 position){
      wrench_per_link.at(id).position = position;
    }

    Vector3 getForce( uint id ){ return wrench_per_link.at(id).force; }
    Vector3 getForceVisualization( uint id ){ return forceScale*wrench_per_link.at(id).force; }
    Vector3 getTorque( uint id ){ return wrench_per_link.at(id).torque; }
    Vector3 getPosition( uint id ){ return wrench_per_link.at(id).position; }
};


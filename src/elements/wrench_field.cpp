#include "loader.h"
#include "wrench_field.h"

using namespace Math3D;

bool WrenchField::LoadFromWorldFile(const char* file)
{
  TiXmlDocument doc(file);
  TiXmlElement *root = GetRootNodeFromDocument(doc);
  return Load(root);
}

bool WrenchField::Load(TiXmlElement *node)
{
  CheckNodeName(node, "world");

  TiXmlElement* forcefieldsettings = FindSubNode(node, "forcefield");

  if(!forcefieldsettings){
    std::cout << "world xml file has no forcefieldsettings" << std::endl;
    return false;
  }

  TiXmlElement* forceuniform = FindSubNode(forcefieldsettings, "uniform");
  TiXmlElement* forceradial = FindSubNode(forcefieldsettings, "radial");

  GetStreamAttribute(forceuniform,"force") >> Funiform;
  //GetStreamAttribute(forceradial,"source") >> pin.q_goal;

  return true;
}

void WrenchField::init(uint Nlinks){
  wrench_per_link.clear();
  wrench_per_link.resize(Nlinks);
}

WrenchField::WrenchField(){
  Funiform[0]=0;
  Funiform[1]=0;
  Funiform[2]=-9.81; //earth gravity
  //Funiform[2]=-3.711; //mars gravity
  //Funiform[2]=-1.315; //europa gravity

  //Lx = -4;
  //Ly = -4;
  //Lz = -1;
  //Ux = 4;
  //Uy = 4;
  //Uz = 1;
  //double dstep = 0.5;
  //dx = dstep; dy = dstep; dz = dstep;

  Fx.clear();Fy.clear();Fz.clear();

  //double x = Lx;
  //while(x <= Ux){
  //  Fx.push_back(0);
  //  x+=dstep;
  //}
  //double y = Ly;
  //while(y <= Uy){
  //  Fy.push_back(0);
  //  y+=dstep;
  //}
  //double z = Lz;
  //while(z <= Uz){
  //  Fz.push_back(-2*Funiform[2]);
  //  z+=dstep;
  //}
}
Vector3 WrenchField::getLowerLimit(){
  Vector3 L;L[0]=Lx;L[1]=Ly;L[2]=Lz;
  return L;
}
Vector3 WrenchField::getUpperLimit(){
  Vector3 U;U[0]=Ux;U[1]=Uy;U[2]=Uz;
  return U;
}
Vector3 WrenchField::getStepSize(){
  Vector3 S;S[0]=dx;S[1]=dy;S[2]=dz;
  return S;
}

double WrenchField::getForceOneDimensional(std::vector<double> F, double r, double l, double u, double dr){
  if(r < l) return 0;
  if(r > u) return 0;
  uint kr = floor((r-l)/dr);
  return F.at(kr);
}

Vector3 WrenchField::getForceFieldAtPosition(Vector3 &xyz){

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

uint WrenchField::size(){ 
  return wrench_per_link.size(); 
}

void WrenchField::setForce( uint id, Vector3 force){
  wrench_per_link.at(id).force = force;
}

void WrenchField::setTorque( uint id, Vector3 torque){
  wrench_per_link.at(id).torque = torque;
}

void WrenchField::setPosition( uint id, Vector3 position){
  wrench_per_link.at(id).position = position;
}

Vector3 WrenchField::getForce( uint id ){ 
  return wrench_per_link.at(id).force; 
}
Vector3 WrenchField::getForceVisualization( uint id ){ 
  return forceScale*wrench_per_link.at(id).force; 
}
Vector3 WrenchField::getTorque( uint id ){ 
  return wrench_per_link.at(id).torque; 
}
Vector3 WrenchField::getPosition( uint id ){ 
  return wrench_per_link.at(id).position; 
}

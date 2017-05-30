#include <KrisLibrary/GLdraw/GLColor.h>
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
  if(!CheckNodeName(node, "world")) return false;

  TiXmlElement* forcefieldsettings = FindSubNode(node, "forcefield");

  forcefields.clear();

  if(!forcefieldsettings){
    std::cout << "world xml file has no forcefield" << std::endl;
    std::cout << " -- setting earth gravity" << std::endl;
    SmartPointer<ForceField> f(new UniformForceField(Vector3(0,0,-9.81)));
    forcefields.push_back(f);
    return false;
  }
  
  //############################################################################
  //Uniform Force Fields: If there are multiple fields, we just add them all
  //into a single one
  //############################################################################
  TiXmlElement* forceuniform = FindSubNode(forcefieldsettings, "uniform");
  Vector3 Funiform;
  GetStreamAttribute(forceuniform,"force") >> Funiform;

  using namespace GLDraw;
  GLColor colorForce(0.8,0.8,0.8);
  while(forceuniform){
    forceuniform = FindNextSiblingNode(forcefieldsettings, "uniform");
    Vector3 FuniformNext;
    GetStreamAttribute(forceuniform,"force") >> FuniformNext;
    stringstream ssc = GetStreamAttribute(forceuniform,"color");
    Vector3 cc;
    if(ssc.str()!="NONE"){
      ssc >> cc;
      colorForce[0]=cc[0]; colorForce[1]=cc[1]; colorForce[2]=cc[2];
    }
    Funiform += FuniformNext;
  }


  SmartPointer<ForceField> fu(new UniformForceField(Funiform));
  fu->SetColor(colorForce);

  forcefields.push_back(fu);

  //############################################################################
  //Radial Force Fields
  //############################################################################
  TiXmlElement* forceradial = FindSubNode(forcefieldsettings, "radial");

  while(forceradial!=NULL){
    Vector3 source;
    double power, radius;
    GLColor colorForce(0.8,0.8,0.8);
    GetStreamAttribute(forceradial,"source") >> source;
    GetStreamAttribute(forceradial,"power") >> power;
    GetStreamAttribute(forceradial,"radius") >> radius;
    stringstream ssc = GetStreamAttribute(forceradial,"color");
    Vector3 cc;
    if(ssc.str()!="NONE"){
      ssc >> cc;
      colorForce[0]=cc[0]; colorForce[1]=cc[1]; colorForce[2]=cc[2];
    }

    SmartPointer<ForceField> fr(new RadialForceField(source, power, radius));
    fr->SetColor(colorForce);
    forcefields.push_back(fr);

    forceradial = FindNextSiblingNode(forceradial, "radial");
  }
  //############################################################################
  //Uniform Random Force Fields
  //############################################################################
  TiXmlElement* forcerandom = FindSubNode(forcefieldsettings, "uniformrandom");

  while(forcerandom!=NULL){
    Vector3 minforce, maxforce;
    GLColor colorForce(0.8,0.8,0.8);
    GetStreamAttribute(forcerandom,"minforce") >> minforce;
    GetStreamAttribute(forcerandom,"maxforce") >> maxforce;
    stringstream ssc = GetStreamAttribute(forcerandom,"color");
    Vector3 cc;
    if(ssc.str()!="NONE"){
      ssc >> cc;
      colorForce[0]=cc[0]; colorForce[1]=cc[1]; colorForce[2]=cc[2];
    }

    SmartPointer<ForceField> fr(new UniformRandomForceField(minforce, maxforce));
    fr->SetColor(colorForce);
    forcefields.push_back(fr);

    forcerandom = FindNextSiblingNode(forcerandom, "uniformrandom");
  }
  //############################################################################
  //Gaussian Random Force Fields
  //############################################################################
  forcerandom = FindSubNode(forcefieldsettings, "gaussianrandom");

  while(forcerandom!=NULL){
    Vector3 mean, stddev;
    GLColor colorForce(0.8,0.8,0.8);
    GetStreamAttribute(forcerandom,"mean") >> mean;
    GetStreamAttribute(forcerandom,"stddeviation") >> stddev;
    stringstream ssc = GetStreamAttribute(forcerandom,"color");
    Vector3 cc;
    if(ssc.str()!="NONE"){
      ssc >> cc;
      colorForce[0]=cc[0]; colorForce[1]=cc[1]; colorForce[2]=cc[2];
    }

    SmartPointer<ForceField> fg(new GaussianRandomForceField(mean, stddev));
    fg->SetColor(colorForce);
    forcefields.push_back(fg);

    forcerandom = FindNextSiblingNode(forcerandom, "gaussianrandom");
  }
  //############################################################################
  //Bounding Box Force Fields
  //############################################################################
  TiXmlElement* forcebox = FindSubNode(forcefieldsettings, "orientedbox");

  while(forcebox!=NULL){
    double power;
    Vector3 center, direction, extension;
    GLColor colorForce(0.8,0.8,0.8);
    GetStreamAttribute(forcebox,"power") >> power;
    GetStreamAttribute(forcebox,"center") >> center;
    GetStreamAttribute(forcebox,"direction") >> direction;
    GetStreamAttribute(forcebox,"extension") >> extension;
    stringstream ssc = GetStreamAttribute(forcebox,"color");
    Vector3 cc;
    if(ssc.str()!="NONE"){
      ssc >> cc;
      colorForce[0]=cc[0]; colorForce[1]=cc[1]; colorForce[2]=cc[2];
    }

    SmartPointer<ForceField> fb(new OrientedBoundingBoxForceField(power, center, direction, extension));
    fb->SetColor(colorForce);
    forcefields.push_back(fb);

    forcebox = FindNextSiblingNode(forcebox, "orientedbox");
  }

  return true;
}

void WrenchField::init(uint Nlinks){
  wrench_per_link.clear();
  wrench_per_link.resize(Nlinks);
}

WrenchField::WrenchField(){
  forcefields.clear();
}

const std::vector<SmartPointer<ForceField> >& WrenchField::GetForceFields() const{
  return forcefields;
}

Vector3 WrenchField::getForceFieldAtPosition(Vector3 &position){

  Vector3 F(0,0,0);
  for(int i = 0; i < forcefields.size(); i++){
    F += forcefields.at(i)->getForceAtPosition(position);
  }

  return F;
}
Vector3 WrenchField::getTorqueFieldAtPosition(Vector3 &position, Vector3 &origin){

  Vector3 T(0,0,0);
  return T;
}

uint WrenchField::size(){ 
  return wrench_per_link.size(); 
}

//############################################################################
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
Vector3 WrenchField::getTorque( uint id ){ 
  return wrench_per_link.at(id).torque; 
}
Vector3 WrenchField::getPosition( uint id ){ 
  return wrench_per_link.at(id).position; 
}

//############################################################################

void WrenchField::setCOMLinearMomentum( Math3D::Vector3 linearmomentum){
  com.momentum.linear = linearmomentum;
}
void WrenchField::setCOMAngularMomentum( Math3D::Vector3 angularmomentum){
  com.momentum.angular = angularmomentum;
}
void WrenchField::setCOMForce(Vector3 force){
  com.wrench.force = force;
}
void WrenchField::setCOMTorque(Vector3 torque){
  com.wrench.torque = torque;
}
void WrenchField::setCOMPosition(Vector3 position){
  com.wrench.position = position;
}

//############################################################################
Vector3 WrenchField::getCOMLinearMomentum(){ 
  return com.momentum.linear; 
}
Vector3 WrenchField::getCOMAngularMomentum(){ 
  return com.momentum.angular; 
}
Vector3 WrenchField::getCOMForce(){ 
  return com.wrench.force; 
}
Vector3 WrenchField::getCOMTorque(){ 
  return com.wrench.torque; 
}
Vector3 WrenchField::getCOMPosition(){ 
  return com.wrench.position; 
}

//############################################################################

void WrenchField::print(){
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "WrenchField" << std::endl;
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "Force Fields" << std::endl;
  for(int i = 0; i < forcefields.size(); i++){
    forcefields.at(i)->print();
  }
  std::cout << std::string(80, '-') << std::endl;

}

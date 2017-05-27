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
  while(forceuniform){
    forceuniform = FindNextSiblingNode(forcefieldsettings, "uniform");
    Vector3 FuniformNext;
    GetStreamAttribute(forceuniform,"force") >> FuniformNext;
    Funiform += FuniformNext;
  }

  SmartPointer<ForceField> fu(new UniformForceField(Funiform));
  forcefields.push_back(fu);

  //############################################################################
  //Radial Force Fields
  //############################################################################
  TiXmlElement* forceradial = FindSubNode(forcefieldsettings, "radial");

  while(forceradial!=NULL){
    Vector3 source;
    double power, radius;
    GetStreamAttribute(forceradial,"source") >> source;
    GetStreamAttribute(forceradial,"power") >> power;
    GetStreamAttribute(forceradial,"radius") >> radius;

    SmartPointer<ForceField> fr(new RadialForceField(source, power, radius));
    forcefields.push_back(fr);

    forceradial = FindNextSiblingNode(forceradial, "radial");
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
Vector3 WrenchField::getTorque( uint id ){ 
  return wrench_per_link.at(id).torque; 
}
Vector3 WrenchField::getPosition( uint id ){ 
  return wrench_per_link.at(id).position; 
}
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

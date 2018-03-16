#include <KrisLibrary/GLdraw/GLColor.h>
#include "file_io.h"
#include "wrench_field.h"
#include "elements/forcefields/forcefield_uniform.h"
#include "elements/forcefields/forcefield_radial.h"
#include "elements/forcefields/forcefield_random.h"
#include "elements/forcefields/forcefield_cylindrical.h"

using namespace Math3D;
using namespace GLDraw;

bool WrenchField::Load(const char* file)
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
    ForceFieldPtr f(new UniformForceField(Vector3(0,0,-9.81)));
    forcefields.push_back(f);
    return false;
  }
  
  //############################################################################
  //Uniform Force Fields: If there are multiple fields, we just add them all
  //into a single one
  //############################################################################
  TiXmlElement* forceuniform = FindSubNode(forcefieldsettings, "uniform");

  Vector3 Funiform(0,0,0);
  while(forceuniform){
    Funiform += GetAttribute<Vector3>(forceuniform, "force");
    forceuniform = FindNextSiblingNode(forcefieldsettings);
  }

  ForceFieldPtr fu(new UniformForceField(Funiform));
  forcefields.push_back(fu);

  //############################################################################
  //Radial Force Fields
  //############################################################################
  TiXmlElement* forceradial = FindSubNode(forcefieldsettings, "radial");

  while(forceradial){
    Vector3 source = GetAttribute<Vector3>(forceradial,"source");
    double power = GetAttribute<double>(forceradial,"power");
    double radius = GetAttribute<double>(forceradial,"radius");
    Vector3 cc = GetAttributeDefault<Vector3>(forceradial,"color",Vector3(0.5,0.5,0.5));
    GLColor colorForce(cc[0],cc[1],cc[2]);

    ForceFieldPtr fr(new RadialForceField(source, power, radius));
    fr->cForce = colorForce;
    forcefields.push_back(fr);

    forceradial = FindNextSiblingNode(forceradial);
  }
  //############################################################################
  //Drag Force Field
  //############################################################################
  TiXmlElement* forcedrag = FindSubNode(forcefieldsettings, "drag");

  if(forcedrag){
    double viscosity = GetAttribute<double>(forcedrag, "viscosity");
    if(viscosity==0){
      std::cout << "viscosity needs to be non-zero." << std::endl;
      exit(1);
    }
    ForceFieldPtr fd(new DragForceField(viscosity));
    forcefields.push_back(fd);
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
    fr->cForce = colorForce;
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
    fg->cForce = colorForce;
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
    fb->cForce = colorForce;
    forcefields.push_back(fb);

    forcebox = FindNextSiblingNode(forcebox, "orientedbox");
  }
  //############################################################################
  //Cylindrical Force Fields
  //############################################################################
  TiXmlElement* field = FindSubNode(forcefieldsettings, "cylindrical");

    //<cylindrical source="3 3 0" direction="0.2 0.2 1.0" elongation="3" power="2" color="0.3 0.3 0.7"/>
  while(field!=NULL){
    double elongation, radius, power;
    Vector3 source, direction;
    GLColor colorForce(0.8,0.8,0.8);

    GetStreamAttribute(field,"source") >> source;
    GetStreamAttribute(field,"direction") >> direction;
    GetStreamAttribute(field,"elongation") >> elongation;
    GetStreamAttribute(field,"radius") >> radius;
    GetStreamAttribute(field,"power") >> power;

    stringstream ssc = GetStreamAttribute(field,"color");
    Vector3 cc;
    if(ssc.str()!="NONE"){
      ssc >> cc;
      colorForce[0]=cc[0]; colorForce[1]=cc[1]; colorForce[2]=cc[2];
    }

    SmartPointer<ForceField> fb(new CylindricalForceField(source, direction, elongation, radius, power));
    fb->cForce = colorForce;
    forcefields.push_back(fb);

    field = FindNextSiblingNode(field, "cylindrical");
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

Vector3 WrenchField::getForce(const Vector3 &position, const Vector3 &velocity){

  Vector3 F(0,0,0);
  for(uint i = 0; i < forcefields.size(); i++){
    F += forcefields.at(i)->getForce(position, velocity);
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

void WrenchField::DrawGL(GUIState &state)
{
  for(uint k = 0; k < forcefields.size(); k++){
    forcefields.at(k)->DrawGL(state);
  }
}
void WrenchField::print(){
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "Force Fields" << std::endl;
  std::cout << std::string(80, '-') << std::endl;
  for(uint i = 0; i < forcefields.size(); i++){
    forcefields.at(i)->print();
  }
  std::cout << std::string(80, '-') << std::endl;

}

#include "forcefield_uniform.h"

UniformForceField::UniformForceField(Math3D::Vector3 force_):
  force(force_)
{
}
Math3D::Vector3 UniformForceField::getForce(const Math3D::Vector3& position, const Math3D::Vector3& velocity)
{
  return force;
}
void UniformForceField::print()
{
  std::cout << "UniformForceField : force " << force << std::endl;
}
ForceFieldTypes UniformForceField::type()
{
  return UNIFORM;
}
void UniformForceField::DrawGL(GUIState &state)
{
  // double step = 0.5;
  // Real length = step/6;
  // Real linewidth=0.01;
  // Vector3 dir(1,1,0);
  // GLColor cForce(1,1,1,0.6);
  // for(double x = -3; x < 3; x+=step){
  //   for(double y = -3; y < 3; y+=step){
  //     for(double z = 0.2; z <= 2; z+=step){
  //       Vector3 pos(x,y,z);

  //       //glDisable(GL_LIGHTING);
  //       //glEnable(GL_BLEND); 
  //       //glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

  //       glPushMatrix();
  //       setColor(cForce);

  //       glTranslate(pos);
  //       //glMaterialfv(GL_FRONT,GL_AMBIENT_AND_DIFFUSE,cForce);
  //       drawCylinder(dir*length,linewidth);

  //       glPushMatrix();

  //       //glMaterialfv(GL_FRONT,GL_AMBIENT_AND_DIFFUSE,cForce);
  //       Real arrowLen = 3*linewidth;
  //       Real arrowWidth = 1*linewidth;
  //       glTranslate(dir*length);
  //       drawCone(dir*arrowLen,arrowWidth,8);

  //       glPopMatrix();
  //       glPopMatrix();
  //       //glEnable(GL_LIGHTING);
  //     }//forz
  //   }//fory
  // }//forx
}


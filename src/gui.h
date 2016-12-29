#pragma once
#include <Interface/SimTestGUI.h>
#include <ode/ode.h>


class ForceFieldGUI : public SimTestBackend
{
  //class GLUISimTestGUI : public GLScreenshotProgram<GLUIGUI>

  public:
  ForceFieldGUI(RobotWorld *world)
      : SimTestBackend(world)
  {
  }

  virtual void RenderWorld()
  {
    BaseT::RenderWorld();
    double step = 0.5;
    Real length = step/6;
    Real linewidth=0.01;
    Vector3 dir(1,1,0);
    GLColor cForce(1,1,1,0.6);
    for(double x = -3; x < 3; x+=step){
      for(double y = -3; y < 3; y+=step){
        for(double z = 0.2; z <= 2; z+=step){
          Vector3 pos(x,y,z);
  
          glDisable(GL_LIGHTING);
  
          glEnable(GL_BLEND); 
          glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
          cForce.setCurrentGL();
  
          glPushMatrix();
  
          glTranslate(pos);
          //glMaterialfv(GL_FRONT,GL_AMBIENT_AND_DIFFUSE,cForce);
          drawCylinder(dir*length,linewidth);
  
          glPushMatrix();
  
          //glMaterialfv(GL_FRONT,GL_AMBIENT_AND_DIFFUSE,cForce);
          Real arrowLen = 3*linewidth;
          Real arrowWidth = 1*linewidth;
          glTranslate(dir*length);
          drawCone(dir*arrowLen,arrowWidth,8);
  
          glPopMatrix();
          glPopMatrix();
        }
      }
    }
  }

  virtual bool OnCommand(const string& cmd,const string& args){
    if(cmd=="advance") {
      ODERobot *robot = sim.odesim.robot(0);
      std::cout << "Force" << std::endl;
      std::cout << robot->robot.name << std::endl;
      double px,py,pz;
      double fx,fy,fz;
      fx = 2.0;
      fy = 0.0;
      fz = 0.0;
      //dBodyAddForceAtPos(robot->body(0),fx,fy,fz,px,py,pz);
    }
    return BaseT::OnCommand(cmd,args);
  }

};


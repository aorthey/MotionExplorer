#pragma once
#include <Interface/SimTestGUI.h>

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
  
    for(int x = -3; x < 3; x++){
      for(int y = -3; y < 3; y++){
        for(double z = 0.2; z <= 2.5; z+=0.5){
          Vector3 pos(x,y,z);
          Vector3 dir(1,1,0);
          Real length = 0.2;
          Real linewidth=0.01;
          GLColor cForce(1,1,1,0.6);
  
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
    }
    return BaseT::OnCommand(cmd,args);
  }

};


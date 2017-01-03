#pragma once
#include <Interface/SimTestGUI.h>
#include <ode/ode.h>


const GLColor bodyColor(0.9,0.9,0.9);
const GLColor selectedLinkColor(0.9,0.2,0.2);

class ForceFieldBackend : public SimTestBackend
{
  private:
    bool drawForceField;
    bool drawExtras; 

  public:
  typedef SimTestBackend BaseT; //Need to parse it through SimTest to get wrenchies

  ForceFieldBackend(RobotWorld *world)
      : SimTestBackend(world)
  {
    drawForceField = false;
    drawExtras = true;
  }

  virtual void RenderWorld()
  {
    if(drawExtras){
      vector<ViewRobot> viewRobots = world->robotViews;
      ViewRobot *viewRobot = &viewRobots[0];
      double COMradius = 0.05;
      double frameLength = 0.2;
      viewRobot->DrawCenterOfMass(COMradius);
      viewRobot->DrawLinkFrames(frameLength);
      viewRobot->DrawLinkSkeleton();
      viewRobot->SetColors(bodyColor);
      //std::cout << cur_driver << std::endl;
      //std::cout << cur_link << std::endl;
      //if(cur_driver>-1){
        //viewRobot->SetColor(cur_driver, selectedLinkColor);
      //}
    }

    if(drawForceField){
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
            glEnable(GL_LIGHTING);
          }//forz
        }//fory
      }//forx
    }//if(drawforcefield)
    BaseT::RenderWorld();
  }//RenderWorld

  virtual bool OnCommand(const string& cmd,const string& args){
    BaseT::OnCommand(cmd,args);
    if(cmd=="advance") {
      ODERobot *robot = sim.odesim.robot(0);
      std::cout << "Force" << std::endl;
      std::cout << robot->robot.name << std::endl;

      Config q;
      robot->GetConfig(q);
      double px,py,pz;
      double fx,fy,fz;
      fx = 10.0;
      fy = 0.0;
      fz = 10.0;
      px = 0.0;
      py = 0.0;
      pz = 0.0;
      dBodyAddForceAtPos(robot->body(7),fx,fy,fz,px,py,pz);
    }
  }


};


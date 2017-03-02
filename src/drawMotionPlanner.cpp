#include "drawMotionPlanner.h"

namespace GLDraw{
  void drawRobotExtras(ViewRobot *robot, GLColor bodyColor, double COMradius)
  {
    robot->DrawCenterOfMass(COMradius);
    robot->DrawLinkSkeleton();
    robot->SetColors(bodyColor);
  }
  void drawIKextras(ViewRobot *viewRobot, Robot *robot, std::vector<IKGoal> &constraints, std::vector<int> linksInCollision, GLColor selectedLinkColor)
  {
    for(uint i = 0; i < constraints.size(); i++){

      const IKGoal goal = constraints[i];

      viewRobot->SetColor(goal.link, selectedLinkColor);
      ViewIKGoal viewik = ViewIKGoal();
      viewik.Draw(goal, *robot);
      viewik.DrawLink(goal, *viewRobot);

    }
    for(uint i = 0; i < linksInCollision.size(); i++){
      glDisable(GL_LIGHTING);
      glEnable(GL_BLEND);
      glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
      glPushMatrix();
      //glMultMatrix(Matrix4(T));
      viewRobot->DrawLink_Local(linksInCollision[i]);
      glPopMatrix();
      glDisable(GL_BLEND);
    }

  }
  void drawUniformForceField()
  {
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
  }
  void drawPathSweptVolume(Robot *robot, std::vector<std::vector<Matrix4> > mats, vector<GLDraw::GeometryAppearance> appearanceStack, double sweptvolumeScale, GLColor sweptvolumeColor)
  {
    //loopin' through the waypoints
    for(uint i = 0; i < mats.size(); i++){
      for(uint j=0;j<robot->links.size();j++) {
        if(robot->IsGeometryEmpty(j)) continue;
        Matrix4 matij = mats.at(i).at(j);

        glPushMatrix();
        glMultMatrix(matij);

        sweptvolumeColor.setCurrentGL();
        glScalef(sweptvolumeScale, sweptvolumeScale, sweptvolumeScale);

        GLDraw::GeometryAppearance& a = appearanceStack.at(j);
        a.SetColor(sweptvolumeColor);

        a.DrawGL();
        glPopMatrix();

      }
    }
  }
};


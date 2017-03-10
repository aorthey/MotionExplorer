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
  void drawPlannerStartGoal(Robot *robot, const Config &p_init, const Config &p_goal)
  {
    if(!p_init.empty() && !p_goal.empty()){
      GLColor colorOriginal;
      GLColor colorInit(0,1,0);
      GLColor colorGoal(1,0,0);
      double scale = 1.01;

      //GLDraw::GeometryAppearance& a = *robot->geomManagers[0].Appearance();
      //a.GetColor(colorOriginal);

      robot->UpdateConfig(p_init);
      for(uint j=0;j<robot->links.size();j++) {
        if(robot->IsGeometryEmpty(j)) continue;
        Matrix4 mat = robot->links[j].T_World;
        glPushMatrix();
        glMultMatrix(mat);
        glScalef(scale, scale, scale);
        GLDraw::GeometryAppearance& a = *robot->geomManagers[j].Appearance();
        a.SetColor(colorInit);
        a.DrawGL();
        glPopMatrix();
      }
      
      robot->UpdateConfig(p_goal);
      for(uint j=0;j<robot->links.size();j++) {
        if(robot->IsGeometryEmpty(j)) continue;
        Matrix4 mat = robot->links[j].T_World;
        glPushMatrix();
        glMultMatrix(mat);
        glScalef(scale, scale, scale);
        GLDraw::GeometryAppearance& a = *robot->geomManagers[j].Appearance();
        a.SetColor(colorGoal);
        a.DrawGL();
        a.SetColor(colorOriginal);
        glPopMatrix();
      }
    }
  }

  void drawPlannerTree(const SerializedTree &_stree)
  {
    double linewidth = 0.01;

    uint nearestNode = 0;
    double bestD = dInf;
    for(uint i = 0; i < _stree.size(); i++){
      SerializedTreeNode node = _stree.at(i);
      double d = node.cost_to_goal;
      if(d<bestD){
        bestD = d;
        nearestNode = i;
      }
    }
    for(uint i = 0; i < _stree.size(); i++){
      //std::cout << "Tree GUI:" << tree.at(0).first << std::endl;
      SerializedTreeNode node = _stree.at(i);
      Vector3 pos(node.position(0),node.position(1),node.position(2));
      Vector3 rot(node.position(3),node.position(4),node.position(5));

      std::vector<Vector3> dirs = node.directions;

      glDisable(GL_LIGHTING);
      glEnable(GL_BLEND); 
      glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
      
      //compute color from cost_to_goal
      double d = node.cost_to_goal;
      double shade = exp(-d*d/0.5); //\in [0,1]
      GLColor color(shade,0,1.0-shade);

      color.setCurrentGL();

      glPushMatrix();
      glTranslate(pos);
      //glMaterialfv(GL_FRONT,GL_AMBIENT_AND_DIFFUSE,cForce);

      if(i==nearestNode){
        GLColor cNearest(1,1,0);
        cNearest.setCurrentGL();
        glPointSize(15.0);                                     
        //std::cout << "Nearest Node " << node.position << " d=" <<bestD << std::endl;
      }else{
        glPointSize(5.0);                                     
      }
      drawPoint(Vector3(0,0,0));
      for(uint j = 0; j < dirs.size(); j++){
        Vector3 dir(dirs.at(j)[0], dirs.at(j)[1], dirs.at(j)[2]);
        drawCylinder(dir,linewidth);
      }

      glPushMatrix();

      glPopMatrix();
      glPopMatrix();
      glEnable(GL_LIGHTING);
    }
  }

#include <GL/freeglut.h>
  void drawAxesLabels(Camera::Viewport& viewport)
  {
    //TODO: (1) does not support scale, (2) does not exactly cooincide with
    //coordwidget, wtf?
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho((double)viewport.x, (double)viewport.y,
        (double)viewport.w, (double)viewport.h,
        -1000., 1000.);
    glTranslated(0., 0., 0.);
    glMatrixMode(GL_MODELVIEW);

    double l = 0.5;
    double o = 0 ;

    double cx = 0;
    double cy = 0;
    double xx, xy, yx, yy , zx, zy;

    float fvViewMatrix[ 16 ];
    glGetFloatv( GL_MODELVIEW_MATRIX, fvViewMatrix );
    glLoadIdentity();

    xx = l * fvViewMatrix[0];
    xy = l * fvViewMatrix[1];
    yx = l * fvViewMatrix[4];
    yy = l * fvViewMatrix[5];
    zx = l * fvViewMatrix[8];
    zy = l * fvViewMatrix[9];

    double lineWidth = 0.1;
    //double lineWidth = 0.1;
    glLineWidth(lineWidth);
    //glColor4ubv(color);

    glBegin(GL_LINES);
    glVertex2d(cx, cy);
    glVertex2d(cx + xx, cy + xy);
    glVertex2d(cx, cy);
    glVertex2d(cx + yx, cy + yy);
    glVertex2d(cx, cy);
    glVertex2d(cx + zx, cy + zy);
    glEnd();
    glRasterPos2d(cx + xx + o, cy + xy + o);
    glutBitmapString(GLUT_BITMAP_HELVETICA_18, (unsigned char*) "X");
    glRasterPos2d(cx + yx + o, cy + yy + o);
    glutBitmapString(GLUT_BITMAP_HELVETICA_18, (unsigned char*) "Y");
    glRasterPos2d(cx + zx + o, cy + zy + o);
    glutBitmapString(GLUT_BITMAP_HELVETICA_18, (unsigned char*) "Z");
  }
};


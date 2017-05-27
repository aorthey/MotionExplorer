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
      viewik.widgetSize = 0.1;
      viewik.lineWidth = 10;
      viewik.lineColor = GLColor(1.0,0.5,0);
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
  void drawWrenchField(WrenchField &wrenchfield){
    for(int i = 0; i < wrenchfield.size(); i++){
      Vector3 pos = wrenchfield.getPosition(i);
      Vector3 force = wrenchfield.getForce(i);

      double scale = 0.5;
      force = scale*force/force.norm();

      glDisable(GL_LIGHTING);

      glEnable(GL_BLEND); 
      glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

      //cForce.setCurrentGL();
      GLColor cWrench(0,0,0.5);
      cWrench.setCurrentGL();
      
      glPushMatrix();
      glTranslate(pos);

      //glPointSize(10);
      //drawPoint(Vector3(0,0,0));

      glLineWidth(5);

      glBegin(GL_LINES);
      glVertex3f(0.0, 0.0, 0.0);
      glVertex3f(force[0],force[1],force[2]);
      glEnd();

      glPopMatrix();
      glEnable(GL_LIGHTING);
    }

  }

  void drawForceField(WrenchField &W){
    Real linewidth=0.01;
    std::vector<SmartPointer<ForceField> > forcefields = W.GetForceFields();
    for(int i = 0; i < forcefields.size(); i++){
      SmartPointer<ForceField> f = forcefields.at(i);
      if(f->type() == UNIFORM){
        //TODO: how to visualize a uniform field
      }else if(f->type() == RADIAL){
        SmartPointer<RadialForceField>& fr = *reinterpret_cast<SmartPointer<RadialForceField>*>(&forcefields.at(i));
        //Vector3 source = dynamic_cast<RadialForceField*>(f)->GetSource()
        Vector3 source = fr->GetSource();
        double radius = fr->GetRadius();

        glDisable(GL_LIGHTING);
        glEnable(GL_BLEND); 
        glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

        GLColor cForce(0.2,0.7,0.2);
        cForce.setCurrentGL();
        
        glPushMatrix();
        glTranslate(source);

        glPointSize(5);
        drawPoint(Vector3(0,0,0));

        glLineWidth(2);

        double theta_step = M_PI/4;
        double gamma_step = theta_step;
        for(double theta = 0; theta < 2*M_PI; theta+=theta_step){
          for(double gamma = 0; gamma < M_PI; gamma+=gamma_step){
            double xs = radius*sin(theta)*cos(gamma);
            double ys = radius*sin(theta)*sin(gamma);
            double zs = radius*cos(theta);
            glBegin(GL_LINES);
            glVertex3f(0,0,0);
            glVertex3f(xs,ys,zs);
            glEnd();
          }
        }


        glPopMatrix();
        glEnable(GL_LIGHTING);

      }
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

  void drawGLPathSweptVolume(Robot *robot, std::vector<std::vector<Matrix4> > mats, vector<GLDraw::GeometryAppearance> appearanceStack, GLColor sweptvolumeColor, double sweptvolumeScale)
  {
    //loopin' through the waypoints
    //glEnable(GL_CULL_FACE);
    //glCullFace(GL_BACK);
    for(uint i = 0; i < mats.size(); i++){
      for(uint j=0;j<robot->links.size();j++) {
        if(robot->IsGeometryEmpty(j)) continue;
        Matrix4 matij = mats.at(i).at(j);


        glPushMatrix();
        glMultMatrix(matij);

        //sweptvolumeColor.setCurrentGL();
        glScalef(sweptvolumeScale, sweptvolumeScale, sweptvolumeScale);

        GLDraw::GeometryAppearance& a = appearanceStack.at(j);
        a.SetColor(sweptvolumeColor);

        //glBlendFunc(GL_ONE_MINUS_DST_ALPHA, GL_DST_ALPHA);
        a.DrawGL();
        glPopMatrix();

      }
    }
  }
  void drawGLPathKeyframes(Robot *robot, std::vector<uint> keyframe_indices, std::vector<std::vector<Matrix4> > mats, vector<GLDraw::GeometryAppearance> appearanceStack,GLColor color, double scale)
  {
    for(uint k = 0; k < keyframe_indices.size(); k++){
      uint i = keyframe_indices.at(k);

      for(uint j=0;j<robot->links.size();j++) {
        if(robot->IsGeometryEmpty(j)) continue;
        Matrix4 matij = mats.at(i).at(j);

        glPushMatrix();
        glMultMatrix(matij);

        glScalef(scale, scale, scale);

        GLDraw::GeometryAppearance& a = appearanceStack.at(j);
        if(a.geom != robot->geometry[j]) a.Set(*robot->geometry[j]);

        a.SetColor(color);

        a.DrawGL();
        glPopMatrix();

      }
    }

  }

  void drawGLPathStartGoal(Robot *robot, const Config &p_init, const Config &p_goal)
  {
    GLColor colorInit(0.3,1,0.3);
    GLColor colorGoal(1,0.3,0.3);
    double scale = 1.01;
    if(!p_init.empty()) drawRobotAtConfig(robot, p_init, colorInit, scale);
    if(!p_goal.empty()) drawRobotAtConfig(robot, p_goal, colorGoal, scale);
  }


  void drawRobotAtConfig(Robot *robot, const Config &q, GLColor color, double scale){
    robot->UpdateConfig(q);
    for(uint j=0;j<robot->links.size();j++) {
      if(robot->IsGeometryEmpty(j)) continue;
      Matrix4 mat = robot->links[j].T_World;
      glPushMatrix();
      glMultMatrix(mat);
      glScalef(scale, scale, scale);
      GLDraw::GeometryAppearance& a = *robot->geomManagers[j].Appearance();
      a.SetColor(color);
      a.DrawGL();
      glPopMatrix();
    }
  }


  void drawPlannerTree(const SerializedTree &_stree, GLColor colorTree)
  {
    double edgeWidth = 0.1;
    double vertexSize = 0.1;

    //too costly for each round
    //uint nearestNode = 0;
    //double bestD = dInf;
    //for(uint i = 0; i < _stree.size(); i++){
    //  SerializedTreeNode node = _stree.at(i);
    //  double d = node.cost_to_goal;
    //  if(d<bestD){
    //    bestD = d;
    //    nearestNode = i;
    //  }
    //}
    glDisable(GL_LIGHTING);
    glEnable(GL_BLEND); 
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
      
    for(uint i = 0; i < _stree.size(); i++){
      SerializedTreeNode node = _stree.at(i);
      Vector3 pos(node.position(0),node.position(1),node.position(2));
      Vector3 rot(node.position(3),node.position(4),node.position(5));

      std::vector<Vector3> dirs = node.directions;

      ////Fancy Color gradient, red near goal, blue far away, gaussian //distributed
      //double d = node.cost_to_goal;
      //double shade = exp(-d*d/0.5); //\in [0,1]
      //GLColor color(shade,0,1.0-shade);

      colorTree.setCurrentGL();

      glPushMatrix();
      glTranslate(pos);
      glPointSize(vertexSize);
      drawPoint(Vector3(0,0,0));

      for(uint j = 0; j < dirs.size(); j++){
        colorTree.setCurrentGL();
        glBegin(GL_LINES);
        glLineWidth(edgeWidth);
        glVertex3f(0.0, 0.0, 0.0);
        glVertex3f(dirs.at(j)[0], dirs.at(j)[1], dirs.at(j)[2]);
        glEnd();
      }


      glPushMatrix();

      glPopMatrix();
      glPopMatrix();
      glEnable(GL_LIGHTING);
    }
    //std::cout << "Visualized Tree with " << _stree.size() << " vertices and " << Nedges << " edges." << std::endl;
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
  void drawFrames(std::vector< std::vector<Vector3> > &frames, std::vector<double> frameLength){
    Real linewidth=0.005;
    for(int i = 0; i < frames.size(); i++){
      Vector3 p = frames.at(i).at(0);
      Vector3 e1 = frames.at(i).at(1);
      Vector3 e2 = frames.at(i).at(2);
      Vector3 e3 = frames.at(i).at(3);
  
      GLColor c1(1,0,0);
      GLColor c2(0,1,0);
      GLColor c3(0,0,1);

      glDisable(GL_LIGHTING);

      glEnable(GL_BLEND); 
      glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

      glPushMatrix();
      c1.setCurrentGL();
      glTranslate(p);
      drawCylinder(e1*frameLength.at(i),linewidth);
      glPopMatrix();

      glPushMatrix();
      c2.setCurrentGL();
      glTranslate(p);
      drawCylinder(e2*frameLength.at(i),linewidth);
      glPopMatrix();

      glPushMatrix();
      c3.setCurrentGL();
      glTranslate(p);
      drawCylinder(e3*frameLength.at(i),linewidth);
      glPopMatrix();
      glEnable(GL_LIGHTING);
    }
  }
};


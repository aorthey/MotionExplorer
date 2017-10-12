#include "KrisLibrary/math3d/basis.h"
#include "drawMotionPlanner.h"
#include "controller/controller.h"

namespace GLDraw{

  struct GLCheckeredSphere
  {
    GLCheckeredSphere();
    void Draw();

    Real radius;
    Vector3 center;
    GLColor c1,c2;
    int numSlices,numStacks;
  };

  GLCheckeredSphere::GLCheckeredSphere()
    :radius(1),center(Zero),numSlices(16),numStacks(8)
  {
    c1.set(0.7,0.7,0.7);
    c2.set(0.3,0.3,0.3);
  }

  //theta is the vertical range, phi is the rotational range
  void DrawSphereArc(Real r,Real theta0,Real theta1,Real phi0,Real phi1,int numSlices,int numStacks)
  {
    Real thetaInc = (theta1-theta0)/Real(numStacks);
    Real phiInc = (phi1-phi0)/Real(numSlices);
    Real phi=phi0;
    Real theta;
    for(int i=0;i<numSlices;i++,phi+=phiInc) {
      Real x1=Cos(phi);
      Real x2=Cos(phi+phiInc);
      Real y1=Sin(phi);
      Real y2=Sin(phi+phiInc);
      theta=theta0;
      glBegin(GL_TRIANGLE_STRIP);
      for(int j=0;j<=numStacks;j++,theta+=thetaInc) {
        Real cz=Cos(theta);
        Real sz=Sin(theta);
        glNormal3f(x2*sz,y2*sz,cz);
        glVertex3f(r*x2*sz,r*y2*sz,r*cz);
        glNormal3f(x1*sz,y1*sz,cz);
        glVertex3f(r*x1*sz,r*y1*sz,r*cz);
      }
      glEnd();
    }
  }

  void GLCheckeredSphere::Draw()
  {
    //glEnable(GL_LIGHTING);
    glPushMatrix();
    {
      glTranslate(center);
      //glMaterialfv(GL_FRONT,GL_AMBIENT_AND_DIFFUSE,c1.rgba); 
      DrawSphereArc(radius, 0,Pi_2,  0,Pi_2,    numSlices/4,numStacks/2);
      DrawSphereArc(radius, 0,Pi_2,  Pi,3*Pi_2, numSlices/4,numStacks/2);
      DrawSphereArc(radius, Pi_2,Pi, Pi_2,Pi,   numSlices/4,numStacks/2);
      DrawSphereArc(radius, Pi_2,Pi, 3*Pi_2,TwoPi,numSlices/4,numStacks/2);
      //glMaterialfv(GL_FRONT,GL_AMBIENT_AND_DIFFUSE,c2.rgba); 
      DrawSphereArc(radius, 0,Pi_2,  Pi_2,Pi,   numSlices/4,numStacks/2);
      DrawSphereArc(radius, 0,Pi_2,  3*Pi_2,TwoPi,numSlices/4,numStacks/2);
      DrawSphereArc(radius, Pi_2,Pi, 0,Pi_2,    numSlices/4,numStacks/2);
      DrawSphereArc(radius, Pi_2,Pi, Pi,3*Pi_2, numSlices/4,numStacks/2);
    }
    glPopMatrix();
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
      //glDisable(GL_LIGHTING);
      //glEnable(GL_BLEND);
      //glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
      glPushMatrix();
      //glMultMatrix(Matrix4(T));
      viewRobot->DrawLink_Local(linksInCollision[i]);
      glPopMatrix();
      //glDisable(GL_BLEND);
    }

  }
  void drawWrenchField(WrenchField &wrenchfield){
    ////glEnable(GL_LIGHTING);
    //glDisable(GL_DEPTH_TEST);
    GLColor cWrench(0,1,1);

    for(uint i = 0; i < wrenchfield.size(); i++){
      Vector3 pos = wrenchfield.getPosition(i);
      Vector3 force = wrenchfield.getForce(i);
      Vector3 sforce = 0.5*force / (1+force.length());
      Real r=0.01;

      //force at link i
      glPushMatrix();
      setColor(cWrench);
      glTranslate(pos);
      //glMaterialfv(GL_FRONT,GL_AMBIENT_AND_DIFFUSE,cWrench);
      drawCylinder(sforce,r);
      glPushMatrix();
      glTranslate(sforce);
      drawCone(3*r*sforce/sforce.length(),2*r,8);
      glPopMatrix();
      glPopMatrix();

    }
    Vector3 com(0,0,3);
    Vector3 lmomentum = wrenchfield.getCOMLinearMomentum();
    Vector3 amomentum = wrenchfield.getCOMAngularMomentum();

    lmomentum = 0.5*lmomentum / (1+lmomentum.length());
    amomentum = 0.5*amomentum / (1+amomentum.length());

    GLColor cLinMom(0,1,1);
    GLColor cAngMom(1,0,1);

    GLDraw::GLCheckeredSphere sph;
    sph.center = com;
    sph.radius = 0.1;
    sph.Draw();
    //Real r=0.01;
    drawCylinderArrowAtPosition(com, lmomentum, cLinMom);
    drawCylinderArrowAtPosition(com, amomentum, cAngMom);

    sph.center = wrenchfield.getCOMPosition();
    sph.Draw();
    drawCylinderArrowAtPosition(sph.center, lmomentum, cLinMom);
    drawCylinderArrowAtPosition(sph.center, amomentum, cAngMom);


    //glEnable(GL_DEPTH_TEST);
    //glDisable(GL_LIGHTING);


  }
  void drawCylinderArrowAtPosition(Vector3 &pos, Vector3 &dir, GLColor &color){
    double r = 0.01;
    glPushMatrix();
    glTranslate(pos);
    //glMaterialfv(GL_FRONT,GL_AMBIENT_AND_DIFFUSE,color);
    drawCylinder(dir, r);
    glPushMatrix();
    glTranslate(dir);
    drawCone(3*r*dir/dir.length(),2*r,8);
    glPopMatrix();
    glPopMatrix();

  }
  void setColor(GLColor &c){
    //glColor4f(c[0],c[1],c[2],c[3]);
    //glColor3f(c[0],c[1],c[2]);
    c.setCurrentGL();
  }

  void drawForceField(WrenchField &W){
    //Real linewidth=0.01;
    std::vector<SmartPointer<ForceField> > forcefields = W.GetForceFields();

    //glDisable(GL_LIGHTING);
    //glEnable(GL_BLEND); 
    //glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

    for(uint fctr = 0; fctr < forcefields.size(); fctr++){
      SmartPointer<ForceField> f = forcefields.at(fctr);
      if(f->type() == UNIFORM){
        //TODO: how to visualize a uniform field
      }else if(f->type() == RADIAL){

        SmartPointer<RadialForceField>& fr = *reinterpret_cast<SmartPointer<RadialForceField>*>(&forcefields.at(fctr));
        //Vector3 source = dynamic_cast<RadialForceField*>(f)->GetSource()
        Vector3 source = fr->GetSource();
        double radius = fr->GetRadius();
        double power = fr->GetPower();

        glPushMatrix();
        GLColor cForce = fr->GetColor();
        setColor(cForce);
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


            Vector3 middle(xs/2,ys/2,zs/2);
            glPushMatrix();
            glTranslate(middle);
            //drawPoint(Vector3(0,0,0));

            Vector3 direction = 0.1*middle; 
            if(power < 0) {
              direction *= -1;
            }
            GLDraw::drawCone(direction, 0.5*direction.norm());
            glPopMatrix();
          }
        }


        glPopMatrix();

      }else if(f->type() == OBB){
        SmartPointer<OrientedBoundingBoxForceField>& fb = *reinterpret_cast<SmartPointer<OrientedBoundingBoxForceField>*>(&forcefields.at(fctr));
        double power = fb->GetPower();
        Vector3 center = fb->GetCenter();
        Vector3 extension = fb->GetExtension();
        Matrix3 R = fb->GetRotation();
        
        glPushMatrix();
        GLColor cForce = fb->GetColor();
        setColor(cForce);
        glTranslate(center);
        Matrix4 RH(R);
        RH(3,3) = 1;
        glMultMatrixd(RH);

        glPointSize(5);
        drawPoint(Vector3(0,0,0));

        glLineWidth(2);

        double ystep = 0.5;
        double zstep = 0.5;
        for(double y = -extension[1]/2; y < extension[1]/2; y+=ystep){
          for(double z = -extension[2]/2; z < extension[2]/2; z+=zstep){

            double x = extension[0]/2;

            glBegin(GL_LINES);
            glVertex3f(-x,y,z);
            glVertex3f(x,y,z);
            glEnd();

            double length = 0.1;
            Vector3 direction(length,0,0);
            if(power < 0) {
              direction *= -1;
            }
            glPushMatrix();
            Vector3 middle(-x/2,y,z);
            glTranslate(middle);
            GLDraw::drawCone(direction, length/2);
            glPopMatrix();
            glPushMatrix();
            middle = Vector3(x/2,y,z);
            glTranslate(middle);
            GLDraw::drawCone(direction, length/2);
            glPopMatrix();
          }
        }


        glPopMatrix();
      }else if(f->type() == CYLINDRICAL){

        SmartPointer<CylindricalForceField>& fr = *reinterpret_cast<SmartPointer<CylindricalForceField>*>(&forcefields.at(fctr));
        //Vector3 source = dynamic_cast<RadialForceField*>(f)->GetSource()
        Vector3 source = fr->GetSource();
        Vector3 direction = fr->GetDirection();
        direction /= direction.length();
        double elongation = fr->GetElongation();
        double radius = fr->GetRadius();
        double power = fr->GetPower();
        GLColor cForce = fr->GetColor();

        glPushMatrix();
        setColor(cForce);
        glTranslate(source);

        //draw cable axis
        glPointSize(5);
        drawPoint(Vector3(0,0,0));
        glLineWidth(4);
        Vector3 vv = 0.5*elongation*direction;
        glBegin(GL_LINES);
        glVertex3f(-vv[0],-vv[1],-vv[2]);
        glVertex3f(vv[0],vv[1],vv[2]);
        glEnd();


        double verticalDistanceConcentricCircles = min(1.0, elongation);
        double horizontalDistanceConcentricCircles = min(0.5, radius);

        uint numSteps = 16; 

        for(double dv = -0.5*elongation; dv <= 0.5*elongation; dv+=verticalDistanceConcentricCircles){
          for(double dr = horizontalDistanceConcentricCircles; dr <= radius; dr+=horizontalDistanceConcentricCircles){
              glPushMatrix();
              float inc = fTwoPi/numSteps;
              Vector3 eu,ev;
              GetCanonicalBasis(direction,eu,ev);
              Complex x,dx;
              dx.setPolar(One,inc);
              
              glBegin(GL_LINE_LOOP);
              x.set(dr,0);
              for(uint j=0; j<numSteps; j++) {
                glVertex3v(direction*dv + x.x*eu+x.y*ev);
                x=x*dx;
              }
              glEnd();

              uint Narrows = 5;
              inc = fTwoPi/Narrows;
              dx.setPolar(One,inc);
              x.set(dr,0);

              for(uint j = 0; j < Narrows; j++){
                glPushMatrix();
                Vector3 rr = x.x*eu + x.y*ev;
                Vector3 arrowS(direction*dv + rr);
                x=x*dx;

                glTranslate(arrowS);
                Vector3 coneori = cross(direction,rr);
                coneori /= coneori.length();
                double length = 0.1;
                if(power < 0) {
                  coneori *= -1;
                }
                GLDraw::drawCone(length*coneori, 0.5*length);
                glPopMatrix();
              }
              glPopMatrix();

          }
        }


        glPopMatrix();
      }
    }
    //glDisable(GL_BLEND); 
    //glEnable(GL_LIGHTING);


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
  
          //glDisable(GL_LIGHTING);
          //glEnable(GL_BLEND); 
          //glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
  
          glPushMatrix();
          setColor(cForce);
  
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
          //glEnable(GL_LIGHTING);
        }//forz
      }//fory
    }//forx
  }

  void drawSwathVolume(Robot *robot, std::vector<std::vector<Matrix4> > mats, vector<GLDraw::GeometryAppearance> appearanceStack, GLColor swathVolumeColor)
  {
    drawGLPathSweptVolume(robot, mats, appearanceStack, swathVolumeColor, 1.0);
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
    Config qq; qq.resize(robot->q.size());qq.setZero();
    for(int k = 0; k < q.size(); k++) qq(k)=q(k);
    robot->UpdateConfig(qq);
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
    double edgeWidth = 3;
    glDisable(GL_LIGHTING);
    glEnable(GL_BLEND); 
      
    for(uint i = 0; i < _stree.size(); i++){
      SerializedTreeNode node = _stree.at(i);
      Vector3 pos(node.position(0),node.position(1),node.position(2));
      Vector3 rot(node.position(3),node.position(4),node.position(5));

      std::vector<Vector> dirs = node.directions;

      ////Fancy Color gradient, red near goal, blue far away, gaussian //distributed
      //double d = node.cost_to_goal;
      //double shade = exp(-d*d/0.5); //\in [0,1]
      //GLColor color(shade,0,1.0-shade);

      glPushMatrix();
      setColor(colorTree);
      glTranslate(pos);

      //glPointSize(vertexSize);
      glPointSize(10);
      drawPoint(Vector3(0,0,0));

      glPushMatrix();
      glLineWidth(edgeWidth);
      for(uint j = 0; j < dirs.size(); j++){
        glBegin(GL_LINES);
        glVertex3f(0.0, 0.0, 0.0);
        glVertex3f(dirs.at(j)[0], dirs.at(j)[1], dirs.at(j)[2]);
        glEnd();
      }
      glPopMatrix();

      glPopMatrix();
    }
    glDisable(GL_BLEND); 
    glEnable(GL_LIGHTING);
    //glEnable(GL_LIGHTING);
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
    for(uint i = 0; i < frames.size(); i++){
      Vector3 p = frames.at(i).at(0);
      Vector3 e1 = frames.at(i).at(1);
      Vector3 e2 = frames.at(i).at(2);
      Vector3 e3 = frames.at(i).at(3);
  
      GLColor c1(1,0,0);
      GLColor c2(0,1,0);
      GLColor c3(0,0,1);

      //glDisable(GL_LIGHTING);

      //glEnable(GL_BLEND); 
      //glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

      glPushMatrix();
      setColor(c1);
      glTranslate(p);
      drawCylinder(e1*frameLength.at(i),linewidth);
      glPopMatrix();

      glPushMatrix();
      setColor(c2);
      glTranslate(p);
      drawCylinder(e2*frameLength.at(i),linewidth);
      glPopMatrix();

      glPushMatrix();
      setColor(c3);
      glTranslate(p);
      drawCylinder(e3*frameLength.at(i),linewidth);
      glPopMatrix();
      //glEnable(GL_LIGHTING);
    }
  }
  void drawCenterOfMassPathFromController(WorldSimulation &sim){
    int linewidth = 4;

    if(!(sim.robotControllers.size()>0)) return;

    SmartPointer<ContactStabilityController>& controller = *reinterpret_cast<SmartPointer<ContactStabilityController>*>(&sim.robotControllers[0]);

    ControllerState output = controller->GetControllerState();

    glDisable(GL_LIGHTING);
    glEnable(GL_BLEND); 

    GLColor color(1,0,0);
    for(int i = 0; i < int(output.predicted_com.size())-1; i++){
      Vector3 com_cur = output.predicted_com.at(i);
      Vector3 com_next = output.predicted_com.at(i+1);

      Vector3 dc = com_next - com_cur;
        
      glPushMatrix();
      setColor(color);
      glLineWidth(linewidth);
      glTranslate(com_cur);

      //glPointSize(5);
      //drawPoint(Vector3(0,0,0));

      glBegin(GL_LINES);
      glVertex3f(0,0,0);
      glVertex3f(dc[0],dc[1],dc[2]);
      glEnd();

      glPopMatrix();
    }
    GLColor green(0,1,0);
    for(int i = 0; i < int(output.com_window.size())-1; i++){
      Vector3 com_cur = output.com_window.at(i);
      Vector3 com_next = output.com_window.at(i+1);

      Vector3 dc = com_next - com_cur;
        
      glPushMatrix();
      setColor(green);
      glTranslate(com_cur);

      glLineWidth(linewidth);
      glBegin(GL_LINES);
      glVertex3f(0,0,0);
      glVertex3f(dc[0],dc[1],dc[2]);
      glEnd();

      glPopMatrix();
    }
    glEnable(GL_LIGHTING);
    glDisable(GL_BLEND); 
  }
  void drawDistanceRobotTerrain(const ODERobot *robot, const Terrain* terrain){
    //ODERobot *robot = sim.odesim.robot(0);
    uint Nlinks = robot->robot.links.size();
    //const Terrain* terrain = sim.odesim.terrain(0);
    const Geometry::AnyCollisionGeometry3D tgeom = (*terrain->geometry);
    Geometry::AnyCollisionGeometry3D tt(tgeom);
    robot->robot.CleanupCollisions();
    robot->robot.InitMeshCollision(tt);

    //glDisable(GL_LIGHTING);
    //glEnable(GL_BLEND); 
    GLColor yellow(1,1,0);
    GLColor white(1,1,1);
    for(uint i = 0; i < Nlinks; i++){
      dBodyID bodyid = robot->body(i);
      if(bodyid){
        if(!robot->robot.IsGeometryEmpty(i)){
          RobotLink3D *link = &robot->robot.links[i];

          Geometry::AnyCollisionQuery *query = robot->robot.envCollisions[i];
          //double d = query->Distance(0,0.1);
          std::vector<Vector3> vp1,vp2;
          query->InteractingPoints(vp1,vp2);
          if(vp1.size()!=1){
            //std::cout << "Warning: got " << vp1.size() << " contact points for single rigid body" << std::endl;
            //std::cout << "Ignoring Link " << i << "/" << Nlinks << std::endl;
          }else{
            //Matrix4 mat = link->T_World;
            //glMultMatrix(mat);
            Vector3 p1 = link->T_World*vp1.front();
            Vector3 p2 = vp2.front();
            //Vector3 dp = p1-p2;

            glPushMatrix();
            glLineWidth(3);
            glPointSize(5);
            setColor(white);
            drawPoint(p1);
            drawPoint(p2);
            glBegin(GL_LINES);
            setColor(yellow);
            glVertex3f(p1[0],p1[1],p1[2]);
            glVertex3f(p2[0],p2[1],p2[2]);
            glEnd();
            glPopMatrix();
          }
        }
      }
    }
  }
  void drawShortestPath( SimplicialComplex& cmplx ){

    if(cmplx.path.size()<=1) return;
    glDisable(GL_LIGHTING);
    glEnable(GL_BLEND);
    glPushMatrix();
    GLColor red(0.8,0.2,0.2,1);
    setColor(red);
    glPointSize(30);
    glLineWidth(15);
    //std::cout << std::string(80, '-') << std::endl;
    for(uint i = 0; i < cmplx.path.size()-1; i++){
      //std::cout << cmplx.path.at(i) << std::endl;
      drawPoint(cmplx.path.at(i));
      drawLineSegment(cmplx.path.at(i),
                      cmplx.path.at(i+1));
    }
    glPopMatrix();
    glEnable(GL_LIGHTING);
    glDisable(GL_BLEND);
  }
  void drawPath( const std::vector<Config> &path, GLColor &c, double linewidth, double ptsize){
    glDisable(GL_LIGHTING);
    glEnable(GL_BLEND);
    glPushMatrix();

    glPointSize(ptsize);
    glLineWidth(linewidth);
    setColor(c);
    for(uint i = 0; i < path.size()-1; i++){
      Config c1 = path.at(i);
      Config c2 = path.at(i+1);
      Vector3 q1(c1[0],c1[1],c1[2]);
      Vector3 q2(c2[0],c2[1],c2[2]);
      drawPoint(q1);
      drawLineSegment(q1, q2);
    }
    glPopMatrix();
    glDisable(GL_BLEND);
    glEnable(GL_LIGHTING);
    glLineWidth(1);
  }
  void drawSimplicialComplex( SimplicialComplex& cmplx ){

    glDisable(GL_LIGHTING);
    glEnable(GL_BLEND);
    GLColor magenta(0.8,0,0.8,0.5);
    GLColor black(0.1,0.1,0.1,1);
    GLColor red(0.9,0.1,0.1,1);
    GLColor blue(0.1,0.1,0.9,1);
    GLColor white(1,1,1,1);
    GLColor grey(0.7,0.7,0.7,1);
    glPushMatrix();
    glPointSize(10);
    double dmax = cmplx.max_distance_shortest_path;
    double dmin = cmplx.min_distance_shortest_path;

    for(uint i = 0; i < cmplx.V.size(); i++){
      if(!cmplx.distance_shortest_path.empty()){
        double di = (cmplx.distance_shortest_path.at(i)-dmin)/(dmax-dmin);
        GLColor ci; ci.blend(red, blue, di); setColor(ci);
      }
      drawPoint(cmplx.V.at(i));
    }
    glLineWidth(10);
    setColor(magenta);
    for(uint i = 0; i < cmplx.E.size(); i++){
      drawLineSegment(cmplx.E.at(i).first, 
                      cmplx.E.at(i).second);
    }
    setColor(magenta);
    for(uint i = 0; i < cmplx.F.size(); i++){
      drawTriangle(cmplx.F.at(i).at(0),
                   cmplx.F.at(i).at(1),
                   cmplx.F.at(i).at(2) );
    }
    for(uint i = 0; i < cmplx.T.size(); i++){
      drawQuad(cmplx.T.at(i).at(0),
                   cmplx.T.at(i).at(1),
                   cmplx.T.at(i).at(2),
                   cmplx.T.at(i).at(3) );
    }
    glPopMatrix();
    glDisable(GL_BLEND);
    glEnable(GL_LIGHTING);
  }

  void drawForceEllipsoid( const ODERobot *robot ){
    uint Nlinks = robot->robot.links.size();
    glDisable(GL_LIGHTING);
    glEnable(GL_BLEND); 
    for(uint i = 0; i < Nlinks; i++){
      dBodyID bodyid = robot->body(i);
      if(bodyid){
        if(!robot->robot.IsGeometryEmpty(i)){
          RobotLink3D *link = &robot->robot.links.at(i);
          Vector3 com = link->com;
          Matrix J;
          robot->robot.GetFullJacobian(com, i, J);
          uint N=J.numCols();
          Matrix Jp(3,N);
          J.getSubMatrixCopy(3,0,Jp);

          J = Jp;
          Math::SVDecomposition<Real> svd(J);
          Matrix U = svd.U;
          Math::SVDecomposition<Real>::DiagonalMatrixT Sigma = svd.W;

          if(1){//i==14 || i==32 || i==39 || i==45){
            std::vector<Vector3> axes;
            Vector3 center = link->T_World*com;
            for(int k = 0; k < 4; k++){
              //remove zero singular values
              if(Sigma(k)<1e-10){
                continue;
              }
              Vector uk = U.col(k);
              Vector3 u(uk[0],uk[1],uk[2]);
              u /= u.norm();
              Vector3 dp = 1.0/(Sigma(k)*Sigma(k))*u;
              dp *= 0.5;
              //Vector3 p1 = center - dp;
              //Vector3 p2 = center + dp;
              axes.push_back(dp);
            }
            if(axes.size()>3){
              std::cout << "more than three axes for Ellipsoid. Something is wrong" << std::endl;
              exit(0);
            }
            if(axes.size()<3){
              std::cout << "Warning: ellipsoid is singular" << std::endl;
            }else{
              glPushMatrix();
              GLColor magenta(0.5,0,1,0.5);
              setColor(magenta);
              drawWireEllipsoid(center, axes.at(0),axes.at(1),axes.at(2),8);
              glPopMatrix();
            }
          }
        }
      }
    }
    glEnable(GL_LIGHTING);
    glDisable(GL_BLEND); 
  }

      
  void drawWireEllipsoid(Vector3 &c, Vector3 &u, Vector3 &v, Vector3 &w, int numSteps)
  {
    //x = a*cos(t) cos(s)
    //y = b*cos(t) sin(s)
    //z = c*sin(t)
    //
    // -pi/2 <= t <= pi/2 
    // -pi <= s <= pi

    float tStep = M_PI/(float)numSteps;
    float sStep = 2*M_PI/(float)16;

    glPushMatrix();
    glLineWidth(2);
    for(float t = -M_PI/2; t <= M_PI/2; t += tStep)
    {
      glBegin(GL_LINE_LOOP);
      for(float s = -M_PI; s <= M_PI; s += sStep)
      {
        Vector3 point = cos(t)*cos(s)*u + cos(t)*sin(s)*v + sin(t)*w + c;
        glVertex3f(point[0],point[1],point[2]);
      }
      glEnd();
    }
    for(float t = -M_PI/2; t <= M_PI/2; t += tStep)
    {
      glBegin(GL_LINE_LOOP);
      for(float s = -M_PI; s <= M_PI; s += sStep)
      {
        Vector3 point = cos(t)*cos(s)*v + cos(t)*sin(s)*w + sin(t)*u + c;
        glVertex3f(point[0],point[1],point[2]);
      }
      glEnd();
    }
    glPopMatrix();
  }
  void drawEllipsoid(Vector3 &c, Vector3 &u, Vector3 &v, Vector3 &w, int numSteps)
  {
    //x = a*cos(t) cos(s)
    //y = b*cos(t) sin(s)
    //z = c*sin(t)
    //
    // -pi/2 <= t <= pi/2 
    // -pi <= s <= pi

    float tStep = M_PI/(float)numSteps;
    float sStep = 2*M_PI/(float)numSteps;

    //glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    for(float t = -M_PI/2; t <= M_PI/2; t += tStep)
    {
      glBegin(GL_TRIANGLE_STRIP);
      for(float s = -M_PI; s <= M_PI; s += sStep)
      {
        Vector3 p1 = cos(t)*cos(s)*u + cos(t)*sin(s)*v + sin(t)*w + c;
        Vector3 p2 = cos(t+tStep)*cos(s)*u + cos(t+tStep)*sin(s)*v + sin(t+tStep)*w + c;
        glVertex3f(p1[0],p1[1],p1[2]);
        glVertex3f(p2[0],p2[1],p2[2]);
      }
      glEnd();
    }
    //for(float s = -M_PI; s <= M_PI; s += sStep)
    //{
    //  glBegin(GL_LINE_LOOP);
    //  glVertex3f(c[0],c[1],c[2]);
    //  for(float t = -M_PI/2; t <= M_PI/2; t += tStep)
    //  {
    //    Vector3 point = cos(t)*cos(s)*v + cos(t)*sin(s)*w + sin(t)*u + c;
    //    glVertex3f(point[0],point[1],point[2]);
    //  }
    //  glEnd();
    //}
  }
};


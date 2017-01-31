#pragma once

#include <Interface/SimTestGUI.h>
#include <Modeling/MultiPath.h>
#include <KrisLibrary/robotics/IK.h>
#include <KrisLibrary/robotics/IKFunctions.h>
#include <KrisLibrary/geometry/CollisionMesh.h>
#include <KrisLibrary/GLdraw/drawMesh.h>

#include <View/ViewIK.h>
#include <ode/ode.h>

const GLColor bodyColor(0.9,0.9,0.9);
const GLColor selectedLinkColor(1.0,1.0,0.5);
const double sweptvolumeScale = 0.90;
GLColor sweptvolumeColor(0.7,0.0,0.9,0.5);

class ForceFieldBackend : public SimTestBackend
{
  private:
    bool drawForceField;
    bool drawExtras; 
    bool drawIKextras;
    bool drawPath;
    vector<IKGoal> _constraints;
    vector<int> _linksInCollision;
    string _robotname;
    //swept volume
    std::vector<std::vector<Matrix4> > _mats;
    vector<GLDraw::GeometryAppearance> _appearanceStack;

  public:
    typedef SimTestBackend BaseT; //Need to parse it through SimTest to get wrenchies

  ForceFieldBackend(RobotWorld *world)
      : SimTestBackend(world)
  {
    drawForceField = false;
    drawExtras = false;
    drawIKextras = false;
    drawPath = false;

  }
  virtual void Start()
  {
    BaseT::Start();
    //settings["desired"]["color"][0] = 1;
    //settings["desired"]["color"][1] = 0;
    //settings["desired"]["color"][2] = 0;
    //settings["desired"]["color"][3] = 0.5;
    //camera.dist = 4;
    //viewport.n = 0.1;
    //viewport.f = 100;
    //viewport.setLensAngle(DtoR(90.0));
    //DisplayCameraTarget();
    //Camera::CameraController_Orbit camera;
    //Camera::Viewport viewport;
    show_frames_per_second = true;

  }
  virtual bool OnCommand(const string& cmd,const string& args){
    BaseT::OnCommand(cmd,args);
    if(cmd=="advance") {
      //ODERobot *robot = sim.odesim.robot(0);
      //std::cout << "Force" << std::endl;
      //std::cout << robot->robot.name << std::endl;

      //Config q;
      //robot->GetConfig(q);
      //double px,py,pz;
      //double fx,fy,fz;
      //fx = 10.0;
      //fy = 0.0;
      //fz = 10.0;
      //px = 0.0;
      //py = 0.0;
      //pz = 0.0;
      //dBodyAddForceAtPos(robot->body(7),fx,fy,fz,px,py,pz);
    }
  }
  //############################################################################
  //############################################################################
  void SetIKConstraints( vector<IKGoal> constraints, string robotname){
    _constraints = constraints;
    _robotname = robotname;
    drawIKextras = true;
  }
  void SetIKCollisions( vector<int> linksInCollision )
  {
    _linksInCollision = linksInCollision;
    drawIKextras = true;
  }

  //############################################################################
  //############################################################################
  void VisualizePathSweptVolume(const MultiPath &path)
  {
    _mats.clear();
    Robot *robot = world->robots[0];

    double dstep = 0.1;
    Config q;
    Config dq;

    //orthezticate the matrices
    for(double d = 0; d <= path.Duration(); d+=dstep)
    {
      path.Evaluate(d, q);
      robot->UpdateConfig(q);
      std::vector<Matrix4> mats_config;

      for(size_t i=0;i<robot->links.size();i++) {
        Matrix4 mat = robot->links[i].T_World;
        mats_config.push_back(mat);
      }
      _mats.push_back(mats_config);
    }
    std::cout << "[SweptVolume] #waypoints " << _mats.size() << std::endl;
    _appearanceStack.clear();
    _appearanceStack.resize(robot->links.size());
    for(size_t i=0;i<robot->links.size();i++) {
      //if(robot->geomManagers[i].IsAppearanceShared())
      //{
      //  robot->geomManagers[i].SetUniqueAppearance();
      //}
      GLDraw::GeometryAppearance& a = *robot->geomManagers[i].Appearance();
      //a.SetColor(sweptvolumeColor);
      _appearanceStack[i]=a;
    }
    std::cout << "[SweptVolume] #geometries " << _appearanceStack.size() << std::endl;
    drawPath = true;

  }

  virtual void RenderWorld()
  {

    if(drawExtras){
      vector<ViewRobot> viewRobots = world->robotViews;
      ViewRobot *viewRobot = &viewRobots[0];
      double COMradius = 0.05;
      double frameLength = 0.2;
      viewRobot->DrawCenterOfMass(COMradius);
      //viewRobot->DrawLinkFrames(frameLength);
      viewRobot->DrawLinkSkeleton();
      viewRobot->SetColors(bodyColor);
    }
    //############################################################################
    // IK extras: contact links, contact directions
    //############################################################################
    if(drawIKextras){

      vector<ViewRobot> viewRobots = world->robotViews;
      ViewRobot *viewRobot = &viewRobots[0];
      for(int i = 0; i < _constraints.size(); i++){

        const IKGoal goal = _constraints[i];

        viewRobot->SetColor(goal.link, selectedLinkColor);
        ViewIKGoal viewik = ViewIKGoal();
        Robot *robot = world->GetRobot(this->_robotname);
        viewik.Draw(goal, *robot);
        viewik.DrawLink(goal, *viewRobot);

        //glDisable(GL_LIGHTING);
        //glEnable(GL_BLEND);
        //glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
        //glPushMatrix();
        //glMultMatrix(Matrix4(T));
        //robotviewer.DrawLink_Local(goal.link);
        //glPopMatrix();
        //glDisable(GL_BLEND);

      }
      for(int i = 0; i < _linksInCollision.size(); i++){
        glDisable(GL_LIGHTING);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
        glPushMatrix();
        //glMultMatrix(Matrix4(T));
        viewRobot->DrawLink_Local(_linksInCollision[i]);
        glPopMatrix();
        glDisable(GL_BLEND);
      }

    }

    //############################################################################
    // visualize a flow/force field on R^3
    //############################################################################
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
    //
    //############################################################################
    // Visualize a MultiPath: Swept Volume 
    //############################################################################
    if(drawPath){
      Robot *robot = world->robots[0];
      //loopin' through the waypoints
      for(int i = 0; i < _mats.size(); i++){
        for(int j=0;j<robot->links.size();j++) {
          if(robot->IsGeometryEmpty(j)) continue;
          Matrix4 matij = _mats.at(i).at(j);

          glPushMatrix();
          glMultMatrix(matij);

          sweptvolumeColor.setCurrentGL();
          glScalef(sweptvolumeScale, sweptvolumeScale, sweptvolumeScale);

          GLDraw::GeometryAppearance& a = _appearanceStack.at(j);
          a.SetColor(sweptvolumeColor);

          //a.drawVertices = false;
          //a.drawEdges = false;
          //a.drawFaces = false;
          //a.lightFaces = true;
          a.DrawGL();
          glPopMatrix();

        }
      }
    }
    BaseT::RenderWorld();
  }//RenderWorld



};


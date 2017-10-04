#include <KrisLibrary/robotics/IK.h>
#include <KrisLibrary/robotics/IKFunctions.h>
#include <KrisLibrary/robotics/Stability.h>
#include <Planning/StanceCSpace.h>
#include "planner.h"
#include "iksolver_hrp2.h"

class MotionPlannerContact: public MotionPlanner
{
  public:
    MotionPlannerContact(RobotWorld *world): MotionPlanner(world){};
    virtual bool solve(Config &p_init, Config &p_goal){
      this->_p_init = p_init;
      this->_p_goal = p_goal;
      Robot *robot = _world->robots[_irobot];
      robot->UpdateConfig(_p_init);
      this->_world->InitCollisions();

      std::cout << std::string(80, '-') << std::endl;
      std::cout << "Motion Planner:" << this->getName() << std::endl;
      std::cout << "p_init =" << p_init << std::endl;
      std::cout << "p_goal =" << p_goal << std::endl;
      std::cout << std::string(80, '-') << std::endl;

      WorldPlannerSettings worldsettings;
      worldsettings.InitializeDefault(*_world);

      //SingleRobotCSpace geometric_cspace = SingleRobotCSpace(*_world,_irobot,&worldsettings);
      //if(!IsFeasible( robot, geometric_cspace, _p_goal)) return false;
      //if(!IsFeasible( robot, geometric_cspace, _p_init)) return false;

      ContactCSpace cspace(*_world, _irobot, &worldsettings);

      //#######################################################################
      //set some IK goal
      //#######################################################################
      Matrix3 R;
      double theta = M_PI/4;
      double xl = 0.0;
      double yl = -2.0;
      double zl = 0.0;

      R.setRotateZ(theta);
      IKSolverHRP2 iks(_world);
      IKGoal contact1 = iks.LinkToGoalTransRot("l_sole",xl,yl,zl,R);
      //IKGoal contact2 = iks.LinkToGoalTransRot("r_sole",0,-2.15,0.0,I);
      //cspace.AddContact(contact2);
      //vector<IKGoal> ikcnstr = ik.GetConstraints();

      PropertyMap pmap;
      cspace.Properties(pmap);
      std::cout << pmap << std::endl;

      cspace.AddContact(contact1);

      for(int i = 0; i < 1000; i++){
        Config x;
        cspace.Sample(x);
        if(cspace.IsFeasible(x)){
          robot->UpdateConfig(x);
          Vector3 lfoot(xl,yl,zl);
          Vector3 c = robot->GetCOM() - lfoot;

          Vector3 ex(1,0,0),ey(0,1,0),cx,cy;
          R.mul(ex,cx);
          R.mul(ey,cy);

          double ctx = dot(c,cx);
          double cty = dot(c,cy);

          double dx = sqrt((xl-ctx)*(xl-ctx));
          double dy = sqrt((yl-cty)*(yl-cty));

          if(dx>0.05) continue;
          this->q = x;
          std::cout << "Found config at iter " << i << std::endl;
          return true;
          break;
        }
      }

      return false;
    }

    Config q;
};

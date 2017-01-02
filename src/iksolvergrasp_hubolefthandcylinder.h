#pragma once
#include "iksolvergrasp.h"

class IKSolverGraspHuboLeftHandCylinder: public IKSolverGrasp
{
  public:
    IKSolverGraspHuboLeftHandCylinder(RobotWorld *world, int objectid):
      IKSolverGrasp(world)
    {
      this->objectid = world->RigidObjectID(objectid);
    }

    string GetRobotName(){
      return "hubo";
    }

    vector<IKGoal> GetProblem(){
      std::cout << "go " << std::endl;
      vector<IKGoal> problem;
      Matrix3 I;
      I.setRotateZ(Pi/2);

      Matrix3 R;
      R.setRotateZ(Pi/2);
      Matrix3 L;
      L.setRotateZ(Pi);

//Link[14] leftThumbProximal mass 0.096
//Link[15] leftThumbMedial mass 0.048
//Link[16] leftThumbDistal mass 0.08
//Link[17] leftPinkyProximal mass 0.096
//Link[18] leftPinkyMedial mass 0.048
//Link[19] leftPinkyDistal mass 0.08
//Link[20] leftRingProximal mass 0.096
//Link[21] leftRingMedial mass 0.048
//Link[22] leftRingDistal mass 0.08
//Link[23] leftMiddleProximal mass 0.096
//Link[24] leftMiddleMedial mass 0.048
//Link[25] leftMiddleDistal mass 0.08
//Link[26] leftIndexProximal mass 0.096
//Link[27] leftIndexMedial mass 0.048
//Link[28] leftIndexDistal mass 0.08

      std::cout << "go " << std::endl;
      //0.27m dist (1inch)
      problem.push_back( LinkToGoalRot("leftIndexDistal",0.4,-0.0,1.2,R) );
      problem.push_back( LinkToGoalRot("leftMiddleDistal",0.4,-0.0,1.173,R) );
      //problem.push_back( LinkToGoalRot("leftRingDistal",0.4,-0.2,1.46,R) );
      //problem.push_back( LinkToGoalRot("leftPinkyDistal",0.4,-0.2,1.19,R) );
      //problem.push_back( LinkToGoalRot("leftThumbDistal",0.45,-0.2,1.2,L) );
      std::cout << "go " << std::endl;

      //Set both feet on the floor
      problem.push_back( LinkToGoalRot("Body_LAR",0.4,-0.5,0.1,I) );
      problem.push_back( LinkToGoalRot("Body_RAR",0.6,-0.5,0.1,I) );

      return problem;
    }
};

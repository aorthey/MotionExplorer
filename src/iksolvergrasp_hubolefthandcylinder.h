#pragma once
#include "iksolvergrasp.h"

class IKSolverGraspHuboLeftHandCylinder: public IKSolverGrasp
{
  private:
    vector<Real> fixedDofValues;
    vector<int> fixedDofs;
  public:
    IKSolverGraspHuboLeftHandCylinder(RobotWorld *world, int objectid):
      IKSolverGrasp(world)
    {
      this->objectid = world->RigidObjectID(objectid);
    }

    string GetRobotName(){
      return "hubo";
    }

    void ComputeFixedDofs(){
      //fix everything except joints 8-28 (left arm and left hand)
      fixedDofs.clear();
      fixedDofValues.clear();

      Config q = this->_robot->q;
      for(int i =0; i<8;i++){
        std::cout << "fixed dof" << i << std::endl;
        fixedDofs.push_back(i);
        fixedDofValues.push_back(q[i]);
      }
      for(int i =29; i<62;i++){
        std::cout << "fixed dof" << i << std::endl;
        fixedDofs.push_back(i);
        fixedDofValues.push_back(q[i]);
      }

      //set finger fixed
      //int i = 25;
      //fixedDofs.push_back(i);
      //fixedDofValues.push_back(q[i]);
      //i = 28;
      //fixedDofs.push_back(i);
      //fixedDofValues.push_back(q[i]);
    }

    vector<Real> GetFixedDofValues(){
      return fixedDofValues;
    }
    vector<int> GetFixedDofs(){
      return fixedDofs;
    }
    vector<IKGoal> GetConstraints(){
      vector<IKGoal> constraints;
      Matrix3 I;
      I.setRotateZ(Pi/2);

      Matrix3 R;
      R.setRotateZ(Pi/2);

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

      //0.27m dist (1inch)
      //1.2 1.173
      this->_tolerance = 1e0;
      constraints.push_back( LinkToGoalTransRot("leftIndexDistal",0.2,-0.0,0.98,R) );
      constraints.push_back( LinkToGoalTransRot("leftMiddleDistal",0.2,-0.0,0.953,R) );
      constraints.push_back( LinkToGoalTransRot("leftRingDistal",0.2,-0.0,0.94,R) );
      constraints.push_back( LinkToGoalTransRot("leftPinkyDistal",0.2,-0.02,0.927,R) );
      //constraints.push_back( LinkToGoalRot("leftThumbDistal",0.22,-0.0,0.98,R) );
      //constraints.push_back( LinkToGoal("leftThumbDistal",0.26,-0.03,0.975) );
      //constraints.push_back( LinkToGoalRot("leftThumbDistal",0.26,-0.03,0.975,L) );
      Matrix3 L;
      L.setRotateZ(-Pi/4);
      constraints.push_back( LinkToGoalRot("leftThumbDistal",L) );

      //Set both feet on the floor
      constraints.push_back( LinkToGoalTransRot("Body_LAR",0.4,-0.5,0.1,I) );
      constraints.push_back( LinkToGoalTransRot("Body_RAR",0.6,-0.5,0.1,I) );

      return constraints;
    }
};

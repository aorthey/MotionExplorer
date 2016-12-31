#pragma once
#include "iksolver.h"

//--------------------------------------------------------------------------------
//Robot hubo
//--------------------------------------------------------------------------------
//Link[0] Body_Torso0 mass 0
//Link[1] Body_Torso1 mass 0
//Link[2] Body_Torso2 mass 0
//Link[3] Body_Torso3 mass 0
//Link[4] Body_Torso4 mass 0
//Link[5] Body_Torso5 mass 7.50334
//
//Link[6] Body_HNR mass 0.1
//Link[7] Body_HNP mass 0.374251
//
//Link[8] Body_LSP mass 0.646655
//Link[9] Body_LSR mass 0.413368
//Link[10] Body_LSY mass 1.15461
//Link[11] Body_LEP mass 0.44094
//Link[12] Body_LWY mass 0.537738
//Link[13] Body_LWP mass 0.164393
//
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
//
//Link[29] Body_RSP mass 0.646655
//Link[30] Body_RSR mass 0.413368
//Link[31] Body_RSY mass 1.15461
//Link[32] Body_REP mass 0.44094
//Link[33] Body_RWY mass 0.537738
//Link[34] Body_RWP mass 0.164393
//
//Link[35] rightThumbProximal mass 0.096
//Link[36] rightThumbMedial mass 0.048
//Link[37] rightThumbDistal mass 0.08
//Link[38] rightPinkyProximal mass 0.096
//Link[39] rightPinkyMedial mass 0.048
//Link[40] rightPinkyDistal mass 0.08
//Link[41] rightRingProximal mass 0.096
//Link[42] rightRingMedial mass 0.048
//Link[43] rightRingDistal mass 0.08
//Link[44] rightMiddleProximal mass 0.096
//Link[45] rightMiddleMedial mass 0.048
//Link[46] rightMiddleDistal mass 0.08
//Link[47] rightIndexProximal mass 0.096
//Link[48] rightIndexMedial mass 0.048
//Link[49] rightIndexDistal mass 0.08
//
//Link[50] Body_Hip mass 3.41719
//Link[51] Body_LHY mass 0.826125
//Link[52] Body_LHR mass 1.93266
//Link[53] Body_LHP mass 2.8201
//Link[54] Body_LKP mass 1.80912
//Link[55] Body_LAP mass 1.63501
//Link[56] Body_LAR mass 1.20318
//Link[57] Body_RHY mass 0.826125
//Link[58] Body_RHR mass 1.93266
//Link[59] Body_RHP mass 2.8201
//Link[60] Body_RKP mass 1.80912
//Link[61] Body_RAP mass 1.63501
//Link[62] Body_RAR mass 1.20318


class IKSolverHubo: public IKSolver
{
  public:
  IKSolverHubo(RobotWorld *world):
    IKSolver(world)
  {
  }
  //#########################################################################
  string GetRobotName(){
    return "hubo";
  }

  vector<IKGoal> GetProblem(){
    vector<IKGoal> problem;
    Matrix3 I;
    I.setRotateZ(Pi/2);

    problem.push_back( LinkToGoalRot("Body_LAR",0.4,-0.5,0.1,I) );
    problem.push_back( LinkToGoalRot("Body_RAR",0.6,-0.5,0.1,I) );
    problem.push_back( LinkToGoal("leftIndexDistal",0.4,-0.2,1.2) );
    problem.push_back( LinkToGoal("rightIndexDistal",0.6,-0.2,1.2) );

    return problem;
  }
  //#########################################################################
  IKGoal LinkToGoalRot( const char *linkName, double x, double y, double z, Matrix3 &rotation)
  {
    int linkid = _robot->LinkIndex(linkName);
    Vector3 localPosition(0,0,0);
    Vector3 position(x,y,z);

    IKGoal goal;
    goal.link = linkid;
    goal.localPosition = localPosition;
    goal.SetFixedPosition(position);
    goal.SetFixedRotation(rotation);
    return goal;
  }
  IKGoal LinkToGoal( const char *linkName, double x, double y, double z)
  {
    int linkid = _robot->LinkIndex(linkName);
    Vector3 localPosition(0,0,0);
    Vector3 position(x,y,z);

    IKGoal goal;
    goal.link = linkid;
    goal.localPosition = localPosition;
    goal.SetFixedPosition(position);
    return goal;
  }
    
};


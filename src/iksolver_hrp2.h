#pragma once
#include "iksolver.h"

//--------------------------------------------------------------------------------
//Robot HRP2
//--------------------------------------------------------------------------------
//Link[0] base0 mass 0
//Link[1] base1 mass 0
//Link[2] base2 mass 0
//Link[3] base3 mass 0
//Link[4] base4 mass 0
//Link[5] base_link mass 0.0001
//
//Link[6] BODY mass 15.0295
//Link[7] CHEST_LINK0 mass 0.786369
//Link[8] torso mass 11.5246
//Link[9] Accelerometer mass 0.0001
//Link[10] Gyro mass 0.0001
//Link[11] HEAD_LINK0 mass 0.374766
//Link[12] HEAD_LINK1 mass 0.993355
//
//Link[13] camera_bottom_left mass 0.0001
//Link[14] camera_bottom_left_optical mass 0.0001
//Link[15] camera_bottom_right mass 0.0001
//Link[16] camera_bottom_right_optical mass 0.0001
//Link[17] camera_top_left mass 0.0001
//Link[18] camera_top_left_optical mass 0.0001
//Link[19] camera_top_right mass 0.0001
//Link[20] camera_top_right_optical mass 0.0001
//Link[21] gaze mass 0.0001
//
//Link[22] LARM_LINK0 mass 0.810076
//Link[23] LARM_LINK1 mass 1.03448
//Link[24] LARM_LINK2 mass 1.59929
//Link[25] LARM_LINK3 mass 0.77725
//Link[26] LARM_LINK4 mass 0.554669
//Link[27] l_wrist mass 0.573818
//Link[28] LARM_LINK6 mass 0.0804856
//Link[29] LHAND_LINK0 mass 0.0632782
//Link[30] LHAND_LINK1 mass 0.0775929
//Link[31] LHAND_LINK2 mass 0.205701
//Link[32] LHAND_LINK3 mass 0.0745618
//Link[33] LHAND_LINK4 mass 0.0710285
//Link[34] LeftHandForceSensor mass 0.0001
//Link[35] l_gripper mass 0.0001
//
//Link[36] RARM_LINK0 mass 0.810076
//Link[37] RARM_LINK1 mass 1.03448
//Link[38] RARM_LINK2 mass 1.59929
//Link[39] RARM_LINK3 mass 0.77725
//Link[40] RARM_LINK4 mass 0.554669
//Link[41] r_wrist mass 0.573818
//Link[42] RARM_LINK6 mass 0.0804856
//Link[43] RHAND_LINK0 mass 0.0632782
//Link[44] RHAND_LINK1 mass 0.0775929
//Link[45] RHAND_LINK2 mass 0.205701
//Link[46] RHAND_LINK3 mass 0.0745618
//Link[47] RHAND_LINK4 mass 0.0710285
//Link[48] RightHandForceSensor mass 0.0001
//Link[49] r_gripper mass 0.0001
//
//Link[50] LLEG_LINK0 mass 1.23548
//Link[51] LLEG_LINK1 mass 0.80553
//Link[52] LLEG_LINK2 mass 2.07632
//Link[53] LLEG_LINK3 mass 1.74762
//Link[54] LLEG_LINK4 mass 0.65375
//Link[55] l_ankle mass 1.63877
//Link[56] LeftFootForceSensor mass 0.0001
//Link[57] l_sole mass 0.0001
//
//Link[58] RLEG_LINK0 mass 1.23548
//Link[59] RLEG_LINK1 mass 0.80553
//Link[60] RLEG_LINK2 mass 2.07632
//Link[61] RLEG_LINK3 mass 1.74762
//Link[62] RLEG_LINK4 mass 0.65375
//Link[63] r_ankle mass 1.63877
//Link[64] RightFootForceSensor mass 0.0001
//Link[65] r_sole mass 0.0001


class IKSolverHRP2: public IKSolver
{
  public:
  IKSolverHRP2(RobotWorld *world):
    IKSolver(world)
  {
  }
  //#########################################################################
  string GetRobotName(){
    return "hrp2_14";
  }

  vector<IKGoal> GetConstraints(){
    vector<IKGoal> constraints;
    Matrix3 I;
    I.setRotateZ(0);

    constraints.push_back( LinkToGoalTransRot("l_sole",0,-2.0,0.0,I) );
    constraints.push_back( LinkToGoalTransRot("r_sole",0,-2.15,0.0,I) );
    //constraints.push_back( LinkToGoalTransRot("l_gripper",0,-1.5,1.0,I) );

    return constraints;
  }
  //#########################################################################
    
};


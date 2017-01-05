#pragma once
#include "iksolvergrasp.h"

//--------------------------------------------------------------------------------
//Robot robonaut
//--------------------------------------------------------------------------------
//Link[0] dummy mass 1
//Link[1] baseplate mass 5
//Link[2] base mass 50
//Link[3] robot_base mass 0
//Link[4] waist_center mass 10
//Link[5] backpack mass 10
//Link[6] body_cover mass 0.1
//Link[7] chest_center mass 0
//Link[8] chest_base mass 0
//Link[9] left_shoulder_roll mass 6.16886
//Link[10] left_shoulder_pitch mass 4.42706
//Link[11] left_upper_arm mass 4.85344
//Link[12] left_elbow mass 2.97103
//Link[13] left_lower_arm mass 6.93996
//Link[14] left_wrist_pitch mass 0.116573
//Link[15] left_wrist_yaw mass 1.18841
//Link[16] left_palm mass 1.18841
//Link[17] left_index_base mass 0.001
//Link[18] left_index_proximal mass 0.001
//Link[19] left_index_medial mass 0.001
//Link[20] left_index_distal mass 0.001
//Link[21] left_index_tip mass 0.001
//Link[22] left_little_proximal mass 0.001
//Link[23] left_little_medial mass 0.001
//Link[24] left_little_distal mass 0.001
//Link[25] left_little_tip mass 0.001
//Link[26] left_middle_base mass 0.001
//Link[27] left_middle_proximal mass 0.001
//Link[28] left_middle_medial mass 0.001
//Link[29] left_middle_distal mass 0.001
//Link[30] left_middle_tip mass 0.001
//Link[31] left_ring_proximal mass 0.001
//Link[32] left_ring_medial mass 0.001
//Link[33] left_ring_distal mass 0.001
//Link[34] left_ring_tip mass 0.001
//Link[35] left_thumb_base mass 0.05
//Link[36] left_thumb_proximal mass 0.001
//Link[37] left_thumb_medial_prime mass 0.001
//Link[38] left_thumb_medial mass 0.001
//Link[39] left_thumb_distal mass 0.001
//Link[40] left_thumb_tip mass 0.001
//Link[41] right_shoulder_roll mass 6.16886
//Link[42] right_shoulder_pitch mass 4.42706
//Link[43] right_upper_arm mass 4.85344
//Link[44] right_elbow mass 2.97103
//Link[45] right_lower_arm mass 6.93996
//Link[46] right_wrist_pitch mass 0.116573
//Link[47] right_wrist_yaw mass 1.18841
//Link[48] right_palm mass 0.1
//Link[49] right_index_base mass 0.05
//Link[50] right_index_proximal mass 0.05
//Link[51] right_index_medial mass 0.05
//Link[52] right_index_distal mass 0.05
//Link[53] right_index_tip mass 0.05
//Link[54] right_little_proximal mass 0.05
//Link[55] right_little_medial mass 0.05
//Link[56] right_little_distal mass 0.05
//Link[57] right_little_tip mass 0.05
//Link[58] right_middle_base mass 0.001
//Link[59] right_middle_proximal mass 0.05
//Link[60] right_middle_medial mass 0.05
//Link[61] right_middle_distal mass 0.05
//Link[62] right_middle_tip mass 0.05
//Link[63] right_ring_proximal mass 0.05
//Link[64] right_ring_medial mass 0.05
//Link[65] right_ring_distal mass 0.05
//Link[66] right_ring_tip mass 0.05
//Link[67] right_thumb_base mass 0.05
//Link[68] right_thumb_proximal mass 0.05
//Link[69] right_thumb_medial_prime mass 0.05
//Link[70] right_thumb_medial mass 0.05
//Link[71] right_thumb_distal mass 0.05
//Link[72] right_thumb_tip mass 0.05
//Link[73] neck_base mass 0.1
//Link[74] neck_lower_pitch mass 0.1
//Link[75] neck_roll mass 0.1
//Link[76] neck_upper_pitch mass 0.1
//Link[77] asus_frame mass 0
//Link[78] openni_depth_frame mass 0
//Link[79] simulated_asus_frame mass 0
//Link[80] asus_asus_ir_link mass 0
//Link[81] asus_asus_ir_optical_frame mass 0
//Link[82] asus_asus_rgb_link mass 0
//Link[83] asus_asus_rgb_optical_frame mass 0
//Link[84] vision_center_frame mass 0
//Link[85] left_camera_frame mass 0
//Link[86] left_camera_optical_frame mass 0
//Link[87] right_camera_frame mass 0
//Link[88] right_camera_optical_frame mass 0
//Link[89] robot_reference mass 0


class IKSolverGraspRobonaut: public IKSolverGrasp
{
  public:
    IKSolverGraspRobonaut(RobotWorld *world, int objectid):
      IKSolverGrasp(world)
    {
      this->objectid = world->RigidObjectID(objectid);
    }

    string GetRobotName(){
      return "robonaut";
    }

    void ComputeFixedDofs(){
      fixedDofs.clear();
      fixedDofValues.clear();

      Config q = this->_robot->q;
      for(int i =0; i<10;i++){
        fixedDofs.push_back(i);
        fixedDofValues.push_back(q[i]);
      }
//Link[22] left_little_proximal mass 0.001
//Link[23] left_little_medial mass 0.001
//Link[24] left_little_distal mass 0.001
//Link[25] left_little_tip mass 0.001
//Link[31] left_ring_proximal mass 0.001
//Link[32] left_ring_medial mass 0.001
//Link[33] left_ring_distal mass 0.001
//Link[34] left_ring_tip mass 0.001
      for(int i =22; i<26;i++){
        fixedDofs.push_back(i);
        fixedDofValues.push_back(q[i]);
      }
      for(int i =31; i<35;i++){
        fixedDofs.push_back(i);
        fixedDofValues.push_back(q[i]);
      }
      for(int i =41; i<this->_robot->q.size();i++){
        fixedDofs.push_back(i);
        fixedDofValues.push_back(q[i]);
      }
    }

    vector<Real> GetFixedDofValues(){
      return fixedDofValues;
    }
    vector<int> GetFixedDofs(){
      return fixedDofs;
    }
    vector<IKGoal> GetConstraints(){
      vector<IKGoal> constraints;

      //L.setIdentity();
//Link[10] left_shoulder_pitch mass 4.42706
//Link[11] left_upper_arm mass 4.85344
//Link[12] left_elbow mass 2.97103
//Link[13] left_lower_arm mass 6.93996
//Link[14] left_wrist_pitch mass 0.116573
//Link[15] left_wrist_yaw mass 1.18841
//Link[16] left_palm mass 1.18841
//Link[17] left_index_base mass 0.001
//Link[18] left_index_proximal mass 0.001
//Link[19] left_index_medial mass 0.001
//Link[20] left_index_distal mass 0.001
//Link[21] left_index_tip mass 0.001
//Link[22] left_little_proximal mass 0.001
//Link[23] left_little_medial mass 0.001
//Link[24] left_little_distal mass 0.001
//Link[25] left_little_tip mass 0.001
//Link[26] left_middle_base mass 0.001
//Link[27] left_middle_proximal mass 0.001
//Link[28] left_middle_medial mass 0.001
//Link[29] left_middle_distal mass 0.001
//Link[30] left_middle_tip mass 0.001
//Link[31] left_ring_proximal mass 0.001
//Link[32] left_ring_medial mass 0.001
//Link[33] left_ring_distal mass 0.001
//Link[34] left_ring_tip mass 0.001
//Link[35] left_thumb_base mass 0.05
//Link[36] left_thumb_proximal mass 0.001
//Link[37] left_thumb_medial_prime mass 0.001
//Link[38] left_thumb_medial mass 0.001
//Link[39] left_thumb_distal mass 0.001
//Link[40] left_thumb_tip mass 0.001

      //0.27m dist (1inch)
      //1.2 1.173
      //this->_tolerance = 1e-1;

      Matrix3 L;
      L.setRotateZ(Pi/2);
      Matrix3 LX;
      LX.setRotateY(Pi);
      L = LX*L;

      Matrix3 Lt;
      Lt.setRotateZ(Pi/2);
      Matrix3 LY;
      LY.setRotateY(Pi);
      Matrix3 LZ;
      //LZ.setRotateY(-Pi);
      LZ.setRotateY(-Pi-Pi/4-Pi/8);
      Lt = LZ*LY*Lt;

      constraints.push_back( LinkToGoalTransRot("left_thumb_distal",0.37,-0.05,1.0,Lt) );
      constraints.push_back( LinkToGoalTransRot("left_index_distal",0.3,-0.0,1.01,L) );
      constraints.push_back( LinkToGoalTransRot("left_middle_distal",0.3,-0.0,0.97,L) );

      return constraints;
    }
};

#pragma once
#include <Modeling/Robot.h>
#include "gui.h"
#include "elements/path_pwl_euclid.h"
#include "planner/irreducible_projector.h"

//Link[0] base0 mass 0
//Link[1] base1 mass 0
//Link[2] base2 mass 0
//Link[3] base3 mass 0
//Link[4] base4 mass 0
//Link[5] base_link mass 0.0001
//Link[6] l_sole mass 0.0001
//Link[7] l_ankle mass 1.63877
//Link[8] LLEG_LINK4 mass 0.65375
//Link[9] LLEG_LINK3 mass 1.74762
//Link[10] LLEG_LINK2 mass 2.07632
//Link[11] LLEG_LINK1 mass 0.80553
//Link[12] LLEG_LINK0 mass 1.23548
//Link[13] BODY mass 15.0295
//Link[14] CHEST_LINK0 mass 0.786369
//Link[15] torso mass 11.5246
//Link[16] Accelerometer mass 0.0001
//Link[17] Gyro mass 0.0001
//Link[18] HEAD_LINK0 mass 0.374766
//Link[19] HEAD_LINK1 mass 0.993355
//Link[20] camera_bottom_left mass 0.0001
//Link[21] camera_bottom_left_optical mass 0.0001
//Link[22] camera_bottom_right mass 0.0001
//Link[23] camera_bottom_right_optical mass 0.0001
//Link[24] camera_top_left mass 0.0001
//Link[25] camera_top_left_optical mass 0.0001
//Link[26] camera_top_right mass 0.0001
//Link[27] camera_top_right_optical mass 0.0001
//Link[28] gaze mass 0.0001
//Link[29] LeftFootForceSensor mass 0.0001

//Link[0] base0 mass 0
//Link[1] base1 mass 0
//Link[2] base2 mass 0
//Link[3] base3 mass 0
//Link[4] base4 mass 0
//Link[5] base_link mass 0.0001
//Link[6] l_sole mass 0.0001
//Link[7] l_ankle mass 1.63877
//Link[8] LLEG_LINK4 mass 0.65375
//Link[9] LLEG_LINK3 mass 1.74762
//Link[10] LLEG_LINK2 mass 2.07632
//Link[11] LLEG_LINK1 mass 0.80553
//Link[12] LLEG_LINK0 mass 1.23548
//Link[13] BODY mass 15.0295
//Link[14] CHEST_LINK0 mass 0.786369
//Link[15] torso mass 11.5246
//Link[16] Accelerometer mass 0.0001
//Link[17] Gyro mass 0.0001
//Link[18] HEAD_LINK0 mass 0.374766
//Link[19] HEAD_LINK1 mass 0.993355
//Link[20] camera_bottom_left mass 0.0001
//Link[21] camera_bottom_left_optical mass 0.0001
//Link[22] camera_bottom_right mass 0.0001
//Link[23] camera_bottom_right_optical mass 0.0001
//Link[24] camera_top_left mass 0.0001
//Link[25] camera_top_left_optical mass 0.0001
//Link[26] camera_top_right mass 0.0001
//Link[27] camera_top_right_optical mass 0.0001
//Link[28] gaze mass 0.0001
//Link[29] LARM_LINK0 mass 0.810076
//Link[30] LARM_LINK1 mass 1.03448
//Link[31] LARM_LINK2 mass 1.59929
//Link[32] LARM_LINK3 mass 0.77725
//Link[33] LARM_LINK4 mass 0.554669
//Link[34] l_wrist mass 0.573818
//Link[35] LARM_LINK6 mass 0.0804856
//Link[36] LHAND_LINK0 mass 0.0632782
//Link[37] LHAND_LINK1 mass 0.0775929
//Link[38] LHAND_LINK2 mass 0.205701
//Link[39] LHAND_LINK3 mass 0.0745618
//Link[40] LHAND_LINK4 mass 0.0710285
//Link[41] LeftHandForceSensor mass 0.0001
//Link[42] l_gripper mass 0.0001
//Link[43] RARM_LINK0 mass 0.810076
//Link[44] RARM_LINK1 mass 1.03448
//Link[45] RARM_LINK2 mass 1.59929
//Link[46] RARM_LINK3 mass 0.77725
//Link[47] RARM_LINK4 mass 0.554669
//Link[48] r_wrist mass 0.573818
//Link[49] RARM_LINK6 mass 0.0804856
//Link[50] RHAND_LINK0 mass 0.0632782
//Link[51] RHAND_LINK1 mass 0.0775929
//Link[52] RHAND_LINK2 mass 0.205701
//Link[53] RHAND_LINK3 mass 0.0745618
//Link[54] RHAND_LINK4 mass 0.0710285
//Link[55] RightHandForceSensor mass 0.0001
//Link[56] r_gripper mass 0.0001
//Link[57] RLEG_LINK0 mass 1.23548
//Link[58] RLEG_LINK1 mass 0.80553
//Link[59] RLEG_LINK2 mass 2.07632
//Link[60] RLEG_LINK3 mass 1.74762
//Link[61] RLEG_LINK4 mass 0.65375
//Link[62] r_ankle mass 1.63877
//Link[63] RightFootForceSensor mass 0.0001
//Link[64] r_sole mass 0.0001
//Link[65] LeftFootForceSensor mass 0.0001


class IrreducibleProjectorHRP2: public IrreducibleProjector
{
  public:

    IrreducibleProjectorHRP2(Robot *robot): IrreducibleProjector(robot){};

    virtual std::vector<Config> getSubLinkKeyframes()
    {
      uint N = _robot->links.size();

      std::vector<double> lengths(3);
      double length = 0.25;
      for(int j = 0; j < lengths.size(); j++){
        lengths.at(j)=length;
      }

      //###########################################################################
      // set zero config wholebodypath
      //###########################################################################
      std::vector<Config> wholeBodyPath;
      for(int i = 0; i < _rootPath.size(); i++){
        Config q;
        q.resize(_robot->q.size());

        q.setZero();
        Config qirr = _rootPath.at(i);

        for(int j = 0; j < 28; j++){
          q(j) = qirr(j);
        }

        q(58)=qirr(12);
        q(59)=qirr(11);
        q(60)=qirr(10);
        q(61)=qirr(9);
        q(62)=qirr(8);
        q(63)=qirr(7);
//Link[6] l_sole mass 0.0001
//Link[7] l_ankle mass 1.63877
//Link[8] LLEG_LINK4 mass 0.65375
//Link[9] LLEG_LINK3 mass 1.74762
//Link[10] LLEG_LINK2 mass 2.07632
//Link[11] LLEG_LINK1 mass 0.80553
//Link[12] LLEG_LINK0 mass 1.23548

//Link[58] RLEG_LINK0 mass 1.23548
//Link[59] RLEG_LINK1 mass 0.80553
//Link[60] RLEG_LINK2 mass 2.07632
//Link[61] RLEG_LINK3 mass 1.74762
//Link[62] RLEG_LINK4 mass 0.65375
//Link[63] r_ankle mass 1.63877
//Link[64] RightFootForceSensor mass 0.0001
//Link[65] r_sole mass 0.0001
//
//Link[28] gaze mass 0.0001
//Link[29] LARM_LINK0 mass 0.810076
//Link[30] LARM_LINK1 mass 1.03448
//Link[31] LARM_LINK2 mass 1.59929
//Link[32] LARM_LINK3 mass 0.77725
//Link[33] LARM_LINK4 mass 0.554669
//Link[34] l_wrist mass 0.573818
//Link[35] LARM_LINK6 mass 0.0804856
        q(29) = -M_PI/2;
        q(31) = -M_PI/2;
        q(35) = 0.1;

        q(43) = -M_PI/2;
        q(45) = -M_PI/2;
        q(49) = 0.1;

        q(44) = -M_PI/2;
        q(30) = M_PI/2;
//Link[43] RARM_LINK0 mass 0.810076
//Link[44] RARM_LINK1 mass 1.03448
//Link[45] RARM_LINK2 mass 1.59929
//Link[46] RARM_LINK3 mass 0.77725
//Link[47] RARM_LINK4 mass 0.554669
//Link[48] r_wrist mass 0.573818
//Link[49] RARM_LINK6 mass 0.0804856
//
        std::cout << q << std::endl;
        wholeBodyPath.push_back(q);
      }

      //###########################################################################
      // compute 
      //###########################################################################
      uint rootLinkIdLeftArm = 29;
      uint rootLinkIdRightArm = 43;

      vector<string> linkNames = _robot->linkNames;

      std::cout << "Computing irreducible path from root link: " << linkNames[rootLinkIdRightArm] << std::endl;
      std::cout << "Computing irreducible path from root link: " << linkNames[rootLinkIdLeftArm] << std::endl;

      _positionAlongRootPath.clear();
      _rotationAlongRootPath.clear();

      for(int i = 0; i < _rootPath.size(); i++){
        Config qr = _rootPath.at(i);

        Vector3 T1 = GetPositionAtLink(qr, rootLinkIdRightArm);
        Vector3 T0(T1[0],T1[1],0);
        //Vector3 T0(qr[0],qr[1],0);
        //std::cout << T0 << "," << T1 << std::endl;

        Matrix3 R0;
        R0.setRotateZ(qr[3]+M_PI/2);

        _positionAlongRootPath.push_back(T0);
        _rotationAlongRootPath.push_back(R0);

      }

      pairDoubleVecVec thetagamma = ComputeThetaGammaFromRootPath( _positionAlongRootPath ,_rotationAlongRootPath, lengths);

      doubleVecVec thetas = thetagamma.first;
      doubleVecVec gammas = thetagamma.second;

      for(int i = 0; i < wholeBodyPath.size(); i++){
        Config q = wholeBodyPath.at(i);
        double offset = q(59)+ q(60)+ q(61) - q(15);
        wholeBodyPath.at(i)(43) = offset-M_PI/2;
        wholeBodyPath.at(i)(29) = offset-M_PI/2;

        double t1 = thetas.at(i).at(0);
        double t3 = thetas.at(i).at(1);
        double t5 = thetas.at(i).at(2);

        if(std::isnan(t1)) t1 = 0;
        if(std::isnan(t3)) t3 = 0;
        if(std::isnan(t5)) t5 = 0;

        wholeBodyPath.at(i)(44) = (t1-M_PI/2);
        wholeBodyPath.at(i)(46) = t3;
        wholeBodyPath.at(i)(48) = t5;

        //
        //wholeBodyPath.at(i)(30) = -(t1-M_PI/2);
        //wholeBodyPath.at(i)(32) = -t3;
        //wholeBodyPath.at(i)(34) = -t5;
        
//Link[44] RARM_LINK1 mass 1.03448
//Link[45] RARM_LINK2 mass 1.59929
//Link[46] RARM_LINK3 mass 0.77725
//Link[47] RARM_LINK4 mass 0.554669
//Link[48] r_wrist mass 0.573818
        //std::cout << wholeBodyPath.at(i) << std::endl;
      }

      _positionAlongRootPath.clear();
      _rotationAlongRootPath.clear();

      for(int i = _rootPath.size()-1; i >= 0; i--){
        Config qr = _rootPath.at(i);

        Vector3 T1 = GetPositionAtLink(qr, rootLinkIdLeftArm);

        Vector3 T0(T1[0],T1[1],0);
        //Vector3 T0(qr[0],qr[1],0);
        //std::cout << T0 << "," << T1 << std::endl;

        Matrix3 R0;
        R0.setRotateZ(qr[3]-M_PI/2);

        _positionAlongRootPath.push_back(T0);
        _rotationAlongRootPath.push_back(R0);

      }

      thetagamma = ComputeThetaGammaFromRootPath( _positionAlongRootPath ,_rotationAlongRootPath, lengths);

      thetas = thetagamma.first;
      gammas = thetagamma.second;

      for(int i = wholeBodyPath.size()-1; i>=0; i--){
        Config q = wholeBodyPath.at(i);

        //double offset = q(59)+ q(60)+ q(61) - q(15);
        //wholeBodyPath.at(i)(43) = offset-M_PI/2;
        //wholeBodyPath.at(i)(29) = offset-M_PI/2;

        double t1 = thetas.at(wholeBodyPath.size()-1-i).at(0);
        double t3 = thetas.at(wholeBodyPath.size()-1-i).at(1);
        double t5 = thetas.at(wholeBodyPath.size()-1-i).at(2);

        std::cout << std::string(80, '-') << std::endl;
        std::cout << t1 << "," << t3 << "," << t5 << std::endl;

        if(std::isnan(t1)) t1 = 0;
        if(std::isnan(t3)) t3 = 0;
        if(std::isnan(t5)) t5 = 0;

        std::cout << t1 << "," << t3 << "," << t5 << std::endl;

        wholeBodyPath.at(i)(30) = (t1+M_PI/2);
        wholeBodyPath.at(i)(32) = t3;
        wholeBodyPath.at(i)(34) = t5;
        
      }



    return wholeBodyPath;
  }



};

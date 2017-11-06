#include "elements/path_pwl_SE3.h"
#include <iostream>

PathPiecewiseLinearSE3::PathPiecewiseLinearSE3()
{
}

PathPiecewiseLinearSE3* PathPiecewiseLinearSE3::from_keyframes(const std::vector<Config> &keyframes){
  PathPiecewiseLinearSE3* path = new PathPiecewiseLinearSE3();
  path->keyframes=keyframes;
  path->Ndim = keyframes.at(0).size();
  path->Nkeyframes = keyframes.size();
  path->interpolate();
  return path;
}

//-3.1415 <= q(3) <= +3.1415
//-1.57   <= q(4) <= +1.57
//-3.1415 <= q(5) <= +3.1415
void PathPiecewiseLinearSE3::interpolate(){

  for(uint i = 0; i < Nkeyframes-1; i++){
    Config q1 = keyframes.at(i);
    Config q2 = keyframes.at(i+1);
    //#############################################
    double r = q1(3)-q2(3);
    if(sqrtf(r*r)>M_PI)
      boundary_cross_yaw.push_back(true);
    else
      boundary_cross_yaw.push_back(false);
    //#############################################
    //if(fabs(q1(4)-q2(4))>0.5*M_PI)
    //  boundary_cross_pitch.push_back(true);
    //else
    //  boundary_cross_pitch.push_back(false);
    ////#############################################
    //if(fabs(q1(5)-q2(5))>M_PI)
    //  boundary_cross_roll.push_back(true);
    //else
    //  boundary_cross_roll.push_back(false);

  }

  length = 0;
  interLength.clear();
  for(uint i = 0; i < Nkeyframes-1; i++){
    Config q1 = keyframes.at(i);
    Config q2 = keyframes.at(i+1);

    double n = (q1-q2).norm();
    double n2 = (q1-q2).normSquared();

    if(boundary_cross_yaw.at(i)){
      double r = q1(3)-q2(3);
      n2 -= r*r;
      if(q1(3)<q2(3)){
        r+=M_PI;
      }else{
        r-=M_PI;
      }
      n2 += r*r;
    }

    double d = sqrtf(n2);
    length += d;
    interLength.push_back(d);
  }
}

double GetBoundaryOrientation(double v1, double v2, double tloc){
  //moving over the top edge
  double d1 = sqrtf((M_PI-v1)*(M_PI-v1));
  double d2 = sqrtf((v2-M_PI)*(v2-M_PI));
  double pos = tloc*(d1+d2)/(d1+d2);
  double ori = 0.0;
  if(pos<=d1){
    ori=v1+pos;
  }else{
    //des(3)=-M_PI+pos-d1;
    ori=-M_PI+(pos-d1);
  }
  return ori;
}
//tloc in [0,1]
Config PathPiecewiseLinearSE3::ConfigOnSegment(int i, int j, double tloc) const{
  Config q1 = keyframes.at(i);
  Config q2 = keyframes.at(j);
  Config qdes = q1+tloc*(q2-q1);

  if(boundary_cross_yaw.at(i)){
    //distance is at least M_PI
    double v1 = q1(3);
    double v2 = q2(3);
    if(v1>v2){
      //std::cout << "over edge" << qdes(3) << "->";
      qdes(3) = GetBoundaryOrientation(v1,v2,tloc);
      //std::cout << qdes(3) << std::endl;
    }else{
      //std::cout << "under edge" << qdes(3) << "->";
      qdes(3) = GetBoundaryOrientation(v2,v1,tloc);
      //std::cout << qdes(3) << std::endl;
    }
  }

  return qdes;
}

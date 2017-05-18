#pragma once
#include <string>
#include <KrisLibrary/robotics/RobotKinematics3D.h> //Config

struct PlannerInput{
  std::string name_robot;
  std::string name_algorithm;
  Config q_init;
  Config q_goal;
  Config qMin;
  Config qMax;
  Config se3min;
  Config se3max;

  friend std::ostream& operator<< (std::ostream& out, const PlannerInput& pin) 
  {
    out << std::string(80, '-') << std::endl;
    out << "[PlannerInput]" << std::endl;
    out << std::string(80, '-') << std::endl;
    out << "q_init     : " << pin.q_init << std::endl;
    out << "q_goal     : " << pin.q_goal << std::endl;
    out << "q_min      : " << pin.qMin << std::endl;
    out << "q_max      : " << pin.qMax << std::endl;
    out << "algorithm  : " << pin.name_algorithm << std::endl;
    out << std::string(80, '-') << std::endl;
    return out;
  }
};


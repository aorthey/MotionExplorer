#pragma once
#include <string>
#include <KrisLibrary/robotics/RobotKinematics3D.h> //Config

struct PlannerInput{
  std::string name_robot;
  std::string name_algorithm;

  Config q_init;
  Config q_goal;
  Config dq_init;
  Config dq_goal;

  Config qMin;
  Config qMax;

  int robot_idx;
  int robot_idx_outer_shell;

  double epsilon_goalregion;
  double max_planning_time;
  double timestep_min, timestep_max;

  Config se3min;
  Config se3max;

  int drawTree;
  int drawSweptVolume;
  int drawMilestones;
  int drawStartGoal;

  friend std::ostream& operator<< (std::ostream& out, const PlannerInput& pin) 
  {
    out << std::string(80, '-') << std::endl;
    out << "[PlannerInput]" << std::endl;
    out << std::string(80, '-') << std::endl;
    out << "q_init            : " << pin.q_init << std::endl;
    out << "  dq_init         : " << pin.dq_init << std::endl;
    out << "q_goal            : " << pin.q_goal << std::endl;
    out << "  dq_goal         : " << pin.dq_goal << std::endl;
    out << "SE3_min           : " << pin.se3min << std::endl;
    out << "SE3_max           : " << pin.se3max << std::endl;
    out << "algorithm         : " << pin.name_algorithm << std::endl;
    out << "discr timestep    : [" << pin.timestep_min << "," << pin.timestep_max << "]" << std::endl;
    out << "max planning time : " << pin.max_planning_time << " (seconds)" << std::endl;
    out << "epsilon_goalregion: " << pin.epsilon_goalregion<< std::endl;
    out << "robot index       : " << pin.robot_idx << std::endl;
    out << "robot index       : " << pin.robot_idx_outer_shell << std::endl;
    out << std::string(80, '-') << std::endl;
    return out;
  }
};


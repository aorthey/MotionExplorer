//#include "util.h"
#include "planner/planner.h"
#include "planner/cspace_factory.h"
#include "planner/planner_strategy_geometric.h"
#include <KrisLibrary/utils/stringutils.h>

MotionPlanner::MotionPlanner(RobotWorld *world_, PlannerInput& input_):
  world(world_), input(input_)
{
}
PlannerOutput MotionPlanner::GetOutput(){
  return output;
}
PlannerInput MotionPlanner::GetInput(){
  return input;
}

bool MotionPlanner::IsFeasible( Robot *robot, SingleRobotCSpace &cspace, Config &q){
  if(!cspace.IsFeasible(q)) {
    std::cout << std::string(80, '*') << std::endl;
    std::cout << "Robot " << robot->name << std::endl;
    std::cout << "Config " << q << std::endl;
    std::cout << std::string(80, '*') << std::endl;
    vector<bool> infeasible;
    cspace.CheckObstacles(q,infeasible);
    uint N = 0;
    for(size_t i=0;i<infeasible.size();i++){
      if(!infeasible[i]) cout<<"  ok "<<cspace.ObstacleName(i)<<endl;

      if(infeasible[i]){
        N++;
        int icq = i - q.size();
        cout<<"-->"<<cspace.ObstacleName(i) << " | pen-depth: ";
        cout << cspace.collisionQueries[icq].PenetrationDepth() << " | dist: ";
        cout << cspace.collisionQueries[icq].Distance(1e-3,1e-3) << std::endl;
      }
    }
    std::cout << N << "/" << cspace.collisionQueries.size() << " queries are in collision."<< std::endl;
    cout<<"configuration is infeasible, violated "<<N<<" constraints:"<<endl;
    std::cout << q << std::endl;
    std::cout << std::string(80, '*') << std::endl;
    return false;
  }
  //check joint limits
  for(int i = 0; i < robot->q.size(); i++){
    if(q(i) < robot->qMin(i) || q(i) > robot->qMax(i)){
      std::cout << std::string(80, '*') << std::endl;
      std::cout << "ERROR!" << std::endl;
      std::cout << std::string(80, '*') << std::endl;
      std::cout << "Joint limits invalid for configuration" << std::endl;
      std::cout << q << std::endl;
      std::cout << "entry "<<i<< " violation: " << robot->qMin(i) << " < " << q(i) << " < " << robot->qMax(i) << std::endl;
      std::cout << std::string(80, '*') << std::endl;
      return false;
    }
  }
  return true;
}
bool MotionPlanner::solve()
{
  if(!input.exists){
    std::cout << "No Planner Settings founds." << std::endl;
    return false;
  }
  std::string algorithm = input.name_algorithm;
  if(algorithm=="" || algorithm=="NONE"){
    std::cout << "No Planner Algorithm detected" << std::endl;
    return false;
  }
  std::vector<int> idxs = input.robot_idxs;

  Config p_init = input.q_init;
  Config p_goal = input.q_goal;

  assert(p_init.size() == p_goal.size());

  this->world->InitCollisions();
  std::cout << input << std::endl;

  //###########################################################################
  // Setup Klampt CSpace
  //###########################################################################

  WorldPlannerSettings worldsettings;
  worldsettings.InitializeDefault(*world);

  CSpaceFactory factory(input);
  CSpaceOMPL* cspace;

  SingleRobotCSpace* kcspace;
  SingleRobotCSpace* cspace_nested;
  SingleRobotCSpace* cspace_inner;
  SingleRobotCSpace* cspace_outer;

  //################################################################################
  if(StartsWith(algorithm.c_str(),"hierarchical")) {

    if(algorithm.size()<14){
      std::cout << "Error Hierarchical Algorithm: \n             Usage hierarchical:<algorithm>  \n            Example: hierarchical:ompl:rrt" << std::endl;
      std::cout << "your input: " << algorithm.c_str() << std::endl;
      exit(0);
    }
    input.name_algorithm = algorithm.substr(13,algorithm.size()-13);

    for(uint k = 0; k < idxs.size(); k++){
      uint ridx = idxs.at(k);
      output.nested_idx.push_back(ridx);
      Robot *rk = world->robots[ridx];

      Config qi = p_init; qi.resize(rk->q.size());
      Config qg = p_goal; qg.resize(rk->q.size());

      output.nested_q_init.push_back(qi);
      output.nested_q_goal.push_back(qg);
      output.hierarchy.AddLevel( ridx, qi, qg);
    }

    //remove all nested robots except the original one
    for(uint k = 0; k < idxs.size()-1; k++){
      output.removable_robot_idxs.push_back(idxs.at(k));
    }

    std::cout << std::string(80, '-') << std::endl;
    std::cout << "Hierarchical Planner: " << std::endl;
    std::cout << std::string(80, '-') << std::endl;
    std::cout << " Robots  " << std::endl;
    for(uint k = 0; k < output.nested_idx.size(); k++){
      uint ridx = output.nested_idx.at(k);
      Robot *rk = world->robots[ridx];
      std::cout << " Level" << k << "         : idx " << ridx << " name " << rk->name << std::endl;
      Config qi = output.nested_q_init.at(k);
      Config qg = output.nested_q_goal.at(k);
      std::cout << "   qinit        : " << qi << std::endl;
      std::cout << "   qgoal        : " << qi << std::endl;
    }

    if(input.freeFloating){
      //################################################################################
      //hierarchical: free floating
      //################################################################################
      for(uint i = 0; i < idxs.size(); i++){
        uint ridx = idxs.at(i);
        Robot *ri = world->robots[ridx];
        SingleRobotCSpace* cspace_klampt_i = new SingleRobotCSpace(*world,ridx,&worldsettings);
        CSpaceOMPL* cspace_i;
        cspace_i = factory.MakeGeometricCSpaceRotationalInvariance(ri, cspace_klampt_i);
        cspace_i->print();
        PlannerStrategyGeometric strategy;
        output.robot_idx = ridx;
        strategy.plan(input, cspace_i, output);
        return true;
      }


    }else{
      //################################################################################
      //hierarchical: contacts
      //################################################################################
      if(idxs.size() < 3){
        std::cout << "algorithm workspace requires at least ORIGINAL robot, INNER SHELL, OUTER SHELL robot" << std::endl;
        exit(0);
      }

      uint ridx = idxs.at(0);
      Robot *ri = world->robots[ridx];
      output.robot_idx = ridx;

      cspace_inner = new SingleRobotCSpace(*world,idxs.at(1),&worldsettings);
      cspace_outer = new SingleRobotCSpace(*world,idxs.at(2),&worldsettings);

      cspace = factory.MakeGeometricCSpaceRotationalInvariance(ri, cspace_inner);
      cspace = factory.MakeCSpaceDecoratorInnerOuter(cspace, cspace_outer);
    }
    //################################################################################
  }else{
//################################################################################
//Single Robot OMPL Cspace
//################################################################################
    if(idxs.size() < 1){
      std::cout << "Planner requires at least one robot to plan with" << std::endl;
      exit(0);
    }
    int robot_idx = input.robot_idxs.at(0);
    Robot *robot = world->robots[robot_idx];

    kcspace = new SingleRobotCSpace(*world,robot_idx,&worldsettings);

    if(!IsFeasible( robot, *kcspace, p_init)) return false;
    if(!IsFeasible( robot, *kcspace, p_goal)) return false;

    cspace = factory.MakeGeometricCSpace(robot, kcspace);
  }

  cspace->print();
  PlannerStrategyGeometric strategy;
  strategy.plan(input, cspace, output);
  return true;
}

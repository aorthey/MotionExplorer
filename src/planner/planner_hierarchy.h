#pragma once

#include "planner/planner.h"
#include "elements/swept_volume.h"
#include <KrisLibrary/robotics/RobotKinematics3D.h> //Config
#include "planner/cspace_factory.h"
#include "planner/planner_strategy_geometric.h"
#include <KrisLibrary/utils/stringutils.h>

class HierarchicalMotionPlanner: public MotionPlanner{

  public:
    HierarchicalMotionPlanner(RobotWorld *world_, PlannerInput& input_):
      MotionPlanner(world_, input_)
    {
      current_level = 0;
      current_level_node = 0;
      current_path.push_back(0);
    }

    virtual bool solve(){
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
        if(!input.freeFloating){
          std::cout << "Hierarchical planner works only with freefloating robots right now x__X;;" << std::endl;
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
      }

    }

    //folder-like operations on hierarchical path space
    void ExpandPath(){
      int Nmax=output.hierarchy.NumberLevels();
      if(current_level<Nmax-1) current_level++;
    }
    void CollapsePath(){
      if(current_level>0) current_level--;
    }

    void NextPath(){
      int Nmax=output.hierarchy.NumberNodesOnLevel(current_level);
      if(current_level_node<Nmax) current_level_node++;
      else current_level_node = 0;
    }
    void PreviousPath(){
      int Nmax=output.hierarchy.NumberNodesOnLevel(current_level);
      if(current_level_node>0) current_level_node--;
      else current_level_node = Nmax;
    }

    const std::vector<Config>& GetSelectedPath(){
      return output.hierarchy.GetPathFromNodes( current_path );
    }

    const SweptVolume& GetSelectedPathSweptVolume(){
      PathNode* node = output.hierarchy.GetPathNodeFromNodes( current_path );
      uint idx = output.hierarchy.GetRobotIdx( current_level );
      Robot *robot = world->robots[idx];
      return node->GetSweptVolume(robot);
    }

    Robot* GetSelectedPathRobot(){
      uint idx = output.hierarchy.GetRobotIdx( current_level );
      Robot *robot = world->robots[idx];
      return robot;
    }

    const Config GetSelectedPathInitConfig(){
      return output.hierarchy.GetInitConfig(current_level);
    }
    const Config GetSelectedPathGoalConfig(){
      return output.hierarchy.GetGoalConfig(current_level);
    }
    const std::vector<int>& GetSelectedPathIndices(){
      return current_path;
    }

    int GetCurrentLevel(){
      return current_level;
    }

    //for each level: first: number of all nodes, second: node selected on that
    //level. produces a tree of nodes with distance 1 to central path
    //const std::vector<std::pair<int,int> >& GetCaterpillarTreeIndices();
    
  private:
    int current_level;
    int current_level_node;
    std::vector<int> current_path;

    //PathspaceHierarchy hierarchy;

};


#pragma once
#include <Planning/PlannerSettings.h>
#include <Planning/ContactCSpace.h>
#include <Planning/RobotTimeScaling.h>
#include <Simulation/WorldSimulation.h>
#include <KrisLibrary/planning/AnyMotionPlanner.h>
#include <KrisLibrary/planning/KinodynamicMotionPlanner.h>
#include <Modeling/DynamicPath.h>
#include <Modeling/Paths.h>
#include <Modeling/MultiPath.h>

#include <tinyxml.h>
#include <vector>

#include "planner/serialized_tree.h"
#include "planner/planner_input.h"
#include "planner/planner_output.h"

class MotionPlanner{

  protected:
    PlannerInput input;
    PlannerOutput output;

    RobotWorld *_world;
    Robot *robot;
    int _irobot;
    int _icontroller;
    Config _p_init;
    Config _p_goal;
    bool _isSolved;

    SerializedTree _stree;

  public:

    PlannerInput GetInput();
    PlannerOutput GetOutput();

    explicit MotionPlanner(RobotWorld *world);
    const KinodynamicMilestonePath& GetPath();
    const std::vector<Config>& GetKeyframes();
    const SerializedTree& GetTree();

    void SerializeTree( const RoadmapPlanner& graph, SerializedTree& stree);
    void SerializeTree( const KinodynamicTree& tree, SerializedTree& stree);
    void SerializeTree( const KinodynamicTree::Node* node, SerializedTree &stree);
    void SerializeTreeAddCostToGoal(SerializedTree &stree, CSpace *base, Config &goal);
    //Delete all nodes from the tree with distance < epsilon to another node
    void SerializeTreeCullClosePoints(SerializedTree &_stree, CSpace *base, double epsilon=0.1);
    void SerializeTreeRandomlyCullPoints(SerializedTree &_stree, uint N=1000);

    virtual bool solve(Config &p_init, Config &p_goal, double timelimit=100.0, bool shortcutting=true){
      std::cout << "Error: deprecated function" << std::endl;
      exit(0);
    }
    virtual bool solve(PlannerInput& input_) = 0;

    //void SendCommandStringController(string cmd, string arg);
    //bool SendToController();
    bool IsFeasible(Robot *robot, SingleRobotCSpace &cspace, Config &q);

    virtual std::string getName();
};


// * The type field can be left as "any", in which a default planning algorithm will be
// * used. Otherwise, a given planner algorithm can be designated as follows:
// * - prm: the Probabilistic Roadmap algorithm
// * - lazyprm: the Lazy-PRM algorithm (interface not implemented yet)
// * - perturbation: the PerturbationTree algorithm (interface not implemented yet)
// * - est: the Expanding Space Trees algorithm (interface not implemented yet)
// * - rrt: the Rapidly Exploring Random Trees algorithm
// * - sbl: the Single-Query Bidirectional Lazy planner
// * - sblprt: the probabilistic roadmap of trees (PRT) algorithm with SBL as the inter-root planner.
// * - rrt*: the RRT* algorithm for optimal motion planning
// * - prm*: the PRM* algorithm for optimal motion planning
// * - lazyprm*: the Lazy-PRM* algorithm for optimal motion planning
// * - lazyrrg*: the Lazy-RRG* algorithm for optimal motion planning
// * - fmm: the fast marching method algorithm for resolution-complete optimal motion planning
// * - fmm*: an anytime fast marching method algorithm for optimal motion planning
// * 
// * If KrisLibrary is built with OMPL support, you can also use the type specifier
// * "ompl:[X]" where [X] is one of:
// * - prm, lazyprm, prm*, lazyprm*, spars
// * - rrt, rrtconnect, birrt, lazyrrt, lbtrrt, rrt*, informedrrt*
// * - est, fmt, sbl, stride
// * The appropriate OMPL planner will be created for that given type, and MotionPlannerFactory
// * parameters will be mapped as closely as possible to the OMPL parameters.
// * (tested with OMPL 1.1.0)

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
#include "planner/planner_output.h"

struct PlannerSettings{
  const uint iterations = 1e3;
  const double goalSeekProbability = 0.1;
  const double goalRegionConvergence = 0.1;
  const uint maxDisplayedPointsInTree = 1e4;
  const uint Nreachableset = 10; //for RGRRT
  const double discretizationOutputPath = 0.01;
  const Vector3 worldboundsMin = Vector3(-3,-3,1);
  const Vector3 worldboundsMax = Vector3(+3,+3,3);
};

class MotionPlanner{

  protected:

    PlannerSettings plannersettings;
    RobotWorld *_world;
    Robot *robot;
    int _irobot;
    int _icontroller;
    Config _p_init;
    Config _p_goal;
    bool _isSolved;
    bool _shortcutting;
    double _timelimit;
    //KinodynamicMilestonePath _path;
    SerializedTree _stree;
    PlannerOutput output;
  public:

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

    virtual bool solve(Config &p_init, Config &p_goal, double timelimit=100.0, bool shortcutting=true);

    //void SendCommandStringController(string cmd, string arg);
    //bool SendToController();
    bool IsFeasible(Robot *robot, SingleRobotCSpace &cspace, Config &q);

    virtual std::string getName();
    //virtual bool Save(const char* file=NULL);
    //virtual bool Save(TiXmlElement *node);
    //virtual bool Load(const char* file);
    //virtual bool Load(TiXmlElement *node);

  protected:
    //virtual bool solve_internal(Config &p_init, Config &p_goal);
};
//bool Save(const std::vector<Config> &keyframes, const char* file);
//bool Save(const std::vector<Config> &keyframes, TiXmlElement *node);
//bool Load(std::vector<Config> &keyframes, const char* file);
//bool Load(std::vector<Config> &keyframes, TiXmlElement *node);


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

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

struct SerializedTreeNode{
  Vector position; //\in R^6 local chart on SE(3)
  Vector config;
  std::vector<Vector3> directions;
  double cost_to_goal;
  //uint id;
  //uint parentid;
  //std::vector<int> parentids;
  Vector3 GetXYZ(){
    Vector3 pos;
    pos[0] = position[0];
    pos[1] = position[1];
    pos[2] = position[2];
    return pos;
  }
  bool Save(TiXmlElement *node)
  {
    node->SetValue("node");

    {
      TiXmlElement cc("position");
      stringstream ss;
      ss<<position;
      TiXmlText text(ss.str().c_str());
      cc.InsertEndChild(text);
      node->InsertEndChild(cc);
    }
    {
      TiXmlElement cc("config");
      stringstream ss;
      ss<<config;
      TiXmlText text(ss.str().c_str());
      cc.InsertEndChild(text);
      node->InsertEndChild(cc);
    }
    {
      TiXmlElement cc("directions");
      for(int i = 0; i < directions.size(); i++){
        TiXmlElement ccc("d");
        stringstream ss;
        ss<<directions.at(i);
        TiXmlText text(ss.str().c_str());
        ccc.InsertEndChild(text);
        cc.InsertEndChild(ccc);
      }
      node->InsertEndChild(cc);
    }
    {
      TiXmlElement cc("cost_to_goal");
      stringstream ss;
      ss<<cost_to_goal;
      TiXmlText text(ss.str().c_str());
      cc.InsertEndChild(text);
      node->InsertEndChild(cc);
    }

  }
  bool Load(TiXmlElement *node)
  {
    if(0!=strcmp(node->Value(),"node")) {
      fprintf(stderr,"SerializedTreeNode Load XML: node \"%s\" not of node type\n",node->Value());
      return false;
    }

    TiXmlElement* e=node->FirstChildElement();
    while(e != NULL) 
    {
      if(0==strcmp(e->Value(),"position")) {
        stringstream ss(e->GetText());
        ss >> position;
      }
      if(0==strcmp(e->Value(),"config")) {
        stringstream ss(e->GetText());
        ss >> config;
      }
      if(0==strcmp(e->Value(),"cost_to_goal")) {
        stringstream ss(e->GetText());
        ss >> cost_to_goal;
      }
      if(0==strcmp(e->Value(),"directions")) {
        TiXmlElement* d=e->FirstChildElement();
        while(d != NULL)
        {
          if(d->GetText()!=NULL) {
            stringstream ss(d->GetText());
            Vector3 dir;
            ss >> dir;
            directions.push_back(dir);
            d = d->NextSiblingElement();
          }
        }
      }
      e = e->NextSiblingElement();
    }
    return true;

  }
};

typedef std::vector< SerializedTreeNode > SerializedTree;

struct PlannerSettings{
  const uint iterations = 1e2;
  const double goalSeekProbability = 1;
  const double goalRegionConvergence = 0.1;
  const uint maxDisplayedPointsInTree = 1e4;
  const uint Nreachableset = 1e2;
  const double discretizationOutputPath = 0.01;
  const Vector3 worldboundsMin = Vector3(-4,-4,1);
  const Vector3 worldboundsMax = Vector3(+4,+4,3);
};

class MotionPlanner{

  private:
    PlannerSettings plannersettings;
    RobotWorld *_world;
    int _irobot;
    int _icontroller;
    WorldSimulation *_sim;
    Config _p_init;
    Config _p_goal;
    bool _isSolved;
    bool _shortcutting;
    double _timelimit;
    KinodynamicMilestonePath _path;
    std::vector<Config> _keyframes;
    SerializedTree _stree;

  public:

    explicit MotionPlanner(RobotWorld *world, WorldSimulation *sim);
    const KinodynamicMilestonePath& GetPath();
    const std::vector<Config>& GetKeyframes();
    const SerializedTree& GetTree();
    void SerializeTree( const KinodynamicTree& tree, SerializedTree& stree);
    void SerializeTree( const KinodynamicTree::Node* node, SerializedTree &stree);
    void SerializeTreeAddCostToGoal(SerializedTree &stree, CSpace *base, Config &goal);
    //Delete all nodes from the tree with distance < epsilon to another node
    void SerializeTreeCullClosePoints(SerializedTree &_stree, CSpace *base, double epsilon=0.1);
    void SerializeTreeRandomlyCullPoints(SerializedTree &_stree, uint N=1000);

    bool solve(Config &p_init, Config &p_goal, double timelimit=100.0, bool shortcutting=true);
    void SendCommandStringController(string cmd, string arg);
    bool SendToController();
    void CheckFeasibility(Robot *robot, SingleRobotCSpace &cspace, Config &q);

    virtual bool PlanPath();
    void PostProcess();

    //void Save();
    //void SavePath();
    //void SaveTree();

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

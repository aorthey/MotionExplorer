#include "planner/planner.h"
#include "util.h"
#include "cspace_sentinel.h"
#include "cspace_epsilon_neighborhood.h"

MotionPlanner::MotionPlanner(RobotWorld *world, WorldSimulation *sim):
  _world(world),_sim(sim)
{
  _irobot = 0;
  _icontroller = 0;
}
const KinodynamicMilestonePath& MotionPlanner::GetPath()
{
  return _path;
}
const std::vector<Config>& MotionPlanner::GetKeyframes()
{
  return _keyframes;
}
const SerializedTree& MotionPlanner::GetTree()
{
  return _stree;
}

void MotionPlanner::SerializeTreeCullClosePoints(SerializedTree &_stree, CSpace *base, double epsilon)
{
  uint N_nodes_erased = 0;
  uint N_nodes_start = _stree.size();

  uint curiter = 0;
  uint nextiter = curiter+1;
  std::cout << "culling points" << std::endl;
  while(curiter < _stree.size()){
    std::cout << curiter << "/" << _stree.size() << std::endl;
    SerializedTreeNode cur = _stree.at(curiter);
    nextiter = curiter + 1;

    while(nextiter < _stree.size()){

      SerializedTreeNode next = _stree.at(nextiter);
      double d = base->Distance(cur.position, next.position);
      if(d<epsilon){
        //std::vector<Vector3> dirs = next.directions;
        //cur.directions.insert( cur.directions.end(), dirs.begin(), dirs.end() );
        _stree.erase(_stree.begin()+nextiter);
        //std::cout << "erasing node" << nextiter  << " (too close to node "<<curiter <<")"<< std::endl;
        nextiter --; N_nodes_erased++;
      }
      nextiter++;
    }
    curiter++;
  }
  std::cout << "Erased " << N_nodes_erased << "/" << N_nodes_start << std::endl;
}
//delete all node except N randomly choosen ones
void MotionPlanner::SerializeTreeRandomlyCullPoints(SerializedTree &_stree, uint N)
{

  uint Nall = _stree.size();

  if(N>=Nall) return;
  
  SerializedTree snew;
  SerializedTreeNode si;
  si = _stree.at(0);
  si.directions.clear();
  snew.push_back(si);
  for(int i = 1; i < N; i++){
    si = _stree.at(RandInt(Nall));
    si.directions.clear();
    snew.push_back(si);
  }
  _stree.clear();
  _stree=snew;

  ////set new directions
  //for(uint i = 0; i < snew.size(); i++){
  //  SerializedTreeNode parent = snew.at(i);
  //  uint parentid = parent.id;

  //  for(uint j = 0; j < snew.size(); j++){

  //    SerializedTreeNode child = snew.at(j);
  //    if(child.parentid==parentid)
  //    {
  //      Vector3 v = child.GetXYZ()-parent.GetXYZ();
  //      parent.directions.add(v);

  //    si.directions.clear();
  //    snew.push_back(si);
  //  }
  //}
}

void MotionPlanner::SerializeTreeAddCostToGoal(SerializedTree &stree, CSpace *base, Config &goal)
{
  for(uint i = 0; i < stree.size(); i++){
    stree.at(i).cost_to_goal = base->Distance(stree.at(i).config, goal);
  }
}

#include <KrisLibrary/graph/Graph.h>
void MotionPlanner::SerializeTree( const RoadmapPlanner& graph, SerializedTree& stree)
{
  std::cout << "serializing tree with " << graph.roadmap.nodes.size() << " nodes" << std::endl;
  for(uint i = 0; i < graph.roadmap.nodes.size(); i++)
  {
    SerializedTreeNode snode;
    snode.config = graph.roadmap.nodes.at(i);
    snode.position.resize(6);
    for(int i = 0; i < 6; i++){
      snode.position(i) = snode.config(i);
    }

    Vector3 vnode(snode.position(0),snode.position(1),snode.position(2));
      //typedef std::map<int,EdgeDataPtr> EdgeList;

    //TODO: do in O(n)
    uint Nedges = graph.roadmap.edges.at(i).size();
    uint Ncoedges = graph.roadmap.co_edges.at(i).size();
    //typedef typename std::list<EdgeData>::iterator EdgeDataPtr;
    //typedef std::map<int,EdgeDataPtr> EdgeList;

    RoadmapPlanner::Roadmap::EdgeList edges = graph.roadmap.edges.at(i);
    RoadmapPlanner::Roadmap::CoEdgeList co_edges = graph.roadmap.co_edges.at(i);
    typedef RoadmapPlanner::Roadmap::EdgeList::iterator EdgeIterator;
    typedef RoadmapPlanner::Roadmap::CoEdgeList::iterator CoEdgeIterator;

    for(EdgeIterator it = edges.begin(); it!=edges.end(); ++it)
    {
      //std::cout << it->first << std::endl;
      uint j = it->first;
      Config goal = graph.roadmap.nodes.at(j);
      Vector3 vgoal(goal(0),goal(1),goal(2));
      Vector3 v;
      v = vgoal - vnode;
      snode.directions.push_back(v);
    }
    for(CoEdgeIterator it = co_edges.begin(); it!=co_edges.end(); ++it)
    {
      uint j = it->first;
      Config goal = graph.roadmap.nodes.at(j);
      Vector3 vgoal(goal(0),goal(1),goal(2));
      Vector3 v;
      v = vnode - vgoal;
      snode.directions.push_back(v);
    }
    _stree.push_back(snode);
    //
    //      Config goal = graph.roadmap.nodes.at(j);
    //      Vector3 vgoal(goal(0),goal(1),goal(2));
    //      Vector3 v;
    //      v = vgoal - vnode;
    //      snode.directions.push_back(v);
    //    }
    //  }
    //  _stree.push_back(snode);

    //if(Nedges>0)
    //{
    //  for(uint j = 0; j < graph.roadmap.nodes.size(); j++)
    //  {
    //    if(graph.roadmap.HasEdge(i,j))
    //    {
    //      Config goal = graph.roadmap.nodes.at(j);
    //      Vector3 vgoal(goal(0),goal(1),goal(2));
    //      Vector3 v;
    //      v = vgoal - vnode;
    //      snode.directions.push_back(v);
    //    }
    //  }
    //  _stree.push_back(snode);
    //}
  }

}
void MotionPlanner::SerializeTree( const KinodynamicTree::Node* node, SerializedTree &stree)
{
  SerializedTreeNode snode;

  State s = *node;
  snode.position.resize(6);
  for(int i = 0; i < 6; i++){
    snode.position(i) = s(i);
  }
  snode.config = s;

  std::vector<Vector3> directions;

  std::vector<KinodynamicTree::Node* > children;
  node->enumChildren(children);
  for(uint i = 0; i < children.size(); i++){
    Vector3 childdir;
    State c = *children.at(i);
    for(int j = 0; j < 3; j++){
      childdir[j] = c(j)-snode.position(j);
    }
    directions.push_back(childdir);
    SerializeTree(children.at(i),stree);
  }
  snode.directions = directions;

  //DEBUG
  //std::cout << snode.first << std::endl;
  //std::cout << "Childrens: " << children.size() << std::endl;
  //for(int i = 0; i < directions.size(); i++){
  //  std::cout << snode.second.at(i) << std::endl;
  //}
  stree.push_back(snode);

}

void MotionPlanner::SerializeTree( const KinodynamicTree& tree, SerializedTree& stree){
  SerializeTree(tree.root, stree);
}
std::string MotionPlanner::getName(){
  return "Motion Planner";
}

bool MotionPlanner::IsFeasible( Robot *robot, SingleRobotCSpace &cspace, Config &q){
  if(!cspace.IsFeasible(q)) {
    std::cout << std::string(80, '*') << std::endl;
    std::cout << "ERROR!" << std::endl;
    std::cout << std::string(80, '*') << std::endl;
    cout<<"configuration is infeasible, violated constraints:"<<endl;
    std::cout << q << std::endl;
    vector<bool> infeasible;
    cspace.CheckObstacles(q,infeasible);
    for(size_t i=0;i<infeasible.size();i++){
      //if(!infeasible[i]) cout<<"  OK"<<cspace.ObstacleName(i)<<endl;
      if(infeasible[i]){
        int icq = i - (int)robot->joints.size();
        cout<<"-->"<<cspace.ObstacleName(i) << " | pen-depth: ";
        cout << cspace.collisionQueries[icq].PenetrationDepth() << " | dist: ";
        cout << cspace.collisionQueries[icq].Distance(1e-3,1e-3) << std::endl;
      }
    }
    std::cout << cspace.collisionQueries.size() << std::endl;
    std::cout << std::string(80, '*') << std::endl;
    return false;
  }
  //check that rotations are in [0,2pi]
  for(int i = 3; i < 6; i++){
    if(q(i)<0 || q(i)>2*M_PI){
      std::cout << std::string(80, '*') << std::endl;
      std::cout << "ERROR!" << std::endl;
      std::cout << std::string(80, '*') << std::endl;
      std::cout << "Rotation invalid for configuration" << std::endl;
      std::cout << q << std::endl;
      std::cout << "entry "<<i<< " " << q(i) << " outside of [0,2*pi]" << std::endl;
      std::cout << std::string(80, '*') << std::endl;
      return false;
    }
  }
  return true;
}

bool MotionPlanner::solve(Config &p_init, Config &p_goal, double timelimit, bool shortcutting)
{

  this->_p_init = p_init;
  this->_p_goal = p_goal;
  this->_timelimit = timelimit;
  this->_shortcutting = shortcutting;
  Robot *robot = _world->robots[_irobot];
  robot->UpdateConfig(_p_init);
  util::SetSimulatedRobot(robot,*_sim,_p_init);
  this->_world->InitCollisions();

  std::cout << std::string(80, '-') << std::endl;

  std::cout << "Motion Planner:" << this->getName() << std::endl;
  std::cout << "p_init =" << p_init << std::endl;
  std::cout << "p_goal =" << p_goal << std::endl;

  std::cout << std::string(80, '-') << std::endl;

  //###########################################################################
  // Planner Settings
  //###########################################################################

//** Real   collisionEpsilon threshold for edge feasibility checks
//** Vector  distanceWeights for non-euclidean distance metric
//** AABB3D  worldBounds base position sampling range for free-floating robots
//** Real  contactEpsilon convergence threshold for contact solving
//** int   contactIKMaxIters max iters for contact solving
//** PropertyMap   properties other properties 

  WorldPlannerSettings worldsettings;
  worldsettings.InitializeDefault(*_world);
  //worldsettings.robotSettings[0].worldBounds = AABB3D(plannersettings.worldboundsMin,plannersettings.worldboundsMax);
  //worldsettings.robotSettings[0].contactEpsilon = 1e-2;
  //worldsettings.robotSettings[0].contactIKMaxIters = 100;

  SingleRobotCSpace geometric_cspace = SingleRobotCSpace(*_world,_irobot,&worldsettings);
  //std::cout << cspace.settings->robotSettings[0].worldBounds << std::endl;

  //Check Kinematic Feasibility at init/goal positions
  if(!IsFeasible( robot, geometric_cspace, _p_init)) return false;
  if(!IsFeasible( robot, geometric_cspace, _p_goal)) return false;

  KinodynamicCSpaceSentinelAdaptor kcspace(&geometric_cspace);
  PropertyMap pmap;
  kcspace.Properties(pmap);
  std::cout << pmap << std::endl;

  CSpaceGoalSetEpsilonNeighborhood goalSet(&kcspace, _p_goal, plannersettings.goalRegionConvergence);

  bool geometric =false;


  if(!geometric){
    //###########################################################################
    // Dynamic Planning
    //###########################################################################


    //RRTKinodynamicPlanner krrt(&kcspace);
    ///////LazyRRTKinodynamicPlanner krrt(&kcspace);
    RGRRT krrt(&kcspace, plannersettings.Nreachableset);

    krrt.goalSeekProbability= plannersettings.goalSeekProbability;
    krrt.goalSet = &goalSet;
    krrt.Init(_p_init);

    ///////RRTKinodynamicPlanner2 krrt(&kcspace);
    ///////krrt.Init(_p_init,_p_goal);
    ///////BidirectionalRRTKP
    ///////BidirectionalRRTKP krrt(&kcspace);
    ///////UnidirectionalRRTKP krrt(&kcspace);
    ///////

    _isSolved = krrt.Plan(plannersettings.iterations);

    _stree.clear();
    SerializeTree(krrt.tree, _stree);
    SerializeTreeAddCostToGoal(_stree, &kcspace, _p_goal);
    ///////SerializeTreeCullClosePoints(_stree, &kcspace,0.3);
    ///////SerializeTreeRandomlyCullPoints(_stree, plannersettings.maxDisplayedPointsInTree);
    ///////SerializeTreeCost(krrt.tree, _stree, &goalSet);

    if(_isSolved)
    {
      std::cout << "Found solution path" << std::endl;
      KinodynamicMilestonePath path;
      krrt.CreatePath(path);

      Config cur;
      double dstep = plannersettings.discretizationOutputPath;
      vector<Config> keyframes;
      for(double d = 0; d <= 1; d+=dstep)
      {
        path.Eval(d,cur);
        std::cout << d << cur << std::endl;
        _keyframes.push_back(cur);
      }
      std::cout << std::string(80, '-') << std::endl;

    }else{
      std::cout << "[Motion Planner] No path found." << std::endl;
      std::cout << "Tree size " << _stree.size() << std::endl;
      for(int i = 0; i < _stree.size(); i++){
        std::cout << _stree.at(i).position << std::endl;
      }
    }
    return _isSolved;
  }else{
    //###########################################################################
    // Kinematic Planning
    //###########################################################################
    MotionPlannerFactory factory;
    //factory.perturbationRadius = 0.1;
    //factory.type = "rrt";
    //factory.type = "prm";
    //factory.type = "sbl";
    //factory.type = "sblprt";
    //factory.type = "rrt*";
    //factory.type = "lazyrrg*";
    //factory.type = "fmm"; //warning: slows down system 
    //factory.type = "sbl";
    //factory.type = "ompl:rrt";
    //factory.type = "ompl:est";
    //factory.type = "ompl:rrt*";
    //factory.type = "ompl:fmt";
    //factory.type = "ompl:sbl";
    factory.type = "ompl:rrtconnect";
    factory.shortcut = true;

    SmartPointer<MotionPlannerInterface> planner = factory.Create(&kcspace,_p_init,_p_goal);

    HaltingCondition cond;
    cond.foundSolution=false;
    cond.timeLimit = 5;

    std::cout << "Start Planning" << std::endl;
    MilestonePath mpath;
    string res = planner->Plan(mpath,cond);
    if(mpath.edges.empty())
    {
     printf("Planning failed\n");
     this->_isSolved = false;
    }else{
     printf("Planning succeeded, path has length %g\n",mpath.Length());
     this->_isSolved = true;
    }

    ////###########################################################################
    //// Extract roadmap from planner
    ////###########################################################################
    RoadmapPlanner roadmap(&kcspace);
    planner->GetRoadmap(roadmap);

    std::cout << roadmap.roadmap.NumNodes() << " Nodes" << std::endl;
    _stree.clear();
    SerializeTree(roadmap, _stree);
    SerializeTreeAddCostToGoal(_stree, &kcspace, _p_goal);


    ////###########################################################################
    //// Time Optimize Path, Convert to MultiPath and Save Path
    ////###########################################################################
    if(_isSolved)
    {
      std::cout << "Found solution path" << std::endl;
      //KinodynamicMilestonePath path;
      //planner.CreatePath(path);

      Config cur;
      double dstep = plannersettings.discretizationOutputPath;
      vector<Config> keyframes;
      for(double d = 0; d <= 1; d+=dstep)
      {
        mpath.Eval(d,cur);
        //std::cout << d << cur << std::endl;
        _keyframes.push_back(cur);
      }
      std::cout << std::string(80, '-') << std::endl;

    }else{
      std::cout << "[Motion Planner] No path found." << std::endl;
      std::cout << "Tree size " << _stree.size() << std::endl;
      for(int i = 0; i < _stree.size(); i++){
        std::cout << _stree.at(i).position << std::endl;
      }
    }
    return _isSolved;
  }
}

void MotionPlanner::SendCommandStringController(string cmd, string arg)
{
  if(!_sim->robotControllers[_icontroller]->SendCommand(cmd,arg)) {
    std::cout << std::string(80, '-') << std::endl;
    std::cout << "ERROR in controller commander" << std::endl;
    std::cout << cmd << " command  does not work with the robot's controller" << std::endl;
    std::cout << std::string(80, '-') << std::endl;
    throw "Controller command not supported!";
  }
}
bool MotionPlanner::SendToController()
{
  if(!_isSolved){ return false; }

  double dstep = 0.1;
  Config q;
  Config dq;

  for(int i = 0; i < _keyframes.size()-1; i++){
    //_path.Evaluate(d, q, dq);
    q = _keyframes.at(i);
    Config q2 = _keyframes.at(i+1);
    double dt = 1.0/_keyframes.size();
    dq = (q-q2)/dt;
    stringstream qstr;
    qstr<<q<<dq;
    string cmd( (i<=0)?("set_qv"):("append_qv") );
    SendCommandStringController(cmd,qstr.str());
  }
  //{
  //  _path.Evaluate(d, q, dq);
  //  stringstream qstr;
  //  qstr<<q<<dq;
  //  string cmd( (d<=0)?("set_qv"):("append_qv") );
  //  SendCommandStringController(cmd,qstr.str());
  //}

  std::cout << "Sending Path to Controller" << std::endl;
  //Robot *robot = _world->robots[_irobot];
  //util::SetSimulatedRobot(robot,*_sim,_p_init);
  //robot->UpdateConfig(_p_goal);
  std::cout << "Done Path to Controller" << std::endl;
  return true;
}


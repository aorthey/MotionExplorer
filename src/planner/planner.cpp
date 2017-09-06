#include "planner/planner.h"
#include "util.h"
#include "info.h"
//#include "cspace_sentinel.h"
//#include "cspace_epsilon_neighborhood.h"

MotionPlanner::MotionPlanner(RobotWorld *world_, PlannerInput& input_):
  world(world_), input(input_)
{
  if(input.exists){
    _irobot = input.robot_idx;
    robot = world->robots[_irobot];
    _icontroller = 0;
    output.q_init = input.q_init;
    output.q_goal= input.q_goal;

    output.name_algorithm = input.name_algorithm;
    output.drawTree = input.drawTree;
    output.drawSimplicialComplex = input.drawSimplicialComplex;
    output.drawSweptVolume = input.drawSweptVolume;
    output.drawMilestones = input.drawMilestones;
    output.drawStartGoal = input.drawStartGoal;
  }else{
    _irobot = 0;
    robot = world->robots[_irobot];
    _icontroller = 0;

    //output.robot_idx = _irobot;
    //output.robot = robot;
    output.drawTree = 0;
    output.drawSimplicialComplex = 0;
    output.drawSweptVolume = 0;
    output.drawMilestones = 0;
    output.drawStartGoal = 0;
  }

}
PlannerOutput MotionPlanner::GetOutput(){
  return output;
}
PlannerInput MotionPlanner::GetInput(){
  return input;
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
  for(uint i = 1; i < N; i++){
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

    Vector vnode;vnode.resize(3);
    for(uint k = 0; k < 3; k++) vnode(k) = snode.position(k);

    //TODO: do in O(n)
    //uint Nedges = graph.roadmap.edges.at(i).size();
    //uint Ncoedges = graph.roadmap.co_edges.at(i).size();
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
      Vector vgoal;vgoal.resize(3);
      for(uint k = 0; k < 3; k++) vgoal(k) = goal(k);
      Vector v = vgoal - vnode;
      snode.directions.push_back(v);
    }
    for(CoEdgeIterator it = co_edges.begin(); it!=co_edges.end(); ++it)
    {
      uint j = it->first;
      Config goal = graph.roadmap.nodes.at(j);
      Vector vgoal;vgoal.resize(3);
      for(uint k = 0; k < 3; k++) vgoal(k) = goal(k);
      Vector v = vnode - vgoal;
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

  std::vector<Vector> directions;

  std::vector<KinodynamicTree::Node* > children;
  node->enumChildren(children);
  for(uint i = 0; i < children.size(); i++){
    Vector childdir;childdir.resize(3);
    State c = *children.at(i);
    for(int j = 0; j < 3; j++){
      childdir[j] = c(j)-snode.position(j);
    }
    directions.push_back(childdir);
    SerializeTree(children.at(i),stree);
  }
  snode.directions = directions;

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

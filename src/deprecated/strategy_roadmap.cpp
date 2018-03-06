#include "planner/strategy/strategy_roadmap.h"
#include <boost/foreach.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/incremental_components.hpp>
#include <boost/pending/disjoint_sets.hpp>

StrategyRoadmap::StrategyRoadmap(CSpaceOMPL *cspace){

  og::SimpleSetup ss(cspace->SpacePtr());
  const ob::SpaceInformationPtr si = ss.getSpaceInformation();
  ss.setStateValidityChecker(cspace->StateValidityCheckerPtr());
  ob::ProblemDefinitionPtr pdef = ss.getProblemDefinition();

  roadmap_planner = std::make_shared<og::PRM>(si);
  roadmap_planner->setProblemDefinition(pdef);
  roadmap_planner->setup();
  double t = 0.001;
  roadmap_planner->growRoadmap(t);

  using namespace boost;
  typedef ompl::geometric::PRM::Graph OMPLRoadmap;
  typedef boost::graph_traits< OMPLRoadmap >::vertex_descriptor Vertex;
  typedef boost::graph_traits< OMPLRoadmap >::vertices_size_type VertexIndex;
  typedef VertexIndex* Rank;
  typedef Vertex* Parent;
  typedef component_index<VertexIndex> Components;

  OMPLRoadmap g = roadmap_planner->getRoadmap();

  N_FeasibleVertices = num_vertices(g);
  std::vector<VertexIndex> rank(N_FeasibleVertices);
  std::vector<Vertex> parent(N_FeasibleVertices);

  disjoint_sets<Rank, Parent> ds(&rank[0], &parent[0]);

  initialize_incremental_components(g, ds);
  incremental_components(g, ds);

  Components components(parent.begin(), parent.end());
  N_ConnectedComponents = components.size();
  std::cout << "after " << t << " seconds we found " << N_ConnectedComponents << " components with " << N_FeasibleVertices << " vertices on cspace slice." << std::endl;

  //BOOST_FOREACH(VertexIndex current_index, components) {
  //  std::cout << "component " << current_index << " contains: ";

  //  //components[current_index]
  //  // Iterate through the child vertex indices for [current_index]
  //  BOOST_FOREACH(VertexIndex child_index,
  //                components[current_index]) {
  //    std::cout << child_index << " ";
  //  }

  //  std::cout << std::endl;
  //}

}
bool StrategyRoadmap::hasFeasibleVertex(){
  return (N_FeasibleVertices>0);
}
int StrategyRoadmap::NumberConnectedComponents(){
  return N_ConnectedComponents;
}

void StrategyRoadmap::planMore(){
}

void StrategyRoadmap::plan( const StrategyInput &input, StrategyOutput &output){
  exit(0);
}


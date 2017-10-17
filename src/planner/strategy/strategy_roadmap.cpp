#include "planner/strategy/strategy_roadmap.h"
#include <boost/foreach.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/incremental_components.hpp>
#include <boost/pending/disjoint_sets.hpp>

StrategyRoadmap::StrategyRoadmap(CSpaceOMPL *cspace){

  og::SimpleSetup ss(cspace->SpacePtr());
  const ob::SpaceInformationPtr si = ss.getSpaceInformation();
  ss.setStateValidityChecker(cspace->StateValidityCheckerPtr(si));
  ob::ProblemDefinitionPtr pdef = ss.getProblemDefinition();

  roadmap_planner = std::make_shared<og::PRM>(si);
  roadmap_planner->setProblemDefinition(pdef);
  roadmap_planner->setup();
  double t = 0.01;
  roadmap_planner->growRoadmap(t);
  uint N = roadmap_planner->milestoneCount();

  using namespace boost;

  // typedef adjacency_list <vecS, vecS, undirectedS> Graph;
  // typedef graph_traits<Graph>::vertex_descriptor Vertex;
  // typedef graph_traits<Graph>::vertices_size_type VertexIndex;

  typedef ompl::geometric::PRM::Graph OMPLRoadmap;
  typedef boost::graph_traits< OMPLRoadmap >::vertex_descriptor Vertex;
  typedef boost::graph_traits< OMPLRoadmap >::vertices_size_type VertexIndex;
  OMPLRoadmap g = roadmap_planner->getRoadmap();

  std::vector<VertexIndex> rank(num_vertices(g));
  std::vector<Vertex> parent(num_vertices(g));


  typedef VertexIndex* Rank;
  typedef Vertex* Parent;

  disjoint_sets<Rank, Parent> ds(&rank[0], &parent[0]);

  initialize_incremental_components(g, ds);
  incremental_components(g, ds);

  typedef component_index<VertexIndex> Components;
  Components components(parent.begin(), parent.end());
  uint Nc = components.size();
  std::cout << "after " << t << " seconds we found " << Nc << " components with " << N << " vertices on cspace slice." << std::endl;

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

void StrategyRoadmap::plan( const StrategyInput &input, CSpaceOMPL *cspace, StrategyOutput &output){
  exit(0);
}


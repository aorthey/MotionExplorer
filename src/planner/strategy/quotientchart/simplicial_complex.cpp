#include "simplicial_complex.h"
#include "common.h"
#include <ompl/tools/config/SelfConfig.h>
#include <Eigen/Core>

using namespace ompl::base;
using namespace Eigen;
using namespace ompl::geometric;
using namespace ompl::geometric::topology;

#define DEBUG 
#undef DEBUG

SimplicialComplex::SimplicialComplex(ob::SpaceInformationPtr si_, ob::Planner* planner_, double epsilon_max_neighborhood_):
  si(si_), epsilon_max_neighborhood(epsilon_max_neighborhood_), max_dimension(si_->getStateDimension())
{
  nn_feasible.reset(tools::SelfConfig::getDefaultNearestNeighbors<Vertex>(planner_));
  nn_feasible->setDistanceFunction([this](const Vertex a, const Vertex b)
                           {
                             return Distance(a,b);
                           });
  nn_infeasible.reset(tools::SelfConfig::getDefaultNearestNeighbors<Vertex>(planner_));
  nn_infeasible->setDistanceFunction([this](const Vertex a, const Vertex b)
                           {
                             return Distance(a,b);
                           });
  //k_simplices.resize(max_dimension+1);
  hasse_diagram.SetMaxDimension(max_dimension);
}

void SimplicialComplex::AddStart(const ob::State *s)
{
  Vertex v = AddFeasible(s);
  graph[v].isStart = true;
}

void SimplicialComplex::AddGoal(const ob::State *s)
{
  Vertex v = AddFeasible(s);
  graph[v].isGoal = true;
}

double SimplicialComplex::Distance(const Vertex a, const Vertex b)
{
  return si->distance(graph[a].state, graph[b].state);
}

bool SimplicialComplex::HaveIntersectingSpheres(const Vertex a, const Vertex b)
{
  double d_overall = Distance(a, b);
  double d_a = graph[a].open_neighborhood_distance;
  double d_b = graph[b].open_neighborhood_distance;
  return ((d_a+d_b) >= d_overall);
}

//#############################################################################
//Remove Simplices from Complex
//#############################################################################

bool SimplicialComplex::EdgeExists(const Vertex a, const Vertex b)
{
  return boost::edge(a, b, graph).second;
}

void SimplicialComplex::RemoveEdge(const Vertex a, const Vertex b)
{
  Edge e = boost::edge(a, b, graph).first;
  HasseDiagram::SimplexNode sn = graph[e].simplex_representation;
  hasse_diagram.RemoveNode(sn);
  boost::remove_edge(e, graph);
}

// void SimplicialComplex::RemoveSimplexNode(SimplexNode s)
// {
//   SC_OEIterator eo, eo_end, next;
//   tie(eo, eo_end) = boost::out_edges(s, S);
//   for (next = eo; eo != eo_end; eo = next) {
//     ++next;
//     SimplexNode sn = boost::target(*eo, S);
//     RemoveSimplexNode(sn);
//   }

//   //remove simplex from k-simplices map
//   uint N = S[s].vertices.size()-1;
//   auto it = k_simplices.at(N).find(S[s].vertices);

//   if(it!=k_simplices.at(N).end()){
//     k_simplices.at(N).erase(it);
//   }else{
//     std::cout << "removing simplex " << S[s].vertices << std::endl;
//     std::cout << "simplex " << S[s].vertices << " has not been found in map." << std::endl;
//     std::cout << k_simplices.at(N) << std::endl;
//     exit(0);
//   }

//   //remove facet ptrs
//   boost::clear_vertex(s, S);
//   //remove simplex from hasse diagram
//   boost::remove_vertex(s, S);
// }

// SimplicialComplex::SimplexNode SimplicialComplex::AddSimplexNode(std::vector<Vertex> v)
// {
//   std::sort(v.begin(), v.end());
//   uint N = v.size()-1;
//   auto it = k_simplices.at(N).find(v);
//   if(it!=k_simplices.at(N).end()){
//     return (*it).second;
//   }else{
//     SimplexNode s = boost::add_vertex(S);
//     S[s].vertices = v;
//     k_simplices.at(v.size()-1)[v] = s;
//     if(v.size()>2) HasseDiagramAddIncomingEdges(s);
//     return s;
//   }
// }

// void SimplicialComplex::HasseDiagramAddIncomingEdges(SimplexNode sigma)
// {
//   const std::vector<Vertex>& vertices = S[sigma].vertices;
//   uint N = vertices.size();
//   uint N_facet_dimension = N-2;
  
//   //iterate over all facets by taking all vertex permuatations of size N-1
//   std::string bitmask(N-1, 1); // N-1 leading 1's
//   bitmask.resize(N, 0); // 1 trailing 0

//   do{
//     std::vector<unsigned long int> facet;
//     for (uint i = 0; i < N; i++)
//     {
//       if (bitmask[i]) facet.push_back(vertices.at(i));
//     }
//     auto it = k_simplices.at(N_facet_dimension).find(facet);
//     SimplexNode tau;
//     if(it == k_simplices.at(N_facet_dimension).end())
//     {
//       //std::cout << "tried adding coface " << vertices << " to facet " << facet << std::endl;
//       //std::cout << "BUT: facet " << facet << " does not exist in simplex_map" << std::endl;
//       //std::cout << "simplex map: " << k_simplices.at(N_facet_dimension) << std::endl;
//       //exit(0);
//       tau = AddSimplexNode(facet);
//     }else{
//       tau = (*it).second;
//     }
//     boost::add_edge(tau, sigma, S);
//   }while(std::prev_permutation(bitmask.begin(), bitmask.end()));
// }


//#############################################################################
//Add edge and simplices
//#############################################################################

std::pair<SimplicialComplex::Edge, bool> SimplicialComplex::AddEdge(const Vertex a, const Vertex b)
{
  //#######################################################################
  //Add edge to 1-skeleton
  //#######################################################################
  double d = Distance(a,b);
  EdgeInternalState properties(d);
  std::pair<SimplicialComplex::Edge, bool> e = boost::add_edge(a, b, properties, graph);

  //#######################################################################
  //create simplex
  //#######################################################################
  std::vector<Vertex> vertices{a,b};
  graph[e.first].simplex_representation = hasse_diagram.AddNode(vertices);
  return e;
}

void SimplicialComplex::AddSimplices(const Vertex v, RoadmapNeighbors nn)
{
  //#######################################################################
  //Get all neighbors which will share an edge
  //#######################################################################
  std::vector<Vertex> suspected_neighbors;
  nn->nearestR(v, 2*epsilon_max_neighborhood, suspected_neighbors);

  std::vector<Vertex> neighbors;
  for(uint k = 0; k < suspected_neighbors.size(); k++){
    Vertex vk = suspected_neighbors.at(k);
    if(HaveIntersectingSpheres(v,vk)){
      AddEdge(v, vk);
      neighbors.push_back(vk);
    }
  }

  std::vector<Vertex> sigma; sigma.push_back(v);
  AddSimplex(sigma, neighbors);
}

void SimplicialComplex::AddSimplex( std::vector<Vertex>& sigma, std::vector<Vertex>& neighbors)
{
  if(sigma.size()>max_dimension) return;

  //we already added vertices for edge-simplices, so we do not want to have
  //double entries in that case
  //SimplexNode s = k_simplices.at(sigma.size())[sigma];
  if(sigma.size()>1){
    for(uint k = 0; k < neighbors.size(); k++)
    {
      Vertex vk = neighbors.at(k);
      std::vector<Vertex> tau(sigma); tau.push_back(vk);
      hasse_diagram.AddNode(tau);
    }
  }

  for(uint k = 0; k < neighbors.size(); k++){
    Vertex vk = neighbors.at(k);
    std::vector<Vertex> neighbors_of_neighbor;
    for(uint j = k+1; j < neighbors.size(); j++){
      Vertex vj = neighbors.at(j);
      if(EdgeExists(vk, vj))
      {
        neighbors_of_neighbor.push_back(vj);
      }
    }
    std::vector<Vertex> tau(sigma); tau.push_back(vk);
    AddSimplex(tau, neighbors_of_neighbor);
  }
}

//#############################################################################
//Add infeasible or feasible vertices to simplicial complex
//#############################################################################

//General Strategy for adding a feasible (infeasible) vertex
//(1) Add state to graph
//(2) get nearest infeasible (feasible) vertex. compute max sphere
//(3) update sphere radii:
//    for all nearest infeasible (feasible) vertices, check if this new sample
//    intersects any spheres. If yes, shrink sphere and remove edges between
//    infeasible (feasible) vertices.
//(4) connect to nearest feasible (infeasible) vertices:
//    get all feasible (infeasible) neighbors, and check if the spheres are intersecting. If yes,
//    connect them with an edge
//(5) add to feasible (infeasible) nearest neighbors 

SimplicialComplex::Vertex SimplicialComplex::AddInfeasible(const ob::State *s)
{
  Vertex v = Add(s, nn_infeasible, nn_feasible);
  graph[v].isInfeasible = true;
  return v;
}

SimplicialComplex::Vertex SimplicialComplex::AddFeasible(const ob::State *s)
{
  Vertex v = Add(s, nn_feasible, nn_infeasible, true);
  return v;
}

SimplicialComplex::Vertex SimplicialComplex::Add(const ob::State *s, RoadmapNeighbors nn_positive, RoadmapNeighbors nn_negative, bool addSimplices)
{
  //#######################################################################
  //Add vertex to graph
  //#######################################################################
  Vertex v = boost::add_vertex(graph);
  graph[v].state = si->cloneState(s);

  //#######################################################################
  //Compute size of the sphere
  //#######################################################################
  if(nn_negative->size()>0)
  {
    const Vertex vnegative = nn_negative->nearest(v);
    double dn = Distance(v, vnegative);
    graph[v].open_neighborhood_distance = std::min(dn, epsilon_max_neighborhood);
  }else{
    graph[v].open_neighborhood_distance = epsilon_max_neighborhood;
  }

  //#######################################################################
  //Connect to all intersecting samples, create local 1-skeleton
  //#######################################################################

  if(addSimplices)
  {
    AddSimplices(v, nn_positive);
  }

  //#######################################################################
  //Update all sphere radii of the negative neighbors. If a sphere radius is changed, remove edges
  //which are not contained in the sphere anymore
  //#######################################################################
  std::vector<Vertex> neighbors;
  nn_negative->nearestR(v, epsilon_max_neighborhood, neighbors);

  bool morphological_change = false;
  for(uint k = 0; k < neighbors.size(); k++){
    Vertex vk = neighbors.at(k);
    double ds_k = Distance(v, vk);
    double dold = graph[vk].open_neighborhood_distance;
    if(ds_k < dold){
      graph[vk].open_neighborhood_distance = ds_k;

      //sphere got updated. We need to remove all edges which have a segment
      //lying outside the sphere 
      OEIterator eo, eo_end, next;

      tie(eo, eo_end) = boost::out_edges(vk, graph);
      std::vector<Vertex> edge_removal;
      for (next = eo; eo != eo_end; eo = next) {
        ++next;
        const Vertex vkn = boost::target(*eo, graph);
        if(!HaveIntersectingSpheres(vk, vkn))
        {
          edge_removal.push_back(vkn);
        }
      }
      for(uint k = 0; k < edge_removal.size(); k++){
        const Vertex vkn = edge_removal.at(k);
        RemoveEdge(vk, vkn);
      }
      if(edge_removal.size()>0) morphological_change = true;
    }
  }
  if(morphological_change) ntry=0;
  else ntry++;
  ntry_over_iterations.push_back(ntry);

  //#######################################################################
  //add vertex to neighborhood structure
  //#######################################################################

  nn_positive->add(v);
  return v;
}

const SimplicialComplex::Graph& SimplicialComplex::GetGraph()
{
  return graph;
}

namespace ompl{ namespace geometric{ namespace topology{
  std::ostream& operator<< (std::ostream& out, const SimplicialComplex& sc)
  {
    out << "SimplicialComplex: " << std::endl;
    for(uint k = 0; k < sc.hasse_diagram.k_simplices.size(); k++){
      out << "simplices of size " << k << ":" << sc.hasse_diagram.k_simplices.at(k).size() << std::endl;
    }
    return out;
    return out;
  }
}}};

std::vector<std::vector<SimplicialComplex::Vertex>> SimplicialComplex::GetSimplicesOfDimension(uint k)
{
  std::vector<std::vector<SimplicialComplex::Vertex>> v;

  HasseDiagram::SimplicialComplexDiagram S = hasse_diagram.GetDiagram();

  std::cout << *this << std::endl;
  for(uint n = 2; n < k; n++){
    for(auto it=hasse_diagram.k_simplices.at(n).begin(); it!=hasse_diagram.k_simplices.at(n).end(); ++it)
    {
      uint cofaces = boost::out_degree(it->second, S);
      if(cofaces==0) v.push_back(it->first);
    }
  }
  return v;
}

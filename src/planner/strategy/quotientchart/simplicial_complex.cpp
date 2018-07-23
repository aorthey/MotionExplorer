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
  k_simplices.resize(max_dimension+1);
}

void SimplicialComplex::AddStart(const ob::State *s)
{
  Vertex v = AddFeasible(s);
  G[v].isStart = true;
}

void SimplicialComplex::AddGoal(const ob::State *s)
{
  Vertex v = AddFeasible(s);
  G[v].isGoal = true;
}

double SimplicialComplex::Distance(const Vertex a, const Vertex b)
{
  return si->distance(G[a].state, G[b].state);
}

bool SimplicialComplex::HaveIntersectingSpheres(const Vertex a, const Vertex b)
{
  double d_overall = Distance(a, b);
  double d_a = G[a].open_neighborhood_distance;
  double d_b = G[b].open_neighborhood_distance;
  return ((d_a+d_b) >= d_overall);
}

//#############################################################################
//Remove Simplices from Complex
//#############################################################################

bool SimplicialComplex::EdgeExists(const Vertex a, const Vertex b)
{
  return boost::edge(a, b, G).second;
}

void SimplicialComplex::RemoveEdge(const Vertex a, const Vertex b)
{
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "deleting edge : " << a << " - " << b << std::endl;
  Edge e = boost::edge(a, b, G).first;
  SimplexNode sn = G[e].simplex_representation;
  if(a != S[sn].vertices.at(0) && a != S[sn].vertices.at(1))
  {
    std::cout << "simplex representation differs." << std::endl;
    std::cout << "edge " << a << "-" << b << std::endl;
    std::cout << "simplex : " << S[sn].vertices << std::endl;
    exit(0);
  }
  RemoveSimplexNode(sn);
  boost::remove_edge(e, G);
}
void SimplicialComplex::RemoveSimplexNode(SimplexNode &s)
{
  std::cout << "simplex " << S[s].vertices << std::endl;
  SC_OEIterator eo, eo_end, next;
  tie(eo, eo_end) = boost::out_edges(s, S);
  for (next = eo; eo != eo_end; eo = next) {
    ++next;
    SimplexNode sn = boost::target(*eo, S);
    RemoveSimplexNode(sn);
  }

  //remove simplex from k-simplices map
  uint N = S[s].vertices.size();
  std::cout << "removing simplex " << S[s].vertices << std::endl;
  auto it = k_simplices.at(N).find(S[s].vertices);
  k_simplices.at(N).erase(it);

  //remove facet ptrs
  boost::clear_vertex(s, S);
  //remove simplex from hasse diagram
  boost::remove_vertex(s, S);

}

SimplicialComplex::SimplexNode SimplicialComplex::AddSimplexNode(std::vector<Vertex> v)
{
  std::sort(v.begin(), v.end());
  SimplexNode s = boost::add_vertex(S);
  std::cout << "add simplex : " << v << std::endl;
  S[s].vertices = v;
  k_simplices.at(v.size())[v] = &s;
  return s;
}


//#############################################################################
//Add edge and simplices
//#############################################################################

std::pair<SimplicialComplex::Edge, bool> SimplicialComplex::AddEdge(const Vertex a, const Vertex b)
{
  std::cout << "adding edge : " << a << "-" << b << std::endl;

  //#######################################################################
  //Add edge to 1-skeleton
  //#######################################################################
  double d = Distance(a,b);
  EdgeInternalState properties(d);
  std::pair<SimplicialComplex::Edge, bool> e = boost::add_edge(a, b, properties, G);

  //#######################################################################
  //create simplex
  //#######################################################################
  std::vector<Vertex> vertices{a,b};
  G[e.first].simplex_representation = AddSimplexNode(vertices);

  std::cout << "simplex : " << S[G[e.first].simplex_representation].vertices << std::endl;
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
  std::sort(neighbors.begin(), neighbors.end());

  AddSimplex(sigma, neighbors);
}

void SimplicialComplex::AddSimplex( std::vector<Vertex>& sigma, std::vector<Vertex>& N)
{
  if(sigma.size()<=max_dimension+1) return;

  //we already added vertices for edge-simplices, so we do not want to have
  //double entries in that case
  if(sigma.size()>1){
    for(uint k = 0; k < N.size(); k++)
    {
      Vertex vk = N.at(k);
      std::vector<Vertex> tau(sigma); tau.push_back(vk);
      AddSimplexNode(tau);
    }
  }
  for(uint k = 0; k < N.size(); k++){
    Vertex vk = N.at(k);
    std::vector<Vertex> M;
    for(uint j = k+1; j < N.size(); j++){
      Vertex vj = N.at(j);
      if(EdgeExists(vk, vj))
      {
        M.push_back(vj);
      }
    }
    std::vector<Vertex> tau(sigma); tau.push_back(vk);
    AddSimplex(tau, M);
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
  G[v].isInfeasible = true;
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
  Vertex v = boost::add_vertex(G);
  G[v].state = si->cloneState(s);

  //#######################################################################
  //Compute size of the sphere
  //#######################################################################
  if(nn_negative->size()>0)
  {
    const Vertex vnegative = nn_negative->nearest(v);
    double dn = Distance(v, vnegative);
    G[v].open_neighborhood_distance = std::min(dn, epsilon_max_neighborhood);
  }else{
    G[v].open_neighborhood_distance = epsilon_max_neighborhood;
  }

  //#######################################################################
  //Connect to all intersecting samples, create local 1-skeleton
  //#######################################################################

  if(addSimplices)
  {
    std::cout << std::string(80, '-') << std::endl;
    std::cout << "adding vertex : " << v << std::endl;
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
    double dold = G[vk].open_neighborhood_distance;
    if(ds_k < dold){
      G[vk].open_neighborhood_distance = ds_k;

      //sphere got updated. We need to remove all edges which have a segment
      //lying outside the sphere 
      OEIterator eo, eo_end, next;

      tie(eo, eo_end) = boost::out_edges(vk, G);
      std::vector<Vertex> edge_removal;
      for (next = eo; eo != eo_end; eo = next) {
        ++next;
        const Vertex vkn = boost::target(*eo, G);
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
  return G;
}

std::vector<std::vector<SimplicialComplex::Vertex>> SimplicialComplex::GetSimplicesOfDimension(uint k)
{
  std::vector<std::vector<SimplicialComplex::Vertex>> v;

  std::cout << "SimplicialComplex: " << std::endl;
  for(uint k = 0; k < k_simplices.size(); k++){
    std::cout << "simplices of size " << k << ":" << k_simplices.at(k).size() << std::endl;
  }
  std::cout << "input dimension: " << k-1 << std::endl;
  for(auto it=k_simplices.at(k-1).begin(); it!=k_simplices.at(k-1).end(); ++it)
  {
    v.push_back(it->first);
  }
  return v;
}

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

void SimplicialComplex::RemoveEdge(const Vertex a, const Vertex b)
{
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "deleting edge : " << a << " - " << b << std::endl;
  Edge e = boost::edge(a, b, G).first;
  RemoveCofaces(e);
  boost::remove_edge(e, G);
}

void SimplicialComplex::RemoveCofaces(Edge &e)
{
  std::cout << "edge has " << G[e].cofaces.size() << " cofaces." << std::endl;
  uint N = G[e].cofaces.size();
  for(uint k = 0; k < N; k++)
  {
    std::cout << "coface [" << k << "] " << G[e].cofaces.at(k)->vertices << std::endl;
  }

  const std::vector<Simplex*> cofaces = G[e].cofaces;
  for(uint k = 0; k < cofaces.size(); k++)
  {
    std::cout << "remove coface " << cofaces.at(k)->vertices << std::endl;
    Simplex *s = cofaces.at(k);
    RemoveSimplexAndCofaces(s);
    delete s;
  }

  G[e].cofaces.clear();
  std::cout << std::string(80, '-') << std::endl;
}

void SimplicialComplex::RemoveSimplexAndCofaces(Simplex *s)
{
  std::vector<Vertex> vertices = s->vertices;

  uint N = vertices.size();

  //(1) remove simplex from the global simplexMap
  ISimplexMap it = simplex_map.find(vertices);
  simplex_map.erase(it);

  //(2) remove the ptrs to simplex from all facets
  if(N>3){
    //in case all facets are simplices
    for(uint k = 0; k < s->facets.size(); k++)
    {
      Simplex *facet = s->facets.at(k);
      for(uint j = 0; j < facet->cofaces.size(); j++)
      {
        Simplex *coface = facet->cofaces.at(j);
        if(coface == s)
        {
          facet->cofaces.erase(facet->cofaces.begin() + j);
          break;
        }
      }
    }
  }else{
    //in case all facets are edges

    for(uint k = 0; k < s->edge_facets.size(); k++)
    {
      std::vector<Vertex> vfacet = s->edge_facets.at(k);
      std::cout << "delete ptr to " << vertices << " from facet " << vfacet << std::endl;
      Edge e = boost::edge(vfacet.at(0), vfacet.at(1), G).first;

      // if(!boost::edge(vfacet.at(0), vfacet.at(1), G).second)
      // {
      //   std::cout << "Edge " << vfacet << " does not exist." << std::endl;
      //   exit(0);
      // }

      for(uint j = 0; j < G[e].cofaces.size(); j++)
      {
        Simplex *coface = G[e].cofaces.at(j);
        if(coface == s)
        {
          G[e].cofaces.erase(G[e].cofaces.begin() + j);
          break;
        }
      }
    }
  }
  //(3) recursively delete all its cofaces
  for(uint k = 0; k < s->cofaces.size(); k++){
    RemoveSimplexAndCofaces(s->cofaces.at(k));
  }
}
//#############################################################################
//Add Simplices to Complex
//#############################################################################

std::pair<SimplicialComplex::Edge, bool> SimplicialComplex::AddEdge(const Vertex a, const Vertex b)
{
  EdgeInternalState properties(Distance(a,b));
  return boost::add_edge(a, b, properties, G);
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

  //#######################################################################
  //Compute simplices
  //#######################################################################
  if(neighbors.size()>0)
  {
    std::vector<Vertex> sigma;
    sigma.push_back(v);
    AddSimplexAndCofaces(sigma, neighbors);
  }
}

void SimplicialComplex::comb(int N, Simplex *coface, std::vector<Vertex> vertices)
{
  std::string bitmask(N-1, 1); // K leading 1's
  bitmask.resize(N, 0); // N-K trailing 0's

  do {
    std::vector<unsigned long int> facet;
    for (int i = 0; i < N; i++)
    {
      if (bitmask[i]) facet.push_back(vertices.at(i));
    }
    std::cout << "add coface " << coface->vertices << " to facet " << facet << " ";
    if(N>3){
      Simplex *kfacet = simplex_map[facet];
      if(kfacet == nullptr)
      {
        std::cout << "facet " << facet << " does not exist in simplex_map" << std::endl;
      }
      kfacet->AddCoface(coface);
      coface->facets.push_back(kfacet);
      std::cout << "(" << kfacet->cofaces.size() << " cofaces)" << std::endl;
    }else{
      Edge e = boost::edge(facet.at(0), facet.at(1), G).first;
      G[e].cofaces.push_back(coface);
      coface->edge_facets.push_back(facet);
      std::cout << "(" << G[e].cofaces.size() << " cofaces)" << std::endl;
    }
  } while (std::prev_permutation(bitmask.begin(), bitmask.end()));
}

void SimplicialComplex::AddSimplexAndCofaces(const std::vector<Vertex> sigma, std::vector<Vertex> neighbors, Simplex *parent)
{
  if(sigma.size()>max_dimension) return;

  uint K = sigma.size();
  uint N = neighbors.size();

  for(uint i = 0; i < N; i++){
    Vertex vi = neighbors.at(i);
    if(K > 1){

      std::vector<Vertex> gamma_v(sigma);
      gamma_v.push_back(vi);
      std::sort(gamma_v.begin(), gamma_v.end());
      Simplex* gamma = new Simplex(gamma_v);
      simplex_map[gamma_v] = gamma;

      std::cout << "new simplex: " << gamma_v << std::endl;
      //connect pointers such that nearby cofaces are connected
      comb(K+1, gamma, gamma_v);
    }

    std::vector<Vertex> neighbors_of_neighbor;
    for(uint j = i+1; j < N; j++){
      Vertex vj = neighbors.at(j);
      if(HaveIntersectingSpheres(vi, vj))
      {
        neighbors_of_neighbor.push_back(vj);
      }
    }
    if(neighbors_of_neighbor.size()>0){
      std::vector<Vertex> tau(sigma);
      tau.push_back(vi);
      AddSimplexAndCofaces(tau, neighbors_of_neighbor);
    }
  }
}




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

  for(ISimplexMap it=simplex_map.begin(); it!=simplex_map.end(); ++it)
  {
    if(it->first.size()<=k){
      v.push_back(it->first);
    }
  }
  return v;
}

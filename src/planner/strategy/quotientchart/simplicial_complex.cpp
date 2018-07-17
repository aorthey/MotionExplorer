#include "simplicial_complex.h"
#include "common.h"
#include <ompl/tools/config/SelfConfig.h>
#include <gudhi/Debug_utils.h>

using namespace ompl::geometric;
using namespace ompl::base;

SimplicialComplex::SimplicialComplex(ob::SpaceInformationPtr si_, ob::Planner* planner_, double epsilon_max_neighborhood_):
  si(si_), epsilon_max_neighborhood(epsilon_max_neighborhood_)
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

void SimplicialComplex::AddEdge(const Vertex a, const Vertex b)
{
  EdgeInternalState properties(Distance(a,b));
  boost::add_edge(a, b, properties, G);
}
void SimplicialComplex::RemoveEdge(const Vertex a, const Vertex b)
{
  Edge e = boost::edge(a, b, G).first;
  boost::remove_edge(e, G);
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
  Vertex v = Add(s, nn_feasible, nn_infeasible);
  simplexTree.insert_simplex({(int)v});

  // std::vector<Vertex> neighbors;
  // nn_positive->nearestR(v, 2*epsilon_max_neighborhood, neighbors);
  // for(uint k = 0; k < ; k++){
  // }

  std::pair<adjacency_iterator, adjacency_iterator> neighbors =
    boost::adjacent_vertices(boost::vertex(v,G), G);
 
  for(; neighbors.first != neighbors.second; ++neighbors.first)
  {
    simplexTree.insert_simplex({(int)v, (int)*neighbors.first});
  }

  return v;
}

SimplicialComplex::Vertex SimplicialComplex::Add(const ob::State *s, RoadmapNeighbors nn_positive, RoadmapNeighbors nn_negative)
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
  std::vector<Vertex> neighbors;
  nn_positive->nearestR(v, 2*epsilon_max_neighborhood, neighbors);
  double d = G[v].open_neighborhood_distance;

  for(uint k = 0; k < neighbors.size(); k++){
    Vertex vk = neighbors.at(k);
    double dall = Distance(v, vk);
    double dk = G[vk].open_neighborhood_distance;
    if(d+dk > dall){
      AddEdge(v, vk);
    }
  }

  //#######################################################################
  //Update all sphere radii of the negative neighbors. If a sphere radius is changed, remove edges
  //which are not contained in the sphere anymore
  //#######################################################################
  neighbors.clear();
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
        double dall = Distance(vk, vkn);
        double ds_kn = G[vkn].open_neighborhood_distance;
        if(ds_kn + ds_k < dall){
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
  //#######################################################################
  //add vertex to neighborhood structure
  //#######################################################################

  nn_positive->add(v);
  return v;
}

const SimplicialComplex::Graph& SimplicialComplex::GetGraph()
{
  // std::cout << "* The complex contains " << simplexTree.num_simplices() << " simplices\n";
  // std::cout << "   - dimension " << simplexTree.dimension() << "\n";
  
  // std::cout << "* Iterator on Simplices in the filtration, with [filtration value]:\n";
  // for (auto f_simplex : simplexTree.filtration_simplex_range()) {
  //   std::cout << "   "
  //             << "[" << simplexTree.filtration(f_simplex) << "] ";
  //   for (auto vertex : simplexTree.simplex_vertex_range(f_simplex)) std::cout << "(" << vertex << ")";
  //   std::cout << std::endl;
  // }
  return G;
}

        // struct Ksimplex{
        //   Ksimplex(std::vector<int> vertices_):
        //     vertices(vertices_) 
        //   {
        //     uint N = vertices.size();
        //     if(N>2) comb(N,N-1);
        //   };

        //   void comb(int N, int K)
        //   {
        //     std::string bitmask(K, 1); // K leading 1's
        //     bitmask.resize(N, 0); // N-K trailing 0's
        //     do {
        //       std::vector<int> facet;
        //       for (int i = 0; i < N; i++)
        //       {
        //           if (bitmask[i]) facet.push_back(vertices.at(i));
        //       }
        //       Ksimplex *kfacet = simplicial_complex[facet];
        //       facets.push_back(kfacet);
        //       kfacet->AddCoFace(this);

        //     } while (std::prev_permutation(bitmask.begin(), bitmask.end()));
        //   }

        //   void Clear()
        //   {
        //     for(uint k = 0; k < cofaces.size(); k++){
        //       cofaces.at(k)->Clear();
        //     }
        //     for(uint k = 0; k < facets.size(); k++){
        //       Ksimplex *facet = facets.at(k);
        //       for(uint j = 0; j < facet->cofaces.size(); j++){
        //         Ksimplex *coface = facet->cofaces.at(j);
        //         if(coface == this)
        //         {
        //           facet->cofaces.erase(facet->cofaces.begin() + j);
        //           break;
        //         }
        //       }
        //     }
        //     //no pointers should be left, we can remove this Ksimplex
        //   }

        //   void AddCoFace(Ksimplex* coface)
        //   {
        //     cofaces.push_back(coface);
        //   }

        //   std::vector<Ksimplex*> facets;
        //   std::vector<Ksimplex*> cofaces;
        //   std::vector<int> vertices;
        // };

        // std::map<std::vector<int>, Ksimplex*> simplicial_complex;

        // //typedef std::vector< std::vector<int> > LocalSimplicialComplex;
        // //typedef std::pair<const ob::State*, const ob::State*> EdgeVertices;
        // //std::map<const ob::State*, LocalSimplicialComplex> simplicial_complex;
        // //std::map<Edge, LocalSimplicialComplex> simplicial_complex;

        // RoadmapNeighbors nn_infeasible;

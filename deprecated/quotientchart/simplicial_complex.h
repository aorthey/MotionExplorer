#pragma once
#include "simplicial_complex_hasse_diagram.h"
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/Planner.h>
#include <ompl/datastructures/NearestNeighbors.h>
//#include <boost/pending/disjoint_sets.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <unordered_map>

namespace ob = ompl::base;

namespace ompl
{
  namespace geometric
  {
    namespace topology
    {
      //@brief: implements vietoris rips complex
      class SimplicialComplex
      {
        public:
          //###################################################################
          //1-skeleton representation of simplicial complex as boost undirected graph
          //###################################################################
          class VertexInternalState{
            public:
              VertexInternalState() = default;
              VertexInternalState(const VertexInternalState &vis) = default;
              ob::State *state{nullptr};
              double open_neighborhood_distance{0.0};
              bool isStart{false};
              bool isGoal{false};
              bool isInfeasible{false};
          };

          class EdgeInternalState{
            public:
              EdgeInternalState() = default;
              EdgeInternalState(double weight_): weight(weight_){};
              void setWeight(double weight_){
                weight = weight_;
              }
              double getWeight(){
                return weight;
              }
              HasseDiagram::SimplexNode simplex_representation;
            private:
              double weight;
          };

          typedef boost::adjacency_list<
             boost::vecS, 
             boost::vecS, 
             boost::undirectedS,
             VertexInternalState,
             EdgeInternalState
           > Graph;

          typedef boost::graph_traits<Graph> BGT;
          typedef BGT::vertex_descriptor Vertex;
          typedef BGT::edge_descriptor Edge;
          typedef BGT::vertices_size_type VertexIndex;
          typedef BGT::in_edge_iterator IEIterator;
          typedef BGT::out_edge_iterator OEIterator;
          typedef BGT::adjacency_iterator adjacency_iterator;
          typedef Vertex* VertexParent;
          typedef VertexIndex* VertexRank;
          typedef std::shared_ptr<NearestNeighbors<Vertex>> RoadmapNeighbors;

          void RemoveEdge(const Vertex v, const Vertex b);
          bool EdgeExists(const Vertex a, const Vertex b); 

          void AddSimplex( std::vector<Vertex>& sigma, std::vector<Vertex>& N);

          //###################################################################
          //data structures
          //###################################################################
          ob::SpaceInformationPtr si;
          double epsilon_max_neighborhood{0};
          uint max_dimension{0};
          RoadmapNeighbors nn_infeasible;
          RoadmapNeighbors nn_feasible;
          Graph graph;
          HasseDiagram hasse_diagram;

          //###################################################################
          //
          //###################################################################
          SimplicialComplex(ob::SpaceInformationPtr si_, ob::Planner* planner_, double epsilon_max_neighborhood_ = 2);
          std::vector<std::vector<Vertex>> GetSimplicesOfDimension(uint k);
          double Distance(const Vertex a, const Vertex b);
          bool HaveIntersectingSpheres(const Vertex a, const Vertex b);
          std::pair<Edge, bool> AddEdge(const Vertex a, const Vertex b);
          Vertex Add(const ob::State *s, RoadmapNeighbors nn_positive, RoadmapNeighbors nn_negative, bool addSimplices=false);
          void AddSimplices(const Vertex v, RoadmapNeighbors nn);

          //ntry is the number of failures to update the simplicial complex when
          //adding new samples. The more failures we accumulate, the better our
          //approximation will be. As suggested by simeon_2000, we could terminate
          //the algorithm if ntry becomes larger than some user set value M.
          //However, in our case, it seems more adequate to compute some kind of
          //average ntry (over the last 100 samples or something).:W
          uint ntry{0};
          std::vector<int> ntry_over_iterations;

          friend std::ostream& operator<< (std::ostream&, const SimplicialComplex&);

        public:
          //all methods which should be made available at the end to the outside
          Vertex AddFeasible(const ob::State *s);
          Vertex AddInfeasible(const ob::State *s);
          void AddStart(const ob::State *s);
          void AddGoal(const ob::State *s);
          const Graph& GetGraph();


      };

    }
  }
}

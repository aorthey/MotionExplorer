#pragma once

#include "quotient.h"
#include "planner/cover/open_set.h"
#include <ompl/geometric/planners/PlannerIncludes.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/base/Cost.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/pending/disjoint_sets.hpp>
#include <boost/property_map/property_map.hpp>
#include <stack>
#include <KrisLibrary/math/infnan.h> //dInf, fInf, IsNaN(x)
using Math::dInf;

namespace ob = ompl::base;
namespace og = ompl::geometric;

//copied from ompl/geometric/planners/PRM.h plus some modifications/simplifications
namespace ompl
{
  namespace base
  {
      OMPL_CLASS_FORWARD(OptimizationObjective);
  }
  namespace geometric
  {
    class QuotientGraph: public og::Quotient{

        typedef og::Quotient BaseT;
      public:

        typedef unsigned long int VertexIndexType;

        class VertexInternalState{
          public:
            VertexInternalState(){};
            ob::State *state{nullptr};
            uint total_connection_attempts{0};
            uint successful_connection_attempts{0};
            bool on_shortest_path{false};
            unsigned long int associated_target{0};
            unsigned long int associated_source{0};
            double associated_t{-1};
            double open_neighborhood_distance{0};
            cover::OpenSet *open_neighborhood{nullptr};
        };

        class EdgeInternalState{
          public:
            EdgeInternalState(ob::Cost cost_): cost(cost_), original_cost(cost_)
            {};
            void setWeight(double d){
              cost = ob::Cost(d);
            }
            ob::Cost getCost(){
              return cost;
            }
            void setOriginalWeight(){
              cost = original_cost;
            }
          private:
            ob::Cost cost{+dInf};
            ob::Cost original_cost{+dInf};
            bool isSufficient;
        };

        typedef boost::adjacency_list<
           boost::vecS, 
           boost::vecS, 
           boost::undirectedS,
           VertexInternalState,
           EdgeInternalState
         >Graph;

        typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
        typedef boost::graph_traits<Graph>::edge_descriptor Edge;
        typedef boost::graph_traits<Graph>::vertices_size_type VertexIndex;
        typedef boost::graph_traits<Graph>::in_edge_iterator IEIterator;
        typedef Vertex* VertexParent;
        typedef VertexIndex* VertexRank;

        typedef std::shared_ptr<NearestNeighbors<Vertex>> RoadmapNeighbors;
        typedef std::function<const std::vector<Vertex> &(const Vertex)> ConnectionStrategy;

      public:

        QuotientGraph(const ob::SpaceInformationPtr &si, Quotient *previous = nullptr);
        ~QuotientGraph();

        virtual uint GetNumberOfVertices() override;
        virtual uint GetNumberOfEdges() override;
        ob::PathPtr GetShortestPath();
        ob::PathPtr GetSolutionPath();

        virtual void Grow(double t);
        virtual void Init() override;

        template <template <typename T> class NN>
        void setNearestNeighbors();

        void getPlannerData(ob::PlannerData &data) const override;

        void setup() override;

        virtual void clear() override;
        void clearQuery();
        virtual void ClearVertices();

        ob::OptimizationObjectivePtr opt_;
        std::vector<Vertex> startM_;
        std::vector<Vertex> goalM_;
        std::vector<Vertex> shortestVertexPath_;

        void uniteComponents(Vertex m1, Vertex m2);
        bool sameComponent(Vertex m1, Vertex m2);
        ob::Cost bestCost_{+dInf};

        virtual void CheckForSolution(ob::PathPtr &solution) override;

    protected:

        Graph G;
        ob::PathPtr solution_path;

        virtual double Distance(const Vertex a, const Vertex b) const; // standard si->distance
        virtual bool Connect(const Vertex a, const Vertex b);
        virtual Vertex addMilestone(ob::State *state);

        virtual Vertex CreateNewVertex(ob::State *state);
        virtual void ConnectVertexToNeighbors(Vertex m);

        ob::Cost costHeuristic(Vertex u, Vertex v) const;
        std::vector<ob::State *> xstates;

        RoadmapNeighbors nn_;

        std::map<Vertex, VertexRank> rank;
        std::map<Vertex, Vertex> parent;
        boost::disjoint_sets<boost::associative_property_map<std::map<Vertex, VertexRank> >, boost::associative_property_map<std::map<Vertex, Vertex> > > 
          disjointSets_{boost::make_assoc_property_map(rank), boost::make_assoc_property_map(parent)};

        ConnectionStrategy connectionStrategy_;

        RNG rng_;
        bool addedNewSolution_{false};
        unsigned long int iterations_{0};

        void growRoadmap(const ob::PlannerTerminationCondition &ptc, ob::State *workState);
        void expandRoadmap(const ob::PlannerTerminationCondition &ptc, std::vector<ob::State *> &workStates);

        ob::PathPtr GetSolutionPath(const Vertex &start, const Vertex &goal);

        virtual void RandomWalk(const Vertex &v);

    };
  };
};


#pragma once

#include "planner/strategy/quotientgraph/quotient_graph.h"
#include "planner/strategy/explorer/PathVisibilityChecker.h"
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/util/RandomNumbers.h>
#include <ompl/geometric/PathGeometric.h>
#include <boost/graph/subgraph.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/random.hpp> 
#include <boost/random/linear_congruential.hpp>
#include <boost/random/variate_generator.hpp>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl
{
  namespace geometric
  {
    class QuotientGraphSparse: public og::QuotientGraph{

        typedef og::QuotientGraph BaseT;
      public:

        QuotientGraphSparse(const ob::SpaceInformationPtr &si, Quotient *parent = nullptr);
        virtual ~QuotientGraphSparse() override;

        virtual void Grow(double t) = 0;
        virtual void getPlannerData(ob::PlannerData &data) const override;

        virtual void DeleteConfiguration(Configuration *q);
        virtual Vertex AddConfiguration(Configuration *q) override;
        Vertex AddConfigurationSparse(Configuration *q);
        void AddEdgeSparse(const Vertex a, const Vertex b);

        // void AddEdge(const Configuration* q1, const Configuration* q2);
        virtual void setup() override;
        virtual void clear() override;

        virtual void Init();
        //Copied from SPARS
        void findGraphNeighbors(Configuration *q, std::vector<Configuration*> &graphNeighborhood,
                                std::vector<Configuration*> &visibleNeighborhood);
        bool checkAddConnectivity(Configuration* q, std::vector<Configuration*> &visibleNeighborhood);
        bool checkAddInterface(Configuration *q, std::vector<Configuration*> &graphNeighborhood, std::vector<Configuration*> &visibleNeighborhood);

        void Rewire(Vertex &v);
        void Rewire();
        void printAllPathsUtil(Vertex u, Vertex d, bool visited[], int path[], int &path_index);
        void enumerateAllPaths();

        int selectedPath{-1}; //selected path to sample from (if children try to sample this space)
        bool SampleQuotient(ob::State *q_random_graph) override;

    protected:
        double sparseDelta_{0.};
        double sparseDeltaFraction_{.25};

        void setSparseDeltaFraction(double D)
        {
            sparseDeltaFraction_ = D;
        }
        double getSparseDeltaFraction() const
        {
            return sparseDeltaFraction_;
        }
        std::vector<og::PathGeometric> pathStack_;

        uint Nhead{5}; //head -nX (to display only X top paths)
        std::vector<std::vector<ob::State*>> pathStackHead_;
        void PrintPathStack();


        Graph graphSparse_;
        RoadmapNeighborsPtr nearestSparse_;
        std::vector<Configuration*> graphNeighborhood;
        std::vector<Configuration*> visibleNeighborhood;

        Vertex v_start_sparse;
        Vertex v_goal_sparse;

        PathVisibilityChecker* pathVisibilityChecker_;
        void AddPathToStack(std::vector<ob::State*> &path);

        // std::vector<Vertex> shortest_path_start_goal;
        // Configuration *q_start;
        // Configuration *q_goal;
        // Vertex v_start;
        // Vertex v_goal;
        // RNG rng_;
        // typedef boost::minstd_rand RNGType;
        // RNGType rng_boost;

    };
  };
};



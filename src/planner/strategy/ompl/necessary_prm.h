#pragma once

#include <ompl/geometric/planners/PlannerIncludes.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/pending/disjoint_sets.hpp>
#include <ompl/base/goals/GoalRegion.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <mutex>
#include <utility>
#include <vector>
#include <map>

namespace ob = ompl::base;
namespace og = ompl::geometric;


namespace ompl
{
    namespace base
    {
        OMPL_CLASS_FORWARD(OptimizationObjective);
    }

    namespace geometric
    {
        class NecessaryPRM : public base::Planner
        {
        public:
            struct vertex_state_t
            {
                typedef boost::vertex_property_tag kind;
            };

            struct vertex_total_connection_attempts_t
            {
                typedef boost::vertex_property_tag kind;
            };

            struct vertex_successful_connection_attempts_t
            {
                typedef boost::vertex_property_tag kind;
            };

            /**
             @brief The underlying roadmap graph.

             @par Any BGL graph representation could be used here. Because we
             expect the roadmap to be sparse (m<n^2), an adjacency_list is more
             appropriate than an adjacency_matrix.

             @par Obviously, a ompl::base::State* vertex property is required.
             The incremental connected components algorithm requires
             vertex_predecessor_t and vertex_rank_t properties.
             If boost::vecS is not used for vertex storage, then there must also
             be a boost:vertex_index_t property manually added.

             @par Edges should be undirected and have a weight property.
             */
            typedef boost::adjacency_list<
                boost::vecS, boost::vecS, boost::undirectedS,
                boost::property<
                    vertex_state_t, base::State *,
                    boost::property<
                        vertex_total_connection_attempts_t, unsigned long int,
                        boost::property<vertex_successful_connection_attempts_t, unsigned long int,
                                        boost::property<boost::vertex_predecessor_t, unsigned long int,
                                                        boost::property<boost::vertex_rank_t, unsigned long int>>>>>,
                boost::property<boost::edge_weight_t, base::Cost>>
                Graph;

            typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
            typedef boost::graph_traits<Graph>::edge_descriptor Edge;
            typedef std::shared_ptr<NearestNeighbors<Vertex>> RoadmapNeighbors;
            typedef std::function<const std::vector<Vertex> &(const Vertex)> ConnectionStrategy;
            typedef std::function<bool(const Vertex &, const Vertex &)> ConnectionFilter;

            NecessaryPRM(const base::SpaceInformationPtr &si0, const ob::SpaceInformationPtr &si1);
            ~NecessaryPRM() override;
            void setProblemDefinition(const base::ProblemDefinitionPtr &pdef) override;

            /** \brief Set the connection strategy function that specifies the
             milestones that connection attempts will be make to for a
             given milestone.

             \par The behavior and performance of NecessaryPRM can be changed drastically
             by varying the number and properties if the milestones that are
             connected to each other.

             \param pdef A function that takes a milestone as an argument and
             returns a collection of other milestones to which a connection
             attempt must be made. The default connection strategy is to connect
             a milestone's 10 closest neighbors.
             */
            void setConnectionStrategy(const ConnectionStrategy &connectionStrategy)
            {
                connectionStrategy_ = connectionStrategy;
                userSetConnectionStrategy_ = true;
            }
            void setMaxNearestNeighbors(unsigned int k);

            /** \brief Set the function that can reject a milestone connection.

             \par The given function is called immediately before a connection
             is checked for collision and added to the roadmap. Other neighbors
             may have already been connected before this function is called.
             This allows certain heuristics that use the structure of the
             roadmap (like connected components or useful cycles) to be
             implemented by changing this function.

             \param connectionFilter A function that takes the new milestone,
             a neighboring milestone and returns whether a connection should be
             attempted.
             */
            void setConnectionFilter(const ConnectionFilter &connectionFilter)
            {
                connectionFilter_ = connectionFilter;
            }

            void getPlannerData(base::PlannerData &data) const override;
            void constructRoadmap(const base::PlannerTerminationCondition &ptc);
            void growRoadmap(double growTime);
            void growRoadmap(const base::PlannerTerminationCondition &ptc);
            void expandRoadmap(double expandTime);
            void expandRoadmap(const base::PlannerTerminationCondition &ptc);
            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            void clearQuery();
            void clear() override;

            template <template <typename T> class NN>
            void setNearestNeighbors()
            {
                if (nn_ && nn_->size() == 0)
                    OMPL_WARN("Calling setNearestNeighbors will clear all states.");
                clear();
                nn_ = std::make_shared<NN<Vertex>>();
                if (!userSetConnectionStrategy_)
                    connectionStrategy_ = ConnectionStrategy();
                if (isSetup())
                    setup();
            }

            void setup() override;
            const Graph &getRoadmap() const;
            unsigned long int milestoneCount() const;
            unsigned long int edgeCount() const;
            const RoadmapNeighbors &getNearestNeighbors();

        protected:
            void freeMemory();
            Vertex addMilestone(base::State *state);
            void uniteComponents(Vertex m1, Vertex m2);
            bool sameComponent(Vertex m1, Vertex m2);
            void growRoadmap(const base::PlannerTerminationCondition &ptc, base::State *workState);
            void expandRoadmap(const base::PlannerTerminationCondition &ptc, std::vector<base::State *> &workStates);
            void checkForSolution(const base::PlannerTerminationCondition &ptc, base::PathPtr &solution);
            bool maybeConstructSolution(const std::vector<Vertex> &starts, const std::vector<Vertex> &goals,
                                        base::PathPtr &solution);

            bool addedNewSolution() const;
            base::PathPtr constructSolution(const Vertex &start, const Vertex &goal);
            base::Cost costHeuristic(Vertex u, Vertex v) const;

            double distanceFunction(const Vertex a, const Vertex b) const;
            std::string getIterationCount() const;
            std::string getBestCost() const;
            std::string getMilestoneCountString() const;
            std::string getEdgeCountString() const;

            base::ValidStateSamplerPtr sampler_;
            base::StateSamplerPtr simpleSampler_;
            RoadmapNeighbors nn_;
            Graph g_;
            std::vector<Vertex> startM_;
            std::vector<Vertex> goalM_;
            boost::property_map<Graph, vertex_state_t>::type stateProperty_;
            boost::property_map<Graph, vertex_total_connection_attempts_t>::type totalConnectionAttemptsProperty_;
            boost::property_map<Graph, vertex_successful_connection_attempts_t>::type
                successfulConnectionAttemptsProperty_;
            boost::property_map<Graph, boost::edge_weight_t>::type weightProperty_;
            boost::disjoint_sets<boost::property_map<Graph, boost::vertex_rank_t>::type,
                                 boost::property_map<Graph, boost::vertex_predecessor_t>::type> disjointSets_;
            ConnectionStrategy connectionStrategy_;
            ConnectionFilter connectionFilter_;
            bool userSetConnectionStrategy_{false};
            RNG rng_;
            bool addedNewSolution_{false};
            mutable std::mutex graphMutex_;
            base::OptimizationObjectivePtr opt_;
            unsigned long int iterations_{0};
            base::Cost bestCost_{std::numeric_limits<double>::quiet_NaN()};
            ob::SpaceInformationPtr si_level1;
        };
    }
}

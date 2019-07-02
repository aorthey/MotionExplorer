#pragma once
#include "QuotientTopology.h"
#include <ompl/datastructures/PDF.h>
#include <ompl/geometric/PathSimplifier.h>


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
    //QuotientTopologySparse: Reimplementation of SPARSTwo. Note SPARSTwo has been
    //built up from code used by PRM. 
    class QuotientTopologySparse: public og::QuotientTopology{

      typedef og::QuotientTopology BaseT;
      public:

        QuotientTopologySparse(const ob::SpaceInformationPtr &si, Quotient *parent_);
        virtual ~QuotientTopologySparse() override;
        virtual void Grow(double t) override;

        /** \brief Enumeration which specifies the reason a guard is added to the spanner. */
        enum GuardType
        {
            START,
            GOAL,
            COVERAGE,
            CONNECTIVITY,
            INTERFACE,
            QUALITY,
        };

        // /** \brief Pair of vertices which support an interface. */
        // using VertexPair = std::pair<normalized_index_type, normalized_index_type>;

        // /** \brief Interface information storage class, which does bookkeeping for criterion four. */
        // struct InterfaceData
        // {
        //     /** \brief States which lie inside the visibility region of a vertex and support an interface. */
        //     base::State *pointA_{nullptr};
        //     base::State *pointB_{nullptr};

        //     /** \brief States which lie just outside the visibility region of a vertex and support an interface. */
        //     base::State *sigmaA_{nullptr};
        //     base::State *sigmaB_{nullptr};

        //     /** \brief Last known distance between the two interfaces supported by points_ and sigmas. */
        //     double d_{std::numeric_limits<double>::infinity()};

        //     /** \brief Constructor */
        //     InterfaceData() = default;

        //     /** \brief Clears the given interface data. */
        //     void clear(const base::SpaceInformationPtr &si)
        //     {
        //         if (pointA_ != nullptr)
        //         {
        //             si->freeState(pointA_);
        //             pointA_ = nullptr;
        //         }
        //         if (pointB_ != nullptr)
        //         {
        //             si->freeState(pointB_);
        //             pointB_ = nullptr;
        //         }
        //         if (sigmaA_ != nullptr)
        //         {
        //             si->freeState(sigmaA_);
        //             sigmaA_ = nullptr;
        //         }
        //         if (sigmaB_ != nullptr)
        //         {
        //             si->freeState(sigmaB_);
        //             sigmaB_ = nullptr;
        //         }
        //         d_ = std::numeric_limits<double>::infinity();
        //     }

        //     /** \brief Sets information for the first interface (i.e. interface with smaller index vertex). */
        //     void setFirst(const base::State *p, const base::State *s, const base::SpaceInformationPtr &si)
        //     {
        //         if (pointA_ != nullptr)
        //             si->copyState(pointA_, p);
        //         else
        //             pointA_ = si->cloneState(p);
        //         if (sigmaA_ != nullptr)
        //             si->copyState(sigmaA_, s);
        //         else
        //             sigmaA_ = si->cloneState(s);
        //         if (pointB_ != nullptr)
        //             d_ = si->distance(pointA_, pointB_);
        //     }

        //     /** \brief Sets information for the second interface (i.e. interface with larger index vertex). */
        //     void setSecond(const base::State *p, const base::State *s, const base::SpaceInformationPtr &si)
        //     {
        //         if (pointB_ != nullptr)
        //             si->copyState(pointB_, p);
        //         else
        //             pointB_ = si->cloneState(p);
        //         if (sigmaB_ != nullptr)
        //             si->copyState(sigmaB_, s);
        //         else
        //             sigmaB_ = si->cloneState(s);
        //         if (pointA_ != nullptr)
        //             d_ = si->distance(pointA_, pointB_);
        //     }
        // };

        // /** \brief the hash which maps pairs of neighbor points to pairs of states */
        // using InterfaceHash = std::unordered_map<VertexPair, InterfaceData>;

        // /** \brief Sets the stretch factor */
        // void setStretchFactor(double t)
        // {
        //     stretchFactor_ = t;
        // }

        /** \brief Sets vertex visibility range as a fraction of max. extent. */
        void setSparseDeltaFraction(double D)
        {
            sparseDeltaFraction_ = D;
        }

        /** \brief Sets interface support tolerance as a fraction of max. extent. */
        void setDenseDeltaFraction(double d)
        {
            denseDeltaFraction_ = d;
        }

        /** \brief Retrieve the maximum consecutive failure limit. */
        unsigned int getMaxFailures() const
        {
            return maxFailures_;
        }

        /** \brief Retrieve the dense graph interface support delta. */
        double getDenseDeltaFraction() const
        {
            return denseDeltaFraction_;
        }

        /** \brief Retrieve the sparse graph visibility range delta. */
        double getSparseDeltaFraction() const
        {
            return sparseDeltaFraction_;
        }

        /** \brief Retrieve the spanner's set stretch factor. */
        double getStretchFactor() const
        {
            return stretchFactor_;
        }

        virtual void setup() override;
        virtual void clear() override;


    protected:

        /** \brief Check that the query vertex is initialized (used for internal nearest neighbor searches) */
        void checkQueryStateInitialization();

        /** \brief Checks to see if the sample needs to be added to ensure coverage of the space */
        bool checkAddCoverage(const base::State *qNew, std::vector<Configuration*> &visibleNeighborhood);
        virtual void Init();

        // /** \brief Checks to see if the sample needs to be added to ensure connectivity */
        // bool checkAddConnectivity(const base::State *qNew, std::vector<Vertex> &visibleNeighborhood);

        // /** \brief Checks to see if the current sample reveals the existence of an interface, and if so, tries to
        //  * bridge it. */
        // bool checkAddInterface(const base::State *qNew, std::vector<Vertex> &graphNeighborhood,
        //                        std::vector<Vertex> &visibleNeighborhood);

        // /** \brief Checks vertex v for short paths through its region and adds when appropriate. */
        // bool checkAddPath(Vertex v);

        void findGraphNeighbors(Configuration *q, std::vector<Configuration*> &graphNeighborhood,
                                std::vector<Configuration*> &visibleNeighborhood);

        // /** \brief Approaches the graph from a given vertex */
        // void approachGraph(Vertex v);

        // /** \brief Finds the representative of the input state, st  */
        // Vertex findGraphRepresentative(base::State *st);

        // /** \brief Finds representatives of samples near qNew_ which are not his representative */
        // void findCloseRepresentatives(base::State *workArea, const base::State *qNew, Vertex qRep,
        //                               std::map<Vertex, base::State *> &closeRepresentatives,
        //                               const base::PlannerTerminationCondition &ptc);

        // /** \brief High-level method which updates pair point information for repV_ with neighbor r */
        // void updatePairPoints(Vertex rep, const base::State *q, Vertex r, const base::State *s);

        // /** \brief Computes all nodes which qualify as a candidate v" for v and vp */
        // void computeVPP(Vertex v, Vertex vp, std::vector<Vertex> &VPPs);

        // /** \brief Computes all nodes which qualify as a candidate x for v, v', and v" */
        // void computeX(Vertex v, Vertex vp, Vertex vpp, std::vector<Vertex> &Xs);

        // /** \brief Rectifies indexing order for accessing the vertex data */
        // VertexPair index(Vertex vp, Vertex vpp);

        // /** \brief Retrieves the Vertex data associated with v,vp,vpp */
        // InterfaceData &getData(Vertex v, Vertex vp, Vertex vpp);

        // /** \brief Performs distance checking for the candidate new state, q against the current information */
        // void distanceCheck(Vertex rep, const base::State *q, Vertex r, const base::State *s, Vertex rp);

        // /** \brief When a new guard is added at state st, finds all guards who must abandon their interface
        //  * information and deletes that information */
        // void abandonLists(base::State *st);

        /** \brief Construct a guard for a given state (\e state) and store it in the nearest neighbors data
         * structure */
        Vertex addGuard(base::State *state, GuardType type);

        // /** \brief Connect two guards in the roadmap */
        // void connectGuards(Vertex v, Vertex vp);

        /** \brief Stretch Factor as per graph spanner literature (multiplicative bound on path quality) */
        double stretchFactor_{3.};

        /** \brief Maximum visibility range for nodes in the graph as a fraction of maximum extent. */
        double sparseDeltaFraction_{.25};

        /** \brief Maximum range for allowing two samples to support an interface as a fraction of maximum extent.
         */
        double denseDeltaFraction_{.001};

        /** \brief The number of consecutive failures to add to the graph before termination */
        unsigned int maxFailures_{5000};

        /** \brief Number of sample points to use when trying to detect interfaces. */
        unsigned int nearSamplePoints_;

        /** \brief A path simplifier used to simplify dense paths added to the graph */
        PathSimplifierPtr psimp_;

        // /** \brief Access to the weights of each Edge */
        // boost::property_map<Graph, boost::edge_weight_t>::type weightProperty_;

        // /** \brief Access to the colors for the vertices */
        // boost::property_map<Graph, vertex_color_t>::type colorProperty_;

        // /** \brief Access to the interface pair information for the vertices */
        // boost::property_map<Graph, vertex_interface_data_t>::type interfaceDataProperty_;

        // /** \brief Data structure that maintains the connected components */
        // boost::disjoint_sets<boost::property_map<Graph, boost::vertex_rank_t>::type,
        //                      boost::property_map<Graph, boost::vertex_predecessor_t>::type> disjointSets_;
        /** \brief Random number generator */
        // RNG rng_;

        // bool addedSolution_{false};

        /** \brief A counter for the number of consecutive failed iterations of the algorithm */
        // unsigned int consecutiveFailures_{0};

        /** \brief Maximum visibility range for nodes in the graph */
        double sparseDelta_{0.};

        /** \brief Maximum range for allowing two samples to support an interface */
        // double denseDelta_{0.};

        base::Cost costHeuristic(Vertex u, Vertex v) const;
        base::Cost bestCost_{std::numeric_limits<double>::quiet_NaN()};

    };

  };
};

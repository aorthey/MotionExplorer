#pragma once
#include "planner/strategy/quotientchart/quotient_chart.h"
#include <ompl/datastructures/PDF.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/datastructures/NearestNeighbors.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl
{
  namespace geometric
  {

    //Quotient-space Sufficient neighborhood graph planner (QNG)
    class QNG: public og::QuotientChart
    {
      typedef og::QuotientChart BaseT;
    public:

      QNG(const ob::SpaceInformationPtr &si, Quotient *parent = nullptr);
      ~QNG(void);
      virtual void clear() override;
      virtual void setup() override;
      void getPlannerData(ob::PlannerData &data) const override;

      virtual uint GetNumberOfVertices() const override;
      virtual uint GetNumberOfEdges() const override;
      
      
    protected:
      class Configuration{
        public:
          Configuration() = default;
        Configuration(const base::SpaceInformationPtr &si) : state(si->allocState())
        {}
        Configuration(const base::SpaceInformationPtr &si, const ob::State *state_) : state(si->cloneState(state_))
        {}
        ~Configuration(){};
        double GetRadius() const
        {
          return openNeighborhoodRadius;
        }
        void SetRadius(double radius)
        {
          openNeighborhoodRadius = radius;
        }
        void Remove(const base::SpaceInformationPtr &si)
        {
          si->freeState(state);
        }
        void SetPDFElement(void *element_)
        {
          element = element_;
        }
        void* GetPDFElement()
        {
          return element;
        }
        double GetImportance()
        {
          return openNeighborhoodRadius;
          //return ((double)number_successful_expansions+1)/((double)number_attempted_expansions+2);
          //return 1.0/((double)number_attempted_expansions+1);
        }

        uint number_attempted_expansions{0};
        uint number_successful_expansions{0};

        base::State *state{nullptr};
        Configuration *coset{nullptr}; //the underlying coset this Vertex elongs to (on the quotient-space)
        bool isSufficientFeasible{false};
        double openNeighborhoodRadius{0.0}; //might be L1 or L2 radius
        void *element;
      };

      class EdgeInternalState{
        public:
          EdgeInternalState() = default;
          EdgeInternalState(ob::Cost cost_): cost(cost_), original_cost(cost_)
          {};
          EdgeInternalState(const EdgeInternalState &eis)
          {
            cost = eis.cost;
            original_cost = eis.original_cost;
            isSufficient = eis.isSufficient;
          }
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
          bool isSufficient{false};
      };

      typedef boost::adjacency_list<
         boost::vecS, 
         boost::vecS, 
         boost::undirectedS,
         Configuration,
         //EdgeInternalState
         boost::property<boost::edge_index_t,int, EdgeInternalState> 
       > Graph;

      typedef boost::graph_traits<Graph> BGT;
      typedef BGT::vertex_descriptor Vertex;
      typedef BGT::edge_descriptor Edge;
      typedef BGT::vertices_size_type VertexIndex;
      typedef BGT::in_edge_iterator IEIterator;
      typedef BGT::out_edge_iterator OEIterator;
      typedef Vertex* VertexParent;
      typedef VertexIndex* VertexRank;
      //typedef std::function<const std::vector<Vertex> &(const Vertex)> ConnectionStrategy;
      typedef std::shared_ptr<NearestNeighbors<Configuration*>> NearestNeighborsPtr;
      typedef ompl::PDF<Configuration*> PDF;
      typedef PDF::Element PDF_Element;

      void SetSubGraph( QuotientChart *sibling, uint k ) override;

      void AddConfiguration(Configuration *q);

      //need to supply q_coset, the pointer to the underlying equivalence class
      Configuration* AddState(const ob::State *state, Configuration *q_coset);
      void RemoveConfiguration(Configuration *q);

      bool sampleUniformOnNeighborhoodBoundary(Vertex sample, const Vertex center);
      bool sampleHalfBallOnNeighborhoodBoundary(Vertex sample, const Vertex center);
      Vertex EstimateBestNextState(Vertex q_last, Vertex q_current);

      virtual bool Sample(Vertex q_random);
      bool IsSampleInsideCover(Configuration *q);
      void Grow(double t) override;

      void RemoveCoveredSamples(Configuration *q);

      Configuration* Nearest(Configuration *q) const;
      bool Connect(const Configuration *q_from, Configuration *q_to);

      double Distance(const Configuration *q_from, const Configuration *q_to);
      double DistanceQ1(const Configuration *q_from, const Configuration *q_to);
      double DistanceX1(const Configuration *q_from, const Configuration *q_to);
      double DistanceCover(const Configuration *q_from, const Configuration *q_to);
      double DistanceOpenNeighborhood(const Configuration *q_from, const Configuration *q_to);

      void ConstructSolution(Configuration *q_goal);

      RNG rng_;
      double goalBias{0.05}; //in [0,1]
      double voronoiBias{0.3}; //in [0,1]

      NearestNeighborsPtr nearest_cover{nullptr};
      NearestNeighborsPtr nearest_vertex{nullptr};

      Configuration *q_start{nullptr};
      Configuration *q_goal{nullptr};
      PDF pdf_necessary_configurations;
      PDF pdf_all_configurations;

      double threshold_clearance{0.01};
      double epsilon_min_distance{1e-10};
      bool saturated{false}; //if space is saturated, then we the whole free space has been found

      Graph graph;

    public:

      const PDF& GetPDFNecessaryVertices() const;
      const PDF& GetPDFAllVertices() const;
      double GetGoalBias() const;

    };
  }
}


#pragma once
#include "planner/strategy/quotientchart/quotient_chart.h"
#include <ompl/datastructures/PDF.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <boost/pending/disjoint_sets.hpp>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl
{
  namespace geometric
  {
    class QuotientChartCover: public og::QuotientChart
    {
      typedef og::QuotientChart BaseT;
    public:

      uint verbose{2};

      QuotientChartCover(const ob::SpaceInformationPtr &si, Quotient *parent = nullptr);
      ~QuotientChartCover(void);
      virtual void clear() override;
      virtual void setup() override;

      typedef uint vertex_index_type;
      
    protected:

      //#######################################################################
      //Configuration
      //#######################################################################
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
          pdf_element = element_;
        }
        void* GetPDFElement()
        {
          return pdf_element;
        }
        void SetNecessaryPDFElement(void *element_)
        {
          pdf_necessary_element = element_;
        }
        void* GetNecessaryPDFElement()
        {
          return pdf_necessary_element;
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
        Configuration *parent_neighbor{nullptr};

        bool isSufficientFeasible{false};
        void *pdf_element;
        void *pdf_necessary_element;

        bool isStart{false};
        bool isGoal{false};

        vertex_index_type index{0};

        //#####################################################################
        //Neighborhood Computations
        //#####################################################################
        double openNeighborhoodRadius{0.0}; //might be L1 or L2 radius
      };

      class EdgeInternalState{
        public:
          EdgeInternalState() = default;
          EdgeInternalState(ob::Cost cost_): cost(cost_)
          {};
          EdgeInternalState(double w): cost(w)
          {};
          EdgeInternalState(const EdgeInternalState &eis)
          {
            cost = eis.cost;
            isSufficient = eis.isSufficient;
          }
          void setWeight(double d){
            cost = ob::Cost(d);
          }
          ob::Cost getCost(){
            return cost;
          }
        private:
          ob::Cost cost{+dInf};
          bool isSufficient{false};
      };

      class GraphBundle{
        // put PDFs and neighborhood structures here
        std::string name;
      };

      typedef boost::adjacency_list<
         boost::listS, //do not change to vecS, otherwise vertex indices are not stable after removal
         boost::listS, 
         boost::undirectedS,
         Configuration*,
         //boost::property<boost::edge_index_t,int, EdgeInternalState> 
         EdgeInternalState,
         GraphBundle
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
      
      //keep manual track of indices, because we sometimes need to remove
      //vertices
      typedef std::map<typename boost::graph_traits<Graph>::vertex_descriptor, vertex_index_type> VertexToIndexMap;
      VertexToIndexMap vertexToIndexStdMap;
      boost::associative_property_map<VertexToIndexMap> vertexToIndex{vertexToIndexStdMap};

      typedef std::map<vertex_index_type, typename boost::graph_traits<Graph>::vertex_descriptor> IndexToVertexMap;
      IndexToVertexMap indexToVertexStdMap;
      boost::associative_property_map<IndexToVertexMap> indexToVertex{indexToVertexStdMap};

      vertex_index_type index_ctr{0};

      virtual void CopyChartFromSibling( QuotientChart *sibling, uint k ) override;

      // std::map<Vertex, VertexRank> vrank;
      // std::map<Vertex, Vertex> vparent;
      // boost::disjoint_sets<boost::associative_property_map<std::map<Vertex, VertexRank> >, boost::associative_property_map<std::map<Vertex, Vertex> > > 
      //   disjointSets_{boost::make_assoc_property_map(vrank), boost::make_assoc_property_map(vparent)};


      //#######################################################################
      //Configuration Create, Remove, Add 
      //#######################################################################

      Configuration* CreateConfigurationFromStateAndCoset(const ob::State *state, Configuration *q_coset);

      Vertex AddConfigurationToCover(Configuration *q);
      void RemoveConfigurationFromCover(Configuration *q);
      void AddEdge(Configuration *q_from, Configuration *q_to);

      Configuration* GetStartConfiguration() const;
      Configuration* GetGoalConfiguration() const;

      bool IsConfigurationInsideCover(Configuration *q);
      void RemoveConfigurationsFromCoverCoveredBy(Configuration *q);
      bool ComputeNeighborhood(Configuration *q);

      //#######################################################################
      //Distance Computations
      //#######################################################################
      double DistanceQ1(const Configuration *q_from, const Configuration *q_to);
      double DistanceX1(const Configuration *q_from, const Configuration *q_to);

      double DistanceConfigurationConfigurationCover(const Configuration *q_from, const Configuration *q_to);
      double DistanceConfigurationConfiguration(const Configuration *q_from, const Configuration *q_to);
      //Note: this is a pseudometric: invalidates second axiom of metric : d(x,y) = 0  iff x=y. But here we only have d(x,x)=0
      double DistanceNeighborhoodNeighborhood(const Configuration *q_from, const Configuration *q_to);
      //Note: this is a pseudometric: invalidates second axiom of metric : d(x,y) = 0  iff x=y. But here we only have d(x,x)=0
      double DistanceConfigurationNeighborhood(const Configuration *q_from, const Configuration *q_to);

      //#######################################################################
      //Sampling 
      //#######################################################################
      Configuration* SampleBoundary(std::string type);
      Configuration* Sample();
      Configuration* SampleValid(ob::PlannerTerminationCondition &ptc);
      Configuration* SampleQuotientCover(ob::State *state) const;
      void SampleGoal(Configuration*);
      void SampleUniform(Configuration*);

      bool sampleUniformOnNeighborhoodBoundary(ob::State *state, const Configuration *center);

      void Connect(const Configuration*, const Configuration*, Configuration*);
      void Grow(double t) override;
      void Init() override;
      bool GetSolution(ob::PathPtr &solution) override;

      bool Interpolate(const Configuration *q_from, Configuration *q_to);

      //#######################################################################
      //Neighborhood Set Computations
      //#######################################################################
      bool IsConfigurationInsideNeighborhood(Configuration *q, Configuration *qn);
      std::vector<Configuration*> GetConfigurationsInsideNeighborhood(Configuration *q);
      bool IsNeighborhoodInsideNeighborhood(Configuration *lhs, Configuration *rhs);

      //#######################################################################
      //Cover Algorithms
      //#######################################################################
      std::vector<Vertex> GetCoverPath(const Vertex& start, const Vertex& goal);
      Configuration* Nearest(Configuration *q) const;

      virtual void getPlannerDataAnnotated(base::PlannerData &data) const override;
      PlannerDataVertexAnnotated getAnnotatedVertex(Vertex vertex) const;
      PlannerDataVertexAnnotated getAnnotatedVertex(ob::State* state, double radius, bool sufficient) const;
      //#######################################################################
      RNG rng_;
      double goalBias{0.0}; //in [0,1]
      double voronoiBias{0.3}; //in [0,1]

      NearestNeighborsPtr nearest_cover{nullptr};
      NearestNeighborsPtr nearest_vertex{nullptr};

      Configuration *q_start{nullptr};
      Configuration *q_goal{nullptr};

      Vertex v_start;
      Vertex v_goal;

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



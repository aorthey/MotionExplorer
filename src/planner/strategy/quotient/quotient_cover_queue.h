#pragma once
#include "quotient_cover.h"
#include <ompl/datastructures/PDF.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <boost/pending/disjoint_sets.hpp>
#include <queue>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl
{
  namespace geometric
  {

    //@TODO: Rename to QuotientCoverAnalyzer, and put all the analysis tools in this class 
    //(nearest to goal, largest radius, least connection), and keep track of them here. All
    //other strategies can use this as a central points of information to aid
    //their choice of the best cover region to expand

    class StepStrategy;
    class QuotientCoverQueue: public og::QuotientCover
    {
      typedef og::QuotientCover BaseT;
    public:

      uint verbose{0};

      QuotientCoverQueue(const ob::SpaceInformationPtr &si, Quotient *parent = nullptr);
      ~QuotientCoverQueue(void);
      void clear() override;
      void setup() override;

      virtual Configuration* SampleCoverBoundary() override;
      virtual void AddConfigurationToPDF(Configuration *q) override;
      virtual void Print(std::ostream& out) const override;
      virtual Configuration* SampleUniformQuotientCover(ob::State *state) override;
      virtual Vertex AddConfigurationToCover(Configuration *q) override;

      void AddConfigurationToPriorityQueue(Configuration *q);
    protected:
      StepStrategy *step_strategy;

      uint NUMBER_OF_EXPANSION_SAMPLES{0};
      const double shortestPathBias{1.0};

      //Two Priorityqueues:
      // PriorityQueue Candidates: Nodes which have a computed neighborhood, but
      // have not been added yet. They can be thought of as (soon-to-be)
      // single-connection nodes
      // PriorityQueue Members: Nodes which are members of the cover.  They are
      // ordered depending on how likely they will lead to a discovery of free
      // space

      struct CmpCandidateConfigurationPtrs
      {
        bool operator()(const Configuration* lhs, const Configuration* rhs) const;
      };
      struct CmpMemberConfigurationPtrs
      {
        bool operator()(const Configuration* lhs, const Configuration* rhs) const;
      };
      typedef std::priority_queue<Configuration*, std::vector<Configuration*>, CmpCandidateConfigurationPtrs> CandidateConfigurationPriorityQueue;
      typedef std::priority_queue<Configuration*, std::vector<Configuration*>, CmpMemberConfigurationPtrs> MemberConfigurationPriorityQueue;

      CandidateConfigurationPriorityQueue priority_queue_candidate_configurations;
      MemberConfigurationPriorityQueue priority_queue_member_configurations;

    public:
      const CandidateConfigurationPriorityQueue& GetPriorityQueue();
      void PrintQueue(int n_head = std::numeric_limits<int>::infinity()); //print the fist n items
    };
  }
}


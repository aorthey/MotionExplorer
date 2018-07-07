#pragma once
#include "planner/strategy/quotient/quotient_graph.h"

namespace ob = ompl::base;

namespace ompl
{
  namespace geometric
  {
    class QuotientChart: public QuotientGraph
    {
        typedef og::QuotientGraph BaseT;
        typedef og::QuotientGraph::Graph Graph;
      public:
        QuotientChart(const ob::SpaceInformationPtr &si, QuotientChart *parent_ = nullptr);

        virtual void Grow(double t);
        bool FoundNewPath();
        virtual void setup() override;

        double GetImportance() const;
        void SetImportance(double); 
        uint GetLevel() const;
        void SetLevel(uint);
        uint GetHorizontalIndex() const;
        uint GetNumberOfSiblings() const;
        void SetHorizontalIndex(uint);
        uint GetNumberOfPaths() const;

        void AddSibling(QuotientChart *sibling_);
        void SetChild(QuotientChart *child_);

        //@brief: assume that there are K different solution paths on the graph. 
        //Extract the k-th solution path, plus all surrounding vertices.
        //"Surrounding" and "different" are implementation specific, for example
        //they could be "all visible vertices from path" and "not homotopic".
        //This is left for subclasses to implement.
        Graph GetPathSubgraph(uint k); 
        void SetGraph( Graph G_, QuotientChart *sibling);
        virtual void getPlannerData(ob::PlannerData &data) const override;


      private:
        double importance{0};//how important is the current chart for solving the problem
        double density{0};

        uint horizontal_index{0};
        uint level{0};

        uint number_of_paths{0};
        bool local_chart{false}; //local: a refinement chart around a given path, global: a chart exploring the whole quotient-space

        std::vector<QuotientGraph*> siblings;
        QuotientGraph *child{nullptr}; //singleton!
    };
  }
}

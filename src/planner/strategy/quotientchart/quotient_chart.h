#pragma once
#include "planner/strategy/quotient/quotient_graph.h"
#include "elements/plannerdata_vertex_annotated.h"

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
        QuotientChart(const ob::SpaceInformationPtr &si, Quotient *parent_ = nullptr);

        virtual void Grow(double t);
        bool FoundNewPath();
        virtual void setup() override;

        double GetImportance() const;
        void SetImportance(double); 
        std::vector<int> GetPath() const;
        uint GetHorizontalIndex() const;
        uint GetNumberOfSiblings() const;
        void SetHorizontalIndex(uint);
        uint GetNumberOfPaths() const;

        void AddSibling(QuotientChart *sibling_);

        //IsSaturated(): chart cannot be grown anymore
        //e.g. the sampling sequence hit some upper density limit,
        //or the structure has not been changed for at least 1/M new samples
        //(see visibilityPRM)
        bool IsSaturated() const; 

        //@brief: assume that there are K different solution paths on the graph. 
        //Extract the k-th solution path, plus all surrounding vertices.
        //"Surrounding" and "different" are implementation specific, for example
        //they could be "all visible vertices from path" and "not homotopic".
        //This is left for subclasses to implement.
        virtual void SetSubGraph( QuotientChart *sibling, uint k );
        virtual void getPlannerData(ob::PlannerData &data) const override;
        //virtual void setPlannerDataVertexProperties(PlannerDataVertexAnnotated &p);
        virtual PlannerDataVertexAnnotated getPlannerDataVertex(ob::State *state, Vertex v) const;

      private:
        PlannerDataVertexAnnotated getAnnotatedVertex(Vertex vertex, std::vector<int> &path, std::map<const Vertex, ob::State*> &vertexToStates) const;
      protected:

        double importance{0};//how important is the current chart for solving the problem
        double density{0};

        uint horizontal_index{0};

        uint number_of_paths{0};
        bool local_chart{false}; //local: a refinement chart around a given path, global: a chart exploring the whole quotient-space

        std::vector<QuotientGraph*> siblings;
    };
  }
}

#pragma once
#include "planner/strategy/quotient/quotient.h"
#include "elements/plannerdata_vertex_annotated.h"

namespace ob = ompl::base;

namespace ompl
{
  namespace geometric
  {
    class QuotientChart: public Quotient
    {
        typedef og::Quotient BaseT;
      public:
        QuotientChart(const ob::SpaceInformationPtr &si, Quotient *parent_ = nullptr);

        virtual void Grow(double t) = 0;
        virtual void getPlannerDataAnnotated(ob::PlannerData &data) const = 0;

        bool FoundNewPath();

        virtual void setup() override;

        std::vector<int> GetChartPath() const;

        uint GetChartHorizontalIndex() const;
        void SetChartHorizontalIndex(uint);

        uint GetChartNumberOfSiblings() const;
        uint GetChartNumberOfComponents() const;

        void AddChartSibling(QuotientChart *sibling_);

        virtual bool IsSaturated() const;
        //@brief: assume that there are K different solution paths on the graph. 
        //Extract the k-th solution path, plus all surrounding vertices.
        //"Surrounding" and "different" are implementation specific, for example
        //they could be "all visible vertices from path" and "not homotopic".
        //This is left for subclasses to implement.

        virtual void CopyChartFromSibling( QuotientChart *sibling, uint k ) = 0;

        virtual void getPlannerData(ob::PlannerData &data) const override;


        //virtual void setPlannerDataVertexProperties(PlannerDataVertexAnnotated &p);
        //virtual PlannerDataVertexAnnotated getPlannerDataVertex(ob::State *state, Vertex v) const;

      //private:
        //PlannerDataVertexAnnotated getAnnotatedVertex(Vertex vertex, std::vector<int> &path, std::map<const Vertex, ob::State*> &vertexToStates) const;
      protected:

        std::vector<int> chartPath;
        uint chartHorizontalIndex{0};
        uint chartNumberOfComponents{0};
        bool isLocalChart{false}; //local: a refinement chart around a given path, global: a chart exploring the whole quotient-space
        std::vector<QuotientChart*> chartSiblings;
    };
  }
}

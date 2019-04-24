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
        typedef Quotient BaseT;
      public:
        QuotientChart(const ob::SpaceInformationPtr &si, Quotient *parent_ = nullptr);

        virtual void Grow(double t) = 0;
        virtual void getPlannerDataAnnotated(ob::PlannerData &data) const = 0;

        virtual bool FoundNewComponent();

        virtual void setup() override;
        virtual void clear() override;

        std::vector<int> GetChartPath() const;
        void UpdateChartPath();

        uint GetChartHorizontalIndex() const;
        void SetChartHorizontalIndex(uint);

        uint GetChartNumberOfSiblings() const;
        uint GetChartNumberOfComponents() const;

        void AddChartSibling(QuotientChart *sibling_);
        void DeleteSubCharts();

        virtual bool IsSaturated() const;

        //@brief: assume that there are K different solution paths on the graph. 
        //Extract the k-th solution path, plus all surrounding vertices.
        //"Surrounding" and "different" are implementation specific, for example
        //they could be "all visible vertices from path" and "not homotopic".
        //This is left for subclasses to implement.
        virtual void CopyChartFromSibling( QuotientChart *sibling, uint k ) = 0;

        virtual void getPlannerData(ob::PlannerData &data) const override;
        virtual void Print(std::ostream& out) const override;
      protected:

        std::vector<int> chartPath;
        uint chartHorizontalIndex{0};
        uint chartNumberOfComponents{0};
        bool isLocalChart{false}; //local: a refinement chart around a given path, global: a chart exploring the whole quotient-space
        std::vector<QuotientChart*> chartSiblings;
    };
  }
}

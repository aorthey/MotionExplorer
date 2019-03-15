#pragma once
#include "planner/strategy/quotientgraph/quotient_graph.h"
#include <ompl/datastructures/PDF.h>


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
    //QRRT: Quotient-Space Rapidly Exploring Random Tree Algorithm
    class QRRT: public og::QuotientGraph{

      typedef og::QuotientGraph BaseT;
      public:

        QRRT(const ob::SpaceInformationPtr &si, Quotient *parent_);
        virtual ~QRRT() override;
        virtual void Grow(double t) override;
        virtual bool GetSolution(ob::PathPtr &solution) override;
        double GetImportance() const override;
        virtual bool Sample(ob::State *q_random) override;

        virtual void setup() override;
        virtual void clear() override;

        void setGoalBias(double goalBias);
        double getGoalBias() const;
        void setRange(double distance);
        double getRange() const;

      protected:

        double maxDistance{.0};
        double goalBias{.05};
        double epsilon{.0};

        ob::Goal *goal;
        Configuration *q_random{nullptr};

        virtual bool SampleQuotient(ob::State*) override;

    };

  };
};

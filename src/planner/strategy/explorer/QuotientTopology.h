#pragma once
#include "QuotientGraphSparse.h"
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
    //QuotientTopology 
    class QuotientTopology: public og::QuotientGraphSparse{

      typedef og::QuotientGraphSparse BaseT;
      public:

        QuotientTopology(const ob::SpaceInformationPtr &si, Quotient *parent_);
        virtual ~QuotientTopology() override;
        virtual void Grow(double t) override;
        virtual bool GetSolution(ob::PathPtr &solution) override;

        virtual void setup() override;
        virtual void clear() override;

        void setGoalBias(double goalBias);
        double getGoalBias() const;
        void setRange(double distance);
        double getRange() const;

        Configuration *q_random{nullptr};
      protected:

        double maxDistance{.0};
        double goalBias{.05};
        double epsilon{.0};

        ob::Goal *goal;


    };

  };
};

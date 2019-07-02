#pragma once
#include "QuotientSubGraph.h"
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
    class QuotientTopology: public og::QuotientSubGraph{

      typedef og::QuotientSubGraph BaseT;
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
        Vertex v_last_added;

        std::vector<Vertex> shortestPathVertices;

        double maxDistance{.0};
        double goalBias{.05};
        double shortestPathBias{.05};
        double epsilon{.0};

        ob::Goal *goal;


    };

  };
};

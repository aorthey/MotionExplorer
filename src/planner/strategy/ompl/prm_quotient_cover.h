#pragma once
#include "prm_quotient_connect.h"
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
    class PRMQuotientConnectCover: public og::PRMQuotientConnect{

      public:

        PRMQuotientConnectCover(const ob::SpaceInformationPtr &si, Quotient *previous_);

        virtual Vertex CreateNewVertex(ob::State *state) override;

        virtual void getPlannerData(ob::PlannerData &data) const override;

        virtual void ClearVertices() override;
      //protected:

        //virtual bool SampleGraph(ob::State*) override;

        //virtual bool Connect(const Vertex a, const Vertex b) override;
      protected:
        double delta{0.05};

    };

  };
};

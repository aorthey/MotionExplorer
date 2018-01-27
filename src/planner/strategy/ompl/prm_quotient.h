#pragma once
#include "prm_basic.h"
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
    class PRMQuotient: public og::PRMBasic{

      public:

        PRMQuotient(const ob::SpaceInformationPtr &si, Quotient *previous_);
        virtual ~PRMQuotient() override;

        void getPlannerData(base::PlannerData &data) const override;

        double getSamplingDensity();

        virtual void Init();

        void setup() override;
        void clear() override;

        Vertex lastSourceVertexSampled;
        Vertex lastTargetVertexSampled;
        double lastTSampled;
        bool isSampled{false};

        virtual void Grow(double t) override;

      protected:

        //Overrides Distance/Sample/Connect
        virtual double Distance(const Vertex a, const Vertex b) const override;
        virtual bool Sample(ob::State*) override;
        virtual bool Connect(const Vertex a, const Vertex b) override;
        virtual bool SampleGraph(ob::State*) override;
        virtual ompl::PDF<og::PRMBasic::Edge> GetEdgePDF();

        virtual Vertex addMilestone(ob::State *state) override;

    };

  };
};

#pragma once
#include "q_rrt.h"


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
    class QRRTConnect: public og::QRRT{

      typedef og::QRRT BaseT;
      public:

        QRRTConnect(const ob::SpaceInformationPtr &si, Quotient *parent_);
        virtual bool Sample(ob::State *q_random) override;
        PDF vpdf;
        virtual void Grow() override;
        virtual Vertex AddConfiguration(Configuration *q) override;
        virtual void clear() override;

        Configuration *q_last_added{nullptr};

      protected:

        virtual bool SampleQuotient(ob::State*) override;

    };

  };
};

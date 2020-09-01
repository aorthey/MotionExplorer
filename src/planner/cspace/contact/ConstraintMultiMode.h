#pragma once
#include <ompl/base/Constraint.h>
#include <ompl/util/RandomNumbers.h>

class ConstraintMultiMode: public ompl::base::Constraint
{
  public:
    ConstraintMultiMode(int ambientSpaceDim);

    virtual int getNumberOfModes() = 0;
    virtual int setRandomMode();
    virtual void setMode(int mode);
    int getMode() const;

  private:
    int mode_{0};
    ompl::RNG randomNumberGenerator_;
};


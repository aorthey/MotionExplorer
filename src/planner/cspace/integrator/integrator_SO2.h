#pragma once
#include <KrisLibrary/planning/KinodynamicCSpace.h>
#include <KrisLibrary/math/random.h>
#include <KrisLibrary/math3d/primitives.h>
#include <KrisLibrary/math3d/rotation.h>
#include <KrisLibrary/math/diffeq.h>
#include <Planning/RobotCSpace.h>
#include <vector>
#include "planner/cspace/cspace.h"

using namespace Math3D;
class KinodynamicCSpaceOMPL;

#include <ompl/base/StateSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

namespace oc = ompl::control;
namespace ob = ompl::base;

class IntegratorSO2 : public oc::StatePropagator
{
public:

    IntegratorSO2(oc::SpaceInformationPtr si, KinodynamicCSpaceOMPL *cspace_) : 
        oc::StatePropagator(si.get()), cspace(cspace_)
    {
    }
    virtual void propagate(const ob::State *state, const oc::Control* control, const double duration, ob::State *result) const override;

    KinodynamicCSpaceOMPL *cspace;
};


#pragma once
#include <KrisLibrary/planning/KinodynamicCSpace.h>
#include <KrisLibrary/math3d/primitives.h>
#include <KrisLibrary/math3d/rotation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/StateSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/SpaceInformation.h>
namespace oc = ompl::control;
namespace ob = ompl::base;

ob::ScopedState<> ConfigToOMPLState(const Config &q, const ob::StateSpacePtr &s);
ob::State* ConfigToOMPLStatePtr(const Config &q, const ob::StateSpacePtr &s);
Config OMPLStateToConfig(const ob::ScopedState<> &qompl, const ob::StateSpacePtr &s);
Config OMPLStateToConfig(const ob::State *qompl, const ob::StateSpacePtr &s);
Config OMPLStateToConfig(const ob::SE3StateSpace::StateType *qomplSE3, const ob::RealVectorStateSpace::StateType *qomplRnState, const ob::StateSpacePtr &s);


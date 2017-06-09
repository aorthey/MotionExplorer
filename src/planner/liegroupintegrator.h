#pragma once
#include <KrisLibrary/planning/KinodynamicCSpace.h>
#include <KrisLibrary/math/random.h>
#include <KrisLibrary/math3d/primitives.h>
#include <KrisLibrary/math3d/rotation.h>
#include <KrisLibrary/math/diffeq.h>
#include <Planning/RobotCSpace.h>
#include <vector>

//#include <ompl/control/SpaceInformation.h>
//SE(3) element: X Y Z rotZ rotY rotX
//SE(3) element: X Y Z yaw pitch roll
//SE(3) element: homogenous 4x4 matrix, rotation 3x3, translation 3x1
//using namespace Math3D;
//namespace oc = ompl::control;

class LieGroupIntegrator
{

  public:
    //LieGroupIntegrator(oc::SpaceInformationPtr si): oc::StatePropagator(si.get())
    LieGroupIntegrator()
    {
    };
    //integration matrix elements
    Matrix4 Integrate(const Math3D::Matrix4& p0, const Math3D::Matrix4& dp0, double dt);

    //integration vector elements
    void Simulate(const State& x0, const ControlInput& u,std::vector<State>& p);
    void SimulateEndpoint(const State& x0, const ControlInput& u,State& x1);

    //integration ompl vector elements
    //namespace ob = ompl::base;
    //virtual void propagate(const ompl::base::State *state, const oc::Control* control, const double duration, ompl::base::State *result) const override;

    void Euler_step(std::vector<Matrix4>& p, const Matrix4& dp0, double dt);
    Matrix4 MatrixExponential(const Matrix4& x);
    Matrix4 SE3Derivative(const ControlInput& u);

    Matrix4 StateToSE3(const State& x);
    void SE3ToState(State& x, const Matrix4& x_SE3);

};

#pragma once
#include <Modeling/World.h> //RobotWorld*
#include <KrisLibrary/robotics/RobotKinematics3D.h> //Config (implementation in KrisLibrary/math/VectorTemplate.h)
#include <KrisLibrary/math/infnan.h> //dInf, fInf, IsNaN(x)
#include <KrisLibrary/math3d/primitives.h> //Vector{2,3,4} Matrix{2,3,4} RigidTransform
using Math::dInf;
using Math::fInf;
const static double iInf = std::numeric_limits<int>::infinity();
#include <KrisLibrary/planning/CSpace.h>
typedef CSpace CSpaceKlampt;

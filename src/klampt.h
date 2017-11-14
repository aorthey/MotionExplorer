#pragma once
#include <KrisLibrary/robotics/RobotKinematics3D.h> //Config
#include <KrisLibrary/math/infnan.h> //dInf, fInf, IsNaN(x)
using Math::dInf;
using Math::fInf;
const static double iInf = std::numeric_limits<int>::infinity();
#include <KrisLibrary/planning/CSpace.h>
typedef CSpace CSpaceKlampt;




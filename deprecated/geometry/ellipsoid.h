#pragma once
#include "gui/common.h"
#include "gui/gui_state.h"
#include "klampt.h"
#include <Eigen/Core>


class Ellipsoid
{
  public:
    Ellipsoid() = delete;
    Ellipsoid(Eigen::MatrixXd C_, Eigen::VectorXd d_);

    void DrawGL(GUIState&);

    GLDraw::GLColor color{grey};
    int numSteps{16};
    bool filled{false};

  private:
    void drawEllipsoid(Vector3 &c, Vector3 &u, Vector3 &v, Vector3 &w);
    void drawWireEllipsoid(Vector3 &c, Vector3 &u, Vector3 &v, Vector3 &w);

    Eigen::MatrixXd C;
    Eigen::VectorXd d;
};

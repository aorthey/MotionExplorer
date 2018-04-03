#include <Eigen/Core>
class VertexRepresentation{

  public:
    VertexRepresentation(Eigen::MatrixXd A, Eigen::VectorXd b);

    Eigen::MatrixXd GetVertices();
  private:
    Eigen::MatrixXd A_eigen;
    Eigen::VectorXd b_eigen;
};

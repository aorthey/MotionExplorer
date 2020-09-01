#pragma once
#include <algorithm>    // std::all_of
#include <KrisLibrary/math3d/primitives.h> //Vector3

class ConstraintMode
{

  public:

    ConstraintMode();
    bool isValid() const;
    void add(int c);
    void clear();
    int size() const;
    int at(int) const;
    bool canReach(const ConstraintMode &lhs) const;
    bool isLargerAs(const ConstraintMode &lhs) const;
    int countInactiveConstraints() const;
    friend std::ostream& operator<< (std::ostream& out, const ConstraintMode& cmode);

    const std::vector<int>& getConstraintModeIndices() const;

    const std::vector<int>& getFixedConstraintIndices() const;

    const std::vector<Math3D::Vector3>& getFixedConstraintContactPoints() const;

    int numberFixedConstraints() const;

    void addFixedConstraint(int, Math3D::Vector3);
    void clearFixedConstraint();


  protected:

    std::vector<int> constraintModeIndices_;

    std::vector<Math3D::Vector3> fixedConstraintContactPoints_;

    std::vector<int> fixedConstraintIndices_;

};

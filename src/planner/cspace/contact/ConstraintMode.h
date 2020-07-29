#pragma once
#include <algorithm>    // std::all_of

class ConstraintMode
{
  public:
    ConstraintMode();
    bool isValid();
    void add(int c);
    void clear();
    int size() const;
    int at(int) const;
    bool canReach(const ConstraintMode &lhs);
    bool isLargerAs(const ConstraintMode &lhs);
    int countInactiveConstraints() const;
    friend std::ostream& operator<< (std::ostream& out, const ConstraintMode& cmode);
  protected:
    std::vector<int> constraintMode_;

};

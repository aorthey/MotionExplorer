#include "planner/cspace/contact/ConstraintMode.h"
#include <iostream>

ConstraintMode::ConstraintMode()
{
}


bool ConstraintMode::isValid()
{
  //at least one contraint needs to be active (i.e. not equal to zero)
  if(constraintMode_.size() <= 0) return false;

  bool zeros = std::all_of(constraintMode_.begin(), constraintMode_.end(), 
      [](int content) { return content==0; });
  // std::cout << "Zeros:" << (zeros?"Yes":"No") << " " << constraintMode_.size() << " constraints" << std::endl;

  return !zeros;
}
int ConstraintMode::size() const
{
  return constraintMode_.size();
}
int ConstraintMode::at(int k) const
{
  return constraintMode_.at(k);
}

void ConstraintMode::add(int c)
{
  constraintMode_.push_back(c);
}
void ConstraintMode::clear()
{
  constraintMode_.clear();
}
bool ConstraintMode::canReach(const ConstraintMode &lhs)
{
  int r = this->countInactiveConstraints();
  int l = lhs.countInactiveConstraints();
  return (r == l-1 || r == l+1);
}
bool ConstraintMode::isLargerAs(const ConstraintMode &lhs)
{
  int r = this->countInactiveConstraints();
  int l = lhs.countInactiveConstraints();
  return (r > l);
}
int ConstraintMode::countInactiveConstraints() const
{
  return std::count(constraintMode_.begin(), constraintMode_.end(), 0);
}

std::ostream& operator<< (std::ostream& out, const ConstraintMode& cmode)
{
  for(int k = 0; k < cmode.size(); k++){
    out << cmode.at(k) << " ";
  }
  return out;
}

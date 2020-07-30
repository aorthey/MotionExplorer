#include "planner/cspace/contact/ConstraintMode.h"
#include <iostream>

ConstraintMode::ConstraintMode()
{
}


bool ConstraintMode::isValid() const
{
  //at least one contraint needs to be active (i.e. not equal to zero)
  if(constraintModeIndices_.size() <= 0) return false;

  bool zeros = std::all_of(constraintModeIndices_.begin(), constraintModeIndices_.end(), 
      [](int content) { return content<=0; });

  // bool ones = std::all_of(constraintModeIndices_.begin(), constraintModeIndices_.end(), 
  //     [](int content) { return content>=1; });
  return (!zeros);
}

int ConstraintMode::numberFixedConstraints() const
{
  return fixedConstraintIndices_.size();
}

int ConstraintMode::size() const
{
  return constraintModeIndices_.size();
}
int ConstraintMode::at(int k) const
{
  return constraintModeIndices_.at(k);
}

void ConstraintMode::add(int c)
{
  constraintModeIndices_.push_back(c);
}
void ConstraintMode::clear()
{
  constraintModeIndices_.clear();
  clearFixedConstraint();
}
bool ConstraintMode::canReach(const ConstraintMode &rhs) const
{
  if(!rhs.isValid()) return false;

  const std::vector<int>& cRhs = rhs.getConstraintModeIndices();
  if(rhs.size() != size())
  {
    std::cout << "ERROR: size of arrays do not match up." << std::endl;
    throw "";
  }
  int ctr = 0;
  //compute number of modes in which lhs and rhs differ
  for(uint k = 0; k < cRhs.size(); k++)
  {
    if(at(k) != cRhs.at(k))
    {
      ctr++;
    }
    if(ctr > 1) break;
  }
  //either they are in same mode or they differ by at most one mode
  return (ctr <= 1);
}

const std::vector<int>& ConstraintMode::getConstraintModeIndices() const
{
  return constraintModeIndices_;
}
const std::vector<int>& ConstraintMode::getFixedConstraintIndices() const
{
  return fixedConstraintIndices_;
}
const std::vector<Math3D::Vector3>& ConstraintMode::getFixedConstraintContactPoints() const
{
  return fixedConstraintContactPoints_;
}

bool ConstraintMode::isLargerAs(const ConstraintMode &lhs) const
{
  int r = this->countInactiveConstraints();
  int l = lhs.countInactiveConstraints();
  return (r > l);
}
int ConstraintMode::countInactiveConstraints() const
{
  return std::count(constraintModeIndices_.begin(), constraintModeIndices_.end(), 0);
}
void ConstraintMode::addFixedConstraint(int idx, Math3D::Vector3 v)
{
    fixedConstraintIndices_.push_back(idx);
    fixedConstraintContactPoints_.push_back(v);
}
void ConstraintMode::clearFixedConstraint()
{
    fixedConstraintIndices_.clear();
    fixedConstraintContactPoints_.clear();
}

std::ostream& operator<< (std::ostream& out, const ConstraintMode& cmode)
{
  for(int k = 0; k < cmode.size(); k++){
    out << cmode.at(k) << " ";
  }
  return out;
}

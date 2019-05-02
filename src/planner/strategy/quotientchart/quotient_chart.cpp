#include "quotient_chart.h"
#include "common.h"
#include <ompl/geometric/planners/prm/ConnectionStrategy.h>
#include <boost/foreach.hpp>

using namespace ompl::geometric;
#define foreach BOOST_FOREACH

QuotientChart::QuotientChart(const ob::SpaceInformationPtr &si, Quotient *parent_)
  : BaseT(si, parent_)
{
  UpdateChartPath();
}

void QuotientChart::setup() 
{
  if(!isLocalChart) BaseT::setup();
}

void QuotientChart::clear() 
{
  BaseT::clear();
  for(uint k = 0; k < chartSiblings.size(); k++){
    chartSiblings.at(k)->clear();
  }
  isLocalChart = false;
  chartNumberOfComponents = 0;
  chartNumberOfComponentsLastIteration = 0;
}

void QuotientChart::DeleteSubCharts()
{
  for(uint k = 0; k < chartSiblings.size(); k++){
    chartSiblings.at(k)->DeleteSubCharts();
  }
  for(uint k = 0; k < chartSiblings.size(); k++){
    delete chartSiblings.at(k);
  }
  chartSiblings.clear();
  if(child){
    static_cast<QuotientChart*>(child)->DeleteSubCharts();
    delete child;
  }
}

uint QuotientChart::GetChartHorizontalIndex() const
{
  return chartHorizontalIndex;
}
void QuotientChart::SetChartHorizontalIndex(uint chartHorizontalIndex_)
{
  chartHorizontalIndex = chartHorizontalIndex_;

  UpdateChartPath();
}

void QuotientChart::UpdateChartPath() 
{
  chartPath.clear();
  chartPath.push_back(chartHorizontalIndex);
  QuotientChart *parent = static_cast<QuotientChart*>(GetParent());
  while(parent!=nullptr)
  {
    chartPath.push_back(parent->GetChartHorizontalIndex());
    parent = static_cast<QuotientChart*>(parent->GetParent());
  }
  std::reverse(std::begin(chartPath), std::end(chartPath));
}

bool QuotientChart::FoundNewComponent()
{
  if(chartNumberOfComponents<=0 && hasSolution){
    chartNumberOfComponents++;
    return true;
  }else{
    return false;
  }
}

void QuotientChart::AddChartSibling(QuotientChart *sibling_)
{
  chartSiblings.push_back(sibling_);
}

uint QuotientChart::GetChartNumberOfComponents() const
{
  return chartNumberOfComponents;
}

uint QuotientChart::GetChartNumberOfSiblings() const
{
  return chartSiblings.size();
}
std::vector<int> QuotientChart::GetChartPath() const
{
  return chartPath;
}
bool QuotientChart::IsSaturated() const
{
  return false;
}

void QuotientChart::getPlannerData(ob::PlannerData &data) const
{
  getPlannerDataAnnotated(data);
  // std::cout << std::string(80, '-') << std::endl;
  // std::cout << *this << std::endl;
  for(uint i = 0; i < chartSiblings.size(); i++){
    chartSiblings.at(i)->getPlannerData(data);
  }
  if(child!=nullptr) child->getPlannerData(data);
}

void QuotientChart::Print(std::ostream& out) const
{
  BaseT::Print(out);
  out << std::endl << " |-- [Chart] " << (isLocalChart?"(local)":"") << " level " << level << " | hIdx " << chartHorizontalIndex
    << " | siblings " << chartSiblings.size() << " | chart_path [" << chartPath << "]";
}

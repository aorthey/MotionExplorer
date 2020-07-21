#include <ompl/base/SpaceInformation.h>
#include <ompl/tools/benchmark/Benchmark.h>

#include <boost/math/constants/constants.hpp>
#include <boost/range/irange.hpp>
#include <boost/range/algorithm_ext/push_back.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>

namespace ot = ompl::tools;
namespace ob = ompl::base;
namespace og = ompl::geometric;

const double edgeWidth = 0.1; //original STRIDE paper had edgewidth = 0.1

// Note: Number of all simplifications is
// unsigned int numberSimplifications = std::pow(2, curDim - 1);
// But here we will only create three simplifications, the trivial one, the
// discrete one and a two-step simplifications, which we found worked well in
// this experiment. You can experiment with finding better simplifications.
// std::cout << "dimension: " << curDim << " simplifications:" << numberSimplifications << std::endl;

std::vector<int> getHypercubeAdmissibleProjection(int dim)
{
    std::vector<int> discrete;
    boost::push_back(discrete, boost::irange(2, dim + 1));
    // std::cout << "[";
    // for(uint k = 0; k < discrete.size(); k++){
    //   std::cout << discrete.at(k) << ",";
    // }
    // std::cout << "]" << std::endl;
    return discrete;
}

std::vector<std::vector<int>> getHypercubeAdmissibleProjections(int dim)
{
    std::vector<std::vector<int>> projections;

    // trivial: just configuration space
    // discrete: use all admissible projections
    std::vector<int> trivial{dim};

    std::vector<int> discrete;
    boost::push_back(discrete, boost::irange(2, dim + 1));

    // std::vector<int> twoStep;
    // boost::push_back(twoStep, boost::irange(2, dim + 1, 2));

    // if (twoStep.back() != dim)
    //     twoStep.push_back(dim);
    // projections.push_back(twoStep);

    projections.push_back(discrete);
    auto last = std::unique(projections.begin(), projections.end());
    projections.erase(last, projections.end());

    std::cout << "Projections for dim " << dim << std::endl;
    for(unsigned int k = 0; k < projections.size(); k++){
        std::vector<int> pk = projections.at(k);
        std::cout << k << ": ";
        for(unsigned int j = 0; j < pk.size(); j++){
          std::cout << pk.at(j) << (j<pk.size()-1?",":"");
        }
        std::cout << std::endl;
    }

    return projections;
}


// Only states near some edges of a hypercube are valid. The valid edges form a
// narrow passage from (0,...,0) to (1,...,1). A state s is valid if there exists
// a k s.t. (a) 0<=s[k]<=1, (b) for all i<k s[i]<=edgeWidth, and (c) for all i>k
// s[i]>=1-edgewidth.
class HyperCubeValidityChecker : public ob::StateValidityChecker
{
public:
    HyperCubeValidityChecker(const ob::SpaceInformationPtr &si, int dimension)
      : ob::StateValidityChecker(si), dimension_(dimension)
    {
        si->setStateValidityCheckingResolution(0.001);
    }

    bool isValid(const ob::State *state) const override
    {
        const auto *s = static_cast<const ob::RealVectorStateSpace::StateType *>(state);
        bool foundMaxDim = false;

        for (int i = dimension_ - 1; i >= 0; i--)
        {
            if (!foundMaxDim)
            {
                if ((*s)[i] > edgeWidth)
                    foundMaxDim = true;
            }
            else if ((*s)[i] < (1. - edgeWidth))
                return false;
        }
        return true;
    }

protected:
    int dimension_;
};

template<typename T>  
ob::PlannerPtr GetMultiLevelPlanner(std::vector<int> sequenceLinks, ob::SpaceInformationPtr si, std::string name="Planner")
{
    // ompl::msg::setLogLevel(ompl::msg::LOG_DEV2);
    std::vector<ob::SpaceInformationPtr> si_vec;

    for (unsigned int k = 0; k < sequenceLinks.size() - 1; k++)
    {
        int links = sequenceLinks.at(k);

        auto spaceK(std::make_shared<ompl::base::RealVectorStateSpace>(links));
        ompl::base::RealVectorBounds bounds(links);
        bounds.setLow(0.);
        bounds.setHigh(1.);
        spaceK->setBounds(bounds);

        auto siK = std::make_shared<ob::SpaceInformation>(spaceK);
        siK->setStateValidityChecker(std::make_shared<HyperCubeValidityChecker>(siK, links));

        spaceK->setup();
        si_vec.push_back(siK);
    }
    si_vec.push_back(si);

    auto planner = std::make_shared<T>(si_vec, name);
    return planner;
}


#include <ompl/base/SpaceInformation.h>
#include <ompl/tools/benchmark/Benchmark.h>

//include planners
#include <ompl/geometric/planners/bitstar/BITstar.h>
#include <ompl/geometric/planners/bitstar/ABITstar.h>
#include <ompl/geometric/planners/est/BiEST.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/est/ProjEST.h>
#include <ompl/geometric/planners/fmt/BFMT.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/pdst/PDST.h>
#include <ompl/geometric/planners/prm/LazyPRM.h> //TODO: segfault
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/prm/SPARS.h>
#include <ompl/geometric/planners/prm/SPARStwo.h>
#include <ompl/geometric/planners/multilevel/QRRT.h>
#include <ompl/geometric/planners/multilevel/QRRTStar.h>
#include <ompl/geometric/planners/multilevel/QMP.h>
#include <ompl/geometric/planners/multilevel/QMPStar.h>
#include <ompl/geometric/planners/multilevel/SPQR.h>
#include <ompl/geometric/planners/rrt/BiTRRT.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTXstatic.h>
#include <ompl/geometric/planners/rrt/SORRTstar.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/geometric/planners/sbl/pSBL.h> //TODO: parallel algorithms return infeasible solutions
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/sst/SST.h>
#include <ompl/geometric/planners/stride/STRIDE.h>

#include <boost/lexical_cast.hpp>

namespace ot = ompl::tools;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void printBenchmarkResults(const ot::Benchmark &b)
{
    ot::Benchmark::CompleteExperiment experiment = b.getRecordedExperimentData();

    std::vector<double> meanTime;
    std::vector<std::string> plannerName;
    std::map<double, std::pair<std::string, int>> plannerTimes;

    for (unsigned int k = 0; k < experiment.planners.size(); k++)
    {
        ot::Benchmark::PlannerExperiment pk = experiment.planners.at(k);
        std::vector<ot::Benchmark::RunProperties> runs = pk.runs;

        unsigned int N = runs.size();
        double time = 0;
        double percentSuccess = 0.0;
        for (unsigned int j = 0; j < N; j++)
        {
            ot::Benchmark::RunProperties run = runs.at(j);
            double timeJrun = std::atof(run["time REAL"].c_str());
            bool runSolved = std::atoi(run["solved BOOLEAN"].c_str());

            if(!runSolved) timeJrun = experiment.maxTime;

            time += timeJrun;
            if (timeJrun < experiment.maxTime)
                percentSuccess++;
        }

        time = time / (double)N;
        percentSuccess = 100.0 * (percentSuccess / (double)N);
        pk.name.erase(0, 10);

        plannerTimes[time] = std::make_pair(pk.name, percentSuccess);
    }

    std::cout << "Finished Benchmark (Runtime: " << experiment.maxTime << ", RunCount: " << experiment.runCount << ")"
              << std::endl;
    std::cout << "Placement <Rank> <Time (in Seconds)> <Success (in Percentage)>" << std::endl;
    unsigned int ctr = 1;
    std::cout << std::string(80, '-') << std::endl;
    for (auto const &p : plannerTimes)
    {
        std::cout << "Place <" << ctr 
          << "> Time: <" << p.first 
          << "> \%Success: <" << p.second.second 
          << "> (" << p.second.first << ")" << std::endl;
        ctr++;
    }
    std::cout << std::string(80, '-') << std::endl;
}

void printEstimatedTimeToCompletion(unsigned int numberPlanners, unsigned int run_count, unsigned int runtime_limit)
{
    std::cout << std::string(80, '-') << std::endl;
    double worst_case_time_estimate_in_seconds = numberPlanners * run_count * runtime_limit;
    double worst_case_time_estimate_in_minutes = worst_case_time_estimate_in_seconds / 60.0;
    double worst_case_time_estimate_in_hours = worst_case_time_estimate_in_minutes / 60.0;
    std::cout << "Number of Planners           : " << numberPlanners << std::endl;
    std::cout << "Number of Runs Per Planner   : " << run_count << std::endl;
    std::cout << "Time Per Run (s)             : " << runtime_limit << std::endl;
    std::cout << "Worst-case time requirement  : ";

    if (worst_case_time_estimate_in_hours < 1)
    {
        if (worst_case_time_estimate_in_minutes < 1)
        {
            std::cout << worst_case_time_estimate_in_seconds << "s" << std::endl;
        }
        else
        {
            std::cout << worst_case_time_estimate_in_minutes << "m" << std::endl;
        }
    }
    else
    {
        std::cout << worst_case_time_estimate_in_hours << "h" << std::endl;
    }
    std::cout << std::string(80, '-') << std::endl;
}

static unsigned int numberRuns{0};

void PostRunEventHyperCube(const ob::PlannerPtr &planner, ot::Benchmark::RunProperties &run)
{
    static unsigned int pid = 0;

    ob::SpaceInformationPtr si = planner->getSpaceInformation();
    ob::ProblemDefinitionPtr pdef = planner->getProblemDefinition();

    unsigned int states = boost::lexical_cast<int>(run["graph states INTEGER"]);
    double time = boost::lexical_cast<double>(run["time REAL"]);
    double memory = boost::lexical_cast<double>(run["memory REAL"]);

    bool solved = boost::lexical_cast<bool>(run["solved BOOLEAN"]);

    double cost = std::numeric_limits<double>::infinity();
    if ( run.find("solution length REAL") != run.end() ) 
    {
      cost = boost::lexical_cast<double>(run["solution length REAL"]);
    }

    std::cout << "Run " << pid << "/" << numberRuns << " [" << planner->getName() << "] "
              << (solved ? "solved" : "FAILED") 
              << "(time: " << time 
              << ", cost: " << cost
              << ", states: " << states 
              << ", memory: " << memory
              << ")" << std::endl;
    std::cout << std::string(80, '-') << std::endl;
    pid++;
}

int numberPlanners = 0;

void addPlanner(ompl::tools::Benchmark &benchmark, const ompl::base::PlannerPtr &planner, double range = 1e-2)
{
    ompl::base::ParamSet &params = planner->params();
    if (params.hasParam(std::string("range")))
        params.setParam(std::string("range"), ompl::toString(range));
    benchmark.addPlanner(planner);
    numberPlanners++;
}

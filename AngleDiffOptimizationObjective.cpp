#include "AngleDiffOptimizationObjective.h"
#include "Defs.h"
#include "Utils.h"
#include <memory>
#if OMPL_HAVE_EIGEN3
#include "ompl/base/samplers/informed/PathLengthDirectInfSampler.h"
#else
#include "ompl/base/samplers/informed/RejectionInfSampler.h"
#endif

using namespace std;
namespace ob = ompl::base;

ompl::base::AngleDiffOptimizationObjective::AngleDiffOptimizationObjective(const SpaceInformationPtr &si)
  : ompl::base::PathLengthOptimizationObjective(si)
{
    description_ = "Angle Difference";

    // Setup a default cost-to-go heuristics:
    setCostToGoHeuristic(base::goalRegionCostToGo);
}

ompl::base::Cost ompl::base::AngleDiffOptimizationObjective::motionCost(const State *s1, const State *s2) const
{
    Utils * utils = new Utils();
    double res = utils->GetTotalAngle(s1, s2);
    /*
    const auto *pos1 = s1->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
    const auto *angle1 = s1->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
    const auto *pos2 = s2->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
    const auto *angle2 = s2->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
    printf("point1: (%.2f,%.2f,%.2f), angle: %.2f\n", pos1->values[0],pos1->values[1],pos1->values[2], angle1->values[0]);
    printf("point2: (%.2f,%.2f,%.2f), angle: %.2f\n", pos2->values[0],pos2->values[1],pos2->values[2], angle2->values[0]);

    printf("cost = %.2f\n",res);
    */
    delete utils;
    return ompl::base::Cost(res);
}

ompl::base::Cost ompl::base::AngleDiffOptimizationObjective::motionCostHeuristic(const State *s1,
                                                                                  const State *s2) const
{
    return motionCost(s1, s2);
}



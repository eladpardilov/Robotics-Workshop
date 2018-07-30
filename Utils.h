
#ifndef UTILS_H_
#define UTILS_H_

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/MotionValidator.h>
#include <ompl/config.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

/* Utility functions class */
class Utils
{
public:
	/* 
	 *	Get the total angle of the movement between 2 states -
	 *		sum of the rotate angle required to align with the movement direction (from the starting point),
	 *		and the angle required to align with the end point direction (from the movement direction)
	 */
    double GetTotalAngle(const ob::State *s1, const ob::State *s2);
    double GetTotalAngle(double x1, double y1, double theta1, double x2, double y2, double theta2);
	/* Get the euclidian distance between 2 states, according to 3D space */
    double GetEuclidDistance(const ob::State *s1, const ob::State *s2);
    double GetEuclidDistance(double x1, double y1, double z1, double x2, double y2, double z2);
};

#endif
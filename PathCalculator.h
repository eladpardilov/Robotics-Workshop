
/*
 * PathCalculator.h
 *
 *  Created on: May 9, 2018
 *      Author: Dana
 */

#ifndef PATHCALCULATOR_H_
#define PATHCALCULATOR_H_

#include <cstdio>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include "SimpleBatchPRM.h"
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/MotionValidator.h>
#include <ompl/config.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs_c.h>
#include <opencv2/imgproc.hpp>

namespace ob = ompl::base;
namespace og = ompl::geometric;

extern cv::Mat global_mat;
extern double mat_min, mat_max;

class PathCalculator
{
private:
	int* coordinates = NULL;
	int radius;
	int num_states;
	int output_index;
	
	/* 
	 * A class implementing ompl's Motion Validator,
	 * determining if the motion from one state to another is possible,
	 * taking into account the theoretical constraints
	 */
	class myMotionValidator : public ob::MotionValidator
	{
	public:
		/* Constructor */
		myMotionValidator(const ob::SpaceInformationPtr &si);
		/* 
		 * Variations of functions getting the states s1 & s2,
		 * and checking the movement s1 -> s2.
		 * returns true if the movement is possible, false otherwise
		 */
		bool checkMotion(const ob::State *s1, const ob::State *s2) const;
		bool checkMotion(const ob::State *s1, const ob::State *s2,  std::pair<ob::State *, double> &lastValid) const;
	private:
		/* 
		 * A function getting a pair of coordinates and check if the
		 * movement is possible from pos1 to pos2, movement will be possible if
		 * - the path is collision free
		 * - the distance is no greater than METERS_PER_PIXEL * MAX_DISTANCE_FOR_STEP
		 * returns true if the movement is possible from this perspective, false otherwise
		 */
		bool CheckLineBetweenPoints(int* pos1, int* pos2) const;
		/* 
		 * A function getting a pair of coordinates and check if the
		 * movement is possible from pos1 to pos2, movement will be possible if one of the terms is met:
		 * - the turn rate this movement requires < the maximal turn rate the user supplied
		 * - the target state (s2) is the objective, in which case the angle doesn't matter
		 * returns true if the movement is possible from this perspective, false otherwise
		 */
		bool CheckAngleBetweenPoints(double dist, const ob::State *s1, const ob::State *s2) const;
		/* 
		 * A function getting array of pixel coordinates (representing a straight line)
		 * and the length of the array, and checking if any of the pixels along the line collides with the terrain.
		 * returns true if there are no collisions (line is valid), false otherwise
		 */
		bool IsLineValid(int** line, int len) const;
	};

	/* 
	 * A class implementing ompl's Valid State Sampler,
	 * This is a problem-specific sampler that automatically generates valid states,
	 * taking into account the theoretical and terrain constraints
	 */
	class MyValidStateSampler : public ob::ValidStateSampler
	{
	public:
		/* Constructor */
		MyValidStateSampler(const ob::SpaceInformation *si);
		/* 
		 * Generates a sample in the valid part of the R^3 state space, and an angle for the state
		 * Valid states satisfy the following constraints:
		 * - max(global_map[x,y],funnel_map[x,y]) < z < mat_max
		 * - 0 < x,y < MAP_SIZE
		 * - 0 < angle < 360
		 */
		bool sample(ob::State *state) override;
		/* Not supported - dummy implementation */
		bool sampleNear(ob::State* /*state*/, const ob::State* /*near*/, const double /*distance*/) override;
	protected:
		ompl::RNG rng_;
	};

	/* New valid state sampler allocator */
	static ob::ValidStateSamplerPtr allocMyValidStateSampler(const ob::SpaceInformation *si);
	/* State validity checker for the model, this function is an input for setStateValidityChecker */
	static bool isStateValid(const ob::State *state);
	/* Creates a 3D funnel around the goal state, in order to make the sampling more efficient.
	 * the area below the funnel won't be valid for sampling
	 * as we expect the helicopter to ascend only when close to the target
	 */
	void calcFunnel(int* goal);

public:
	/* Constructor */
	PathCalculator (cv::Mat mat, int* coordinates, int max_turn_rate, double max_up_down_rate, int radius, int num_states);
	/* destructor   */
	~PathCalculator();
	/* Plan the rescue according to the given data */
	void PlanRoute();
	/* Dimensions of the map */
	int rows;
	int cols;
};

#endif /* PATHCALCULATOR_H_ */

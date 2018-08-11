
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
	cv::Mat mat;
	int* coordinates = NULL;
	double max_turn_rate;
	double max_up_down_rate;
	int radius;
	int num_states;
	int output_index;
	ob::PathPtr thePath;
	
	class myMotionValidator : public ob::MotionValidator
	{
	public:
		myMotionValidator(const ob::SpaceInformationPtr &si);
		bool checkMotion(const ob::State *s1, const ob::State *s2) const;
		bool checkMotion(const ob::State *s1, const ob::State *s2,  std::pair<ob::State *, double> &lastValid) const;
		bool IsLineValid(int** line, int len) const;
		bool CheckLineBetweenPoints(int* pos1, int* pos2) const;
		bool CheckAngleBetweenPoints(double dist, const ob::State *s1, const ob::State *s2) const;
		double FindAngle(double theta, double phi) const;

	};
	static ob::ValidStateSamplerPtr allocMyValidStateSampler(const ob::SpaceInformation *si);
	static bool isStateValid(const ob::State *state);
	void calcFunnel(int* goal);

public:
	PathCalculator (cv::Mat mat, int* coordinates, int max_turn_rate, double max_up_down_rate, int radius, int num_states);
	void PlanRoute();
	ob::PathPtr getPath() {return thePath;}
	void finish();
	
	int rows;
	int cols;

};

#endif /* PATHCALCULATOR_H_ */

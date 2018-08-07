
#include "Defs.h"
#include "PathCalculator.h"

#include <cstdio>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/MotionValidator.h>
#include <ompl/config.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs_c.h>
#include <opencv2/imgproc.hpp>

#ifndef POSTPROCESSOR_H_
#define POSTPROCESSOR_H_

class PostProcessor
{
private:
	int* coordinates = NULL;
	int rows;
	int cols;
	int output_index;
	int FindPathByPercentage(double euc_percent, double angle_percent, bool is_lowest);

	
public:
	PostProcessor(int* coordinates);
	void Show();
	void ShowOnePath(int path_index);
	int FindBestPath();
	int FindSafestPath();
	int FindFastestPath();
	int FindLowestPath();
};

#endif /* POSTPROCESSOR_H_ */
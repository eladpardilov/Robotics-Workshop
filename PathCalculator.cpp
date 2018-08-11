#include <cstdio>
#include "PathCalculator.h"
#include "Defs.h"
#include <stdio.h>
#include <fstream>
#include <string>
#include <stdlib.h>
#include "Utils.h"

using namespace std;

//float** global_map;
cv::Mat global_mat;
cv::Mat funnel_mat;
double mat_min, mat_max, up_down_rate;
int turn_rate;

PathCalculator::PathCalculator(cv::Mat mat, int* coordinates, int max_turn_rate, double max_up_down_rate, int radius, int num_states)
{
	this->mat = mat;
	global_mat = mat;
	cv::minMaxLoc(global_mat, &mat_min, &mat_max);
	this->coordinates = coordinates;
	this->max_turn_rate = max_turn_rate;
	this->max_up_down_rate = max_up_down_rate;
	this->radius = radius;
	this->num_states = num_states;
	cv::Size size = global_mat.size();
	turn_rate = max_turn_rate;
	up_down_rate = max_up_down_rate;
	rows = size.height;
	cols = size.width;
	this->output_index = 0;	
}

PathCalculator::myMotionValidator::myMotionValidator(const ob::SpaceInformationPtr &si) : ob::MotionValidator(si) {

}


double PathCalculator::myMotionValidator::FindAngle(double theta, double phi) const
{
	double abs_val = std::abs(theta - phi);
	if (abs_val < 180)
	{
		return abs_val;
	}
	else
	{
		return (360 - abs_val);
	}
}

bool PathCalculator::myMotionValidator::CheckAngleBetweenPoints(double dist, const ob::State *s1, const ob::State *s2) const
{
	double angle;
	double line_time;
	bool res;
	const auto *angle2 = s2->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
	// if s2 is the destination, we are close enough and the angle doesn't matter
	
	if (angle2->values[0] < 0) {
		/*
		const auto *pos1 = s1->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
		const auto *angle1 = s1->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
		printf("point: (%.2f,%.2f,%.2f), angle: %.2f\n", pos1->values[0],pos1->values[1],pos1->values[2], angle1->values[0]);
		*/
		return true;
	}
	
    Utils * utils = new Utils();
    angle = utils->GetTotalAngle(s1, s2);
    delete utils;

	line_time = dist / CONSTANT_VELOCITY;

	if ((angle / line_time) > double(turn_rate))
		res = false;
	else {
		res = true;
		/*
		double phi;
			const auto *pos1 = s1->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
		    const auto *angle1 = s1->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
		    const auto *pos2 = s2->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
	    	phi = atan2(pos2->values[1] - pos1->values[1], pos2->values[0] - pos1->values[0]); // getting the angle in PI units
			phi = (( phi >= 0 ? phi : ( 2*PI + phi ) ) * 360 / ( 2*PI ));
			printf("PHI: %.2f\n",phi);
		    printf("point1: (%.2f,%.2f,%.2f), angle: %.2f\n", pos1->values[0],pos1->values[1],pos1->values[2], angle1->values[0]);
		    printf("point2: (%.2f,%.2f,%.2f), angle: %.2f\n", pos2->values[0],pos2->values[1],pos2->values[2], angle2->values[0]);

		printf("(angle / line_time) = %.2f/%.2f = %.2f <= %.2f  = turn_rate\n", angle, line_time, angle/line_time, (double)turn_rate); */
	}

	return res;
}


bool PathCalculator::myMotionValidator::CheckLineBetweenPoints(int* pos1, int* pos2) const
{
	int start_x, start_y, end_x, end_y, start_z, end_z;
	int ** result = NULL;
	int line_len = 0;
	int i, x, y, z;
	int mx, my, mz;
	double vertical_velocity, motion_time, horiz_dist, vertical_dist, dist;
	bool res;

	if (pos1[0] < pos2[0])
	{
		start_x = pos1[0];
		start_y = pos1[1];
		start_z = pos1[2];
		end_x = pos2[0];
		end_y = pos2[1];
		end_z = pos2[2];
	}  else {
		start_x = pos2[0];
		start_y = pos2[1];
		start_z = pos2[2];
		end_x = pos1[0];
		end_y = pos1[1];
		end_z = pos2[2];
	}

	mx = end_x - start_x;
	my = end_y - start_y;
	mz = end_z - start_z;

	// check up/down rate
	horiz_dist = sqrt(mx*mx + my*my);
	dist = sqrt(mx*mx + my*my + mz*mz);
	vertical_dist = abs(end_z - start_z);
	motion_time = dist / CONSTANT_VELOCITY; // in seconds
	vertical_velocity = vertical_dist / motion_time; // pixels per second, in Z axis
	if (vertical_velocity > up_down_rate || (horiz_dist * HORIZ_VERTICAL_MAX_RATE) < vertical_dist)
		return false;

	// Special cases
	if (start_x == end_x && start_y == end_y && start_z == end_z)
	{
		// NULL
		return true;
		
		//return result;
	}

/*
	if (start_z < end_z)
		end_z = start_z;
	else
		start_z = end_z;
	mz = 0;
*/
	// Only one axis changes
	if (start_x == end_x && start_y == end_y)
	{
		line_len = abs(end_z - start_z - 1); // Doens't include the start and end points
		result = new int*[line_len];
		for (z = 0 ; z < line_len ; z++)
		{
			result[z] = new int[3];
			result[z][0] = start_x;
			result[z][1] = start_y;
			result[z][2] = start_z + 1 + z;
		}

		goto CHECK_VALIDITY;
	}

	if (start_x == end_x && start_z == end_z)
	{
		line_len = abs(end_y - start_y - 1); // Doens't include the start and end points
		result = new int*[line_len];
		for (y = 0 ; y < line_len ; y++)
		{
			result[y] = new int[3];
			result[y][0] = start_x;
			result[y][1] = start_y + 1 + y;
			result[y][2] = start_z;
		}

		goto CHECK_VALIDITY;
	}

	if (start_y == end_y && start_z == end_z)
	{
		line_len = abs(end_x - start_x - 1); // Doens't include the start and end points
		result = new int*[line_len];
		for (x = 0 ; x < line_len ; x++)
		{
			result[x] = new int[3];
			result[x][0] = start_x + 1 + x;
			result[x][1] = start_y;
			result[x][2] = start_z;
		}

		goto CHECK_VALIDITY;
	}

	// z,y must change
	if (start_x == end_x)
	{
		int run = end_y - start_y;
		int rise = end_z - start_z;
		int m = ((float) rise) / ((float) run);
		float b = start_y - (m * start_z);
		float z;
		
		line_len = abs(end_y - start_y - 1); // Doesn't include the start and end point
		result = new int*[line_len];
		for (y = start_y + 1; y <= end_y; y++)
		{
			result[y - start_y] = new int[2];
			result[y - start_y][0] = start_x;
			result[y - start_y][1] = y;
			z = (m * y) + b; // Find z
			z = int(round(z)); // round to nearest int
			result[y - start_y][2] = z;
		}

		goto CHECK_VALIDITY;
	}

	// Now, even if y or z don't change, we don't care. It will still work
	line_len = abs(end_x - start_x - 1); // Doens't include the start and end points
	result = new int*[line_len];

	for (x = 0 ; x < line_len; x++ )
	{
		result[x] = new int[3];
		result[x][0] = start_x + 1 + x;
		i = (x + 1) / mx;
		result[x][1] = int(round(start_y + i * my));
		result[x][2] = int(round(start_z + i * mz));
	}

	goto CHECK_VALIDITY;

	CHECK_VALIDITY:
	res = IsLineValid(result, line_len);

	for(i = 0 ; i < line_len ; i++)
		delete[] result[i];
	delete[] result;

	return res;

}


bool PathCalculator::myMotionValidator::IsLineValid(int** line, int len) const
{
	//int line_len = len(line);
	if (line == NULL)
		return true;

	for (int i=0; i < len; i++)
	{

		if (global_mat.at<float>(int(round(line[i][1])),int(round(line[i][0]))) > line[i][2])
		{
			//printf("x = %d, y = %d, z = %d z_map = %f\n", int(round(line[i][0])), int(round(line[i][1])), line[i][2], global_mat.at<float>(int(round(line[i][1])),int(round(line[i][0]))));
			return false;
		}		
	}
	return true;
}


bool PathCalculator::myMotionValidator::checkMotion(const ob::State *s1, const ob::State *s2) const
{
	const auto *pos1 = s1->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
	const auto *pos2 = s2->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
	bool res;
	bool isLine;
	bool isAngle;
	double dist;
	int* pos1_arr = new int[3];
	int* pos2_arr = new int[3];
    Utils * utils = new Utils();
    dist = utils->GetEuclidDistance(s1, s2);
    delete utils;
	
	if (dist > MAX_DISTANCE_FOR_STEP)
	{
		return false;
	}

	pos1_arr[0] = int(round(pos1->values[0]));
	pos1_arr[1] = int(round(pos1->values[1]));
	pos1_arr[2] = int(round(pos1->values[2]));

	pos2_arr[0] = int(round(pos2->values[0]));
	pos2_arr[1] = int(round(pos2->values[1]));
	pos2_arr[2] = int(round(pos2->values[2]));

	//int** checkPoints = LineBetweenPoints(pos1_arr, pos2_arr);
	//if (checkPoints == NULL)
	//	return true;

	//int onePointSize = sizeof(checkPoints[0]) / sizeof(checkPoints[0][0]);
	//int numPoints = sizeof(checkPoints) / onePointSize;

	//int numPoints = sizeof(checkPoints)/sizeof(checkPoints[0]);
	isLine = CheckLineBetweenPoints(pos1_arr, pos2_arr);
	isAngle = CheckAngleBetweenPoints(dist, s1, s2);

	res = (isLine && isAngle);
/*
	if (res) {
		if (!(x == 0 && y == 0 && z == 0)) {
			printf("Point1: %f,%f,%f, Angle:%f\nPoint2: %f,%f,%f, Angle:%f\n",
			pos1->values[0],pos1->values[1],pos1->values[2],angle1->values[0],
			pos2->values[0],pos2->values[1],pos2->values[2],angle2->values[0]);
			printf("Exists\n");
		}
	}
	else
		printf("Doesn't Exist\n"); */

	delete[] pos1_arr;
	delete[] pos2_arr;

	return (res);

//	global_mat.at<int>(x_int, y_int)

	//return (dist < MAX_DISTANCE_FOR_STEP);
}

bool PathCalculator::myMotionValidator::checkMotion(const ob::State *s1, const ob::State *s2,  __attribute__((unused)) std::pair<ob::State *, double> &lastValid) const
{
	return checkMotion(s1, s2);
}


// This is a problem-specific sampler that automatically generates valid
// states; it doesn't need to call SpaceInformation::isValid. This is an
// example of constrained sampling. If you can explicitly describe the set valid
// states and can draw samples from it, then this is typically much more
// efficient than generating random samples from the entire state space and
// checking for validity.
class MyValidStateSampler : public ob::ValidStateSampler
{
public:
	MyValidStateSampler(const ob::SpaceInformation *si) : ValidStateSampler(si)
	{
		name_ = "my sampler";
	}
	// Generate a sample in the valid part of the R^3 state space
	// Valid states satisfy the following constraints: z > global_map[x,y]
	bool sample(ob::State *state) override
	{
		const auto *pos = state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);
		const auto *angle = state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
		//double* val = static_cast<ob::RealVectorStateSpace::StateType*>(state)->values;

		double x = rng_.uniformReal(0,MAP_SIZE);
		double y = rng_.uniformReal(0,MAP_SIZE);
		int x_int = (int)x;
		int y_int = (int)y;
	//printf("point at map: (%.2f,%.2f,%.2f), ", x,y,global_mat.at<float>(x_int, y_int));
		if (global_mat.at<float>(y_int, x_int) > mat_max)
			return false;
		double z_min = max(global_mat.at<float>(y_int, x_int), funnel_mat.at<float>(y_int, x_int));
		double z = rng_.uniformReal(z_min, mat_max);

		pos->values[0] = x;
		pos->values[1] = y;
		pos->values[2] = z;
		angle->values[0] = rng_.uniformReal(0, 360);
		if (si_->isValid(state)) {
			//printf("point taken: (%.2f,%.2f,%.2f)", x,y,z);
			//printf(" valid state\n");
			return true;
		}
		//printf("\n");

		return false;
	}
	// We don't need this in the example below.
	bool sampleNear(ob::State* /*state*/, const ob::State* /*near*/, const double /*distance*/) override
	{
		throw ompl::Exception("MyValidStateSampler::sampleNear", "not implemented");
		return false;
	}
protected:
	ompl::RNG rng_;
};


// return an instance of my sampler
ob::ValidStateSamplerPtr PathCalculator::allocMyValidStateSampler(const ob::SpaceInformation *si)
{
	return std::make_shared<MyValidStateSampler>(si);
}


// this function is needed, even when we can write a sampler like the one
// above, because we need to check path segments for validity
bool PathCalculator::isStateValid(const ob::State *state)
{
	const auto *pos = state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0);

	//const ob::RealVectorStateSpace::StateType& pos = *state->as<ob::RealVectorStateSpace::StateType>();
	if ((pos->values[0]<MAP_SIZE && pos->values[0]>0) && (pos->values[1]<MAP_SIZE && pos->values[1]>0))
		return pos->values[2] < mat_max + 1;
	else
		return false;
}

// this function is needed, even when we can write a sampler like the one
// above, because we need to check path segments for validity
void PathCalculator::calcFunnel(int* goal)
{
	float r, h;
	// Diff between z0 and max height in the map
	float cone_h = float(mat_max - global_mat.at<float>(goal[1], goal[0]));
	//float cone_h = mat_max;
	// Start radius
	float cone_r = float(this->radius);
	// Calc Tangent Theta of the cone
	float tanTheta = cone_r / cone_h;
	// initialize funnel matrix
	funnel_mat = cv::Mat(global_mat.size(), CV_32F);

	for (int y = 0; y < this->rows; y++)
	{
		for (int x = 0; x < this->cols; x++)
		{
			r = sqrt(pow((x - goal[0]),2) + pow((y - goal[1]),2));
			if (r < cone_r)
			{
				// Find cone height
				h = r / tanTheta;
				// Add destination heigth (offset)
				h = h + global_mat.at<float>(goal[1], goal[0]);
				funnel_mat.at<float>(y, x) = h;
			}
			else
			{
				funnel_mat.at<float>(y, x) = mat_max;
			}
		}
	}
}

void PathCalculator::PlanRoute()
{
	int * goal_arr = new int[2];
	char file_name[11];

	// construct the state space we are planning in
	//auto space(std::make_shared<ob::RealVectorStateSpace>(4));
	ob::CompoundStateSpace *cs = new ompl::base::CompoundStateSpace();
	cs->addSubspace(ompl::base::StateSpacePtr(std::make_shared<ob::RealVectorStateSpace>(3)), 1.0);
	cs->addSubspace(ompl::base::StateSpacePtr(std::make_shared<ob::RealVectorStateSpace>(1)), 0.0);
	ob::StateSpacePtr space(cs);
	// auto space(std::make_shared<ob::SE3StateSpace>());

	// set the bounds for the R^3 part of SE(3)
	ob::RealVectorBounds bounds(3);
	ob::RealVectorBounds angle_bound(1);
	bounds.setLow(0);
	bounds.setHigh(MAP_SIZE);
	bounds.setLow(2, mat_min);
	bounds.setHigh(2, mat_max);
	angle_bound.setLow(-1);
	angle_bound.setHigh(360);
	cs->getSubspace(0)->as<ob::RealVectorStateSpace>()->setBounds(bounds);
	cs->getSubspace(1)->as<ob::RealVectorStateSpace>()->setBounds(angle_bound);

	// construct an instance of  space information from this state space
	auto si(std::make_shared<ob::SpaceInformation>(space));
	// set state validity checking for this space
	si->setStateValidityChecker(isStateValid);
	//si->setStateValidityCheckingResolution(0.03);
	si->setValidStateSamplerAllocator(allocMyValidStateSampler);
	si->setMotionValidator(std::make_shared<myMotionValidator>(si));
	si->setup();

	// create a planner for the defined space
	auto planner(std::make_shared<og::SimpleBatchPRM>(si, true));
	planner->setNumMilestones(this->num_states);
	ob::ScopedState<ob::SE3StateSpace> goal(space);
	goal[0] = coordinates[0];
	goal[1] = coordinates[1];
	goal[2] = global_mat.at<float>(goal[1],goal[0]);
	// we will use the -1 to identify we are at the destination and angle doesn't matter
	goal[3] = -1;
	printf("point goal: (%.2f,%.2f,%.2f)\n", goal[0],goal[1],goal[2]);
	// goal->rotation().setIdentity();
	//planner->setConnectionStrategyType("K");
	planner->setup();

	// Calc the funnel area of the points generator
	goal_arr[0] = int(goal[0]);
	goal_arr[1] = int(goal[1]);
	calcFunnel(goal_arr);
	int cnt = 0;
	for (double angle = 0; angle < 2*PI - EPSILON; angle += 2*PI/NUM_POINTS_AROUND_CENTER) {
	//for (double angle=2*PI*5/8; angle<=2*PI; angle+=2*PI) {
		ob::ScopedState<ob::RealVectorStateSpace> start(space);
		start[0] = MAP_SIZE/2 + radius * cos(angle);
		start[1] = MAP_SIZE/2 + radius * sin(angle);
		start[3] = (int)round((360 * angle / (2 * PI) + 180)) % 360;
	//	start[0] = 0.01;
	//	start[1] = 20;
		start[2] = mat_max;
		printf("point start: (%.2f,%.2f,%.2f), angle: %.2f\n", start[0],start[1],start[2], start[3]);

		auto pdef(std::make_shared<ob::ProblemDefinition>(si));
		// set the start and goal states
		pdef->setStartAndGoalStates(start, goal);
		pdef->setOptimizationObjective(planner->getBalancedObjective(si));
		// set the problem we are trying to solve for the planner
		planner->setProblemDefinition(pdef);
		// perform setup steps for the planner

		// print the settings for this space
		// si->printSettings(std::cout);
		// print the problem settings
		// pdef->print(std::cout);

		// attempt to solve the problem within ten seconds of planning time
		if (cnt == 0)
			planner->miniSetup();

		ob::PlannerStatus solved = planner->ob::Planner::solve(SOLVING_TIME);
		if (solved)
		{
			// get the goal representation from the problem definition (not the same as the goal state)
			// and inquire about the found path
			thePath = pdef->getSolutionPath();
			std::cout << "Found solution:" << std::endl;

			// print the path to screen
			std::cout << "path - begin" << std::endl;
			thePath->print(std::cout);

			std::ofstream fout;

			sprintf(file_name, "dots_%d.txt", this->output_index);
			this->output_index++;
			fout.open(file_name, std::ios::out | std::ios::trunc);
			//fout.open(DOTS_FILE_NAME, std::ios::out | std::ios::trunc);
			if(!fout)
				std::cout << "Error opening file" << std::endl;
			thePath->print(fout);
			fout.close();
			std::cout << "path - end" << std::endl;
		}
		else
			std::cout << "No solution found" << std::endl;
		cnt++;
	
	}
	// start->rotation().setIdentity();
	delete[] goal_arr;
}


void PathCalculator::finish()
{
	funnel_mat.release();
}
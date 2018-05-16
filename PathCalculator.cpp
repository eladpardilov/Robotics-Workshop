#include <cstdio>
#include "PathCalculator.h"
#include "Defs.h"
#include <stdio.h>
#include <fstream>
#include <string>

using namespace std;

float** global_map;
cv::Mat global_mat;
double mat_min, mat_max;

PathCalculator::PathCalculator(cv::Mat mat, int* coordinates, int radius, int velocity)
{
	this->mat = mat;
	global_mat = mat;
	cv::minMaxLoc(global_mat, &mat_min, &mat_max);
	this->coordinates = coordinates;
	this->radius = radius;
	this->velocity = velocity;
	cv::Size size = global_mat.size();
	rows = size.height;
	cols = size.width;
	
}

PathCalculator::myMotionValidator::myMotionValidator(const ob::SpaceInformationPtr &si) : ob::MotionValidator(si) {

}

bool PathCalculator::myMotionValidator::checkMotion(const ob::State *s1, const ob::State *s2) const
{
	const ob::RealVectorStateSpace::StateType& pos1 =
			*s1->as<ob::RealVectorStateSpace::StateType>();
	const ob::RealVectorStateSpace::StateType& pos2 =
			*s2->as<ob::RealVectorStateSpace::StateType>();

	double x = pos1[0] - pos2[0];
	double y = pos1[1] - pos2[1];
	double z = pos1[2] - pos2[2];
	double dist;

	dist = x*x + y*y + z*z;
	dist = sqrt(dist);
	return (dist < MAX_DISTANCE_FOR_STEP);
		//check line-free
		//return true/false;
}

bool PathCalculator::myMotionValidator::checkMotion(const ob::State *s1, const ob::State *s2,  std::pair<ob::State *, double> &lastValid) const
{
	const ob::RealVectorStateSpace::StateType& pos1 =
			*s1->as<ob::RealVectorStateSpace::StateType>();
	const ob::RealVectorStateSpace::StateType& pos2 =
			*s2->as<ob::RealVectorStateSpace::StateType>();

	double x = pos1[0] - pos2[0];
	double y = pos1[1] - pos2[1];
	double z = pos1[2] - pos2[2];
	double dist;

	dist = x*x + y*y + z*z;
	dist = sqrt(dist);
	return (dist < MAX_DISTANCE_FOR_STEP);
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
        double* val = static_cast<ob::RealVectorStateSpace::StateType*>(state)->values;
        double x = rng_.uniformReal(0,MAP_SIZE);
        double y = rng_.uniformReal(0,MAP_SIZE);
        int x_int = (int)x;
        int y_int = (int)y;
	//printf("point at map: (%.2f,%.2f,%.2f), ", x,y,global_mat.at<float>(x_int, y_int));
        if (global_mat.at<float>(y_int, x_int) > mat_max)
        	return false;
        double z = rng_.uniformReal(global_mat.at<float>(y_int, x_int), mat_max);

        val[0] = x;
        val[1] = y;
        val[2] = z;
        if (si_->isValid(state)) {
//        	printf("point taken: (%.2f,%.2f,%.2f)", x,y,z);
//        	printf(" valid state\n");
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
    const ob::RealVectorStateSpace::StateType& pos = *state->as<ob::RealVectorStateSpace::StateType>();
    if ((pos[0]<MAP_SIZE && pos[0]>0) && (pos[1]<MAP_SIZE && pos[1]>0)) {
	    return pos[2] < mat_max;
    }
    else
	    return false;
}

void PathCalculator::PlanRoute()
{
    // construct the state space we are planning in
    auto space(std::make_shared<ob::RealVectorStateSpace>(3));
    // auto space(std::make_shared<ob::SE3StateSpace>());

    // set the bounds for the R^3 part of SE(3)
    ob::RealVectorBounds bounds(3);
    bounds.setLow(0);
    bounds.setHigh(MAP_SIZE);
    bounds.setLow(2, mat_min);
    bounds.setHigh(2, mat_max);
    space->setBounds(bounds);

    // construct an instance of  space information from this state space
    auto si(std::make_shared<ob::SpaceInformation>(space));

    // set state validity checking for this space
    si->setStateValidityChecker(isStateValid);
    si->setStateValidityCheckingResolution(0.03);
    si->setValidStateSamplerAllocator(allocMyValidStateSampler);
    si->setMotionValidator(std::make_shared<myMotionValidator>(si));

    // create the start state at [1 1 1]
    ob::ScopedState<ob::SE3StateSpace> start(space);
    start[0] = 60;
    start[1] = 1;
    start[2] = global_mat.at<float>(start[1], start[0]);
    printf("point start: (%.2f,%.2f,%.2f)\n", start[0],start[1],start[2]);
    // start->rotation().setIdentity();

    // create the goal state at [99 99 1]
    ob::ScopedState<ob::SE3StateSpace> goal(space);

    goal[0] = 300;
    goal[1] = 255;
    goal[2] = global_mat.at<float>(goal[1],goal[0]);
    printf("point goal: (%.2f,%.2f,%.2f)\n", goal[0],goal[1],goal[2]);
    // goal->rotation().setIdentity();

    auto pdef(std::make_shared<ob::ProblemDefinition>(si));
    // set the start and goal states
    pdef->setStartAndGoalStates(start, goal);
    // create a planner for the defined space
    auto planner(std::make_shared<og::PRM>(si, true));
    // set the problem we are trying to solve for the planner
    planner->setProblemDefinition(pdef);
    // perform setup steps for the planner
    planner->setup();

    // print the settings for this space
    // si->printSettings(std::cout);
    // print the problem settings
    // pdef->print(std::cout);

    // attempt to solve the problem within ten seconds of planning time

    ob::PlannerStatus solved = planner->ob::Planner::solve(300.0);

    if (solved)
    {
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
        thePath = pdef->getSolutionPath();
        std::cout << "Found solution:" << std::endl;

        // print the path to screen
        std::cout << "path- begin" << std::endl;
        thePath->print(std::cout);
        
        std::ofstream fout;
        fout.open(DOTS_FILE_NAME, std::ios::out | std::ios::trunc);
        if(!fout)
			std::cout << "Error opening file" << std::endl;
        thePath->print(fout);
        fout.close();
        std::cout << "path- end" << std::endl;
    }
    else
        std::cout << "No solution found" << std::endl;
}


void PathCalculator::Show()
{
	float x1, y1, z1, x2,y2,z2;
    int numOfPoints;
    float colorFactor;
    
    //reading from the "path->print()" file
    FILE * readFile;
    readFile = fopen(DOTS_FILE_NAME, "r");
	fscanf(readFile, "Geometric path with %d states\n", &numOfPoints);
   
    //creating a Mat to display
	//cv::Mat image;
   
/*
   try{
	image = global_mat;
}catch(exception& e){
	cout << e.what() << endl;
	return;}
	if(! image.data ) 
	{
      std::cout <<  "Could not open or find the image" << std::endl ;
      return;
    }*/
    
    cout << "rows,cols: " << rows << " " << cols << endl;
    
   /* for(int i=0; i<rows; i++)
    {
		for(int j=0; j<cols; j++)
		{			
			global_mat.at<int>(i,j) = (int)((255.0*(global_mat.at<float>(i,j) - mat_min)) / (float)(mat_max - mat_min));
			
			//if(global_mat.at<float>(i,j) > 255)
			//	cout << "hi " << global_mat.at<int>(i,j) << endl;
			
		}
		
	}*/
 
	cout << "min,max: " << mat_min << " " << mat_max << endl;
	cout << "type: " << global_mat.type() << endl;
	
	cv::namedWindow( "Our Plane Path", cv::WINDOW_AUTOSIZE );
	
	//for(auto state : states_)
	//	cout << state->components[0] << endl;
	
	cv::Mat normalized_img;
	
	global_mat.convertTo(normalized_img, CV_8U, 255.0f/(mat_max - mat_min), (-mat_min * 255.0f)/(mat_max - mat_min));
	
	fscanf(readFile,  "RealVectorState [%f %f %f]\n", &x1, &y1, &z1);
	fscanf(readFile,  "RealVectorState [%f %f %f]\n", &x2, &y2, &z2);
	cv::line(normalized_img, cv::Point(x1,y1), cv::Point(x2,y2), cv::Scalar(255,255,0), 1);
	//circle(image, Point(x1,y1), 3, Scalar(255,0,0), FILLED);
	for(int i=3; i<=numOfPoints; i++)
	{
		//Old last-point is now first-point
		x1=x2;
		y1=y2;
		z1=z2;
			
		fscanf(readFile,  "RealVectorState [%f %f %f]\n", &x2, &y2, &z2);
			
		cv::line(normalized_img, cv::Point(x1,y1), cv::Point(x2,y2), cv::Scalar(255,255,0), 1);
		//cv::circle(image, cv::Point(x2,y2), 3, cv::Scalar(255,0,0), cv::FILLED);
	}
	
	cv::circle(normalized_img, cv::Point(60,1), 3, cv::Scalar(255,0,0), cv::FILLED);
	cv::circle(normalized_img, cv::Point(300,255), 3, cv::Scalar(255,0,0), cv::FILLED);


	fclose(readFile);
	
	cv::imshow( "Our Plane Path", normalized_img);
	
	cv::waitKey(0);
	
}

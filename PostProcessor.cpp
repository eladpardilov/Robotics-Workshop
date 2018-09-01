#include <cstdio>

#include <stdio.h>
#include <fstream>
#include <string>
#include <stdlib.h>
#include <sstream>

#include "PostProcessor.h"
#include "Utils.h"

using namespace std;

PostProcessor::PostProcessor(int* coordinates)
{
	this->coordinates = coordinates;
	this->output_index = 0;
	
}

int PostProcessor::FindBestPath()
{
	return FindPathByPercentage(0.7, 0.3);
}

int PostProcessor::FindFastestPath()
{
	return FindPathByPercentage(0.9, 0.1);
}

int PostProcessor::FindSafestPath()
{
	return FindPathByPercentage(0.1, 0.9);
}

int PostProcessor::FindPathByPercentage(double euc_percent, double angle_percent)
{
	Utils utils_obj;
	FILE * readFile;
	char file_name[11];
	float x1, y1, z1, t1, x2, y2, z2, t2;
	int numOfPoints;
	int best_index = 0;
	double best_cost;
	double path_cost = 0;
	double line_cost = 0;


	for (int path_index=0; path_index < NUM_POINTS_AROUND_CENTER; path_index++)
	//for (; this->output_index < NUM_POINTS_AROUND_CENTER; this->output_index++)
	{
		path_cost = 0;
		sprintf(file_name, "dots_%d.txt", path_index);
		readFile = fopen(file_name, "r");
		fscanf(readFile, "Geometric path with %d states\n", &numOfPoints);
	

		// Read the first 2 points
		fscanf(readFile, "Compound state [\n");
		fscanf(readFile,  "RealVectorState [%f %f %f]\n", &x1, &y1, &z1);
		fscanf(readFile,  "RealVectorState [%f]\n", &t1);
		fscanf(readFile, "]\n");
		fscanf(readFile, "Compound state [\n");
		fscanf(readFile,  "RealVectorState [%f %f %f]\n", &x2, &y2, &z2);
		fscanf(readFile,  "RealVectorState [%f]\n", &t2);
		fscanf(readFile, "]\n");

		line_cost = (utils_obj.GetEuclidDistance(x1, y1, z1, x2, y2, z2)/10) * euc_percent + (utils_obj.GetTotalAngle(x1, y1, t1, x2, y2, t2)/360) * angle_percent;
		path_cost = path_cost + line_cost; 


		//printf(" Path num %d, num points %d\n", path_index, numOfPoints);
		// Go over the rest of the path

		for(int point_index = 3; point_index <= numOfPoints; point_index ++)
		{
			//Old last-point is now first-point
			x1=x2;
			y1=y2;
			z1=z2;
			t1=t2;
				
			fscanf(readFile, "Compound state [\n");
			fscanf(readFile,  "RealVectorState [%f %f %f]\n", &x2, &y2, &z2);
			fscanf(readFile,  "RealVectorState [%f]\n", &t2);
			fscanf(readFile, "]\n");

			line_cost = (utils_obj.GetEuclidDistance(x1, y1, z1, x2, y2, z2)/10)*euc_percent + (utils_obj.GetTotalAngle(x1, y1, t1, x2, y2, t2)/360)*angle_percent;
			//printf("auc =  %f, angle = %f, line = %f",utils_obj.GetEuclidDistance(x1, y1, z1, x2, y2, z2), utils_obj.GetTotalAngle(x1, y1, t1, x2, y2, t2), line_cost);
			path_cost = path_cost + line_cost; 
		}

		// Check if this is the best path so far
		if (path_index == 0)
		{
			best_cost = path_cost;
		}
		else
		{
			if (path_cost < best_cost)
			{
				best_cost = path_cost;
				best_index = path_index;
			}
		}
	}
	//printf("Best path index: %d\n",best_index);
	//printf("Best path cost: %f\n",best_cost);
	fclose(readFile);
	return best_index;

} 

void PostProcessor::PrepareOnePath(int path_index, const char* path_type)
{
	printf("Prepare one path func\n");
	float x1, y1, z1, x2,y2,z2, temp;
	int numOfPoints, i;
	float colorFactor;
	char file_name[12];
	char gm_file_name[18];
	char gms_file_name[19];
	char img_file_name[13];
	char to_be_added[500];

	sprintf(gm_file_name, "gm_input_%s.txt", path_type);
	sprintf(gms_file_name, "gms_input_%s.txt", path_type);
	sprintf(img_file_name, "img_%s.jpg", path_type);

	cout << "min,max: " << mat_min << " " << mat_max << endl;
	cout << "type: " << global_mat.type() << endl;	

	cv::Mat normalized_img;
	
	global_mat.convertTo(normalized_img, CV_8U, 255.0f/(mat_max - mat_min), (-mat_min * 255.0f)/(mat_max - mat_min));

	//reading from the "path->print()" file
	FILE * readFile;
	FILE * gmFile;
	FILE * gmsFile;
	//for (; this->output_index < NUM_POINTS_AROUND_CENTER; this->output_index++) {
	gmFile = fopen(gm_file_name, "w");
	gmsFile = fopen(gms_file_name, "w");
	if (!gmFile || !gmsFile) {
		cout << "cannot open file gmFile - errno: " << errno << endl;
		return;
	}
	sprintf(file_name, "dots_%d.txt", path_index);
	readFile = fopen(file_name, "r");
	if (!readFile) {
		cout << "cannot open file readFile - errno: " << errno << endl;
		return;
	}
	fscanf(readFile, "Geometric path with %d states\n", &numOfPoints);

	float xs[100];
	float ys[100];
	float zs[100];

	unsigned int idx = 0;
	fscanf(readFile, "Compound state [\n");
	fscanf(readFile,  "RealVectorState [%f %f %f]\n", &x1, &y1, &z1);
	fscanf(readFile,  "RealVectorState [%f]\n", &temp);
	fscanf(readFile, "]\n");
	xs[idx] = x1;
	ys[idx] = y1;
	zs[idx] = z1;
	idx++;
	fscanf(readFile, "Compound state [\n");
	fscanf(readFile,  "RealVectorState [%f %f %f]\n", &x2, &y2, &z2);
	fscanf(readFile,  "RealVectorState [%f]\n", &temp);
	fscanf(readFile, "]\n");
	xs[idx] = x2;
	ys[idx] = y2;
	zs[idx] = z2;
	idx++;
	fprintf(gmFile, "%f %f %f\n", x1, MAP_SIZE - y1, z1 * Z_AXIS_DIV_FACTOR);
	cv::line(normalized_img, cv::Point(x1,y1), cv::Point(x2,y2), cv::Scalar(255,255,0), 1);
	//circle(image, Point(x1,y1), 3, Scalar(255,0,0), FILLED);
	for(i = 3; i<=numOfPoints; i++)
	{
		//Old last-point is now first-point
		x1=x2;
		y1=y2;
		z1=z2;
		
		fscanf(readFile, "Compound state [\n");
		fscanf(readFile,  "RealVectorState [%f %f %f]\n", &x2, &y2, &z2);
		fscanf(readFile,  "RealVectorState [%f]\n", &temp);
		fscanf(readFile, "]\n");
		xs[idx] = x2;
		ys[idx] = y2;
		zs[idx] = z2;
		idx++;
		fprintf(gmFile, "%f %f %f\n", x1, MAP_SIZE - y1, z1 * Z_AXIS_DIV_FACTOR);
		cv::line(normalized_img, cv::Point(x1,y1), cv::Point(x2,y2), cv::Scalar(255,255,0), 1);
		//cv::circle(image, cv::Point(x2,y2), 3, cv::Scalar(255,0,0), cv::FILLED);
	}

	cv::circle(normalized_img, cv::Point(this->coordinates[0],this->coordinates[1]), 3, cv::Scalar(255,0,0), cv::FILLED);

	float prevVal, curVal;
	enum {
		Ascending,
		Descending
	} direction = Ascending;

	fprintf(gmsFile, "%f %f %f\n", xs[0], MAP_SIZE - ys[0], zs[0] * Z_AXIS_DIV_FACTOR);
	prevVal = zs[0];
	
	for (i = 1; i < numOfPoints; i++) {
		curVal = zs[i];
		if (prevVal < curVal) {  // (still) ascending?
			direction = Ascending;
		}
		else if (prevVal > curVal) { // (still) descending?
			if (direction != Descending) { // starts descending?
				fprintf(gmsFile, "%f %f %f\n", xs[i], MAP_SIZE - ys[i], zs[i] * Z_AXIS_DIV_FACTOR);
				direction = Descending;
				memset(to_be_added, 0, 500);
			} else {
				sprintf(to_be_added + strlen(to_be_added),"%f %f %f\n", xs[i], MAP_SIZE - ys[i], zs[i] * Z_AXIS_DIV_FACTOR);
			}
		}
		// prevVal == curVal is simply ignored...
		prevVal = curVal;
	}

	fprintf(gmsFile, "%s", to_be_added);

	fclose(readFile);
	fclose(gmFile);
	cv::imwrite(img_file_name, normalized_img);
}
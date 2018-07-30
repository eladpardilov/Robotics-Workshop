#include <cstdio>

#include <stdio.h>
#include <fstream>
#include <string>
#include <stdlib.h>

#include "PostProcessor.h"

using namespace std;

PostProcessor::PostProcessor(int* coordinates)
{
	this->coordinates = coordinates;
	
}





void PostProcessor::Show()
{
	float x1, y1, z1, x2,y2,z2, temp;
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
	
	//cout << "rows,cols: " << rows << " " << cols << endl;
	
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
	fscanf(readFile, "Compound state [\n");
	fscanf(readFile,  "RealVectorState [%f %f %f]\n", &x1, &y1, &z1);
	fscanf(readFile,  "RealVectorState [%f]\n", &temp);
	fscanf(readFile, "]\n");
	fscanf(readFile, "Compound state [\n");
	fscanf(readFile,  "RealVectorState [%f %f %f]\n", &x2, &y2, &z2);
	fscanf(readFile,  "RealVectorState [%f]\n", &temp);
	fscanf(readFile, "]\n");
	cv::line(normalized_img, cv::Point(x1,y1), cv::Point(x2,y2), cv::Scalar(255,255,0), 1);
	//circle(image, Point(x1,y1), 3, Scalar(255,0,0), FILLED);
	for(int i=3; i<=numOfPoints; i++)
	{
		//Old last-point is now first-point
		x1=x2;
		y1=y2;
		z1=z2;
			
		fscanf(readFile, "Compound state [\n");
		fscanf(readFile,  "RealVectorState [%f %f %f]\n", &x2, &y2, &z2);
		fscanf(readFile,  "RealVectorState [%f]\n", &temp);
		fscanf(readFile, "]\n");
		
		cv::line(normalized_img, cv::Point(x1,y1), cv::Point(x2,y2), cv::Scalar(255,255,0), 1);
		//cv::circle(image, cv::Point(x2,y2), 3, cv::Scalar(255,0,0), cv::FILLED);
	}
	
	cv::circle(normalized_img, cv::Point(this->coordinates[0],this->coordinates[1]), 3, cv::Scalar(255,0,0), cv::FILLED);
	cv::circle(normalized_img, cv::Point(0,20), 3, cv::Scalar(255,0,0), cv::FILLED);


	fclose(readFile);
	cv::imshow( "Our Plane Path", normalized_img);
	
	cv::waitKey(0);
	
}
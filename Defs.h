/*
 * Defs.h
 *
 *  Created on: May 9, 2018
 *      Author: Dana
 */

#ifndef DEFS_H_
#define DEFS_H_

#define RETURN_CODE_ERROR (-1)
#define RETURN_CODE_SUCCESS (0)
#define MAIN_NUM_OF_ARGS (4)
#define MAX_HEIGHT	(50)
#define	MAP_SIZE	(360)
#define MAX_DISTANCE_FOR_STEP (10)
#define NUM_POINTS_AROUND_CENTER (12)
#define Z_AXIS_DIV_FACTOR (30.0)
#define SOLVING_TIME (15.0)
// 30 meters = 1 pixel, 150km/h = 41.67m/s = 1.39pixel/s
#define CONSTANT_VELOCITY (1.39)
#define DOTS_FILE_NAME "dots.txt"
#define MAP_TIF "N46E010.tif"
//#define MAP_TIF "sample.tif"

#endif /* DEFS_H_ */

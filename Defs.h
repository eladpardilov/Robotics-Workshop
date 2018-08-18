#ifndef DEFS_H_
#define DEFS_H_

#define DEFAULT_NUM_STATES 10000
#define DEFAULT_RADIUS 5000/30
#define DEFAULT_ANGLE 30
#define DEFAULT_X 180
#define DEFAULT_Y 180
#define DEFAULT_RATE 50 
#define DEFAULTF_FILE "map.tif"


#define RETURN_CODE_ERROR (-1)
#define RETURN_CODE_SUCCESS (0)

#define MAX_HEIGHT	(50)
#define	MAP_SIZE	(360)
#define HORIZ_VERTICAL_MAX_RATE (2)
#define MAX_DISTANCE_FOR_STEP (10)
#define NUM_POINTS_AROUND_CENTER (12)
#define METERS_PER_PIXEL (30)
#define Z_AXIS_DIV_FACTOR (30.0)
#define MAX_PRM_TRIALS (20)
#define SOLVING_TIME (600.0)
// 30 meters = 1 pixel, 150km/h = 41.67m/s = 1.39pixel/s
#define CONSTANT_VELOCITY (1.39)
#define DOTS_FILE_PREFIX "dots_"
#define DOTS_FILE_SUFFIX ".txt"
#define DOTS_FILE_NAME "dots.txt"
#define EPSILON (0.000001)
#define MAP_TIF "N46E010.tif"
#define PI (3.1415926535897)

#define BEST_IMG_FILE "img_best.jpg" 
#define SAFE_IMG_FILE "img_safe.jpg" 
#define FAST_IMG_FILE "img_fast.jpg" 

#endif /* DEFS_H_ */

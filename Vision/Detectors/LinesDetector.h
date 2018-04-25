#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/core/cvstd.hpp"
#include <string>
#include <math.h>
#include "../VisionThread.h"
using namespace cv;
using namespace std;

#define NUM_CHANNELS 3
#define MAX_HUE_VALUE 179   //Hue is in range [0,179].
#define MIN_GREEN_HUE 40 //40   //Heuristic. 32
#define MAX_GREEN_HUE 70 // 70   //Heuristic. 90
#define MIN_SATURATION_FIELD 100 //Heuristic. 120
#define HUE_DISCRETIZATION 180  //Hue is in range [0,179].
#define MAX_BALL_DIAMETER 250  //Heuristic. should be even.
#define MIN_BALL_DIAMETER 16   //Heuristic. should be even. May cause problems with field's line morphological closing performed on field_mat if below 25.
#define MAX_WHITE_SATURATION 150 //Heuristic. saturation range is [0,255]. First try - with 77. (if using gaussian blur on original img -switch to 120)
#define MIN_WHITE_VALUE 120 //Heuristic. value range is [0,255]. First try - with 178 (if using gaussian blur on original img switch to 170).
#define PI 3.14159265358979 //Value of Pi (11 digits after the point).
#define SUB_IMAGE_HALF_WIDTH (MAX_BALL_DIAMETER-MIN_BALL_DIAMETER/2)//As we crop the image to get a sub-image(square shape) in the algorithm - we define this value to be the maximal needed half width of this image.
#define STRUCTURE_ELEMENT_SIZE 3//The size of the structure element's matrix.
#define BOUNDING_HORIZONTAL_LINE_FIX 30 //(At most) 30 more pixels (~8% of frame with height 480 px). will check if corners are +-40 pixels around the field's bounding horizontal line.


class LinesDetector{
public:

	static void GetLinesPosts(Mat image); //The result will be set into the gc variable. please note that if no post was found all values in x,y of points will be set to -1. otherwise if only 1 post was found only the left post will be set to the appropriate value. (otherwise 2 posts-> update of both posts left and right).
	static void FieldCalibration(Mat& hue_matrix, uchar& field_min_hue, uchar& field_max_hue) ;	//Function calculates the min hue value of the green field and max hue value of the green field.
																								//Takes- hue_matrix as input. Returns field_min_hue and field_max_hue
	static void CalculateBoundingHorizontalLine(Mat& field_mat, ushort& bounding_horizontal_line);
};






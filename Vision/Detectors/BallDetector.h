#ifndef VISION_DETECTORS_BallDetector_H_
#define VISION_DETECTORS_BallDetector_H_

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/core/cvstd.hpp"
#include <string>
#include <math.h>
#include <iostream> //Only for collecting data from algorithm (log files etc...)
#include <sstream>

#include "MachineLearning/BallNeuralNet.h"
#include "BallCandidateRansac.h"
#include "../VisionThread.h"

using namespace cv;
using namespace std;

#define NUM_CHANNELS 3
#define MAX_HUE_VALUE 179   //Hue is in range [0,179].
#define MIN_GREEN_HUE 40 //40   //Heuristic. 32
#define MAX_GREEN_HUE 70 // 70   //Heuristic. 90
#define MIN_SATURATION_FIELD 100 //Heuristic. 120
#define HUE_DISCRETIZATION 180  //Hue is in range [0,179].
#define MAX_BALL_DIAMETER 225  //Heuristic. should be even.
#define MIN_BALL_DIAMETER 16   //Heuristic. should be even. May cause problems with field's line morphological closing performed on field_mat if below 25.
#define MAX_WHITE_SATURATION 110 //Heuristic. saturation range is [0,255]. First try - with 77. (if using gaussian blur on original img -switch to 120)
#define MIN_WHITE_VALUE 120 //Heuristic. value range is [0,255]. First try - with 178 (if using gaussian blur on original img switch to 170).
#define PI 3.14159265358979 //Value of Pi (11 digits after the point). 
#define SUB_IMAGE_HALF_WIDTH (MAX_BALL_DIAMETER-MIN_BALL_DIAMETER/2)//As we crop the image to get a sub-image(square shape) in the algorithm - we define this value to be the maximal needed half width of this image.
#define STRUCTURE_ELEMENT_SIZE 3 //The size of the structure element's matrix.
#define DEG_QUANTIZATION 361 //Discretization level of degrees.
#define INPUT_SIZE_BALL_NEURAL_NET 3500 //The size of an input image is 35X70=2450.
#define HIDDEN_LAYER_SIZE_BALL_NEURAL_NET 25 //Number of activation units in hidden layer.
#define OUTPUT_SIZE_BALL_NEURAL_NET 1 //This neural network will only have 1 class for classification.
#define PATH_INPUT_HIDDEN_WEIGHTS_BALL_NEURAL_NET "/home/robot/workspace/RoboCup2017/Vision/Detectors/MachineLearning/BallDetectorNeuralNet/weights_input_hidden_ball_detector.txt"
#define PATH_HIDDEN_OUTPUT_WEIGHTS_BALL_NEURAL_NET "/home/robot/workspace/RoboCup2017/Vision/Detectors/MachineLearning/BallDetectorNeuralNet/weights_hidden_output_ball_detector.txt"
#define NEURAL_NET_INPUT_IMAGE_WIDTH 70
#define NEURAL_NET_INPUT_IMAGE_HEIGHT 50
#define HUE_QUANTIZATION 180 //180 levels for the hue axis range - [0,179]

class BallDetector{
public:

	BallDetector();
	virtual ~BallDetector();
	static void GetBallCenter(Point& returned_center,int& returned_radius);
	//Function calculates the min hue value of the green field and max hue value of the green field.
	//Takes- hue_matrix as input. Returns field_min_hue and field_max_hue
	static void CalculateDistanceToBall(int& radius,double& calculated_distance); //Calculate the distance from the ball using a linear regression model.
	//TODO implement this method ^^^^^^^^
	static double Sigmoid(double x); //This method calculates the sigmoid function of a given real value.
private:
	static bool IsFirstRun;
	static double ball_hue_histogram[HUE_QUANTIZATION];
	static BallNeuralNet BallDetectionNeuralNet; //This NeuralNet object holds the neural net parameters for the whole running of the program. It has the query() function which lets one ask the neural net whether a blob is a ball.
	static void FieldCalibration(Mat& hue_matrix, uchar& field_min_hue, uchar& field_max_hue) ;
	static void CalculateCorrelationToCircleHist(Mat bw_mat,double& correlation);
	static void CalculateBoundingHorizontalLine(Mat& field_mat, ushort& bounding_horizontal_line);
	static void CircleFitKasa(vector<Point>& contour,Point& center, double& radius);
	static void CircleFitRansac(Mat& frame,Mat& frame_edges,vector<BallCandidateRansac>& found_circles,int min_radius,int max_radius); //Search for ball using the Ransac scheme.
	static void GetCircleBoundingRectangleInFrame(Point& center,int& radius,int& frame_rows,int& frame_cols,Rect& bounding_rect); //This method gets the circle's center,radius and the frame's number of rows and columns and returns the bounding rectangle in the bounding_rect object.
	static void CalculateBallColorsCorrelation(Mat& hue_mat,Rect& candidate_bounding_rect,double& correlation,uchar& field_min_hue ,uchar& field_max_hue);
	static void BallHistogramCalibration(Mat& hue_mat , Rect& ball_roi,uchar& field_min_hue ,uchar& field_max_hue ); //This method sets the ball_hue_histogram values for the use of the CalculateBallColorsCorrelation later on to prevent misses/false alarms.
	static void CountNumFieldPixels(Mat& candidate,uchar& field_min_hue,uchar& field_max_hue,uint& num_field_pixels); //This methods counts how many field pixels are in candidate matrix - by using the field_min_hue and field_max_hue received by field calibration. result is in  num_field_pixels.
	//static void CalculateFieldConvexHUll(Mat& field_mat,vector<Point2i>& hull); //This method gets the field matrix after calibration and calculates the convex hull of the field.
};

#endif /* VISION_DETECTORS_BallDetector_H_ */



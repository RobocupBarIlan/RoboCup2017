/*
 * GoalKeeperDetector.h
 *
 *  Created on: Jun 29, 2017
 *      Author: root
 */

#ifndef VISION_DETECTORS_GOALKEEPERSDETECTOR_H_
#define VISION_DETECTORS_GOALKEEPERSDETECTOR_H_

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/core/cvstd.hpp"
#include <string>
#include <math.h>
#include <iostream> //Only for collecting data from algorithm (log files etc...)
#include <sstream>
#include "../VisionThread.h"

#define NUM_CHANNELS 3

using namespace std;
using namespace cv;
class GoalKeepersDetector {
public:
	static void GetGoalKeepers(Point& ourTeamGoalKeeperXY, Point& otherTeamGoalKeeperXY); //This method returns (x1,y1) and (x2,y2) of center of the detected shirt of the goal keeper. (x,y) are set to -1,-1 if no goalkeeper is found (for either of the goalkeepers [our's/the other team's].
	GoalKeepersDetector();
	virtual ~GoalKeepersDetector();
	static bool IS_FIRST_RUN; //Needed to know whether a calibration of the goalkeepers' colors is needed.
	static	int min_hue_our_goalkeeper;
	static	int max_hue_our_goalkeeper;
	static	int min_value_our_goalkeeper;
	static	int max_value_our_goalkeeper;
	static	int min_hue_other_team_goalkeeper;
	static  int max_hue_other_team_goalkeeper;
	static 	int min_value_other_team_goalkeeper;
	static	int max_value_other_team_goalkeeper;

};

#endif /* VISION_DETECTORS_GOALKEEPERSDETECTOR_H_ */

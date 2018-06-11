/*
 * GoalCandidate.h
 *
 *  Created on: Jun 21, 2017
 *      Author: root
 */

#ifndef VISION_DETECTORS_GOALCANDIDATE_H_
#define VISION_DETECTORS_GOALCANDIDATE_H_
#include <opencv2/opencv.hpp>
#include "opencv2/core/cvstd.hpp"
#include <string>
#include <math.h>
using namespace cv;
using namespace std;

class GoalCandidate {
public:
	Point m_left_post[2]; //Holds the detected left post's 2 bottom corners.
	Point m_right_post[2]; //Holds the detected right post's 2 bottom corners. if m_right_post[0].x=-1 then only 1 post was detected.
	int m_width_left; //Holds the width of the left post.
	int m_width_right; //Holds the width of the right post.
	GoalCandidate(vector<Point>& left_post,vector<Point>& right_post);
	GoalCandidate();
	virtual ~GoalCandidate();
};

#endif /* VISION_DETECTORS_GOALCANDIDATE_H_ */

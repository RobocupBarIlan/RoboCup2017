/*
 * BallCandidateRansac.h
 *
 * This class holds a candidate found by the Ransac scheme. It holds the Center,radius and support of the candidate.
 */

#ifndef VISION_DETECTORS_BALLCANDIDATERANSAC_H_
#define VISION_DETECTORS_BALLCANDIDATERANSAC_H_
#include <opencv2/opencv.hpp>
using namespace cv;

class BallCandidateRansac {
public:
	BallCandidateRansac(Point center,int radius,int support);
	virtual ~BallCandidateRansac();
	Point m_center;
	int m_radius;
	int m_support; //In number of points supporting the candidate.

	//Define a comparator struct for sorting a vector by support.
	struct by_support {
	    bool operator()(BallCandidateRansac const &a, BallCandidateRansac const &b) {
	        return a.m_support > b.m_support;
	    }
	};
};

#endif /* VISION_DETECTORS_BALLCANDIDATERANSAC_H_ */

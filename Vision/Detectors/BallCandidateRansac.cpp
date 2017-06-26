/*
 * BallCandidateRansac.cpp
 *
 *  Created on: Jun 12, 2017
 *      Author: root
 */

#include "BallCandidateRansac.h"










BallCandidateRansac::BallCandidateRansac(Point center,int radius,int support) {
	this->m_center.x=center.x;
	this->m_center.y=center.y;
	this->m_radius=radius;
	this->m_support=support;
}

BallCandidateRansac::~BallCandidateRansac() {
	// TODO Auto-generated destructor stub
}


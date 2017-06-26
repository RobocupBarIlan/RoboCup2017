/*
 * GoalCandidate.cpp
 *
 *  Created on: Jun 21, 2017
 *      Author: root
 */

#include "GoalCandidate.h"


GoalCandidate::GoalCandidate(vector<Point>& left_post,vector<Point>& right_post)
{
	// TODO Auto-generated constructor stub
	this->m_left_post[0].x=left_post[0].x;
	this->m_left_post[0].y=left_post[0].y;
	this->m_left_post[1].x=left_post[1].x;
	this->m_left_post[1].y=left_post[1].y;

	this->m_right_post[0].x=right_post[0].x;
	this->m_right_post[0].y=right_post[0].y;
	this->m_right_post[1].x=right_post[1].x;
	this->m_right_post[1].y=right_post[1].y;

	this->m_width_left=abs(m_left_post[1].x-m_left_post[0].x);
	this->m_width_right=abs(m_right_post[1].x-m_right_post[0].x);
}

GoalCandidate::GoalCandidate()
{
	this->m_left_post[0].x=-1;
	this->m_left_post[0].y=-1;
	this->m_left_post[1].x=-1;
	this->m_left_post[1].y=-1;

	this->m_right_post[0].x=-1;
	this->m_right_post[0].y=-1;
	this->m_right_post[1].x=-1;
	this->m_right_post[1].y=-1;
}

GoalCandidate::~GoalCandidate() {
	// TODO Auto-generated destructor stub
}


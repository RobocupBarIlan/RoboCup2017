//============================================================================
// Name        : RoboCup2017.cpp
// Author      : 
// Version     :
//============================================================================

#include <iostream>

#include "../Vision/VisionThread.h"
#include "../Brain/BrainThread.h"
#include "../Brain/Motion/Motion.h"
#include "../Communication/CommunicationThread.h"
#include "../Vision/Detectors/GoalCandidate.h"



/*
 * This method makes sure that all signals had been registered to the different threads before any operation is done.
 */
void waitRegisterSignalDone()
{
	while (!VisionThread::IsRegisterSingalsDone()) //Wait until VisionThread registered all signal listeners.
	{
	}
}


int main() {
	//Motion::GetInstance()->FreeAllEngines();
	//cout << "~~~~~~~~~~~~~~Initiating threads:~~~~~~~~~~~~~~" << endl; // prints !!!Hello World!!!
	VisionThread::GetVisionThreadInstance()->init();
	waitRegisterSignalDone();
	//Must sleep for 3 seconds at the beginning to let the camera warm-up:
	VisionThread::MillisSleep(3000);


	//BrainThread::GetBrainThreadInstance()->init();
	//CommunicationThread::GetCommunicationThreadInstance()->init();


//	Point center;
//	int radius;
//	double distance;

	GoalCandidate gc;

	while(1)
	{
//	//Getting data from the vision thread example:
////		int center_x;
////		int center_y;
////		double distance;
////		VisionThread::SafeReadBallCenterInFrameAndDistance(center_x,center_y,distance);
////		cout<<"center.x: "<<center_x<<"center.y: "<<center_y<<endl;
////		cout<<"distance:"<<distance<<endl;
//
//
//		BallDetector::GetBallCenter(center,radius);


//
//		//BallDetector::CalculateDistanceToBall(radius,distance);
//		//cout<<"distance:"<<distance<<endl;
//
		GoalDetector::GetGoalPosts(gc);
//		//VisionThread::MillisSleep(100);
//		//break;
	}

	pthread_exit(NULL); //Exit the main thread while keeping the other threads alive.


}



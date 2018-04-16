//============================================================================
// Name        : RoboCup2017.cpp
// Author      : 
// Version     :
//============================================================================

#include <iostream>


#include "../Vision/VisionThread.h"
#include "../Brain/BrainThread.h"
#include "../Fallen/FallenThread.h"
#include "../Brain/Motion/Motion.h"
#include "../Communication/CommunicationThread.h"
#include "../Vision/Detectors/GoalCandidate.h"
#include "../Vision/Detectors/GoalKeepersDetector.h"



/*
 * This method makes sure that all signals had been registered to the different threads before any operation is done.
 */
void waitRegisterSignalDone()
{
	while (!VisionThread::IsRegisterSingalsDone())// && !BrainThread::IsRegisterSingalsDone()) //Wait until VisionThread & BrainThread registered all signal listeners.
	{
	}
}

int main() {
	Motion::GetInstance()->FreeAllEngines();
	cout << "~~~~~~~~~~~~~~Initiating threads:~~~~~~~~~~~~~~" << endl; // prints !!!Hello World!!!
	VisionThread::GetVisionThreadInstance()->init();
	BrainThread::GetBrainThreadInstance()->init();
	waitRegisterSignalDone();
//	CommunicationThread::GetCommunicationThreadInstance()->init();
//BrainThread::checkTiltPan(10,10);
	//Must sleep for 3 seconds at the beginning to let the camera warm-up (clean garabage in buffer):
	VisionThread::MillisSleep(3000); //Sleep to clean the buffer
	VisionThread::IPM();

	//Must calibrate the ball before first run!!!:
	//VisionThread::SafeReadBallCenterInFrameAndDistance(center_x,center_y,distance);


//	Point center;
//	int radius;
//	double distance;
//
//	GoalCandidate gc;
//
//
//	Point g1,g2;
//	int center_x;
//	int center_y;
//	double distance;
//
//	while(1)
//	{
////////	//Getting data from the vision thread example:
////
////
//////    	VisionThread::SafeReadBallCenterInFrameAndDistance(center_x,center_y,distance);
////
//////
////		GoalKeepersDetector::GetGoalKeepers(g1,g2);
//////
//		VisionThread::SafeReadGoalInFrame(gc);
//////
////////		//VisionThread::MillisSleep(100);
//////		//break;
//	}
//////
//	pthread_exit(NULL); //Exit the main thread while keeping the other threads alive.


}



/*
 * BrainThread.h
 *
 */


#ifndef NULL
#define NULL   ((void *) 0)
#endif

#ifndef BRAIN_BRAINTHREAD_H_
#define BRAIN_BRAINTHREAD_H_

#include <pthread.h>
#include <cstdlib>
#include <iostream>
#include <unistd.h>
#include <signal.h>
#include <mutex>
#include <atomic>
#include <string>
#include <math.h>
#include <opencv2/opencv.hpp>
#include "../Vision/VisionThread.h"
#include "Motion/Motion.h"


#define INIT_VALUE -1 //Will be used to initialize class members before running any code.
#define NOT_FOUND_OBJECT_VALUE -1 //Will be used when no object is found. we will set the appropriate class members to -1.
#define START_STATE 1
#define LOOK_FOR_BALL_STATE 2
#define WALK_TO_BALL_STATE 3
#define LOOK_FOR_GOAL_STATE 4
#define KICK_STATE 5
#define CHANGE_SPOT_STATE 6
#define TiltMax -10 // maximum head tilt
#define TiltMin -54 // minimum head tilt
#define PANMAXLEFT 67 // // maximum head pan
#define PANMAXRIGHT -67 // // minimum head pan
#define NOT_FOUND_OBJECT_VALUE -1 // Will be used when no object is found. we will set the appropriate class members to -1.
#define SINUS_CONST 2.68
#define PI 3.14159265
#define TILTMOVEMENT 15;
#define PANMOVEMENT 14;

using namespace std;
class BrainThread { //Singleton - only one object should be instantiated!
private:
	pthread_t m_brain_thread; //Will control all the brain tasks.
    static BrainThread* Brain_Thread_Instance; //Contains the one and only possible instance of brain thread.
	BrainThread();
	int m_state_name;
	Motion* m_Motion;

public:
	pthread_t getBrainThread(); //Returns the brain_thread of type pthread_t class member.
	static BrainThread* GetBrainThreadInstance(); //This method makes sure we don't create more than 1 object of this class.
	void init(); //This method initiates the brain thread.
	virtual ~BrainThread();
	Motion* getMotion();
	void setState(int new_state);
	int getState();
	void start();
	void lookForBall();
	void GoToBall();
	void lookForGoal();
	void kick();
	void changeSpot();
	void centerBall();
};

#endif /* BRAIN_BRAINTHREAD_H_ */

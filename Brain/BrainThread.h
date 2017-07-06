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
#include "../Fallen/FallenThread.h"
#include "Motion/Motion.h"
#include "../Communication/UdpListener.h"

#ifndef PI
#define PI 3.14159265
#endif
#define INIT_VALUE -1 //Will be used to initialize class members before running any code.
#define NOT_FOUND_OBJECT_VALUE -1 //Will be used when no object is found. we will set the appropriate class members to -1.
#define START_STATE 1
#define LOOK_FOR_BALL_STATE 2
#define GO_TO_BALL_STATE 3
#define LOOK_FOR_GOAL_STATE 4
#define KICK_STATE 5
#define CHANGE_SPOT_STATE 6
#define FINISHED_STATE 7
#define TILT_MAX 10 // maximum head tilt
#define TILT_MIN -45 // minimum head tilt
#define PAN_MAX_LEFT 67 // // maximum head pan
#define PAN_MAX_RIGHT -67 // // minimum head pan
#define NOT_FOUND_OBJECT_VALUE -1 // Will be used when no object is found. we will set the appropriate class members to -1.
#define SINUS_CONST 2.68
#define TILT_MOVEMENT 15;
#define PAN_MOVEMENT 14;

using namespace std;

class BrainThread { //Singleton - only one object should be instantiated!
private:
	pthread_t m_brain_thread; //Will control all the brain tasks.
    static BrainThread* Brain_Thread_Instance; //Contains the one and only possible instance of brain thread.
	BrainThread();
	int m_state_name;
	Motion* m_Motion;
    static bool Is_Register_Signals_Done; //A flag to indicate registering signals is done
    static void StateMachine(); //This method handle all states the playerRobot can be during the game.
    static void GoalKeeperStateMachine(); //This method handle all states the goalKeeperRobot can be during the game.
    static void HandleRefereeMessage(); //This method is called to handle a signal - NEW_REFEREE_MESSAGE.

public:
	enum BRAIN_THREAD_SIGNALS {NEW_REFEREE_MESSAGE=4 ,PLAYER_INFO_MESSAGE=5 ,TEAM_INFO_MESSAGE=6, FALLEN_MESSAGE=7};
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
	static void RegisterSignals(); //This method registers all signals which can be sent to the brain thread.
	static void SignalCallbackHandler(int signum); //This method handles all the possible signals which can be sent to the brain thread.
	static bool IsRegisterSingalsDone(); //This method tells whether the RegisterSignals() method has already been called. It is crucial so we won't send signals before that is done.
};

#endif /* BRAIN_BRAINTHREAD_H_ */

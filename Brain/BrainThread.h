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
#include "../Brain/Motion/Motion.h"
#include "../Vision/VisionThread.h"
#include "../Fallen/FallenThread.h"
#include "../Communication/UdpListener.h"
#include "BallFollower.h"

#ifndef PI
#define PI 3.14159265
#endif
#define INIT_VALUE -1 //Will be used to initialize class members before running any code.
#define NOT_FOUND_OBJECT_VALUE -1 //Will be used when no object is found. we will set the appropriate class members to -1.
#define START_STATE 1
#define LOOK_FOR_BALL_STATE 2
#define WALK_TO_BALL_STATE 3
#define LOOK_FOR_GOAL_STATE 4
#define REPOSITION_BEFORE_KICK 8
#define KICK_STATE 5
#define CHANGE_SPOT_STATE 6
#define FINISHED_STATE 7
#define TILT_MAX 10 // maximum head tilt
#define TILT_MIN -45 // minimum head tilt
#define PAN_MAX_LEFT 67 // // maximum head pan
#define PAN_MAX_RIGHT -67 // // minimum head pan
#define NOT_FOUND_OBJECT_VALUE -1 // Will be used when no object is found. we will set the appropriate class members to -1.
#define SINUS_CONST 2.68
#define TILT_MOVEMENT 100;
#define PAN_MOVEMENT 100;


using namespace std;

class BrainThread { //Singleton - only one object should be instantiated!
private:
	pthread_t m_brain_thread; //Will control all the brain tasks.
    static BrainThread* Brain_Thread_Instance; //Contains the one and only possible instance of brain thread.
	BrainThread();
	int m_state_name;
	Motion* m_Motion;
    static bool Is_Register_Signals_Done; //A flag to indicate registering signals is done
    int m_kick;
    BallFollower m_follower;
    int m_change_spot_counter;

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
	void setKick(int new_kick);
	int getKick();
	void start();
	void lookForBall();
	void lookForGoal();
	void kick();
	void changeSpot();
	void finish();
	void centerBall();
	void followBall();

    pthread_t m_center_ball_thread; //Will control centering the ball
    bool centerBall_thread_open = false;
    static std::mutex WriteDetectedDataMutexBrain;  //Create a mutex for calls to the vision thread - so another thread (the brain for example) won't read wrong data - i.e to make the proccess thread-safe.
    static std::mutex WriteDetectedDataMutexBrainCenter;
    static int m_center_x;
    static int m_center_y;
    static int m_continue_center_thread;
    static std::atomic<bool> Is_Ball_Writing_Done_Brain;
    static std::atomic<bool> Is_Ball_Reading_Done_Brain;
	//static std::mutex WriteDetectedDataMutex;  //Create a mutex for calls to the vision thread - so another thread (the brain for example) won't read wrong data - i.e to make the proccess thread-safe.
	//static std::mutex FrameReadWriteMutex; //Create a mutex for reading and writing to m_frame. this should handle the data consistency issues that might occure.
	static void RegisterSignals(); //This method registers all signals which can be sent to the brain thread.
	static void SignalCallbackHandler(int signum); //This method handles all the possible signals which can be sent to the brain thread.
	static bool IsRegisterSingalsDone(); //This method tells whether the RegisterSignals() method has already been called. It is crucial so we won't send signals before that is done.
	static void checkTiltPan(float tilt,float pan);

};

#endif /* BRAIN_BRAINTHREAD_H_ */

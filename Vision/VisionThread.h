/*
 * VisionThread.h
 *
 * This class main purpose it to provide a safe read of different vision algorithm outputs - such as ball detection, gate detection and goalkeeper detection.
 * It provides the methods:
 *	SafeReadBallCenterInFrameAndDistance()
 *  SafeReadGoalCenterInFrameAndDistance() TODO
 *  SafeReadGoalKeeperCenterInFrameAndDistance() TODO
 */


#ifndef NULL
#define NULL   ((void *) 0)
#endif

#ifndef VISION_VISIONTHREAD_H_
#define VISION_VISIONTHREAD_H_

#include <pthread.h>
#include <cstdlib>
#include <iostream>
#include <unistd.h>
#include <time.h>
#include <signal.h>
#include <mutex>
#include <atomic>
#include <math.h>
#include <opencv2/opencv.hpp>
#include "Detectors/BallDetector.h"
#include "Detectors/GoalDetector.h"
//	// OpenCV imports
//	#include <opencv2/imgproc/imgproc.hpp>
//	#include <opencv2/highgui/highgui.hpp>

#define INIT_VALUE -1 //Will be used to initialize class members before running any code.
#define NOT_FOUND_OBJECT_VALUE -1 //Will be used when no object is found. we will set the appropriate class members to -1.

using namespace std;
// namespaces
using namespace cv;

#define PI 3.1415926
class VisionThread { //Singleton - only one object should be instantiated!
private:
	pthread_t m_vision_thread; //Will control all the vision tasks.
	static int BallCenterX; //Will contain the ball center x coordinate after a signal to calculate the ball center is called. will contain INIT_VALUE
	static int BallCenterY; //Will contain the ball center y coordinate after a signal to calculate the ball center is called.
	static double BallDistance; //Will contain the distance towards the ball if ball is actually found.
    static GoalCandidate DetectedGoalCandidate; //Will contain the GoalCandidate object found by the goal detection algorithm.
	static VisionThread* Vision_Thread_Instance; //Contains the one and only possible instance of vision thread.
    static bool Is_Register_Signals_Done; //A flag to indicate registering signals is done.
    static std::atomic<bool> Is_Ball_Writing_Done; //An atomic boolean flag to indicate that a writing to an object is finished and a consumer thread can read it. It is used as a synchronization tool (read only after write)
    static std::atomic<bool> Is_Goal_Writing_Done;//An atomic boolean flag to indicate that a writing to an object is finished and a consumer thread can read it. It is used as a synchronization tool (read only after write)
    static void GetBallCenterInFrameAndDistance(); //This method is called to handle a signal - GET_BALL_CENTER_IN_FRAME_AND_DISTANCE .
	static void GetGoalCandidate(); //This method called is called to handle a signal -GET_GOAL_CENTER_IN_FRAME_AND_DISTANCE.
    static std::atomic<bool> IS_NO_BALL_COMPUTATION; //An atomic boolean flag to indicate that a computation for the ball is currently running. this flag is required so we won't have 2 calls for ball computation running together -interferring each other!
	static std::atomic<bool> IS_NO_GOAL_COMPUTATION; //An atomic boolean flag to indicate that a computation for the ball is currently running. this flag is required so we won't have 2 calls for goal computation running together -interferring each other!
    VisionThread();

public:
	static Mat Frame;
	enum VISION_THREAD_SIGNALS { GET_BALL_CENTER_IN_FRAME_AND_DISTANCE=2, GET_GOAL_IN_FRAME=4, GET_GOALKEEPER_CENTER_IN_FRAME_AND_DISTANCE=5};
	static std::mutex WriteDetectedDataMutex;  //Create a mutex for calls to the vision thread - so another thread (the brain for example) won't read wrong data - i.e to make the proccess thread-safe.
	static std::mutex FrameReadWriteMutex; //Create a mutex for reading and writing to m_frame. this should handle the data consistency issues that might occure.
	pthread_t getVisionThread(); //Returns the vision_thread of type pthread_t class member.
	static VisionThread* GetVisionThreadInstance(); //This method makes sure we don't create more than 1 object of this class.
    static void RegisterSignals(); //This method registers all signals which can be sent to the vision thread.
    static void SignalCallbackHandler(int signum); //This method handles all the possible signals which can be sent to the vision thread.
	static void SafeReadBallCenterInFrameAndDistance(int& center_x,int& center_y,double& distance); //This method gets - center_x,center_y,distance and sets the last calculated values safely into them.
	static void SafeReadGoalInFrame(GoalCandidate& gc); //This method sets the last calculated values of the goal safely into gc.
	static void IPM();
	static void SafeReadeCapturedFrame(Mat& captured_frame); //This method lets another caller to read the captured by the vision thread frame safely without any data inconsistency issues.
	void init(); //This method initiates the vision thread.
	static bool IsRegisterSingalsDone(); //This method tells whether the RegisterSignals() method has already been called. It is crucial so we won't send signals before that is done.
	static int MillisSleep(long miliseconds);
	static std::atomic<bool> IS_PROCCESSING_IMAGE; //This flag indicates that a request for image processing was done and currently running. this flag prevents image capturing while proccessing to save resources.
	static std::atomic<bool> IS_READING_FRAME; //THis flag indicates that a detector is trying to read the frame
	virtual ~VisionThread();

};

#endif /* VISION_VISIONTHREAD_H_ */

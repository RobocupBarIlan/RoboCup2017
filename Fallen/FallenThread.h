/*
 * FallenThread.h
 *
 *  This class main purpose is to indicates when the robot fall and send a proper signal to Brain thread
 * 	It provides the methods:
 *
 */

#ifndef NULL
#define NULL   ((void *) 0)
#endif

#ifndef FALLEN_FALLENTHREAD_H_
#define FALLEN_FALLENTHREAD_H_

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
#include "../Brain/BrainThread.h"
#include "../Vision/VisionThread.h"

#define INIT_VALUE -1 //Will be used to initialize class members before running any code.

using namespace std;

class FallenThread { //Singleton - only one object should be instantiated!
private:
	pthread_t m_fallen_thread; //Will control all the fallen tasks.
	static FallenThread* Fallen_Thread_Instance; //Contains the one and only possible instance of fallen thread.
	static bool Is_Register_Signals_Done; //A flag to indicate registering signals is done
	FallenThread();

public:
	enum FALLEN_THREAD_SIGNALS { FALLEN_MASSEGE=4 };

	pthread_t getFallenThread(); //Returns the fallen_thread of type pthread_t class member.
	static FallenThread* GetFallenThreadInstance(); //This method makes sure we don't create more than 1 object of this class.
    //static void RegisterSignals(); //This method registers all signals which can be sent to the fallen thread.
    static void SignalCallbackHandler(int signum); //This method handles all the possible signals which can be sent to the fallen thread.
	void init(); //This method initiates the fallen thread.
	static bool IsRegisterSingalsDone(); //This method tells whether the RegisterSignals() method has already been called. It is crucial so we won't send signals before that is done.
	virtual ~FallenThread();
	void IsFallen();
};

#endif /* FALLEN_FALLENTHREAD_H_ */

#pragma once


/*
* CommunicationThread.h
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

#ifndef COMMUNICATIONN_COMMUNICATIONNTHREAD_H_
#define COMMUNICATIONN_COMMUNICATIONNTHREAD_H_

#include "UdpSender.h"
#include "UdpListener.h"
#include <pthread.h>
#include <cstdlib>
#include <iostream>
#include <unistd.h>
#include <time.h>
#include <signal.h>
#include <mutex>
#include <atomic>

//#define INIT_VALUE -1 //Will be used to initialize class members before running any code.
//#define NOT_FOUND_OBJECT_VALUE -1 //Will be used when no object is found. we will set the appropriate class members to -1.

using namespace std;
class CommunicationThread { //Singleton - only one object should be instantiated!
private:
	pthread_t m_communication_thread; //Will control all the communication tasks.
	static CommunicationThread* Communication_Thread_Instance; //Contains the one and only possible instance of vision thread.
	static bool Is_Register_Signals_Done; //A flag to indicate registering signals is done.
										  //static std::atomic<bool> Is_Writing_Done; //An atomic boolean flag to indicate that a writing to an object is finished and a consumer thread can read it. It is used as a synchronization tool (read only after write)
	CommunicationThread();

public:
	//enum COMMUNICATIONN_THREAD_SIGNALS { NEW_MESSAGE_SIGNAL = 2 };
	//static std::mutex CommunicationMutex;  //Create a mutex for calls to the Communication thread - so another thread (the brain for example) won't read wrong data - i.e to make the proccess thread-safe.
	pthread_t getCommunicationThread(); //Returns the Communication_thread of type pthread_t class member.
	static CommunicationThread* GetCommunicationThreadInstance(); //This method makes sure we don't create more than 1 object of this class.
	//static void RegisterSignals(); //This method registers all signals which can be sent to the Communication thread.
	//static void SignalCallbackHandler(int signum); //This method handles all the possible signals which can be sent to the brain thread.
	void init(); //This method initiates the vision thread.
	static bool IsRegisterSingalsDone(); //This method tells whether the RegisterSignals() method has already been called. It is crucial so we won't send signals before that is done.
	//static int MillisSleep(long miliseconds);
	virtual ~CommunicationThread();

};

#endif /* COMMUNICATIONN_COMMUNICATIONNTHREAD_H_ */

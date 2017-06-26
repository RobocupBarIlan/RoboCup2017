/*
 * CommunicationThread.h
 *
 */


#ifndef NULL
#define NULL   ((void *) 0)
#endif

#ifndef COMMUNICATION_COMMUNICATIONTHREAD_H_
#define COMMUNICATION_COMMUNICATIONTHREAD_H_

#include <pthread.h>
#include <cstdlib>
#include <iostream>
#include <unistd.h>
#include <signal.h>
#include <mutex>
#include <atomic>
#include <string>
#include <math.h>




using namespace std;
class CommunicationThread { //Singleton - only one object should be instantiated!
private:
	pthread_t m_communication_thread; //Will control all the brain tasks.
    static CommunicationThread* Communication_Thread_Instance; //Contains the one and only possible instance of brain thread.
	CommunicationThread();

public:
	pthread_t getCommunicationThread(); //Returns the brain_thread of type pthread_t class member.
	static CommunicationThread* GetCommunicationThreadInstance(); //This method makes sure we don't create more than 1 object of this class.
	void init(); //This method initiates the brain thread.
	virtual ~CommunicationThread();

};

#endif /*  COMMUNICATION_COMMUNICATIONTHREAD_H_ */

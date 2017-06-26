/*
 * CommunicationThread.cpp
 *
 */

#include "CommunicationThread.h"

	// Global static pointer used to ensure a single instance of the class:
	CommunicationThread* CommunicationThread::Communication_Thread_Instance = NULL;
CommunicationThread::CommunicationThread() {

}

CommunicationThread::~CommunicationThread() {
	// TODO Auto-generated destructor stub
}


void *runCommunication(void *arg)
{


	pthread_exit(NULL);
}

/*
 * Sets up a new thread - the vision thread.
 */
void CommunicationThread::init()
{
	static int NUM_INIT_CALLS=0; //This variable is used to check that the init() method is called only once!
	if(NUM_INIT_CALLS==0) //If it's first time init() is called:
	{
		NUM_INIT_CALLS++;
		int status = pthread_create(&m_communication_thread, NULL, runCommunication,  (void*) "communication thread");
		if(status) //If could not start a new thread - notify me:
		{
			cout<<"Error! Could not initiate the communication thread :("<<endl;
		}
		else
		{
			cout<<"*	Communication thread successfully initiated"<<endl;
		}
	}
}
/*
 * Returns the vision thread object.
 */
pthread_t CommunicationThread::getCommunicationThread()
{
	return this->m_communication_thread;
}

	/* This function is called to create an instance of the class.
	    Calling the constructor publicly is not allowed (it is private!).
	*/

CommunicationThread* CommunicationThread::GetCommunicationThreadInstance()
{
	if ( Communication_Thread_Instance==NULL)   // Allow only 1 instance of this class
		Communication_Thread_Instance = new CommunicationThread();
	return Communication_Thread_Instance;
}



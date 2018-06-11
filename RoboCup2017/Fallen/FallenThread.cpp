/*
 * fallenThread.cpp
 *
 */

#include "FallenThread.h"

	// Global static variables of fallenThread class:
	FallenThread* FallenThread::Fallen_Thread_Instance = NULL;
	bool FallenThread::Is_Register_Signals_Done=false;

FallenThread::FallenThread() {

}

FallenThread::~FallenThread() {
	// TODO Auto-generated destructor stub
}

void *runFallen(void *arg)
{
	//FallenThread::RegisterSignals();

	while(true)
	{
		FallenThread::GetFallenThreadInstance()->IsFallen();
	}

	pthread_exit(NULL);
}

/*
 * Sets up a new thread - the fallen thread.
 */
void FallenThread::init()
{
	static int NUM_INIT_CALLS=0; //This variable is used to check that the init() method is called only once!
	if(NUM_INIT_CALLS==0) //If it's first time init() is called:
	{
		NUM_INIT_CALLS++;
		int status = pthread_create(&m_fallen_thread, NULL, runFallen,  (void*) "fallen thread");
		if(status) //If could not start a new thread - notify me:
		{
			cout<<"Error! Could not initiate the fallen thread :("<<endl;
		}
		else
		{
			cout<<"*	Fallen thread successfully initiated"<<endl;
		}
	}
}

/* This function runs all the time when robot is ON and indicates
	when it fallen to brain thread
*/
void FallenThread::IsFallen()
{
	if (MotionStatus::FALLEN != STANDUP)
	{
		//First send signal to the brain thread - which will trigger the getUp() method.
		pthread_kill(BrainThread::GetBrainThreadInstance()->getBrainThread(),BrainThread::FALLEN_MESSAGE);
		//VisionThread::MillisSleep(7000);
		while(MotionStatus::FALLEN != STANDUP)	//Wait until get up process is done
			VisionThread::MillisSleep(1000);
	}
}

/*
 * Returns the fallen thread object.
 */
pthread_t FallenThread::getFallenThread()
{
	return this->m_fallen_thread;
}

/* This function is called to create an instance of the class.
	Calling the constructor publicly is not allowed (it is private!).
*/
FallenThread* FallenThread::GetFallenThreadInstance()
{
		   if ( Fallen_Thread_Instance==NULL)   // Allow only 1 instance of this class
			   Fallen_Thread_Instance = new FallenThread();
		   return Fallen_Thread_Instance;
}

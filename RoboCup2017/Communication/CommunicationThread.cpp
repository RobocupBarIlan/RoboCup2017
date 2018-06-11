#include "CommunicationThread.h"



CommunicationThread::CommunicationThread()
{
	
}


CommunicationThread::~CommunicationThread()
{
	
}
/*
* CommunicationThread.cpp
*
*/

#include "CommunicationThread.h"

CommunicationThread* CommunicationThread::Communication_Thread_Instance = NULL; // Global static pointer used to ensure a single instance of the class:
bool CommunicationThread::Is_Register_Signals_Done = false;
//std::mutex CommunicationThread::CommunicationMutex;

//std::atomic<bool> CommunicationThread::Is_Writing_Done(false);

void *runCommunication(void *arg)
{
	//UdpListener* mUdpListener =
	UdpListener::GetInstance();
	//CommunicationThread::RegisterSignals();
	//mUdpListener->Listen();
	while(true)
	{
		//cout<<"runCommunication"<<endl;
	}
	pthread_exit(NULL);
}

/*
* Sets up a new thread - the vision thread.
*/
void CommunicationThread::init()
{
	static int NUM_INIT_CALLS = 0; //This variable is used to check that the init() method is called only once!
	if (NUM_INIT_CALLS == 0) //If it's first time init() is called:
	{
		NUM_INIT_CALLS++;
		//Initialize static members of this class (data):

		int status = pthread_create(&m_communication_thread, NULL, runCommunication, (void*) "communication thread");
		if (status) //If could not start a new thread - notify me:
		{
			cout << "Error! Could not initiate the communication thread :(" << endl;
		}
		else
		{
			cout << "*	communication thread successfully initiated" << endl;
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
	if (Communication_Thread_Instance == NULL)   // Allow only 1 instance of this class
		Communication_Thread_Instance = new CommunicationThread();


	return Communication_Thread_Instance;
}

/*
//* Registers all possible calls to the vision thread for data:
//*/
//void CommunicationThread::RegisterSignals()
//{
//	// Register signals and signal handler:
//	signal(GET_BALL_CENTER_IN_FRAME_AND_DISTANCE, SignalCallbackHandler);
//	signal(GET_GOAL_CENTER_IN_FRAME_AND_DISTANCE, SignalCallbackHandler);
//	signal(GET_GOALKEEPER_CENTER_IN_FRAME_AND_DISTANCE, SignalCallbackHandler);
//	Is_Register_Signals_Done = true;
//}

//void CommunicationThread::SignalCallbackHandler(int signum)
//{
//	switch (signum)
//	{
//	case GET_BALL_CENTER_IN_FRAME_AND_DISTANCE:
//		CommunicationThread::GetBallCenterInFrameAndDistance();
//		break;
//	case GET_GOAL_CENTER_IN_FRAME_AND_DISTANCE:
//		break;
//	case GET_GOALKEEPER_CENTER_IN_FRAME_AND_DISTANCE:
//		break;
//	}
//}

//Check if signals were already registered.
bool CommunicationThread::IsRegisterSingalsDone()
{
	return Is_Register_Signals_Done;
}

//This method is called by the callback handler when another thread signaled the GET_BALL_CENTER_IN_FRAME_AND_DISTANCE signal.
//void CommunicationThread::GetBallCenterInFrameAndDistance()
//{
//	if (IS_NO_BALL_COMPUTATION) //Only if no other computation for ball is running - start a computation. This mechanism makes sure that the calls for computation won't be faster than the computation capability!
//	{
//		IS_NO_BALL_COMPUTATION = false; //Prevent another interrupt from running in between our computation.
//										//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//										//Critical code section - the set of ball center and distance values:
//		Point center;
//		center = BallDetector::GetBallCenter();
//		VisionMutex.lock();
//		BallCenterX = center.x;
//		BallCenterY = center.y;
//		//TODO - add the distance calculation function and then update here : BallDistance=distance.
//		VisionMutex.unlock();
//		IS_NO_BALL_COMPUTATION = true; //Enable a new computation for ball.
//									   //Is_Writing_Done=true; //Enable a safe read (when SafeReadBallCenterInFrameAndDistance will be called it will read the correct data).
//									   //$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//	}
//
//}


//void CommunicationThread::SafeReadBallCenterInFrameAndDistance(int& center_x, int& center_y, double& distance)
//{
//	pthread_kill(CommunicationThread::GetCommunicationThreadInstance()->getCommunicationThread(), CommunicationThread::GET_BALL_CENTER_IN_FRAME_AND_DISTANCE); //First send signal to the vision thread - which will trigger the GetBallCenterInFrameAndDistance() method.
//																																   //while(!Is_Writing_Done){}; //Wait until writing to variables done.
//																																   //$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//																																   //Critical code section - reading the data - must lock for data consistency:
//	VisionMutex.lock();
//	center_x = BallCenterX;
//	center_y = BallCenterY;
//	distance = 0;//TODO - add the distance calculation function and then update here : m_ball_distance=distance.
//				 //	Is_Writing_Done=false; //Disable safe read. Don't allow a reading before next write.
//	VisionMutex.unlock();
//	//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//}


//int  CommunicationThread::MillisSleep(long miliseconds)
//{
//	struct timespec req, rem;
//
//	if (miliseconds > 999)
//	{
//		req.tv_sec = (int)(miliseconds / 1000);                            /* Must be Non-Negative */
//		req.tv_nsec = (miliseconds - ((long)req.tv_sec * 1000)) * 1000000; /* Must be in range of 0 to 999999999 */
//	}
//	else
//	{
//		req.tv_sec = 0;                         /* Must be Non-Negative */
//		req.tv_nsec = miliseconds * 1000000;    /* Must be in range of 0 to 999999999 */
//	}
//
//	return nanosleep(&req, &rem);
//}

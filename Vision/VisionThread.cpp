/*
 * VisionThread.cpp
 *
 */

#include "VisionThread.h"

	VisionThread* VisionThread::Vision_Thread_Instance = NULL; // Global static pointer used to ensure a single instance of the class:
	bool VisionThread::Is_Register_Signals_Done=false;
	std::mutex VisionThread::WriteDetectedDataMutex;
	std::mutex VisionThread::FrameReadWriteMutex;
	int VisionThread::BallCenterX;
	int VisionThread::BallCenterY;
	double VisionThread::BallDistance;
	std::atomic<bool> VisionThread::Is_Ball_Writing_Done(false);
	std::atomic<bool> VisionThread::Is_Goal_Writing_Done(false);
	std::atomic<bool> VisionThread::IS_NO_BALL_COMPUTATION(true);
	std::atomic<bool> VisionThread::IS_NO_GOAL_COMPUTATION(true);
	Mat VisionThread::Frame=Mat::zeros(405,720,CV_8UC3);
	std::atomic<bool> VisionThread::IS_PROCCESSING_IMAGE(false);
	std::atomic<bool> VisionThread::IS_READING_FRAME(false);
	GoalCandidate VisionThread::DetectedGoalCandidate;
VisionThread::VisionThread() {

}

VisionThread::~VisionThread() {
	// TODO Auto-generated destructor stub
}

void *runVideoCapture(void *arg)
{
	VideoCapture cap; // open the default camera
	cap.open(0);
	cap.set(CV_CAP_PROP_FPS,50);
	cap.set(cv::CAP_PROP_FRAME_WIDTH, 720);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, 405);
	if (!cap.isOpened())
	{
		cout<<"Could not open VideoCapture"<<endl;
		pthread_exit(NULL);
	}

	Mat cleaning_frame;
	while(true)
	{
		VisionThread::MillisSleep(20);
		while(VisionThread::IS_READING_FRAME)
		{
			cap>>cleaning_frame;
		}
		//do{VisionThread::MillisSleep(1);}while(VisionThread::IS_PROCCESSING_IMAGE);  //prevents the camera capturing from taking resources while any image proccessing is done. also lets another waiting thread (the vision thread) time for taking the mutex lock (preventing starvation).
		VisionThread::FrameReadWriteMutex.lock();
			cap>>VisionThread::GetVisionThreadInstance()->Frame; // get a new frame from camera
		VisionThread::FrameReadWriteMutex.unlock();

	}

	cap.release();
	pthread_exit(NULL);
}



void *runVision(void *arg)
{
	VisionThread::RegisterSignals();
	pthread_t video_capture_thread;
	int status = pthread_create(&video_capture_thread, NULL, runVideoCapture,  (void*) "video capture thread");
	while(true)
	{
		VisionThread::MillisSleep(1000*60); //Sleep for an arbitrary large enough time - here we set it to 1minute.
	}

	pthread_exit(NULL);
}

/*
 * Sets up a new thread - the vision thread.
 */
void VisionThread::init()
{
	static int NUM_INIT_CALLS=0; //This variable is used to check that the init() method is called only once!
	if(NUM_INIT_CALLS==0) //If it's first time init() is called:
	{
		NUM_INIT_CALLS++;
		//Initialize static members of this class (data):
		BallCenterX=INIT_VALUE;
		BallCenterY=INIT_VALUE;
		BallDistance=INIT_VALUE;
		int status = pthread_create(&m_vision_thread, NULL, runVision,  (void*) "vision thread");
		if(status) //If could not start a new thread - notify me:
		{
			cout<<"Error! Could not initiate the vision thread :("<<endl;
		}
		else
		{
			cout<<"*	Vision thread successfully initiated"<<endl;
		}
	}
}
/*
 * Returns the vision thread object.
 */
pthread_t VisionThread::getVisionThread()
{
	return this->m_vision_thread;
}

	/* This function is called to create an instance of the class.
	    Calling the constructor publicly is not allowed (it is private!).
	*/

VisionThread* VisionThread::GetVisionThreadInstance()
{
		   if ( Vision_Thread_Instance==NULL)   // Allow only 1 instance of this class
			   Vision_Thread_Instance = new VisionThread();


		   return Vision_Thread_Instance;
}

/*
 * Registers all possible calls to the vision thread for data:
 */
void VisionThread::RegisterSignals()
{
	   // Register signals and signal handler:
	   signal(GET_BALL_CENTER_IN_FRAME_AND_DISTANCE , SignalCallbackHandler);
	   signal(GET_GOAL_IN_FRAME , SignalCallbackHandler);
	   signal(GET_GOALKEEPER_CENTER_IN_FRAME_AND_DISTANCE , SignalCallbackHandler);
	   Is_Register_Signals_Done=true;
}

void VisionThread::SignalCallbackHandler(int signum)
{

	switch(signum)
	{
		case GET_BALL_CENTER_IN_FRAME_AND_DISTANCE:
				VisionThread::GetBallCenterInFrameAndDistance();
				break;
		case GET_GOAL_IN_FRAME:
				VisionThread::GetGoalCandidate();
			break;
		case GET_GOALKEEPER_CENTER_IN_FRAME_AND_DISTANCE:
			break;
	}
}

//Check if signals were already registered.
bool VisionThread::IsRegisterSingalsDone()
{
	return Is_Register_Signals_Done;
}

void VisionThread::GetGoalCandidate()
{

	if(IS_NO_GOAL_COMPUTATION)
	{
		IS_NO_GOAL_COMPUTATION=false;
		GoalCandidate gc;
		GoalDetector::GetGoalPosts(gc);
		WriteDetectedDataMutex.lock();
			VisionThread::DetectedGoalCandidate.m_left_post[0]=gc.m_left_post[0];
			VisionThread::DetectedGoalCandidate.m_left_post[1]=gc.m_left_post[1];
			VisionThread::DetectedGoalCandidate.m_right_post[0]=gc.m_right_post[0];
			VisionThread::DetectedGoalCandidate.m_right_post[1]=gc.m_right_post[1];
			VisionThread::DetectedGoalCandidate.m_width_left=abs(gc.m_left_post[1].x-gc.m_left_post[0].x);
			VisionThread::DetectedGoalCandidate.m_width_left=abs(gc.m_right_post[1].x-gc.m_right_post[0].x);
		WriteDetectedDataMutex.unlock();
		IS_NO_GOAL_COMPUTATION=true;
		Is_Goal_Writing_Done=true; //Enable a safe read (when SafeReadGoalCandidate will be called it will read the correct data).
	}
}


//This method is called by the callback handler when another thread signaled the GET_BALL_CENTER_IN_FRAME_AND_DISTANCE signal.
void VisionThread::GetBallCenterInFrameAndDistance()
{
	if(IS_NO_BALL_COMPUTATION) //Only if no other computation for ball is running - start a computation. This mechanism makes sure that the calls for computation won't be faster than the computation capability!
	{
		IS_NO_BALL_COMPUTATION=false; //Prevent another interrupt from running in between our computation.
		//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
		//Critical code section - the set of ball center and distance values:
		Point center;
		int radius;
		BallDetector::GetBallCenter(center,radius);

		WriteDetectedDataMutex.lock();
			BallCenterX=center.x;
			BallCenterY=center.y;
			BallDetector::CalculateDistanceToBall(radius,BallDistance);
			//TODO - add the distance calculation function and then update here : BallDistance=distance.
		WriteDetectedDataMutex.unlock();
		IS_NO_BALL_COMPUTATION=true; //Enable a new computation for ball.
		Is_Ball_Writing_Done=true; //Enable a safe read (when SafeReadBallCenterInFrameAndDistance will be called it will read the correct data).
		//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
	}

}


void VisionThread::SafeReadBallCenterInFrameAndDistance(int& center_x,int& center_y,double& distance)
{
	IS_PROCCESSING_IMAGE=true; //Must be added so the camera won't capture in parallel to us processing the previous frame.
	pthread_kill(VisionThread::GetVisionThreadInstance()->getVisionThread(),VisionThread::GET_BALL_CENTER_IN_FRAME_AND_DISTANCE); //First send signal to the vision thread - which will trigger the GetBallCenterInFrameAndDistance() method.

	while(!Is_Ball_Writing_Done){}; //Wait until writing to variables done.

	//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
	//Critical code section - reading the data - must lock for data consistency:
	WriteDetectedDataMutex.lock();
		center_x=BallCenterX;
		center_y=BallCenterY;
		distance=BallDistance;
		//Change the values of the ball to not detected (so there won't be confusion later on):
		BallCenterX=NOT_FOUND_OBJECT_VALUE;
		BallCenterY=NOT_FOUND_OBJECT_VALUE;
		BallDistance=NOT_FOUND_OBJECT_VALUE;
		Is_Ball_Writing_Done=false; //Disable safe read. Don't allow a reading before next write.
	WriteDetectedDataMutex.unlock();
	IS_PROCCESSING_IMAGE=false;

	//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
}


void VisionThread::SafeReadGoalInFrame(GoalCandidate& gc)
{
	IS_PROCCESSING_IMAGE=true; //Must be added so the camera won't capture in parallel to us processing the previous frame.
	pthread_kill(VisionThread::GetVisionThreadInstance()->getVisionThread(),VisionThread::GET_GOAL_IN_FRAME); //First send signal to the vision thread - which will trigger the GetBallCenterInFrameAndDistance() method.


	while(!Is_Goal_Writing_Done){}; //Wait until writing to variables done.
	//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
	//Critical code section - reading the data - must lock for data consistency:
	WriteDetectedDataMutex.lock();
		gc.m_left_post[0]=VisionThread::DetectedGoalCandidate.m_left_post[0];
		gc.m_left_post[1]=VisionThread::DetectedGoalCandidate.m_left_post[1];
		gc.m_right_post[0]=VisionThread::DetectedGoalCandidate.m_right_post[0];
		gc.m_right_post[1]=VisionThread::DetectedGoalCandidate.m_right_post[1];
		gc.m_width_left=VisionThread::DetectedGoalCandidate.m_width_left;
		gc.m_width_right=VisionThread::DetectedGoalCandidate.m_width_right;

		//Change the values of the goal to not detected (so there won't be confusion later on):
		VisionThread::DetectedGoalCandidate.m_left_post[0]=Point(NOT_FOUND_OBJECT_VALUE,NOT_FOUND_OBJECT_VALUE);
		VisionThread::DetectedGoalCandidate.m_left_post[1]=Point(NOT_FOUND_OBJECT_VALUE,NOT_FOUND_OBJECT_VALUE);
		VisionThread::DetectedGoalCandidate.m_right_post[0]=Point(NOT_FOUND_OBJECT_VALUE,NOT_FOUND_OBJECT_VALUE);
		VisionThread::DetectedGoalCandidate.m_right_post[1]=Point(NOT_FOUND_OBJECT_VALUE,NOT_FOUND_OBJECT_VALUE);
		VisionThread::DetectedGoalCandidate.m_width_left=NOT_FOUND_OBJECT_VALUE;
		VisionThread::DetectedGoalCandidate.m_width_right=NOT_FOUND_OBJECT_VALUE;
		Is_Goal_Writing_Done=false; //Disable safe read. Don't allow a reading before next write.
	WriteDetectedDataMutex.unlock();
	IS_PROCCESSING_IMAGE=false;

	//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
}

int  VisionThread::MillisSleep(long miliseconds)
{
   struct timespec req, rem;

   if(miliseconds > 999)
   {
        req.tv_sec = (int)(miliseconds / 1000);                            /* Must be Non-Negative */
        req.tv_nsec = (miliseconds - ((long)req.tv_sec * 1000)) * 1000000; /* Must be in range of 0 to 999999999 */
   }
   else
   {
        req.tv_sec = 0;                         /* Must be Non-Negative */
        req.tv_nsec = miliseconds * 1000000;    /* Must be in range of 0 to 999999999 */
   }

   return nanosleep(&req , &rem);
}


void VisionThread::SafeReadeCapturedFrame(Mat& captured_frame)
{
	captured_frame=Mat::zeros(405,720,CV_8UC3);

    VisionThread::FrameReadWriteMutex.lock(); //An attempt for locking the mutex for reading the captured frame.
    IS_READING_FRAME=true;
    //cout<<"m_frame_size_rows"<<VisionThread::GetVisionThreadInstance()->m_frame.rows<<endl;
	if(!VisionThread::GetVisionThreadInstance()->Frame.empty())
	{
//		cout<<VisionThread::GetVisionThreadInstance()->Frame.rows<<" cols:"<<VisionThread::GetVisionThreadInstance()->Frame.cols<<endl;
		//flip(VisionThread::GetVisionThreadInstance()->Frame,VisionThread::GetVisionThreadInstance()->Frame,2);
		resize(VisionThread::GetVisionThreadInstance()->Frame,VisionThread::GetVisionThreadInstance()->Frame,Size(720,405),0,0);
		captured_frame=VisionThread::GetVisionThreadInstance()->Frame.clone();
	}
	IS_READING_FRAME=false;
	VisionThread::FrameReadWriteMutex.unlock();

}

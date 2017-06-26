/*
 * BrainThread.cpp
 *
 */

#include "BrainThread.h"

	// Global static pointer used to ensure a single instance of the class:
	BrainThread* BrainThread::Brain_Thread_Instance = NULL;
BrainThread::BrainThread() {
	m_state_name = START_STATE ;
	m_Motion = Motion::GetInstance();
}

BrainThread::~BrainThread() {
	// TODO Auto-generated destructor stub
}


void *runBrain(void *arg)
{
	Motion* motion = BrainThread::GetBrainThreadInstance()->getMotion();
	motion->FreeAllEngines();
	motion->StartEngines();

	cout<<"check"<<endl;
	int center_x, center_y;
	double distance;
	VisionThread::SafeReadBallCenterInFrameAndDistance(center_x,center_y,distance);

//	BrainThread::GetBrainThreadInstance()->centerBall();
	BrainThread::GetBrainThreadInstance()->lookForBall();
//	BrainThread::GetBrainThreadInstance()->GoToBall();

//	usleep(1000000);
//	//BrainThread::GetBrainThreadInstance()->changeSpot();
//	while(true)	//To change "while(game not finish) by referee order"
//	{
//
//		//Getting data from the vision thread example:
//		int center_x;
//		int center_y;
//		double distance;
//		//VisionThread::SafeReadBallCenterInFrameAndDistance(center_x,center_y,distance);
//		//cout<<"center.x: "<<center_x<<"center.y: "<<center_y<<endl;
//		switch (BrainThread::GetBrainThreadInstance()->getState())
//		{
//			case START_STATE:
//				BrainThread::GetBrainThreadInstance()->start();
//				break;
//			case LOOK_FOR_BALL_STATE:
//				BrainThread::GetBrainThreadInstance()->lookForBall();
//				break;
//			case WALK_TO_BALL_STATE:
//				BrainThread::GetBrainThreadInstance()->walkToBall();
//				break;
//			case LOOK_FOR_GOAL_STATE:
//				BrainThread::GetBrainThreadInstance()->lookForGoal();
//				break;
//			case KICK_STATE:
//				BrainThread::GetBrainThreadInstance()->kick();
//				break;
//			case CHANGE_SPOT_STATE:
//				BrainThread::GetBrainThreadInstance()->changeSpot();
//		}
//	}
		//usleep(100000000);

	pthread_exit(NULL);
}

/*
 * Sets up a new thread - the vision thread.
 */
void BrainThread::init()
{
	static int NUM_INIT_CALLS=0; //This variable is used to check that the init() method is called only once!
	if(NUM_INIT_CALLS==0) //If it's first time init() is called:
	{
		NUM_INIT_CALLS++;
		int status = pthread_create(&m_brain_thread, NULL, runBrain,  (void*) "brain thread");
		if(status) //If could not start a new thread - notify me:
		{
			cout<<"Error! Could not initiate the brain thread :("<<endl;
		}
		else
		{
			cout<<"*	Brain thread successfully initiated"<<endl;
		}
	}
}
/*
 * Returns the vision thread object.
 */
pthread_t BrainThread::getBrainThread()
{
	return this->m_brain_thread;
}

	/* This function is called to create an instance of the class.
	    Calling the constructor publicly is not allowed (it is private!).
	*/

BrainThread* BrainThread::GetBrainThreadInstance()
{
	if ( Brain_Thread_Instance==NULL)   // Allow only 1 instance of this class
		Brain_Thread_Instance = new BrainThread();
	return Brain_Thread_Instance;
}

void BrainThread::setState(int new_state)
{
	m_state_name = new_state;
}

int BrainThread::getState()
{
	return m_state_name;
}

Motion* BrainThread::getMotion()
{
	return m_Motion;
}

void BrainThread::start()
{

}

void BrainThread::lookForBall()
{
	float pan = PANMAXRIGHT, tilt = -32;
	Motion* motion = BrainThread::GetBrainThreadInstance()->getMotion();
	motion->SetHeadTilt(HeadTilt(tilt,pan));
	VisionThread::MillisSleep(100);
	bool going_left = true, finish_scan = false;
	int center_x, center_y;
	double distance;
//	while (1)
//	{
//		tilt = tilt+1;
//		motion->SetHeadTilt(HeadTilt(tilt,pan));
//		VisionThread::SafeReadBallCenterInFrameAndDistance(center_x,center_y,distance);
//		cout<<"center.x: "<<center_x<<"center.y: "<<center_y<<endl;
//		//usleep(1000000*5);
//
//	}
	//VisionThread::SafeReadBallCenterInFrameAndDistance(center_x,center_y,distance);
	while(1)//center_x == -1 && !finish_scan) // ball wasnt found
	{
		if (going_left)
		{
			tilt = -32 + 22*sin(pan*PI/PANMAXRIGHT);
			pan += 6;
			if (pan == PANMAXLEFT)
				going_left = false;
		}
		else
		{
			tilt = -32 - 22*sin(pan*PI/PANMAXRIGHT);
			pan -= 6;
			if (pan == PANMAXRIGHT)
			{
				going_left = true;
				finish_scan = true;
			}
		}
		motion->SetHeadTilt(HeadTilt(tilt,pan));

		//VisionThread::MillisSleep(5);
		VisionThread::SafeReadBallCenterInFrameAndDistance(center_x,center_y,distance);
	}
	if (center_x != -1)
		BrainThread::GetBrainThreadInstance()->setState(WALK_TO_BALL_STATE);
	else
		BrainThread::GetBrainThreadInstance()->setState(CHANGE_SPOT_STATE);
	//motion->FreeAllEngines();
}

void BrainThread::changeSpot()
{
		cout << "changed spot" << endl;
		int angle = 45;
		Motion* motion = BrainThread::GetBrainThreadInstance()->getMotion();
		//turn 45 deg to the left
		motion->StartWalking(-5,0,24);
		//usleep(1388.89*angle*24);
		//motion->StopWalking();
		//BrainThread::GetBrainThreadInstance()->setState(LOOK_FOR_BALL_STATE);
}

void BrainThread::GoToBall()
{
		Motion* motion = BrainThread::GetBrainThreadInstance()->getMotion();	//Pointer to motion class member
		int center_x;
		int center_y;
		double lastDistance=0;
		//VisionThread::SafeReadBallCenterInFrameAndDistance(center_x,center_y,lastDistance);	//Get vision variables

		double pan = PANMAXRIGHT, tilt = 0;	//Delete this part when the VISION is back
		m_Motion->SetHeadTilt(HeadTilt(tilt,pan));

		double angleToBall = m_Motion->GetHeadTilt().Pan;	//Get head position (pan)
		//double angleToBall = 60;
		cout<<"pan is " << angleToBall << endl;
		m_Motion->TurnByAngle(angleToBall); //turn to ball
		m_Motion->StartWalking(8, 0, 0);
		//motion->StartWalking(-5, 5,-12);
		//motion->StartWalking(0, 0,-25);
		double currentDistance = lastDistance;
		/*while (currentDistance>0 && MotionStatus::FALLEN==STANDUP)	//As long as didn't arrive to ball and didn't fall
		{
			VisionThread::SafeReadBallCenterInFrameAndDistance(center_x,center_y,currentDistance);
			if (lastDistance != currentDistance)
			{+
				lastDistance = currentDistance;
				cout << "Distance from ball is " << currentDistance << endl;
			}
		}
		if (currentDistance<=0)
		{*/
		VisionThread::MillisSleep(3000);
		m_Motion->StopWalking();
		cout<<"end"<<endl;
		//}
		//motion->FreeAllEngines();

}

void BrainThread::lookForGoal()
{

}

void BrainThread::kick()
{

}

void BrainThread::centerBall()
{
	while(1)
	{
		Motion* motion = BrainThread::GetBrainThreadInstance()->getMotion();
		int center_x, center_y;
		double distance;
		float tilt = motion->GetHeadTilt().Tilt;
		float pan =	motion->GetHeadTilt().Pan;
		VisionThread::SafeReadBallCenterInFrameAndDistance(center_x,center_y,distance);
		if (center_x > 337)
			pan -= (center_x - 320)/PANMOVEMENT;
		if (0 < center_x < 303)
			pan += (320 - center_x)/PANMOVEMENT;
		if (center_y > 255)
			tilt += (center_y - 255)/TILTMOVEMENT;
		if (0 <center_y < 225)
			tilt -= (center_y - 225)/TILTMOVEMENT;
		if (center_x != -1 && center_y != -1)
			motion->SetHeadTilt(HeadTilt(tilt,pan));
		usleep(100000);
		VisionThread::SafeReadBallCenterInFrameAndDistance(center_x,center_y,distance);
		cout<<"center.x: "<<center_x<<"center.y: "<<center_y<<endl;
		usleep(1000000*5);
	}
}



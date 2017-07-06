/*
 * BrainThread.cpp
 *
 */

#include "BrainThread.h"
	// Global static pointer used to ensure a single instance of the class:
	bool BrainThread::Is_Register_Signals_Done=false;
	BrainThread* BrainThread::Brain_Thread_Instance = NULL;

	BrainThread::BrainThread()
	{
		m_state_name = START_STATE ;
		m_Motion = Motion::GetInstance();
	}

BrainThread::~BrainThread() {
	// TODO Auto-generated destructor stub
}

void BrainThread::RegisterSignals()
{
	//cout<<"BrainThread::RegisterSignals()"<<endl;
	signal(NEW_REFEREE_MESSAGE, SignalCallbackHandler);
	signal(TEAM_INFO_MESSAGE, SignalCallbackHandler);
	signal(PLAYER_INFO_MESSAGE, SignalCallbackHandler);
	signal(FALLEN_MESSAGE, SignalCallbackHandler);
	Is_Register_Signals_Done = true;
}

bool BrainThread::IsRegisterSingalsDone()
{
	return BrainThread::Is_Register_Signals_Done;
}

void BrainThread::SignalCallbackHandler(int signum)
{
	//cout<<"BrainThread::SignalCallbackHandler"<<endl;
	switch (signum)
	{
		case TEAM_INFO_MESSAGE:
				//TO DO- method that take care of referee message/data
			break;
		case PLAYER_INFO_MESSAGE:
				//TO DO- method that take care of referee message/data
			break;
		case NEW_REFEREE_MESSAGE:
				cout<<"NEW_REFEREE_MESSAGE"<<endl;
				//BrainThread::HandleRefereeMessage();
			break;
		case FALLEN_MESSAGE:	//In case fallen signal received, starts get up process
				cout<<"FALLEN_MESSAGE"<<endl;
				Motion* motion = BrainThread::GetBrainThreadInstance()->getMotion();
				while(MotionStatus::FALLEN != STANDUP)	//If stand up didn't succeeded-> wait until it's done
					motion->GetUp();
			break;
	}
}

void *runBrain(void *arg)
{
	BrainThread::RegisterSignals();
	Motion* motion = BrainThread::GetBrainThreadInstance()->getMotion();

	//motion->FreeAllEngines();
	motion->StartEngines();
	VisionThread::MillisSleep(2000);
	cout<<"StartEngines-> done"<<endl;

	int center_x, center_y;
	double distance;
	//Must calibrate the ball before first run!!!:
	motion->SetHeadTilt(HeadTilt(-3.000,-0.176));
	VisionThread::MillisSleep(2000);
	VisionThread::SafeReadBallCenterInFrameAndDistance(center_x,center_y,distance);

	/***Initiating the fallen thread***/
	//FallenThread::GetFallenThreadInstance()->init();

	BrainThread::GetBrainThreadInstance()->lookForBall();
//	BrainThread::GetBrainThreadInstance()->centerBall();
//	BrainThread::GetBrainThreadInstance()->lookForBall();
//	BrainThread::GetBrainThreadInstance()->GoToBall();

//	if (PlayerInfo.isGoalkeeper)
//	{
//		cout << "Goalkeeper mode selected"<<endl;
//		BrainThread::GoalKeeperStateMachine();
//	}
//	else
//	{
//		BrainThread::StateMachine();
//	}


	pthread_exit(NULL);
}

/*
 * Sets up a new thread - the brain thread.
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
 * Returns the brain thread object.
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
	if (Brain_Thread_Instance==NULL)   // Allow only 1 instance of this class
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

void BrainThread::HandleRefereeMessage()
{
	int referee_message = UdpListener::new_info.messageNumber;
	switch(referee_message)
	{
		case (UdpListener::RefereeInfo::State_Initial):

				BrainThread::GetBrainThreadInstance()->start();
			break;
		case (UdpListener::RefereeInfo::State_Ready):

			break;
		case (UdpListener::RefereeInfo::State_Playing):
				BrainThread::GetBrainThreadInstance()->lookForBall();

			break;
		case (UdpListener::RefereeInfo::State_Finished):
				BrainThread::GetBrainThreadInstance()->setState(FINISHED_STATE);
			break;
	}
}

void BrainThread::StateMachine()
{
//	while(BrainThread::getState()!=STATE_FINISHED)	//To change "while(game not finish) by referee order"
//	{
//		switch (BrainThread::GetBrainThreadInstance()->getState())
//		{
//			case START_STATE:
//				BrainThread::GetBrainThreadInstance()->start();
//				break;
//			case LOOK_FOR_BALL_STATE:
//				BrainThread::GetBrainThreadInstance()->lookForBall();
//				break;
//			case GO_TO_BALL_STATE:
//				BrainThread::GetBrainThreadInstance()->goToBall();
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
}
void BrainThread::start()
{

}

void BrainThread::lookForBall()
{
	{
		float pan = PAN_MAX_RIGHT, tilt = -15;
		Motion* motion = BrainThread::GetBrainThreadInstance()->getMotion();
		motion->SetHeadTilt(HeadTilt(tilt,pan));
		VisionThread::MillisSleep(1000);
		bool going_left = true, finish_scan = false;
		int center_x, center_y;
		double distance;

		VisionThread::SafeReadBallCenterInFrameAndDistance(center_x,center_y,distance);
		while(center_x == -1)//center_x == -1 && !finish_scan) // ball wasn't found
		{
			if (going_left)
			{
				tilt = -10 + 17*sin(pan*PI/PAN_MAX_RIGHT);
				if (tilt < -15)
					pan +=1;
				else
					pan += 6;
				if (pan >= PAN_MAX_LEFT)
				{
					going_left = false;
					pan = PAN_MAX_LEFT;
				}
			}
			else
			{
				tilt = -10 - 17*sin(pan*PI/PAN_MAX_LEFT);
				if (tilt < -15)
					pan -= 1;
				else
					pan -= 6;
				if (pan <= PAN_MAX_RIGHT)
				{
					pan = PAN_MAX_RIGHT;
					going_left = true;
					finish_scan = true;
				}
			}
			motion->SetHeadTilt(HeadTilt(tilt,pan));
			VisionThread::MillisSleep(25);
			VisionThread::SafeReadBallCenterInFrameAndDistance(center_x,center_y,distance);
			cout<<"center.x: "<<center_x<<"center.y: "<<center_y<<endl;
			if (center_x != -1) // double check
			{
				VisionThread::MillisSleep(25);
				VisionThread::SafeReadBallCenterInFrameAndDistance(center_x,center_y,distance);
			}
		}
		if (center_x != -1)
		{
			cout<<"BallFound"<<endl;
			BrainThread::GetBrainThreadInstance()->centerBall();
			BrainThread::GetBrainThreadInstance()->setState(GO_TO_BALL_STATE);
		}
		else
		{
			cout<<"Ball not found ->change spot"<<endl;
			//BrainThread::GetBrainThreadInstance()->setState(CHANGE_SPOT_STATE);
		//motion->FreeAllEngines();
		}
	}
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
		//int center_x, center_y;
		double lastDistance=0;
		//VisionThread::SafeReadBallCenterInFrameAndDistance(center_x,center_y,lastDistance);	//Get vision variables

		//double pan = PAN_MAX_RIGHT, tilt = 0;	//Delete this part when the VISION is back
		//m_Motion->SetHeadTilt(HeadTilt(tilt,pan));

		double angleToBall = motion->GetHeadTilt().Pan;	//Get head position (pan)
		//double angleToBall = 60;
		cout<<"pan is " << angleToBall << endl;
		motion->TurnByAngle(angleToBall); //turn to ball
		//motion->StartWalking(8, 0, 0);
		//motion->StartWalking(-5, 5,-12);
		//motion->StartWalking(0, 0,-25);
		//double currentDistance = lastDistance;
		/*while (currentDistance>0)	//As long as didn't arrive to ball
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
		//motion->StopWalking();
		cout<<"end"<<endl;
		//}
		//motion->FreeAllEngines();

}

void BrainThread::lookForGoal()
{

}

void BrainThread::kick()
{
	//Motion* motion = BrainThread::GetBrainThreadInstance()->getMotion();
	//motion->RunAction(ActionPage::RightKick);
}

void BrainThread::centerBall()
{
	int center_x, center_y, last_center_x, last_center_y, two_last_center_x, two_last_center_y;
	double distance;
	Motion* motion = BrainThread::GetBrainThreadInstance()->getMotion();
	bool delay_flag = false, visited = false;
	cout<< "Starting center ball()" <<endl;
	float tilt = motion->GetHeadTilt().Tilt;
	float pan =	motion->GetHeadTilt().Pan;
	cout<<"Tilt: "<<tilt<<" Pan: "<<pan<<endl;
	VisionThread::SafeReadBallCenterInFrameAndDistance(center_x,center_y,distance);
	cout<<"center.x: "<<center_x<<"center.y: "<<center_y<<endl;
	last_center_x = center_x;
	last_center_y = center_y;
	two_last_center_x = center_x;   //just for the first time
	two_last_center_y = center_y;
	while (center_x < 345 || center_x > 375 || center_y < 187 || center_y > 217)
	{
		const double WIDTH_HEIGHT_FRAME_RATIO=16/9;
		//pan -= 0.015645*center_x-5.632411;
		//tilt += -0.048859*center_y+9.893952;
		tilt -= WIDTH_HEIGHT_FRAME_RATIO*2.2*(center_y - 202)/101.0;
		pan -= 2.2*(center_x - 360)/180.0;
		if(pan<PAN_MAX_RIGHT)
			pan=PAN_MAX_RIGHT;
		if(pan>PAN_MAX_LEFT)
			pan=PAN_MAX_LEFT;
		if(tilt<TILT_MIN)
			tilt=TILT_MIN;
		if(tilt>TILT_MAX)
			tilt=TILT_MAX;
		if (center_x != -1 && center_y != -1)
			motion->SetHeadTilt(HeadTilt(tilt,pan));
		cout<<"Tilt: "<<tilt<<" Pan: "<<pan<<endl;
		//VisionThread::MillisSleep(75);
		VisionThread::SafeReadBallCenterInFrameAndDistance(center_x,center_y,distance);
		cout<<"center.x: "<<center_x<<"center.y: "<<center_y<<endl;
	}
	cout <<"The ball is centered"<<endl;
//	int center_x, center_y;
//	double distance;
//	Motion* motion = BrainThread::GetBrainThreadInstance()->getMotion();
//	motion->SetHeadTilt(HeadTilt(-10.000,-0.176));
//	cout<<"sethead"<<endl;
//	VisionThread::MillisSleep(5000);
//	while(1)
//	{
//		float tilt = motion->GetHeadTilt().Tilt;
//		float pan =	motion->GetHeadTilt().Pan;
//		cout<<"Tilt: "<<tilt<<" Pan: "<<pan<<endl;
//		VisionThread::SafeReadBallCenterInFrameAndDistance(center_x,center_y,distance);
//		cout<<"center.x: "<<center_x<<"center.y: "<<center_y<<endl;
//		VisionThread::MillisSleep(2000);
//		while (center_x < 290 || center_x > 350 || center_y < 210 || center_x > 270)
//		{
//			cout <<"check1->centerBall()"<<endl;
//			if (center_x > 337)
//				pan += (center_x - 320)/PAN_MOVEMENT;
//			if (0 < center_x && center_x < 303)
//				pan -= (320 - center_x)/PAN_MOVEMENT;
//			if (center_y > 255)
//				tilt -= (center_y - 255)/TILT_MOVEMENT;
//			if (0 < center_y &&  center_y < 225)
//				tilt += (225 - center_y)/TILT_MOVEMENT;
//			if (center_x != -1 && center_y != -1)
//				motion->SetHeadTilt(HeadTilt(tilt,pan));
//			cout<<"Tilt: "<<tilt<<" Pan: "<<pan<<endl;
//			VisionThread::MillisSleep(2000);
//			VisionThread::SafeReadBallCenterInFrameAndDistance(center_x,center_y,distance);
//			cout<<"center.x: "<<center_x<<"center.y: "<<center_y<<endl;
//		}
//		cout <<"check3"<<endl;
//
//		cout<<"center.x: "<<center_x<<"center.y: "<<center_y<<endl;
//		VisionThread::MillisSleep(1000);
//	}
}

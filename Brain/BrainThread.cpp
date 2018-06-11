/*
 * BrainThread.cpp
 *
 */

#include "BrainThread.h"

	// Global static variables of brainThread class:
	bool BrainThread::Is_Register_Signals_Done(false);
	std::mutex BrainThread::WriteDetectedDataMutexBrain;
	std::mutex BrainThread::WriteDetectedDataMutexBrainCenter;
	std::atomic<bool> BrainThread::Is_Ball_Writing_Done_Brain(false);
	int BrainThread::m_center_x;
	int BrainThread::m_center_y;
	int BrainThread::m_continue_center_thread;
	BrainThread* BrainThread::Brain_Thread_Instance = NULL;
	void *runCenterBall(void *arg);

BrainThread::BrainThread() {
	m_state_name = LOOK_FOR_BALL_STATE;
	m_state_name = WALK_TO_BALL_STATE;
	m_continue_center_thread = 0;
	m_change_spot_counter = 0;
	m_kick = 0;
	m_follower = BallFollower();
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
				//cout<<int(new_info.messageNumber)<<endl;
				cout<<"NEW_REFEREE_MESSAGE"<<endl;
			break;
		case FALLEN_MESSAGE:	//In case fallen signal received, starts get up process
				cout<<"FALLEN_MESSAGE"<<endl;
				Motion* motion = BrainThread::GetBrainThreadInstance()->getMotion();
				//while(MotionStatus::FALLEN != STANDUP)	//If stand up didn't succeeded-> wait until it's done
				//{
					motion->GetUp();
					//GetBrainThreadInstance()->setState(LOOK_FOR_BALL_STATE);	//After getUp from falling, return to LOOK_FOR_BALL_STATE
				//}
					break;
	}
}

void *runBrain(void *arg)
{
	BrainThread::RegisterSignals();
	Motion* motion = BrainThread::GetBrainThreadInstance()->getMotion();
	motion->StartEngines();
	VisionThread::MillisSleep(3000);
	cout<<"StartEngines-> done"<<endl;

	int center_x, center_y;
	double distance;
	//Must calibrate the ball before first run!!!:
	motion->SetHeadTilt(HeadTilt(-3.000,0.000));
	VisionThread::MillisSleep(2000);
	VisionThread::SafeReadBallCenterInFrameAndDistance(center_x,center_y,distance);

	/***Initiating the fallen thread***/
	FallenThread::GetFallenThreadInstance()->init();

	if (!BrainThread::GetBrainThreadInstance()->centerBall_thread_open)
	{
		int status = pthread_create(&BrainThread::GetBrainThreadInstance()->m_center_ball_thread, NULL, runCenterBall,  (void*) "center ball thread");
		if(status) //If could not start a new thread - notify me:
		{
			cout<<"Error! Could not initiate the Center_ball thread :("<<endl;
		}
		else
		{
			cout<<"*	Center_ball thread successfully initiated"<<endl;
		}
		BrainThread::GetBrainThreadInstance()->centerBall_thread_open = true;
	}

//	BrainThread::GetBrainThreadInstance()->lookForBall();
//	BrainThread::GetBrainThreadInstance()->centerBall();
//	BrainThread::GetBrainThreadInstance()->lookForBall();
//	BrainThread::GetBrainThreadInstance()->GoToBall();
//	BrainThread::GetBrainThreadInstance()->followBall();
//	BrainThread::GetBrainThreadInstance()->kick();
//	BrainThread::GetBrainThreadInstance()->changeSpot();

	//while(BrainThread::GetBrainThreadInstance()->getState()!=FINISHED_STATE)	//Stay here until game over
	while(true)	//To change "while(game not finish) by referee order"
	{
		switch (BrainThread::GetBrainThreadInstance()->getState())
		{
			case START_STATE:
				BrainThread::GetBrainThreadInstance()->start();
				break;
			case LOOK_FOR_BALL_STATE:
				BrainThread::GetBrainThreadInstance()->lookForBall();
				break;
			case WALK_TO_BALL_STATE:
				cout<<"starting followBall"<<endl;
				BrainThread::GetBrainThreadInstance()->followBall();
				break;
			case KICK_STATE:
				BrainThread::GetBrainThreadInstance()->kick();
				break;
			case CHANGE_SPOT_STATE:
				BrainThread::GetBrainThreadInstance()->changeSpot();
				break;
			case FINISHED_STATE:
				BrainThread::GetBrainThreadInstance()->finish();
				break;
		}
	}
	cout<<"end RunBrain"<<endl;
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
	if ( Brain_Thread_Instance==NULL)   // Allow only 1 instance of this class
		Brain_Thread_Instance = new BrainThread();
	return Brain_Thread_Instance;
}

void BrainThread::checkTiltPan(float tilt,float pan)
{
	Motion* motion = BrainThread::GetBrainThreadInstance()->getMotion();
	motion->StartEngines();
	cout<<"StartEngines-> done"<<endl;
	sleep(3);
	motion->TurnByAngle(90);
//	cout<<"StartEngines-> done"<<endl;
//	cout<<"Tilt: "<< motion->GetHeadTilt().Tilt<<" Pan: "<<motion->GetHeadTilt().Pan<<endl;
//	motion->SetHeadTilt(HeadTilt(tilt,pan));
//	cout<<"Tilt: "<< motion->GetHeadTilt().Tilt<<" Pan: "<<motion->GetHeadTilt().Pan<<endl;
}


/* This function is called only for followBall function
 * and used to center the ball all the time in different thread.
 * This centerBall thread is initiate from runBrain function.
*/

void *runCenterBall(void *arg)
{
	int center_x, center_y;
	double distance;
	Motion* motion = BrainThread::GetBrainThreadInstance()->getMotion();
	cout<< "Center ball thread" <<endl;
	float tilt = motion->GetHeadTilt().Tilt;
	float pan =	motion->GetHeadTilt().Pan;
	cout<<"Tilt: "<<tilt<<" Pan: "<<pan<<endl;

	int continue_center;
	bool flag = false;
	while (true)
	{
		BrainThread::WriteDetectedDataMutexBrainCenter.lock();
			continue_center = BrainThread::m_continue_center_thread;
		BrainThread::WriteDetectedDataMutexBrainCenter.unlock();
		if (continue_center)
		{
			cout<<"got inside"<<endl;
			if (flag)
			{
				cout<<"if got inside"<<endl;
				tilt = motion->GetHeadTilt().Tilt;
				pan = motion->GetHeadTilt().Pan;
				flag = false;
				//VisionThread::MillisSleep(500);
			}
			VisionThread::SafeReadBallCenterInFrameAndDistance(center_x,center_y,distance);
			cout<<"thread center.x: "<<center_x<<"thread center.y: "<<center_y<<endl;
			const double WIDTH_HEIGHT_FRAME_RATIO=16/9;
			tilt -= WIDTH_HEIGHT_FRAME_RATIO*2.2*(center_y - 202)/95.0;
			if (tilt > -30)
				pan -= 2.2*(center_x - 360)/120.0;
			else
				pan -= 2.2*(center_x - 360)/180.0;
			if(pan<PAN_MAX_RIGHT)
				pan=PAN_MAX_RIGHT;
			if(pan>PAN_MAX_LEFT)
				pan=PAN_MAX_LEFT;
			if(tilt<TILT_MIN)
				tilt=TILT_MIN;
			if(tilt>TILT_MAX)
				tilt=TILT_MAX;
			cout<<"center Tilt: "<<tilt<<" center Pan: "<<pan<<endl;
			if (center_x != -1 && center_y != -1)
				motion->SetHeadTilt(HeadTilt(tilt,pan));
			BrainThread::WriteDetectedDataMutexBrain.lock();
			BrainThread::m_center_x = center_x;
			BrainThread::m_center_y = center_y;
			BrainThread::WriteDetectedDataMutexBrain.unlock();
		}
		else
			flag = true;
	}
	pthread_exit(NULL);
}

void BrainThread::setState(int new_state)
{
	m_state_name = new_state;
}

int BrainThread::getState()
{
	return m_state_name;
}

void BrainThread::setKick(int new_kick)
{
	m_kick = new_kick;
}

int BrainThread::getKick()
{
	return m_kick;
}

Motion* BrainThread::getMotion()
{
	return m_Motion;
}

void BrainThread::start()
{

}

void BrainThread::finish()
{

}

void BrainThread::lookForBall()
{
	cout<<"Look for ball state"<<endl;
	float pan = PAN_MAX_RIGHT, tilt = -15;
	Motion* motion = GetBrainThreadInstance()->getMotion();
	motion->SetHeadTilt(HeadTilt(tilt,pan));
	VisionThread::MillisSleep(2000);
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
				pan +=2;
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
				pan -= 2;
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
		VisionThread::MillisSleep(100);
		VisionThread::SafeReadBallCenterInFrameAndDistance(center_x,center_y,distance);
		cout<<"center.x: "<<center_x<<"center.y: "<<center_y<<endl;
		if (center_x != -1) // double check
		{
			VisionThread::MillisSleep(1000);
			VisionThread::SafeReadBallCenterInFrameAndDistance(center_x,center_y,distance);
		}
	}
	if (center_x != -1)
	{
		cout<<"BallFound"<<endl;
		GetBrainThreadInstance()->centerBall();
		if (GetBrainThreadInstance()->getState() == LOOK_FOR_BALL_STATE)
		{
			GetBrainThreadInstance()->setState(WALK_TO_BALL_STATE);
			//VisionThread::MillisSleep(100);
		}
	}
	else
		cout<<"Ball not found ->change spot"<<endl;
		//BrainThread::GetBrainThreadInstance()->setState(CHANGE_SPOT_STATE);
	//motion->FreeAllEngines();
}

void BrainThread::changeSpot()
{
	cout << "changed spot" << endl;
	int angle = 45;
	Motion* motion = BrainThread::GetBrainThreadInstance()->getMotion();
	//turn 45 deg to the left
	if (m_change_spot_counter<3)
	{
		m_change_spot_counter++;
		motion->StartWalking(-5,0,24);
		usleep(1388.89*angle*24);
		motion->StopWalking();
	}
	else
	{
		m_change_spot_counter = 0;
		motion->StartWalking(5,0,0);
		usleep(1388.89*angle*24);
		motion->StopWalking();
	}
	GetBrainThreadInstance()->setState(LOOK_FOR_BALL_STATE);
}

void BrainThread::lookForGoal()
{

}

void BrainThread::kick()
{
	VisionThread::MillisSleep(500);
	Motion* motion = GetBrainThreadInstance()->getMotion();
	cout<<"kick function"<<endl;
	if (GetBrainThreadInstance()->getKick() == 1)
	{
		motion->RunAction(ActionPage::LeftKick);
	}
	else if (GetBrainThreadInstance()->getKick() == -1)
	{
		motion->RunAction(ActionPage::RightKick);
	}
	GetBrainThreadInstance()->setState(LOOK_FOR_BALL_STATE);
	VisionThread::MillisSleep(1500);
}

void BrainThread::centerBall()
{
	int center_x, center_y;
	double distance;
	Motion* motion = GetBrainThreadInstance()->getMotion();
	cout<< "Center ball" <<endl;
	float tilt = motion->GetHeadTilt().Tilt;
	float pan =	motion->GetHeadTilt().Pan;
	cout<<"Tilt: "<<tilt<<" Pan: "<<pan<<endl;
	VisionThread::SafeReadBallCenterInFrameAndDistance(center_x,center_y,distance);
	cout<<"center.x: "<<center_x<<"center.y: "<<center_y<<endl;
	int no_ball = 0;
	int ball_in_edge = 0;
	while (true)
	{
		const double WIDTH_HEIGHT_FRAME_RATIO=16/9;
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
		{
			motion->SetHeadTilt(HeadTilt(tilt,pan));
			cout<<"center did set head"<<endl;
		}
		cout<<"Tilt: "<<tilt<<" Pan: "<<pan<<endl;

		VisionThread::SafeReadBallCenterInFrameAndDistance(center_x,center_y,distance);
		cout<<"center.x: "<<center_x<<"center.y: "<<center_y<<endl;
		if (center_x == -1)
			no_ball++;
		else
			ball_in_edge++;
		if (((center_x > 345 && center_x < 375) && (center_y > 187 && center_y < 217)) || no_ball == 20 || ball_in_edge == 20)
		{
			Head::GetInstance()->m_Joint.SetEnableHeadOnly(false);
			if (no_ball == 50)
			{
				cout<<"We lost the ball"<<endl;
				GetBrainThreadInstance()->setState(LOOK_FOR_BALL_STATE);
			}
			else
			{
				cout <<"The ball is centered"<<endl;
			}
			return;
		}
	}
}

void BrainThread::followBall()
{
	WriteDetectedDataMutexBrainCenter.lock();
		m_continue_center_thread = 1;
	WriteDetectedDataMutexBrainCenter.unlock();
	m_follower.m_NoBallCount = 0;
	WriteDetectedDataMutexBrain.lock();
		double d_center_x = m_center_x;
		double d_center_y = m_center_y;
	WriteDetectedDataMutexBrain.unlock();
//	cout<<"starting long delay"<<endl;
//	VisionThread::MillisSleep(5000);
	cout<<"d_center_y: "<<d_center_x<<endl;
	Point2D ball_position(d_center_x , d_center_y);
	while (true)
	{
		cout<<"ball_position.X: "<<ball_position.X<<endl;
		m_follower.Process(ball_position);
		WriteDetectedDataMutexBrain.lock();
			ball_position.X = m_center_x;
			ball_position.Y = m_center_y;
		WriteDetectedDataMutexBrain.unlock();
		cout<<"m_NoBallCount: "<<m_follower.m_NoBallCount<<endl;
		if ((m_follower.m_NoBallCount > m_follower.m_NoBallMaxCount) && (Walking::GetInstance()->IsRunning() == false))
		{
			WriteDetectedDataMutexBrainCenter.lock();
					m_continue_center_thread = 0;
			WriteDetectedDataMutexBrainCenter.unlock();
			cout<<"setting look for ball state"<<endl;
			GetBrainThreadInstance()->setState(LOOK_FOR_BALL_STATE);
			VisionThread::MillisSleep(100);
			return;
		}
		if ((m_follower.KickBall == 1 || m_follower.KickBall == -1) && (Walking::GetInstance()->IsRunning() == false))
		{
			WriteDetectedDataMutexBrainCenter.lock();
					m_continue_center_thread = 0;
			WriteDetectedDataMutexBrainCenter.unlock();
			cout<<"kick"<<endl;
			GetBrainThreadInstance()->setKick(m_follower.KickBall);
			GetBrainThreadInstance()->setState(KICK_STATE);
			VisionThread::MillisSleep(400);
			return;
		}
	}
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
				BrainThread::GetBrainThreadInstance()->finish();
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

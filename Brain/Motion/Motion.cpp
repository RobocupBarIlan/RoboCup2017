#include "Motion.h"
#include "MotionIncludes.h"
#include "../../Brain/BrainThread.h"

Motion* Motion::m_instance = NULL;
Motion* Motion::GetInstance()
{
    if(m_instance == NULL)
    {
    	m_instance = new Motion();
        return m_instance;
    }
    else
    {
        return m_instance;
    }
}

void ChangeCurrentDir()
{
    char exepath[1024] = {0};
    if(readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
        chdir(dirname(exepath));
}

Motion::Motion()
{
}

LinuxArbotixPro linux_arbotixpro(U2D_DEV_NAME0);
ArbotixPro arbotixpro(&linux_arbotixpro);
LinuxMotionTimer linuxMotionTimer;

void Motion::StartEngines()
{
	printf("===== Action script for DARwIn =====\n\n");
	ChangeCurrentDir();
	minIni* ini = new minIni(INI_FILE_PATH);
	Action::GetInstance()->LoadFile(MOTION_FILE_PATH);

	//////////////////// Framework Initialize ////////////////////////////
	if (MotionManager::GetInstance()->Initialize(&arbotixpro) == false) {
		linux_arbotixpro.SetPortName(U2D_DEV_NAME1);
		if (MotionManager::GetInstance()->Initialize(&arbotixpro) == false) {
			printf("Fail to initialize Motion Manager!\n");
		}
	}
	Walking::GetInstance()->LoadINISettings(ini);
	usleep(100);
	MotionManager::GetInstance()->LoadINISettings(ini);

	MotionManager::GetInstance()->AddModule(
			(MotionModule*) Action::GetInstance());
	MotionManager::GetInstance()->AddModule(
			(MotionModule*) Head::GetInstance());
	MotionManager::GetInstance()->AddModule(
			(MotionModule*) Walking::GetInstance());

	linuxMotionTimer.Initialize(MotionManager::GetInstance());
	linuxMotionTimer.Start();
	/////////////////////////////////////////////////////////////////////

	Walking::GetInstance()->m_Joint.SetEnableBody(false);
	Head::GetInstance()->m_Joint.SetEnableBody(false);
	Action::GetInstance()->m_Joint.SetEnableBody(true);
	MotionManager::GetInstance()->SetEnable(true);

	MotionManager::GetInstance()->ResetGyroCalibration();

	Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
}

Motion::~Motion()
{

}

void Motion::WalkStraight(int cmToWalk)
{

}

void Motion::StartWalking()
{
	Walking::GetInstance()->m_Joint.SetEnableBody(false);
	//Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->X_MOVE_AMPLITUDE = 5;
	Walking::GetInstance()->Y_MOVE_AMPLITUDE = 0;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
	Walking::GetInstance()->Start();
}

void Motion::StartWalking(int x_amp, int y_amp, int a_amp)
{
	Walking::GetInstance()->m_Joint.SetEnableBody(false);
	//Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	Walking::GetInstance()->X_MOVE_AMPLITUDE = x_amp;
	Walking::GetInstance()->Y_MOVE_AMPLITUDE = y_amp;
	Walking::GetInstance()->A_MOVE_AMPLITUDE = a_amp;
	Walking::GetInstance()->Start();
}

void Motion::StartWalking(double distance)
{
	double timeToWalk = distance /(WALK_SPEED+0.0);
	StartWalking();
	usleep(timeToWalk*1000000); //convert sec to microsec
	StopWalking();
}

void Motion::StopWalking()
{
	Walking::GetInstance()->Stop();
	//Walking::GetInstance()->m_Joint.SetEnableBody(false);
	//Action::GetInstance()->m_Joint.SetEnableBody(true, true);
}

void Motion::GetUp()
{
	Walking::GetInstance()->Stop();
	while (Walking::GetInstance()->IsRunning() == 1) usleep(8000);

	Action::GetInstance()->m_Joint.SetEnableBody(true, true);

	if (MotionStatus::FALLEN == FORWARD)
	{
		VisionThread::MillisSleep(500);
		//RunAction(ActionPage::FrontGetup);   // FORWARD GETUP 70
		cout<<"Robot has fallen forward"<<endl;
	}
	else if (MotionStatus::FALLEN == BACKWARD)
	{
		VisionThread::MillisSleep(500);
		//RunAction(ActionPage::BackGetup);   // BACKWARD GETUP 75
		cout<<"Robot has fallen backward"<<endl;
	}
	//while (Action::GetInstance()->IsRunning() == 1) usleep(8000);
	// Go back to Walk Ready
	//mWalkReady(arbotixpro);
	//BrainThread::GetBrainThreadInstance()->setState(GO_TO_BALL_STATE);}
}

void Motion::TurnByAngle(double angle)
{
	if(angle == 0)
		return;
	else if(angle > 0)
	{
		StartWalking(0,0,30);
	}
	else if(angle < 0)
	{
		angle = -angle;
		StartWalking(0,0,-30);
	}
	//usleep(2000*100000);
	usleep(factor*angle*27);
	StopWalking();
	SetHeadTilt(HeadTilt(GetHeadTilt().Tilt,0));
}

void Motion::TurnToGoal(double angle)
{
	StartWalking(0,40,5);
	usleep(88888.8888*angle);
	StopWalking();
}

void Motion::FreeAllEngines()
{
	arbotixpro.WriteByte(ArbotixPro::ID_CM, MXDXL::P_TORQUE_ENABLE, 0, 0);
}

void Motion::SetHeadTilt(HeadTilt headTilt)
{
	Head::GetInstance()->m_Joint.SetEnableHeadOnly(true);
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true);
	Head::GetInstance()->MoveByAngle(headTilt.Pan, headTilt.Tilt);
	//WaitForActionFinish();

}

HeadTilt Motion::GetHeadTilt()
{
	float tilt = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_TILT);
	float pan =  MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_PAN);
	return HeadTilt(tilt, pan);
}

void Motion::RunAction(ActionPage actionPage)
{
	Action::GetInstance()->m_Joint.SetEnableBody(true, true);
	WaitForActionFinish();
	Action::GetInstance()->Start((int)actionPage);
	WaitForActionFinish();
}

void Motion::WaitForActionFinish()
{
	int usecondsBetweenSamples = 8*1000;
	// Checks every 'usecondsBetweenSamples' if the action still running.
	while(IsActionRunning())
	{
		usleep(usecondsBetweenSamples);
	}
}

bool Motion::IsActionRunning()
{
	return Action::GetInstance()->IsRunning();

}

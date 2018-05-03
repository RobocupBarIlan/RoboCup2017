#include "MotionIncludes.h"


//#define MOTION_FILE_PATH    "/home/robot/HROS5-Framework/Data/motion_4096.bin"
#define MOTION_FILE_PATH    "/home/robot/HROS5-Framework/Data/soccer_4096.bin"
#define INI_FILE_PATH    	"/home/robot/HROS5-Framework/Data/config.ini"

#define WALK_SPEED 6 //cm in 1sec

#pragma once

class HeadTilt
{
public:
	// Tilt is up-down. Pan is right-left.
	float Tilt;
	float Pan;

	HeadTilt(float tilt, float pan)
	{
		Tilt = tilt;
		Pan = pan;
	}


};

class Motion
{
public:
	// Singletone
	static Motion* GetInstance();

	~Motion();

	void WalkStraight(int cmToWalk);
	void StartWalking();
	void StartWalking(int x_amp, int y_amp, int a_amp);
	void StartTurning(int x_amp, int y_amp, int z_amp, int a_amp);
	void StartWalking(double distance);
	void StopWalking();
	void GetUp();
	void TurnByAngle(double angle);
	void TurnToGoal(double angle);
	
	void SetHeadTilt(HeadTilt headTilt);
	HeadTilt GetHeadTilt();

	void RunAction(ActionPage actionPage);
	void WaitForActionFinish();

	void FreeAllEngines();
	void StartEngines();

	//const float factor = 1333.3333333;
	const float factor = 666.666665;

private:
	Motion();
	static Motion* m_instance;

	// Returns true as long as any motion is being done
	bool IsActionRunning();
};



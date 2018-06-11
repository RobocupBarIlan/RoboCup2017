#ifndef _MOTIONINCLUDES_H_
#define _MOTIONINCLUDES_H_

#include "Includes.h"

#include "Camera.h"
#include "Point.h"
#include "mjpg_streamer.h"
#include "minIni.h"
#include "LinuxCamera.h"
#include "ColorFinder.h"
#include "Action.h"
#include "Head.h"
#include "Walking.h"
#include "MXDXL.h"
#include "MotionManager.h"
#include "LinuxMotionTimer.h"
#include "ArbotixPro.h"
#include "LinuxArbotixPro.h"
#include "LinuxActionScript.h"
#include "JointData.h"

#define U2D_DEV_NAME0 "/dev/ttyUSB0"
#define U2D_DEV_NAME1 "/dev/ttyUSB1"

enum class ActionPage {
	Init = 9,
	BendToBall = 5,
	RightKick = 80,
	LeftKick = 85,
	BackGetup = 75,
	FrontGetup = 70,
	RightDive = 60,
	LeftDive = 65
};

#endif

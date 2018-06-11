/*
 * GoalKeeperDetector.cpp
 *
 *  Created on: Jun 29, 2017
 *      Author: root
 */

#include "GoalKeepersDetector.h"

bool GoalKeepersDetector::IS_FIRST_RUN(true);
int GoalKeepersDetector::min_hue_our_goalkeeper;
int	GoalKeepersDetector::max_hue_our_goalkeeper;
int GoalKeepersDetector::min_value_our_goalkeeper;
int GoalKeepersDetector::max_value_our_goalkeeper;

int GoalKeepersDetector::min_hue_other_team_goalkeeper;
int GoalKeepersDetector::max_hue_other_team_goalkeeper;
int GoalKeepersDetector::min_value_other_team_goalkeeper;
int GoalKeepersDetector::max_value_other_team_goalkeeper;

Mat frame,frame_hsv;
Mat hsv_channels[NUM_CHANNELS]; //Will contain all 3 HSV channels splitted.
const int MIN_CONNECTED_COMPONENT_AREA=400; //Will tell the minimum area of the connected component found to be considered as the shirt of a goal keeper. This value is heuristic.

void mouseHandlerForOurGoalKeeperCalibration(int event, int x, int y, int flags, void* param)
{
    if (event == CV_EVENT_LBUTTONDOWN) //If clicked on point:
    {
//    	cout<<"x:"<<x<<" y:"<<y<<endl;
    	if(GoalKeepersDetector::min_hue_our_goalkeeper>hsv_channels[0].at<uchar>(y,x))
    	{
    		GoalKeepersDetector::min_hue_our_goalkeeper=hsv_channels[0].at<uchar>(y,x);
    	}
    	if(GoalKeepersDetector::max_hue_our_goalkeeper<hsv_channels[0].at<uchar>(y,x))
    	{
    		GoalKeepersDetector::max_hue_our_goalkeeper=hsv_channels[0].at<uchar>(y,x);
    	}
    	if(GoalKeepersDetector::min_value_our_goalkeeper>hsv_channels[2].at<uchar>(y,x))
    	{
    		GoalKeepersDetector::min_value_our_goalkeeper=hsv_channels[2].at<uchar>(y,x);
    	}
    	if(GoalKeepersDetector::max_value_our_goalkeeper<hsv_channels[2].at<uchar>(y,x))
    	{
    		GoalKeepersDetector::max_value_our_goalkeeper=hsv_channels[2].at<uchar>(y,x);
    	}
    	cout << "min_hue: " << GoalKeepersDetector::min_hue_our_goalkeeper << " max_hue: " << GoalKeepersDetector::max_hue_our_goalkeeper << " min value: " << GoalKeepersDetector::min_value_our_goalkeeper << endl;

    }
}

void mouseHandlerForOtherTeamGoalKeeperCalibration(int event, int x, int y, int flags, void* param)
{
    if (event == CV_EVENT_LBUTTONDOWN) //If clicked on point:
    {
//    	cout<<"x:"<<x<<" y:"<<y<<endl;
    	if(GoalKeepersDetector::min_hue_other_team_goalkeeper>hsv_channels[0].at<uchar>(y,x))
    	{
    		GoalKeepersDetector::min_hue_other_team_goalkeeper=hsv_channels[0].at<uchar>(y,x);
    	}
    	if(GoalKeepersDetector::max_hue_other_team_goalkeeper<hsv_channels[0].at<uchar>(y,x))
    	{
    		GoalKeepersDetector::max_hue_other_team_goalkeeper=hsv_channels[0].at<uchar>(y,x);
    	}
    	if(GoalKeepersDetector::min_value_other_team_goalkeeper>hsv_channels[2].at<uchar>(y,x))
    	{
    		GoalKeepersDetector::min_value_other_team_goalkeeper=hsv_channels[2].at<uchar>(y,x);
    	}
    	if(GoalKeepersDetector::max_value_other_team_goalkeeper<hsv_channels[2].at<uchar>(y,x))
    	{
    		GoalKeepersDetector::max_value_other_team_goalkeeper=hsv_channels[2].at<uchar>(y,x);
    	}
    }
}

void GoalKeepersDetector::GetGoalKeepers(Point& ourTeamGoalKeeperXY, Point& otherTeamGoalKeeperXY)
{
	VisionThread::SafeReadeCapturedFrame(frame); //Read safely the captured frame.
	GaussianBlur(frame, frame, cv::Size(3, 3), 1.5,1.5);
	cvtColor(frame,frame_hsv,CV_BGR2HSV);
	split(frame_hsv, hsv_channels); //Split to H,S,V channels so we can use the hue,saturation&value matrices more conveniently later on.
	//Init values:
	ourTeamGoalKeeperXY.x=-1;
	ourTeamGoalKeeperXY.y=-1;
	otherTeamGoalKeeperXY.x=-1;
	otherTeamGoalKeeperXY.y=-1;

	if(GoalKeepersDetector::IS_FIRST_RUN) //Perform calibration to both goalkeepers' t-shirts.
	{
		min_hue_our_goalkeeper=255;
		max_hue_our_goalkeeper=0;
		min_value_our_goalkeeper=255;
		max_value_our_goalkeeper=0;

		min_hue_other_team_goalkeeper=255;
		max_hue_other_team_goalkeeper=0;
		min_value_other_team_goalkeeper=255;
		max_value_other_team_goalkeeper=0;
		//Calibration of our own goalkeeper's shirt:
		 namedWindow("OUR_TEAM_CALIBRATION", 1);

		 //set the callback function for any mouse event
	     setMouseCallback("OUR_TEAM_CALIBRATION", mouseHandlerForOurGoalKeeperCalibration, NULL);
	     cout<<"Click on pixels representing the color of our goalkeeper\'s shirt, to finish the process press \'q\'"<<endl;
		 imshow("OUR_TEAM_CALIBRATION", frame);

	    // Wait until user presses 'q'
		//while((waitKey(100) & 0xFF) != 'q');

		 waitKey(10000);

		 destroyWindow("OUR_TEAM_CALIBRATION");

		//---------------------------------------------------------------------------------------

		 //Calibration of the other team's goalkeeper shirt:
		 namedWindow("OTHER_TEAM_CALIBRATION", 1);

		 //set the callback function for any mouse event
	     setMouseCallback("OTHER_TEAM_CALIBRATION", mouseHandlerForOtherTeamGoalKeeperCalibration, NULL);
	     cout<<"Click on pixels representing the color of the other team\'s goalkeeper shirt, to finish the process press \'q\'"<<endl;
		 imshow("OTHER_TEAM_CALIBRATION", frame);

	    // Wait until user presses 'q'
		//while((waitKey(1) & 0xFF) != 'q');
		 waitKey(10000);


		 destroyWindow("OTHER_TEAM_CALIBRATION");
		GoalKeepersDetector::IS_FIRST_RUN=false;
	}
	else //No need to perform calibration. we already have the required values.
	{
		Mat our_team_color_threshold,other_team_color_threshold;
		our_team_color_threshold=Mat::zeros(frame.rows,frame.cols,CV_8UC1);
		other_team_color_threshold=Mat::zeros(frame.rows,frame.cols,CV_8UC1);
		for(int i=0;i<frame.rows;i++)
		{
			for(int j=0;j<frame.cols;j++)
			{
				if(hsv_channels[0].at<uchar>(i,j)<=GoalKeepersDetector::max_hue_our_goalkeeper &&hsv_channels[0].at<uchar>(i,j)>=GoalKeepersDetector::min_hue_our_goalkeeper && hsv_channels[2].at<uchar>(i,j)>= GoalKeepersDetector::min_value_our_goalkeeper && hsv_channels[2].at<uchar>(i,j)<= GoalKeepersDetector::max_value_our_goalkeeper)
				{
					our_team_color_threshold.at<uchar>(i,j)=255;
				}
				if(hsv_channels[0].at<uchar>(i,j)<=GoalKeepersDetector::max_hue_other_team_goalkeeper &&hsv_channels[0].at<uchar>(i,j)>=GoalKeepersDetector::min_hue_other_team_goalkeeper && hsv_channels[2].at<uchar>(i,j)>= GoalKeepersDetector::min_value_other_team_goalkeeper && hsv_channels[2].at<uchar>(i,j)<= GoalKeepersDetector::max_value_other_team_goalkeeper)
				{
					other_team_color_threshold.at<uchar>(i,j)=255;
				}
			}
		}
		namedWindow("our_team_color_threshold", 1);
		imshow("our_team_color_threshold",our_team_color_threshold);
		waitKey(1);

		namedWindow("other_team_color_threshold", 1);
		imshow("other_team_color_threshold",other_team_color_threshold);
		waitKey(1);

		//Now search for large enough connected components in the threshold matrices and if found - declare a goalkeeper detected:
		Mat labels1, stats1, centroids1;
	    int nLabels = connectedComponentsWithStats(our_team_color_threshold, labels1, stats1, centroids1, 8, CV_16U);
	    for(int i=1;i<nLabels;i++)
	    {

	    	if(stats1.at<int>(i,CC_STAT_AREA)>=MIN_CONNECTED_COMPONENT_AREA)
	    	{
	    		ourTeamGoalKeeperXY.x=cvRound(centroids1.at<double>(i,0));
	    		ourTeamGoalKeeperXY.y=cvRound(centroids1.at<double>(i,1));
	    		circle(frame,ourTeamGoalKeeperXY,3,Scalar(255,0,255),2);
	    		break;
	    	}
	    }
	    Mat labels2,stats2,centroids2;
	    nLabels=connectedComponentsWithStats(other_team_color_threshold,labels2,stats2,centroids2,8,CV_16U);
	    for(int i=1;i<nLabels;i++)
	    {

	    	if(stats2.at<int>(i,CC_STAT_AREA)>=MIN_CONNECTED_COMPONENT_AREA)
	    	{
	    		otherTeamGoalKeeperXY.x=cvRound(centroids2.at<double>(i,0));
	    		otherTeamGoalKeeperXY.y=cvRound(centroids2.at<double>(i,1));
	    		circle(frame,otherTeamGoalKeeperXY,3,Scalar(0,255,255),2);
	    		break;
	    	}
	    }

		imshow("frame",frame);
		waitKey(1);
	}
}

GoalKeepersDetector::GoalKeepersDetector()
{
	// TODO Auto-generated constructor stub
}

GoalKeepersDetector::~GoalKeepersDetector()
{
	// TODO Auto-generated destructor stub
}

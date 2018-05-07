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
VisionThread::VisionThread()
{

}

VisionThread::~VisionThread()
{
	// TODO Auto-generated destructor stub
}

Mat VisionThread::FetchFrame()
{
	Mat source;
	VisionThread::SafeReadeCapturedFrame(source);
	return source;
}

void VisionThread::DetectLines()
{
LinesDetector lt;

Mat ipm_image;
Mat src, src_gray, src_hsv;
Mat dst, detected_edges;
Mat src_gray2;
int edgeThresh = 1;
int const max_lowThreshold = 255;

int low_red = 39;	//  39  39  39  39  39  39  39  39  48  48
int low_green = 76;//90 //  54  39 121  15  29 106  41  83  83 126
int low_blue = 66;//?  //   35  32  56  79  23 107 107 107 107 107
int high_red = 81;   //  88  90  88 102  88  76  76  76  76  76
int high_green = 255;// 255 255 255 255 255 255 255 255 255 198
int high_blue = 252; // 252 252 252 252 252 252 252 252 252 218

int low_red2 = 0;	// 080 080 000 080 000
int low_green2 = 0; // 011 000 000 000 000
int low_blue2 = 180;  // 112 161 255 154 161
int high_red2 = 255;// 107 133 091 132 176
int high_green2 = 68;// 161 036 037 057 042
int high_blue2 = 255; // 246 255 255 255 255
int ratio = 3;
int kernel_size = 3;

int alpha_ = 20, beta_ = 90, gamma_ = 90;
int f_ = 1300, dist_ = 200;
int threshold = 25, minLinLength = 20, maxLineGap = 23;
//ipm_image = IPM(alpha_, dist_, f_);


int thresh = 100;


//imshow("Result2", ipm_image);
const char* window_name = "Green";
const char* window_name2 = "White";
namedWindow(window_name, CV_WINDOW_NORMAL);
namedWindow("Result2", CV_WINDOW_NORMAL);
namedWindow(window_name2, CV_WINDOW_NORMAL);
namedWindow("Result3", CV_WINDOW_NORMAL);

createTrackbar("Min Hue:", window_name, &low_red, max_lowThreshold);
 createTrackbar("Min Saturation:", window_name, &low_green, max_lowThreshold);
 createTrackbar("Min Value:", window_name, &low_blue, max_lowThreshold);
 createTrackbar("Max Hue:", window_name, &high_red, max_lowThreshold);
 createTrackbar("Max Saturation:", window_name, &high_green, max_lowThreshold);
 createTrackbar("Max Value:", window_name, &high_blue, max_lowThreshold);
 createTrackbar("Min Hue:", window_name2, &low_red2, max_lowThreshold);
 createTrackbar("Min Saturation:", window_name2, &low_green2, max_lowThreshold);
 createTrackbar("Min Value:", window_name2, &low_blue2, max_lowThreshold);
 createTrackbar("Max Hue:", window_name2, &high_red2, max_lowThreshold);
 createTrackbar("Max Saturation:", window_name2, &high_green2, max_lowThreshold);
 createTrackbar("Max Value:", window_name2, &high_blue2, max_lowThreshold);


 createTrackbar( "Threshold:", "Source44", &thresh, 500);


createTrackbar("threshold", "Result2", &threshold, 300);
createTrackbar("minLinLength", "Result2", &minLinLength, 300);
createTrackbar("maxLineGap", "Result2", &maxLineGap, 300);

//createTrackbar("alpha", "Result3", &alpha_, 180);
createTrackbar("focal", "Result3", &f_, 2000);
createTrackbar("distance", "Result3", &dist_, 500);

for (int i = 0; i < 1000000; i++)
{

////	createTrackbar("f", "Result2", &f_, 2000);
////	createTrackbar("Distance", "Result2", &dist_, 2000);
//
	/// Load an image
	 // 61 51 83
	src = FetchFrame();


	 Size size(((double) src.size().width / src.size().height) * 500, 500);//the dst image size,e.g.100x100
	 Mat field, robots;//dst image
	 resize(src, dst, size);//resize image
	 field = dst.clone();
	 robots = dst.clone();

	 Mat& Field = field;
	 Mat& Robots = robots;

	CannyThreshold(src, src_gray, src_hsv,
detected_edges,
src_gray2,
edgeThresh,
low_red,
low_green,
low_blue,
high_red,
high_green,
high_blue,
low_red2,
low_green2,
low_blue2,
high_red2,
high_green2,
high_blue2,
ratio,
kernel_size, Field, Robots);
	Motion* motion = BrainThread::GetBrainThreadInstance()->getMotion();
	HeadTilt ht = motion->GetHeadTilt();
	alpha_ = 20;//ht.Tilt;
	ipm_image = IPM(Field, alpha_, dist_, f_);

	Mat src_gray9;
	int max_thresh = 255; RNG rng(12345);
    cvtColor(ipm_image, src_gray9, cv::COLOR_RGB2GRAY);

	//Mat white2 = Ellipse(src, src_gray9, thresh, max_thresh, rng);

	//imshow("Result2", ipm_image);
	lt.GetLinesPosts(ipm_image, threshold, minLinLength, maxLineGap);
	 /// Load an image
	 //src = imread(ipm_image, 1);
	 // 61 51 83
	 //if (!src.data)
	 //{
	 // return -1;
	 //}

}
waitKey(0);
}

void VisionThread::CannyThreshold(Mat src, Mat src_gray, Mat src_hsv,
Mat detected_edges,
Mat src_gray2,
int edgeThresh,
int low_red,
int low_green,
int low_blue,
int high_red,
int high_green,
int high_blue,
int low_red2,
int low_green2,
int low_blue2,
int high_red2,
int high_green2,
int high_blue2,
int ratio,
int kernel_size, Mat& White_zone, Mat& Robots
/*int, void**/)
{
	 /// Reduce noise with a kernel 3x3
	 cvtColor(src, src_hsv, CV_RGB2HSV);
	 //imshow("Original", src);
	 inRange(src_hsv, Scalar(low_red, low_green, low_blue), Scalar(high_red, high_green, high_blue), src_gray);
	 inRange(src_hsv, Scalar(low_red2, low_green2, low_blue2), Scalar(high_red2, high_green2, high_blue2), src_gray2);
	 //blur(src_gray, detected_edges, Size(3, 3));
	 /// Canny detector
	 //Canny(detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size);
	 /// Using Canny's output as a mask, we display our result
	 //dst = Scalar::all(0);
	 //src.copyTo(dst, detected_edges);
	 Mat clo = src.clone();
	 Mat clo1;
	 Mat clo2;

	 medianBlur(src_gray, src_gray, 3);
	 medianBlur(src_gray2, src_gray2, 3);

	 bitwise_and(clo, clo, clo2, src_gray);
	 Mat grass_white;
	 bitwise_or(src_gray, src_gray2, grass_white, noArray());
	 Mat gray = src_gray.clone();
	 //cvtColor(src_gray, src_gray, COLOR_ COLOR_RGBA2GRAY, 0);
	 threshold(gray, gray, 100, 200, THRESH_BINARY);
	 vector<vector<Point> > contours;
	 vector<Vec4i> hierarchy;
	 vector<vector<Point> > hull;
	 Mat dst = Mat::zeros(gray.rows, gray.cols, CV_8UC1);
	 findContours(gray, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE);
	 // approximates each contour to convex hull
	 for (int i = 0; i < contours.size(); i++) {
	 	Mat tmp;
	  vector<Point > cnt = contours[i];
	 // You can try more different parameters
	 	convexHull(cnt, tmp, false, true);
	 	hull.push_back(tmp);
	 }
	 // draw contours with random Scalar
	 for (int i = 0; i < contours.size(); i++) {
		 Mat dst_tmp;

		 double area = contourArea(contours[i]);
		 if (area > 1200)
		 {
			 Mat temp_mat_dilate;
			 Mat temporary_mat = Mat::zeros(gray.rows, gray.cols, CV_8UC1);
	 	Scalar colorHull = Scalar(255, 255, 255);
	 	drawContours(temporary_mat, hull, i, colorHull, -1, 8, hierarchy, 0);
//	 	int dilation_type;
//	 	int dilation_size = 10;
//	 	Mat element = getStructuringElement( MORPH_RECT,
//	 	                                       Size( 2*dilation_size + 1, 2*dilation_size+1 ),
//	 	                                       Point( dilation_size, dilation_size ) );
//	 	  /// Apply the dilation operation
//
//	 	dilate(temporary_mat, temp_mat_dilate, element);
		 bitwise_or(temporary_mat, dst, dst_tmp);
	 	dst = dst_tmp.clone();
		 }
	 }

	 	   int status = 0;
	 	   for (int y = 0; y < dst.rows; y++)
	 	   {
	 	 	  Scalar intensity = dst.at<uchar>(Point(0, y));
	 	 	  if (intensity.val[0] != 0  && status == 0)
	 	 	  {
	 	 		  status = 1;
	 	 	  }
	 	 	  if (intensity.val[0] == 0  && status == 1)
	 	 	  {
	 	 		  status = 2;
	 	 	  }
	 	 	  if (intensity.val[0] != 0  && status == 2)
	 	 	  {
	 	 		  status = 1;
	 	 	  }
	 	 	  if (status == 2)
	 	 	  {
	 	 		dst.at<uchar>(y, 0) = 255;
	 	 	  }

	 	   }

	 	   status = 0;
	 	   for (int y = 0; y < dst.rows; y++)
	 	   {
	 	 	  Scalar intensity = dst.at<uchar>(Point(dst.cols-1, y));
	 	 	  if (intensity.val[0] != 0  && status == 0)
	 	 	  {
	 	 		  status = 1;
	 	 	  }
	 	 	  if (intensity.val[0] == 0  && status == 1)
	 	 	  {
	 	 		  status = 2;
	 	 	  }
	 	 	  if (intensity.val[0] != 0  && status == 2)
	 	 	  {
	 	 		  status = 1;
	 	 	  }
	 	 	  if (status == 2)
	 	 	  {
	 	 		dst.at<uchar>(y, dst.cols-1) = 255;
	 	 	  }
	 	   }

	 	   status = 0;
	 	   for (int x = 0; x < dst.cols; x++)
	 	   {
	 	 	  Scalar intensity = dst.at<uchar>(Point(x, dst.rows-1));
	 	 	  if (intensity.val[0] != 0  && status == 0)
	 	 	  {
	 	 		  status = 1;
	 	 	  }
	 	 	  if (intensity.val[0] == 0  && status == 1)
	 	 	  {
	 	 		  status = 2;
	 	 	  }
	 	 	  if (intensity.val[0] != 0  && status == 2)
	 	 	  {
	 	 		  status = 1;
	 	 	  }
	 	 	  if (status == 2)
	 	 	  {
	 	 		dst.at<uchar>(dst.rows-1, x) = 255;
	 	 	  }
	 	   }

	 	  status = 0;
	 	   for (int x = 0; x < dst.cols; x++)
	 	   {
	 		   int x_left = -1;
	 		   int write = 0;
	 	 	  Scalar intensity = dst.at<uchar>(Point(x, 0));
	 	 	  if (intensity.val[0] != 0  && x_left == -1)
	 	 	  {
	 	 		  x_left == x;
	 	 	  }
	 	 	  if (intensity.val[0] == 0  && x_left == -1)
	 	 	  {
	 	 		  write = 1;
	 	 	  }
	 	 	  if (intensity.val[0] != 0  && write == 1)
	 	 	  {
	 	 		  for (int x2 = x_left; x2 <= x; x2++)
	 	 		  {
	 		 	 		dst.at<uchar>(0, x2) = 255;
	 	 		  }
	 	 		write = 0;
	 	 	  }
	 	   }


		 	 Mat grayC = dst.clone();
		 	 vector<vector<Point> > contoursC;
		 	 vector<Vec4i> hierarchyC;
		 	 vector<vector<Point> > hullC;
		 	 Mat dstC = Mat::zeros(grayC.rows, grayC.cols, CV_8UC1);
		 	 findContours(grayC, contoursC, hierarchyC, RETR_CCOMP, CHAIN_APPROX_SIMPLE);
		 	 // approximates each contour to convex hull
		 	 for (int i = 0; i < contoursC.size(); i++)
		 	 {
		 	 	Mat tmpC;
		 	  vector<Point > cntC = contoursC[i];
		 	 // You can try more different parameters
		 	 	convexHull(cntC, tmpC, false, true);
		 	 	hullC.push_back(tmpC);
		 	 }
		 	 RNG rng(12345);

		 	   for( int i = 0; i< contoursC.size(); i++ )
		 	      {
		 	        Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
		 	        drawContours( dstC, contoursC, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
		 	        drawContours( dstC, hullC, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
		 	      }

	 	 // draw contours with random Scalar
	 	 for (int i = 0; i < contoursC.size(); i++)
	 	 {
	 		 Mat dst_tmpC;

	 		 double areaC = contourArea(contoursC[i]);
	 		Mat temp_mat_dilateC;
	 		Mat temporary_matC = Mat::zeros(grayC.rows, grayC.cols, CV_8UC1);
	 	 	Scalar colorHull = Scalar(255, 255, 255);
	 	 	drawContours(temporary_matC, hullC, i, colorHull, -1, 8, hierarchyC, 0);
	 		 bitwise_or(temporary_matC, dstC, dst_tmpC);
	 	 	dstC = dst_tmpC.clone();
	 	 }

	 Mat Total_Field, GreenNWhite, white_field_binary, white_field, field_objects;
	 Mat Green, White;
	 Mat Green_Field, White_Field, Robot_Field;
	 Mat F_B, G_B, W_B, R_B;
	 F_B = dstC.clone();
	 G_B = src_gray.clone();
	 W_B = src_gray2.clone();

	 bitwise_and(F_B, G_B, Green, noArray());
	 bitwise_and(clo, clo, Green_Field, Green);
	 imshow("Grass", Green_Field);

	 bitwise_and(clo, clo, Total_Field, F_B);
	 imshow("Total Field", Total_Field);

	 bitwise_and(F_B, W_B, White, noArray());
	 bitwise_and(clo, clo, White_Field, White);
	 imshow("White Lines", White_Field);

	 bitwise_or(G_B, W_B, GreenNWhite, noArray());
	 bitwise_not(GreenNWhite, GreenNWhite);
	 bitwise_and(GreenNWhite, F_B, R_B, noArray());
	 bitwise_and(clo, clo, Robot_Field, R_B);
	 imshow("Robots", Robot_Field);
Robots = Robot_Field;
White_zone = White_Field;

//	 bitwise_not(grass_white, grass_white, noArray());
//	 bitwise_and(dstC, dstC, dst2, grass_white);
//	 bitwise_and(dstC, dstC, white_field_binary, src_gray2);
//
//	 bitwise_and(clo, clo, dst3, dst);
//	 bitwise_and(dst3, dst3, field_objects, dst2);
//	 medianBlur(field_objects, field_objects, 3);
//	 bitwise_and(clo, clo, white_field, white_field_binary);
//
//
//	 imshow("White lines", white_field);
//	 imshow("Grass", white_field);
//	 imshow("Field Objects", field_objects);
//	 imshow("Total Field", dst3);
	//return white_field;
	 //return White_Field;
}

Mat VisionThread::Ellipse(Mat src, Mat src_gray, int thresh, int max_thresh, RNG rng)
{
	  Mat threshold_output;
	  vector<vector<Point> > contours;
	  vector<Vec4i> hierarchy;

	  /// Detect edges using Threshold
	  threshold( src_gray, threshold_output, thresh, 255, THRESH_BINARY );
	  /// Find contours
	  findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

	  /// Find the rotated rectangles and ellipses for each contour
	  vector<RotatedRect> minRect( contours.size() );
	  vector<RotatedRect> minEllipse( contours.size() );

	  for( int i = 0; i < contours.size(); i++ )
	     { minRect[i] = minAreaRect( Mat(contours[i]) );
	       if( contours[i].size() > 5 )
	         { minEllipse[i] = fitEllipse( Mat(contours[i]) ); }
	     }

	  /// Draw contours + rotated rects + ellipses
	  Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );
	  Mat drawing1 = Mat::zeros( threshold_output.size(), CV_8UC3 );

      int max_index = -1;
      double max_area = 0;
      Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );

	  for( int i = 0; i< contours.size(); i++ )
	     {
	       // contour
	       drawContours( drawing1, contours, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
	       // ellipse
if ( (double) minEllipse[i].size.height / minEllipse[i].size.width < 3.5)
	       if (minEllipse[i].size.area() > max_area)
	       {
	    	   max_area = minEllipse[i].size.area();
	    	   max_index = i;
	       }

	       // rotated rectangle
	       Point2f rect_points[4]; minRect[i].points( rect_points );
	       //for( int j = 0; j < 4; j++ )
	          //line( drawing, rect_points[j], rect_points[(j+1)%4], color, 1, 8 );
	     }
	  if (max_index != -1)
	  {
		  cout << minEllipse[max_index].size.height << " " << minEllipse[max_index].size.width << endl;
ellipse( src, minEllipse[max_index], color, 2, 8 );
	  }
	  /// Show in a window
	  namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
	  imshow( "Contours", src );
	  return drawing;
}

Mat VisionThread::IPM(Mat regular, int alpha_, int dist_, int f_)
{

	int frameWidth = 640;
	int frameHeight = 480;
int beta_ = 90;
int gamma_ = 90;
	/*
	 * This code illustrates bird's eye view perspective transformation using opencv
	 * Paper: Distance Determination for an Automobile Environment using Inverse Perspective Mapping in OpenCV
	 * Link to paper: https://www.researchgate.net/publication/224195999_Distance_determination_for_an_automobile_environment_using_Inverse_Perspective_Mapping_in_OpenCV
	 * Code taken from: http://www.aizac.info/birds-eye-view-homography-using-opencv/
	 */

//
//		if (!capture.isOpened())
//		{
//			cout<<"Could not open VideoCapture"<<endl;
//			pthread_exit(NULL);
//		}

	    // mat container to receive images
	    Mat source, destination;

//	    // check if capture was successful
//	    if( !capture.isOpened()) throw "Error reading video";

		//createTrackbar("Alpha", "Result", &alpha_, 180);
		////createTrackbar("Beta", "Result", &beta_, 180);
		////createTrackbar("Gamma", "Result", &gamma_, 180);
		//createTrackbar("f", "Result", &f_, 2000);
		//createTrackbar("Distance", "Result", &dist_, 2000);
		//namedWindow("Result", 1);

int counter = 0;
		//while( true ) {
source = regular;
			//VisionThread::SafeReadeCapturedFrame(source);
			//capture >> source;

			resize(source, source,Size(frameWidth, frameHeight));

			double focalLength, dist, alpha, beta, gamma;

			alpha =((double)alpha_ -90) * PI/180;
			beta =((double)beta_ -90) * PI/180;
			gamma =((double)gamma_ -90) * PI/180;
			focalLength = (double)f_;
			dist = (double)dist_;

			Size image_size = source.size();
			double w = (double)image_size.width, h = (double)image_size.height;

			// Projecion matrix 2D -> 3D
			Mat A1 = (Mat_<float>(4, 3)<<
				1, 0, -w/2,
				0, 1, -h/2,
				0, 0, 0,
				0, 0, 1 );

			// Rotation matrices Rx, Ry, Rz

			Mat RX = (Mat_<float>(4, 4) <<
				1, 0, 0, 0,
				0, cos(alpha), -sin(alpha), 0,
				0, sin(alpha), cos(alpha), 0,
				0, 0, 0, 1 );

			Mat RY = (Mat_<float>(4, 4) <<
				cos(beta), 0, -sin(beta), 0,
				0, 1, 0, 0,
				sin(beta), 0, cos(beta), 0,
				0, 0, 0, 1	);

			Mat RZ = (Mat_<float>(4, 4) <<
				cos(gamma), -sin(gamma), 0, 0,
				sin(gamma), cos(gamma), 0, 0,
				0, 0, 1, 0,
				0, 0, 0, 1	);

			// R - rotation matrix
			Mat R = RX * RY * RZ;

			// T - translation matrix
			Mat T = (Mat_<float>(4, 4) <<
				1, 0, 0, 0,
				0, 1, 0, 0,
				0, 0, 1, dist,
				0, 0, 0, 1);

			// K - intrinsic matrix
			Mat K = (Mat_<float>(3, 4) <<
				focalLength, 0, w/2, 0,
				0, focalLength, h/2, 0,
				0, 0, 1, 0
				);

			Mat transformationMat = K * (T * (R * A1));

			warpPerspective(source, destination, transformationMat, image_size, INTER_CUBIC | WARP_INVERSE_MAP);
			//destination = source;
			imshow("Result", destination);
			//cout << counter++ << endl;
			waitKey(100);
			return destination;
		//}
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
		}//candidate
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

//	switch(signum)
//	{
//		case GET_BALL_CENTER_IN_FRAME_AND_DISTANCE:
//				VisionThread::GetBallCenterInFrameAndDistance();
//				break;
//		case GET_GOAL_IN_FRAME:
//				VisionThread::GetGoalCandidate();
//			break;
//		case GET_GOALKEEPER_CENTER_IN_FRAME_AND_DISTANCE:
//			break;
//	}
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

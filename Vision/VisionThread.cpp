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

void VisionThread::HOG()
{
	// Global Variables
	Mat src; Mat hsv; Mat hue;
	int bins = 25;

	int edgeThresh = 69;
	int lowThreshold;
	int const max_lowThreshold = 100;
	int ratio = 3;
	int kernel_size = 3;
	char* window_name = "Edge Map";
	double minVal, maxVal;
	Mat dst;

	Mat kernel;
	Point anchor;
	double delta;
	int ddepth;
	Mat example_image;
	Mat robot_vision;
	int c;

	// Load an image
	example_image = imread("../images/11.jpg");
	Mat temp;

	for (int qwerty = 0; qwerty < 10000; qwerty++)
	{
		robot_vision = FetchFrame();//imread("../images/5.jpg", 1);

		// Load image
		if (!example_image.data)
		{
			cout << "Error1" << endl;
			//return -1;
		}
		if (!robot_vision.data)
		{
			cout << "Error2" << endl;
			//return -1;
		}

		// Reduce noise with a kernel 3x3
		blur(example_image, temp, Size(3, 3));

		// Canny detector
		//Canny(temp, example_image, 69, 33 * ratio, kernel_size);
		// Using Canny's output as a mask, we display our result
		//	imshow("temp", example_image);

		dst = temp.clone();
		src = temp.clone();

		//		src.copyTo(dst, temp);
		//		imshow("1", example_image);

		// Reduce noise with a kernel 3x3
		blur(robot_vision, temp, Size(3, 3));

		// Canny detector
		//Canny(temp, robot_vision, 69, 69 * ratio, kernel_size);
		//	imshow("2", robot_vision);

		// Using Canny's output as a mask, we display our result
		//dst = Scalar::all(0);
		//dst = src.clone();

		dst = src.clone();
		//src.copyTo(dst, robot_vision);
		//	imshow("3", dst);


		cout << "finished";
		example_image.convertTo(src, CV_32F, 1 / 255.0);
		int window_index = 0;
		// Calculate gradients gx, gy
		Mat gx, gy;
		Sobel(example_image, gx, CV_32F, 1, 0, 1);
		Sobel(example_image, gy, CV_32F, 0, 1, 1);
		//imshow("gx", gx);
		//imshow("gy", gy);

		// C++ Calculate gradient magnitude and direction (in degrees)
		Mat mag_example_image, angle_example_image;
		cartToPolar(gx, gy, mag_example_image, angle_example_image, 1);

		Mat test_rect;
		Rect rect_boundaries;
		int rect_width = 100;
		int rect_height = rect_width * example_image.rows / example_image.cols;
		for (int y = 0; y < robot_vision.rows-10; y+=20)
		{
			for (int x = 0; x < robot_vision.cols-10; x+=20)
			{
				rect_boundaries.x = x;
				rect_boundaries.y = y;
				if (x + rect_width >= robot_vision.cols)
				{
					rect_boundaries.width = robot_vision.cols - 1 - x;
				}
				else
				{
					rect_boundaries.width = rect_width;
				}

				if (y + rect_height >= robot_vision.rows)
				{
					rect_boundaries.height = robot_vision.rows - 1 - y;
				}
				else
				{
					rect_boundaries.height = rect_height;
				}
				test_rect = robot_vision(rect_boundaries);

				//imshow("test_rect", test_rect);

				Sobel(test_rect, gx, CV_32F, 1, 0, 1);
				Sobel(test_rect, gy, CV_32F, 0, 1, 1);

				// C++ Calculate gradient magnitude and direction (in degrees)
				Mat mag_test_rect, angle_test_rect;
				cartToPolar(gx, gy, mag_test_rect, angle_test_rect, 1);

				//	imshow("11", angle_example_image);
				//	imshow("22", angle_test_rect);

				minMaxLoc(angle_example_image, &minVal, &maxVal);
				//cout << "Min: " << minVal << " Max: " << maxVal << endl;
				minMaxLoc(angle_test_rect, &minVal, &maxVal);
				//cout << "Min: " << minVal << " Max: " << maxVal << endl;

				// Separate the image in 3 places ( B, G and R )
				vector<Mat> example_image_vector, test_rect_vector;
				split(angle_example_image, example_image_vector);
				split(angle_test_rect, test_rect_vector);

				// Establish the number of bins
				int histSize = 360;

				// Set the ranges ( for B,G,R) )
				float range[] = { 0, 360 };
				const float* histRange = { range };

				bool uniform = true; bool accumulate = false;

				Mat b_hist, test_rect_hist, example_image_hist;

				// Compute the histograms:
				calcHist(&example_image_vector[0], 1, 0, Mat(), example_image_hist, 1, &histSize, &histRange, uniform, !accumulate);
				calcHist(&test_rect_vector[0], 1, 0, Mat(), test_rect_hist, 1, &histSize, &histRange, uniform, !accumulate);

				// Draw the histograms for B, G and R
				int hist_w = 360; int hist_h = 400;
				int bin_w = cvRound((double)hist_w / histSize);

				Mat histImage(hist_h, hist_w, CV_8UC3, Scalar(0, 0, 0));
				Mat histImage1(hist_h, hist_w, CV_8UC3, Scalar(0, 0, 0));
				Mat histImage2(hist_h, hist_w, CV_8UC3, Scalar(0, 0, 0));

				double max_val_example;
				minMaxLoc(example_image_hist, &minVal, &max_val_example);
				//cout << "Min: " << minVal << " Max: " << maxVal << endl;
				double max_val_robot;
				minMaxLoc(test_rect_hist, &minVal, &max_val_robot);
				//cout << "Min: " << minVal << " Max: " << maxVal << endl;

				Mat print1, print2;

				print1 = test_rect_hist.clone();
				print2 = example_image.clone();
				// Normalize the result to [ 0, histImage.rows ]
				normalize(print1, print1, 0, histImage.rows, NORM_MINMAX, -1, Mat());
				normalize(print2, print2, 0, histImage.rows, NORM_MINMAX, -1, Mat());

				int maximum = 0;
				// Draw for each channel
				for (int i = 0; i < histSize; i++)
				{
					/*
					//cout << "bin_w: " << bin_w << " hist_h: " <<
					line(histImage, Point(bin_w*(i - 1), hist_h - cvRound(example_image_hist.at<float>(i - 1))),
						Point(bin_w*(i), hist_h - cvRound(example_image_hist.at<float>(i))),
						Scalar(0, 255, 0), 2, 8, 0);

					line(histImage, Point(bin_w*(i - 1), hist_h - cvRound(test_rect_hist.at<float>(i - 1))),
						Point(bin_w*(i), hist_h - cvRound(test_rect_hist.at<float>(i))),
						Scalar(0, 0, 255), 2, 8, 0);
					 */

					/*
					line(histImage1, Point(bin_w*(i), hist_h - cvRound(example_image_hist.at<float>(i)) * hist_h / max_val_example),
						Point(bin_w*(i), hist_h-1),
						Scalar(0, 255, 0), 2, 8, 0);
		if (cvRound(test_rect_hist.at<float>(i)) > maximum)
				maximum = cvRound(test_rect_hist.at<float>(i));

					line(histImage2, Point(bin_w*(i), hist_h - cvRound(test_rect_hist.at<float>(i)) * hist_h / max_val_robot),
						Point(bin_w*(i), hist_h-1),
						Scalar(0, 0, 255), 2, 8, 0);
					*/
				}
				//cout << maximum << endl;

				// Initialize arguments for the filter
				anchor = Point(-1, -1);
				delta = 0;
				ddepth = -1;

				// Update kernel size for a normalized box filter
				normalize(example_image_hist, example_image_hist, 0, 1/*histImage.rows*/, NORM_MINMAX, -1, Mat());
				normalize(test_rect_hist, test_rect_hist, 0, 1, NORM_MINMAX, -1, Mat());
				cv::Size s = example_image.size();
				//cout << s.height << "x" << s.width;

				//cout << example_image_hist.t();
				//cout << endl << endl << endl;
				double example_image_size = example_image.size().height * example_image.size().width;
				//cout << example_image_size << endl;
				//example_image_hist = 1000 * (example_image_hist / example_image_size);
				//cout << example_image_hist.t() << endl;
				double robot_image_size = test_rect.size().height * test_rect.size().width;
				//		test_rect_hist = 1000 * (test_rect_hist/* / robot_image_size*/);


				// Apply filter (CONVLUTION)
				filter2D(test_rect_hist, dst, ddepth, example_image_hist, anchor, delta, BORDER_DEFAULT);


				//cout << example_image_hist.t() << endl;
				//	cout << test_rect_hist.t() << endl;

				minMaxLoc(dst, &minVal, &maxVal);
				//cout << "Min: " << minVal << " Max: " << maxVal << endl;

				double std_dev = 0;
				for (int i = 1/*0*/; i < histSize/4; i++)
				{
					std_dev += pow((test_rect_hist.at<float>(i))- (example_image_hist.at<float>(i)) , 2);
				}
				//	cout << "Standard Deviastion: " << std_dev << endl;
				if (/*maxVal >= 15*/std_dev <= 0.1)
				{
					rectangle(robot_vision,rect_boundaries,Scalar(255,0,0),1);
				}

				// Display
				//namedWindow("calcHist Demo", CV_WINDOW_AUTOSIZE);
				//imshow("calcHist Demo", histImage1);
				//imshow("calcHist Demo2", histImage2);

				//waitKey(250);
			}
		}

		imshow("Results HOG", robot_vision);
		waitKey(17);
	}
	// Wait until user exit program by pressing a key
	waitKey(0);
}

// For machine learning
void VisionThread::AutoImageCapture()
{
	int counter = 100;

	// Get the frame
	Mat save_img;
	vector<int> compression_params;
	compression_params.push_back(CV_IMWRITE_PXM_BINARY);
	compression_params.push_back(1);
	while (1)
	{
		save_img = FetchFrame();;

		int a = counter;
		string ss="";   //create empty string
	    while(a)
	    {
	    	int x=a%10;
		    a/=10;
		    char i='0';
		    i=i+x;
		    ss=i+ss;      //append new character at the front of the string!
		}

		putText(save_img, ss, Point(0, 100), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 0));
		imshow("1", save_img);
		// Save the frame into a file
		imwrite(/*"images/" +*/ "robot" + ss +  ".ppm", save_img, compression_params); // A JPG FILE IS BEING SAVED
		waitKey(100);
		counter++;
		if (counter == 100)
			break;
	}
}

void VisionThread::RobotsDetection(Mat frame)
{
	String cascPath = "../images/stopSignDetector.xml";
	Mat gray1, crop_img, gray;
	const cv::String str = "a";
	CascadeClassifier signCascade = CascadeClassifier(cascPath);
	bool do_func = true;
	if( !signCascade.load( cascPath ) ){ printf("--(!)Error loading\n"); do_func = false; };

	//while (do_func)
	{
		std::vector<Rect> faces;
		Mat video_capture = frame;//FetchFrame();
		Mat img = video_capture.clone();
		int width = img.cols;
		int high = img.rows;
		cvtColor(img, gray, COLOR_BGR2GRAY);

		// Detect signs in the image
		cout << "WIdth: " << width << " Height: " << high << endl;
		signCascade.detectMultiScale(gray, faces,1.1,10, 0,Size(10,10));

		// Draw a rectangle around the signs
		for (int i = 0; i < faces.size(); i++)
		{
			Rect rect = faces[i];
			int x = rect.x;
			int y = rect.y;
			int w = rect.width;
			int h = rect.height;
			crop_img = img(rect);
			rectangle(img, rect, Scalar(0, 255, 0), 2);
		}

		imshow("Video", img);
		waitKey(20);
	}
}

void VisionThread::DetectLines()
{
	LinesDetector lt;
	GoalDetector gd;

	Mat ipm_image;
	Mat src, src_gray, src_hsv;
	Mat dst, detected_edges;
	Mat src_gray2;

	int edgeThresh = 1;
	int const max_lowThreshold = 255;

	int low_red = 39;	//  39  39  39  39  39  39  39  39  48  48
	int low_green = 76;//90 //  54  39 121  15  29 106  41  83  83 126
	int low_blue = 66;//?  //   35  32  56  79  23 107 107 107 107 107
	int high_red = 83;   //  88  90  88 102  88  76  76  76  76  76
	int high_green = 255;// 255 255 255 255 255 255 255 255 255 198
	int high_blue = 252; // 252 252 252 252 252 252 252 252 252 218

	int low_red2 = 0;	// 080 080 000 080 000
	int low_green2 = 0; // 011 000 000 000 000
	int low_blue2 = 180;  // 112 161 255 154 161 - 120
	int high_red2 = 255;// 107 133 091 132 176
	int high_green2 = 68;// 161 036 037 057 042 - 150
	int high_blue2 = 255; // 246 255 255 255 255
	int ratio = 3;
	int kernel_size = 3;

	int threshold = 5/*12*/, minLinLength = 10, maxLineGap = 7;
	int dot_dis = 25;

	int alpha_ = 20, beta_ = 90, gamma_ = 90;
	int f_ = 670, dist_ = 138;

	int thresh = 100;

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

	createTrackbar("dot_distance", "Result2", &dot_dis, 60);

	createTrackbar("alpha", "Result3", &alpha_, 180);
	createTrackbar("focal", "Result3", &f_, 2000);
	createTrackbar("distance", "Result3", &dist_, 500);

	static std::vector<Point> point_list;
	point_list = ResetGuesses();

	for (int i = 0; i < 1000000; i++)
	{
		src = FetchFrame();

		Size size(((double) src.size().width / src.size().height) * 500, 500);//the dst image size,e.g.100x100
		Mat field, robots, goal;//dst image
		resize(src, dst, size);//resize image
		field = dst.clone();
		robots = dst.clone();
		goal = dst.clone();

		Mat& Field = field;
		Mat& Robots = robots;
		Mat& Goal = goal;

		GoalCandidate gc;
		//GoalKeepersDetector
		Point g1,g2;
		int center_x;
		int center_y;
		double distance;
		vector<vector<int>> junctions;

		clock_t begin = clock(); // Start the clock.
		Motion* motion = BrainThread::GetBrainThreadInstance()->getMotion();

		int h = -10+9;
		motion->SetHeadTilt(HeadTilt(-h, 0));
		HeadTilt ht = motion->GetHeadTilt();
		waitKey(100);
		cout << "Tilt: " << ht.Tilt-18 << "Pan: " << ht.Pan << endl;
		//ScanCenterGoal();

			CannyThreshold(src, src_gray, src_hsv, detected_edges, src_gray2, edgeThresh, low_red, low_green, low_blue, high_red, high_green, high_blue, low_red2, low_green2, low_blue2, high_red2, high_green2, high_blue2, ratio, kernel_size, Field, Goal, Robots);

			alpha_ = abs(ht.Tilt-18);
			imshow("123456", src);

			ipm_image = IPM(Field, alpha_, dist_, f_);

			Mat src_gray9;
			int max_thresh = 255; RNG rng(12345);
			cvtColor(ipm_image, src_gray9, cv::COLOR_RGB2GRAY);
			//Mat white2;
			vector<Vec4i> lines = lt.GetLines(ipm_image, threshold, minLinLength, maxLineGap);
			junctions = lt.GetJunctions(lines, ipm_image, dot_dis);

			ipm_image = IPM(Field, alpha_, dist_, f_);
			CalculateJunctionDistances(junctions, ipm_image, IPM(src, alpha_, dist_, f_), point_list);

			gd.GetGoalPosts2(gc, Goal);

		clock_t end = clock();
		double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
		cout << "elapsed time:" << elapsed_secs <<"\n"<< endl;
			Mat black;
			black = Mat::zeros(Field.rows, Field.cols, CV_8UC3);
			circle(black, cvPoint((gc.m_left_post[0].x + gc.m_left_post[1].x)*0.5, gc.m_left_post[1].y), 6, Scalar(255,0,0), -1);
			circle(black, cvPoint((gc.m_right_post[0].x + gc.m_right_post[1].x)*0.5, gc.m_right_post[1].y), 6, Scalar(255,0,0), -1);
			black = IPM(black, alpha_, dist_, f_);
			imshow("CIRCLES", black);

			std::vector<cv::Point2f> points;
			for (int x = 0; x < black.cols; x++)
			{
				for (int y = 0; y < black.rows; y++)
				{
					if ((int) black.at<cv::Vec3b>(y,x)[0] != 0 || (int) black.at<cv::Vec3b>(y,x)[1] != 0 || (int)  black.at<cv::Vec3b>(y,x)[2] != 0)
					{
						cv::Point2f p;
						p.x = x;
						p.y = y;
						points.push_back(p);
					}
				}
			}

			Mat centers, labels;
			if (points.size() > 0)
			{
			double std_div = 0;//kmeans(points, 2, labels, TermCriteria(TermCriteria::EPS+TermCriteria::COUNT, 10, 1.0), 3, KMEANS_PP_CENTERS, centers);
				Scalar color = Scalar(255, 255, 255);

				if (centers.rows >= 1)
				{
					circle(black, Point(centers.at<float>(0,0), centers.at<float>(0,1)), 2, color, -1);
					vector<int> x_y_type;
					x_y_type.push_back(centers.at<float>(0,0));
					x_y_type.push_back(centers.at<float>(0,1));
					x_y_type.push_back(7);
					junctions.push_back(x_y_type);
				}
				if (centers.rows == 2 && std_div >= 20000)
				{
					 cout << "std_div: " << std_div << endl;

					circle(black, Point(centers.at<float>(1,0), centers.at<float>(1,1)), 2, color, -1);
					vector<int> x_y_type;
					x_y_type.push_back(centers.at<float>(1,0));
					x_y_type.push_back(centers.at<float>(1,1));
					x_y_type.push_back(4);
					junctions.push_back(x_y_type);
				}

			}
				ipm_image = IPM(Field, alpha_, dist_, f_);
				CalculateJunctionDistances(junctions, ipm_image, IPM(src, alpha_, dist_, f_), point_list);

			imshow("CIRCLES", black);
//
//			double ratio = Ellipse(ipm_image, src_gray9, thresh, max_thresh, rng, white2);
//			cout << ratio << endl;
//			waitKey(1000);

		waitKey(20);
	}
	waitKey(0);
}
//Localization2();
//HOG();
//RobotsDetection(src);
//GoalKeepersDetector::GetGoalKeepers(g1,g2);
//VisionThread::SafeReadGoalInFrame(gc);


void VisionThread::ScanCenterGoal()
{
	GoalCandidate gc;
	GoalDetector gd;
	Motion* motion = BrainThread::GetBrainThreadInstance()->getMotion();
	int h =-10-60;
	int pan = -36; // Possible Pan range -36 to 35.
	int hit_counter = 0;
	int miss_counter = 0;
	motion->SetHeadTilt(HeadTilt(-h, pan));
	HeadTilt ht = motion->GetHeadTilt();

	Mat Field = FetchFrame(); // Change that with line that take only the goal area (convex).
	while (pan <= 35)
	{
		gd.GetGoalPosts2(gc, Field);
		if ((gc.m_width_left == 0 || gc.m_width_right == 0) || abs((gc.m_left_post[0].x+gc.m_left_post[1].x+gc.m_right_post[0].x+gc.m_right_post[1].x)/4 - Field.cols/2) > 70)
		{
			miss_counter++;
			hit_counter = 0;
		}
		else
		{
			miss_counter = 0;
			hit_counter++;
		}
		if (miss_counter == 3)
		{
			int pixels_to_pan = 40;
			int max1 = max((gc.m_left_post[0].x+gc.m_left_post[1].x)/2, (gc.m_right_post[0].x+gc.m_right_post[1].x)/2);
			//cout << "MAX: " << max1 << endl;
			//cout << "Rot: " << ceil((0.0+Field.cols - max1) / pixels_to_pan)  << endl;
			pan +=ceil((0.0+Field.cols - max1) / pixels_to_pan);
			motion->SetHeadTilt(HeadTilt(-h, pan));
			miss_counter = 0;
		}
		if (hit_counter == 3)
		{
			break;
		}
		waitKey(20);
	}
	if (pan <= 35)
	{
	cout << "Pan: " << pan << endl;
	}
	else
	{
		cout << "Center of Goal not found!" << endl;
	}
}

void VisionThread::CalculateJunctionDistances(vector<vector<int>> junctions, Mat ipm_image, Mat Field, vector<Point>& point_list)
{
	Motion* motion = BrainThread::GetBrainThreadInstance()->getMotion();
	HeadTilt ht = motion->GetHeadTilt();
	int alpha_ = abs(ht.Tilt-18);
	double cm_in_pixels = (3.85 * alpha_ + 15.2) / 100;
	//cout << "Cemtimeter in Pixels: " << cm_in_pixels << endl;
	double distance = 0.6 * tan(PI * (90 - alpha_) / 180);
	double robot_to_grass = 100 * (0.4979405412 * log(distance) + 0.4378291179);
	//cout << "Distance of robot to grass: " << robot_to_grass << endl;

	vector<vector<int>> distances_to_junctions;
	distances_to_junctions.push_back(vector<int>());
	distances_to_junctions.push_back(vector<int>());
	distances_to_junctions.push_back(vector<int>());
	distances_to_junctions.push_back(vector<int>());
	distances_to_junctions.push_back(vector<int>());
	distances_to_junctions.push_back(vector<int>());
	distances_to_junctions.push_back(vector<int>());
	distances_to_junctions.push_back(vector<int>());
	distances_to_junctions.push_back(vector<int>());


	Mat ipm_image2 = Field;
	imshow("FIELD", ipm_image2);
	int y_ipm = ipm_image2.rows-1;
	int grass_row = -1;
	cout << "Bottom_y: " << y_ipm << endl;
	while (grass_row == -1)
	{
		if((int) ipm_image2.at<cv::Vec3b>(y_ipm,ipm_image2.cols/2)[1] != 0)
		{
			grass_row = y_ipm;
		}
		y_ipm--;
	}

	cout << "Bottom_y: " << y_ipm << endl;

	char str[200];
	double pixels_junction_grass;
	for (int i = 0; i < junctions.size(); i++)
	{
		//cout << "d: " << junctions[i][1] << endl;
		pixels_junction_grass = sqrt( pow(junctions[i][0] - ipm_image.cols/2, 2) + pow(junctions[i][1] - y_ipm,2) );
		//cout << "Dis junction to grass in pixels: " << pixels_junction_grass << endl;


		double distance_junction_robot = pixels_junction_grass / cm_in_pixels + robot_to_grass;
		int a = (int) distance_junction_robot;
		int cou = 0;
		String ss;
	    while(a)
	    {
	    	int x=a%10;
		    a/=10;
		    char ch='0'+x;
		    ss=ch+ss;      //append new character at the front of the string!
		}
		putText(ipm_image, ss, Point2f(junctions[i][0] + 10, junctions[i][1]), FONT_HERSHEY_PLAIN, 2, Scalar(255,255,0,255), 1, CV_AA, false);
		if (junctions[i][2] == -1)
		{
			sprintf(str, "L");
			distances_to_junctions[LL].push_back(distance_junction_robot);
			putText(ipm_image, str, Point2f(junctions[i][0], junctions[i][1]), FONT_HERSHEY_PLAIN, 2, Scalar(255,0,0,255), 4, CV_AA, false);
		}
		else if (junctions[i][2] == 5)
		{
			sprintf(str, "T");
			distances_to_junctions[TT].push_back(distance_junction_robot);

			putText(ipm_image, str, Point2f(junctions[i][0], junctions[i][1]), FONT_HERSHEY_PLAIN, 2, Scalar(0,255,0,255), 4, CV_AA, false);
		}
		else if (junctions[i][2] == 6)
		{
			sprintf(str, "X");
			distances_to_junctions[XX].push_back(distance_junction_robot);

			putText(ipm_image, str, Point2f(junctions[i][0], junctions[i][1]), FONT_HERSHEY_PLAIN, 2, Scalar(0,0,255,255), 4, CV_AA, false);
		}
		else if (junctions[i][2] == 7)
		{
			sprintf(str, "G");
			distances_to_junctions[GG].push_back(distance_junction_robot);

			putText(ipm_image, str, Point2f(junctions[i][0]- 5 , junctions[i][1]), FONT_HERSHEY_PLAIN, 2, Scalar(0,255,0,255), 4, CV_AA, false);
			//VisionThread::Distance_to_Goal = distance_junction_robot;
			//cout << "Distance to goal: " << VisionThread::Distance_to_Goal << "[cm]" << endl;
		}
		else if (junctions[i][2] == 8)
		{
			sprintf(str, "C");
			distances_to_junctions[CC].push_back(distance_junction_robot);

			putText(ipm_image, str, Point2f(junctions[i][0], junctions[i][1]), FONT_HERSHEY_PLAIN, 2, Scalar(0,0,255,255), 4, CV_AA, false);
		}
		else if (junctions[i][2] == 3)
		{
			sprintf(str, "L3");
			distances_to_junctions[LL3].push_back(distance_junction_robot);

			putText(ipm_image, str, Point2f(junctions[i][0], junctions[i][1]), FONT_HERSHEY_PLAIN, 2, Scalar(0,0,255,255), 4, CV_AA, false);
		}
		else if (junctions[i][2] == 2)
		{
			sprintf(str, "L2");
			distances_to_junctions[LL2].push_back(distance_junction_robot);

			putText(ipm_image, str, Point2f(junctions[i][0], junctions[i][1]), FONT_HERSHEY_PLAIN, 2, Scalar(0,0,255,255), 4, CV_AA, false);
		}
		else if (junctions[i][2] == 4)
		{
			sprintf(str, "L4");
			distances_to_junctions[LL4].push_back(distance_junction_robot);

			putText(ipm_image, str, Point2f(junctions[i][0], junctions[i][1]), FONT_HERSHEY_PLAIN, 2, Scalar(0,0,255,255), 4, CV_AA, false);
		}
		else if (junctions[i][2] == 1)
		{
			sprintf(str, "L1");
			distances_to_junctions[LL1].push_back(distance_junction_robot);

			putText(ipm_image, str, Point2f(junctions[i][0], junctions[i][1]), FONT_HERSHEY_PLAIN, 2, Scalar(0,0,255,255), 4, CV_AA, false);
		}
	}
	imshow("Junctions Distances", ipm_image);
	imshow("dfg", Field);

	if (distances_to_junctions.size() > 0)
	{
Localization2(Point(0,0), distances_to_junctions, point_list);
	}
}

void VisionThread::Localization()
{
	// Contain the field image.
	Mat source;

	int minimum_x = 70, minimum_y = 70;

	// Vector that contain the steps that the robot takes.
	vector<Point> steps;
	steps.push_back(Point(20, 0));
	steps.push_back(Point(20, 20));
	steps.push_back(Point(20, 10));
	steps.push_back(Point(10, 10));
	steps.push_back(Point(0, 20));
	steps.push_back(Point(5, 5));
	steps.push_back(Point(5, 5));
	steps.push_back(Point(5, 5));
	steps.push_back(Point(5, 5));
	steps.push_back(Point(5, 5));
	steps.push_back(Point(5, 5));
	steps.push_back(Point(5, 5));
	steps.push_back(Point(5, 5));
	steps.push_back(Point(5, 5));
	steps.push_back(Point(5, 5));
	steps.push_back(Point(5, 5));
	steps.push_back(Point(5, 5));
	steps.push_back(Point(5, 5));
	steps.push_back(Point(5, 5));

	vector<vector<int>> distances_to_junctions;
	distances_to_junctions.push_back(vector<int>());
	distances_to_junctions.push_back(vector<int>());
	distances_to_junctions.push_back(vector<int>());
	distances_to_junctions.push_back(vector<int>());
	distances_to_junctions.push_back(vector<int>());
	distances_to_junctions[LL].push_back(0);
	distances_to_junctions[CC].push_back(1);
	distances_to_junctions[TT].push_back(2);

	vector<vector<Point>> junctions_location;

	vector<Point> v_p;

	// L junction: (8)
	v_p.push_back(Point(71 - 70, 71 - 70));   // 0
	v_p.push_back(Point(71 - 70, 670 - 70));  // 1
	v_p.push_back(Point(169 - 70, 121 - 70)); // 2
	v_p.push_back(Point(169 - 70, 619 - 70)); // 3
	v_p.push_back(Point(871 - 70, 121 - 70)); // 4
	v_p.push_back(Point(871 - 70, 619 - 70)); // 5
	v_p.push_back(Point(970 - 70, 71 - 70));  // 6
	v_p.push_back(Point(970 - 70, 670 - 70)); // 7
	junctions_location.push_back(v_p);
	v_p.clear();

	// T junction: (6)
	v_p.push_back(Point(71 - 70, 121 - 70));  // 0
	v_p.push_back(Point(71 - 70, 619 - 70));  // 1
	v_p.push_back(Point(520 - 70, 71 - 70));  // 2
	v_p.push_back(Point(520 - 70, 670 - 70)); // 3
	v_p.push_back(Point(970 - 70, 121 - 70)); // 4
	v_p.push_back(Point(970 - 70, 619 - 70)); // 5
	junctions_location.push_back(v_p);
	v_p.clear();

	// X junction: (3)
	v_p.push_back(Point(278 - 70, 368 - 70));
	v_p.push_back(Point(520 - 70, 370 - 70));
	v_p.push_back(Point(759 - 70, 369 - 70));
	junctions_location.push_back(v_p);
	v_p.clear();

	// G junction: (4)
	v_p.push_back(Point(71 - 70, 237 - 70));
	v_p.push_back(Point(71 - 70, 506 - 70));
	v_p.push_back(Point(970 - 70, 237 - 70));
	v_p.push_back(Point(970 - 70, 506 - 70));
	junctions_location.push_back(v_p);
	v_p.clear();

	// C junction: (2)
	v_p.push_back(Point(520 - 70, 298 - 70));
	v_p.push_back(Point(520 - 70, 447 - 70));
	junctions_location.push_back(v_p);
	v_p.clear();

	RNG rng;
	vector<Point> point_list;
	int real_robot_x = 250;
	int real_robot_y = 150;

	bool show_radius_circles = false;

	int number_of_dots = 2700;
	for (int i = 0; i < number_of_dots/*point_list.size()*/; i++)
	{
		int dot_x = rng.uniform(0, 900);
		int dot_y = rng.uniform(0, 600);

		Scalar color = Scalar(255, 0, 255);

		if (point_list.size() < number_of_dots)
		{
			point_list.push_back(Point(dot_x, dot_y));
		}

		circle(source, Point(minimum_x + point_list[i].x, minimum_y + point_list[i].y), 1, color, 1);
	}

	//source = imread("../images/field.png"); // Re-read the image.
	//imshow("Result", source);
	//waitKey(5000);

	// Loop through list of steps.
	for (int step = 0; step < steps.size(); step++)
	{
		source = imread("../images/field.png"); // Re-read the image.

		// Draw the junctions on the image.
		for (int i = 0; i < junctions_location.size(); i++)
		{
			for (int j = 0; j < junctions_location[i].size(); j++)
			{
				Scalar color = Scalar(0, 0, 0);
				if (i == LL)
				{
					color = Scalar(255, 0, 0);
				}
				else if (i == TT)
				{
					color = Scalar(0, 255, 0);
				}
				else if (i == XX)
				{
					color = Scalar(0, 0, 255);
				}
				else if (i == GG)
				{
					color = Scalar(255, 255, 0);
				}
				else if (i == CC)
				{
					color = Scalar(0, 255, 255);
				}

				circle(source, Point(minimum_x + junctions_location[i][j].x, minimum_y + junctions_location[i][j].y), 3, color, 3);
			}
		}

		double error = 0.2;

		// Draw the circles around the junctions.
		for (int i = 0; i < junctions_location.size(); i++)
		{
			for (int j = 0; j < junctions_location[i].size(); j++)
			{
				Scalar color = Scalar(0, 0, 0);
				if (i == LL)
				{
					color = Scalar(255, 0, 0);
				}
				else if (i == TT)
				{
					color = Scalar(0, 255, 0);
				}
				else if (i == XX)
				{
					color = Scalar(0, 0, 255);
				}
				else if (i == GG)
				{
					color = Scalar(255, 255, 0);
				}
				else if (i == CC)
				{
					color = Scalar(0, 255, 255);
				}

				for (int u2 = 0; u2 < distances_to_junctions[i].size(); u2++) // CHECK WHY THE LOOP IS HERE.
				{
					double distance = sqrt(pow(real_robot_x - junctions_location[i][distances_to_junctions[i][u2]].x, 2) + pow(real_robot_y - junctions_location[i][distances_to_junctions[i][u2]].y, 2));
					if (show_radius_circles)
					{
						circle(source, Point(minimum_x + junctions_location[i][j].x, minimum_y + junctions_location[i][j].y), distance * (1 - error), color, 1);
						circle(source, Point(minimum_x + junctions_location[i][j].x, minimum_y + junctions_location[i][j].y), distance * (1 + error), color, 1);
					}
				}
			}
		}

		for (int k = 0; k < point_list.size(); k++)
		{
			Scalar color = Scalar(255, 0, 255);

			circle(source, Point(minimum_x + point_list[k].x, minimum_y + point_list[k].y), 1, color, 1);
		}

		ellipse(source, Point(minimum_x + real_robot_x, minimum_y + real_robot_y), Size(15, 40), 0, 0, 360, Scalar(0, 0, 0), 1);

		real_robot_x += steps[step].x;
		real_robot_y += steps[step].y;
		vector<Point> point_list2;

		for (int i = 0; i < point_list.size(); i++)
		{
			point_list[i].x += steps[step].x;
			point_list[i].y += steps[step].y;

			Scalar color = Scalar(0, 0, 0);

			bool is_reset = false;

			for (int t = 0; t < junctions_location.size(); t++) // t - Type of junction.
				{
				//bool is_certain_type = false;
					for (int u2 = 0; u2 < distances_to_junctions[t].size(); u2++) // Loop through known distances of a certain type.
					{
						bool is_certain_type_dis = false;
						for (int j = 0; j < junctions_location[t].size(); j++) // j - loop thourgh the junction of a certain type. (Loop through all junctions)
						{
							double distance = sqrt(pow(real_robot_x - junctions_location[t][distances_to_junctions[t][u2]].x, 2) + pow(real_robot_y - junctions_location[t][distances_to_junctions[t][u2]].y, 2));
							double distance_to_junction = sqrt(pow(point_list[i].x - junctions_location[t][j].x, 2) + pow(point_list[i].y - junctions_location[t][j].y, 2));
							//cout << "D: " << distance << " D_t_j: " << distance_to_junction << endl;
							if (distance_to_junction >= distance * (1 - error) && distance_to_junction <= distance * (1 + error))
							{
								is_certain_type_dis = true;
							}
						}
						if (is_certain_type_dis == false)
						{
							is_reset = true;
						}
					}
				}
				//circle(source, Point(point_list[i].x, point_list[i].y), 1, color, 1);
				if (point_list[i].x < 0 || point_list[i].x > 900 || point_list[i].y > 600 || point_list[i].y < 0 || is_reset)
				{
					//int new_i = rng.uniform(0, i);
					//int dot_x = rng.uniform(point_list[i].x - 10, point_list[i].x + 10);
					//int dot_y = rng.uniform(point_list[i].y - 10, point_list[i].y + 10);

					point_list[i].x = 0;// dot_x;
					point_list[i].y = 0;// dot_y;
					point_list.erase(point_list.begin() + i);
			}
			else
			{
				point_list2.push_back(point_list[i]);
			}
		}

		int number_of_point_list2 = point_list2.size();
		while (point_list2.size() < number_of_dots)
		{
			int dot_x = rng.uniform(-30, 30);
			int dot_y = rng.uniform(-30, 30);
			int random_point_list2_point = rng.uniform(0, number_of_point_list2);

			Scalar color = Scalar(255, 0, 255);

			point_list2.push_back(Point(point_list2[random_point_list2_point].x + dot_x, point_list2[random_point_list2_point].y + dot_y));

			circle(source, Point(minimum_x + point_list2[point_list2.size()-1].x, minimum_y + point_list2[point_list2.size()-1].y), 1, color, 1);
		}

		point_list = point_list2;

		//cout << endl << " Size: " << point_list.size() << endl;


		std::vector<cv::Point2f> points;
		for (int i = 0; i < point_list.size(); i++)
		{
			cv::Point2f p;
			p.x = point_list[i].x;
			p.y = point_list[i].y;
			points.push_back(p);
		}
		Mat centers, labels;
		 double std_div = kmeans(points, 1, labels,
		            TermCriteria(TermCriteria::EPS+TermCriteria::COUNT, 10, 1.0),
		               3, KMEANS_PP_CENTERS, centers);
		 cout << "std_div: " << std_div << endl;
			Scalar color = Scalar(0, 0, 0);
			if (std_div <= number_of_dots * 10000)
			{
			circle(source, Point(minimum_x + centers.at<float>(0,0), minimum_y + centers.at<float>(0,1)), 5, color, -1);
			}
			imshow("Result", source);
		waitKey(1000);
	}
	waitKey(-1);
}

vector<Point> VisionThread::ResetGuesses()
{
	RNG rng;
	int number_of_dots = 2700;
	static std::vector<Point> point_list;

	for (int i = 0; i < number_of_dots/*point_list.size()*/; i++)
	{
		int dot_x = rng.uniform(0, 900);
		int dot_y = rng.uniform(0, 600);

		Scalar color = Scalar(255, 0, 255);

		if (point_list.size() < number_of_dots)
		{
			point_list.push_back(Point(dot_x, dot_y));
		}

		//circle(source, Point(minimum_x + point_list[i].x, minimum_y + point_list[i].y), 1, color, 1);
	}
	return point_list;
}

void VisionThread::Localization2(Point step, vector<vector<int>> distances_to_junctions, vector<Point>& point_list)
{

	for (int i = 0; i < distances_to_junctions.size(); i++)
	{
		for (int j = 0; j < distances_to_junctions[i].size(); j++)
		{
			cout << "DISTANCE: " << distances_to_junctions[i][j] << endl;
		}
	}
	// Contain the field image.
	Mat source;
	int number_of_dots = 2700;

	int minimum_x = 70, minimum_y = 70;

	vector<vector<Point>> junctions_location;

	vector<Point> v_p;

	// L junction: (8)
	junctions_location.push_back(v_p);
	v_p.clear();

	// T junction: (6)
	v_p.push_back(Point(71 - 70, 121 - 70));  // 0
	v_p.push_back(Point(71 - 70, 619 - 70));  // 1
	v_p.push_back(Point(520 - 70, 71 - 70));  // 2
	v_p.push_back(Point(520 - 70, 670 - 70)); // 3
	v_p.push_back(Point(970 - 70, 121 - 70)); // 4
	v_p.push_back(Point(970 - 70, 619 - 70)); // 5
	junctions_location.push_back(v_p);
	v_p.clear();

	// X junction: (3)
	v_p.push_back(Point(278 - 70, 368 - 70));
	v_p.push_back(Point(520 - 70, 370 - 70));
	v_p.push_back(Point(759 - 70, 369 - 70));
	junctions_location.push_back(v_p);
	v_p.clear();

	// G junction: (4)
	v_p.push_back(Point(71 - 70, 237 - 70));
	v_p.push_back(Point(71 - 70, 506 - 70));
	v_p.push_back(Point(970 - 70, 237 - 70));
	v_p.push_back(Point(970 - 70, 506 - 70));
	junctions_location.push_back(v_p);
	v_p.clear();

	// C junction: (2)
	v_p.push_back(Point(520 - 70, 298 - 70));
	v_p.push_back(Point(520 - 70, 447 - 70));
	junctions_location.push_back(v_p);
	v_p.clear();


	// L1 junction: (2)
	v_p.push_back(Point(71 - 70, 670 - 70));  // 1
	v_p.push_back(Point(970 - 70, 71 - 70));  // 6
	junctions_location.push_back(v_p);
	v_p.clear();

	// L2 junction: (2)
	v_p.push_back(Point(169 - 70, 619 - 70)); // 3
	v_p.push_back(Point(871 - 70, 121 - 70)); // 4
	junctions_location.push_back(v_p);
	v_p.clear();
	// L3 junction: (2)

	v_p.push_back(Point(169 - 70, 121 - 70)); // 2
	v_p.push_back(Point(871 - 70, 619 - 70)); // 5
	junctions_location.push_back(v_p);
	v_p.clear();

	// L4 junction: (2)
	v_p.push_back(Point(71 - 70, 71 - 70));   // 0
	v_p.push_back(Point(970 - 70, 670 - 70)); // 7
	junctions_location.push_back(v_p);
	v_p.clear();

	RNG rng;
	int real_robot_x = 350;
	int real_robot_y = 450;

	bool show_radius_circles = false;

	//source = imread("../images/field.png"); // Re-read the image.
	//imshow("Result", source);
	//waitKey(5000);

		source = imread("../images/field.png"); // Re-read the image.

		// Draw the junctions on the image.
		for (int i = 0; i < junctions_location.size(); i++)
		{
			for (int j = 0; j < junctions_location[i].size(); j++)
			{
				Scalar color = Scalar(0, 0, 0);
				if (i == LL)
				{
					color = Scalar(255, 0, 0);
				}
				if (i == LL2)
				{
					color = Scalar(128, 128, 128);
				}
				else if (i == TT)
				{
					color = Scalar(0, 255, 0);
				}
				else if (i == XX)
				{
					color = Scalar(0, 0, 255);
				}
				else if (i == GG)
				{
					color = Scalar(255, 255, 0);
				}
				else if (i == CC)
				{
					color = Scalar(0, 255, 255);
				}

				circle(source, Point(minimum_x + junctions_location[i][j].x, minimum_y + junctions_location[i][j].y), 3, color, 3);
			}
		}

		double error = 0.3;


		// Draw the circles around the junctions.
		for (int i = 0; i < junctions_location.size(); i++)
		{
			for (int j = 0; j < junctions_location[i].size(); j++)
			{
				Scalar color = Scalar(0, 0, 0);
				if (i == LL)
				{
					color = Scalar(255, 0, 0);
				}
				else if (i == TT)
				{
					color = Scalar(0, 255, 0);
				}
				else if (i == XX)
				{
					color = Scalar(0, 0, 255);
				}
				else if (i == GG)
				{
					color = Scalar(255, 255, 0);
				}
				else if (i == CC)
				{
					color = Scalar(0, 255, 255);
				}

				for (int u2 = 0; u2 < distances_to_junctions[i].size(); u2++) // CHECK WHY THE LOOP IS HERE.
				{
					double distance = distances_to_junctions[i][u2];
					if (show_radius_circles)
					{
						circle(source, Point(minimum_x + junctions_location[i][j].x, minimum_y + junctions_location[i][j].y), distance * (1 - error), color, 1);
						circle(source, Point(minimum_x + junctions_location[i][j].x, minimum_y + junctions_location[i][j].y), distance * (1 + error), color, 1);
					}
				}
			}
		}




		for (int k = 0; k < point_list.size(); k++)
		{
			Scalar color = Scalar(255, 0, 255);

			circle(source, Point(minimum_x + point_list[k].x, minimum_y + point_list[k].y), 1, color, 1);
		}



		ellipse(source, Point(minimum_x + real_robot_x, minimum_y + real_robot_y), Size(15, 40), 0, 0, 360, Scalar(0, 0, 0), 1);

		//int R = rng.uniform(0, steps[step].x);
		//int angle = rng.uniform(0, 360);
		real_robot_x += step.x; //R * cos(PI * angle / 180 );
		real_robot_y += step.y; //R * sin(PI * angle / 180 );

		vector<Point> point_list2;


		for (int i = 0; i < point_list.size(); i++)
		{
			double R = sqrt(pow(step.x, 2) + pow(step.y, 2));
			int angle = rng.uniform(0, 360);
			point_list[i].x += R * cos(PI * angle / 180 );
			point_list[i].y += R * sin(PI * angle / 180 );

			Scalar color = Scalar(0, 0, 0);

			bool is_reset = false;

			for (int t = 0; t < junctions_location.size(); t++) // t - Type of junction.
				{
				//bool is_certain_type = false;
					for (int u2 = 0; u2 < distances_to_junctions[t].size(); u2++) // Loop through known distances of a certain type.
					{
						bool is_certain_type_dis = false;
						for (int j = 0; j < junctions_location[t].size(); j++) // j - loop thourgh the junction of a certain type. (Loop through all junctions)
						{
							double distance = distances_to_junctions[t][u2];
							double distance_to_junction = sqrt(pow(point_list[i].x - junctions_location[t][j].x, 2) + pow(point_list[i].y - junctions_location[t][j].y, 2));
							//cout << "D: " << distance << " D_t_j: " << distance_to_junction << endl;
							if (distance_to_junction >= distance * (1 - error) && distance_to_junction <= distance * (1 + error))
							{
								is_certain_type_dis = true;
							}
						}
						if (is_certain_type_dis == false)
						{
							is_reset = true;
						}
					}
				}
				//circle(source, Point(point_list[i].x, point_list[i].y), 1, color, 1);
				if (point_list[i].x < 0 || point_list[i].x > 900 || point_list[i].y > 600 || point_list[i].y < 0 || is_reset)
				{
					//int new_i = rng.uniform(0, i);
					//int dot_x = rng.uniform(point_list[i].x - 10, point_list[i].x + 10);
					//int dot_y = rng.uniform(point_list[i].y - 10, point_list[i].y + 10);

					point_list[i].x = 0;// dot_x;
					point_list[i].y = 0;// dot_y;
					point_list.erase(point_list.begin() + i);
			}
			else
			{
				point_list2.push_back(point_list[i]);
			}
		}




		int number_of_point_list2 = point_list2.size();
		while (point_list2.size() < number_of_dots)
		{
			int dot_x = rng.uniform(-30, 30);
			int dot_y = rng.uniform(-30, 30);
			Scalar color = Scalar(255, 0, 255);

			if (number_of_point_list2 == 0)
			{
				int dot_x = rng.uniform(0, 900);
				int dot_y = rng.uniform(0, 600);
				point_list2.push_back(Point(dot_x, dot_y));

			}
			else
			{
				int random_point_list2_point = rng.uniform(0, number_of_point_list2);
				point_list2.push_back(Point(point_list2[random_point_list2_point].x + dot_x, point_list2[random_point_list2_point].y + dot_y));
			}

			circle(source, Point(minimum_x + point_list2[point_list2.size()-1].x, minimum_y + point_list2[point_list2.size()-1].y), 1, color, 1);
		}

		point_list = point_list2;




		//cout << endl << " Size: " << point_list.size() << endl;


		std::vector<cv::Point2f> points;
		for (int i = 0; i < point_list.size(); i++)
		{
			cv::Point2f p;
			p.x = point_list[i].x;
			p.y = point_list[i].y;
			points.push_back(p);
		}
		Mat centers, labels;
		 double std_div = kmeans(points, 1, labels,
		            TermCriteria(TermCriteria::EPS+TermCriteria::COUNT, 10, 1.0),
		               3, KMEANS_PP_CENTERS, centers);
		 cout << "std_div: " << std_div << endl;
			Scalar color = Scalar(0, 0, 0);
			if (std_div <= number_of_dots * 10000)
			{
			circle(source, Point(minimum_x + centers.at<float>(0,0), minimum_y + centers.at<float>(0,1)), 5, color, -1);
			}
			imshow("Result1234567890", source);
			waitKey(20);
}

void VisionThread::AutomaticCalibration(Mat frame)
{
	LinesDetector lt;
	int f_ = 1091, dist_ = 199;
	Mat ipm_image;
	int threshold = 25, minLinLength = 36, maxLineGap = 18;

	createTrackbar("threshold", "Result2", &threshold, 300);
	createTrackbar("minLinLength", "Result2", &minLinLength, 300);
	createTrackbar("maxLineGap", "Result2", &maxLineGap, 300);

	int min_i;
	double min_std_dev = 100000000;
	double min_distance;

	for (int i = 1; i <= 70; i++)
	{
		ipm_image = IPM(frame, i, dist_, f_); // Field <-> frame
		vector<Vec4i> lines = lt.GetLines(ipm_image, threshold, minLinLength, maxLineGap);

		double sum_distances = 0;
		int num_distances = 0;

		 vector<double> distances;

		for (int index1 = 0; index1 < lines.size(); index1++)
		{
			for (int index2 = index1+1; index2 < lines.size(); index2++)
			{
				Vec4i line1 = lines[index1];
				Vec4i line2 = lines[index2];

				double m_1, m_2;

				if (line1[2] != line1[0])
					m_1 = (line1[3] - line1[1]) / (line1[2] - line1[0]); // m_i - NEW
				else
					m_1 = 1000000000;

				if (line2[2] != line2[0])
					m_2 = (line2[3] - line2[1]) / (line2[2] - line2[0]); // m_i - NEW
				else
					m_2 = 1000000000;

				double angle;
				if ((m_1 * m_2) != -1)
				{
					angle = abs(atan( (m_1 - m_2) / (1 + m_1 * m_2)) );
				}
				else
				{
					angle = PI / 2;
				}

				if (angle <= 5 * (PI / 180))
				{
					//cout << "a";
					int l1_x1 = line1[0];
					int l1_x2 = line1[1];
					int l1_y1 = line1[2];
					int l1_y2 = line1[3];

					int l2_x1 = line2[0];
					int l2_x2 = line2[1];
					int l2_y1 = line2[2];
					int l2_y2 = line2[3];

					Point p1, p2;
					p1.x = l2_x1;
					p1.y = l2_y1;
					p2.x = l2_x2;
					p2.y = l2_y2;

					int x0 = l1_x1;
					int y0 = l1_y1;
					double distance1 = abs( (p2.y - p1.y) *  x0 - (p2.x - p1.x) * y0 + p2.x * p1.y - p2.y * p1.x) / sqrt(pow(p2.y - p1.y, 2) + pow(p2.x - p1.x, 2));

					x0 = l1_x2;
					y0 = l1_y2;
					double distance2 = abs( (p2.y - p1.y) *  x0 - (p2.x - p1.x) * y0 + p2.x * p1.y - p2.y * p1.x) / sqrt(pow(p2.y - p1.y, 2) + pow(p2.x - p1.x, 2));

					p1.x = l1_x1;
					p1.y = l1_y1;
					p2.x = l1_x2;
					p2.y = l1_y2;

					x0 = l2_x1;
					y0 = l2_y1;
					double distance3 = abs( (p2.y - p1.y) *  x0 - (p2.x - p1.x) * y0 + p2.x * p1.y - p2.y * p1.x) / sqrt(pow(p2.y - p1.y, 2) + pow(p2.x - p1.x, 2));

					x0 = l2_x2;
					y0 = l2_y2;
					double distance4 = abs( (p2.y - p1.y) *  x0 - (p2.x - p1.x) * y0 + p2.x * p1.y - p2.y * p1.x) / sqrt(pow(p2.y - p1.y, 2) + pow(p2.x - p1.x, 2));

					double max_distance = max(distance1, distance2);
					max_distance = max(max_distance, distance3);
					max_distance = max(max_distance, distance4);

					distances.push_back(max_distance);

					sum_distances += max_distance;
					num_distances++;
				}
			}
		}
		double average_distance = sum_distances / num_distances;

		double standard_deviation = 0;
		for (int j = 0; j < distances.size(); j++)
		{

			standard_deviation += pow(distances[j] - average_distance, 2);
		}

		if (standard_deviation <= min_std_dev && distances.size() > 1)
		{
			cout << distances.size() << endl;
			min_std_dev = standard_deviation;
			min_i = i;
			min_distance = average_distance;
		}

		distances.clear();
		lines.clear();
	}
	cout << "Tilt: " << min_i << " Distance: " << min_distance << "Stn Dev: " << min_std_dev << endl;
}

void VisionThread::AutomaticCalibration2(Mat frame)
{
	LinesDetector lt;
	int f_ = 1091, dist_ = 199;
	Mat ipm_image;
	int threshold = 25, minLinLength = 36, maxLineGap = 18;

	createTrackbar("threshold", "Result2", &threshold, 300);
	createTrackbar("minLinLength", "Result2", &minLinLength, 300);
	createTrackbar("maxLineGap", "Result2", &maxLineGap, 300);

	int min_i;
	double prependicular = 0;
	double min_distance;

	for (int i = 1; i <= 70; i++)
	{
		ipm_image = IPM(frame, i, dist_, f_); // Field <-> frame
		vector<Vec4i> lines = lt.GetLines(ipm_image, threshold, minLinLength, maxLineGap);

		int num_prependiculare = 0;

		 vector<double> distances;

		for (int index1 = 0; index1 < lines.size(); index1++)
		{
			for (int index2 = index1+1; index2 < lines.size(); index2++)
			{
				Vec4i line1 = lines[index1];
				Vec4i line2 = lines[index2];

				double m_1, m_2;

				if (line1[2] != line1[0])
					m_1 = (line1[3] - line1[1]) / (line1[2] - line1[0]); // m_i - NEW
				else
					m_1 = 1000000000;

				if (line2[2] != line2[0])
					m_2 = (line2[3] - line2[1]) / (line2[2] - line2[0]); // m_i - NEW
				else
					m_2 = 1000000000;

				if (m_1 * m_2 >= -5 && m_1 * m_2 <= -0.5)
				{
					num_prependiculare++;
				}
			}
		}
		if (num_prependiculare >= prependicular)
		{
			prependicular = num_prependiculare;
			min_i = i;
		}
		distances.clear();
		lines.clear();
	}
	cout << "Tilt: " << min_i << "Per: " << prependicular << endl;
}

void VisionThread::CannyThreshold(Mat src, Mat src_gray, Mat src_hsv, Mat detected_edges, Mat src_gray2,
int edgeThresh, int low_red, int low_green, int low_blue, int high_red, int high_green, int high_blue,
int low_red2, int low_green2, int low_blue2, int high_red2, int high_green2, int high_blue2, int ratio,
int kernel_size, Mat& White_zone, Mat& Goal_area, Mat& Robots)
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
	 Mat Green_Field, White_Field, Robot_Field, Goal_Area;
	 Mat F_B, G_B, W_B, R_B, Goal_B;
	 F_B = dstC.clone();
	 G_B = src_gray.clone();
	 W_B = src_gray2.clone();
	 Goal_B = dstC.clone();

	 bitwise_and(F_B, G_B, Green, noArray());
	 bitwise_and(clo, clo, Green_Field, Green);
	 imshow("Grass", Green_Field);

	 bitwise_not(F_B, Goal_B);
	 bitwise_and(clo, clo, Goal_Area, Goal_B);
	 imshow("Goal Area", Goal_Area);

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
Goal_area = Goal_Area;

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

double VisionThread::Ellipse(Mat src, Mat src_gray, int thresh, int max_thresh, RNG rng, Mat& white)
{
	  Mat threshold_output;
	  vector<vector<Point> > contours;
	  vector<Vec4i> hierarchy;

	  Mat srcC, src_grayC;

	  srcC = src.clone();
	  src_grayC = src_gray.clone();

	  /// Detect edges using Threshold
	  threshold( src_grayC, threshold_output, thresh, 255, THRESH_BINARY );
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
      double max_area = 100;
      Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
double ratio = -1;
	  for( int i = 0; i< contours.size(); i++ )
	     {
	       // contour
	       drawContours( drawing1, contours, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
	       // ellipse
if ( (double) minEllipse[i].size.height / minEllipse[i].size.width < 2)
	       if (minEllipse[i].size.area() > max_area)
	       {
	    	   ratio = (double) minEllipse[i].size.height / minEllipse[i].size.width;
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
		  //cout << minEllipse[max_index].size.height << " " << minEllipse[max_index].size.width << endl;
ellipse( srcC, minEllipse[max_index], color, 2, 8 );
	  }
	  /// Show in a window
	  namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
	  imshow( "Contours", srcC );
	  white = drawing;
	  return ratio;
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

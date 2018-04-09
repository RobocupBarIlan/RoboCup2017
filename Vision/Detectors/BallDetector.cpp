#include "BallDetector.h"

		static int call_id=0;

/*
 * static members init:
 */

		bool BallDetector::IsFirstRun=true;
		double BallDetector::ball_hue_histogram[HUE_QUANTIZATION];



BallDetector::BallDetector() {
	// TODO Auto-generated constructor stub

}

BallDetector::~BallDetector() {
	// TODO Auto-generated destructor stub
}



/*
 * This is the main method which is called by the Vision thread to detect a ball.
 * Returns - center of ball and radius to the caller.
 */

void BallDetector::GetBallCenter(Point& returned_center,int& returned_radius)
{
	Mat frame, frame_hsv,field_mat, original_field_mat, whites_mat, field_space_mat,white_blobs_clone,white_blobs,white_bulbs_edges; //frame=origianl image. frame_hsv=image after transform to hsv. field_mat=only pixels in bounds of field's green after calibration are 0 (all the rest 255). whites_mat=only white pixels in image. field_space_mat=all pixels below bounding_horizontal_line (i.e -assumed to be field). candidates_mat=pixels which are not field and white and below bounding horizontal line.
	Mat hsv_channels[NUM_CHANNELS]; //Will contain all 3 HSV channels splitted.
	Mat bgr_channels[NUM_CHANNELS]; //Will contain all 3 BGR channels splitted.
	uchar field_min_hue, field_max_hue; //Will mark which pixel is a field pixel after calib.
	ushort bounding_horizontal_line; //Will mark the
	Point center;


	VisionThread::SafeReadeCapturedFrame(frame);


//	namedWindow("my_frame_check");
//	imshow("original_frame",frame);
//	waitKey(0);

//			split(frame, bgr_channels); //Split to B,G,R channels so we can manipulate the image later on.
////			//Reduce noise:
//			GaussianBlur(bgr_channels[0], bgr_channels[0], Size(3, 3), 1, 1);
//			GaussianBlur(bgr_channels[1], bgr_channels[1], Size(3, 3), 1, 1);
//			GaussianBlur(bgr_channels[2], bgr_channels[2], Size(3, 3), 1, 1);
//			merge(bgr_channels, NUM_CHANNELS, frame); //Merge the channels back to one matrix after manipulation.

////			Unsharp filter (HPF):
//			Mat blurred_frame;
//			GaussianBlur(frame, blurred_frame, cv::Size(3, 3), 0.5,0.5);
//			cv::addWeighted(frame, 1.5, blurred_frame, -0.5, 0, frame);

			cvtColor(frame, frame_hsv, CV_BGR2HSV); //Convert original RGB representation to HSV representation.
			split(frame_hsv, hsv_channels); //Split to H,S,V channels so we can use the hue,saturation&value matrices more conviniently later on.
			BallDetector::FieldCalibration(hsv_channels[0], field_min_hue, field_max_hue);
			field_mat = Mat::zeros(frame.rows, frame.cols, CV_8UC1); //Generate a 1-channel matrix with size of original image (unsigned char). set all pixels to initail value 255=(11111111)_2
			whites_mat = Mat::zeros(frame.rows, frame.cols, CV_8UC1); //Generate a 1-channel matrix with size of original image (unsigned char).


			//Generate the field_mat and whites_mat:
			for (int i = 0; i < frame.rows; i++)
			{
				for (int j = 0; j < frame.cols; j++)
				{
					//Check hue value against bounds:
					if (hsv_channels[0].at<uchar>(i, j) >= field_min_hue && hsv_channels[0].at<uchar>(i, j) <= field_max_hue && hsv_channels[1].at<uchar>(i, j)>MAX_WHITE_SATURATION / 2.0)
					{
						field_mat.at<uchar>(i, j) = 255; //If in range (i.e- field pixel) set to 0.
					}
					//Check saturation& value against bounds:

					if (hsv_channels[1].at<uchar>(i, j) <= MAX_WHITE_SATURATION && hsv_channels[2].at<uchar>(i, j) >= MIN_WHITE_VALUE)
					{
						whites_mat.at<uchar>(i, j) = 255; //If in range (i.e- white pixel) set to 11111111 in binary.
					}
				}
			}
			medianBlur(field_mat,field_mat,5);
			//imshow("frame_check", frame);
			//waitKey(1);
			//namedWindow( "whites_mat", WINDOW_AUTOSIZE );
			//imshow("whites_mat",whites_mat);
			//waitKey(1);
			//Calculate bounding horizontal line for field in image:
			BallDetector::CalculateBoundingHorizontalLine(field_mat, bounding_horizontal_line);

			//Generate field_space_mat=all pixels below bounding_horizontal_line (i.e -assumed to be field) are 255=(11111111)_2 and 0 else.
			field_space_mat = Mat::zeros(frame.rows, frame.cols, CV_8UC1);
			for (int i = frame.rows - 1; i > 0; i--)
			{
				if (i >= bounding_horizontal_line) //If below bounding_horizontal_line
				{
					for (int j = 0; j < frame.cols; j++) //Set all values to 255 there:
					{
						field_space_mat.at<uchar>(i, j) = 255;
					}
				}
				else
				{
					break;
				}
			}
//			imshow("field_space_mat", field_space_mat);
//			waitKey(1);
			//waitKey(1);




			Mat frame_luminance;
			Mat frame_yuv;
			Mat frame_edges,frame_edges_clone;
			cvtColor(frame, frame_yuv, CV_BGR2YUV);
			Mat yuv_channels[3];
			split(frame_yuv, yuv_channels);
			frame_luminance = yuv_channels[0];

		//	blur(frame_luminance,frame_luminance,Size(3,3));



			//GaussianBlur(frame_luminance, frame_luminance, cv::Size(3, 3), 1.5,1.5);


			//medianBlur(frame_luminance,frame_luminance,3);

			GaussianBlur(frame_luminance, frame_luminance, cv::Size(3, 3), 2,2);
			//blur(frame_luminance,frame_luminance,Size(3,3));
			medianBlur(frame_luminance,frame_luminance,3);


			//Unsharp:
			Mat blurred_luminance;
			GaussianBlur(frame_luminance, blurred_luminance, cv::Size(3, 3), 2,2);
			cv::addWeighted(frame_luminance,2, blurred_luminance, -1, 0, frame_luminance);

			GaussianBlur(frame_luminance, frame_luminance, cv::Size(3, 3), 2,2);
			//blur(frame_luminance,frame_luminance,Size(3,3));
			medianBlur(frame_luminance,frame_luminance,3);



//			Mat grad_x, grad_y,grad;
//			Mat abs_grad_x, abs_grad_y;
//			/// Gradient X
//			Sobel( frame_luminance, grad_x, CV_16S, 1, 0, 3);
//			/// Gradient Y
//			Sobel(frame_luminance , grad_y, CV_16S, 0, 1, 3);
//
//			convertScaleAbs( grad_x, abs_grad_x );
//			convertScaleAbs( grad_y, abs_grad_y );
//			addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad,CV_8UC1 );
//			frame_luminance=frame_luminance-grad; //Enhance edges

//			GaussianBlur(frame_luminance, frame_luminance, cv::Size(3, 3), 1,1);
//			medianBlur(frame_luminance,frame_luminance,3);

			//GaussianBlur(frame_luminance, frame_luminance, Size(5, 5), 1);
			bitwise_and(frame_luminance, field_space_mat, frame_luminance);
			//medianBlur(frame_gray, frame_gray,3);
//			imshow("frame_bitwise_and_field_space_mat", frame_luminance);
//			waitKey(1);
			//waitKey(1);




			int thresh = 19000;
			Canny(frame_luminance, frame_edges, thresh, thresh * 2, 7);
//			imshow("original_frame_canny", frame_edges);
//			waitKey(1);

			frame_edges_clone = frame_edges.clone();


//
//			Mat whites_edges;
//			Mat blurred_whites_mat;
//			medianBlur(whites_mat,blurred_whites_mat,3);
//			imshow("blurred_whites_mat",blurred_whites_mat);
//
//			Mat grad_x, grad_y,grad;
//			Mat abs_grad_x, abs_grad_y;
//			/// Gradient X
//			Sobel( blurred_whites_mat, grad_x, CV_16S, 1, 0, 3);
//			/// Gradient Y
//			Sobel(blurred_whites_mat , grad_y, CV_16S, 0, 1, 3);
//
//			convertScaleAbs( grad_x, abs_grad_x );
//			convertScaleAbs( grad_y, abs_grad_y );
//			addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad,CV_8UC1 );
//			whites_edges=grad;
//
//			Canny(blurred_whites_mat,whites_edges,1,1,7);
//			imshow("whites_edges",whites_edges);





			//frame_edges=ev_mat_normalized;
			//Remove space out of field:
			//bitwise_and(frame_edges, field_space_mat, frame_edges);
			frame_edges_clone = frame_edges.clone();

			//imshow("frame_edges",frame_edges);



			//Remove lines using HoughLines:

			vector<Vec2f> lines;
			HoughLines(frame_edges, lines, 1, CV_PI/180, 60);
			double y_double;
			int y;
			float rho,theta;
			  for( size_t i = 0; i < lines.size(); i++ )
			  {
			     rho = lines[i][0];
			     theta = lines[i][1];
		    	 if(theta>0)
		    	 {
					 for(int x=0;x<frame.cols;x++)
					 {
						 	 y_double=(-1/tan(theta))*x+rho/sin(theta);
							 y=cvRound(y_double);
							 if(y>=0 && y<frame.rows)
							 {
								 frame_edges_clone.at<uchar>(y,x)=0;
								 //frame_edges_clone.at<uchar>(max(0,cvFloor(y_double)),x)=0;
								 //frame_edges_clone.at<uchar>(min(frame.rows-1,cvCeil(y_double)),x)=0;
							 }
					 }

		    	 }
		    	 else //Vertical line - x=rho
		    	 {
		    		 for(int y=0;y<frame.rows;y++)
		    		 {
		    			 frame_edges_clone.at<uchar>(y,cvRound(rho))=0;
		    		 }
		    	 }

			  }

//			  imshow("canny_after_lines_removal",frame_edges_clone);
//			  waitKey(10);
				vector<BallCandidateRansac> found_circles;
				CircleFitRansac(frame,frame_edges_clone,found_circles,MIN_BALL_DIAMETER/2,MAX_BALL_DIAMETER/2);
				//cout<<"found circles #:"<<found_circles.size()<<endl;
				if(found_circles.size()==0) //No candidate found:
				{

					imshow("frame",frame);
					waitKey(10);
					if(IsFirstRun)
					{
						cout<<"Can\'t find any ball candidate for calibration! program is terminated."<<endl;
					}
					//No ball found:
					returned_center.x=-1;
					returned_center.y=-1;
					returned_radius=-1;
					return;
				}
				//Sort the found_circles by the support they have (in descending order):
				std::sort(found_circles.begin(), found_circles.end(), BallCandidateRansac::by_support());
				bool isBallImageSelected=false; //A flag that indicates if the user selected the ball's image (for ball colors calibration process).
				char userInput; //Holds the answer of the user.
				Rect candidate_bounding_rect;

				//Ball's colors calibration process - if first run:
				if(IsFirstRun) //Ask the user which part of the image is the ball - so we have hue histogram of colors for the ball:
				{
					// createButton("dummy_button", my_button_cb, NULL, QT_PUSH_BUTTON, 1);
					for(int i=0;i<found_circles.size();i++) //For each circle candidate - ask the user if it is the ball:
					{

						//TODO show image (crope rectangle around) of candidate and ask the user if it is the ball or not - if it is(user input y/n) - set the hue vector/histogram (which this class should hold). Later on implement the color correlation method for detection of ball.
						//cout<<found_circles[i].m_support<<endl;
						BallDetector::GetCircleBoundingRectangleInFrame(found_circles[i].m_center,found_circles[i].m_radius,frame.rows,frame.cols,candidate_bounding_rect);
						if(candidate_bounding_rect.width!=-1) //If ball center is in frame (o.w can't calibrate the ball's colors):
						{
							imshow("candidate",frame(candidate_bounding_rect));
							cout<<"Does the following image (\"candidate\" window) show the ball and only the ball? [enter y for \'yes\' and n for \'no\']"<<endl;
							waitKey(30);
							cin>>userInput;
							while(userInput!='y' && userInput!='n')
							{
								cout<<"You must enter \'y\' for yes or \'n\' for no !"<<endl;
								cout<<"Does the following image (\"candidate\" window) shows the ball and only the ball? [enter y for \'yes\' and n for \'no\']"<<endl;
								cin>>userInput;
							}

							if(userInput=='y') //User selected the ball's image.
							{
								isBallImageSelected=true;
							}
							if(isBallImageSelected)
							{
								break;
							}
						}
					}

					if(!isBallImageSelected) //No image selected - can't continue with program - terminate it.
					{
						exit(EXIT_FAILURE);
					}

					//Take the selected image of ball (by using the candidate_bounding_rect) and perform ball colors calibration:
					//field_min_hue=MIN_GREEN_HUE;
					//field_max_hue=MAX_GREEN_HUE;
					BallDetector::BallHistogramCalibration(hsv_channels[0],candidate_bounding_rect,field_min_hue, field_max_hue);
					IsFirstRun=false;
				}
				else //If not first run - ball's calibration is already done - we can check which candidate is the ball:
				{
					double correlation=0;
					const double COLORS_CORRELATION_THRESHOLD=0.75; //The minimum colors correlation required for a candidate to be considered as the ball. Heuristic.
					const double FIELD_PIXELS_THRESHOLD=0.3; //The maximum percentage allowed of field pixels in image of candidate. Heuristic.
					//field_min_hue=MIN_GREEN_HUE;
					//field_max_hue=MAX_GREEN_HUE;
					Mat candidate;
					uint num_field_pixels;
					double percentage_field_pixels=0;
					int check=0;
					for(uint i=0;i<found_circles.size();i++) //For each circle (from the most supported to the least - check the colors correlation):
					{
						//Get bounding rectangle of candidate. width and height will be -1 if the center of the ball is out of the frame!
						BallDetector::GetCircleBoundingRectangleInFrame(found_circles[i].m_center,found_circles[i].m_radius,frame.rows,frame.cols,candidate_bounding_rect);
						if(candidate_bounding_rect.width!=-1 && candidate_bounding_rect.width>0) //If candidate's center is in frame - compute the colors correlation.
						{


							candidate=hsv_channels[0];
							candidate=candidate(candidate_bounding_rect);

							BallDetector::CountNumFieldPixels(candidate,field_min_hue,field_max_hue,num_field_pixels);
							percentage_field_pixels=num_field_pixels/(0.0+candidate.rows*candidate.cols);

							//field_min_hue=0;
							//field_max_hue=0;
							BallDetector::CalculateBallColorsCorrelation(hsv_channels[0],candidate_bounding_rect,correlation,field_min_hue,field_max_hue);

							if(correlation>=COLORS_CORRELATION_THRESHOLD && percentage_field_pixels<=FIELD_PIXELS_THRESHOLD)
							{
								//cout<<"percentage:"<<percentage_field_pixels<<endl;
								//cout<<"correlation_of_max:"<<correlation<<endl;
								//cout<<"field_pixels:"<<percentage_field_pixels<<endl;
								returned_center.x=found_circles[i].m_center.x;
								returned_center.y=found_circles[i].m_center.y;
								returned_radius=found_circles[i].m_radius;
								circle(frame,found_circles[i].m_center,found_circles[i].m_radius,Scalar(0,0,255),2);
								namedWindow("frame",WINDOW_AUTOSIZE);
								imshow("frame",frame);
								waitKey(1);
								return;
							}
						}
					}
				}



				imshow("frame",frame);
				waitKey(1);
			//clock_t end = clock();
			//double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
			//cout << "elapsed time:" << elapsed_secs << endl;


			returned_center.x=-1;
			returned_center.y=-1;
			returned_radius=-1;
			return;
}




/*This method calibrates the green field hue bounds. It takes the hue matrix and returns the- field_min_hue and field_max_hue.
	We create the histogram of hue for the given matrix and take PERCENTAGE_THRESHOLD (=90%) in the middle of the green spectrum -
	assuming values on the edges of the green spectrum might be noise.*/
void BallDetector::FieldCalibration(Mat& hue_matrix, uchar& field_min_hue, uchar& field_max_hue)
{
	int hue_histogram[HUE_DISCRETIZATION];
	int total_num_of_green_pixels=0; //Tells how many green pixel in total range of greens as specified by MIN_GREEN_HUE & MAX_GREEN_HUE.
	int sum_of_green_pixels_in_histogram; //Will count green pixels until we reach the threshold defined by PERCENTAGE_THRESHOLD.
	const double PERCENTAGE_THRESHOLD = 0.85; //Determines how many green pixels we get rid of. the larger the less.
	const int TOTAL_NUM_OF_PIXELS = hue_matrix.cols*hue_matrix.rows;
	const int NOT_ENOUGH_GREENS_IN_IMAGE_THRESHOLD = 0.2; //Heuristic. might be changed. if not enough green pixels we can't estimate the field!.
	//Initialize the hue histogram with zeros:
	for (uint i = 0; i < HUE_DISCRETIZATION; i++)
	{
		hue_histogram[i] = 0;
	}
	//generate hue histogram:
	for (uint i = 0; i < hue_matrix.rows; i++)
	{
		for (uint j = 0; j < hue_matrix.cols; j++)
		{
			hue_histogram[hue_matrix.at<uchar>(i, j)] = hue_histogram[hue_matrix.at<uchar>(i, j)] + 1;
		}
	}
	//Sum all pixels to get total_num_of_green_pixels.
	for (uint k = 0; k < HUE_DISCRETIZATION; k++)
	{
		total_num_of_green_pixels = total_num_of_green_pixels + hue_histogram[k];
	}
	if (total_num_of_green_pixels < NOT_ENOUGH_GREENS_IN_IMAGE_THRESHOLD*TOTAL_NUM_OF_PIXELS) //If not enough green pixels in image:
	{
		cout << "Not enough green pixels in image to do field green calibration!" << endl;
		//Return default bounds of green- we don't have enough data to estimate.
		field_min_hue = MIN_GREEN_HUE;
		field_max_hue = MAX_GREEN_HUE;
		return;
	}
	//Now that we have histogram and sum of green pixels- we check where PERCENTAGE_THRESHOLD (=90%)of the green pixels lie on the spectrum:
	int i = MIN_GREEN_HUE;
	sum_of_green_pixels_in_histogram = hue_histogram[i];
	while (sum_of_green_pixels_in_histogram / (total_num_of_green_pixels + 0.0) < (1 - PERCENTAGE_THRESHOLD) / 2)
	{
		i++;
		sum_of_green_pixels_in_histogram = sum_of_green_pixels_in_histogram + hue_histogram[i];
	}
	field_min_hue = i; //field min hue is found.
	i = MAX_GREEN_HUE;
	sum_of_green_pixels_in_histogram = hue_histogram[i];
	while (sum_of_green_pixels_in_histogram / (total_num_of_green_pixels + 0.0) < (1 - PERCENTAGE_THRESHOLD) / 2)
	{
		i--;
		sum_of_green_pixels_in_histogram = sum_of_green_pixels_in_histogram + hue_histogram[i];
	}
	field_max_hue = i; //field min hue is found.

	//cout << "field max " << int(field_max_hue)<<endl; //TEST
	//cout << "field min "<< int(field_min_hue)<<endl;  //TEST


}








/*This method calculates the bounding horizontal line of the field. i.e - it is the horizontal line which PERCENTAGE_THRESHOLD of the pixels
marked as field by field_mat are *below*.*/
void BallDetector::CalculateBoundingHorizontalLine(Mat& field_mat, ushort& bounding_horizontal_line)
{
	const double PERCENTAGE_THRESHOLD = 0.99; // Assumption is that we have approx. 1% noise:
	uint num_of_field_pixels_below_bounding_line = 0;
	uint total_num_of_field_pixels = 0;
	//uint bounding_horizontal_line_ = field_mat.rows; //Initialize the bounding horizontal line to the lowest row of image.
	uint* sum_of_rows_vector = new uint[field_mat.rows]; //Dyanmic array - Will hold the sum of each row in field_mat.

														 //Initialize sum_of_rows_vector to zeros:
	for (int i = 0; i < field_mat.rows; i++)
	{
		sum_of_rows_vector[i] = 0;
	}


	//sum all of green points row by row:
	for (int i = field_mat.rows - 1; i > 0; i--)
	{
		for (int j = 0; j < field_mat.cols; j++)
		{
			sum_of_rows_vector[i] = sum_of_rows_vector[i] + ((field_mat.at<uchar>(i, j)>0)?1:0);
		}
		total_num_of_field_pixels = total_num_of_field_pixels + sum_of_rows_vector[i]; //Sum of sum of rows = total num. of field pixels.
	}

	//push the bounding_horizontal_line_ up until  PERCENTAGE_THRESHOLD of the 'field pixels' are below.
	ushort row_num = field_mat.rows - 1;
	num_of_field_pixels_below_bounding_line = sum_of_rows_vector[row_num]; //Initialize to sum of last row of image.
	while (num_of_field_pixels_below_bounding_line / (total_num_of_field_pixels + 0.0) < PERCENTAGE_THRESHOLD)
	{
		//cout << "row_num:" << row_num << "," << "proportion:" << to_string(num_of_field_pixels_below_bounding_line / (total_num_of_field_pixels + 0.0)) << endl;
		row_num--;
		num_of_field_pixels_below_bounding_line = num_of_field_pixels_below_bounding_line + sum_of_rows_vector[row_num]; //add the row above.
	}
	bounding_horizontal_line = row_num; //row_num contains the bounding horizontal line after loop.

										//cout << "bounding horizontal_line:" << bounding_horizontal_line << "\n" <<endl; //TEST

	delete[] sum_of_rows_vector; //Deallocate dyanmic array.
}


double BallDetector::Sigmoid(double x)
{
	return 1/(1+exp(-1*x));
}


/*
	My implementation for the Kasa algorithm:
	The following algorithm solves for the best circle fit of a curve using algebra-
	Explanation:
	First we know that the equation of a circle is:
	(x - x0)^2 + (y - y0)^2 = r^2
	Expand it we get:
	(x^2 - 2 * x0 * x + x0^2) + (y^2 - 2 * y0 * y + y0^2) = r^2
	Reordering them to form "ax + by = c":
	(2 * x0 * x) + (2 * y0 * y) + (r^2 - x0^2 - y0^2) = x^2 + y^2
	So we get:
	c1 = 2 * x0;
	c2 = 2 * y0;
	c3 = r^2 - x0^2 - y0^2

	So the matrix is:
	A               c             Y
	============================================
	[ 2 * x1, 2 * y1, 1 ]            [ x1^2 + y1^2 ]
	[ 2 * x2, 2 * y2, 1 ]   [ c1 ]   [ x2^2 + y2^2 ]
	[ 2 * x3, 2 * y3, 1 ] * [ c2 ] = [ x3^2 + y3^2 ]
	[        ...        ]   [ c3 ]   [     ...     ]
	[ 2 * xn, 2 * yn, 1 ]            [ xn^2 + yn^2 ]
	now solve for c we get the coefficients of the best fit circle equation.

*/
void BallDetector::CircleFitKasa(vector<Point>& contour,Point& center, double& radius)
{
	if (contour.size() >= 3) //O.w we can't fit
	{
		double normalization_factor = contour.size(); //pow(max(frame_width, frame_height), 2);
		Mat A = Mat::ones(Size(3, contour.size()), CV_64FC1); //Using ones because the 3rd column should be all 1s.
		Mat A_transpose;
		Mat A_transpose_mult_A;
		Mat A_transpose_mult_Y;
		Mat c = Mat::zeros(Size(1, 3), CV_64FC1);
		Mat Y = Mat::zeros(Size(1, contour.size()), CV_64FC1);
		for (uint i = 0; i < contour.size(); i++) //Init matrices:
		{
			A.at<double>(i, 0) = (contour[i].x/ normalization_factor);
			A.at<double>(i, 1) = (contour[i].y/ normalization_factor);
			A.at<double>(i, 2) = A.at<double>(i, 2) / normalization_factor;
			Y.at<double>(i, 0) = pow((contour[i].x/normalization_factor),2)+pow((contour[i].y/normalization_factor),2);
		}

		transpose(A, A_transpose);
		A_transpose_mult_A = A_transpose*A;
		A_transpose_mult_Y = A_transpose*Y;
		solve(A_transpose_mult_A, A_transpose_mult_Y, c); //Solve the linear system. (might use LU decomposition if matrix is singular).
		c.at<double>(0, 0) = normalization_factor*c.at<double>(0, 0);
		c.at<double>(1, 0) = normalization_factor*c.at<double>(1, 0);
		c.at<double>(2, 0) = normalization_factor*c.at<double>(2, 0);
		center.x = cvRound(0.5*c.at<double>(0, 0));
		center.y = cvRound(0.5*c.at<double>(1, 0));
		radius = sqrt(c.at<double>(2, 0) + pow(0.5*c.at<double>(0, 0), 2) + pow(0.5*c.at<double>(1, 0), 2));
		//std::cout << "center.x:  " << center.x << "center.y:  " << center.y << "radius:  " << radius << endl;
	}
	else //Can't fit circle - return error
	{
		center.x = -1;
		center.y = -1;
		radius = -1;
	}
}



/*
 * This method gets the frame_edges mat [canny & lines deletion performed on original frame] (CV_8UC1 type) and searches for a circle using the RANSAC algorithm-
 * It selects 3 edge points randomly and checks if the circle they are on has many support points (points which are approximately , up to a threshold, on the circle).
 *________________________________________________________________________________________
 * min_radius and max_radius define the min and max circle's radius to search for.
 */
void BallDetector::CircleFitRansac(Mat& frame,Mat& frame_edges,vector<BallCandidateRansac>& found_circles,int min_radius,int max_radius)
{

	const int THRESHOLD_DISTANCE=2; //up to  pixels of error between distance from checked circle's center and its center.
	const int SUPPORT_THRESHOLD=cvRound(PI*MIN_BALL_DIAMETER); //minimum number of edge points to support a circle.
	int random_point_index_1;
	int random_point_index_2;
	int random_point_index_3;
	vector<Point2i> edge_points;
	//int cnt=0;
	for(int i=0;i<frame_edges.rows;i++)
	{
		for(int j=0;j<frame_edges.cols;j++)
		{
			if(frame_edges.at<uchar>(i,j)==255)
			{
				edge_points.push_back(Point(j,i));
			}
		}
	}

	if(edge_points.size()>=3) //If at least 3 edge points (for circle) in frame:
	{
//		cout<<"edge_points:"<<edge_points.size()<<endl;
		const int NUM_ITERATIONS=1000000/edge_points.size();
		Point checked_circle_center;
		double checked_circle_radius;
		int num_supporting_points;
		double dist_from_center;
		vector<Point2i> chosen_3_points;
		const int MAX_RESHUFFLES=80; //Heuristic. we will re-shuffle if the shuffled (2nd/3rd point) is not in a maximum of MAX_BALL_DIAMETER distance from the first point shuffled.
		int reshuffles_count; //Will count how many shuffles we already did. we don't re-shuffle infinitely to prevent deadlocks.
		for(int i=0;i<NUM_ITERATIONS;i++)
		{
			chosen_3_points.clear();
			reshuffles_count=0;
			//Pick randomly 3 points:
			random_point_index_1=rand()%edge_points.size();
			random_point_index_2=rand()%edge_points.size();
			while(reshuffles_count<MAX_RESHUFFLES&& (random_point_index_2==random_point_index_1 || sqrt(pow(edge_points[random_point_index_1].x-edge_points[random_point_index_2].x,2)+pow(edge_points[random_point_index_1].y-edge_points[random_point_index_2].y,2))>MAX_BALL_DIAMETER))
			{
				random_point_index_2=rand()%edge_points.size();
				reshuffles_count++;
			}
			if(reshuffles_count==MAX_RESHUFFLES) //It means that we could not shuffle an appropriate point:
			{
				continue;
			}

			random_point_index_3=rand()%edge_points.size();
			reshuffles_count=0;
			while(reshuffles_count<MAX_RESHUFFLES &&(random_point_index_3==random_point_index_1 || sqrt(pow(edge_points[random_point_index_1].x-edge_points[random_point_index_3].x,2)+pow(edge_points[random_point_index_1].y-edge_points[random_point_index_3].y,2))>MAX_BALL_DIAMETER || sqrt(pow(edge_points[random_point_index_3].x-edge_points[random_point_index_2].x,2)+pow(edge_points[random_point_index_3].y-edge_points[random_point_index_2].y,2))>MAX_BALL_DIAMETER))
			{
				random_point_index_3=rand()%edge_points.size();
				reshuffles_count++;
			}
			if(reshuffles_count==MAX_RESHUFFLES) //It means that we could not shuffle an appropriate point:
			{
				continue;
			}


			chosen_3_points.push_back(Point(edge_points[random_point_index_1].x,edge_points[random_point_index_1].y));
			chosen_3_points.push_back(Point(edge_points[random_point_index_2].x,edge_points[random_point_index_2].y));
			chosen_3_points.push_back(Point(edge_points[random_point_index_3].x,edge_points[random_point_index_3].y));



					BallDetector::CircleFitKasa(chosen_3_points,checked_circle_center,checked_circle_radius);

					//check only circles with searched radius:
					if(checked_circle_radius>=min_radius && checked_circle_radius<=max_radius)
					{
						num_supporting_points=0;
						for(uint j=0;j<edge_points.size();j++) //Search for circle supporting points:
						{
							dist_from_center=sqrt(pow(checked_circle_center.x-edge_points[j].x,2)+pow(checked_circle_center.y-edge_points[j].y,2));
							//cout<<"dist:"<<dist_from_center<<endl;
							if(dist_from_center>=checked_circle_radius-THRESHOLD_DISTANCE && dist_from_center<=checked_circle_radius+THRESHOLD_DISTANCE)
							{
								num_supporting_points++;
							}
						}
						if(num_supporting_points>SUPPORT_THRESHOLD) //Add the circle candidate to the vectors of centers and radii:
						{
							found_circles.push_back(BallCandidateRansac(Point(checked_circle_center.x,checked_circle_center.y),cvRound(checked_circle_radius),num_supporting_points));
						}

			}
		}

	}
}

/*
 * This method gets the circle's center,radius and the frame's number of rows and columns and returns the bounding rectangle in the bounding_rect object.
 * The method won't work if the circle's center point is not completely in frame, a thing which will result in returning NULL in the bounding_rect!
 */
void BallDetector::GetCircleBoundingRectangleInFrame(Point& center,int& radius,int& frame_rows,int& frame_cols,Rect& bounding_rect)
{
	int left,top,width,height;
	if(center.x>0 && center.x<frame_cols && center.y>0 && center.y<frame_rows) //If circle's center is in frame:
	{
		left=max(0,center.x-radius);
		top=max(0,center.y-radius);
		width=min(frame_cols-left,2*radius);
		height=min(frame_rows-top,2*radius);
		bounding_rect=Rect(left,top,width,height);
	}
	else
	{
		bounding_rect.x=-1;
		bounding_rect.y=-1;
		bounding_rect.width=-1;
		bounding_rect.height=-1;
	}
}



/*
 *This method sets the ball_hue_histogram values for the use of the CalculateBallColorsCorrelation later on to prevent misses/false alarms.
 * The values of the field_min_hue and field_max_hue are needed to delete the field's colors so we won't use them in colors correlation!.
 */
void BallDetector::BallHistogramCalibration(Mat& hue_mat , Rect& ball_roi ,uchar& field_min_hue ,uchar& field_max_hue )
{
	Mat ball_hue_roi=hue_mat(ball_roi);//Get region of ball.
	//Initialize the ball_hue_histogram class static array to zeros first:
	for(int i=0;i<HUE_QUANTIZATION;i++)
	{
		ball_hue_histogram[i]=0;
	}


	//Calculate the histogram of the ball [some field pixels will be added however it should be a small number if candidate chosen properly]:
	for(int i=0;i<ball_hue_roi.rows;i++)
	{
		for(int j=0;j<ball_hue_roi.cols;j++)
		{
			//if(ball_hue_roi.at<uchar>(i,j)<field_min_hue || ball_hue_roi.at<uchar>(i,j)>field_max_hue) //Don't count the field colors into the colors histogram.
				ball_hue_histogram[ball_hue_roi.at<uchar>(i,j)]++; //increment by 1 the number of pixels with the (i,j)'s hue value.
		}
	}

	//Normalize the array. first divide by rowsXcols (to prevent possible numeric errors with large numbers) and then normalize for norm_2=1:
	for(int i=0;i<HUE_QUANTIZATION;i++)
	{
		ball_hue_histogram[i]=ball_hue_histogram[i]/(ball_hue_roi.rows*ball_hue_roi.cols);
	}

	double sqrt_sum_squares=0; //Will hold the normalization factor for norm_2 = 1 for the ball_hue_histogram vector.

	for(int i=0;i<HUE_QUANTIZATION;i++)
	{
		sqrt_sum_squares=sqrt_sum_squares+pow(ball_hue_histogram[i],2);
	}
	sqrt_sum_squares=sqrt(sqrt_sum_squares);

	for(int i=0;i<HUE_QUANTIZATION;i++) //Normalize the ball_hue_histogram vector for norm_2 to be equal 1 (we need it because we later on compute colors correlation).
	{
		ball_hue_histogram[i]=ball_hue_histogram[i]/sqrt_sum_squares;
	}


}



/*
 * This method gets the candidate ball hue matrix and computes the colors correlation to the user's selected ball (the user selects the ball on first call to get ball center).
 */
void BallDetector::CalculateBallColorsCorrelation(Mat& hue_mat,Rect& candidate_bounding_rect,double& correlation,uchar& field_min_hue ,uchar& field_max_hue)
{
	Mat candidate_hue=hue_mat(candidate_bounding_rect);
	double candidate_hue_histogram[HUE_QUANTIZATION];
	//Init to zeros:
	for(int i=0;i<HUE_QUANTIZATION;i++)
	{
		 candidate_hue_histogram[i]=0;
	}

	for(int i=0;i<candidate_hue.rows;i++)
	{

		for(int j=0;j<candidate_hue.cols;j++)
		{
			if(candidate_hue.at<uchar>(i,j)<field_min_hue || candidate_hue.at<uchar>(i,j)>field_max_hue) //Don't count the field colors into the colors histogram.
				candidate_hue_histogram[candidate_hue.at<uchar>(i,j)]++;
		}
	}



	//Normalize the array. first divide by rowsXcols (to prevent possible numeric errors with large numbers) and then normalize for norm_2=1:
	double normalization_factor=candidate_hue.rows*candidate_hue.cols;

	for(int i=0;i<HUE_QUANTIZATION;i++)
	{
		candidate_hue_histogram[i]=candidate_hue_histogram[i]/normalization_factor;
	}

	double sqrt_sum_squares=0; //Will hold the normalization factor for norm_2 = 1 for the ball_hue_histogram vector.

	for(int i=0;i<HUE_QUANTIZATION;i++)
	{
		sqrt_sum_squares=sqrt_sum_squares+pow(candidate_hue_histogram[i],2);
	}
	sqrt_sum_squares=sqrt(sqrt_sum_squares);

	correlation=0;

	for(int i=0;i<HUE_QUANTIZATION;i++) //Normalize the ball_hue_histogram vector for norm_2 to be equal 1 (we need it because we later on compute colors correlation).
	{
		candidate_hue_histogram[i]=candidate_hue_histogram[i]/sqrt_sum_squares;
		correlation+=candidate_hue_histogram[i]*ball_hue_histogram[i]; //Dot product is correlation.
	}

}

/*
 *This methods counts how many field pixels are in candidate matrix - by using the field_min_hue and field_max_hue received by field calibration. result is in  num_field_pixels.
 */

void BallDetector::CountNumFieldPixels(Mat& candidate,uchar& field_min_hue,uchar& field_max_hue,uint& num_field_pixels)
{
	num_field_pixels=0; //Init.
	for(int i=0;i<candidate.rows;i++)
	{
		for(int j=0;j<candidate.cols;j++)
		{
			if(candidate.at<uchar>(i,j)>=field_min_hue && candidate.at<uchar>(i,j)<=field_max_hue)
			{
				num_field_pixels++;
			}
		}
	}
}



void BallDetector::CalculateDistanceToBall(int& radius,double& calculated_distance)
{
	calculated_distance=(597.3040-18.6316*radius+0.236*pow(radius,2)-0.001*pow(radius,3));
}



//
////This method gets the b&w (after thresholding) field matrix after calibration and calculates the convex hull of the field.
//void BallDetector::CalculateFieldConvexHUll(Mat& field_mat,vector<Point>& hull)
//{
//	//Find largest connected component in field_mat:
//
//	Mat labels,centroids,stats;
//	connectedComponentsWithStats(field_mat,labels,stats,centroids,8,CV_16U);
//	char16_t max_size_component=0,index_of_max=0;
//	for(int i=0;i<centroids.rows;i++)
//	{
//		if(stats.at<int>(i,CC_STAT_AREA)>max_size_component)
//		{
//			max_size_component=stats.at<int>(i,CC_STAT_AREA);
//			index_of_max=i;
//		}
//	}
//	//Zero all the pixels that have different label than the index_of_max - leaving us with the connected component only!:
//	for(int i=0;i<labels.rows;i++)
//	{
//		for(int j=0;j<labels.cols;j++)
//		{
//
//			if(labels.at<char16_t>(i,j)!=index_of_max)
//			{
//				labels.at<char16_t>(i,j)=0;
//			}
//			else
//			{
//				cout<<"labels:"<<labels.at<char16_t>(i,j)<<endl;
//				labels.at<char16_t>(i,j)=255;
//			}
//		}
//	}
//	imshow("labels",labels);
////	//Now we have the index of the largest connected component, let's compute its convex hull:
////	convexHull(labels,hull);
////
////
////    Mat img = Mat::zeros(field_mat.size(),CV_8UC1);
////    int hullcount = (int)hull.size();
////    Point pt0 = hull[0];
////    for(int i = 0; i < hullcount; i++ )
////    {
////        Point pt =hull[i];
////        line(img, pt0, pt, Scalar(0, 255, 0), 1,LINE_AA);
////        pt0 = pt;
////    }
////    imshow("hull", img);
//
//
//
//}


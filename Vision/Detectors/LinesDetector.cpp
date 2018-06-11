

#include "LinesDetector.h"
#include <math.h>
#define PI 3.14159265358979 //Value of Pi (11 digits after the point).

vector<Vec4i> LinesDetector::GetLines(Mat image, int threshold, int minLinLength, int maxLineGap)
{

	Mat frame,frame_hsv, field_mat, whites_mat, field_space_mat; //frame=origianl image. frame_hsv=image after transform to hsv. field_mat=only pixels in bounds of field's green after calibration are 0 (all the rest 255). whites_mat=only white pixels in image. field_space_mat=all pixels below bounding_horizontal_line (i.e -assumed to be field). candidates_mat=pixels which are not field and white and below bounding horizontal line.


	Mat frame_gray, original_field_mat, candidates_mat;


	Mat dst, cdst, cdstP, src1;
	Mat hsv_channels[NUM_CHANNELS]; //Will contain all 3 HSV channels splitted.
	Mat bgr_channels[NUM_CHANNELS]; //Will contain all 3 BGR channels splitted.
	uchar field_min_hue, field_max_hue; //Will mark which pixel is a field pixel after calib.
	ushort bounding_horizontal_line;

	//VisionThread::SafeReadeCapturedFrame(frame);
	frame = image; // Put into frame the frame that we took from the camera.

	split(frame, bgr_channels); //Split to B,G,R channels so we can manipulate the image later on.

	// Do GaussianBlur on the frame - Reduce noise:
	GaussianBlur(bgr_channels[0], bgr_channels[0], Size(3, 3), 2, 2);
	GaussianBlur(bgr_channels[1], bgr_channels[1], Size(3, 3), 2, 2);
	GaussianBlur(bgr_channels[2], bgr_channels[2], Size(3, 3), 2, 2);
	merge(bgr_channels, NUM_CHANNELS, frame); //Merge the channels back to one matrix after manipulation.

	//int idx = 0;

	cvtColor(frame, frame_hsv, CV_BGR2HSV); //Convert original RGB representation to HSV representation.

	split(frame_hsv, hsv_channels); //Split to H,S,V channels so we can use the hue,saturation&value matrices more conviniently later on.

	LinesDetector::FieldCalibration(hsv_channels[0], field_min_hue, field_max_hue);

	src1=frame.clone();

	//![edge_detection]
	// Edge detection
	Canny(src1, dst, 50, 200, 3);
	//![edge_detection]

	// Copy edges to the images that will display the results in BGR
	cvtColor(dst, cdst, COLOR_GRAY2BGR);
	cdstP = cdst.clone();

	//![hough_lines]
	// Standard Hough Line Transform
	vector<Vec2f> lines; // will hold the results of the detection

	//![hough_lines_p]
	// Probabilistic Line Transform
	vector<Vec4i> linesP; // will hold the results of the detection
	HoughLinesP(dst, linesP, 1, CV_PI/180, threshold, minLinLength, maxLineGap ); // runs the actual detection
	//![hough_lines_p]
	//![draw_lines_p]

	return linesP;

	waitKey(30);
}

vector<vector<int>> LinesDetector::GetJunctions(vector<Vec4i> linesP2, Mat frame, int dots_dis)
{
	vector<vector<int>> output;
	vector<Vec4i> linesP = linesP2;
	Mat cdstP = frame.clone();
	// Draw the lines
	double* thetas = new double[linesP.size()];
	for( size_t i = 0; i < linesP.size(); i++ )
	{
		Vec4i l = linesP[i];
//		if (l[3] == l[1])
//		{
//			thetas[i] = PI / 2;
//		}
//		else
//		{
//			thetas[i] = atan((l[0] - l[2])/(l[3] - l[1]));
//		}
		if (l[2] != l[0])
		{
		thetas[i] = (l[3] - l[1]) / (l[2] - l[0]); // m_i - NEW
		}
		else
		{
			thetas[i] = 1000000000;
		}
		line(cdstP, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, LINE_AA);
	}

	 vector<Point> x_y_coords;
	 vector<Point> type_nums;

	for(size_t i = 0; i < linesP.size(); i++ )
	{
		for(size_t j = i+1; j < linesP.size(); j++ )
		{
			bool is_junction_T = false, is_junction_X = false;
//			if (thetas[i] < 0)
//			{
//				thetas[i] += PI;
//			}
//			if (thetas[j] < 0)
//			{
//				thetas[j] += PI;
//			}
			double angle;
			if ((thetas[j] * thetas[i]) != -1)
			{
			angle = abs(atan( (thetas[i] - thetas[j]) / (1 + thetas[j] * thetas[i])) );
			}
			else
			{
				angle = PI / 2;
			}

			//cout << (180 / PI) * abs(angle - PI / 2) << endl;

			if (/*abs(abs(thetas[i] - thetas[j]) - PI / 2)*/abs(angle - PI / 2) <= 8 * PI / 180)
			{
				Vec4i l_i = linesP[i];
				Vec4i l_j = linesP[j];
				//line( frame, Point(l_i[0], l_i[1]), Point(l_i[2], l_i[3]), Scalar(255,0,0), 3, LINE_AA);
				//line( frame, Point(l_j[0], l_j[1]), Point(l_j[2], l_j[3]), Scalar(0,255,0), 3, LINE_AA);

				double x1 = l_i[0];
				double y1 = l_i[1];
				double x2 = l_i[2];
				double y2 = l_i[3];
				double x3 = l_j[0];
				double y3 = l_j[1];
				double x4 = l_j[2];
				double y4 = l_j[3];
				double m1 = (double) (l_i[3] - l_i[1]) / (l_i[2] - l_i[0]);
				double m2 = (double) (l_j[3] - l_j[1]) / (l_j[2] - l_j[0]);

				if (m1 != m2)
				{
					double intersection_x = (-x3 * m2 + y3 + x1 * m1 - y1) / (m1 - m2);
					double intersection_y = m1 * intersection_x - x1 * m1 + y1;
					double threshold_dis = 15;
					double dis_from_p1 = sqrt(pow(intersection_x - x1, 2) + pow(intersection_y - y1, 2));
					double dis_from_p2 = sqrt(pow(intersection_x - x2, 2) + pow(intersection_y - y2, 2));
					double dis_from_p3 = sqrt(pow(intersection_x - x3, 2) + pow(intersection_y - y3, 2));
					double dis_from_p4 = sqrt(pow(intersection_x - x4, 2) + pow(intersection_y - y4, 2));
					double line1 = sqrt(pow(x2 - x1, 2) + pow(y2- y1, 2));
					double line2 = sqrt(pow(x4 - x3, 2) + pow(y4- y3, 2));

					bool is_valid_line = (line1 * 1.4 > dis_from_p1 + dis_from_p2) && (line2 * 1.4 > dis_from_p3 + dis_from_p4);
					if (is_valid_line)
					{
						 Point x_y_coord, type_num;
						 x_y_coord.x = intersection_x;
						 x_y_coord.y = intersection_y;

						 type_num.x = 0;
						 double dist_x, dist_y;

						 if (abs(m1) < abs(m2)/*abs(x2-x1) > abs(x4-x3)*/) // LINE 1 IS AXIS X.
						 {
							 double center_x = (x1+x2)/2;
							 double center_y = (y1+y2)/2;
							 // x3 y3     x4 y4
							 if (abs(x3 - center_x) > abs(x4 - center_x))
							 {
								 dist_x = x3 - center_x;
							 }
							 else
							 {
								 dist_x = x4 - center_x;
							 }

							 if (abs(y3 - center_y) > abs(y4 - center_y))
							 {
								 dist_y = y3 - center_y;
							 }
							 else
							 {
								 dist_y = y4 - center_y;
							 }
						 }
						 else // LIne 2 IS AXIS X.
						 {
							 double center_x = (x3+x4)/2;
							 double center_y = (y3+y4)/2;
							 // x1 y1     x2 y2
							 dist_x = max(abs(x1 - center_x), abs(x2 - center_x));
							 dist_y = max(abs(y1 - center_y), abs(y2 - center_y));


							 if (abs(x1 - center_x) > abs(x2 - center_x))
							 {
								 dist_x = x1 - center_x;
							 }
							 else
							 {
								 dist_x = x2 - center_x;
							 }

							 if (abs(y1 - center_y) > abs(y2 - center_y))
							 {
								 dist_y = y1 - center_y;
							 }
							 else
							 {
								 dist_y = y2 - center_y;
							 }


						 }
						 if (dist_x >= 0 && dist_y < 0)
						 {
							 //cout << "L3" << endl;
							 type_num.x = 3;
						 }
						 else if (dist_x < 0 && dist_y < 0)
						 {
							 //cout << "L2" << endl;
							 type_num.x = 2;
						 }
						 else if (dist_x >= 0 && dist_y >= 0)
						 {
							// cout << "L4" << endl;
							 type_num.x = 4;
						 }
						 else if (dist_x < 0 && dist_y >= 0)
						 {
							 //cout << "L1" << endl;
							 type_num.x = 1;
						 }

						// Type T - (2)
						if ((dis_from_p1 >= threshold_dis) + (dis_from_p2 >= threshold_dis) + (dis_from_p3 >= threshold_dis) + (dis_from_p4 >= threshold_dis) == 3)
						{
							 type_num.x = 5;
						    is_junction_T = true;
						}
						// Type X - (3)
						else if ((dis_from_p1 >= threshold_dis) + (dis_from_p2 >= threshold_dis) + (dis_from_p3 >= threshold_dis) + (dis_from_p4 >= threshold_dis) == 4)
						{
							 type_num.x = 6;
							 is_junction_X = true;
						}

						/*if (!is_junction_T && !is_junction_X) // Type L - (1)
						{
							 type_num.x = 1;
						}*/
						// L - Blue.
						// T - Green.
						// X - Red.

						 x_y_coords.push_back(x_y_coord);

						 type_num.y = 1;
						 type_nums.push_back(type_num);
					}
				}
			}
		}
	}
	delete[] thetas;

	for (int i = 0; i < type_nums.size(); i++)
	{
		for (int j = i+1; j < type_nums.size(); j++)
		{
			double distance = sqrt(pow(x_y_coords[i].x - x_y_coords[j].x, 2) + pow(x_y_coords[i].y - x_y_coords[j].y, 2));

			if (distance <= dots_dis && type_nums[i].y + type_nums[j].y != 0)
			{

				x_y_coords[i].x = round((type_nums[i].y * x_y_coords[i].x + type_nums[j].y * x_y_coords[j].x) / (type_nums[i].y + type_nums[j].y));
				x_y_coords[i].y = round((type_nums[i].y * x_y_coords[i].y + type_nums[j].y * x_y_coords[j].y) / (type_nums[i].y + type_nums[j].y));

				type_nums[i].x = max(type_nums[i].x, type_nums[j].x);
				type_nums[i].y = type_nums[i].y + type_nums[j].y;

				x_y_coords[j].x = - dots_dis;
				x_y_coords[j].y = - dots_dis;
				type_nums[j].x = 0;
				type_nums[j].y = 0;
			}
			//cout << "X: " << x_y_coords[i].x << " Y:" << x_y_coords[i].y << " Type: " << type_nums[i].x << " Num: " << type_nums[i].y << endl;
		}
	}

	for (int i = 0; i < type_nums.size(); i++)
	{
		if (type_nums[i].y != 0)
		{
			//cout << "X: " << x_y_coords[i].x << " Y:" << x_y_coords[i].y << " Type: " << type_nums[i].x << " Num: " << type_nums[i].y << endl;
			char str[200];

			if (type_nums[i].x == -1)
			{
				sprintf(str, "L");
				putText(frame, str, Point2f(x_y_coords[i].x, x_y_coords[i].y), FONT_HERSHEY_PLAIN, 2, Scalar(255,0,0,255), 4, CV_AA, false);
				vector<int> x_y_type;
				x_y_type.push_back(x_y_coords[i].x);
				x_y_type.push_back(x_y_coords[i].y);
				x_y_type.push_back(type_nums[i].x);
				output.push_back(x_y_type);

				//circle(frame, cvPoint(x_y_coords[i].x, x_y_coords[i].y), 10, Scalar(255,0,0), 3);
			}
			else if (type_nums[i].x == 5)
			{
				sprintf(str, "T");
				putText(frame, str, Point2f(x_y_coords[i].x, x_y_coords[i].y), FONT_HERSHEY_PLAIN, 2, Scalar(0,255,0,255), 4, CV_AA, false);
				vector<int> x_y_type;
				x_y_type.push_back(x_y_coords[i].x);
				x_y_type.push_back(x_y_coords[i].y);
				x_y_type.push_back(type_nums[i].x);
				output.push_back(x_y_type);

				//circle(frame, cvPoint(x_y_coords[i].x, x_y_coords[i].y), 10, Scalar(0,255,0), 3);
			}
			else if (type_nums[i].x == 6)
			{
				sprintf(str, "X");
				putText(frame, str, Point2f(x_y_coords[i].x, x_y_coords[i].y), FONT_HERSHEY_PLAIN, 2, Scalar(0,0,255,255), 4, CV_AA, false);
				vector<int> x_y_type;
				x_y_type.push_back(x_y_coords[i].x);
				x_y_type.push_back(x_y_coords[i].y);
				x_y_type.push_back(type_nums[i].x);
				output.push_back(x_y_type);

				//circle(frame, cvPoint(x_y_coords[i].x, x_y_coords[i].y), 10, Scalar(0,0,255), 3);
			}
			else if (type_nums[i].x == 3)
			{
				sprintf(str, "L3");
				putText(frame, str, Point2f(x_y_coords[i].x, x_y_coords[i].y), FONT_HERSHEY_PLAIN, 2, Scalar(0,0,255,255), 4, CV_AA, false);
				vector<int> x_y_type;
				x_y_type.push_back(x_y_coords[i].x);
				x_y_type.push_back(x_y_coords[i].y);
				x_y_type.push_back(type_nums[i].x);
				output.push_back(x_y_type);

				//circle(frame, cvPoint(x_y_coords[i].x, x_y_coords[i].y), 10, Scalar(0,0,255), 3);
			}
			else if (type_nums[i].x == 2)
			{
				sprintf(str, "L2");
				putText(frame, str, Point2f(x_y_coords[i].x, x_y_coords[i].y), FONT_HERSHEY_PLAIN, 2, Scalar(0,0,255,255), 4, CV_AA, false);
				vector<int> x_y_type;
				x_y_type.push_back(x_y_coords[i].x);
				x_y_type.push_back(x_y_coords[i].y);
				x_y_type.push_back(type_nums[i].x);
				output.push_back(x_y_type);

				//circle(frame, cvPoint(x_y_coords[i].x, x_y_coords[i].y), 10, Scalar(0,0,255), 3);
			}
			else if (type_nums[i].x == 4)
			{
				sprintf(str, "L4");
				putText(frame, str, Point2f(x_y_coords[i].x, x_y_coords[i].y), FONT_HERSHEY_PLAIN, 2, Scalar(0,0,255,255), 4, CV_AA, false);
				vector<int> x_y_type;
				x_y_type.push_back(x_y_coords[i].x);
				x_y_type.push_back(x_y_coords[i].y);
				x_y_type.push_back(type_nums[i].x);
				output.push_back(x_y_type);

				//circle(frame, cvPoint(x_y_coords[i].x, x_y_coords[i].y), 10, Scalar(0,0,255), 3);
			}
			else if (type_nums[i].x == 1)
			{
				sprintf(str, "L1");
				putText(frame, str, Point2f(x_y_coords[i].x, x_y_coords[i].y), FONT_HERSHEY_PLAIN, 2, Scalar(0,0,255,255), 4, CV_AA, false);
				vector<int> x_y_type;
				x_y_type.push_back(x_y_coords[i].x);
				x_y_type.push_back(x_y_coords[i].y);
				x_y_type.push_back(type_nums[i].x);
				output.push_back(x_y_type);

				//circle(frame, cvPoint(x_y_coords[i].x, x_y_coords[i].y), 10, Scalar(0,0,255), 3);
			}
		}
	}

	//imshow("Source", src1);
	// imshow("Detected Lines (in red) - Standard Hough Line Transform", cdst);
	imshow("Detected Lines (in red) - Probabilistic Line Transform", cdstP);
	imshow("Lines_detector_frame",frame);
	return output;
}

/*This method calibrates the green field hue bounds. It takes the hue matrix and returns the- field_min_hue and field_max_hue.
	We create the histogram of hue for the given matrix and take PERCENTAGE_THRESHOLD (=90%) in the middle of the green spectrum -
	assuming values on the edges of the green spectrum might be noise.*/
void LinesDetector::FieldCalibration(Mat& hue_matrix, uchar& field_min_hue, uchar& field_max_hue)
{
	int hue_histogram[HUE_DISCRETIZATION];
	int total_num_of_green_pixels=0; //Tells how many green pixel in total range of greens as specified by MIN_GREEN_HUE & MAX_GREEN_HUE.
	int sum_of_green_pixels_in_histogram; //Will count green pixels until we reach the threshold defined by PERCENTAGE_THRESHOLD.
	const double PERCENTAGE_THRESHOLD = 0.9; //Determines how many green pixels we get rid of. the larger the less.
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

/*This method calculates the bounding horizontal line of the field. i.e - it is the horizontal line which PERCENTAGE_THRESHOLD (=90%) of the pixels
marked as field by field_mat are *below*.*/
void LinesDetector::CalculateBoundingHorizontalLine(Mat& field_mat, ushort& bounding_horizontal_line)
{
	const double PERCENTAGE_THRESHOLD = 0.99; //Heuristic.
	uint num_of_field_pixels_below_bounding_line=0;
	uint total_num_of_field_pixels=0;
	uint bounding_horizontal_line_ = field_mat.rows; //Initialize the bounding horizontal line to the lowest row of image.
	uint* sum_of_rows_vector=new uint[field_mat.rows]; //Dyanmic array - Will hold the sum of each row in field_mat.

	//Initialize sum_of_rows_vector to zeros:
	for (int i = 0; i < field_mat.rows; i++)
	{
		sum_of_rows_vector[i] = 0;
	}


	//sum all of green points row by row:
	for (uint i = field_mat.rows-1; i > 0; i--)
	{
		for (uint j = 0; j < field_mat.cols; j++)
		{
			sum_of_rows_vector[i] = sum_of_rows_vector[i] + (field_mat.at<uchar>(i, j)^255); //^255(bitwise xor) for conversion from 255=(11111111)_2 to 0 and 0 to 255=(11111111)_2.
		}
		total_num_of_field_pixels = total_num_of_field_pixels + sum_of_rows_vector[i]; //Sum of sum of rows = total num. of field pixels.
	}

	//push the bounding_horizontal_line_ up until  PERCENTAGE_THRESHOLD of the 'field pixels' are below.
	ushort row_num = field_mat.rows - 1;
	num_of_field_pixels_below_bounding_line = sum_of_rows_vector[row_num]; //Initialize to sum of last row of image.
	while (num_of_field_pixels_below_bounding_line / (total_num_of_field_pixels + 0.0) < PERCENTAGE_THRESHOLD)
	{
		row_num--;
		num_of_field_pixels_below_bounding_line = num_of_field_pixels_below_bounding_line + sum_of_rows_vector[row_num]; //add the row above.
	}
	bounding_horizontal_line = row_num; //row_num contains the bounding horizontal line after loop.

	//cout << "bounding horizontal_line:" << bounding_horizontal_line << "\n" <<endl; //TEST

	delete[] sum_of_rows_vector; //Deallocate dyanmic array.
}

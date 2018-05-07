

#include "LinesDetector.h"
#include <math.h>
#define PI 3.14159265358979 //Value of Pi (11 digits after the point).

void LinesDetector::GetLinesPosts(Mat image, int threshold, int minLinLength, int maxLineGap){
	    Mat frame,frame_hsv, frame_gray,original_field_mat, field_mat, whites_mat, field_space_mat, candidates_mat; //frame=origianl image. frame_hsv=image after transform to hsv. field_mat=only pixels in bounds of field's green after calibration are 0 (all the rest 255). whites_mat=only white pixels in image. field_space_mat=all pixels below bounding_horizontal_line (i.e -assumed to be field). candidates_mat=pixels which are not field and white and below bounding horizontal line.
		Mat dst,line1,cdst,cdstP,src1;
	    Mat hsv_channels[NUM_CHANNELS]; //Will contain all 3 HSV channels splitted.
		Mat bgr_channels[NUM_CHANNELS]; //Will contain all 3 BGR channels splitted.
		uchar field_min_hue, field_max_hue; //Will mark which pixel is a field pixel after calib.
		ushort bounding_horizontal_line;

		//VisionThread::SafeReadeCapturedFrame(frame);
frame = image;
		clock_t begin = clock();
					split(frame, bgr_channels); //Split to B,G,R channels so we can manipulate the image later on.
		//			//Reduce noise:
					GaussianBlur(bgr_channels[0], bgr_channels[0], Size(3, 3), 2, 2);
					GaussianBlur(bgr_channels[1], bgr_channels[1], Size(3, 3), 2, 2);
					GaussianBlur(bgr_channels[2], bgr_channels[2], Size(3, 3), 2, 2);
					merge(bgr_channels, NUM_CHANNELS, frame); //Merge the channels back to one matrix after manipulation.


		int idx=0;
				cvtColor(frame, frame_hsv, CV_BGR2HSV); //Convert original RGB representation to HSV representation.

				split(frame_hsv, hsv_channels); //Split to H,S,V channels so we can use the hue,saturation&value matrices more conviniently later on.
				LinesDetector::FieldCalibration(hsv_channels[0], field_min_hue, field_max_hue);
				field_mat = Mat::zeros(frame.rows, frame.cols, CV_8UC1); //Generate a 1-channel matrix with size of original image (unsigned char). set all pixels to initail value 255=(11111111)_2
				whites_mat = Mat::zeros(frame.rows, frame.cols, CV_8UC1); //Generate a 1-channel matrix with size of original image (unsigned char).

																		  //Generate the field_mat and whites_mat:
				//Generate the field_mat and whites_mat:
				for (int i = 0; i < frame.rows; i++)
				{
					for (int j = 0; j < frame.cols; j++)
					{

						//Check saturation& value against bounds:
						if (hsv_channels[1].at<uchar>(i, j) <= MAX_WHITE_SATURATION && hsv_channels[2].at<uchar>(i, j) >= MIN_WHITE_VALUE)
						{
							whites_mat.at<uchar>(i, j) = 255; //If in range (i.e- white pixel) set to 11111111 in binary.
						}

						//Check hue value against bounds:
						if (hsv_channels[0].at<uchar>(i, j) >= field_min_hue && hsv_channels[0].at<uchar>(i, j) <= field_max_hue && whites_mat.at<uchar>(i, j)==0)
						{
							field_mat.at<uchar>(i, j) = 255; //If in range (i.e- field pixel) set to 0.
						}
					}
				}
//				medianBlur(field_mat,field_mat,5);
//				imshow("field_mat", field_mat);
//				imshow("whites_mat",whites_mat);
//				original_field_mat = field_mat.clone(); //Will be using the original mat later on with CHT.
//				//imshow("original_field_mat", original_field_mat);
//				//waitKey();
//				//Perform morphological closing to field_mat:

//				erode(field_mat, field_mat, structure_element_); //First perform erosion to remove lone points and noise

//				structure_element_ = getStructuringElement(MORPH_ELLIPSE, Size(MIN_BALL_DIAMETER - 1, MIN_BALL_DIAMETER - 1));
//				//Perform closing to the field_mat with circular structure element  -so we get trid of lines!-

//				erode(field_mat, field_mat, structure_element_);


				int count_num_field_pixels_in_row;
				for (int i = 0; i < field_mat.rows; i++)
				{
					count_num_field_pixels_in_row=0;
					for (int j = 0; j < field_mat.cols; j++)
					{
						if(field_mat.at<uchar>(i,j)==255)
						{
							count_num_field_pixels_in_row++;
						}
					}
					if(count_num_field_pixels_in_row<field_mat.cols/2) //If most of the row is not field pixels - delete all field pixels in row:
					{
						for(int k=0;k<field_mat.cols;k++)
						{
							field_mat.at<uchar>(i,k)=0;
						}
					}
				}

				Mat structure_element_ = getStructuringElement(MORPH_RECT, Size(2*STRUCTURE_ELEMENT_SIZE, 2*STRUCTURE_ELEMENT_SIZE));
				dilate(field_mat, field_mat, structure_element_);
				imshow("field_mat_Lines", field_mat);
				waitKey(1);

				bitwise_not(field_mat, field_mat);
				imshow("field_mat_Lines2", field_mat);

				Mat field_mat_clone=field_mat.clone(); //Copy because we draw rectangles on cloned image.

				//Calculate bounding horizontal line for field in image:
				LinesDetector::CalculateBoundingHorizontalLine(field_mat, bounding_horizontal_line);
//				if (bounding_horizontal_line < BOUNDING_HORIZONTAL_LINE_FIX)
//				{
//					bounding_horizontal_line = 0;
//				}
//				else
//				{
//					bounding_horizontal_line = bounding_horizontal_line - BOUNDING_HORIZONTAL_LINE_FIX;
//				}

				//Because the bounding horizontal line may cut the bottom of the Lines we add some 'insurance' extra pixels - 15% of frame- 72 pixels
				//Generate field_space_mat=all pixels below bounding_horizontal_line (i.e -assumed to be field) are 255=(11111111)_2 and 0 else.
				field_space_mat = Mat::zeros(frame.rows, frame.cols, CV_8UC1);
				for (uint i = frame.rows - 1; i > 0; i--)
				{
					if ((int)i >= bounding_horizontal_line-BOUNDING_HORIZONTAL_LINE_FIX ) //If below bounding_horizontal_line minus the bounding horizontal fix (which lets a bit more pixels from the frame to enter the field_space_mat)
					{
						for (uint j = 0; j < frame.cols; j++) //Set all values to 255 there:
						{
							field_space_mat.at<uchar>(i, j) = 255;
						}
					}
					else
					{
						break;
					}
				}
//				bitwise_not(field_space_mat,field_space_mat);
				imshow("field_space_mat", field_space_mat);
				waitKey(1);
				imshow("whites_mat", whites_mat);
				//bitwise_and(whites_mat, field_space_mat, whites_mat);

				Mat element = getStructuringElement( MORPH_RECT,
				                                       Size( 3, 3 ));

			    //erode(whites_mat,whites_mat,element); //Perform erosion to get rid of noise.
//				medianBlur(whites_mat1,whites_mat,3);
			    //GaussianBlur(whites_mat,whites_mat,Size(3,3),1.5,1.5);

				//GaussianBlur(whites_mat,whites_mat,Size(7,7),3,3);





				imshow("whites_matG", whites_mat);
				waitKey(1);
				src1=whites_mat.clone();
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
//				    HoughLines(dst, lines, 1, CV_PI/180, 150, 0, 0 ); // runs the actual detection
//				    //![hough_lines]
//				    //![draw_lines]
//				    // Draw the lines
//				    for( size_t i = 0; i < lines.size(); i++ )
//				    {
//				        float rho = lines[i][0], theta = lines[i][1];
//				        Point pt1, pt2;
//				        double a = cos(theta), b = sin(theta);
//				        double x0 = a*rho, y0 = b*rho;
//				        pt1.x = cvRound(x0 + 1000*(-b));
//				        pt1.y = cvRound(y0 + 1000*(a));
//				        pt2.x = cvRound(x0 - 1000*(-b));
//				        pt2.y = cvRound(y0 - 1000*(a));
//				        line( cdst, pt1, pt2, Scalar(0,0,255), 3, CV_AA);
//				    }
				    //![draw_lines]

				    //![hough_lines_p]
				    // Probabilistic Line Transform
				    vector<Vec4i> linesP; // will hold the results of the detection
				    HoughLinesP(dst, linesP, 1, CV_PI/180, threshold, minLinLength, maxLineGap ); // runs the actual detection
				    //![hough_lines_p]
				    //![draw_lines_p]
				    // Draw the lines
				    double* thetas = new double[linesP.size()];
				    for( size_t i = 0; i < linesP.size(); i++ )
				    {
				        Vec4i l = linesP[i];
				        if (l[3] == l[1])
				        {
				        	thetas[i] = PI / 2;
				        }
				        else
				        {
				        	thetas[i] = atan((l[0] - l[2])/(l[3] - l[1]));
				        }

				        line( cdstP, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, LINE_AA);
				        ////////////////////////////////////////////////////////////////////////

				        //////////////////////////////////////////////////////////////////////////////
				    }
				    for( size_t i = 0; i < linesP.size(); i++ )
				    {
					    for( size_t j = i+1; j < linesP.size(); j++ )
					    {
					    	bool is_junction_T = false, is_junction_X = false;
					    	if (thetas[i] < 0)
					    	{
					    		thetas[i] += PI;
					    	}
					    	if (thetas[j] < 0)
					    	{
					    		thetas[j] += PI;
					    	}

					    	if (abs(abs(thetas[i] - thetas[j]) - PI / 2) <= 40 * PI / 180)
					    	{
						        Vec4i l_i = linesP[i];
						        Vec4i l_j = linesP[j];
						        //line( frame, Point(l_i[0], l_i[1]), Point(l_i[2], l_i[3]), Scalar(255,0,0), 3, LINE_AA);
						        //line( frame, Point(l_j[0], l_j[1]), Point(l_j[2], l_j[3]), Scalar(0,255,0), 3, LINE_AA);

						        /// ----- ADDITION FOR T JUNCTION ------ ///
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

						        	bool is_valid_line = (line1 * 1.1 > dis_from_p1 + dis_from_p2) && (line2 * 1.1 > dis_from_p3 + dis_from_p4);
						        	if (is_valid_line)
						        	{
						        		circle(frame, cvPoint(intersection_x, intersection_y), 10, Scalar(0,255,255), 3);

						        	if ((dis_from_p1 >= threshold_dis) + (dis_from_p2 >= threshold_dis) + (dis_from_p3 >= threshold_dis) + (dis_from_p4 >= threshold_dis) == 3)
						        	{
						        		is_junction_T = true;
						        		circle(frame, cvPoint(intersection_x, intersection_y), 10, Scalar(0,0,255), 3);
						        	}
						        	else if ((dis_from_p1 >= threshold_dis) + (dis_from_p2 >= threshold_dis) + (dis_from_p3 >= threshold_dis) + (dis_from_p4 >= threshold_dis) == 4)
						        	{
						        		is_junction_X = true;
						        		circle(frame, cvPoint(intersection_x, intersection_y), 10, Scalar(0,255,0), 3);
						        	}

							        if (!is_junction_T && !is_junction_X)
							        {
						        		circle(frame, cvPoint(intersection_x, intersection_y), 10, Scalar(255,0,0), 3);
							        }
						        	}
						        }

						        /// ----- END OF ADDITION FOR T JUNCTION ------ ///

					    	}
					    }
				    }
				    delete[] thetas;

				    for( size_t i = 0; i < linesP.size(); i++ )
				    {
				        Vec4i l = linesP[i];
				        //line( frame, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, LINE_AA);
				    }

				clock_t end = clock();
				double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
				cout << "elapsed time:" << elapsed_secs <<"\n"<< endl;
			    imshow("Source", src1);
			   // imshow("Detected Lines (in red) - Standard Hough Line Transform", cdst);
			    imshow("Detected Lines (in red) - Probabilistic Line Transform", cdstP);
				imshow("Lines_detector_frame",frame);
				waitKey(30);
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

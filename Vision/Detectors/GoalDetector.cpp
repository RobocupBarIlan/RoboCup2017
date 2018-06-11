

#include "GoalDetector.h"


void GoalDetector::GetGoalPosts(GoalCandidate& gc)
{
	Mat frame,frame_hsv, frame_gray,original_field_mat, field_mat, whites_mat, field_space_mat, candidates_mat; //frame=origianl image. frame_hsv=image after transform to hsv. field_mat=only pixels in bounds of field's green after calibration are 0 (all the rest 255). whites_mat=only white pixels in image. field_space_mat=all pixels below bounding_horizontal_line (i.e -assumed to be field). candidates_mat=pixels which are not field and white and below bounding horizontal line.
	Mat hsv_channels[NUM_CHANNELS]; //Will contain all 3 HSV channels splitted.
	Mat bgr_channels[NUM_CHANNELS]; //Will contain all 3 BGR channels splitted.
	uchar field_min_hue, field_max_hue; //Will mark which pixel is a field pixel after calib.
	ushort bounding_horizontal_line;

	VisionThread::SafeReadeCapturedFrame(frame); // DELETE IN THE FUTURE.
	imshow("1", frame);

	split(frame, bgr_channels); //Split to B,G,R channels so we can manipulate the image later on.
	//Reduce noise:
	GaussianBlur(bgr_channels[0], bgr_channels[0], Size(3, 3), 2, 2);
	GaussianBlur(bgr_channels[1], bgr_channels[1], Size(3, 3), 2, 2);
	GaussianBlur(bgr_channels[2], bgr_channels[2], Size(3, 3), 2, 2);
	merge(bgr_channels, NUM_CHANNELS, frame); //Merge the channels back to one matrix after manipulation.
	imshow("2", frame);

	int idx=0;
	cvtColor(frame, frame_hsv, CV_BGR2HSV); //Convert original RGB representation to HSV representation.
	imshow("3", frame_hsv);

	split(frame_hsv, hsv_channels); //Split to H,S,V channels so we can use the hue,saturation&value matrices more conviniently later on.
	GoalDetector::FieldCalibration(hsv_channels[0], field_min_hue, field_max_hue);
	field_mat = Mat::zeros(frame.rows, frame.cols, CV_8UC1); //Generate a 1-channel matrix with size of original image (unsigned char). set all pixels to initail value 255=(11111111)_2
	whites_mat = Mat::zeros(frame.rows, frame.cols, CV_8UC1); //Generate a 1-channel matrix with size of original image (unsigned char).

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
	imshow("4", whites_mat);
	imshow("5", field_mat);

	// medianBlur(field_mat,field_mat,5);
	// imshow("field_mat", field_mat);
	//	imshow("whites_mat",whites_mat);
	// original_field_mat = field_mat.clone(); //Will be using the original mat later on with CHT.
	// imshow("original_field_mat", original_field_mat);
	// waitKey();
	// Perform morphological closing to field_mat:

	// erode(field_mat, field_mat, structure_element_); //First perform erosion to remove lone points and noise

	// structure_element_ = getStructuringElement(MORPH_ELLIPSE, Size(MIN_BALL_DIAMETER - 1, MIN_BALL_DIAMETER - 1));
	// Perform closing to the field_mat with circular structure element  -so we get trid of lines!-

	// erode(field_mat, field_mat, structure_element_);

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
	imshow("6", field_mat);

	Mat structure_element_ = getStructuringElement(MORPH_RECT, Size(2*STRUCTURE_ELEMENT_SIZE, 2*STRUCTURE_ELEMENT_SIZE));
	dilate(field_mat, field_mat, structure_element_);
	// imshow("field_mat", field_mat);
	// waitKey(1);
	imshow("7", field_mat);

	bitwise_not(field_mat, field_mat);
	Mat field_mat_clone=field_mat.clone(); //Copy because we draw rectangles on cloned image.
	imshow("8", field_mat);

	//Calculate bounding horizontal line for field in image:
	GoalDetector::CalculateBoundingHorizontalLine(field_mat, bounding_horizontal_line);
	cout << bounding_horizontal_line << endl;

	// if (bounding_horizontal_line < BOUNDING_HORIZONTAL_LINE_FIX)
	// {
	//	bounding_horizontal_line = 0;
	//}
	//else
	//{
	//	bounding_horizontal_line = bounding_horizontal_line - BOUNDING_HORIZONTAL_LINE_FIX;
	//}

	//Because the bounding horizontal line may cut the bottom of the goal we add some 'insurance' extra pixels - 15% of frame- 72 pixels
	//Generate field_space_mat=all pixels below bounding_horizontal_line (i.e -assumed to be field) are 255=(11111111)_2 and 0 else.
	field_space_mat = Mat::zeros(frame.rows, frame.cols, CV_8UC1);
	for (uint i = frame.rows - 1; i > 0; i--)
	{
		if (i >= bounding_horizontal_line+BOUNDING_HORIZONTAL_LINE_FIX ) //If below bounding_horizontal_line minus the bounding horizontal fix (which lets a bit more pixels from the frame to enter the field_space_mat)
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

	bitwise_not(field_space_mat,field_space_mat);
	imshow("9", field_space_mat);

	// imshow("field_space_mat", field_space_mat);
	// waitKey(1);

	bitwise_and(whites_mat, field_space_mat, whites_mat);
	imshow("10", whites_mat);

	Mat element = getStructuringElement( MORPH_RECT, Size( 3, 3 ));

	//erode(whites_mat,whites_mat,element); //Perform erosion to get rid of noise.
	medianBlur(whites_mat,whites_mat,3);
	imshow("11", whites_mat);

	//GaussianBlur(whites_mat,whites_mat,Size(3,3),1.5,1.5);
	//imshow("whites_mat", whites_mat);
	//waitKey(1);
	Mat whites_mat_clone=whites_mat.clone(); //Used to visualize the proccess (only for demonstrations etc.)

	// Parameters for Shi-Tomasi algorithm
	const int MAX_CORNERS_DISTANCE=80; //Max width of goal post
	vector<Point2f> corners;
	double qualityLevel = 0.0001; //Relative (to the best found corner in terms of high eigenvalues of the Harris matrix) quality of corner candidate.
	double minDistance = 5; //Min distance between detected corners (minimum num. of pixels between post's corners - i.e goal post width).
	int blockSize = 3;
	bool useHarrisDetector = false; //Use the cornerMinEigenVal() instead of corner harris.
	double k = 0.04;
	int maxCorners = 4000;

	/// Copy the source image
	Mat whites_mat_copy;
	whites_mat_copy = whites_mat.clone();

	// Apply corner detection
	goodFeaturesToTrack(whites_mat,
		corners,
		maxCorners,
		qualityLevel,
		minDistance,
		Mat(),
		blockSize,
		useHarrisDetector,
		k);

	// Draw corners detected
	//	cout << "** Number of corners detected: " << corners.size() << endl;
	const int R = 3;
	for (int i = 0; i < corners.size(); i++)
	{
		circle(whites_mat_copy, corners[i], R, Scalar(127, 127, 127), -1, 8, 0);
	}
	imshow("12", whites_mat_copy);
	//imshow("corners", whites_mat_copy);
	//waitKey(1);

	Mat candidate_i;
	int corner_1_x,corner_1_y,corner_2_x,corner_2_y;
	double slope;
	Rect goal_post_candidate_bounding_rect;// will get a bounding rectangle for candidate and check if it is filled with white pixels.
	Rect below_goal_post_candidate_bounding_rect; // will get a bounding rectangle for candidate and check if it is filled with field(green) pixels.
	Rect aside_goal_post_candidate_bounding_rect; // will get a bounding rectangle aside the candidate to check if no white pixels in it.
	int count_num_white_pixels_in_rect;
	int count_num_field_pixels_in_rect;
	Mat post_bounding_rect_mat; //Will hold a POST_WHITE_PIXELS_CHECK_NUM_PIXELS* (delta of corners x-coordinate) rectangle above the detected corners.
	Mat below_post_bounding_rect_mat; //Will hold a POST_FIELD_PIXELS_CHECK_NUM_PIXELS * (delta of corners x-coordinate) rectangle below the detected corners.
	Mat aside_post_bounding_rect_mat;//Will hold a POST_NO_WHITE_PIXELS_ASIDE_NUM_PIXELS * POST_WHITE_PIXELS_CHECK_NUM_PIXELS  rectangle aside the detected corners.
	double percentage_white_pixels; //Will hold the percentage of white pixels in bounding rectangle above candidate post.
	double percentage_field_pixels;//Will hold the percentage of field pixels in bounding rectangle below candidate post.

	bitwise_not(field_mat,field_mat);
	imshow("13", field_mat);

	vector<vector<Point>> post_candidates;
	//Now for every 2 candidates with relatively the same y-axis value (which indicates that both corners might be part of an upside-down T shape) - crop an image around:
	for(uint i=0;i<corners.size();i++)
	{
		corner_1_x=min(cvRound(corners[i].x),whites_mat.cols-1);
		corner_1_y=min(cvRound(corners[i].y),whites_mat.rows-1);
		for(uint j=/*0*/i+1;j<corners.size();j++)
		{
			corner_2_x=min(cvRound(corners[j].x),whites_mat.cols-1);
			corner_2_y=min(cvRound(corners[j].y),whites_mat.rows-1);
			if(corner_1_x-corner_2_x!=0 && abs(corner_1_y-bounding_horizontal_line)<=2*BOUNDING_HORIZONTAL_LINE_FIX && abs(corner_2_y-bounding_horizontal_line)<=2*BOUNDING_HORIZONTAL_LINE_FIX ) //Vertical line between edges- can't be edges of post -skip current candidate. also check for candidates only near the bounding horizontal line.
			{
				slope=(corner_1_y-corner_2_y+0.0)/(corner_1_x-corner_2_x);
				//cout<<slope<<endl;
				//For every 2 corners- if they are close enough then check if they are bounding a post of the goal between them
				if(/*(j<i) && */abs(slope)<1 && sqrt(pow(corner_1_x-corner_2_x,2)+pow(corner_1_y-corner_2_y,2))<MAX_CORNERS_DISTANCE)
				{
					//Check whether there are white pixels between 2 corners and above POST_WHITE_PIXELS_CHECK_NUM_PIXELS
					const int POST_WHITE_PIXELS_CHECK_NUM_PIXELS=30; //We check that there is a rectangle of height POST_HEIGHT_CHECK_NUM_PIXELS=60 and width abs(corner_1_x-corner_2_x) with mostly white pixels inside -to validate that there's a post:
					const double PERCENTAGE_WHITE_PIXELS_IN_RECT=0.7; //Heuristic - the bounding rectangle must have at least 80% white pixels in it!
					const int POST_FIELD_PIXELS_CHECK_NUM_PIXELS=30; //We check that there is a rectangle of height POST_HEIGHT_CHECK_NUM_PIXELS=60 and width abs(corner_1_x-corner_2_x) with mostly field(green) pixels inside -to validate that there's a post:
					const double PERCENTAGE_FIELD_PIXELS_IN_RECT=0.7 ;//Heuristic - the bounding rectangle must have at least 80% field(green) pixels in it!
					const int POST_NO_WHITE_PIXELS_ASIDE_NUM_PIXELS=20;//We check that a rectangle of width POST_NO_WHITE_PIXELS_ASIDE_NUM_PIXELS to the left and right of the post candidate as no more than 20% white pixels.
					const double PERCENTAGE_NO_WHITE_PIXELS_IN_RECT=0.3; //Heuristic- the bounding rectangle must have at most 20% white pixels in it!
					if(!(min(corner_1_y,corner_2_y)<POST_WHITE_PIXELS_CHECK_NUM_PIXELS)) //If not enough space above higher corner to check for white pixels above - abort checking.
					{
						goal_post_candidate_bounding_rect.x=min(corner_1_x,corner_2_x);
						goal_post_candidate_bounding_rect.y=min(corner_1_y,corner_2_y)-POST_WHITE_PIXELS_CHECK_NUM_PIXELS;
						goal_post_candidate_bounding_rect.width=abs(corner_1_x-corner_2_x);
						goal_post_candidate_bounding_rect.height=POST_WHITE_PIXELS_CHECK_NUM_PIXELS;
						count_num_white_pixels_in_rect=0;

						post_bounding_rect_mat=whites_mat(goal_post_candidate_bounding_rect);
						for(int k=0;k<post_bounding_rect_mat.rows;k++)
						{
							for(int m=0;m<post_bounding_rect_mat.cols;m++)
							{
								if(post_bounding_rect_mat.at<uchar>(k,m)>0)
								{
									count_num_white_pixels_in_rect++;
								}
							}
						}
						percentage_white_pixels=(count_num_white_pixels_in_rect+0.0)/(goal_post_candidate_bounding_rect.width*goal_post_candidate_bounding_rect.height);
						//cout<<"percentage"<<percentage_white_pixels<<endl;
						if(!(percentage_white_pixels<PERCENTAGE_WHITE_PIXELS_IN_RECT)) //There's enough white pixels in between the 2 corners checked. continue checking current candidate.
						{
							// MAYBE instead of field_mat.rows we will use horizontal_line?
							if((/*max*/min(corner_1_y,corner_2_y)+POST_FIELD_PIXELS_CHECK_NUM_PIXELS<field_mat.rows)) //if enough space below the post candidate to check for field pixels:
							{
								//Get bounding area of region below post:
								below_goal_post_candidate_bounding_rect.x=min(corner_1_x,corner_2_x);
								below_goal_post_candidate_bounding_rect.y=min(corner_1_y,corner_2_y);
								below_goal_post_candidate_bounding_rect.width=abs(corner_1_x-corner_2_x);
								below_goal_post_candidate_bounding_rect.height=POST_FIELD_PIXELS_CHECK_NUM_PIXELS;
								count_num_field_pixels_in_rect=0;

								below_post_bounding_rect_mat=field_mat(below_goal_post_candidate_bounding_rect);
								for(int k=0;k<below_post_bounding_rect_mat.rows;k++)
								{
									for(int m=0;m<below_post_bounding_rect_mat.cols;m++)
									{
										if(below_post_bounding_rect_mat.at<uchar>(k,m)>0)
										{
											count_num_field_pixels_in_rect++;
										}
									}
								}
								percentage_field_pixels=(count_num_field_pixels_in_rect+0.0)/(below_goal_post_candidate_bounding_rect.width*below_goal_post_candidate_bounding_rect.height);
								rectangle(field_mat_clone,below_goal_post_candidate_bounding_rect,Scalar(120),2);
								// cout<<"percentage_field_pixels"<<percentage_field_pixels<<endl;
								if(!(percentage_field_pixels<PERCENTAGE_FIELD_PIXELS_IN_RECT)) //If the candidate passed the constraint of number of field pixels below:
								{
									//imshow("Rect", below_post_bounding_rect_mat);
									//waitKey(1000);
									//Check whether to left of the left corner(and the right of the right corner) there are no white pixels (and there should be the field pixels (or other pixels out of field boundary)). This check is mostly needed to prevent half post detection.
									if(min(corner_1_x,corner_2_x)-POST_NO_WHITE_PIXELS_ASIDE_NUM_PIXELS>0 && max(corner_1_x,corner_2_x)+POST_NO_WHITE_PIXELS_ASIDE_NUM_PIXELS<whites_mat.cols) //If enough space to check to the left and right of the corners:
									{
										// LEFT OF POST DETECT.
										if(corner_1_x>corner_2_x && corner_2_y-POST_WHITE_PIXELS_CHECK_NUM_PIXELS/3>=0)
										{
											aside_goal_post_candidate_bounding_rect.x=corner_2_x-POST_NO_WHITE_PIXELS_ASIDE_NUM_PIXELS;
											aside_goal_post_candidate_bounding_rect.y=corner_2_y-POST_WHITE_PIXELS_CHECK_NUM_PIXELS/3;
										}
										else if(corner_2_x>corner_1_x &&corner_1_y-POST_WHITE_PIXELS_CHECK_NUM_PIXELS/3>=0)
										{
											aside_goal_post_candidate_bounding_rect.x=corner_1_x-POST_NO_WHITE_PIXELS_ASIDE_NUM_PIXELS;
											aside_goal_post_candidate_bounding_rect.y=corner_1_y-POST_WHITE_PIXELS_CHECK_NUM_PIXELS/3;
										}
										else //Doesn't have enough space to check if there are many white pixels aside the post - jump to next iteration.
										{
											continue;
										}

										aside_goal_post_candidate_bounding_rect.width=POST_NO_WHITE_PIXELS_ASIDE_NUM_PIXELS;
										aside_goal_post_candidate_bounding_rect.height=POST_WHITE_PIXELS_CHECK_NUM_PIXELS/3;
										count_num_white_pixels_in_rect=0;
										aside_post_bounding_rect_mat=whites_mat(aside_goal_post_candidate_bounding_rect);
										for(int k=0;k<aside_post_bounding_rect_mat.rows;k++)
										{
											for(int m=0;m<aside_post_bounding_rect_mat.cols;m++)
											{
												if(aside_post_bounding_rect_mat.at<uchar>(k,m)>0)
												{
													count_num_white_pixels_in_rect++;
												}
											}
										}

										// RIGHT OF POST DETECT
										//	if(percentage_white_pixels<0.3)
										// cout<<"percentage_white_pixels_left"<<percentage_white_pixels<<endl;
										percentage_white_pixels=(count_num_white_pixels_in_rect+0.0)/(aside_post_bounding_rect_mat.rows*aside_post_bounding_rect_mat.cols);
										if(percentage_white_pixels<=PERCENTAGE_NO_WHITE_PIXELS_IN_RECT)
										{
											rectangle(whites_mat_clone,aside_goal_post_candidate_bounding_rect,Scalar(60),2);
											imshow("whites_mat_clone", whites_mat_clone);
											//****************************************************************
											if(corner_1_x>corner_2_x && corner_1_y+POST_WHITE_PIXELS_CHECK_NUM_PIXELS/3<whites_mat.rows)
											{
												aside_goal_post_candidate_bounding_rect.x=corner_1_x;
												aside_goal_post_candidate_bounding_rect.y=corner_1_y-POST_WHITE_PIXELS_CHECK_NUM_PIXELS/3;
											}
											else if(corner_2_x>corner_1_x &&corner_2_y-POST_WHITE_PIXELS_CHECK_NUM_PIXELS/3>=0)
											{
												aside_goal_post_candidate_bounding_rect.x=corner_2_x;
												aside_goal_post_candidate_bounding_rect.y=corner_2_y-POST_WHITE_PIXELS_CHECK_NUM_PIXELS/3;
											}
											else //Doesn't have enough space to check if there are many white pixels aside the post - jump to next iteration.
											{
												continue;
											}

											aside_goal_post_candidate_bounding_rect.width=POST_NO_WHITE_PIXELS_ASIDE_NUM_PIXELS;
											aside_goal_post_candidate_bounding_rect.height=POST_WHITE_PIXELS_CHECK_NUM_PIXELS/3;
											count_num_white_pixels_in_rect=0;
											aside_post_bounding_rect_mat=whites_mat(aside_goal_post_candidate_bounding_rect);
											for(int k=0;k<aside_post_bounding_rect_mat.rows;k++)
											{
												for(int m=0;m<aside_post_bounding_rect_mat.cols;m++)
												{
													if(aside_post_bounding_rect_mat.at<uchar>(k,m)>0)
													{
														count_num_white_pixels_in_rect++;
													}
												}
											}

											percentage_white_pixels=(count_num_white_pixels_in_rect+0.0)/(aside_post_bounding_rect_mat.rows*aside_post_bounding_rect_mat.cols);
											//if(percentage_white_pixels<0.3)
											//cout<<"percentage_white_pixels_right"<<percentage_white_pixels<<endl;
											//****************************************************************
											if(percentage_white_pixels<=PERCENTAGE_NO_WHITE_PIXELS_IN_RECT)
											{
												rectangle(whites_mat_clone,aside_goal_post_candidate_bounding_rect,Scalar(120),2);
												imshow("whites_mat_clone", whites_mat_clone);

												post_candidates.push_back(vector<Point>());
												post_candidates[post_candidates.size()-1].push_back(Point(corner_1_x,corner_1_y));
												post_candidates[post_candidates.size()-1].push_back(Point(corner_2_x,corner_2_y));

												//	imshow("post_candidate",whites_mat(Rect(left,top,width,height)));
												//	waitKey();
											}
										}
									}
								}
							}
							//TODO check if there's a T structure
						}
					}
				}
				//if(width>0 && height>0 && idx<3000)
				//{
				//imwrite("post_candidate"+std::to_string(idx)+".png",whites_mat(Rect(left,top,width,height)));
				//idx++;
				//}
			}
		}
	}

	int max_width=0;
	int index_of_max_width_candidate=0;
	int index_of_second_max_width_candidate=-1;
	const int MIN_DIST_BETWEEN_POSTS=200;

	cout << post_candidates.size() << endl;

	if(post_candidates.size()>0) //If any candidate found:
	{
		for(int i=0;i<post_candidates.size();i++) //For every post conadidate found - choose the one with largest width first.
		{
			if(abs(post_candidates[i][0].x-post_candidates[i][1].x)>max_width)
			{
				max_width=abs(post_candidates[i][0].x-post_candidates[i][1].x);
				index_of_max_width_candidate=i;
			}
		}
		if(post_candidates.size()>1)//If more than 1 candidate - search for another candidate:
		{
			max_width=0;
			for(int i=0;i<post_candidates.size();i++) //For every post conadidate found - choose the one with largest width first.
			{
				if(i!=index_of_max_width_candidate && abs(post_candidates[i][0].x-post_candidates[i][1].x)>max_width && (abs((post_candidates[i][0].x+post_candidates[i][1].x)/2.0 -(post_candidates[index_of_max_width_candidate][0].x+post_candidates[index_of_max_width_candidate][1].x)/2.0))>=MIN_DIST_BETWEEN_POSTS  )
				{
					max_width=abs(post_candidates[i][0].x-post_candidates[i][1].x);
					index_of_second_max_width_candidate=i;
				}
			}
		}

		if(index_of_second_max_width_candidate!=-1) //2 posts found:
		{
			if((post_candidates[index_of_max_width_candidate][0].x+post_candidates[index_of_max_width_candidate][1].x)/2.0 > (post_candidates[index_of_second_max_width_candidate][0].x+post_candidates[index_of_second_max_width_candidate][1].x)/2.0) //First candidate is the right post and second candidate is the left post.
			{
				gc=GoalCandidate(post_candidates[index_of_second_max_width_candidate],post_candidates[index_of_max_width_candidate]);
			}
			else
			{
				gc=GoalCandidate(post_candidates[index_of_max_width_candidate],post_candidates[index_of_second_max_width_candidate]);
			}
		}
		else //Only one post detected:
		{
			post_candidates.push_back(vector<Point>());
			post_candidates[post_candidates.size()-1].push_back(Point(-1,-1));
			post_candidates[post_candidates.size()-1].push_back(Point(-1,-1));
			gc=GoalCandidate(post_candidates[index_of_max_width_candidate],post_candidates[post_candidates.size()-1]);
		}
	}
	else //No candidate found - return -1 in all fields.
	{
		post_candidates.push_back(vector<Point>());
		post_candidates[post_candidates.size()-1].push_back(Point(-1,-1));
		post_candidates[post_candidates.size()-1].push_back(Point(-1,-1));
		gc=GoalCandidate(post_candidates[post_candidates.size()-1],post_candidates[post_candidates.size()-1]);
	}

	//Only for showing the result:
	if(gc.m_left_post[0].x!=-1) //If found any post it will be stored into the left post.
	{
		Rect left_post,right_post;
		//Draw a rectangle around detected post/s:
		if(gc.m_right_post[0].x!=-1) //If found more than 1 post:
		{
			left_post.x=min(gc.m_left_post[0].x,gc.m_left_post[1].x);
			left_post.y=0;
			left_post.height=min(gc.m_left_post[0].y,gc.m_left_post[1].y);
			left_post.width=abs(gc.m_left_post[0].x-gc.m_left_post[1].x);
			cout << "left post: " << left_post << endl;
			right_post.x=min(gc.m_right_post[0].x,gc.m_right_post[1].x);
			right_post.y=0;
			right_post.height=min(gc.m_right_post[0].y,gc.m_right_post[1].y);
			right_post.width=abs(gc.m_right_post[0].x-gc.m_right_post[1].x);
			cout << "right post: " << right_post << endl;

			rectangle(frame,left_post,Scalar(255,0,0),2);
			rectangle(frame,right_post,Scalar(255,0,0),2);
		}
		else //Only one post found:
		{
			left_post.x=min(gc.m_left_post[0].x,gc.m_left_post[1].x);
			left_post.y=0;
			left_post.height=min(gc.m_left_post[0].y,gc.m_left_post[1].y);
			left_post.width=abs(gc.m_left_post[0].x-gc.m_left_post[1].x);
			cout << "left post: " << left_post << endl;
			rectangle(frame,left_post,Scalar(255,0,0),2);
		}
	}
	imshow("goal_detector_frame",frame);
	//imshow("whites_mat_after_rect",whites_mat_clone);
	//imshow("field_mat_after_rect",field_mat_clone);
	waitKey(1);
}

void GoalDetector::GetGoalPosts2(GoalCandidate& gc, Mat Goal_area)
{
	Mat frame,frame_hsv, frame_gray,original_field_mat, field_mat, whites_mat, field_space_mat, candidates_mat; //frame=origianl image. frame_hsv=image after transform to hsv. field_mat=only pixels in bounds of field's green after calibration are 0 (all the rest 255). whites_mat=only white pixels in image. field_space_mat=all pixels below bounding_horizontal_line (i.e -assumed to be field). candidates_mat=pixels which are not field and white and below bounding horizontal line.
	Mat hsv_channels[NUM_CHANNELS]; //Will contain all 3 HSV channels splitted.
	Mat bgr_channels[NUM_CHANNELS]; //Will contain all 3 BGR channels splitted.
	uchar field_min_hue, field_max_hue; //Will mark which pixel is a field pixel after calib.
	ushort bounding_horizontal_line;

	//frame = Goal_area.clone();
	VisionThread::SafeReadeCapturedFrame(frame); // DELETE IN THE FUTURE.
	//imshow("1", frame);

	split(frame, bgr_channels); //Split to B,G,R channels so we can manipulate the image later on.
	//Reduce noise:
	GaussianBlur(bgr_channels[0], bgr_channels[0], Size(3, 3), 2, 2);
	GaussianBlur(bgr_channels[1], bgr_channels[1], Size(3, 3), 2, 2);
	GaussianBlur(bgr_channels[2], bgr_channels[2], Size(3, 3), 2, 2);
	merge(bgr_channels, NUM_CHANNELS, frame); //Merge the channels back to one matrix after manipulation.
	//imshow("2", frame);

	int idx=0;
	cvtColor(frame, frame_hsv, CV_BGR2HSV); //Convert original RGB representation to HSV representation.
	//imshow("3", frame_hsv);

	split(frame_hsv, hsv_channels); //Split to H,S,V channels so we can use the hue,saturation&value matrices more conviniently later on.
	GoalDetector::FieldCalibration(hsv_channels[0], field_min_hue, field_max_hue);

	field_mat = Mat::zeros(frame.rows, frame.cols, CV_8UC1); //Generate a 1-channel matrix with size of original image (unsigned char). set all pixels to initail value 255=(11111111)_2
	whites_mat = Mat::zeros(frame.rows, frame.cols, CV_8UC1); //Generate a 1-channel matrix with size of original image (unsigned char).
	//cout << (uint) field_min_hue << " " << (uint) field_max_hue << endl;

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

		//	//Check hue value against bounds:
		//	if (hsv_channels[0].at<uchar>(i, j) >= field_min_hue && hsv_channels[0].at<uchar>(i, j) <= field_max_hue && whites_mat.at<uchar>(i, j)==0)
		//	{
		//		field_mat.at<uchar>(i, j) = 255; //If in range (i.e- field pixel) set to 0. // RE-COMMENT THIS
		//	}
		}
	}
	//imshow("4", whites_mat);
	//imshow("5", field_mat);

	// medianBlur(field_mat,field_mat,5);
	// imshow("field_mat", field_mat);
	//	imshow("whites_mat",whites_mat);
	// original_field_mat = field_mat.clone(); //Will be using the original mat later on with CHT.
	// imshow("original_field_mat", original_field_mat);
	// waitKey();
	// Perform morphological closing to field_mat:

	// erode(field_mat, field_mat, structure_element_); //First perform erosion to remove lone points and noise

	// structure_element_ = getStructuringElement(MORPH_ELLIPSE, Size(MIN_BALL_DIAMETER - 1, MIN_BALL_DIAMETER - 1));
	// Perform closing to the field_mat with circular structure element  -so we get trid of lines!-

	// erode(field_mat, field_mat, structure_element_);

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
	//imshow("6", field_mat);

	Mat structure_element_ = getStructuringElement(MORPH_RECT, Size(2*STRUCTURE_ELEMENT_SIZE, 2*STRUCTURE_ELEMENT_SIZE));
	dilate(field_mat, field_mat, structure_element_);
	// imshow("field_mat", field_mat);
	// waitKey(1);
	//imshow("7", field_mat);

	bitwise_not(field_mat, field_mat);
	Mat field_mat_clone=field_mat.clone(); //Copy because we draw rectangles on cloned image.
	//imshow("8", field_mat);

	//Calculate bounding horizontal line for field in image:
	GoalDetector::CalculateBoundingHorizontalLine(field_mat, bounding_horizontal_line);
	//cout << bounding_horizontal_line << endl;

	// if (bounding_horizontal_line < BOUNDING_HORIZONTAL_LINE_FIX)
	// {
	//	bounding_horizontal_line = 0;
	//}
	//else
	//{
	//	bounding_horizontal_line = bounding_horizontal_line - BOUNDING_HORIZONTAL_LINE_FIX;
	//}

	//Because the bounding horizontal line may cut the bottom of the goal we add some 'insurance' extra pixels - 15% of frame- 72 pixels
	//Generate field_space_mat=all pixels below bounding_horizontal_line (i.e -assumed to be field) are 255=(11111111)_2 and 0 else.
	field_space_mat = Mat::zeros(frame.rows, frame.cols, CV_8UC1);
	for (uint i = frame.rows - 1; i > 0; i--)
	{
		if (i >= bounding_horizontal_line+BOUNDING_HORIZONTAL_LINE_FIX ) //If below bounding_horizontal_line minus the bounding horizontal fix (which lets a bit more pixels from the frame to enter the field_space_mat)
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

	bitwise_not(field_space_mat,field_space_mat);
	//imshow("9", field_space_mat);

	// imshow("field_space_mat", field_space_mat);
	// waitKey(1);

	//bitwise_and(whites_mat, field_space_mat, whites_mat); (TEMPORARY??)
	//imshow("10", whites_mat);

	Mat element = getStructuringElement( MORPH_RECT, Size( 3, 3 ));

	//erode(whites_mat,whites_mat,element); //Perform erosion to get rid of noise.
	medianBlur(whites_mat,whites_mat,3);
	//imshow("11", whites_mat);

	//GaussianBlur(whites_mat,whites_mat,Size(3,3),1.5,1.5);
	//imshow("whites_mat", whites_mat);
	//waitKey(1);
	Mat whites_mat_clone=whites_mat.clone(); //Used to visualize the proccess (only for demonstrations etc.)

	// Parameters for Shi-Tomasi algorithm
	const int MAX_CORNERS_DISTANCE=80; //Max width of goal post
	vector<Point2f> corners;
	double qualityLevel = 0.0001; //Relative (to the best found corner in terms of high eigenvalues of the Harris matrix) quality of corner candidate.
	double minDistance = 5; //Min distance between detected corners (minimum num. of pixels between post's corners - i.e goal post width).
	int blockSize = 3;
	bool useHarrisDetector = false; //Use the cornerMinEigenVal() instead of corner harris.
	double k = 0.04;
	int maxCorners = 4000;

	/// Copy the source image
	Mat whites_mat_copy;
	whites_mat_copy = whites_mat.clone();

	// Apply corner detection
	goodFeaturesToTrack(whites_mat,
		corners,
		maxCorners,
		qualityLevel,
		minDistance,
		Mat(),
		blockSize,
		useHarrisDetector,
		k);

	// Draw corners detected
	//	cout << "** Number of corners detected: " << corners.size() << endl;
	const int R = 3;
	for (int i = 0; i < corners.size(); i++)
	{
		//circle(whites_mat_copy, corners[i], R, Scalar(127, 127, 127), -1, 8, 0);
	}
	//imshow("12", whites_mat_copy);
	//imshow("corners", whites_mat_copy);
	//waitKey(1);

	Mat candidate_i;
	int corner_1_x,corner_1_y,corner_2_x,corner_2_y;
	double slope;
	Rect goal_post_candidate_bounding_rect;// will get a bounding rectangle for candidate and check if it is filled with white pixels.
	Rect below_goal_post_candidate_bounding_rect; // will get a bounding rectangle for candidate and check if it is filled with field(green) pixels.
	Rect aside_goal_post_candidate_bounding_rect; // will get a bounding rectangle aside the candidate to check if no white pixels in it.
	int count_num_white_pixels_in_rect;
	int count_num_field_pixels_in_rect;
	Mat post_bounding_rect_mat; //Will hold a POST_WHITE_PIXELS_CHECK_NUM_PIXELS* (delta of corners x-coordinate) rectangle above the detected corners.
	Mat below_post_bounding_rect_mat; //Will hold a POST_FIELD_PIXELS_CHECK_NUM_PIXELS * (delta of corners x-coordinate) rectangle below the detected corners.
	Mat aside_post_bounding_rect_mat;//Will hold a POST_NO_WHITE_PIXELS_ASIDE_NUM_PIXELS * POST_WHITE_PIXELS_CHECK_NUM_PIXELS  rectangle aside the detected corners.
	double percentage_white_pixels; //Will hold the percentage of white pixels in bounding rectangle above candidate post.
	double percentage_field_pixels;//Will hold the percentage of field pixels in bounding rectangle below candidate post.

	bitwise_not(field_mat,field_mat);
	//imshow("13", field_mat);



	Mat goal_area = whites_mat_copy.clone();
	Mat dst, cdst, cdstP;

	vector<vector<Point>> post_candidates;


	//![edge_detection]
	// Edge detection
	Canny(goal_area, dst, 50, 200, 3);
	//![edge_detection]
//imshow("asdfgfd", dst);
	// Copy edges to the images that will display the results in BGR
	cvtColor(dst, cdst, COLOR_GRAY2BGR);
	cdstP = cdst.clone();

	//![hough_lines]
	// Standard Hough Line Transform
	vector<Vec2f> lines; // will hold the results of the detection

	//![hough_lines_p]
	// Probabilistic Line Transform
	vector<Vec4i> linesP; // will hold the results of the detection
	HoughLinesP(dst, linesP, 1, CV_PI/180, 15, 30, 10 ); // runs the actual detection

	int candidates = 0;
	for( size_t i = 0; i < linesP.size(); i++ )
	{
		Vec4i l = linesP[i];

		double m;
		if ((l[2] != l[0]))
		{
			m = (l[3] - l[1]) / (l[2] - l[0]);
		}
		else
		{
			m = 100/*000*/;
		}
		if (!(abs(m) <= 1))
		{
			for (int j = i+1; j < linesP.size(); j++)
			{
				double accuracy = 1;
				Vec4i l2 = linesP[j];


				double m2;
				if ((l2[2] != l2[0]))
				{
					m2 = (l2[3] - l2[1]) / (l2[2] - l2[0]);
				}
				else
				{
					m2 = 100/*000*/;
				}

				double angle;
				if ((m * m2) != -1)
				{
				angle = abs(atan( (m - m2) / (1 + m2 * m)) );
				}
				else
				{
					angle = PI / 2;
				}

				int minimum_x = min(l[2], l[0]);
				minimum_x = min(minimum_x, l2[0]);
				minimum_x = min(minimum_x, l2[2]);

				int maximum_x = max(l[2], l[0]);
				maximum_x = max(maximum_x, l2[0]);
				maximum_x = max(maximum_x, l2[2]);

				int minimum_y = min(l[3], l[1]);
				minimum_y = min(minimum_y, l2[1]);
				minimum_y = min(minimum_y, l2[3]);

				int maximum_y = max(l[3], l[1]);
				maximum_y = max(maximum_y, l2[1]);
				maximum_y = max(maximum_y, l2[3]);

				double distance = abs(-m * l[0] + l[1] - (-m2 * l2[0] + l2[1])) / sqrt(1+ pow(m,2));
				if ((maximum_x - minimum_x) >= 12 && (maximum_y - minimum_y) >= 50 && /*(double) (maximum_y - minimum_y) / (maximum_x - minimum_x) >= 2.5 &&*/ (maximum_x - minimum_x) <= 80 /*&& minimum_x >= 20*/ && angle <= 10 * PI / 180)
				{
					int height = 30;
					if (maximum_y + height >= goal_area.rows)
					{
						height = goal_area.rows - 1 - maximum_y;
					}
					//cout << "height: " << height << "maximum_y1: " << maximum_y1 << "goal_area.rows: " << goal_area.rows << endl;
					Rect below_goal_post_candidate_bounding_rect3;
					below_goal_post_candidate_bounding_rect3.x=minimum_x;
					below_goal_post_candidate_bounding_rect3.y=maximum_y;
					below_goal_post_candidate_bounding_rect3.width=abs(maximum_x-minimum_x);
					below_goal_post_candidate_bounding_rect3.height=height;
					int count_num_field_pixels_in_rect=0;

					Mat below_post_bounding_rect_mat=goal_area(below_goal_post_candidate_bounding_rect3);
					for(int k=0;k<below_post_bounding_rect_mat.rows;k++)
					{
						for(int m=0;m<below_post_bounding_rect_mat.cols;m++)
						{
							if(below_post_bounding_rect_mat.at<uchar>(k,m)>0)
							{
								count_num_field_pixels_in_rect++;
							}
						}
					}
					double ercentage_field_pixels;
					if (height > 0)
						ercentage_field_pixels=(count_num_field_pixels_in_rect+0.0)/(below_goal_post_candidate_bounding_rect3.width*below_goal_post_candidate_bounding_rect3.height);
					else
						ercentage_field_pixels=0;
					//if (ercentage_field_pixels <= 0.2)
					{
						accuracy *= (1-ercentage_field_pixels);
						// LEFT OF POST.
						int window_width = 35;
						if (minimum_x-window_width < 0)
						{
							window_width = minimum_x;
						}
						Rect below_goal_post_candidate_bounding_rect1;
						below_goal_post_candidate_bounding_rect1.x=minimum_x-window_width;
						below_goal_post_candidate_bounding_rect1.y=minimum_y;
						below_goal_post_candidate_bounding_rect1.width=window_width;
						below_goal_post_candidate_bounding_rect1.height=maximum_y-minimum_y;
						int count_num_field_pixels_in_rect=0;

						Mat below_post_bounding_rect_mat=goal_area(below_goal_post_candidate_bounding_rect1);
						for(int k=0;k<below_post_bounding_rect_mat.rows;k++)
						{
							for(int m=0;m<below_post_bounding_rect_mat.cols;m++)
							{
								if(below_post_bounding_rect_mat.at<uchar>(k,m)>0)
								{
									count_num_field_pixels_in_rect++;
								}
							}
						}
						double ercentage_field_pixels;
						if (window_width > 0)
							ercentage_field_pixels=(count_num_field_pixels_in_rect+0.0)/(below_goal_post_candidate_bounding_rect1.width*below_goal_post_candidate_bounding_rect1.height);
						else
							ercentage_field_pixels = 0;

						//if (ercentage_field_pixels <= 0.4)
						{
							accuracy *= (1-ercentage_field_pixels);
							int width = window_width;
							if (maximum_x + width >= goal_area.cols)
							{
								width = goal_area.cols - 1 - maximum_x;
							}

							// RIGHT TO THE POST.
							Rect below_goal_post_candidate_bounding_rect2;
							below_goal_post_candidate_bounding_rect2.x=maximum_x;
							below_goal_post_candidate_bounding_rect2.y=minimum_y;
							below_goal_post_candidate_bounding_rect2.width=width;
							below_goal_post_candidate_bounding_rect2.height=maximum_y-minimum_y;
							int count_num_field_pixels_in_rect=0;

							Mat below_post_bounding_rect_mat=goal_area(below_goal_post_candidate_bounding_rect2);
							for(int k=0;k<below_post_bounding_rect_mat.rows;k++)
							{
								for(int m=0;m<below_post_bounding_rect_mat.cols;m++)
								{
									if(below_post_bounding_rect_mat.at<uchar>(k,m)>0)
									{
										count_num_field_pixels_in_rect++;
									}
								}
							}
							double ercentage_field_pixels=(count_num_field_pixels_in_rect+0.0)/(below_goal_post_candidate_bounding_rect2.width*below_goal_post_candidate_bounding_rect2.height);
							//if (ercentage_field_pixels <= 0.4)
							{
								accuracy *= (1-ercentage_field_pixels);

								Rect below_goal_post_candidate_bounding_rect;
								below_goal_post_candidate_bounding_rect.x=minimum_x;
								below_goal_post_candidate_bounding_rect.y=minimum_y;
								below_goal_post_candidate_bounding_rect.width=maximum_x-minimum_x;
								below_goal_post_candidate_bounding_rect.height=maximum_y-minimum_y;
								int count_num_field_pixels_in_rect=0;

								Mat below_post_bounding_rect_mat=goal_area(below_goal_post_candidate_bounding_rect);
								int sampled_dots = 0;

								int count_0 = 0;
								int count_4 = 0;
								bool is_left = false;
								//cout << l[0] << " " << l[1] << " " << l[2] << " " << l[3] << " " << l[4] << " " << l[5] << " " << l[6] << " " << l[7] << endl;
								for(int k=0;k<below_post_bounding_rect_mat.rows;k++)
								{
									for(int m=0;m<below_post_bounding_rect_mat.cols;m++)
									{
										Point a, b, c, d, e;

										e.x = minimum_x + m;
										e.y = minimum_y + k;

										if (l[3] < l[1])
										{
											b.x = l[0];
											b.y = l[1];
											a.x = l[2];
											a.y = l[3];
										}
										else
										{
											a.x = l[0];
											a.y = l[1];
											b.x = l[2];
											b.y = l[3];
										}

										if (l2[3] < l2[1])
										{
											c.x = l2[0];
											c.y = l2[1];
											d.x = l2[2];
											d.y = l2[3];
										}
										else
										{
											d.x = l2[0];
											d.y = l2[1];
											c.x = l2[2];
											c.y = l2[3];
										}

										if (b.x > c.x || b.x > d.x)
										{
											is_left = true;
										}
										bool is_left1 = ((b.x - a.x)*(e.y - a.y) - (b.y - a.y)*(e.x - a.x)) > 0;
										bool is_left2 = ((c.x - b.x)*(e.y - b.y) - (c.y - b.y)*(e.x - b.x)) > 0;
										bool is_left3 = ((d.x - c.x)*(e.y - c.y) - (d.y - c.y)*(e.x - c.x)) > 0;
										bool is_left4 = ((a.x - d.x)*(e.y - d.y) - (a.y - d.y)*(e.x - d.x)) > 0;

										int sum = is_left1 + is_left3 + is_left3 + is_left4;
										if ((!is_left && sum == 0) || (is_left && sum == 4))
										{
											if (sum == 0)
											{
												count_0++;
											}
											if (sum == 4)
											{
												count_4++;
											}
											sampled_dots++;
											if(below_post_bounding_rect_mat.at<uchar>(k,m)>0)
											{
												count_num_field_pixels_in_rect++;
											}
										}
									}
								}

								double ercentage_field_pixels=(count_num_field_pixels_in_rect+0.0)/sampled_dots;
								if (/*ercentage_field_pixels >= 0.2 &&*/ sampled_dots >= 200)
								{
									accuracy *= (ercentage_field_pixels);

									//	cout << "Candidate: " << candidates << " count_num_field_pixels_in_rect: " << count_num_field_pixels_in_rect << " sampled_dots: " << sampled_dots << " Percent: " << ercentage_field_pixels << " accuracy: "  << accuracy << endl;
									//rectangle(goal_area,below_goal_post_candidate_bounding_rect,Scalar(120),2);
									if (accuracy >= 0.4)
									{
									candidates++;
									//cout << "Candidate: " << candidates << " Minimum_x: " << minimum_x << " MInimum_y: " << minimum_y << " Maximum_x: " << maximum_x << " Maximum_y: " << maximum_y << " Accuracy: " << accuracy << " Sampledots: " << sampled_dots << endl;
									post_candidates.push_back(vector<Point>());
									below_goal_post_candidate_bounding_rect.x=minimum_x;
									below_goal_post_candidate_bounding_rect.y=minimum_y;
									below_goal_post_candidate_bounding_rect.width=maximum_x-minimum_x;
									below_goal_post_candidate_bounding_rect.height=maximum_y-minimum_y;
									post_candidates[post_candidates.size()-1].push_back(Point(minimum_x , minimum_y));
									post_candidates[post_candidates.size()-1].push_back(Point(maximum_x ,maximum_y ));
									post_candidates[post_candidates.size()-1].push_back(Point(/*count_num_field_pixels_in_rect*/accuracy * 10000 ,/*count_num_field_pixels_in_rect*/distance ));
									//rectangle(frame,below_goal_post_candidate_bounding_rect,Scalar(0,255,0),2);
									}
								}
							}
						}
					}
				}
			}
			line(cdstP, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, LINE_AA);
		}
	}
//cout << "Before Cull: " << post_candidates.size() << endl;
int pixels_distance = 11;
	for(int i=0;i<post_candidates.size(); i++) //For every post conadidate found - choose the one with largest width first.
	{
		for(int j=i+1;j<post_candidates.size(); j++) //For every post conadidate found - choose the one with largest width first.
		{
			if (post_candidates[i][0].x >= post_candidates[j][0].x-pixels_distance && post_candidates[i][0].y >= post_candidates[j][0].y-pixels_distance && post_candidates[i][1].x <= post_candidates[j][1].x+pixels_distance && post_candidates[i][1].y <= post_candidates[j][1].y+pixels_distance)
			{
				// i is contained in j;
				post_candidates.erase(post_candidates.begin() + i);
				//i--;
				//j--;
				i = 0;
			}
			else if (post_candidates[j][0].x >= post_candidates[i][0].x-pixels_distance && post_candidates[j][0].y >= post_candidates[i][0].y-pixels_distance && post_candidates[j][1].x <= post_candidates[i][1].x+pixels_distance && post_candidates[j][1].y <= post_candidates[i][1].y+pixels_distance)
			{
				// j is contained in i;
				post_candidates.erase(post_candidates.begin() + j);
				j--;
			}
		}
	}
	//cout << "After Cull: " << post_candidates.size() << endl;

	for(int i=0;i<post_candidates.size(); i++)
	{
		Rect post_candidate_rect;
		post_candidate_rect.x = post_candidates[i][0].x;
		post_candidate_rect.y = post_candidates[i][0].y;
		post_candidate_rect.height = post_candidates[i][1].y - post_candidates[i][0].y;
		post_candidate_rect.width = post_candidates[i][1].x - post_candidates[i][0].x;
		//cout << "Candidate: " << i << " Minimum_x: " << post_candidate_rect.x << " MInimum_y: " << post_candidate_rect.y << " Maximum_x: " << post_candidates[i][1].x << " Maximum_y: " << post_candidates[i][1].y << " Accuracy: " << post_candidates[i][2].x << endl;
		rectangle(frame,post_candidate_rect,Scalar(0,255,0),2);
	}

	int max_dots_sampled=0;
	int index_of_max_width_candidate=0;
	int index_of_second_max_width_candidate=-1;
	const int MIN_DIST_BETWEEN_POSTS=100;

	//cout << post_candidates.size() << endl;

	if(post_candidates.size()>0) //If any candidate found:
	{
		for(int i=0;i<post_candidates.size(); i++) //For every post conadidate found - choose the one with largest width first.
		{
			if(post_candidates[i][2].x > max_dots_sampled && post_candidates[i][1].y - post_candidates[i][0].y > 120)
			{
				max_dots_sampled=post_candidates[i][2].x;
				index_of_max_width_candidate=i;
			}
		}
		//cout << "Mad dots sampled 1: " << max_dots_sampled << endl;
		if(post_candidates.size()>1)//If more than 1 candidate - search for another candidate:
		{
			max_dots_sampled=0;
			//cout <<index_of_second_max_width_candidate << endl;
			for(int i=0;i<post_candidates.size();i++) //For every post conadidate found - choose the one with largest width first.
			{
				if(i!=index_of_max_width_candidate && post_candidates[i][2].x>max_dots_sampled && (abs((post_candidates[i][0].x+post_candidates[i][1].x)/2.0 -(post_candidates[index_of_max_width_candidate][0].x+post_candidates[index_of_max_width_candidate][1].x)/2.0))>=MIN_DIST_BETWEEN_POSTS  )
				{
					max_dots_sampled=post_candidates[i][2].x;
					index_of_second_max_width_candidate=i;
				}
			}
		//	cout <<index_of_second_max_width_candidate << endl;

		//	cout << "Mad dots sampled 2: " << max_dots_sampled << endl;
		}

		if(index_of_second_max_width_candidate!=-1) //2 posts found:
		{
			if((post_candidates[index_of_max_width_candidate][0].x+post_candidates[index_of_max_width_candidate][1].x)/2.0 > (post_candidates[index_of_second_max_width_candidate][0].x+post_candidates[index_of_second_max_width_candidate][1].x)/2.0) //First candidate is the right post and second candidate is the left post.
			{
				gc=GoalCandidate(post_candidates[index_of_second_max_width_candidate],post_candidates[index_of_max_width_candidate]);

					Rect left_post,right_post;

					right_post.x= post_candidates[index_of_max_width_candidate][0].x;
					right_post.y = post_candidates[index_of_max_width_candidate][0].y;
					right_post.height=post_candidates[index_of_max_width_candidate][1].y - post_candidates[index_of_max_width_candidate][0].y;
					right_post.width=post_candidates[index_of_max_width_candidate][1].x - post_candidates[index_of_max_width_candidate][0].x;
					rectangle(frame,right_post,Scalar(255,0,0),2);

					left_post.x= post_candidates[index_of_second_max_width_candidate][0].x;
					left_post.y = post_candidates[index_of_second_max_width_candidate][0].y;
					left_post.height=post_candidates[index_of_second_max_width_candidate][1].y - post_candidates[index_of_second_max_width_candidate][0].y;
					left_post.width=post_candidates[index_of_second_max_width_candidate][1].x - post_candidates[index_of_second_max_width_candidate][0].x;
					rectangle(frame,left_post,Scalar(255,0,0),2);
			}
			else
			{
				Rect left_post,right_post;

				left_post.x= post_candidates[index_of_max_width_candidate][0].x;
				left_post.y = post_candidates[index_of_max_width_candidate][0].y;
				left_post.height=post_candidates[index_of_max_width_candidate][1].y - post_candidates[index_of_max_width_candidate][0].y;
				left_post.width=post_candidates[index_of_max_width_candidate][1].x - post_candidates[index_of_max_width_candidate][0].x;
				rectangle(frame,left_post,Scalar(255,0,0),2);

				right_post.x= post_candidates[index_of_second_max_width_candidate][0].x;
				right_post.y = post_candidates[index_of_second_max_width_candidate][0].y;
				right_post.height=post_candidates[index_of_second_max_width_candidate][1].y - post_candidates[index_of_second_max_width_candidate][0].y;
				right_post.width=post_candidates[index_of_second_max_width_candidate][1].x - post_candidates[index_of_second_max_width_candidate][0].x;
				rectangle(frame,right_post,Scalar(255,0,0),2);

				gc=GoalCandidate(post_candidates[index_of_max_width_candidate],post_candidates[index_of_second_max_width_candidate]);
			}
		}
		else //Only one post detected:
		{
			Rect left_post,right_post;
			left_post.x= post_candidates[index_of_max_width_candidate][0].x;
			left_post.y = post_candidates[index_of_max_width_candidate][0].y;
			left_post.height=post_candidates[index_of_max_width_candidate][1].y - post_candidates[index_of_max_width_candidate][0].y;
			left_post.width=post_candidates[index_of_max_width_candidate][1].x - post_candidates[index_of_max_width_candidate][0].x;
			rectangle(frame,left_post,Scalar(255,0,0),2);

			post_candidates.push_back(vector<Point>());
			post_candidates[post_candidates.size()-1].push_back(Point(-1,-1));
			post_candidates[post_candidates.size()-1].push_back(Point(-1,-1));

			gc=GoalCandidate(post_candidates[index_of_max_width_candidate],post_candidates[post_candidates.size()-1]);
		}
	}
	else //No candidate found - return -1 in all fields.
	{
		post_candidates.push_back(vector<Point>());
		post_candidates[post_candidates.size()-1].push_back(Point(-1,-1));
		post_candidates[post_candidates.size()-1].push_back(Point(-1,-1));
		gc=GoalCandidate(post_candidates[post_candidates.size()-1],post_candidates[post_candidates.size()-1]);
	}

	imshow("Detected Goal Posts", frame);
	//imshow("goal_detector_frame",cdstP);


	//imshow("sadfghjkjhgfd", goal_area);
	//imshow("whites_mat_after_rect",whites_mat_clone);
	//imshow("field_mat_after_rect",field_mat_clone);
	waitKey(1);
	waitKey(500);
}




void GoalDetector::GetGoalPosts3(GoalCandidate& gc, Mat Goal_area)
{
	Mat frame,frame_hsv, frame_gray,original_field_mat, field_mat, whites_mat, field_space_mat, candidates_mat; //frame=origianl image. frame_hsv=image after transform to hsv. field_mat=only pixels in bounds of field's green after calibration are 0 (all the rest 255). whites_mat=only white pixels in image. field_space_mat=all pixels below bounding_horizontal_line (i.e -assumed to be field). candidates_mat=pixels which are not field and white and below bounding horizontal line.
	Mat hsv_channels[NUM_CHANNELS]; //Will contain all 3 HSV channels splitted.
	Mat bgr_channels[NUM_CHANNELS]; //Will contain all 3 BGR channels splitted.
	uchar field_min_hue, field_max_hue; //Will mark which pixel is a field pixel after calib.
	ushort bounding_horizontal_line;

	//frame = Goal_area.clone();
	VisionThread::SafeReadeCapturedFrame(frame); // DELETE IN THE FUTURE.
	imshow("1", frame);

	split(frame, bgr_channels); //Split to B,G,R channels so we can manipulate the image later on.
	//Reduce noise:
	GaussianBlur(bgr_channels[0], bgr_channels[0], Size(3, 3), 2, 2);
	GaussianBlur(bgr_channels[1], bgr_channels[1], Size(3, 3), 2, 2);
	GaussianBlur(bgr_channels[2], bgr_channels[2], Size(3, 3), 2, 2);
	merge(bgr_channels, NUM_CHANNELS, frame); //Merge the channels back to one matrix after manipulation.
	imshow("2", frame);

	int idx=0;
	cvtColor(frame, frame_hsv, CV_BGR2HSV); //Convert original RGB representation to HSV representation.
	imshow("3", frame_hsv);

	split(frame_hsv, hsv_channels); //Split to H,S,V channels so we can use the hue,saturation&value matrices more conviniently later on.
	GoalDetector::FieldCalibration(hsv_channels[0], field_min_hue, field_max_hue);

	field_mat = Mat::zeros(frame.rows, frame.cols, CV_8UC1); //Generate a 1-channel matrix with size of original image (unsigned char). set all pixels to initail value 255=(11111111)_2
	whites_mat = Mat::zeros(frame.rows, frame.cols, CV_8UC1); //Generate a 1-channel matrix with size of original image (unsigned char).
cout << (uint) field_min_hue << " " << (uint) field_max_hue << endl;

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

//			//Check hue value against bounds:
//			if (hsv_channels[0].at<uchar>(i, j) >= field_min_hue && hsv_channels[0].at<uchar>(i, j) <= field_max_hue && whites_mat.at<uchar>(i, j)==0)
//			{
//				field_mat.at<uchar>(i, j) = 255; //If in range (i.e- field pixel) set to 0.
//			}
		}
	}
	imshow("4", whites_mat);
	imshow("5", field_mat);

	// medianBlur(field_mat,field_mat,5);
	// imshow("field_mat", field_mat);
	//	imshow("whites_mat",whites_mat);
	// original_field_mat = field_mat.clone(); //Will be using the original mat later on with CHT.
	// imshow("original_field_mat", original_field_mat);
	// waitKey();
	// Perform morphological closing to field_mat:

	// erode(field_mat, field_mat, structure_element_); //First perform erosion to remove lone points and noise

	// structure_element_ = getStructuringElement(MORPH_ELLIPSE, Size(MIN_BALL_DIAMETER - 1, MIN_BALL_DIAMETER - 1));
	// Perform closing to the field_mat with circular structure element  -so we get trid of lines!-

	// erode(field_mat, field_mat, structure_element_);

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
	imshow("6", field_mat);

	Mat structure_element_ = getStructuringElement(MORPH_RECT, Size(2*STRUCTURE_ELEMENT_SIZE, 2*STRUCTURE_ELEMENT_SIZE));
	dilate(field_mat, field_mat, structure_element_);
	// imshow("field_mat", field_mat);
	// waitKey(1);
	imshow("7", field_mat);

	bitwise_not(field_mat, field_mat);
	Mat field_mat_clone=field_mat.clone(); //Copy because we draw rectangles on cloned image.
	imshow("8", field_mat);

	//Calculate bounding horizontal line for field in image:
	GoalDetector::CalculateBoundingHorizontalLine(field_mat, bounding_horizontal_line);
	cout << bounding_horizontal_line << endl;

	// if (bounding_horizontal_line < BOUNDING_HORIZONTAL_LINE_FIX)
	// {
	//	bounding_horizontal_line = 0;
	//}
	//else
	//{
	//	bounding_horizontal_line = bounding_horizontal_line - BOUNDING_HORIZONTAL_LINE_FIX;
	//}

	//Because the bounding horizontal line may cut the bottom of the goal we add some 'insurance' extra pixels - 15% of frame- 72 pixels
	//Generate field_space_mat=all pixels below bounding_horizontal_line (i.e -assumed to be field) are 255=(11111111)_2 and 0 else.
	field_space_mat = Mat::zeros(frame.rows, frame.cols, CV_8UC1);
	for (uint i = frame.rows - 1; i > 0; i--)
	{
		if (i >= bounding_horizontal_line+BOUNDING_HORIZONTAL_LINE_FIX ) //If below bounding_horizontal_line minus the bounding horizontal fix (which lets a bit more pixels from the frame to enter the field_space_mat)
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

	bitwise_not(field_space_mat,field_space_mat);
	imshow("9", field_space_mat);

	// imshow("field_space_mat", field_space_mat);
	// waitKey(1);

	//bitwise_and(whites_mat, field_space_mat, whites_mat); (TEMPORARY??)
	imshow("10", whites_mat);

	Mat element = getStructuringElement( MORPH_RECT, Size( 3, 3 ));

	//erode(whites_mat,whites_mat,element); //Perform erosion to get rid of noise.
	medianBlur(whites_mat,whites_mat,3);
	imshow("11", whites_mat);

	//GaussianBlur(whites_mat,whites_mat,Size(3,3),1.5,1.5);
	//imshow("whites_mat", whites_mat);
	//waitKey(1);
	Mat whites_mat_clone=whites_mat.clone(); //Used to visualize the proccess (only for demonstrations etc.)

	// Parameters for Shi-Tomasi algorithm
	const int MAX_CORNERS_DISTANCE=80; //Max width of goal post
	vector<Point2f> corners;
	double qualityLevel = 0.0001; //Relative (to the best found corner in terms of high eigenvalues of the Harris matrix) quality of corner candidate.
	double minDistance = 5; //Min distance between detected corners (minimum num. of pixels between post's corners - i.e goal post width).
	int blockSize = 3;
	bool useHarrisDetector = false; //Use the cornerMinEigenVal() instead of corner harris.
	double k = 0.04;
	int maxCorners = 4000;

	/// Copy the source image
	Mat whites_mat_copy;
	whites_mat_copy = whites_mat.clone();

	// Apply corner detection
	goodFeaturesToTrack(whites_mat,
		corners,
		maxCorners,
		qualityLevel,
		minDistance,
		Mat(),
		blockSize,
		useHarrisDetector,
		k);

	// Draw corners detected
	//	cout << "** Number of corners detected: " << corners.size() << endl;
	const int R = 3;
	for (int i = 0; i < corners.size(); i++)
	{
		//circle(whites_mat_copy, corners[i], R, Scalar(127, 127, 127), -1, 8, 0);
	}
	imshow("12", whites_mat_copy);
	//imshow("corners", whites_mat_copy);
	//waitKey(1);

	Mat candidate_i;
	int corner_1_x,corner_1_y,corner_2_x,corner_2_y;
	double slope;
	Rect goal_post_candidate_bounding_rect;// will get a bounding rectangle for candidate and check if it is filled with white pixels.
	Rect below_goal_post_candidate_bounding_rect; // will get a bounding rectangle for candidate and check if it is filled with field(green) pixels.
	Rect aside_goal_post_candidate_bounding_rect; // will get a bounding rectangle aside the candidate to check if no white pixels in it.
	int count_num_white_pixels_in_rect;
	int count_num_field_pixels_in_rect;
	Mat post_bounding_rect_mat; //Will hold a POST_WHITE_PIXELS_CHECK_NUM_PIXELS* (delta of corners x-coordinate) rectangle above the detected corners.
	Mat below_post_bounding_rect_mat; //Will hold a POST_FIELD_PIXELS_CHECK_NUM_PIXELS * (delta of corners x-coordinate) rectangle below the detected corners.
	Mat aside_post_bounding_rect_mat;//Will hold a POST_NO_WHITE_PIXELS_ASIDE_NUM_PIXELS * POST_WHITE_PIXELS_CHECK_NUM_PIXELS  rectangle aside the detected corners.
	double percentage_white_pixels; //Will hold the percentage of white pixels in bounding rectangle above candidate post.
	double percentage_field_pixels;//Will hold the percentage of field pixels in bounding rectangle below candidate post.

	bitwise_not(field_mat,field_mat);
	imshow("13", field_mat);



	Mat goal_area = whites_mat_copy.clone();
	Mat dst, cdst, cdstP;

	vector<vector<Point>> post_candidates;


	//![edge_detection]
	// Edge detection
	Canny(goal_area, dst, 50, 200, 3);
	//![edge_detection]
imshow("asdfgfd", dst);
	// Copy edges to the images that will display the results in BGR
	cvtColor(dst, cdst, COLOR_GRAY2BGR);
	cdstP = cdst.clone();

	//![hough_lines]
	// Standard Hough Line Transform
	vector<Vec2f> lines; // will hold the results of the detection

	//![hough_lines_p]
	// Probabilistic Line Transform
	vector<Vec4i> linesP; // will hold the results of the detection
	HoughLinesP(dst, linesP, 1, CV_PI/180, 15, 30, 10 ); // runs the actual detection

	int candidates = 0;
	for( size_t i = 0; i < linesP.size(); i++ )
	{
		Vec4i l = linesP[i];

		double m;
		if ((l[2] != l[0]))
		{
			m = (l[3] - l[1]) / (l[2] - l[0]);
		}
		else
		{
			m = 100000;
		}
		if (!(abs(m) > 1))
		{



			for (int j = i+1; j < linesP.size(); j++)
			{
				double accuracy = 1;
				Vec4i l2 = linesP[j];


				double m2;
				if ((l2[2] != l2[0]))
				{
					m2 = (l2[3] - l2[1]) / (l2[2] - l2[0]);
				}
				else
				{
					m2 = 100000;
				}

				double angle;
				if ((m * m2) != -1)
				{
				angle = abs(atan( (m - m2) / (1 + m2 * m)) );
				}
				else
				{
					angle = PI / 2;
				}




				int minimum_x = min(l[2], l[0]);
				minimum_x = min(minimum_x, l2[0]);
				minimum_x = min(minimum_x, l2[2]);

				int maximum_x = max(l[2], l[0]);
				maximum_x = max(maximum_x, l2[0]);
				maximum_x = max(maximum_x, l2[2]);

				int minimum_y = min(l[3], l[1]);
				minimum_y = min(minimum_y, l2[1]);
				minimum_y = min(minimum_y, l2[3]);

				int maximum_y = max(l[3], l[1]);
				maximum_y = max(maximum_y, l2[1]);
				maximum_y = max(maximum_y, l2[3]);

				if (/*(double) (maximum_y - minimum_y) / (maximum_x - minimum_x) >= 2.5 &&*/ (maximum_y - minimum_y) <= 80 /*&& minimum_x >= 20*/ /*&& angle <= 10 * PI / 180*/)
				{
					int height = 30;
					if (maximum_y + height >= goal_area.rows)
					{
						height = goal_area.rows - 1 - maximum_y;
					}
					//cout << "height: " << height << "maximum_y1: " << maximum_y1 << "goal_area.rows: " << goal_area.rows << endl;
//					Rect below_goal_post_candidate_bounding_rect3;
//					below_goal_post_candidate_bounding_rect3.x=minimum_x;
//					below_goal_post_candidate_bounding_rect3.y=maximum_y;
//					below_goal_post_candidate_bounding_rect3.width=abs(maximum_x-minimum_x);
//					below_goal_post_candidate_bounding_rect3.height=height;
//					int count_num_field_pixels_in_rect=0;
//
//					Mat below_post_bounding_rect_mat=goal_area(below_goal_post_candidate_bounding_rect3);
//					for(int k=0;k<below_post_bounding_rect_mat.rows;k++)
//					{
//						for(int m=0;m<below_post_bounding_rect_mat.cols;m++)
//						{
//							if(below_post_bounding_rect_mat.at<uchar>(k,m)>0)
//							{
//								count_num_field_pixels_in_rect++;
//							}
//						}
//					}
					double ercentage_field_pixels=0;
					/*if (height > 0)
						ercentage_field_pixels=(count_num_field_pixels_in_rect+0.0)/(below_goal_post_candidate_bounding_rect3.width*below_goal_post_candidate_bounding_rect3.height);
					else
						ercentage_field_pixels=0;
*/					//if (ercentage_field_pixels <= 0.2)
					{
						accuracy *= (1-ercentage_field_pixels);
						// TOP OF POST.
						int window_height = 35;
						if (minimum_y-window_height < 0)
						{
							window_height = minimum_y;
						}
						Rect below_goal_post_candidate_bounding_rect1;
						below_goal_post_candidate_bounding_rect1.x=minimum_x;
						below_goal_post_candidate_bounding_rect1.y=minimum_y-window_height;
						below_goal_post_candidate_bounding_rect1.width=maximum_x-minimum_x;
						below_goal_post_candidate_bounding_rect1.height=window_height;
						int count_num_field_pixels_in_rect=0;

						Mat below_post_bounding_rect_mat=goal_area(below_goal_post_candidate_bounding_rect1);
						for(int k=0;k<below_post_bounding_rect_mat.rows;k++)
						{
							for(int m=0;m<below_post_bounding_rect_mat.cols;m++)
							{
								if(below_post_bounding_rect_mat.at<uchar>(k,m)>0)
								{
									count_num_field_pixels_in_rect++;
								}
							}
						}
						double ercentage_field_pixels;
						if (window_height > 0)
							ercentage_field_pixels=(count_num_field_pixels_in_rect+0.0)/(below_goal_post_candidate_bounding_rect1.width*below_goal_post_candidate_bounding_rect1.height);
						else
							ercentage_field_pixels = 0;

						//if (ercentage_field_pixels <= 0.4)
						{
							accuracy *= (1-ercentage_field_pixels);
							int height = window_height;
							if (maximum_y + height >= goal_area.rows)
							{
								height = goal_area.rows - 1 - maximum_y;
							}

							// BOTTOM TO THE POST.
							Rect below_goal_post_candidate_bounding_rect2;
							below_goal_post_candidate_bounding_rect2.x=minimum_x;
							below_goal_post_candidate_bounding_rect2.y=maximum_y;
							below_goal_post_candidate_bounding_rect2.width=maximum_x-minimum_x;
							below_goal_post_candidate_bounding_rect2.height=height;
							int count_num_field_pixels_in_rect=0;

							Mat below_post_bounding_rect_mat=goal_area(below_goal_post_candidate_bounding_rect2);
							for(int k=0;k<below_post_bounding_rect_mat.rows;k++)
							{
								for(int m=0;m<below_post_bounding_rect_mat.cols;m++)
								{
									if(below_post_bounding_rect_mat.at<uchar>(k,m)>0)
									{
										count_num_field_pixels_in_rect++;
									}
								}
							}
							double ercentage_field_pixels=(count_num_field_pixels_in_rect+0.0)/(below_goal_post_candidate_bounding_rect2.width*below_goal_post_candidate_bounding_rect2.height);
							//if (ercentage_field_pixels <= 0.4)
							{
								accuracy *= (1-ercentage_field_pixels);

						Rect below_goal_post_candidate_bounding_rect;
						below_goal_post_candidate_bounding_rect.x=minimum_x;
						below_goal_post_candidate_bounding_rect.y=minimum_y;
						below_goal_post_candidate_bounding_rect.width=maximum_x-minimum_x;
						below_goal_post_candidate_bounding_rect.height=maximum_y-minimum_y;
						int count_num_field_pixels_in_rect=0;

						Mat below_post_bounding_rect_mat=goal_area(below_goal_post_candidate_bounding_rect);
						int sampled_dots = 0;

						int count_0 = 0;
						int count_4 = 0;
						bool is_left = false;
						//cout << l[0] << " " << l[1] << " " << l[2] << " " << l[3] << " " << l[4] << " " << l[5] << " " << l[6] << " " << l[7] << endl;
						for(int k=0;k<below_post_bounding_rect_mat.rows;k++)
						{
							for(int m=0;m<below_post_bounding_rect_mat.cols;m++)
							{
								Point a, b, c, d, e;

								e.x = minimum_x + m;
								e.y = minimum_y + k;

								if (l[3] < l[1])
								{
									b.x = l[0];
									b.y = l[1];
									a.x = l[2];
									a.y = l[3];
								}
								else
								{
									a.x = l[0];
									a.y = l[1];
									b.x = l[2];
									b.y = l[3];
								}

								if (l2[3] < l2[1])
								{
									c.x = l2[0];
									c.y = l2[1];
									d.x = l2[2];
									d.y = l2[3];
								}
								else
								{
									d.x = l2[0];
									d.y = l2[1];
									c.x = l2[2];
									c.y = l2[3];
								}

								if (b.x > c.x || b.x > d.x)
								{
									is_left = true;
								}
								bool is_left1 = ((b.x - a.x)*(e.y - a.y) - (b.y - a.y)*(e.x - a.x)) > 0;
								bool is_left2 = ((c.x - b.x)*(e.y - b.y) - (c.y - b.y)*(e.x - b.x)) > 0;
								bool is_left3 = ((d.x - c.x)*(e.y - c.y) - (d.y - c.y)*(e.x - c.x)) > 0;
								bool is_left4 = ((a.x - d.x)*(e.y - d.y) - (a.y - d.y)*(e.x - d.x)) > 0;

								int sum = is_left1 + is_left3 + is_left3 + is_left4;
								if ((!is_left && sum == 0) || (is_left && sum == 4))
								{
									if (sum == 0)
									{
										count_0++;
									}
									if (sum == 4)
									{
										count_4++;
									}
									sampled_dots++;
									if(below_post_bounding_rect_mat.at<uchar>(k,m)>0)
									{
										count_num_field_pixels_in_rect++;
									}
								}
							}
						}

						double ercentage_field_pixels=(count_num_field_pixels_in_rect+0.0)/sampled_dots;
						if (ercentage_field_pixels >= 0.2 && sampled_dots >= 50)
						{
							accuracy *= (ercentage_field_pixels);
						//	cout << "Candidate: " << candidates << " count_num_field_pixels_in_rect: " << count_num_field_pixels_in_rect << " sampled_dots: " << sampled_dots << " Percent: " << ercentage_field_pixels << " accuracy: "  << accuracy << endl;
									//rectangle(goal_area,below_goal_post_candidate_bounding_rect,Scalar(120),2);
									candidates++;
									post_candidates.push_back(vector<Point>());
									below_goal_post_candidate_bounding_rect.x=minimum_x;
									below_goal_post_candidate_bounding_rect.y=minimum_y;
									below_goal_post_candidate_bounding_rect.width=maximum_x-minimum_x;
									below_goal_post_candidate_bounding_rect.height=maximum_y-minimum_y;
									post_candidates[post_candidates.size()-1].push_back(Point(minimum_x , minimum_y));
									post_candidates[post_candidates.size()-1].push_back(Point(maximum_x ,maximum_y ));
									post_candidates[post_candidates.size()-1].push_back(Point(/*count_num_field_pixels_in_rect*/accuracy * 10000 ,/*count_num_field_pixels_in_rect*/accuracy * 10000  ));
								//	rectangle(frame,below_goal_post_candidate_bounding_rect,Scalar(0,255,0),2);

						}
							}
						}
					}
				}

			}
			line(cdstP, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, LINE_AA);
		}
	}



	int max_dots_sampled=0;
	int index_of_max_width_candidate=0;
	int index_of_second_max_width_candidate=-1;
	const int MIN_DIST_BETWEEN_POSTS=100;

	cout << post_candidates.size() << endl;

	if(post_candidates.size()>0) //If any candidate found:
	{
		for(int i=0;i<post_candidates.size();i++) //For every post conadidate found - choose the one with largest width first.
		{
			if(post_candidates[i][2].x > max_dots_sampled)
			{
				max_dots_sampled=post_candidates[i][2].x;
				index_of_max_width_candidate=i;
			}
		}
		if(post_candidates.size()>1)//If more than 1 candidate - search for another candidate:
		{
			max_dots_sampled=0;
			for(int i=0;i<post_candidates.size();i++) //For every post conadidate found - choose the one with largest width first.
			{
				if(i!=index_of_max_width_candidate && post_candidates[i][2].x>max_dots_sampled && (abs((post_candidates[i][0].x+post_candidates[i][1].x)/2.0 -(post_candidates[index_of_max_width_candidate][0].x+post_candidates[index_of_max_width_candidate][1].x)/2.0))>=MIN_DIST_BETWEEN_POSTS  )
				{
					max_dots_sampled=post_candidates[i][2].x;
					index_of_second_max_width_candidate=i;
				}
			}
		}

		if(index_of_second_max_width_candidate!=-1) //2 posts found:
		{
			if((post_candidates[index_of_max_width_candidate][0].x+post_candidates[index_of_max_width_candidate][1].x)/2.0 > (post_candidates[index_of_second_max_width_candidate][0].x+post_candidates[index_of_second_max_width_candidate][1].x)/2.0) //First candidate is the right post and second candidate is the left post.
			{
				gc=GoalCandidate(post_candidates[index_of_second_max_width_candidate],post_candidates[index_of_max_width_candidate]);


					Rect left_post,right_post;

					left_post.x= post_candidates[index_of_max_width_candidate][0].x;
					left_post.y = post_candidates[index_of_max_width_candidate][0].y;
					left_post.height=post_candidates[index_of_max_width_candidate][1].y - post_candidates[index_of_max_width_candidate][0].y;
					left_post.width=post_candidates[index_of_max_width_candidate][1].x - post_candidates[index_of_max_width_candidate][0].x;
					rectangle(frame,left_post,Scalar(255,0,0),2);
				//	cout << "Left post width: " << left_post.width << endl;

					right_post.x= post_candidates[index_of_second_max_width_candidate][0].x;
					right_post.y = post_candidates[index_of_second_max_width_candidate][0].y;
					right_post.height=post_candidates[index_of_second_max_width_candidate][1].y - post_candidates[index_of_second_max_width_candidate][0].y;
					right_post.width=post_candidates[index_of_second_max_width_candidate][1].x - post_candidates[index_of_second_max_width_candidate][0].x;
					rectangle(frame,right_post,Scalar(255,0,0),2);
				//	cout << "Right post width: " << right_post.width << endl;

					if (post_candidates[index_of_max_width_candidate][0].x > post_candidates[index_of_second_max_width_candidate][0].x)
					{
						cout << " Left: " << post_candidates[index_of_second_max_width_candidate][1].x - post_candidates[index_of_second_max_width_candidate][0].x;
						cout << " Right: " << post_candidates[index_of_max_width_candidate][1].x - post_candidates[index_of_max_width_candidate][0].x << endl;
					}
					else
					{
						cout << " Left: " << post_candidates[index_of_max_width_candidate][1].x - post_candidates[index_of_max_width_candidate][0].x;
						cout << " Right: " << post_candidates[index_of_second_max_width_candidate][1].x - post_candidates[index_of_second_max_width_candidate][0].x << endl;

					}

			}
			else
			{
				Rect left_post,right_post;

				left_post.x= post_candidates[index_of_max_width_candidate][0].x;
				left_post.y = post_candidates[index_of_max_width_candidate][0].y;
				left_post.height=post_candidates[index_of_max_width_candidate][1].y - post_candidates[index_of_max_width_candidate][0].y;
				left_post.width=post_candidates[index_of_max_width_candidate][1].x - post_candidates[index_of_max_width_candidate][0].x;
				rectangle(frame,left_post,Scalar(255,0,0),2);

				right_post.x= post_candidates[index_of_second_max_width_candidate][0].x;
				right_post.y = post_candidates[index_of_second_max_width_candidate][0].y;
				right_post.height=post_candidates[index_of_second_max_width_candidate][1].y - post_candidates[index_of_second_max_width_candidate][0].y;
				right_post.width=post_candidates[index_of_second_max_width_candidate][1].x - post_candidates[index_of_second_max_width_candidate][0].x;
				rectangle(frame,right_post,Scalar(255,0,0),2);
			//	cout << "Left: " << left_post.width << " right: " << right_post.width << endl;


				if (post_candidates[index_of_max_width_candidate][0].x > post_candidates[index_of_second_max_width_candidate][0].x)
				{
					cout << " Left: " << post_candidates[index_of_second_max_width_candidate][1].x - post_candidates[index_of_second_max_width_candidate][0].x;
					cout << " Right: " << post_candidates[index_of_max_width_candidate][1].x - post_candidates[index_of_max_width_candidate][0].x << endl;
				}
				else
				{
					cout << " Left: " << post_candidates[index_of_max_width_candidate][1].x - post_candidates[index_of_max_width_candidate][0].x;
					cout << " Right: " << post_candidates[index_of_second_max_width_candidate][1].x - post_candidates[index_of_second_max_width_candidate][0].x << endl;

				}

				gc=GoalCandidate(post_candidates[index_of_max_width_candidate],post_candidates[index_of_second_max_width_candidate]);
			}
		}
		else //Only one post detected:
		{
			Rect left_post,right_post;
			left_post.x= post_candidates[index_of_max_width_candidate][0].x;
			left_post.y = post_candidates[index_of_max_width_candidate][0].y;
			left_post.height=post_candidates[index_of_max_width_candidate][1].y - post_candidates[index_of_max_width_candidate][0].y;
			left_post.width=post_candidates[index_of_max_width_candidate][1].x - post_candidates[index_of_max_width_candidate][0].x;
			rectangle(frame,left_post,Scalar(255,0,0),2);

			post_candidates.push_back(vector<Point>());
			post_candidates[post_candidates.size()-1].push_back(Point(-1,-1));
			post_candidates[post_candidates.size()-1].push_back(Point(-1,-1));


			cout << "Left: " << left_post.width << endl;

			//gc=GoalCandidate(post_candidates[index_of_max_width_candidate],post_candidates[post_candidates.size()-1]);
		}
	}
	else //No candidate found - return -1 in all fields.
	{
		post_candidates.push_back(vector<Point>());
		post_candidates[post_candidates.size()-1].push_back(Point(-1,-1));
		post_candidates[post_candidates.size()-1].push_back(Point(-1,-1));
		//gc=GoalCandidate(post_candidates[post_candidates.size()-1],post_candidates[post_candidates.size()-1]);
	}


	imshow("dfghffj", frame);



	imshow("sadfghjkjhgfd", goal_area);
	cout << candidates << endl;



	imshow("goal_detector_frame",cdstP);
	//imshow("whites_mat_after_rect",whites_mat_clone);
	//imshow("field_mat_after_rect",field_mat_clone);
	waitKey(1);
}




/*This method calibrates the green field hue bounds. It takes the hue matrix and returns the- field_min_hue and field_max_hue.
	We create the histogram of hue for the given matrix and take PERCENTAGE_THRESHOLD (=90%) in the middle of the green spectrum -
	assuming values on the edges of the green spectrum might be noise.*/
void GoalDetector::FieldCalibration(Mat& hue_matrix, uchar& field_min_hue, uchar& field_max_hue)
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
void GoalDetector::CalculateBoundingHorizontalLine(Mat& field_mat, ushort& bounding_horizontal_line)
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

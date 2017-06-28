

#include "GoalDetector.h"


void GoalDetector::GetGoalPosts(GoalCandidate& gc){
	    Mat frame,frame_hsv, frame_gray,original_field_mat, field_mat, whites_mat, field_space_mat, candidates_mat; //frame=origianl image. frame_hsv=image after transform to hsv. field_mat=only pixels in bounds of field's green after calibration are 0 (all the rest 255). whites_mat=only white pixels in image. field_space_mat=all pixels below bounding_horizontal_line (i.e -assumed to be field). candidates_mat=pixels which are not field and white and below bounding horizontal line.
		Mat hsv_channels[NUM_CHANNELS]; //Will contain all 3 HSV channels splitted.
		Mat bgr_channels[NUM_CHANNELS]; //Will contain all 3 BGR channels splitted.
		uchar field_min_hue, field_max_hue; //Will mark which pixel is a field pixel after calib.
		ushort bounding_horizontal_line;


		VisionThread::SafeReadeCapturedFrame(frame);
					split(frame, bgr_channels); //Split to B,G,R channels so we can manipulate the image later on.
		//			//Reduce noise:
					GaussianBlur(bgr_channels[0], bgr_channels[0], Size(3, 3), 2, 2);
					GaussianBlur(bgr_channels[1], bgr_channels[1], Size(3, 3), 2, 2);
					GaussianBlur(bgr_channels[2], bgr_channels[2], Size(3, 3), 2, 2);
					merge(bgr_channels, NUM_CHANNELS, frame); //Merge the channels back to one matrix after manipulation.


		int idx=0;
				cvtColor(frame, frame_hsv, CV_BGR2HSV); //Convert original RGB representation to HSV representation.

				split(frame_hsv, hsv_channels); //Split to H,S,V channels so we can use the hue,saturation&value matrices more conviniently later on.
				GoalDetector::FieldCalibration(hsv_channels[0], field_min_hue, field_max_hue);
				field_mat = Mat::zeros(frame.rows, frame.cols, CV_8UC1); //Generate a 1-channel matrix with size of original image (unsigned char). set all pixels to initail value 255=(11111111)_2
				whites_mat = Mat::zeros(frame.rows, frame.cols, CV_8UC1); //Generate a 1-channel matrix with size of original image (unsigned char).

																		  //Generate the field_mat and whites_mat:
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
//				imshow("field_mat", field_mat);
//				imshow("whites_mat",whites_mat);
//				original_field_mat = field_mat.clone(); //Will be using the original mat later on with CHT.
//				//imshow("original_field_mat", original_field_mat);
//				//waitKey();
//				//Perform morphological closing to field_mat:
				Mat structure_element_ = getStructuringElement(MORPH_RECT, Size(STRUCTURE_ELEMENT_SIZE, STRUCTURE_ELEMENT_SIZE));
//				erode(field_mat, field_mat, structure_element_); //First perform erosion to remove lone points and noise
				medianBlur(field_mat, field_mat, 7); //Median filter to remove small blocks of white in black areas and vise versa.
//				structure_element_ = getStructuringElement(MORPH_ELLIPSE, Size(MIN_BALL_DIAMETER - 1, MIN_BALL_DIAMETER - 1));
//				//Perform closing to the field_mat with circular structure element  -so we get trid of lines!-
				dilate(field_mat, field_mat, structure_element_);
//				erode(field_mat, field_mat, structure_element_);


				imshow("field_mat", field_mat);
				waitKey(1);

				bitwise_not(field_mat, field_mat);
				Mat field_mat_clone=field_mat.clone(); //Copy because we draw rectangles on cloned image.

				//Calculate bounding horizontal line for field in image:
				GoalDetector::CalculateBoundingHorizontalLine(field_mat, bounding_horizontal_line);
//				if (bounding_horizontal_line < BOUNDING_HORIZONTAL_LINE_FIX)
//				{
//					bounding_horizontal_line = 0;
//				}
//				else
//				{
//					bounding_horizontal_line = bounding_horizontal_line - BOUNDING_HORIZONTAL_LINE_FIX;
//				}

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
				imshow("field_space_mat", field_space_mat);
				waitKey(1);

				bitwise_and(whites_mat, field_space_mat, whites_mat);

				Mat element = getStructuringElement( MORPH_RECT,
				                                       Size( 3, 3 ));

			    //erode(whites_mat,whites_mat,element); //Perform erosion to get rid of noise.
				medianBlur(whites_mat,whites_mat,3);
//			    GaussianBlur(whites_mat,whites_mat,Size(3,3),1.5,1.5);
				imshow("whites_mat", whites_mat);
				waitKey(1);
				Mat whites_mat_clone=whites_mat.clone(); //Used to visualize the proccess (only for demonstrations etc.)
				/// Parameters for Shi-Tomasi algorithm
				const int MAX_CORNERS_DISTANCE=80; //Max width of goal post
				vector<Point2f> corners;
				double qualityLevel = 0.001; //Relative (to the best found corner in terms of high eigenvalues of the Harris matrix) quality of corner candidate.
				double minDistance = 5; //Min distance between detected corners (minimum num. of pixels between post's corners - i.e goal post width).
				int blockSize = 3;
				bool useHarrisDetector = false; //Use the cornerMinEigenVal() instead of corner harris.
				double k = 0.04;
				int maxCorners = 2000;

				/// Copy the source image
				Mat whites_mat_copy;
				whites_mat_copy = whites_mat.clone();

				/// Apply corner detection
				goodFeaturesToTrack(whites_mat,
					corners,
					maxCorners,
					qualityLevel,
					minDistance,
					Mat(),
					blockSize,
					useHarrisDetector,
					k);

				/// Draw corners detected
			//	cout << "** Number of corners detected: " << corners.size() << endl;
				const int R = 3;
				for (int i = 0; i < corners.size(); i++)
				{
					circle(whites_mat_copy, corners[i], R, Scalar(127, 127,
						127), -1, 8, 0);
				}

				imshow("corners", whites_mat_copy);
				waitKey(1);
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

				vector<vector<Point>> post_candidates;
				//Now for every 2 candidates with relatively the same y-axis value (which indicates that both corners might be part of an upside-down T shape) - crop an image around:
				for(uint i=0;i<corners.size();i++)
				{
					corner_1_x=min(cvRound(corners[i].x),whites_mat.cols-1);
					corner_1_y=min(cvRound(corners[i].y),whites_mat.rows-1);
					for(uint j=0;j<corners.size();j++)
					{
						corner_2_x=min(cvRound(corners[j].x),whites_mat.cols-1);
						corner_2_y=min(cvRound(corners[j].y),whites_mat.rows-1);
						if(corner_1_x-corner_2_x!=0 && abs(corner_1_y-bounding_horizontal_line)<=BOUNDING_HORIZONTAL_LINE_FIX && abs(corner_2_y-bounding_horizontal_line)<=BOUNDING_HORIZONTAL_LINE_FIX ) //Vertical line between edges- can't be edges of post -skip current candidate. also check for candidates only near the bounding horizontal line.
						{

							slope=(corner_1_y-corner_2_y+0.0)/(corner_1_x-corner_2_x);
							//cout<<slope<<endl;
							//For every 2 corners- if they are close enough then check if they are bounding a post of the goal between them
							if((j<i) && abs(slope)<1 && sqrt(pow(corner_1_x-corner_2_x,2)+pow(corner_1_y-corner_2_y,2))<MAX_CORNERS_DISTANCE)
							{


								if(abs(corner_1_x-corner_2_x)>0)
								{
									//Check whether there are white pixels between 2 corners and above POST_WHITE_PIXELS_CHECK_NUM_PIXELS
									const int POST_WHITE_PIXELS_CHECK_NUM_PIXELS=30; //We check that there is a rectangle of height POST_HEIGHT_CHECK_NUM_PIXELS=60 and width abs(corner_1_x-corner_2_x) with mostly white pixels inside -to validate that there's a post:
									const double PERCENTAGE_WHITE_PIXELS_IN_RECT=0.8; //Heuristic - the bounding rectangle must have at least 80% white pixels in it!
									const int POST_FIELD_PIXELS_CHECK_NUM_PIXELS=30; //We check that there is a rectangle of height POST_HEIGHT_CHECK_NUM_PIXELS=60 and width abs(corner_1_x-corner_2_x) with mostly field(green) pixels inside -to validate that there's a post:
									const double PERCENTAGE_FIELD_PIXELS_IN_RECT=0.8 ;//Heuristic - the bounding rectangle must have at least 80% field(green) pixels in it!
									const int POST_NO_WHITE_PIXELS_ASIDE_NUM_PIXELS=20;//We check that a rectangle of width POST_NO_WHITE_PIXELS_ASIDE_NUM_PIXELS to the left and right of the post candidate as no more than 20% white pixels.
									const double PERCENTAGE_NO_WHITE_PIXELS_IN_RECT=0.15; //Heuristic- the bounding rectangle must have at most 20% white pixels in it!
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

											if((max(corner_1_y,corner_2_y)+POST_FIELD_PIXELS_CHECK_NUM_PIXELS<field_mat.rows)) //if enough space below the post candidate to check for field pixels:
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
													 //Check whether to left of the left corner(and the right of the right corner) there are no white pixels (and there should be the field pixels (or other pixels out of field boundary)). This check is mostly needed to prevent half post detection.
													 if(min(corner_1_x,corner_2_x)-POST_NO_WHITE_PIXELS_ASIDE_NUM_PIXELS>0 && max(corner_1_x,corner_2_x)+POST_NO_WHITE_PIXELS_ASIDE_NUM_PIXELS<whites_mat.cols) //If enough space to check to the left and right of the corners:
													 {
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
//															if(percentage_white_pixels<0.3)
//																cout<<"percentage_white_pixels_left"<<percentage_white_pixels<<endl;
															percentage_white_pixels=(count_num_white_pixels_in_rect+0.0)/(aside_post_bounding_rect_mat.rows*aside_post_bounding_rect_mat.cols);
															if(percentage_white_pixels<=PERCENTAGE_NO_WHITE_PIXELS_IN_RECT)
															{
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
//																		if(percentage_white_pixels<0.3)
//																			cout<<"percentage_white_pixels_right"<<percentage_white_pixels<<endl;
																		//****************************************************************
																		if(percentage_white_pixels<=PERCENTAGE_NO_WHITE_PIXELS_IN_RECT)
																		{
																			rectangle(whites_mat_clone,goal_post_candidate_bounding_rect,Scalar(120),2);
																			post_candidates.push_back(vector<Point>());
																			post_candidates[post_candidates.size()-1].push_back(Point(corner_1_x,corner_1_y));
																			post_candidates[post_candidates.size()-1].push_back(Point(corner_2_x,corner_2_y));

						//													imshow("post_candidate",whites_mat(Rect(left,top,width,height)));
						//													waitKey();
																		}
															}

													 }

												}


											}

											//TODO check if there's a T structure
										}
									}

								}
							}
//								if(width>0 && height>0 && idx<3000)
//								{
//									imwrite("post_candidate"+std::to_string(idx)+".png",whites_mat(Rect(left,top,width,height)));
//									idx++;
//								}
						}
					}
				}

				int max_width=0;
				int index_of_max_width_candidate=0;
				int index_of_second_max_width_candidate=-1;
				const int MIN_DIST_BETWEEN_POSTS=200;
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

						 right_post.x=min(gc.m_right_post[0].x,gc.m_right_post[1].x);
						 right_post.y=0;
						 right_post.height=min(gc.m_right_post[0].y,gc.m_right_post[1].y);
						 right_post.width=abs(gc.m_right_post[0].x-gc.m_right_post[1].x);

						 rectangle(frame,left_post,Scalar(255,0,0),2);
						 rectangle(frame,right_post,Scalar(255,0,0),2);
					}
					else //Only one post found:
					{
						 left_post.x=min(gc.m_left_post[0].x,gc.m_left_post[1].x);
						 left_post.y=0;
						 left_post.height=min(gc.m_left_post[0].y,gc.m_left_post[1].y);
						 left_post.width=abs(gc.m_left_post[0].x-gc.m_left_post[1].x);
						 rectangle(frame,left_post,Scalar(255,0,0),2);
					}
				}
				imshow("goal_detector_frame",frame);
				imshow("whites_mat_after_rect",whites_mat_clone);
				imshow("field_mat_after_rect",field_mat_clone);
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
	const double PERCENTAGE_THRESHOLD = 0.97; //Determines how many green pixels we get rid of. the larger the less.
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







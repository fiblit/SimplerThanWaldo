#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include <opencv2/opencv.hpp>
#include "opencv2/xfeatures2d.hpp"


#include <algorithm>
#include <iostream>
#include <ctime>
#include <ratio>
#include <chrono>

/*****Dalton Safe Zone*****/
//Dalton's poorly named stuff
#include "Render.h"
/*****Dalton Safe Zone*****/

 
using namespace cv;
using namespace std;







void MouseCallBackFunc(int event, int x, int y, int flags, void* Current_Pose_Ptr)
{
	if  ( event == EVENT_LBUTTONDOWN )
	{
		Pose_2D* Current_Pose = (Pose_2D*)Current_Pose_Ptr;
		Point Selected_Point;
		if(Current_Pose->Initialized_Parts < 10 || !(Current_Pose->Part_Initialization_Confirmed))
		{
			switch(Current_Pose->Initialized_Parts)
			{
				case 0: 
				{
					Selected_Point.x = x;
					Selected_Point.y = y;
					Current_Pose->Head.Box.Corner_Points.push_back(Selected_Point);
				
					//check if at least 3 points have been initialized
					if(Current_Pose->Head.Box.Corner_Points.size() > 3)
					{
						Current_Pose->Initialized_Parts += 1;

						Current_Pose->Head.Box.R_Rect = minAreaRect(Mat(Current_Pose->Head.Box.Corner_Points));
					}
					cout << "Head Position (" << x << ", " << y << ")" << endl;
					break;
				}
				case 1: 
				{
					Selected_Point.x = x;
					Selected_Point.y = y;
					Current_Pose->Torso.Box.Corner_Points.push_back(Selected_Point);
				
					if(Current_Pose->Torso.Box.Corner_Points.size() > 3)
					{
						Current_Pose->Initialized_Parts += 1;
					
						Current_Pose->Torso.Box.R_Rect = minAreaRect(Mat(Current_Pose->Torso.Box.Corner_Points));
					}
					cout << "Torso Position (" << x << ", " << y << ")" << endl;
					break;
				}
				case 2: 
				{
					Selected_Point.x = x;
					Selected_Point.y = y;
					Current_Pose->Arm_Upper_L.Box.Corner_Points.push_back(Selected_Point);
				
					if(Current_Pose->Arm_Upper_L.Box.Corner_Points.size() > 3)
					{
						Current_Pose->Initialized_Parts += 1;
					
						Current_Pose->Arm_Upper_L.Box.R_Rect = minAreaRect(Mat(Current_Pose->Arm_Upper_L.Box.Corner_Points));
					}
					cout << "Left Upper Arm Position (" << x << ", " << y << ")" << endl;
					break;
				}
				case 3: 
				{
					Selected_Point.x = x;
					Selected_Point.y = y;
					Current_Pose->Arm_Lower_L.Box.Corner_Points.push_back(Selected_Point);
				
					if(Current_Pose->Arm_Lower_L.Box.Corner_Points.size() > 3)
					{
						Current_Pose->Initialized_Parts += 1;
					
						Current_Pose->Arm_Lower_L.Box.R_Rect = minAreaRect(Mat(Current_Pose->Arm_Lower_L.Box.Corner_Points));
					}
					cout << "Left Lower Arm Position (" << x << ", " << y << ")" << endl;
					break;
				}
				case 4: 
				{
				
					Selected_Point.x = x;
					Selected_Point.y = y;
					Current_Pose->Arm_Upper_R.Box.Corner_Points.push_back(Selected_Point);

					if(Current_Pose->Arm_Upper_R.Box.Corner_Points.size() > 3)
					{
						Current_Pose->Initialized_Parts += 1;
					
						Current_Pose->Arm_Upper_R.Box.R_Rect = minAreaRect(Mat(Current_Pose->Arm_Upper_R.Box.Corner_Points));
					}
					cout << "Right Upper Arm Position (" << x << ", " << y << ")" << endl;
					break;
				}
				case 5: 
				{
					Selected_Point.x = x;
					Selected_Point.y = y;
					Current_Pose->Arm_Lower_R.Box.Corner_Points.push_back(Selected_Point);
				
					if(Current_Pose->Arm_Lower_R.Box.Corner_Points.size() > 3)
					{
						Current_Pose->Initialized_Parts += 1;
					
						Current_Pose->Arm_Lower_R.Box.R_Rect = minAreaRect(Mat(Current_Pose->Arm_Lower_R.Box.Corner_Points));
					}
					cout << "Right Lower Arm Position (" << x << ", " << y << ")" << endl;
					break;
				}
				case 6: 
				{
					Selected_Point.x = x;
					Selected_Point.y = y;
					Current_Pose->Leg_Upper_L.Box.Corner_Points.push_back(Selected_Point);
				
					if(Current_Pose->Leg_Upper_L.Box.Corner_Points.size() > 3)
					{
						Current_Pose->Initialized_Parts += 1;
					
						Current_Pose->Leg_Upper_L.Box.R_Rect = minAreaRect(Mat(Current_Pose->Leg_Upper_L.Box.Corner_Points));
					}
					cout << "Left Upper Leg Position (" << x << ", " << y << ")" << endl;
					break;
				}
				case 7: 
				{
					Selected_Point.x = x;
					Selected_Point.y = y;
					Current_Pose->Leg_Lower_L.Box.Corner_Points.push_back(Selected_Point);
				
					if(Current_Pose->Leg_Lower_L.Box.Corner_Points.size() > 3)
					{
						Current_Pose->Initialized_Parts += 1;
					
						Current_Pose->Leg_Lower_L.Box.R_Rect = minAreaRect(Mat(Current_Pose->Leg_Lower_L.Box.Corner_Points));
					}
					cout << "Left Lower Leg Position (" << x << ", " << y << ")" << endl;
					break;
				}
				case 8: 
				{
					Selected_Point.x = x;
					Selected_Point.y = y;
					Current_Pose->Leg_Upper_R.Box.Corner_Points.push_back(Selected_Point);
				
					if(Current_Pose->Leg_Upper_R.Box.Corner_Points.size() > 3)
					{
						Current_Pose->Initialized_Parts += 1;
					
						Current_Pose->Leg_Upper_R.Box.R_Rect = minAreaRect(Mat(Current_Pose->Leg_Upper_R.Box.Corner_Points));
					}
					cout << "Right Upper Leg Position (" << x << ", " << y << ")" << endl;
					break;
				}
				case 9: 
				{
					Selected_Point.x = x;
					Selected_Point.y = y;
					Current_Pose->Leg_Lower_R.Box.Corner_Points.push_back(Selected_Point);
				
					if(Current_Pose->Leg_Lower_R.Box.Corner_Points.size() > 3)
					{
						Current_Pose->Initialized_Parts += 1;
					
						Current_Pose->Leg_Lower_R.Box.R_Rect = minAreaRect(Mat(Current_Pose->Leg_Lower_R.Box.Corner_Points));
					}
					cout << "Right Lower Leg Position (" << x << ", " << y << ")" << endl;
					break;
				}
				default:
				{
					cout << "\nAll part positions have been initialized.  Press a key to continue.\n" << endl;
					break;
				}
			}
		}
		else
		{
			switch(Current_Pose->Initialized_Joints)
			{
				case 0: 
				{
					Selected_Point.x = x;
					Selected_Point.y = y;
					Current_Pose->Head.P_External = Selected_Point;
					Current_Pose->Initialized_Joints++;

					cout << "Head Top Position (" << x << ", " << y << ")" << endl;
					break;
				}
				case 1: 
				{
					Selected_Point.x = x;
					Selected_Point.y = y;
					Current_Pose->Torso.Chest_C = Selected_Point;
					Current_Pose->Initialized_Joints++;

					cout << "Chest Center Position (" << x << ", " << y << ")" << endl;
					break;
				}
				case 2: 
				{
					Selected_Point.x = x;
					Selected_Point.y = y;
					Current_Pose->Torso.Chest_L = Selected_Point;
					Current_Pose->Initialized_Joints++;

					cout << "Left Shoulder Position (" << x << ", " << y << ")" << endl;
					break;
				}
				case 3: 
				{
					Selected_Point.x = x;
					Selected_Point.y = y;
					Current_Pose->Torso.Chest_R = Selected_Point;
					Current_Pose->Initialized_Joints++;

					cout << "Right Shoulder Position (" << x << ", " << y << ")" << endl;
					break;
				}
				case 4: 
				{
			
					Selected_Point.x = x;
					Selected_Point.y = y;
					Current_Pose->Torso.Hip_C = Selected_Point;
					Current_Pose->Initialized_Joints++;

					cout << "Hip Center Position (" << x << ", " << y << ")" << endl;
					break;
				}
				case 5: 
				{
					Selected_Point.x = x;
					Selected_Point.y = y;
					Current_Pose->Torso.Hip_L = Selected_Point;
					Current_Pose->Initialized_Joints++;

					cout << "Left Hip Position (" << x << ", " << y << ")" << endl;
					break;
				}
				case 6: 
				{
					Selected_Point.x = x;
					Selected_Point.y = y;
					Current_Pose->Torso.Hip_R = Selected_Point;
					Current_Pose->Initialized_Joints++;

					cout << "Right Hip Position (" << x << ", " << y << ")" << endl;
					break;
				}
				case 7: 
				{
					Selected_Point.x = x;
					Selected_Point.y = y;
					Current_Pose->Arm_Upper_L.P_External = Selected_Point;
					Current_Pose->Initialized_Joints++;

					cout << "Left Elbow Position (" << x << ", " << y << ")" << endl;
					break;
				}
				case 8: 
				{
					Selected_Point.x = x;
					Selected_Point.y = y;
					Current_Pose->Arm_Lower_L.P_External = Selected_Point;
					Current_Pose->Initialized_Joints++;

					cout << "Left Wrist Position (" << x << ", " << y << ")" << endl;
					break;
				}
				case 9: 
				{
					Selected_Point.x = x;
					Selected_Point.y = y;
					Current_Pose->Arm_Upper_R.P_External = Selected_Point;
					Current_Pose->Initialized_Joints++;

					cout << "Right Elbow Position (" << x << ", " << y << ")" << endl;
					break;
				}
				case 10: 
				{
					Selected_Point.x = x;
					Selected_Point.y = y;
					Current_Pose->Arm_Lower_R.P_External = Selected_Point;
					Current_Pose->Initialized_Joints++;

					cout << "Right Wrist Position (" << x << ", " << y << ")" << endl;
					break;
				}
				case 11: 
				{
					Selected_Point.x = x;
					Selected_Point.y = y;
					Current_Pose->Leg_Upper_L.P_External = Selected_Point;
					Current_Pose->Initialized_Joints++;

					cout << "Left Knee Position (" << x << ", " << y << ")" << endl;
					break;
				}
				case 12: 
				{
					Selected_Point.x = x;
					Selected_Point.y = y;
					Current_Pose->Leg_Lower_L.P_External = Selected_Point;
					Current_Pose->Initialized_Joints++;

					cout << "Left Ankle Position (" << x << ", " << y << ")" << endl;
					break;
				}
				case 13: 
				{
					Selected_Point.x = x;
					Selected_Point.y = y;
					Current_Pose->Leg_Upper_R.P_External = Selected_Point;
					Current_Pose->Initialized_Joints++;

					cout << "Right Knee Position (" << x << ", " << y << ")" << endl;
					break;
				}
				case 14: 
				{
					Selected_Point.x = x;
					Selected_Point.y = y;
					Current_Pose->Leg_Lower_R.P_External = Selected_Point;
					Current_Pose->Initialized_Joints++;

					cout << "Right Ankle Position (" << x << ", " << y << ")" << endl;
					break;
				}
				default:
				{
					cout << "\nAll joint positions have been initialized.  Press a key to continue.\n" << endl;
					break;
				}
			}
		}
	}
}




void Draw_Pose(Mat Frame_In, Mat* Frame_Out, Pose_2D Frame_Pose, int Mode)
{
	
	Frame_In.copyTo(*Frame_Out);
	
	(*Frame_Out).convertTo((*Frame_Out), CV_8UC3);

	switch(Mode)
	{
		case 0: 
		{
			
			Point2f Vertices[4];
			
			Frame_Pose.Head.Box.R_Rect.points(Vertices);
			for (int i = 0; i < 4; i++)
			{
				line(*Frame_Out, Vertices[i], Vertices[(i+1)%4], Scalar(0,255,0), 2);
			}
			
			
			Frame_Pose.Torso.Box.R_Rect.points(Vertices);
			for (int i = 0; i < 4; i++)
			{
				line(*Frame_Out, Vertices[i], Vertices[(i+1)%4], Scalar(0,255,0), 2);
			}			
			
			
			Frame_Pose.Arm_Upper_R.Box.R_Rect.points(Vertices);
			for (int i = 0; i < 4; i++)
			{
				line(*Frame_Out, Vertices[i], Vertices[(i+1)%4], Scalar(0,255,0), 2);
			}
			
			
			Frame_Pose.Arm_Upper_L.Box.R_Rect.points(Vertices);
			for (int i = 0; i < 4; i++)
			{
				line(*Frame_Out, Vertices[i], Vertices[(i+1)%4], Scalar(0,255,0), 2);
			}
			
			
			Frame_Pose.Arm_Lower_R.Box.R_Rect.points(Vertices);
			for (int i = 0; i < 4; i++)
			{
				line(*Frame_Out, Vertices[i], Vertices[(i+1)%4], Scalar(0,255,0), 2);
			}
			
			
			Frame_Pose.Arm_Lower_L.Box.R_Rect.points(Vertices);
			for (int i = 0; i < 4; i++)
			{
				line(*Frame_Out, Vertices[i], Vertices[(i+1)%4], Scalar(0,255,0), 2);
			}
			
			
			Frame_Pose.Leg_Upper_R.Box.R_Rect.points(Vertices);
			for (int i = 0; i < 4; i++)
			{
				line(*Frame_Out, Vertices[i], Vertices[(i+1)%4], Scalar(0,255,0), 2);
			}
			
			
			Frame_Pose.Leg_Upper_L.Box.R_Rect.points(Vertices);
			for (int i = 0; i < 4; i++)
			{
				line(*Frame_Out, Vertices[i], Vertices[(i+1)%4], Scalar(0,255,0), 2);
			}
			
			
			Frame_Pose.Leg_Lower_R.Box.R_Rect.points(Vertices);
			for (int i = 0; i < 4; i++)
			{
				line(*Frame_Out, Vertices[i], Vertices[(i+1)%4], Scalar(0,255,0), 2);
			}
			
			
			Frame_Pose.Leg_Lower_L.Box.R_Rect.points(Vertices);
			for (int i = 0; i < 4; i++)
			{
				line(*Frame_Out, Vertices[i], Vertices[(i+1)%4], Scalar(0,255,0), 2);
			}
			
			break;
		}
		
		case 1:
		{
			//draw torso lines
			line(*Frame_Out, Frame_Pose.Chest_C, Frame_Pose.Chest_L, Scalar(0,0,255), 2);
			line(*Frame_Out, Frame_Pose.Chest_C, Frame_Pose.Chest_R, Scalar(0,0,255), 2);
			line(*Frame_Out, Frame_Pose.Chest_C, Frame_Pose.Hip_C, Scalar(0,0,255), 2);
			line(*Frame_Out, Frame_Pose.Hip_C, Frame_Pose.Hip_L, Scalar(0,0,255), 2);
			line(*Frame_Out, Frame_Pose.Hip_C, Frame_Pose.Hip_R, Scalar(0,0,255), 2);
			
			//draw head line
			line(*Frame_Out, Frame_Pose.Chest_C, Frame_Pose.Head_Top, Scalar(0,0,255), 2);
			
			//draw arm lines
			line(*Frame_Out, Frame_Pose.Chest_L, Frame_Pose.Elbow_L, Scalar(0,0,255), 2);
			line(*Frame_Out, Frame_Pose.Elbow_L, Frame_Pose.Wrist_L, Scalar(0,0,255), 2);
			line(*Frame_Out, Frame_Pose.Chest_R, Frame_Pose.Elbow_R, Scalar(0,0,255), 2);
			line(*Frame_Out, Frame_Pose.Elbow_R, Frame_Pose.Wrist_R, Scalar(0,0,255), 2);
			
			//draw leg lines
			line(*Frame_Out, Frame_Pose.Hip_L, Frame_Pose.Knee_L, Scalar(0,0,255), 2);
			line(*Frame_Out, Frame_Pose.Knee_L, Frame_Pose.Ankle_L, Scalar(0,0,255), 2);
			line(*Frame_Out, Frame_Pose.Hip_R, Frame_Pose.Knee_R, Scalar(0,0,255), 2);
			line(*Frame_Out, Frame_Pose.Knee_R, Frame_Pose.Ankle_R, Scalar(0,0,255), 2);
			
			
			//then draw all of the joints as circles
			circle(*Frame_Out, Frame_Pose.Chest_C, 3, Scalar(255,0,0),-1);
			circle(*Frame_Out, Frame_Pose.Chest_L, 3, Scalar(255,0,0),-1);
			circle(*Frame_Out, Frame_Pose.Chest_R, 3, Scalar(255,0,0),-1);
			
			circle(*Frame_Out, Frame_Pose.Hip_C, 3, Scalar(255,0,0),-1);
			circle(*Frame_Out, Frame_Pose.Hip_L, 3, Scalar(255,0,0),-1);
			circle(*Frame_Out, Frame_Pose.Hip_R, 3, Scalar(255,0,0),-1);
			
			circle(*Frame_Out, Frame_Pose.Head_Top, 3, Scalar(255,0,0),-1);
			
			circle(*Frame_Out, Frame_Pose.Elbow_L, 3, Scalar(255,0,0),-1);
			circle(*Frame_Out, Frame_Pose.Wrist_L, 3, Scalar(255,0,0),-1);
			circle(*Frame_Out, Frame_Pose.Elbow_R, 3, Scalar(255,0,0),-1);
			circle(*Frame_Out, Frame_Pose.Wrist_R, 3, Scalar(255,0,0),-1);
			
			circle(*Frame_Out, Frame_Pose.Knee_L, 3, Scalar(255,0,0),-1);
			circle(*Frame_Out, Frame_Pose.Ankle_L, 3, Scalar(255,0,0),-1);
			circle(*Frame_Out, Frame_Pose.Knee_R, 3, Scalar(255,0,0),-1);
			circle(*Frame_Out, Frame_Pose.Ankle_R, 3, Scalar(255,0,0),-1);
			
			break;
		}
		
		default:
		{
			break;
		}
	}
}




bool Rect_Contains_Keypoint(RotatedRect R_Rect, KeyPoint KP)
{
	Point2f P = KP.pt;
	Point2f Vertices[4];
	R_Rect.points(Vertices);
	vector<Point2f> Contour;
	for(int i = 0; i < 4; i++)
	{
		Contour.push_back(Vertices[i]);	
	}

	Contour.push_back(Vertices[0]);
	
	double Result = pointPolygonTest(Contour, P, false);
	
	if(Result >= 0)
	{
		return true;
	}
	else
	{
		return false;
	}
	
}



void Find_Part_Related_Features(void* Current_Pose_Ptr, vector<KeyPoint>* Frame_Keypoints_SIFT, vector<KeyPoint>* Frame_Keypoints_SURF)
{
	Pose_2D* Current_Pose = (Pose_2D*)Current_Pose_Ptr;
	
	//check for each keypoint which part bounding boxes it is inside of SIFT
	for(int i = 0; i < (*Frame_Keypoints_SIFT).size(); i++)
	{
		if(Rect_Contains_Keypoint(Current_Pose->Head.Box.R_Rect, (*Frame_Keypoints_SIFT)[i]))
		{
			Current_Pose->Head.Part_Keypoints_SIFT.push_back(i);
		}
		
		if(Rect_Contains_Keypoint(Current_Pose->Torso.Box.R_Rect, (*Frame_Keypoints_SIFT)[i]))
		{
			Current_Pose->Torso.Part_Keypoints_SIFT.push_back(i);
		}
		
		if(Rect_Contains_Keypoint(Current_Pose->Arm_Upper_R.Box.R_Rect, (*Frame_Keypoints_SIFT)[i]))
		{
			Current_Pose->Arm_Upper_R.Part_Keypoints_SIFT.push_back(i);
		}
		
		if(Rect_Contains_Keypoint(Current_Pose->Arm_Upper_L.Box.R_Rect, (*Frame_Keypoints_SIFT)[i]))
		{
			Current_Pose->Arm_Upper_L.Part_Keypoints_SIFT.push_back(i);
		}
		
		if(Rect_Contains_Keypoint(Current_Pose->Arm_Lower_R.Box.R_Rect, (*Frame_Keypoints_SIFT)[i]))
		{
			Current_Pose->Arm_Lower_R.Part_Keypoints_SIFT.push_back(i);
		}
		
		if(Rect_Contains_Keypoint(Current_Pose->Arm_Lower_L.Box.R_Rect, (*Frame_Keypoints_SIFT)[i]))
		{
			Current_Pose->Arm_Lower_L.Part_Keypoints_SIFT.push_back(i);
		}
		
		if(Rect_Contains_Keypoint(Current_Pose->Leg_Upper_R.Box.R_Rect, (*Frame_Keypoints_SIFT)[i]))
		{
			Current_Pose->Leg_Upper_R.Part_Keypoints_SIFT.push_back(i);
		}
		
		if(Rect_Contains_Keypoint(Current_Pose->Leg_Upper_L.Box.R_Rect, (*Frame_Keypoints_SIFT)[i]))
		{
			Current_Pose->Leg_Upper_L.Part_Keypoints_SIFT.push_back(i);
		}
		
		if(Rect_Contains_Keypoint(Current_Pose->Leg_Lower_R.Box.R_Rect, (*Frame_Keypoints_SIFT)[i]))
		{
			Current_Pose->Leg_Lower_R.Part_Keypoints_SIFT.push_back(i);
		}
		
		if(Rect_Contains_Keypoint(Current_Pose->Leg_Lower_L.Box.R_Rect, (*Frame_Keypoints_SIFT)[i]))
		{
			Current_Pose->Leg_Lower_L.Part_Keypoints_SIFT.push_back(i);
		}
	}


	//check for each keypoint which part bounding boxes it is inside of SURF
	for(int i = 0; i < (*Frame_Keypoints_SURF).size(); i++)
	{
		if(Rect_Contains_Keypoint(Current_Pose->Head.Box.R_Rect, (*Frame_Keypoints_SURF)[i]))
		{
			Current_Pose->Head.Part_Keypoints_SURF.push_back(i);
		}
		
		if(Rect_Contains_Keypoint(Current_Pose->Torso.Box.R_Rect, (*Frame_Keypoints_SURF)[i]))
		{
			Current_Pose->Torso.Part_Keypoints_SURF.push_back(i);
		}
		
		if(Rect_Contains_Keypoint(Current_Pose->Arm_Upper_R.Box.R_Rect, (*Frame_Keypoints_SURF)[i]))
		{
			Current_Pose->Arm_Upper_R.Part_Keypoints_SURF.push_back(i);
		}
		
		if(Rect_Contains_Keypoint(Current_Pose->Arm_Upper_L.Box.R_Rect, (*Frame_Keypoints_SURF)[i]))
		{
			Current_Pose->Arm_Upper_L.Part_Keypoints_SURF.push_back(i);
		}
		
		if(Rect_Contains_Keypoint(Current_Pose->Arm_Lower_R.Box.R_Rect, (*Frame_Keypoints_SURF)[i]))
		{
			Current_Pose->Arm_Lower_R.Part_Keypoints_SURF.push_back(i);
		}
		
		if(Rect_Contains_Keypoint(Current_Pose->Arm_Lower_L.Box.R_Rect, (*Frame_Keypoints_SURF)[i]))
		{
			Current_Pose->Arm_Lower_L.Part_Keypoints_SURF.push_back(i);
		}
		
		if(Rect_Contains_Keypoint(Current_Pose->Leg_Upper_R.Box.R_Rect, (*Frame_Keypoints_SURF)[i]))
		{
			Current_Pose->Leg_Upper_R.Part_Keypoints_SURF.push_back(i);
		}
		
		if(Rect_Contains_Keypoint(Current_Pose->Leg_Upper_L.Box.R_Rect, (*Frame_Keypoints_SURF)[i]))
		{
			Current_Pose->Leg_Upper_L.Part_Keypoints_SURF.push_back(i);
		}
		
		if(Rect_Contains_Keypoint(Current_Pose->Leg_Lower_R.Box.R_Rect, (*Frame_Keypoints_SURF)[i]))
		{
			Current_Pose->Leg_Lower_R.Part_Keypoints_SURF.push_back(i);
		}
		
		if(Rect_Contains_Keypoint(Current_Pose->Leg_Lower_L.Box.R_Rect, (*Frame_Keypoints_SURF)[i]))
		{
			Current_Pose->Leg_Lower_L.Part_Keypoints_SURF.push_back(i);
		}
	}
}




void Update_Part_Points_and_Boxes(void* Current_Pose_Ptr, vector<DMatch> Matches_SIFT, vector<KeyPoint> Previous_Keypoints_SIFT, vector<KeyPoint> Frame_Keypoints_SIFT, vector<DMatch> Matches_SURF, vector<KeyPoint> Previous_Keypoints_SURF, vector<KeyPoint> Frame_Keypoints_SURF)
{
	Pose_2D* Current_Pose = (Pose_2D*)Current_Pose_Ptr;
	

	//Find new head bounding box
	vector<Point2f> Good_Previous_Point;
	vector<Point2f> Good_Current_Point;
	vector<Point2f> Previous_Corners;
	vector<Point2f> Current_Corners;
	
	vector<Point2f> Previous_Joint;
	vector<Point2f> Current_Joint;
	Point2f Vertices[4];
	Mat H;
	int Previous_Index;
	int Current_Index;


	RotatedRect Suggested_R_Rect;
	float Rect_Width_Ratio, Rect_Height_Ratio;

	Good_Previous_Point.clear();
	Good_Current_Point.clear();
	Previous_Corners.clear();
	Current_Corners.clear();
	Previous_Joint.clear();
	Current_Joint.clear();

	for(int i=0; i < Matches_SIFT.size(); i++)
	{
		Previous_Index = (Matches_SIFT[i]).queryIdx;
		Current_Index = (Matches_SIFT[i]).trainIdx;
		if(find(Current_Pose->Head.Part_Keypoints_SIFT.begin(), Current_Pose->Head.Part_Keypoints_SIFT.end(), Previous_Index) != Current_Pose->Head.Part_Keypoints_SIFT.end())
		{
			if(norm(Mat(Frame_Keypoints_SIFT[Current_Index].pt), Mat(Previous_Keypoints_SIFT[Previous_Index].pt)) < 10)
			{
				Good_Previous_Point.push_back(Previous_Keypoints_SIFT[Previous_Index].pt);
				Good_Current_Point.push_back(Frame_Keypoints_SIFT[Current_Index].pt);
			}
		}
	}

	for(int i=0; i < Matches_SURF.size(); i++)
	{
		Previous_Index = (Matches_SURF[i]).queryIdx;
		Current_Index = (Matches_SURF[i]).trainIdx;
		if(find(Current_Pose->Head.Part_Keypoints_SURF.begin(), Current_Pose->Head.Part_Keypoints_SURF.end(), Previous_Index) != Current_Pose->Head.Part_Keypoints_SURF.end())
		{
			if(norm(Mat(Frame_Keypoints_SURF[Current_Index].pt), Mat(Previous_Keypoints_SURF[Previous_Index].pt)) < 10)
			{
				Good_Previous_Point.push_back(Previous_Keypoints_SURF[Previous_Index].pt);
				Good_Current_Point.push_back(Frame_Keypoints_SURF[Current_Index].pt);
			}
		}
	}

	
	if(Good_Previous_Point.size() > 1)
	{
		H = estimateRigidTransform(Good_Previous_Point, Good_Current_Point, true);
		if(H.rows >= 2 && H.cols >= 2)
		{

			Current_Pose->Head.Box.R_Rect.points(Vertices);
			for(int i = 0; i < 4; i++)
			{
				Previous_Corners.push_back(Vertices[i]);	
			}
			Previous_Joint.push_back(Current_Pose->Head.P_External);

			transform(Previous_Corners, Current_Corners, H);
			transform(Previous_Joint, Current_Joint, H);

			Suggested_R_Rect = minAreaRect(Mat(Current_Corners));

			Rect_Width_Ratio = abs((((float)Current_Pose->Head.Box.R_Rect.size.width / (float)Suggested_R_Rect.size.width) - 1));
			Rect_Height_Ratio = abs((((float)Current_Pose->Head.Box.R_Rect.size.height / (float)Suggested_R_Rect.size.height) - 1));

			if(Rect_Width_Ratio < 0.1 && Rect_Height_Ratio < 0.1)
			{
				Current_Pose->Head.Box.R_Rect = Suggested_R_Rect;
				Current_Pose->Head.P_External = Current_Joint[0];
			}
		
		}
	}
	


	
	//Find new torso bounding box
	Good_Previous_Point.clear();
	Good_Current_Point.clear();
	Previous_Corners.clear();
	Current_Corners.clear();
	Previous_Joint.clear();
	Current_Joint.clear();

	for(int i=0; i < Matches_SIFT.size(); i++)
	{
		Previous_Index = (Matches_SIFT[i]).queryIdx;
		Current_Index = (Matches_SIFT[i]).trainIdx;
		if(find(Current_Pose->Torso.Part_Keypoints_SIFT.begin(), Current_Pose->Torso.Part_Keypoints_SIFT.end(), Previous_Index) != Current_Pose->Torso.Part_Keypoints_SIFT.end())
		{
			if(norm(Mat(Frame_Keypoints_SIFT[Current_Index].pt), Mat(Previous_Keypoints_SIFT[Previous_Index].pt)) < 10)
			{
				Good_Previous_Point.push_back(Previous_Keypoints_SIFT[Previous_Index].pt);
				Good_Current_Point.push_back(Frame_Keypoints_SIFT[Current_Index].pt);
			}
		}
	}


	for(int i=0; i < Matches_SURF.size(); i++)
	{
		Previous_Index = (Matches_SURF[i]).queryIdx;
		Current_Index = (Matches_SURF[i]).trainIdx;
		if(find(Current_Pose->Torso.Part_Keypoints_SURF.begin(), Current_Pose->Torso.Part_Keypoints_SURF.end(), Previous_Index) != Current_Pose->Torso.Part_Keypoints_SURF.end())
		{
			if(norm(Mat(Frame_Keypoints_SURF[Current_Index].pt), Mat(Previous_Keypoints_SURF[Previous_Index].pt)) < 10)
			{
				Good_Previous_Point.push_back(Previous_Keypoints_SURF[Previous_Index].pt);
				Good_Current_Point.push_back(Frame_Keypoints_SURF[Current_Index].pt);
			}
		}
	}

	

	
	if(Good_Previous_Point.size() > 1)
	{
		H = estimateRigidTransform(Good_Previous_Point, Good_Current_Point, true);
		if(H.rows >= 2 && H.cols >= 2)
		{

			Current_Pose->Torso.Box.R_Rect.points(Vertices);
			for(int i = 0; i < 4; i++)
			{
				Previous_Corners.push_back(Vertices[i]);	
			}
			Previous_Joint.push_back(Current_Pose->Torso.Chest_C);
			Previous_Joint.push_back(Current_Pose->Torso.Chest_L);
			Previous_Joint.push_back(Current_Pose->Torso.Chest_R);

			Previous_Joint.push_back(Current_Pose->Torso.Hip_C);
			Previous_Joint.push_back(Current_Pose->Torso.Hip_L);
			Previous_Joint.push_back(Current_Pose->Torso.Hip_R);

			transform(Previous_Corners, Current_Corners, H);
			transform(Previous_Joint, Current_Joint, H);

			Suggested_R_Rect = minAreaRect(Mat(Current_Corners));

			Rect_Width_Ratio = abs((((float)Current_Pose->Torso.Box.R_Rect.size.width / (float)Suggested_R_Rect.size.width) - 1));
			Rect_Height_Ratio = abs((((float)Current_Pose->Torso.Box.R_Rect.size.height / (float)Suggested_R_Rect.size.height) - 1));
			if(Rect_Width_Ratio < 0.1 && Rect_Height_Ratio < 0.1)
			{
				Current_Pose->Torso.Box.R_Rect = Suggested_R_Rect;
				Current_Pose->Torso.Chest_C = Current_Joint[0];
				Current_Pose->Torso.Chest_L = Current_Joint[1];
				Current_Pose->Torso.Chest_R = Current_Joint[2];
				Current_Pose->Torso.Hip_C = Current_Joint[3];
				Current_Pose->Torso.Hip_L = Current_Joint[4];
				Current_Pose->Torso.Hip_R = Current_Joint[5];
			}
		
		}
	}

	


	//Find new right upper arm bounding box
	Good_Previous_Point.clear();
	Good_Current_Point.clear();
	Previous_Corners.clear();
	Current_Corners.clear();
	Previous_Joint.clear();
	Current_Joint.clear();

	for(int i=0; i < Matches_SIFT.size(); i++)
	{
		Previous_Index = (Matches_SIFT[i]).queryIdx;
		Current_Index = (Matches_SIFT[i]).trainIdx;
		if(find(Current_Pose->Arm_Upper_R.Part_Keypoints_SIFT.begin(), Current_Pose->Arm_Upper_R.Part_Keypoints_SIFT.end(), Previous_Index) != Current_Pose->Arm_Upper_R.Part_Keypoints_SIFT.end())
		{
			if(norm(Mat(Frame_Keypoints_SIFT[Current_Index].pt), Mat(Previous_Keypoints_SIFT[Previous_Index].pt)) < 30)
			{
				Good_Previous_Point.push_back(Previous_Keypoints_SIFT[Previous_Index].pt);
				Good_Current_Point.push_back(Frame_Keypoints_SIFT[Current_Index].pt);
			}
		}
	}


	for(int i=0; i < Matches_SURF.size(); i++)
	{
		Previous_Index = (Matches_SURF[i]).queryIdx;
		Current_Index = (Matches_SURF[i]).trainIdx;
		if(find(Current_Pose->Arm_Upper_R.Part_Keypoints_SURF.begin(), Current_Pose->Arm_Upper_R.Part_Keypoints_SURF.end(), Previous_Index) != Current_Pose->Arm_Upper_R.Part_Keypoints_SURF.end())
		{
			if(norm(Mat(Frame_Keypoints_SURF[Current_Index].pt), Mat(Previous_Keypoints_SURF[Previous_Index].pt)) < 30)
			{
				Good_Previous_Point.push_back(Previous_Keypoints_SURF[Previous_Index].pt);
				Good_Current_Point.push_back(Frame_Keypoints_SURF[Current_Index].pt);
			}
		}
	}

	if(Good_Previous_Point.size() > 1)
	{
		H = estimateRigidTransform(Good_Previous_Point, Good_Current_Point, true);

		if(H.rows >= 2 && H.cols >= 2)
		{

			Current_Pose->Arm_Upper_R.Box.R_Rect.points(Vertices);
			for(int i = 0; i < 4; i++)
			{
				Previous_Corners.push_back(Vertices[i]);	
			}
			Previous_Joint.push_back(Current_Pose->Arm_Upper_R.P_External);

			transform(Previous_Corners, Current_Corners, H);
			transform(Previous_Joint, Current_Joint, H);

			Suggested_R_Rect = minAreaRect(Mat(Current_Corners));

			Rect_Width_Ratio = abs((((float)Current_Pose->Arm_Upper_R.Box.R_Rect.size.width / (float)Suggested_R_Rect.size.width) - 1));
			Rect_Height_Ratio = abs((((float)Current_Pose->Arm_Upper_R.Box.R_Rect.size.height / (float)Suggested_R_Rect.size.height) - 1));

			if(Rect_Width_Ratio < 0.5 && Rect_Height_Ratio < 0.5)
			{
				Current_Pose->Arm_Upper_R.Box.R_Rect = Suggested_R_Rect;
				Current_Pose->Arm_Upper_R.P_External = Current_Joint[0];
			}
		
		}
	}


	//Find new left upper arm bounding box
	Good_Previous_Point.clear();
	Good_Current_Point.clear();
	Previous_Corners.clear();
	Current_Corners.clear();
	Previous_Joint.clear();
	Current_Joint.clear();

	for(int i=0; i < Matches_SIFT.size(); i++)
	{
		Previous_Index = (Matches_SIFT[i]).queryIdx;
		Current_Index = (Matches_SIFT[i]).trainIdx;
		if(find(Current_Pose->Arm_Upper_L.Part_Keypoints_SIFT.begin(), Current_Pose->Arm_Upper_L.Part_Keypoints_SIFT.end(), Previous_Index) != Current_Pose->Arm_Upper_L.Part_Keypoints_SIFT.end())
		{
			if(norm(Mat(Frame_Keypoints_SIFT[Current_Index].pt), Mat(Previous_Keypoints_SIFT[Previous_Index].pt)) < 30)
			{
				Good_Previous_Point.push_back(Previous_Keypoints_SIFT[Previous_Index].pt);
				Good_Current_Point.push_back(Frame_Keypoints_SIFT[Current_Index].pt);
			}
		}
	}


	for(int i=0; i < Matches_SURF.size(); i++)
	{
		Previous_Index = (Matches_SURF[i]).queryIdx;
		Current_Index = (Matches_SURF[i]).trainIdx;
		if(find(Current_Pose->Arm_Upper_L.Part_Keypoints_SURF.begin(), Current_Pose->Arm_Upper_L.Part_Keypoints_SURF.end(), Previous_Index) != Current_Pose->Arm_Upper_L.Part_Keypoints_SURF.end())
		{
			if(norm(Mat(Frame_Keypoints_SURF[Current_Index].pt), Mat(Previous_Keypoints_SURF[Previous_Index].pt)) < 30)
			{
				Good_Previous_Point.push_back(Previous_Keypoints_SURF[Previous_Index].pt);
				Good_Current_Point.push_back(Frame_Keypoints_SURF[Current_Index].pt);
			}
		}
	}

	
	if(Good_Previous_Point.size() > 1)
	{
		H = estimateRigidTransform(Good_Previous_Point, Good_Current_Point, true);
		if(H.rows >= 2 && H.cols >= 2)
		{

			Current_Pose->Arm_Upper_L.Box.R_Rect.points(Vertices);
			for(int i = 0; i < 4; i++)
			{
				Previous_Corners.push_back(Vertices[i]);	
			}
			Previous_Joint.push_back(Current_Pose->Arm_Upper_L.P_External);

			transform(Previous_Corners, Current_Corners, H);
			transform(Previous_Joint, Current_Joint, H);

			Suggested_R_Rect = minAreaRect(Mat(Current_Corners));

			Rect_Width_Ratio = abs((((float)Current_Pose->Arm_Upper_L.Box.R_Rect.size.width / (float)Suggested_R_Rect.size.width) - 1));
			Rect_Height_Ratio = abs((((float)Current_Pose->Arm_Upper_L.Box.R_Rect.size.height / (float)Suggested_R_Rect.size.height) - 1));
			if(Rect_Width_Ratio < 0.1 && Rect_Height_Ratio < 0.1)
			{
				Current_Pose->Arm_Upper_L.Box.R_Rect = Suggested_R_Rect;
				Current_Pose->Arm_Upper_L.P_External = Current_Joint[0];
			}
		
		}
	}




	//Find new left lower arm bounding box
	Good_Previous_Point.clear();
	Good_Current_Point.clear();
	Previous_Corners.clear();
	Current_Corners.clear();
	Previous_Joint.clear();
	Current_Joint.clear();

	for(int i=0; i < Matches_SIFT.size(); i++)
	{
		Previous_Index = (Matches_SIFT[i]).queryIdx;
		Current_Index = (Matches_SIFT[i]).trainIdx;
		if(find(Current_Pose->Arm_Lower_L.Part_Keypoints_SIFT.begin(), Current_Pose->Arm_Lower_L.Part_Keypoints_SIFT.end(), Previous_Index) != Current_Pose->Arm_Lower_L.Part_Keypoints_SIFT.end())
		{
			if(norm(Mat(Frame_Keypoints_SIFT[Current_Index].pt), Mat(Previous_Keypoints_SIFT[Previous_Index].pt)) < 30)
			{
				Good_Previous_Point.push_back(Previous_Keypoints_SIFT[Previous_Index].pt);
				Good_Current_Point.push_back(Frame_Keypoints_SIFT[Current_Index].pt);
			}
		}
	}


	for(int i=0; i < Matches_SURF.size(); i++)
	{
		Previous_Index = (Matches_SURF[i]).queryIdx;
		Current_Index = (Matches_SURF[i]).trainIdx;
		if(find(Current_Pose->Arm_Lower_L.Part_Keypoints_SURF.begin(), Current_Pose->Arm_Lower_L.Part_Keypoints_SURF.end(), Previous_Index) != Current_Pose->Arm_Lower_L.Part_Keypoints_SURF.end())
		{
			if(norm(Mat(Frame_Keypoints_SURF[Current_Index].pt), Mat(Previous_Keypoints_SURF[Previous_Index].pt)) < 30)
			{
				Good_Previous_Point.push_back(Previous_Keypoints_SURF[Previous_Index].pt);
				Good_Current_Point.push_back(Frame_Keypoints_SURF[Current_Index].pt);
			}
		}
	}

	
	if(Good_Previous_Point.size() > 1)
	{
		H = estimateRigidTransform(Good_Previous_Point, Good_Current_Point, true);
		if(H.rows >= 2 && H.cols >= 2)
		{

			Current_Pose->Arm_Lower_L.Box.R_Rect.points(Vertices);
			for(int i = 0; i < 4; i++)
			{
				Previous_Corners.push_back(Vertices[i]);	
			}
			Previous_Joint.push_back(Current_Pose->Arm_Lower_L.P_External);

			transform(Previous_Corners, Current_Corners, H);
			transform(Previous_Joint, Current_Joint, H);

			Suggested_R_Rect = minAreaRect(Mat(Current_Corners));

			Rect_Width_Ratio = abs((((float)Current_Pose->Arm_Lower_L.Box.R_Rect.size.width / (float)Suggested_R_Rect.size.width) - 1));
			Rect_Height_Ratio = abs((((float)Current_Pose->Arm_Lower_L.Box.R_Rect.size.height / (float)Suggested_R_Rect.size.height) - 1));
  

			if(Current_Pose->Arm_Lower_L.Box.R_Rect.size.width < 0.1 && Rect_Height_Ratio < 0.1)
			{
				Current_Pose->Arm_Lower_L.Box.R_Rect = Suggested_R_Rect;
				Current_Pose->Arm_Lower_L.P_External = Current_Joint[0];
			}
		
		}
	}




	//Find new right lower arm bounding box
	Good_Previous_Point.clear();
	Good_Current_Point.clear();
	Previous_Corners.clear();
	Current_Corners.clear();
	Previous_Joint.clear();
	Current_Joint.clear();

	for(int i=0; i < Matches_SIFT.size(); i++)
	{
		Previous_Index = (Matches_SIFT[i]).queryIdx;
		Current_Index = (Matches_SIFT[i]).trainIdx;
		if(find(Current_Pose->Arm_Lower_R.Part_Keypoints_SIFT.begin(), Current_Pose->Arm_Lower_R.Part_Keypoints_SIFT.end(), Previous_Index) != Current_Pose->Arm_Lower_R.Part_Keypoints_SIFT.end())
		{
			if(norm(Mat(Frame_Keypoints_SIFT[Current_Index].pt), Mat(Previous_Keypoints_SIFT[Previous_Index].pt)) < 30)
			{
				Good_Previous_Point.push_back(Previous_Keypoints_SIFT[Previous_Index].pt);
				Good_Current_Point.push_back(Frame_Keypoints_SIFT[Current_Index].pt);
			}
		}
	}


	for(int i=0; i < Matches_SURF.size(); i++)
	{
		Previous_Index = (Matches_SURF[i]).queryIdx;
		Current_Index = (Matches_SURF[i]).trainIdx;
		if(find(Current_Pose->Arm_Lower_R.Part_Keypoints_SURF.begin(), Current_Pose->Arm_Lower_R.Part_Keypoints_SURF.end(), Previous_Index) != Current_Pose->Arm_Lower_R.Part_Keypoints_SURF.end())
		{
			if(norm(Mat(Frame_Keypoints_SURF[Current_Index].pt), Mat(Previous_Keypoints_SURF[Previous_Index].pt)) < 30)
			{
				Good_Previous_Point.push_back(Previous_Keypoints_SURF[Previous_Index].pt);
				Good_Current_Point.push_back(Frame_Keypoints_SURF[Current_Index].pt);
			}
		}
	}

	
	if(Good_Previous_Point.size() > 1)
	{
		H = estimateRigidTransform(Good_Previous_Point, Good_Current_Point, true);
		if(H.rows >= 2 && H.cols >= 2)
		{

			Current_Pose->Arm_Lower_R.Box.R_Rect.points(Vertices);
			for(int i = 0; i < 4; i++)
			{
				Previous_Corners.push_back(Vertices[i]);	
			}
			Previous_Joint.push_back(Current_Pose->Arm_Lower_R.P_External);

			transform(Previous_Corners, Current_Corners, H);
			transform(Previous_Joint, Current_Joint, H);

			Suggested_R_Rect = minAreaRect(Mat(Current_Corners));

			Rect_Width_Ratio = abs((((float)Current_Pose->Arm_Lower_R.Box.R_Rect.size.width / (float)Suggested_R_Rect.size.width) - 1));
			Rect_Height_Ratio = abs((((float)Current_Pose->Torso.Box.R_Rect.size.height / (float)Suggested_R_Rect.size.height) - 1));
			if(Rect_Width_Ratio < 0.1 && Rect_Height_Ratio < 0.1)
			{
				Current_Pose->Arm_Lower_R.Box.R_Rect = Suggested_R_Rect;
				Current_Pose->Arm_Lower_R.P_External = Current_Joint[0];
			}
		
		}
	}
		
	







	//Find new right upper leg bounding box
	Good_Previous_Point.clear();
	Good_Current_Point.clear();
	Previous_Corners.clear();
	Current_Corners.clear();
	Previous_Joint.clear();
	Current_Joint.clear();

	for(int i=0; i < Matches_SIFT.size(); i++)
	{
		Previous_Index = (Matches_SIFT[i]).queryIdx;
		Current_Index = (Matches_SIFT[i]).trainIdx;
		if(find(Current_Pose->Leg_Upper_R.Part_Keypoints_SIFT.begin(), Current_Pose->Leg_Upper_R.Part_Keypoints_SIFT.end(), Previous_Index) != Current_Pose->Leg_Upper_R.Part_Keypoints_SIFT.end())
		{
			if(norm(Mat(Frame_Keypoints_SIFT[Current_Index].pt), Mat(Previous_Keypoints_SIFT[Previous_Index].pt)) < 10)
			{
				Good_Previous_Point.push_back(Previous_Keypoints_SIFT[Previous_Index].pt);
				Good_Current_Point.push_back(Frame_Keypoints_SIFT[Current_Index].pt);
			}
		}
	}



	for(int i=0; i < Matches_SURF.size(); i++)
	{
		Previous_Index = (Matches_SURF[i]).queryIdx;
		Current_Index = (Matches_SURF[i]).trainIdx;
		if(find(Current_Pose->Leg_Upper_R.Part_Keypoints_SURF.begin(), Current_Pose->Leg_Upper_R.Part_Keypoints_SURF.end(), Previous_Index) != Current_Pose->Leg_Upper_R.Part_Keypoints_SURF.end())
		{
			if(norm(Mat(Frame_Keypoints_SURF[Current_Index].pt), Mat(Previous_Keypoints_SURF[Previous_Index].pt)) < 10)
			{
				Good_Previous_Point.push_back(Previous_Keypoints_SURF[Previous_Index].pt);
				Good_Current_Point.push_back(Frame_Keypoints_SURF[Current_Index].pt);
			}
		}
	}

	
	if(Good_Previous_Point.size() > 1)
	{
		H = estimateRigidTransform(Good_Previous_Point, Good_Current_Point, true);
		if(H.rows >= 2 && H.cols >= 2)
		{

			Current_Pose->Leg_Upper_R.Box.R_Rect.points(Vertices);
			for(int i = 0; i < 4; i++)
			{
				Previous_Corners.push_back(Vertices[i]);	
			}
			Previous_Joint.push_back(Current_Pose->Leg_Upper_R.P_External);

			transform(Previous_Corners, Current_Corners, H);
			transform(Previous_Joint, Current_Joint, H);

			Suggested_R_Rect = minAreaRect(Mat(Current_Corners));

			Rect_Width_Ratio = abs((((float)Current_Pose->Leg_Upper_R.Box.R_Rect.size.width / (float)Suggested_R_Rect.size.width) - 1));
			Rect_Height_Ratio = abs((((float)Current_Pose->Leg_Upper_R.Box.R_Rect.size.height / (float)Suggested_R_Rect.size.height) - 1));
			if(Rect_Width_Ratio < 0.1 && Rect_Height_Ratio < 0.1)
			{
				Current_Pose->Leg_Upper_R.Box.R_Rect = Suggested_R_Rect;
				Current_Pose->Leg_Upper_R.P_External = Current_Joint[0];
			}
		
		}
	}
	

	
	//Find new left upper leg bounding box
	Good_Previous_Point.clear();
	Good_Current_Point.clear();
	Previous_Corners.clear();
	Current_Corners.clear();
	Previous_Joint.clear();
	Current_Joint.clear();

	for(int i=0; i < Matches_SIFT.size(); i++)
	{
		Previous_Index = (Matches_SIFT[i]).queryIdx;
		Current_Index = (Matches_SIFT[i]).trainIdx;
		if(find(Current_Pose->Leg_Upper_L.Part_Keypoints_SIFT.begin(), Current_Pose->Leg_Upper_L.Part_Keypoints_SIFT.end(), Previous_Index) != Current_Pose->Leg_Upper_L.Part_Keypoints_SIFT.end())
		{
			if(norm(Mat(Frame_Keypoints_SIFT[Current_Index].pt), Mat(Previous_Keypoints_SIFT[Previous_Index].pt)) < 10)
			{
				Good_Previous_Point.push_back(Previous_Keypoints_SIFT[Previous_Index].pt);
				Good_Current_Point.push_back(Frame_Keypoints_SIFT[Current_Index].pt);
			}
		}
	}


	for(int i=0; i < Matches_SURF.size(); i++)
	{
		Previous_Index = (Matches_SURF[i]).queryIdx;
		Current_Index = (Matches_SURF[i]).trainIdx;
		if(find(Current_Pose->Leg_Upper_L.Part_Keypoints_SURF.begin(), Current_Pose->Leg_Upper_L.Part_Keypoints_SURF.end(), Previous_Index) != Current_Pose->Leg_Upper_L.Part_Keypoints_SURF.end())
		{
			if(norm(Mat(Frame_Keypoints_SURF[Current_Index].pt), Mat(Previous_Keypoints_SURF[Previous_Index].pt)) < 10)
			{
				Good_Previous_Point.push_back(Previous_Keypoints_SURF[Previous_Index].pt);
				Good_Current_Point.push_back(Frame_Keypoints_SURF[Current_Index].pt);
			}
		}
	}

	
	if(Good_Previous_Point.size() > 1)
	{

		H = estimateRigidTransform(Good_Previous_Point, Good_Current_Point, true);
		if(H.rows >= 2 && H.cols >= 2)
		{

			Current_Pose->Leg_Upper_L.Box.R_Rect.points(Vertices);
			for(int i = 0; i < 4; i++)
			{
				Previous_Corners.push_back(Vertices[i]);	
			}
			Previous_Joint.push_back(Current_Pose->Leg_Upper_L.P_External);

			transform(Previous_Corners, Current_Corners, H);
			transform(Previous_Joint, Current_Joint, H);

			Suggested_R_Rect = minAreaRect(Mat(Current_Corners));

			Rect_Width_Ratio = abs((((float)Current_Pose->Leg_Upper_L.Box.R_Rect.size.width / (float)Suggested_R_Rect.size.width) - 1));
			Rect_Height_Ratio = abs((((float)Current_Pose->Leg_Upper_L.Box.R_Rect.size.height / (float)Suggested_R_Rect.size.height) - 1));
			if(Rect_Width_Ratio < 0.1 && Rect_Height_Ratio < 0.1)
			{
				Current_Pose->Leg_Upper_L.Box.R_Rect = Suggested_R_Rect;
				Current_Pose->Leg_Upper_L.P_External = Current_Joint[0];
			}
		}
		
	}




	//Find new left lower leg bounding box
	Good_Previous_Point.clear();
	Good_Current_Point.clear();
	Previous_Corners.clear();
	Current_Corners.clear();
	Previous_Joint.clear();
	Current_Joint.clear();

	for(int i=0; i < Matches_SIFT.size(); i++)
	{
		Previous_Index = (Matches_SIFT[i]).queryIdx;
		Current_Index = (Matches_SIFT[i]).trainIdx;
		if(find(Current_Pose->Leg_Lower_L.Part_Keypoints_SIFT.begin(), Current_Pose->Leg_Lower_L.Part_Keypoints_SIFT.end(), Previous_Index) != Current_Pose->Leg_Lower_L.Part_Keypoints_SIFT.end())
		{
			if(norm(Mat(Frame_Keypoints_SIFT[Current_Index].pt), Mat(Previous_Keypoints_SIFT[Previous_Index].pt)) < 10)
			{
				Good_Previous_Point.push_back(Previous_Keypoints_SIFT[Previous_Index].pt);
				Good_Current_Point.push_back(Frame_Keypoints_SIFT[Current_Index].pt);
			}
		}
	}


	for(int i=0; i < Matches_SURF.size(); i++)
	{
		Previous_Index = (Matches_SURF[i]).queryIdx;
		Current_Index = (Matches_SURF[i]).trainIdx;
		if(find(Current_Pose->Leg_Lower_L.Part_Keypoints_SURF.begin(), Current_Pose->Leg_Lower_L.Part_Keypoints_SURF.end(), Previous_Index) != Current_Pose->Leg_Lower_L.Part_Keypoints_SURF.end())
		{
			if(norm(Mat(Frame_Keypoints_SURF[Current_Index].pt), Mat(Previous_Keypoints_SURF[Previous_Index].pt)) < 10)
			{
				Good_Previous_Point.push_back(Previous_Keypoints_SURF[Previous_Index].pt);
				Good_Current_Point.push_back(Frame_Keypoints_SURF[Current_Index].pt);
			}
		}
	}

	
	if(Good_Previous_Point.size() > 1)
	{

		H = estimateRigidTransform(Good_Previous_Point, Good_Current_Point, true);
		if(H.rows >= 2 && H.cols >= 2)
		{
			Current_Pose->Leg_Lower_L.Box.R_Rect.points(Vertices);
			for(int i = 0; i < 4; i++)
			{
				Previous_Corners.push_back(Vertices[i]);	
			}
			Previous_Joint.push_back(Current_Pose->Leg_Lower_L.P_External);

			transform(Previous_Corners, Current_Corners, H);
			transform(Previous_Joint, Current_Joint, H);

			Suggested_R_Rect = minAreaRect(Mat(Current_Corners));

			Rect_Width_Ratio = abs((((float)Current_Pose->Leg_Lower_L.Box.R_Rect.size.width / (float)Suggested_R_Rect.size.width) - 1));
			Rect_Height_Ratio = abs((((float)Current_Pose->Leg_Lower_L.Box.R_Rect.size.height / (float)Suggested_R_Rect.size.height) - 1));
			if(Rect_Width_Ratio < 0.1 && Rect_Height_Ratio < 0.1)
			{
				Current_Pose->Leg_Lower_L.Box.R_Rect = Suggested_R_Rect;
				Current_Pose->Leg_Lower_L.P_External = Current_Joint[0];
			}
		}
	}




	//Find new right lower leg bounding box
	Good_Previous_Point.clear();
	Good_Current_Point.clear();
	Previous_Corners.clear();
	Current_Corners.clear();
	Previous_Joint.clear();
	Current_Joint.clear();

	for(int i=0; i < Matches_SIFT.size(); i++)
	{
		Previous_Index = (Matches_SIFT[i]).queryIdx;
		Current_Index = (Matches_SIFT[i]).trainIdx;
		if(find(Current_Pose->Leg_Lower_R.Part_Keypoints_SIFT.begin(), Current_Pose->Leg_Lower_R.Part_Keypoints_SIFT.end(), Previous_Index) != Current_Pose->Leg_Lower_R.Part_Keypoints_SIFT.end())
		{
			if(norm(Mat(Frame_Keypoints_SIFT[Current_Index].pt), Mat(Previous_Keypoints_SIFT[Previous_Index].pt)) < 10)
			{
				Good_Previous_Point.push_back(Previous_Keypoints_SIFT[Previous_Index].pt);
				Good_Current_Point.push_back(Frame_Keypoints_SIFT[Current_Index].pt);
			}
		}
	}


	for(int i=0; i < Matches_SURF.size(); i++)
	{
		Previous_Index = (Matches_SURF[i]).queryIdx;
		Current_Index = (Matches_SURF[i]).trainIdx;
		if(find(Current_Pose->Leg_Lower_R.Part_Keypoints_SURF.begin(), Current_Pose->Leg_Lower_R.Part_Keypoints_SURF.end(), Previous_Index) != Current_Pose->Leg_Lower_R.Part_Keypoints_SURF.end())
		{
			if(norm(Mat(Frame_Keypoints_SURF[Current_Index].pt), Mat(Previous_Keypoints_SURF[Previous_Index].pt)) < 10)
			{
				Good_Previous_Point.push_back(Previous_Keypoints_SURF[Previous_Index].pt);
				Good_Current_Point.push_back(Frame_Keypoints_SURF[Current_Index].pt);
			}
		}
	}

	
	if(Good_Previous_Point.size() > 1)
	{

		H = estimateRigidTransform(Good_Previous_Point, Good_Current_Point, true);
		if(H.rows >= 2 && H.cols >= 2)
		{
			Current_Pose->Leg_Lower_R.Box.R_Rect.points(Vertices);
			for(int i = 0; i < 4; i++)
			{
				Previous_Corners.push_back(Vertices[i]);	
			}
			Previous_Joint.push_back(Current_Pose->Leg_Lower_R.P_External);

			transform(Previous_Corners, Current_Corners, H);
			transform(Previous_Joint, Current_Joint, H);

			Suggested_R_Rect = minAreaRect(Mat(Current_Corners));

			Rect_Width_Ratio = abs((((float)Current_Pose->Leg_Lower_R.Box.R_Rect.size.width / (float)Suggested_R_Rect.size.width) - 1));
			Rect_Height_Ratio = abs((((float)Current_Pose->Leg_Lower_R.Box.R_Rect.size.height / (float)Suggested_R_Rect.size.height) - 1));
			if(Rect_Width_Ratio < 0.1 && Rect_Height_Ratio < 0.1)
			{
				Current_Pose->Leg_Lower_R.Box.R_Rect = Suggested_R_Rect;
				Current_Pose->Leg_Lower_R.P_External = Current_Joint[0];
			}
		}
		
	}
	
	
}



void Transfer_Points_Up_Structure(void* Current_Pose_Ptr)
{
	Pose_2D* Current_Pose = (Pose_2D*)Current_Pose_Ptr;
	Current_Pose->Chest_C = Current_Pose->Torso.Chest_C;
	Current_Pose->Chest_L = Current_Pose->Torso.Chest_L;
	Current_Pose->Chest_R = Current_Pose->Torso.Chest_R;
	
	Current_Pose->Hip_C = Current_Pose->Torso.Hip_C;
	Current_Pose->Hip_L = Current_Pose->Torso.Hip_L;
	Current_Pose->Hip_R = Current_Pose->Torso.Hip_R;

	Current_Pose->Head_Top = Current_Pose->Head.P_External;
	
	
	Current_Pose->Elbow_L = Current_Pose->Arm_Upper_L.P_External;
	Current_Pose->Elbow_R = Current_Pose->Arm_Upper_R.P_External;
	
	Current_Pose->Wrist_L = Current_Pose->Arm_Lower_L.P_External;
	Current_Pose->Wrist_R = Current_Pose->Arm_Lower_R.P_External;
	
	Current_Pose->Knee_L = Current_Pose->Leg_Upper_L.P_External;
	Current_Pose->Knee_R = Current_Pose->Leg_Upper_R.P_External;
	
	Current_Pose->Ankle_L = Current_Pose->Leg_Lower_L.P_External;
	Current_Pose->Ankle_R = Current_Pose->Leg_Lower_R.P_External;	
}


int Find_Size_Of_Mask()
{

	
	return 0;
}


int main(int argc, char* argv[])
{

	/*****Dalton Safe Zone*****/
	//extract files
	Results * r_3d = initialize_parameters();
	/*****Dalton Safe Zone*****/


	////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//------------------------------------ Variable Declaration For Algorithm-------------------------------------//
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	//General Variables
	bool bSuccess;
	
	int X_Index;
	int Y_Index;
	
	VideoCapture Input_Video;
	VideoWriter Output_Video;
	
	const string Output_Video_File_Name = "Result.avi";
	int ex = static_cast<int>(Input_Video.get(CV_CAP_PROP_FOURCC));
	

	
	//Pose Output Variables
	Pose_2D Current_Pose;
	Current_Pose.Initialized_Parts = 0;
	Current_Pose.Initialized_Joints = 0;
	Current_Pose.Part_Initialization_Confirmed = false;
	
	Mat First_Frame;
	Mat Frame_With_Pose;
	Mat Output_Image;
	
	namedWindow("Detected 2D Pose",CV_WINDOW_AUTOSIZE); 
	//set mouse callback for assigning initial part positions
	setMouseCallback("Detected 2D Pose", MouseCallBackFunc, &Current_Pose);
	
	
	
	//Variables for tracking time of program operation
	chrono::high_resolution_clock::time_point Time_Start = chrono::high_resolution_clock::now();
	chrono::high_resolution_clock::time_point Time_End = chrono::high_resolution_clock::now();
	chrono::duration<double> Time_Span = chrono::duration_cast<chrono::duration<double>>(Time_End - Time_Start);

	double Frame_Rate_Buffer [32];
	double Average_Frame_Rate = 0;
	int Frame_Buffer_Index = -1;
	
	
	
	//Variables for conducting motion detection
	Mat Previous_Frame, Current_Frame, Next_Frame;
	Mat Previous_Frame_Gray_Blurred, Current_Frame_Gray_Blurred, Next_Frame_Gray_Blurred;
	
	Mat Absdiff_Previous_Next, Absdiff_Current_Next;
	Mat BW_And_Result;
	
	
	
	
	//variables for bounding box
	HOGDescriptor hog;
	hog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());
	
	vector<Rect> found, found_filtered;
	size_t BB_found_rect_index_1, BB_found_rect_index_2;
	Rect BB_rect;

	
	
	//Variables for extracting pixels which correspond to people inside of a bounding box
	Mat Grab_Cut_Mask;
	Mat BgdModel, FgdModel;
	Mat Grab_Cut_Result_Mask;

	int Dilation_Erosion_Size = 1;
	int Dilation_Erosion_Iterations = 3;

	Mat Dilation_Erosion_Element = getStructuringElement(MORPH_RECT,
							     Size(2 * Dilation_Erosion_Size + 1, 2 * Dilation_Erosion_Size + 1),
							     Point(Dilation_Erosion_Size, Dilation_Erosion_Size));
	

	
	Mat Combined_Mask(1280, 720, CV_8UC1);
	
	Mat Pose_Mask(Size(1280, 720), CV_8UC1);
	

	//Variables for Part feature detector with SIFT and SURF
	Ptr<Feature2D> SIFT_Detector = xfeatures2d::SIFT::create();
	Ptr<Feature2D> SURF_Detector = xfeatures2d::SURF::create();

	vector<KeyPoint> Frame_Keypoints_SIFT;  
	vector<KeyPoint> Previous_Keypoints_SIFT;
	vector<KeyPoint> Frame_Keypoints_SURF;  
	vector<KeyPoint> Previous_Keypoints_SURF;

	Mat Frame_Descriptors_SIFT, Previous_Descriptors_SIFT, Frame_Descriptors_SURF, Previous_Descriptors_SURF;
	
	BFMatcher Matcher(NORM_L2, true);
  	vector<DMatch> Matches_SIFT;
	vector<DMatch> Matches_SURF;
	
	
	

	
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//------------------------------------------ Program Initialization ------------------------------------------//
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	
	
	switch(argc)
	{
		case 1:
		{
			Input_Video.open(0); // open the video camera no. 0
			if (!Input_Video.isOpened())  // if not success, exit program
			{
				cout << "Cannot open camera device 0" << endl;
				return -1;
			}
			break;
		}
		
		case 3: 
		{
			string argv1 = argv[1];
			if(argv1.compare("VID") == 0)
			{
				Input_Video.open(argv[2]); // open the camera with provided device number
				if (!Input_Video.isOpened())  // if not success, exit program
				{
					cout << "Cannot open the provided video file" << endl;
					return -1;
				}
				break;
			}
			else if(argv1.compare("CAM") == 0)
			{
				Input_Video.open(atoi(argv[2])); // open the video at file location
				if (!Input_Video.isOpened())  // if not success, exit program
				{
					cout << "Cannot open the provided camera" << endl;
					return -1;
				}
				break;	
			}
			else
			{
				cout << "Invalid input parameter type.  Should be either VID or CAM." << endl;
				return -1;
				break;
			}
		}
		default:
		{
			Input_Video.open(0); // open the video camera no. 0
			if (!Input_Video.isOpened())  // if not success, exit program
			{
				cout << "Cannot open camera device 0" << endl;
				return -1;
			}
			break;
		}
	}

			  
	Output_Video.open(Output_Video_File_Name, ex, Input_Video.get(CV_CAP_PROP_FPS), Size(1280, 720), true);
	
	if (!Output_Video.isOpened())
	{
		cout  << "Could not open the output video for writing.\n" << endl;
		return -1;
	}
	
	double dWidth = Input_Video.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
	double dHeight = Input_Video.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video

	cout << "Original Frame size : " << dWidth << " x " << dHeight << endl;
	
	
	bSuccess = Input_Video.read(First_Frame); // read a new frame from video
	
	if (!bSuccess) //if not success, break loop
	{
		 cout << "Cannot read a frame from video or camera stream" << endl;
		 return -1;
	}
	
	First_Frame.convertTo(First_Frame, CV_8U);
	resize(First_Frame, First_Frame, Size(1280, 720), 0, 0, INTER_NEAREST);


	//show the frame



	imshow("Detected 2D Pose", First_Frame); 
	
	// Wait until user press some key after all bounding boxes have been clicked
	
	cout << "Please click four points around each of the following locations to define the bounding box around each part.  Please click around parts in the listed order." << endl;
	cout << "\nHead\nTorso\nLeft Upper Arm\nLeft Lower Arm\nRight Upper Arm\nRight Lower Arm\nLeft Upper Leg\nLeft Lower Leg\nRight Upper Leg\nRight Lower Leg\n"<< endl;
	
	while(Current_Pose.Initialized_Parts < 10)
	{
		waitKey(0);
		if(Current_Pose.Initialized_Parts < 10)
		{
			cout << "Please select remaining " << 10 - Current_Pose.Initialized_Parts << " body part locations." << endl;
		}
	}
	
	Current_Pose.Part_Initialization_Confirmed = true;
	cout << "\nAll part positions have been initialized.  Press a key to continue.\n" << endl;
	waitKey(0);
	// Wait until user press some key after all joints have been clicked
	
	cout << "\n\nPlease click the location of the following joints in the following order." << endl;
	cout << "\nTop of Head\nCenter of Chest\nLeft Shoulder\nRight Shoulder\nHip Center\nLeft side of Hip\nRight side of Hip\nLeft Elbow\nLeft Wrist\nRight Elbow\nRight Wrist\nLeft Knee\nLeft Ankle\nRight Knee\nRight Ankle\n"<< endl;
	
	while(Current_Pose.Initialized_Joints < 15)
	{
		waitKey(0);
		if(Current_Pose.Initialized_Joints < 15)
		{
			cout << "Please select remaining " << 15 - Current_Pose.Initialized_Joints << " joint locations." << endl;
		}
	}
	
	//Find all joints in the image based on the bounding boxes defined
	Transfer_Points_Up_Structure(&Current_Pose);
	
	//Detect KeyPoints in first frame
	SIFT_Detector->detect(First_Frame, Frame_Keypoints_SIFT);
	SIFT_Detector->compute(First_Frame, Frame_Keypoints_SIFT, Frame_Descriptors_SIFT);

	SURF_Detector->detect(First_Frame, Frame_Keypoints_SURF);
	SURF_Detector->compute(First_Frame, Frame_Keypoints_SURF, Frame_Descriptors_SURF);
	
	//detirmine which features lie inside which part bounding box
	Current_Pose.Head.Part_Keypoints_SIFT.clear();
	Current_Pose.Torso.Part_Keypoints_SIFT.clear();
	Current_Pose.Arm_Upper_R.Part_Keypoints_SIFT.clear();
	Current_Pose.Arm_Upper_L.Part_Keypoints_SIFT.clear();
	Current_Pose.Arm_Lower_R.Part_Keypoints_SIFT.clear();
	Current_Pose.Arm_Lower_L.Part_Keypoints_SIFT.clear();
	Current_Pose.Leg_Upper_R.Part_Keypoints_SIFT.clear();
	Current_Pose.Leg_Upper_L.Part_Keypoints_SIFT.clear();
	Current_Pose.Leg_Lower_R.Part_Keypoints_SIFT.clear();
	Current_Pose.Leg_Lower_L.Part_Keypoints_SIFT.clear();


	Current_Pose.Head.Part_Keypoints_SURF.clear();
	Current_Pose.Torso.Part_Keypoints_SURF.clear();
	Current_Pose.Arm_Upper_R.Part_Keypoints_SURF.clear();
	Current_Pose.Arm_Upper_L.Part_Keypoints_SURF.clear();
	Current_Pose.Arm_Lower_R.Part_Keypoints_SURF.clear();
	Current_Pose.Arm_Lower_L.Part_Keypoints_SURF.clear();
	Current_Pose.Leg_Upper_R.Part_Keypoints_SURF.clear();
	Current_Pose.Leg_Upper_L.Part_Keypoints_SURF.clear();
	Current_Pose.Leg_Lower_R.Part_Keypoints_SURF.clear();
	Current_Pose.Leg_Lower_L.Part_Keypoints_SURF.clear();



	Find_Part_Related_Features(&Current_Pose, &Frame_Keypoints_SIFT, &Frame_Keypoints_SURF);
	
	
	
	Draw_Pose(First_Frame, &Frame_With_Pose, Current_Pose, 0);
	Draw_Pose(Frame_With_Pose, &Frame_With_Pose, Current_Pose, 1);

	imshow("Detected 2D Pose", Frame_With_Pose); //show the frame

	cout << "\nInitial pose shown in initial frame.  Press any key to begin tracking through remainder of video.\n" << endl;
	
	waitKey(0);
	
	
	
	
	
	//Set up initial loaded images and related data	for infinite loop
	
	First_Frame.copyTo(Current_Frame);
	
	GaussianBlur(Current_Frame, Current_Frame_Gray_Blurred, Size(5,5), 0, 0, BORDER_DEFAULT);
	
	bSuccess = Input_Video.read(Next_Frame); // read a new frame from video
		
	if (!bSuccess) //if not success, return 0
	{
		 cout << "\nCannot read a frame from video stream" << endl;
		 return 0;
	}
	
	Next_Frame.convertTo(Next_Frame, CV_8U);
	resize(Next_Frame, Next_Frame, Size(1280, 720), 0, 0, INTER_NEAREST);

	
	GaussianBlur(Next_Frame, Next_Frame_Gray_Blurred, Size(5,5), 0, 0, BORDER_DEFAULT);
	
	

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//--------------------------------------------- Program Main Loop --------------------------------------------//
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	
	while (1)
	{
		//start execution timer
		Time_Start = chrono::high_resolution_clock::now();
		
		//shuffle previous, current, and next frames around and their associated derived frames
		Current_Frame.copyTo(Previous_Frame);
		Next_Frame.copyTo(Current_Frame);

		Current_Frame_Gray_Blurred.copyTo(Previous_Frame_Gray_Blurred);
		Next_Frame_Gray_Blurred.copyTo(Current_Frame_Gray_Blurred);
		
		//get the new next image frame
		bSuccess = Input_Video.read(Next_Frame); // read a new frame from video
		
		if (!bSuccess) //if not success, break loop
		{
		     cout << "\nCannot read a frame from video stream" << endl;
		     break;
		}
		
		Next_Frame.convertTo(Next_Frame, CV_8U);
		resize(Next_Frame, Next_Frame, Size(1280, 720), 0, 0, INTER_NEAREST);
		
		GaussianBlur(Next_Frame, Next_Frame_Gray_Blurred, Size(5,5), 0, 0, BORDER_DEFAULT);


		//conduct analysis to find matched SIFT and SURF features
		
		//shuffle current and previous keypoints
		Previous_Keypoints_SIFT.clear();
		Previous_Keypoints_SURF.clear();
		for(int i = 0; i < Frame_Keypoints_SIFT.size(); i++)
		{
			Previous_Keypoints_SIFT.push_back(Frame_Keypoints_SIFT[i]);
		}
		for(int i = 0; i < Frame_Keypoints_SURF.size(); i++)
		{
			Previous_Keypoints_SURF.push_back(Frame_Keypoints_SURF[i]);
		}
		Frame_Keypoints_SURF.clear();
		
		SIFT_Detector->detect(Current_Frame, Frame_Keypoints_SIFT);
		SIFT_Detector->compute(Current_Frame, Frame_Keypoints_SIFT, Frame_Descriptors_SIFT);
		SIFT_Detector->compute(Previous_Frame, Previous_Keypoints_SIFT, Previous_Descriptors_SIFT);

		SURF_Detector->detect(Current_Frame, Frame_Keypoints_SURF);
		SURF_Detector->compute(Current_Frame, Frame_Keypoints_SURF, Frame_Descriptors_SURF);
		SURF_Detector->compute(Previous_Frame, Previous_Keypoints_SURF, Previous_Descriptors_SURF);

		Matcher.match(Previous_Descriptors_SIFT, Frame_Descriptors_SIFT, Matches_SIFT);
		Matcher.match(Previous_Descriptors_SURF, Frame_Descriptors_SURF, Matches_SURF);



		

		//matches that are close to each other in the image
		vector<DMatch> Best_Image_Distance_Matches;
		
		/*		
		for(int i=0; i < Matches_SIFT.size(); i++)
		{
			int Previous_Index = (Matches_SIFT[i]).queryIdx;
			int Current_Index = (Matches_SIFT[i]).trainIdx;
			if(norm(Mat(Frame_Keypoints_SIFT[Current_Index].pt), Mat(Previous_Keypoints_SIFT[Previous_Index].pt)) < 10)
			{
				Best_Image_Distance_Matches.push_back(Matches_SIFT[i]);
			}
		}
		*/


		
		
		//Update bounding boxes using SIFT features
		Update_Part_Points_and_Boxes(&Current_Pose, Matches_SIFT, Previous_Keypoints_SIFT, Frame_Keypoints_SIFT, Matches_SURF, Previous_Keypoints_SURF, Frame_Keypoints_SURF);
		
		
		//Find all joints in the image based on the bounding boxes defined
		Transfer_Points_Up_Structure(&Current_Pose);

		//detirmine which features lie inside which part bounding box
		Current_Pose.Head.Part_Keypoints_SIFT.clear();
		Current_Pose.Torso.Part_Keypoints_SIFT.clear();
		Current_Pose.Arm_Upper_R.Part_Keypoints_SIFT.clear();
		Current_Pose.Arm_Upper_L.Part_Keypoints_SIFT.clear();
		Current_Pose.Arm_Lower_R.Part_Keypoints_SIFT.clear();
		Current_Pose.Arm_Lower_L.Part_Keypoints_SIFT.clear();
		Current_Pose.Leg_Upper_R.Part_Keypoints_SIFT.clear();
		Current_Pose.Leg_Upper_L.Part_Keypoints_SIFT.clear();
		Current_Pose.Leg_Lower_R.Part_Keypoints_SIFT.clear();
		Current_Pose.Leg_Lower_L.Part_Keypoints_SIFT.clear();


		Current_Pose.Head.Part_Keypoints_SURF.clear();
		Current_Pose.Torso.Part_Keypoints_SURF.clear();
		Current_Pose.Arm_Upper_R.Part_Keypoints_SURF.clear();
		Current_Pose.Arm_Upper_L.Part_Keypoints_SURF.clear();
		Current_Pose.Arm_Lower_R.Part_Keypoints_SURF.clear();
		Current_Pose.Arm_Lower_L.Part_Keypoints_SURF.clear();
		Current_Pose.Leg_Upper_R.Part_Keypoints_SURF.clear();
		Current_Pose.Leg_Upper_L.Part_Keypoints_SURF.clear();
		Current_Pose.Leg_Lower_R.Part_Keypoints_SURF.clear();
		Current_Pose.Leg_Lower_L.Part_Keypoints_SURF.clear();
		
		Find_Part_Related_Features(&Current_Pose, &Frame_Keypoints_SIFT, &Frame_Keypoints_SURF);

		
		//Next do analysis on the frames to detirmine which portions are part of moving objects

		//Extract the initial bounding box estimate using the HOG transform
		
        	hog.detectMultiScale(Current_Frame, found, 0, Size(8,8), Size(32,32), 1.05, 2);
		
		//loop through found bounding boxes and check to see if the bounding box should be included in the final list of found individuals
		for (BB_found_rect_index_1 = 0; BB_found_rect_index_1 < found.size(); BB_found_rect_index_1++)
		{
			BB_rect = found[BB_found_rect_index_1];
			//check if the bounding box matches another bounding box in the list, if it does, then do not add it to the list
			for (BB_found_rect_index_2 = 0; BB_found_rect_index_2 < found.size(); BB_found_rect_index_2++)
			{
				if (BB_found_rect_index_2 != BB_found_rect_index_1 && (BB_rect & found[BB_found_rect_index_2]) == BB_rect)
				{
					break;
				}
			}
			// if no other bounding boxes were found that matched this one, then add it to the list of valid bounding boxes.
			if (BB_found_rect_index_2 == found.size())
			{
				found_filtered.push_back(BB_rect);
			}
		}
		
		//iterate through the bounding boxes to detirmine each detected person's pose
		if (found_filtered.size() > 0)
		{
			for (BB_found_rect_index_1 = 0; BB_found_rect_index_1 < found_filtered.size(); BB_found_rect_index_1++)
			{
				//first use derived bounding box to calculate the foreground pixels that are a part of the individual the box is centered on
				// note that there may be parts missing from the created mask
				BB_rect = found_filtered[BB_found_rect_index_1];
				Grab_Cut_Mask.create(Current_Frame.size(), CV_8UC1);
				Grab_Cut_Result_Mask.create(Grab_Cut_Mask.size(), CV_8UC1);
				grabCut(Current_Frame, Grab_Cut_Mask, BB_rect, BgdModel, FgdModel, 1, GC_INIT_WITH_RECT);

				Grab_Cut_Result_Mask = Grab_Cut_Mask & 1;
			
				// Apply repeated dilation and erosion operations to try to connect any floating elements that are close together
				morphologyEx(Grab_Cut_Result_Mask, Grab_Cut_Result_Mask, MORPH_CLOSE, Dilation_Erosion_Element, Point(-1,-1), Dilation_Erosion_Iterations);

				//combine the grab cut mask with the motion data mask
				Combined_Mask = (Grab_Cut_Result_Mask);
			
			}
		}
		else
		{
			Combined_Mask = Scalar(0);
		}
		
		//dilate(Combined_Mask, Combined_Mask, Dilation_Erosion_Element);
		
		// Clear the previous image from the output
		Output_Image = Scalar(0, 0, 0);

		// copy only the elements that the mask designates
		Current_Frame.copyTo(Output_Image, Combined_Mask);
		//Draw bounding box around the found individual, only gets last individual found if multiple for now
		
		for (BB_found_rect_index_1 = 0; BB_found_rect_index_1 < found_filtered.size(); BB_found_rect_index_1++)
		{
			BB_rect = found_filtered[BB_found_rect_index_1];
			rectangle(Output_Image, BB_rect.tl(), BB_rect.br(), Scalar(0,255,0), 2);
		}
		
		found.clear();
		found_filtered.clear();
		
		
		
		//draw the currently found 2D pose on the current frame
		//drawKeypoints(Current_Frame, Current_Pose.Head.Part_Keypoints_SIFT, Frame_With_Pose);






		/*****Dalton Safe Zone*****/
		Pose solution = extract3D_from_Pose_2D(r_3d->extractor, Current_Pose);
        r_3d->solution = solution;
        pair<vector<jointnames::jointnames>, vector<Vec3d>> labeled_3D_joints = Pose2D_to_labeled_3Djoints(Current_Pose);
        r_3d->original = Pose(labeled_3D_joints.first, labeled_3D_joints.second);
        r_3d->project = true;
		/*****Dalton Safe Zone*****/


		Draw_Pose(Current_Frame, &Frame_With_Pose, Current_Pose, 0);
		Draw_Pose(Frame_With_Pose, &Frame_With_Pose, Current_Pose, 1);

		
		
		
		
		//show the frame and write it to an output file
		
		//imshow("Detected 2D Pose", Output_Image); 
		//Output_Video.write(Output_Image);
		
		//drawMatches(Previous_Frame, Previous_Keypoints_SIFT, Current_Frame, Frame_Keypoints_SIFT, Matches_SIFT, Frame_With_Pose);

		//drawMatches(Previous_Frame, Previous_Keypoints_SIFT, Current_Frame, Frame_Keypoints_SIFT, Best_Image_Distance_Matches, Frame_With_Pose);

		imshow("Detected 2D Pose", Frame_With_Pose); 
		Output_Video.write(Frame_With_Pose);

		//waitKey(0);
		
		if (waitKey(20) == 27) //wait for 'esc' key press for 10ms. If 'esc' key is pressed, break loop
		{
		    cout << "\nesc key pressed by user" << endl;
		    break; 
		}
		
		Time_End = chrono::high_resolution_clock::now();
		Time_Span = chrono::duration_cast<chrono::duration<double>>(Time_End - Time_Start);
		
		Frame_Rate_Buffer[(++Frame_Buffer_Index) & 31] = (1 / Time_Span.count());
		Average_Frame_Rate = 0;

		for(int i = 0; i < 32; i++)
		{
			Average_Frame_Rate += Frame_Rate_Buffer[i];
		}

		Average_Frame_Rate = Average_Frame_Rate / 32;
		
		cout << "\r                         " << "\rFrame Rate: " << Average_Frame_Rate << " fps"<< flush;
		
	}


	
	return 0;

}

#ifndef POSE_2D_H
#define POSE_2D_H

#include <opencv2/opencv.hpp>


//structures for tracking part positions
struct BoundingBox
{
	std::vector<cv::Point> Corner_Points;
	cv::RotatedRect R_Rect;
};

struct Limb
{
	BoundingBox Box;
	cv::Point P_External;
	
	std::vector<int> Part_Keypoints_SIFT;
	std::vector<int> Part_Keypoints_SURF;
};

struct Body
{
	BoundingBox Box;
	cv::Point Chest_C;
	cv::Point Chest_L;
	cv::Point Chest_R;
	cv::Point Hip_C;
	cv::Point Hip_L;
	cv::Point Hip_R;
	
	std::vector<int> Part_Keypoints_SIFT;
	std::vector<int> Part_Keypoints_SURF;
};


struct Pose_2D
{
	int Initialized_Parts;
	int Initialized_Joints;
	bool Part_Initialization_Confirmed;

	Body Torso;

	Limb Head;
	Limb Arm_Upper_R;
	Limb Arm_Upper_L;
	Limb Arm_Lower_R;
	Limb Arm_Lower_L;
	Limb Leg_Upper_R;
	Limb Leg_Upper_L;
	Limb Leg_Lower_R;
	Limb Leg_Lower_L;

	cv::Point Head_Top;
	cv::Point Chest_C;
	cv::Point Chest_L;
	cv::Point Chest_R;
	cv::Point Hip_C;
	cv::Point Hip_L;
	cv::Point Hip_R;
	cv::Point Elbow_L;
	cv::Point Elbow_R;
	cv::Point Wrist_L;
	cv::Point Wrist_R;
	cv::Point Knee_L;
	cv::Point Knee_R;
	cv::Point Ankle_L;
	cv::Point Ankle_R;
};


/*
struct Pose_2D
{
    int Initialized_Positions;
    cv::Point Head_Top;
    cv::Point Head_Bot;
    cv::Point C_Chest;
    cv::Point L_Chest;
    cv::Point R_Chest;
    cv::Point C_Hip;
    cv::Point L_Hip;
    cv::Point R_Hip;
    cv::Point L_Elbow;
    cv::Point R_Elbow;
    cv::Point L_Wrist;
    cv::Point R_Wrist;
    cv::Point L_Knee;
    cv::Point R_Knee;
    cv::Point L_Ankle;
    cv::Point R_Ankle;
};

*/

#endif//POSE_2D_H

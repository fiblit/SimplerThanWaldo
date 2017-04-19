#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include <opencv2/opencv.hpp>


#include <iostream>
#include <ctime>
#include <ratio>
#include <chrono>


 
using namespace cv;
using namespace std;


struct Pose_2D
{
	int Initialized_Positions;
	Point Head_Top;
	Point Head_Bot;
	Point C_Chest;
	Point L_Chest;
	Point R_Chest;
	Point C_Hip;
	Point L_Hip;
	Point R_Hip;
	Point L_Elbow;
	Point R_Elbow;
	Point L_Wrist;
	Point R_Wrist;
	Point L_Knee;
	Point R_Knee;
	Point L_Ankle;
	Point R_Ankle;
};



void MouseCallBackFunc(int event, int x, int y, int flags, void* Current_Pose_2D_Ptr)
{
	if  ( event == EVENT_LBUTTONDOWN )
	{
		Pose_2D* Current_Pose_2D = (Pose_2D*)Current_Pose_2D_Ptr;
		
		switch(Current_Pose_2D->Initialized_Positions)
		{
			case 0: 
			{
				Current_Pose_2D->Head_Top.x = x;
				Current_Pose_2D->Head_Top.y = y;
				cout << "Top of Head Position (" << x << ", " << y << ")" << endl;
				break;
			}
			case 1: 
			{
				Current_Pose_2D->Head_Bot.x = x;
				Current_Pose_2D->Head_Bot.y = y;
				cout << "Bottom of Head Position (" << x << ", " << y << ")" << endl;
				break;
			}
			case 2: 
			{
				Current_Pose_2D->C_Chest.x = x;
				Current_Pose_2D->C_Chest.y = y;
				cout << "Chest Center Position (" << x << ", " << y << ")" << endl;
				break;
			}
			case 3: 
			{
				Current_Pose_2D->L_Chest.x = x;
				Current_Pose_2D->L_Chest.y = y;
				cout << "Left Shoulder Position (" << x << ", " << y << ")" << endl;
				break;
			}
			case 4: 
			{
				Current_Pose_2D->R_Chest.x = x;
				Current_Pose_2D->R_Chest.y = y;
				cout << "Right Shoulder Position (" << x << ", " << y << ")" << endl;
				break;
			}
			case 5: 
			{
				Current_Pose_2D->C_Hip.x = x;
				Current_Pose_2D->C_Hip.y = y;
				cout << "Center of Hip Position (" << x << ", " << y << ")" << endl;
				break;
			}
			case 6: 
			{
				Current_Pose_2D->L_Hip.x = x;
				Current_Pose_2D->L_Hip.y = y;
				cout << "Left Side of Hip Position (" << x << ", " << y << ")" << endl;
				break;
			}
			case 7: 
			{
				Current_Pose_2D->R_Hip.x = x;
				Current_Pose_2D->R_Hip.y = y;
				cout << "Right Side of Hip Position (" << x << ", " << y << ")" << endl;
				break;
			}
			case 8: 
			{
				Current_Pose_2D->L_Elbow.x = x;
				Current_Pose_2D->L_Elbow.y = y;
				cout << "Left Elbow Position (" << x << ", " << y << ")" << endl;
				break;
			}
			case 9: 
			{
				Current_Pose_2D->L_Wrist.x = x;
				Current_Pose_2D->L_Wrist.y = y;
				cout << "Left Wrist Position (" << x << ", " << y << ")" << endl;
				break;
			}
			case 10: 
			{
				Current_Pose_2D->R_Elbow.x = x;
				Current_Pose_2D->R_Elbow.y = y;
				cout << "Right Elbow Position (" << x << ", " << y << ")" << endl;
				break;
			}
			case 11: 
			{
				Current_Pose_2D->R_Wrist.x = x;
				Current_Pose_2D->R_Wrist.y = y;
				cout << "Right Wrist Position (" << x << ", " << y << ")" << endl;
				break;
			}
			case 12: 
			{
				Current_Pose_2D->L_Knee.x = x;
				Current_Pose_2D->L_Knee.y = y;
				cout << "Left Knee Position (" << x << ", " << y << ")" << endl;
				break;
			}
			case 13: 
			{
				Current_Pose_2D->L_Ankle.x = x;
				Current_Pose_2D->L_Ankle.y = y;
				cout << "Left Ankle Position (" << x << ", " << y << ")" << endl;
				break;
			}
			case 14: 
			{
				Current_Pose_2D->R_Knee.x = x;
				Current_Pose_2D->R_Knee.y = y;
				cout << "Right Knee Position (" << x << ", " << y << ")" << endl;
				break;
			}
			case 15: 
			{
				Current_Pose_2D->R_Ankle.x = x;
				Current_Pose_2D->R_Ankle.y = y;
				cout << "Right Ankle Position (" << x << ", " << y << ")" << endl;
				break;
			}
			default:
			{
				break;
			}
		}
		if(Current_Pose_2D->Initialized_Positions < 16)
		{
			Current_Pose_2D->Initialized_Positions += 1;
		}
		else
		{
			cout << "All joint positions have been initialized.  Press a key to continue." << endl;
		}
	}
}



void Draw_Pose_2D(Mat Frame, Pose_2D Frame_Pose_2D)
{

	line(Frame, Frame_Pose_2D.Head_Top, Frame_Pose_2D.Head_Bot, Scalar(255,0,0), 2);
	line(Frame, Frame_Pose_2D.Head_Bot, Frame_Pose_2D.C_Chest, Scalar(255,0,0), 2);
	line(Frame, Frame_Pose_2D.C_Chest, Frame_Pose_2D.L_Chest, Scalar(255,0,0), 2);
	line(Frame, Frame_Pose_2D.C_Chest, Frame_Pose_2D.R_Chest, Scalar(255,0,0), 2);
	line(Frame, Frame_Pose_2D.C_Chest, Frame_Pose_2D.C_Hip, Scalar(255,0,0), 2);
	line(Frame, Frame_Pose_2D.C_Hip, Frame_Pose_2D.L_Hip, Scalar(255,0,0), 2);
	line(Frame, Frame_Pose_2D.C_Hip, Frame_Pose_2D.R_Hip, Scalar(255,0,0), 2);
	line(Frame, Frame_Pose_2D.L_Chest, Frame_Pose_2D.L_Elbow, Scalar(255,0,0), 2);
	line(Frame, Frame_Pose_2D.L_Elbow, Frame_Pose_2D.L_Wrist, Scalar(255,0,0), 2);
	line(Frame, Frame_Pose_2D.R_Chest, Frame_Pose_2D.R_Elbow, Scalar(255,0,0), 2);
	line(Frame, Frame_Pose_2D.R_Elbow, Frame_Pose_2D.R_Wrist, Scalar(255,0,0), 2);
	line(Frame, Frame_Pose_2D.L_Hip, Frame_Pose_2D.L_Knee, Scalar(255,0,0), 2);
	line(Frame, Frame_Pose_2D.L_Knee, Frame_Pose_2D.L_Ankle, Scalar(255,0,0), 2);
	line(Frame, Frame_Pose_2D.R_Hip, Frame_Pose_2D.R_Knee, Scalar(255,0,0), 2);
	line(Frame, Frame_Pose_2D.R_Knee, Frame_Pose_2D.R_Ankle, Scalar(255,0,0), 2);


	circle(Frame, Frame_Pose_2D.Head_Top, 3, Scalar(0,0,255), -1);
	circle(Frame, Frame_Pose_2D.Head_Bot, 3, Scalar(0,0,255), -1);
	circle(Frame, Frame_Pose_2D.C_Chest, 3, Scalar(0,0,255), -1);
	circle(Frame, Frame_Pose_2D.L_Chest, 3, Scalar(0,0,255), -1);
	circle(Frame, Frame_Pose_2D.R_Chest, 3, Scalar(0,0,255), -1);
	circle(Frame, Frame_Pose_2D.C_Hip, 3, Scalar(0,0,255), -1);
	circle(Frame, Frame_Pose_2D.L_Hip, 3, Scalar(0,0,255), -1);
	circle(Frame, Frame_Pose_2D.R_Hip, 3, Scalar(0,0,255), -1);
	circle(Frame, Frame_Pose_2D.L_Elbow, 3, Scalar(0,0,255), -1);
	circle(Frame, Frame_Pose_2D.L_Wrist, 3, Scalar(0,0,255), -1);
	circle(Frame, Frame_Pose_2D.R_Elbow, 3, Scalar(0,0,255), -1);
	circle(Frame, Frame_Pose_2D.R_Wrist, 3, Scalar(0,0,255), -1);
	circle(Frame, Frame_Pose_2D.L_Elbow, 3, Scalar(0,0,255), -1);
	circle(Frame, Frame_Pose_2D.L_Wrist, 3, Scalar(0,0,255), -1);
	circle(Frame, Frame_Pose_2D.R_Knee, 3, Scalar(0,0,255), -1);
	circle(Frame, Frame_Pose_2D.R_Ankle, 3, Scalar(0,0,255), -1);
	circle(Frame, Frame_Pose_2D.L_Knee, 3, Scalar(0,0,255), -1);
	circle(Frame, Frame_Pose_2D.L_Ankle, 3, Scalar(0,0,255), -1);
}



int main(int argc, char* argv[])
{
	// General Program Variables
	bool bSuccess;
	
	VideoCapture Input_Video;
	VideoWriter Output_Video;
	
	
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

	
	const string Output_Video_File_Name = "Result.avi";
	int ex = static_cast<int>(Input_Video.get(CV_CAP_PROP_FOURCC));
			  
	Output_Video.open(Output_Video_File_Name, ex, Input_Video.get(CV_CAP_PROP_FPS), Size(256,256), true);
	
	if (!Output_Video.isOpened())
	{
		cout  << "Could not open the output video for writing.\n" << endl;
		return -1;
	}
	
	double dWidth = Input_Video.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
	double dHeight = Input_Video.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video

	cout << "Original Frame size : " << dWidth << " x " << dHeight << endl;
	
	
	Pose_2D Current_Pose_2D;
	Current_Pose_2D.Initialized_Positions = 0;
	
	namedWindow("Detected 2D Pose_2D",CV_WINDOW_AUTOSIZE); //create a window 
	setMouseCallback("Detected 2D Pose_2D", MouseCallBackFunc, &Current_Pose_2D);
	
	
	Mat First_Frame;
	
	bSuccess = Input_Video.read(First_Frame); // read a new frame from video
	
	if (!bSuccess) //if not success, break loop
	{
		 cout << "Cannot read a frame from video or camera stream" << endl;
		 return -1;
	}
	
	First_Frame.convertTo(First_Frame, CV_8U);
	resize(First_Frame, First_Frame, Size(256,256), 0, 0, INTER_LINEAR);
	
	imshow("Detected 2D Pose_2D", First_Frame); //show the frame
	
	// Wait until user press some key after all joints have been clicked
	cout << "Please click on the following locations in the listed order." << endl;
	cout << "\nTop of Head\nBottom of Head\nCenter of Chest\nLeft Shoulder\nRight Shoulder\nCenter of Hips\nLeft Side of Hip\nRight Side of Hip\nLeft Elbow\nLeft Wrist\nRight Elbow\nRight Wrist\nLeft Knee\nLeft Ankle\nRight Knee\nRight Ankle\n"<< endl;
	
	while(Current_Pose_2D.Initialized_Positions < 16)
	{
		waitKey(0);
		if(Current_Pose_2D.Initialized_Positions < 16)
		{
			cout << "Please select remaining " << 16 - Current_Pose_2D.Initialized_Positions << " body part locations." << endl;
		}
	}

	Draw_Pose_2D(First_Frame, Current_Pose_2D);
	imshow("Detected 2D Pose_2D", First_Frame); //show the frame

	cout << "\nInitial pose shown in initial frame.  Press any key to begin tracking through remainder of video.\n" << endl;
	
    	waitKey(0);
	
	
	
	
	
	
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//------------------------------------ Variable Declaration For Algorithm-------------------------------------//
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	// General Variables
	int X_Index;
	int Y_Index;
	

	
	
	// Variables for tracking time of program operation
	chrono::high_resolution_clock::time_point Time_Start = chrono::high_resolution_clock::now();
	chrono::high_resolution_clock::time_point Time_End = chrono::high_resolution_clock::now();
	chrono::duration<double> Time_Span = chrono::duration_cast<chrono::duration<double>>(Time_End - Time_Start);

	double Frame_Rate_Buffer [32];
	double Average_Frame_Rate = 0;
	int Frame_Buffer_Index = -1;
	
	//variables for conducting motion detection
	Mat Previous_Frame, Current_Frame, Next_Frame;
	Mat Previous_Frame_Gray, Current_Frame_Gray, Next_Frame_Gray;
	Mat Previous_Frame_Gray_Blurred, Current_Frame_Gray_Blurred, Next_Frame_Gray_Blurred;
	
	Mat Absdiff_Previous_Next, Absdiff_Current_Next;
	Mat BW_And_Result;
	
	vector<vector<Point> > Motion_Contours;
	vector<Vec4i> Motion_Contour_Hierarchy;
	
	int Min_Contour_Size = 5;
	int Contour_Index;
	
	Mat Final_Motion_Mask(Size(256,256), CV_8UC1);
	
	

	
	//variables for bounding box
	HOGDescriptor hog;
    hog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());
	
	vector<Rect> found, found_filtered;
	size_t BB_found_rect_index_1, BB_found_rect_index_2;
	Rect BB_rect;
	
	//variables for extracting pixels which correspond to people inside of a bounding box
	Mat Grab_Cut_Mask;
	Mat BgdModel, FgdModel;
	Mat Grab_Cut_Result_Mask;

	int Dilation_Erosion_Size = 2;
	int Dilation_Erosion_Iterations = 3;

	Mat Dilation_Erosion_Element = getStructuringElement(MORPH_RECT,
							     Size(2 * Dilation_Erosion_Size + 1, 2 * Dilation_Erosion_Size + 1),
							     Point(Dilation_Erosion_Size, Dilation_Erosion_Size));
	

	
	Mat Combined_Mask(256, 256, CV_8UC1);
	vector<vector<Point> > Final_Mask_Contours;
	vector<Vec4i> Final_Mask_Contour_Hierarchy;
	size_t Largest_Contour_Size = 0;
	int Largest_Contour_Index = 0;
	
	Mat Pose_2D_Mask(Size(256,256), CV_8UC1);
	
	
	
	Mat Grad_X, Grad_Y;
	Mat Grad_Mag, Grad_Dir;
	Mat Detected_Edges;
	
	
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//------------------------------------------ Program Initialization ------------------------------------------//
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////	
	
	//Set up initial loaded images and related data	
	bSuccess = Input_Video.read(Current_Frame); // read a new frame from video
		
	if (!bSuccess) //if not success, return 0
	{
		 cout << "\nCannot read a frame from video stream" << endl;
		 return 0;
	}
	
	Current_Frame.convertTo(Current_Frame, CV_8U);
	resize(Current_Frame, Current_Frame, Size(256,256), 0, 0, INTER_LINEAR);
	cvtColor(Current_Frame, Current_Frame_Gray, CV_BGR2GRAY);
	
	GaussianBlur(Current_Frame_Gray, Current_Frame_Gray_Blurred, Size(5,5), 0, 0, BORDER_DEFAULT);
	
	bSuccess = Input_Video.read(Next_Frame); // read a new frame from video
		
	if (!bSuccess) //if not success, return 0
	{
		 cout << "\nCannot read a frame from video stream" << endl;
		 return 0;
	}
	
	Next_Frame.convertTo(Next_Frame, CV_8U);
	resize(Next_Frame, Next_Frame, Size(256,256), 0, 0, INTER_LINEAR);
	cvtColor(Next_Frame, Next_Frame_Gray, CV_BGR2GRAY);
	
	GaussianBlur(Next_Frame_Gray, Next_Frame_Gray_Blurred, Size(5,5), 0, 0, BORDER_DEFAULT);
	
	
	//declare output image and its size
	Mat Output_Image;
	

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

		Current_Frame_Gray.copyTo(Previous_Frame_Gray);
		Next_Frame_Gray.copyTo(Current_Frame_Gray);

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
		resize(Next_Frame, Next_Frame, Size(256,256), 0, 0, INTER_LINEAR);
		cvtColor(Next_Frame, Next_Frame_Gray, CV_BGR2GRAY);
		
		GaussianBlur(Next_Frame_Gray, Next_Frame_Gray_Blurred, Size(5,5), 0, 0, BORDER_DEFAULT);
		
		

		//Start by doing HOG analysis on image to detirmine if any features of interest can be found

		Mat Gradient_X, Gradient_Y, Gradient_Mag, Gradient_Dir;
		
















		//Next do analysis on the frames to detirmine which portions are part of moving objects
		
		//Now calculate the difference between the current frame and the next frame, along with the previous frame and the next frame
		
		absdiff(Previous_Frame_Gray_Blurred, Next_Frame_Gray_Blurred, Absdiff_Previous_Next);
		absdiff(Current_Frame_Gray_Blurred, Next_Frame_Gray_Blurred, Absdiff_Current_Next);
		
		//bit wise and the differences to Input_Videoture only the current motion changes
		bitwise_and(Absdiff_Previous_Next, Absdiff_Current_Next, BW_And_Result);
		
		//threshold to only capture changes that are significant
		threshold(BW_And_Result, BW_And_Result, 25, 255, CV_THRESH_BINARY);
		
		//Dilate the result to fill in holes
		dilate(BW_And_Result, BW_And_Result, Dilation_Erosion_Element);
		

			
		//Find contours in the derived mask
		
		findContours(BW_And_Result, Motion_Contours, Motion_Contour_Hierarchy, RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
		
		Final_Motion_Mask = Scalar(0);
		
		for(Contour_Index = 0; Contour_Index < Motion_Contours.size(); Contour_Index++)
		{
			if(contourArea(Motion_Contours[Contour_Index]) > Min_Contour_Size)
			{
				Rect Contour_BB = boundingRect(Mat(Motion_Contours[Contour_Index]));
				
				for(X_Index = Contour_BB.x; X_Index < Contour_BB.x + Contour_BB.width; X_Index++)
				{
					for(Y_Index = Contour_BB.y; Y_Index < Contour_BB.y + Contour_BB.height; Y_Index++)
					{
						if(pointPolygonTest(Motion_Contours[Contour_Index], Point2f(X_Index, Y_Index), false) >= 0)
						{
							Final_Motion_Mask.at<uchar>(Y_Index, X_Index) = 1;
						}
					}
				}
			}		
		}
		
		
		//Extract the initial bounding box estimate using the HOG transform
		
		//Enable this section to use OpenCV's implemented people finding Homography finder.
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
				Combined_Mask = (Grab_Cut_Result_Mask | Final_Motion_Mask);
			
			}
		}
		else
		{
			Combined_Mask = Scalar(0);
		}
		
		dilate(Combined_Mask, Combined_Mask, Dilation_Erosion_Element);
		
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
		Draw_Pose_2D(Output_Image, Current_Pose_2D);
		imshow("Detected 2D Pose_2D", Output_Image); //show the frame
		
		//Write frame to output file
		Output_Video.write(Output_Image);
		
		if (waitKey(500) == 27) //wait for 'esc' key press for 10ms. If 'esc' key is pressed, break loop
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

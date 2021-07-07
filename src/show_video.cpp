#include "ros/ros.h"
#include <iostream>
#include <stdio.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
using namespace cv;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "show_image");
	ros::NodeHandle node_obj;

    namedWindow( "Example2", cv::WINDOW_AUTOSIZE );
    VideoCapture cap;
    cap.open(String(argv[1]));
    Mat frame;

    for(;;)
    {
        cap >> frame;

        if(frame.empty()) 
            break;

        imshow("Example3", frame);

        if(waitKey(33) >= 0)
            break;

    }

    return 0;
}
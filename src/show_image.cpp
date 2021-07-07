#include "ros/ros.h"
#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp> //Include file for every supported OpenCV function
using namespace cv;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "show_image");
	ros::NodeHandle node_obj;

    ROS_INFO("OpenCV Test");

    Mat img = imread( argv[1], -1 );
    if( img.empty() ) return -1;
    namedWindow( "Example1", cv::WINDOW_AUTOSIZE );
    imshow( "Example1", img );
    waitKey( 0 );
    destroyWindow( "Example1" );
}
#include "ros/ros.h"
#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp> //Include file for every supported OpenCV function
/*#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"*/
//using namespace cv;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "opencv_test");
	ros::NodeHandle node_obj;

    ROS_INFO("Ciao");

    cv::Mat img = cv::imread(argv[1],-1);
    if( img.empty() ) return -1;
    cv::namedWindow( "Example1", cv::WINDOW_AUTOSIZE );
    cv::imshow( "Example1", img );
    cv::waitKey( 0 );
    cv::destroyWindow( "Example1" );
    return 0;

    /*Mat img = imread( argv[1], -1 );
    if( img.empty() ) return -1;
    namedWindow( "Example1", cv::WINDOW_AUTOSIZE );
    imshow( "Example1", img );
    waitKey( 0 );
    destroyWindow( "Example1" );*/
}
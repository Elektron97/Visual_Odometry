/***************VISUAL ODOMETRY*****************
* Stimare la posa da sequenze di Immagini.     *
***********************************************/


/**********************************************TOPICS AND MSGS****************************************************
*topics:      /zeno/dvl                                         911 msgs    : uuv_sensor_ros_plugins_msgs/DVL    *
*             /zeno/imu                                        6555 msgs    : sensor_msgs/Imu                    *   
*             /zeno/laser                                       656 msgs    : sensor_msgs/LaserScan              *
*             /zeno/pose_gt                                    2622 msgs    : nav_msgs/Odometry                  *
*             /zeno/zeno/cameraleft/camera_image/compressed    2359 msgs    : sensor_msgs/CompressedImage        *
*             /zeno/zeno/cameraright/camera_image/compressed   2359 msgs    : sensor_msgs/CompressedImage        *
******************************************************************************************************************/

/*INCLUDE LIBRARIES*/
#include "ros/ros.h"
#include "/usr/include/eigen3/Eigen/Eigen"
#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp> 

/*DEFINE*/
#define FIRST_IMAGE 580
#define MIN_NUM_FEATURES 20

/*NAMESPACE*/
using namespace std;
using namespace Eigen;

/*GLOBAL VARIABLES*/ //Da mettere in YAML?
string detector_method = "SURF"; //to do: enum | HARRIS | FAST | KAZE | ORB | SURF | SIFT
string feature_method = "MATCHING"; //to do: bool | MATCHING | TRACKING
string estimation_method = "ESSENTIAL"; //to do: bool | ESSENTIAL | HOMOGRAPHY
bool fromBag = true; // True: Simulation Data | False: Real Data
string camera = "frontal"; //to do: bool | frontal | bottom

bool sdr_Rbc = true; // TRUE: ENU | FALSE: NED -> Dichiaro nel main, eigen antipatico
Matrix3f Rbc; 

bool JumpCond = true;
int distance = 10;  //distance in pixels for Jumping Condition

bool Loop_closing = false;
bool motion2D = true;   //Planar motion: [x y yaw]
//bool magnetic_comp = true;

/*FUNCTIONS DECLARATION*/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "visual_Odometry");
	ros::NodeHandle node_obj;

    /*UPLOAD DATA*/


    /*INIT*/

    while(ros::ok())
    {
        ROS_INFO("TEST");
        break;
    }
    return 0;
}
/***************VISUAL ODOMETRY*****************
* Stimare la posa da sequenze di Immagini.     *
***********************************************/

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
int distance = 10;

bool Loop_closing = false;
bool motion2D = true;
//bool magnetic_comp = true;


/*FUNCTIONS DECLARATION*/


int main(int argc, char **argv)
{
    ros::init(argc, argv, "visual_Odometry");
	ros::NodeHandle node_obj;

    /*UPLOAD DATA*/
    //To do: rosbag input -> Subscribe?

    /*INIT*/

    while(ros::ok())
    {
        ROS_INFO("TEST");
        break;
    }
    return 0;
}
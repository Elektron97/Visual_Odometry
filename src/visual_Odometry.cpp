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
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

/*ROS MSGS*/
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/CompressedImage.h"

//msg from uuv ???

/*DEFINE*/
#define FIRST_IMAGE 580
#define viewId_stop 810
#define MIN_NUM_FEATURES 20
#define FREQUENCY 10

/*NAMESPACE*/
using namespace std;
using namespace Eigen;
using namespace cv;

/*PARAMETERS*/ //Da mettere in YAML?
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

/*GLOBAL VARIABLES*/
sensor_msgs::Imu imu;   //Al momento inutile!
sensor_msgs::LaserScan laser;
sensor_msgs::CompressedImage camera_sx;
sensor_msgs::CompressedImage camera_dx;

nav_msgs::Odometry ground_truth;

int discard = 0;

/*FUNCTIONS DECLARATION*/
Mat ros2cv(sensor_msgs::CompressedImage image);

/*CALLBACK*/
void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
    /************************************************  
    *    std_msgs/Header header                     *
    *       uint32 seq                              *
    *       time stamp                              *
    *       string frame_id                         *
    *    geometry_msgs/Quaternion orientation       *
    *       float64 x                               *
    *       float64 y                               *
    *       float64 z                               *
    *       float64 w                               *
    *    float64[9] orientation_covariance          *
    *    geometry_msgs/Vector3 angular_velocity     *
    *      float64 x                                *
    *      float64 y                                *
    *      float64 z                                *
    *    float64[9] angular_velocity_covariance     *
    *    geometry_msgs/Vector3 linear_acceleration  *
    *      float64 x                                *
    *      float64 y                                *
    *      float64 z                                *
    *    float64[9] linear_acceleration_covariance  *
    *************************************************/

    imu.header = msg->header;
    imu.orientation = msg->orientation;
    imu.orientation_covariance = msg->orientation_covariance;
    imu.angular_velocity = msg ->angular_velocity;
    imu.angular_velocity_covariance = msg->angular_velocity_covariance;
    imu.linear_acceleration = msg->linear_acceleration;
    imu.linear_acceleration_covariance = msg->linear_acceleration_covariance;
}

void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    /****************************
    * std_msgs/Header header    *
    *    uint32 seq             *
    *    time stamp             *
    *    string frame_id        *
    *  float32 angle_min        *
    *  float32 angle_max        *
    *  float32 angle_increment  *
    *  float32 time_increment   *
    *  float32 scan_time        *
    *  float32 range_min        *
    *  float32 range_max        *
    *  float32[] ranges         *
    *  float32[] intensities    *
    *****************************/

   laser.header = msg->header;
   laser.angle_min = msg->angle_min;
   laser.angle_max = msg->angle_max;
   laser.angle_increment = msg->angle_increment;
   laser.time_increment = msg->time_increment;
   laser.scan_time = msg->scan_time;
   laser.range_min = msg->range_min;
   laser.range_max = msg->range_max;
   laser.ranges = msg->ranges;
   laser.intensities = msg->intensities;
}

void cameraSX_callback(const sensor_msgs::CompressedImage::ConstPtr& msg)
{   
    /************************
    *std_msgs/Header header *
    *    uint32 seq         *
    *    time stamp         *
    *    string frame_id    *
    *string format          *
    *uint8[] data           *
    *************************/

    camera_sx.header = msg->header;
    camera_sx.format = msg->format;
    camera_sx.data = msg->data;
    discard++;
}

void cameraDX_callback(const sensor_msgs::CompressedImage::ConstPtr& msg)
{   
    /************************
    *std_msgs/Header header *
    *    uint32 seq         *
    *    time stamp         *
    *    string frame_id    *
    *string format          *
    *uint8[] data           *
    *************************/

    camera_dx.header = msg->header;
    camera_dx.format = msg->format;
    camera_dx.data = msg->data;
}

void groundTruth_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    ground_truth.header = msg->header;
    ground_truth.child_frame_id = msg->child_frame_id;
    ground_truth.pose = msg->pose;
    ground_truth.twist = msg->twist;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "visual_Odometry");
	ros::NodeHandle node_obj;

    //Pub Object
    ros::Publisher pub =  node_obj.advertise<nav_msgs::Odometry>("/odom",10);

    //Sub Objects
	ros::Subscriber sub_imu=node_obj.subscribe("/zeno/imu", 1, imu_callback);
	ros::Subscriber sub_laser=node_obj.subscribe("/zeno/laser", 1, laser_callback);

	//ros::Subscriber sub_dvl=node_obj.subscribe("/zeno/dvl", 1, dvl_callback);

	ros::Subscriber sub_cameraSX=node_obj.subscribe("/zeno/zeno/cameraleft/camera_image/compressed", 1, cameraSX_callback);
    ros::Subscriber sub_cameraDX=node_obj.subscribe("/zeno/zeno/cameraright/camera_image/compressed", 1, cameraDX_callback);

    //Image transport funziona SOLO con sensor_msgs/Image e non sensor_msgs/CompressedImage
    /*image_transport::ImageTransport it(node_obj);
    image_transport::Subscriber sub_cameraSX = it.subscribe("/zeno/zeno/cameraleft/camera_image/compressed", 1, cameraSX_callback);
    image_transport::Subscriber sub_cameraDX = it.subscribe("/zeno/zeno/cameraright/camera_image/compressed", 1, cameraDX_callback);*/
    
	ros::Subscriber sub_GT=node_obj.subscribe("/zeno/posegt", 1, groundTruth_callback);

	ros::Rate loop_rate(FREQUENCY);	//10 Hz Prediction step

    /*Aspetto la FIRST_IMAGE*/
    ROS_INFO("Waiting %d-th frame...", FIRST_IMAGE);
    while(discard < FIRST_IMAGE) //scarto le prime immagini
    {
        ros::spinOnce();
    }
    ROS_WARN("START!");

    /*INITIALIZATION*/


    /*ITERATIONS*/
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();	

        if(discard > viewId_stop)
            break;

        /*SHOW IMAGE FROM BAG FILE*/
        Mat image_test = ros2cv(camera_sx);
        if( image_test.empty() ) return -1;
        namedWindow( "CameraSX", cv::WINDOW_AUTOSIZE );
        imshow("CameraSX", image_test);
        waitKey(33);
    }
    destroyWindow("CameraSX");
    ROS_WARN("Video Finito!");

    return 0;
}

Mat ros2cv(sensor_msgs::CompressedImage image)
{
    //provo SENZA image_transport -> Non ottimizzato. 
    //La tengo per ricordo
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    }

    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    return cv_ptr->image;
}
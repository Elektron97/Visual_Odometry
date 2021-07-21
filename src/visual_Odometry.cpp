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
#include <opencv2/xfeatures2d/nonfree.hpp>
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
using namespace cv::xfeatures2d;

/*CAMERA UTILITY*/
//Intrinsic Parameters
const double fx = 407.0646129842357;
const double fy = 407.0646129842357;
const double ccxLeft = 384.5;
const double ccyLeft = 246.5;

//Distortion Coefficients
const double k1 = 0.0;
const double k2 = 0.0;
const double p1 = 0.0;
const double p2 = 0.0;

//showImg utility
const int fps = 33;
bool showFrame = false;
bool showMatch = true;

//SURF parameters
int minHessian = 100;

const double focal_length[] = {fx, fy};
const double principalPoint[] = {ccxLeft, ccyLeft};

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

/*FUNCTIONS DECLARATION*/
Mat ros2cv(sensor_msgs::CompressedImage image);
Mat get_image(Mat current_img, Mat cameraMatrix, Mat distortionCoeff); 
vector<DMatch> detectAndMatchFeatures(Mat img1, Mat img2);

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

    /*ros::Subscriber risulta meno efficente di image_transport.*/
    //http://wiki.ros.org/compressed_image_transport
	ros::Subscriber sub_cameraSX=node_obj.subscribe("/zeno/zeno/cameraleft/camera_image/compressed", 1, cameraSX_callback);
    ros::Subscriber sub_cameraDX=node_obj.subscribe("/zeno/zeno/cameraright/camera_image/compressed", 1, cameraDX_callback);

	ros::Subscriber sub_GT=node_obj.subscribe("/zeno/posegt", 1, groundTruth_callback);

	ros::Rate loop_rate(FREQUENCY);	//10 Hz Prediction step

    /*Aspetto la FIRST_IMAGE*/
    ROS_INFO("Waiting %d-th frame...", FIRST_IMAGE);
    
    while(camera_sx.header.seq < FIRST_IMAGE) //scarto le prime immagini
    {
        ros::spinOnce();
    }
    ROS_WARN("START!");

    /*INITIALIZATION*/
    //Define Camera matrix and Distortion Coeff.
    Mat cameraMatrix = (Mat1d(3, 3) << fx, 0, ccxLeft, 0, fy, ccyLeft, 0, 0, 1);
    Mat distortionCoeff = (Mat1d(1, 4) << k1, k2, p1, p2);

    //Undistort Image
    Mat prev_img = get_image(ros2cv(camera_sx), cameraMatrix, distortionCoeff);

    /*GET GROUND TRUTH POSE*/
    //Creare Matrice da Quaternione
    //Vettore traslazione

    /*ITERATIONS*/
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();	

        if(camera_sx.header.seq > viewId_stop)
            break;

        /*SHOW IMAGE FROM BAG FILE*/
        if(showFrame)
        {
            namedWindow("Image", cv::WINDOW_AUTOSIZE);
            imshow("Image", ros2cv(camera_sx));
            waitKey(fps);
        }

        /*FEATURE MATCHING*/
        Mat curr_img = get_image(ros2cv(camera_sx), cameraMatrix, distortionCoeff);

        //-- Step 1: Detect the keypoints using SURF Detector, compute the descriptors
        Ptr<SURF> detector = SURF::create( minHessian );
        vector<KeyPoint> keypoints1, keypoints2;
        Mat descriptors1, descriptors2;
        detector->detectAndCompute( prev_img, noArray(), keypoints1, descriptors1 );
        detector->detectAndCompute( curr_img, noArray(), keypoints2, descriptors2 );

        //-- Step 2: Matching descriptor vectors with a brute force matcher
        // Since SURF is a floating-point descriptor NORM_L2 is used
        Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::BRUTEFORCE);
        vector< DMatch > matches;
        matcher->match( descriptors1, descriptors2, matches );

        if(showMatch)
        {
            //-- Draw matches
            Mat img_matches;
            drawMatches( prev_img, keypoints1, curr_img, keypoints2, matches, img_matches );
            //-- Show detected matches
            imshow("Matches", img_matches );
            waitKey(fps);
        }

        /*POSE ESTIMATION*/
        //Stima dell'Essential Matrix -> cv::findEssentialMat()
        //Essa richiede i Keypoint in vector<Point2d>, non <KeyPoint>
        vector<Point2f> keypoints1_conv, keypoints2_conv;
        /*KeyPoint::convert(keypoints1, keypoints1_conv);
        KeyPoint::convert(keypoints2, keypoints2_conv);*/

        //Non va bene perche' devo tenere conto del match!
        //Eseguendo infatti ho problemi di dimensioni!

        // Convert keypoints into Point2f
        for (vector<DMatch>::const_iterator it= matches.begin(); it!= matches.end(); ++it) 
        {    
            // Get the position of keypoints1
            keypoints1_conv.push_back(keypoints1[it->queryIdx].pt); //query per keypoints1
            // Get the position of keypoints2
            keypoints2_conv.push_back(keypoints2[it->trainIdx].pt); //train per keypoints2
        }

        vector<uchar> RANSAC_status;

        //default values: RANSAC, prob =0.999, threshold = 1.0
        Mat E = findEssentialMat(keypoints1_conv, keypoints2_conv, cameraMatrix, RANSAC, 0.999, 1.0, RANSAC_status);

        //RANSAC_status, vettore contenente N elementi (N = length(keypoints)) in cui indica:
        // 0 = outlier
        // 1 = inlier

        int outlierCount = 0;
        for(int i = 0; i < RANSAC_status.size(); i++)
        {
            if(RANSAC_status[i] == 0)
                outlierCount ++;

        }

        int inlierCount = RANSAC_status.size() - outlierCount;

        // The above three variables are used to save the inner point and the matching relationship
        vector<Point2f> leftInlier;
        vector<Point2f> rightInlier;
        vector<DMatch> inlierMatches;
        

        inlierMatches.resize(inlierCount);
        leftInlier.resize(inlierCount);
        rightInlier.resize(inlierCount);

        /*inlierCount = 0;
        for (int i=0; i<RANSAC_status.size(); i++)
        {
            if (RANSAC_status[i] != 0)
            {
                leftInlier[inlierCount].x = keypoints1_conv.at<float>(i, 0);
                leftInlier[inlierCount].y = keypoints1_conv.at<float>(i, 1);
                rightInlier[inlierCount].x = keypoints2_conv.at<float>(i, 0);
                rightInlier[inlierCount].y = keypoints2_conv.at<float>(i, 1);
                inlierMatches[inlierCount].queryIdx = inlierCount;
                inlierMatches[inlierCount].trainIdx = inlierCount;
                inlierCount++;
            }
        }*/
        
        /*/ / Convert the inner point to the format that drawMatches can use
        vector<KeyPoint> key1(InlinerCount);
        vector<KeyPoint> key2(InlinerCount);
        KeyPoint::convert(m_LeftInlier, key1);
        KeyPoint::convert(m_RightInlier, key2);
        
        // Display the inner point matching after the calculation F
        // Mat m_matLeftImage;
        // Mat m_matRightImage;
        // The above two variables hold the left and right images.
        Mat OutImage;
        drawMatches(m_matLeftImage, key1, m_matRightImage, key2, m_InlierMatches, OutImage);
        cvNamedWindow( "Match features", 1);
        cvShowImage("Match features", &(IplImage(OutImage)));
        cvWaitKey( 0 );
        cvDestroyWindow( "Match features" );*/



        prev_img = curr_img; 
    }
    ROS_WARN("Video Finito!");

    return 0;
}

Mat ros2cv(sensor_msgs::CompressedImage image)
{
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

Mat get_image(Mat current_img, Mat cameraMatrix, Mat distortionCoeff)
{
    /*****Undistort Image************************ 
    * Matlab: undistortImage(img, cameraParams) *
    * Dato che hanno usato "cameraIntrinsic",   *
    * Radial e Tang. Distortion = [0 0]         *
    * Per default!                              *
    ********************************************/

    Mat gray_img;
    cvtColor(current_img, gray_img, COLOR_RGB2GRAY); //void cvtColor()

    Mat undistorted_image;
    undistort(gray_img, undistorted_image, cameraMatrix, distortionCoeff);
    return undistorted_image;

}

vector<DMatch> detectAndMatchFeatures(Mat img1, Mat img2)
{
    //-- Step 1: Detect the keypoints using SURF Detector, compute the descriptors
    Ptr<SURF> detector = SURF::create( minHessian );
    vector<KeyPoint> keypoints1, keypoints2;
    Mat descriptors1, descriptors2;
    detector->detectAndCompute( img1, noArray(), keypoints1, descriptors1 );
    detector->detectAndCompute( img2, noArray(), keypoints2, descriptors2 );

    //-- Step 2: Matching descriptor vectors with a brute force matcher
    // Since SURF is a floating-point descriptor NORM_L2 is used
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::BRUTEFORCE);
    vector< DMatch > matches;
    matcher->match( descriptors1, descriptors2, matches );

    if(showMatch)
    {
        //-- Draw matches
        Mat img_matches;
        drawMatches( img1, keypoints1, img2, keypoints2, matches, img_matches );
        //-- Show detected matches
        imshow("Matches", img_matches );
        waitKey(fps);
    }

    return matches;
}
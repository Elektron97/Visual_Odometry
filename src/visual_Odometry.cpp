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
bool showMatch = false;
bool showInlier = true;

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

/*STRUCTS*/
struct KeyPoint_Match
{
    vector<KeyPoint> Kpoints1;
    vector<KeyPoint> Kpoints2;
    vector<DMatch> match;
};

struct KpAsPoint2f_Match
{
    vector<Point2f> Kpoints1;
    vector<Point2f> Kpoints2;
    vector<DMatch> match;
};

/*FUNCTIONS DECLARATION*/
Mat ros2cv(sensor_msgs::CompressedImage image);
Mat get_image(Mat current_img, Mat cameraMatrix, Mat distortionCoeff); 
KeyPoint_Match detectAndMatchFeatures(Mat img1, Mat img2);
KpAsPoint2f_Match keyPoint2Point2f(KeyPoint_Match kp_match);
KeyPoint_Match point2f2keyPoint(KpAsPoint2f_Match kp_pnt2f);
void show_info(int outlier, int inlier, int keypoints_matched);
KpAsPoint2f_Match extract_Inlier(vector<Point2f> keypoints1_conv, vector<Point2f> keypoints2_conv, Mat cameraMatrix);
void show_inlier(KpAsPoint2f_Match inlier_match_p2f, Mat prev_img, Mat curr_img);
vector<Mat> estimateRelativePose(vector<Point2f> keypoints1_conv, vector<Point2f> keypoints2_conv, Mat cameraMatrix);
bool isRotationMatrix(Mat &R); //perche' &R?
Vec3f rotationMatrixToEulerAngles(Mat &R);
vector<Mat> cameraPoseToExtrinsic(Mat R_in, Mat t_in);
Mat projectionMatrix(Mat R, Mat t, Mat cameraIntrinsic);
float scaleFactor(float distance, Mat worldPoints);

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
    uint32_t prev_time = camera_sx.header.seq;

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

        KeyPoint_Match detect_match = detectAndMatchFeatures(prev_img, curr_img);

        /*POSE ESTIMATION*/
        KpAsPoint2f_Match kP_converted = keyPoint2Point2f(detect_match);

        vector<Mat> relativeTransf = estimateRelativePose(kP_converted.Kpoints1, kP_converted.Kpoints2, cameraMatrix);

        Mat R = relativeTransf[0];
        Mat t = relativeTransf[1];
        //Mat RANSAC_mask = relativeTransf[2]; Al momento inutile

        //Show Inlier
        show_inlier(extract_Inlier(kP_converted.Kpoints1, kP_converted.Kpoints2, cameraMatrix), prev_img, curr_img);

        //rotm2eul
        Vec3f euler_angles = rotationMatrixToEulerAngles(R);
    
        //NAV 2D
        //x, y:
        float x = t.at<float>(0);
        float y = t.at<float>(1);
        //yaw: 
        float yaw_angle = euler_angles(0);

        //getLastAvaibleAltitude
        float distance = laser.ranges[0]; //from MATLAB laser_msg{i, 1}.Ranges(1);
        
        /*TRIANGULATE POINTS AND ESTIMATE SCALE FACTOR*/
        //MATLAB cameraPoseExtrinsic
        //Vettori colonna NON riga!
        //World coordinates -> Camera coordinates
        
        //Camera Pose in World Frame
        Mat R_world = (Mat1d(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
        Mat t_world = (Mat1d(3, 1) << 0, 0, 0);

        vector<Mat> worldTransf = cameraPoseToExtrinsic(R_world, t_world);
        vector<Mat> currTransf = cameraPoseToExtrinsic(R, t); 

        //To do: cameraMatrix -> [Intrinsic]*[R | t]
        //perche' non usare projectPoints?

        Mat worldMatrix = projectionMatrix(worldTransf[0], worldTransf[1], cameraMatrix); 
        Mat currMatrix = projectionMatrix(currTransf[0], worldTransf[1], cameraMatrix); 

        cv::Mat world_points; //(4, leftInlier.size(), CV_64F);
        //triangulatePoints(worldMatrix, currMatrix, leftInlier, rightInlier, world_points);
        //Perche' fa tutto 0?

        //To do: Reprojection Error
        //Reietto worldpoints che hanno troppo
        //reprojection error

        /*---------------------------------------------------*/

        //Scale Factor
        /*float SF = scaleFactor(distance, world_points);
        
        x *= SF;
        y *= SF;*/

        //Linear Velocity Estimation
        uint32_t deltaT = camera_sx.header.seq - prev_time;
        //delta Position?
        //delta Euler?

        /*ABSOLUTE POSE*/





        //Update prev data
        prev_img = curr_img; 
        prev_time = camera_sx.header.seq;
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

KeyPoint_Match detectAndMatchFeatures(Mat img1, Mat img2)
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

    KeyPoint_Match output;
    output.Kpoints1 = keypoints1;
    output.Kpoints2 = keypoints2;
    output.match = matches;

    return output;
}

//Converto tenendo conto del match
KpAsPoint2f_Match keyPoint2Point2f(KeyPoint_Match kp_match) 
{
    // Convert keypoints into Point2f
    vector<Point2f> keypoints1_conv, keypoints2_conv;
    for (vector<DMatch>::const_iterator it= kp_match.match.begin(); it!= kp_match.match.end(); ++it) 
    {    
        // Get the position of keypoints1
        keypoints1_conv.push_back(kp_match.Kpoints1[it->queryIdx].pt); //query per keypoints1
        // Get the position of keypoints2
        keypoints2_conv.push_back(kp_match.Kpoints2[it->trainIdx].pt); //train per keypoints2
    }

    KpAsPoint2f_Match kp_p2f;
    kp_p2f.Kpoints1 = keypoints1_conv;
    kp_p2f.Kpoints2 = keypoints2_conv;
    kp_p2f.match = kp_match.match; //invariato

    return kp_p2f;
}

KeyPoint_Match point2f2keyPoint(KpAsPoint2f_Match kp_pnt2f)
{
    KeyPoint_Match kp_match;
    kp_match.match = kp_pnt2f.match;

    KeyPoint::convert(kp_pnt2f.Kpoints1, kp_match.Kpoints1);
    KeyPoint::convert(kp_pnt2f.Kpoints2, kp_match.Kpoints2);

    return kp_match;
}

void show_info(int outlier, int inlier, int keypoints_matched)
{
    ROS_INFO("********RANSAC ALGORITHM*********");
    ROS_INFO("Num. Keypoints Matched: %d", keypoints_matched);
    ROS_INFO("Num. Detected Outliers: %d", outlier);
    ROS_INFO("Num. Inlier: %d", inlier);
}

KpAsPoint2f_Match extract_Inlier(vector<Point2f> keypoints1_conv, vector<Point2f> keypoints2_conv, Mat cameraMatrix)
{
    vector<uchar> RANSAC_mask;
    Mat E = findEssentialMat(keypoints1_conv, keypoints2_conv, cameraMatrix, RANSAC, 0.999, 1.0, RANSAC_mask);
    //RANSAC_mask, vettore contenente N elementi (N = length(keypoints)) in cui indica:
    // 0 = outlier
    // 1 = inlier

    int outlierCount = 0;
    for(int i = 0; i < RANSAC_mask.size(); i++)
    {
        if(RANSAC_mask[i] == 0)
            outlierCount ++;

    }

    int inlierCount = RANSAC_mask.size() - outlierCount;

    //PROBLEMA: Troppi OUTLIER! Essential Matrix non esatta?
    if(showInlier)
        show_info(outlierCount, inlierCount, RANSAC_mask.size());

    // The above three variables are used to save the inner point and the matching relationship
    KpAsPoint2f_Match inlier_match_p2f;
    
    inlier_match_p2f.match.resize(inlierCount);
    inlier_match_p2f.Kpoints1.resize(inlierCount);
    inlier_match_p2f.Kpoints2.resize(inlierCount);

    inlierCount = 0;
    
    //Select Inlier -> C'e' un modo migliore!
    for (int i=0; i<RANSAC_mask.size(); i++)
    {
        if (RANSAC_mask[i] != 0)
        {
            inlier_match_p2f.Kpoints1[inlierCount] = keypoints1_conv.at(i);
            inlier_match_p2f.Kpoints2[inlierCount] = keypoints2_conv.at(i);
            inlier_match_p2f.match[inlierCount].queryIdx = inlierCount;
            inlier_match_p2f.match[inlierCount].trainIdx = inlierCount;
            inlierCount++;
        }
    }

    return inlier_match_p2f;

}

void show_inlier(KpAsPoint2f_Match inlier_match_p2f, Mat prev_img, Mat curr_img)
{
    // Convert the inner point to the format that drawMatches can use
    KeyPoint_Match inlier_match = point2f2keyPoint(inlier_match_p2f);

    //show inlier
    if(showInlier)
    {
        //-- Draw matches
        Mat img_matches_ransac;
        drawMatches( prev_img, inlier_match.Kpoints1, curr_img, inlier_match.Kpoints2, inlier_match.match, img_matches_ransac);
        //-- Show detected matches
        imshow("Matches after RANSAC", img_matches_ransac);
        waitKey(fps);
    }
}

vector<Mat> estimateRelativePose(vector<Point2f> keypoints1_conv, vector<Point2f> keypoints2_conv, Mat cameraMatrix)
{
    Mat R, t, RANSAC_mask;

    //default values: RANSAC, prob =0.999, threshold = 1.0
    Mat E = findEssentialMat(keypoints1_conv, keypoints2_conv, cameraMatrix, RANSAC, 0.999, 1.0, RANSAC_mask);
    recoverPose(E, keypoints1_conv, keypoints2_conv, cameraMatrix, R, t, RANSAC_mask);    

    vector<Mat> transf_inlier = {R, t, RANSAC_mask};
    return transf_inlier;
}

bool isRotationMatrix(Mat &R)
{
    // Checks if a matrix is a valid rotation matrix.

    Mat Rt;
    transpose(R, Rt);
    Mat shouldBeIdentity = Rt * R;
    Mat I = Mat::eye(3,3, shouldBeIdentity.type());

    return  norm(I, shouldBeIdentity) < 1e-6;

}

Vec3f rotationMatrixToEulerAngles(Mat &R)
{
    // Calculates rotation matrix to euler angles
    // The result is the same as MATLAB except the order
    // of the euler angles ( x and z are swapped ).

    assert(isRotationMatrix(R));

    float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );

    bool singular = sy < 1e-6; // If

    float x, y, z;
    if (!singular)
    {
        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    }
    else
    {
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }
    return Vec3f(x, y, z);

}

//triangulate section

vector<Mat> cameraPoseToExtrinsic(Mat R_in, Mat t_in)
{
    Mat R_out = R_in.t();
    Mat t_out = -R_in.t()*t_in;
    vector<Mat> output = {R_out, t_out};
    
    return output;
}

Mat projectionMatrix(Mat R, Mat t, Mat cameraIntrinsic)
{
    Mat R_t;
    hconcat(R, t, R_t);

    return cameraIntrinsic*R_t; 
}

float scaleFactor(float distance, Mat worldPoints)
{
    float Zsum = 0;
    int N = worldPoints.cols;

    //Calcolo la media
    for(int i = 0; i < N; i++)
    {
        Zsum += worldPoints.at<float>(2, i);
    } 
    
    float Zmean = Zsum / N;

    if(Zmean == 0)
    {
        //ROS_ERROR("Z pari a 0!");
        return 1.0; //per continuare il codice
    }

    else
        return distance/Zmean;
}
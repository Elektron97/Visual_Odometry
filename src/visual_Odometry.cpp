/***************VISUAL ODOMETRY*****************
* Stimare la posa da sequenze di Immagini.     *
***********************************************/


/**********************************************TOPICS AND MSGS***************************************************
*topics:      /drivers/altitude                                     msg: marta_msgs/Altitude                    *
*             /drivers/depth                                        msg: marta_msgs/Depth                       *   
*             /drivers/dvl                                          msg: marta_msgs/Dvl                         *
*             /drivers/imu                                          msg: marta_msgs/Imu                         *
*             /imu_compensated                                      msg: marta_msgs/Imu                         *
*             /nav_status                                           msg: marta_msgs/NavStatus                   *
*             /nav_status_ned_compensated                           msg: geometry_msgs/Point                    *
*             /pylon_camera/image_raw/compressed                    msg: sensor_msgs/CompressedImage            *
*****************************************************************************************************************/

/*INCLUDE*/
//library
#include "ros/ros.h"

//custom library
#include <visual_odometry/camera_utility.hpp>

/*ROS MSGS*/
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/PointCloud2.h"

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

//marta_msgs
#include "marta_msgs/Altitude.h"
#include "marta_msgs/Imu.h"

//custom msg
#include "visual_odometry/vo_results.h"
#include "visual_odometry/fail_check.h"

/*DEFINE*/
#define FIRST_IMAGE 1
#define FREQUENCY 10

/*NAMESPACE*/
using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;

/*ENUM*/
enum fail_V0 {FAIL_DETECTION, NOT_MOVING, FAIL_RECOVER, SUCCESS};

//Settings
bool motion2D = false;   //If true -> Planar motion: [x y yaw]

/*GLOBAL VARIABLES*/
//Intrinsic Parameters
double fx;
double fy;
double ccxLeft;
double ccyLeft;

//Distortion Coefficients
double k1;
double k2;
double p1;
double p2;

Mat Rbc; //Rot. Matrix from {C} -> {B}

//Sensor  
sensor_msgs::CompressedImage camera_sx;

//Ground Truth
nav_msgs::Odometry ground_truth;

geometry_msgs::Point nav_ned_compensated;
marta_msgs::Altitude altitude;
marta_msgs::Imu imu_obj;

//function declaration
void getCameraParam(ros::NodeHandle node_obj);
void updateGT(geometry_msgs::Point posGT);
void updateGT(marta_msgs::Imu imu);
//void updateGT(geometry_msgs::Point posGT, marta_msgs::Imu imu);
void print_VOresult(geometry_msgs::Vector3 estimate_pos, geometry_msgs::Vector3 estimate_rpy);
visual_odometry::vo_results publish_VOResults(Mat orientation, Mat location, Mat R, Mat t, double SF, ros::Duration deltaT);
visual_odometry::fail_check publish_FailCheck(int fail_succ);

/*CALLBACK*/
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

void navCompensated_callback(const geometry_msgs::Point::ConstPtr& msg)
{
    nav_ned_compensated.x = msg->x;
    nav_ned_compensated.y = msg->y;
    nav_ned_compensated.z = msg->z;

    updateGT(nav_ned_compensated);
}

void imu_callback(const marta_msgs::Imu::ConstPtr& msg)
{
    imu_obj.header = msg->header;
    imu_obj.orientation = msg->orientation;
    imu_obj.quaternion = msg->quaternion;
    imu_obj.angular_rate = msg->angular_rate;
    imu_obj.magnetic_field = msg->magnetic_field;
    imu_obj.acceleration = msg->acceleration;
    imu_obj.free_acceleration = msg->free_acceleration;

    updateGT(imu_obj);
}

void altitude_callback(const marta_msgs::Altitude::ConstPtr& msg)
{
    altitude.header = msg->header;
    altitude.altitude = msg->altitude;
    altitude.validity = msg->validity;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "visual_Odometry");
	ros::NodeHandle node_obj;

    //Pub Object
    ros::Publisher pub_results = node_obj.advertise<visual_odometry::vo_results>("VO_results", 10);
    ros::Publisher pub_fail = node_obj.advertise<visual_odometry::fail_check>("VO_fail_check", 10);
    ros::Publisher pub_pcl = node_obj.advertise<sensor_msgs::PointCloud2>("/world_points", 10);
    
	ros::Subscriber sub_cameraSX = node_obj.subscribe("/pylon_camera/image_raw/compressed", 1, cameraSX_callback);

    ros::Subscriber sub_NavNed = node_obj.subscribe("/nav_status_ned_compensated", 1, navCompensated_callback);
    ros::Subscriber sub_IMU = node_obj.subscribe("/imu_compensated", 1, imu_callback);
    ros::Subscriber sub_Altitude = node_obj.subscribe("/drivers/altitude", 1, altitude_callback);

	ros::Rate loop_rate(FREQUENCY);	//10 Hz Prediction step

    /*Aspetto la FIRST_IMAGE*/
    ROS_INFO("Waiting %d-th frame...", FIRST_IMAGE);
    
    while(camera_sx.header.seq <= FIRST_IMAGE ) //aspetto le prime immagini
    {
        ros::spinOnce();
    }

    ROS_WARN("START!");

    /*INITIALIZATION*/
    //NED matrix rotation
    Rbc = (Mat1d(3, 3) << 0, -1, 0, 1, 0, 0, 0, 0, 1); //{Camera} -> {Body}

    //Load Camera Intrinsic from Param Server
    getCameraParam(node_obj);

    //Intrinsic Parameters
    Mat cameraMatrix = (Mat1d(3, 3) << fx, 0, ccxLeft, 0, fy, ccyLeft, 0, 0, 1);
    Mat distortionCoeff = (Mat1d(1, 4) << k1, k2, p1, p2);

    //Undistort Image
    Mat prev_img = get_image(ros2cv(camera_sx), cameraMatrix, distortionCoeff);
    ros::Time prev_time = camera_sx.header.stamp;

    //Inizializzo le Trasformazioni dal GT -> AbsPose
    //convert quaternion in Rotational Matrix
    Mat orientation_body = quat2Mat(ground_truth.pose.pose.orientation); //{Body_k-1} -> {ENU}
    Mat orientation = orientation_body*Rbc; //{Camera_k-1} -> {ENU} 
    Mat location = pos2Mat(ground_truth.pose.pose.position); //trasl {ENU} -> {Body} in frame {ENU}

    //Relative Orientation and Location
    Mat R = Mat::eye(3, 3, CV_64F); //{k-1} -> {k}
    Mat t = Mat::zeros(3, 1, CV_64F); //{k} -> {k-1} in frame {k}. ||t|| = 1

    //Scale Factor
    double SF = 1.0; //default value
    
    //World Points in {W} coordinates
    Mat world_pointsW;

    //deltaT for twist
    ros::Duration deltaT;

    int fail_succ; //Check fail state

    /*ITERATIONS*/
    while(ros::ok())
    {
        ros::spinOnce();    //Read Sensor Data
        loop_rate.sleep();

        /*SHOW IMAGE FROM BAG FILE*/
        if(showFrame)
        {
            namedWindow("Resized Image", WINDOW_AUTOSIZE);
            imshow("Resized Image", desiredResize(ros2cv(camera_sx)));
            waitKey(fps);
        }

        /*FEATURE MATCHING*/
        Mat curr_img = get_image(ros2cv(camera_sx), cameraMatrix, distortionCoeff);

        /*namedWindow("Undistorted Image", WINDOW_AUTOSIZE);
        imshow("Undistorted Image", curr_img);
        waitKey(fps);*/

        KeyPoint_Match detect_match = detectAndMatchFeatures(prev_img, curr_img);

        /*POSE ESTIMATION*/
        KpAsPoint2f_Match kP_converted = keyPoint2Point2f(detect_match);

        //ROS_INFO("N Key Point Matched");
        //cout << kP_converted.Kpoints1.size() << endl;

        if(checkMinFeat(kP_converted))
        {
            ROS_ERROR("Num. Features sotto il minimo. Skip Iteration!");
            visual_odometry::fail_check fail_msg = publish_FailCheck(FAIL_DETECTION);
            pub_fail.publish(fail_msg);
            continue;
        }      

        if(!checkIfMoving(kP_converted))
        {
            ROS_WARN("Robot is not moving! Skip Iteration!");
            visual_odometry::fail_check fail_msg = publish_FailCheck(NOT_MOVING);
            pub_fail.publish(fail_msg);
            continue;
        } 

        else
            ROS_INFO("Robot is moving! Estimating pose...");


        RelativePose rel_pose = estimateRelativePose(kP_converted, cameraMatrix);

        KpAsPoint2f_Match inlier_converted = rel_pose.inlier_points;

        show_inlier(inlier_converted, prev_img, curr_img); //if showInlier == true 

        if(rel_pose.success)
        {
            R = rel_pose.R;
            t = rel_pose.t;
            fail_succ = SUCCESS;
        }

        else
        {
            ROS_ERROR("FAILED RECOVERING POSE");
            //R_k-1 == R_k
            //t_k-1 == t_k
            fail_succ = FAIL_RECOVER;
        }

        /******NOTA SULLA TRASF. OMOGENEA********
         * R e t sono gli elementi della trasf. *
         * omogenea da {k-1} a {k}.             *
         * Dunque, un vettore espresso in {k-1} *
         * viene rototraslato nel frame {k}.    *
         ****************************************/

        if(motion2D)
        {
            //NAV 2D
            t.at<double>(2) = 0.0;
            //rotm2eul
            Vec3f euler_angles = rotationMatrixToEulerAngles(R);
            //yaw 
            double yaw_angle = euler_angles(0);

            R = rotz(yaw_angle);
        }

        //getLastAvaibleAltitude
        float distance = altitude.altitude;
        
        /*TRIANGULATE POINTS AND ESTIMATE SCALE FACTOR*/
        Mat world_points;
        
        //Salto triangPoints se success e' false
        if(rel_pose.success)
        {
            world_points = triangPoints(inlier_converted.Kpoints1, inlier_converted.Kpoints2, R, t, cameraMatrix);
            if(!world_points.empty())
                SF = scaleFactor(distance, world_points);   //Update Scale Factor

            //else -> SF_k == SF_k-1
        }

        //else
            //SF_k == SF_k-1;

        /*ABSOLUTE POSE*/
        if(rel_pose.success)
        {
            vector<Mat> absPose = absolutePose(orientation, location, R, t, SF, world_points);
        
            world_pointsW = absPose[2]; //[wp]^W
            location = absPose[0];      //[t_w,k]^W
            orientation = absPose[1];   //R_wk ({k} -> {W})
        }

        else
        {
            vector<Mat> absPose = absolutePose(orientation, location, R, t, SF);

            location = absPose[0];      //[t_w,k]^W
            orientation = absPose[1];   //R_wk ({k} -> {W})
        }

        if(motion2D)
            location.at<double>(2) = ground_truth.pose.pose.position.z; //uso il GT per la profondita'

        /*VELOCITY ESTIMATION*/
        //deltaT
        ros::Time curr_time = camera_sx.header.stamp;
        deltaT = curr_time - prev_time;

        visual_odometry::vo_results results = publish_VOResults(orientation, location, R, t, SF, deltaT);
        pub_results.publish(results);

        //Pub Fail Check
        pub_fail.publish(publish_FailCheck(fail_succ));

        /*PUBLISH WORLD POINTS AS POINT CLOUD*/
        if(rel_pose.success)
        {
            //creating cloud object
            pcl::PointCloud<pcl::PointXYZ> cloud;

            //creating pc2 object
            sensor_msgs::PointCloud2 wp_cloud;

            cloud.points.resize(world_pointsW.cols);

            for(int i = 0; i < world_pointsW.cols; i++)
            {
                cloud.points[i].x = world_pointsW.at<double>(0, i);
                cloud.points[i].y = world_pointsW.at<double>(1, i);
                cloud.points[i].z = world_pointsW.at<double>(2, i);
            }

            pcl::toROSMsg(cloud, wp_cloud);
            wp_cloud.header.frame_id = "world_points";

            pub_pcl.publish(wp_cloud);
        }

        /*UPDATE PREV DATA*/
        prev_img = curr_img; 
        prev_time = curr_time;
    }
    ROS_WARN("Video Finito!");

    return 0;
}

void getCameraParam(ros::NodeHandle node_obj)
{
    //Intrinsic Matrix
    node_obj.getParam("/camera_intrinsic/fx", fx);
    node_obj.getParam("/camera_intrinsic/fy", fy);
    node_obj.getParam("/camera_intrinsic/ccxLeft", ccxLeft);
    node_obj.getParam("/camera_intrinsic/ccyLeft", ccyLeft);

    //Distortion Coefficients
    node_obj.getParam("/distortion_coefficient/radial/k1", k1);
    node_obj.getParam("/distortion_coefficient/radial/k2", k2);
    node_obj.getParam("/distortion_coefficient/tangential/p1", p1);
    node_obj.getParam("/distortion_coefficient/tangential/p2", p2);
}

void updateGT(geometry_msgs::Point posGT)
{
    /*Position Ground Truth from nav_compensated*/
    ground_truth.pose.pose.position = posGT;
}

void updateGT(marta_msgs::Imu imu)
{
    /*Orientation and Twist Ground Truth from Imu*/
    //Compute deltaT for Integration
    ros::Duration integrationTime = imu.header.stamp - ground_truth.header.stamp;

    //then update GT Header
    ground_truth.header = imu.header;

    //Quaternion
    ground_truth.pose.pose.orientation.w = imu.quaternion.w;
    ground_truth.pose.pose.orientation.x = imu.quaternion.x;
    ground_truth.pose.pose.orientation.y = imu.quaternion.y;
    ground_truth.pose.pose.orientation.z = imu.quaternion.z;

    //Angular Rate
    ground_truth.twist.twist.angular = imu.angular_rate;

    //Linear Velocity: Integration of Acceleration
    //v(k) = v(k-1) + T*a(k)
    ground_truth.twist.twist.linear.x = ground_truth.twist.twist.linear.x + (integrationTime.toSec())*imu.acceleration.x;
    ground_truth.twist.twist.linear.y = ground_truth.twist.twist.linear.y + (integrationTime.toSec())*imu.acceleration.y;
    ground_truth.twist.twist.linear.z = ground_truth.twist.twist.linear.z + (integrationTime.toSec())*imu.acceleration.z;
}

/*void updateGT(geometry_msgs::Point posGT, marta_msgs::Imu imu)
{
    //Position Ground Truth from nav_compensated
    ground_truth.pose.pose.position = posGT;

    //Orientation and Twist Ground Truth from Imu
    //Compute deltaT for Integration
    ros::Duration integrationTime = imu.header.stamp - ground_truth.header.stamp;

    //then update GT Header
    ground_truth.header = imu.header;

    //Quaternion
    ground_truth.pose.pose.orientation.w = imu.quaternion.w;
    ground_truth.pose.pose.orientation.x = imu.quaternion.x;
    ground_truth.pose.pose.orientation.y = imu.quaternion.y;
    ground_truth.pose.pose.orientation.z = imu.quaternion.z;

    //Angular Rate
    ground_truth.twist.twist.angular = imu.angular_rate;

    //Linear Velocity: Integration of Acceleration
    //v(k) = v(k-1) + T*a(k)
    ground_truth.twist.twist.linear.x = ground_truth.twist.twist.linear.x + (integrationTime.toSec())*imu.acceleration.x;
    ground_truth.twist.twist.linear.y = ground_truth.twist.twist.linear.y + (integrationTime.toSec())*imu.acceleration.y;
    ground_truth.twist.twist.linear.z = ground_truth.twist.twist.linear.z + (integrationTime.toSec())*imu.acceleration.z;
}*/

visual_odometry::vo_results publish_VOResults(Mat orientation, Mat location, Mat R, Mat t, double SF, ros::Duration deltaT)
{
    /**********PUBLISH VO RESULTS************
     * orientation/location: Absolute Pose  *
     * R/t: Relative Pose                   *
     * SF: Scale Factor                     *
     * deltaT: diff. of time for twist      *
     ****************************************/

    //Using 2 Global Variables
    //>Rbc: Rot. Matrix from {C} to {B}
    //>motion2D: true: planar motion | false: 3D motion
    //>ground_truth: ground truth data

    visual_odometry::vo_results results;

    //Estimate Pose and Twist
    geometry_msgs::Vector3 estimate_rpy = mat2Euler(orientation*Rbc.t()); //{b} -> {W} (istante k)
    geometry_msgs::Vector3 estimate_pos = mat2Vec3(location);

    print_VOresult(estimate_pos, estimate_rpy);

    geometry_msgs::Twist estimate_twist = estimateTwist(deltaT, R, t, SF, estimate_rpy, Rbc, motion2D);

    geometry_msgs::Point GTpos = ground_truth.pose.pose.position; //{ENU} -> {Body} (istante k)
    geometry_msgs::Vector3 GTrpy = quat2Euler(ground_truth.pose.pose.orientation); //{Body} -> {ENU} (istante k)
    geometry_msgs::Twist GTtwist = ground_truth.twist.twist;

    results.estimate_pos = estimate_pos;
    results.estimate_rpy = estimate_rpy;
    results.estimate_twist = estimate_twist;

    results.error_pos = absDiff_Vec3(GTpos, estimate_pos);
    results.error_rpy = absDiff_Vec3(GTrpy, estimate_rpy, true);
    results.error_twist.linear = absDiff_Vec3(GTtwist.linear, estimate_twist.linear);
    results.error_twist.angular = absDiff_Vec3(GTtwist.angular, estimate_twist.angular);

    results.SF = SF;

    return results;
}

visual_odometry::fail_check publish_FailCheck(int fail_succ)
{
    visual_odometry::fail_check fail_msg;

    switch(fail_succ)
    {
        case FAIL_DETECTION:
            fail_msg.fail_detect = true;
            fail_msg.not_moving = false;
            fail_msg.fail_pose = false;
            fail_msg.success = false;
        break;

        case NOT_MOVING:
            fail_msg.fail_detect = false;
            fail_msg.not_moving = true;
            fail_msg.fail_pose = false;
            fail_msg.success = false;
        break;

        case FAIL_RECOVER:
            fail_msg.fail_detect = false;
            fail_msg.not_moving = false;
            fail_msg.fail_pose = true;
            fail_msg.success = false;
        break;

        case SUCCESS:
            fail_msg.fail_detect = false;
            fail_msg.not_moving = false;
            fail_msg.fail_pose = false;
            fail_msg.success = true;
        break;
    }

    return fail_msg;
}

void print_VOresult(geometry_msgs::Vector3 estimate_pos, geometry_msgs::Vector3 estimate_rpy)
{
    ROS_WARN("************************************Visual Odometry**************************************");
    ROS_INFO("Estimate Position:        [x:     %f, \t y:      %f, \t z:   %f]", estimate_pos.x, estimate_pos.y, estimate_pos.z);
    ROS_INFO("Estimate RPY:             [roll:  %f, \t pitch:  %f, \t yaw: %f ]", estimate_rpy.x, estimate_rpy.y, estimate_rpy.z);
    ROS_INFO("-----------------------------------------------------------------------------------------");
}
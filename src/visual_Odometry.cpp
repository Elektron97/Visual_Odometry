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

/*INCLUDE*/
//library
#include "ros/ros.h"

//custom library
#include <visual_odometry/camera_utility.hpp>

/*ROS MSGS*/
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/PointCloud2.h"

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

//custom msg
#include "visual_odometry/vo_results.h"

/*DEFINE*/
#define FIRST_IMAGE 1
#define FREQUENCY 10

/*NAMESPACE*/
using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;

Mat Rbc; //Matrice {Camera} -> {Body}

//Settings
bool motion2D = true;   //Planar motion: [x y yaw]

/*GLOBAL VARIABLES*/
sensor_msgs::Imu imu;   
sensor_msgs::LaserScan laser;
sensor_msgs::CompressedImage camera_sx;
sensor_msgs::CompressedImage camera_dx;

nav_msgs::Odometry ground_truth;

//function declaration
void print_VOresult(geometry_msgs::Vector3 estimate_pos, geometry_msgs::Vector3 estimate_rpy, geometry_msgs::Point GTpos, geometry_msgs::Vector3 GTrpy);

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
    /********************************************
    *std_msgs/Header header                     *
    *  uint32 seq                               *
    *  time stamp                               *
    *  string frame_id                          *
    *string child_frame_id                      *
    *geometry_msgs/PoseWithCovariance pose      *
    *  geometry_msgs/Pose pose                  *
    *    geometry_msgs/Point position           *
    *      float64 x                            *
    *      float64 y                            *
    *      float64 z                            *
    *    geometry_msgs/Quaternion orientation   *
    *      float64 x                            *
    *      float64 y                            *
    *      float64 z                            *
    *      float64 w                            *
    *  float64[36] covariance                   *
    *geometry_msgs/TwistWithCovariance twist    *
    *  geometry_msgs/Twist twist                *
    *    geometry_msgs/Vector3 linear           *
    *      float64 x                            *
    *      float64 y                            *
    *      float64 z                            *
    *    geometry_msgs/Vector3 angular          *
    *      float64 x                            *
    *      float64 y                            *
    *      float64 z                            *
    *  float64[36] covariance                   *
    *********************************************/

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
    ros::Publisher pub_pcl = node_obj.advertise<sensor_msgs::PointCloud2>("/world_points", 10);

    ros::Publisher pub_results = node_obj.advertise<visual_odometry::vo_results>("VO_results", 10);

    //Sub Objects
	ros::Subscriber sub_imu=node_obj.subscribe("/zeno/imu", 1, imu_callback);
	ros::Subscriber sub_laser=node_obj.subscribe("/zeno/laser", 1, laser_callback);

    /*ros::Subscriber risulta meno efficente di image_transport.*/
    //http://wiki.ros.org/compressed_image_transport

    //image_transport::ImageTransport it(node_obj);
    //image_transport::Subscriber sub_cameraSX = it.subscribe("/zeno/zeno/cameraleft/camera_image/compressed", 1, cameraSX_callback);
    //image_transport::Subscriber sub_cameraDX = it.subscribe("/zeno/zeno/cameraright/camera_image/compressed", 1, cameraDX_callback);
    
	ros::Subscriber sub_cameraSX=node_obj.subscribe("/zeno/zeno/cameraleft/camera_image/compressed", 1, cameraSX_callback);
    ros::Subscriber sub_cameraDX=node_obj.subscribe("/zeno/zeno/cameraright/camera_image/compressed", 1, cameraDX_callback);

	ros::Subscriber sub_GT=node_obj.subscribe("/zeno/pose_gt", 1, groundTruth_callback);

	ros::Rate loop_rate(FREQUENCY);	//10 Hz Prediction step

    /*Aspetto la FIRST_IMAGE*/
    ROS_INFO("Waiting %d-th frame...", FIRST_IMAGE);
    
    while(camera_sx.header.seq <= FIRST_IMAGE ) //aspetto le prime immagini
    {
        ros::spinOnce();
    }

    ROS_WARN("START!");

    /*INITIALIZATION*/
    //ENU matrix rotation
    Rbc = (Mat1d(3, 3) << 0, 0, 1, -1, 0, 0, 0, -1, 0); //{Camera} -> {Body}

    //Intrinsic Parameters
    Mat cameraMatrix = (Mat1d(3, 3) << fx, 0, ccxLeft, 0, fy, ccyLeft, 0, 0, 1);
    Mat distortionCoeff = (Mat1d(1, 4) << k1, k2, p1, p2);

    //Undistort Image
    Mat prev_img = get_image(ros2cv(camera_sx), cameraMatrix, distortionCoeff);
    ros::Time prev_time = camera_sx.header.stamp;

    //Inizializzo le Trasformazioni dal GT -> AbsPose
    //convert quaternion in Rotational Matrix
    Mat orientation_body = quat2Mat(ground_truth.pose.pose.orientation); //{Body_k-1} -> {Camera_k-1}
    Mat orientation = orientation_body*Rbc; //{Camera_k-1} -> {ENU} 
    Mat location = pos2Mat(ground_truth.pose.pose.position); //trasl {ENU} -> {Body} in frame {ENU}
 
    //boolean variables for fail detection
    bool fail_detection = false; //Quando il n° di features e' minore della tolleranza

    //Relative Orientation and Location
    Mat R = Mat::eye(3, 3, CV_64F); //{k-1} -> {k}
    Mat t = Mat::zeros(3, 1, CV_64F); //{k} -> {k-1} in frame {k}. ||t|| = 1

    //Scale Factor
    double SF = 1.0; //default value
    
    //World Points in {W} coordinates
    Mat world_pointsW;

    /*ITERATIONS*/
    while(ros::ok())
    {
        ros::spinOnce();    //Read Sensor Data
        loop_rate.sleep();

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

        fail_detection = checkMinFeat(kP_converted);
        
        if(fail_detection)
        {
            ROS_ERROR("Num. Features sotto il minimo. Skip Iteration!");
            continue;
        }      

        if(!checkIfMoving(kP_converted))
        {
            ROS_WARN("Robot is not moving! Skip Iteration!");
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
        }

        else
        {
            ROS_ERROR("FAILED RECOVERING POSE");
            //R_k-1 == R_k
            //t_k-1 == t_k
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
        float distance = laser.ranges[0]; //from MATLAB laser_msg{i, 1}.Ranges(1);
        
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

        /*VELOCITY ESTIMATION*/
        //deltaT
        ros::Time curr_time = camera_sx.header.stamp;
        ros::Duration deltaT = curr_time - prev_time;

        //linear velocity
        Mat estimate_vel = SF*t/deltaT.toSec();

        //angular velocity
        double deltaPsi = atan2(R.at<double>(1, 0), R.at<double>(0, 0)); //deltaYaw = atan2(sin(yaw), cos(yaw))
        double omega = deltaPsi/deltaT.toSec();  //True only in motion2D

        Mat estimate_ang = (Mat1d(3, 1) << 0, 0, omega);

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

        /*PUBLISH*/
        geometry_msgs::Vector3 estimate_rpy = mat2Euler(orientation*Rbc.t()); //{b} -> {W} (istante k)
        geometry_msgs::Vector3 estimate_pos = mat2Vec3(location);

        geometry_msgs::Twist estimate_twist;
        estimate_twist.linear = mat2Vec3(estimate_vel);
        estimate_twist.angular = mat2Vec3(estimate_ang);

        geometry_msgs::Point GTpos = ground_truth.pose.pose.position; //{ENU} -> {Body} (istante k)
        geometry_msgs::Vector3 GTrpy = quat2Euler(ground_truth.pose.pose.orientation); //{Body} -> {ENU} (istante k)
        geometry_msgs::Twist GTtwist = ground_truth.twist.twist;

        /*PUBLISH ERROR*/
        visual_odometry::vo_results results;

        results.estimate_pos = estimate_pos;
        results.estimate_rpy = estimate_rpy;
        results.estimate_twist = estimate_twist;

        results.error_pos = absDiff_Vec3(GTpos, estimate_pos);
        results.error_rpy = absDiff_Vec3(GTrpy, estimate_rpy);
        results.error_twist.linear = absDiff_Vec3(GTtwist.linear, estimate_twist.linear);
        results.error_twist.angular = absDiff_Vec3(GTtwist.angular, GTtwist.angular);

        pub_results.publish(results);

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

        /*SHOW RESULTS*/
        print_VOresult(estimate_pos, estimate_rpy, GTpos, GTrpy);

        /*UPDATE PREV DATA*/
        prev_img = curr_img; 
        prev_time = curr_time;
    }
    ROS_WARN("Video Finito!");

    return 0;
}

void print_VOresult(geometry_msgs::Vector3 estimate_pos, geometry_msgs::Vector3 estimate_rpy, geometry_msgs::Point GTpos, geometry_msgs::Vector3 GTrpy)
{
    ROS_WARN("************************************Visual Odometry**************************************");
    ROS_INFO("Estimate Position:        [x:     %f, \t y:      %f, \t z:   %f]", estimate_pos.x, estimate_pos.y, estimate_pos.z);
    ROS_INFO("Estimate RPY:             [roll:  %f, \t pitch:  %f, \t yaw: %f ]", estimate_rpy.x, estimate_rpy.y, estimate_rpy.z);
    ROS_INFO("-----------------------------------------------------------------------------------------");
    ROS_INFO("Ground Truth Position:    [x:     %f, \t y:      %f, \t z:   %f]", GTpos.x, GTpos.y, GTpos.z);
    ROS_INFO("Ground Truth RPY:         [roll:  %f, \t pitch:  %f, \t yaw: %f ]\n\n", GTrpy.x, GTrpy.y, GTrpy.z);
}
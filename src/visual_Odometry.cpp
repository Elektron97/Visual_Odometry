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

//msg for debug 
#include "geometry_msgs/Pose2D.h"

/*DEFINE*/
#define FIRST_IMAGE 1
#define viewId_stop 810 //da modificare
#define FREQUENCY 10

/*NAMESPACE*/
using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;

Mat Rbc; //Matrice {Body} -> {Camera}

//Settings
bool jumpCond = true;
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
    ros::Publisher pub_err = node_obj.advertise<geometry_msgs::Vector3>("/error_pos", 10);
    ros::Publisher pub_pcl = node_obj.advertise<sensor_msgs::PointCloud2>("/world_points", 10);

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
    Rbc = (Mat1d(3, 3) << 0, 0, 1, -1, 0, 0, 0, -1, 0); //Body -> Camera

    //Intrinsic Parameters
    Mat cameraMatrix = (Mat1d(3, 3) << fx, 0, ccxLeft, 0, fy, ccyLeft, 0, 0, 1);
    Mat distortionCoeff = (Mat1d(1, 4) << k1, k2, p1, p2);

    //Undistort Image
    Mat prev_img = get_image(ros2cv(camera_sx), cameraMatrix, distortionCoeff);
    uint32_t prev_time = camera_sx.header.seq;

    //Inizializzo le Trasformazioni dal GT -> AbsPose
    //convert quaternion in Rotational Matrix
    Mat orientation_body = quat2Mat(ground_truth.pose.pose.orientation); //ENU -> Body (all'istante k-1)
    Mat orientation = Rbc*orientation_body; //ENU -> Camera (all'istante k-1)
    Mat location = pos2Mat(ground_truth.pose.pose.position);

    //boolean variables for fail detection
    bool fail_detection = false; //Quando il n di features e' minore della tolleranza
    
    /*ITERATIONS*/
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();	

        //if(camera_sx.header.seq > viewId_stop)
        //    break;

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
        
        vector<uchar> RANSAC_mask;
        Mat E = findEssentialMat(kP_converted.Kpoints1, kP_converted.Kpoints2, cameraMatrix, RANSAC, 0.999, 1.0, RANSAC_mask);

        //extract Inlier
        KpAsPoint2f_Match inlier_converted = extract_Inlier(kP_converted.Kpoints1, kP_converted.Kpoints2, RANSAC_mask); 

        //Show Inlier
        show_inlier(inlier_converted, prev_img, curr_img);

        if(!checkIfMoving(inlier_converted))
        {
            //ROS_WARN("Skip Iteration!");
            continue;
        } 

        else
            ROS_INFO("Robot is moving! Estimating pose..."); 

        Mat R, t;
        //finally, recoverPose()
        //recoverPose(E, kP_converted.Kpoints1, kP_converted.Kpoints2, cameraMatrix, R, t, RANSAC_mask);
        recoverPose(E, inlier_converted.Kpoints1, inlier_converted.Kpoints2, cameraMatrix, R, t);
        //cout << t << endl;

        /*************NOTA SU R, t***************
         * currFrame = k; prevFrame = k-1       *
         * [...]^k -> espresso in coordinate k  *
         *                                      *
         * R: {k-1} -> {k}                      *
         * t: {k-1 -> k}^(k-1)                  *
         * **************************************/

        if(motion2D)
        {
            //NAV 2D
            t.at<double>(2) = 0.0;

            //rotm2eul
            Vec3f euler_angles = rotationMatrixToEulerAngles(R);
            //yaw: 
            double yaw_angle = euler_angles(0);

            R = rotz(yaw_angle);
        }

        //getLastAvaibleAltitude
        float distance = laser.ranges[0]; //from MATLAB laser_msg{i, 1}.Ranges(1);
        
        /*TRIANGULATE POINTS AND ESTIMATE SCALE FACTOR*/
        Mat world_points = triangPoints(inlier_converted.Kpoints1, inlier_converted.Kpoints2, R, t, cameraMatrix);

        //Scale Factor
        double SF = scaleFactor(distance, world_points);

        /*VELOCITY ESTIMATION*/
        //deltaT
        uint32_t curr_time = camera_sx.header.seq;
        uint32_t deltaT = curr_time - prev_time;

        //linear velocity
        Mat estimate_vel = SF*t/deltaT;

        //angular velocity
        double deltaPsi = atan2(R.at<double>(0, 0), R.at<double>(0, 1));
        double estimate_ang = deltaPsi/deltaT;

        /*ABSOLUTE POSE*/
        vector<Mat> absPose = absolutePose(orientation, location, R, t, SF, world_points);
        
        Mat world_pointsW = absPose[2];
        location = absPose[0];
        orientation = absPose[1];

        if(motion2D)
            location.at<double>(2) = ground_truth.pose.pose.position.z; //uso il GT per la profondita'

        geometry_msgs::Vector3 estimate_rpy = mat2Euler(orientation);
        geometry_msgs::Vector3 estimate_pos;
        estimate_pos.x = location.at<double>(0);
        estimate_pos.y = location.at<double>(1);
        estimate_pos.z = location.at<double>(2);

        geometry_msgs::Point GTpos = ground_truth.pose.pose.position;
        geometry_msgs::Vector3 GTrpy = mat2Euler(Rbc*quat2Mat(ground_truth.pose.pose.orientation));

        /*PUBLISH ERROR*/
        geometry_msgs::Vector3 error_pos;
        error_pos.x = abs(GTpos.x - estimate_pos.x);
        error_pos.y = abs(GTpos.y - estimate_pos.y);
        error_pos.z = abs(GTpos.z - estimate_pos.z);

        pub_err.publish(error_pos);

        /*PUBLISH WORLD POINTS AS POINT CLOUD*/
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

        /*SHOW RESULTS*/
        //print_VOresult(estimate_pos, estimate_rpy, GTpos, GTrpy);

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
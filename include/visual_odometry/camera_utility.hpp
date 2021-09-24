/*CAMERA UTILITY*/

/*INCLUDE*/
#include "ros/ros.h"
#include <iostream>
#include <opencv2/opencv.hpp> 
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/tf.h>

/*ROS MSGS*/
#include "sensor_msgs/CompressedImage.h"

/*NAMESPACES*/
using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;

/*CONSTANTS*/
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
bool showFrame = true;
bool showMatch = false;
bool showInlier= false;

//SURF parameters
int minHessian = 100;

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

/*FUNCTIONS*/
/*********Declaration**********/

//Ros - OpenCV interface
Mat ros2cv(sensor_msgs::CompressedImage image);

//Math Utility -> Another Library
Mat rotz(double angle);
Mat tf2Mat(tf::Matrix3x3 m);
tf::Matrix3x3 Mat2tf(Mat m);
Mat quat2Mat(geometry_msgs::Quaternion quat);
geometry_msgs::Vector3 mat2Euler(Mat R);
geometry_msgs::Vector3 quat2Euler(geometry_msgs::Quaternion quat);
Mat pos2Mat(geometry_msgs::Point pos);
Mat coordTransf(Mat vector, Mat R, Mat t);

//Detect and Match Features
Mat get_image(Mat current_img, Mat cameraMatrix, Mat distortionCoeff); 
KeyPoint_Match detectAndMatchFeatures(Mat img1, Mat img2);

//Structs Interface
KpAsPoint2f_Match keyPoint2Point2f(KeyPoint_Match kp_match);
KeyPoint_Match point2f2keyPoint(KpAsPoint2f_Match kp_pnt2f);

//Inlier
void show_info(int outlier, int inlier, int keypoints_matched);
KpAsPoint2f_Match extract_Inlier(vector<Point2f> keypoints1_conv, vector<Point2f> keypoints2_conv, vector<uchar> RANSAC_mask);
void show_inlier(KpAsPoint2f_Match inlier_match_p2f, Mat prev_img, Mat curr_img);

//Euler and Rotm
bool isRotationMatrix(Mat &R); 
Vec3f rotationMatrixToEulerAngles(Mat &R);

//Triangulate Position and Scale Factor
vector<Mat> cameraPoseToExtrinsic(Mat R_in, Mat t_in);
Mat projectionMatrix(Mat R, Mat t, Mat cameraIntrinsic);
Mat triangPoints(vector<Point2f> keypoints1_conv, vector<Point2f> keypoints2_conv, Mat R, Mat t, Mat cameraMatrix);
vector<double> reproject_error(Mat world_points, Mat R, Mat t, Mat cameraMatrix, vector<Point2f> img_points);
double scaleFactor(float distance, Mat worldPoints);

//Absolute Pose
vector<Mat> absolutePose(Mat rotm, Mat tran, Mat orient, Mat loc, double SF, Mat world_points);

/*********Source**********/
//TO DO -> Metterli in un .cpp?

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

Mat rotz(double angle)
{
    /*To do: commenta come e' costruita la mat*/
    return (Mat1d(3, 3) << cos(angle), -sin(angle), 0, sin(angle), cos(angle), 0, 0, 0, 1);
}

Mat tf2Mat(tf::Matrix3x3 m)
{
    Mat rotm_mat(3, 3, CV_64F);

    for(int i = 0; i < 3; i++)
    {
        tf::Vector3 test = m.getRow(i);

        rotm_mat.at<double>(i, 0) = test.getX();
        rotm_mat.at<double>(i, 1) = test.getY();
        rotm_mat.at<double>(i, 2) = test.getZ();
    }

    return rotm_mat;
}

tf::Matrix3x3 Mat2tf(Mat m)
{
    tf::Matrix3x3 tf_matrix(m.at<double>(0, 0), m.at<double>(0, 1), m.at<double>(0, 2), 
                            m.at<double>(1, 0), m.at<double>(1, 1), m.at<double>(1, 2), 
                            m.at<double>(2, 0), m.at<double>(2, 1), m.at<double>(2, 2));
    return tf_matrix;
}

Mat quat2Mat(geometry_msgs::Quaternion quat)
{
    tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf::Matrix3x3 m(q);

    return tf2Mat(m);
}

geometry_msgs::Vector3 mat2Euler(Mat R)
{
    tf::Matrix3x3 rotm = Mat2tf(R);
    geometry_msgs::Vector3 rpy;
    
    //conversion in RPY
    rotm.getRPY(rpy.x, rpy.y, rpy.z);

    return rpy;
}

geometry_msgs::Vector3 quat2Euler(geometry_msgs::Quaternion quat)
{
    tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf::Matrix3x3 m(q);

    geometry_msgs::Vector3 rpy;
    
    //conversion in RPY
    m.getRPY(rpy.x, rpy.y, rpy.z);

    return rpy;
}

Mat pos2Mat(geometry_msgs::Point pos)
{
    Mat pos_mat = (Mat1d(3, 1) << pos.x, pos.y, pos.z);
    return pos_mat;
}

Mat coordTransf(Mat vector, Mat R, Mat t)
{
    return R*vector + t;
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

KpAsPoint2f_Match extract_Inlier(vector<Point2f> keypoints1_conv, vector<Point2f> keypoints2_conv, vector<uchar> RANSAC_mask)
{
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

    /****************************
    * Soluzione piu' elegante   *
    *****************************/

   /*  vector<cv::Point2f> inlier_match_points1, inlier_match_points2;
  for(int i = 0; i < mask.rows; i++) {
    if(mask.at<unsigned char>(i)){
      inlier_match_points1.push_back(selected_points1[i]);
      inlier_match_points2.push_back(selected_points2[i]);
    }
  }*/

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

vector<Mat> cameraPoseToExtrinsic(Mat R_in, Mat t_in)
{
    /*To do: vedere da Matlab il codice, 
       riportarlo come commento qua*/

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

Mat triangPoints(vector<Point2f> keypoints1_conv_inlier, vector<Point2f> keypoints2_conv_inlier, Mat R, Mat t, Mat cameraMatrix)
{
    //Convertire in triangulation_points? (https://gist.github.com/cashiwamochi/8ac3f8bab9bf00e247a01f63075fedeb)

    /*vector<Point2d> triangulation_points1, triangulation_points2;
    for(int i = 0; i < mask.rows; i++) 
    {
        if(mask.at<unsigned char>(i))
        {
            triangulation_points1.push_back(Point2f((float)keypoints1_conv_inlier[i].x, (float)keypoints1_conv_inlier[i].y));
            triangulation_points2.push_back(Point2f((float)keypoints2_conv_inlier[i].x, (float)keypoints2_conv_inlier[i].y));
        }
    }*/
    
    Mat R_prev = (Mat1d(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
    Mat t_prev = (Mat1d(3, 1) << 0, 0, 0);

    /************************************************************
     * La trasformazione R e t _prev sono identita'             *
     * perche' rappresentano un riferimento rispetto            *
     * al frame dell'immagine precedente.                       *
     * Quindi per avere la trasformazione dal frame precedente  *
     * a quello successivo, occorre che il riferimento per la   *
     * triangolazione sia identita'.                            *
     ************************************************************/

    vector<Mat> prevTransf = cameraPoseToExtrinsic(R_prev, t_prev);
    vector<Mat> currTransf = cameraPoseToExtrinsic(R, t); 

    /********************************
     * currTransf[0] = R_{k-1, k}   *
     * currTransf[1] = t_{k, k-1}^k *
     ********************************/

    Mat prevMatrix = projectionMatrix(prevTransf[0], prevTransf[1], cameraMatrix); 
    Mat currMatrix = projectionMatrix(currTransf[0], currTransf[1], cameraMatrix); 

    Mat world_points; 
    triangulatePoints(prevMatrix, currMatrix, keypoints1_conv_inlier, keypoints2_conv_inlier, world_points);
    //triangulatePoints(worldMatrix, currMatrix, triangulation_points1, triangulation_points2, world_points);

    /****************************************************
     * world_points sono espressi in coordinate {k-1}   *
     ****************************************************/

    world_points = world_points.rowRange(0, 3);
    world_points.convertTo(world_points, CV_64F); //Converto nel tipo coerente con gli altri elementi

    /*----------Reproject Error:------------*/
    vector<double> reproject_prev = reproject_error(world_points, prevTransf[0], -t_prev, cameraMatrix, keypoints1_conv_inlier);
    vector<double> reproject_curr = reproject_error(world_points, currTransf[0], -t, cameraMatrix, keypoints2_conv_inlier);  

    vector<double> reproject_mean(reproject_prev.size());

    for(int i = 0; i < reproject_prev.size(); i++)
    {
        ROS_INFO("Reproject Error:");
        ROS_INFO("Prev frame: %f | Curr frame: %f", reproject_prev[i], reproject_curr[i]);

        reproject_mean[i] = (reproject_prev[i] + reproject_curr[i])/2.0;
        ROS_INFO("Mean: %f", reproject_mean[i]);
    }
    ROS_INFO("-----------------------------------------");
    /*---------------------------------------*/

    //Convert from Prev camera coord in Curr camera coord
    for(int i = 0; i < world_points.cols; i++)
    {
        world_points.col(i) = coordTransf(world_points.col(i), currTransf[0], currTransf[1]);
    }

    //Adesso li ho convertiti in coordinate {k}.

    return world_points;
}

vector<double> reproject_error(Mat world_points, Mat R, Mat t, Mat cameraMatrix, vector<Point2f> img_points)
{
    //Reproject world_points (3D) in 2D image plan, using projectPoints
    //vector<Point2f> reproject_point;
    Mat reproject_point;
    Mat Rvec;
    Rodrigues(R, Rvec);

    projectPoints(world_points, Rvec, t, cameraMatrix, noArray(), reproject_point); 

    //Compute reproject error
    vector<double> reproject_err(img_points.size());

    for(int i = 0; i < reproject_point.rows; i++)
    {
        //ROS_INFO("Keypoint 2D: [%f, %f] \t Reprojected World Point: [%f, %f]", img_points[i].x, img_points[i].y, reproject_point.at<double>(i, 0), reproject_point.at<double>(i, 1));
        reproject_err[i] = sqrt(pow(img_points[i].x - reproject_point.at<double>(i, 0), 2.0) + pow(img_points[i].y - reproject_point.at<double>(i, 1), 2.0));
    }
    //ROS_INFO("----------------------");

    return reproject_err;
}

double scaleFactor(float distance, Mat worldPoints)
{
    double Zsum = 0.0;
    int N = worldPoints.cols;

    //Calcolo la media
    for(int i = 0; i < N; i++)
    {
        Zsum += worldPoints.at<double>(2, i);
    } 
    
    double Zmean = Zsum / N;

    if(Zmean == 0)
    {
        ROS_ERROR("Z pari a 0!");
        return 1.0; //per continuare il codice
    }

    else
        return distance/Zmean;
}

vector<Mat> absolutePose(Mat rotm, Mat tran, Mat orient, Mat loc, double SF, Mat world_points)
{
    /************************************************************
     * Input:                                                   *
     * >rotm: Matrice di rotazione {W} -> {k-1}                 *
     * >tran: Vettore da {W} a {k-1}, espresso in coord. {W}    *
     * >orient: Matrice di rotazione {k} -> {k-1}               *
     * >loc: Vettore da {k-1} a {k}, espresso in coord. {k-1}   *
     * >SF: Scale factor.                                       *
     * >world_points: Punti nello spazio {k}.                   *
    *************************************************************/

    //Trasformo loc = t_{k-1, k}^(k-1) in t_{w, k}^w
    Mat scaled_loc = coordTransf(SF*loc, rotm.t(), tran);
    
    //orient_wk: Matrice di rotazione dal frame {W} al frame {k}
    Mat orient_wk = orient*rotm;

    //Trasformo in coordinate {W} i world_points.
    Mat world_pointsW(world_points.rows, world_points.cols, CV_64F);

    for(int i = 0; i < world_points.cols; i++)
    {
        world_pointsW.col(i) = coordTransf(SF*world_points.col(i), orient_wk.t(), scaled_loc);
    }

    vector<Mat> absPose = {scaled_loc, orient_wk, world_pointsW};

    return absPose;
}

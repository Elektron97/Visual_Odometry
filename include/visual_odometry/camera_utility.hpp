/*CAMERA UTILITY*/

/*INCLUDE*/
#include "ros/ros.h"
#include <iostream>
#include <opencv2/opencv.hpp> 
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

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
bool showFrame = false;
bool showMatch = false;
bool showInlier = true;

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

//Detect and Match Features
Mat get_image(Mat current_img, Mat cameraMatrix, Mat distortionCoeff); 
KeyPoint_Match detectAndMatchFeatures(Mat img1, Mat img2);

//Structs Interface
KpAsPoint2f_Match keyPoint2Point2f(KeyPoint_Match kp_match);
KeyPoint_Match point2f2keyPoint(KpAsPoint2f_Match kp_pnt2f);

//Inlier
void show_info(int outlier, int inlier, int keypoints_matched);
KpAsPoint2f_Match extract_Inlier(vector<Point2f> keypoints1_conv, vector<Point2f> keypoints2_conv, Mat cameraMatrix);
void show_inlier(KpAsPoint2f_Match inlier_match_p2f, Mat prev_img, Mat curr_img);

//Estimate Relative Pose
vector<Mat> estimateRelativePose(vector<Point2f> keypoints1_conv, vector<Point2f> keypoints2_conv, Mat cameraMatrix);
bool isRotationMatrix(Mat &R); 
Vec3f rotationMatrixToEulerAngles(Mat &R);

//Triangulate Position and Scale Factor
vector<Mat> cameraPoseToExtrinsic(Mat R_in, Mat t_in);
Mat projectionMatrix(Mat R, Mat t, Mat cameraIntrinsic);
Mat triangPoints(vector<Point2f> keypoints1_conv, vector<Point2f> keypoints2_conv, Mat R, Mat t, Mat cameraMatrix);
float scaleFactor(float distance, Mat worldPoints);

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

Mat triangPoints(vector<Point2f> keypoints1_conv_inlier, vector<Point2f> keypoints2_conv_inlier, Mat R, Mat t, Mat cameraMatrix)
{
    Mat R_world = (Mat1d(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
    Mat t_world = (Mat1d(3, 1) << 0, 0, 0);

    vector<Mat> worldTransf = cameraPoseToExtrinsic(R_world, t_world);
    vector<Mat> currTransf = cameraPoseToExtrinsic(R, t); 

    //To do: cameraMatrix -> [Intrinsic]*[R | t]
    //perche' non usare projectPoints?

    Mat worldMatrix = projectionMatrix(worldTransf[0], worldTransf[1], cameraMatrix); 
    Mat currMatrix = projectionMatrix(currTransf[0], worldTransf[1], cameraMatrix); 

    cv::Mat world_points; //(4, leftInlier.size(), CV_64F);
    triangulatePoints(worldMatrix, currMatrix, keypoints1_conv_inlier, keypoints2_conv_inlier, world_points);
    //Perche' fa tutto 0?

    //To do: Reprojection Error
    //Reietto worldpoints che hanno troppo
    //reprojection error

    return world_points;
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



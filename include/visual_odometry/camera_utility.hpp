/*CAMERA UTILITY*/

/*INCLUDE*/
#include <visual_odometry/math_utility.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

/*ROS MSGS*/
#include "sensor_msgs/CompressedImage.h"

/*DEFINE*/
#define DISTANCE 20.0
#define MIN_NUM_FEATURES 20.0
#define CLIP_LIMIT 16

/*NAMESPACES*/
using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;

/*ENUM*/
enum rel_pose_method {ESSENTIAL, HOMOGRAPHY};

/*CONSTANTS*/
//showImg utility
const int fps = 33;
bool showFrame = false;
bool showPrep = false;
bool showMatch = false;
bool showInlier = false;

/*Detect and Match parameters*/
//SURF parameters
int minHessian = 50;

int nOctaves = 4;
int nOctaveLayers = 3;
bool extended = false;
bool upright = false;

//LOWE threshold
const float ratio_thresh = 0.7f; //0.7f;

//Reject Features in Black Background
int width_low = 28;
int height_low = 20;
int width_high = 614;
int height_high = 482;

/*Relative Pose parameters*/
//RANSAC Parameters
rel_pose_method rel_method = ESSENTIAL;

double ransac_prob[] = {0.99, 0.99}; //ESSENTIAL | HOMOGRAPHY
double ransac_threshold[] = {3.0, 2.0}; //ESSENTIAL | HOMOGRAPHY

const float inlier_threshold[] = {0.3, 0.3}; //ESSENTIAL | HOMOGRAPHY
//Valid Point Fraction Threshold
const float VPF_threshold = 0.85; //0.85;

const double distance_threshold = 50.0;

/*Triangulation*/
const float reprojection_tolerance = 0.5;

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
//Preprocessing
Mat desiredResize(Mat img, int desired_width, int desired_height);
Mat get_image(Mat current_img, int desired_width, int desired_height, Mat cameraMatrix, Mat distortionCoeff);

void updateCameraMatrix(Mat image, int desired_width, int desired_height, Mat& cameraMatrix);
KeyPoint_Match point2f2keyPoint(KpAsPoint2f_Match kp_pnt2f);

//Inlier
void show_info(int outlier, int inlier, int keypoints_matched);
KpAsPoint2f_Match extract_Inlier(vector<Point2f> keypoints1_conv, vector<Point2f> keypoints2_conv, vector<uchar> RANSAC_mask);
void show_inlier(KpAsPoint2f_Match inlier_match_p2f, Mat prev_img, Mat curr_img);

//Triangulate Position and Scale Factor
vector<Mat> cameraPoseToExtrinsic(Mat R_in, Mat t_in);
Mat projectionMatrix(Mat R, Mat t, Mat cameraIntrinsic);
Mat triangPoints(vector<Point2f> keypoints1_conv, vector<Point2f> keypoints2_conv, Mat R, Mat t, Mat cameraMatrix);
vector<double> reproject_error(Mat world_points, Mat R, Mat t, Mat cameraMatrix, vector<Point2f> img_points);
double scaleFactor(float distance, Mat worldPoints);

//Absolute Pose
vector<Mat> absolutePose(Mat rotm, Mat tran, Mat orient, Mat loc, double SF, Mat world_points);
vector<Mat> absolutePose(Mat rotm, Mat tran, Mat orient, Mat loc, double SF); //overload for success = false

//Robustness of code
bool checkIfMoving(KpAsPoint2f_Match kP);
bool checkMinFeat(KpAsPoint2f_Match kP);

Mat my_convertFromHom(Mat points4d);
Mat filter_convertWP(Mat world_points, vector<double> reproject_mean, Mat R, Mat t);

int recoverPoseHomography(Mat H, KpAsPoint2f_Match inlier, Mat cameraMatrix, Mat& R, Mat& t);

void optRelativePose(KpAsPoint2f_Match kP_converted, Mat cameraMatrix, Mat& R, Mat& t, vector<uchar>& RANSAC_mask, bool& success);
void opt_DetectFeatures(Mat img1, Mat img2, KpAsPoint2f_Match& kP_converted);

/*********Source**********/

Mat desiredResize(Mat img, int desired_width, int desired_height)
{
    //Check if the img is in the desired size
    int original_width = img.cols;
    int original_height = img.rows;

    if( (original_width != desired_width) || (original_height != desired_height) )
    {
        Mat resized_img;
        resize(img, resized_img, Size(desired_width, desired_height), 0, 0, INTER_AREA);

        return resized_img;
    }

    else
        return img;

}

void updateCameraMatrix(Mat image, int desired_width, int desired_height, Mat& cameraMatrix)
{
    double RW = ((double)desired_width)/((double)image.cols);
    double RH = ((double)desired_height)/((double)image.rows);

    Mat rescale_matrix = (Mat1d(3, 3) << RW, 0, 0, 0, RH, 0, 0, 0, 1);
    cameraMatrix = rescale_matrix*cameraMatrix;
}        
        
Mat get_image(Mat current_img, int desired_width, int desired_height, Mat cameraMatrix, Mat distortionCoeff)
{
    //PREPROCESSING IMAGE
    //Resize
    Mat resized_img = desiredResize(current_img, desired_width, desired_height);

    //Undistort
    Mat undistorted_image;
    undistort(resized_img, undistorted_image, cameraMatrix, distortionCoeff);
    //undistort(resized_img, undistorted_image, cameraMatrix, noArray());

    //RGB2GRAY
    Mat gray_img;
    cvtColor(undistorted_image, gray_img, COLOR_RGB2GRAY);

    //return gray_img;

    //Color Correction: CLAHE Algorithm
    Ptr<CLAHE> clahe = createCLAHE();
    clahe->setClipLimit(CLIP_LIMIT);

    Mat preprocessed_img;
    clahe->apply(gray_img, preprocessed_img);

    if(showPrep)
    {
        namedWindow("Preprocessed Image", WINDOW_AUTOSIZE);
        imshow("Preprocessed Image", preprocessed_img);
        waitKey(fps);
    }

    return preprocessed_img;
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

    float inlier_perc = 100.0*inlier/(outlier + inlier);
    ROS_INFO("Inlier Perc: %f", inlier_perc);
}

KpAsPoint2f_Match extract_Inlier(vector<Point2f> keypoints1_conv, vector<Point2f> keypoints2_conv, vector<uchar> RANSAC_mask)
{
    //RANSAC_mask, vettore contenente N elementi (N = length(keypoints)) in cui indica:
    // 0 = outlier
    // 1 = inlier

    int outlierCount = 0;
    int i = 0;

    for(i; i < RANSAC_mask.size(); i++)
    {
        if(RANSAC_mask[i] == 0)
            outlierCount ++;

    }

    int inlierCount = RANSAC_mask.size() - outlierCount;

    if(showInlier)
        show_info(outlierCount, inlierCount, RANSAC_mask.size());

    // The above three variables are used to save the inner point and the matching relationship
    KpAsPoint2f_Match inlier_match_p2f;
    
    inlier_match_p2f.match.resize(inlierCount);
    inlier_match_p2f.Kpoints1.resize(inlierCount);
    inlier_match_p2f.Kpoints2.resize(inlierCount);

    inlierCount = 0;
    for (i = 0; i<RANSAC_mask.size(); i++)
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

vector<Mat> cameraPoseToExtrinsic(Mat R_in, Mat t_in)
{
    /****************************************************
     * La funzione di fatto implementa l'inversa        *
     * di una trasformazione omogenea.                  *
     *                                                  *
     * T = [R d; 0 1] -> inv(T) = [R.t() -R.t()*d; 0 1] *
     ****************************************************/  

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
    /***************************TRIANG POINTS****************************
     * keypoints_conv_inlier (1, 2): inlier in formato vector<Point2f>. *
     * R: Rotazione {k-1} -> {k}.                    |                  *
     * t: Traslazione {k} -> {k-1} espresso in {k}.  |-> T {k-1} -> {k} *
     * cameraMatrix.                                                    *
     ********************************************************************/
    
    Mat R_prev = (Mat1d(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
    Mat t_prev = (Mat1d(3, 1) << 0, 0, 0);

    /************************************************************
     * La trasformazione R e t _prev sono identita' in quanto   *
     * il frame {k-1} fa da {W} e dunque i world_points ricavati*
     * dalla triangolazione saranno espressi in {k-1}.          *
     * In sostanza, scrivendo R = eye(3) e d = zeros(3, 1)      *
     * sto scrivendo {W} = {k-1}.                               *
     ************************************************************/

    /************Projection Matrix*******************
     * In generale la projection Matrix permette    *
     * la triangolazione.                           *
     * Essa e' costruita a partire da R e t,        *
     * elementi che costituiscono la trasformata    *
     * omogenea T: {k-1} -> {k}.                    *
     ************************************************/

    Mat prevMatrix = projectionMatrix(R_prev, t_prev, cameraMatrix); 
    Mat currMatrix = projectionMatrix(R, t, cameraMatrix);

    Mat world_points4d;
    triangulatePoints(prevMatrix, currMatrix, keypoints1_conv_inlier, keypoints2_conv_inlier, world_points4d);

    /****************************************************
     * world_points sono espressi in coordinate {k-1}   *
     ****************************************************/

    Mat world_points = my_convertFromHom(world_points4d);
    world_points.convertTo(world_points, CV_64F);

    /*---------------------------------------Reproject Error:--------------------------------------------------*/
    vector<double> reproject_prev = reproject_error(world_points, R_prev, t_prev, cameraMatrix, keypoints1_conv_inlier);
    vector<double> reproject_curr = reproject_error(world_points, R, t, cameraMatrix, keypoints2_conv_inlier);  

    vector<double> reproject_mean(reproject_curr.size());

    int i = 0;
    for(i; i < reproject_curr.size(); i++)
    {
        //ROS_INFO("Reproject Error:");
        //ROS_INFO("Prev frame: %f | Curr frame: %f", reproject_prev[i], reproject_curr[i]);

        reproject_mean[i] = (reproject_prev[i] + reproject_curr[i])/2.0;
        //ROS_INFO("Mean: %f", reproject_mean[i]);
    }
    //ROS_INFO("-----------------------------------------");
    /*----------------------------------------------------------------------------------------------------------*/

    //Discard bad world_points with high reprojection error
    Mat goodWP = filter_convertWP(world_points, reproject_mean, R, t);

    return goodWP;
}

vector<double> reproject_error(Mat world_points, Mat R, Mat t, Mat cameraMatrix, vector<Point2f> img_points)
{

    /************************REPROJECT ERROR*********************************
     * world_points: Punti 3D in coordinate {W} da proiettare in {C}.       *
     * R e t: Trasformazione omogenea che trasforma da {W} a {C}            *
     * cameraMatrix                                                         *
     * img_points: Punti con cui confrontare i world_points riproiettati.   *
     ************************************************************************/

    //Reproject world_points (3D) in 2D image plan, using projectPoints
    Mat reproject_points;
    Mat Rvec;
    Rodrigues(R, Rvec);

    projectPoints(world_points, Rvec, t, cameraMatrix, noArray(), reproject_points); 

    //Compute reproject error
    vector<double> reproject_err(img_points.size());

    int i = 0;
    for(i; i < reproject_points.rows; i++)
    {
        //ROS_INFO("Keypoint 2D: [%f, %f] \t Reprojected World Point: [%f, %f]", img_points[i].x, img_points[i].y, reproject_points.at<double>(i, 0), reproject_points.at<double>(i, 1));
        reproject_err[i] = sqrt(pow(img_points[i].x - reproject_points.at<double>(i, 0), 2.0) + pow(img_points[i].y - reproject_points.at<double>(i, 1), 2.0));
    }
    //ROS_INFO("----------------------");

    return reproject_err;
}

double scaleFactor(float distance, Mat world_points)
{
    double Zsum = 0.0;
    int N = world_points.cols;

    CV_Assert((world_points.rows == 3) && (world_points.type() == CV_64F) && (N != 0));

    //Calcolo la media
    int i = 0;
    for(i; i < N; i++)
    {
        Zsum += world_points.at<double>(2, i);
    } 
    
    double Zmean = Zsum / N;

    if(Zmean == 0)
    {
        ROS_ERROR("Z pari a 0!");
        return 1.0;
    }

    else
    {
        return distance/Zmean;
    }
        
}

vector<Mat> absolutePose(Mat rotm, Mat tran, Mat orient, Mat loc, double SF, Mat world_points)
{
    /************************************************************
     * Input:                                                   *
     * >rotm: Matrice di rotazione {k-1} -> {W}                 *
     * >tran: Versore da {W} a {k-1}, espresso in coord. {W}    *
     * >orient: Matrice di rotazione {k-1} -> {k}               *
     * >loc: Vettore da {k} a {k-1}, espresso in coord. {k}     *
     * >SF: Scale factor.                                       *
     * >world_points: Punti nello spazio {k}.                   *
     *                                                          *
     * Output:                                                  *
     * >world_pointsW: Punti nello spazio {W}                   *
     * >R_wk                                                    *
     * >t_wk                                                    *
    *************************************************************/

   //Rotation Matrix from {k} to {W}
    Mat orient_wk = rotm*orient.t(); //{W} <- {k-1} <- {k}
    Mat scaled_locW = coordTransf(-SF*orient.t()*loc, rotm, tran);

    /*****************NOTA SULLA TRASFORMAZIONE {k} -> {W}*****************************
     * La matrice di trasf. omogenea {k} -> {W}                                       *
     * e' composta dalla matrice R_wk e del vettore                                   *
     * t_wk.                                                                          *
     * Dunque la matrice Omogenea associata sara' del tipo:                           *
     *      [orient_wk.             scaled_locW]                                      *
     * T =  |                                  |                                      *
     *      [0                                1]                                      *
     **********************************************************************************/

    //Trasformo in coordinate {W} i world_points.
    Mat world_pointsW = Mat::zeros(world_points.rows, world_points.cols, CV_64F);

    int i = 0;
    int j = 0;

    for(i; i < world_points.cols; i++)
    {
        //Local Variable for Loops
        Mat local_WP = coordTransf(SF*world_points.col(i), orient_wk, scaled_locW);

        //Fill WP in 3 components
        for(j = 0; j < world_points.rows; j++)
        {
            world_pointsW.at<double>(j, i) = local_WP.at<double>(j);
        }
    }

    vector<Mat> absPose = {scaled_locW, orient_wk, world_pointsW};

    return absPose;
}

//Overload for case success = false
vector<Mat> absolutePose(Mat rotm, Mat tran, Mat orient, Mat loc, double SF)
{
    /************************************************************
     * Input:                                                   *
     * >rotm: Matrice di rotazione {k-1} -> {W}                 *
     * >tran: Versore da {W} a {k-1}, espresso in coord. {W}    *
     * >orient: Matrice di rotazione {k-1} -> {k}               *
     * >loc: Vettore da {k} a {k-1}, espresso in coord. {k}     *
     * >SF: Scale factor.                                       *
     * >world_points: Punti nello spazio {k}.                   *
     *                                                          *
     * Output:                                                  *
     * >world_pointsW: Punti nello spazio {W}                   *
     * >R_wk                                                    *
     * >t_wk                                                    *
    *************************************************************/

   //Rotation Matrix from {k} to {W}
    Mat orient_wk = rotm*orient.t(); //{W} <- {k-1} <- {k}
    Mat scaled_locW = coordTransf(-SF*orient.t()*loc, rotm, tran);

    /*****************NOTA SULLA TRASFORMAZIONE {k} -> {W}*****************************
     * La matrice di trasf. omogenea {k} -> {W}                                       *
     * e' composta dalla matrice R_wk e del vettore                                   *
     * t_wk.                                                                          *
     * Dunque la matrice Omogenea associata sara' del tipo:                           *
     *      [orient_wk.             scaled_locW]                                      *
     * T =  |                                  |                                      *
     *      [0                                1]                                      *
     **********************************************************************************/

    vector<Mat> absPose = {scaled_locW, orient_wk};

    return absPose;
}

bool checkIfMoving(KpAsPoint2f_Match kP)
{
    /************CHECK IF MOVING*********************
     * La funzione controlla se il robot si muove   *
     * facendo la norma dello scarto tra i Keypoints*    
     * matchati.                                    *
     ************************************************/

    int n_kP = kP.Kpoints1.size();
    vector<double> pixelTrasl(n_kP);

    int i = 0;
    for(i; i < n_kP; i++)
    {
        pixelTrasl[i] = sqrt(powf(kP.Kpoints1[i].x - kP.Kpoints2[i].x, 2.0) + powf(kP.Kpoints1[i].y - kP.Kpoints2[i].y, 2.0));
    }

    if(median(pixelTrasl) < DISTANCE)
        return false;

    else
        return true;
}

bool checkMinFeat(KpAsPoint2f_Match kP)
{
    return (kP.Kpoints1.size() < MIN_NUM_FEATURES);
}

Mat my_convertFromHom(Mat points4d)
{
    CV_Assert((points4d.rows == 4) &&  (points4d.type() == CV_32F));

    int cols = points4d.cols;

    Mat points3d(3, cols, points4d.type());

    int i = 0;
    for(i; i < cols; i++)
    {
        points3d.col(i) = (points4d.col(i).rowRange(0, 3))/(points4d.col(i).at<float>(3));
    }

    return points3d;    
}

Mat filter_convertWP(Mat world_points, vector<double> reproject_mean, Mat R, Mat t)
{
    CV_Assert((world_points.rows == 3) && (world_points.cols == reproject_mean.size()) && (world_points.type() == CV_64F)); 

    Mat goodWP;
    int i = 0; //no re-create iteration variable

    for(i; i < world_points.cols; i++)
    {
        if((reproject_mean[i] < reprojection_tolerance) && (world_points.at<double>(2, i) > 0))
        {
            //converto direttamente nel frame {k}
            goodWP.push_back(coordTransf(world_points.col(i), R, t).t()); //converto direttamente nel frame {k}
        }
            
            
    }

    //If is Empty, do not filter with other conditions
    if(goodWP.empty())
        return goodWP;

    //selecting only world_points with acceptable value of Z coord
    else
    {
        vector<double> mean_var = var_avgMat(goodWP.col(2));

        Mat goodWP_z;

        for(i = 0; i < goodWP.rows; i++)
        {
            if( (goodWP.at<double>(i, 2) <= mean_var[0] + 3.0*sqrt(mean_var[1])) && (goodWP.at<double>(i, 2) >= mean_var[0] - 3.0*sqrt(mean_var[1])) )
                goodWP_z.push_back(goodWP.row(i));
        }

        return goodWP_z.t();
    }
}

int recoverPoseHomography(Mat H, KpAsPoint2f_Match inlier, Mat cameraMatrix, Mat& R, Mat& t)
{
    //Decompose Homography Matrix
    vector<Mat> R_candidates, t_candidates;
    int solutions = decomposeHomographyMat(H, cameraMatrix, R_candidates, t_candidates, noArray());

    //n_solutions usually 4
    //1-2) R1 +-t1
    //3-4) R2 +-t2
    //Cheirality Check: triangulated_points.z > 0

    Mat eye_m = (Mat1d(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
    Mat zero_v = (Mat1d(3, 1) << 0, 0, 0);
    Mat proj_std = projectionMatrix(eye_m, zero_v, cameraMatrix);

    vector<Mat> triangulateCandidates(solutions);
    vector<int> n_goodTP(solutions);

    for(int i = 0; i < solutions; i++)
    {
        triangulatePoints(proj_std, projectionMatrix(R_candidates[i], t_candidates[i], cameraMatrix), inlier.Kpoints1, inlier.Kpoints2, triangulateCandidates[i]);
        triangulateCandidates[i] = my_convertFromHom(triangulateCandidates[i]);

        //Per ogni punto triangolato, controllo quant hanno z > 0
        for(int j = 0; j < triangulateCandidates[i].cols; j++)
        {
            bool good_condition = ((triangulateCandidates[i].at<double>(2, j) > 0) && (triangulateCandidates[i].at<double>(2, j) < distance_threshold));
            //bool good_condition = (triangulateCandidates[i].at<double>(2, j) > 0);
            if(good_condition)
                n_goodTP[i]++;

        }

    }
    
    //extract index of best candidates
    int idx_max = max_element(n_goodTP.begin(), n_goodTP.end()) - n_goodTP.begin();

    R = R_candidates[idx_max];
    t = t_candidates[idx_max];

    return n_goodTP[idx_max];
}

void optRelativePose(KpAsPoint2f_Match kP_converted, Mat cameraMatrix, Mat& R, Mat& t, KpAsPoint2f_Match& inlier_converted, bool& success)
{
    vector<uchar> RANSAC_mask;

    switch(rel_method)
    {
        case ESSENTIAL:
        {
            //Optimized Relative Pose
            Mat E = findEssentialMat(kP_converted.Kpoints1, kP_converted.Kpoints2, cameraMatrix, RANSAC, ransac_prob[rel_method], ransac_threshold[rel_method], RANSAC_mask);

            int outlierCount = 0;
            int i = 0;

            for(i; i < RANSAC_mask.size(); i++)
            {
                if(RANSAC_mask[i] == 0)
                    outlierCount ++;

            }
            int inlierCount = RANSAC_mask.size() - outlierCount;

            inlier_converted = extract_Inlier(kP_converted.Kpoints1, kP_converted.Kpoints2, RANSAC_mask);

            if((float)inlierCount/(float)RANSAC_mask.size() <= inlier_threshold[rel_method])
            {
                success = false;
                ROS_WARN("Few Inliers");
            }
                

            else
            {
                Mat rel_rot, rel_trasl;
                int validInlier = recoverPose(E, inlier_converted.Kpoints1, inlier_converted.Kpoints2, cameraMatrix, rel_rot, rel_trasl);

                float validPointFraction = (float)validInlier/(float)inlierCount;

                ROS_INFO("VPF: %f", validPointFraction);

                if(validPointFraction >= VPF_threshold)
                {
                    ROS_WARN("Valid relative Pose");
                    R = rel_rot;
                    t = rel_trasl;
                    success = true; 
                }

                else
                    success = false;
            }
        }
        break;
        
        case HOMOGRAPHY:
        {
            //Optimized Relative Pose
            Mat H = findHomography(kP_converted.Kpoints1, kP_converted.Kpoints2, RANSAC, ransac_threshold[rel_method], RANSAC_mask, 2000, ransac_prob[rel_method]);

            int outlierCount = 0;
            int i = 0;

            for(i; i < RANSAC_mask.size(); i++)
            {
                if(RANSAC_mask[i] == 0)
                    outlierCount ++;

            }
            int inlierCount = RANSAC_mask.size() - outlierCount;

            inlier_converted = extract_Inlier(kP_converted.Kpoints1, kP_converted.Kpoints2, RANSAC_mask);

            if((float)inlierCount/(float)RANSAC_mask.size() < inlier_threshold[rel_method])
            {
                success = false;
                ROS_WARN("Few Inliers");
            }
                

            else
            {
                Mat rel_rot, rel_trasl;
                int validInlier = recoverPoseHomography(H, inlier_converted, cameraMatrix, rel_rot, rel_trasl);

                float validPointFraction = (float)validInlier/(float)inlierCount;

                ROS_INFO("VPF: %f", validPointFraction);

                if(validPointFraction >= VPF_threshold)
                {
                    ROS_WARN("Valid relative Pose");
                    R = rel_rot;
                    t = rel_trasl;
                    success = true; 
                }

                else
                    success = false;
            }
        }
        break;

        default:
        break;                

    }
            
}

void opt_DetectFeatures(Mat img1, Mat img2, KpAsPoint2f_Match& kP_converted)
{
    //-- Step 1: Detect the keypoints using SURF Detector, compute the descriptors
    //Ptr<SURF> detector = SURF::create( minHessian);
    Ptr<SURF> detector = SURF::create( minHessian, nOctaves, nOctaveLayers, extended, upright);
    vector<KeyPoint> keypoints1, keypoints2;
    Mat descriptors1, descriptors2;
    detector->detectAndCompute( img1, noArray(), keypoints1, descriptors1 );
    detector->detectAndCompute( img2, noArray(), keypoints2, descriptors2 );

    //-- Step 2: Matching descriptor vectors with a flann based matcher
    // Since SURF is a floating-point descriptor NORM_L2 is used
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);

    vector< vector< DMatch > > knn_matches;
    matcher->knnMatch( descriptors1, descriptors2, knn_matches, 2);

    //-- Filter matches using the Lowe's ratio test
    //Default ratio_thresh: 0.7f; 
    vector<DMatch> matches;
    size_t i = 0;
    bool lowe_condition = false;
    bool black_background_condition = false;
    //bool black_background_condition = true;

    //Filter matches in black background
    for (i; i < knn_matches.size(); i++)
    {
        lowe_condition = (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance);
        black_background_condition = ((keypoints1[i].pt.x >= width_low ) && (keypoints1[i].pt.x <= width_high)) && ((keypoints1[i].pt.y >= height_low ) && (keypoints1[i].pt.y <= height_high));

        if (lowe_condition && black_background_condition)
        {
            matches.push_back(knn_matches[i][0]);
        }
    }

    if(showMatch)
    {
        //-- Draw matches
        Mat img_matches;
        drawMatches( img1, keypoints1, img2, keypoints2, matches, img_matches );
        //-- Show detected matches
        imshow("Matches", img_matches );
        waitKey(fps);
    }

    vector<Point2f> keypoints1_conv, keypoints2_conv;
    vector<DMatch>::const_iterator it = matches.begin();

    for (it; it!= matches.end(); ++it) 
    {    
        // Get the position of keypoints1
        keypoints1_conv.push_back(keypoints1[it->queryIdx].pt); //query per keypoints1
        // Get the position of keypoints2
        keypoints2_conv.push_back(keypoints2[it->trainIdx].pt); //train per keypoints2
    }

    kP_converted.Kpoints1 = keypoints1_conv;
    kP_converted.Kpoints2 = keypoints2_conv;
    kP_converted.match = matches;
} 
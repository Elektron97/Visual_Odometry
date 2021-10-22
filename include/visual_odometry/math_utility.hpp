/*MATH UTILITY*/

/*INCLUDE*/
#include "ros/ros.h"
#include <iostream>
#include <opencv2/opencv.hpp> 
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/tf.h>

/*ROS MSGS*/
#include "sensor_msgs/CompressedImage.h"

/*NAMESPACES*/
using namespace std;
using namespace cv;

/*FUNCTIONS*/
/*********Declaration**********/

//Ros - OpenCV interface
Mat ros2cv(sensor_msgs::CompressedImage image);

//Math Utility
Mat rotz(double angle);
Mat tf2Mat(tf::Matrix3x3 m);
tf::Matrix3x3 Mat2tf(Mat m);
Mat quat2Mat(geometry_msgs::Quaternion quat);
geometry_msgs::Vector3 mat2Euler(Mat R);
geometry_msgs::Vector3 quat2Euler(geometry_msgs::Quaternion quat);
Mat pos2Mat(geometry_msgs::Point pos);
Mat coordTransf(Mat vector, Mat R, Mat t); //Trasformazione esplicita omogenea
Mat omogMatrix(Mat R, Mat t); //Matrice Omogenea

//Euler and Rotm
bool isRotationMatrix(Mat &R); 
Vec3f rotationMatrixToEulerAngles(Mat &R);

double median(vector<double> v);

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
    //La funzione implementa la trasformazione omogenea
    return R*vector + t;
}

Mat omogMatrix(Mat R, Mat t)
{
    /**** Matrice Omogenea:******
     *      [R      t]          *
     * T =  |        |          *
     *      [0      1]          *
     ****************************/

    Mat T;
    Mat omg_v = (Mat1d(1, 4) << 0, 0, 0, 1);
    Mat Rt;
    hconcat(R, t, Rt);
    vconcat(Rt, omg_v, T);
    
    return T;
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

double median(vector<double> v)
{
  size_t size = v.size();

  if (size == 0)
  {
    ROS_ERROR("Empy vector");  // Undefined, really.
  }

  else
  {
    sort(v.begin(), v.end());
    if (size % 2 == 0)
      return (v[size / 2.0 - 1] + v[size / 2.0]) / 2.0;

    else 
      return v[size / 2.0];
  }
}

/*CLASSES*/
/*class OmogTransf
{
    Mat R;
    Mat t;
    Mat T;

    public:
    OmogTransf(Mat rot, Mat trasl)
    {
        R = rot;
        t = trasl;

        T = omogMatrix(rot, trasl);
    }

    Mat rotoTrasl(Mat vector)
    {
        //Dimensioni del vettore
        int rows_v = vector.rows;
        int cols_v = vector.cols;

        bool isRowVector = (rows_v == 1);
        bool isColVector = (cols_v == 1);
        bool isScalar = (isRowVector && isColVector);

        //if(isRowVector || isScalar)
        //    ROS_ERROR("Vettore riga e Scalari non supportati.");

        if(isColVector)
        {
            if(rows_v == 3)
                return coordTransf(vector, R, t);
            
            elseif(rows_v == 4)
                return coordTransf(vector.rowRange(0, 3), R, t);

            //else
            //    ROS_ERROR("Dimensione non supportata.");
        }
            
    }

    OmogTransf inverseOmog()
    {
        OmogTransf invT(R.t(), -R.t()*t);
        return invT;
    }

    
};*/
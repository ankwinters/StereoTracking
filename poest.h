//
// Created by lab on 1/8/16.
//

#ifndef POEST_PNP_H
#define POEST_PNP_H
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <vector>
using namespace cv;
using namespace std;

typedef int FEATURE_TYPE;
#define SIFT_FEATURE 0
#define ORB_FEATURE 1


//Mat camera_mat=(Mat_<double>(3,3)<< 586.7, 0., 399.5,
//        0., 586.4,299.5,
//        0., 0., 1.);

//Advanced data structure for keypoints matching
class FeaturedImg
{
public:
    /*
    FeaturedImg(const Mat &image,const vector<KeyPoint>& points_2d,vector<int> idx,
                const vector<Point3f>& points_3d,const Mat& points_descrip)
            :img(image),key_pts(points_2d),matched_idx(idx),matched_3d(points_3d),key_descrips(points_3d)
    {

    }*/

    Mat img;
    vector<KeyPoint> key_pts;
    Mat key_descrips;
    vector<int> matched_idx;
    vector<Point3f> matched_3d;
    //For coordinate computing
    Point2f top_left={0.,0.};

};
//Math class

class Quaternion
{
public:
    Quaternion(double R,double G,double B,double A):
            r(R),g(G),b(B),a(A),s(0){ }
    Quaternion(double R,double G,double B,double A,double S):
            r(R),g(G),b(B),a(A),s(S){ }

    inline void ToRMatrix(Mat &R);
    inline void ToRMat(Mat &R);

private:
    //real part
    double r;
    //imaginary part
    double g;
    double b;
    double a;
    double s;
};

class PnpImg
//Img class with R t info
{
public:
    /*
    FeaturedImg(const Mat &image,const vector<KeyPoint>& points_2d,vector<int> idx,
                const vector<Point3f>& points_3d,const Mat& points_descrip)
            :img(image),key_pts(points_2d),matched_idx(idx),matched_3d(points_3d),key_descrips(points_3d)
    {

    }*/
    Mat img;
    Mat R_t;
    //For coordinate computing
    Point2f top_left;

};

class PoseEst
{
public:
    void SetCameraMatrix(const Mat & camera_mat)
    {
        camera_matrix=camera_mat.clone();
    }
    void SolvePnP(const vector<Point2f> &image_coords,const vector<Point3f> &world_coords,
                  Mat &R,Mat &t);
    Point3f CalcWldCoord(const Mat &R,const Mat &t,const Point2f &img_coord);
    void MarkPtOnImg(Mat &img,const Point2f &img_coord);

private:
    /*

    Mat camera_matrix=(Mat_<float>(3,3)<< 586.7, 0., 399.5,
            0., 586.4,299.5,
            0., 0., 1.);*/
    Mat camera_matrix=(Mat_<float>(3,3)<< 537.6, 0., 400,
            0., 537.6,300,
            0., 0., 1.);
    vector<double> disto;


};
// Preprocess of features matching
// First,Slicing the two image to find the object contour

class BasicImageProcess
{
public:
    bool Histogram(const Mat &input,Mat &output);
    void BasicMatching(Mat &img_1, Mat &img_2, int max_points,
                       vector<KeyPoint> &key_img1,vector<KeyPoint> &key_img2,
                       Mat &descrip_1,Mat &descrip_2,vector< DMatch > &good_matches,
                       vector<Point2f> &matched_points_1, vector<Point2f> &matched_points_2);
    void BasicMatching(FEATURE_TYPE type,Mat &img_1, Mat &img_2,
                       vector<KeyPoint> &key_img1,vector<KeyPoint> &key_img2,
                       Mat &descrip_1,Mat &descrip_2, vector< DMatch > &good_matches, Mat &matched_img);


protected:
    bool SliceImage(const Mat &input, Mat &output, Point2f &top_left);
    bool DetectExtract(const Mat &img,vector<KeyPoint> &key_points,
                       Mat &descrip,FEATURE_TYPE type=ORB_FEATURE, int minHessian=400);
    bool FindGoodMatches(vector<DMatch> &raw_matches, const vector<KeyPoint> &query_pts,
                         const vector<KeyPoint> &train_pts,int num_points, vector<DMatch> &good_matches);
    bool GetMatchCoords(vector<DMatch> &matches,vector<KeyPoint> &key1,vector<KeyPoint> &key2,
                        vector<Point2f> &matched_pts_1,vector<Point2f> &matched_pts_2);
    bool RefineKp(FeaturedImg &fimg);





};

class StereoImageProcess:public BasicImageProcess
{
public:
    //ImageProcess(const Mat &color_image)

    bool ImageInput(const Mat &img_L, Mat &out_img_L, const Mat &img_R,Mat &out_img_R);
    bool StereoConstruct(const vector<Point2f> &matched_points_L,const vector<Point2f> &matched_points_R,
                         vector<Point3f> &world_points,const double baseline,const double f);
    bool StereoConstruct(FeaturedImg &left, const FeaturedImg &right,
                         vector<Point2f> &image_points_L,vector<Point3f> &world_points,
                         const double baseline=120.0,const double f=2.5, const double pixel_size=4.65e-3);
    FeaturedImg Matching(Mat &img_L, Mat &img_R, int max_points,vector<Point2f> &matched_points_1,
                         vector<Point2f> &matched_points_2);
    void FeaturesMatching(FeaturedImg &left, FeaturedImg &right,Mat &img_matches,FEATURE_TYPE type=ORB_FEATURE);

    void stereo_test(Mat &img1, Mat &img2);
    /* SliceImage:To detect the object from the both images.
     * Some advanced skills TBD
     * Shortage:partial detect impossible now.
     */
    //For debug
    void PrintCorners();
    void Featuremethod(Mat &image);
private:
    bool DetectObject(Mat &src_img,Mat &obj_img);
    //Find Img coords in the original image
    void OriginImgCoord(vector<Point2f> &pts_L,vector<Point2f> &pts_R);
    Point2f corner_L;
    Point2f corner_R;






};

//Track interest points from two images

class ObjectTracker:public BasicImageProcess
{
public:
    ObjectTracker(const FeaturedImg &fiducial):refer(fiducial){ }
         ObjectTracker()= default;
    void Track(FeaturedImg &target,vector<DMatch> &good_matches);

    void CalcMotions(vector<Point3f> &ref,vector<Point3f> &tgt,Mat &Rot,Mat &Tran);


private:
    FeaturedImg refer;
    bool RefineMatches(const vector<DMatch> &raw_matches, vector<DMatch> &good_matches,FEATURE_TYPE type=ORB_FEATURE);
    bool RefineMotions();

private:
    inline double GetNormal(const Mat &p1, const Mat &p2, const Mat &p3,Mat &output);
    inline double CalcNorm(const Mat &input);
    inline Point3f CalcCentroid(vector<Point3f> &pts);



};
/*
 * inline functions
 */
void Quaternion::ToRMatrix(Mat &R)
{

    //Mat R=Mat::zeros(3,3,CV_32F);
    R.at<double>(0,0)=r*r+g*g-b*b-a*a;
    R.at<double>(1,0)=2*(g*b-r*a);
    R.at<double>(2,0)=2*(g*a+r*b);

    R.at<double>(0,1)=2*(g*b+r*a);
    R.at<double>(1,1)=r*r-g*g+b*b-a*a;
    R.at<double>(2,1)=2*(b*a-r*g);

    R.at<double>(0,2)=2*(g*a-r*b);
    R.at<double>(1,2)=2*(b*a+r*g);
    R.at<double>(2,2)=r*r-g*g-b*b+a*a;
    return;
}

void Quaternion::ToRMat(Mat &R)
{
    //Alternative method to calc R
    R.at<double>(0,0)=1-2*b*b-2*a*a;
    R.at<double>(1,0)=2*(g*b+a*r);
    R.at<double>(2,0)=2*(g*a-b*r);

    R.at<double>(0,1)=2*(g*b-a*r);
    R.at<double>(1,1)=1-2*g*g-2*a*a;
    R.at<double>(2,1)=2*(b*a+g*r);

    R.at<double>(0,2)=2*(g*a+b*r);
    R.at<double>(1,2)=2*(b*a-g*r);
    R.at<double>(2,2)=1-2*g*g-2*b*b;
    return;

}




double ObjectTracker::GetNormal(const Mat &p1, const Mat &p2, const Mat &p3,Mat &output)
{
    double a,b,c;
    a = ( (p2.at<double>(1,0)-p1.at<double>(1,0))*(p3.at<double>(2,0)-p1.at<double>(2,0))-
          (p2.at<double>(2,0)-p1.at<double>(2,0))*(p3.at<double>(1,0)-p1.at<double>(1,0)) );
    b = ( (p2.at<double>(2,0)-p1.at<double>(2,0))*(p3.at<double>(0,0)-p1.at<double>(0,0))-
          (p2.at<double>(0,0)-p1.at<double>(0,0))*(p3.at<double>(2,0)-p1.at<double>(2,0)) );
    c = ( (p2.at<double>(0,0)-p1.at<double>(0,0))*(p3.at<double>(1,0)-p1.at<double>(1,0))-
          (p2.at<double>(1,0)-p1.at<double>(1,0))*(p3.at<double>(0,0)-p1.at<double>(0,0)) );

    //a = ( (p2.y-p1.y)*(p3.z-p1.z)-(p2.z-p1.z)*(p3.y-p1.y) );
    //b = ( (p2.z-p1.z)*(p3.x-p1.x)-(p2.x-p1.x)*(p3.z-p1.z) );
    //c = ( (p2.x-p1.x)*(p3.y-p1.y)-(p2.y-p1.y)*(p3.x-p1.x) );

    double length=sqrt(a*a+b*b+c*c);
    if(true)
    {
        output.at<double>(0,0)=a/length;
        output.at<double>(1,0)=b/length;
        output.at<double>(2,0)=c/length;
    }
    return length;

}


double ObjectTracker::CalcNorm(const Mat &input)
{
    double a=input.at<double>(0,0);
    double b=input.at<double>(1,0);
    double c=input.at<double>(2,0);
    return sqrt(a*a+b*b+c*c);
}

Point3f ObjectTracker::CalcCentroid(vector<Point3f> &pts)
{
    Point3f p;
    for(auto item:pts)
    {
        p+=item;
    }
    p.x=p.x/pts.size();
    p.y=p.y/pts.size();
    p.z=p.z/pts.size();
    return p;
}

#endif //POEST_PNP_H

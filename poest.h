//
// Created by lab on 1/8/16.
//

#ifndef POEST_PNP_H
#define POEST_PNP_H
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <vector>
using namespace cv;

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
    Point2f top_left;

};

class PoseEst
{
public:
    bool SetCameraMatrix(const Mat & camera_mat)
    {
        camera_matrix=camera_mat;
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
    bool ImageBlur(const Mat &input,Mat &output);
    void BasicMatching(Mat &img_1, Mat &img_2, int max_points,
                          vector<KeyPoint> &key_img1,vector<KeyPoint> &key_img2,
                          Mat &descrip_1,Mat &descrip_2,vector< DMatch > &good_matches,
                          vector<Point2f> &matched_points_1, vector<Point2f> &matched_points_2);


protected:
    bool SliceImage(const Mat &input, Mat &output, Point2f &top_left);
    bool DetectExtract(const Mat &img,vector<KeyPoint> &key_points,
                       Mat &descrip,FEATURE_TYPE type=ORB_FEATURE, int minHessian=400);
    bool FindGoodMatches(vector<DMatch> &raw_matches, const vector<KeyPoint> &query_pts,
                         const vector<KeyPoint> &train_pts,int num_points, vector<DMatch> &good_matches);
    bool GetMatchCoords(vector<DMatch> &matches,vector<KeyPoint> &key1,vector<KeyPoint> &key2,
                         vector<Point2f> &matched_pts_1,vector<Point2f> &matched_pts_2);





};

class StereoImageProcess:public BasicImageProcess
{
public:
    //ImageProcess(const Mat &color_image)

    bool ImageInput(const Mat &img_L, Mat &out_img_L, const Mat &img_R,Mat &out_img_R);
    bool StereoConstruct(const vector<Point2f> &matched_points_L,const vector<Point2f> &matched_points_R,
                         vector<Point3f> &world_points,const double baseline,const double f);
    FeaturedImg Matching(Mat &img_L, Mat &img_R, int max_points,vector<Point2f> &matched_points_1,
                         vector<Point2f> &matched_points_2);
    void FeaturesMatching(Mat &img_1, Mat &img_2, int max_points,
                          vector<Point2f> &matched_points_1, vector<Point2f> &matched_points_2);

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
    void Track(FeaturedImg &target);


private:
    FeaturedImg refer;


};



#endif //POEST_PNP_H

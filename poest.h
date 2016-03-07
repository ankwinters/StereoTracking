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

using namespace cv;


Mat camera_matrix=(Mat_<double>(3,3)<< 586.7, 0., 399.5,
        0., 586.4,299.5,
        0., 0., 1.);

class Pose_est
{
public:

    void PnPmethod(int x, int y);
    void Featuremethod(Mat &image);

    void SolvePnP(const vector<Point2f> &image_coords,const vector<Point3f> &world_coords,
                  Mat &R,Mat &t);

    bool stereo_construct(vector<Point2f> &matched_points_L,vector<Point2f> &matched_points_R,
                          vector<Point3f> &world_points,const double baseline,const double f);
    void stereo_test(Mat &img1, Mat &img2);
private:



};
// Preprocess of features matching
// First,Slicing the two image to find the object contour

class StereoImageProcess
{
public:
    //ImageProcess(const Mat &color_image)


    bool ImageInput(const Mat &img_L, Mat &out_img_L,const Mat &img_R,Mat &out_img_R);
    /* SliceImage:To detect the object from the both images.
     * Some advanced skills TBD
     * Shortage:partial detect impossible now.
     */
    bool SliceImage(const Mat &input, Mat &output, Point2f &top_left);
    void ORB_matching(Mat &img1, Mat &img2, int num_points,
                      vector<Point2f> &matched_points_L,vector<Point2f> &matched_points_R);
    //For debug
    void PrintCorners();


private:
    bool DetectObject(Mat &src_img,Mat &obj_img);
    //Point2i corner_L;
    //Point2i corner_R;
    Point2f corner_L;
    Point2f corner_R;
};

#endif //POEST_PNP_H

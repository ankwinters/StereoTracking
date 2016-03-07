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
extern vector<Point3f> world_coord;
extern vector<Point2f> image_coord;
extern Mat camera_matrix;
extern vector<double> disto;
extern Mat R;
extern Mat t;
extern Mat image;

class Pose_est
{
public:
    void PnPmethod(int x, int y);
    void Featuremethod();
    void sift_sift_flann();

    bool stereo_construct(vector<Point2f> &matched_points_L,vector<Point2f> &matched_points_R,
                          vector<Point3f> &world_points,const double baseline,const double f);
    void stereo_test(Mat &img1, Mat &img2);
private:
    void SolvePnP();


};
// Preprocess of features matching
// First,Slicing the two image to find the object contour

class StereoImageProcess
{
public:
    //ImageProcess(const Mat &color_image)

    /* SliceImage:To detect the object from the both images.
     * Some advanced skills TBD
     * Shortage:partial detect impossible now.
     */
    bool ImageInput(const Mat &img_L, Mat &out_img_L,const Mat &img_R,Mat &out_img_R);
    bool SliceImage(const Mat &input, Mat &output, Point2i &top_left);
    void ORB_matching(Mat &img1, Mat &img2, int num_points,
                      vector<Point2f> &matched_points_L,vector<Point2f> &matched_points_R);

    void PrintCorners();


private:
    bool DetectObject(Mat &src_img,Mat &obj_img);
    Point2i corner_L;
    Point2i corner_R;
};

#endif //POEST_PNP_H

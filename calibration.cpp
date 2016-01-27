#include <cmath>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core_c.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "calibration.h"

using namespace cv;
using namespace std;
double CameraCalib::ComputeReprojErrors( const vector<vector<Point3f> >& object_points,
                                  const vector<vector<Point2f> >& image_points,
                                  const vector<Mat>& r_vecs, const vector<Mat>& t_vecs,
                                  const Mat& camera_matrix , const Mat& dist_coeffs,
                                  vector<float>& perViewErrors)
{
    vector<Point2f> image_points2;
    int i, totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(object_points.size());

    for( i = 0; i < (int)object_points.size(); ++i )
    {
        projectPoints( Mat(object_points[i]), r_vecs[i], t_vecs[i], camera_matrix,  // project
                       dist_coeffs, image_points2);
        err = norm(Mat(image_points[i]), Mat(image_points2), CV_L2);              // difference

        int n = (int)object_points[i].size();
        perViewErrors[i] = (float) std::sqrt(err*err/n);                        // save for this view
        totalErr        += err*err;                                             // sum it up
        totalPoints     += n;
    }

    return std::sqrt(totalErr/totalPoints);              // calculate the arithmetical mean
}


void CameraCalib::CalcBoardCornerPositions(Size board_size, float square_size, vector<Point3f>& corners,
                  Pattern pattern_type=CHESSBOARD)
{
    corners.clear();
    if(pattern_type==CHESSBOARD)
    {
        for (int i = 0; i < board_size.height; ++i)
            for (int j = 0; j < board_size.width; ++j)
                corners.push_back(Point3f(float(j * square_size), float(i * square_size), 0));
    }
    else
    {
        cerr<<"Need chessboard pattern input!!"<<endl;
        exit(-1);
    }

}

void CameraCalib::SearchCorner(const Mat& view, const Size board_size, Mat& corners)
{
    bool found;
    //Rough estimate
    found=findChessboardCorners(view, board_size, corners,
                                CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
    if (found)                // If done with success,
    {
        // improve the found corners' coordinate accuracy for chessboard

            Mat viewGray;
            cvtColor(view, viewGray, CV_BGR2GRAY);
            cornerSubPix( viewGray, corners, Size(11,11),
                          Size(-1,-1), TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
    }
}
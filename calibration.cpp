#include <cmath>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "calibration.h"

double CameraCalib::ComputeReprojErrors( const vector<vector<Point3f> >& objectPoints,
                                  const vector<vector<Point2f> >& imagePoints,
                                  const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                                  const Mat& cameraMatrix , const Mat& distCoeffs,
                                  vector<float>& perViewErrors)
{
    vector<Point2f> imagePoints2;
    int i, totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(objectPoints.size());

    for( i = 0; i < (int)objectPoints.size(); ++i )
    {
        projectPoints( Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix,  // project
                       distCoeffs, imagePoints2);
        err = norm(Mat(imagePoints[i]), Mat(imagePoints2), CV_L2);              // difference

        int n = (int)objectPoints[i].size();
        perViewErrors[i] = (float) std::sqrt(err*err/n);                        // save for this view
        totalErr        += err*err;                                             // sum it up
        totalPoints     += n;
    }

    return std::sqrt(totalErr/totalPoints);              // calculate the arithmetical mean
}



#ifndef POEST_CALIBRATION_H
#define POEST_CALIBRATION_H
// Single Camera calibration with chessborad photos
class CameraCalib
{
public:
    bool ComputeKmat();
    double ComputeReprojErrors( const vector<vector<Point3f> >& objectPoints,
                                  const vector<vector<Point2f> >& imagePoints,
                                  const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                                  const Mat& cameraMatrix , const Mat& distCoeffs,
                                  vector<float>& perViewErrors);

    virtual bool SaveAllParas();
};
//stereo-camera Calibration
class StereoCameraCalib:public CameraCalib
{
public:
    bool ComputeLensPose();
    bool SaveAllParas();
};
#endif //POEST_CALIBRATION_H

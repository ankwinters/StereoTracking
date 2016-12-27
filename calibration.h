

#ifndef POEST_CALIBRATION_H
#define POEST_CALIBRATION_H
// Single Camera calibration with chessborad photos
#include <opencv2/core/core.hpp>
using namespace cv;

enum Pattern
{
    CHESSBOARD,
    CIRCLES_GRID,
    ASYMMETRIC_CIRCLES_GRID
};


class CameraCalib
{
public:
    bool ComputeKmat();
    double ComputeReprojErrors( const vector<vector<Point3f> >& object_points,
                                  const vector<vector<Point2f> >& image_points,
                                  const vector<Mat>& r_vecs, const vector<Mat>& t_vecs,
                                  const Mat& camera_matrix , const Mat& dist_coeffs,
                                  vector<float>& perViewErrors);

    virtual bool SaveAllParas();
private:
    void ReadImage();
    bool SearchCorner(const Mat& view, const Size board_size, Mat& corners);

    void CalcBoardCornerPositions(Size board_size, float square_size, vector<Point3f>& corners,
                                  Pattern patternType=CHESSBOARD);

};
//stereo-camera Calibration
class StereoCameraCalib:public CameraCalib
{
public:
    bool ComputeLensPose();
    bool SaveAllParas();
};
#endif //POEST_CALIBRATION_H

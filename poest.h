//
// Created by lab on 1/8/16.
//

#ifndef POEST_PNP_H
#define POEST_PNP_H
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

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
private:


};
#endif //POEST_PNP_H

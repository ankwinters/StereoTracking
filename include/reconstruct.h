//
// Created by lab on 16-12-27.
//

#ifndef TRACKER_RECONSTRUCT_H
#define TRACKER_RECONSTRUCT_H

#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
namespace tracker {
    using namespace cv;
    using namespace std;
    class Triangulate{
    public:
        Triangulate() = default;
        void Reconstruct3d(vector<Point2f> &left, vector<Point2f> &right, vector<Point3f> &coord_3d);

    private:
        float baseline=0.12;
        float pixel_size=4.65e-6;
        float f=2.5e-3;
        Mat camera_matrix=(Mat_<float>(3,3)<< 586.7, 0., 512.0,
                0., 586.4,384.0,
                0., 0., 1.);

    };

}

#endif //TRACKER_RECONSTRUCT_H

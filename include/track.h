//
// Created by lab on 16-12-27.
//

#ifndef TRACKER_TRACK_H
#define TRACKER_TRACK_H


#include <vector>
#include <string>

#include "features.h"
#include <opencv2/opencv.hpp>
namespace tracker {
    using namespace std;
    using namespace cv;


    class Track {
    public:
        Track() = default;
        void LoadImages(const string &path_to_dir, vector<string> &images_left,
                        vector<string> &images_right);

    private:
        void Tracking(const TrackNode &keyframe, TrackNode &current);

    private:
        //LogOption opt;


    };

    class TrackNode {
    public:
        vector<KeyPoint> pts2d;
        Mat descrip;
        Mat R_t;
    };

    class KeyFrame {
    public:
        //Key frame num
        unsigned int num;

    };



    class Motions{


    };
}
#endif //TRACKER_TRACK_H

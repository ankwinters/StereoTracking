//
// Created by lab on 16-12-27.
//

#ifndef POEST_FEATURE_MATCH_H
#define POEST_FEATURE_MATCH_H


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include "util.h"

#include <string>
#include <vector>

namespace tracker {
    using namespace cv;
    using namespace std;
    //record feature type
    enum FeatureType {
        SIFT_FEATURE = 0,
        SURF,
        ORB
    };
    // feature options setting
    class FeatureOpt {
    public:
        // Num of features
        int num_fea;
        FeatureType type;
    };

    class Features {
    public:
        //Features(LogOption logopt): this->opt(logopt) { fea_opt.num_fea=3000;fea_opt.type==SIFT_FEATURE; }
        Features(){ this->opt=check;  fea_opt.num_fea=3000;fea_opt.type=SIFT_FEATURE; }
        // Matching the left img with the right
        void StereoMatching(const Mat &img_1, const Mat &img_2,
                            vector<Point2f> & pts1, vector<Point2f> &pts2,
                            vector<KeyPoint> &keys1, Mat &descriptor);
        // Matching the key frame with the current one
        void SeqMatching(const vector<KeyPoint> & pts_kf,const Mat &des_kf,
                         const vector<KeyPoint> &pts2, const Mat &des2,
                         vector<DMatch> &matches);
    private:
        void detect_extract(const Mat &img,vector<KeyPoint> &key_points,Mat &descriptor);
        void features_match(const vector<KeyPoint> &query_pts,const Mat &query_des,
                            const vector<KeyPoint> &train_pts,const Mat &train_des,
                            vector<DMatch> &good_matches);
        void gen_descriptor(const Mat &img,const vector<int> &key_nums,
                            const vector<KeyPoint> &key_pts,
                            Mat &output);
    private:
        inline void keypts_to_pts(const vector<DMatch> &matches,
                                  const vector<KeyPoint> &query_pts, const vector<KeyPoint> &train_pts,
                                  vector<Point2f> &queries, vector<Point2f> &trains);

    private:
        LogOption opt;
        FeatureOpt fea_opt;

    };
}

#endif //POEST_FEATURE_MATCH_H

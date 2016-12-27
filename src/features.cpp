//
// Created by lab on 16-12-27.
//

#include "features.h"
#include <cstdlib>
#include <iostream>

namespace tracker {

    void Features::DetectExtract(const Mat &img, vector<KeyPoint> &key_points, Mat &descrip) {
       if(fea_opt.type == SIFT) {
           SIFT sift_detect;
           sift_detect.detect(img,key_points);
           sift_detect.compute(img,key_points,descrip);
       } else {
           cerr<<"Sorry! Only SIFT is supported now!"<<endl;
           cerr<<"Exit..."<<endl;
           exit(-1);
       }


    }

    void Features::StereoMatching(const Mat &img_1, const Mat &img_2, vector<Point2i> &pt1, vector<Point2i> &pt2,
                                  Mat &descrip1) {



    }
}
//
// Created by lab on 16-12-27.
//

#include "feature_match.h"

#include <cstdlib>
#include <iostream>
#include <opencv2/features2d/features2d.hpp>


namespace tracker {

    void Features::StereoMatching(const Mat &img_1, const Mat &img_2, vector<Point2f> &pts1, vector<Point2f> &pts2,
                                  vector<KeyPoint> &keys1,Mat &descriptor) {
        vector<KeyPoint> key_img1;
        vector<KeyPoint> key_img2;
        Mat descrip_1;
        Mat descrip_2;
        //Step1: detect & extract features
        if(opt == debug or opt == check) {
            cout<<"Extract features..."<<endl;
        }
        detect_extract(img_1, key_img1, descrip_1);
        detect_extract(img_2, key_img2, descrip_2);
        //Step2: match
        if(opt == debug or opt == check) {
            cout<<"Matching features..."<<endl;
        }
        vector<DMatch> good_matches;
        features_match(key_img1, descrip_1, key_img2, descrip_2,good_matches);
        //Step3: output
        if(opt == debug or opt == check) {
            cout<<"Writing matched points..."<<endl;
        }
        keypts_to_pts(good_matches,key_img1,key_img2,pts1,pts2);
        vector<int> valid_key1;
        if(opt == check) {
            // check if it is a good match
            Mat img;
            drawMatches(img_1,key_img1,img_2,key_img2,good_matches,img);
            imshow( "Good Matches", img);
            waitKey(0);
        }
        for(int i=0;i<good_matches.size();i++) {
            int k = good_matches[i].queryIdx;
            valid_key1.push_back(k);
            keys1.push_back(key_img1[k]);
        }
        if(opt == debug or opt == check) {
            cout<<"Generating descriptor for the left image..."<<endl;
        }
        gen_descriptor(img_1,valid_key1,key_img1,descriptor);

    }

    void Features::detect_extract(const Mat &img, vector<KeyPoint> &key_points, Mat &descriptor) {
        if(fea_opt.type == SIFT_FEATURE) {
            SIFT sift_detect;
            sift_detect.detect(img,key_points);
            sift_detect.compute(img,key_points,descriptor);
        } else {
            cerr<<"Sorry! Only SIFT is supported now!"<<endl;
            cerr<<"Exit..."<<endl;
            exit(-1);
        }
    }

    void
    Features::features_match(const vector<KeyPoint> &query_pts, const Mat &query_des, const vector<KeyPoint> &train_pts,
                             const Mat &train_des, vector<DMatch> &good_matches) {
        good_matches.clear();
        vector<DMatch> raw_matches;
        if(fea_opt.type==SIFT_FEATURE){
            // match features
            FlannBasedMatcher matcher;
            matcher.match(query_des,train_des,raw_matches);
            double max_dist = 0; double min_dist = 100;
            //-- Quick calculation of max and min distances between keypoints
            for( int i = 0; i<raw_matches.size(); i++ ) {
                double dist = raw_matches[i].distance;
                if (dist < min_dist) min_dist = dist;
                if (dist > max_dist) max_dist = dist;
            }
            //-- Draw only "good" matches (i.e. whose distance is less than 0.6*max_dist )
            //-- PS.- radiusMatch can also be used here.
            vector<Point2f> matches_1;
            vector<Point2f> matches_2;
            keypts_to_pts(raw_matches,query_pts,train_pts,matches_1,matches_2);
            const double threshold=0.05;
            for( int i = 0; i<raw_matches.size(); i++ ) {
                double y1 = matches_1[i].y;
                double y2 = matches_2[i].y;
                double diff = ((y1 - y2) < 0) ? (y2 - y1) : (y1 - y2);
                if (diff / y1 < threshold && raw_matches[i].distance < 0.3 * max_dist) {
                    good_matches.push_back(raw_matches[i]);
                }
            }
        } else {
            exit(-1);
        }

    }

    void Features::keypts_to_pts(const vector<DMatch> &matches, const vector<KeyPoint> &query_pts,
                                 const vector<KeyPoint> &train_pts, vector<Point2f> &queries, vector<Point2f> &trains) {
        queries.clear();
        trains.clear();
        for( int i = 0; i < matches.size(); i++ )
        {
            queries.push_back(query_pts[matches[i].queryIdx].pt);
            trains.push_back(train_pts[matches[i].trainIdx].pt);
        }

    }

    void Features::gen_descriptor(const Mat &img, const vector<int> &key_nums, const vector<KeyPoint> &key_pts,
                                  Mat &output) {
        vector<KeyPoint> valid_keys;
        for(int i=0;i<key_nums.size();i++) {
            int n = key_nums[i];
            valid_keys.push_back(key_pts[n]);
        }
        if(fea_opt.type ==SIFT_FEATURE){
            SIFT sift;
            sift.compute(img,valid_keys,output);
        } else {
            exit(-1);
        }
    }

    void Features::SeqMatching(const vector<KeyPoint> &pts_kf, const Mat &des_kf,const vector<KeyPoint> &pts2,
                               const Mat &des2, vector<DMatch> &matches) {
        features_match(pts_kf,pts2, des_kf,des2,matches);

    }



}
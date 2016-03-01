//
// Created by lab on 1/8/16.
//
#include "poest.h"
#include <iostream>
#include <chrono>
using namespace std;
void Pose_est::PnPmethod(int x, int y)
{
    if(image_coord.size()<world_coord.size())
    {
        /* To collect enough 2D-plane counterparts of the points in 3D space.
         */
        circle(image,Point(x,y),5,Scalar(0,0,255),-1);
        image_coord.push_back(Point2f(x, y));

    }

    if(image_coord.size()==world_coord.size() && R.rows==0)
    {
        /* When there are enough points collected,
         * calculate the matrix R|t with PnP algorithm
         */
        ///*counting time*/ std::chrono::time_point<std::chrono::system_clock> start, end;
        Mat Rod;
        //start=std::chrono::system_clock::now();
        solvePnP(world_coord, image_coord, camera_matrix, disto, Rod, t, false, CV_EPNP);
        //end=std::chrono::system_clock::now();
        //std::chrono::duration<double> elapsed_seconds = end-start;
        Rodrigues(Rod,R);
        //cout<<"solvePnP ITER time:"<<elapsed_seconds.count()<<endl;
        cout<<"R matrix:"<<R<<endl;
        cout<<"t matrix:"<<t<<endl;

    }
    if(R.rows>0)
    {

        Mat R_t;
        hconcat(R,t,R_t);
        cout<<"[R|t] = "<<R_t<<endl;

        Mat d3_p1=(Mat_<double>(4,1) << 10.8, 29.8, 0, 1);
        Mat d2_p1=camera_matrix*R_t;
        d2_p1=d2_p1*d3_p1;
        d2_p1=d2_p1/d2_p1.at<double>(2,0);

        Mat d3_p2=(Mat_<double>(4,1)<<10.8,0,4.6,1);
        Mat d2_p2=camera_matrix*R_t;
        d2_p2=d2_p2*d3_p2;
        d2_p2=d2_p2/d2_p2.at<double>(2,0);

        Mat d3_p3=(Mat_<double>(4,1)<<2.7,0,4.3,1);
        Mat d2_p3=camera_matrix*R_t;
        d2_p3=d2_p3*d3_p3;
        d2_p3=d2_p3/d2_p3.at<double>(2,0);


        cout<<"p1 position:"<<d2_p1<<endl;
        cout<<"p2 position:"<<d2_p2<<endl;

        circle(image,Point(d2_p1.at<double>(0,0),d2_p1.at<double>(1,0)),5,Scalar(255,0,0),-1);
        circle(image,Point(d2_p2.at<double>(0,0),d2_p2.at<double>(1,0)),5,Scalar(255,0,0),-1);
        circle(image,Point(d2_p3.at<double>(0,0),d2_p3.at<double>(1,0)),5,Scalar(255,0,0),-1);

    }
}

void Pose_est::Featuremethod()
{
    //1 initialize
    int minHessian = 400;
    SiftFeatureDetector sift_detect( minHessian );

    //2detect
    vector<KeyPoint> keypoints;
    sift_detect.detect( image, keypoints );

    //3 绘制特征点match
    /*
    Mat siftImg;
    drawKeypoints( image, keypoints, siftImg, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
    imshow("Sift keypoints", siftImg );
    cout<<"keypoint numbers of sift: "<<keypoints.size()<<endl;
     */
    //Once again with ORB
    OrbFeatureDetector orb_detect(minHessian);
    orb_detect.detect(image ,keypoints);
    Mat orbImg;
    drawKeypoints(image, keypoints, orbImg, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
    imshow("ORB keypoints", orbImg);
    cout<<"Keypoints of ORB number: "<<keypoints.size()<<endl;


}

void Pose_est::ORB_matching(Mat &img1, Mat &img2)
{
    int minHessian = 400;
    OrbFeatureDetector orb_detector(minHessian);
    //Step1:feature detection
    vector<KeyPoint> key_img1,key_img2;
    orb_detector.detect(img1,key_img1);
    orb_detector.detect(img2,key_img2);
    //Step2:compute
    OrbDescriptorExtractor orb_extractor;
    Mat descrip_img1,descrip_img2;
    orb_extractor.compute(img1,key_img1,descrip_img1);
    orb_extractor.compute(img2,key_img2,descrip_img2);
    //Step3:matching

    //cout<<"Debug info!!!!!!!!"<<endl;
    //FlannBasedMatcher matcher;
    BFMatcher matcher;
    std::vector< DMatch > matches;
    cout<<"Debug info!!!!!!!!"<<endl;
    matcher.match(descrip_img1,descrip_img2,matches);
   /*  Step4:Find good match
    double max_dist = 0; double min_dist = 100;
    for( int i = 0; i < descrip_img1.rows; i++ )
    {
        double dist = matches[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
    }

    std::vector< DMatch > good_matches;
    for( int i = 0; i < descrip_img1.rows; i++ )
    { if( matches[i].distance <= max(2*min_dist, 0.02) )
        { good_matches.push_back( matches[i]); }
    }*/
    //Step4:draw matches
    Mat img_matches;
    drawMatches( img1, key_img1, img2, key_img2,
                 matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                 vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
    imshow( "Good Matches", img_matches );

    return;



}


void Pose_est::sift_sift_flann()
{

}
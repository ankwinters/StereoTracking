//
// Created by lab on 1/8/16.
//
#include "poest.h"
#include <iostream>
#include <chrono>
using namespace std;

bool Pose_est::stereo_construct(const vector<Point2f> &matched_points_L, const vector<Point2f> &matched_points_R,
                                vector<Point3f> &world_points,const double baseline,const double f)
{
    size_t num_points=matched_points_L.size();
    double pixel_size=4.65e-3;
    //Z=b*f/d
    double d,Z,Y,X;
    for(size_t i=0;i<num_points;i++)
    {
        d=matched_points_L[i].x-matched_points_R[i].x;
        Z=baseline*f/(d*pixel_size);
        Y=Z*matched_points_R[i].y*pixel_size/f;
        X=Z*matched_points_R[i].x*pixel_size/f;
        world_points.push_back(Point3f(X,Y,Z));
    }

    return true;
}
void Pose_est::SolvePnP(const vector<Point2f> &image_coords, const vector<Point3f> &world_coords,
                        vector<double> &disto,Mat &R_mat, Mat &t_vec)
{
    if(image_coords.size()==world_coords.size() && R_mat.rows==0)
    {
        /* When there are enough points collected,
         * calculate the matrix R|t with PnP algorithm
         */
        ///*counting time*/ std::chrono::time_point<std::chrono::system_clock> start, end;
        Mat R_Rod;
        //start=std::chrono::system_clock::now();
        solvePnP(world_coords, image_coords, this->camera_matrix, disto, R_Rod, t_vec, false, CV_EPNP);
        //end=std::chrono::system_clock::now();
        //std::chrono::duration<double> elapsed_seconds = end-start;
        Rodrigues(R_Rod,R_mat);
        //cout<<"solvePnP ITER time:"<<elapsed_seconds.count()<<endl;
        cout<<"R matrix:"<<R_mat<<endl;
        cout<<"t matrix:"<<t_vec<<endl;

    }

}



void Pose_est::stereo_test(Mat &imgL, Mat &imgR)
{

    Mat imgDisparity16S = Mat( imgL.rows, imgL.cols, CV_16S );
    Mat imgDisparity8U = Mat( imgL.rows, imgL.cols, CV_8UC1 );
    int ndisparities=16*5;
    int SADWindowSize=21;
    StereoBM sbm(StereoBM::BASIC_PRESET,ndisparities,SADWindowSize);
    sbm( imgL, imgR, imgDisparity16S, CV_16S );
    double minVal=0;
    double maxVal=0;
    minMaxLoc( imgDisparity16S, &minVal, &maxVal );
    cout<<"Min disp:"<<minVal<<" Max value: "<<maxVal<<endl;
    //-- 4. Display it as a CV_8UC1 image
    imgDisparity16S.convertTo( imgDisparity8U, CV_8UC1, 255/(maxVal - minVal));

    const char *windowDisparity = "Disparity";
    namedWindow( windowDisparity, WINDOW_NORMAL );
    imshow( windowDisparity, imgDisparity8U );

    //-- 5. Save the image
    //imwrite("SBM_sample.png", imgDisparity16S);



    return ;
}


/*
 * ImageProcess class
 */


bool StereoImageProcess::SliceImage(const Mat &input, Mat &output, Point2f &top_left)
{

    Mat gray;
    Mat input_copy=input.clone();
    cvtColor(input_copy, gray, CV_BGR2GRAY);
    //Temporarily set threshold to 40
    //To be modified by some advanced method
    threshold(gray, gray,44, 255,THRESH_BINARY_INV); //Threshold the gray
    //Make black to white and vice versa
    bitwise_not(gray,gray);
    imshow("gray",gray);

    vector<vector<cv::Point> > contours;

    findContours( gray, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE );

    // iterate through each contour.
    int largest_contour_index=0;
    double largest_area=0;
    Rect bounding_rect;

    for( int i = 0; i< contours.size(); i++ )
    {
        //  Calc the area of contour

        double a=contourArea( contours[i],false);
        if(a>largest_area){
            largest_area=a;
            // Store the index of largest contour
            largest_contour_index=i;
            // Find the bounding rectangle for biggest contour
            bounding_rect=boundingRect(contours[i]);
            top_left=bounding_rect.tl();
        }
    }


/*
    drawContours( input, contours,largest_contour_index, Scalar( 255,255,255));
    rectangle(input, bounding_rect,  Scalar(0,255,0),2, 8,0);
    namedWindow( "Display window", CV_WINDOW_AUTOSIZE );
    imshow( "Display window", input );
*/
    input_copy=input.clone();
    output=input_copy(bounding_rect);
   // imshow("output",output);




}

void StereoImageProcess::ORB_Matching(Mat &img_L, Mat &img_R, int num_points,
                                vector<Point2f> &matched_points_L,vector<Point2f> &matched_points_R)
{

    int minHessian = 400;
    OrbFeatureDetector orb_detector(minHessian);
    //Step1:feature detection
    vector<KeyPoint> key_imgL,key_imgR;
    orb_detector.detect(img_L,key_imgL);
    orb_detector.detect(img_R,key_imgR);
    //Step2:compute
    OrbDescriptorExtractor orb_extractor;
    Mat descrip_imgL,descrip_imgR;
    orb_extractor.compute(img_L,key_imgL,descrip_imgL);
    orb_extractor.compute(img_R,key_imgR,descrip_imgR);
    //Step3:matching

    //cout<<"Debug info!!!!!!!!"<<endl;
    //FlannBasedMatcher matcher;
    BFMatcher matcher;
    std::vector< DMatch > matches;
    //cout<<"Debug info!!!!!!!!"<<endl;
    matcher.match(descrip_imgL,descrip_imgR,matches);
    /* TB improved
     * Step4:Find good match
     */
    vector< DMatch > good_matches;
    FindGoodMatches(matches,descrip_imgL,good_matches);





    Mat img_matches;
    //drawMatches(img_R,key_imgR,img_L,key_imgL,good_matches,img_matches);
    drawMatches( img_L, key_imgL, img_R, key_imgR,
                 good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                 vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
    imshow( "Good Matches", img_matches );

    long num_matches = good_matches.size();
    //Step5:Output the coordinates of matched points.
    //vector<Point2f> key_points_L;
    //vector<Point2f> key_points_R;
    //KeyPoint::convert(key_imgL,key_points_L);
    //KeyPoint::convert(key_imgR,key_points_R);
    //vector<Point2f> matched_points_L;
    //vector<Point2f> matched_points_R;

    for (int i=0;i<num_matches;i++)
    {
        //Tip:Queryidx refers the left input while trainIdx means the right.
        int idx_L=good_matches[i].queryIdx;
        int idx_R=good_matches[i].trainIdx;
        matched_points_L.push_back(key_imgL[idx_L].pt+corner_L);
        matched_points_R.push_back(key_imgR[idx_R].pt+corner_R);
    }
    /*debug info
     *
    cout<<"matched_points of the left image:"<<endl;
    for(auto i:matched_points_L)
        cout<<i<<" ";
    cout<<endl<<"matched_points of the right image:"<<endl;
    for(auto j:matched_points_R)
        cout<<j<<" ";
    */




    return;

}


bool StereoImageProcess::ImageInput(const Mat &img_L, Mat &out_img_L,const Mat &img_R,Mat &out_img_R)
{
    return ( SliceImage(img_L,out_img_L,this->corner_L) &&
             SliceImage(img_R,out_img_R,this->corner_R)  );
}

void StereoImageProcess::PrintCorners()
{

    cout<<"Corners for imgL:"<<corner_L<<endl;

    cout<<"Corners for imgR:"<<corner_R<<endl;

}



bool StereoImageProcess::DetectObject(Mat &src_img, Mat &obj_img)
{
    return false;
}

void StereoImageProcess::Featuremethod(Mat &image)
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


bool StereoImageProcess::FindGoodMatches(const vector<DMatch> &raw_matches, const Mat &img_descrip,vector<DMatch> &good_matches)
{
    double max_dist = 0; double min_dist = 100;
    for( int i = 0; i < img_descrip.rows; i++ )
    {
        double dist = raw_matches[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
    }

    /************Big bug here:descrip_imgL not descrip_imgR**********/
    for( int i = 0; i < img_descrip.rows; i++ )
    { if( raw_matches[i].distance <= max(3*min_dist, 0.02) )
        { good_matches.push_back( raw_matches[i]); }
    }
    /* Step4:Find good match
    // Temporarily method:Get 10 of the least ones.
    //cout<<"debug info"<<endl;
    //sort(matches.begin(),matches.end(), [](DMatch i,DMatch j){return (i.distance<j.distance);});
    //cout<<"debug info"<<endl;
    //Step4:draw matches
    //vector< DMatch > good_matches;

    //good_matches.resize(num_points);
    //copy_n(matches.begin(),num_points,good_matches.begin());
    /*debug info
     *
    cout<<"KeyPoints L:"<<key_imgL.size()<<"KeyPoints R:"<<key_imgR.size()<<endl;
    cout<<"matches size:"<<matches.size()<<" Good matches size:"<<good_matches.size()<<endl;

    for(auto i:matches)
        if(i.queryIdx<0)
            cout<<i.queryIdx<<" ";
    cout<<endl;
    */
    return true;
}

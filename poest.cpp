//
// Created by lab on 1/8/16.
//
#include "poest.h"
#include <iostream>
#include <chrono>
using namespace std;



/*
 * ImageProcess class
 */
//DetectExtract method
//To detect key points and descriptors corresponding to them
bool BasicImageProcess::DetectExtract( const Mat &img,vector<KeyPoint> &key_points, Mat &descrip,
                                       FEATURE_TYPE type, int minHessian)
{
    OrbFeatureDetector orb_detector(minHessian);
    //Step1:feature detection
    orb_detector.detect(img,key_points);
    //Step2:compute
    OrbDescriptorExtractor orb_extractor;
    orb_extractor.compute(img,key_points,descrip);
    return true;
}
void BasicImageProcess::BasicMatching(Mat &img_1, Mat &img_2, int max_points,
                                         vector<KeyPoint> &key_img1,vector<KeyPoint> &key_img2,
                                         Mat &descrip_1,Mat &descrip_2,
                                         vector<Point2f> &matched_points_1, vector<Point2f> &matched_points_2)
{
    DetectExtract(img_1, key_img1, descrip_1);
    DetectExtract(img_2, key_img2, descrip_2);
    //Step3:matching

    //cout<<"Debug info!!!!!!!!"<<endl;
    //FlannBasedMatcher matcher;

    BFMatcher matcher(NORM_HAMMING);
    std::vector< DMatch > matches;

    matcher.match(descrip_1,descrip_2,matches);

    /* TB improved
     * Step4:Find good match
     */
    vector< DMatch > good_matches;
    FindGoodMatches(matches,descrip_1,max_points,good_matches);
    Mat img_matches;
    //drawMatches(img_2,key_imgR,img_1,key_imgL,good_matches,img_matches);
    drawMatches( img_1, key_img1, img_2, key_img2,
                 good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                 vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
    imshow( "Good Matches", img_matches );

    //Step5:Output the coordinates of matched points.
    GetMatchCoords(good_matches,key_img1,key_img2,matched_points_1,matched_points_2);
    return;

}




bool BasicImageProcess::SliceImage(const Mat &input, Mat &output, Point2f &top_left)
{

    Mat gray;
    Mat input_copy=input.clone();
    cvtColor(input_copy, gray, CV_BGR2GRAY);
    //Temporarily set threshold to 40
    //To be modified by some advanced method
    threshold(gray, gray,44, 255,THRESH_BINARY_INV); //Threshold the gray
    //Make black to white and vice versa
    bitwise_not(gray,gray);
    //imshow("gray",gray);

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


bool BasicImageProcess::FindGoodMatches(vector<DMatch> &raw_matches,const Mat &img_descrip,
                                        int num_points,vector<DMatch> &good_matches)
{
    // Step4:Find good match
    // Temporarily method:Get 10 of the least ones.
    //cout<<"debug info"<<endl;
    sort(raw_matches.begin(),raw_matches.end(), [](DMatch i,DMatch j){return (i.distance<j.distance);});
    //cout<<"debug info"<<endl;
    //Step4:draw matches
    //vector< DMatch > good_matches;

    //good_matches.resize(num_points);
    //copy_n(matches.begin(),num_points,good_matches.begin());
    //debug info

    for(int i=0;i<num_points;i++)
    {
        good_matches.push_back(raw_matches[i]);
    }





    return true;
}

bool BasicImageProcess::GetMatchCoords(vector<DMatch> &matches, vector<KeyPoint> &key1, vector<KeyPoint> &key2,
                                       vector<Point2f> &matched_pts_1, vector<Point2f> &matched_pts_2)
{
    long num_matches = matches.size();
    //Step5:Output the coordinates of matched points.

    for (int i=0;i<num_matches;i++)
    {
        //Tip:Queryidx refers the left input while trainIdx means the right.
        int idx_L=matches[i].queryIdx;
        int idx_R=matches[i].trainIdx;
        matched_pts_1.push_back(key1[idx_L].pt);
        matched_pts_2.push_back(key2[idx_R].pt);
    }
    return true;
}
/*
 * StereoImageProcess class
 * Get 3D reconstruction from binocular visions
 */


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
bool StereoImageProcess::StereoConstruct(const vector<Point2f> &matched_points_L, const vector<Point2f> &matched_points_R,
                                         vector<Point3f> &world_points,const double baseline,const double f)
{
    //Get coords of the original image
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


void StereoImageProcess::stereo_test(Mat &imgL, Mat &imgR)
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


void ObjectTracker::Track(FeaturedImg &target)
{
    BFMatcher matcher(NORM_HAMMING);
    std::vector< DMatch > matches;
    //cout<<"Debug info!!!!!!!!"<<endl;
    matcher.match(refer.key_descrips,target.key_descrips,matches);




}

Point3f PoseEst::CalcWldCoord(const Mat &R, const Mat &t, const Point2f &img_coord)
{
    //Concat R&t to [R|t]
    Mat R_t;
    hconcat(R,t,R_t);
    Mat trans_mat=this->camera_matrix*R_t;
    //Since [img_coord 1]'=trans_mat*img_coord=K*[R|t]*[wld_coord 1]'
    //[wld_coord 1]'=pinv_mat*[img_coord 1]'
    SVD svd(trans_mat);
    Mat pinv_mat = svd.vt.t()*Mat::diag(1./svd.w)*svd.u.t();
    //Add an extra dimension to img_coord which makes it [img_coord 1]'
    cout<<pinv_mat<<endl;
    /*
    Mat img_pt=(Mat_<float>(3,1)<<img_coord.x,img_coord.y,1.);
    Mat wld_pt=pinv_mat*img_pt;
    //Reduce an dimension from [wld_coord 1]'
    return Point3f(wld_pt.at<float>(0,0),wld_pt.at<float>(1,0),wld_pt.at<float>(2,0));
     */
    return Point3f(0.,0.,0.);

}
void PoseEst::SolvePnP(const vector<Point2f> &image_coords, const vector<Point3f> &world_coords,
                       Mat &R_mat, Mat &t_vec)
{
    if(image_coords.size()==world_coords.size() && R_mat.rows==0)
    {
        /* When there are enough points collected,
         * calculate the matrix R|t with PnP algorithm
         */
        ///*counting time*/ std::chrono::time_point<std::chrono::system_clock> start, end;
        Mat R_Rod;
        //start=std::chrono::system_clock::now();
        solvePnP(world_coords, image_coords, this->camera_matrix, this->disto, R_Rod, t_vec, false, CV_EPNP);
        //end=std::chrono::system_clock::now();
        //std::chrono::duration<double> elapsed_seconds = end-start;
        Rodrigues(R_Rod,R_mat);
        //cout<<"solvePnP ITER time:"<<elapsed_seconds.count()<<endl;
        cout<<"R matrix:"<<R_mat<<endl;
        cout<<"t matrix:"<<t_vec<<endl;

    }

}

void PoseEst::MarkPtOnImg(Mat &img, const Point2f &img_coord)
{
    circle(img,img_coord,5,Scalar(255,0,0),-1);
}

void StereoImageProcess::OriginImgCoord(vector<Point2f> &pts_L,vector<Point2f> &pts_R)
{
    if(pts_L.size()!=pts_R.size())
    {
        cerr<<"Error input!"<<endl;
        return;
    }
    for(int i=0;i<pts_L.size();i++)
    {
        pts_L[i]=pts_L[i]+this->corner_L;
        pts_R[i]=pts_R[i]+this->corner_R;
    }

}

void StereoImageProcess::FeaturesMatching(Mat &img_1, Mat &img_2, int max_points,
                                         vector<Point2f> &matched_points_1, vector<Point2f> &matched_points_2)
{
    vector<KeyPoint> key_img1;
    vector<KeyPoint> key_img2;
    Mat descrip_1;
    Mat descrip_2;
    BasicImageProcess::BasicMatching(img_1, img_2, max_points,key_img1,key_img2,
                     descrip_1,descrip_2,matched_points_1, matched_points_2);
    OriginImgCoord(matched_points_1,matched_points_2);
}
FeaturedImg StereoImageProcess::FeaturesMatching(Mat &img_L, Mat &img_R, int max_points)
{
    vector<KeyPoint> key_L;
    vector<KeyPoint> key_R;
    Mat descrip_L,descrip_R;
    vector<Point2f> matched_points_L;
    vector<Point2f> matched_points_R;
    BasicImageProcess::BasicMatching(img_L,img_R,max_points,key_L,key_R,descrip_L,descrip_R,
                                        matched_points_L,matched_points_R);



    return  FeaturedImg(img_L, key_L, vector<Point3f>(),descrip_L);


}






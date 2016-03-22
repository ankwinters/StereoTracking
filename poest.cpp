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
    if(type==ORB_FEATURE)
    {
        OrbFeatureDetector orb_detector(minHessian);
        //Step1:feature detection
        orb_detector.detect(img, key_points);
        //Step2:compute
        OrbDescriptorExtractor orb_extractor;
        orb_extractor.compute(img, key_points, descrip);
    }
    else if(type==SIFT_FEATURE)
    {

        SiftFeatureDetector sift_detect( minHessian );

        sift_detect.detect( img, key_points );
        SiftDescriptorExtractor sift_extractor;
        sift_extractor.compute(img,key_points,descrip);
    }
    return true;
}
void BasicImageProcess::BasicMatching(Mat &img_1, Mat &img_2, int max_points,
                                         vector<KeyPoint> &key_img1,vector<KeyPoint> &key_img2,
                                         Mat &descrip_1,Mat &descrip_2, vector< DMatch > &good_matches,
                                         vector<Point2f> &matched_points_1, vector<Point2f> &matched_points_2)
{

    DetectExtract(img_1, key_img1, descrip_1);
    DetectExtract(img_2, key_img2, descrip_2);
    //Step3:matching

    //cout<<"Debug info!!!!!!!!"<<endl;

//    Ptr<flann::IndexParams> indexParams=new flann::LshIndexParams();
  //  FlannBasedMatcher matcher(indexParams);

    BFMatcher matcher(NORM_HAMMING);
    std::vector< DMatch > matches;

    matcher.match(descrip_1,descrip_2,matches);

    // TB improved
    //  Step4:Find good match
    FindGoodMatches(matches,key_img1,key_img2,max_points,good_matches);
    /*
    DetectExtract(img_1, key_img1, descrip_1,SIFT_FEATURE);
    DetectExtract(img_2, key_img2, descrip_2,SIFT_FEATURE);
    FlannBasedMatcher matcher;
    //std::vector< DMatch > good_matches;
    matcher.match(descrip_1,descrip_2,good_matches);
    */



    Mat img_matches;
    //drawMatches(img_2,key_imgR,img_1,key_imgL,good_matches,img_matches);
    drawMatches( img_1, key_img1, img_2, key_img2,
                 good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                 vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
    imshow( "Good Matches", img_matches );
    imwrite("../matches.jpg",img_matches );
    exit(0);


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
    //equalizeHist( gray, gray );
    //imshow("gray",gray);
    //imwrite( "../Gray_His.jpg", gray );

    threshold(gray, gray,44, 255,THRESH_BINARY_INV); //Threshold the gray
    //Make black to white and vice versa
    bitwise_not(gray,gray);
    //imshow("gray",gray);
    //imwrite( "../Gray_Image.jpg", gray );

    vector<vector<cv::Point> > contours;

    findContours( gray, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE );

    // iterate through each contour.
    int largest_contour_index=0;
    double largest_area=0;
    Rect bounding_rect;
    Point2f tl;
    Point2f br;

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
            //top_left=bounding_rect.tl();
            tl=bounding_rect.tl();
            br=bounding_rect.br();
        }
    }
    //do some change

    const int size=20;
    float x=0;
    float y=0;

    (tl.x-size>0)?(x=tl.x-size):(x=tl.x);
    (tl.y-size>0)?(y=tl.y-size):(y=tl.y);

    top_left=Point2f(x,y);

    (br.x+size<input.cols)?(x=br.x+size):(x=br.x);
    (br.y+size<input.rows)?(y=br.y+size):(y=br.y);

    bounding_rect=Rect(top_left,Point2f(x,y));



/*
    drawContours( input, contours,largest_contour_index, Scalar( 255,255,255));
    rectangle(input, bounding_rect,  Scalar(0,255,0),2, 8,0);
    namedWindow( "Display window", CV_WINDOW_AUTOSIZE );
    imshow( "Display window", input );
*/
    //drawContours( input, contours,largest_contour_index, Scalar( 255,255,255));
    input_copy=input.clone();
    //Implement Sharpening Filter
    Mat kernel = (Mat_<float>(3,3) << 0,-1,0,-1,5,-1,0,-1,0);
    filter2D(input_copy,input_copy,input.depth(),kernel);
    output=input_copy(bounding_rect);

   // imshow("output",output);
    //rectangle(input_copy, bounding_rect,  Scalar(0,255,0),2, 8,0);
    //imwrite("../out.jpg",input_copy);
    //imwrite( "../output.jpg", output );




}


bool BasicImageProcess::FindGoodMatches(vector<DMatch> &raw_matches,const vector<KeyPoint> &query_pts,
                                        const vector<KeyPoint> &train_pts,
                                        int num_points,vector<DMatch> &good_matches)
{
    //Define error=(y1-y2)/y1*100%
    //Then error<threshold should be determined.

    double threshold=0.02;
    int dist=50;
    vector<Point2f> matches_1;
    vector<Point2f> matches_2;

    for( int i = 0; i < raw_matches.size(); i++ )
    {
        matches_1.push_back(query_pts[raw_matches[i].queryIdx].pt);
        matches_2.push_back(train_pts[raw_matches[i].trainIdx].pt);

    }
    for (int i=0;i<matches_1.size();i++)
    {
        double y1=matches_1[i].y;
        double y2=matches_2[i].y;
        double diff=((y1-y2)<0)?(y2-y1):(y1-y2);
        if( diff/y1<threshold && raw_matches[i].distance<dist)
            good_matches.push_back(raw_matches[i]);
    }


    //Mask returned by findHomography is an 8-bit, single-channel cv::Mat
    //(or std::vector<uchar>, if you prefer) containing either 0 or 1 indicating the outlier status.
    /*
    vector<unsigned char> mask;
    Mat h = cv::findHomography(matches_1,matches_2,CV_RANSAC,0.5,mask);
    for(int i=0;i<matches_1.size();i++)
    {
        if(mask.at(i)!=0)
            good_matches.push_back(raw_matches[i]);
    }
    */


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
        Y=Z*matched_points_L[i].y*pixel_size/f;
        X=Z*matched_points_L[i].x*pixel_size/f;
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
void StereoImageProcess::OriginImgCoord(vector<Point2f> &pts_L,vector<Point2f> &pts_R)
{
    if(pts_L.size()!=pts_R.size())
    {
        cerr<<"Error input!"<<endl;
        return;
    }
    //Get the coordinates in the original image.
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
    vector< DMatch > good_matches;
    Mat descrip_1;
    Mat descrip_2;
    BasicImageProcess::BasicMatching(img_1, img_2, max_points,key_img1,key_img2,
                                     descrip_1,descrip_2,good_matches,matched_points_1, matched_points_2);
    OriginImgCoord(matched_points_1,matched_points_2);
}
FeaturedImg StereoImageProcess::Matching(Mat &img_L, Mat &img_R, int max_points,
                                         vector<Point2f> &matched_points_L, vector<Point2f> &matched_points_R)
{
    FeaturedImg left,right;
    left.img=img_L;
    right.img=img_R;
    left.top_left=this->corner_L;
    right.top_left=this->corner_R;
    vector< DMatch > matches;
    BasicImageProcess::BasicMatching(left.img, right.img, max_points,left.key_pts,right.key_pts,
                                     left.key_descrips,right.key_descrips,matches,
                                     matched_points_L, matched_points_R);
    //Get idx of the left image
    for(int i=1;i<matches.size();i++)
    {
        int idx=matches[i].queryIdx;
        left.matched_idx.push_back(idx);
    }
    OriginImgCoord(matched_points_L,matched_points_R);

    return left;


}

void ObjectTracker::Track(FeaturedImg &target)
{

    /*
     * Matching key points from two frames.
     * Key points from the referer should be refined
     * because of the previous binocular matching.
     */


    vector<KeyPoint> refer_keys;
    vector<KeyPoint> target_keys;
    for(int i=0;i<this->refer.matched_idx.size();i++)
    {
        int idx=this->refer.matched_idx[i];
        refer_keys.push_back(this->refer.key_pts[idx]);
    }

    for(int j=0;j<target.matched_idx.size();j++)
    {
        int idx=target.matched_idx[j];
        target_keys.push_back(target.key_pts[idx]);
    }



    OrbDescriptorExtractor orb_extractor;
    orb_extractor.compute(this->refer.img, refer_keys, this->refer.key_descrips);
    orb_extractor.compute(target.img, target_keys, target.key_descrips);

    BFMatcher matcher(NORM_HAMMING);
    vector< DMatch > matches;



    matcher.match(refer.key_descrips,target.key_descrips,matches);

    /*
     * Step4:Find good match
     */
    vector< DMatch > good_matches;
    RefineMatches(matches,good_matches);
    Mat img_matches;
    //drawMatches(img_2,key_imgR,img_1,key_imgL,good_matches,img_matches);
    drawMatches( refer.img, refer_keys, target.img, target_keys,
                 good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                 vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
    imshow( "Two frame Matches", img_matches );



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
    if(image_coords.size()==world_coords.size() )
       //&& R_mat.rows==0)
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
        cout<<"Camera matrix:"<<this->camera_matrix<<endl;

    }

}

void PoseEst::MarkPtOnImg(Mat &img, const Point2f &img_coord)
{
    circle(img,img_coord,5,Scalar(255,0,0),-1);
}


bool ObjectTracker::RefineMatches(const vector<DMatch> &raw_matches, vector<DMatch> &good_matches,
                                  FEATURE_TYPE type)
{
    int dist=55;
    for (int i=0;i<raw_matches.size();i++)
    {
        if( raw_matches[i].distance<dist)
            good_matches.push_back(raw_matches[i]);
    }


    return true;
}



void ObjectTracker::CalcMotions(vector<Point3f> &ref, vector<Point3f> &tgt, Mat &Rot, Mat &Tran)
{
    //* Calc with the method brought by Horn(1987)
    int num=3;//Number of points
    assert(ref.size()==num && tgt.size()==num);
    //Step1 : calc the centroid
    //calc the centroid of the points from previous frame & current one
    //Change points to the centroid-based coords
    Point3f Pp=CalcCentroid(ref);
    Point3f Pc=CalcCentroid(tgt);
    vector<Mat_<double> > priv;
    vector<Mat_<double> > curr;
    for(int i=0;i<num;i++)
    {
        priv.push_back(Mat_<double>(ref[i]-Pp));
        cout<<"priv["<<i<<"]:"<<priv[i]<<endl;
    }
    for(int i=0;i<num;i++)
    {
        curr.push_back(Mat_<double>(tgt[i]-Pc));
        cout<<"curr["<<i<<"]:"<<curr[i]<<endl;
    }
    /*

    //Debug info
    cout<<"Pp:"<<Pp<<endl;
    cout<<"Pc:"<<Pc<<endl;

    //Step2:Calc norm
    Mat Np=Mat::zeros(3,1,CV_64F);
    double len_Np=GetNormal(priv[0],priv[1],priv[2],Np);
    Mat Nc=Mat::zeros(3,1,CV_64F);
    double len_Nc=GetNormal(curr[0],curr[1],curr[2],Nc);
    Mat Na=Nc.cross(Np);

    //Step3: Calc qa & qp
    //Key:Find cos_half_fi,sin_half_fi & cos_half_th,sin_half_th
    //0<fi<Pi,
    //For qa
    double cos_fi=Nc.dot(Np);
    double cos_half_fi=sqrt((1+cos_fi)/2);
    double sin_half_fi=sqrt((1-cos_fi)/2);
    //Debug info
    cout<<"cos_fi:"<<cos_fi<<endl;
    cout<<"cos_half_fi:"<<cos_half_fi<<endl;
    cout<<"sin_half_fi:"<<sin_half_fi<<endl;
    Quaternion qa(cos_half_fi,
                 Na.at<double>(0,0)*sin_half_fi, Na.at<double>(1,0)*sin_half_fi, Na.at<double>(2,0)*sin_half_fi);
    //For qp, C&S need to be calculated first
    //Warning!!! Tgt Points need a rotation




    double C=0;
    for(int i=0;i<num;i++)
    {
        C+=curr[i].dot(priv[i]);
    }
    double S=0;
    for(int i=0;i<num;i++)
    {
        Mat temp=curr[i].cross(priv[i]);
        Mat temp_N=Mat_<double>(temp);
        //cout<<"Temp["<<i<<"]:"<<temp_N<<endl;
        S=S+Np.dot(temp_N);
    }

    double cos_th=C/sqrt(C*C+S*S);
    double cos_half_th=sqrt((1+cos_th)/2);
    double sin_half_th=sqrt((1-cos_th)/2);

    //Debug info
    cout<<"cos_th:"<<cos_th<<endl;
    cout<<"cos_half_th:"<<cos_half_th<<endl;
    cout<<"sin_half_th:"<<sin_half_th<<endl;

    Quaternion qp(cos_half_th,
                 Np.at<double>(0,0)*sin_half_th, Np.at<double>(1,0)*sin_half_th, Np.at<double>(2,0)*sin_half_th);
    //Step4:Calc quaternion-derived rotation matrix Ra &Rq

    Mat Ra=Mat::zeros(3,3,CV_64F);
    qa.ToRMat(Ra);
    Mat Rp=Mat::zeros(3,3,CV_64F);
    qp.ToRMat(Rp);
    Rot=Rp*Ra;

    //Debug info
    cout<<"Ra:"<<Ra<<endl;
    cout<<"Rp:"<<Rp<<endl;
    cout<<"R:"<<Rot<<endl;

    //Step6:Calc matrix T
    Mat _Poc,_Pop;
    _Poc=Mat_<double>(Pc);
    _Pop=Mat_<double>(Pp);
    //transpose(Mat(P_oc),_Poc);
    //transpose(Mat(P_op),_Pop);
    Tran=_Pop-Rot*_Poc;

    cout<<"Tran:"<<Tran<<endl;
     */




}
bool BasicImageProcess::Histogram(const Mat &input, Mat &output)
{
    return true;
}




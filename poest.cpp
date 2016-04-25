//
// Created by lab on 1/8/16.
//
#include "poest.h"
#include <iostream>
#include <chrono>
#include <iomanip>

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

        ORB orb_detector(minHessian);
        //Step1:feature detection
        orb_detector.detect(img, key_points);
        orb_detector.compute(img, key_points, descrip);
        //Step2:compute
        //ORB orb_extractor;
        //orb_extractor.compute(img, key_points, descrip);
        //FREAK freak_detector;

        //freak_detector.compute(img,key_points,descrip);
    }
    else if(type==SIFT_FEATURE)
    {

        SIFT sift_detect( minHessian );
        sift_detect.detect( img, key_points );
        sift_detect.compute(img,key_points,descrip);
    }
    return true;
}


//***************
//* Only extract
//***************
bool BasicImageProcess::Extract(const Mat &img, vector<KeyPoint> &key_points, Mat &descrip,
                                FEATURE_TYPE type)
{
    if(type==ORB_FEATURE)
    {

        ORB orb_detector;
        orb_detector.compute(img, key_points, descrip);

    }
    else if(type==SIFT_FEATURE)
    {

        SIFT sift_detect;
        sift_detect.compute(img,key_points,descrip);

    }
    return true;
}

void BasicImageProcess::BasicMatching(Mat &img_1, Mat &img_2, int max_points,
                                      vector<KeyPoint> &key_img1,vector<KeyPoint> &key_img2,
                                      Mat &descrip_1,Mat &descrip_2, vector< DMatch > &good_matches,
                                      vector<Point2d> &matched_points_1, vector<Point2d> &matched_points_2)
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


    //Step5:Output the coordinates of matched points.
    GetMatchCoords(good_matches,key_img1,key_img2,matched_points_1,matched_points_2);
    return;

}

void BasicImageProcess::BasicMatching(FEATURE_TYPE type,Mat &img_1, Mat &img_2,
                                      vector<KeyPoint> &key_img1, vector<KeyPoint> &key_img2, Mat &descrip_1, Mat &descrip_2,
                                      vector<DMatch> &good_matches, Mat &matched_img)
{
    cout<<"Debug info!!!!!!!!"<<endl;
    DescriptorMatcher *matcher= nullptr;
    DetectExtract(img_1, key_img1, descrip_1,type);
    DetectExtract(img_2, key_img2, descrip_2,type);
    //Step3:matching
    //cout<<"Debug info!!!!!!!!"<<endl;
    std::vector<DMatch> matches;

    cout<<"img1 cols:"<<img_1.cols<<" img2:cols:"<<img_2.cols<<endl;
    if(type==ORB_FEATURE)
    {
        matcher=new BFMatcher(NORM_HAMMING);

    }
    else if(type==SIFT_FEATURE)
    {
        matcher=new FlannBasedMatcher;

    }
    else
       exit(-1);
    matcher->match(descrip_1, descrip_2, matches);
    delete matcher;


    // TB improved
    //  Step4:Find good match
    int max_points=100;
    FindGoodMatches(matches,key_img1,key_img2,max_points,good_matches,type);
    //drawMatches(img_2,key_imgR,img_1,key_imgL,good_matches,img_matches);
    drawMatches( img_1, key_img1, img_2, key_img2,
                 good_matches, matched_img, Scalar::all(-1), Scalar::all(-1),
                 vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
    //imshow( "Good Matches", matched_img);
    //imwrite("../matches.jpg",matched_img );

}



bool BasicImageProcess::SliceImage(const Mat &input, Mat &output, Point2d &top_left)
{

    cvtColor(input, output, CV_BGR2GRAY);

    cout<<"Any problem here?"<<endl;
    //output=input.clone();
    top_left=Point2d(0,0);


    //equalizeHist( output,output );

     //imshow("output",output);
    //rectangle(input_copy, bounding_rect,  Scalar(0,255,0),2, 8,0);
    //imwrite("../out.jpg",input_copy);
    //imwrite( "../output.jpg", output );

}


bool BasicImageProcess::FindGoodMatches(vector<DMatch> &raw_matches,const vector<KeyPoint> &query_pts,
                                        const vector<KeyPoint> &train_pts,
                                        int num_points,vector<DMatch> &good_matches,FEATURE_TYPE type)
{
    //Define error=(y1-y2)/y1*100%
    //Then error<threshold should be determined.

    double threshold=0.05;

    vector<Point2d> matches_1;
    vector<Point2d> matches_2;

    for( int i = 0; i < raw_matches.size(); i++ )
    {
        matches_1.push_back(query_pts[raw_matches[i].queryIdx].pt);
        matches_2.push_back(train_pts[raw_matches[i].trainIdx].pt);

    }
    if(type==ORB_FEATURE)
    {

        int      dist = 30;
        for (int i = 0; i < raw_matches.size(); i++)
        {
            double y1   = matches_1[i].y;
            double y2   = matches_2[i].y;
            double diff = ((y1 - y2) < 0) ? (y2 - y1) : (y1 - y2);
            if (diff / y1 < threshold && raw_matches[i].distance < dist)
                good_matches.push_back(raw_matches[i]);
        }
        return true;
    }
    else if(type==SIFT_FEATURE)
    {
        double max_dist = 0; double min_dist = 100;
        //-- Quick calculation of max and min distances between keypoints
        for( int i = 0; i<raw_matches.size(); i++ )
        {
            double dist = raw_matches[i].distance;
            if( dist < min_dist ) min_dist = dist;
            if( dist > max_dist ) max_dist = dist;
        }

        //-- Draw only "good" matches (i.e. whose distance is less than 0.6*max_dist )
        //-- PS.- radiusMatch can also be used here.
        for( int i = 0; i<raw_matches.size(); i++ )
        {
            double y1   = matches_1[i].y;
            double y2   = matches_2[i].y;
            double diff = ((y1 - y2) < 0) ? (y2 - y1) : (y1 - y2);
            if( diff / y1 < threshold && raw_matches[i].distance < 0.2*max_dist )
            {
                good_matches.push_back( raw_matches[i]);
            }
        }
        return true;

    }
    return false;


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
}

bool BasicImageProcess::GetMatchCoords(vector<DMatch> &matches, vector<KeyPoint> &key1, vector<KeyPoint> &key2,
                                       vector<Point2d> &matched_pts_1, vector<Point2d> &matched_pts_2)
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



bool StereoImageProcess::StereoConstruct(const vector<Point2d> &matched_points_L, const vector<Point2d> &matched_points_R,
                                         vector<Point3d> &world_points,const double baseline,const double f,const double pixel_size)
{
    //Get coords of the original image
    size_t num_points=matched_points_L.size();
    //double pixel_size=4.65e-3;
    //Z=b*f/d
    double d,Z,Y,X;
    //cout<<"debug info.."<<endl;

    for(size_t i=0;i<num_points;i++)
    {
        //Transform them into the same world coordinate.
        d=matched_points_L[i].x-matched_points_R[i].x;
        Z=baseline*f/(d*pixel_size);
        Y=pixel_size*Z*(matched_points_L[i].y-this->camera_matrix.at<double>(1,2))/f;
        X=pixel_size*Z*(matched_points_L[i].x-this->camera_matrix.at<double>(0,2))/f;
        //cout<<"X["<<i<<"]:"<<X<<endl;
        //Cartesian coordinate system.(0,0,0) is right at perspective point
        world_points.push_back(Point3d(-X,-Y,Z));
    }

    return true;
}

bool StereoImageProcess::StereoConstruct(FeaturedImg &left, const FeaturedImg &right,
                                         vector<Point2d> &matched_points_L,vector<Point3d> &world_points,
                                         const double baseline, const double f, const double pixel_size)
{

    //Z=b*f/d
    for(auto idx:left.matched_idx)
    {
        matched_points_L.push_back(left.key_pts[idx].pt);
    }

    vector<Point2d> matched_points_R;
    for(auto idx:right.matched_idx)
    {
        matched_points_R.push_back(right.key_pts[idx].pt);
    }
    OriginImgCoord(matched_points_L,matched_points_R);

    double d,Z,Y,X;
    for(size_t i=0;i<matched_points_L.size();i++)
    {
        d=matched_points_L[i].x-matched_points_R[i].x;
        Z=baseline*f/(d*pixel_size);
        Y=pixel_size*Z*(matched_points_L[i].y-this->camera_matrix.at<double>(1,2))/f;
        X=pixel_size*Z*(matched_points_L[i].x-this->camera_matrix.at<double>(0,2))/f;
        world_points.push_back(Point3d(-X,-Y,Z));
        left.matched_3d.push_back(Point3d(-X,-Y,Z));
    }

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
void StereoImageProcess::OriginImgCoord(vector<Point2d> &pts_L,vector<Point2d> &pts_R)
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

void StereoImageProcess::FeaturesMatching(FeaturedImg &left,FeaturedImg &right,Mat &img_matches,FEATURE_TYPE type)
{


    left.top_left=this->corner_L;
    right.top_left=this->corner_R;
    vector< DMatch > matches;
    BasicImageProcess::BasicMatching(type,left.img, right.img, left.key_pts,right.key_pts,
                                     left.key_descrips,right.key_descrips,matches,
                                     img_matches);
    //Get idx of the left image
    for(int i=0;i<matches.size();i++)
    {
        int idx_L=matches[i].queryIdx;
        int idx_R=matches[i].trainIdx;
        left.matched_idx.push_back(idx_L);
        right.matched_idx.push_back(idx_R);
    }


}




FeaturedImg StereoImageProcess::Matching(Mat &img_L, Mat &img_R, int max_points,
                                         vector<Point2d> &matched_points_L, vector<Point2d> &matched_points_R)
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
    for(int i=0;i<matches.size();i++)
    {
        int idx=matches[i].queryIdx;
        left.matched_idx.push_back(idx);
    }
    OriginImgCoord(matched_points_L,matched_points_R);

    return left;


}

void ObjectTracker::Track(FeaturedImg &target,vector<DMatch> &good_matches,FEATURE_TYPE type)
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

    DescriptorMatcher *matcher= nullptr;

    Extract(this->refer.img,refer_keys,this->refer.key_descrips,type);
    Extract(target.img,target_keys,target.key_descrips,type);

    vector< DMatch > matches;
    if(type==ORB_FEATURE)
    {
        matcher=new BFMatcher(NORM_HAMMING);

    }
    else if(type==SIFT_FEATURE)
    {
        matcher=new FlannBasedMatcher;

    }
    else
        exit(-1);

    matcher->match(refer.key_descrips,target.key_descrips,matches);
    delete matcher;

    //matcher.match(refer.key_descrips,target.key_descrips,matches);


    /*
     * Step4:Find good match
     */
    //vector< DMatch > good_matches;
    RefineMatches(matches,good_matches,type);
    Mat img_matches;
    //drawMatches(img_2,key_imgR,img_1,key_imgL,good_matches,img_matches);
    drawMatches( refer.img, refer_keys, target.img, target_keys,
                 good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                 vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
    imshow( "Two frame Matches", img_matches );



}


void PoseEst::SolvePnP(const vector<Point2d> &image_coords, const vector<Point3d> &world_coords,
                       Mat &R_mat, Mat &t_vec)
{
    if( (image_coords.size()!=0) && (image_coords.size()==world_coords.size()) )
        //&& R_mat.rows==0)
    {
        /* When there are enough points collected,
         * calculate the matrix R|t with PnP algorithm
         */
        ///*counting time*/ std::chrono::time_point<std::chrono::system_clock> start, end;
        Mat R_Rod;
        //start=std::chrono::system_clock::now();
        //solvePnP(world_coords, image_coords, this->camera_matrix, this->disto, R_Rod, t_vec, false, CV_EPNP);
        //solvePnP(world_coords, image_coords, this->camera_matrix, this->disto, R_Rod, t_vec, false, CV_ITERATIVE);
        solvePnPRansac(world_coords, image_coords, this->camera_matrix, this->disto, R_Rod, t_vec,false,500,3.0);
        //end=std::chrono::system_clock::now();
        //std::chrono::duration<double> elapsed_seconds = end-start;
        Rodrigues(R_Rod,R_mat);
        //cout<<"solvePnP ITER time:"<<elapsed_seconds.count()<<endl;
        auto mat_print=[](Mat &a){
            //cout<<"[";
            for(int i=0; i<a.rows; i++)
            {
                for (int j = 0; j < a.cols; j++)
                    cout << fixed<<setprecision(4) << a.at<double>(i, j) << ",";
                cout<<endl;
            }
        };

       // cout<<"R :";
       // mat_print(R_mat);
       // cout<<"t :";
      //  mat_print(t_vec);
        Mat R_t;
        hconcat(R_mat,t_vec,R_t);
        cout<<endl;
        mat_print(R_t);

        auto repro_error=[&]()
        {
            vector<Point2d> projected;
            double totalErr=0;
            projectPoints(world_coords,R_Rod,t_vec,this->camera_matrix,this->disto,projected);
            for(int i = 0; i < (int)image_coords.size(); ++i )
            {


                auto err = norm(Mat(projected[i]), Mat(image_coords[i]), CV_L2);              // difference
                //cout<<"image orignal:"<<image_coords[i]<<" projected:"<<projected[i]<<" error"<<err<<endl;

                totalErr        += err*err;                                             // sum it up
            }
            return totalErr;

        };
        cout<<"Reprojection error:"<<repro_error()<<endl;

        cout<<"Camera matrix:"<<this->camera_matrix<<endl;

    }

}

void PoseEst::PnPCheck(FeaturedImg &left,Mat &R_mat, Mat &t_vec)
{
    if( (left.matched_3d.size()!=0) && (left.matched_3d.size()==left.matched_idx.size()) )
        //&& R_mat.rows==0)
    {
        /* When there are enough points collected,
         * calculate the matrix R|t with PnP algorithm
         */
        vector<Point2d> image_coords;
        for(int i=0;i<left.matched_idx.size();i++)
        {
            image_coords.push_back(left.key_pts[left.matched_idx[i]].pt);
        }
        Mat R_Rod;
        vector<int> inliers;

        solvePnPRansac(left.matched_3d, image_coords, this->camera_matrix, this->disto, R_Rod, t_vec,false,500,3.0,0.99,inliers);
        //end=std::chrono::system_clock::now();
        //std::chrono::duration<double> elapsed_seconds = end-start;
        Rodrigues(R_Rod,R_mat);
        //cout<<"solvePnP ITER time:"<<elapsed_seconds.count()<<endl;
        //cout<<"debug info"<<endl;

        for(int i=0,idx=0;i<inliers.size();i++,idx++)
        {

            while(idx<inliers[i])
            {
                left.matched_idx[idx]=left.matched_idx.back();
                left.matched_3d[idx]=left.matched_3d.back();
                image_coords[idx]=image_coords.back();

                left.matched_3d.pop_back();
                left.matched_idx.pop_back();
                image_coords.pop_back();
                idx++;

                //left.matched_idx.erase(left.matched_idx.begin()+idx-erased);
                //left.matched_3d.erase(left.matched_3d.begin()+idx-erased);
                //optional
                //image_coords.erase(image_coords.begin()+idx-erased);
                //erased++;
                //idx++;
            }

        }






        //print Matrix
        auto mat_print=[](Mat &a){
            //cout<<"[";
            for(int i=0; i<a.rows; i++)
            {
                for (int j = 0; j < a.cols; j++)
                    cout << fixed<<setprecision(4) << a.at<double>(i, j) << ",";
                cout<<endl;
            }
        };

        // cout<<"R :";
        // mat_print(R_mat);
        // cout<<"t :";
        //  mat_print(t_vec);
        Mat R_t;
        hconcat(R_mat,t_vec,R_t);
        cout<<endl;
        mat_print(R_t);

        auto repro_error=[&](vector<Point3d> &world_coords)
        {
            vector<Point2d> projected;
            double totalErr=0;
            projectPoints(world_coords,R_Rod,t_vec,this->camera_matrix,this->disto,projected);
            for(int i = 0; i < (int)image_coords.size(); ++i )
            {


                auto err = norm(Mat(projected[i]), Mat(image_coords[i]), CV_L2);              // difference
                cout<<"image orignal:"<<image_coords[i]<<" projected:"<<projected[i]<<" error"<<err<<endl;

                totalErr        += err*err;                                             // sum it up
            }
            return totalErr;

        };
        cout<<"Reprojection error:"<<repro_error(left.matched_3d)<<endl;



        cout<<"Camera matrix:"<<this->camera_matrix<<endl;

    }

}

void PoseEst::MarkPtOnImg(Mat &img, const Point2d &img_coord)
{
    circle(img,img_coord,5,Scalar(255,0,0),-1);
}


bool ObjectTracker::RefineMatches(const vector<DMatch> &raw_matches, vector<DMatch> &good_matches,
                                  FEATURE_TYPE type)
{
    int dist=40;
    //Refine with hamming dist
    for (int i=0;i<raw_matches.size();i++)
    {
        if( raw_matches[i].distance<dist)
            good_matches.push_back(raw_matches[i]);
    }


    return true;
}



bool ObjectTracker::CalcMotions(vector<Point3d> &ref, vector<Point3d> &tgt, Mat &Rot, Mat &Tran)
{
    //* Calc with the method brought by Horn(1987)
    int num=3;//Number of points
    if(ref==tgt)
    {
        Rot=Mat::eye(3,3,CV_64F);
        Tran=Mat::zeros(3,1,CV_64F);
        return true;
    }
    //Three points should not belong to the same line
    if( !LineCheck(ref) && !LineCheck(tgt) )
        return false;

    //Cannot form a line

    //Step1 : calc the centroid
    //calc the centroid of the points from previous frame & current one
    //Change points to the centroid-based coords
    Point3d Pp=CalcCentroid(ref);
    Point3d Pc=CalcCentroid(tgt);
    vector<Mat_<double> > priv;
    vector<Mat_<double> > curr;
    for(int i=0;i<num;i++)
    {
        priv.push_back(Mat_<double>(ref[i]-Pp));
        //cout<<"priv["<<i<<"]:"<<priv[i]<<endl;
    }
    for(int i=0;i<num;i++)
    {
        curr.push_back(Mat_<double>(tgt[i]-Pc));
        //cout<<"curr["<<i<<"]:"<<curr[i]<<endl;
    }


    //Debug info
    //cout<<"Pp:"<<Pp<<endl;
    // cout<<"Pc:"<<Pc<<endl;

    //Step2:Calc norm
    Mat Np=Mat::zeros(3,1,CV_64F);
    double len_Np=GetNormal(priv[0],priv[1],priv[2],Np);
    Mat Nc=Mat::zeros(3,1,CV_64F);
    double len_Nc=GetNormal(curr[0],curr[1],curr[2],Nc);


    //Debug info
    // cout<<"Np:"<<Np<<endl;
    // cout<<"Nc:"<<Nc<<endl;
    // cout<<"Na:"<<Na<<endl;

    //Step3: Calc qa & qp
    //Key:Find cos_half_fi,sin_half_fi & cos_half_th,sin_half_th
    //0<fi<Pi/2,
    double cos_fi=Nc.dot(Np);
    // 0<fi<Pi 0<fi/2<Pi/2
    //if(cos_fi<0)
    //{
    //    cos_fi=-cos_fi;
    //    Np = -Np;
    //}
    double cos_half_fi=sqrt((1+cos_fi)/2);
    double sin_half_fi=sqrt((1-cos_fi)/2);

    //Debug info
    // cout<<"cos_fi:"<<cos_fi<<endl;
    // cout<<"cos_half_fi:"<<cos_half_fi<<endl;
    // cout<<"sin_half_fi:"<<sin_half_fi<<endl;

    Mat Na=Nc.cross(Np);
    Quaternion qa(cos_half_fi,
                  Na.at<double>(0,0)*sin_half_fi, Na.at<double>(1,0)*sin_half_fi, Na.at<double>(2,0)*sin_half_fi);
    Mat Ra=Mat::zeros(3,3,CV_64F);
    qa.ToRMat(Ra);
    //For qp, C&S need to be calculated first
    //Warning!!! Tgt Points need a rotation to stay in the same plane as Ref
    //********************************************
    for(int i=0;i<num;i++)
    {
        curr[i]=Ra*curr[i];
    }
    //********************************************
    //Now points from two set are in the same plane
    double C=0;
    for(int i=0;i<num;i++)
    {
        C+=curr[i].dot(priv[i]);
    }
    double S=0;
    for(int i=0;i<num;i++)
    {
        Mat temp=curr[i].cross(priv[i]);
        //Mat temp_N=Mat_<double>(temp);
        //cout<<"Temp["<<i<<"]:"<<temp_N<<endl;
        S=S+Np.dot(temp);
    }
    //Find cos_half_th,sin_half_th to calc qp

    double cos_th=C/sqrt(C*C+S*S);

    double cos_half_th=sqrt((1+cos_th)/2);
    double sin_half_th=sqrt((1-cos_th)/2);
    //**Warning: it possible that th>Pi,that is to say sin_th<0 or S<0
    if(S<0)
        cos_half_th=-cos_half_th;

    //Debug info
    //cout<<"cos_th:"<<cos_th<<endl;
    //cout<<"cos_half_th:"<<cos_half_th<<endl;
    //cout<<"sin_half_th:"<<sin_half_th<<endl;

    Quaternion qp(cos_half_th,
                  Np.at<double>(0,0)*sin_half_th, Np.at<double>(1,0)*sin_half_th, Np.at<double>(2,0)*sin_half_th);
    //Step4:Calc quaternion-derived rotation matrix Ra &Rq

    Mat Rp=Mat::zeros(3,3,CV_64F);
    qp.ToRMat(Rp);
    Rot=Rp*Ra;

    //Debug info
    //cout<<"Ra:"<<Ra<<endl;
    //cout<<"Rp:"<<Rp<<endl;
    //cout<<"Relative R:"<<Rot<<endl;

    //Step6:Calc matrix T
    Mat _Poc,_Pop;
    _Poc=Mat_<double>(Pc);
    _Pop=Mat_<double>(Pp);
    Tran=_Pop-Rot*_Poc;
    //cout<<"Relative T:"<<Tran<<endl;
    return true;
}


bool ChessboardGTruth::FindCorners(const Mat &input,  vector<Point2d> &corners)
{
    bool found;
    vector<Point2f> cor;
    found=findChessboardCorners( input, board_size, cor,
                                 CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK );


    if(found)
    {

        Mat viewGray;
        //cout<<"debug info"<<endl;
        cvtColor(input, viewGray, CV_BGR2GRAY);
        //**Bug:cornerSubPix() only support Point2f
        cornerSubPix( viewGray, cor, Size(11,11),
                      Size(-1,-1), TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
        //debug info
        //cout<<"debug.."<<endl;
        //Mat view=input.clone();
        //circle(view,corners[8],5,Scalar(255,0,0),-1);
        //imshow("view",view);
        for(int i=0;i<cor.size();i++)
            corners.push_back(Point2d(cor[i].x,cor[i].y));


        return true;
    }
    else
        return false;
}

void ChessboardGTruth::OneFrameTruth(const Mat &left, const Mat &right, Mat &R, Mat &T)
{
    vector<Point2d> matched_L;
    vector<Point2d> matched_R;
    vector<Point3d> world_coords;

    if(FindCorners(left,matched_L) &&
       FindCorners(right,matched_R))
    {


        StereoConstruct(matched_L,matched_R,world_coords);
        PoseEst poest;
        poest.SolvePnP(matched_L,world_coords,R,T);
    }
    return;

}

void ChessboardGTruth::OneFrameTruth(const Mat &left, const Mat &right, Mat &R, Mat &T, vector<Point2d> &matched_L,
                                     vector<Point3d> &world)
{
    vector<Point2d> matched_R;
    if(FindCorners(left,matched_L) &&
       FindCorners(right,matched_R))
    {
        StereoConstruct(matched_L,matched_R,world);
        PoseEst poest;
        poest.SolvePnP(matched_L,world,R,T);
    }
    return;

}

void ChessboardGTruth::OneFrameTruth(const Mat &left, const Mat &right, Mat &R, Mat &T, vector<Point2d> &matched_L,
                                     vector<Point2d> &matched_R, vector<Point3d> &world)
{
    //FindCorners(right,matched_R);

    if( (FindCorners(left,matched_L)) &&
                (FindCorners(right,matched_R)) )
    {
        StereoConstruct(matched_L,matched_R,world);
        PoseEst poest;
        poest.SolvePnP(matched_L,world,R,T);
        //cout<<"Debug info..."<<endl;
        return;
    }
   // exit(-1);
}

void ChessboardGTruth::FramesTruth(const Mat &first, const Mat &second, Mat &R, Mat &T)
{

}

double ObjectTracker::CalcRTerror(const Mat &R, const Mat &T,const vector<Point3d> &ref,const vector<Point3d> &tgt,
                                  vector<double> &err)
{
    assert(ref.size()==tgt.size());

    double error=0;

    for(int i=0;i<tgt.size();i++)
    {
        //cout<<"debug info"<<endl;
        Mat RT_tgt=Mat::zeros(3,1,CV_64F);
        RT_tgt=R*Mat_<double>(tgt[i])+T;
        Mat RT_ref=Mat::zeros(3,1,CV_64F);
        RT_ref=Mat_<double>(ref[i]);
        /*
        double x=RT_ref.at<double>(0,0)-RT_tgt.at<double>(0,0);
        double y=RT_ref.at<double>(1,0)-RT_tgt.at<double>(1,0);
        double z=RT_ref.at<double>(2,0)-RT_tgt.at<double>(2,0);
        double errori=x*x+y*y+z*z;
         */
        double errori = norm(RT_ref, RT_tgt, CV_L2);

        //cout<<"Matched ["<<i<<"] error:"<<errori<<endl;
        err.push_back(errori);
        error+=errori;
    }
    cout<<"errors:"<<error<<endl;
    return error;

}

bool ObjectTracker::RansacMotion(const vector<Point3d> &priv, const vector<Point3d> &curr, Mat &Rot, Mat &Tran,
                                 int iteration, double err_threash,double inlier_percent)
{
    assert(priv.size()==curr.size());
    RNG _random((unsigned)time(NULL));

    Mat _R,_T;

    vector<double> errors;
    double max_inlier_rate=inlier_percent;
    //Maybe unnecessary
    vector<int> inliers;
    double min_errors=99999999.0;


    int max_iterations=1000000;


    //Start Ransac
    for(int i=0;i<iteration;i++)
    {
        //choose 3 random items from the whole set
        errors.clear();
        inliers.clear();
        int idx[3]={0,0,0};
        while(idx[0]==idx[1] || idx[1]==idx[2] || idx[2]==idx[0])
        {
            idx[0] = _random.next() % (int) priv.size();
            idx[1] = _random.next() % (int) priv.size();
            idx[2] = _random.next() % (int) priv.size();
        }
        vector<Point3d> ref;
        vector<Point3d> tgt;
        for(int j=0;j<3;j++)
        {
            ref.push_back(priv[idx[j]]);
            tgt.push_back(curr[idx[j]]);
        }
        //Calc model
        if(!CalcMotions(ref,tgt,_R,_T))
        {
            iteration++;
            continue;
        }
        double sum_err=CalcRTerror(_R,_T,priv,curr,errors);
        //Divide the set into inliers and outliers
        for(int j=0;j<errors.size();j++)
        {
            if(errors[j]<err_threash)
            {
                inliers.push_back(j);
            }
        }
        //Check if the model is well fit

        //**warning:must cast following types into double
        double rate=(double)inliers.size()/(double)curr.size();
        cout<<"Current ["<<i<<"] rate:"<<rate<<endl;
        if(rate>=max_inlier_rate)
        {
            //Good fit
            //Maybe all inliers
            if(sum_err<min_errors)
            {
                Rot             = _R;
                Tran            = _T;
                max_inlier_rate = rate;
                min_errors=sum_err;
            }
        }
        else if(iteration-i<10 && max_inlier_rate==inlier_percent && iteration<max_iterations)
            iteration++;

    }




    if(max_inlier_rate==inlier_percent && iteration==max_iterations)
        return false;
    //Debug info

    Mat_<double> R_t;
    hconcat(Rot,Tran,R_t);
    auto mat_print=[](Mat &a){
        //cout<<"[";
        for(int i=0; i<a.rows; i++)
        {
            for (int j = 0; j < a.cols; j++)
                cout << fixed<<setprecision(4) << a.at<double>(i, j) << ",";
            cout<<endl;
        }
    };


    cout<<"inliers rate:"<<max_inlier_rate<<" Best errors:"<<min_errors<<" Best R|t:"<<endl;
    mat_print(R_t);
    return (min_errors==99999999.0);

}




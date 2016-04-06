#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>
#include <fstream>
#include <chrono>

#include "poest.h"
//#include "pointcloud.h"
using namespace cv;
using namespace std;

/*  Pose estimation
 *  s*m'=K*[R|t]*M
 *  Since K can be acquired by camera calibration with a chessboard
 *  A set of ways to estimate [R|t] are implemented.
 */

Mat image;
vector<Point3d> world_coord;
//vector<Point2d> image_coord;
vector<Point3d> world_coord_2;
Mat R,t;
Mat R2,t2;


Mat imgL,imgR;
FeaturedImg image_L,image_R;
FeaturedImg image_L2,image_R2;
Mat imgL_2,imgR_2;
vector<Point2d> matches_L,matches_R;
vector<Point2d> matches_L2,matches_R2;
vector<Point3d> matches_P,matches_N;
PoseEst poseEst;


FeaturedImg first;
void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
    if  ( event == EVENT_LBUTTONDOWN )
    {
        cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
        //poseEst.PnPmethod(x,y);
        //poseEst.Featuremethod();
        //poseEst.stereo_test(imgL,imgR);
        if(R.rows==0)
        {

            StereoImageProcess imgprocess;

            Mat img_matched;

            //Timer starts.
            auto start=std::chrono::system_clock::now();

            ChessboardGTruth gt;

            //gt.FindCorners(imgR,matches_R);
            gt.OneFrameTruth(imgL,imgR,R,t,matches_L,matches_R,world_coord);

            gt.OneFrameTruth(imgL_2,imgR_2,R2,t2,matches_L2,matches_R2,world_coord_2);
            ObjectTracker tk;

            tk.RansacMotion(world_coord,world_coord_2,R,t);

            //tk.CalcMotions(a,b,R,t);
            //tk.CalcRTerror(R,t,world_coord,world_coord_2);


            //imgprocess.ImageInput(imgL,image_L.img,imgR,image_R.img);
            //imgprocess.FeaturesMatching(image_L,image_R,img_matched);
            //imgprocess.StereoConstruct(image_L,image_R,matches_L,world_coord);

            //poseEst.SolvePnP(matches_L, world_coord, R, t);
            //Timer ends.
            auto end=std::chrono::system_clock::now();
            std::chrono::duration<double> elapsed_seconds = end-start;
            cout<<"Pose estimation time:"<<elapsed_seconds.count()<<endl;


            /**For debugging.Input all data into a file**/
            fstream file;
            file.open("points.txt", std::fstream::in | std::fstream::out | std::fstream::trunc);
            file << "matched_points of the left image:" << endl;
            for (auto i:matches_L)
                file << i << " ";
            file << endl << "matched_points of the right image:" << endl;
            for (auto j:matches_R)
                file << j << " ";
            file << endl << "3D reconstruction:" << endl;
            for (auto k:world_coord)
                file << k << " ";
            file << endl << "3D reconstruction2:" << endl;
            for (auto k:world_coord_2)
                file << k << " ";
            file << "\n"<<endl ;
            file.close();
        }
        //For debug
        else
        {
            /*
            poseEst.MarkPtOnImg(image,Point2d(x,y));
            Point3d A=poseEst.CalcWldCoord(R,t,Point2d(x,y));
            cout<<"Let's see Point 3d:"<<A<<endl;
            */
            cout<<"can't run?"<<endl;
        }


    }
    if  ( event == EVENT_RBUTTONDOWN )
    {
        cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;

        StereoImageProcess imgprocess;
        Mat img_matched;
        /**For debugging.Input all data into a file**/
        //imgprocess.ImageInput(imgL_2,imgL_2,imgR_2,imgR_2);
        //imgprocess.PrintCorners();

        //imgprocess.FeaturesMatching(imgL, imgR, 5, matches_L, matches_R);

        //auto second=imgprocess.Matching(imgL_2,imgR_2,5,matches_L2,matches_R2);
        //imgprocess.StereoConstruct(matches_L2, matches_R2, world_coord_2, 120.0, 2.5);
        //poseEst.SolvePnP(matches_L2, world_coord_2, R, t);
        auto start=std::chrono::system_clock::now();
        imgprocess.ImageInput(imgL,image_L.img,imgR,image_R.img);
        imgprocess.FeaturesMatching(image_L,image_R,img_matched,ORB_FEATURE);
        //imshow("img_matched",img_matched);

        imgprocess.StereoConstruct(image_L,image_R,matches_L,world_coord);
        poseEst.PnPCheck(image_L,R,t);


        imgprocess.ImageInput(imgL_2,image_L2.img,imgR_2,image_R2.img);
        imgprocess.FeaturesMatching(image_L2,image_R2,img_matched,ORB_FEATURE);
        //imshow("img_matched",img_matched);

        imgprocess.StereoConstruct(image_L2,image_R2,matches_L2,world_coord_2);
        //poseEst.SolvePnP(matches_L2,world_coord_2,R2,t2);
        poseEst.PnPCheck(image_L2,R2,t2);

        ObjectTracker tk(image_L);
        vector<DMatch> a;
        tk.Track(image_L2,a);
        world_coord.clear();
        world_coord_2.clear();
        for(int i=0;i<a.size();i++)
        {
            world_coord.push_back(image_L.matched_3d[a[i].queryIdx]);
            world_coord_2.push_back(image_L2.matched_3d[a[i].trainIdx]);

        }

        tk.RansacMotion(world_coord,world_coord_2,R,t,500,10,0.6);


        auto end=std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end-start;
        cout<<"Pose estimation time:"<<elapsed_seconds.count()<<endl;


        /*
        vector<DMatch> matched;


        ObjectTracker tk(image_L);
        //cout<<"Debug info"<<endl;
        tk.Track(image_L2,matched);

        vector<Point3d> a;
        vector<Point3d> b;

        Mat rotation,translation;

        cout<<"debug info:matched_3d size is"<<image_L.matched_3d.size()<<endl;
        //for(auto i:image_L.matched_3d)
        //    cout<<"first matched 3d:"<<i<<endl;

        for(int i=0;i<3;i++)
        {
            a.push_back(image_L.matched_3d[matched[i].queryIdx]);
            b.push_back(image_L2.matched_3d[matched[i].trainIdx]);
        }
        for(auto i:a)
            cout<<"points from a:"<<i<<endl;
        for(auto j:b)
            cout<<"points from b:"<<j<<endl;

        tk.CalcMotions(a,b,rotation,translation);
        */


        /*
         * Debug info
         */

        fstream file;
        file.open("points.txt", std::fstream::in | std::fstream::out | std::fstream::app);
        file << "matched_points of the second left image:" << endl;
        for (auto i:matches_L2)
            file << i << " ";
        file << endl << "matched_points of the second right image:" << endl;
        for (auto j:matches_R2)
            file << j << " ";
        file << endl << "3D reconstruction:" << endl;
        for (auto k:world_coord_2)
            file << k << " ";
        file << endl;
        file.close();

    }
}

inline void ReadImage(const char *URL1,const char *URL2,Mat &img1,Mat &img2)
{
    img1=imread(URL1,CV_LOAD_IMAGE_COLOR);
    img2=imread(URL2,CV_LOAD_IMAGE_COLOR);

    //img1=imread(URL1,CV_LOAD_IMAGE_GRAYSCALE);
    //img2=imread(URL2,CV_LOAD_IMAGE_GRAYSCALE);

    //imgprocess.ImageInput(imgL,imgL,imgR,imgR);
    //imgprocess.PrintCorners();

    //imshow("imgL",imgL);
    //imshow("imgR",imgR);



}

void Test()
{
    ObjectTracker a;
    Mat r,t;
    vector<Point3d> ref;
    ref.push_back(Point3d(12.,13.,0));
    ref.push_back(Point3d(7.,6.,0));
    ref.push_back(Point3d(3.,2.2,0));

    vector<Point3d> tgt;
    tgt.push_back(Point3d(0,13.,12.));
    tgt.push_back(Point3d(0,6.,7.));
    tgt.push_back(Point3d(0,2.2,3.));
    a.CalcMotions(tgt,ref,r,t);
    /*
    Mat camera_matrix=(Mat_<float>(3,2)<< 537.6, 0., 400,
            0., 537.6,300);
    cout<<camera_matrix.size()<<endl;
     */


}
int main( int argc, char** argv)
{




    if( argc < 2)
    {
        Test();
        cout <<" Usage: poest ImageToLoadL ImageToLoadR" << endl;
        return -1;
    }
    ReadImage(argv[1],argv[2],imgL,imgR);
    ReadImage(argv[3],argv[4],imgL_2,imgR_2);

    image = imread(argv[1], CV_LOAD_IMAGE_COLOR);   // Read the file


    cout<<"image size:"<<image.rows<<"*"<<image.cols<<endl;
    namedWindow( "PnP", WINDOW_AUTOSIZE );// Create a window for display.
    //set the callback function for any mouse event
    setMouseCallback("PnP", CallBackFunc, NULL);
    while(waitKey(50)<0)
    {
        imshow("PnP", image);                   // Show our image inside it.

    }



    return 0;
}
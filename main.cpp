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
vector<Point3f> world_coord;
//vector<Point2f> image_coord;
vector<Point3f> world_coord_2;
Mat R,t;
Mat R2,t2;


Mat imgL,imgR;
Mat imgL_2,imgR_2;
vector<Point2f> matches_L,matches_R;
vector<Point2f> matches_L2,matches_R2;
vector<Point3f> matches_P,matches_N;
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
            //Timer starts.
            auto start=std::chrono::system_clock::now();
            imgprocess.ImageInput(imgL,imgL,imgR,imgR);
            //imgprocess.PrintCorners();

            //imgprocess.FeaturesMatching(imgL, imgR, 5, matches_L, matches_R);

            first=imgprocess.Matching(imgL,imgR,5,matches_L,matches_R);
            imgprocess.StereoConstruct(matches_L, matches_R, world_coord, 120.0, 2.5);
            poseEst.SolvePnP(matches_L, world_coord, R, t);
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
            file << endl;
            file.close();
        }
        //For debug
        else
        {
            /*
            poseEst.MarkPtOnImg(image,Point2f(x,y));
            Point3f A=poseEst.CalcWldCoord(R,t,Point2f(x,y));
            cout<<"Let's see Point 3d:"<<A<<endl;
            */
            cout<<"can't run?"<<endl;
        }


    }
    if  ( event == EVENT_RBUTTONDOWN && R.rows!=0)
    {
        cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;

        StereoImageProcess imgprocess;
        /**For debugging.Input all data into a file**/
        imgprocess.ImageInput(imgL_2,imgL_2,imgR_2,imgR_2);
        //imgprocess.PrintCorners();

        //imgprocess.FeaturesMatching(imgL, imgR, 5, matches_L, matches_R);

        auto second=imgprocess.Matching(imgL_2,imgR_2,5,matches_L2,matches_R2);
        imgprocess.StereoConstruct(matches_L2, matches_R2, world_coord_2, 120.0, 2.5);
        //poseEst.SolvePnP(matches_L2, world_coord_2, R, t);

        ObjectTracker tk(first);
        tk.Track(second);

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
    vector<Point3f> ref;
    ref.push_back(Point3f(12.,13.,0));
    ref.push_back(Point3f(7.,6.,0));
    ref.push_back(Point3f(3.,2.2,0));

    vector<Point3f> tgt;
    tgt.push_back(Point3f(0,13.,12.));
    tgt.push_back(Point3f(0,6.,7.));
    tgt.push_back(Point3f(0,2.2,3.));
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
    //ReadImage(argv[3],argv[4],imgL_2,imgR_2);

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
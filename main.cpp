#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>
#include <fstream>

#include "poest.h"
//#include "pointcloud.h"
using namespace cv;
using namespace std;

/*  Pose estimation
 *  s*m'=K*[R|t]*M
 *  Since K can be acquired by camera calibration with a chessboard
 *  A set of ways to estimate [R|t] are implemented.
 */
Mat R;
Mat t;
Mat image;
vector<Point3f> world_coord;
vector<Point2f> image_coord;
vector<double> disto;



Mat imgL,imgR;
vector<Point2f> matches_L,matches_R;
Pose_est poseEst;
StereoImageProcess imgprocess;

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
    if  ( event == EVENT_LBUTTONDOWN )
    {
        cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
        //poseEst.PnPmethod(x,y);
        //poseEst.Featuremethod();
        //poseEst.stereo_test(imgL,imgR);
        imgprocess.ORB_Matching(imgL,imgR,10,matches_L,matches_R);
        poseEst.stereo_construct(matches_L,matches_R,world_coord,120.0,2.5);
        poseEst.SolvePnP(matches_R,world_coord,disto,R,t);

        /**For debugging.Input all data into a file**/
        fstream file;
        file.open("points.txt",std::fstream::in | std::fstream::out | std::fstream::trunc);
        file<<"matched_points of the left image:"<<endl;
        for(auto i:matches_L)
            file<<i<<" ";
        file<<endl<<"matched_points of the right image:"<<endl;
        for(auto j:matches_R)
            file<<j<<" ";
        file<<endl<<"3D reconstruction:"<<endl;
        for(auto k:world_coord)
            file<<k<<" ";
        file<<endl;
        file.close();

    }
}

inline void ReadImage(const char *URL1,const char *URL2)
{
    imgL=imread(URL1,CV_LOAD_IMAGE_COLOR);
    imgR=imread(URL2,CV_LOAD_IMAGE_COLOR);

    //imgL=imread(URL1,CV_LOAD_IMAGE_GRAYSCALE);
    //imgR=imread(URL2,CV_LOAD_IMAGE_GRAYSCALE);

    imgprocess.ImageInput(imgL,imgL,imgR,imgR);
    //imgprocess.PrintCorners();

    //imshow("imgL",imgL);
    //imshow("imgR",imgR);



}

int main( int argc, char** argv)
{


    if( argc < 2)
    {
        cout <<" Usage: poest ImageToLoadL ImageToLoadR" << endl;
        return -1;
    }


    ReadImage(argv[1],argv[2]);

    image = imread(argv[1], CV_LOAD_IMAGE_COLOR);   // Read the file
    namedWindow( "PnP", WINDOW_AUTOSIZE );// Create a window for display.
    //set the callback function for any mouse event
    setMouseCallback("PnP", CallBackFunc, NULL);
    while(waitKey(50)<0)
    {
        imshow("PnP", image);                   // Show our image inside it.

    }



    return 0;
}
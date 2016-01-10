#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>

#include "pnp.h"
using namespace cv;
using namespace std;
/*
void drawPoints(cv::Mat image, std::vector<cv::Point2f> &list_points_2d, std::vector<cv::Point3f> &list_points_3d, cv::Scalar color)
{
  for (unsigned int i = 0; i < list_points_2d.size(); ++i)
  {
    cv::Point2f point_2d = list_points_2d[i];
    cv::Point3f point_3d = list_points_3d[i];

    // Draw Selected points
    cv::circle(image, point_2d, radius, color, -1, lineType );

    std::string idx = IntToString(i+1);
    std::string x = IntToString((int)point_3d.x);
    std::string y = IntToString((int)point_3d.y);
    std::string z = IntToString((int)point_3d.z);
    std::string text = "P" + idx + " (" + x + "," + y + "," + z +")";

    point_2d.x = point_2d.x + 10;
    point_2d.y = point_2d.y - 10;
    cv::putText(image, text, point_2d, fontFace, fontScale*0.5, color, thickness_font, 8);
  }
}*/
Mat image;
vector<Point3f> world_coord;
vector<Point2f> image_coord;
Mat camera_matrix=(Mat_<double>(3,3)<< 489.0, 0, 319.5, 0, 489.0, 239.5, 0, 0, 1);
Mat disto;
Mat R;
Mat t;


void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
    if  ( event == EVENT_LBUTTONDOWN )
    {
        cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;

        circle(image,Point(x,y),5,Scalar(0,0,255),-1);
        if(image_coord.size()<4)
        {
            image_coord.push_back(Point2f(x, y));

        }
        else
        {

           solvePnPRansac(world_coord, image_coord, camera_matrix,disto,R,t);
            cout<<"R matrix:"<<R<<endl;
            cout<<"t matrix:"<<t<<endl;

        }

    }
}

int main( int argc, char** argv)
{


    if( argc != 2)
    {
        cout <<" Usage: display_image ImageToLoadAndDisplay" << endl;
        return -1;
    }
    world_coord.push_back(Point3f(0,0,0));
    world_coord.push_back(Point3f(0,0,4.6));
    world_coord.push_back(Point3f(10.8,0,0));
    world_coord.push_back(Point3f(10.8,29.8,4.6));


    image = imread(argv[1], CV_LOAD_IMAGE_COLOR);   // Read the file


    namedWindow( "PnP", WINDOW_AUTOSIZE );// Create a window for display.
    //set the callback function for any mouse event
    setMouseCallback("PnP", CallBackFunc, NULL);
    while(waitKey(20)<0)
    {
        imshow("PnP", image);                   // Show our image inside it.

    }
    /*
    image_coord.push_back(Point2f(0,3));
    image_coord.push_back(Point2f(432,234));
    for(auto item:image_coord)
        cout<<item<<endl;

    return 0;*/
}
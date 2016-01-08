#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
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

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
    if  ( event == EVENT_LBUTTONDOWN )
    {
        cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;

        circle(image,Point(x,y),5,Scalar(0,0,255),-1);
    }
}

int main( int argc, char** argv)
{
    if( argc != 2)
    {
        cout <<" Usage: display_image ImageToLoadAndDisplay" << endl;
        return -1;
    }


    image = imread(argv[1], CV_LOAD_IMAGE_COLOR);   // Read the file


    namedWindow( "PnP", WINDOW_AUTOSIZE );// Create a window for display.
    //set the callback function for any mouse event
    setMouseCallback("PnP", CallBackFunc, NULL);
    while(waitKey(20)<0)
    {
        imshow("PnP", image);                   // Show our image inside it.

    }

    return 0;
}
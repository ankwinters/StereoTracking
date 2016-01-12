//
// Created by lab on 1/8/16.
//
#include "poest.h"
#include <iostream>
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

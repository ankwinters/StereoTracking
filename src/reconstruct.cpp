//
// Created by lab on 16-12-27.
//

#include "reconstruct.h"

namespace tracker{

    void Triangulate::Reconstruct3d(vector<Point2f> &matched_L, vector<Point2f> &matched_R, vector<Point3f> &coord_3d) {
        size_t num_points=matched_L.size();
        //double pixel_size=4.65e-3;
        //Z=b*f/d
        float d,Z,Y,X;
        //cout<<"debug info.."<<endl;

        for(size_t i=0;i<num_points;i++)
        {
            //* Transform them into the same world coordinate.
            d=matched_L[i].x-matched_R[i].x;
            Z=baseline*f/(d*pixel_size);
            Y=pixel_size*Z*(matched_L[i].y-this->camera_matrix.at<float>(1,2))/f;
            X=pixel_size*Z*(matched_L[i].x-this->camera_matrix.at<float>(0,2))/f;
            //cout<<"X["<<i<<"]:"<<X<<endl;
            //* Cartesian coordinate system.(0,0,0) is right at perspective point
            coord_3d.push_back(Point3d(X,-Y,-Z));
        }

    }
}
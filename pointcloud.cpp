//
// Created by lab on 1/14/16.
//
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/common/projection_matrix.h>

/*
int main (int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if(argc!=2)
    {
        std::cout<<"usage:poest <path to pcd file>"<<std::endl;
        return -1;
    }

    if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read pcd file \n");
        return (-1);
    }
    std::cout << "Loaded "
    << cloud->width * cloud->height
    << " data points from test_pcd.pcd with the following fields: "
    << std::endl;
    for (size_t i = 0; i < cloud->points.size() ; ++i)
        std::cout << "    " << cloud->points[i].x
        << " "    << cloud->points[i].y
        << " "    << cloud->points[i].z << std::endl;

    return (0);
}*/

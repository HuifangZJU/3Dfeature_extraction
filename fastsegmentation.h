#ifndef FASTSEGMENTATION_H
#define FASTSEGMENTATION_H

#pragma once
#include <iostream>
#include <vector>
#include <math.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/Dense>
#include "../point_types_hf.h"

class FastSegmentation
{
    public:
        FastSegmentation(void);
        ~FastSegmentation(void);

        std::vector<pcl::PointIndices> getclusters(void);

        void setInput(pcl::PointCloud<PointT>::Ptr input_cloud,pcl::PointCloud<pcl::Normal>::Ptr input_normals,pcl::PointIndices p);
        void initialize(void);
        void regionGrowing(void);
        std::vector<int> getNeighbors(int pointIndex);
        void knnclustering(std::vector<pcl::PointIndices> & clusters, pcl::PointIndices & rgbp);
        void mindisclustering(void);

private:
    pcl::PointCloud<PointT>::Ptr cloud;
    pcl::PointCloud<pcl::Normal>::Ptr normals;
    pcl::PointIndices points,xyzp,rgbp;
    int WIDTH_,HEIGHT_;
    std::vector<int> inputflag_;
    std::vector<int> segFlag_;
    std::vector<pcl::PointIndices> clusters_;
    std::vector<Eigen::Vector3d> centers_;
    double thNormalHigh,thNormal,thDOF;
};


#endif // FASTSEGMENTATION_H

#define PCL_NO_PRECOMPILE
#include "segmentercy.h"
#include "seg/objPartition.h"
#include "seg/surfaceSegment.h"
#include "seg/multiFrameMerge.h"
#include <pcl/console/parse.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>
#include <fstream>

using namespace pcl;
segmenterCY::segmenterCY()
{
}
segmenterCY::~segmenterCY()
{
}
void segmenterCY::setInputCloud(PointCloud<PointT>::Ptr cloud_in)
{
    cloud=cloud_in;
}
void segmenterCY::process()
{
    //filtering
    StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(0.5);
    sor.filter(*cloud);

    PassThrough<PointT> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-2.0, 4.0);
    pass.filter(*cloud);

    search::Search<PointT>::Ptr tree = boost::shared_ptr<search::Search<PointT> >(new search::KdTree<PointT>);
    PointCloud <Normal>::Ptr normals(new PointCloud <Normal>);
    NormalEstimation<PointT, Normal> normal_estimator;
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(cloud);
    normal_estimator.setRadiusSearch(0.1);
//    normal_estimator.setKSearch(100);
    normal_estimator.compute(*normals);

        surfaceSegment<PointT, Normal> reg;
        reg.setMinClusterSize(150);
        reg.setMaxClusterSize(1000000);
        reg.setSearchMethod(tree);
        reg.setNumberOfNeighbours(40);
        reg.setInputCloud(cloud);
        reg.setInputNormals(normals);
        reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
        reg.setCurvatureThreshold(1.0);
        std::vector<PointCloud<PointT>::Ptr> seg_cloud;
        std::vector <PointIndices> clusters;
        reg.extract(seg_cloud, clusters);
     cout<<"segments number: "<<clusters.size()<<endl;

    //write to cloud
    for(int i=0;i<cloud->points.size();i++)
    {
       cloud->points[i].segment = 0;
    }

    std::vector< PointIndices >::iterator i_segment;
    int nextid=1;
    for (i_segment = clusters.begin(); i_segment != clusters.end(); i_segment++)
    {
        std::vector<int>::iterator i_point;
        for (i_point = i_segment->indices.begin(); i_point != i_segment->indices.end(); i_point++)
        {
            int index;
            index = *i_point;
            cloud->points[index].segment = nextid;
        }
        nextid++;
    }



}

#ifndef EASYVIEWER_H
#define EASYVIEWER_H
//#define PCL_NO_PRECOMPILE
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "featureextractor.h"
#include "easyViewer.h"
#include "point_types_hf.h"

class EasyViewer
{
public:
    EasyViewer();
    ~EasyViewer();

    void setInput(pcl::PointCloud<PointT>::Ptr cloud,std::vector<FeatureExtractor::nodePatch> Nodes,std::vector<FeatureExtractor::edgeLine> Edges);
    void setViewer(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer,bool keyboard=false,std::string kind="node");
    boost::shared_ptr<pcl::visualization::PCLVisualizer> getViewer(void);
    void drawCloud(int kind=1);


    void drawNodes(int color=1,std::string filepath="",int fileid=1);
    void drawEdges(void);
    void drawNodeCenters(void);
    void drawNodeConvexHull(void);
    void drawNodeNormals(void);
    void drawNodeBDbox(int kind=1);
    void drawCloudBDbox(double x_max,double x_min,double y_max,double y_min,double z_max,double z_min);
    void drawBDbox(double x_max,double x_min,double y_max,double y_min,double z_max,double z_min,int index);

    void setLabelColors(int* labels,int labelnumber);
//    void nodekeyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer_void);
//    void edgekeyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer_void);
    void showcloud(void);
    void clearall(void);

private:
    pcl::PointCloud<PointT>::Ptr cloud ;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    std::vector<FeatureExtractor::nodePatch> Nodes;
    std::vector<FeatureExtractor::edgeLine> Edges;
    std::vector<Eigen::Vector3i> interested_colors;
    int* interested_labels;
    int viewer_label;

};





#endif // EASYVIEWER_H

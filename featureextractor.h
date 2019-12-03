#ifndef FEATURE_EXTRACTOR_H
#define FEATURE_EXTRACTOR_H
#define PCL_NO_PRECOMPILE
#include "../point_types_hf.h"
#include <pcl/PCLPointField.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/bounding_box.h>
#include <CGAL/wlop_simplify_and_regularize_point_set.h>
#include <CGAL/property_map.h>
#include <CGAL/bilateral_smooth_point_set.h>
#include <CGAL/tags.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/pca_estimate_normals.h>
#include <CGAL/mst_orient_normals.h>


class FeatureExtractor{
public:
    //declaration
    struct nodePatch;
    struct edgeLine;
    struct limit;


    //definition
    struct limit
    {
        float x_max,x_min,y_max,y_min,z_max,z_min;
    };

    struct nodePatch
    {
        int node_id;
        std::vector<int> segment;
        int label;
        int r,g,b;
        Eigen::Vector3d hsv;
        std::vector<pcl::PointXYZ> boundingbox;
        float area,height;
        float dis2bd;
        float eigen0,eigen1,eigen2;
        Eigen::Matrix3f eigenvector;
        float x_max,x_min,y_max,y_min,z_max,z_min;
        FeatureExtractor::limit local_limit;
        float dis2xmin,dis2xmax,dis2ymin,dis2ymax;
        float linearness,planarness,normal_z,viewdirection;
        Eigen::Vector3f center;
        Eigen::Vector3f normal;
        pcl::PointIndices contour;
        pcl::PointIndices points;
        pcl::PointIndices rgbpoints;
        pcl::PointCloud<pcl::PointXYZ> convex_hull;
        std::vector<float> base_colors;
        std::vector<int> neighbors;   //neighbor nodes that have a edge connection
        std::vector<int> neighbors2order;
        std::vector<int> adjacency;   //geometry adjacent patches
        std::vector<PointVectorPair> simplified_points;
        std::vector<float> features;

    };
    struct edgeLine
    {
        int edge_id;
        int start;
        int end;
        float mindis,hor_dis,ver_dis;
        float angle_bt_nm,angle_bt_ver,coplanarity,parallel,perpendicularity;
        int adjacency;
        std::vector<float> features;
        Eigen::Vector3d dis_hsv;
    };

public:
    FeatureExtractor();
    ~FeatureExtractor();

    void setInputCloud(pcl::PointCloud<PointT>::Ptr cloud,bool seg_label_agree=true,bool remove_unknown=false);
    pcl::PointCloud<PointT>::Ptr getInputCloud(void);
    std::vector<nodePatch> getNodes(void);
    std::vector<edgeLine> getEdges(void);
    std::vector<float> getLimit(void);
    void initialize(void);
    void buildNodes(void);
    void buildNodes(std::vector<pcl::PointIndices> clusters);
    void countsegments(void);

private:
    bool buildnode(nodePatch & np, pcl::PointIndices &p);
    void setNodeFlag_(void);
    void refineFeatures(void);
    void splitsegments(void);
    void splitsegment(int nodeid);
    void getSimplifiedPoints(nodePatch &np);
    std::vector<PointVectorPair> simplifyPoints(pcl::PointCloud<PointT>::Ptr cloud,pcl::PointIndices p);
    void getPCAfigures(nodePatch &np);
    void extractNodecenter(nodePatch &np);
    void extractNodeheight(nodePatch &np);
    void extractNodeposition(nodePatch &np);
    void extractNodelimits(nodePatch &np);
    void extractNodeNormal(nodePatch &np);
    void extractNodeShapeIndicator(nodePatch &np);
    void extractNodeHSV(nodePatch &np);
    void extractNodebasecolors(nodePatch &np);
    void extractNodearea(nodePatch &np);

public:
    void buildEdges(void);
    void extractEdgefeatures(void);
private:

    void findNeighbors(void);
    void constructGraph(void);
    void checkNeighbors(void);
    bool mergeNeighboringNodes(void);
    void mergeNodes(int i,int j);
    void extractMergeFetures(nodePatch &np);
    void deleteInvalidNodes(void);
    void extractFirstOrderNeighbors(int i);
    void extractSecondOrderNeighbors(int i);
    void extractEdgemindis(int i);
    double computeMindisof2patch(int i,int j);
    void extractEdgevisualfeatures(int edgeid);
    void extractEdgeadjacency(int nodeid);
    void extractEdgegeometryfeatures(int edgeid);
    void extractEdgepositionfeatures(int edgeid);
    void extractEdgecoplanarity(int edgeid);
    Eigen::Vector3f getMergednormal(nodePatch np1,nodePatch np2);

    double pointdis(int p1,int p2);
    double pointdis(double x1,double y1,double z1,double x2,double y2,double z2);
    int getMainlabel(nodePatch np);
    std::vector<int> getNeighbors(int point_index);


private:
    pcl::PointCloud<PointT>::Ptr cloud ;
    pcl::PointCloud<pcl::Normal>::Ptr normals;
    int cloud_WIDTH,cloud_HEIGHT;

    //node flag for each point
    std::vector<int> _flag;
    double x_min,x_max,y_min,y_max,z_min,z_max,max_dis;
    Eigen::Vector3f cloud_center;
    std::vector<float> limits;
    std::vector<int> segment;
    std::vector<int> seg_count;
    std::vector<pcl::PointIndices> seg_indices;
    std::vector<nodePatch> Nodes,nodes_;
    std::vector<edgeLine> Edges,edges_;
    bool SEG_LABEL_CONSISTENCY,REMOVE_UNKNOWN;

    //total number of simplified points
    int simplifiedpoints;


};

#endif // FEATURE_EXTRACTOR_H

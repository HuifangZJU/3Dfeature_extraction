#ifndef SUPERFEATURE_H
#define SUPERFEATURE_H
#define PCL_NO_PRECOMPILE

#include <pcl/PCLPointField.h>
#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>

#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/segmentation/supervoxel_clustering.h>
#include <vtkPolyLine.h>
#include <pcl/conversions.h>
#include <ctime>

struct SemanticPoint
{
    PCL_ADD_POINT4D
    PCL_ADD_RGB
    unsigned int segment;
    unsigned int label;
    unsigned int cameraIndex;
    float distance;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (SemanticPoint,
                                  (float,x,x)
                                  (float,y,y)
                                  (float,z,z)
                                  (unsigned int,rgba,rgba)
                                  (unsigned int,segment,segment)
                                  (unsigned int,label,label)
)
typedef SemanticPoint PointT;

class superfeature
{
public:
    struct nodePatch
    {
        int segment;
        int node_id;
        int label;
        pcl::PointIndices points;
        int r,g,b;
        Eigen::Vector3d hsv;
        float height,dis2bd,dis2xmin,dis2xmax,dis2ymin,dis2ymax;
        Eigen::Vector3f center;
        Eigen::Vector3f normal;
        std::vector<int> neighbors;
    };
    struct edgeLine
    {
        int edge_id;
        int start;
        int end;
        float angle_bt_nm;
        Eigen::Vector3d dis_hsv;
    };

public:
    superfeature();
    ~superfeature();

    void setInputCloud(pcl::PointCloud<PointT>::Ptr cloud,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_rgba);
    pcl::PointCloud<PointT>::Ptr getInputCloud(void);
    std::vector<nodePatch> getNodes(void);
    std::vector<edgeLine> getEdges(void);
    std::vector<float> getLimit(void);
    void initialize(void);
    void constructGraph(void);
    void resetCloudLabels(pcl::PointCloud<pcl::PointXYZL>::Ptr supercloud, std::map <int,int> seg2node,
                          std::map <int,int> isnode);
    void buildEdges(void);

    void extractNodecolor(int nodeid);
    void extractNodefeatures(void);

    void extractEdgefeatures(void);
    void extractEdgevisualfeatures(int edgeid);
    void extractEdgegeometryfeatures(int edgeid);

    double pointdis(int p1,int p2);
    double pointdis(double x1,double y1,double z1,double x2,double y2,double z2);
    std::vector<int> getNeighbors(int point_index);
//    static void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer_void);
    pcl::PointCloud<PointT>::Ptr cloud ;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_rgba;
    pcl::PointCloud<pcl::Normal>::Ptr normals;
    int WIDTH,HEIGHT;
    double x_min,x_max,y_min,y_max,z_min,z_max,max_dis;
    Eigen::Vector3f cloud_center;
    std::vector<float> limits;
    std::vector<int> segments;
    std::vector<int> labels;
    std::vector<pcl::PointIndices> seg_indices;

    std::vector<nodePatch> Nodes;
    std::vector<edgeLine> Edges;

};

#endif // SUPERFEATURE_H



























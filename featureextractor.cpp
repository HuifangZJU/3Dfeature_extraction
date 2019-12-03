#include "featureextractor.h"
#include <iostream>
#include <math.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/pca.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_types_conversion.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/segmentation/region_growing.h>
#include "../point_types_hf.h"
#include "fastsegmentation.h"

#include <CGAL/Simple_cartesian.h>
#include <CGAL/bounding_box.h>
#include <CGAL/wlop_simplify_and_regularize_point_set.h>
#include <CGAL/property_map.h>
#include <CGAL/tags.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/pca_estimate_normals.h>
#include <CGAL/convex_hull_2.h>

//#include <omp.h>


#include <CGAL/mst_orient_normals.h>
using namespace pcl;
using namespace std;
#define PI 3.1415926
#define FLOOR 2


FeatureExtractor::FeatureExtractor()
{
}

FeatureExtractor::~FeatureExtractor()
{
}

void FeatureExtractor::setInputCloud(PointCloud<PointT>::Ptr cloud_in,bool seg_label_agree,bool remove_unknown)
{
    //input cloud should be already segmented
    cloud=cloud_in;
    SEG_LABEL_CONSISTENCY=seg_label_agree;
    REMOVE_UNKNOWN = remove_unknown;
    if(SEG_LABEL_CONSISTENCY)
    {
        for(int i=0;i<cloud->points.size();i++)
        {
            int temp=cloud->points[i].label;
            cloud->points[i].segment=temp;
        }
    }
}
PointCloud<PointT>::Ptr FeatureExtractor::getInputCloud()
{
    return cloud;
}

vector<FeatureExtractor::nodePatch> FeatureExtractor::getNodes()
{
    return Nodes;
}
vector<FeatureExtractor::edgeLine> FeatureExtractor::getEdges()
{
    return Edges;
}
vector<float> FeatureExtractor::getLimit()
{

   return limits;
}

void FeatureExtractor::initialize()
{
    //initialize global variables
    simplifiedpoints=0;
    segment.clear();seg_count.clear();seg_indices.clear();
    Nodes.clear(); Edges.clear();
    cloud_WIDTH = cloud->width;
    cloud_HEIGHT = cloud->height;
    _flag.resize(cloud_WIDTH*cloud_HEIGHT,0);

    //get cloud limits
    int ite=0;
    while(!__finite(cloud->points[ite].x))
    { ite++;}
    x_min=cloud->points[ite].x;
    x_max=cloud->points[ite].x;
    y_min=cloud->points[ite].y;
    y_max=cloud->points[ite].y;
    z_min=cloud->points[ite].z;
    z_max=cloud->points[ite].z;
    for (int i=0;i<cloud->points.size();i++)
    {
        if(__finite(cloud->points[i].x))
        {
            if(cloud->points[i].x>x_max) x_max=cloud->points[i].x;
            if(cloud->points[i].x<x_min) x_min=cloud->points[i].x;
            if(cloud->points[i].y>y_max) y_max=cloud->points[i].y;
            if(cloud->points[i].y<y_min) y_min=cloud->points[i].y;
            if(cloud->points[i].z>z_max) z_max=cloud->points[i].z;
            if(cloud->points[i].z<z_min) z_min=cloud->points[i].z;
        }
    }
    limits.push_back(x_max);
    limits.push_back(x_min);
    limits.push_back(y_max);
    limits.push_back(y_min);
    limits.push_back(z_max);
    limits.push_back(z_min);

    //comput cloud max_distance
    max_dis=pointdis(x_max,y_max,z_max,x_min,y_min,z_min);

    //estimate cloud center
    cloud_center(0)=(x_max-x_min)/2;
    cloud_center(1)=(y_max-y_min)/2;
    cloud_center(2)=(z_max-z_min)/2;
    //estimate cloud normals
//    int times;
//    int starts = clock();
//    NormalEstimation<PointT,Normal> ne;
//    PointCloud<Normal>::Ptr normals_ (new PointCloud<Normal>);
//    ne.setInputCloud(cloud);
//    search::KdTree<PointT>::Ptr tree (new search::KdTree<PointT> ());
//    ne.setSearchMethod (tree);
//    ne.setRadiusSearch(0.05);
//    ne.compute (*normals_);
//    normals=normals_;
//    int finishs = clock();
//    times = (double)(finishs - starts) / CLOCKS_PER_SEC;
//    cout<<"computing normal cost "<<times<<" s"<<endl;
}

void FeatureExtractor::countsegments()
{
    //get indices for each segment
    for (int i=0;i<cloud->points.size();i++)
    {
        if(cloud->points[i].segment==0){continue;}
        if(REMOVE_UNKNOWN)
        {
           if(cloud->points[i].label==0){continue;}
        }
        int flag=0;
        for(int j=0;j<segment.size();j++)
        {
            if(cloud->points[i].segment == segment[j])
            {
                seg_count[j]++;
                PointIndices& ptemp=seg_indices[j];
                ptemp.indices.push_back(i);
                flag=1;
                break;
            }
        }
        if(flag==0)
        {
            PointIndices pIndx;
            pIndx.indices.push_back(i);
            segment.push_back(cloud->points[i].segment);
            seg_count.push_back(1);
            seg_indices.push_back(pIndx);
        }

    }
//     splitsegments();
}
void FeatureExtractor::splitsegments()
{
    //split each segment using a self-defined region-growing
    vector<int> segment_;
    vector<int> seg_count_;
    vector<PointIndices> seg_indices_;
    int seg=0;
    for(int i=0;i<segment.size();i++)
    {
        PointIndices &p = seg_indices[i];
        vector <PointIndices> clusters;
        PointIndices xyzp,rgbp;
        for(int i=0;i<p.indices.size();i++)
        {
            int index = p.indices[i];
            if(__finite(cloud->points[index].x))
                xyzp.indices.push_back(index);
            else
                rgbp.indices.push_back(index);
        }
        if(xyzp.indices.size()<50)
             clusters.push_back(p);
        else
        {
            PointIndices::Ptr pi_t(new PointIndices);
            PointIndices &pi = *pi_t;
            pi = xyzp;
            PCA<PointT> pca;
            pca.setInputCloud(cloud);
            pca.setIndices(pi_t);
            Eigen::Vector3f eigenvalue=pca.getEigenValues();
//            cout<<eigenvalue(0)<<"  "<<eigenvalue(1)<<"  "<<eigenvalue(2)<<"  ";
//            cout<<eigenvalue(1)/eigenvalue(2)<<endl;
          if(eigenvalue(1)/eigenvalue(2)>50)
          {
               clusters.push_back(p);
//               cout<<" it is a plane !"<<endl;
          }

          else
          {
              FastSegmentation fs;
              fs.setInput(cloud,normals,p);
              fs.initialize();
              fs.regionGrowing();
              clusters=fs.getclusters();
          }

        }

        for(int j=0;j<clusters.size();j++)
        {
            seg++;
            segment_.push_back(seg);
            seg_indices_.push_back(clusters[j]);
            seg_count_.push_back(clusters[j].indices.size());
        }

    }
    segment=segment_;
    seg_count=seg_count_;
    seg_indices=seg_indices_;
}


void FeatureExtractor::buildNodes()
{
    countsegments();
    int id=1;
    for(int i=0;i<segment.size();i++)
    {
        if(seg_count[i]<50)
            continue;
        nodePatch np;
        PointIndices & p=seg_indices[i];
        if(!buildnode(np, p))
            continue;
        np.node_id=id;
        np.segment.push_back(segment[i]);
        if(SEG_LABEL_CONSISTENCY)
            np.label= cloud->points[p.indices[0]].label;
        else
        {
            int maxlabel=getMainlabel(np);
            np.label=maxlabel;
        }
        Nodes.push_back(np);
        id++;

    }
    cout<<"Nodes number "<<Nodes.size()<<endl;
//    while(mergeNeighboringNodes()){}
    setNodeFlag_();
    refineFeatures();

}


//build nodes with given segment indices
void FeatureExtractor::buildNodes(vector<PointIndices> clusters)
{
    int id=1;
    #pragma omp parallel for
    for(int i=0;i<clusters.size();i++)
    {
        PointIndices &p = clusters[i];
        if(p.indices.size()>50)
        {
            nodePatch np;
            if(!buildnode(np, p))
                continue;
            np.node_id=id;
            np.segment.push_back(segment[i]);
            if(SEG_LABEL_CONSISTENCY)
                np.label= cloud->points[p.indices[0]].label;
            else
            {
                int maxlabel=getMainlabel(np);
                np.label=maxlabel;
            }
            Nodes.push_back(np);
            id++;
        }

    }
    cout<<"Nodes number "<<Nodes.size()<<endl;
//    while(mergeNeighboringNodes()){}
    setNodeFlag_();
    refineFeatures();
}

bool FeatureExtractor::buildnode(nodePatch & np, PointIndices &p)
{
    for(int j=0;j<p.indices.size();j++)
    {
        int index=p.indices[j];
        if(__finite(cloud->points[index].x))
            np.points.indices.push_back(index);
        else
            np.rgbpoints.indices.push_back(index);
    }
    if(np.points.indices.size()<50)
        return false;
    getSimplifiedPoints(np);
    getPCAfigures(np);
    extractNodecenter(np);
    extractNodearea(np);
    if(np.area<0.1)
        return false;
    extractNodeheight(np);
    extractNodeposition(np);
    extractNodelimits(np);
    extractNodeNormal(np);
    extractNodeShapeIndicator(np);
    extractNodeHSV(np);
    extractNodebasecolors(np);
    return true;
}



void FeatureExtractor::setNodeFlag_()
{
    #pragma omp parallel for
    for(int i=0;i<Nodes.size();i++)
    {
        nodePatch & np = Nodes[i];
        int nodeid=np.node_id;
        for(int i=0;i<np.points.indices.size();i++)
        {
            int index = np.points.indices[i];
            _flag[index]=nodeid;
        }
        for(int i=0;i<np.rgbpoints.indices.size();i++)
        {
            int index = np.rgbpoints.indices[i];
            _flag[index]=nodeid;
        }
    }

}

void FeatureExtractor::refineFeatures()
{
    //normalize the distance to boundary
    float dis2xmin_min=0,dis2xmin_max=max_dis;
    float dis2ymin_min=0,dis2ymin_max=max_dis;
    float dis2xmax_min=0,dis2xmax_max=max_dis;
    float dis2ymax_min=0,dis2ymax_max=max_dis;
    for(int i=0;i<Nodes.size();i++)
    {
        nodePatch &np=Nodes[i];
        if(np.dis2xmin>dis2xmin_max)  dis2xmin_max=np.dis2xmin;
        if(np.dis2xmin<dis2xmin_min)  dis2xmin_min=np.dis2xmin;

        if(np.dis2xmax>dis2xmax_max)  dis2xmax_max=np.dis2xmax;
        if(np.dis2xmax<dis2xmax_min)  dis2xmax_min=np.dis2xmax;

        if(np.dis2ymin>dis2ymin_max)  dis2ymin_max=np.dis2ymin;
        if(np.dis2ymin<dis2ymin_min)  dis2ymin_min=np.dis2ymin;

        if(np.dis2ymax>dis2ymax_max)  dis2ymax_max=np.dis2ymax;
        if(np.dis2ymax<dis2ymax_min)  dis2ymax_min=np.dis2ymax;
    }
    if((dis2xmin_max-dis2xmin_min)*(dis2xmax_max-dis2xmax_min)*(dis2ymin_max-dis2ymin_min)*(dis2ymax_max-dis2ymax_min)!=0)
    {
         for(int i=0;i<Nodes.size();i++)
        {
            nodePatch &np=Nodes[i];
            np.dis2xmin=(np.dis2xmin-dis2xmin_min)/(dis2xmin_max-dis2xmin_min);
            np.dis2xmax=(np.dis2xmax-dis2xmax_min)/(dis2xmax_max-dis2xmax_min);

            np.dis2ymin=(np.dis2ymin-dis2ymin_min)/(dis2ymin_max-dis2ymin_min);
            np.dis2ymax=(np.dis2ymax-dis2ymax_min)/(dis2ymax_max-dis2ymax_min);

            float a=np.dis2xmin,b=np.dis2ymin;
            if(a>np.dis2xmax) a=np.dis2xmax;
            if(b>np.dis2ymax) b=np.dis2ymax;

            np.dis2bd=a;
            if(a>b) np.dis2bd=b;

        }
    }
    double horizon;
    int floorcnt=0;

    for(int i=0;i<Nodes.size();i++)
    {
        nodePatch &np=Nodes[i];
        if(np.label==FLOOR)
        {
            horizon=horizon+np.height;
            floorcnt++;
        }
    }
    if(floorcnt>0)
    {
        horizon=horizon/floorcnt;
        for(int i=0;i<Nodes.size();i++)
        {
            nodePatch &np=Nodes[i];
            np.height=np.height-horizon;
        }
    }

    for(int i=0;i<Nodes.size();i++)
    {
        nodePatch &np=Nodes[i];
        np.features.push_back(np.hsv(0));
        np.features.push_back(np.hsv(1));
        np.features.push_back(np.hsv(2));

        for(int j=0;j<np.base_colors.size();j++)
        {
            np.features.push_back(np.base_colors[j]);
        }

        np.features.push_back(np.area);
        np.features.push_back(np.height);
        np.features.push_back(np.normal_z);
        np.features.push_back(np.dis2bd);
        np.features.push_back(np.linearness);
        np.features.push_back(np.planarness);
    }
}


/******************************Extract node limits******************************/
void FeatureExtractor::extractNodelimits(nodePatch & np)
{
    float xmin,xmax,ymin,ymax,zmin,zmax;
    PointIndices & pi=np.points;
    xmin=cloud->points[pi.indices[0]].x;xmax=xmin;
    ymin=cloud->points[pi.indices[0]].y;ymax=ymin;
    zmin=cloud->points[pi.indices[0]].z;zmax=zmin;
    for(int j=0;j<pi.indices.size();j++)
    {

        if(cloud->points[pi.indices[j]].x>xmax) {xmax=cloud->points[pi.indices[j]].x;}
        if(cloud->points[pi.indices[j]].x<xmin) {xmin=cloud->points[pi.indices[j]].x;}
        if(cloud->points[pi.indices[j]].y>ymax) {ymax=cloud->points[pi.indices[j]].y;}
        if(cloud->points[pi.indices[j]].y<ymin) {ymin=cloud->points[pi.indices[j]].y;}
        if(cloud->points[pi.indices[j]].z>zmax) {zmax=cloud->points[pi.indices[j]].z;}
        if(cloud->points[pi.indices[j]].z<zmin) {zmin=cloud->points[pi.indices[j]].z;}
    }
    np.x_max=xmax; np.x_min=xmin;
    np.y_max=ymax; np.y_min=ymin;
    np.z_max=zmax; np.z_min=zmin;
    //cout<<x_max<<" "<<x_min<<" "<<y_max<<" "<<y_min<<" "<<z_max<<" "<<z_min<<endl;
}


/******************************Extract node center******************************/
void FeatureExtractor::extractNodecenter(nodePatch & np)
{
    PointIndices &p =np.points;
    Eigen::Vector3f point,center;
    center<<0,0,0;
    for(int j=0;j<p.indices.size();j++)
    {
      point<<cloud->points[p.indices[j]].x,cloud->points[p.indices[j]].y,cloud->points[p.indices[j]].z;
      center=center+point;
    }
    np.center=center/p.indices.size();
    //cout<<id<<" center : "<<np.center(0)<<" "<<np.center(1)<<" "<<np.center(2)<<endl;
}

/******************************Extract node height******************************/
void FeatureExtractor::extractNodeheight(nodePatch & np)
{
    np.height = np.center(2);
    //cout<<np.height<<endl;
}

/******************************Extract node position******************************/
void FeatureExtractor::extractNodeposition(nodePatch & np)
{
    np.dis2xmin=abs(np.center(0)-x_min);
    np.dis2xmax=abs(np.center(0)-x_max);
    np.dis2ymin=abs(np.center(1)-y_min);
    np.dis2ymax=abs(np.center(1)-y_max);
    //cout<<np.dis2xmin<<" "<<np.dis2xmax<<"  "<<np.dis2ymin<<"  "<<np.dis2ymax<<endl;
}

/******************************Extract node area and local coordinate bdbox******************************/
void FeatureExtractor::extractNodearea(nodePatch & np)
{
    Eigen::Matrix3f eigenvector = np.eigenvector;
    Eigen::Vector3f center = np.center;
    Eigen::Vector3f R,U,L;
    R=eigenvector.col(0);U=eigenvector.col(1);L=eigenvector.col(2);
    vector<PointVectorPair> & points_pair = np.simplified_points;
    vector<Eigen::Vector3f> localpoints;
    for(int j=0;j<points_pair.size();j++)
    {
      PointVectorPair onepair = points_pair[j];
      Eigen::Vector3f diff,cp;
      diff(0)=onepair.first.x()-center(0);
      diff(1)=onepair.first.y()-center(1);
      diff(2)=onepair.first.z()-center(2);

      cp(0) = diff(0)*R(0)+diff(1)*R(1)+diff(2)*R(2);
      cp(1) = diff(0)*U(0)+diff(1)*U(1)+diff(2)*U(2);
      cp(2) = diff(0)*L(0)+diff(1)*L(1)+diff(2)*L(2);
      localpoints.push_back(cp);
    }

    typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
    typedef K::Point_2 Point_2;
    typedef vector<Point_2> Points;

    vector<CGAL::Exact_predicates_inexact_constructions_kernel::Point_2> points_2d,convex_hull;
    for(int i=0;i<localpoints.size();i++)
    {
        points_2d.push_back(CGAL::Exact_predicates_inexact_constructions_kernel::Point_2
                            (localpoints[i](0),localpoints[i](1)));
    }
   //BUG: IF points_2d.size<4
    if(points_2d.size()>3)
    {
        CGAL::convex_hull_2( points_2d.begin(), points_2d.end(), back_inserter(convex_hull));
    }
    else
    {
        cout<<endl<<"BUG: point size smaller than 4 when extracting convex hull!   ";
        cout<<"simplified point size : "<<np.simplified_points.size()<<endl;
        return;
    }
    for(int i=0;i<convex_hull.size();i++)
    {
        CGAL::Exact_predicates_inexact_constructions_kernel::Point_2  temp = convex_hull[i];

        PointXYZ  bdpoint;
        bdpoint.x=R(0)*temp.x()+U(0)*temp.y()+center(0);
        bdpoint.y=R(1)*temp.x()+U(1)*temp.y()+center(1);
        bdpoint.z=R(2)*temp.x()+U(2)*temp.y()+center(2);
        np.convex_hull.points.push_back(bdpoint);
    }
    float area=0;
    for(int i=0;i<convex_hull.size();i++)
    {
        int j=(i+1)%convex_hull.size();
        CGAL::Exact_predicates_inexact_constructions_kernel::Point_2  temp1 = convex_hull[i],temp2=convex_hull[j];
        area=area+temp1.x()*temp2.y()-temp1.y()*temp2.x();
    }

    np.area=area;

    //local limit
    Eigen::Vector3f onepoint = localpoints[0];
    double x_min=onepoint(0),x_max=onepoint(0);
    double y_min=onepoint(1),y_max=onepoint(1);
    double z_min=onepoint(2),z_max=onepoint(2);

    for(int j=0;j<localpoints.size();j++)
    {
      onepoint = localpoints[j];
      if(onepoint(0)>x_max) {x_max=onepoint(0);}if(onepoint(0)<x_min) {x_min=onepoint(0);}
      if(onepoint(1)>y_max) {y_max=onepoint(1);}if(onepoint(1)<y_min) {y_min=onepoint(1);}
      if(onepoint(2)>z_max) {z_max=onepoint(2);}if(onepoint(2)<z_min) {z_min=onepoint(2);}
    }
    np.local_limit.x_max=x_max;
    np.local_limit.x_min=x_min;
    np.local_limit.y_max=y_max;
    np.local_limit.y_min=y_min;
    np.local_limit.z_max=z_max;
    np.local_limit.z_min=z_min;
    PointXYZ bdpoint;
    vector<PointXYZ> localbounding;
    bdpoint.x= x_min;bdpoint.y= y_max;bdpoint.z= z_max;
    localbounding.push_back(bdpoint);
    bdpoint.x= x_min;bdpoint.y= y_min;bdpoint.z= z_max;
    localbounding.push_back(bdpoint);
    bdpoint.x= x_max;bdpoint.y= y_min;bdpoint.z= z_max;
    localbounding.push_back(bdpoint);
    bdpoint.x= x_max;bdpoint.y= y_max;bdpoint.z= z_max;
    localbounding.push_back(bdpoint);
    bdpoint.x= x_min;bdpoint.y= y_max;bdpoint.z= z_min;
    localbounding.push_back(bdpoint);
    bdpoint.x= x_min;bdpoint.y= y_min;bdpoint.z= z_min;
    localbounding.push_back(bdpoint);
    bdpoint.x= x_max;bdpoint.y= y_min;bdpoint.z= z_min;
    localbounding.push_back(bdpoint);
    bdpoint.x= x_max;bdpoint.y= y_max;bdpoint.z= z_min;
    localbounding.push_back(bdpoint);


    for(int i=0;i<localbounding.size();i++)
    {
        PointXYZ  temp = localbounding[i];
        PointXYZ  bdpoint;
        bdpoint.x=R(0)*temp.x+U(0)*temp.y+L(0)*temp.z+center(0);
        bdpoint.y=R(1)*temp.x+U(1)*temp.y+L(1)*temp.z+center(1);
        bdpoint.z=R(2)*temp.x+U(2)*temp.y+L(2)*temp.z+center(2);
        np.boundingbox.push_back(bdpoint);
    }

//    for(int i=0;i<np.boundingbox.size();i++)
//    {
//        cout<<np.boundingbox[i]<<"   ";
//    }
//    cout<<endl;
}


/******************************Extract node mean HSV******************************/
void FeatureExtractor::extractNodeHSV(nodePatch & np)
{
    PointCloud<PointXYZRGB> cloud_node;
    PointCloud<PointXYZHSV> cloud_hsv;
    PointIndices p;
    for(int i=0;i<np.points.indices.size();i++)
    {
        int index=np.points.indices[i];
        p.indices.push_back(index);
    }
    for(int i=0;i<np.rgbpoints.indices.size();i++)
    {
        int index=np.rgbpoints.indices[i];
        p.indices.push_back(index);
    }
    for(int j=0;j<p.indices.size();j++)
    {
        PointXYZRGB point;
        point.r=cloud->points[p.indices[j]].r;
        point.g=cloud->points[p.indices[j]].g;
        point.b=cloud->points[p.indices[j]].b;
        cloud_node.push_back(point);
    }
    PointCloudXYZRGBtoXYZHSV(cloud_node,cloud_hsv);
    double h=0,s=0,v=0;
    int r=0,g=0,b=0;
    for(int j=0;j<cloud_hsv.points.size();j++)
    {
        h=h+cloud_hsv.points[j].h;
        s=s+cloud_hsv.points[j].s;
        v=v+cloud_hsv.points[j].v;
        r=r+cloud_node.points[j].r;
        g=g+cloud_node.points[j].g;
        b=b+cloud_node.points[j].b;
    }
    if(p.indices.size()>0)
    {
    np.hsv(0)=h/p.indices.size();
    np.hsv(1)=s/p.indices.size();
    np.hsv(2)=v/p.indices.size();
    np.r=r/p.indices.size();
    np.g=g/p.indices.size();
    np.b=b/p.indices.size();
    }
}

/******************************Extract node base color histogram******************************/
void FeatureExtractor::extractNodebasecolors(nodePatch & np)
{

    PointCloud<PointXYZRGB> cloud_node;
    PointCloud<PointXYZHSV> cloud_hsv;
    PointIndices &p = np.points;
    for(int j=0;j<p.indices.size();j++)
    {
        PointXYZRGB point;
        point.x=cloud->points[p.indices[j]].x;
        point.y=cloud->points[p.indices[j]].y;
        point.z=cloud->points[p.indices[j]].z;
        point.r=cloud->points[p.indices[j]].r;
        point.g=cloud->points[p.indices[j]].g;
        point.b=cloud->points[p.indices[j]].b;
        cloud_node.push_back(point);
    }
    PointCloudXYZRGBtoXYZHSV(cloud_node,cloud_hsv);
    vector<float> & bc = np.base_colors;
    bc.resize(10,0);
    int sum=0;
    float h,s,v;
    for(int j=0;j<cloud_hsv.points.size();j++)
    {
        h=cloud_hsv.points[j].h/2;
        s=cloud_hsv.points[j].s*255;
        v=cloud_hsv.points[j].v*255;
        if(v<46) {bc[0]++;sum++;}
        if(s<43 && v>46 && v<220) {bc[1]++;sum++;}
        if(s<30 && v>221 && v<255) {bc[2]++;sum++;}
        if(s>43 && v>46)
        {
            if(h>156 || h<10) {bc[3]++;sum++;}
            if(h>11  && h<25) {bc[4]++;sum++;}
            if(h>26  && h<34) {bc[5]++;sum++;}
            if(h>35  && h<77) {bc[6]++;sum++;}
            if(h>78  && h<99) {bc[7]++;sum++;}
            if(h>100 && h<124) {bc[8]++;sum++;}
            if(h>125 && h<155) {bc[9]++;sum++;}
        }
    }
    for(int i=0;i<bc.size();i++)
    {
        bc[i]=bc[i]/double(sum);
    }

}

/******************************Extract edge adjacency******************************/
void FeatureExtractor::extractEdgeadjacency(int id)
{
    edgeLine & el = Edges[id];
    edgeLine & el2 =Edges[id+1];
    int startnodeid = el.start;
    int endnodeid = el.end;
    nodePatch &np = Nodes[startnodeid-1];
    nodePatch &np2 = Nodes[endnodeid-1];

    //two nodes are adjacent if they are intersect in the 3D space
    if(np.x_max+0.01<np2.x_min || np.x_min>np2.x_max+0.01
            || np.y_max+0.01<np2.y_min || np.y_min>np2.y_max+0.01
            || np.z_max+0.01<np2.z_min || np.z_min>np2.z_max+0.01)
        return;
    np.adjacency.push_back(np2.node_id);
    el.adjacency=1;
    np2.adjacency.push_back(np.node_id);
    el2.adjacency=1;


}

/******************************Extract PCA figures******************************/
void FeatureExtractor::getPCAfigures(nodePatch & np)
{
    PointIndices::Ptr pi_t(new PointIndices);
    PointIndices &pi = *pi_t;
    pi = np.points;
    PCA<PointT> pca;
    pca.setInputCloud(cloud);
    pca.setIndices(pi_t);
    Eigen::Vector3f eigenvalue=pca.getEigenValues();
    Eigen::Matrix3f eigenvector=pca.getEigenVectors();
    np.eigen0=eigenvalue(0);
    np.eigen1=eigenvalue(1);
    np.eigen2=eigenvalue(2);
    np.eigenvector=eigenvector;
}

/******************************Extract node normal******************************/
void FeatureExtractor::extractNodeNormal(nodePatch & np)
{
     np.normal=np.eigenvector.col(2);
     Eigen::Vector3f & normal=np.normal;
     Eigen::Vector3f relativecenter=cloud_center-np.center;
     //flip normals to the center
     if(normal(0)*relativecenter(0)+normal(1)*relativecenter(1)+normal(2)*relativecenter(2)<0)
         normal=normal*(-1);
     np.normal_z=abs(normal(2));
     if(np.normal_z>0.94){np.normal_z=1;}
     else if(np.normal_z<0.34){np.normal_z=0;}
}

/******************************Extract node shape******************************/
void FeatureExtractor::extractNodeShapeIndicator(nodePatch & np)
{
    if(np.eigen1 != 0)
        np.linearness=np.eigen0/np.eigen1;
    else
        np.linearness=9999;

    if(np.eigen2 != 0)
        np.planarness=np.eigen1/np.eigen2;
    else
    {
        if(np.eigen1 != 0)
            np.planarness=9999;
        else
            np.planarness=0;
    }
}

/******************************Extract node simplified points******************************/
void FeatureExtractor::getSimplifiedPoints(nodePatch & np)
{
    PointIndices & p =np.points;
    vector<PointVectorPair> points_pair = simplifyPoints(cloud,p);
    np.simplified_points=points_pair;
    simplifiedpoints=simplifiedpoints+points_pair.size();
}

vector<PointVectorPair> FeatureExtractor::simplifyPoints(PointCloud<PointT>::Ptr cloud_in,PointIndices p)
{
    vector<CGAL_Point> points;
    vector<CGAL_Point> output;
    Eigen::Vector3f point;
    for(int i=0;i<p.indices.size();i++)
    {
        point<<cloud_in->points[p.indices[i]].x,cloud_in->points[p.indices[i]].y,cloud_in->points[p.indices[i]].z;
        points.push_back(CGAL_Point(point(0),point(1),point(2)));
    }
    // percentage of points to retain.
    double retain_percentage = 10;
    if(points.size()*0.1<50)
    {
        retain_percentage=100*float(50)/float(points.size());
    }

    //cout<<"retain_percentage : "<<retain_percentage<<endl;
    double neighbor_radius = 0.15;   // neighbors size.
    CGAL::wlop_simplify_and_regularize_point_set<CGAL::Parallel_tag>
                      (points.begin(),points.end(),back_inserter(output),int(retain_percentage),neighbor_radius);

     //compute simplified pointcloud normals
     vector<PointVectorPair> points_pair;
     for(int i=0;i<output.size();i++)
     {
         PointVectorPair temp;
         temp.first=output[i];
         points_pair.push_back(temp);
     }
     //cout<<"simplified size is : "<<points_pair.size()<<endl;
     const int nb_neighbors = 50;
     CGAL::pca_estimate_normals(points_pair.begin(), points_pair.end(),
                                CGAL::First_of_pair_property_map<PointVectorPair>(),
                                CGAL::Second_of_pair_property_map<PointVectorPair>(),nb_neighbors);
     vector<PointVectorPair>::iterator unoriented_points_begin =
         CGAL::mst_orient_normals(points_pair.begin(), points_pair.end(),
                                  CGAL::First_of_pair_property_map<PointVectorPair>(),
                                  CGAL::Second_of_pair_property_map<PointVectorPair>(),nb_neighbors);
     points_pair.erase(unoriented_points_begin, points_pair.end());
     return points_pair;
}

void FeatureExtractor::buildEdges()
{
    findNeighbors();
    int id=1;
    for(int i=0;i<Nodes.size();i++)
    {
        nodePatch &np = Nodes[i];
        for(int j=0;j<np.neighbors.size();j++)
        {
            int neighborIdx= np.neighbors[j]-1;
            if(neighborIdx<i)
                continue;
            nodePatch &np2 = Nodes[neighborIdx];
            edgeLine el;
            el.edge_id=id;
            el.start=np.node_id;
            el.end=np2.node_id;
            el.adjacency=1;
            Edges.push_back(el);
            id++;
            el.edge_id=id;
            el.start=np2.node_id;
            el.end=np.node_id;
            el.adjacency=1;
            Edges.push_back(el);
            id++;
        }
        for(int j=0;j<np.neighbors2order.size();j++)
        {
            int neighborIdx= np.neighbors2order[j]-1;
            if(neighborIdx<i)
                continue;
            nodePatch &np2 = Nodes[neighborIdx];
            edgeLine el;
            el.edge_id=id;
            el.start=np.node_id;
            el.end=np2.node_id;
            el.adjacency=-1;
            Edges.push_back(el);
            id++;
            el.edge_id=id;
            el.start=np2.node_id;
            el.end=np.node_id;
            el.adjacency=-1;
            Edges.push_back(el);
            id++;
        }

    }
    cout<<"Edges number: "<<Edges.size()/2<<endl;
}




void FeatureExtractor::findNeighbors()
{
//    cout<<"----extract first order neighbors..."<<endl;

    for(int i=0;i<Nodes.size();i++)
    {
        extractFirstOrderNeighbors(i);
    }
//    cout<<"----extract second order neighbors..."<<endl;

    for(int i=0;i<Nodes.size();i++)
    {
        extractSecondOrderNeighbors(i);
    }
    checkNeighbors();

}


void FeatureExtractor::extractFirstOrderNeighbors(int i)
{
    //clear the neighbors
    nodePatch & np = Nodes[i];
    np.neighbors.clear();
    for(int j=0;j<Nodes.size();j++)
    {
        if(j==i)
            continue;
        nodePatch &np2 = Nodes[j];
        if(np.x_max+0.01<np2.x_min || np.x_min>np2.x_max+0.01
                || np.y_max+0.01<np2.y_min || np.y_min>np2.y_max+0.01
                || np.z_max+0.01<np2.z_min || np.z_min>np2.z_max+0.01)
            continue;
        np.neighbors.push_back(np2.node_id);
    }
}

void FeatureExtractor::extractSecondOrderNeighbors(int i)
{
    //extract two order neighbors
    nodePatch &np=Nodes[i];
    np.neighbors2order.clear();
    if(np.neighbors.size()>5)
        return;
    for(int j=0;j<np.neighbors.size();j++)
    {
        int currentNeighbor = np.neighbors[j]-1;
        nodePatch &np2=Nodes[currentNeighbor];
        for(int m=0;m<np2.neighbors.size();m++)
        {
            int currenIdx = np2.neighbors[m];
            //only consider the bigger indices to ensure uniqueness
            if(currenIdx-1<np.node_id)
                continue;
            //find a new 2 order neighbor
            int flag=0;
            for(int n=0;n<np.neighbors.size();n++)
            {
                if(currenIdx==np.neighbors[n])
                {
                    flag=1;
                    break;
                }
            }
            //check if it is already in the 2 order neighbor list
            if(flag==0)
            {
                int exitflag=0;
                for(int n=0;n<np.neighbors2order.size();n++)
                {
                    if(currenIdx==np.neighbors2order[n])
                    {
                        exitflag=1;
                        break;
                    }
                }
                if(exitflag==0)
                {
                    double temp=computeMindisof2patch(i,currenIdx-1);
                    //if two nodes are too far away, continue;
                    if(temp>0.6)
                        continue;
                    np.neighbors2order.push_back(currenIdx);
                    Nodes[currenIdx-1].neighbors2order.push_back(np.node_id);

                }
            }
        }
    }


}
void FeatureExtractor::checkNeighbors()
{
    for(int i=0;i<Nodes.size();i++)
    {
        nodePatch & np=Nodes[i];

        //put neighbors into a new vector
        vector<int> neighbors;
        for(int j=0;j<np.neighbors.size();j++)
        {
            neighbors.push_back(np.neighbors[j]);
        }
        for(int j=0;j<np.neighbors2order.size();j++)
        {
            neighbors.push_back(np.neighbors2order[j]);
        }

        //check if there are repeated neighbors
        for(int j=0;j<neighbors.size();j++)
        {
            for(int m=j+1;m<neighbors.size();m++)
            {
                if(neighbors[j]==neighbors[m])
                    cout<<"Error: repeated node neighbors!"<<endl;
            }
        }

    }

}
double FeatureExtractor::computeMindisof2patch(int i,int j)
{
    nodePatch &np=Nodes[i];
    nodePatch &np2=Nodes[j];
    double min_dis=max_dis;
    vector<PointVectorPair> simplified_i=np.simplified_points;
    vector<PointVectorPair> simplified_j=np2.simplified_points;
    for(int m=0;m<simplified_i.size();m++)
    {
        for(int n=0;n<simplified_j.size();n++)
        {
            PointVectorPair pair_i=simplified_i[m];
            PointVectorPair pair_j=simplified_j[n];
            double temp_dis=pointdis(pair_i.first.x(),pair_i.first.y(),pair_i.first.z(),
                                     pair_j.first.x(),pair_j.first.y(),pair_j.first.z());

            if(temp_dis<min_dis)
            {min_dis=temp_dis;}
        }
    }
    return min_dis;
}

bool FeatureExtractor::mergeNeighboringNodes(void)
{
    bool merged=false;
    int orig_node_number=Nodes.size();
    for(int i=0;i<Nodes.size();i++)
    {
        nodePatch &np=Nodes[i];
        if(np.points.indices.size()<5)
            continue;
        extractNodelimits(np);
    }
    vector<int> merge_flag;
    merge_flag.resize(Nodes.size(),1);
    for(int current_index=0;current_index<Nodes.size();current_index++)
    {
        nodePatch &node_s=Nodes[current_index];
        if(merge_flag[current_index]==0 || node_s.points.indices.size()<5)
            continue;
        else
        {
            extractMergeFetures(node_s);
            extractFirstOrderNeighbors(current_index);
            vector<int> neighbors=node_s.neighbors;
            for(int j=0;j<neighbors.size();j++)
            {
                int neighbor_index=neighbors[j]-1;
                nodePatch & node_e = Nodes[neighbor_index];
                if(merge_flag[neighbor_index]==0 || node_e.points.indices.size()<5)
                    continue;
                //compute coplanarity
                extractMergeFetures(node_e);
                int coplanarity;
                Eigen::Vector3f center_v=node_s.center-node_e.center;
                center_v=center_v/sqrt(pow(center_v(0),2)+pow(center_v(1),2)+pow(center_v(2),2));
                float angle_bt_nm,angle_bt_center_normal;
                angle_bt_nm=node_s.normal(0)*node_e.normal(0)+node_s.normal(1)*node_e.normal(1)
                        +node_s.normal(2)*node_e.normal(2);
                angle_bt_center_normal=center_v(0)*node_s.normal(0)+center_v(1)*node_s.normal(1)+center_v(2)*node_s.normal(2);
                double mergeconst = double(min(node_s.points.indices.size(),node_e.points.indices.size()))/double(cloud->points.size());
                mergeconst = 60*mergeconst;
//                if(abs(angle_bt_nm)>0.99 && angle_bt_center_normal<0.005 && abs(node_s.center(2)-node_e.center(2))<0.1)
                double angle_bt_nm_th = min(0.99*mergeconst,0.99);
                double angle_bt_center_th = max(0.005/mergeconst,0.005);
                double center_dis_th = max(0.1/mergeconst,0.1);

                 if(abs(angle_bt_nm)> angle_bt_nm_th &&
                         abs(angle_bt_center_normal)<angle_bt_center_th &&
                         abs(node_s.center(2)-node_e.center(2))<center_dis_th)
                    coplanarity=1;
                else
                    coplanarity=-1;
//                 cout<<"coplanarity : "<<coplanarity<<endl<<endl;
                if(coplanarity>0)
                {
                    mergeNodes(current_index,neighbor_index);
                    merged=true;
                    merge_flag[neighbor_index]=0;
                }
            }
        }
    }

    vector<nodePatch>::iterator it;
    int nodeid=1;
    for(it=Nodes.begin();it!=Nodes.end();)
    {
        nodePatch & np =*it;
        int current_index=np.node_id-1;
        if(merge_flag[current_index]==0)
            it=Nodes.erase(it);
        else
        {
            np.node_id=nodeid;
            nodeid++;
            it++;
        }
    }
    int current_node_number=Nodes.size();
    cout<<"merged "<<orig_node_number-current_node_number<<" nodes........."<<endl;
    return merged;

}


void FeatureExtractor::mergeNodes(int i,int j)
{
    nodePatch & np_keep= Nodes[i];
    nodePatch & np_remove = Nodes[j];


    PointIndices & p_keep = np_keep.points;
    PointIndices & p_remove = np_remove.points;

    PointIndices & rgbp_keep = np_keep.rgbpoints;
    PointIndices & rgbp_remove = np_remove.rgbpoints;

    vector<PointVectorPair> & sp_keep = np_keep.simplified_points;
    vector<PointVectorPair> & sp_remove = np_remove.simplified_points;
    if(p_keep.indices.size()<p_remove.indices.size())
        np_keep.label=np_remove.label;
    for(int k=0;k<np_remove.segment.size();k++)
        np_keep.segment.push_back(np_remove.segment[k]);
    for(int k=0;k<p_remove.indices.size();k++)
        p_keep.indices.push_back(p_remove.indices[k]);
    for(int k=0;k<rgbp_remove.indices.size();k++)
        rgbp_keep.indices.push_back(rgbp_remove.indices[k]);
    for(int k=0;k<sp_remove.size();k++)
        sp_keep.push_back(sp_remove[k]);

}



void FeatureExtractor::extractMergeFetures(nodePatch & np)
{
    getPCAfigures(np);
    extractNodecenter(np);
    extractNodeNormal(np);
}


void FeatureExtractor::extractEdgefeatures()
{
    //cout<<"starting edgefeature1"<<endl;

    for(int i=0;i<Edges.size();i=i+2)
    {
        extractEdgemindis(i);
        extractEdgevisualfeatures(i);
        extractEdgegeometryfeatures(i);
        extractEdgecoplanarity(i);
        extractEdgepositionfeatures(i);
    }
    for(int i=0;i<Edges.size();i++)
    {
        edgeLine & el=Edges[i];
        el.features.push_back(el.dis_hsv(0));
        el.features.push_back(el.dis_hsv(1));
        el.features.push_back(el.dis_hsv(2));
        el.features.push_back(el.adjacency);
        el.features.push_back(el.hor_dis);
        el.features.push_back(el.ver_dis);
        el.features.push_back(el.angle_bt_nm);
        el.features.push_back(el.mindis);
        el.features.push_back(el.coplanarity);
        el.features.push_back(el.angle_bt_ver);
    }

    //cout<<"starting edgefeature2"<<endl;
}

void FeatureExtractor:: extractEdgemindis(int id)
{
    edgeLine & el = Edges[id];
    edgeLine & el2 = Edges[id+1];
    double min_dis=computeMindisof2patch(el.start-1,el.end-1);
    el.mindis = min_dis;
    el2.mindis = min_dis;
}

void FeatureExtractor::extractEdgevisualfeatures(int id)
{
    edgeLine & el = Edges[id];
    edgeLine & el2 = Edges[id+1];
    nodePatch & node_s = Nodes[el.start-1];
    nodePatch & node_e = Nodes[el.end-1];
    el.dis_hsv=node_s.hsv-node_e.hsv;
    el.dis_hsv(0)=abs(el.dis_hsv(0));
    el.dis_hsv(1)=abs(el.dis_hsv(1));
    el.dis_hsv(2)=abs(el.dis_hsv(2));
    el2.dis_hsv=el.dis_hsv;
}

void FeatureExtractor::extractEdgegeometryfeatures(int id)
{
    edgeLine & el = Edges[id];
    edgeLine & el2 = Edges[id+1];
    nodePatch & node_s = Nodes[el.start-1];
    nodePatch & node_e = Nodes[el.end-1];

    Eigen::Vector3f c_vector=node_e.center-node_s.center;
    /****************Angle between normals**********************/

     Eigen::Vector3f &normalstart=node_s.normal;
     Eigen::Vector3f &normalend = node_e.normal;

     el.angle_bt_ver=abs(acos(normalstart(2))-acos(normalend(2)));
     el2.angle_bt_ver=el.angle_bt_ver;

     el.angle_bt_nm=normalstart(0)*normalend(0)+normalstart(1)*normalend(1)+normalstart(2)*normalend(2);
     if(el.angle_bt_nm<0)
     {
         el.angle_bt_nm=-el.angle_bt_nm;
     }
     el2.angle_bt_nm = el.angle_bt_nm;

}
void FeatureExtractor::extractEdgecoplanarity(int id)
{

    edgeLine & el = Edges[id];
    // cout<<"start : "<<el.start<<"   end : "<<el.end<<endl;
    edgeLine & el2 = Edges[id+1];
    nodePatch & node_s = Nodes[el.start-1];
    nodePatch & node_e = Nodes[el.end-1];
    Eigen::Vector3f center_v=node_s.center-node_e.center;
    center_v=center_v/sqrt(pow(center_v(0),2)+pow(center_v(1),2)+pow(center_v(2),2));
    float angle_bt_center_normal;
    angle_bt_center_normal=center_v(0)*node_s.normal(0)+center_v(1)*node_s.normal(1)+center_v(2)*node_s.normal(2);
    if(abs(el.angle_bt_nm)<0.8)
    {
        el.coplanarity=-1;
        el2.coplanarity=-1;
        return;
    }
    Eigen::Vector3f normal=getMergednormal(node_s,node_e);
    float angles=normal(0)*node_s.normal(0)+normal(1)*node_s.normal(1)+normal(2)*node_s.normal(2);
    float anglee=normal(0)*node_e.normal(0)+normal(1)*node_e.normal(1)+normal(2)*node_e.normal(2);
    float center_angle=center_v(0)*normal(0)+center_v(1)*normal(1)+center_v(2)*normal(2);
    if(abs(angles)>0.9 && abs(anglee) >0.9 )
    {
        if(abs(center_angle)<0.1 )
        {
            if(center_angle!=0)
            {
                el.coplanarity=1/abs(center_angle);
                el2.coplanarity=1/abs(center_angle);
            }
            else
            {
                el.coplanarity=9999;
                el2.coplanarity=9999;
            }
        }
        else
        {
            el.coplanarity=-1;
            el2.coplanarity=-1;
        }
    }
    else
    {
        el.coplanarity=-1;
        el2.coplanarity=-1;
    }
    //cout<<el.coplanarity<<" "<<el2.coplanarity<<endl;
}
Eigen::Vector3f FeatureExtractor::getMergednormal(nodePatch node_s,nodePatch node_e)
{
    PointCloud<PointT>::Ptr simplifiedpointcloud(new PointCloud<PointT>);
    PointCloud<PointT> & simcloud=*simplifiedpointcloud;
    vector<PointVectorPair> &pp=node_s.simplified_points;
    //cout<<pp.size()<<"  ";
    for (int j=0;j<pp.size();j++)
    {
         PointT p;
         PointVectorPair pptemp=pp[j];
         p.x=pptemp.first.x();
         p.y=pptemp.first.y();
         p.z=pptemp.first.z();
         simcloud.points.push_back(p);
    }

    vector<PointVectorPair> &pp2=node_e.simplified_points;
    //cout<<pp2.size()<<endl;

    for (int j=0;j<pp2.size();j++)
    {
         PointT p;
         PointVectorPair pptemp=pp2[j];
         p.x=pptemp.first.x();
         p.y=pptemp.first.y();
         p.z=pptemp.first.z();
         simcloud.points.push_back(p);
    }
    //cout<<simplifiedpointcloud->points.size()<<endl;

    PCA<PointT> pca;
    pca.setInputCloud(simplifiedpointcloud);

    //Eigen::Vector3f eigenvalue=pca.getEigenValues();
    Eigen::Matrix3f eigenvector=pca.getEigenVectors();
    Eigen::Vector3f normal=eigenvector.col(2);
    return normal;

}
void FeatureExtractor::extractEdgepositionfeatures(int id)
{
    edgeLine & el = Edges[id];
    edgeLine & el2 = Edges[id+1];
    nodePatch & node_s = Nodes[el.start-1];
    nodePatch & node_e = Nodes[el.end-1];
    float horizandis;
    horizandis=sqrt(pow(node_s.center(0)-node_e.center(0),2)+pow(node_s.center(1)-node_e.center(1),2));
    el.hor_dis=horizandis;
    el2.hor_dis=horizandis;
    el.ver_dis=node_s.center(2)-node_e.center(2);
    el2.ver_dis=-el.ver_dis;

}
double FeatureExtractor::pointdis(int p1,int p2)
{
    double dis;
    //if(__finite(cloud->points[p1].x) && __finite(cloud->points[p2].x))
    dis=pow(cloud->points[p1].x-cloud->points[p2].x,2)+pow(cloud->points[p1].y-cloud->points[p2].y,2)
            +pow(cloud->points[p1].z-cloud->points[p2].z,2);
    dis=sqrt(dis);
    return dis;
}

double FeatureExtractor::pointdis(double x1, double y1, double z1, double x2, double y2, double z2)
{
    double dis;
    dis=pow(x1-x2,2)+pow(y1-y2,2)+pow(z1-z2,2);
    dis=sqrt(dis);
    return dis;
}

vector<int> FeatureExtractor::getNeighbors(int pointIndex)
{
    vector<int> neiborIndex;
    int nbIdx[8]={pointIndex-cloud_HEIGHT-1,pointIndex-cloud_HEIGHT,pointIndex-cloud_HEIGHT+1,pointIndex-1,pointIndex+1,
                  pointIndex+cloud_HEIGHT-1,pointIndex+cloud_HEIGHT,pointIndex+cloud_HEIGHT+1};
    for(int i=0;i<8;i++)
    {
        if(nbIdx[i]>-1 && nbIdx[i]<cloud->points.size())
        {neiborIndex.push_back(nbIdx[i]);}
    }
    return neiborIndex;

}

int FeatureExtractor::getMainlabel(nodePatch np)
{

    vector<int> labelnode;
    vector<int> labelcnt;
    pcl::PointIndices pi;
    for(int i=0;i<np.points.indices.size();i++)
    {
        int index=np.points.indices[i];
        pi.indices.push_back(index);
    }
    for(int i=0;i<np.rgbpoints.indices.size();i++)
    {
        int index=np.rgbpoints.indices[i];
        pi.indices.push_back(index);
    }
    int unknowncnt=0;
    for(int j=0;j<pi.indices.size();j++)
    {
        int index=pi.indices[j];
        if(cloud->points[index].label == 0)
        {
            unknowncnt++;
            continue;
        }
        int flag=0;
        for(int m=0;m<labelnode.size();m++)
        {
            if(cloud->points[index].label == labelnode[m])
            {
                labelcnt[m]++;
                flag=1;
                break;
            }
        }
        if(flag==0)
        {
            labelcnt.push_back(1);
            labelnode.push_back(cloud->points[index].label);
        }
    }
    double unknownpercentage=double(unknowncnt)/double(pi.indices.size());
    if(unknownpercentage>0.9)
        return 0;
    if(labelnode.size()!= labelcnt.size())
    {
        cout<<"error counting!"<<endl;
    }
    int majorlabel;
    if(labelcnt.size()==0)
    {
        majorlabel=0;
    }
    else if(labelcnt.size()==1)
        majorlabel=labelnode[0];
    else
    {
        int labelmax=0;
        for(int m=1;m<labelnode.size();m++)
        {
            if(labelcnt[m]>labelcnt[labelmax])
                labelmax=m;
        }
        majorlabel=labelnode[labelmax];
    }
    return majorlabel;

}



//#include "featureextractor.h"
//#include <iostream>
//#include <math.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/common/pca.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/point_types_conversion.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/filters/statistical_outlier_removal.h>
//#include <pcl/search/kdtree.h>
//#include <pcl/search/impl/kdtree.hpp>
//#include "point_types_hf.h"

//#include <CGAL/Simple_cartesian.h>
//#include <CGAL/bounding_box.h>
//#include <CGAL/wlop_simplify_and_regularize_point_set.h>
//#include <CGAL/property_map.h>
//#include <CGAL/bilateral_smooth_point_set.h>
//#include <CGAL/tags.h>
//#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
//#include <CGAL/pca_estimate_normals.h>
//#include <CGAL/convex_hull_2.h>


//#include <CGAL/mst_orient_normals.h>
//using namespace pcl;
//using namespace std;
//#define PI 3.1415926
//#define FLOOR 2


//FeatureExtractor::FeatureExtractor()
//{
//}

//FeatureExtractor::~FeatureExtractor()
//{
//}

//void FeatureExtractor::setInputCloud(PointCloud<PointT>::Ptr cloud_in,bool seg_label_agree)
//{
//    cloud=cloud_in;
//    SEG_LABEL_CONSISTENCY=seg_label_agree;
////    cout<<"cloud number : "<<cloud->points.size()<<endl;
//    if(SEG_LABEL_CONSISTENCY)
//    {
//        for(int i=0;i<cloud->points.size();i++)
//        {
//            int temp=cloud->points[i].label;
////            cout<<temp<<"  ";
//            cloud->points[i].segment=temp;
//        }
//    }
//}
//PointCloud<PointT>::Ptr FeatureExtractor::getInputCloud(void)
//{
//    return cloud;
//}

//std::vector<FeatureExtractor::nodePatch> FeatureExtractor::getNodes(void)
//{
//    return Nodes;
//}
//std::vector<FeatureExtractor::edgeLine> FeatureExtractor::getEdges(void)
//{
//    return Edges;
//}
//std::vector<float> FeatureExtractor::getLimit(void)
//{

//   return limits;
//}

//void FeatureExtractor::initialize()
//{
//    simplifiedpoints=0;smallseg=0;
//    segment.clear();seg_count.clear();seg_indices.clear();
//    Nodes.clear(); Edges.clear();label.clear();
//    WIDTH = cloud->width;
//    HEIGHT = cloud->height;
//    _flag.resize(WIDTH*HEIGHT,0);
//     /*****************Initialize kdtree********************************/


//    /*****************Filtering********************************/
////    PointIndices noisy_indices;
////    PointCloud<PointT>::Ptr cloud_filtered(new PointCloud<PointT>);
////    StatisticalOutlierRemoval<PointT> sor(true);
////    sor.setInputCloud (cloud);
////    sor.setMeanK (50);
////    sor.setStddevMulThresh (1);
////    sor.filter (*cloud_filtered);
////    sor.getRemovedIndices(noisy_indices);
////    for(int i=0;i<noisy_indices.indices.size();i++)
////    {
////        _flag[noisy_indices.indices[i]]=-1;
////    }
////    for(int i=0;i<cloud->points.size();i++)
////    {
////        if(!__finite(cloud->points[i].x))
////        {
////            _flag[i]=-1;
////        }
////    }
////    cout<<"Noisy points : "<<noisy_indices.indices.size()<<endl;
//    /****************Compute the point normals*****************/
////    NormalEstimation<PointT,Normal> ne;
////    ne.setInputCloud(cloud);
////    search::KdTree<PointT>::Ptr tree (new search::KdTree<PointXYZ> ());
////    ne.setSearchMethod (tree);
////    ne.setRadiusSearch (0.03);
////    ne.compute (*normals);
//}

//void FeatureExtractor::countsegments()
//{

//    int ite=0;
//    while(!__finite(cloud->points[ite].x))
//    { ite++;}
//    x_min=cloud->points[ite].x;
//    x_max=cloud->points[ite].x;
//    y_min=cloud->points[ite].y;
//    y_max=cloud->points[ite].y;
//    z_min=cloud->points[ite].z;
//    z_max=cloud->points[ite].z;
////    int invalid_label=cloud->points[ite-1].segment;

//    for (int i=0;i<cloud->points.size();i++)
//    {
//        //remove invalid points

////        if (!__finite(cloud->points[i].x) || cloud->points[i].segment==invalid_label)
////        {
////            _flag[i]=-1;
////            continue;
////        }
//        if(_flag[i]==-1){continue;}
//        if(SEG_LABEL_CONSISTENCY)
//        {
//            if(cloud->points[i].label==0){continue;}
//        }
//        else
//        {
//            if(cloud->points[i].segment==0){continue;}
//        }
////
//        // extract the size of the whole image
//        if(cloud->points[i].x>x_max)
//            x_max=cloud->points[i].x;
//        if(cloud->points[i].x<x_min)
//            x_min=cloud->points[i].x;
//        if(cloud->points[i].y>y_max)
//            y_max=cloud->points[i].y;
//        if(cloud->points[i].y<y_min)
//            y_min=cloud->points[i].y;
//        if(cloud->points[i].z>z_max)
//            z_max=cloud->points[i].z;
//        if(cloud->points[i].z<z_min)
//            z_min=cloud->points[i].z;
//        int flag=0;
//        for(int j=0;j<segment.size();j++)
//        {
//            if(cloud->points[i].segment == segment[j])
//            {
//                seg_count[j]++;
//                PointIndices& ptemp=seg_indices[j];
//                ptemp.indices.push_back(i);
//                flag=1;
//                break;
//            }
//        }
//        if(flag==0)
//        {
//            PointIndices pIndx;
//            pIndx.indices.push_back(i);
//            segment.push_back(cloud->points[i].segment);
//            label.push_back(cloud->points[i].label);
//            seg_count.push_back(1);
//            seg_indices.push_back(pIndx);
//        }

//    }
//    limits.push_back(x_max);
//    limits.push_back(x_min);
//    limits.push_back(y_max);
//    limits.push_back(y_min);
//    limits.push_back(z_max);
//    limits.push_back(z_min);
////     cout<<x_min<<" "<<x_max<<" "<<y_min<<" "<<y_max<<" "<<z_min<<" "<<z_max<<endl;
//    max_dis=pointdis(x_max,y_max,z_max,x_min,y_min,z_min);
//    cloud_center(0)=(x_max-x_min)/2;
//    cloud_center(1)=(y_max-y_min)/2;
//    cloud_center(2)=(z_max-z_min)/2;


////    cout<<"Max_dis:  "<<max_dis<<endl;
//    if((segment.size()!=seg_count.size()) || (segment.size()!=seg_indices.size()))
//        PCL_ERROR("count error...");
////     cout<<"Segments number "<<segment.size()<<endl;
////     for(int i=0;i<segment.size();i++)
////     {
////         cout<<"seg : "<<segment[i]<<" ; label : "<<label[i]<<endl;
////     }
//}

//void FeatureExtractor::buildNodes()
//{
//    int id=1;
//    for(int i=0;i<segment.size();i++)
//    {
//        if(seg_count[i]>50)
//        {
//            nodePatch np;
//            PointIndices & p=seg_indices[i];
//            for(int j=0;j<p.indices.size();j++)
//            {
//                _flag[p.indices[j]]=id;
//            }
//            np.node_id=id;
//            np.segment=segment[i];
//            np.points=p;
//            if(SEG_LABEL_CONSISTENCY)
//                np.label=label[i];
//            else
//            {
//                int maxlabel=getMainlabel(np);
//                np.label=maxlabel;
//            }
//            Nodes.push_back(np);
//            id++;
//        }

//    }

//    //    cout<<"Nodes number: "<<Nodes.size()<<endl;
//}


//int FeatureExtractor::extractNodefeatures()
//{

//    for(int i=0;i<Nodes.size();i++)
//    {
////        cout<<i<<"th nodes : "<<endl;
//        extractCGALfeatures(i);
//        extractNodecenter(i);
//        extractAxisAlignedbdbox(i);
//        extractPCLfeatures(i);
//        extractNodecolor(i);
//        extractNodebdbox(i);
////        cout<<"end"<<endl;
//  //      extractNodecontour(i);

//    }
//    if(SUCCESS==0)
//        return 0;
////    cout<<"Simplified points number : "<<simplifiedpoints<<endl;
//    //normalize the distance to boundary
//    float dis2xmin_min=0,dis2xmin_max=max_dis;
//    float dis2ymin_min=0,dis2ymin_max=max_dis;
//    float dis2xmax_min=0,dis2xmax_max=max_dis;
//    float dis2ymax_min=0,dis2ymax_max=max_dis;
//    for(int i=0;i<Nodes.size();i++)
//    {
//        nodePatch &np=Nodes[i];
//        if(np.dis2xmin>dis2xmin_max)  dis2xmin_max=np.dis2xmin;
//        if(np.dis2xmin<dis2xmin_min)  dis2xmin_min=np.dis2xmin;

//        if(np.dis2xmax>dis2xmax_max)  dis2xmax_max=np.dis2xmax;
//        if(np.dis2xmax<dis2xmax_min)  dis2xmax_min=np.dis2xmax;

//        if(np.dis2ymin>dis2ymin_max)  dis2ymin_max=np.dis2ymin;
//        if(np.dis2ymin<dis2ymin_min)  dis2ymin_min=np.dis2ymin;

//        if(np.dis2ymax>dis2ymax_max)  dis2ymax_max=np.dis2ymax;
//        if(np.dis2ymax<dis2ymax_min)  dis2ymax_min=np.dis2ymax;
//    }
//    if((dis2xmin_max-dis2xmin_min)*(dis2xmax_max-dis2xmax_min)*(dis2ymin_max-dis2ymin_min)*(dis2ymax_max-dis2ymax_min)!=0)
//    {
//        for(int i=0;i<Nodes.size();i++)
//        {
//            nodePatch &np=Nodes[i];
//            np.dis2xmin=(np.dis2xmin-dis2xmin_min)/(dis2xmin_max-dis2xmin_min);
//            np.dis2xmax=(np.dis2xmax-dis2xmax_min)/(dis2xmax_max-dis2xmax_min);

//            np.dis2ymin=(np.dis2ymin-dis2ymin_min)/(dis2ymin_max-dis2ymin_min);
//            np.dis2ymax=(np.dis2ymax-dis2ymax_min)/(dis2ymax_max-dis2ymax_min);

//            float a=np.dis2xmin,b=np.dis2ymin;
//            if(a>np.dis2xmax) a=np.dis2xmax;
//            if(b>np.dis2ymax) b=np.dis2ymax;

//            np.dis2bd=a;
//            if(a>b) np.dis2bd=b;

//        }
//    }
//    double horizon;
//    int floorcnt=0;
//    for(int i=0;i<Nodes.size();i++)
//    {
//        nodePatch &np=Nodes[i];
//        if(np.label==FLOOR)
//        {
//            horizon=horizon+np.height;
//            floorcnt++;
//        }
//    }
//    if(floorcnt>0)
//    {
//        horizon=horizon/floorcnt;
//        for(int i=0;i<Nodes.size();i++)
//        {
//            nodePatch &np=Nodes[i];
//            np.height=np.height-horizon;
//        }
//    }

//    for(int i=0;i<Nodes.size();i++)
//    {
//        nodePatch &np=Nodes[i];
//        np.features.push_back(np.hsv(0));
//        np.features.push_back(np.hsv(1));
//        np.features.push_back(np.hsv(2));

//        for(int j=0;j<np.base_colors.size();j++)
//        {
//            np.features.push_back(np.base_colors[j]);
//        }

//        np.features.push_back(np.area);
//        np.features.push_back(np.height);
//        np.features.push_back(np.normal_z);
//        np.features.push_back(np.principal_z);
//        np.features.push_back(np.dis2bd);
//        np.features.push_back(np.linearness);
//        np.features.push_back(np.planarness);
//        np.features.push_back(np.viewdirection);
//    }
//    return 1;

//}

//void FeatureExtractor::extractAxisAlignedbdbox(int id)
//{
//    /******************Extract node center******************/
//    nodePatch & np = Nodes[id];
////    vector<PointVectorPair> & pvp =np.simplified_points;
////    float x_min,x_max,y_min,y_max,z_min,z_max;
////    PointVectorPair vpoint=pvp[0];
////    x_min=vpoint.first.x();x_max=x_min;
////    y_min=vpoint.first.y();y_max=y_min;
////    z_min=vpoint.first.z();z_max=z_min;
////    for(int j=0;j<pvp.size();j++)
////    {
////        vpoint=pvp[j];
////        if(vpoint.first.x()>x_max) {x_max=vpoint.first.x();}if(vpoint.first.x()<x_min) {x_min=vpoint.first.x();}
////        if(vpoint.first.y()>y_max) {y_max=vpoint.first.y();}if(vpoint.first.y()<y_min) {y_min=vpoint.first.y();}
////        if(vpoint.first.z()>z_max) {z_max=vpoint.first.z();}if(vpoint.first.z()<z_min) {z_min=vpoint.first.z();}
////    }

////    np.x_max=x_max; np.x_min=x_min;
////    np.y_max=y_max; np.y_min=y_min;
////    np.z_max=z_max; np.z_min=z_min;
////    double deltax=x_max-x_min;
////    double deltay=y_max-y_min;
////    double deltaz=z_max-z_min;
////    np.patchsize.push_back(x_max-x_min);
////    np.patchsize.push_back(y_max-y_min);
////    np.patchsize.push_back(z_max-z_min);
//    float xmin,xmax,ymin,ymax,zmin,zmax;
//    PointIndices & pi=np.points;
//    xmin=cloud->points[pi.indices[0]].x;xmax=xmin;
//    ymin=cloud->points[pi.indices[0]].y;ymax=ymin;
//    zmin=cloud->points[pi.indices[0]].z;zmax=zmin;
//    for(int j=0;j<pi.indices.size();j++)
//    {

//        if(cloud->points[pi.indices[j]].x>xmax) {xmax=cloud->points[pi.indices[j]].x;}
//        if(cloud->points[pi.indices[j]].x<xmin) {xmin=cloud->points[pi.indices[j]].x;}
//        if(cloud->points[pi.indices[j]].y>ymax) {ymax=cloud->points[pi.indices[j]].y;}
//        if(cloud->points[pi.indices[j]].y<ymin) {ymin=cloud->points[pi.indices[j]].y;}
//        if(cloud->points[pi.indices[j]].z>zmax) {zmax=cloud->points[pi.indices[j]].z;}
//        if(cloud->points[pi.indices[j]].z<zmin) {zmin=cloud->points[pi.indices[j]].z;}
//    }
//    np.x_max=xmax; np.x_min=xmin;
//    np.y_max=ymax; np.y_min=ymin;
//    np.z_max=zmax; np.z_min=zmin;
//    np.bd_v=np.z_max-np.z_min;
//    if(np.y_max-np.y_min>np.x_max-np.x_min)
//    {
//        np.bd_h=np.y_max-np.y_min;
//    }
//    else
//    {
//        np.bd_h=np.x_max-np.x_min;
//    }
//    //if(np.height>2){np.height=3;}

//    //cout<<x_max<<" "<<x_min<<" "<<y_max<<" "<<y_min<<" "<<z_max<<" "<<z_min<<endl;
//}

//void FeatureExtractor::extractNodecenter(int id)
//{
//    /******************Extract node center******************/
//   nodePatch & np = Nodes[id];
//    PointIndices &p =np.points;
//    Eigen::Vector3f point,center;
//    center<<0,0,0;
//    for(int j=0;j<p.indices.size();j++)
//    {
//      point<<cloud->points[p.indices[j]].x,cloud->points[p.indices[j]].y,cloud->points[p.indices[j]].z;
//      center=center+point;
//    }
//   // cout<<" center : "<<center(0)<<" "<<center(1)<<" "<<center(2)<<endl;
//   // cout<<p.indices.size()<<endl;
//    if(p.indices.size()>0)
//        np.center=center/p.indices.size();
//    else
//    {
//        cout<<"Error : Node "<<id<<" has no points!";
//        return;
//    }

//    np.height = np.center(2);

//    np.dis2xmin=abs(np.center(0)-x_min);
//    np.dis2xmax=abs(np.center(0)-x_max);
//    np.dis2ymin=abs(np.center(1)-y_min);
//    np.dis2ymax=abs(np.center(1)-y_max);
//    //cout<<np.height<<endl;
//    //cout<<np.dis2xmin<<" "<<np.dis2xmax<<"  "<<np.dis2ymin<<"  "<<np.dis2ymax<<endl;



//    //cout<<id<<" center : "<<np.center(0)<<" "<<np.center(1)<<" "<<np.center(2)<<endl;
//}


//bool FeatureExtractor::isLocalIntersect(int i,int j)
//{
//    nodePatch & np1 = Nodes[i];
//    nodePatch & np2 = Nodes[j];
//    vector<pcl::PointXYZ> boundingbox_i = np1.boundingbox;
//    vector<pcl::PointXYZ> boundingbox_j = np2.boundingbox;
//    bool i2j=false,j2i=false;
//    for(int k=0;k<np1.boundingbox.size();k++)
//    {
//        PointXYZ p=np1.boundingbox[k];
//        if(isLocalIntersect(p,j))
//        {
//            i2j=true;
//            break;
//        }
//    }
//    if(i2j)
//        return true;
//    for(int k=0;k<np2.boundingbox.size();k++)
//    {
//        PointXYZ p=np2.boundingbox[k];
//        if(isLocalIntersect(p,i))
//        {
//            j2i=true;
//            break;
//        }
//    }
//    if(j2i)
//        return true;
//    else
//        return false;
//}
//bool FeatureExtractor::isLocalIntersect(pcl::PointXYZ p, int id)
//{
//    nodePatch & np = Nodes[id];
//    Eigen::Matrix3f eigenvector = np.eigenvector;
//    Eigen::Vector3f center = np.center;
//    Eigen::Vector3f R,U,L;
//    R=eigenvector.col(0);U=eigenvector.col(1);L=eigenvector.col(2);

//    Eigen::Vector3f diff,cp;
//    diff(0)=p.x-center(0);
//    diff(1)=p.y-center(1);
//    diff(2)=p.z-center(2);


//    cp(0) = diff(0)*R(0)+diff(1)*R(1)+diff(2)*R(2);
//    cp(1) = diff(0)*U(0)+diff(1)*U(1)+diff(2)*U(2);
//    cp(2) = diff(0)*L(0)+diff(1)*L(1)+diff(2)*L(2);

//    if((cp(0)>np.local_limit.x_min && cp(0)<np.local_limit.x_max &&
//        cp(1)>np.local_limit.y_min && cp(1)<np.local_limit.y_max &&
//        cp(2)>np.local_limit.z_min && cp(2)<np.local_limit.z_max))
//        return true;
//    else
//        return false;

//}

//void FeatureExtractor::extractNodebdbox(int id)
//{
//    /******************Extract node bdbox******************/
//    nodePatch & np = Nodes[id];
//    Eigen::Matrix3f eigenvector = np.eigenvector;
//    Eigen::Vector3f center = np.center;
//    Eigen::Vector3f R,U,L;
//    R=eigenvector.col(0);U=eigenvector.col(1);L=eigenvector.col(2);
//    vector<PointVectorPair> & points_pair = np.simplified_points;
//    vector<Eigen::Vector3f> localpoints;
//    for(int j=0;j<points_pair.size();j++)
//    {
//      PointVectorPair onepair = points_pair[j];
//      Eigen::Vector3f diff,cp;
//      diff(0)=onepair.first.x()-center(0);
//      diff(1)=onepair.first.y()-center(1);
//      diff(2)=onepair.first.z()-center(2);


//      cp(0) = diff(0)*R(0)+diff(1)*R(1)+diff(2)*R(2);
//      cp(1) = diff(0)*U(0)+diff(1)*U(1)+diff(2)*U(2);
//      cp(2) = diff(0)*L(0)+diff(1)*L(1)+diff(2)*L(2);
//      localpoints.push_back(cp);
//    }
//    Eigen::Vector3f onepoint = localpoints[0];
//    double x_min=onepoint(0),x_max=onepoint(0);
//    double y_min=onepoint(1),y_max=onepoint(1);
//    double z_min=onepoint(2),z_max=onepoint(2);

//    for(int j=0;j<localpoints.size();j++)
//    {
//      onepoint = localpoints[j];
//      if(onepoint(0)>x_max) {x_max=onepoint(0);}if(onepoint(0)<x_min) {x_min=onepoint(0);}
//      if(onepoint(1)>y_max) {y_max=onepoint(1);}if(onepoint(1)<y_min) {y_min=onepoint(1);}
//      if(onepoint(2)>z_max) {z_max=onepoint(2);}if(onepoint(2)<z_min) {z_min=onepoint(2);}
//    }
//    np.local_limit.x_max=x_max;
//    np.local_limit.x_min=x_min;
//    np.local_limit.y_max=y_max;
//    np.local_limit.y_min=y_min;
//    np.local_limit.z_max=z_max;
//    np.local_limit.z_min=z_min;


//    PointXYZ bdpoint;
//    vector<PointXYZ> localbounding;
//    bdpoint.x= x_min;bdpoint.y= y_max;bdpoint.z= z_max;
//    localbounding.push_back(bdpoint);
//    bdpoint.x= x_min;bdpoint.y= y_min;bdpoint.z= z_max;
//    localbounding.push_back(bdpoint);
//    bdpoint.x= x_max;bdpoint.y= y_min;bdpoint.z= z_max;
//    localbounding.push_back(bdpoint);
//    bdpoint.x= x_max;bdpoint.y= y_max;bdpoint.z= z_max;
//    localbounding.push_back(bdpoint);
//    bdpoint.x= x_min;bdpoint.y= y_max;bdpoint.z= z_min;
//    localbounding.push_back(bdpoint);
//    bdpoint.x= x_min;bdpoint.y= y_min;bdpoint.z= z_min;
//    localbounding.push_back(bdpoint);
//    bdpoint.x= x_max;bdpoint.y= y_min;bdpoint.z= z_min;
//    localbounding.push_back(bdpoint);
//    bdpoint.x= x_max;bdpoint.y= y_max;bdpoint.z= z_min;
//    localbounding.push_back(bdpoint);


//    for(int i=0;i<localbounding.size();i++)
//    {
//        PointXYZ  temp = localbounding[i];
//        PointXYZ  bdpoint;
//        bdpoint.x=R(0)*temp.x+U(0)*temp.y+L(0)*temp.z+center(0);
//        bdpoint.y=R(1)*temp.x+U(1)*temp.y+L(1)*temp.z+center(1);
//        bdpoint.z=R(2)*temp.x+U(2)*temp.y+L(2)*temp.z+center(2);
//        np.boundingbox.push_back(bdpoint);
//    }

////    for(int i=0;i<np.boundingbox.size();i++)
////    {
////        cout<<np.boundingbox[i]<<"   ";
////    }
////    cout<<endl;

//    typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
//    typedef K::Point_2 Point_2;
//    typedef std::vector<Point_2> Points;

//    vector<CGAL::Exact_predicates_inexact_constructions_kernel::Point_2> points_2d,convex_hull;
//    for(int i=0;i<localpoints.size();i++)
//    {
//        points_2d.push_back(CGAL::Exact_predicates_inexact_constructions_kernel::Point_2
//                            (localpoints[i](0),localpoints[i](1)));
//    }
//   //BUG: IF points_2d.size<4
//    if(points_2d.size()>3)
//    {
//        CGAL::convex_hull_2( points_2d.begin(), points_2d.end(), std::back_inserter(convex_hull));
//        SUCCESS=1;
//    }
//    else
//    {
//        cout<<endl<<"BUG: point size smaller than 4 when extracting convex hull!   ";
//        cout<<"simplified point size : "<<np.simplified_points.size()<<endl;
//        SUCCESS=0;
//        return;
//    }
//    float area=0;
//    for(int i=0;i<convex_hull.size();i++)
//    {
//        int j=(i+1)%convex_hull.size();
//        CGAL::Exact_predicates_inexact_constructions_kernel::Point_2  temp1 = convex_hull[i],temp2=convex_hull[j];
//        area=area+temp1.x()*temp2.y()-temp1.y()*temp2.x();
//    }

//    np.area=area;
////    cout<<area<<endl;

//    for(int i=0;i<convex_hull.size();i++)
//    {
//        CGAL::Exact_predicates_inexact_constructions_kernel::Point_2  temp = convex_hull[i];

//        PointXYZ  bdpoint;

//        bdpoint.x=R(0)*temp.x()+U(0)*temp.y()+center(0);
//        bdpoint.y=R(1)*temp.x()+U(1)*temp.y()+center(1);
//        bdpoint.z=R(2)*temp.x()+U(2)*temp.y()+center(2);
//        np.convex_hull.points.push_back(bdpoint);
//    }
////    cout<<id<<" convex_hull vertices : "<<np.convex_hull.points.size()<<endl;


////    cout<<id<<"th bounding box"<<endl;
////    for(int i=0;i<np.convex_hull.points.size();i++)
////    {
////        cout<<np.convex_hull.points[i].x<<" "<<np.convex_hull.points[i].y<<" "<<np.convex_hull.points[i].z<<endl;
////    }
////    cout<<np.x_min<<" "<<np.x_max<<" "<<np.y_min<<" "<<np.y_max<<" "<<np.z_min<<" "<<np.z_max<<endl;
//}



//void FeatureExtractor::extractNodecolor(int id)
//{
//    /*******************Extract mean color********************/
//    PointCloud<PointXYZRGB> cloud_node;
//    PointCloud<PointXYZHSV> cloud_hsv;
//    nodePatch & np = Nodes[id];
//    PointIndices &p = np.points;
//    for(int j=0;j<p.indices.size();j++)
//    {
//        PointXYZRGB point;
//        point.x=cloud->points[p.indices[j]].x;
//        point.y=cloud->points[p.indices[j]].y;
//        point.z=cloud->points[p.indices[j]].z;
//        point.r=cloud->points[p.indices[j]].r;
//        point.g=cloud->points[p.indices[j]].g;
//        point.b=cloud->points[p.indices[j]].b;
//        cloud_node.push_back(point);
//    }
//    PointCloudXYZRGBtoXYZHSV(cloud_node,cloud_hsv);
//    double h=0,s=0,v=0;
//    int r=0,g=0,b=0;
//    for(int j=0;j<cloud_hsv.points.size();j++)
//    {
//        h=h+cloud_hsv.points[j].h;
//        s=s+cloud_hsv.points[j].s;
//        v=v+cloud_hsv.points[j].v;
//        r=r+cloud_node.points[j].r;
//        g=g+cloud_node.points[j].g;
//        b=b+cloud_node.points[j].b;
//    }
////    cout<<id<<" : "<<r<<" "<<g<<" "<<b<<endl;
//    if(p.indices.size()>0)
//    {
//    np.hsv(0)=h/p.indices.size();
//    np.hsv(1)=s/p.indices.size();
//    np.hsv(2)=v/p.indices.size();
//    np.r=r/p.indices.size();
//    np.g=g/p.indices.size();
//    np.b=b/p.indices.size();
//    }
//    else
//    {
//        cout<<"Error : Node "<<id<<" has no points !";
//    }
////    cout<<id<<" : "<<np.r<<" "<<np.g<<" "<<np.b<<endl;
//    vector<float> & bc = np.base_colors;
//    bc.resize(10,0);
//    int sum=0;
//    for(int j=0;j<cloud_hsv.points.size();j++)
//    {
//        h=cloud_hsv.points[j].h/2;
//        s=cloud_hsv.points[j].s*255;
//        v=cloud_hsv.points[j].v*255;
//        if(v<46) {bc[0]++;sum++;}
//        if(s<43 && v>46 && v<220) {bc[1]++;sum++;}
//        if(s<30 && v>221 && v<255) {bc[2]++;sum++;}
//        if(s>43 && v>46)
//        {
//            if(h>156 || h<10) {bc[3]++;sum++;}
//            if(h>11  && h<25) {bc[4]++;sum++;}
//            if(h>26  && h<34) {bc[5]++;sum++;}
//            if(h>35  && h<77) {bc[6]++;sum++;}
//            if(h>78  && h<99) {bc[7]++;sum++;}
//            if(h>100 && h<124) {bc[8]++;sum++;}
//            if(h>125 && h<155) {bc[9]++;sum++;}
//        }
//    }
//    for(int i=0;i<bc.size();i++)
//    {
//        bc[i]=bc[i]/double(sum);
//    }
//}

//void FeatureExtractor::extractNodearea(int id)
//{
//    nodePatch & np = Nodes[id];;

//}



//void FeatureExtractor::extractNodecontour(int id)
//{
//    nodePatch & np = Nodes[id];
//    PointIndices & p=np.points;
//    PointIndices & p_c=np.contour;
//    for(int j=0;j<p.indices.size();j++)
//    {
//        bool contour_flag=false;
////        int count=0;
//        vector<int> neighbors=getNeighbors(p.indices[j]);
//        //cout<<"neighbor size: "<<neighbors.size();
////        for(int k=0;k<neighbors.size();k++)
////        {
////            if(cloud->points[neighbors[k]].segment==invalid_label)
////            {count++;}
////        }
////        if(neighbors.size()-count<3){continue;}
//        for(int k=0;k<neighbors.size();k++)
//        {
//            /*****Considr the NAN points and non-graph points when extract the contour*****/
//            if((cloud->points[p.indices[j]].segment != cloud->points[neighbors[k]].segment) |
//                    !__finite(cloud->points[neighbors[k]].x))
//            {
//                contour_flag=true;
//                break;
//            }

//        }
//        if(contour_flag == true)
//        {
//            p_c.indices.push_back(p.indices[j]);
//        }

//    }

//}

//void FeatureExtractor::extractEdgeadjacency(int id)
//{
//    edgeLine & el = Edges[id];
//    edgeLine & el2 =Edges[id+1];
//    int startnodeid = el.start;
//    int endnodeid = el.end;
//    nodePatch &np = Nodes[startnodeid-1];
//    nodePatch &np2 = Nodes[endnodeid-1];

//    if(np.x_max+0.01<np2.x_min || np.x_min>np2.x_max+0.01
//            || np.y_max+0.01<np2.y_min || np.y_min>np2.y_max+0.01
//            || np.z_max+0.01<np2.z_min || np.z_min>np2.z_max+0.01)
////    if(np.x_max<np2.x_min || np.x_min>np2.x_max
////            || np.y_max<np2.y_min || np.y_min>np2.y_max
////            || np.z_max<np2.z_min || np.z_min>np2.z_max)
//        return;

//    //if(el.mindis>0.05) return;
//    np.adjacency.push_back(np2.node_id);
//    el.adjacency=1;
//    np2.adjacency.push_back(np.node_id);
//    el2.adjacency=1;


//}

//void FeatureExtractor::extractPCLfeatures(int id)
//{
//    nodePatch & np = Nodes[id];
////    PointCloud<PointT>::Ptr simplifiedpointcloud(new PointCloud<PointT>);
////    vector<PointVectorPair> pp=np.simplified_points;
////    for (int j=0;j<pp.size();j++)
////    {
////         PointT p;
////         PointVectorPair pptemp=pp[j];
////         p.x=pptemp.first.x();
////         p.y=pptemp.first.y();
////         p.z=pptemp.first.z();
////         p.r=np.r;p.g=np.g;p.b=np.b;
////         simplifiedpointcloud->points.push_back(p);
////    }
////     pca.setInputCloud(simplifiedpointcloud);
//    PointIndices::Ptr pi_t(new PointIndices);
//    PointIndices &pi = *pi_t;
//    pi = np.points;
//    PCA<PointT> pca;
//    pca.setInputCloud(cloud);
//    pca.setIndices(pi_t);
//    Eigen::Vector3f eigenvalue=pca.getEigenValues();
//    Eigen::Matrix3f eigenvector=pca.getEigenVectors();
//    np.eigen0=eigenvalue(0);
//    np.eigen1=eigenvalue(1);
//    np.eigen2=eigenvalue(2);
//    np.scatter=eigenvalue(0);
//    np.normal=eigenvector.col(2);
//    np.eigenvector=eigenvector;

//    if(eigenvalue(1)!=0)
//        np.linearness=eigenvalue(0)/eigenvalue(1);
//    else
//        np.linearness=9999;

//    if(eigenvalue(2)!=0)
//        np.planarness=eigenvalue(1)/eigenvalue(2);
//    else
//    {
//        if(eigenvalue(1)!=0)
//            np.planarness=9999;
//        else
//            np.planarness=0;
//    }

////    cout<<"Node "<<id<<": "<<eigenvalue(0)<<"  "<<eigenvalue(1)<<"  "<<eigenvalue(2)<<endl;
////    Eigen::Vector3f & center=np.center;

//    Eigen::Vector3f & normal=np.normal;
//    Eigen::Vector3f & center=np.center;
//    Eigen::Vector3f relativecenter=cloud_center-np.center;

//    //flip normals to the center
////    if(normal(2)<0){normal=normal*(-1);}
//    //    double cos_theta=(center(0)-x_min)*normal(0)+(center(1)-y_min)*normal(1)+
//    //            (center(2)-z_max)*normal(2);
//    //    if(cos_theta>0){normal *= -1;}


//    if(normal(0)*relativecenter(0)+normal(1)*relativecenter(1)+normal(2)*relativecenter(2)<0)
//        normal=normal*(-1);
////    normal=normal*(-1);
//    np.normal_z=abs(normal(2));
//    if(np.normal_z>0.94){np.normal_z=1;}
//    else if(np.normal_z<0.34){np.normal_z=0;}
//    Eigen::Vector3f centertmp=center;
//    float nor=sqrt(pow(center(0),2)+pow(center(1),2)+pow(center(2),2));
//    centertmp(0)=centertmp(0)/nor;
//    centertmp(1)=centertmp(1)/nor;
//    centertmp(2)=centertmp(2)/nor;
//    np.viewdirection=normal(0)*centertmp(0)+normal(1)*centertmp(1)+normal(2)*centertmp(2);

//    float p1,p2;
//    p1=eigenvector.col(0)(2);
//    p2=eigenvector.col(1)(2);
//    if(abs(p1)>abs(p2))
//    {
//        np.principal_z=abs(p1);
//    }
//    else
//        np.principal_z=abs(p2);


//    //normal=normal/sqrt(pow(normal(0),2)+pow(normal(1),2)+pow(normal(2),2));

////    cos_theta=center(0)*normal(0)+center(1)*normal(1)+center(2)*normal(2);
// }

//void FeatureExtractor::extractCGALfeatures(int id)
//{
//    nodePatch & np = Nodes[id];
//    PointIndices & p =np.points;
////    cout<<"original points size : "<<p.indices.size()<<endl;
//    vector<CGAL_Point> points;
//    vector<CGAL_Point> output;
//    Eigen::Vector3f point;
//    for(int i=0;i<p.indices.size();i++)
//    {
//        point<<cloud->points[p.indices[i]].x,cloud->points[p.indices[i]].y,cloud->points[p.indices[i]].z;
//        points.push_back(CGAL_Point(point(0),point(1),point(2)));
//    }
//    double retain_percentage = 10;   // percentage of points to retain.
//    if(points.size()*0.1<50)
//    {
//        retain_percentage=100*float(50)/float(points.size());
//    }
////    cout<<"retain_percentage : "<<retain_percentage<<endl;
//    double neighbor_radius = 0.15;   // neighbors size.
//     CGAL::wlop_simplify_and_regularize_point_set
//                            <CGAL::Parallel_tag> // parallel version
//                            (points.begin(),
//                             points.end(),
//                             std::back_inserter(output),
//                             int(retain_percentage),
//                             neighbor_radius
//                             );

//     //compute simplified pointcloud normals

//     std::vector<PointVectorPair> points_pair;
//     for(int i=0;i<output.size();i++)
//     {
//         PointVectorPair temp;
//         temp.first=output[i];
//         points_pair.push_back(temp);
//     }
////     cout<<"simplified size is : "<<points_pair.size()<<endl;
//     const int nb_neighbors = 50;
//     CGAL::pca_estimate_normals(points_pair.begin(), points_pair.end(),
//                                CGAL::First_of_pair_property_map<PointVectorPair>(),
//                                CGAL::Second_of_pair_property_map<PointVectorPair>(),
//                                nb_neighbors);

//     std::vector<PointVectorPair>::iterator unoriented_points_begin =
//         CGAL::mst_orient_normals(points_pair.begin(), points_pair.end(),
//                                  CGAL::First_of_pair_property_map<PointVectorPair>(),
//                                  CGAL::Second_of_pair_property_map<PointVectorPair>(),
//                                  nb_neighbors);

//     points_pair.erase(unoriented_points_begin, points_pair.end());

////     cout<<"oriented points size : "<<points_pair.size()<<endl;
//     // Algorithm parameters
//     int k = 120;                 // size of neighborhood. The bigger the smoother the result will be.
//                                  // This value should bigger than 1.
//     double sharpness_angle = 25; // control sharpness of the result.
//                                  // The bigger the smoother the result will be
//     int iter_number = 3;         // number of times the projection is applied


//     for (int i = 0; i < iter_number; ++i)
//     {
//       /* double error = */
//       CGAL::bilateral_smooth_point_set <CGAL::Parallel_tag>(
//             points_pair.begin(),
//             points_pair.end(),
//             CGAL::First_of_pair_property_map<PointVectorPair>(),
//             CGAL::Second_of_pair_property_map<PointVectorPair>(),
//             k,
//             sharpness_angle);
//     }

//     np.simplified_points=points_pair;
////     cout<<"filtered size is : "<<points_pair.size()<<endl;
//     simplifiedpoints=simplifiedpoints+points_pair.size();
////     cout<<id<<" simplified points size : "<<np.simplified_points.size()<<endl;
//}

//void FeatureExtractor::buildEdges()
//{
//    findNeighbors();

//    int id=1;
//    for(int i=0;i<Nodes.size();i++)
//    {
//         nodePatch &np = Nodes[i];
//        for(int j=0;j<np.neighbors.size();j++)
//        {
//            if(np.neighbors[j]-1<i)
//                continue;
//            nodePatch &np2 = Nodes[np.neighbors[j]-1];
//            double min_dis=computeMindisof2patch(i,np.neighbors[j]-1);
//            edgeLine el;
//            el.edge_id=id;
//            el.start=np.node_id;
//            el.end=np2.node_id;
//            el.mindis=min_dis;
//            el.adjacency=-1;
//            Edges.push_back(el);
//            id++;
//            el.edge_id=id;
//            el.start=np2.node_id;
//            el.end=np.node_id;
//            el.mindis=min_dis;
//            el.adjacency=-1;
//            Edges.push_back(el);
//            id++;
//        }

//    }
//    //cout<<"Neighborsum: "<<neighborsum<<endl;
//    cout<<"Edges number: "<<Edges.size()/2<<endl;

//    cout<<"----checking neighbor uniqueness..."<<endl;

//    for(int i=0;i<Nodes.size();i++)
//    {
//        nodePatch & np=Nodes[i];
//        for(int j=0;j<np.neighbors.size();j++)
//        {
//            for(int m=j+1;m<np.neighbors.size();m++)
//            {
//                if(np.neighbors[j]==np.neighbors[m])
//                    cout<<"Error: repeated node neighbors!"<<endl;
//            }
//        }

//    }

//}

//void FeatureExtractor::findNeighbors()
//{
//    cout<<"----extract first order neighbors..."<<endl;
//    extractFirstOrderNeighbors();
//    cout<<"----find two closest neighbors for none-neighboring nodes..."<<endl;
//    for(int i=0;i<Nodes.size();i++)
//    {
//        nodePatch &np=Nodes[i];
//        if(np.neighbors.size()>1)continue;

//        float distance1=max_dis,distance2=max_dis;
//        int neareast1=Nodes.size(),neareast2=Nodes.size();
//        for(int j=0;j<Nodes.size();j++)
//        {
//            if(j==i)continue;
//            if(np.neighbors.size()==1)
//            {
//                if(j==np.neighbors[0]-1)
//                    continue;
//            }
//            nodePatch &np2=Nodes[j];
//            float temp=computeMindisof2patch(i,j);
//            if(temp>distance2 )
//                continue;
//            if(temp>distance1)
//            {
//               distance2=temp;
//               neareast2=j;
//            }
//            else
//            {
//                distance2=distance1;
//                neareast2=neareast1;
//                distance1=temp;
//                neareast1=j;
//            }

//        }
//        if(neareast1<Nodes.size())
//        {
//            nodePatch &np_neareast1=Nodes[neareast1];
//            np.neighbors.push_back(np_neareast1.node_id);
//            np_neareast1.neighbors.push_back(np.node_id);
//        }
//        if(neareast2<Nodes.size())
//        {
//            nodePatch &np_neareast2=Nodes[neareast2];
//            np.neighbors.push_back(np_neareast2.node_id);
//            np_neareast2.neighbors.push_back(np.node_id);
//        }
////        cout<<"Node number is : "<<Nodes.size()<<", nearest neighbors : "<<neareast1<<"  "<<neareast2<<endl;

//    }

////    extractSecondOrderNeighbors();

//    cout<<"----put neighbor indexes in vectors..."<<endl;
//    int neighborsum=0;
//    for(int i=0;i<Nodes.size();i++)
//    {
//        nodePatch &np =Nodes[i];
//        for(int j=0;j<np.neighbors2order.size();j++)
//        {
//            np.neighbors.push_back(np.neighbors2order[j]);
//        }
////        cout<<np.node_id<<" neighbors : ";
////        for(int j=0;j<np.neighbors.size();j++)
////        {
////            cout<<np.neighbors[j]<<" ";
////        }
////        cout<<endl;
//        neighborsum=neighborsum+np.neighbors.size();
//    }
////    for(int i=0;i<Nodes.size();i++)
////    {
////        nodePatch &np=Nodes[i];
////        cout<<np.node_id<<" 2orderneighbors : ";
////        for(int j=0;j<np.neighbors2order.size();j++)
////            cout<<np.neighbors2order[j]<<" ";
////        cout<<endl;
////    }

//}

//void FeatureExtractor::extractFirstOrderNeighbors(void)
//{
//    int cnt=0;
//    for(int i=0;i<Nodes.size();i++)
//    {
//        for(int j=i+1;j<Nodes.size();j++)
//        {
//            nodePatch &np = Nodes[i];
//            nodePatch &np2 = Nodes[j];
////            cout<<np.x_max<<"   "<<np.x_min<<"  "<<np.y_max<<"  "<<np.y_min<<"  "<<np.z_max<<"  "<<np.z_min<<endl;
////            if(np.x_max+0.05<np2.x_min || np.x_min>np2.x_max+0.05
////                    || np.y_max+0.05<np2.y_min || np.y_min>np2.y_max+0.05
////                    || np.z_max+0.05<np2.z_min || np.z_min>np2.z_max+0.05)
//            if(np.x_max<np2.x_min || np.x_min>np2.x_max
//                    || np.y_max<np2.y_min || np.y_min>np2.y_max
//                    || np.z_max<np2.z_min || np.z_min>np2.z_max)
//            continue;
//            cnt++;
//            np.neighbors.push_back(np2.node_id);
//            np2.neighbors.push_back(np.node_id);
//        }
//    }
//}

//void FeatureExtractor::extractSecondOrderNeighbors(void)
//{
//    int cnt2=0;
//    //extract two order neighbors
//    for(int i=0;i<Nodes.size();i++)
//    {
//        nodePatch &np=Nodes[i];
//        //np.neighbors2order.clear();
//        if(np.neighbors.size()>4) continue;
//         //cout<<"np neighbor size: "<<np.neighbors.size()<<endl;
//        for(int j=0;j<np.neighbors.size();j++)
//        {
//            nodePatch &np2=Nodes[np.neighbors[j]-1];
//             //cout<<"np2 neighbor size: "<<np2.neighbors.size()<<endl;

//            for(int m=0;m<np2.neighbors.size();m++)
//            {
//                //cout<<"m : "<<m<<endl;
//                if(np2.neighbors[m]-1<np.node_id) continue;

//                int flag=0;
//                for(int n=0;n<np.neighbors.size();n++)
//                {
//                    if(np2.neighbors[m]==np.neighbors[n])
//                    {
//                        flag=1;
//                        break;
//                    }
//                }
//                if(flag==0)
//                {
//                    int exitflag=0;
//                    for(int n=0;n<np.neighbors2order.size();n++)
//                    {
//                        if(np2.neighbors[m]==np.neighbors2order[n])
//                        {
//                            exitflag=1;
//                            break;
//                        }
//                    }
//                    if(exitflag==0)
//                    {
//                        double temp=computeMindisof2patch(i,np2.neighbors[m]-1);
//                        if(temp>0.5) continue;
//                        np.neighbors2order.push_back(np2.neighbors[m]);
//                        //cout<<np.node_id<<"push back "<<np2.neighbors[m]<<endl;
//                        Nodes[np2.neighbors[m]-1].neighbors2order.push_back(np.node_id);
//                        //cout<<np2.neighbors[m]<<"push back "<<np.node_id<<endl;
//                        cnt2++;
//                    }
//                }
//            }
//        }
//    }
////    for(int i=0;i<Nodes.size();i++)
////    {
////        nodePatch &np =Nodes[i];

////        cout<<np.node_id<<" 2 order neighbors : ";
////        for(int j=0;j<np.neighbors2order.size();j++)
////        {
////            cout<<np.neighbors2order[j]<<" ";
////        }
////        cout<<endl;
////    }
//}

//double FeatureExtractor::computeMindisof2patch(int i,int j)
//{
//    nodePatch &np=Nodes[i];
//    nodePatch &np2=Nodes[j];
//    double min_dis=max_dis;
//    vector<PointVectorPair> simplified_i=np.simplified_points;
//    vector<PointVectorPair> simplified_j=np2.simplified_points;
//    for(int m=0;m<simplified_i.size();m++)
//    {
//        for(int n=0;n<simplified_j.size();n++)
//        {
//            PointVectorPair pair_i=simplified_i[m];
//            PointVectorPair pair_j=simplified_j[n];
//            double temp_dis=pointdis(pair_i.first.x(),pair_i.first.y(),pair_i.first.z(),
//                                     pair_j.first.x(),pair_j.first.y(),pair_j.first.z());

//            if(temp_dis<min_dis)
//            {min_dis=temp_dis;}
//        }
//    }
//    return min_dis;
//}

//void FeatureExtractor::constructGraph(void)
//{

//}

//bool FeatureExtractor::mergeNeighboringNodes(void)
//{
//   vector<nodePatch>::iterator it;
//   int orig_node_number=Nodes.size();
//   for(it= Edges.begin();it!= Edges.end();)
//   {
//       edgeLine el=*it;
//       int i = el.edge_id;
//       extractEdgevisualfeatures(i-1);
//       extractEdgecoplanarity(i-1);

//       if(el.coplanarity>0 && el.dis_hsv(0)<30)



//    if(*it % 3 ==0)
//         it=iVec.erase(it);    //删除元素，返回值指向已删除元素的下一个位置
//       else
//           ++it;    //指向下一个位置
//   }

//}

//void FeatureExtractor::deleteInvalidNodes(void)
//{

//}





//void FeatureExtractor::extractEdgefeatures()
//{
//    //cout<<"starting edgefeature1"<<endl;
//    for(int i=0;i<Edges.size();i=i+2)
//    {
//        extractEdgevisualfeatures(i);
//        extractEdgegeometryfeatures(i);
//        extractEdgeadjacency(i);
//        extractEdgecoplanarity(i);
//        extractEdgepositionfeatures(i);
//    }
//    for(int i=0;i<Edges.size();i++)
//    {
//        edgeLine & el=Edges[i];
//        el.features.push_back(el.dis_hsv(0));
//        el.features.push_back(el.dis_hsv(1));
//        el.features.push_back(el.dis_hsv(2));
//        el.features.push_back(el.coplanarity);
//        el.features.push_back(el.mindis);
//        el.features.push_back(el.angle_bt_nm);
//        el.features.push_back(el.angle_bt_ver);
//        el.features.push_back(el.parallel);
//        el.features.push_back(el.perpendicularity);
//    }

//    //cout<<"starting edgefeature2"<<endl;
//}

//void FeatureExtractor::extractEdgevisualfeatures(int id)
//{
//    edgeLine & el = Edges[id];
//    edgeLine & el2 = Edges[id+1];
//    nodePatch & node_s = Nodes[el.start-1];
//    nodePatch & node_e = Nodes[el.end-1];
//    el.dis_hsv=node_s.hsv-node_e.hsv;
//    el.dis_hsv(0)=abs(el.dis_hsv(0));
//    el.dis_hsv(1)=abs(el.dis_hsv(1));
//    el.dis_hsv(2)=abs(el.dis_hsv(2));
//    el2.dis_hsv=el.dis_hsv;
//}

//void FeatureExtractor::extractEdgegeometryfeatures(int id)
//{
//    edgeLine & el = Edges[id];
//    edgeLine & el2 = Edges[id+1];
//    nodePatch & node_s = Nodes[el.start-1];
//    nodePatch & node_e = Nodes[el.end-1];

//    Eigen::Vector3f c_vector=node_e.center-node_s.center;
//    /****************Angle between normals**********************/

//     Eigen::Vector3f &normalstart=node_s.normal;
//     Eigen::Vector3f &normalend = node_e.normal;

//     el.angle_bt_ver=abs(acos(normalstart(2))-acos(normalend(2)));
//     el2.angle_bt_ver=el.angle_bt_ver;

//     el.angle_bt_nm=normalstart(0)*normalend(0)+normalstart(1)*normalend(1)+normalstart(2)*normalend(2);
//     if(el.angle_bt_nm<0)
//     {
//         el.angle_bt_nm=-el.angle_bt_nm;
//     }
//     el2.angle_bt_nm = el.angle_bt_nm;

////    }
//    /***********************parallel************************/
//     double th=cos(PI/6);
//     if(el.angle_bt_nm>th)
//     {
//         el.parallel=1;
//         el2.parallel=1;
//     }
//     else
//     {
//         el.parallel=-1;
//         el2.parallel=-1;
//     }

////    double dij=c_vector(0)*node_s.normal(0)+c_vector(1)*node_s.normal(1)
////                   +c_vector(2)*node_s.normal(2);

////    double dji=-(c_vector(0)*node_e.normal(0)+c_vector(1)*node_e.normal(1)
////                   +c_vector(2)*node_e.normal(2));


//    /***********************perpendicularity************************/
//     double th2=cos(PI/3);
//     if(el.angle_bt_nm<th2)
//     {
//         el.perpendicularity=1;
//         el2.perpendicularity=1;
//     }
//     else
//     {
//         el.perpendicularity=-1;
//         el2.perpendicularity=-1;
//     }

//}
//void FeatureExtractor::extractEdgecoplanarity(int id)
//{

//    edgeLine & el = Edges[id];
//    // cout<<"start : "<<el.start<<"   end : "<<el.end<<endl;
//    edgeLine & el2 = Edges[id+1];
//    nodePatch & node_s = Nodes[el.start-1];
//    nodePatch & node_e = Nodes[el.end-1];
//    Eigen::Vector3f center_v=node_s.center-node_e.center;
//    center_v=center_v/sqrt(pow(center_v(0),2)+pow(center_v(1),2)+pow(center_v(2),2));
//    float angle_bt_center_normal;
//    angle_bt_center_normal=center_v(0)*node_s.normal(0)+center_v(1)*node_s.normal(1)+center_v(2)*node_s.normal(2);
//    if(abs(el.angle_bt_nm)<0.8)
//    {
//        el.coplanarity=-1;
//        el2.coplanarity=-1;
//        return;
//    }

//    PointCloud<PointT>::Ptr simplifiedpointcloud(new PointCloud<PointT>);
//    PointCloud<PointT> & simcloud=*simplifiedpointcloud;
//    vector<PointVectorPair> &pp=node_s.simplified_points;
//    //cout<<pp.size()<<"  ";
//    for (int j=0;j<pp.size();j++)
//    {
//         PointT p;
//         PointVectorPair pptemp=pp[j];
//         p.x=pptemp.first.x();
//         p.y=pptemp.first.y();
//         p.z=pptemp.first.z();
//         simcloud.points.push_back(p);
//    }

//    vector<PointVectorPair> &pp2=node_e.simplified_points;
//    //cout<<pp2.size()<<endl;

//    for (int j=0;j<pp2.size();j++)
//    {
//         PointT p;
//         PointVectorPair pptemp=pp2[j];
//         p.x=pptemp.first.x();
//         p.y=pptemp.first.y();
//         p.z=pptemp.first.z();
//         simcloud.points.push_back(p);
//    }
//    //cout<<simplifiedpointcloud->points.size()<<endl;

//    PCA<PointT> pca;
//    pca.setInputCloud(simplifiedpointcloud);

//    //Eigen::Vector3f eigenvalue=pca.getEigenValues();
//    Eigen::Matrix3f eigenvector=pca.getEigenVectors();
//    Eigen::Vector3f normal=eigenvector.col(2);

//    float angles=normal(0)*node_s.normal(0)+normal(1)*node_s.normal(1)+normal(2)*node_s.normal(2);
//    float anglee=normal(0)*node_e.normal(0)+normal(1)*node_e.normal(1)+normal(2)*node_e.normal(2);
//    //float planarness=eigenvalue(1)/eigenvalue(2);
//    //float pls=node_s.planarness/planarness,ple=node_e.planarness/planarness;
//    //cout<<center_v(0)<<" "<<center_v(1)<<" "<<center_v(2)<<endl;
//    float center_angle=center_v(0)*normal(0)+center_v(1)*normal(1)+center_v(2)*normal(2);
//    //cout<<id+1<<" "<<angles<<" "<<anglee<<" "<<center_angle<<endl;
//    if(abs(angles)>0.9 && abs(anglee) >0.9 )
//    {
//        if(abs(center_angle)<0.1 )
//        {
//            if(center_angle!=0)
//            {
//                el.coplanarity=1/abs(center_angle);
//                el2.coplanarity=1/abs(center_angle);
//            }
//            else
//            {
//                el.coplanarity=9999;
//                el2.coplanarity=9999;
//            }
//        }
//        else
//        {
//            el.coplanarity=-1;
//            el2.coplanarity=-1;
//        }
//    }
//    else
//    {
//        el.coplanarity=-1;
//        el2.coplanarity=-1;
//    }
//    //cout<<el.coplanarity<<" "<<el2.coplanarity<<endl;
//}



//void FeatureExtractor::extractEdgepositionfeatures(int id)
//{
//    edgeLine & el = Edges[id];
//    edgeLine & el2 = Edges[id+1];
//    nodePatch & node_s = Nodes[el.start-1];
//    nodePatch & node_e = Nodes[el.end-1];
//    float horizandis;
//    horizandis=sqrt(pow(node_s.center(0)-node_e.center(0),2)+pow(node_s.center(1)-node_e.center(1),2));
//    el.hor_dis=horizandis;
//    el2.hor_dis=horizandis;
//    el.ver_dis=node_s.center(2)-node_e.center(2);
//    el2.ver_dis=-el.ver_dis;

//}


//double FeatureExtractor::pointdis(int p1,int p2)
//{
//    double dis;
//    //if(__finite(cloud->points[p1].x) && __finite(cloud->points[p2].x))
//    dis=pow(cloud->points[p1].x-cloud->points[p2].x,2)+pow(cloud->points[p1].y-cloud->points[p2].y,2)
//            +pow(cloud->points[p1].z-cloud->points[p2].z,2);
//    dis=sqrt(dis);
//    return dis;
//}

//double FeatureExtractor::pointdis(double x1, double y1, double z1, double x2, double y2, double z2)
//{
//    double dis;
//    dis=pow(x1-x2,2)+pow(y1-y2,2)+pow(z1-z2,2);
//    dis=sqrt(dis);
//    return dis;
//}

//vector<int> FeatureExtractor::getNeighbors(int pointIndex)
//{
//    vector<int> neiborIndex;
//    int nbIdx[8]={pointIndex-WIDTH-1,pointIndex-WIDTH,pointIndex-WIDTH+1,pointIndex-1,pointIndex+1,
//                  pointIndex+WIDTH-1,pointIndex+WIDTH,pointIndex+WIDTH+1};
//    for(int i=0;i<8;i++)
//    {
//        if(nbIdx[i]>-1 && nbIdx[i]<cloud->points.size())
//        {neiborIndex.push_back(nbIdx[i]);}
//    }
//    return neiborIndex;

//}

//int FeatureExtractor::getMainlabel(nodePatch np)
//{

//    vector<int> labelnode;
//    vector<int> labelcnt;
//    pcl::PointIndices &pi = np.points;
//    int unknowncnt=0;
//    for(int j=0;j<pi.indices.size();j++)
//    {
//        int index=pi.indices[j];
//        if(cloud->points[index].label == 0)
//        {
//            unknowncnt++;
//            continue;
//        }
//        int flag=0;
//        for(int m=0;m<labelnode.size();m++)
//        {
//            if(cloud->points[index].label == labelnode[m])
//            {
//                labelcnt[m]++;
//                flag=1;
//                break;
//            }
//        }
//        if(flag==0)
//        {
//            labelcnt.push_back(1);
//            labelnode.push_back(cloud->points[index].label);
//        }
//    }
//    double unknownpercentage=double(unknowncnt)/double(pi.indices.size());
//    if(unknownpercentage>0.9)
//        return 0;
//    if(labelnode.size()!= labelcnt.size())
//    {
//        cout<<"error counting!"<<endl;
//    }
//    int majorlabel;
//    if(labelcnt.size()==0)
//    {
//        majorlabel=0;
//    }
//    else if(labelcnt.size()==1)
//        majorlabel=labelnode[0];
//    else
//    {
//        int labelmax=0;
//        for(int m=1;m<labelnode.size();m++)
//        {
//            if(labelcnt[m]>labelcnt[labelmax])
//                labelmax=m;
//        }
//        majorlabel=labelnode[labelmax];
//    }
//    return majorlabel;

//}




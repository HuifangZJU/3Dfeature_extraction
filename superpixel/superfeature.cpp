#include "superfeature.h"
#include <iostream>
#include <math.h>
#include <pcl/features/pfh.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/pca.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_types_conversion.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>

#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/octree/impl/octree_base.hpp>
#include <vtkPolyLine.h>
#include <pcl/conversions.h>
#include <ctime>


using namespace pcl;
using namespace std;
#define PI 3.1415926

superfeature::superfeature()
{
}
superfeature::~superfeature()
{
}
void superfeature::setInputCloud(PointCloud<PointT>::Ptr cloud_in,PointCloud<PointXYZRGBA>::Ptr cloud_)
{
    cloud=cloud_in;
    cloud_rgba=cloud_;

}
PointCloud<PointT>::Ptr superfeature::getInputCloud(void)
{
    return cloud;
}

std::vector<superfeature::nodePatch> superfeature::getNodes(void)
{
    return Nodes;
}
std::vector<superfeature::edgeLine> superfeature::getEdges(void)
{
    return Edges;
}
std::vector<float> superfeature::getLimit(void)
{

   return limits;
}

void superfeature::initialize()
{
    segments.clear();seg_indices.clear();
    Nodes.clear(); Edges.clear();labels.clear();
    WIDTH = cloud->width;
    HEIGHT = cloud->height;

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

        if(cloud->points[i].segment==0){continue;}
        // extract the size of the whole image
        if(cloud->points[i].x>x_max)
            x_max=cloud->points[i].x;
        if(cloud->points[i].x<x_min)
            x_min=cloud->points[i].x;
        if(cloud->points[i].y>y_max)
            y_max=cloud->points[i].y;
        if(cloud->points[i].y<y_min)
            y_min=cloud->points[i].y;
        if(cloud->points[i].z>z_max)
            z_max=cloud->points[i].z;
        if(cloud->points[i].z<z_min)
            z_min=cloud->points[i].z;

    }
    limits.push_back(x_max);
    limits.push_back(x_min);
    limits.push_back(y_max);
    limits.push_back(y_min);
    limits.push_back(z_max);
    limits.push_back(z_min);
//    cout<<"1limits:";
//     cout<<x_min<<" "<<x_max<<" "<<y_min<<" "<<y_max<<" "<<z_min<<" "<<z_max<<endl;
    max_dis=pointdis(x_max,y_max,z_max,x_min,y_min,z_min);
    cloud_center(0)=(x_max-x_min)/2;
    cloud_center(1)=(y_max-y_min)/2;
    cloud_center(2)=(z_max-z_min)/2;
   // cout<<"Max_dis:  "<<max_dis<<endl;
}

void superfeature::constructGraph()
{

    //single pcds
    bool use_transform = false;
    //parameters
    float voxel_resolution = 0.008f;
    float seed_resolution = 0.1f;
    float color_importance = 0.2f;
    float spatial_importance = 0.4;
    float normal_importance = 1.0f;

    //extracting the supervoxels
    cout<<"Extracting supervoxels ......";
    pcl::SupervoxelClustering<PointXYZRGBA> super (voxel_resolution, seed_resolution, use_transform);
    super.setInputCloud (cloud_rgba);
    super.setColorImportance (color_importance);
    super.setSpatialImportance (spatial_importance);
    super.setNormalImportance (normal_importance);
    std::map <uint32_t, pcl::Supervoxel<PointXYZRGBA>::Ptr > supervoxel_clusters;
    super.extract (supervoxel_clusters);
    cout<<supervoxel_clusters.size()<<endl;
    std::map <uint32_t, pcl::Supervoxel<PointXYZRGBA>::Ptr >::iterator ite;
    std::map <int,int> seg2node;
    std::map <int,int> isnode;
    int id=1;    
    for(ite=supervoxel_clusters.begin();ite!=supervoxel_clusters.end();ite++)
    {
      pcl::Supervoxel<PointXYZRGBA>::Ptr supervoxel = ite->second;
      if(!__finite(supervoxel->normal_.normal_x))
      {
          isnode[ite->first]=0;
          continue;
      }
      nodePatch np;
      np.node_id = id;
      np.segment = ite->first;
      segments.push_back(ite->first);
      seg2node[ite->first] = id;
      isnode[ite->first]=1;
      np.center<<supervoxel->centroid_.x,supervoxel->centroid_.y,supervoxel->centroid_.z;
      //cout<<supervoxel->centroid_.x<<"   "<<supervoxel->centroid_.y<<"   "<<supervoxel->centroid_.z<<endl;
      np.r=supervoxel->centroid_.r;np.g=supervoxel->centroid_.g;np.b=supervoxel->centroid_.b;

      np.height = np.center(2);
      np.normal<<supervoxel->normal_.normal_x,supervoxel->normal_.normal_y,supervoxel->normal_.normal_z;
      //cout<<supervoxel->normal_.normal_x<<"   "<<supervoxel->normal_.normal_y<<"   "<<supervoxel->normal_.normal_z<<endl;
      Nodes.push_back(np);
      id++;
    }
     cout<<"Nodes number "<<Nodes.size()<<endl;

    //Construct graphs

     cout<<"Constructing gragh ......";

    std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
    super.getSupervoxelAdjacency (supervoxel_adjacency);
    cout<<"superadjacency size is : "<<supervoxel_adjacency.size()<<endl;

    std::multimap<uint32_t,uint32_t>::iterator label_itr = supervoxel_adjacency.begin ();
    for ( ; label_itr != supervoxel_adjacency.end (); )
    {

      //Get the segment
      uint32_t supervoxel_label = label_itr->first;
      if(isnode.at(supervoxel_label)==0)
      {
          label_itr = supervoxel_adjacency.upper_bound (supervoxel_label);
          continue;
      }
      int nodeid = seg2node.at(supervoxel_label);
       nodePatch &np=Nodes[nodeid-1];
      //Get the neighbor pixels
      std::multimap<uint32_t,uint32_t>::iterator adjacent_itr = supervoxel_adjacency.equal_range (supervoxel_label).first;
//      cout<<"adding "<<nodeid<<" neighbors...";
      for ( ; adjacent_itr!=supervoxel_adjacency.equal_range (supervoxel_label).second; ++adjacent_itr)
      {
         if(isnode.at(adjacent_itr->second)==0) continue;
        int neighborid = seg2node.at(adjacent_itr->second);
        np.neighbors.push_back(neighborid);
//        cout<<"--"<<adjacent_itr->second<<"  ";
      }
//      cout<<endl;
      label_itr = supervoxel_adjacency.upper_bound (supervoxel_label);
    }
    int sum=0;
    cout<<"Nodes number: "<<Nodes.size()<<endl;
    for(int i=0;i<Nodes.size();i++)
    {
        nodePatch &np=Nodes[i];
        sum=sum+np.neighbors.size();
    }
    cout<<"neighbors : "<<sum<<endl;
    buildEdges();

    cout<<"Resetting labels......";
    PointCloud<PointXYZL>::Ptr supercloud = super.getLabeledCloud();
    resetCloudLabels(supercloud,seg2node,isnode);
    cout<<"Done"<<endl;

}

void superfeature::resetCloudLabels(PointCloud<PointXYZL>::Ptr supercloud,std::map <int,int> seg2node,
                                    std::map <int,int> isnode)
{

  if(cloud->points.size() != supercloud->points.size())
      cout<<"Error : Clustered point cloud has different number of points"<<endl;
  for(int i=0;i<cloud->points.size();i++)
  {
      cloud->points[i].segment = supercloud->points[i].label;
      if (supercloud->points[i].label==0)
      {
          cloud->points[i].label=0;
          continue;
      }
      if(isnode.at(supercloud->points[i].label)==0)
          continue;
//      cout<<"1....";
      int nodeid = seg2node.at(supercloud->points[i].label);
//      cout<<"2...."<<endl;
      nodePatch &np =Nodes[nodeid-1];
      np.points.indices.push_back(i);
  }
  int invalid_sum=0;
  for(int i=0;i<Nodes.size();i++)
  {
      std::vector<int> labelnode;
      std::vector<int> labelcnt;

      nodePatch &np = Nodes[i];
      pcl::PointIndices &pi = np.points;
      if (pi.indices.size()==0)
      {
          cout<<"....error :" <<i<<";  "<<endl;
          cout<<np.center<<"   "<<np.normal<<endl;          
      }
      for(int j=0;j<pi.indices.size();j++)
      {

          int index=pi.indices[j];
          if(cloud->points[index].label == 0)
              continue;
//          if (cloud->points[index].label !=0 && cloud->points[index].label != 1)
//          cout<<index<<" : "<<cloud->points[index].label<<"        ";
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
//      for(int j=0;j<labelcnt.size();j++)
//      {
//          cout<<"label "<<labelnode[j]<<" cnt "<<labelcnt[j];
//      }

      if(labelnode.size()!= labelcnt.size())
      {
          cout<<"error counting!"<<endl;
      }
      int majorlabel;

      if(labelcnt.size()==0)
      {
          majorlabel=0;
          invalid_sum++;
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
      np.label = majorlabel;
      labels.push_back(majorlabel);

      //cout<<"node "<<i<<" :  segment "<<segments[i]<<"  label "<<labels[i]<<endl;
      //cout<<endl;
      //cout<<"segment "<<i<<" is "<<majorlabel<<endl;
      for(int j=0;j<pi.indices.size();j++)
      {
          int index=pi.indices[j];
          cloud->points[index].label= majorlabel;
      }

  }
  cout<<"Invalid supervoxel number is : "<<invalid_sum<<endl;

}


void superfeature::extractNodefeatures()
{
    cout<<"Extracting node features ......"<<endl;
    for(int i=0;i<Nodes.size();i++)
    {
       // cout<<i<<"th nodes : "<<endl;
        extractNodecolor(i);
    }
    cout<<"dis2bd"<<endl;
    //normalize the distance to boundary
    float dis2xmin_min=0,dis2xmin_max=max_dis;
    float dis2ymin_min=0,dis2ymin_max=max_dis;
    float dis2xmax_min=0,dis2xmax_max=max_dis;
    float dis2ymax_min=0,dis2ymax_max=max_dis;
    for(int i=0;i<Nodes.size();i++)
    {
        nodePatch &np=Nodes[i];
        np.dis2xmin=abs(np.center(0)-x_min);
        np.dis2xmax=abs(np.center(0)-x_max);
        np.dis2ymin=abs(np.center(1)-y_min);
        np.dis2ymax=abs(np.center(1)-y_max);
        if(np.dis2xmin>dis2xmin_max)  dis2xmin_max=np.dis2xmin;
        if(np.dis2xmin<dis2xmin_min)  dis2xmin_min=np.dis2xmin;

        if(np.dis2xmax>dis2xmax_max)  dis2xmax_max=np.dis2xmax;
        if(np.dis2xmax<dis2xmax_min)  dis2xmax_min=np.dis2xmax;

        if(np.dis2ymin>dis2ymin_max)  dis2ymin_max=np.dis2ymin;
        if(np.dis2ymin<dis2ymin_min)  dis2ymin_min=np.dis2ymin;

        if(np.dis2ymax>dis2ymax_max)  dis2ymax_max=np.dis2ymax;
        if(np.dis2ymax<dis2ymax_min)  dis2ymax_min=np.dis2ymax;
    }
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
    cout<<"Extracting node features ......Done"<<endl;

}

void superfeature::extractNodecolor(int id)
{

    /*******************Extract mean color********************/
    PointCloud<PointXYZRGB> cloud_node;
    PointCloud<PointXYZHSV> cloud_hsv;
    nodePatch &np = Nodes[id];
    PointIndices &p = np.points;
//    if(p.indices.size()==0)
//    {
//        if(np.r>np.g && np.r>np.b)
//        {
//            float min=np.g;
//            if(np.g>np.b) min=np.b;
//            np.hsv(0) = (np.g-np.b)/(np.r-min);
//            np.hsv(1) = (np.r-min)/np.r;
//            np.hsv(2)=np.r;
//        }
//        if(np.g>np.r && np.g>np.b)
//        {
//            float min=np.r;
//            if(np.r>np.b) min=np.b;
//            np.hsv(0) = 2+(np.b-np.r)/(np.g-min);
//            np.hsv(1) = (np.g-min)/np.g;
//            np.hsv(2)=np.g;
//        }
//        if(np.b>np.r && np.b>np.g)
//        {
//            float min=np.r;
//            if(np.r>np.g) min=np.g;
//            np.hsv(0) = 4+(np.r-np.g)/(np.b-min);
//            np.hsv(1) = (np.b-min)/np.b;
//            np.hsv(2)=np.b;
//        }
//        np.hsv(0)=60*np.hsv(0);
//        if(np.hsv(0)<0)
//        {
//            np.hsv(0)=np.hsv(0)+360;
//        }
//        return;
//    }
    //cout<<p.indices.size()<<endl;
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
    Nodes[id].hsv(0)=h/p.indices.size();
    Nodes[id].hsv(1)=s/p.indices.size();
    Nodes[id].hsv(2)=v/p.indices.size();
    Nodes[id].r=r/p.indices.size();
    Nodes[id].g=g/p.indices.size();
    Nodes[id].b=b/p.indices.size();
}

void superfeature::buildEdges(void)
{
    int id=1;
    for(int i=0;i<Nodes.size();i++)
    {
        nodePatch &np = Nodes[i];
//        cout<<np.node_id<<" : ";
        for(int j=0;j<np.neighbors.size();j++)
        {           
            nodePatch &np2 = Nodes[np.neighbors[j]-1];
//            cout<<np2.node_id<<"  ";
            edgeLine el;
            el.edge_id=id;
            el.start=np.node_id;
            el.end=np2.node_id;
            Edges.push_back(el);
            id++;
//            el.edge_id=id;
//            el.start=np2.node_id;
//            el.end=np.node_id;
//            Edges.push_back(el);
//            id++;
        }
//        cout<<endl;

    }
    //cout<<"Neighborsum: "<<neighborsum<<endl;
    cout<<"Edge number : "<<Edges.size()/2<<endl;

}

void superfeature::extractEdgefeatures()
{
    cout<<"Extracting node features ......"<<endl;
    for(int i=0;i<Edges.size();i++)
    {
        extractEdgevisualfeatures(i);
        extractEdgegeometryfeatures(i);
    }
    cout<<"Extracting node features ......Done"<<endl;
}

void superfeature::extractEdgevisualfeatures(int id)
{
    edgeLine & el = Edges[id];    
    nodePatch & node_s = Nodes[el.start-1];
    nodePatch & node_e = Nodes[el.end-1];
    el.dis_hsv=node_s.hsv-node_e.hsv;
    el.dis_hsv(0)=abs(el.dis_hsv(0));
    el.dis_hsv(1)=abs(el.dis_hsv(1));
    el.dis_hsv(2)=abs(el.dis_hsv(2));    
}

void superfeature::extractEdgegeometryfeatures(int id)
{
    edgeLine & el = Edges[id];   
    nodePatch & node_s = Nodes[el.start-1];
    nodePatch & node_e = Nodes[el.end-1];
     /****************Angle between normals**********************/

      Eigen::Vector3f &normalstart=node_s.normal;
      Eigen::Vector3f &normalend = node_e.normal;

      el.angle_bt_nm=normalstart(0)*normalend(0)+normalstart(1)*normalend(1)+normalstart(2)*normalend(2);
      if(el.angle_bt_nm<0)
      {
          el.angle_bt_nm=-el.angle_bt_nm;
      }


}

double superfeature::pointdis(int p1,int p2)
{
    double dis;
    //if(__finite(cloud->points[p1].x) && __finite(cloud->points[p2].x))
    dis=pow(cloud->points[p1].x-cloud->points[p2].x,2)+pow(cloud->points[p1].y-cloud->points[p2].y,2)
            +pow(cloud->points[p1].z-cloud->points[p2].z,2);
    dis=sqrt(dis);
    return dis;
}

double superfeature::pointdis(double x1, double y1, double z1, double x2, double y2, double z2)
{
    double dis;
    dis=pow(x1-x2,2)+pow(y1-y2,2)+pow(z1-z2,2);
    dis=sqrt(dis);
    return dis;
}

vector<int> superfeature::getNeighbors(int pointIndex)
{
    vector<int> neiborIndex;
    int nbIdx[8]={pointIndex-WIDTH-1,pointIndex-WIDTH,pointIndex-WIDTH+1,pointIndex-1,pointIndex+1,
                  pointIndex+WIDTH-1,pointIndex+WIDTH,pointIndex+WIDTH+1};
    for(int i=0;i<8;i++)
    {
        if(nbIdx[i]>-1 && nbIdx[i]<cloud->points.size())
        {neiborIndex.push_back(nbIdx[i]);}
    }
    return neiborIndex;

}


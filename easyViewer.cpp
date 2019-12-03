//#define PCL_NO_PRECOMPILE
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/impl/pcl_visualizer.hpp>
#include <pcl/console/parse.h>
#include "easyViewer.h"
#include "point_types_hf.h"
#include "featureextractor.h"




using namespace pcl;
using namespace std;

EasyViewer::EasyViewer()
{

}

EasyViewer::~EasyViewer()
{

}
void EasyViewer::setInput(PointCloud<PointT>::Ptr cloud_in,std::vector<FeatureExtractor::nodePatch> Nodes_in,std::vector<FeatureExtractor::edgeLine> Edges_in)
{
    cloud=cloud_in;
    Nodes=Nodes_in;
    Edges=Edges_in;

}

void EasyViewer::setViewer(boost::shared_ptr<visualization::PCLVisualizer> viewer_in,bool keyboard,string kind)
{
    viewer=viewer_in;

}
void EasyViewer::setLabelColors(int* labels,int label_number)
{

    for(int i=0;i<label_number;i++)
    {
        int r=rand()%255,g=rand()%255,b=rand()%255;
        Eigen::Vector3i temp;
        temp<<r,g,b;
        interested_colors.push_back(temp);
    }
    interested_labels=labels;
//    cout<<"color number :"<<interested_colors.size()<<endl;

}

boost::shared_ptr<pcl::visualization::PCLVisualizer> EasyViewer::getViewer()
{
    return viewer;
}
void EasyViewer::drawCloud(int kind)
{

    /*****************Show Simplified point cloud**************/
       if(kind == 1)//draw original cloud
       {
           cout<<"original cloud......"<<endl;
           visualization::PointCloudColorHandlerRGBField<PointT> rgb2(cloud);
           viewer->addPointCloud<PointT> (cloud, rgb2, "cloud");
           viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE,1,"cloud");
       }
       if(kind == 2)//draw simplified cloud
       {
           cout<<"simplified cloud......"<<endl;
           PointCloud<PointT>::Ptr simplifiedpointcloud(new PointCloud<PointT>);
           for(int i=0;i<Nodes.size();i++)
           {
               FeatureExtractor::nodePatch &np=Nodes[i];
               vector<PointVectorPair> pp=np.simplified_points;

               for (int j=0;j<pp.size();j++)
               {
                    PointT p;
                    PointVectorPair pptemp=pp[j];
                    p.x=pptemp.first.x();
                    p.y=pptemp.first.y();
                    p.z=pptemp.first.z();
                    p.r=np.r;p.g=np.g;p.b=np.b;
                    simplifiedpointcloud->points.push_back(p);
               }
           }
           visualization::PointCloudColorHandlerRGBField<PointT> rgb(simplifiedpointcloud);
           viewer->addPointCloud<PointT> (simplifiedpointcloud, rgb, "simplified cloud");
           viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE,2,"simplified cloud");
       }
       if(kind == 3)//pixel-wise ground truth
       {
           cout<<"ground truth cloud......"<<endl;
           for(int i=0;i<cloud->points.size();i++)
           {
               int r,g,b;
               int templabel=cloud->points[i].label;
               if(templabel==0)
               {
                   cloud->points[i].r=0;
                   cloud->points[i].g=0;
                   cloud->points[i].b=0;
                   continue;
               }
               r=interested_colors[templabel-1](0);
               g=interested_colors[templabel-1](1);
               b=interested_colors[templabel-1](2);
               cloud->points[i].r=r;
               cloud->points[i].g=g;
               cloud->points[i].b=b;
           }
           visualization::PointCloudColorHandlerRGBField<PointT> rgb3(cloud);
           viewer->addPointCloud<PointT> (cloud, rgb3, "cloud");
           viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE,1,"cloud");
       }

}


void EasyViewer::drawNodes(int color,string filepath,int fileid)
{
    for(int i=0;i<cloud->points.size();i++)
    {
       cloud->points[i].a=0;
       cloud->points[i].r=0;
       cloud->points[i].g=0;
       cloud->points[i].b=0;
    }
    switch (color) {
    case 2://random color
        for(int i=0;i<Nodes.size();i++)
        {
            int r=rand()%255,g=rand()%255,b=rand()%255;
            PointIndices & p=Nodes[i].points;
            for(int j=0;j<p.indices.size();j++)
            {

                cloud->points[p.indices[j]].r=r;
                cloud->points[p.indices[j]].g=g;
                cloud->points[p.indices[j]].b=b;
            }
        }
        break;
    case 3://show ground truth
        for(int i=0;i<Nodes.size();i++)
        {
            FeatureExtractor::nodePatch &np=Nodes[i];
            PointIndices p=np.points;
            int r=0,g=0,b=0;
            int gt=interested_colors.size();
            for(int j=0;j<interested_colors.size();j++)
            {
                if(np.label==interested_labels[j])
                {gt=j;break;}
            }
            if(gt!=interested_colors.size())
            {
                r=interested_colors[gt](0);
                g=interested_colors[gt](1);
                b=interested_colors[gt](2);
            }
            for(int j=0;j<p.indices.size();j++)
            {

                cloud->points[p.indices[j]].r=r;
                cloud->points[p.indices[j]].g=g;
                cloud->points[p.indices[j]].b=b;
            }
        }
        break;
    case 4://from file
    {
        stringstream sspath;
        sspath<<filepath<<fileid<<".txt";
        string stpath=sspath.str();

        ifstream fs;
        string stresult;
        fs.open(stpath.c_str());
        if(!fs)
            cout<<"file open error!! File name is :"<<endl<<stpath<<endl;
        getline(fs,stresult);
        stringstream labels;
        labels<<stresult;
        vector<int> predict;
        for(int i=0;i<10000;i++)
        {
            string temp;
            labels>>temp;
            int a=atoi(temp.c_str());
            if(a==0)
                break;
            predict.push_back(a);
        }
        cout<<"predict number : "<<predict.size()<<endl;
        for(int i=0;i<Nodes.size();i++)
        {
            FeatureExtractor::nodePatch &np=Nodes[i];
            PointIndices p=np.points;
            int r,g,b;
            if(predict[i]>interested_colors.size())
                continue;
            r=interested_colors[predict[i]-1](0);
            g=interested_colors[predict[i]-1](1);
            b=interested_colors[predict[i]-1](2);
            for(int j=0;j<p.indices.size();j++)
            {

                cloud->points[p.indices[j]].r=r;
                cloud->points[p.indices[j]].g=g;
                cloud->points[p.indices[j]].b=b;
            }
        }
        break;
    }
    default:
        for(int i=0;i<Nodes.size();i++)
        {
            int r=Nodes[i].r;
            int g=Nodes[i].g;
            int b=Nodes[i].b;
            PointIndices & p=Nodes[i].points;
            for(int j=0;j<p.indices.size();j++)
            {

                cloud->points[p.indices[j]].r=r;
                cloud->points[p.indices[j]].g=g;
                cloud->points[p.indices[j]].b=b;
            }
        }
        break;
    }
    visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
    viewer->addPointCloud<PointT> (cloud, rgb, "node");
    viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE,1,"node");

}
void EasyViewer::drawEdges()
{
    /*****************Draw the edges***************/
    for(int i=0;i<Edges.size();i=i+1)
    {
        int node_start=Edges[i].start;
        int node_end=Edges[i].end;
       // cout<<i<<" : "<<node_start<<"  "<<node_end<<endl;
        PointXYZ p1,p2;
        p1.x=Nodes[node_start-1].center(0);
        p1.y=Nodes[node_start-1].center(1);
        p1.z=Nodes[node_start-1].center(2);
        p2.x=Nodes[node_end-1].center(0);
        p2.y=Nodes[node_end-1].center(1);
        p2.z=Nodes[node_end-1].center(2);
        stringstream tx;
        tx<<"edge"<<i;
        string tex=tx.str();
        viewer->addLine (p1, p2,1,0,0,tex);
        viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,2, tex);
    }
}
void EasyViewer::drawNodeCenters()
{
    /*******************Draw the node centers***************/
    PointCloud<PointXYZRGB>::Ptr centerpoints_pr (new PointCloud<PointXYZRGB>);
    PointCloud<PointXYZRGB> & centerpoints=*centerpoints_pr;
    PointXYZRGB point;
    point.x=0;point.y=0;point.z=0;
    centerpoints.push_back(point);
    for(int i=0;i<Nodes.size();i++)
    {
        point.x=Nodes[i].center(0);
        point.y=Nodes[i].center(1);
        point.z=Nodes[i].center(2);
        point.r=255;point.g=0;point.b=0;
        centerpoints.push_back(point);
      }
    visualization::PointCloudColorHandlerRGBField<PointXYZRGB> rgb_center(centerpoints_pr);
    viewer->addPointCloud<PointXYZRGB>(centerpoints_pr,rgb_center,"centers");
    viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE,8,"centers");
}
void EasyViewer::drawNodeConvexHull()
{
    /*******************Draw the node contour***************/
        PointCloud<PointXYZ>::Ptr convex_hull_pr (new PointCloud<PointXYZ>);
        PointCloud<PointXYZ> & convex_hull=*convex_hull_pr;
        for(int i=0;i<Nodes.size();i++)
        {
            PointCloud<PointXYZ> & temp=Nodes[i].convex_hull;
            for(int j=0;j<temp.points.size();j++)
            {
                int j1=j%temp.points.size();
                int j2=(j+1)%temp.points.size();
                PointXYZ p1,p2;
                p1=temp.points[j1];p2=temp.points[j2];
               convex_hull.points.push_back(p1);
               stringstream ss;
               ss<<"convex_hull#"<<i<<"#"<<j;
               string te = ss.str();
               viewer->addLine (p1, p2,0,1,0,te);
            }
        }
        viewer->addPointCloud<PointXYZ> (convex_hull_pr, "convex_hull");
        viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE,3,"convex_hull");
        viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_COLOR,0,1,0,"convex_hull");
}

void EasyViewer::drawNodeNormals()
{
    /*****************Draw the normals**************/
        for(int i=0;i<Nodes.size();i++)
        {
            PointXYZ p1,p2;
            p1.x=Nodes[i].center(0);
            p1.y=Nodes[i].center(1);
            p1.z=Nodes[i].center(2);
            p2.x=p1.x+Nodes[i].normal(0)/2;
            p2.y=p1.y+Nodes[i].normal(1)/2;
            p2.z=p1.z+Nodes[i].normal(2)/2;          
            stringstream tx;
            tx<<"arrow"<<i;
            string tex=tx.str();
            viewer->addArrow(p2,p1,1,0,0,false,tex);
        }
}
void EasyViewer::drawCloudBDbox(double x_max,double x_min,double y_max,double y_min,double z_max,double z_min)
{
    drawBDbox(x_max,x_min,y_max,y_min,z_max,z_min,0);
//    PointXYZ p1,p2;
//    stringstream ss;
//    ss<<"boundingbox";
//    string te = ss.str();
//    string tex;
//       /************z************/
//    p1.x=x_min;p1.y=y_min;p1.z=z_min;
//    p2.x=x_min;p2.y=y_min;p2.z=z_max;
//    tex=te+"a";
//    viewer->addLine (p1, p2,1,0,0,tex);
//    viewer->setShapeRenderingProperties(visualization::PCL_VISUALIZER_LINE_WIDTH,3,tex);

//    p1.x=x_max;p1.y=y_min;p1.z=z_min;
//    p2.x=x_max;p2.y=y_min;p2.z=z_max;
//    tex=te+"b";
//    viewer->addLine (p1, p2,1,0,0,tex);

//    p1.x=x_min;p1.y=y_max;p1.z=z_min;
//    p2.x=x_min;p2.y=y_max;p2.z=z_max;
//    tex=te+"c";
//    viewer->addLine (p1, p2,1,0,0,tex);

//    p1.x=x_max;p1.y=y_max;p1.z=z_min;
//    p2.x=x_max;p2.y=y_max;p2.z=z_max;
//    tex=te+"d";
//    viewer->addLine (p1, p2,1,0,0,tex);

//    /***********y************/
//    p1.x=x_min;p1.y=y_min;p1.z=z_min;
//    p2.x=x_min;p2.y=y_max;p2.z=z_min;
//    tex=te+"e";
//    viewer->addLine (p1, p2,0,1,0,tex);
//    viewer->setShapeRenderingProperties(visualization::PCL_VISUALIZER_LINE_WIDTH,3,tex);

//    p1.x=x_max;p1.y=y_min;p1.z=z_min;
//    p2.x=x_max;p2.y=y_max;p2.z=z_min;
//    tex=te+"f";
//    viewer->addLine (p1, p2,0,1,0,tex);

//    p1.x=x_min;p1.y=y_min;p1.z=z_max;
//    p2.x=x_min;p2.y=y_max;p2.z=z_max;
//    tex=te+"g";
//    viewer->addLine (p1, p2,0,1,0,tex);

//    p1.x=x_max;p1.y=y_min;p1.z=z_max;
//    p2.x=x_max;p2.y=y_max;p2.z=z_max;
//    tex=te+"h";
//    viewer->addLine (p1, p2,0,1,0,tex);

//    /************x**********/
//    p1.x=x_min;p1.y=y_min;p1.z=z_min;
//    p2.x=x_max;p2.y=y_min;p2.z=z_min;
//    tex=te+"i";
//    viewer->addLine (p1, p2,0,0,1,tex);
//    viewer->setShapeRenderingProperties(visualization::PCL_VISUALIZER_LINE_WIDTH,3,tex);

//    p1.x=x_min;p1.y=y_max;p1.z=z_min;
//    p2.x=x_max;p2.y=y_max;p2.z=z_min;
//    tex=te+"j";
//    viewer->addLine (p1, p2,0,0,1,tex);

//    p1.x=x_min;p1.y=y_min;p1.z=z_max;
//    p2.x=x_max;p2.y=y_min;p2.z=z_max;
//    tex=te+"k";
//    viewer->addLine (p1, p2,0,0,1,tex);

//    p1.x=x_min;p1.y=y_max;p1.z=z_max;
//    p2.x=x_max;p2.y=y_max;p2.z=z_max;
//    tex=te+"l";
//    viewer->addLine (p1, p2,0,0,1,tex);
}

void EasyViewer::drawBDbox(double x_max, double x_min, double y_max, double y_min, double z_max, double z_min,int index)
{
    PointXYZ p1,p2;
    stringstream ss;
    ss<<"#"<<index<<"boundingbox";
    string te = ss.str();
    string tex;
       /************z************/
    p1.x=x_min;p1.y=y_min;p1.z=z_min;
    p2.x=x_min;p2.y=y_min;p2.z=z_max;
    tex=te+"a";
    viewer->addLine (p1, p2,1,0,0,tex);
    viewer->setShapeRenderingProperties(visualization::PCL_VISUALIZER_LINE_WIDTH,3,tex);

    p1.x=x_max;p1.y=y_min;p1.z=z_min;
    p2.x=x_max;p2.y=y_min;p2.z=z_max;
    tex=te+"b";
    viewer->addLine (p1, p2,1,0,0,tex);

    p1.x=x_min;p1.y=y_max;p1.z=z_min;
    p2.x=x_min;p2.y=y_max;p2.z=z_max;
    tex=te+"c";
    viewer->addLine (p1, p2,1,0,0,tex);

    p1.x=x_max;p1.y=y_max;p1.z=z_min;
    p2.x=x_max;p2.y=y_max;p2.z=z_max;
    tex=te+"d";
    viewer->addLine (p1, p2,1,0,0,tex);

    /***********y************/
    p1.x=x_min;p1.y=y_min;p1.z=z_min;
    p2.x=x_min;p2.y=y_max;p2.z=z_min;
    tex=te+"e";
    viewer->addLine (p1, p2,0,1,0,tex);
    viewer->setShapeRenderingProperties(visualization::PCL_VISUALIZER_LINE_WIDTH,3,tex);

    p1.x=x_max;p1.y=y_min;p1.z=z_min;
    p2.x=x_max;p2.y=y_max;p2.z=z_min;
    tex=te+"f";
    viewer->addLine (p1, p2,0,1,0,tex);

    p1.x=x_min;p1.y=y_min;p1.z=z_max;
    p2.x=x_min;p2.y=y_max;p2.z=z_max;
    tex=te+"g";
    viewer->addLine (p1, p2,0,1,0,tex);

    p1.x=x_max;p1.y=y_min;p1.z=z_max;
    p2.x=x_max;p2.y=y_max;p2.z=z_max;
    tex=te+"h";
    viewer->addLine (p1, p2,0,1,0,tex);

    /************x**********/
    p1.x=x_min;p1.y=y_min;p1.z=z_min;
    p2.x=x_max;p2.y=y_min;p2.z=z_min;
    tex=te+"i";
    viewer->addLine (p1, p2,0,0,1,tex);
    viewer->setShapeRenderingProperties(visualization::PCL_VISUALIZER_LINE_WIDTH,3,tex);

    p1.x=x_min;p1.y=y_max;p1.z=z_min;
    p2.x=x_max;p2.y=y_max;p2.z=z_min;
    tex=te+"j";
    viewer->addLine (p1, p2,0,0,1,tex);

    p1.x=x_min;p1.y=y_min;p1.z=z_max;
    p2.x=x_max;p2.y=y_min;p2.z=z_max;
    tex=te+"k";
    viewer->addLine (p1, p2,0,0,1,tex);

    p1.x=x_min;p1.y=y_max;p1.z=z_max;
    p2.x=x_max;p2.y=y_max;p2.z=z_max;
    tex=te+"l";
    viewer->addLine (p1, p2,0,0,1,tex);

}

void EasyViewer::drawNodeBDbox(int kind)
{
    if(kind ==2)
    {
        for(int i=0;i<Nodes.size();i++)
        {
//            cout<<i<<" : ";
            FeatureExtractor::nodePatch np;
            np=Nodes[i];
            PointXYZ p1,p2;
            /************z direction************/
            for(int j=4;j<8;j++)
            {
                p1=np.boundingbox[j];
                p2=np.boundingbox[j-4];
                stringstream ss;
                ss<<"boundingbox#"<<i<<"#"<<j;
//                cout<<j<<"   ";
                string tex = ss.str();
                viewer->addLine (p1, p2,1,0,0,tex);
                viewer->setShapeRenderingProperties(visualization::PCL_VISUALIZER_LINE_WIDTH,2,tex);
            }

            /************z up plane***********/
            for(int j=0;j<3;j++)
            {
                p1=np.boundingbox[j];
                p2=np.boundingbox[j+1];
                stringstream ss;
                ss<<"boundingbox#"<<i<<"#"<<j;
//                cout<<j<<"   ";
                string tex = ss.str();
                viewer->addLine (p1, p2,1,0,0,tex);
                viewer->setShapeRenderingProperties(visualization::PCL_VISUALIZER_LINE_WIDTH,2,tex);
            }
            p1=np.boundingbox[3];
            p2=np.boundingbox[0];
            stringstream ss1;
            ss1<<"boundingbox#"<<i<<"#"<<3;
//               cout<<3<<"   ";
            string tex1 = ss1.str();
            viewer->addLine (p1, p2,1,0,0,tex1);
            viewer->setShapeRenderingProperties(visualization::PCL_VISUALIZER_LINE_WIDTH,2,tex1);

            /************z down plane**********/
            for(int j=4;j<7;j++)
            {
                p1=np.boundingbox[j];
                p2=np.boundingbox[j+1];
                stringstream ss;
                ss<<"boundingbox#"<<i<<"#"<<j+4;
//                   cout<<j+4<<"   ";
                string tex = ss.str();
                viewer->addLine (p1, p2,1,0,0,tex);
                viewer->setShapeRenderingProperties(visualization::PCL_VISUALIZER_LINE_WIDTH,3,tex);
            }
            p1=np.boundingbox[7];
            p2=np.boundingbox[4];
            stringstream ss2;
            ss2<<"boundingbox#"<<i<<"#"<<11;
//               cout<<11<<endl;
            string tex2 = ss2.str();
            viewer->addLine (p1, p2,1,0,0,tex2);
            viewer->setShapeRenderingProperties(visualization::PCL_VISUALIZER_LINE_WIDTH,3,tex2);
        }

    }
    else
    {
        for(int i=0;i<Nodes.size();i++)
        {
            FeatureExtractor::nodePatch np;
            np=Nodes[i];
            drawBDbox(np.x_max,np.x_min,np.y_max,np.y_min,np.z_max,np.z_min,i);
        }

    }
}

void EasyViewer::clearall()
{
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();
}

void EasyViewer::showcloud()
{

    viewer->initCameraParameters();
    viewer->setBackgroundColor(0.0,0.0,0.0);
    while (!viewer->wasStopped ())
    {
      viewer->spinOnce (100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

}


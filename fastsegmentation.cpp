#include <iostream>
#include <algorithm>
#include <vector>
#include <math.h>
#include "fastsegmentation.h"
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <pcl/common/pca.h>

using namespace pcl;
using namespace std;

FastSegmentation::FastSegmentation(void)
{
}
FastSegmentation::~FastSegmentation(void)
{
}


void FastSegmentation::setInput(PointCloud<PointT>::Ptr input_cloud, PointCloud<Normal>::Ptr input_normals,
                                     PointIndices p)
{
    cloud = input_cloud;
    normals = input_normals;
    points = p;
    WIDTH_ = input_cloud->width;
    HEIGHT_ = input_cloud->height;

}

vector<PointIndices> FastSegmentation::getclusters(void)
{
    return clusters_;

}


vector<int> FastSegmentation::getNeighbors(int pointIndex)
{
    vector<int> neiborIndex;
    int nbIdx[8]={pointIndex-HEIGHT_-1,pointIndex-HEIGHT_,pointIndex-HEIGHT_+1,pointIndex-1,pointIndex+1,
                  pointIndex+HEIGHT_-1,pointIndex+HEIGHT_,pointIndex+HEIGHT_+1};
    for(int i=0;i<8;i++)
    {
        if(nbIdx[i]>-1 && nbIdx[i]<cloud->points.size() && __finite(cloud->points[nbIdx[i]].x))
        {neiborIndex.push_back(nbIdx[i]);}
    }
    return neiborIndex;

}



void FastSegmentation::initialize()
{
    thDOF=0.7;
//    cout<<"region growing threshold : "<<thDOF<<endl;
    clusters_.clear();
    segFlag_.resize(cloud->points.size(),0);
    inputflag_.resize(cloud->points.size(),0);
    for(int i=0;i<points.indices.size();i++)
    {
        int index = points.indices[i];
        inputflag_[index]=1;
        if(__finite(cloud->points[index].x))
            xyzp.indices.push_back(index);
        else
            rgbp.indices.push_back(index);
    }
}

void FastSegmentation::regionGrowing()
{
    int xyznumber=xyzp.indices.size();
    if(xyznumber<50)
    {
        clusters_.push_back(points);
        return;
    }

//    cout<<"points number : "<<xyznumber<<endl;

    //region growing
    int ite=1;
    for (int i=0; i<xyzp.indices.size();i++)
    {

        int index=xyzp.indices[i];

        //continue if the point has been clustered
        if(segFlag_[index]>0) continue;

        //initialization for a new seed
        segFlag_[index]=ite;
//        cout<<"iteration "<<ite<<" : ";
        int listIdx=0;
        Eigen::Vector3d current_center;
        Eigen::Vector3d current_normal;
        PointIndices current_indices;
        current_center<<cloud->points[index].x,cloud->points[index].y,cloud->points[index].z;
        current_normal<<normals->points[index].normal_x,normals->points[index].normal_y,normals->points[index].normal_z;
        current_indices.indices.push_back(index);
        if(!__finite(current_normal(0)))
        {
            clusters_.push_back(current_indices);
            centers_.push_back(current_center);
            ite++;
        }

        //region growing
        while(listIdx != current_indices.indices.size())
        {
            //8neighbors
            int currentPoint=current_indices.indices[listIdx];
//            cout<<"listIds : "<<listIdx<<" , current point : "<<currentPoint<<endl;
            vector<int> nbIdx=getNeighbors(currentPoint);
            int nbNum = nbIdx.size();
//            for(int n=0;n<nbNum;n++)
//            {
//                cout<<nbIdx[n]<<" ";
//            }
//            cout<<endl;

            double meandis=0;
            vector<double> nbDis;

            //compute meandis
            for(int j=0;j<nbNum;j++)
            {
                double dx=cloud->points[currentPoint].x-cloud->points[nbIdx[j]].x;
                double dy=cloud->points[currentPoint].y-cloud->points[nbIdx[j]].y;
                double dz=cloud->points[currentPoint].z-cloud->points[nbIdx[j]].z;
                nbDis.push_back(sqrt(pow(dx,2)+pow(dy,2)+pow(dz,2)));
                meandis=meandis+nbDis[j];
            }
            meandis=meandis/nbNum;
//            cout<<"meandis : "<<meandis<<", neighbor dis ";
//            for(int n=0;n<nbNum;n++)
//            {
//                cout<<nbDis[n]<<"  ";
//            }
//            cout<<endl;
            for(int j=0;j<nbNum;j++)
            {
                int neighborIdx = nbIdx[j];
                if(inputflag_[neighborIdx]==0)
                    continue;                
                if(!__finite(normals->points[neighborIdx].normal_x))
                    continue;
                if(segFlag_[neighborIdx]>0)
                    continue;
                if(nbDis[j]>meandis*2)
                    continue;
                Eigen::Vector3d nnormal;
                nnormal<<normals->points[neighborIdx].normal_x,normals->points[neighborIdx].normal_y,
                    normals->points[neighborIdx].normal_z;
                double normaldis=current_normal.dot(nnormal);
//                cout<<"normal dis : "<<normaldis<<endl;

                    if( normaldis>thDOF)
                {
                    segFlag_[neighborIdx]=ite;
                    Eigen::Vector3d center;
                    center<<cloud->points[neighborIdx].x,cloud->points[neighborIdx].y,cloud->points[neighborIdx].z;
                    current_center=current_center+center;
                    current_normal=(current_normal*current_indices.indices.size()+nnormal)/(current_indices.indices.size()+1);
                    current_indices.indices.push_back(neighborIdx);
                }

            }
//            cout<<"current_normal : "<<current_normal(0)<<"  "<<current_normal(1)<<"  "<<current_normal(2)<<endl;
            listIdx++;
        }
//        int temp;
//        cout<<"input a number : "<<endl;
//        cin>>temp;
//        cout<<current_indices.indices.size()<<" points."<<endl;
        current_center=current_center/current_indices.indices.size();
        clusters_.push_back(current_indices);
        centers_.push_back(current_center);
        ite++;
    }

    vector<PointIndices>::iterator it;
    for(it=clusters_.begin();it!=clusters_.end();)
    {
        PointIndices & p =*it;
        if(p.indices.size()<50)
        {
            for(int k=0;k<p.indices.size();k++)
            {
                rgbp.indices.push_back(p.indices[k]);
            }
            it=clusters_.erase(it);
        }
        else
        {
            it++;
        }
    }
    if(clusters_.size() < 2)
    {
        clusters_.push_back(points);
        return;
    }

    PointIndices & rgbp_temp = rgbp;
    vector <PointIndices> & clusters_temp = clusters_;
//    cout<<"cluster size after region growing : "<<clusters_.size()<<endl;
    knnclustering(clusters_temp,rgbp_temp);
}



void FastSegmentation::knnclustering(vector<PointIndices> & clusters, PointIndices & rgbp)
{
    //initilize c_flag   
   vector<int> cflag_;
//   cout<<"There are "<<rgbp.indices.size()<<" points have not been clustered "<<endl;
//   cout<<"clusters size "<<clusters.size()<<endl;
   cflag_.resize(cloud->points.size(),-1);
   for(int i=0;i<clusters.size();i++)
   {
       PointIndices & p =clusters[i];
       for(int j=0;j<p.indices.size();j++)
       {
           int index=p.indices[j];
           cflag_[index]=i;
       }
   }
   for(int i=0;i<rgbp.indices.size();i++)
   {
       int index = rgbp.indices[i];
       cflag_[index]=clusters.size();
   }

   int ite=1;

   double th=0.4;

   while(th<1)
   {
       th=th+0.1;

                int clustered = rgbp.indices.size();
                while(clustered!=0)
                {
                   clustered=0;
//                   cout<<"iteration "<<ite++<<" : ";
                   vector<int> & pcolor = rgbp.indices;
                   vector<int>::iterator i_point;
                   for(i_point=pcolor.begin();i_point!=pcolor.end();)
                   {
                       int currentIdx = *i_point;
                       vector<int> nbIdx;
                       vector<int> flagIdx;
                       vector<int> flagcnt;

                       nbIdx = getNeighbors(currentIdx);
                       int rgbneighbor=0;
                       for(int i=0;i<nbIdx.size();i++)
                       {
                           //continue if the neighbor is not belong to this very segment
                           if(cflag_[nbIdx[i]]<0)
                               continue;
                           if( cflag_[nbIdx[i]] == clusters.size() )
                           {

                               rgbneighbor++;
                               continue;
                           }
                           int flag=0;
                           for (int j=0;j<flagIdx.size();j++)
                           {
                               if (cflag_[nbIdx[i]]==flagIdx[j]){flagcnt[j]++;flag=1;break;}
                           }
                           if (flag==1){continue;}
                           else
                           {

                               flagIdx.push_back(cflag_[nbIdx[i]]);
                               flagcnt.push_back(1);
                           }

                       }

                       if(double(rgbneighbor)/double(nbIdx.size())>th)
                       {
                //               cout<<"rgb ratio: "<<double(rgbneighbor)/double(nbIdx.size())<<endl;
                           i_point++;
                           continue;

                       }
                       int maxflag=0,maxcnt=0;
                       for(int i=0;i<flagIdx.size();i++)
                       {
                           if(flagcnt[i]>maxcnt)
                           {
                               maxflag=flagIdx[i];
                               maxcnt=flagcnt[i];
                           }
                       }
                       clusters[maxflag].indices.push_back(currentIdx);
                       cflag_[currentIdx]=maxflag;
                       i_point=pcolor.erase(i_point);
                       clustered++;
                   }
//                   cout<<rgbp.indices.size()<<endl;
                }
   }



}

void FastSegmentation::mindisclustering(void)
{
//    //cout<<"unclassified points: "<<segments_[0].pointslist.size()<<endl;
//    vector<int>::iterator i_point;
//    /*while(segments_[0].pointslist.size()!=0)
//    {*/


//        for(i_point=segments_[0].pointslist.begin();i_point!=segments_[0].pointslist.end();)
//        {
//            double deltax=cloud->points[*i_point].x-segments_[1].centroid[0];
//            double deltay=cloud->points[*i_point].y-segments_[1].centroid[1];
//            double deltaz=cloud->points[*i_point].z-segments_[1].centroid[2];
//            double mindis=pow(deltax,2)+pow(deltay,2)+pow(deltaz,2);
//            int idx=1;
//            for (int i=2;i<segments_.size();i++)
//            {
//                deltax=cloud->points[*i_point].x-segments_[i].centroid[0];
//                deltay=cloud->points[*i_point].y-segments_[i].centroid[1];
//                deltaz=cloud->points[*i_point].z-segments_[i].centroid[2];

//                double dis=pow(deltax,2)+pow(deltay,2)+pow(deltaz,2);

//                if (dis<mindis){mindis=dis;idx=i;}
//            }

//            segments_[idx].pointslist.push_back(*i_point);
//            segments_[0].pointslist.erase(i_point);

//        }
//    //}

//    cout<<"unclassified points: "<<segments_[0].pointslist.size()<<endl;
}



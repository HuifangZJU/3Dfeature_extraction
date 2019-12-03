//cornell data interested_labels ={1,2,3,5,6,7,9,10,11,12,14,16,22,28,34,36,112};
//1:wall   2:floor      3:tableTop      5:chairBackRest     6:cpuFront     7:monitor
//9:paper  10:tableLeg  11:keyboard     12:chairBase        14:tableDrawer 16:chairBack
//22:printerFront   28:book   34:cpuTop    36:cpuside    112:printerSide


#define PCL_NO_PRECOMPILE
#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
//#include"FeatureExtractor.h"
#include "featureextractor.h"
#include <string>
#include <strstream>
#include "point_types_hf.h"
#include "easyViewer.h"
#include "segmentercy.h"
#include <stdarg.h>

using namespace pcl;
using namespace std;
PointCloud<PointT>::Ptr cloud(new PointCloud<PointT>) ;
string file_name,file_path,scene;
EasyViewer eV;
vector<FeatureExtractor::nodePatch> Nodes ;
vector<FeatureExtractor::edgeLine> Edges ;
float x_max,x_min,y_max,y_min,z_max,z_min;
int viewer_label=0;
int interested_labels[37];
int class_number;


void nodekeyboardEventOccurred(const visualization::KeyboardEvent &event, void* viewer_void)
{
    if(Nodes.size()==0)
    {
       cout<<"Viwer didn't get any node!"<<endl;
       return;
    }
    boost::shared_ptr<visualization::PCLVisualizer> viewer =
            *static_cast<boost::shared_ptr<visualization::PCLVisualizer> *> (viewer_void);

    if (event.getKeySym () == "c" && event.keyDown ())
     {
       string pic="node.png";
       viewer->saveScreenshot(pic);
     }
      if (event.getKeySym () == "s" && event.keyDown ())
      {
          if(viewer_label>Nodes.size()-1)
          {
              cout<<"This is the last segment !"<<endl;
              return;
          }
          FeatureExtractor::nodePatch &np =Nodes[viewer_label];
          PointCloud<PointT>::Ptr cloud_tp_pr(new PointCloud<PointT>);
          PointCloud<PointT> & cloud_tp=*cloud_tp_pr;
          PointIndices& tmp2=np.points;
          cout<<"######Node "<<np.node_id<<"#######"<<endl;
          cout<<"Label : "<<np.label<<endl;
          //draw the current node with red color
          for (int i=0;i<tmp2.indices.size();i++)
          {
              cloud_tp.points.push_back(cloud->points[tmp2.indices[i]]);
           }
          for (int i=0;i<cloud_tp.points.size();i++)
          {
              cloud_tp.points[i].r=255;cloud_tp.points[i].g=0;cloud_tp.points[i].b=0;
          }

//          show adjacent segments for current node
//          vector<int> adj=np.neighbors;
//          cout<<adj.size()<<" adjacency : ";
//          for(int i=0;i<adj.size();i++)
//          {
//              double r=rand()%256,g=rand()%256,b=rand()%256;
//              cout<<adj[i]<<" ";
//              PointIndices & tmp=Nodes[adj[i]-1].points;
//              PointT point;
//              for(int j=0;j<tmp.indices.size();j++)
//              {
//                  point.x=cloud->points[tmp.indices[j]].x;
//                  point.y=cloud->points[tmp.indices[j]].y;
//                  point.z=cloud->points[tmp.indices[j]].z;
//                  point.r=r;
//                  point.g=g;
//                  point.b=b;
//                  cloud_tp.points.push_back(point);
//              }
//          }
//          cout<<endl;
          cout<<"Area : "<<np.area<<";  "
                <<"Height : "<<np.height<<";  "
                 <<"Point number : "<<np.points.indices.size()<<endl;

          string ss,sst;
          stringstream s,st;
          s<<"cloud#"<<viewer_label;
          ss=s.str();
          st<<"cloud#"<<viewer_label-1;
          sst=st.str();

          viewer_label++;
          visualization::PointCloudColorHandlerRGBField<PointT> rgb_t(cloud_tp_pr);
          viewer->addPointCloud<PointT> (cloud_tp_pr, rgb_t, ss);
          viewer->removePointCloud(sst);
          viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE,4,ss);
      }
      if (event.getKeySym () == "a" && event.keyDown ())
      {
          if(viewer_label<2)
          {
              cout<<"This is the first segment !"<<endl;
              return;
          }
          viewer_label--;
          stringstream st;
          st<<"cloud#"<<viewer_label;
          string sst;
          sst=st.str();
          viewer->removePointCloud(sst);

          PointCloud<PointT>::Ptr cloud_tp_pr(new PointCloud<PointT>);
          FeatureExtractor::nodePatch &np=Nodes[viewer_label-1];
          PointCloud<PointT> & cloud_tp=*cloud_tp_pr;
          PointIndices& tmp2=np.points;
          cout<<"#####Node "<<np.node_id<<"#####"<<endl;
               cout<<"Label : "<<np.label<<endl;
          //draw the current node with red color
          for (int i=0;i<tmp2.indices.size();i++)
          {
              cloud_tp.points.push_back(cloud->points[tmp2.indices[i]]);
           }
          for (int i=0;i<cloud_tp.points.size();i++)
          {
              cloud_tp.points[i].r=255;cloud_tp.points[i].g=0;cloud_tp.points[i].b=0;
          }
//          show adjacent segments for current node
//          vector<int> adj=np.neighbors;
//          cout<<adj.size()<<" adjacency : ";
//          for(int i=0;i<adj.size();i++)
//          {
//              double r=rand()%256,g=rand()%256,b=rand()%256;
//              cout<<adj[i]<<" ";
//              PointIndices & tmp=Nodes[adj[i]-1].points;
//              PointT point;
//              for(int j=0;j<tmp.indices.size();j++)
//              {
//                  point.x=cloud->points[tmp.indices[j]].x;
//                  point.y=cloud->points[tmp.indices[j]].y;
//                  point.z=cloud->points[tmp.indices[j]].z;
//                  point.r=r;point.g=g;point.b=b;
//                  cloud_tp.points.push_back(point);
//              }
//          }
//          cout<<endl;
          cout<<"Area : "<<np.area<<";  "
                <<"Height : "<<np.height<<";  "
               <<"HSV : "<<np.hsv(0)<<" "<<np.hsv(1)<<" "<<np.hsv(2)<<endl;
          string ss;
          stringstream s;
          s<<"cloud#"<<viewer_label-1;
          ss=s.str();
          visualization::PointCloudColorHandlerRGBField<PointT> rgb_t(cloud_tp_pr);
          viewer->addPointCloud<PointT> (cloud_tp_pr, rgb_t, ss);
          viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE,4,ss);
      }
      if(event.getKeySym () == "d" && event.keyDown ())
      {
           cout<<"Relabel segment!!!"<<endl<<"Input the type number: ";
           int kind;
           cin>>kind;
           std::cout<<"Got it!!!!!!! Next patch!!!!"<<endl;
           FeatureExtractor::nodePatch &np=Nodes[viewer_label-1];
           PointIndices& tmp=np.points;
           np.label=kind;
           for(int i=0;i<tmp.indices.size();i++)
           {
               cloud->points[tmp.indices[i]].label=kind;
           }
           eV.clearall();
           eV.drawNodes(3);


      }
      if(event.getKeySym () == "f" && event.keyDown ())
      {
         exit(1);
      }
}
void edgekeyboardEventOccurred(const visualization::KeyboardEvent &event, void* viewer_void)
{
    if(Edges.size()==0)
    {
       cout<<"Viwer didn't get any edge!"<<endl;
       return;
    }

    boost::shared_ptr<visualization::PCLVisualizer> viewer =
            *static_cast<boost::shared_ptr<visualization::PCLVisualizer> *> (viewer_void);

    if (event.getKeySym () == "c" && event.keyDown ())
     {
       string pic="edge.png";
       viewer->saveScreenshot(pic);
     }
      if (event.getKeySym () == "s" && event.keyDown ())
      {
          if(viewer_label>Edges.size()-2)
          {
              cout<<"This is the last edge !"<<endl;
              return;
          }
          FeatureExtractor::edgeLine &el=Edges[viewer_label];
          int i=el.start-1,j=el.end-1;
          PointCloud<PointT>::Ptr cloud_tp_pr(new PointCloud<PointT>);
          PointCloud<PointT> & cloud_tp=*cloud_tp_pr;
          PointIndices& tmpi=Nodes[i].points;
          PointIndices& tmpj=Nodes[j].points;

          cout<<"#####Edge "<<viewer_label+1<<"#####"<<endl;
          for (int i=0;i<tmpi.indices.size();i++)
          {
              cloud_tp.points.push_back(cloud->points[tmpi.indices[i]]);
           }
          for (int i=0;i<tmpj.indices.size();i++)
          {
              cloud_tp.points.push_back(cloud->points[tmpj.indices[i]]);
           }
          for (int i=0;i<cloud_tp.points.size();i++)
          {
              cloud_tp.points[i].r=255;cloud_tp.points[i].g=0;cloud_tp.points[i].b=0;
          }
          PointXYZ pi,pj;
          pi.x=Nodes[i].center(0);pi.y=Nodes[i].center(1);pi.z=Nodes[i].center(2);
          pj.x=Nodes[j].center(0);pj.y=Nodes[j].center(1);pj.z=Nodes[j].center(2);
          cout<<"Start node label : "<<Nodes[i].label<<"; size : "<<tmpi.indices.size()<<endl<<
                "End node label : "<<Nodes[j].label<<"; size : "<<tmpj.indices.size()<<endl;

          cout<<"HSV dis : "<<el.dis_hsv(0)<<"  "<<el.dis_hsv(1)<<"  "<<el.dis_hsv(2)<<endl;
          cout<<"Angle between normal : "<<el.angle_bt_nm<<";  "<<endl<<endl;
//                        <<"Convexity : "<<el.convexity<<";  "<<endl;

          string ss,sst,ss_line,sst_line;
          stringstream s,st;
          s<<"cloud#"<<viewer_label;
          ss=s.str();
          st<<"cloud#"<<viewer_label-2;
          sst=st.str();
          s<<"line";ss_line=s.str();
          st<<"line";sst_line=st.str();
          viewer->addLine(pi,pj,0,1,0,ss_line);
          viewer->removeShape(sst_line);

          viewer_label=viewer_label+2;
          visualization::PointCloudColorHandlerRGBField<PointT> rgb_t(cloud_tp_pr);
          viewer->addPointCloud<PointT> (cloud_tp_pr, rgb_t, ss);

          viewer->removePointCloud(sst);

          viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE,4,ss);
          viewer->setShapeRenderingProperties(visualization::PCL_VISUALIZER_LINE_WIDTH,3,ss_line);

      }
      if (event.getKeySym () == "a" && event.keyDown ())
      {
          if(viewer_label<3)
          {
              cout<<"This is the first segment !"<<endl;
              return;
          }
          viewer_label=viewer_label-2;
          stringstream st;
          st<<"cloud#"<<viewer_label;
          string sst;
          sst=st.str();
          viewer->removePointCloud(sst);

          PointCloud<PointT>::Ptr cloud_tp_pr(new PointCloud<PointT>);
          PointCloud<PointT> & cloud_tp=*cloud_tp_pr;
          FeatureExtractor::edgeLine &el=Edges[viewer_label-2];
          int i=el.start-1,j=el.end-1;
          PointIndices& tmpi=Nodes[i].points;
          PointIndices& tmpj=Nodes[j].points;

          cout<<"#####Edge "<<viewer_label-1<<"#####"<<endl;
          for (int i=0;i<tmpi.indices.size();i++)
          {
              cloud_tp.points.push_back(cloud->points[tmpi.indices[i]]);
           }
          for (int i=0;i<tmpj.indices.size();i++)
          {
              cloud_tp.points.push_back(cloud->points[tmpj.indices[i]]);
           }
          for (int i=0;i<cloud_tp.points.size();i++)
          {
              cloud_tp.points[i].r=255;cloud_tp.points[i].g=0;cloud_tp.points[i].b=0;
          }
          PointXYZ pi,pj;
          pi.x=Nodes[i].center(0);pi.y=Nodes[i].center(1);pi.z=Nodes[i].center(2);
          pj.x=Nodes[j].center(0);pj.y=Nodes[j].center(1);pj.z=Nodes[j].center(2);

          cout<<"Start node label : "<<Nodes[i].label<<"    End node label : "<<Nodes[j].label<<endl;
           cout<<"HSV dis : "<<el.dis_hsv(0)<<"  "<<el.dis_hsv(1)<<"  "<<el.dis_hsv(2)<<endl;
          cout<<"Angle between normal : "<<el.angle_bt_nm<<";  "<<endl<<endl;

          string ss_line,sst_line;
          string ss;
          stringstream s;
          s<<"cloud#"<<viewer_label-2;
          ss=s.str();

          s<<"line";ss_line=s.str();
          st<<"line";sst_line=st.str();
          viewer->addLine(pi,pj,0,1,0,ss_line);
          viewer->removeShape(sst_line);

          visualization::PointCloudColorHandlerRGBField<PointT> rgb_t(cloud_tp_pr);
          viewer->addPointCloud<PointT> (cloud_tp_pr, rgb_t, ss);
          viewer->setPointCloudRenderingProperties(visualization::PCL_VISUALIZER_POINT_SIZE,4,ss);
          viewer->setShapeRenderingProperties(visualization::PCL_VISUALIZER_LINE_WIDTH,3,ss_line);


      }

}


void savecrffeatures(int fileid)
{

    ofstream myfile;
    stringstream ss;
    ss<<"/media/huifang/Document/SUNfeatures/"<<fileid<<".txt";
    string sstxt=ss.str();
    const char* c_s=sstxt.c_str();
    myfile.open(c_s,ios::trunc);


    for(int i=0;i<Nodes.size();i++)
    {
        FeatureExtractor::nodePatch & np=Nodes[i];
        int flag=0;
        for(int j=0;j<class_number;j++)
        {
            if(np.label == interested_labels[j])
            {
                flag=1;
                break;
            }
        }
//        if(flag==0) np.label=0;
//            continue;
        myfile<<"node "<<np.node_id<<" "<<np.label;
        for(int j=0;j<np.features.size();j++)
        {
            myfile<<" "<<np.features[j];
        }
        myfile<<"\n";
    }

    for(int i=0;i<Edges.size();i++)
    {
        FeatureExtractor::edgeLine & el=Edges[i];
        myfile<<"edge "<<el.edge_id<<" "<<el.start<<" "<<el.end;
        for(int j=0;j<el.features.size();j++)
        {
            myfile<<" "<<el.features[j];
        }
        myfile<<"\n";
    }
    myfile.close();
    cout<<"Saved crf features!"<<endl;
}

void savesvmtestingfeatures(std::vector<FeatureExtractor::nodePatch> nodes,int leave_one_out)
{
//    for(int k=0;k<class_number;k++)
//    {

    ofstream myfile;
    stringstream ss;
    ss<<file_path<<"/"<<leave_one_out<<"_gt.txt";
    string sstxt=ss.str();
    const char* c_s=sstxt.c_str();
    myfile.open(c_s,ios::trunc);
    //myfile<<scene<<"\n";
    //myfile<<"NODEFIELD node_id  label  h  s  v  base_color  patchsize  linearness  planarness  scatter\n";

    for(int i=0;i<nodes.size();i++)
    {
        FeatureExtractor::nodePatch & np=nodes[i];
        int flag=0;
        int labelid=0;
        for(int j=0;j<class_number;j++)
        {
            if(np.label == interested_labels[j])
            {
                flag=1;
                labelid=j+1;
            }
        }
//        if(flag==1)
//            np.label=+1;
//        else
//            np.label=-1;
            if(labelid==1)
                np.label=labelid;
            else
                np.label=-1;
        myfile<<np.label;
        for(int j=0;j<np.features.size();j++)
        {
            myfile<<" "<<j+1<<":"<<np.features[j];
////             myfile<<" "<<np.features[j];
        }
        myfile<<"\n";
    }
    myfile.close();
//    }
    cout<<"Saved test the features!"<<endl;
}

void savesvmtrainingfeatures(std::vector<FeatureExtractor::nodePatch> allnodes[],int leave_one_out)
{
//    for(int k=0;k<class_number;k++)
//    {

    ofstream myfile;
    stringstream ss;
    ss<<file_path<<"/"<<leave_one_out<<"_train.txt";
    string sstxt=ss.str();
    const char* c_s=sstxt.c_str();
    myfile.open(c_s,ios::trunc);
    //myfile<<scene<<"\n";
    //myfile<<"NODEFIELD node_id  label  h  s  v  base_color  patchsize  linearness  planarness  scatter\n";
    for(int n=1;n<25;n++)
    {
        if(leave_one_out == n)
                continue;
        vector<FeatureExtractor::nodePatch> nodes;
        nodes=allnodes[n];
        for(int i=0;i<nodes.size();i++)
        {
            FeatureExtractor::nodePatch & np=nodes[i];
            int flag=0,flag_k=0;
            int labelid=0;
            for(int j=0;j<class_number;j++)
            {
                if(np.label == interested_labels[j])
                {
                    flag=1;
                    labelid=j+1;
                    break;
                }
            }
            if(labelid==1)
                np.label=labelid;
            else
                np.label=-1;

//            if(np.label == interested_labels[k])
//                flag_k=1;
//            if(flag_k==1)
//                np.label=+1;
//            else
//                np.label=-1;
            myfile<<np.label;
            for(int j=0;j<np.features.size();j++)
            {
                myfile<<" "<<j+1<<":"<<np.features[j];
//                myfile<<" "<<np.features[j];
            }
            myfile<<"\n";
        }
    }
    myfile.close();

//    }
}
void showfeatures()
{
    //visualization
    boost::shared_ptr<visualization::PCLVisualizer> viewer (new visualization::PCLVisualizer ("node viewer"));
//    boost::shared_ptr<visualization::PCLVisualizer> viewer2 (new visualization::PCLVisualizer ("edge viewer"));
     boost::shared_ptr<visualization::PCLVisualizer> viewer3 (new visualization::PCLVisualizer ("ground truth"));
    eV.setInput(cloud,Nodes,Edges);
    eV.setLabelColors(interested_labels,class_number);
    eV.setViewer(viewer);
    eV.drawNodes(2);
//    eV.drawNodeNormals();
    eV.drawNodeConvexHull();
    viewer->registerKeyboardCallback(nodekeyboardEventOccurred,(void*)&viewer);


    eV.setViewer(viewer3);
    eV.drawNodes(3,"/home/huifang/huifang/qt_workspace/data/office/stitchedpcd/labelresult/labelresult/crf+pisvm/",16);
//    eV.drawNodeBDbox(2);
    eV.drawEdges();
    eV.setViewer(viewer3);
    eV.drawCloud(3);
        viewer3->registerKeyboardCallback(edgekeyboardEventOccurred,(void*)&viewer3);

    while (!viewer->wasStopped ())
    {
      viewer->spinOnce (100);
//      viewer2->spinOnce (100);
      viewer3->spinOnce(100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}
vector<int> readSceneIndex(int scene)
{
    printf("Reading scene indexes.\n");

    stringstream scenepath;
    scenepath<<"/home/huifang/huifang/SUNRGBDtoolbox/sceneindex/"<<scene<<".txt";
    string scenepath_str=scenepath.str();

    ifstream fs;
    fs.open(scenepath_str.c_str());
    if(!fs)
        cout<<"file open error!!"<<endl;
    string stresult;
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
    cout<<"Totally "<<predict.size()<<" instance in the current scene."<<endl;
    return predict;
}
string readSUNPCD(int fileid)
{
    string pcd_path= "/media/huifang/workspace/datas/SUNPCD/SUNPCD/";
    stringstream sspath;
    sspath<<pcd_path<<fileid<<".pcd";
    return sspath.str();
}

inline bool exists_file (const std::string& name) {
  struct stat buffer;
  return (stat (name.c_str(), &buffer) == 0);
}


int main(int argc, char** argv)
{
    class_number=sizeof(interested_labels)/sizeof(int);
    for(int i=0;i<class_number;i++)
    {
        interested_labels[i]=i+1;
    }

    int startIdx=0;
    int endIdx=10336;
    int startscene=1;
    int endscene=46;
    for(int i=1; i<argc; i++)
    {
        if(strcmp (argv[i], "-id") == 0)
        {
          startIdx = atoi(argv[i+1]);
          if(i+2 < argc)
          endIdx = atoi(argv[i+2]);
        }
        if(strcmp (argv[i], "-scene") == 0)
        {
            startscene = atoi(argv[i+1]);
            if(i+2 < argc)
            endscene = atoi(argv[i+2])+1;
        }
    }
    //SUNRGBD data
//    for (int scene=startscene;scene<endscene;scene++)
//    {

//    vector<int> sceneIndex=readSceneIndex(scene);
//    for(int instance=0;instance<sceneIndex.size();instance++)
//    {
//        int m=sceneIndex[instance];
//        if(m<startIdx || m>endIdx)
//            continue;
    ofstream myfile;
    string sstxt="sun_time_cost.txt";
    const char* c_s=sstxt.c_str();
    myfile.open(c_s,ios::trunc);
    for(int m=1;m<10335;m++)
    {
        stringstream ss;
        ss<<"/features/"<<m<<".txt";
        string sstxt=ss.str();
        if(exists_file (sstxt))
            continue;
        file_name = readSUNPCD(m);
//        stringstream temp;
//        temp<<"/home/huifang/huifang/qt_workspace/data/office/stitchedpcd/scene"<<m<<"_ascii.pcd";
//        file_name = temp.str();
        if(io::loadPCDFile<PointT>(file_name,*cloud) == -1 )
        {
            cout<<"Couldn't read the pcd file "<<m<<endl;
            continue;
        }
        StatisticalOutlierRemoval<PointT> sor;
        sor.setInputCloud(cloud);
        sor.setMeanK(50);
        sor.setStddevMulThresh(2.0);
        sor.filter(*cloud);
        cout<<"/****************File "<<m<<" ****************/"<<endl;
        //segmentation
//        cout<<"segmentation............................ ";
//        clock_t starts, finishs;
//        int times;
//        starts = clock();
//        segmenterCY sc;
//        sc.setInputCloud(cloud);
//        sc.process();
//        finishs = clock();
//        times = (double)(finishs - starts) / CLOCKS_PER_SEC;
//        cout<<times<<" s"<<endl;

        //feature extraction

        clock_t startf, finishf;
        float timef;
        startf = clock();
        FeatureExtractor fe;
        fe.setInputCloud(cloud,true,false);
        fe.initialize();
        vector<float> scale=fe.getLimit();
        cout<<"scale : "<<scale[0]-scale[1]<<"  "
                            <<scale[2]-scale[3]<<"  "
                                <<scale[4]-scale[5]<<endl;
        fe.buildNodes();
        //int flag =fe.extractNodefeatures();
//        if(flag==0)
//            continue;
        fe.buildEdges();
        fe.extractEdgefeatures();
        Nodes=fe.getNodes();
        Edges=fe.getEdges();
        finishf = clock();
        timef = (float)(finishf - startf) / CLOCKS_PER_SEC;
        cout<<"feature extraction cost "<<timef<<" s"<<endl;
        myfile<<m<<" "<<scale[0]-scale[1]<<" "<<scale[2]-scale[3]<<" "<<scale[4]-scale[5]<<" "
                            <<Nodes.size()<<" "<<Edges.size()<<" "<<timef<<"\n";

//        showfeatures();
//        savecrffeatures(m);
//        savesvmtestingfeatures(Nodes,m);
//        Nodesall[m]=Nodes;
//        for(m=startIdx;m<endIdx+1;m++)
//        {
//            savesvmtrainingfeatures(Nodesall,m);
//        }
    }
    myfile.close();
//    }
//    }

     return 0;



}
















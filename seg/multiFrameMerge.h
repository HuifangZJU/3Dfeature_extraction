#ifndef _MULTI_FRAME_MERGE_H_
#define _MULTI_FRAME_MERGE_H_

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/nonfree/features2d.hpp> //feature detect in this file
#include <opencv2/features2d/features2d.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/pcl_base.h>
#include <pcl/search/search.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>

#include "surfaceSegment.h"
#include "objPartition.h"
#include "MyPointRepresentation.h"
template<typename PointT, typename NormalT>
class multiFrameMerge
{
public:
	// Types define of cloud points 
	typedef pcl::PointCloud<PointT> CloudT;
	typedef typename CloudT::Ptr CloudPtr;
	//Types define of normals
	typedef pcl::PointCloud <NormalT> Normal;
	typedef typename Normal::Ptr NormalPtr;
	//Types define of kdtree
	typedef pcl::search::KdTree <PointT> Kdtree;
	typedef typename Kdtree::Ptr KdtreePtr;

public:
	multiFrameMerge();
	~multiFrameMerge();
	/** \brief This function transform the depthMat to PointCloud
	* param[in] depthMat is the mat of depth
	* param[out] cloud is the transform of the mat
	*/
	bool
		getCloudPoints(cv::Mat depthMat, CloudPtr &cloud);
	
	/** \brief This function transform two cloud into one coordinate
	* param[in] sourceCloud is the cloud point in the original coordinate or you want
	* param[in] targetCloud is the cloud point you want to transform to the sourceCloud
	* param[in] transformation is the sourceCloud = tarhetCloud * transformation, but it has error so you need ICP
	*/
	void
		ICPTransform(CloudPtr &sourceCloud, CloudPtr &targetCloud, Eigen::Affine3f &transformation);
	void
		multiObjMerge(std::vector<CloudPtr> &seg_sourceCloud, std::vector<CloudPtr> &seg_targetCloud, std::vector<CloudPtr> &obj_cloud);
	void
        prePartition(CloudPtr &cloud, std::vector<CloudPtr> &seg_cloud, std::vector<cubParam> &cubs);
    void
        prePartition(CloudPtr &cloud, std::vector<CloudPtr> &seg_cloud);
    void
        surfacePartition(CloudPtr &cloud, std::vector<CloudPtr> &obj_cloud, pcl::PointCloud <pcl::PointXYZRGB>::Ptr &colored_cloud_show);
	void
		objColored(std::vector<CloudPtr> &object, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &obj_colored);
private:
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr merg_cloud_RGB_;

protected:
	void
        joinMerge(std::vector<std::vector<int> > &adjacency, std::set<std::set<int> > &circle_relationship);
	
    std::vector<std::vector<int> >
        matrix_buer(std::vector<std::vector<int> > &adjacency);
	int
		and_matrix(std::vector<int> &adjacency1, std::vector<int> &adjacency2);
};

#include"multiFrameMerge.hpp"


#endif

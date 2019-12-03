/*!
*\ Copyright  (C) Zhejiang University Robotics Laboratory
*\ All rights reserved
*\ Author: Ying Chen
*\ Email: cysmilebaby@gmail.com
*\ Date:2016-05
*\ Description: object partition from the pre segmentaion of surface
*/


#ifndef _CONNECTION_RELATIONSHIP_
#define _CONNECTION_RELATIONSHIP_

#include <pcl/io/pcd_io.h>
#include <pcl/pcl_base.h>
#include <pcl/search/search.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/features/normal_3d.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_pointcloud_adjacency.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/registration/transformation_estimation_svd.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h> 
#include <pcl/filters/extract_indices.h>

#include <opencv2/opencv.hpp>

#include <Eigen/Dense>

#include <list>
#include <math.h>


template<typename PointT, typename NormalT>
struct boundingParam
{
    PointT min_point_AABB;
    PointT max_point_AABB;
    PointT min_point_OBB;
    PointT max_point_OBB;
    PointT position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    float major_value, middle_value, minor_value;
    Eigen::Vector3f major_vector, middle_vector, minor_vector;
    Eigen::Vector3f mass_center;

};
struct cubParam{
    double width;
    double height;
    double depth;
    Eigen::Quaternionf rotation;
    Eigen::Vector3f position;
    std::vector<Eigen::Vector3f> axis;
};
struct planeParam{
    double width;
    double height;
    std::vector<float> center;
    Eigen::Vector3f normal;
};

/// \brief some function of object partition
///
template<typename PointT, typename NormalT>
class objPartition
{
public:
	typedef pcl::search::Search <PointT> KdTree;
	typedef typename KdTree::Ptr KdTreePtr;
	typedef pcl::PointCloud <NormalT> Normal;
	typedef typename Normal::Ptr NormalPtr;
	typedef pcl::PointCloud <PointT> CloudT;
	typedef typename CloudT::Ptr CloudPtr;
	objPartition();
	~objPartition();

protected:

	/** \brief This meathod compute the cloudpoints convexhull points
	*/
	void
		computeConvexhull();
	/** \brief Thif meathod compute the feature of every surface, the centriod and the centriod normal
	*/
	void
		surfaceFeature();
	/** \brief This meathod compute the surface i and j's ralationship is convex or conhull,1 is convex
	* param[in]: surface_source is one surface
	* param[in]: surface_target is the other surface number
	*/
	bool
		computeConvexCon(int surface_source, int surface_target);
	/** \brief to judge the surface is connected with each other through convexhull points of every surface
	* param[in]: surface_source is one surface
	* param[in]: surface_target is the other surface number
	*/
	bool
		connectConvexhull(int surface_source, int surface_target);
	void
		connectionVec();
    std::vector<std::vector<int> >
		matrix_bool();
	int
		and_matrix(std::vector<int> &adjacency1, std::vector<int> &adjacency2);
	double
		getAngle3D(const Eigen::Vector3f &v1, const Eigen::Vector3f &v2, const bool in_degree);
	/** \brief to merge all connected surface
	*/
	void
		joinMerge();
	/** \brief to merge surfaces which can componect to a circle
	*/
	void
		circleMerge();
    /** \brief get the oriented bounding box of a series cloud
    * \param[in] cloud the series cloud point of segment
    * \param[out] bounding_params the parameters about cloud min and suit bounding box
    */
    void
        getCloudBoundingBox(std::vector<CloudPtr> &cloud, pcl::PointCloud <pcl::PointXYZRGB>::Ptr &colored_cloud);
    void
        testGraphMerge(int boundingBoxNum);

    void
        getPlaneParams(std::vector<CloudPtr> &cloud, std::vector<planeParam> &plane_param);

	struct cmp
	{
		bool operator() (double P1, double P2)
		{
			return P1 < P2;
		}
	};
    static bool
    //
        sortVec(const std::pair<double, int> a, const std::pair<double, int> b)
        { return a.first < b.first;
        }
    static bool
        sortVecInt(const std::pair<int, int> a, const std::pair<int, int> b)
        {
            return a.second > b.second;
        }
public:
    std::vector<cubParam>
        getCub();
    std::vector<cubParam>
        getSurfaceCub();
    void
        simulation(std::vector<cubParam> &cubs_params);

	void
		saveCircle(std::string fileName);
	void
		objectMerge(std::vector<CloudPtr> &object);
	void
		setInputSurface(std::vector<CloudPtr> &surface);
	/** \brief save the partition cloud point into .pcd file
	* param[in]:filePath is the flord you wante to save the .pcd file
	*/
	void
		getObjCloud(std::string filePath);
	void
		objColored(std::vector<CloudPtr> &object, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &obj_colored);

protected:
	std::vector<NormalT> surface_normal_;
    std::vector<std::vector<double> > surface_size_;//the size including width, length and height
	std::vector<PointT> surface_centroid_;
	std::vector<CloudPtr> convexhull_points_;
    std::vector<std::vector<int> > adjacency_;
    std::set<std::set<int> > circle_relationship_;
	std::vector<CloudPtr> surface_;
	std::vector<CloudPtr> surface_out_;
    std::vector<CloudPtr> plane_;
};


#include "objPartition.hpp"


#endif

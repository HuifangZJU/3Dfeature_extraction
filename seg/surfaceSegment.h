
#ifndef _INITIAL_SEGMENT_H_
#define _INITIAL_SEGMENT_H_

#include <pcl/pcl_base.h>
#include <pcl/search/search.h>
#include <pcl/features/boundary.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/extract_clusters.h>
#include <list>
#include <math.h>
#include <time.h>



template<typename PointT, typename NormalT>
class surfaceSegment : public pcl::PCLBase<PointT>
{
public:
	typedef pcl::search::Search <PointT> KdTree;
	typedef typename KdTree::Ptr KdTreePtr;
	typedef pcl::PointCloud <NormalT> Normal;
	typedef typename Normal::Ptr NormalPtr;
	typedef pcl::PointCloud <PointT> CloudT;
	typedef typename CloudT::Ptr CloudPtr;
	using pcl::PCLBase <PointT>::input_;
	using pcl::PCLBase <PointT>::indices_;
	using pcl::PCLBase <PointT>::initCompute;
	using pcl::PCLBase <PointT>::deinitCompute;
public:
	surfaceSegment();
	~surfaceSegment();
	int
		getMinClusterSize();
	void
		setMinClusterSize(int min_cluster_size);
	int
		getMaxClusterSize();
	void
		setMaxClusterSize(int max_cluster_size);
	bool
		getSmoothModeFlag() const;

	void
		setSmoothModeFlag(bool value);
	bool
		getCurvatureTestFlag() const;
	virtual void
		setCurvatureTestFlag(bool value);
	bool
		getResidualTestFlag() const;
	virtual void
		setResidualTestFlag(bool value);
	float
		getSmoothnessThreshold() const;
	void
		setSmoothnessThreshold(float theta);
	float
		getResidualThreshold() const;
	void
		setResidualThreshold(float residual);
	float
		getCurvatureThreshold() const;
	void
		setCurvatureThreshold(float curvature);

	//using KNN to search the neighbours points
	unsigned int
		getNumberOfNeighbours() const;
	void
		setNumberOfNeighbours(unsigned int neighbour_number);
	KdTreePtr
		getSearchMethod() const;
	void
		setSearchMethod(const KdTreePtr& tree);
	NormalPtr
		getInputNormals() const;
	void
		setInputNormals(const NormalPtr& norm);
	/** \brief This method launches the segmentation algorithm and returns the clusters that were
	* obtained during the segmentation.
	* \param[out] clusters clusters that were obtained. Each cluster is an array of point indices.
	*/
	virtual void
		extract(std::vector<CloudPtr> &seg_cloud, std::vector <pcl::PointIndices> &clusters);
	/** \brief For a given point this function builds a segment to which it belongs and returns this segment.
	* \param[in] index index of the initial point which will be the seed for growing a segment.
	* \param[out] cluster cluster to which the point belongs.
	*/
	virtual void
		getSegmentFromPoint(int index, pcl::PointIndices& cluster);
	/** \brief If the cloud was successfully segmented, then function
	* returns colored cloud. Otherwise it returns an empty pointer.
	* Points that belong to the same segment have the same color.
	* But this function doesn't guarantee that different segments will have different
	* color(it all depends on RNG). Points that were not listed in the indices array will have red color.
	*/
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr
		getColoredCloud(std::vector <pcl::PointIndices>& clusters);
	/** \brief If the cloud was successfully segmented, then function
	* returns colored cloud. Otherwise it returns an empty pointer.
	* Points that belong to the same segment have the same color.
	* But this function doesn't guarantee that different segments will have different
	* color(it all depends on RNG). Points that were not listed in the indices array will have red color.
	*/
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
		getColoredCloudRGBA();
	/** \breif to check if there is possibility to execute segmentation algorithm, if ok return true */
	virtual bool
		prepareForSegmentation();

	virtual void
		findPointNeighbours();
	void
		applySmoothsurfaceSegmentAlgorithm();

	/** \brief This method grows a segment for the given seed point. And returns the number of its points.
	* \param[in] initial_seed index of the point that will serve as the seed point
	* \param[in] segment_number indicates which number this segment will have
	*/
	int
		growRegion(int initial_seed, int segment_number);

	/** \brief This function is checking if the point with index 'nghbr' belongs to the segment.
	* If so, then it returns true. It also checks if this point can serve as the seed.
	* \param[in] initial_seed index of the initial point that was passed to the growRegion() function
	* \param[in] point index of the current seed point
	* \param[in] nghbr index of the point that is neighbour of the current seed
	* \param[out] is_a_seed this value is set to true if the point with index 'nghbr' can serve as the seed
	*/
	virtual bool
		validatePoint(int initial_seed, int point, int nghbr, bool& is_a_seed) const;

	/** \brief This function simply assembles the regions from list of point labels.
	* Each cluster is an array of point indices.
	*/
	void
		assembleRegions();
	/** \brief This function merge the unclustered points to the clustered point cloud.
	* Each cluster is an array of point indices.
	*/
	void
        unclusterMerge(std::vector<CloudPtr> &seg_cloud, std::vector <pcl::PointIndices> &clusters, double &ratio);
	void
		euclideanCluster(std::vector<CloudPtr> &seg_cloud, std::vector <pcl::PointIndices> &clusters);

protected:
    /** \brief Get the angle of two normals. */
    double
        getAngle3D(const Eigen::Vector3f &v1, const Eigen::Vector3f &v2, const bool in_degree);
    /** \brief the point of source_poit and target_point connection is convex of convex hull.
      * \param[in] source_point
      * \param[in] target_point
      * \param[in] source_normal
      * \param[in] target_normal
    */
    bool
        convexJudge(const PointT &source_point, const PointT &target_point,const Eigen::Vector3f &source_normal,const Eigen::Vector3f &target_normal);
	/** \brief Stores the minimum number of points that a cluster needs to contain in order to be considered valid. */
	int min_pts_per_cluster_;

	/** \brief Stores the maximum number of points that a cluster needs to contain in order to be considered valid. */
	int max_pts_per_cluster_;

	/** \brief Flag that signalizes if the smoothness constraint will be used. */
	bool smooth_mode_flag_;

	/** \brief If set to true then curvature test will be done during segmentation. */
	bool curvature_flag_;

	/** \brief If set to true then residual test will be done during segmentation. */
	bool residual_flag_;

	/** \brief Thershold used for testing the smoothness between points. */
	float theta_threshold_;

	/** \brief Thershold used in residual test. */
	float residual_threshold_;

	/** \brief Thershold used in curvature test. */
	float curvature_threshold_;

	/** \brief Number of neighbours to find. */
	unsigned int neighbour_number_;

	/** \brief Serch method that will be used for KNN. */
	KdTreePtr search_;

	/** \brief Contains normals of the points that will be segmented. */
	NormalPtr normals_;


	std::vector<std::vector<int> > point_neighbours_;


	std::vector<int> point_labels_;

	/** \brief If set to true then normal/smoothness test will be done during segmentation.
	* It is always set to true for the usual region growing algorithm. It is used for turning on/off the test
	* for smoothness in the child class RegionGrowingRGB.*/
	bool normal_flag_;


	std::vector<int> num_pts_in_segment_;
	/** \brief Stores the number of segments. */
	int number_of_segments_;
	std::vector<NormalPtr> seg_normals_vec_;

	std::vector<CloudPtr> seg_cloud_;
	/** \brief After the segmentation it will contain the segments. */
	std::vector<pcl::PointIndices> clusters_;
	CloudPtr unseg_cloud_;
	pcl::PointIndices unClusters_;
};





#define PCL_INSTANTIATE_surfaceSegment(T) template class surfaceSegment<T, pcl::Normal>;


#include "surfaceSegment.hpp"


#endif//_INITIAL_SEGMENT_H_

#ifndef _SURFACE_SEGMENT_HPP
#define _SURFACE_SEGMENT_HPP

#include"surfaceSegment.h"
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT>
surfaceSegment<PointT, NormalT>::surfaceSegment() :
min_pts_per_cluster_(200),
max_pts_per_cluster_(std::numeric_limits<int>::max()),
smooth_mode_flag_(true),
curvature_flag_(true),
residual_flag_(false),
theta_threshold_(2.3f / 180.0f * static_cast<float> (M_PI)),
residual_threshold_(0.05f),
curvature_threshold_(0.05f),
neighbour_number_(40),
search_(),
normals_(),
point_neighbours_(0),
point_labels_(0),
normal_flag_(true),
num_pts_in_segment_(0),
clusters_(0),
number_of_segments_(0),
seg_cloud_(),
unseg_cloud_(new CloudT),
unClusters_(),
seg_normals_vec_()
{
}


inline bool
comparePair(std::pair<float, int> i, std::pair<float, int> j)
{
	return (i.first < j.first);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT>
surfaceSegment<PointT, NormalT>::~surfaceSegment()
{
	if (search_ != 0)
		search_.reset();
	if (normals_ != 0)
		normals_.reset();

	point_neighbours_.clear();
	point_labels_.clear();
	num_pts_in_segment_.clear();
	clusters_.clear();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> int
surfaceSegment<PointT, NormalT>::getMinClusterSize()
{
	return (min_pts_per_cluster_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
surfaceSegment<PointT, NormalT>::setMinClusterSize(int min_cluster_size)
{
	min_pts_per_cluster_ = min_cluster_size;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> int
surfaceSegment<PointT, NormalT>::getMaxClusterSize()
{
	return (max_pts_per_cluster_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
surfaceSegment<PointT, NormalT>::setMaxClusterSize(int max_cluster_size)
{
	max_pts_per_cluster_ = max_cluster_size;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> bool
surfaceSegment<PointT, NormalT>::getSmoothModeFlag() const
{
	return (smooth_mode_flag_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
surfaceSegment<PointT, NormalT>::setSmoothModeFlag(bool value)
{
	smooth_mode_flag_ = value;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> bool
surfaceSegment<PointT, NormalT>::getCurvatureTestFlag() const
{
	return (curvature_flag_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
surfaceSegment<PointT, NormalT>::setCurvatureTestFlag(bool value)
{
	curvature_flag_ = value;

	if (curvature_flag_ == false && residual_flag_ == false)
		residual_flag_ = true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> bool
surfaceSegment<PointT, NormalT>::getResidualTestFlag() const
{
	return (residual_flag_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
surfaceSegment<PointT, NormalT>::setResidualTestFlag(bool value)
{
	residual_flag_ = value;

	if (curvature_flag_ == false && residual_flag_ == false)
		curvature_flag_ = true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> float
surfaceSegment<PointT, NormalT>::getSmoothnessThreshold() const
{
	return (theta_threshold_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
surfaceSegment<PointT, NormalT>::setSmoothnessThreshold(float theta)
{
	theta_threshold_ = theta;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> float
surfaceSegment<PointT, NormalT>::getResidualThreshold() const
{
	return (residual_threshold_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
surfaceSegment<PointT, NormalT>::setResidualThreshold(float residual)
{
	residual_threshold_ = residual;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> float
surfaceSegment<PointT, NormalT>::getCurvatureThreshold() const
{
	return (curvature_threshold_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
surfaceSegment<PointT, NormalT>::setCurvatureThreshold(float curvature)
{
	curvature_threshold_ = curvature;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> unsigned int
surfaceSegment<PointT, NormalT>::getNumberOfNeighbours() const
{
	return (neighbour_number_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
surfaceSegment<PointT, NormalT>::setNumberOfNeighbours(unsigned int neighbour_number)
{
	neighbour_number_ = neighbour_number;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> typename surfaceSegment<PointT, NormalT>::KdTreePtr
surfaceSegment<PointT, NormalT>::getSearchMethod() const
{
	return (search_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
surfaceSegment<PointT, NormalT>::setSearchMethod(const KdTreePtr& tree)
{
	if (search_ != 0)
		search_.reset();

	search_ = tree;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> typename surfaceSegment<PointT, NormalT>::NormalPtr
surfaceSegment<PointT, NormalT>::getInputNormals() const
{
	return (normals_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
surfaceSegment<PointT, NormalT>::setInputNormals(const NormalPtr& norm)
{
	if (normals_ != 0)
		normals_.reset();

	normals_ = norm;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
surfaceSegment<PointT, NormalT>::extract(std::vector<CloudPtr> &seg_cloud, std::vector <pcl::PointIndices> &clusters)
{
	std::cout << "starting extract!" << std::endl;
	clusters_.clear();
	clusters.clear();
    unClusters_.indices.clear();
	seg_cloud.clear();
	seg_cloud_.clear();
	seg_normals_vec_.clear();

	point_neighbours_.clear();
	point_labels_.clear();
	num_pts_in_segment_.clear();
	number_of_segments_ = 0;

	bool segmentation_is_possible = initCompute();
	if (!segmentation_is_possible)
	{
		deinitCompute();
		return;
	}

	segmentation_is_possible = prepareForSegmentation();
	if (!segmentation_is_possible)
	{
		deinitCompute();
		return;
	}

	findPointNeighbours();
	applySmoothsurfaceSegmentAlgorithm();
	assembleRegions();
	unClusters_.indices.resize(unClusters_.indices.size());
	clusters.resize(clusters_.size());
	std::vector<pcl::PointIndices>::iterator cluster_iter_input = clusters.begin();
	int point_i = 0;
	for (std::vector<pcl::PointIndices>::const_iterator cluster_iter = clusters_.begin(); cluster_iter != clusters_.end(); cluster_iter++)
	{
		CloudPtr seg(new CloudT);
		CloudPtr unseg(new CloudT);
		NormalPtr seg_normal(new Normal);
		NormalPtr unseg_normal(new Normal);
		if ((static_cast<int> (cluster_iter->indices.size()) >= min_pts_per_cluster_) &&
			(static_cast<int> (cluster_iter->indices.size()) <= max_pts_per_cluster_))
		{
			*cluster_iter_input = *cluster_iter;
			cluster_iter_input++;


			for (int cluster_i = 0; cluster_i < cluster_iter->indices.size(); ++cluster_i)
			{
				seg->points.push_back(input_->points[cluster_iter->indices[cluster_i]]);
				seg_normal->points.push_back(normals_->points[cluster_iter->indices[cluster_i]]);
			}
			seg_cloud.push_back(seg);
			seg_cloud_.push_back(seg);
			seg_normals_vec_.push_back(seg_normal);
		}
		else
		{
			for (int cluster_i = 0; cluster_i < cluster_iter->indices.size(); ++cluster_i)
			{
				unClusters_.indices.push_back(cluster_iter->indices[cluster_i]);
				point_i++;
			}
		}
	}
	clusters_ = std::vector<pcl::PointIndices>(clusters.begin(), cluster_iter_input);
	unClusters_.indices.resize(point_i);

	clusters.resize(clusters_.size());
	//clusters_.resize(clusters_.size());
    double ratio = 0.5;
    while(ratio < 0.999 &&ratio >0.01)
        unclusterMerge(seg_cloud, clusters, ratio);




	//std::cout << "something wrong in euclideanCluster!" << std::endl;
	//euclideanCluster(seg_cloud, clusters);

	deinitCompute();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> bool
surfaceSegment<PointT, NormalT>::prepareForSegmentation()
{
	// if user forgot to pass point cloud or if it is empty
	if (input_->points.size() == 0)
		return (false);

	// if user forgot to pass normals or the sizes of point and normal cloud are different
	if (normals_ == 0 || input_->points.size() != normals_->points.size())
		return (false);

	// if residual test is on then we need to check if all needed parameters were correctly initialized
	if (residual_flag_)
	{
		if (residual_threshold_ <= 0.0f)
			return (false);
	}

	// if curvature test is on ...
	// if (curvature_flag_)
	// {
	//   in this case we do not need to check anything that related to it
	//   so we simply commented it
	// }

	// from here we check those parameters that are always valuable
	if (neighbour_number_ == 0)
		return (false);

	// if user didn't set search method
	if (!search_)
		search_.reset(new pcl::search::KdTree<PointT>);

	if (indices_)
	{
		if (indices_->empty())
			PCL_ERROR("[surfaceSegment::prepareForSegmentation] Empty given indices!\n");
		search_->setInputCloud(input_, indices_);
	}
	else
		search_->setInputCloud(input_);

	return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
surfaceSegment<PointT, NormalT>::findPointNeighbours()
{
	int point_number = static_cast<int> (indices_->size());
	std::vector<int> neighbours;
	std::vector<float> distances;
	point_neighbours_.resize(input_->points.size(), neighbours);
	if (input_->is_dense)
	{
		for (int i_point = 0; i_point < point_number; i_point++)
		{
			int point_index = (*indices_)[i_point];
			neighbours.clear();
			search_->nearestKSearch(i_point, neighbour_number_, neighbours, distances);
			point_neighbours_[point_index].swap(neighbours);
		}
	}
	else
	{
		for (int i_point = 0; i_point < point_number; i_point++)
		{
			neighbours.clear();
			int point_index = (*indices_)[i_point];
			if (!pcl::isFinite(input_->points[point_index]))
				continue;
			search_->nearestKSearch(i_point, neighbour_number_, neighbours, distances);
			point_neighbours_[point_index].swap(neighbours);
		}
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
surfaceSegment<PointT, NormalT>::applySmoothsurfaceSegmentAlgorithm()
{
	int num_of_pts = static_cast<int> (indices_->size());
	point_labels_.resize(input_->points.size(), -1);

	std::vector< std::pair<float, int> > point_residual;
	std::pair<float, int> pair;
	point_residual.resize(num_of_pts, pair);

	if (normal_flag_ == true)
	{
		for (int i_point = 0; i_point < num_of_pts; i_point++)
		{
			int point_index = (*indices_)[i_point];
			point_residual[i_point].first = normals_->points[point_index].curvature;
			point_residual[i_point].second = point_index;
		}
		std::sort(point_residual.begin(), point_residual.end(), comparePair);
	}
	else
	{
		for (int i_point = 0; i_point < num_of_pts; i_point++)
		{
			int point_index = (*indices_)[i_point];
			point_residual[i_point].first = 0;
			point_residual[i_point].second = point_index;
		}
	}
	int seed_counter = 0;
	int seed = point_residual[seed_counter].second;

	int segmented_pts_num = 0;
	int number_of_segments = 0;
	while (segmented_pts_num < num_of_pts)
	{
		int pts_in_segment;
		pts_in_segment = growRegion(seed, number_of_segments);
		segmented_pts_num += pts_in_segment;
		num_pts_in_segment_.push_back(pts_in_segment);
		number_of_segments++;

		//find next point that is not segmented yet
		for (int i_seed = seed_counter + 1; i_seed < num_of_pts; i_seed++)
		{
			int index = point_residual[i_seed].second;
			if (point_labels_[index] == -1)
			{
				seed = index;
				break;
			}
		}
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> int
surfaceSegment<PointT, NormalT>::growRegion(int initial_seed, int segment_number)
{
	std::queue<int> seeds;
	seeds.push(initial_seed);
	point_labels_[initial_seed] = segment_number;
	//segment initial
	int num_pts_in_segment = 1;

	while (!seeds.empty())
	{
		int curr_seed;
		curr_seed = seeds.front();
		seeds.pop();

		size_t i_nghbr = 0;
		while (i_nghbr < neighbour_number_ && i_nghbr < point_neighbours_[curr_seed].size())
		{
			int index = point_neighbours_[curr_seed][i_nghbr];
			if (point_labels_[index] != -1)
			{
				i_nghbr++;
				continue;
			}

			bool is_a_seed = false;
			bool belongs_to_segment = validatePoint(initial_seed, curr_seed, index, is_a_seed);

			if (belongs_to_segment == false)
			{
				i_nghbr++;
				continue;
			}

			point_labels_[index] = segment_number;
			num_pts_in_segment++;

			if (is_a_seed)
			{
				seeds.push(index);
			}

			i_nghbr++;
		}// next neighbour
	}// next seed

	return (num_pts_in_segment);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> bool
surfaceSegment<PointT, NormalT>::validatePoint(int initial_seed, int point, int nghbr, bool& is_a_seed) const
{
	is_a_seed = true;

	float cosine_threshold = cosf(theta_threshold_);
	float data[4];

	data[0] = input_->points[point].data[0];
	data[1] = input_->points[point].data[1];
	data[2] = input_->points[point].data[2];
	data[3] = input_->points[point].data[3];
	Eigen::Map<Eigen::Vector3f> initial_point(static_cast<float*> (data));
	Eigen::Map<Eigen::Vector3f> initial_normal(static_cast<float*> (normals_->points[point].normal));

	//check the angle between normals
	if (smooth_mode_flag_ == true)
	{
		Eigen::Map<Eigen::Vector3f> nghbr_normal(static_cast<float*> (normals_->points[nghbr].normal));
		float dot_product = fabsf(nghbr_normal.dot(initial_normal));
		//\BB\B9Ӧ\B8ð\D1\D1\D5ɫ\B5\C4\D0\C5Ϣ\BCӽ\F8\C0\B4
		if (dot_product < cosine_threshold)
		{
			return (false);
		}
	}
	else
	{
		Eigen::Map<Eigen::Vector3f> nghbr_normal(static_cast<float*> (normals_->points[nghbr].normal));
		Eigen::Map<Eigen::Vector3f> initial_seed_normal(static_cast<float*> (normals_->points[initial_seed].normal));
		float dot_product = fabsf(nghbr_normal.dot(initial_seed_normal));
		if (dot_product < cosine_threshold)
			return (false);
	}

	// check the curvature if needed
	if (curvature_flag_ && normals_->points[nghbr].curvature > curvature_threshold_)
	{
		is_a_seed = false;
	}

	// check the residual if needed
	data[0] = input_->points[nghbr].data[0];
	data[1] = input_->points[nghbr].data[1];
	data[2] = input_->points[nghbr].data[2];
	data[3] = input_->points[nghbr].data[3];
	Eigen::Map<Eigen::Vector3f> nghbr_point(static_cast<float*> (data));
	float residual = fabsf(initial_normal.dot(initial_point - nghbr_point));
	if (residual_flag_ && residual > residual_threshold_)
		is_a_seed = false;
	return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
surfaceSegment<PointT, NormalT>::assembleRegions()
{
	int number_of_segments = static_cast<int> (num_pts_in_segment_.size());
	int number_of_points = static_cast<int> (input_->points.size());

	pcl::PointIndices segment;
	clusters_.resize(number_of_segments, segment);

	for (int i_seg = 0; i_seg < number_of_segments; i_seg++)
	{
		clusters_[i_seg].indices.resize(num_pts_in_segment_[i_seg], 0);
	}
	//	int number_of_unClusters = number_of_points - number_of_clusters;
	//	unClusters_.indices.resize(number_of_unClusters);

	std::vector<int> counter;
	counter.resize(number_of_segments, 0);
	int unCluster_index = 0;
	for (int i_point = 0; i_point < number_of_points; i_point++)
	{
		int segment_index = point_labels_[i_point];
		if (segment_index != -1)
		{
			int point_index = counter[segment_index];
			clusters_[segment_index].indices[point_index] = i_point;
			counter[segment_index] = point_index + 1;
		}
	}
	number_of_segments_ = number_of_segments;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
surfaceSegment<PointT, NormalT>::getSegmentFromPoint(int index, pcl::PointIndices& cluster)
{
	cluster.indices.clear();

	bool segmentation_is_possible = initCompute();
	if (!segmentation_is_possible)
	{
		deinitCompute();
		return;
	}

	// first of all we need to find out if this point belongs to cloud
	bool point_was_found = false;
	int number_of_points = static_cast <int> (indices_->size());
	for (int point = 0; point < number_of_points; point++)
	if ((*indices_)[point] == index)
	{
		point_was_found = true;
		break;
	}

	if (point_was_found)
	{
		if (clusters_.empty())
		{
			point_neighbours_.clear();
			point_labels_.clear();
			num_pts_in_segment_.clear();
			number_of_segments_ = 0;

			segmentation_is_possible = prepareForSegmentation();
			if (!segmentation_is_possible)
			{
				deinitCompute();
				return;
			}

			findPointNeighbours();
			applySmoothsurfaceSegmentAlgorithm();
			assembleRegions();
		}
		// if we have already made the segmentation, then find the segment
		// to which this point belongs
		std::vector <pcl::PointIndices>::iterator i_segment;
		for (i_segment = clusters_.begin(); i_segment != clusters_.end(); i_segment++)
		{
			bool segment_was_found = false;
			for (size_t i_point = 0; i_point < i_segment->indices.size(); i_point++)
			{
				if (i_segment->indices[i_point] == index)
				{
					segment_was_found = true;
					cluster.indices.clear();
					cluster.indices.reserve(i_segment->indices.size());
					std::copy(i_segment->indices.begin(), i_segment->indices.end(), std::back_inserter(cluster.indices));
					break;
				}
			}
			if (segment_was_found)
			{
				break;
			}
		}// next segment
	}// end if point was found

	deinitCompute();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> pcl::PointCloud<pcl::PointXYZRGB>::Ptr
surfaceSegment<PointT, NormalT>::getColoredCloud(std::vector <pcl::PointIndices>& clusters)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;
    //std::cout << "all cloud point size:" << input_->points.size();
    std::cout << "cluster size:" << clusters.size() << std::endl;
	if (!clusters.empty())
	{
		colored_cloud = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared();

		srand(static_cast<unsigned int> (time(0)));
		std::vector<unsigned char> colors;
		for (size_t i_segment = 0; i_segment < clusters.size(); i_segment++)
		{
			colors.push_back(static_cast<unsigned char> (rand() % 256));
			colors.push_back(static_cast<unsigned char> (rand() % 256));
			colors.push_back(static_cast<unsigned char> (rand() % 256));
		}

		colored_cloud->width = input_->width;
		colored_cloud->height = input_->height;
		colored_cloud->is_dense = input_->is_dense;
		for (size_t i_point = 0; i_point < input_->points.size(); i_point++)
		{
			pcl::PointXYZRGB point;
			point.x = *(input_->points[i_point].data);
			point.y = *(input_->points[i_point].data + 1);
			point.z = *(input_->points[i_point].data + 2);
			point.r = 255;
			point.g = 255;
			point.b = 255;
			colored_cloud->points.push_back(point);
		}
		
		std::vector< pcl::PointIndices >::iterator i_segment;
		int next_color = 0;
		for (i_segment = clusters.begin(); i_segment != clusters.end(); i_segment++)
		{
			std::vector<int>::iterator i_point;
			for (i_point = i_segment->indices.begin(); i_point != i_segment->indices.end(); i_point++)
			{
				int index;
				index = *i_point;
				colored_cloud->points[index].r = colors[3 * next_color];
				colored_cloud->points[index].g = colors[3 * next_color + 1];
				colored_cloud->points[index].b = colors[3 * next_color + 2];
			}
			next_color++;
		}
	}
    //std::cout << "colored point size:" << colored_cloud->points.size();
	return (colored_cloud);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
surfaceSegment<PointT, NormalT>::getColoredCloudRGBA()
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr colored_cloud;

	if (!clusters_.empty())
	{
		colored_cloud = (new pcl::PointCloud<pcl::PointXYZRGBA>)->makeShared();

		srand(static_cast<unsigned int> (time(0)));
		std::vector<unsigned char> colors;
		for (size_t i_segment = 0; i_segment < clusters_.size(); i_segment++)
		{
			colors.push_back(static_cast<unsigned char> (rand() % 256));
			colors.push_back(static_cast<unsigned char> (rand() % 256));
			colors.push_back(static_cast<unsigned char> (rand() % 256));
		}

		colored_cloud->width = input_->width;
		colored_cloud->height = input_->height;
		colored_cloud->is_dense = input_->is_dense;
		for (size_t i_point = 0; i_point < input_->points.size(); i_point++)
		{
			pcl::PointXYZRGBA point;
			point.x = *(input_->points[i_point].data);
			point.y = *(input_->points[i_point].data + 1);
			point.z = *(input_->points[i_point].data + 2);
			point.r = 255;
			point.g = 0;
			point.b = 0;
			point.a = 0;
			colored_cloud->points.push_back(point);
		}

		std::vector< pcl::PointIndices >::iterator i_segment;
		int next_color = 0;
		for (i_segment = clusters_.begin(); i_segment != clusters_.end(); i_segment++)
		{
			std::vector<int>::iterator i_point;
			for (i_point = i_segment->indices.begin(); i_point != i_segment->indices.end(); i_point++)
			{
				int index;
				index = *i_point;
				colored_cloud->points[index].r = colors[3 * next_color];
				colored_cloud->points[index].g = colors[3 * next_color + 1];
				colored_cloud->points[index].b = colors[3 * next_color + 2];
			}
			next_color++;
		}
	}

	return (colored_cloud);
}
//////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
surfaceSegment<PointT, NormalT>::unclusterMerge(std::vector<CloudPtr> &seg_cloud, std::vector <pcl::PointIndices> &clusters, double &ratio)
{
	//building new kd tree through the origin data to get all point label
	std::vector<int> clusters_labels;
	CloudPtr clusters_points(new CloudT);
	for (int i = 0; i < clusters.size(); ++i)
	{
		for (int j = 0; j < clusters[i].indices.size(); ++j)
		{
			clusters_points->points.push_back(input_->points[clusters[i].indices[j]]);
			clusters_labels.push_back(i);
		}
	}
//	pcl::KdTreeFLANN<pcl::PointXYZ> clusters_kdtree;
    pcl::KdTreeFLANN<PointT> clusters_kdtree;
	clusters_kdtree.setInputCloud(clusters_points);
	//save the neighbor points of all unsegment point, and neighbor_number are the number of neighbors 
    std::vector<std::vector<int> > uncluster_neighbors(unClusters_.indices.size(), std::vector<int>(neighbour_number_, 0));
    std::vector<std::vector<float> > uncluster_neighbors_distance(unClusters_.indices.size(), std::vector<float>(neighbour_number_, 0));
	//search the unseg point
    pcl::PointIndices new_uncluster;
	for (int i = 0; i < unClusters_.indices.size(); ++i)
	{
		int index = unClusters_.indices[i];

        pcl::flipNormalTowardsViewpoint(input_->points[index], 0, 0, 0, normals_->points[index].normal_x, normals_->points[index].normal_y, normals_->points[index].normal_z);
        Eigen::Vector3f source_normal(normals_->points[index].normal_x, normals_->points[index].normal_y, normals_->points[index].normal_z);


        std::set<std::pair<int, int> > unclusterConnect;
        //first is the number of neighbor, second is the index of indices
        std::vector<int> uncluster_neigbor_lables(clusters.size(), 0);
        clusters_kdtree.nearestKSearch(input_->points[index], neighbour_number_, uncluster_neighbors[i], uncluster_neighbors_distance[i]);
        for (int j = 0; j < neighbour_number_; ++j)
        {
            if (uncluster_neighbors[i][j] < clusters_labels.size() && uncluster_neighbors_distance[i][j]<0.01)
            {
                int neighbor_label = clusters_labels[uncluster_neighbors[i][j]];

                pcl::flipNormalTowardsViewpoint(input_->points[neighbor_label], 0, 0, 0, normals_->points[neighbor_label].normal_x, normals_->points[neighbor_label].normal_y, normals_->points[neighbor_label].normal_z);
                Eigen::Vector3f target_normal(normals_->points[neighbor_label].normal_x, normals_->points[neighbor_label].normal_y, normals_->points[neighbor_label].normal_z);
                //if(convexJudge(input_->points[index], input_->points[neighbor_label], source_normal, target_normal))
                if(getAngle3D(source_normal, source_normal, true) < 70)
                    uncluster_neigbor_lables[neighbor_label]++;
            }
        }
        for (int i = 0; i < uncluster_neigbor_lables.size(); ++i)
        {
            std::pair<int, int> unclusterPair;
            unclusterPair.first = uncluster_neigbor_lables[i];
            unclusterPair.second = i;
            unclusterConnect.insert(unclusterPair);
        }

        int max_lable = *max_element(uncluster_neigbor_lables.begin(), uncluster_neigbor_lables.end());
        if(max_lable != 0)
        {
            std::vector<int>::iterator max_position = find(uncluster_neigbor_lables.begin(), uncluster_neigbor_lables.end(), max_lable);
            int position = max_position - uncluster_neigbor_lables.begin();
            clusters_[position].indices.push_back(index);
            seg_cloud_[position]->points.push_back(input_->points[index]);
            clusters[position].indices.push_back(index);
            seg_cloud[position]->points.push_back(input_->points[index]);
            seg_normals_vec_[position]->points.push_back(normals_->points[index]);

        }
        else
        {
            //std::cout << "enter!" << std::endl;
            new_uncluster.indices.push_back(index);
        }

    }
    //std::cout << "the size:" << new_uncluster.indices.size() <<  " and the uncluster size is:" << unClusters_.indices.size() <<std::endl;
    ratio = ((double)new_uncluster.indices.size()/ (double)unClusters_.indices.size());
    std::cout << "the ratio is:" << ratio << std::endl;
    unClusters_.indices.clear();
    for(unsigned i = 0; i < new_uncluster.indices.size(); ++i)
        unClusters_.indices.push_back(new_uncluster.indices[i]);

}
//////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
surfaceSegment<PointT, NormalT>::euclideanCluster(std::vector<CloudPtr> &seg_cloud, std::vector <pcl::PointIndices> &clusters)
{
	seg_cloud.clear();
	//clusters.clear();

	for (size_t i = 0; i < seg_cloud_.size(); ++i)
	{
		pcl::EuclideanClusterExtraction<PointT> ec;
		typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
		std::vector<pcl::PointIndices> cluster_indices;
		//segmentation
		tree->setInputCloud(seg_cloud_[i]);
		//std::cout << "build the tree?" << std::endl;
		ec.setClusterTolerance(0.03); // 2cm
		ec.setMinClusterSize(100);
		ec.setMaxClusterSize(1000000);
		//the tree to search
		ec.setSearchMethod(tree);
		ec.setInputCloud(seg_cloud_[i]);
		ec.extract(cluster_indices);
		size_t cluster_size = 0;
		for (int indices_i = 0; indices_i < cluster_indices.size(); ++indices_i)
			cluster_size += cluster_indices[indices_i].indices.size();
		/*if (seg_cloud_[i]->points.size() != cluster_size)
			std::cout << "the size of cluster and point !=:" << seg_cloud_[i]->points.size() << "_" << cluster_size << std::endl;*/
		//std::cout << "finished extract!" << std::endl;
		
		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
		{
			//std::cout << "what happened?" << std::endl;
			pcl::PointIndices cluster_single;
			CloudPtr cloud_single(new CloudT);
			for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
			{
				cloud_single->points.push_back(seg_cloud_[i]->points[*pit]);
				//cluster_single.indices.push_back(clusters_[i].indices[*pit]);
			}
			seg_cloud.push_back(cloud_single);
			//clusters.push_back(cluster_single);
		}
	}
	std::cout << "finished euclideanCluster" << std::endl;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> double
surfaceSegment<PointT, NormalT>::getAngle3D(const Eigen::Vector3f &v1, const Eigen::Vector3f &v2, const bool in_degree)
{
    // Compute the actual angle
    double rad = v1.normalized().dot(v2.normalized());
    if (rad < -1.0)
        rad = -1.0;
    else if (rad >  1.0)
        rad = 1.0;
    return (in_degree ? acos(rad) * 180.0 / M_PI : acos(rad));
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> bool
surfaceSegment<PointT, NormalT>::convexJudge(const PointT &source_point, const PointT &target_point,const Eigen::Vector3f &source_normal,const Eigen::Vector3f &target_normal)
{
//    Eigen::Vector3f source_normal(source_point_normal.normal_x, source_point_normal.normal_y, source_point_normal.normal_z);
//    Eigen::Vector3f target_normal(target_point_normal.normal_x, target_point_normal.normal_y, target_point_normal.normal_z);


    Eigen::Vector3f source_centroid(source_point.x, source_point.y, source_point.z);
    Eigen::Vector3f target_centroid(target_point.x, target_point.y, target_point.z);
    Eigen::Vector3f source_normal_normalized, target_normal_normalized;

    source_normal_normalized = source_normal.normalized();
    target_normal_normalized = target_normal.normalized();

    bool is_convex = true;
    //bool is_smooth = true;

    float normal_angle = getAngle3D(source_normal_normalized, target_normal_normalized, true);
    //  Geometric comparisons
    Eigen::Vector3f vec_t_to_s, vec_s_to_t;

    vec_t_to_s = source_centroid - target_centroid;
    vec_s_to_t = -vec_t_to_s;

    Eigen::Vector3f ncross;
    ncross = source_normal_normalized.cross(target_normal_normalized);

    // Sanity Criterion: Check if definition convexity/concavity makes sense for connection of given patches
    float intersection_angle = getAngle3D(ncross, vec_t_to_s, true);
    float min_intersect_angle = (intersection_angle < 90.) ? intersection_angle : 180. - intersection_angle;
    //std::cout << "min_intersect_angle" << min_intersect_angle << "--normal_angle:" << normal_angle << std::endl;
    float intersect_thresh = 60. * 1. / (1. + exp(-0.25 * (normal_angle - 25.)));
    if (min_intersect_angle < intersect_thresh)
    {
        //std::cout<< "Concave/Convex not defined for given case!" << std::endl;
        is_convex &= false;
    }


    // vec_t_to_s is the reference direction for angle measurements
    // Convexity Criterion: Check if connection of patches is convex. If this is the case the two supervoxels should be merged.
    if ((getAngle3D(vec_t_to_s, source_normal_normalized, true) - getAngle3D(vec_t_to_s, target_normal_normalized, true)) <= 0)
    {
        is_convex &= true;  // connection convex
    }
    else
    {
        is_convex &= (normal_angle < 10.);  // concave connections will be accepted  if difference of normals is small
    }
    return is_convex;
}
#endif

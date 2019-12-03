#ifndef _MULTI_FRAME_MERGE_HPP_
#define _MULTI_FRAME_MERGE_HPP_

#include "multiFrameMerge.h"


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename NormalT>
multiFrameMerge<PointT, NormalT>::multiFrameMerge():
merg_cloud_RGB_()
{}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename NormalT>
multiFrameMerge<PointT, NormalT>::~multiFrameMerge()
{}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename NormalT> void
multiFrameMerge<PointT, NormalT>::ICPTransform(CloudPtr &sourceCloud, CloudPtr &targetCloud, Eigen::Affine3f &transformation)
{
    // Compute surface normals and curvature
    CloudPtr cloud1(new CloudT);
    CloudPtr cloud2(new CloudT);
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(sourceCloud);
    float leafSize = 0.002f;
    sor.setLeafSize(leafSize, leafSize, leafSize);
    sor.filter(*cloud1);

    pcl::transformPointCloud(*targetCloud, *targetCloud, transformation);
    sor.setInputCloud(targetCloud);
    sor.setLeafSize(leafSize, leafSize, leafSize);
    sor.filter(*cloud2);

    pcl::PointCloud<pcl::PointNormal>::Ptr points_with_normals_src(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr points_with_normals_tgt(new pcl::PointCloud<pcl::PointNormal>);

    pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> norm_est;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    norm_est.setSearchMethod(tree);
    norm_est.setKSearch(30);

    norm_est.setInputCloud(cloud1);
    norm_est.compute(*points_with_normals_src);
    pcl::copyPointCloud(*cloud1, *points_with_normals_src);

    norm_est.setInputCloud(cloud2);
    norm_est.compute(*points_with_normals_tgt);
    pcl::copyPointCloud(*cloud2, *points_with_normals_tgt);

    //
    // Instantiate our custom point representation (defined above) ...
    MyPointRepresentation point_representation;
    //// ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
    float alpha[4] = { 1.0, 1.0, 1.0, 1.0 };
    point_representation.setRescaleValues(alpha);

    //
    // Align
    pcl::IterativeClosestPointNonLinear<pcl::PointNormal, pcl::PointNormal> reg;
    reg.setTransformationEpsilon(1e-6);
    // Set the maximum distance between two correspondences (src<->tgt) to 10cm
    // Note: adjust this based on the size of your datasets

    // Set the point representation
    reg.setPointRepresentation(boost::make_shared<const MyPointRepresentation>(point_representation));

    reg.setInputSource(points_with_normals_src);
    reg.setInputTarget(points_with_normals_tgt);

    //
    // Run the same optimization in a loop and visualize the results
    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev, targetToSource;
    pcl::PointCloud<pcl::PointNormal>::Ptr reg_result = points_with_normals_src;
    reg.setMaximumIterations(10);
    for (int i = 0; i < 8; ++i)
    {
        double correspondenceDistance = 0.03;
        correspondenceDistance -= i * 0.01;
        reg.setMaxCorrespondenceDistance(correspondenceDistance);
        //PCL_INFO("Iteration Nr. %d.\n", i);

        // save cloud for visualization purpose
        points_with_normals_src = reg_result;

        // Estimate
        reg.setInputSource(points_with_normals_src);
        reg.align(*reg_result);

        //accumulate transformation between each Iteration
        Ti = reg.getFinalTransformation() * Ti;

        //if the difference between this transformation and the previous one
        //is smaller than the threshold, refine the process by reducing
        //the maximal correspondence distance
        if (fabs((reg.getLastIncrementalTransformation() - prev).sum()) < reg.getTransformationEpsilon())
            reg.setMaxCorrespondenceDistance(reg.getMaxCorrespondenceDistance() - 0.001);

        prev = reg.getLastIncrementalTransformation();

        leafSize = leafSize + i * 0.001;

        sor.setInputCloud(cloud1);
        sor.setLeafSize(leafSize, leafSize, leafSize);
        sor.filter(*cloud1);
        sor.setInputCloud(cloud2);
        sor.setLeafSize(leafSize, leafSize, leafSize);
        sor.filter(*cloud2);

    }
    //std::cout << "overlaping ratio:" << ;
    // Get the transformation from target to source
    targetToSource = Ti.inverse();
    pcl::transformPointCloud(*targetCloud, *targetCloud, targetToSource);

    //pcl::registration::CorrespondenceRejectorTrimmed overlap;

    //overlap.setSourcePoints(cloud1);
    //overlap.setTargetPoints(cloud2);
    //int overlapRatio = overlap.getOverlapRatio();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename NormalT> bool
multiFrameMerge<PointT, NormalT>::getCloudPoints(cv::Mat depthMat, CloudPtr &cloud)
{
    double cx = 325.5;
    double cy = 253.5;
    double fx = 518.0;
    double fy = 519.0;
    double camera_factor = 1000;
    for (unsigned int y = 0; y < depthMat.rows; ++y)
    {
        for (unsigned int x = 0; x < depthMat.cols; ++x)
        {
            pcl::PointXYZ point;
            unsigned short d = depthMat.at<unsigned short>(y, x);
            if (d != 0)
            {
                point.z = d / camera_factor; // Convert from mm to meters
                point.x = (x - cx) * point.z / fx;
                point.y = (y - cy) * point.z / fy;
                cloud->points.push_back(point);
            }
        }
    }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename NormalT>
void multiFrameMerge<PointT, NormalT>::multiObjMerge(std::vector<CloudPtr> &seg_sourceCloud, std::vector<CloudPtr> &seg_targetCloud, std::vector<CloudPtr> &obj_cloud)
{

    //push the data into one pointcloud
    std::vector<int> seg_labels;
    pcl::PointCloud<pcl::PointXYZ>::Ptr sum_source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr sum_target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < seg_sourceCloud.size(); ++i)
    {
        for (int j = 0; j < seg_sourceCloud[i]->points.size(); ++j)
        {
            sum_source_cloud->points.push_back(seg_sourceCloud[i]->points[j]);
            seg_labels.push_back(i);

        }
    }
    for (int i = 0; i < seg_targetCloud.size(); ++i)
    {
        for (int j = 0; j < seg_targetCloud[i]->points.size(); ++j)
        {
            sum_target_cloud->points.push_back(seg_targetCloud[i]->points[j]);
            int number = i + seg_sourceCloud.size();
            seg_labels.push_back(number);
        }
    }

    //the cluster size of both
    int seg_size = seg_labels.size();
    //std::cout << "the size of all cloud:" << seg_size << std::endl;
    int clusterTotal = seg_targetCloud.size() + seg_sourceCloud.size();
    std::vector<std::vector<int> > adjacent(clusterTotal, std::vector<int>(clusterTotal, 0));
    for (int i = 0; i < adjacent.size(); ++i)
        adjacent[i][i] = 1;

    //build a kd tree for judge the connected relationship
    pcl::KdTreeFLANN<pcl::PointXYZ> seg_kdtree;
    seg_kdtree.setInputCloud(sum_target_cloud);
    int k = 10;
    //first is the max num, second is the second max num
    std::vector<int> max_nearest;

    pcl::VoxelGrid<pcl::PointXYZ> sor;
    for (int i = 0; i < seg_sourceCloud.size(); ++i)
    {

        std::vector<int> seg_max(clusterTotal, 0);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFilter(new pcl::PointCloud<pcl::PointXYZ>);
        sor.setInputCloud(seg_sourceCloud[i]);
        sor.setLeafSize(0.01f, 0.01f, 0.01f);
        sor.filter(*cloudFilter);
        for (int j = 0; j < cloudFilter->points.size(); ++j)
        {
            //std::vector<int> all_cloud_nearest(clusterTotal, 0);
            std::vector<std::vector<int> > seg_neighbors(cloudFilter->points.size(), std::vector<int>(k, 0));
            std::vector<std::vector<float> > seg_neighbors_distance(cloudFilter->points.size(), std::vector<float>(k, 0));

            seg_kdtree.nearestKSearch(cloudFilter->points[j], k, seg_neighbors[j], seg_neighbors_distance[j]);
            //std::cout << "seg_kdtree.nearestKSearch" << std::endl;
            for (int m = 0; m < k; ++m)
            {
                if (seg_neighbors[j][m] >= 0 && seg_neighbors[j][m] < sum_target_cloud->points.size() && seg_neighbors_distance[j][m] < 0.001)
                {
                    int index = seg_neighbors[j][m] + sum_source_cloud->points.size();
                    //int index = seg_neighbors[j][m];
                    int neighbor_label = seg_labels[index];
                    //std::cout << "label:" << neighbor_label << std::endl;
                    seg_max[neighbor_label]++;
                }

            }
        }
        int max_lable = *max_element(seg_max.begin(), seg_max.end());
        if (max_lable > cloudFilter->points.size())
        {
            std::vector<int>::iterator max_position = find(seg_max.begin(), seg_max.end(), max_lable);
            int position = max_position - seg_max.begin();
            max_nearest.push_back(position);
        }
        else
        {
            max_nearest.push_back(i);
        }
    }
    pcl::KdTreeFLANN<pcl::PointXYZ> seg_kdtree_tar;
    seg_kdtree_tar.setInputCloud(sum_source_cloud);
    for (int i = 0; i < seg_targetCloud.size(); ++i)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFilter(new pcl::PointCloud<pcl::PointXYZ>);
        sor.setInputCloud(seg_targetCloud[i]);
        sor.setLeafSize(0.01f, 0.01f, 0.01f);
        sor.filter(*cloudFilter);
        std::vector<int> seg_max(clusterTotal, 0);
        for (int j = 0; j < cloudFilter->points.size(); ++j)
        {
            std::vector<int> all_cloud_nearest(clusterTotal, 0);
            std::vector<std::vector<int> > seg_neighbors(cloudFilter->points.size(), std::vector<int>(k, 0));
            std::vector<std::vector<float> > seg_neighbors_distance(cloudFilter->points.size(), std::vector<float>(k, 0));

            seg_kdtree_tar.nearestKSearch(cloudFilter->points[j], k, seg_neighbors[j], seg_neighbors_distance[j]);

            for (int m = 0; m < k; ++m)
            {
                if (seg_neighbors[j][m] > 0 && seg_neighbors[j][m] < sum_source_cloud->points.size() && seg_neighbors_distance[j][m] < 0.001)
                {
                    int neighbor_label = seg_labels[seg_neighbors[j][m]];
                    seg_max[neighbor_label]++;
                }
            }
        }
        int max_lable = *max_element(seg_max.begin(), seg_max.end());
        if (max_lable > cloudFilter->points.size())
        {
            std::vector<int>::iterator max_position = find(seg_max.begin(), seg_max.end(), max_lable);
            int position = max_position - seg_max.begin();
            max_nearest.push_back(position);
        }
        else
        {
            int index = i + seg_sourceCloud.size();
            max_nearest.push_back(index);
        }
    }
    std::cout << "caculate every target point neareat!" << std::endl;
    for (int i = 0; i < max_nearest.size(); ++i)
    {
        //std::cout << "number:" << max_nearest[i] << std::endl;
        adjacent[i][max_nearest[i]] = 1;
    }

    std::cout << "caculate the adjacent!" << std::endl;

    std::set<std::set<int> > circles;
    joinMerge(adjacent, circles);

    std::set<std::set<int> >::iterator circle_i;
    std::set<int>::iterator circle_j;


    for (circle_i = circles.begin(); circle_i != circles.end(); ++circle_i)
    {
        CloudPtr cloudi(new CloudT);
        for (circle_j = (*circle_i).begin(); circle_j != (*circle_i).end(); ++circle_j)
        {
            if (*circle_j < seg_sourceCloud.size())
            {
                for (int i = 0; i < seg_sourceCloud[*circle_j]->points.size(); ++i)
                {
                    PointT point;
                    point.x = seg_sourceCloud[*circle_j]->points[i].x;
                    point.y = seg_sourceCloud[*circle_j]->points[i].y;
                    point.z = seg_sourceCloud[*circle_j]->points[i].z;
                    cloudi->points.push_back(point);
                }
            }
            else
            {
                int index = *circle_j - seg_sourceCloud.size();
                for (int i = 0; i < seg_targetCloud[index]->points.size(); ++i)
                {
                    PointT point;
                    point.x = seg_targetCloud[index]->points[i].x;
                    point.y = seg_targetCloud[index]->points[i].y;
                    point.z = seg_targetCloud[index]->points[i].z;
                    cloudi->points.push_back(point);
                }
            }

        }
        obj_cloud.push_back(cloudi);
    }

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename NormalT> void
multiFrameMerge<PointT, NormalT>::objColored(std::vector<CloudPtr> &object, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &obj_colored)
{
    int next_color = 0;
    srand(static_cast<unsigned int> (time(0)));
    std::vector<unsigned char> colors;
    for (size_t i_segment = 0; i_segment < object.size(); i_segment++)
    {
        colors.push_back(static_cast<unsigned char> (rand() % 256));
        colors.push_back(static_cast<unsigned char> (rand() % 256));
        colors.push_back(static_cast<unsigned char> (rand() % 256));
    }
    for (int i = 0; i < object.size(); ++i)
    {
        for (int j = 0; j < object[i]->points.size(); ++j)
        {
            pcl::PointXYZRGB point;
            point.x = object[i]->points[j].x;
            point.y = object[i]->points[j].y;
            point.z = object[i]->points[j].z;
            point.r = colors[3 * next_color];
            point.g = colors[3 * next_color + 1];
            point.b = colors[3 * next_color + 2];
            obj_colored->points.push_back(point);
        }
        next_color++;
    }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename NormalT> void
multiFrameMerge<PointT, NormalT>::prePartition(CloudPtr &cloud, std::vector<CloudPtr> &obj_cloud, std::vector<cubParam> &cubs)
{
//	pcl::StatisticalOutlierRemoval<PointT> sor;
//	sor.setInputCloud(cloud);
//	sor.setMeanK(50);
//	sor.setStddevMulThresh(1.0);
//	sor.filter(*cloud);

    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-2.0, 2.0);
    //pass.setFilterFieldName("x");
    //pass.setFilterLimits(0.0, 1.0);
    pass.filter(*cloud);

    pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> >(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<PointT, NormalT> normal_estimator;
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(cloud);
    normal_estimator.setKSearch(100);
    normal_estimator.compute(*normals);

    surfaceSegment<PointT, NormalT> reg;
    reg.setMinClusterSize(150);
    reg.setMaxClusterSize(1000000);
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(40);
    reg.setInputCloud(cloud);


    reg.setInputNormals(normals);
    reg.setSmoothnessThreshold(2.5 / 180.0 * M_PI);
    reg.setCurvatureThreshold(1.0);
    std::vector<CloudPtr> seg_cloud;
    std::vector <pcl::PointIndices> clusters;
    reg.extract(seg_cloud, clusters);

    objPartition<PointT, NormalT> obj;
    obj.setInputSurface(seg_cloud);
    obj.objectMerge(obj_cloud);
    cubs = obj.getCub();
    obj.simulation(cubs);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename NormalT> void
multiFrameMerge<PointT, NormalT>::prePartition(CloudPtr &cloud, std::vector<CloudPtr> &obj_cloud)
{
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud);

    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-2.0, 2.0);
    //pass.setFilterFieldName("x");
    //pass.setFilterLimits(0.0, 1.0);
    pass.filter(*cloud);

    pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> >(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<PointT, NormalT> normal_estimator;
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(cloud);
    normal_estimator.setKSearch(100);
    normal_estimator.compute(*normals);

    surfaceSegment<PointT, NormalT> reg;
    reg.setMinClusterSize(300);
    reg.setMaxClusterSize(1000000);
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(40);
    reg.setInputCloud(cloud);


    reg.setInputNormals(normals);
    reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold(1.0);
    std::vector<CloudPtr> seg_cloud;
    std::vector <pcl::PointIndices> clusters;
    reg.extract(seg_cloud, clusters);

    objPartition<PointT, NormalT> obj;
    obj.setInputSurface(seg_cloud);
    obj.objectMerge(obj_cloud);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename NormalT> void
multiFrameMerge<PointT, NormalT>::surfacePartition(CloudPtr &cloud, std::vector<CloudPtr> &seg_cloud, pcl::PointCloud <pcl::PointXYZRGB>::Ptr &colored_cloud_show)
{
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud);

    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-2.0, 2.0);
    //pass.setFilterFieldName("x");
    //pass.setFilterLimits(0.0, 1.0);
    pass.filter(*cloud);

    pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> >(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(cloud);
    normal_estimator.setKSearch(100);
    normal_estimator.compute(*normals);

    surfaceSegment<PointT, NormalT> reg;
    reg.setMinClusterSize(150);
    reg.setMaxClusterSize(1000000);
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(40);
    reg.setInputCloud(cloud);


    reg.setInputNormals(normals);
    reg.setSmoothnessThreshold(2.5 / 180.0 * M_PI);
    reg.setCurvatureThreshold(1.0);
    std::vector <pcl::PointIndices> clusters;
    reg.extract(seg_cloud, clusters);
    colored_cloud_show = reg.getColoredCloud(clusters);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename NormalT> void
multiFrameMerge<PointT, NormalT>::joinMerge(std::vector<std::vector<int> > &adjacency, std::set<std::set<int> > &circle_relationship)
{
    std::vector<std::vector<int> > joinRelationshipVec;
    joinRelationshipVec = matrix_buer(adjacency);
    std::set<int> relationship;
    for (int i = 0; i < joinRelationshipVec.size(); ++i)
    {
        relationship.clear();
        relationship.insert(i);
        for (int j = 0; j < joinRelationshipVec.size(); ++j)
        {
            if (joinRelationshipVec[i][j] == 1)
                relationship.insert(j);
        }
        circle_relationship.insert(relationship);
    }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename NormalT> std::vector<std::vector<int> >
multiFrameMerge<PointT, NormalT>::matrix_buer(std::vector<std::vector<int> > &adjacency)
{
    std::vector<std::vector<int> > adjacency_mid(adjacency.size(), std::vector<int>(adjacency.size(), 0));
    int sum = 0;
    int sum_adjacency = 1;

    while (sum - sum_adjacency != 0)
    {
        sum = 0;
        sum_adjacency = 0;
        for (int i = 0; i < adjacency.size(); ++i)
        {
            for (int j = 0; j < adjacency[i].size(); ++j)
            {
                adjacency_mid[i][j] = and_matrix(adjacency[i], adjacency[j]);
                sum += adjacency[i][j];
                sum_adjacency += adjacency_mid[i][j];
            }
        }
        adjacency = adjacency_mid;
    }

    return adjacency_mid;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename NormalT> int
multiFrameMerge<PointT, NormalT>::and_matrix(std::vector<int> &adjacency1, std::vector<int> &adjacency2)
{
    std::vector<int> m(adjacency1.size(), 0);
    for (int i = 0; i < adjacency1.size(); ++i)
    {
        m[i] = adjacency1[i]>adjacency2[i] ? adjacency2[i] : adjacency1[i];
    }
    int max_num = *max_element(m.begin(), m.end());
    return max_num;
}
#endif

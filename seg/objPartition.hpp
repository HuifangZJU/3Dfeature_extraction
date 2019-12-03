#ifndef _CONNECT_RELATIONSHIP_HPP_
#define _CONNECT_RELATIONSHIP_HPP_

#include "objPartition.h"

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename NormalT>
objPartition<PointT, NormalT>::objPartition():
surface_normal_(),
surface_centroid_(),
convexhull_points_(),
adjacency_(),
circle_relationship_(),
surface_(),
surface_out_(),
plane_()
{
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename NormalT>
objPartition<PointT, NormalT>::~objPartition()
{
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename NormalT> void
objPartition<PointT, NormalT>::setInputSurface(std::vector<CloudPtr> &surface)
{
    surface_.clear();
    for (size_t i = 0; i < surface.size(); ++i)
    {
        if (surface[i]->points.size() < 30000 && surface[i]->points.size() > 0)
            surface_.push_back(surface[i]);
        else
            surface_out_.push_back(surface[i]);
    }

}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename NormalT> void
objPartition<PointT, NormalT>::computeConvexhull()
{

    convexhull_points_.clear();
    //std::cout << "region cloud points number:" << hull << "**:" << seg_cloud[hull]->points.size() << std::endl;
    pcl::ConvexHull<PointT> chull;
    //compute every cluster convex_hull points
    for (int hull = 0; hull < surface_.size(); ++hull)
    {
        CloudPtr cloud_hull(new CloudT);
        chull.setInputCloud(surface_[hull]);
        chull.reconstruct(*cloud_hull);
        convexhull_points_.push_back(cloud_hull);
        //std::cout << "the size of cloud hull" << cloud_hull->points.size() << std::endl;
    }
    //for (int i = 0; i < convexhull_points_.size(); ++i)
    //{
    //	std::cout << "the size of convexhull" << convexhull_points_.size() <<std::endl;
    //	for (int j = 0; j < convexhull_points_[i]->points.size(); ++j)
    //	{
    //		std::cout << "(x,y,z):" << convexhull_points_[i]->points[j].x << convexhull_points_[i]->points[j].y << convexhull_points_[i]->points[j].z << std::endl;
    //	}
    //}
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename NormalT> void
objPartition<PointT, NormalT>::surfaceFeature()
{
    surface_normal_.clear();
    surface_centroid_.clear();
    surface_size_.clear();
    NormalT source_normal;
    PointT source_centroid(0, 0, 0);

    //compute the plane coefficients
    CloudPtr cloudFilter(new CloudT);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.01);
    pcl::ExtractIndices<PointT> extract;

    pcl::MomentOfInertiaEstimation <PointT> feature_extractor;
    for (int i = 0; i < surface_.size(); ++i)
    {
        std::vector<double> surface_size;
        cloudFilter->clear();
        seg.setInputCloud(surface_[i]);
        seg.segment(*inliers, *coefficients);

        extract.setInputCloud(surface_[i]);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloudFilter);

        source_normal.normal_x = coefficients->values[0];
        source_normal.normal_y = coefficients->values[1];
        source_normal.normal_z = coefficients->values[2];
        source_normal.curvature = coefficients->values[3];
        for (int i = 0; i < cloudFilter->points.size(); ++i)
        {
            source_centroid.x += cloudFilter->points[i].x;
            source_centroid.y += cloudFilter->points[i].y;
            source_centroid.z += cloudFilter->points[i].z;
        }
        source_centroid.x /= cloudFilter->points.size();
        source_centroid.y /= cloudFilter->points.size();
        source_centroid.z /= cloudFilter->points.size();
        feature_extractor.setInputCloud(cloudFilter);
        feature_extractor.compute();
        //compute the width and height of the plane cloud
        PointT min_point_OBB;
        PointT max_point_OBB;
        PointT position_OBB;
        Eigen::Matrix3f rotational_matrix_OBB;
        float major_value, middle_value, minor_value;
        Eigen::Vector3f major_vector, middle_vector, minor_vector;
        Eigen::Vector3f mass_center;
        feature_extractor.getEigenValues(major_value, middle_value, minor_value);
        feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
        feature_extractor.getMassCenter(mass_center);
        feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
        std::set<double> dimensions;
        double width = max_point_OBB.x - min_point_OBB.x;
        double height = max_point_OBB.y - min_point_OBB.y;
        double depth = max_point_OBB.z - min_point_OBB.z;
        dimensions.insert(width);
        dimensions.insert(depth);
        dimensions.insert(height);
        std::cout << "the size of surface:" << width << ", " << height << ", " << depth << std::endl;
        std::set<double>::iterator it_dimensions;
        it_dimensions = dimensions.end();
        surface_size.push_back(width);

        surface_size.push_back(height);

        pcl::flipNormalTowardsViewpoint(source_centroid, 0, 0, 0, source_normal.normal_x, source_normal.normal_y, source_normal.normal_z);
        surface_normal_.push_back(source_normal);
        surface_centroid_.push_back(source_centroid);
        surface_size_.push_back(surface_size);
        plane_.push_back(cloudFilter);
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename NormalT> bool
objPartition<PointT, NormalT>::computeConvexCon(int surface_source, int surface_target)
{

    Eigen::Vector3f source_normal(surface_normal_[surface_source].normal_x, surface_normal_[surface_source].normal_y, surface_normal_[surface_source].normal_z);
    Eigen::Vector3f target_normal(surface_normal_[surface_target].normal_x, surface_normal_[surface_target].normal_y, surface_normal_[surface_target].normal_z);


    Eigen::Vector3f source_centroid(surface_centroid_[surface_source].x, surface_centroid_[surface_source].y, surface_centroid_[surface_source].z);
    Eigen::Vector3f target_centroid(surface_centroid_[surface_target].x, surface_centroid_[surface_target].y, surface_centroid_[surface_target].z);

    source_normal = source_normal.normalized();
    target_normal = target_normal.normalized();

    bool is_convex = true;
    //bool is_smooth = true;

    float normal_angle = getAngle3D(source_normal, target_normal, true);
    //  Geometric comparisons
    Eigen::Vector3f vec_t_to_s, vec_s_to_t;

    vec_t_to_s = source_centroid - target_centroid;
    vec_s_to_t = -vec_t_to_s;

    Eigen::Vector3f ncross;
    ncross = source_normal.cross(target_normal);

    //float expected_distance = ncross.norm() * 0.05;
    //float dot_p_1 = vec_t_to_s.dot(source_normal);
    //float dot_p_2 = vec_s_to_t.dot(target_normal);
    //float point_dist = (std::fabs(dot_p_1) < std::fabs(dot_p_2)) ? std::fabs(dot_p_1) : std::fabs(dot_p_2);
    //const float dist_smoothing = 0.02;  // This is a slacking variable especially important for patches with very similar normals
    ////std::cout << "point_dist:" << point_dist << "_";
    //if (point_dist > (expected_distance + dist_smoothing))
    //{
    //	//std::cout << "point_dist:" << point_dist << "_" << (expected_distance + dist_smoothing) << std::endl;
    //	is_smooth &= false;
    //}

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

    float source_angle = getAngle3D(source_normal, vec_t_to_s, true);
    float target_angle = getAngle3D(target_normal, vec_t_to_s, true);

    // vec_t_to_s is the reference direction for angle measurements
    // Convexity Criterion: Check if connection of patches is convex. If this is the case the two supervoxels should be merged.
    if ((getAngle3D(vec_t_to_s, source_normal, true) - getAngle3D(vec_t_to_s, target_normal, true)) <= -10.0)
    {
        is_convex &= true;  // connection convex
        //std::cout << "connection is convex!" << std::endl;
    }
    else
    {
        //std::cout << "normal_angle:" << normal_angle << std::endl;
        //compute the source_surface to the vec_t_to_s angle and the target_surface to the vec_t_to_s surface normal
        std::cout << "the id of surface:" << surface_source << "_" << surface_target << std::endl;
        float source_angle = getAngle3D(source_normal, vec_t_to_s, true);
        float target_angle = getAngle3D(target_normal, vec_t_to_s, true);
        std::cout << "source angle:" << source_angle << "_" << target_angle << std::endl;
        std::cout << "the abs of angle:" << std::fabs(source_angle - 90.0) << "_" << std::fabs(target_angle - 90.0) << std::endl;
        is_convex &= (normal_angle < 10.)&&(std::fabs(source_angle - 90.0) < 5.0) && (std::fabs(target_angle - 90.0) < 5.0);  // concave connections will be accepted  if difference of normals is small
    }
    //std::cout << "finished the computeConvexCon computation!" << std::endl;
    return is_convex;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename NormalT> bool
objPartition<PointT, NormalT>::connectConvexhull(int surface_source, int surface_target)
{
    int point_num = convexhull_points_[surface_source]->points.size() / 5 >convexhull_points_[surface_target]->points.size() / 5?
                convexhull_points_[surface_target]->points.size() / 5:convexhull_points_[surface_source]->points.size() / 5;
    //std::cout << "the size of all points number:" << point_num << std::endl;
    std::vector<double> distance;
    //std::cout << "convexhulll size:" << convexhull_points_.size() << std::endl;
    for (int i = 0; i < convexhull_points_[surface_source]->points.size(); ++i)
    {

        for (int j = 0; j < convexhull_points_[surface_target]->points.size(); ++j)
        {
            double distance_point = sqrt((convexhull_points_[surface_source]->points[i].x - convexhull_points_[surface_target]->points[j].x)*(convexhull_points_[surface_source]->points[i].x - convexhull_points_[surface_target]->points[j].x)
                + (convexhull_points_[surface_source]->points[i].y - convexhull_points_[surface_target]->points[j].y)*(convexhull_points_[surface_source]->points[i].y - convexhull_points_[surface_target]->points[j].y)
                + (convexhull_points_[surface_source]->points[i].z - convexhull_points_[surface_target]->points[j].z)*(convexhull_points_[surface_source]->points[i].z - convexhull_points_[surface_target]->points[j].z));
            //std::cout << distance_point << "_";
            distance.push_back(distance_point);
        }
    }


    //std::cout << std::endl;
    double distance_sum = 0;
    std::sort(distance.begin(), distance.end(), cmp());
    //for (int i = 0; i < distance.size(); ++i)
    //{
    //	if (distance[i] > 0)
    //		std::cout << distance[i] << "_";
    //}
    //std::cout << "min_distance:" << distance[0].first << "***" << distance[1].first <<std::endl;
    for (int i = 0; i < point_num; ++i)
    {
        distance_sum += distance[i];
        //std::cout << "distance:" << distance[i] << "_";
    }
    //std::cout << "distance_sum:" << distance_sum << std::endl;

    //std::cout << "finished the connectConvexhull computation!" << std::endl;
    if (distance_sum < 0.01*point_num)
        return true;
    else
        return false;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename NormalT> void
objPartition<PointT, NormalT>::connectionVec()
{
    adjacency_.clear();

    adjacency_.resize(surface_.size(), std::vector<int>(surface_.size(), 0));
    //std::cout << "the size of adjacency:" << adjacency_.size() << std::endl;
    //std::cout << "fuck the size of surface normal:" << surface_normal_.size() << " don't equal to surface size:" << surface_.size() << std::endl;
    //std::cout << "the size of convexhull:" << convexhull_points_.size() << std::endl;
    for (size_t i = 0; i < adjacency_.size(); ++i)
    {
        for (size_t j = i + 1; j < adjacency_.size(); ++j)
        {
            if (connectConvexhull(i, j))
            {
                if (computeConvexCon(i, j))
                {
                    adjacency_[i][j] = 1;
                    adjacency_[j][i] = 1;
                }
                else
                {
                    adjacency_[i][j] = 0;
                    adjacency_[j][i] = 0;
                }

            }
            else
            {
                adjacency_[i][j] = 0;
                adjacency_[j][i] = 0;
            }
        }
    }
    for (size_t i = 0; i < adjacency_.size(); ++i)
        adjacency_[i][i] = 1;

    //std::ofstream outFile;
    //outFile.open("adjact_file.txt");
    //for (int i = 0; i < adjacency_.size(); ++i)
    //{
    //	for (int j = 0; j < adjacency_[i].size(); ++j)
    //	{
    //		outFile << adjacency_[i][j] << " ";
    //	}
    //	outFile << "\n";
    //}
    //outFile.close();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename NormalT> std::vector<std::vector<int> >
objPartition<PointT, NormalT>::matrix_bool()
{
    std::vector<std::vector<int> > adjacency_mid(adjacency_.size(), std::vector<int>(adjacency_.size(), 0));
    int sum = 0;
    int sum_adjacency = 1;

    while (sum - sum_adjacency != 0)
    {
        sum = 0;
        sum_adjacency = 0;
        for (int i = 0; i < adjacency_.size(); ++i)
        {
            for (int j = 0; j < adjacency_.size(); ++j)
            {
                adjacency_mid[i][j] = and_matrix(adjacency_[i], adjacency_[j]);
                sum += adjacency_[i][j];
                sum_adjacency += adjacency_mid[i][j];
            }
        }
        adjacency_ = adjacency_mid;
    }

    return adjacency_mid;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename NormalT> double
objPartition<PointT, NormalT>::getAngle3D(const Eigen::Vector3f &v1, const Eigen::Vector3f &v2, const bool in_degree)
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
template<typename PointT, typename NormalT> void
objPartition<PointT, NormalT>::joinMerge()
{
    circle_relationship_.clear();
    std::vector<std::vector<int> > joinRelationshipVec;
    joinRelationshipVec = matrix_bool();

    for (int i = 0; i < joinRelationshipVec.size(); ++i)
    {
        std::set<int> relationship;
        relationship.insert(i);
        for (int j = 0; j < joinRelationshipVec.size(); ++j)
        {
            if (joinRelationshipVec[i][j] == 1)
                relationship.insert(j);
        }
        circle_relationship_.insert(relationship);
    }

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename NormalT> int
objPartition<PointT, NormalT>::and_matrix(std::vector<int> &adjacency1, std::vector<int> &adjacency2)
{
    std::vector<int> m(adjacency1.size(), 0);
    for (int i = 0; i < adjacency1.size(); ++i)
    {
        m[i] = adjacency1[i]>adjacency2[i] ? adjacency2[i] : adjacency1[i];
    }
    int max_num = *max_element(m.begin(), m.end());
    return max_num;

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename NormalT> void
objPartition<PointT, NormalT>::circleMerge()
{
//	circle_relationship_.clear();
//	std::cout << "circle size" << adjacency_.size() << std::endl;
//	std::set<int> relationship;
//	for (unsigned int i = 0; i < adjacency_.size(); ++i)
//	{

//		std::vector<int> nodei;
//		for (unsigned int ii = 0; ii < adjacency_.size(); ++ii)
//		{
//			if (adjacency_[i][ii] == 1)
//			{
//				nodei.push_back(ii);
//			}
//		}
//		if (nodei.size() == 1)
//		{
//			//std::cout << "nodei.size:" << nodei.size() << "the number of i: " << nodei[0] << std::endl;
//			relationship.clear();
//			relationship.insert(i);
//			circle_relationship_.insert(relationship);
//		}
//		else
//		{
//			for (unsigned int j = 0; j < nodei.size(); ++j)
//			{
//				std::vector<int> nodej;
//				for (unsigned int jj = 0; jj < adjacency_.size(); ++jj)
//				{
//					if (adjacency[nodei[j]][jj] == 1)
//					{
//						nodej.push_back(jj);
//					}
//				}
//				if (nodej.size() == 2 && i != nodei[j])
//				{
//					relationship.clear();
//					relationship.insert(i);
//					relationship.insert(nodei[j]);
//					circle_relationship_.insert(relationship);
//				}
//				else
//				{
//					for (unsigned int m = 0; m < nodej.size(); ++m)
//					{
//						std::vector<int> nodem;
//						for (unsigned int mm = 0; mm < adjacency_.size(); ++mm)
//						{
//							if (adjacency[nodej[m]][mm] == 1)
//							{
//								nodem.push_back(mm);
//							}
//						}
//						if (std::find(nodem.begin(), nodem.end(), i) != nodem.end() && i != nodei[j] && i != nodej[m] && nodei[j] != nodej[m])
//						{
//							relationship.clear();
//							relationship.insert(i);
//							relationship.insert(nodei[j]);
//							relationship.insert(nodej[m]);
//							circle_relationship_.insert(relationship);
//						}
//						/*else if (adjacency[nodei[j]][i] == 1 && i != nodei[j] && std::find(nodej.begin(), nodej.end(), nodei[j]) != nodej.end())
//						{
//						relationship.clear();
//						relationship.insert(i);
//						relationship.insert(nodei[j]);
//						circle_relationship.insert(relationship);
//						}
//						else
//						{
//						relationship.clear();
//						relationship.insert(i);
//						circle_relationship.insert(relationship);
//						}*/
//					}
//				}

//			}
//		}


//	}
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename NormalT> void
objPartition<PointT, NormalT>::objectMerge(std::vector<CloudPtr> &object)
{

    surfaceFeature();
    //std::cout << "surfaceFeature finished!" << std::endl;
    computeConvexhull();
    //std::cout << "computeConvexhull finished!" << std::endl;
    connectionVec();
    //std::cout << "connectionVec finished!" << std::endl;
    joinMerge();
    //std::cout << "joinMerge finished!" << std::endl;
    object.clear();
    std::set<std::set<int> >::iterator circle_i;
    std::set<int>::iterator circle_j;

    for (circle_i = circle_relationship_.begin(); circle_i != circle_relationship_.end(); ++circle_i)
    {
        CloudPtr cloudi(new CloudT);
        for (circle_j = (*circle_i).begin(); circle_j != (*circle_i).end(); ++circle_j)
        {
            for (int i = 0; i < surface_[*circle_j]->points.size(); ++i)
            {
                PointT point;
                point.x = surface_[*circle_j]->points[i].x;
                point.y = surface_[*circle_j]->points[i].y;
                point.z = surface_[*circle_j]->points[i].z;
                cloudi->points.push_back(point);
            }
        }
        object.push_back(cloudi);
    }
    for (int i = 0; i < surface_out_.size(); ++i)
        object.push_back(surface_out_[i]);

}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename NormalT> void
objPartition<PointT, NormalT>::getObjCloud(std::string filePath)
{
    std::stringstream fileName;
    for (unsigned int i = 0; i < surface_.size(); ++i)
    {
        fileName.str("");
        fileName << filePath << std::setfill('0') << std::setw(6) << i << ".pcd";
        //pcl::io::savePCDFileASCII(fileName.str(), *(surface_[i]));
        /*pcl::PCDWriter writer;
        writer.write(fileName.str(), *surface_[i]);*/
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename NormalT> void
objPartition<PointT, NormalT>::objColored(std::vector<CloudPtr> &object, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &obj_colored)
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
/////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename NormalT> void
objPartition<PointT, NormalT>::getPlaneParams(std::vector<CloudPtr> &cloud, std::vector<planeParam> &plane_param)
{
    //std::vector<planeParam> plane_param;
    plane_param.reserve(cloud.size());
    for (size_t i = 0; i < cloud.size(); ++i)
    {
        planeParam singleParam;
        pcl::MomentOfInertiaEstimation <PointT> feature_extractor;

        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        // Optional
        seg.setOptimizeCoefficients(true);
        // Mandatory
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.01);

        seg.setInputCloud(cloud[i]);
        seg.segment(*inliers, *coefficients);
        // Project the model inliers
        CloudPtr cloud_projected(new CloudT);
        pcl::ProjectInliers<pcl::PointXYZ> proj;
        proj.setModelType(pcl::SACMODEL_PLANE);
        proj.setInputCloud(cloud[i]);
        proj.setIndices(inliers);
        proj.setModelCoefficients(coefficients);
        proj.filter(*cloud_projected);

        //get the plane coefficients Ax+By+Cz+D=0;
        singleParam.normal(0)=(coefficients->values[0]);
        singleParam.normal(1)=(coefficients->values[1]);
        singleParam.normal(2)=(coefficients->values[2]);
        //singleParam.normal.push_back(coefficients->values[3]);
        feature_extractor.setInputCloud(cloud_projected);
        feature_extractor.compute();
        //std::cout << "*" << std::endl;
        std::vector <float> moment_of_inertia;
        std::vector <float> eccentricity;
        PointT min_point_OBB;
        PointT max_point_OBB;
        PointT position_OBB;
        Eigen::Matrix3f rotational_matrix_OBB;
        float major_value, middle_value, minor_value;
        Eigen::Vector3f major_vector, middle_vector, minor_vector;
        Eigen::Vector3f mass_center;
        feature_extractor.getMomentOfInertia(moment_of_inertia);
        feature_extractor.getEccentricity(eccentricity);
        feature_extractor.getEigenValues(major_value, middle_value, minor_value);
        feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
        feature_extractor.getMassCenter(mass_center);
        feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
        Eigen::Vector3f position(position_OBB.x, position_OBB.y, position_OBB.z);
        Eigen::Quaternionf quat(rotational_matrix_OBB);
        std::set<float> dimensions;
        float width = max_point_OBB.x - min_point_OBB.x;
        float height = max_point_OBB.y - min_point_OBB.y;
        float depth = max_point_OBB.z - min_point_OBB.z;
        dimensions.insert(width);
        dimensions.insert(depth);
        dimensions.insert(height);
        std::set<float>::iterator it_dimensions;
        it_dimensions = dimensions.end();
        singleParam.width = *it_dimensions;
        singleParam.height = *(--it_dimensions);


        singleParam.center.push_back(mass_center(0));
        singleParam.center.push_back(mass_center(1));
        singleParam.center.push_back(mass_center(2));
        plane_param.push_back(singleParam);
    }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename NormalT> std::vector<cubParam>
objPartition<PointT, NormalT>::getSurfaceCub()
{
    std::vector<cubParam> cubsParam;
    CloudPtr cloudFilter(new CloudT);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    //Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.01);
    pcl::ExtractIndices<PointT> extract;
    pcl::MomentOfInertiaEstimation <PointT> feature_extractor;
    for (int i = 0; i < surface_.size(); ++i)
    {
        seg.setInputCloud(surface_[i]);
        seg.segment(*inliers, *coefficients);

        extract.setInputCloud(surface_[i]);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloudFilter);

        feature_extractor.setInputCloud(cloudFilter);
        feature_extractor.compute();
        //compute the width and height of the plane cloud
        std::vector <float> moment_of_inertia;
        std::vector <float> eccentricity;
        pcl::PointXYZ min_point_AABB;
        pcl::PointXYZ max_point_AABB;
        pcl::PointXYZ min_point_OBB;
        pcl::PointXYZ max_point_OBB;
        pcl::PointXYZ position_OBB;
        Eigen::Matrix3f rotational_matrix_OBB;
        float major_value, middle_value, minor_value;
        Eigen::Vector3f major_vector, middle_vector, minor_vector;
        Eigen::Vector3f mass_center;

        feature_extractor.getMomentOfInertia (moment_of_inertia);
        feature_extractor.getEccentricity (eccentricity);
        feature_extractor.getAABB (min_point_AABB, max_point_AABB);
        feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
        feature_extractor.getEigenValues (major_value, middle_value, minor_value);
        feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
        feature_extractor.getMassCenter (mass_center);
        feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);

        cubParam single_cub;

        Eigen::Vector3f position(position_OBB.x, position_OBB.y, position_OBB.z);
        Eigen::Quaternionf quat(rotational_matrix_OBB);
        float width = max_point_OBB.x - min_point_OBB.x;
        float height = max_point_OBB.y - min_point_OBB.y;
        float depth = max_point_OBB.z - min_point_OBB.z;

        single_cub.depth = depth;
        single_cub.width = width;
        single_cub.height = height;
        single_cub.rotation = quat;
        single_cub.position = position;
        cubsParam.push_back(single_cub);

    }
    return cubsParam;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename NormalT> std::vector<cubParam>
objPartition<PointT, NormalT>::getCub()
{
    std::set<std::set<int> >::iterator circle_i;
    std::set<int>::iterator circle_j;
    std::vector<cubParam> cubsParam;

    cubsParam.reserve(circle_relationship_.size());
    int cub_num = 0;
    for (circle_i = circle_relationship_.begin(); circle_i != circle_relationship_.end(); ++circle_i)
    {
//        std::vector<std::pair<int, int> > object_surface_size;
//        for(circle_j = (*circle_i).begin(); circle_j != (*circle_i).end(); ++circle_j)
//        {
//            std::pair<int, int> surface_index_size;
//            surface_index_size.first = *circle_j;
//            surface_index_size.second = plane_[*circle_j]->points.size();
//            object_surface_size.push_back(surface_index_size);
//        }
//        std::sort(object_surface_size.begin(), object_surface_size.end(), sortVec);
//        Eigen::Vector3f z_axis;
//        z_axis << 0, 0, 1;
//        if(object_surface_size.size() > 1)
//        {
//            CloudPtr major_plane(new CloudT);
//            CloudPtr middle_plane(new CloudT);
//            int major_plane_index = object_surface_size[0].first;//the max plane of a object
//            int middle_plane_index = object_surface_size[1].first;//the middle plane of a object
//            Eigen::Vector3f major_normal;//z axis normal direction
//            major_normal << surface_normal_[major_plane_index].normal_x, surface_normal_[major_plane_index].normal_y, surface_normal_[major_plane_index].normal_z;
//            double t = std::acos(major_normal(2));
//            Eigen::Vector3f u = major_normal.cross(z_axis)/(major_normal.cross(z_axis)).norm();
//            Eigen::Matrix3f rot;
//            rot << std::cos(t), -std::sin(t)*u(2), std::sin(t)*u(1),
//                    std::sin(t)*u(2), std::cos(t), -std::sin(t)*u(0),
//                    -std::sin(t)*u(1), std::sin(t)*u(0), std::cos(t);

//            Eigen::Matrix4f trans;
//            trans.setIdentity();
//            trans.block<3, 3> = rot;
//            //Trans.rightCols<1>() = x;
//            pcl::transformPointCloud(plane_[major_plane_index], major_plane, trans);
//            double major_z_mean;
//            for(unsigned plane_i = 0; plane_i < major_plane->points.size(); ++palne_i)
//            {
//                major_z_mean += major_plane->points[plane_i].z;
//            }
//            major_z_mean /= major_plane->points.size();
//            Eigen::Vector3f x;
//            x << 0,0,-major_z_mean;
//            trans.rightCols<1>() = x;
//            pcl::transformPointCloud(plane_[major_plane_index], major_plane, trans);
//            pcl::transformPointCloud(plane_[middle_plane_index], middle_plane, trans);

//            Eigen::Vector3f middle_normal;//z axis normal direction
//            middle_normal << surface_normal_[middle_plane_index].normal_x, surface_normal_[middle_plane_index].normal_y, surface_normal_[middle_plane_index].normal_z;
//            middle_normal = rot*middle_normal;
//            double t2 = std::atan2(middle_normal(1), middle_normal(0));
//            double c2 = std::cos(t2), s2 = std::sin(t2);
//            Eigen::Matrix3f rot2;
//            rot2 << c, -s, 0, s, c, 0, 0, 0, 1;
//            Eigen::Matrix4f trans2;
//            trans2.setIdentity();
//            trans2.block<3, 3> = rot2;
//            pcl::transformPointCloud(major_plane, major_plane, trans2);
//            pcl::transformPointCloud(middle_plane, middle_plane, trans2);
//        }
        //check the circle which the box is compiled with three plane
        if ((*circle_i).size() == 3)
        {
            std::cout << "in the circle is " << cub_num <<", and the size is 3" << std::endl;
            std::vector<std::pair<double, int> > sizeCub;
            Eigen::MatrixXf M(6, 3);
            M = Eigen::MatrixXf::Zero(6, 3);
            Eigen::MatrixXf D(6, 1);
            D = Eigen::MatrixXf::Zero(6, 1);
            Eigen::Vector3f cub_center;
            cubParam single_cub;
            circle_j = (*circle_i).begin();
            //first plane param
            M(0, 0) = surface_normal_[*circle_j].normal_y;
            M(0, 1) = -surface_normal_[*circle_j].normal_x;
            M(1, 1) = surface_normal_[*circle_j].normal_z;
            M(1, 2) = -surface_normal_[*circle_j].normal_y;
            //center.y*normal.x-center.x*normal.y
            D(0, 0) = surface_normal_[*circle_j].normal_x * surface_centroid_[*circle_j].y - surface_normal_[*circle_j].normal_y * surface_centroid_[*circle_j].x;
            D(1, 0) = surface_normal_[*circle_j].normal_y * surface_centroid_[*circle_j].z - surface_normal_[*circle_j].normal_z * surface_centroid_[*circle_j].y;
            //the plane size
            std::pair<double, int> singleSize;
            singleSize.first = surface_size_[*circle_j][0];
            singleSize.second = *circle_j;
            sizeCub.push_back(singleSize);
            //std::cout << "get it!" << std::endl;
            singleSize.first = surface_size_[*circle_j][1];
            singleSize.second = *circle_j;
            sizeCub.push_back(singleSize);
            //second plane param
            ++circle_j;
            M(2, 0) = surface_normal_[*circle_j].normal_y;
            M(2, 1) = -surface_normal_[*circle_j].normal_x;
            M(3, 1) = surface_normal_[*circle_j].normal_z;
            M(3, 2) = -surface_normal_[*circle_j].normal_y;
            D(2, 0) = surface_normal_[*circle_j].normal_x * surface_centroid_[*circle_j].y - surface_normal_[*circle_j].normal_y * surface_centroid_[*circle_j].x;
            D(3, 0) = surface_normal_[*circle_j].normal_y * surface_centroid_[*circle_j].z - surface_normal_[*circle_j].normal_z * surface_centroid_[*circle_j].y;
            //std::cout << "plane2 size:" << plane_param[*circle_j].center[0] << "_" << plane_param[*circle_j].center[1] << "_" << plane_param[*circle_j].center[0] << endl;
            singleSize.first = surface_size_[*circle_j][0];
            singleSize.second = *circle_j;
            sizeCub.push_back(singleSize);

            singleSize.first = surface_size_[*circle_j][1];
            singleSize.second = *circle_j;
            sizeCub.push_back(singleSize);
            //third plane param
            ++circle_j;
            M(4, 0) = surface_normal_[*circle_j].normal_y;
            M(4, 1) = -surface_normal_[*circle_j].normal_x;
            M(5, 1) = surface_normal_[*circle_j].normal_z;
            M(5, 2) = -surface_normal_[*circle_j].normal_y;
            D(4, 0) = surface_normal_[*circle_j].normal_x * surface_centroid_[*circle_j].y - surface_normal_[*circle_j].normal_y * surface_centroid_[*circle_j].x;
            D(5, 0) = surface_normal_[*circle_j].normal_y * surface_centroid_[*circle_j].z - surface_normal_[*circle_j].normal_z * surface_centroid_[*circle_j].y;
            //std::cout << "plane3 size:" << plane_param[*circle_j].center[0] << "_" << plane_param[*circle_j].center[1] << "_" << plane_param[*circle_j].center[0] << endl;
            singleSize.first = surface_size_[*circle_j][0];
            singleSize.second = *circle_j;
            sizeCub.push_back(singleSize);

            singleSize.first = surface_size_[*circle_j][1];
            singleSize.second = *circle_j;
            sizeCub.push_back(singleSize);
            //std::cout << "fuck1" << std::endl;
            //caculate the box size about the width, height, and depth
            std::sort(sizeCub.begin(), sizeCub.end(), sortVec);
            single_cub.depth = (sizeCub[0].first + sizeCub[1].first) / 2;//the minor
            single_cub.height = (sizeCub[2].first + sizeCub[3].first) / 2;//the middle
            single_cub.width = (sizeCub[4].first + sizeCub[5].first) / 2;//the max
            std::vector< int > histogram(surface_normal_.size(), 0);
            //std::cout << "fuck2" << std::endl;
            for (int ii = 2; ii < sizeCub.size(); ++ii)
            {
                histogram[sizeCub[ii].second]++;
            }
            int major = std::max_element(histogram.begin(), histogram.end()) - histogram.begin();
            int middle = (major == sizeCub[4].second) ? sizeCub[5].second : sizeCub[4].second;
            int minor = (middle == sizeCub[0].second) ? sizeCub[1].second : sizeCub[0].second;
            //std::cout << "fuck3" << std::endl;
            //caculate the box center
            cub_center = -M.transpose()*D*(M.transpose()*M).inverse();
            Eigen::Matrix3f rotational;
            //std::cout << "fuck4" << std::endl;
            rotational << surface_normal_[major].normal_x, surface_normal_[middle].normal_x, surface_normal_[minor].normal_x,
                surface_normal_[major].normal_y, surface_normal_[middle].normal_y, surface_normal_[minor].normal_y,
                surface_normal_[major].normal_z, surface_normal_[middle].normal_z, surface_normal_[minor].normal_z;
            std::cout << "fuck5" << std::endl;
            Eigen::Quaternionf quat(rotational);
            single_cub.position = cub_center;
            single_cub.rotation = quat;
            std::cout << "fuck6" << std::endl;
//            single_cub.axis.push_back(major_axis);
//            single_cub.axis.push_back(middle_axis);
//            single_cub.axis.push_back(minor_axis);
            cubsParam.push_back(single_cub);
            std::cout << "cub's width3:" << single_cub.width << "___" << "cub's height:" << single_cub.height << "___" << single_cub.depth << std::endl;
            std::cout << "cub's center:" << single_cub.position << std::endl;
            std::cout << "cub's rotation:" << single_cub.rotation.x() << "__" << single_cub.rotation.y() << "__" << single_cub.rotation.z() << "___" << single_cub.rotation.w() << std::endl;
        }
        //only two plane compile the box
        else if ((*circle_i).size() == 2)
        {
            std::cout << "in the circle is " << cub_num <<", and the size is 2" << std::endl;
            std::vector<std::pair<double, int> > sizeCub;
            Eigen::MatrixXf M(4, 3);
            M = Eigen::MatrixXf::Zero(4, 3);
            Eigen::MatrixXf D(4, 1);
            D = Eigen::MatrixXf::Zero(4, 1);
            Eigen::Vector3f cub_center;
            //check the circle which the box is compiled with three plane

            cubParam single_cub;
            circle_j = (*circle_i).begin();
            //first plane param
            M(0, 0) = surface_normal_[*circle_j].normal_y;
            M(0, 1) = -surface_normal_[*circle_j].normal_x;
            M(1, 1) = surface_normal_[*circle_j].normal_z;
            M(1, 2) = -surface_normal_[*circle_j].normal_y;
            //center.y*normal.x-center.x*normal.y
            D(0, 0) = M(0, 0) * surface_centroid_[*circle_j].x +  M(0, 1) * surface_centroid_[*circle_j].y + M(0, 2) * surface_centroid_[*circle_j].z;
            D(1, 0) = M(1, 0) * surface_centroid_[*circle_j].x +  M(1, 1) * surface_centroid_[*circle_j].y + M(1, 2) * surface_centroid_[*circle_j].z;
            //the plane size
            std::pair<double, int> singleSize;
            singleSize.first = surface_size_[*circle_j][0];
            singleSize.second = *circle_j;
            sizeCub.push_back(singleSize);
            //std::cout << "get it!" << std::endl;
            singleSize.first = surface_size_[*circle_j][1];
            singleSize.second = *circle_j;
            std::cout << "the surface size:" << surface_size_[*circle_j][0] << ", " << surface_size_[*circle_j][1] << std::endl;
            sizeCub.push_back(singleSize);
            //second plane param
            ++circle_j;
            M(2, 0) = surface_normal_[*circle_j].normal_y;
            M(2, 1) = -surface_normal_[*circle_j].normal_x;
            M(3, 1) = surface_normal_[*circle_j].normal_z;
            M(3, 2) = -surface_normal_[*circle_j].normal_y;
            D(2, 0) = M(2, 0) * surface_centroid_[*circle_j].x +  M(2, 1) * surface_centroid_[*circle_j].y + M(2, 2) * surface_centroid_[*circle_j].z;
            D(3, 0) = M(3, 0) * surface_centroid_[*circle_j].x +  M(3, 1) * surface_centroid_[*circle_j].y + M(3, 2) * surface_centroid_[*circle_j].z;
            //std::cout << "plane2 size:" << plane_param[*circle_j].center[0] << "_" << plane_param[*circle_j].center[1] << "_" << plane_param[*circle_j].center[0] << endl;
            singleSize.first = surface_size_[*circle_j][0];
            singleSize.second = *circle_j;
            sizeCub.push_back(singleSize);

            singleSize.first = surface_size_[*circle_j][1];
            singleSize.second = *circle_j;
            sizeCub.push_back(singleSize);
            //caculate the box size about the width, height, and depth
            std::cout << "the surface size:" << surface_size_[*circle_j][0] << ", " << surface_size_[*circle_j][1] << std::endl;
            std::sort(sizeCub.begin(), sizeCub.end(), sortVec);
            Eigen::Vector3f major_axis, middle_axis, minor_axis;

//            std::vector<std::pair<double, int> > sizeCub_err;

//            for(unsigned cub_i = 0; cub_i < 3; ++cub_i)
//            {
//                std::pair<double, int> surface_err;
//                surface_err.first = std::fabs(sizeCub[i + 1].first - sizeCub[i].first);
//                surface_err.second = i;
//                sizeCub_err.push_back(surface_err);
//            }
//            std::sort(sizeCub_err.begin(), sizeCub_err.end(), sortVec);

//            //double connect_length = (sizeCub[sizeCub_err[0].second].first + sizeCub[sizeCub_err[0].second + 1].first) / 2;

//            if(sizeCub_err[0].second == 2)
//            {
//                single_cub.depth = sizeCub[0].first ;//the minor
//                single_cub.height = sizeCub[1].first;//the middle
//                single_cub.width = (sizeCub[sizeCub_err[0].second].first + sizeCub[sizeCub_err[0].second + 1].first) / 2;//the max

//                int middle = sizeCub[1].second;
//                int minor = (middle == sizeCub[0].second) ? sizeCub[1].second : sizeCub[0].second;
//                major_axis << surface_normal_[minor].normal_x, surface_normal_[minor].normal_y,surface_normal_[minor].normal_z;
//                middle_axis << surface_normal_[middle].normal_x, surface_normal_[middle].normal_y,surface_normal_[middle].normal_z;
//                major_axis= minor_axis.cross(middle_axis);

////                int middle = sizeCub[1].second;
////                int minor = (middle == sizeCub[0].second) ? sizeCub[1].second : sizeCub[0].second;
////                major_axis << surface_normal_[minor].normal_x, surface_normal_[minor].normal_y,surface_normal_[minor].normal_z;
////                middle_axis << surface_normal_[middle].normal_x, surface_normal_[middle].normal_y,surface_normal_[middle].normal_z;
////                minor_axis= major_axis.cross(middle_axis);
//            }




            if (sizeCub[3].first - sizeCub[2].first > sizeCub[2].first - sizeCub[1].first)
            {
                if(sizeCub[2].first - sizeCub[1].first > sizeCub[1].first - sizeCub[0].first)
                {
                    single_cub.depth = (sizeCub[0].first + sizeCub[1].first) / 2;//the minor
                    single_cub.height = sizeCub[2].first;//the middle
                    single_cub.width = sizeCub[3].first;//the max


                    int middle = sizeCub[3].second;
                    int minor = (middle == sizeCub[0].second) ? sizeCub[1].second : sizeCub[0].second;
                    major_axis << surface_normal_[minor].normal_x, surface_normal_[minor].normal_y,surface_normal_[minor].normal_z;
                    middle_axis << surface_normal_[middle].normal_x, surface_normal_[middle].normal_y,surface_normal_[middle].normal_z;
                    minor_axis= major_axis.cross(middle_axis);
                    std::cout << "size cub fuck0!" << std::endl;
                }
                else
                {
                    single_cub.depth = sizeCub[0].first;//the minor
                    single_cub.height = (sizeCub[1].first + sizeCub[2].first) / 2;//the middle
                    single_cub.width = sizeCub[3].first;//the max

                    int major = sizeCub[3].second;
                    int minor = (major == sizeCub[0].second) ? sizeCub[1].second : sizeCub[0].second;
                    minor_axis << surface_normal_[major].normal_x, surface_normal_[major].normal_y,surface_normal_[major].normal_z;
                    major_axis << surface_normal_[minor].normal_x, surface_normal_[minor].normal_y,surface_normal_[minor].normal_z;
                    middle_axis = minor_axis.cross(major_axis);
                    std::cout << "size cub fuck4!" << std::endl;
                }

            }
            else
            {
                if(sizeCub[3].second !=  sizeCub[2].second)
                {
                    single_cub.depth = sizeCub[0].first;//the minor
                    single_cub.height = sizeCub[1].first;//the middle
                    single_cub.width = (sizeCub[3].first + sizeCub[2].first) / 2;//the max

                    int major = sizeCub[0].second;
                    int middle = (major == sizeCub[2].second) ? sizeCub[3].second : sizeCub[2].second;
//                    major_axis << surface_normal_[major].normal_x, surface_normal_[major].normal_y,surface_normal_[major].normal_z;
//                    middle_axis << surface_normal_[middle].normal_x, surface_normal_[middle].normal_y,surface_normal_[middle].normal_z;
//                    minor_axis = major_axis.cross(middle_axis);
                    major_axis << surface_normal_[major].normal_x, surface_normal_[major].normal_y,surface_normal_[major].normal_z;
                    minor_axis << surface_normal_[middle].normal_x, surface_normal_[middle].normal_y,surface_normal_[middle].normal_z;
                    middle_axis = minor_axis.cross(major_axis);
                    std::cout << "size cub fuck1!" << std::endl;
                }
                else
                {
                    single_cub.depth = sizeCub[0].first;//the minor
                    single_cub.height = 0.7 * sizeCub[1].first + 0.3 * sizeCub[2].first;//the middle
                    single_cub.width = sizeCub[3].first;//the max

                    int major = sizeCub[3].second;
                    int middle = (major == sizeCub[1].second) ? sizeCub[2].second : sizeCub[1].second;
                    minor_axis << surface_normal_[major].normal_x, surface_normal_[major].normal_y,surface_normal_[major].normal_z;
                    major_axis << surface_normal_[middle].normal_x, surface_normal_[middle].normal_y,surface_normal_[middle].normal_z;
                    middle_axis = minor_axis.cross(major_axis);
                    std::cout << "size cub fuck2!" << std::endl;
                }

            }


//            int major = sizeCub[3].second;
//            int middle = (major == sizeCub[1].second) ? sizeCub[2].second : sizeCub[1].second;
            //int minor = sizeCub[0].second;
            //caculate the box center
            cub_center = (M.transpose()*M).inverse()*M.transpose()*D;
            Eigen::Matrix3f rotational;


            rotational << major_axis(0), middle_axis(0), minor_axis(0),
                major_axis(1), middle_axis(1), minor_axis(1),
                major_axis(2), middle_axis(2), minor_axis(2);
            single_cub.axis.push_back(major_axis);
            single_cub.axis.push_back(middle_axis);
            single_cub.axis.push_back(minor_axis);
            Eigen::Quaternionf quat(rotational);
            single_cub.position = cub_center;
            single_cub.rotation = quat;
            cubsParam.push_back(single_cub);
            std::cout << "cub's width2:" << single_cub.width << "___" << "cub's height:" << single_cub.height << "___" << single_cub.depth << std::endl;
            std::cout << "cub's center:" << single_cub.position << std::endl;
            std::cout << "cub's rotation:" << single_cub.rotation.x() << "__" << single_cub.rotation.y() << "__" << single_cub.rotation.z() << "___" << single_cub.rotation.w() << std::endl;
        }
        //only one plane compile the box
        else
        {
            std::cout << "in the circle is " << cub_num <<", and the size is other" << std::endl;
            cubParam single_cub;
            circle_j = (*circle_i).begin();
            pcl::MomentOfInertiaEstimation <PointT> feature_extractor;
            feature_extractor.setInputCloud(surface_[*circle_j]);
            feature_extractor.compute();
            //std::cout << "*" << std::endl;
            std::vector <float> moment_of_inertia;
            std::vector <float> eccentricity;
            PointT min_point_OBB;
            PointT max_point_OBB;
            PointT position_OBB;
            Eigen::Matrix3f rotational_matrix_OBB;
            float major_value, middle_value, minor_value;
            Eigen::Vector3f major_vector, middle_vector, minor_vector;
            feature_extractor.getMomentOfInertia(moment_of_inertia);
            feature_extractor.getEigenValues(major_value, middle_value, minor_value);
            feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
            feature_extractor.getEccentricity(eccentricity);
            feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);

            //viewer.addCube(min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 1.0, 0.0, aabb.str());
            //get very plane's position,transformation,width,height and depth
            Eigen::Vector3f position(position_OBB.x, position_OBB.y, position_OBB.z);
            Eigen::Quaternionf quat(rotational_matrix_OBB);
            double width = max_point_OBB.x - min_point_OBB.x;
            double height = max_point_OBB.y - min_point_OBB.y;
            double depth = max_point_OBB.z - min_point_OBB.z;

            single_cub.axis.push_back(major_vector);
            single_cub.axis.push_back(middle_vector);
            single_cub.axis.push_back(minor_vector);
            single_cub.depth = depth;
            single_cub.width = width;
            single_cub.height = height;
            single_cub.rotation = quat;
            single_cub.position = position;
            cubsParam.push_back(single_cub);
            std::cout << "cub's width1:" << single_cub.width << "___" << "cub's height:" << single_cub.height << "___" << single_cub.depth << std::endl;
            std::cout << "cub's center:" << single_cub.position << std::endl;
            std::cout << "cub's rotation:" << single_cub.rotation.x() << "__" << single_cub.rotation.y() << "__" << single_cub.rotation.z() << "___" << single_cub.rotation.w() << std::endl;
            //std::cout << "single plane can get the params!" << std::endl;
        }
        ++cub_num;
    }
    return cubsParam;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT, typename NormalT> void
objPartition<PointT, NormalT>::simulation(std::vector<cubParam> &cubs_params)
{
    std::ofstream ofs("cubs.data", std::ofstream::out | std::ofstream::binary);

    cv::Mat paramsCub(cubs_params.size(), 9, CV_32FC1);
    for (unsigned int i = 0; i < cubs_params.size(); ++i)
    {
        paramsCub.at<float>(i, 0) = cubs_params[i].width;
        paramsCub.at<float>(i, 1) = cubs_params[i].height;
        paramsCub.at<float>(i, 2) = cubs_params[i].depth;
        paramsCub.at<float>(i, 3) = cubs_params[i].rotation.x();
        paramsCub.at<float>(i, 4) = cubs_params[i].rotation.y();
        paramsCub.at<float>(i, 5) = cubs_params[i].rotation.z();
        paramsCub.at<float>(i, 6) = cubs_params[i].position.x();
        paramsCub.at<float>(i, 7) = cubs_params[i].position.y();
        paramsCub.at<float>(i, 8) = cubs_params[i].position.z();
    }
    ofs.write((const char *)paramsCub.data, paramsCub.rows * paramsCub.step[0]);
    ofs.close();
}

#endif

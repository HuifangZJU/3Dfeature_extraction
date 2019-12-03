#ifndef POINT_TYPES_HF_H
#define POINT_TYPES_HF_H

#define PCL_NO_PRECOMPILE
#include <pcl/PCLPointField.h>
#include <pcl/point_types.h>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/property_map.h>
#include <CGAL/tags.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

struct SemanticPoint
{
    PCL_ADD_POINT4D
    PCL_ADD_RGB
    unsigned int segment;
    unsigned int label;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (SemanticPoint,
                                  (float,x,x)
                                  (float,y,y)
                                  (float,z,z)
                                  (unsigned int,rgba,rgba)
                                  (unsigned int,segment,segment)
                                  (unsigned int,label,label)
)
//PointT: main point cloud type, has six fields, x,y,z,rgba,segment,label
typedef SemanticPoint PointT;

typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_3   CGAL_Point;
typedef Kernel::Vector_3  CGAL_Vector;
//typedef Kernel::Plane_3   CGAL_Plane;
typedef std::pair<CGAL_Point,CGAL_Vector> PointVectorPair;


#endif // POINT_TYPES_HF_H

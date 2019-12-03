#ifndef SEGMENTERCY_H
#define SEGMENTERCY_H
#include "seg/objPartition.h"
#include "seg/surfaceSegment.h"
#include "seg/multiFrameMerge.h"
#include "point_types_hf.h"

class segmenterCY
{
public:
    segmenterCY();
    ~segmenterCY();

    void setInputCloud(pcl::PointCloud<PointT>::Ptr cloud_in);
    void process(void);

private:
    pcl::PointCloud<PointT>::Ptr cloud ;

};

#endif // SEGMENTERCY_H

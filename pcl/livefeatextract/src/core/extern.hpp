#ifndef __EXTERN_H__
#define __EXTERN_H__

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <fstream>

static std::ofstream FileStr;

struct PointXYZRGBCamSL {
  PCL_ADD_POINT4D;
  
  union {
    struct {
      float rgb;
    };
    float data_c[4];
  };
  uint32_t cameraIndex;
  float distance;
  uint32_t segment;
  uint32_t label;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(
        PointXYZRGBCamSL,
        (float, x, x)
        (float, y, y)
        (float, z, z)
        (float, rgb, rgb)
        (uint32_t, cameraIndex, cameraIndex)
        (float, distance, distance)
        (uint32_t, segment, segment)
        (uint32_t, label, label)
        )

#endif /* __EXTERN_H__ */




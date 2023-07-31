/*
 * @Author: Renwei
 * @Date: 2023-06-06 15:28:26
 * @Description: Point cloud type XYZRing
 */
#ifndef PCL_NO_PRECOMPILE
#define PCL_NO_PRECOMPILE


#include <pcl/point_types.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>


/*
    *一个具有XYZ、intensity、ring的点云类型
    */
struct PointXYZR
{
    PCL_ADD_POINT4D
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZR,  
                                   (float, x, x) (float, y, y)
                                   (float, z, z)
                                   (uint16_t, ring, ring)
)
typedef pcl::PointXYZ PointXYZ; // 点的类型
typedef pcl::PointXYZI PointXYZI; // 点的类型
typedef pcl::PointCloud<PointXYZ> CloudXYZ; // 点云类型
typedef pcl::PointCloud<PointXYZR> CloudXYZR; // 点云类型(含intensity,ring)
typedef pcl::PointCloud<PointXYZI> CloudXYZI; // 点云类型(含intensity,ring)

struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time)
)
typedef pcl::PointCloud<PointXYZIRPYT> CloudPose; 
typedef PointXYZIRPYT  PointTypePose;

#endif  //PCL_NO_PRECOMPILE

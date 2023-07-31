/*
 * @Author: Renwei Yunsheng
 * @Date: 2023-06-06 09:54:09
 * @Description: For SLAM Challenge data reader
 */
/*
 * 
 * 　　┏┓　　　┏┓+ +
 * 　┏┛┻━━━┛┻┓ + +
 * 　┃　　　　　　　┃ 　
 * 　┃　　　━　　　┃ ++ + + +
 *  ████━████ ┃+
 * 　┃　　　　　　　┃ +
 * 　┃　　　┻　　　┃
 * 　┃　　　　　　　┃ + +
 * 　┗━┓　　　┏━┛
 * 　　　┃　　　┃　　　　　　　　　　　
 * 　　　┃　　　┃ + + + +
 * 　　　┃　　　┃
 * 　　　┃　　　┃ +  神兽保佑
 * 　　　┃　　　┃    代码无bug　　
 * 　　　┃　　　┃　　+　　　　　　　　　
 * 　　　┃　 　　┗━━━┓ + +
 * 　　　┃ 　　　　　　　┣┓
 * 　　　┃ 　　　　　　　┏┛
 * 　　　┗┓┓┏━┳┓┏┛ + + + +
 * 　　　　┃┫┫　┃┫┫
 * 　　　　┗┻┛　┗┻┛+ + + +
 */

#ifndef DATA_READER_HPP
#define DATA_READER_HPP

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>

#include <cv_bridge/cv_bridge.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <dirent.h>
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <unistd.h>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>
#include <yaml-cpp/yaml.h>
#include <ros/package.h>
#include "data_reader/cout_tools.hpp"
#include "data_reader/point_type.hpp"
#include "pathchwork/patchworkpp.h"


class DataReader{
private:
    std::string _data_path;
    std::string _config_file_path;
    std::string _work_space_path;
    std::vector<std::string> _file_list;
    std::string _pointCloudTopicName1, _pointCloudTopicName2, _pointCloudTopicName3;

    int _N_SCAN;               // 线束数
    int _Horizon_SCAN;         // 水平采样次数
    float _ang_res_x;          // 水平分辨率
    float _ang_res_y;          // 数值分辨率
    float _ang_bottom;         // 底部线束角度偏移
    float _sensorMinimumRange;
    bool _useCloudRing;

    float _startOrientation;
    float _endOrientation;
    float _orientationDiff;
    float _lidarMinRange, _lidarMaxRange;
    // pcl::PointCloud<pcl::PointXYZI> _fullCloud;     // 帧点云（原始投影）
    // pcl::PointCloud<pcl::PointXYZI> _fullInfoCloud; // 帧点云（带强度）
    float _clusterTolerance, _clusterMinSize, _clusterMaxSize;
    float _box_y_divided_x_thresh_min, _box_y_divided_x_thresh_max, _box_scale_z_min, 
    _box_scale_z_max, _box_scale_x_min, _box_scale_y_min,
    _filter_padding;


    ros::NodeHandle _nh;
    ros::Publisher _cloud_pub1, _cloud_pub2, _cloud_pub3, cloud_ground_pub, cloud_no_ground_pub;
    ros::Publisher _end_pub;
    ros::Publisher _box_pub;
    ros::Publisher _clustered_box_pub;
    ros::Publisher _cloud_filertered_pub;
    patchwork::Params _patchwork_parameters;
    PointXYZI _nanPoint;
    Eigen::Vector3d _vehicle_filter_box;
    bool _if_use_vehicle_filter;
    std::string _frame_id;
    CloudXYZI::Ptr _cloud_info_ptr;
    nav_msgs::Odometry _endFlag;

public:
    DataReader(ros::NodeHandle nodehandle);
    ~DataReader() = default;
    void allocateMemory();
    void resetParameters();
    void readYaml();
    void showInfo();
    void getFileList();
    void getPCD(std::string file_name, CloudXYZI &cloud);
    void cloudFilter(const CloudXYZI& input, CloudXYZI &out);//按范围去除点云，删除无效点
    void rosPub();
    void groundFilter(const CloudXYZI &input_cloud, CloudXYZ &cloud_no_ground, CloudXYZ &cloud_ground);
    void cloud2Eigen(const CloudXYZI &input_cloud, Eigen::MatrixXf &cloud_eigen);
    void eigen2cloud(const Eigen::MatrixXf &input_cloud_eigen, CloudXYZ &input_cloud);
    void vehicleFilter(const CloudXYZ &cloudin, CloudXYZ &cloudout);
    void findStartEndAngle(CloudXYZR &tmp_cloud);
    void projectPointCloud(const CloudXYZ &tmp_cloud, bool if_ground, CloudXYZI &out_cloud);
    void cloudNoHeight(const CloudXYZ &cloudin, CloudXYZ &cloudout);
    void pubFilterBox();
};

#endif // DATA_READER_HPP
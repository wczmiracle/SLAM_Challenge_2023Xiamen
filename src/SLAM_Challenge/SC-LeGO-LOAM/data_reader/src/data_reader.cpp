/*
 * @Author: Renwei Yunsheng
 * @Date: 2023-06-06 10:16:43
 * @Description: Data reader cpp
 */

#include "data_reader/data_reader.hpp"

DataReader::DataReader(ros::NodeHandle nodehandle):_nh(nodehandle)
{
    //参数初始化
        _nanPoint.x = std::numeric_limits<float>::quiet_NaN();
        _nanPoint.y = std::numeric_limits<float>::quiet_NaN();
        _nanPoint.z = std::numeric_limits<float>::quiet_NaN();
        _nanPoint.intensity = -1;

    _work_space_path = ros::package::getPath("data_reader");    
    _config_file_path = _work_space_path+"/config/data_config.yaml";
    // cloud_ground_pub = _nh.advertise<sensor_msgs::PointCloud2>("cloud_ground", 1);
    // cloud_no_ground_pub = _nh.advertise<sensor_msgs::PointCloud2>("cloud_noground", 1);
    _patchwork_parameters.verbose = false;
    readYaml();    
    _cloud_pub1 = _nh.advertise<sensor_msgs::PointCloud2>(_pointCloudTopicName1, 1);
    _cloud_pub2 = _nh.advertise<sensor_msgs::PointCloud2>(_pointCloudTopicName2, 1);
    _cloud_pub3 = _nh.advertise<sensor_msgs::PointCloud2>(_pointCloudTopicName3, 1);
    
    _end_pub = _nh.advertise<nav_msgs::Odometry>("/end_flag", 1);
    
    _box_pub = _nh.advertise<visualization_msgs::Marker>("filter_box", 1);
    _cloud_filertered_pub = _nh.advertise<sensor_msgs::PointCloud2>("cropped_cloud", 1);
    _clustered_box_pub = _nh.advertise<visualization_msgs::MarkerArray>("clustered_box", 1);
    
    _endFlag.pose.pose.position.x = 0;

    getFileList();    
    showInfo();
    rosPub();
}

void DataReader::readYaml()
{
    YAML::Node node = YAML::LoadFile(_config_file_path);
    _data_path = node["PCD_data_path"].as<std::string>();
    
    // 雷达参数
    _N_SCAN = node["N_SCAN"].as<int>();                     // 线束
    _Horizon_SCAN = node["Horizon_SCAN"].as<int>();         // 水平采样次数
    _ang_res_x = node["ang_res_x"].as<float>();             // 水平分辨率
    _ang_res_y = node["ang_res_y"].as<float>();             // 数值分辨率
    _ang_bottom = node["ang_bottom"].as<float>();           // 底部线束角度偏移
    _sensorMinimumRange = node["sensorMinimumRange"].as<float>();
    _useCloudRing = node["useCloudRing"].as<bool>();
    _pointCloudTopicName1 = node["lidarTopic1"].as<std::string>();
    _pointCloudTopicName2 = node["lidarTopic2"].as<std::string>();
    _pointCloudTopicName3 = node["lidarTopic3"].as<std::string>();
    std::vector<float> sensor_range = node["sensorRange"].as<std::vector<float>>();
    _lidarMinRange = sensor_range[0];
    _lidarMaxRange = sensor_range[1];

    std::vector<double> box_size =  node["FilterBoxSize"].as<std::vector<double>>();
    _vehicle_filter_box.x() = box_size[0];
    _vehicle_filter_box.y() = box_size[1];
    _vehicle_filter_box.z() = box_size[2];
    _frame_id = node["frame_id"].as<std::string>();
    _if_use_vehicle_filter = node["ifUseCarFilter"].as<bool>();

    _clusterTolerance = node["clusterTolerance"].as<float>();
    _clusterMinSize = node["clusterMinSize"].as<float>();
    _clusterMaxSize = node["clusterMaxSize"].as<float>();
    _box_y_divided_x_thresh_min = node["box_y_divided_x_thresh_min"].as<float>();
    _box_y_divided_x_thresh_max = node["box_y_divided_x_thresh_max"].as<float>();

    _box_scale_z_min = node["box_scale_z_min"].as<float>();
    _box_scale_z_max = node["box_scale_z_max"].as<float>();
    _box_scale_x_min = node["box_scale_x_min"].as<float>();
    _box_scale_y_min = node["box_scale_y_min"].as<float>();
    _filter_padding = node["filter_padding"].as<float>();


}
void DataReader::allocateMemory()
{
    _cloud_info_ptr.reset(new CloudXYZI);
    _cloud_info_ptr->points.resize(_N_SCAN*_Horizon_SCAN);

}

void DataReader::resetParameters()
{
    _cloud_info_ptr->clear();
}

void DataReader::showInfo()
{   
    std::cout<<COUT_DEBUG<<std::endl;
    std::cout<<"Work Space path: "<<_work_space_path<<std::endl;
    std::cout<<"Config file path: "<<_config_file_path<<std::endl;
    std::cout<<"Data path: "<<_data_path<<std::endl;
    std::cout<<"File num: "<<_file_list.size()<<std::endl;
    std::cout<<COUT_DEBUG<<std::endl;
}

void DataReader::getFileList()
{
    DIR *dir;
    struct dirent* ptr;
    if(!(dir = opendir(_data_path.c_str())))
    {
        std::cout<<"Data folder "<<_data_path<<" does not exist"<<std::endl;
        return;
    }
      while((ptr = readdir(dir))!=0) 
    {
        if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0){
            _file_list.push_back( ptr->d_name);
    	}
    }
    std::sort(_file_list.begin(), _file_list.end());
}

void DataReader::getPCD(std::string file_name, CloudXYZI &cloud)
{   
    std::string full_file_path = _data_path + "/" + file_name;
    // std::cout<<"Read file: "<<full_file_path<<std::endl;
    std::fstream point_file(full_file_path.c_str(), std::ios::in | std::ios::binary);
    if(!point_file.good()){
        std::cout<<COUT_RED<<"Couldn't find file : "<<full_file_path<<std::endl;
    }
    int i;
    for(i=0; point_file.good()&&!point_file.eof(); i++)
    {
        PointXYZI tmp_point;
        _Float32 ring;

        point_file.read((char*)&tmp_point.x, 3*sizeof(_Float32));
        point_file.read((char*)&ring, sizeof(_Float32));
        tmp_point.intensity = 0;
        // std::cout<<"Point x: "<<tmp_point.x<<" y: "<<tmp_point.y<<" z: "<<tmp_point.z<<" r: "
        // <<tmp_point.ring<<std::endl;
        cloud.points.push_back(tmp_point);
    }
    cloud.height = 1;
    cloud.width = cloud.points.size();
    point_file.close();
}

void DataReader::rosPub()
{   
    float count = 0;
    ros::Time time_s, time_e;

    int gap_num = 10;
    int file_cut_size = _file_list.size()/3;


    for(auto iter = _file_list.begin(); iter != _file_list.begin()+file_cut_size+gap_num; iter++)
    {   
        time_s = ros::Time::now();
        if(!ros::ok())break;
        count+=3;
        double time_stamp, time_stamp2, time_stamp3;
        std::string cloud_name1, cloud_name2, cloud_name3;
        cloud_name1 = *iter;


        cloud_name1.erase(cloud_name1.end()-4, cloud_name1.end());
        time_stamp = std::stod(cloud_name1);
        time_stamp = time_stamp/1e6;
        // cout.setf(ios_base::fixed,ios_base::floatfield);
        // std::cout<<COUT_DEBUG<<time_stamp<<COUT_DEBUG;

        CloudXYZI tmp_cloud;
        getPCD(*iter, tmp_cloud);
        CloudXYZI filtered_cloud;
        cloudFilter(tmp_cloud, filtered_cloud);
        sensor_msgs::PointCloud2 tmp_ros_cloud;// tmp_rs_cloud_ground, tmp_ros_cloud_nogroud;
        // CloudXYZ cloud_ground, cloud_noground;
        std::cout<<"Mapping data: "<<COUT_GREEN<<count/_file_list.size()<<COUT_NON<<std::endl;
        
        // ground filter no use
        // groundFilter(filtered_cloud, cloud_noground, cloud_ground);

        // std::cout<<"Point size: "<<tmp_cloud.points.size()<<std::endl;
        // std::cout<<"Ground cloud: "<<cloud_ground.points.size()<<std::endl;
        // std::cout<<"No ground cloud: "<<cloud_noground.points.size()<<std::endl;

        // if(_if_use_vehicle_filter)
        // {
        //     CloudXYZ out_cloud;
        //      vehicleFilter(cloud_noground,  out_cloud);
        //     cloud_noground = out_cloud;
        // }

        // findStartEndAngle(tmp_cloud);
        // projectPointCloud(tmp_cloud);
        // if(f)
        // {
        // CloudXYZI cloud_info_ground, cloud_info_noground;
        // projectPointCloud(cloud_ground, true, cloud_info_ground);
        // projectPointCloud(cloud_noground, false, cloud_info_noground);
        // CloudXYZI full_cloud_info = cloud_info_ground + cloud_info_noground;
        // }
        // else{

        // }
   
        pcl::toROSMsg(filtered_cloud, tmp_ros_cloud);
        // pcl::toROSMsg(cloud_ground, tmp_ros_cloud_ground);
        // pcl::toROSMsg(cloud_noground, tmp_ros_cloud_nogroud);
        tmp_ros_cloud.header.frame_id = "/map";
        tmp_ros_cloud.header.stamp =ros::Time(time_stamp) ;
        // tmp_ros_cloud_ground.header.frame_id = "/map";
        // tmp_ros_cloud_ground.header.stamp = ros::Time::now();
        // tmp_ros_cloud_nogroud.header.frame_id = "/map";
        // tmp_ros_cloud_nogroud.header.stamp = ros::Time::now();
        _cloud_pub1.publish(tmp_ros_cloud);
        // cloud_ground_pub.publish(tmp_ros_cloud_ground);
        // cloud_no_ground_pub.publish(tmp_ros_cloud_nogroud);

        cloud_name2 = *(iter +file_cut_size-gap_num);
        cloud_name2.erase(cloud_name2.end()-4, cloud_name2.end());
        time_stamp2 = std::stod(cloud_name2);
        time_stamp2 = time_stamp2/1e6;
        cout.setf(ios_base::fixed,ios_base::floatfield);
        // std::cout<<COUT_DEBUG<<time_stamp2<<COUT_DEBUG;
        cloud_name2 = *(iter +file_cut_size-gap_num);
        CloudXYZI tmp_cloud2;
        getPCD(cloud_name2, tmp_cloud2);
        CloudXYZI filtered_cloud2;
        cloudFilter(tmp_cloud2, filtered_cloud2);
        sensor_msgs::PointCloud2 tmp_ros_cloud2;
        pcl::toROSMsg(filtered_cloud2, tmp_ros_cloud2);
        tmp_ros_cloud2.header.frame_id = "/map";
        tmp_ros_cloud2.header.stamp =ros::Time(time_stamp2) ;
        _cloud_pub2.publish(tmp_ros_cloud2);

        if(iter +2*file_cut_size-gap_num != _file_list.end())
        {
        cloud_name3 = *(iter +2*file_cut_size-gap_num);
        cloud_name3.erase(cloud_name3.end()-4, cloud_name3.end());
        time_stamp3 = std::stod(cloud_name3);
        time_stamp3 = time_stamp3/1e6;
        cout.setf(ios_base::fixed,ios_base::floatfield);
        // std::cout<<COUT_DEBUG<<time_stamp3<<COUT_DEBUG;
        CloudXYZI tmp_cloud3;
        cloud_name3 = *(iter +2*file_cut_size-gap_num);
        getPCD(cloud_name3, tmp_cloud3);
        CloudXYZI filtered_cloud3;
        cloudFilter(tmp_cloud3, filtered_cloud3);
        sensor_msgs::PointCloud2 tmp_ros_cloud3;
        pcl::toROSMsg(filtered_cloud3, tmp_ros_cloud3);
        tmp_ros_cloud3.header.frame_id = "/map";
        tmp_ros_cloud3.header.stamp =ros::Time(time_stamp3) ;
        _cloud_pub3.publish(tmp_ros_cloud3);
        }



        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        time_e = ros::Time::now();
        // std::cout<<"Read data time: "<<time_e.toSec()-time_s.toSec()<<std::endl;
    }

    std::chrono::milliseconds(3000);

    // 在此处发布完毕消息
    // nav_msgs::Odometry endFlag;
    _endFlag.pose.pose.position.x = 1;
    for(int i = 0; i < 10; ++i){
        _end_pub.publish(_endFlag);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    


}

void DataReader::cloud2Eigen(const CloudXYZI &input_cloud, Eigen::MatrixXf &cloud_eigen)
{
    Eigen::MatrixXf cur_cloud_eigen;
    int num_points = input_cloud.points.size();
    cur_cloud_eigen.resize(num_points, 3);
    for(int i=0; i<num_points; i++)
    {
        cur_cloud_eigen.row(i)<<input_cloud.points[i].x,input_cloud.points[i].y, input_cloud.points[i].z;
    }
cloud_eigen = cur_cloud_eigen;
}

void DataReader::eigen2cloud(const Eigen::MatrixXf &input_cloud_eigen, CloudXYZ &input_cloud)
{
    CloudXYZ cur_input_cloud;
    int num_points = input_cloud_eigen.rows();
    for(int i=0; i<num_points; i++)
    {
        PointXYZ tmp_point;
        tmp_point.x = input_cloud_eigen(i,0);
        tmp_point.y = input_cloud_eigen(i,1);
        tmp_point.z = input_cloud_eigen(i,2);
        cur_input_cloud.points.push_back(tmp_point);
    }
    input_cloud = cur_input_cloud;
}
void DataReader::groundFilter(const CloudXYZI &input_cloud, CloudXYZ &cloud_no_ground, CloudXYZ &cloud_ground)
{   
    ros::Time time1, time2;
    Eigen::MatrixXf cur_cloud_eigen;
    cloud2Eigen(input_cloud, cur_cloud_eigen);    
    time1 = ros::Time::now();

    patchwork::PatchWorkpp Patchworkpp(_patchwork_parameters);
    Patchworkpp.estimateGround(cur_cloud_eigen);
    Eigen::MatrixX3f ground = Patchworkpp.getGround();
    Eigen::MatrixX3f nonground  = Patchworkpp.getNonground();
    double time_taken = Patchworkpp.getTimeTaken();
    time2 = ros::Time::now();

    CloudXYZ cloud_ground_now;
    CloudXYZ cloud_noground_now;
    eigen2cloud(ground, cloud_ground_now);
    eigen2cloud(nonground, cloud_noground_now);
    cloud_no_ground = cloud_noground_now;
    cloud_ground = cloud_ground_now;
    // std::cout<<"Ground Filter time used: "<<time2.toSec()-time1.toSec()<<std::endl;
    // std::cout<<"Patchwork Time used: "<<time_taken<<std::endl;
}


void DataReader::findStartEndAngle(CloudXYZR& tmp_cloud){
    
    size_t cloudSize = tmp_cloud.size();    // 每帧点云点的数量

    _startOrientation = -atan2(tmp_cloud[0].y, tmp_cloud[0].x);
    _endOrientation   = -atan2(tmp_cloud[cloudSize - 1].y,tmp_cloud[cloudSize - 1].x) + 2 * M_PI;
    if (_endOrientation - _startOrientation > 3 * M_PI) {
        _endOrientation -= 2 * M_PI;
    } 
    else if (_endOrientation - _startOrientation < M_PI)
        _endOrientation += 2 * M_PI;
    _orientationDiff = _endOrientation - _startOrientation;

    // std::cout << "orientationDiff: " << _orientationDiff << std::endl;

}


// void DataReader::projectPointCloud(const CloudXYZ &tmp_cloud, bool if_ground, CloudXYZI &out_cloud)
// {
//     CloudXYZI result_cloud;
//     size_t cloudSize = tmp_cloud.size();    // 每帧点云点的数量
//     size_t rowIdn, columnIdn, index;
//     float verticalAngle, horizonAngle, range;

//     pcl::PointXYZI thisPoint;                           // 带强度的点

//     cv::Mat rangeMat = cv::Mat(_N_SCAN, _Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));// range matrix for range image
//     // std::cout << "get file!" << std::endl;
//     for (size_t i = 0; i < cloudSize; ++i)
//     {
//         thisPoint.x = tmp_cloud[i].x;
//         thisPoint.y = tmp_cloud[i].y;
//         thisPoint.z = tmp_cloud[i].z;
//         // -1- 寻找行索引
//         // if (_useCloudRing == true){ // 使用线束
//             // rowIdn = tmp_cloud[i].ring;
//         // }
//         // else{                      // 使用非线束 
//             verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI;
//             rowIdn = (verticalAngle + _ang_bottom) / _ang_res_y;

//         // }
//         if (rowIdn < 0 || rowIdn >= _N_SCAN)
//             continue;
//         // -2- 寻找列索引
//         horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;
//         columnIdn = -round((horizonAngle-90.0)/_ang_res_x) + _Horizon_SCAN/2;
//         if (columnIdn >= _Horizon_SCAN)
//             columnIdn -= _Horizon_SCAN;
//         if (columnIdn < 0 || columnIdn >= _Horizon_SCAN)
//             continue;
//         // -3- 计算深度
//         range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);
//         if (range < _sensorMinimumRange)
//             continue;
        
        
        
//         rangeMat.at<float>(rowIdn, columnIdn) = range;

//         thisPoint.intensity = (float)rowIdn + (float)columnIdn / 10000.0;
//         if(if_ground){
//             thisPoint.intensity += 100;
//         }
//         index = columnIdn  + rowIdn * _Horizon_SCAN;
//         result_cloud.points.push_back(thisPoint);
//         // _fullCloud.points[index] = thisPoint;
//         // _fullInfoCloud.points[index] = thisPoint;
//         // _fullInfoCloud.points[index].intensity = range; // the corresponding range of a point is saved as "intensity"
    
//     }
//     out_cloud = result_cloud;
//     // 可视化
//     // cv::namedWindow("map", CV_WINDOW_NORMAL);
//     // cv::imshow("map", rangeMat);
//     // cv::waitKey(100);
// }

void DataReader::cloudFilter(const CloudXYZI& input, CloudXYZI &out)
{      
    CloudXYZI input_cloud = input;
        std::vector<int> mapping;
    pcl::removeNaNFromPointCloud(input ,  input_cloud,mapping);
    CloudXYZI result_cloud;
    for(size_t i = 0; i < input_cloud.points.size(); i++)
    {
        PointXYZI input_point;
        input_point.x = input_cloud.points[i].x;
        input_point.y = input_cloud.points[i].y;
        input_point.z = input_cloud.points[i].z;
        input_point.intensity = input_cloud.points[i].intensity;
        float range = input_point.x* input_point.x+ input_point.y*input_point.y;
        range = sqrt(range);
        if(range>_lidarMinRange && range <_lidarMaxRange)
        {
            result_cloud.points.push_back(input_point);
        }
    }
    out = result_cloud;
}


void DataReader::vehicleFilter(const CloudXYZ &cloudin, CloudXYZ &cloudout)
{

    pubFilterBox();
    CloudXYZ::Ptr cloud_out_ptr(new CloudXYZ);
    *cloud_out_ptr = cloudin;

    CloudXYZ::Ptr origin_cloud_ptr(new CloudXYZ);
    *origin_cloud_ptr = cloudin;
    pcl::CropBox<PointXYZ> crop_box;    
    CloudXYZ cropped_cloud;
    crop_box.setInputCloud(origin_cloud_ptr);
    crop_box.setMax(Eigen::Vector4f(_vehicle_filter_box.x()/2,
    _vehicle_filter_box.y()/2,_vehicle_filter_box.z()/2, 1));
    crop_box.setMin(Eigen::Vector4f(-_vehicle_filter_box.x()/2,
    -_vehicle_filter_box.y()/2,-_vehicle_filter_box.z()/2, 1));
    crop_box.filter(cropped_cloud);

    CloudXYZ cloud_noHeight;
    cloudNoHeight(cropped_cloud, cloud_noHeight);

    //TODO 改变点云离群点滤除参数
    CloudXYZ::Ptr cloud_noheight_filtered_ptr(new CloudXYZ);
    *cloud_noheight_filtered_ptr = cloud_noHeight;
    pcl::RadiusOutlierRemoval<PointXYZ> out_remove;
    out_remove.setInputCloud(cloud_noheight_filtered_ptr);
    out_remove.setRadiusSearch(1.0);
    out_remove.setMinNeighborsInRadius (10);
    out_remove.setKeepOrganized(false);
    out_remove.filter( *cloud_noheight_filtered_ptr );


    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZ>);
    kdtree->setInputCloud(cloud_noheight_filtered_ptr);
    // kdtree->setEpsilon(0.25);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> cluster; 
    //TODO 改变聚类参数
    cluster.setClusterTolerance(_clusterTolerance);
    cluster.setMinClusterSize(_clusterMinSize);
    cluster.setMaxClusterSize(_clusterMaxSize);
    cluster.setSearchMethod(kdtree);
    cluster.setInputCloud(cloud_noheight_filtered_ptr);
    cluster.extract(cluster_indices);

CloudXYZI cloud_clustered;
std::vector <CloudXYZI> cloud_clustered_list;        
    float id_color = 0;

    for(auto iter = cluster_indices.begin(); iter != cluster_indices.end(); iter++)
    {
        CloudXYZI cloud_clustered_part;
        for(auto iiter = iter->indices.begin(); iiter != iter->indices.end(); iiter++)
        {
            PointXYZI point_tmp;
            point_tmp.x = cloud_noheight_filtered_ptr->points[*iiter].x;
            point_tmp.y = cloud_noheight_filtered_ptr->points[*iiter].y;
            point_tmp.z = cloud_noheight_filtered_ptr->points[*iiter].z;
            point_tmp.intensity = id_color;
            cloud_clustered_part.push_back(point_tmp);
        }
        id_color +=50;
        cloud_clustered_list.push_back(cloud_clustered_part);
        cloud_clustered += cloud_clustered_part;
    }
    cloud_clustered.height = 1;
    cloud_clustered.width = cloud_clustered.points.size();

    //计算bouding box并判断
    visualization_msgs::MarkerArray box_array;
    int count = 0;
    CloudXYZ cloud_no_vehicle;
    for(auto iter = cloud_clustered_list.begin(); iter != cloud_clustered_list.end(); iter++)
    {
        pcl::MomentOfInertiaEstimation <pcl::PointXYZI> feature_extractor;
        CloudXYZI::Ptr cloud_ptr(new CloudXYZI);
        *cloud_ptr = *iter;
        feature_extractor.setInputCloud(cloud_ptr);
        feature_extractor.compute();

        pcl::PointXYZI min_point_OBB;
        pcl::PointXYZI max_point_OBB;
        pcl::PointXYZI position_OBB;
        Eigen::Matrix3f rotational_matrix_OBB;
        count ++;
        feature_extractor.getOBB(min_point_OBB,max_point_OBB, position_OBB, rotational_matrix_OBB); 
        pcl::PointXYZI min_point_AABB;
        pcl::PointXYZI max_point_AABB;
        feature_extractor.getAABB(min_point_AABB,max_point_AABB); 


        Eigen::Quaternionf rotation_q(rotational_matrix_OBB);
        visualization_msgs::Marker box;
        box.ns = std::to_string(count)+"box";
        box.header.frame_id = _frame_id;
        box.header.stamp = ros::Time::now();
        //
        box.pose.position.x = (min_point_AABB.x+max_point_AABB.x)/2;
        box.pose.position.y = (min_point_AABB.y+max_point_AABB.y)/2;
        box.pose.position.z = (min_point_AABB.z+max_point_AABB.z)/2*1000;
        box.scale.x = max_point_AABB.x- min_point_AABB.x+2*_filter_padding;
        box.scale.y = max_point_AABB.y- min_point_AABB.y+2*_filter_padding;
        box.scale.z = (max_point_AABB.z- min_point_AABB.z)*1000+2*_filter_padding;
                box.pose.orientation.w =1;
        box.pose.orientation.x =0;
        box.pose.orientation.y = 0;
        box.pose.orientation.z =0;
        //
        // box.pose.position.x =  position_OBB.x;
        // box.pose.position.y =  position_OBB.y;
        // box.pose.position.z =  position_OBB.z*1000;
        // box.pose.orientation.w = rotation_q.w();
        // box.pose.orientation.x = rotation_q.x();
        // box.pose.orientation.y = rotation_q.y();
        // box.pose.orientation.z = rotation_q.z();
        // box.scale.x = max_point_OBB.x- min_point_OBB.x;
        // box.scale.y = max_point_OBB.y- min_point_OBB.y;
        // box.scale.z = (max_point_OBB.z- min_point_OBB.z)*1000;
        box.type = visualization_msgs::Marker::CUBE;
        box.action = visualization_msgs::Marker::ADD;
        box.color.r = 0.0f;
        box.color.g = 0.0f;
        box.color.b = 1.0f;
        box.color.a = 0.3f;
        float box_scale_yx = box.scale.y/box.scale.x;
        float box_scale_z = box.scale.z;
        if(box_scale_yx >_box_y_divided_x_thresh_min &&box_scale_yx<_box_y_divided_x_thresh_max
        &&box_scale_z>_box_scale_z_min&&box_scale_z<_box_scale_z_max
        &&box.scale.x>_box_scale_x_min&&box.scale.y>_box_scale_y_min)
        {
            box.color.r = 1.0f;
            box.color.g = 0.0f;
            box.color.b = 0.0f;
            box.color.a = 0.6f;
            
            pcl::CropBox<PointXYZ> vehicle_removal;
            vehicle_removal.setInputCloud(cloud_out_ptr);
            // Eigen::Vector3f rot_vec = rotational_matrix_OBB.eulerAngles(0,1,2);
            // vehicle_removal.setRotation(rot_vec);
            vehicle_removal.setMin(Eigen::Vector4f(min_point_AABB.x-_filter_padding, min_point_AABB.y-_filter_padding, min_point_AABB.z*1000-_filter_padding,1));
            vehicle_removal.setMax(Eigen::Vector4f(max_point_AABB.x+_filter_padding, max_point_AABB.y+_filter_padding, max_point_AABB.z*1000+_filter_padding,1));
            // vehicle_removal.setTranslation(Eigen::Vector3f(position_OBB.x, position_OBB.y, position_OBB.z) );
            vehicle_removal.setNegative(true);

            vehicle_removal.filter(*cloud_out_ptr);
            box_array.markers.push_back(box);
            continue;
        }
        
        box_array.markers.push_back(box);
        for(auto cloud_iter = iter->points.begin(); cloud_iter != iter->points.end(); cloud_iter++)
        {
            PointXYZ thispoint;
            thispoint.x = cloud_iter->x;
            thispoint.y = cloud_iter->y;
            thispoint.z = cloud_iter->z*1000;
            cloud_no_vehicle.points.push_back(thispoint);
        }

        }
        _clustered_box_pub.publish(box_array);


    sensor_msgs::PointCloud2 cloud_filtered_for_pub;
    pcl::toROSMsg(*cloud_out_ptr ,cloud_filtered_for_pub);
    cloud_filtered_for_pub.header.frame_id = _frame_id;
    _cloud_filertered_pub.publish(cloud_filtered_for_pub);
    cloudout = *cloud_out_ptr;
}
void DataReader::pubFilterBox()
{
    visualization_msgs::Marker filter_box;
    filter_box.header.frame_id = _frame_id;
    filter_box.header.stamp =  ros::Time::now();
    filter_box.ns = "vehicle_fileter";
    filter_box.type = visualization_msgs::Marker::CUBE;
    filter_box.action = visualization_msgs::Marker::ADD;

    filter_box.scale.x = _vehicle_filter_box.x();
    filter_box.scale.y = _vehicle_filter_box.y();
    filter_box.scale.z = _vehicle_filter_box.z();
    filter_box.pose.position.x = 0;
    filter_box.pose.position.y = 0;
    filter_box.pose.position.z = 0; 
    filter_box.color.r = 0.0f;
    filter_box.color.g = 1.0f;
    filter_box.color.b = 0.0f;
    filter_box.color.a = 0.2f;

    _box_pub.publish(filter_box);

    


}

void DataReader::cloudNoHeight(const CloudXYZ &cloudin, CloudXYZ &cloudout)
{
    CloudXYZ cloud_out;
    for(auto iter=cloudin.points.begin(); iter != cloudin.points.end(); iter++)
    {
        PointXYZ thisPoint;
        thisPoint.x = iter->x;
        thisPoint.y =  iter->y;
        thisPoint.z =  iter->z/1000;
        cloud_out.points.push_back(thisPoint);
    }
    cloudout = cloud_out;
}


/*
 * @Author: Renwei
 * @Date: 2023-06-06 10:49:58
 * @Description: Main函数
 */

#include "data_reader/data_reader.hpp"

int main(int argc, char **argv)
{      
    ros::init(argc, argv, "Data_reader");
    ros::NodeHandle ros_node;
    DataReader data_reader(ros_node);
    
}
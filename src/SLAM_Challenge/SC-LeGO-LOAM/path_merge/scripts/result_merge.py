#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import rospkg
import io
import math
import os
import numpy as np
import transforms3d as tfs
import time
from nav_msgs.msg import Odometry

# 记录所有时间
t0 = 0

# 用于连接不同文件
time_g = 0.0
euler_g = np.array([0,0,0])
trans_g = np.array([0,0,0])

# 文件内的临时全局位姿
time_tmp = 0.0
euler_tmp = np.array([0,0,0])
trans_tmp = np.array([0,0,0])

# 时间戳标志位
flag_t = 0

def take_d5(elem):
    return int(elem[-5])

def merge():
    # 读取文件路径
    rospack = rospkg.RosPack()
    filedir = rospack.get_path('data_reader') + "/results"
    filenames=os.listdir(filedir)
    # 对文件排序
    filenames.sort(key=take_d5)

    # 打开合并后文件
    file = open(rospack.get_path('data_reader') + "/result.txt",'w')

    # 开始遍历文件
    i = 0
    for filename in filenames:

            filepath = filedir + "/" + filename # 当前文件名称
            print(filepath)
            print(i)

            # 当前文件遍历
            for line in io.open(filepath, 'r', encoding='utf-8'):
                    if(i==0):
                            time_tmp = eval(line.split()[0])
                            euler_tmp = np.array([eval(line.split()[4]), eval(line.split()[5]), eval(line.split()[6])])
                            trans_tmp = np.array([eval(line.split()[1]), eval(line.split()[2]), eval(line.split()[3])])
                            sep = " "
                            seq = [str('{:.4f}'.format(time_tmp)), \
                                            str('{:.4f}'.format(trans_tmp[0])), \
                                            str('{:.4f}'.format(trans_tmp[1])), \
                                            str('{:.4f}'.format(trans_tmp[2])), \
                                            str('{:.4f}'.format(euler_tmp[0])),  \
                                            str('{:.4f}'.format(euler_tmp[1])),  \
                                            str('{:.4f}'.format(euler_tmp[2])) ]
                            file.write(sep.join(seq)+'\n')
                    else:
                            time_tmp = eval(line.split()[0])
                            if(time_tmp != time_g and flag_t == 0): continue # 如果当前时间戳和上一帧时间戳还没有对上，就继续去找对应时间戳
                            flag_t = 1 # 如果对应上了，则flag位置1，开始进入循环
                            if(time_tmp == time_g): continue

                            # 记录当前欧拉角与位置
                            euler = np.array([eval(line.split()[4]), eval(line.split()[5]), eval(line.split()[6])])
                            trans = np.array([eval(line.split()[1]), eval(line.split()[2]), eval(line.split()[3])])
                            R = tfs.euler.euler2mat(euler[0], euler[1], euler[2],"sxyz")

                            # 全局位姿更新
                            R_g = tfs.euler.euler2mat(euler_g[0], euler_g[1], euler_g[2],"sxyz")
                            
                            # 位姿关联
                            trans_tmp = np.dot(R_g, trans) + trans_g
                            R = np.dot(R_g, R)
                            euler_tmp = tfs.euler.mat2euler(R,"sxyz")

                            # 记录全局位姿
                            sep = " "
                            seq = [str('{:.4f}'.format(time_tmp)), \
                                            str('{:.4f}'.format(trans_tmp[0])), \
                                            str('{:.4f}'.format(trans_tmp[1])), \
                                            str('{:.4f}'.format(trans_tmp[2]/20.0)), \
                                            str('{:.4f}'.format(euler_tmp[0])),  \
                                            str('{:.4f}'.format(euler_tmp[1])),  \
                                            str('{:.4f}'.format(euler_tmp[2])) ]
                            file.write(sep.join(seq)+'\n')
            
            time_g = time_tmp
            euler_g = euler_tmp
            trans_g = trans_tmp
            flag_t = 0
            i+=1

    file.close()
    print("##########\nSLAM time use:", time.time()-t0,"###########\n")
    file = open(rospack.get_path('data_reader')+"/timeuse.txt", "w")
    file.write(str('{:.4f}'.format(time.time()-t0)))
    file.close()

    rospy.signal_shutdown("SLAM Finish!")
    


def callback_fuc(msg):  
    if(msg.pose.pose.position.x == 1):
        merge()
    
def listener():
    rospy.init_node('path_merge', anonymous=True)  
    rospy.Subscriber("/end_flag", Odometry, callback_fuc) 
    rospy.spin()    
    
if __name__ == '__main__':
    t0 = time.time()
    # merge()
    listener()
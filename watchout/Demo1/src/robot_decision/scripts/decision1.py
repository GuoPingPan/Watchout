#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import math
# import numpy as np
# from random import randint  
import Queue
# from utils import discretization as dn
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Twist
import time
# import operator
# 定义全局变量：

SCALE = 0.5

def floor_tuple(scale, pos=tuple()):
    """
    将点云数据转换成适合的数据格式
    :param scale:  缩放比例
    :param pos: 未经离散化的连续型坐标
    :return: 缩放、离散化以后的连续型坐标
    """
    x = float(pos[0]/scale)
    y = float(pos[1]/scale)
    return tuple((math.floor(x), math.floor(y)))


class Blocklist(object):
    """
    Blocklist类，接收来自点云的数据，离散化为一组元组，代表A*中的blocklist
    """
    def __init__(self, scale, point_cloud_data):
        """
        :param scale: 缩放比例
        :param point_cloud_data: 原始点云数据
        """
        self.scale = scale
        self.pcd = point_cloud_data
        self.blocklist = list()

    def pcd_discretization(self):
        for x in self.pcd:
            block = floor_tuple(self.scale, x)
            if block not in self.blocklist:
                self.blocklist.append(block)
        return self.blocklist


class Node(object):
    """
    定义节点类型，每一个节点类实际上包含了:
    1.当前节点位置pos = (x,y)
    2.当前节点从起点到当前节点CNode的实际最小距离gvalue = g(x)
    3.当前状态下的目标函数fvalue = f(x)
    4.当前节点的父节点位置
    --顺便包含与上述数据有关的处理函数
    """  # todo 改写目标函数的计算公式，寻找最优解
    def __init__(self, pos):
        self.pos = pos
        self.father = None
        self.gvalue = 0
        self.fvalue = 0

    def compute_fx(self, enode, father):
        if not father:
            print('未设置当前节点的父节点！')
        gx_father = father.gvalue  # g'(n1)
        # 采用欧式距离计算父节点到当前节点的距离  d(n1,n2)
        gx_f2n = math.sqrt((father.pos[0] - self.pos[0])**2 + (father.pos[1] - self.pos[1])**2)
        gvalue = gx_f2n + gx_father  # 对于子节点的g(n2) = g(n1) + d(n1,n2)
        # 利用欧式距离计算该点到终点的距离 h(n2)
        hx_n2enode = math.sqrt((self.pos[0] - enode.pos[0])**2 + (self.pos[1] - enode.pos[1])**2)
        fvalue = gvalue + hx_n2enode  # f(n2) = h(n2) + g(n2)
        return gvalue, fvalue

    def set_fx(self, enode, father):
        self.gvalue, self.fvalue = self.compute_fx(enode, father)
        self.father = father

    def update_fx(self, enode, father):
        gvalue, fvalue = self.compute_fx(enode, father)
        if fvalue < self.fvalue:
            self.gvalue, self.fvalue = gvalue, fvalue
            self.father = father


class AStar(object):
    """
    AStar类，保存A*算法的主函数
    输入：
        1.地图大小(n*m)
        2.当前点坐标(x,y) -- 原本应该是“起点坐标”，但是修改后改成当前点坐标
        3.终点坐标(x,y)
        还有个隐含的输入，即障碍物的坐标
    输出与中间变量:
        1.openist，当前节点周围所能寻找的点  todo 这里有问题：可能无法达到“预测路径”的初衷
        2.closelist, 已经规划的点，即用来保存最优路径
        3.blocklist，障碍物坐标
    """
    def __init__(self, mapsize, pos_now, pos_en):
        self.mapsize = mapsize   # 表示地图的投影大小(n*n)，并非屏幕上的地图像素大小
        self.openlist, self.closelist, self.blocklist = [], [], []
        # openlist 待搜索点
        # closelist 已规划点
        # blocklist 障碍物点
        self.snode = Node(pos_now)   # 存储当前点
        self.enode = Node(pos_en)   # 用于存储路径规划的目标节点
        self.cnode = self.snode     # 用于存储当前搜索到的节点
        # 对于第一次而言就是加入起始点

    def run(self):  # 寻路主函数
        self.openlist.append(self.snode)
        while len(self.openlist) > 0:
            # 查找openlist中fx最小的节点,此时fxlist中存储着待搜索点中所有的目标函数大小
            # 定义起点的目标函数fx为0
            fxlist = list(map(lambda x: x.fvalue, self.openlist))
            # lambda匿名函数，返回x的目标函数大小，map函数将x对象(一个node类，包含它本身的所有信息)
            # map函数有两个值，逗号前一项为一个函数，后一项为一个迭代器(迭代对象可能不止一个)
            # 表明按顺序从中取出一个或多个元素传入函数中，并返回一个元组(默认是元组，但不改变迭代器内的对象类型)
            # （大概）
            index_min = fxlist.index(min(fxlist))  # 寻找出fxlist中目标函数最小的点
            self.cnode = self.openlist[index_min]  # openlist和fxlist中得元素一一对应，则最小值的下标也就是最小值点的索引
            del self.openlist[index_min]  # 在openlist中删除index_min节点
            self.closelist.append(self.cnode)  # 在closelist中加入index_min节点
            # 扩展当前fx最小的节点，并进入下一次循环搜索
            self.extend(self.cnode)
            # 如果openlist列表为空，或者当前搜索节点为目标节点，则跳出循环
            if len(self.openlist) == 0 or self.cnode.pos == self.enode.pos:
                break
        if self.cnode.pos == self.enode.pos:
            self.enode.father = self.cnode.father
            return self.closelist[1]  # 返回下一步该走的位置
        else:
            return -1  # 停止不动

    def get_minroute(self):
        minroute = Queue.LifoQueue()
        current_node = self.enode

        while True:
            minroute.put(current_node.pos)
            current_node = current_node.father  # 这里的思想和Dijkstra算法类似，在返回路径的时候，返回其父节点
            if current_node.pos == self.snode.pos:  # 找到根节点
                break

        # minroute.put(self.snode.pos)  # 加入父节点  暂且不要
        return minroute

    def extend(self, cnode):  # 作用是加入当前目标函数最小的节点周围不在closelist和blocklist中的节点进入openlist
        nodes_neighbor = self.get_neighbor(cnode)
        for node in nodes_neighbor:
            # 判断节点node是否在closelist和blocklist中，因为closelist和blocklist中元素均为Node类，所以要用map函数转换为坐标集合
            if node.pos in list(map(lambda x: x.pos, self.closelist)) or node.pos in self.blocklist:
                continue
            else:
                if node.pos in list(map(lambda x: x.pos, self.openlist)):
                    node.update_fx(self.enode, cnode)
                else:
                    node.set_fx(self.enode, cnode)
                    self.openlist.append(node)

    def setblock(self, blocklist):
        """
        获取地图中的障碍物节点，并存入self.blocklist列表中
        注意：self.blocklist列表中存储的是障碍物坐标，不是Node类
        """
        self.blocklist = list()
        self.blocklist.extend(blocklist)
        # for pos in blocklist:
        #     block = Node(pos)
        #     self.blocklist.append(block)

    def get_neighbor(self, cnode):
        offsets = [(-1, 1), (0, 1), (1, 1), (-1, 0), (1, 0), (-1, -1), (0, -1), (1, -1)]  # 八面
        # offsets = [(-1, 0), (0, 1), (1, 0), (0, -1)]  # 四面
        nodes_neighbor = []
        x, y = cnode.pos[0], cnode.pos[1]
        for os in offsets:
            x_new, y_new = x + os[0], y + os[1]
            pos_new = (x_new, y_new)
            # 判断是否在地图范围内,超出范围跳过
            if x_new < self.snode.pos[0] or x_new > self.mapsize[0] - 1 \
                    or y_new < self.snode.pos[0] or y_new > self.mapsize[1]:
                continue
            nodes_neighbor.append(Node(pos_new))

        return nodes_neighbor


def give_start_and_end():
    return tuple((0, 0)), tuple((39, 36)),


def vx_vy(pos_now, pos_next):
    e = tuple((pos_next[1]-pos_now[1], pos_next[0]-pos_now[0]))
    d = math.sqrt(e[0]**2+e[1]**2)
    e = tuple((e[0]/d, e[1]/d))
    vx = e[0]
    vy = e[1]
    return vx, vy





rospy.init_node('Path_Decision',anonymous=True)
vel_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=1000)

def callback(msg):
    print('yes')
    point_cloud_data = pc2.read_points_list(msg,field_names=("x", "y", "z"),skip_nans=True)
    # print(block)
    twist = Twist()
    n = 100
    # print(point_cloud_data)
    mapsize = tuple((n, n))
    pos_now, pos_enode = give_start_and_end()
    routelist = list()  # 记录从起点到终点的路径
    routelist.append(pos_now)
    # while pos_now != pos_enode:
    myastar = AStar(mapsize, pos_now, pos_enode)  # 类的初始化
    # point_cloud_data = dn.create_pcd()
    create_block = Blocklist(SCALE, point_cloud_data)
    blocklist = create_block.pcd_discretization()
    myastar.setblock(blocklist)
    print(pos_now)
    print(blocklist)
    if myastar.run() != -1:
        while pos_now != pos_enode:
            tmp = myastar.get_minroute()  # 获得路径
            print(pos_now)

            if tmp.empty:
                pos_tmp =tmp.get()
                twist.linear.x,twist.linear.y =vx_vy(pos_now, pos_tmp)
                twist.angular.x = twist.angular.y = twist.angular.z = 0
                twist.linear.z = 0
                pos_now = pos_tmp
                vel_pub.publish(twist)
                # time.sleep(2)
                # routelist.append(pos_now)
            else: 
                break

    # for each in block:
    #     print(each.x,each.y,each.z)
    # twist = Twist()
    # vel_pub.publish()


if __name__=='__main__':
    
    rospy.Subscriber('/PointClouds',PointCloud2,callback,queue_size=1000)
    rospy.spin()


# from sensor_msgs.msg import PointCloud2
# from sensor_msgs import point_cloud2
# import rospy
# import time

# def callback_pointcloud(data):
#     assert isinstance(data, PointCloud2)
#     gen = point_cloud2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)
#     time.sleep(1)
#     print type(gen)
#     for p in gen:
#       print " x : %.3f  y: %.3f  z: %.3f" %(p[0],p[1],p[2])

# def main():
#     rospy.init_node('pcl_listener', anonymous=True)
#     rospy.Subscriber('/my_camera/depth/points', PointCloud2, callback_pointcloud)
#     rospy.spin()


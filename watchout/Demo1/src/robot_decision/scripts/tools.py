#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import math
# from random import uniform, randint
import pygame
# from enum import Enum
import time
import rospy
from sensor_msgs.msg import PointCloud2

import sensor_msgs.point_cloud2 as pc2

# 定义全局变量：地图中节点的像素大小
CELL_WIDTH = 25  # 单元格宽度
CELL_HEIGHT = 25  # 单元格长度
BORDER_WIDTH = 1  # 边框宽度
REFLESH = 0.2  # 循环执行时间
WINDOW = 30  # 窗口尺寸


class Color(object):
    """
    定义颜色
    """
    GRID = (190, 235, 243)
    OBJECT = (65, 20, 243)
    END = (255, 0, 0)
    BLOCK = (0, 0, 0)

    @staticmethod
    def random_color():
        """
    设置随机颜色
        """
        r = randint(0, 255)
        g = randint(0, 255)
        b = randint(0, 255)
        return r, g, b


class Map(object):
    def __init__(self, mapsize):
        self.mapsize = mapsize

    def generate_cell(self, cell_width, cell_height):
        """
        定义一个生成器，用来生成地图中的所有节点坐标
        :param cell_width: 节点宽度
        :param cell_height: 节点长度
        :return: 返回地图中的节点
        """
        x_cell = -cell_width
        for num_x in range(self.mapsize[0] // cell_width):
            y_cell = -cell_height
            x_cell += cell_width
            for num_y in range(self.mapsize[1] // cell_height):
                y_cell += cell_height
                yield (x_cell, y_cell)


def transform(pos):
    xnew, ynew = pos[0]*CELL_WIDTH, pos[1]*CELL_HEIGHT
    return xnew, ynew


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
        # return list(map(lambda x: floor_tuple(self.scale, x), self.pcd))
        # 两套方案，一种不不考虑重复，另一种考虑重复


# def create_pcd():
#     w = list()
#     for i in range(0, 50):
#         x = tuple((uniform(-30, 30), uniform(-30, 30), randint(-25, -25)))
#         if x not in w:
#             w.append(x)
#     return w


class Visualization(object):
    """
    点云数据可视化
    虽然有自带软件的可视化，但是不是很满足项目的需求，我重写一个
    """
    def __init__(self, blocklist, pos_now, pos_end, mapsize):
        self.pos_now, self.blocklist, self.mapsize = self.change_xy(pos_now, blocklist, mapsize)
        self.pos_end = pos_end

    @staticmethod
    def draw(mymap, screen, bl_pix, pix_now):
        # 绘制屏幕中的所有单元格
        for (x, y) in mymap.generate_cell(CELL_WIDTH, CELL_HEIGHT):
            if (x, y) in bl_pix:
                # 绘制黑色的障碍物单元格，并留出2个像素的边框
                pygame.draw.rect(screen, Color.BLOCK.value,
                                 ((x+BORDER_WIDTH, y+BORDER_WIDTH),
                                  (CELL_WIDTH-2*BORDER_WIDTH, CELL_HEIGHT-2*BORDER_WIDTH))
                                 )
            else:
                # 绘制可通行单元格，并留出2个像素的边框
                pygame.draw.rect(screen, Color.GRID.value,
                                 ((x+BORDER_WIDTH, y+BORDER_WIDTH),
                                  (CELL_WIDTH-2*BORDER_WIDTH, CELL_HEIGHT-2*BORDER_WIDTH))
                                 )
        pygame.draw.circle(screen, Color.OBJECT.value,
                           (pix_now[0]+CELL_WIDTH//2, pix_now[1]+CELL_HEIGHT//2), CELL_WIDTH//2 - 1)
        pygame.display.flip()

    @staticmethod
    def change_xy(pos_now, blocklist, mapsize=50):  # 之前的可视化只有第一象限，现在改为四个象限,初始原点为(25,25)
        mapsize = mapsize + 2
        tmp = math.floor(mapsize/2)-1
        pos_now = tuple((tmp+pos_now[0], tmp+pos_now[0]))
        blocklist = list(map(lambda block: tuple((tmp+block[0], tmp+block[1])), blocklist))
        return pos_now, blocklist, mapsize

    def visual(self):
        # 初始化导入Pygame模块
        pygame.init()
        # 此处要将地图投影大小转换为像素大小，此处设地图中每个单元格的大小为CELL_WIDTH*CELL_HEIGHT像素
        mymap = Map((self.mapsize*CELL_WIDTH, self.mapsize*CELL_HEIGHT))
        # 初始化显示的窗口并设置尺寸
        screen = pygame.display.set_mode((WINDOW*CELL_WIDTH, WINDOW*CELL_HEIGHT))
        t_end = time.time() + REFLESH
        while time.time() < t_end:
            pygame.display.set_caption('点云数据与目标位置演示：')
            bl_pix = list(map(transform, self.blocklist))  # 转换为像素
            pix_now = (self.pos_now[0]*CELL_WIDTH, self.pos_now[1]*CELL_HEIGHT)
            self.draw(mymap, screen, bl_pix, pix_now)
            # for event in pygame.event.get():
            #     pygame.display.flip()
            #     if event.type == pygame.QUIT:
            #         pygame.quit()
            #         return


def main(msg):
    scale = 2
    while True:
        # point_cloud_data = create_pcd()
        point_cloud_data = pc2.read_points_list(msg,field_names=("x", "y"),skip_nans=True)
        # print(point_cloud_data)
        create_block = Blocklist(scale, point_cloud_data)
        blocklist = create_block.pcd_discretization()
        # print(blocklist)
        pos_now = tuple((0, 0))
        pos_end = tuple((39, 39))
        vis = Visualization(blocklist, pos_now, pos_end, mapsize=30)
        vis.visual()
        for event in pygame.event.get():
            pygame.display.flip()
            if event.type == pygame.QUIT:
                pygame.quit()
                return


if __name__ == "__main__":
    rospy.init_node("fsaf")
    rospy.Subscriber("/PointClouds",PointCloud2,main,queue_size=1)

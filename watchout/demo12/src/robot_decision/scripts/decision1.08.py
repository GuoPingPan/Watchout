#!/usr/bin/env python
#-*- coding:UTF-8 -*-
import math
import pygame
import time
import multiprocessing
# from tools import create_pcd
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf
import threading
from rospy import client
# 定义全局变量：地图中节点的像素大小
CELL_WIDTH = 8  # 单元格宽度
CELL_HEIGHT = 8  # 单元格长度
BORDER_WIDTH = 0  # 边框宽度
REFLESH = 0.1  # 停顿时间
MAPSIZE = 100  # 窗口尺寸
MAXRANK = 3  # 最长记忆时间(单位：次)
SCALE = 0.4


# class _WFM(object):
#     def __init__(self):
#         self.msg = None
#     def cb(self, msg):
#         if self.msg is None:
#             self.msg = msg
class Color(object):
    """
    定义颜色
    """
    GRID = (190, 235, 243)
    OBJECT = (65, 20, 243)
    END = (0, 0, 255)
    ROUTE = (255, 0, 0)
    BLOCK = (0, 0, 0)


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
                yield x_cell, y_cell


def transform(pos):
    xnew, ynew = pos[0]*CELL_WIDTH, pos[1]*CELL_HEIGHT
    return xnew, ynew


class Visualization(object):
    """
    点云数据可视化
    虽然有自带软件的可视化，但是不是很满足项目的需求，我重写一个
    """
    def __init__(self, blocklist, pos_now, pos_end,  routelist, mapsize=MAPSIZE):
        self.pos_now, self.pos_end, self.blocklist, self.routelist, self.mapsize = self.change_xy(pos_now,
                                                                                                  pos_end,
                                                                                                  blocklist,
                                                                                                  routelist,
                                                                                                  mapsize)

    @staticmethod
    def draw(mymap, screen, bl_pix, pix_now, pix_end, ro_pix):
        # 绘制屏幕中的所有单元格
        for (x, y) in mymap.generate_cell(CELL_WIDTH, CELL_HEIGHT):
            if (x, y) in bl_pix:
                # 绘制黑色的障碍物单元格，并留出2个像素的边框
                pygame.draw.rect(screen, Color.BLOCK,
                                 ((x+BORDER_WIDTH, y+BORDER_WIDTH),
                                  (CELL_WIDTH-2*BORDER_WIDTH, CELL_HEIGHT-2*BORDER_WIDTH))
                                 )
            elif (x, y) in ro_pix:
                # 绘制规划路径
                pygame.draw.rect(screen, Color.ROUTE,
                                 ((x+BORDER_WIDTH, y+BORDER_WIDTH),
                                  (CELL_WIDTH-2*BORDER_WIDTH, CELL_HEIGHT-2*BORDER_WIDTH))
                                 )
            else:
                # 绘制可通行单元格，并留出2个像素的边框
                pygame.draw.rect(screen, Color.GRID,
                                 ((x+BORDER_WIDTH, y+BORDER_WIDTH),
                                  (CELL_WIDTH-2*BORDER_WIDTH, CELL_HEIGHT-2*BORDER_WIDTH))
                                 )
        pygame.draw.circle(screen, Color.OBJECT,
                           (int(pix_now[0])+CELL_WIDTH//2, int(pix_now[1])+CELL_HEIGHT//2), CELL_WIDTH//2 - 1)
        pygame.draw.circle(screen, Color.END,
                           (int(pix_end[0])+CELL_WIDTH//2, int(pix_end[1])+CELL_HEIGHT//2), CELL_WIDTH//2 - 1)
        pygame.display.flip()

    @staticmethod
    def change_xy(pos_now, pos_end, blocklist, routelist, mapsize=50):
        # 之前的可视化只有第一象限，现在改为四个象限,初始原点为(25,25)
        mapsize = mapsize + 2
        tmp = math.floor(mapsize/2)-1
        # 上为x轴正方向，左为y轴正方向
        pos_now = tuple((tmp - pos_now[1], tmp - pos_now[0]))
        pos_end = tuple((tmp - pos_end[1], tmp - pos_end[0]))
        blocklist = list(map(lambda block: tuple((tmp - block[1], tmp - block[0])), blocklist))
        routelist = list(map(lambda route: tuple((tmp - route[1], tmp - route[0])), routelist))
        return pos_now, pos_end, blocklist, routelist, mapsize

    def visual(self):
        # 初始化导入Pygame模块
        pygame.init()
        # 此处要将地图投影大小转换为像素大小，此处设地图中每个单元格的大小为CELL_WIDTH*CELL_HEIGHT像素
        mymap = Map((self.mapsize*CELL_WIDTH, self.mapsize*CELL_HEIGHT))
        # 初始化显示的窗口并设置尺寸
        screen = pygame.display.set_mode((self.mapsize*CELL_WIDTH, self.mapsize*CELL_HEIGHT))
        t_end = time.time() + REFLESH
        while time.time() < t_end:
            pygame.display.set_caption('demonstrate')
            bl_pix = list(map(transform, self.blocklist))  # 转换为像素
            ro_pix = list(map(transform, self.routelist))
            pix_now = (self.pos_now[0]*CELL_WIDTH, self.pos_now[1]*CELL_HEIGHT)
            pix_end = (self.pos_end[0]*CELL_WIDTH, self.pos_end[1]*CELL_HEIGHT)
            self.draw(mymap, screen, bl_pix, pix_now, pix_end, ro_pix)


def check(blocklist, routelist):
    """
    check函数，用于减少(不必要的)重新规划的次数
    :param blocklist: 障碍物列表list((x,y)) todo 事实上当list太大的时候运算量会增加，应该改成“新刷新的障碍物列表”
    :param routelist: 路径列表list((x,y))
    :return: 如果障碍物出现在了规划的路径上，则需要重新规划，返回0。反之则返回1
    """
    if not routelist:
        return 0
    for block in blocklist:
        if block in routelist:
            return 0
    return 1


def give_start_and_end():
    """
    给出起点与终点
    :return: 起点与终点(元组)
    """
    return tuple((0, 0)), tuple((0, -30))


def pos_to_angle(pos_now, pos_next):
    """
    输入两个坐标，返回关于向量关于x轴的夹角(-pi, pi)
    :param pos_now: 当前位置
    :param pos_next: 下一个位置
    :return: angle
    """
    dx = pos_next[0] - pos_now[0]
    dy = pos_next[1] - pos_now[1]
    if dx == 0:
        if dy <0 :
            a = -math.pi / 2
        elif dy == 0:
            a = 0
        else:
            a = math.pi
    else:
        angle = math.atan(dy / dx)
        if dx > 0 and dy > 0:
            a = angle
        elif dx< 0 and dy > 0:
            a = angle + math.pi / 2
        elif dx > 0 and dy < 0:
            a = - angle
        elif dx < 0 and dy < 0:
            a = -angle - math.pi / 2
        elif dx>0 and dy == 0:
            a = 0
        else:
            a = math.pi
    return round(a,2)

def handle(routelist):
    """
    人为简单加工routelist
    一方面使其元素(x, y)再包含一个angle表示角度
    另一方面使其过度尽量平滑
        目前来讲就是在拐弯处取两个线段的三分之而长的点，抛弃拐弯点以后连接两个点
    :param routelist:
    :return: routelist
    """
    routelist_new = list()
    angle = tuple((0, ))
    for i in range(len(routelist)):
        now = routelist[i] + angle
        if i< len(routelist) - 1:
            next = routelist[i+1]
            pos = tuple((
                round((next[0]*2 + now[0])*0.33, 2),
                round((next[1]*2 + now[1])*0.33, 2)))
            angle = tuple((pos_to_angle(now, pos), ))
            if not i:
                now = tuple((now[0], now[1])) + angle
                routelist_new.append(now)
            pos = pos + angle
            routelist_new.append(pos)

        else:
            if not i:
                routelist_new.append(now)
            else:
                next = routelist[i] + angle
                routelist_new.append(next)
            break
    return routelist_new, routelist


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


class Block(object):
    def __init__(self, pos):
        self.pos = pos
        self.rank = 0

    def update(self):
        self.rank = self.rank+1


class Blocklist(object):
    """
    Blocklist类，接收来自点云的数据，离散化为一组元组，代表A*中的blocklist
    2021.4.9更新：
        Blocklist类新增记忆前三此点云数据功能，达到稳定点云图的目的。
        为此新增Block类，作为blocklist中的基本元素元素。
        包含两个值：pos-->坐标
                   rank-->等级[1,3]，当rank>3时，将之从Blocklist中删除
        Block类如上。
    """
    def __init__(self, point_cloud_data, pre_blocks, pre_blocklist, maxrank=MAXRANK, scale=SCALE):
        """
        初始化类，此类应该包含几个参数：
        :param scale: 缩放比例
        :param point_cloud_data: 原始点云数据
        :param pre_blocks: 上一次处理后的blocks(由block组成)列表
        :param pre_blocklist: 上一次处理后的blocklist(由block.pos组成)列表
        :param maxrank 最长记忆时间(单位：次)
        """
        self.scale = scale
        self.maxrank = maxrank
        self.pcd = point_cloud_data
        self.now_blocks = pre_blocks  # 最终应该是这次点云数据和p_b的合并
        self.now_blocklist = pre_blocklist  # 最终应该是这次点云数据和p_bl的合并

    def pcd_discretization(self, p_n=None, p_e=None):
        """
        2021.4.9更新：
            为了达到本次更新所达到的效果，将此函数新增刷新block.rank、删除block功能
        :return: 形式正确的now_blocklist
        """
        # 添加，更新：
        for tmp1 in self.pcd:
            block_pos = floor_tuple(self.scale, tmp1)  # todo 改进离散化函数
            # 这里的blocklist起到了方便遍历的作用，但是最终是要用blocks刷新blocklist的
            if block_pos not in self.now_blocklist and block_pos != p_n and block_pos != p_e:
                self.now_blocklist.append(block_pos)
                block = Block(block_pos)
                self.now_blocks.append(block)
            else:
                for i in range(len(self.now_blocks)):
                    if self.now_blocks[i].pos == block_pos:
                        self.now_blocks[i].rank = 0
        # 刷新，删除
        n = len(self.now_blocks)
        tmp2 = 0
        while tmp2 < n:
            try:
                self.now_blocks[tmp2].update()
            except KeyError:
                break
            # 其实这个try是多余的
            if self.now_blocks[tmp2].rank > self.maxrank:
                del self.now_blocks[tmp2]
                tmp2 = tmp2 - 1
                n = n - 1
            tmp2 += 1
        # 生成blocklist
        self.now_blocklist = list()
        for tmp3 in self.now_blocks:
            self.now_blocklist.append(tmp3.pos)
        return self.now_blocklist, self.now_blocks
        # return list(map(lambda x: floor_tuple(self.scale, x), self.pcd))
        # 两套方案，一种不不考虑重复，另一种考虑重复

    @staticmethod
    def clean(blocklist):
        """
        这个函数没有实际意义，单纯就是为了可视化的时候block不会出现在(0,0)
        """
        tmp = tuple((0, 0))

        n = len(blocklist)
        i = 0
        while i < n:
            if blocklist[i] == tmp:
                del blocklist[i]
                n = n-1
                i = i-1
            i += 1
        return blocklist


class Node(object):
    """
    定义节点类型，每一个节点类实际上包含了:
    1.当前节点位置pos = (x,y)
    2.当前节点从起点到当前节点CNode的实际最小距离gvalue = g(x)
    3.当前状态下的目标函数fvalue = f(x)
    4.当前节点的父节点位置
    --顺便包含与上述数据有关的处理函数
    """
    def __init__(self, pos):
        self.pos = pos
        self.father = None
        self.gvalue = 0
        self.fvalue = 0

    def compute_fx(self, enode, father):
        if not father:
            print('??')
        gx_father = father.gvalue  # g'(n1)
        # 采用欧式距离计算父节点到当前节点的距离  d(n1,n2)
        gx_f2n = math.sqrt((father.pos[0] - self.pos[0])**2 + (father.pos[1] - self.pos[1])**2)
        gvalue = gx_f2n + gx_father  # 对于子节点的g(n2) = g(n1) + d(n1,n2)
        # 利用欧式距离计算该点到终点的距离 h(n2)
        hx_n2enode = math.sqrt((self.pos[0] - enode.pos[0])**2 + (self.pos[1] - enode.pos[1])**2)
        fvalue = gvalue + hx_n2enode  # f(n2) = h(n2) + g(n2) todo 改写目标函数的计算公式，寻找最优解
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
    def __init__(self, pos_now, pos_end, blocklist):
        self.openlist, self.closelist = [], []
        # openlist 待搜索点
        # closelist 已规划点
        self.blocklist = blocklist
        # blocklist 障碍物点
        self.snode = Node(pos_now)   # 存储当前点
        self.enode = Node(pos_end)   # 用于存储路径规划的目标节点
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
            # (大概)
            index_min = fxlist.index(min(fxlist))  # 寻找出fxlist中目标函数最小的点
            self.cnode = self.openlist[index_min]  # openlist和fxlist中得元素一一对应，则最小值的下标也就是最小值点的索引
            del self.openlist[index_min]  # 在openlist中删除index_min节点
            self.closelist.append(self.cnode)  # 在closelist中加入index_min节点
            # 扩展当前fx最小的节点，并进入下一次循环搜索
            self._extend(self.cnode)
            # 如果openlist列表为空，或者当前搜索节点为目标节点，则跳出循环
            if len(self.openlist) == 0 or self.cnode.pos == self.enode.pos:
                break
        if self.cnode.pos == self.enode.pos:
            self.enode.father = self.cnode.father
            return self.closelist[1].pos  # 返回下一步该走的位置
        else:
            return -1  # 停止不动

    def get_minroute(self):
        minroute = list()
        current_node = self.enode

        while True:
            minroute.append(current_node.pos)
            current_node = current_node.father  # 这里的思想和Dijkstra算法类似，在返回路径的时候，返回其父节点
            if current_node.pos == self.snode.pos:  # 找到根节点
                # minroute.append(current_node.pos)  # 加入父节点，即起始节点pos_now
                break
        minroute.reverse()
        return minroute

    def _extend(self, cnode):  # 作用是加入当前目标函数最小的节点周围不在closelist和blocklist中的节点进入openlist
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

    @staticmethod
    def get_neighbor(cnode):
        # offsets = [(-1, 1), (0, 1), (1, 1), (-1, 0), (1, 0), (-1, -1), (0, -1), (1, -1)]  # 八面
        offsets = [(-1, 0), (0, 1), (1, 0), (0, -1)]  # 四面
        nodes_neighbor = []
        x, y = cnode.pos[0], cnode.pos[1]
        for os in offsets:
            x_new, y_new = x + os[0], y + os[1]
            pos_new = (x_new, y_new)
            nodes_neighbor.append(Node(pos_new))

        return nodes_neighbor


def change_scale(pos, scale):
    """
    因为建图的比例尺的原因，图上距离和实际距离是相差一个scale的
    :param pos: 比例尺
    :param scale: 比例尺
    :return: 图上位置
    """
    return tuple((pos[0] / scale, pos[1] / scale, pos[2]))


def standardisation_pos(pos, left=2):
    """
    位置信息标准化，将每个位置信息元素保留两位小数
    :param pos: 位置元组
    :param left: 保留位数
    :return: pos_standardized
    """
    pos_standardized = tuple()
    for i in range(len(pos)):
        pos_standardized += tuple((round(float(pos[i]), left),))
    return pos_standardized


def check_pos(pos_now, pos_target, pos_error=0.2, angle_error=0.1):
    """
    检查当前位置是否到了目标位置
    引入误差，在误差允许的范围之内时，返回1，反之返回0
    :param pos_now: 当前位置
    :param pos_target: 目标位置
    :param pos_error: 位置误差，单位为米
    :param angle_error: 角度误差，弧度制
    :return: bool
    """
    for i in range(len(pos_target)):
        if i < 2:
            if pos_now[i] > pos_target[i] + pos_error or \
                    pos_now[i] < pos_target[i] - pos_error:
                return 0
        if i >=2:
            if pos_now[i] > pos_target[i] + angle_error or \
                    pos_now[i] < pos_target[i] - angle_error:
                return 0
    return 1

def transform_vx_vy_w(pos_now, pos_next, v=0.5):
    """
    使用pid法
    :param pos_now: 当前位置
    :param pos_next: 目标位置
    :param v: 合速度
    :return: x,y轴的速度vx、vy和z轴角速度w
    """
    delta_x = pos_next[0]-pos_now[0]
    delta_y = pos_next[1]-pos_now[1]
    delta_z = pos_next[2]-pos_now[2]
    vx = v*delta_x
    vy = v*delta_y
    w = 0.5*delta_z
    return vx, vy, w


def get_vx_vy(receive_route, q, scale):
    """
    速度、方向控制模块，获得规划的路径，控制小车按照规划的路径行走
    :param receive_route: 路径管道口
    :param q: 通信队列，传输当前位置
    :param scale: 比例尺
    思路：获得规划的路径列表，元素为元组，大目标为走完路径，小目标为从当前坐标走到routelist[0]，即下一个坐标pos_next
              积少成多,完成有限个小目标以后，就能完成总的大目标。
              首先接收当前的位置信息pos_now(x1,y1)和angle_now，通过routelist[0]得到pos_next
              输出为vx,vy和一个角度angle发送出去。需要注意的是vx,vy,和angle应该是持续发送的。
                     转换方式为(vx,vy)=p()
    """
    # rospy.init_node("vel")
    velpub = rospy.Publisher("/cmd_vel",Twist,queue_size=1) 
   
    routelist = receive_route.recv()
    print("receive routelist:")
    print(routelist)
    # odom = rospy.wait_for_message("/odom",Odometry,timeout=rospy.Duration(3.0))
    # siny_cosp = 2*(odom.pose.pose.orientation.w*odom.pose.pose.orientation.z +odom.pose.pose.orientation.x*odom.pose.pose.orientation.y)
    # cosy_cosp = 1-2*(pow(odom.pose.pose.orientation.y,2)+pow(odom.pose.pose.orientation.z,2))
    # theta = math.atan2(siny_cosp,cosy_cosp) 
    
    # pos_now = tuple((odom.pose.pose.position.x, odom.pose.pose.position.y, theta)) # 实际的pos_now，由传感器给我
    pos_now = tuple((0, 0, 0))
    pos_now = standardisation_pos(change_scale(pos_now, scale))
    while routelist:
        print("beginning to move...")
        pos_next = standardisation_pos(routelist[0])  # 获取目标位置, 做标准化处理
        print("pos_next:", pos_next)
        # pos_now格式：(x, y, angle), angle为小车转角
        while not check_pos(pos_now, pos_next):
            i = 0
            pos_now = tuple((pos_now[0], pos_now[1], 0)) # 实际的pos_now，由传感器给我
            pos_now = standardisation_pos(change_scale(pos_now, scale) )
            q.put(pos_now)
            # 位置确定函数，确定一个误差范围，当误差允许的情况下pos_now = pos_end，则输出1，否则为0
            vx,vy,w = transform_vx_vy_w(pos_now, pos_next)
            # 发布vx,vy,w
            print("vx=", vx)
            print("vy=", vy)
            print("w=", w)
            vel = Twist()
            vel.linear.x = vx
            vel.linear.y = vy
            vel.angular.z = w
            velpub.publish(vel)
            # time.sleep(0.1)
            if i ==2:
                pos_now = pos_next
                break
        del routelist[0]

    # 下面这个循环是防止进程在routelist为空时堵塞
    while True:
        # wfm1= _WFM()
        # odomsub = rospy.topics.Subscriber("/odom",Odometry,wfm1.cb)
        # odom = wfm1.msg
        # odomsub.unregister()
        # print(odom)
        # odom = rospy.wait_for_message("/odom",Odometry)
        # siny_cosp = 2*(odom.pose.pose.orientation.w*odom.pose.pose.orientation.z +odom.pose.pose.orientation.x*odom.pose.pose.orientation.y)
        # cosy_cosp = 1-2*(pow(odom.pose.pose.orientation.y,2)+pow(odom.pose.pose.orientation.z,2))
        # theta = math.atan2(siny_cosp,cosy_cosp) 
        
        # pos_now = tuple((odom.pose.pose.position.x, odom.pose.pose.position.y, theta)) # 实际的pos_now，由传感器给我
        # pos_now = standardisation_pos(change_scale(pos_now, scale))

        # pos_now = tuple((0, 0, 0)) # 实际的pos_now，由传感器给我
        # if q.full():
        #     _ = q.get()
            
        # print("stay still\nsending pos_now: ", pos_now)
        pos_now = tuple((0,0,0))
        q.put(pos_now)
        vx,vy,w = 0.0, 0.0, 0.0  # 保持当前状态,啥都不变
        vel = Twist()
        vel.linear.x = vx
        vel.linear.y = vy
        vel.angular.z = w
        velpub.publish(vel)
        # time.sleep(0.1)




def path_planning(msg):
    """
    路径规划程序，基本思路：
        循环判别:pos_now!=pos_end--->
        接收点云数据--->判断是否要规划
                                                             --->是，重新规划，重启控制进程，向控制进程传入规划路径routelist
                                                             --->否，跳过规划
    todo 添加轨迹优化算法
    blocks
    """
    ps_now, ps_end = give_start_and_end()  # 起点与终点
    # 定义控制子进程和通讯管道和队列
    q = multiprocessing.Queue(maxsize=1)  # 传输当前位置信息
    receive_route,  send_route = multiprocessing.Pipe(duplex=False)  # 传输规划路径
    processing_control = multiprocessing.Process(target=get_vx_vy, args=(receive_route, q, SCALE))
    # 定义一系列必要变量
    blocks = list()  # 包含记忆参数的障碍物列表，一个元素包含(x,y)和rank
    blocklist = list()  # 单纯的(x,y)的障碍物列表
    routelist = list()  # 路径列表
    flag = 0
    # i = 0
        # point_cloud_data = create_pcd()  # 接收点云数据
        # pcl_raw = rospy.wait_for_message("/PointClouds",PointCloud2,timeout=rospy.Duration(3.0))
        # wfm = _WFM()
        # pclsub = rospy.topics.Subscriber("/PointClouds",PointCloud2,wfm.cb)
        # pcl_raw = wfm.msg
        # pclsub.unregister()
        # print(pcl_raw)

        point_cloud_data = pc2.read_points_list(msg,field_names=('x','y'),skip_nans=True)
        create_block = Blocklist(point_cloud_data, blocks, blocklist, maxrank=3, scale=SCALE)  # 类的初始化
        blocklist, blocks = create_block.pcd_discretization(p_n=ps_now, p_e=ps_end,)  # 生成障碍物列表

        # 4.17 当四面都被围住的时候程序阻塞问题解决，原因是Motion_Models模块没有发布信息
        # blocklist.append(tuple((-1, 0)))
        # blocklist.append(tuple((1, 0)))
        # blocklist.append(tuple((0, 1)))
        # if i % 2 == 0:
        #     blocklist.append(tuple((0, -1)))
        #     i += 1
        # else:
        #     del blocklist[len(blocklist) - 1]
        #     i += 1
        # print(blocklist)

        if not check(blocklist, routelist):  # 开始、重新规划
            # 获取当前位置信息
            if not flag:
                flag = 1
            else:
                # print("receiving pos_now...\nqueuesize:", q.qsize())
                ps_now = q.get(timeout=2)
                # print("receive pos_now: ", ps_now)

            ps_now = tuple((int(ps_now[0]), int(ps_now[1])))
            myastar = AStar(ps_now, ps_end, blocklist)
            myastar.run()
            # print("getting routelist...")
            routelist_new, routelist = handle(myastar.get_minroute())
            # ps_end = routelist_new[-1]
            # routelist格式：pos_now--->pos_next--->pos_next_next...
            # print(routelist)

            if processing_control.is_alive():
                # print("processing_control closing...")
                processing_control.terminate()
                # processing_control.join()
                # print(processing_control.is_alive())
                # processing_control.close()
                # print("processing_control closed")
                processing_control = multiprocessing.Process(target=get_vx_vy, args=(receive_route, q, SCALE))
                processing_control.start()
                print("processing_control restart")
            else:
                processing_control.start()
                print("processing_control start")

            send_route.send(routelist_new)
            # print("send routelist")
        else:
            _ = q.get()
        vis = Visualization(blocklist, ps_now, ps_end, routelist)
        vis.visual()

def finish():
    print("to the end")


def main():
    rospy.init_node("decision")
    pointsub = rospy.Subscriber("/PointClouds",PointCloud2,path_planning)
    # path_planning()
    while True:
        if(not check_pos(ps_now, ps_end)):
            rospy.on_shutdown(finish)
            exit()

    rospy.spin()


if __name__ == "__main__":
    main()
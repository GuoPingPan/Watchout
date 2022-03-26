#!/usr/bin/env python
#-*- coding:UTF-8 -*-
import math
import pygame
import time
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import rospy
from geometry_msgs.msg import Twist,PoseStamped
import threading
from nav_msgs.msg import Odometry
import multiprocessing


# 定义全局变量：地图中节点的像素大小
CELL_WIDTH = 12  # 单元格宽度
CELL_HEIGHT = 12  # 单元格长度
BORDER_WIDTH = 0.5  # 边框宽度
REFLESH = 0.1  # 循环执行时间
MAPSIZE = 60  # 窗口尺寸
SCALE = 0.2  # 缩放量
MAXRANK = 3  # 最长记忆时间(单位：次)


class Color(object):
    """
    定义颜色
    """
    GRID = (190, 235, 243)
    OBJECT = (65, 20, 243)
    END = (255, 0, 0)
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
    def __init__(self, scale, point_cloud_data, pre_blocks, pre_blocklist, maxrank):
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

        # s = 0  # 单纯是为了验证前面的算法是否正确
        # for x in blocklist:
        #     if x == tmp:
        #         s = s + 1
        # print(s)
        # return blocklist


class Visualization(object):
    """
    点云数据可视化
    虽然有自带软件的可视化，但是不是很满足项目的需求，我重写一个
    """
    def __init__(self, blocklist, pos_now, pos_end, mapsize, routelist):
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
                           (int(pix_now[0])+int(CELL_WIDTH//2),int( pix_now[1])+int(CELL_HEIGHT//2)), int(CELL_WIDTH)//2 - 1)
        pygame.draw.circle(screen, Color.END,
                           (int(pix_end[0])+int(CELL_WIDTH//2),int( pix_end[1])+int(CELL_HEIGHT//2)), int(CELL_WIDTH)//2 - 1)
        pygame.display.flip()

    @staticmethod
    def change_xy(pos_now, pos_end, blocklist, routelist, mapsize=50):
        # 之前的可视化只有第一象限，现在改为四个象限,初始原点为(25,25)
        mapsize = mapsize + 2
        tmp = math.floor(mapsize/2)-1
        pos_now = tuple((tmp+pos_now[0], tmp+pos_now[0]))
        pos_end = tuple((tmp+pos_end[0], tmp+pos_end[1]))
        blocklist = list(map(lambda block: tuple((tmp+block[0], tmp+block[1])), blocklist))
        routelist = list(map(lambda route: tuple((tmp+route[0], tmp+route[1])), routelist))
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
            pygame.display.set_caption('example:')
            bl_pix = list(map(transform, self.blocklist))  # 转换为像素
            ro_pix = list(map(transform, self.routelist))
            pix_now = (self.pos_now[0]*CELL_WIDTH, self.pos_now[1]*CELL_HEIGHT)
            pix_end = (self.pos_end[0]*CELL_WIDTH, self.pos_end[1]*CELL_HEIGHT)
            self.draw(mymap, screen, bl_pix, pix_now, pix_end, ro_pix)


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
            print('??')
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
            self._extend(self.cnode)
            # 如果openlist列表为空，或者当前搜索节点为目标节点，则跳出循环
            if len(self.openlist) == 0 or self.cnode.pos == self.enode.pos:
                break
        if self.cnode.pos == self.enode.pos:
            self.enode.father = self.cnode.father
            return self.closelist[1]  # 返回下一步该走的位置
        else:
            return -1  # 停止不动

    def get_minroute(self):
        minroute = list()
        current_node = self.enode

        while True:
            minroute.append(current_node.pos)
            current_node = current_node.father  # 这里的思想和Dijkstra算法类似，在返回路径的时候，返回其父节点
            if current_node.pos == self.snode.pos:  # 找到根节点
                minroute.append(current_node.pos)
                break

        # minroute.put(self.snode.pos)  # 加入父节点  暂且不要
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

    def setblock(self, blocklist):
        """
        获取地图中的障碍物节点，并存入self.blocklist列表中
        注意：self.blocklist列表中存储的是障碍物坐标，不是Node类
        """
        self.blocklist = list()
        self.blocklist.extend(blocklist)

    @staticmethod
    def get_neighbor(cnode):
        # offsets = [(-1, 1), (0, 1), (1, 1), (-1, 0), (1, 0), (-1, -1), (0, -1), (1, -1)]  # 八面
        offsets = [(-1, 0), (0, 1), (1, 0), (0, -1)]  # 四面
        nodes_neighbor = []
        x, y = cnode.pos[0], cnode.pos[1]
        for os in offsets:
            x_new, y_new = x + os[0], y + os[1]
            pos_new = (x_new, y_new)
            # 判断是否在地图范围内,超出范围跳过
            nodes_neighbor.append(Node(pos_new))

        return nodes_neighbor


def give_start_and_end():
    return tuple((0, 0)), tuple((-30, -30)),

def get_begin_and_goal():
    goal = rospy.wait_for_message("/move_base_simple/goal",PoseStamped,timeout)
    return tuple((0,0)),tuple(int(goal.pose.position.x),int(goal.pose.position.y))





def check(blocklist, routelist):
    for block in blocklist:
        if block in routelist:
            return -1
    return 1


rospy.init_node("decision")
point_sub = rospy.Subscriber("/PointClouds",PointCloud2,CreatPath)
velpub = rospy.Publisher("/cmd_vel",Twist,queue_size=10)
   

p_next = -1
n_bl = list()
n_b = list()
routelist = list()
p_n,  p_e= give_start_and_end()
q, out = multiprocessing.Pipe()

def CreatPath(msg):
    """
    路径规划程序，作为一个线程存在
    :param n_bl: 障碍物列表
    :param n_b: 包含障碍物和记忆函数
    :param routelist: 路径
    :param p_n 当前位置
    :param p_e 结束位置
    :param q 管道口，用于向控制进程传输下一个位置的信息
    :return: 更新后的n_bl, n_b, routelist
    """
    assert isinstance(msg,PointCloud2)
    global p_n,p_e,n_bl,n_b,routelist,p_next,q
    while   p_n !=  p_e:
        # print(p_n)
        # 通讯,获得当前点云数据
        print("pcl")
        point_cloud_data = pc2.read_points_list(msg,field_names=('x','y'),skip_nans=True)
        create_block = Blocklist(SCALE, point_cloud_data, n_b, n_bl, maxrank=MAXRANK)
        # n_bl, n_b = create_block.pcd_discretization()
        n_bl, n_b = create_block.pcd_discretization(p_n=p_n, p_e=p_e)

        if p_next == -1 or check(n_bl, routelist) == -1:
            myastar = AStar(MAPSIZE, p_n, p_e)  # 类的初始化
            myastar.setblock(n_bl)
            p_next = myastar.run()
            routelist = myastar.get_minroute()
            routelist.reverse()
            # print(routelist)
        if p_n == p_e:
            return
        # print(0)
        if p_next == -1:
            continue
        if routelist:
            p_next = routelist[0]
            del routelist[0]
        # 通讯，向控制进程输入当前位置和路径信息
        # print("send:", p_next)
        # q.put(p_n, block=1)
        # q.put(p_next, block=True)
        q.send(p_next)
        # 通讯，获得当前位置
        p_n = p_next
        vis = Visualization(n_bl, p_n, p_e, mapsize=MAPSIZE, routelist=routelist)
        vis.visual()

def vx_vy(p_n, p_e, q):
    p_next = p_n
    vx_speed = 0.2
    v_turned = 0.1
    v = Twist()
    while p_n != p_e:
        # if not q.empty:
        #     p_next = q.get()
        p_next = q.recv()
        e = tuple((p_next[0]-p_n[0], p_next[1]-p_n[1]))
        d = math.sqrt(e[0]**2+e[1]**2)
        try:
            e = tuple((e[0]/d, e[1]/d))
        except ZeroDivisionError:
            e = tuple((0.0, 0.0))
        vx = e[0]
        vy = e[1]
        print(vx, vy)
        v.linear.x = vx*vx_speed
        v.linear.y = vx*vx_speed

        v.angular.z = math.atan2(vy,vx)

        while p_n != p_next:
            velpub.publish(v)
        # 通讯
            odom = rospy.wait_for_message("/odom",Odometry,timeout=rospy.Duration(3.0))
            p_n = tuple((math.floor(odom.pose.pose.position.x),math.floor(odom.pose.pose.position.y)))
        
        p_n = p_next



def main():
    # q = multiprocessing.Queue(maxsize=1)  # 用于进程间的下一位置的通讯
    # q.put(1)
    global p_n,p_next,out

    p = multiprocessing.Process(target=vx_vy, args=(p_n, p_next, out))
    p.start()
    # programming(now_blocklist, now_blocks, now_routelist, ps_now, ps_end, q)
    rospy.spin()

if __name__ == "__main__":
    main()
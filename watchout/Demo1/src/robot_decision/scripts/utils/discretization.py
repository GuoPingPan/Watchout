import math
from random import random


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
#     for i in range(0, 5):
#         x = tuple((400*random(), 400*random(), 100*random()))
#         if x not in w:
#             w.append(x)
#     return w


def main():
    scale = 2
    point_cloud_data = create_pcd()
    print(point_cloud_data)
    create_block = Blocklist(scale, point_cloud_data)
    blocklist = create_block.pcd_discretization()
    print(blocklist)


if __name__ == "__main__":
    main()

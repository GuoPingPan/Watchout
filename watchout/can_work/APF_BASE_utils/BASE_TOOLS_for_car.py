"""
修改Artificial Potential Field.py
使它适用于实机小车，要求：
保留可视化部分，但是各实体的位置变化不再由我人为规定，而是由外部传感器更新；
在小车物理运动规律限制部分，应该人为规定vx和vy的最大绝对值，引入角速度。
关于角速度，有如下设置：
在一个REFRESH时间间隔下，由当前传感器传入的姿态和计算出的将要发布出的速度，得出角速度。
需要说明的是，传感器传入的姿态角为z轴转角，以初始状态为参考系，大小为从-pi到pi，
我尽量将各个函数写成接口形式，简化主函数里的代码量，便于寻找。
"""
# 导入必要的包，pygame用于可视化数据，math用于做一些必要的数学运算
import math

# 定义全局变量：
REFRESH = 0.08  # 计算频率
# 定义机器人物理运动规律（加速度）
Acceleration_x = 1  # x轴加速度的最大值
Acceleration_y = 0.5  # y轴加速度的最大值


class APF(object):
    """
    人工势场法
    """
    def __init__(self, pos_now, pos_end, blocklist, d, p, q, s, k, h, vx, vy):
        """
        初始化APF类
        @param pos_now: 当前位置
        @param pos_end: 目标位置
        @param blocklist: 障碍物列表，包含信息有（x，y，vx，vy，w）
        相对地而言，前两个是位置信息，中间两个是速度信息，最后一个是障碍物大小（一半）
        @param d: 引力中的距离常量
        @param p: 引力常量
        @param q: 静止时的qj
        @param s: 梯度与速度的比例系数
        @param k: qj中的时间常量，k越大，相同速度下v就越大
        @param h: qj中的斥力常量，h越大，相同情况下该点的斥力就越大
        @param vx: 当前x轴速度
        @param vy: 当前y轴速度
        """
        self.pos_now = pos_now
        self.pos_end = pos_end
        self.blocklist = blocklist
        self.p = p
        self.d = d
        self.q = q
        self.s = s
        self.k = k
        self.h = h
        self.vx = vx
        self.vy = vy

    @staticmethod
    def get_string_length(cos, a, b):
        """
        获取当前状态下的qj
        @param cos: 由障碍物坐标指向机器人位置的向量与速度方向向量之间的夹角
        @param a: 长轴
        @param b: 短轴
        @return: 当前的qj
        """
        if cos < 0:
            # 当夹角为负值，当做静止处理
            cos = 0
        if cos < 0.7:
            # 当夹角大于45度小于90度，采用 x=ky 制
            k = cos / math.sqrt(max(0, 1 - cos**2))
            qj = a * b * math.sqrt(max(0.0, 1 + k**2)) / math.sqrt(max(0, b**2 * k**2 + a**2))
        else:
            # 当夹角小于等于45度大于0度，采用y=kx制
            k = math.sqrt(max(1 - cos**2, 0)) / cos
            qj = a * b * math.sqrt(1 + k**2) / math.sqrt(max(0, b**2 + k**2 * a**2))
        return qj

    @staticmethod
    def sigmoid(z):
        """
        sigmoid函数，作为改进势场函数时候的一个思路
        """
        return 1.5 / (1 + math.exp(-0.6*z))

    def get_rep_gradient(self, block):
        """
        传入障碍物列表中的元素
        :param block: (x_j, y_j, vx_j, vy_j)
        :return: delta_U_rep
        2021.5.3改进：增加斥力梯度影响因子，该因子与机器人和障碍物之间距离成反比
        2021.5.5思路：应该放大斥力中逃离障碍物范围的部分，减小斥力中使机器人速度减小的部分
        """
        x_r, y_r = self.pos_now
        _, x_b, y_b, vx, vy, w = block
        # k = math.sqrt(vx**2 + vy**2)
        delta_x = x_r - x_b
        delta_y = y_r - y_b
        d = math.sqrt((x_r - x_b)**2 + (y_r - y_b)**2)
        v = math.sqrt(vx**2+vy**2)
        if v <= 0.2:  # 当速度过小，将它当做静止物体处理
            tmp_cos = 0.7
            # tmp_cos与接下来的斥力梯度计算有关
            qj = self.q + w
        else:
            tmp = vx*delta_x + vy * delta_y
            # tmp是速度向量与坐标向量的点乘
            tmp_v = tmp / d
            # tmp_v速度向量与坐标的单位向量的点乘
            tmp_cos = tmp_v / v if tmp_v > 0 else 0
            # tmp_cos是速度的单位向量与坐标的单位向量的点乘，也就是夹角的cos值
            b = self.q + w + round(self.sigmoid(w), 2) + math.sqrt(self.vx**2 + self.vy**2)
            # 短轴长
            # 之所以加上一个sigmoid函数是考虑不能简单加上一个物体半径w，而应该加上一个w的函数，w越大函数值越大，但是有一个极限。
            a = v*self.k + b
            # 长轴长应该是在短轴基础上加上一个关于速度的函数值
            qj = self.get_string_length(tmp_cos, a, b)
            # 取当前的qj

            # 以下两个赋值语句分别是方案一和方案二的实现，直接用就行。
            # qj = self.k * max((vx * delta_x + vy * delta_y) / d, 0) +self.q + w
            # qj = math.exp(tmp_cos**2) * v**2 * (math.exp(tmp_cos) / math.e) **2 + self.q + w
        if d > qj:
            # 如果机器人与障碍物之间的距离d大于qj，斥力梯度为0
            return tuple((0, 0, 0, d - w))
        else:
            # 取斥力梯度的负值
            # tmp是梯度的系数，因为斥力函数是对称式的，因此系数是一样的
            # 对于max(tmp_cos, 0)，其中tmp_cos是当前机器人与障碍物速度的单位向量与坐标的单位向量的点乘，也就是夹角的cos值
            # 乘上这个系数的目的，一方面是为了斥力对机器人影响过渡更加平滑
            # 另一方面是为了机器人行驶到障碍物后边并与速度方向反向时，障碍物的斥力不会对机器人产生太大影响
            # todo 有一个漏洞是机器人超车的时候...
            try:
                tmp = self.h * qj * max(tmp_cos, 0.7) * (1/d - 1/qj) / d**(3 / 2)
                return tuple((tmp * delta_x, tmp * delta_y, self.h * qj / d, d-w))
            except ZeroDivisionError:
                return tuple((0, 0, 0, d-w))

    @staticmethod
    def judge_v_rep(vx, vy, delta_U_rep_x, delta_U_rep_y, model=1):
        f = 1
        # 默认顺时针
        v = math.sqrt(vx**2 + vy**2)
        # 计算合速度
        u_rep = math.sqrt(delta_U_rep_y**2 + delta_U_rep_x**2)
        # cos_tmp: 速度与斥力的夹角的cos值
        if v > 0.05:
            # 速度和零不可比，就对斥力做旋转
            cos_tmp = round((vx * delta_U_rep_x + vy * delta_U_rep_y) / v / u_rep, 2)
        else:
            # 速度和零可比，就不旋转
            return 0, -1
        if model == -1:
            # 不旋转模式
            return cos_tmp, -1
        if not model:
            # 默认逆时针旋转模式
            return cos_tmp, f

        # 接下来判断斥力梯度是顺时针还是逆时针旋转,依旧是运用到旋转矩阵
        # 先计算出速度与x轴(单位向量（1,0）)的夹角
        # 把速度旋转到x轴正向
        cos = vx / v
        sin = math.sqrt(1 - cos**2)
        # 先实验顺时针旋转
        tmp_x = vx * cos + vy * sin
        tmp_y = -vx*sin + vy*cos
        # 如果顺时针旋转后tmp_x >0 , 且math.fabs(tmp_y) < 0.1,说明旋转正确
        # 0.1 是误差
        if tmp_x > 0 and math.fabs(tmp_y) < 0.2:
            # 顺时针旋转正确，接着判断斥力梯度旋转后的y轴坐标大于还是小于0
            tmp_rep_y = -delta_U_rep_x*sin + delta_U_rep_y*cos
        else:
            # 顺时针旋转错误，应该逆时针时针旋转，接着判断斥力梯度旋转后的y轴坐标大于还是小于0
            tmp_rep_y = delta_U_rep_x*sin + delta_U_rep_y*cos
        if tmp_rep_y > 0:
            # 说明最终的斥力梯度应该顺时针
            f = 0
        else:
            f = 1
        return cos_tmp, f

    def disturbance(self, delta_U_att, delta_U_rep_x, delta_U_rep_y):
        """
        扰动函数
        我希望通过此函数能够实现机器人迅速避开障碍物，即当斥力与引力夹角theata的的cos值接近-1时，机器人能绕开
        初步的解决方法为，对斥力做旋转，有个旋转角f(theata),满足：
        theata是引力和斥力梯度的夹角；
        cos(theata)从0到-1变化，即theata从pi/2到pi变化的时候，旋转角从0变为pi/2；
        cos(theata)从0到 1变化，即theata从pi/2到0变化的时候，不旋转。
        2021.5.13：需要注意的是，旋转的方向应该是能够旋转到引力方向的角度最小的方向。
        """
        if (delta_U_rep_y or delta_U_rep_x) and (delta_U_att[0] or delta_U_att[1]):
            # 得到应力与斥力夹角的cos值和旗帜f，f = 0，顺时针；f=1，逆时针
            # cos_tmp, f = self.judge(delta_U_att, delta_U_rep_x, delta_U_rep_y, model=1)
            cos_tmp, f = self.judge_v_rep(self.vx, self.vy, delta_U_rep_x, delta_U_rep_y, model=1)
            # 判断旋转方向，好像还是有点问题，暂且不管。
            if cos_tmp < -0.7 and f != -1:
                # 夹角大于一定值，梯度旋转
                sin = math.sqrt((1 - cos_tmp) / 2)
                cos = math.sqrt((1 + cos_tmp)/2)
                # 由要求可知，旋转的角度theata'应该是theata - pi/2
                # 由三角恒等变换可知：
                # cos(theata') = cos(thaeta - pi/2) = sin(theata) > 0
                # sin(theata') = sin(thaeta - pi/2) = -cos(theata) > 0
                tmp_x = delta_U_rep_x
                tmp_y = delta_U_rep_y
                # 对斥力做旋转，运用到旋转矩阵
                if f == 0:
                    # 顺时针旋转
                    delta_U_rep_x = tmp_x * cos + tmp_y * sin
                    delta_U_rep_y = - tmp_x * sin + tmp_y * cos
                elif f == 1:
                    # 逆时针旋转
                    delta_U_rep_x = tmp_x * cos - tmp_y * sin
                    delta_U_rep_y = tmp_x * sin + tmp_y * cos
        return delta_U_att[0] + delta_U_rep_x, delta_U_att[1] + delta_U_rep_y

    def get_gradient(self, print_d=False):
        """
        计算当前梯度值
        """
        # 传入的坐标是实际坐标
        x_r, y_r = self.pos_now
        x_g, y_g = self.pos_end
        d = math.sqrt((x_r - x_g)**2 + (y_r - y_g)**2)
        # 取引力梯度的负值
        if d <= self.d:
            delta_U_att = tuple((-self.p*(x_r - x_g), -self.p*(y_r - y_g)))
        else:
            tmp = -self.d * self.p / (2 * d)
            delta_U_att = tuple((tmp*(x_r - x_g), tmp*(y_r - y_g)))
        # 斥力梯度
        delta_U_rep_list = list(map(self.get_rep_gradient, self.blocklist))
        delta_U_rep_x = 0
        delta_U_rep_y = 0
        min_d = 0
        if len(delta_U_rep_list):
            min_d = delta_U_rep_list[0][3]
        for tmp in delta_U_rep_list:
            min_d = tmp[3] if tmp[3] < min_d else min_d
            delta_U_rep_x += tmp[0] * tmp[2]
            delta_U_rep_y += tmp[1] * tmp[2]
        if print_d:
            print("离机器人最近的障碍物距离：", round(min_d, 2))
        delta_U_x, delta_U_y = self.disturbance(delta_U_att, delta_U_rep_x, delta_U_rep_y)
        # 返回当前梯度值
        return self.s*delta_U_x, self.s*delta_U_y




class Visualization(object):
    """
    利用pygame写一个虚拟仿真环境
    要求输入：障碍物信息:list((xj,yj,vx_j,vy_j))，机器人当前位置信息(x,y)，目标点信息(x_g,y_g)
    要求输出：机器人速度(vx,vy)
    因为是仿真环境，所以输出部分省略，直接在窗口上做出效果。
    此外，因为传入的数据为实际数据，因此我们需要在可视化的时候进行一定的放缩
    """
    def __init__(self, d, p, q, s, k, h):
        self.d = d
        self.p = p
        self.q = q
        self.s = s
        self.k = k
        self.h = h

    @staticmethod
    def motion(vx, vy, tmp_vx, tmp_vy, refresh_time=REFRESH):
        """
        对运动速度加以限制，使其更加符合机器人运动规律
        @param vx: 上一时刻x轴速度
        @param vy: 上一时刻y轴速度
        @param tmp_vx: 这一时刻理论x轴速度
        @param tmp_vy: 这一时刻理论y轴速度
        @param refresh_time: 两时刻间隔
        @return: 本时刻实际速度vx，vy
        """
        if math.fabs(vx - tmp_vx) / refresh_time > Acceleration_x:
            tmp_x = (vx + Acceleration_x * refresh_time) / tmp_vx if tmp_vx - vx > 0 \
                else (vx - Acceleration_x * refresh_time) / tmp_vx
        else:
            tmp_x = 1
        if math.fabs(vy - tmp_vy) / refresh_time > Acceleration_y:
            tmp_y = (vy + Acceleration_y * refresh_time) / tmp_vy if tmp_vy - vy > 0 \
                else (vy - Acceleration_y * refresh_time) / tmp_vy
        else:
            tmp_y = 1
        return tmp_vx*tmp_x, tmp_vy*tmp_y

    def visual(self, pos_now, pos_end, blocklist, vx, vy, print_v=False, print_d=False):
        """
        此函数作为集成函数，需要解决以下问题：
        输入了当前的机器人信息，障碍物位置信息，将这些信息在可视化窗口screen中表示出来
        利用人工势场法，输出当前状态下的决策速度
        @param pos_now: 当前位置信息（x,y,vx,vy,theata）
        @param pos_end: 目标点坐标（x,y）
        @param blocklist: 障碍物信息，列表里面为元组，元组格式（x,y,vx,vy,width）
        @param print_v: 是否打印当前速度，若为真则打印
        @param print_d: 是否打印当前与机器人位置最近的障碍物，若为真则打印
        @return: vx, vy
        """
        x, y, theata = pos_now
        pos_now = (x, y)
        # 获取期望速度
        apf = APF(pos_now, pos_end, blocklist,
                  self.d, self.p, self.q, self.s, self.k, self.h, vx=vx, vy=vy)
        tmp_vx, tmp_vy = apf.get_gradient(print_d)
        v = round(math.sqrt(vx**2 + vy**2), 2)
        # print("vx = ", round(vx, 2))
        # print("vy = ", round(vy, 2))
        if print_v:
            print("v = ", v)
        # 获取经过物理约束的速度
        vx, vy = self.motion(vx, vy, tmp_vx, tmp_vy)
        w = 0
        return vx, vy, w


def init():
    # 规定计算势场的一些参数
    d = 5
    # 引力中的距离常量，当机器人与目标点距离小于d时，引力梯度会逐渐减小
    p = 0.1
    # 引力常数，p越大算出的引力梯度值越大
    q = 1
    # 静止时的qj大小
    s = 0.8
    # 梯度与速度比例系数，s越大，速度越大
    k = 5
    # 动态计算中关于长轴的一个常量，k越大，长轴越长
    h = 0.8
    # 与斥力梯度的大小有关
    vis = Visualization(d, p, q, s, k, h)
    pos_end = get_pos_end(10, 0)
    return vis, pos_end


def Vis_and_deside(vis, pos_now, pos_end, blocklist, vx, vy, print_v=False, print_d=False):
    # 进行可视化等相关操作
    vx, vy, w  = vis.visual(pos_now=pos_now, pos_end=pos_end, blocklist=blocklist,
                            vx=vx, vy=vy, print_v=print_v, print_d=print_d)
    # 还要返回当前的screen，方便下一次修改(主要我不知道不返回会不会有问题)
    return vx, vy, w


def get_pos_end(x_g, y_g):
    return tuple((x_g, y_g))

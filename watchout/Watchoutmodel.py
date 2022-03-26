import sys

sys.path.insert(0, './yolov5')

from yolov5.utils.datasets import LoadImages, LoadStreams, LoadWebcam, LoadRealsense
from yolov5.utils.general import check_img_size, non_max_suppression, scale_coords
from yolov5.utils.torch_utils import select_device, time_synchronized
from deep_sort_pytorch.utils.parser import get_config
from deep_sort_pytorch.deep_sort import DeepSort
import argparse
import os
import platform
import shutil
import time
from pathlib import Path
import cv2
import torch
import torch.backends.cudnn as cudnn

import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
import rospy
from geometry_msgs.msg import Twist
from my_utils import bbox_rel,compute_color_for_labels,showdepth,goodenbox,draw_boxes,kafilter




class Watchout:
    def __init__(self):

        #   深度初始化
        self.lasttime = rospy.Time.now()
        self.thistime = rospy.Time.now()
        self.scale = 0.001
        self.idcenvel = []  # id cx,cy,vx,vy
        self.depth_thres = 10.0  # 深度阀值
        # 内参
        fx = 609.2713012695312
        cx = 316.67022705078125
        fy = 608.010498046875
        cy = 244.8178253173828
        self.K = np.array([[1.0 / fx, 0, -cx / fx],
                           [0, 1.0 / fy, -cy / fy],
                           [0.0, 0.0, 1.0]])

        self.translation = []
        self.rotation = []
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # Initialize
        source, weights, imgsz = '0', 'yolov5/weights/yolov5s.pt', 640
        self.view_img = True
        self.augment = False
        self.conf_thres = 0.4
        self.iou_thres = 0.5
        self.classes = [0]
        self.agnostic_nms = False

        # initialize deepsort
        cfg = get_config()
        cfg.merge_from_file('deep_sort_pytorch/configs/deep_sort.yaml')
        self.deepsort = DeepSort(cfg.DEEPSORT.REID_CKPT,
                                 max_dist=cfg.DEEPSORT.MAX_DIST,
                                 min_confidence=cfg.DEEPSORT.MIN_CONFIDENCE,
                                 nms_max_overlap=cfg.DEEPSORT.NMS_MAX_OVERLAP,
                                 max_iou_distance=cfg.DEEPSORT.MAX_IOU_DISTANCE,
                                 max_age=cfg.DEEPSORT.MAX_AGE,
                                 n_init=cfg.DEEPSORT.N_INIT,
                                 nn_budget=cfg.DEEPSORT.NN_BUDGET,
                                 use_cuda=True)
        self.device = select_device('')
        self.half = self.device.type != 'cpu'  # half precision only supported on CUDA

        # Load model
        self.model = torch.load(weights, map_location=self.device)['model'].float()  # load to FP32
        self.model.to(self.device).eval()
        if self.half:
            self.model.half()  # to FP16

        # Load dataset
        self.dataset = LoadRealsense('0', img_size=imgsz)
        self.view_img = True
        cudnn.benchmark = True  # set True to speed up constant image size inference

        # Get names and colors
        names = self.model.module.names if hasattr(self.model, 'module') else self.model.names

        # Run inference
        img = torch.zeros((1, 3, imgsz, imgsz), device=self.device)  # init img
        # run once
        _ = self.model(img.half() if self.half else img) if self.device.type != 'cpu' else None

    def drawsquare(self, xyxy, depth):
        # 计算公式 x = (u*depth - cx*depth)/fx   y = (v*depth - cy*depth)/fy
        # 先将 像素坐标 uv1 * depth
        u1, v1, u2, v2 = goodenbox(xyxy)

        uvd = []
        for u in range(u1, u2):
            for v in range(v1, v2):
                depth_ = float(depth[v, u]) * self.scale
                if depth_ > self.depth_thres:
                    continue
                else:
                    uvd.append([u * depth_, v * depth_, depth_])

        yzx = self.K.dot(np.array(uvd).T).T

        # 用均值代替质心
        cx = np.median(yzx[:, 2])
        cy = np.median(yzx[:, 0])

        if (cx == 0): cx = np.mean(yzx[:, 2])
        if (cy == 0): cy = np.mean(yzx[:, 0])


        from scipy.spatial.transform import Rotation as R
        r = R.from_quat(self.rotation)
        rostate_matrix = r.as_matrix()
        vector = np.array((cx,cy,0))
        vector = rostate_matrix.dot(vector)
        vector = vector+self.translation

        return vector[0] , vector[1] ,  0.5

    def twodbox(self, depth, bbox, identities=None, offset=(0, 0)):

        dt = (self.thistime - self.lasttime).to_sec()
        print('dt=', dt)
        idcentvel_tmp = []
        for i, id in enumerate(identities):
            cx, cy, r = self.drawsquare(bbox[i], depth)
            # 妙处：初始化时是空列表，同时完成了第一次时间的初始化
            flag = 0
            for idcv in self.idcenvel:
                if id == idcv[0]:

                    cx, cy = kafilter(idcv[1], idcv[2], idcv[3], idcv[4], cx, cy, dt)

                    vx = (cx - idcv[1]) / dt
                    vy = (cy - idcv[2]) / dt
                    if abs(vx) < 0.01: vx = 0.0
                    if abs(vy) < 0.01: vy = 0.0
                    idcentvel_tmp.append((id, cx, cy, vx, vy, 0.5))
                    flag = 1
                    break
            if not flag:
                vx = vy = 0.0
                idcentvel_tmp.append((id, cx, cy, vx, vy, 0.5))

        ## update idcenvel
        self.idcenvel = idcentvel_tmp
        return self.idcenvel


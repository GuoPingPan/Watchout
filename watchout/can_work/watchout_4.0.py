import sys
sys.path.insert(0, './yolov5')

from yolov5.utils.datasets import LoadImages, LoadStreams,LoadWebcam,LoadRealsense
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

'''
    直接再track.py改watchout，在v3.0的基础上加上了

        - kafilter 卡尔曼滤波

    并且和抗的算法消除联调bug

    version:4.0

'''

import numpy as np
from visualization_msgs.msg import Marker,MarkerArray
import rospy
from numba import jit
# from tf import TransformListener
from APF_BASE_utils import BASE_TOOLS_for_car as To
from geometry_msgs.msg import Twist

from nav_msgs.msg import Odometry

palette = (2 ** 11 - 1, 2 ** 15 - 1, 2 ** 20 - 1)


def bbox_rel(*xyxy):
    """" Calculates the relative bounding box from absolute pixel values. """
    bbox_left = min([xyxy[0].item(), xyxy[2].item()])
    bbox_top = min([xyxy[1].item(), xyxy[3].item()])
    bbox_w = abs(xyxy[0].item() - xyxy[2].item())
    bbox_h = abs(xyxy[1].item() - xyxy[3].item())
    x_c = (bbox_left + bbox_w / 2)
    y_c = (bbox_top + bbox_h / 2)
    w = bbox_w
    h = bbox_h
    return x_c, y_c, w, h



def compute_color_for_labels(label):
    """
    Simple function that adds fixed color depending on the class
    """
    color = [int((p * (label ** 2 - label + 1)) % 255) for p in palette]
    return tuple(color)


def showdepth(boxes,depth):
    for box in boxes:
        x1,y1,x2,y2 = [int(i) for i in box]
        for u in range(x1,x2):
            for v in range(y1,y2):
                print(depth[v,u]*0.001)


#注意 offset 光心偏移
def draw_boxes(img, bbox, identities=None, offset=(0, 0)):
    for i, box in enumerate(bbox):
        x1, y1, x2, y2 = [int(i) for i in box]
        x1 += offset[0]
        x2 += offset[0]
        y1 += offset[1]
        y2 += offset[1]
        #import math
        #x1 = x1 + math.ceil((x2-x1)*0.382)
        #x2 = x1 + math.ceil((x2-x1)*0.618)
        #y1 = y1 + math.ceil((y2-y1)*0.382)
        #y2 = y1 + math.ceil((y2-y1)*0.618)
        # print(img.shape)
        # print(x1,y1,x2,y2)
        # box text and bar
        id = int(identities[i]) if identities is not None else 0
        color = compute_color_for_labels(id)
        label = '{}{:d}'.format("", id)
        t_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_PLAIN, 2, 2)[0]
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 3)
        cv2.rectangle(
             img, (x1, y1), (x1 + t_size[0] + 3, y1 + t_size[1] + 4), color, -1)
        cv2.putText(img, label, (x1, y1 +
                                  t_size[1] + 4), cv2.FONT_HERSHEY_PLAIN, 2, [255, 255, 255], 2)
    return img


def kafilter(cxp,cyp,vx,vy,cx,cy,dt):
    F = np.identity(4)
    for i in range(2):
        F[i,i+2] = dt

    pw = 1./20
    vw = 1./160

    mean = [cxp,cyp,vx,vy]

    # std的存在只是为了初始化协方差
    std = [
        2*pw*cxp,
        2*pw*cyp,
        10*vw*vx,
        10*vw*vy     
    ]

    covariance = np.diag(np.square(std))

    #真正的std的计算
    std_true=[
        pw*cxp,
        pw*cyp,
        vw*vx,
        vw*vy
    ]

    #运动预测方程的误差
    Q = np.diag(np.square(std_true))
    # x' = Fx
    mean = np.dot(F,mean) 
    # P' = FPF'+Q
    covariance = np.linalg.multi_dot((F,covariance,F.transpose())+Q)

    pre = mean[:2]
    pvar = covariance[:2,:2]

    z = [cx,cy]
    zstd = [
        pw*cx,
        pw*cy
    ]
    R = np.diag(np.square(zstd))

    pvar = pvar

    # print('mean = ',mean)
    # print('covariance = ',covariance)
    # print('Q = ',Q)
    # print('mean after = ',mean)
    # print('covariance after = ',covariance)
    # print('R',R)

    import scipy.linalg
    # cho,lower = scipy.linalg.cho_factor(
        # pvar,lower=True,check_finite=False
    # )
    # K = scipy.linalg.cho_solve(
    #     (cho,lower),np.dot(covariance[:2,:2],np.identity(2).transpose()).transpose(),
    #     check_finite=False).transpose()
    

    # K = PH'(HPH'+R)'
    # K = np.dot(pvar,np.linalg.inv(pvar+R))
    # (HPH'+R)`k` = P`
    try:
        cho,lower=scipy.linalg.cho_factor(
            (pvar+R).transpose(),lower=True,check_finite=False
        )
        K = scipy.linalg.cho_solve(
            (cho,lower),pvar.transpose(),check_finite=False
        ).transpose()
        #x' = x +K(z-Hx)
        pose = pre+np.dot(K,z-pre)
        #P' = P - K'HP
        # new_covariance = covariance - np.linalg.multi_dot((
        # K, pvar, K.transpose()))

        return pose[0],pose[1]
    except:
        return cx,cy

class Watchout:
    def __init__(self,opt):

        #   深度初始化
        self.lasttime = rospy.Time.now()
        self.thistime = rospy.Time.now()
        self.scale = 0.001          
        self.idcenvel = []          #id cx,cy,vx,vy
        self.depth_thres = 10.0     #深度阀值
        # 内参
        fx = 609.2713012695312
        cx = 316.67022705078125
        fy = 608.010498046875
        cy = 244.8178253173828 
        self.K = np.array([[1.0/fx,0,-cx/fx],
                           [0,1.0/fy,-cy/fy],
                           [0.0 , 0.0, 1.0]])
        # self.listener = TransformListener()
        self.translation = []
        self.rotation = []
        self.publisher = rospy.Publisher('/cmd_vel',Twist,queue_size=1)

        # Initialize
        source,weights,imgsz = opt.source ,opt.weights, opt.img_size
        self.view_img = opt.view_img
        self.augment = opt.augment
        self.conf_thres = opt.conf_thres
        self.iou_thres = opt.iou_thres
        self.classes = opt.classes
        self.agnostic_nms = opt.agnostic_nms
        
        # initialize deepsort
        cfg = get_config()
        cfg.merge_from_file(opt.config_deepsort)
        self.deepsort = DeepSort(cfg.DEEPSORT.REID_CKPT,
                            max_dist=cfg.DEEPSORT.MAX_DIST, 
                            min_confidence=cfg.DEEPSORT.MIN_CONFIDENCE,
                            nms_max_overlap=cfg.DEEPSORT.NMS_MAX_OVERLAP,
                            max_iou_distance=cfg.DEEPSORT.MAX_IOU_DISTANCE,
                            max_age=cfg.DEEPSORT.MAX_AGE,
                            n_init=cfg.DEEPSORT.N_INIT,
                            nn_budget=cfg.DEEPSORT.NN_BUDGET,
                            use_cuda=True)
        self.device = select_device(opt.device)
        self.half = self.device.type != 'cpu'  # half precision only supported on CUDA

        # Load model
        self.model = torch.load(weights, map_location=self.device)['model'].float()  # load to FP32
        self.model.to(self.device).eval()
        if self.half:
            self.model.half()  # to FP16

        # Load dataset
        if source=='0':
            self.dataset = LoadWebcam(source,imgsz)
            self.view_img = True
            cudnn.benchmark = True  # set True to speed up constant image size inference
        else:
            self.dataset = LoadRealsense('0',img_size=imgsz)
            self.view_img = True
            cudnn.benchmark = True  # set True to speed up constant image size inference
        
        # Get names and colors
        names = self.model.module.names if hasattr(self.model, 'module') else self.model.names

        # Run inference
        img = torch.zeros((1, 3, imgsz, imgsz), device=self.device)  # init img
        # run once
        _ = self.model(img.half() if self.half else img) if self.device.type != 'cpu' else None

    def watch(self, save_img=False):

        vis, pos_end = To.init()
        vx, vy, w = (0 ,0, 0)
        for frame_idx, (path, img, im0, depth) in enumerate(self.dataset):
            t0 = time.time()
            self.thistime = rospy.Time.now()
            img = torch.from_numpy(img).to(self.device)
            img = img.half() if self.half else img.float()  # uint8 to fp16/32
            img /= 255.0  # 0 - 255 to 0.0 - 1.0
            if img.ndimension() == 3:
                img = img.unsqueeze(0)

            # Inference
            t1 = time_synchronized()
            pred = self.model(img, augment=self.augment)[0]

            # Apply NMS
            # [xyxy, conf, cls] n*6
            pred = non_max_suppression( pred,
                                        self.conf_thres,
                                        self.iou_thres, 
                                        classes=self.classes,
                                        agnostic=self.agnostic_nms)
            
            t2 = time_synchronized()

            # Print time (inference + NMS)
            print('Done. (%.3fs)' % ( t2 - t1))

            # Process detections
            for i, det in enumerate(pred):  # detections per image

                im0 = im0.copy()

                if det is not None and len(det):
                    
                    # Rescale boxes from img_size to im0 size 即处理 xyxy
                    det[:, :4] = scale_coords(
                        img.shape[2:], det[:, :4], im0.shape).round()

                    bbox_xywh = []
                    confs = []

                    # Adapt detections to deep sort input format  
                    # deepsort的输入类型为 [cx_2d,cy_2d,w,h,confidence]
                    for *xyxy, conf, cls in det:
                        x_c, y_c, bbox_w, bbox_h = bbox_rel(*xyxy)
                        obj = [x_c, y_c, bbox_w, bbox_h]
                        bbox_xywh.append(obj)
                        confs.append([conf.item()])

                    xywhs = torch.Tensor(bbox_xywh)
                    confss = torch.Tensor(confs)

                    # Pass detections to deepsort
                    # outputs : [x1,y1,x2,y2,id]
                    outputs = self.deepsort.update(xywhs, confss, im0)
                    pos_now = (0, 0, 0)
                    # draw boxes for visualization
                    if len(outputs) > 0:
                        bbox_xyxy = outputs[:, :4]
                        identities = outputs[:, -1]
                        draw_boxes(im0, bbox_xyxy, identities)
                        # self.twodbox(depth,bbox_xyxy,identities)

                        blocklist = self.twodbox(depth,bbox_xyxy,identities)
                        vx, vy, w= To.Vis_and_deside(vis=vis, pos_now=pos_now, vx=vx, vy=vy,pos_end=pos_end, blocklist=blocklist)
                    else:
                        blocklist=[]
                        vx, vy, w = To.Vis_and_deside(vis=vis, pos_now=pos_now, vx=vx, vy=vy, pos_end=pos_end, blocklist=blocklist)
                        vel = Twist()
                        vel.linear.x = vx
                        vel.linear.y = vy
                        self.publisher.publish(vel)
                else:
                    self.deepsort.increment_ages()

                # Stream results
                if self.view_img:
                    cv2.imshow('watchout', im0)
                    if cv2.waitKey(1) == ord('q') or rospy.is_shutdown():  # q to quit
                        print('Done. (%.3fs)' % (time.time() - t0))
                        raise StopIteration
                
                self.lasttime = self.thistime

    def goodenbox(self,bbox_xyxy):
        x1,y1,x2,y2 = [int(i) for i in bbox_xyxy]
        w = x2 - x1
        h = y2 - y1
        #黄金比例切割背景
        import math
        u1 = math.ceil(x1+0.382*w)
        u2 = math.ceil(x1+0.618*w)
        v1 = math.ceil(y1+0.382*h)
        v2 = math.ceil(y1+0.618*h)
        return [u1,v1,u2,v2]


    def drawsquare(self,xyxy,depth):
        # 计算公式 x = (u*depth - cx*depth)/fx   y = (v*depth - cy*depth)/fy
        # 先将 像素坐标 uv1 * depth 
        u1,v1,u2,v2 = self.goodenbox(xyxy)

        uvd = []
        for u in range(u1,u2):
            for v in range(v1,v2):
                depth_ = float(depth[v,u])*self.scale
                if depth_ > self.depth_thres: continue
                else: uvd.append([u*depth_,v*depth_,depth_])
        

        yzx = self.K.dot(np.array(uvd).T).T

        # 用均值代替质心
        cx = np.median(yzx[:,2])
        cy = np.median(yzx[:,0])

        if(cx==0): cx = np.mean(yzx[:,2])
        if(cy==0): cy = np.mean(yzx[:,0])

        # 找到八个顶点
        # xmax = yzx[:,2].max()
        # xmin = yzx[:,2].min()
        # ymax = yzx[:,0].max()
        # ymin = yzx[:,0].min()
        # zmax = yzx[:,1].max()
        # zmin = yzx[:,1].min()

        odom = rospy.wait_for_message("/aft_mapped_to_init",Odometry,timeout=rospy.Duration(3.0))
        self.translation=[odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z]
        self.rotation = [odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z,odom.pose.pose.orientation.w]
        print(self.translation)    
        # import tf
        # try:
        #     self.translation,self.rotation = self.listener.lookupTransform(target_frame='/map',source_frame='/camera_frame',time=rospy.Time(0))
        #     # [x,y,z] [x,y,z,w]
        #     print('transform yes')
        # except(tf.LookupException,
        #     tf.ConnectivityException,
        #     tf.ExtrapolationException):
        #     exit

        # from scipy.spatial.transform import Rotation as R
        # r = R.from_quat(self.rotation)
        # rostate_matrix = r.as_matrix()
        # vector = np.array((cx,cy,0))
        # # print('firstvector=',vector)
        # vector = vector+self.translation
        # vector = rostate_matrix.dot(vector)
        # print('trans=',trans)
        # print('rot=',rostate_matrix)
        # print('second=',vector)

        # return vector[0] , vector[1] ,  (xmax-xmin)/2
        return cx ,cy,0
   

    def twodbox(self,depth,bbox,identities=None,offset=(0,0)):
        
        dt = (self.thistime - self.lasttime).to_sec()
        print('dt=',dt)
        idcentvel_tmp = []
        for i,id in enumerate(identities):
            cx,cy,r = self.drawsquare(bbox[i],depth)
            # 妙处：初始化时是空列表，同时完成了第一次时间的初始化
            flag = 0
            for idcv in self.idcenvel:
                if id == idcv[0]:
                    
                    cx,cy = kafilter(idcv[1],idcv[2],idcv[3],idcv[4],cx,cy,dt)

                    vx = (cx - idcv[1])/dt
                    vy = (cy - idcv[2])/dt
                    if abs(vx) < 0.01: vx=0.0
                    if abs(vy) < 0.01: vy=0.0
                    idcentvel_tmp.append((id,cx,cy,vx,vy,0.5))
                    flag = 1
                    break
            if not flag:
                vx = vy = 0.0
                idcentvel_tmp.append((id,cx,cy,vx,vy,0.5))
        
        ## update idcenvel
        self.idcenvel = idcentvel_tmp
        print(idcentvel_tmp)

        return self.idcenvel










if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', type=str,
                        default='yolov5/weights/yolov5s.pt', help='model.pt path')
    # file/folder, 0 for webcam
    parser.add_argument('--source', type=str,
                        default='inference/images', help='source')
    parser.add_argument('--output', type=str, default='inference/output',
                        help='output folder')  # output folder
    parser.add_argument('--img-size', type=int, default=640,
                        help='inference size (pixels)')
    parser.add_argument('--conf-thres', type=float,
                        default=0.4, help='object confidence threshold')
    parser.add_argument('--iou-thres', type=float,
                        default=0.5, help='IOU threshold for NMS')
    parser.add_argument('--fourcc', type=str, default='mp4v',
                        help='output video codec (verify ffmpeg support)')
    parser.add_argument('--device', default='',
                        help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
    parser.add_argument('--view-img', action='store_true',
                        help='display results')
    parser.add_argument('--save-txt', action='store_true',
                        help='save results to *.txt')
    # class 0 is person
    parser.add_argument('--classes', nargs='+', type=int,
                        default=[0], help='filter by class')
    parser.add_argument('--agnostic-nms', action='store_true',
                        help='class-agnostic NMS')
    parser.add_argument('--augment', action='store_true',
                        help='augmented inference')
    parser.add_argument("--config_deepsort", type=str,
                        default="deep_sort_pytorch/configs/deep_sort.yaml")
    args = parser.parse_args()
    args.img_size = check_img_size(args.img_size)
    print(args)

    rospy.init_node('watchout')

    with torch.no_grad():
        watchout = Watchout(args)
        watchout.watch()


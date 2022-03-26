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

import numpy as np
from visualization_msgs.msg import Marker,MarkerArray
import rospy
from geometry_msgs.msg import Point
from numba import jit

'''
    该文件修改了原来的track.py，并加入了
        - 深度计算
        - maker显示
    
    version:1.0

'''

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
        # # box text and bar
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

class Watchout:
    def __init__(self):
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


        self.lines = [[0,1],[1,3],[3,2],[2,0],
                      [0,4],[2,6],[1,5],[3,7],
                      [4,5],[5,7],[7,6],[6,4]]

        self.pub = rospy.Publisher('Personbox',MarkerArray,queue_size=1)

        self.rate = rospy.Rate(10)

    def watch(self,opt, save_img=False):
        out, source,weights, view_img, save_txt, imgsz = \
            opt.output, opt.source ,opt.weights, opt.view_img, opt.save_txt, opt.img_size

        # initialize deepsort
        cfg = get_config()
        cfg.merge_from_file(opt.config_deepsort)
        deepsort = DeepSort(cfg.DEEPSORT.REID_CKPT,
                            max_dist=cfg.DEEPSORT.MAX_DIST, min_confidence=cfg.DEEPSORT.MIN_CONFIDENCE,
                            nms_max_overlap=cfg.DEEPSORT.NMS_MAX_OVERLAP, max_iou_distance=cfg.DEEPSORT.MAX_IOU_DISTANCE,
                            max_age=cfg.DEEPSORT.MAX_AGE, n_init=cfg.DEEPSORT.N_INIT, nn_budget=cfg.DEEPSORT.NN_BUDGET,
                            use_cuda=True)

        # Initialize
        device = select_device(opt.device)
    
        half = device.type != 'cpu'  # half precision only supported on CUDA

        # Load model
        model = torch.load(weights, map_location=device)[
            'model'].float()  # load to FP32
        model.to(device).eval()
        if half:
            model.half()  # to FP16

        # Set Dataloader
        vid_path, vid_writer = None, None
        view_img = True
        cudnn.benchmark = True  # set True to speed up constant image size inference
        dataset = LoadRealsense('0',img_size=imgsz)
        
        # Get names and colors
        names = model.module.names if hasattr(model, 'module') else model.names

        # Run inference
        t0 = time.time()
        img = torch.zeros((1, 3, imgsz, imgsz), device=device)  # init img
        # run once
        _ = model(img.half() if half else img) if device.type != 'cpu' else None


        for frame_idx, (path, img, im0, depth) in enumerate(dataset):
            t4 = time.time()
            self.thistime = rospy.Time.now()
            img = torch.from_numpy(img).to(device)
            img = img.half() if half else img.float()  # uint8 to fp16/32
            img /= 255.0  # 0 - 255 to 0.0 - 1.0
            if img.ndimension() == 3:
                img = img.unsqueeze(0)

            # Inference
            t1 = time_synchronized()
            pred = model(img, augment=opt.augment)[0]

            # Apply NMS
            # [xyxy, conf, cls] n*6
            pred = non_max_suppression(
                pred, opt.conf_thres, opt.iou_thres, classes=opt.classes, agnostic=opt.agnostic_nms)
            
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
                    # deepsort的输入类型为 centerx,centery,w,h,confidence,
                    for *xyxy, conf, cls in det:
                        x_c, y_c, bbox_w, bbox_h = bbox_rel(*xyxy)
                        obj = [x_c, y_c, bbox_w, bbox_h]
                        bbox_xywh.append(obj)
                        confs.append([conf.item()])

                    xywhs = torch.Tensor(bbox_xywh)
                    confss = torch.Tensor(confs)

                    # Pass detections to deepsort
                    # outputs : x1 y1 x2 y2 id
                    outputs = deepsort.update(xywhs, confss, im0)

                    # draw boxes for visualization
                    if len(outputs) > 0:
                        bbox_xyxy = outputs[:, :4]
                        identities = outputs[:, -1]
                        draw_boxes(im0, bbox_xyxy, identities)
                        t3 = rospy.Time.now()
                        #self.publish3dbox(depth,bbox_xyxy,identities)
                        # if not self.init:
                        #     import threading
                        #     thread = threading.Thread(target=self.publish3dbox,args=(depth,bbox_xyxy,identities))    
                        #     thread.start()
                        #     self.init = 1
                        #     print('开启成功')
                    
                        print(f'Creating markderarrary use {(rospy.Time.now()-t3).to_sec()} s ')
                else:
                    deepsort.increment_ages()

                # Stream results
                if view_img:
                    cv2.imshow('watchout', im0)
                    if cv2.waitKey(1) == ord('q') or rospy.is_shutdown():  # q to quit
                        # thread.join()
                        print('Done. (%.3fs)' % (time.time() - t0))
                        raise StopIteration
                
                self.lasttime = self.thistime
            t5 = time.time()
            print('t5-t4',t5-t4)
    # @jit
    def create_box(self,depth_img,box,offset=(0,0)):

        # 计算公式 x = (u*depth - cx*depth)/fx   y = (v*depth - cy*depth)/fy
        # 先将 像素坐标 uv1 * depth 
        x1,y1,x2,y2 = [int(i) for i in box]
        w = x2 - x1
        h = y2 - y1
        #黄金比例切割背景
        import math
        u1 = math.ceil(x1+0.382*w)
        u2 = math.ceil(x1+0.618*w)
        v1 = math.ceil(y1+0.382*h)
        v2 = math.ceil(y1+0.618*h)
        
        uv1 = []
        for u in range(u1,u2):
            for v in range(v1,v2):
                depth = float(depth_img[v,u])*self.scale
                if  depth > self.depth_thres:
                    continue
                else:
                    uv1.append([u*depth,v*depth,depth])
        
        if(len(uv1)<1):
            print("create_error")
            return 0,0,None

        # 3*n
        uvd = np.array(uv1).T  

        # 将 uvd * 相机内参矩阵 K 转化为相机坐标的 xyz 但 相机坐标的 xyz 对应着三维空间中的 yzx
        # n*3
        yzx = self.K.dot(uvd).T
        # 用均值代替质心
        cx = yzx[:,2].mean()
        cy = yzx[:,0].mean()

        # 找到八个顶点
        xmax = yzx[:,2].max()
        xmin = yzx[:,2].min()
        ymax = yzx[:,0].max()
        ymin = yzx[:,0].min()
        zmax = yzx[:,1].max()
        zmin = yzx[:,1].min()

        points = [Point(xmin,ymin,zmin),Point(xmax,ymin,zmin),
                    Point(xmin,ymax,zmin),Point(xmax,ymax,zmin),
                    Point(xmin,ymin,zmax),Point(xmax,ymin,zmax),
                    Point(xmin,ymax,zmax),Point(xmax,ymax,zmax)]

        # 创建 bbox 
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = rospy.Time.now()
        marker.action = Marker.ADD
        marker.type = Marker.LINE_LIST

        # marker.lifetime = rospy.Duration(0)

        marker.color.r = 1
        marker.color.g = 0
        marker.color.b = 0

        marker.color.a = 1
        marker.scale.x = 0.2
        marker.points = []

        for line in self.lines:
            marker.points.append(points[line[0]])
            marker.points.append(points[line[1]])

        return cx , cy , marker
    # @jit
    def publish3dbox(self,depth_img,bbox,identities=None,offset=(0,0)):
        
        markerarray = MarkerArray()
        dt = (self.thistime - self.lasttime).to_sec()
        idcentvel_tmp = []
        # 生成markerarray 并 进行匹配计算 idcentvel
        for i,id_ in enumerate(identities):
            marker = Marker()
            cx,cy,marker = self.create_box(depth_img,bbox[i],offset)
            marker.id = id_
            markerarray.markers.append(marker)
            flag = 0
            # 妙处：初始化时是空列表，同时完成了第一次时间的初始化
            for idcv in self.idcenvel:
                if id_ == idcv[0]:

                    vx = (cx - idcv[1])/dt
                    vy = (cy - idcv[2])/dt

                    idcentvel_tmp.append([id_,cx,cy,vx,vy])
                    flag = 1
                    break

            if not flag:
                vx=vy=0.0
                idcentvel_tmp.append([id_,cx,cy,vx,vy])
            
        self.idcenvel = idcentvel_tmp
        print('idcenvel',self.idcenvel)
        self.pub.publish(markerarray)





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
    watchout = Watchout()

    with torch.no_grad():
        watchout.watch(args)

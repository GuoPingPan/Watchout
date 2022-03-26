#!~/APPLICATION/anaconda/envs/deepsort/bin python
import rospy
from message_filters import Subscriber,TimeSynchronizer,ApproximateTimeSynchronizer
from sensor_msgs.msg import Image

import sys
sys.path.insert(0,'./yolov5')

from yolov5.utils.datasets import LoadImages,LoadStreams
from yolov5.utils.general import check_img_size,non_max_suppression,scale_coords
from yolov5.utils.torch_utils import select_device,time_synchronized
from deep_sort_pytorch.utils.parser import get_config
from deep_sort_pytorch.deep_sort import DeepSort
import argparse
import os
import platform
import shutil
import time
from pathlib import Path
import cv2
import cv_bridge
import torch
import torch.backends.cuda as cudnn
import numpy as np 

# palette = (2 ** 11 - 1, 2 ** 15 - 1, 2 ** 20 - 1)



# fx = 609.2713012695312
# cx = 316.67022705078125
# fy = 608.010498046875
# cy = 244.8178253173828


'''
K = np.array([[fx,0,fy],
              [0,fy,cy],
              [0,0,1]])
K_inv = np.array([[1.0/fx,0,-cx/fx]
                  [0,1.0/fy,-cy/fy],
                  [0,0,1]])


def uv_to_point(uv,depth):
    global K_inv
    uvd = np.insert(uv,2,1,axis=0)*depth
    xyz = K_inv.dot(uvd)
    return xyz 


boxes_keep = []



def boxes_generator(ltwh_id,depth_img):
    boxes_now = []
    for l,t,w,h,id in ltwh_id:
        for i,id_old in enumerate(box_keep[:,-1]) :
            if id == id_old:
                points = []
                for u in range(l,l+w):
                    for v in range(t,t+h):
                        depth = depth_img.at(u,v)
                        if depth > depth_max or depth < depthmin:
                            continue
                        else:
                            pixel_tmp = np.array([u,v,1])
                            point = depth*K_inv.dot(pixel)
                            points.append(point)
                xmax = points[0].sort().max()
                xmin = points[0].sort().min()
                ymax = points[1].sort().max()
                ymin = points[1].sort().min()
                zmax = points[2].sort().max()
                zmin = points[2].sort().min()

                centroidx = points[0].meidan()
                centoridy = points[1].median()

                dt = (rospy.Time.now()-self.time).to_sec()
                velx = (box_keep[i,8]-centroidx)/dt
                velx = (box_keep[i,9]-centroidx)/dt
                box = [Point(xmin,ymin,zmin),Point(xmax,ymin,zmin),
                        Point(xmin,ymax,zmin),Point(xmax,ymax,zmin),
                        Point(xmin,ymin,zmax),Point(xmax,ymin,zmax),
                        Point(xmin,ymax,zmax),Point(xmax,ymax,zmax),]
            else:
                continue
        boxes_now.append(box)
        
        generate_markerarray(boxes_now)

        boxes_keep.clear()
        boxes_keep = boxes_now
        
    return
def generate_markerarray():
    pass
'''


class Watching:

    def __init__(self):
        self.weights = rospy.get_param('~weights',default='yolov5/weights/yolov5s.pt')
        self.output  = rospy.get_param('~output',default='inference/output')
        self.img_size = rospy.get_param('~img_size',default=640)
        self.conf_thres = rospy.get_param('~conf_thres',default=0.4)
        self.iou_thres = rospy.get_param('~iou_thres',default=0.5)
        self.fourcc = rospy.get_param('~fourcc',default='mp4v')
        self.view_img = rospy.get_param('~view_img',default=True)
        self.save_txt = rospy.get_param('~save_txt',default=False)
        self.classes = rospy.get_param('~classes',default=[0])
        self.agnostic_nms = rospy.get_param('~agnostic_nms',default=False)
        self.argument = rospy.get_param('~augment',default=False)
        self.img_size = check_img_size(self.img_size)
        # initialize deepsort
        self.config_deepsort = rospy.get_param('~config_deepsort',default='deep_sort_pytorch/configs/deep_sort.yaml')
        self.cfg = get_config()
        self.cfg.merge_from_file(self.config_deepsort)
        self.deepsort = DeepSort(self.cfg.DEEPSORT.REID_CKPT,
                            max_dist=self.cfg.DEEPSORT.MAX_DIST, min_confidence=self.cfg.DEEPSORT.MIN_CONFIDENCE,
                            nms_max_overlap=self.cfg.DEEPSORT.NMS_MAX_OVERLAP, max_iou_distance=self.cfg.DEEPSORT.MAX_IOU_DISTANCE,
                            max_age=self.cfg.DEEPSORT.MAX_AGE, n_init=self.cfg.DEEPSORT.N_INIT, nn_budget=self.cfg.DEEPSORT.NN_BUDGET,
                            use_cuda=True)

        # Load model
        device = rospy.get_param('~device',default='0')
        self.device = select_device(device)
        self.half = device.type != 'cpu'  # half precision only supported on CUDA
        self.model = torch.load(self.weights, map_location=self.device)['model'].float()  # load to FP32
        self.model.to(self.device).eval()
        self.time = rospy.Time.now()
        if self.half:
            self.model.half()  # to FP16
        cudnn.benchmark = True
        # Get names and colors
        names = self.model.module.names if hasattr(self.model, 'module') else self.model.names
        # Run inference
        img = torch.zeros((1, 3, self.img_size, self.img_size), device=self.device)  # init img
        _ = model(img.half() if self.half else img) if self.half else None

        self.color_topic = rospy.get_param('~color_topic',default='/camera/color/image_raw')
        self.depth_topic = rospy.get_param('~depth_topic',default='/camera/aligned_depth_to_color/image_raw')
        color_sub = Subscriber(self.color_topic,Image,queue_size=1)
        depth_sub = Subscriber(self.depth_topic,Image,queue_size=1)
        ts = ApproximateTimeSynchronizer([color_sub,depth_sub],10,0.1,allow_headerless=True)
        ts.registerCallback(self.detect)

    def detect(self,color_img,depth_img):
        
        # 整理图片格式
        color = cv_bridge.CvBridge.imgmsg_to_cv2(color_img)
        depth = cv_bridge.CvBridge.imgmsg_to_cv2(depth_img)
        # ret_val, img0 = self.cap.read() type(img0) = np.array
        img = letterbox(color,self.img_size,stride=self.stride)[0]
        # Convert
        img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, to 3x416x416
        img = np.ascontiguousarray(img)
        #                               640             30
        img = torch.from_numpy(img).to(self.device)
        img = img.half() if self.half else img.float()  # uint8 to fp16/32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0
        if img.ndimension() == 3:
            img = img.unsqueeze(0)
    
        # Inference
        pred = self.model(img, augment=self.augment)[0]

        # Apply NMS
        pred = non_max_suppression(
            pred,self.conf_thres,
            self.iou_thres,
            classes=self.classes,
            agnostic=self.agnostic_nms)
        for i,det in enumerate(pred):
            if det is not None and len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(
                    img.shape[2:], det[:, :4], color.shape).round()
                
                # Print results
                for c in det[:, -1].unique():
                    n = (det[:, -1] == c).sum()  # detections per class
                    s += '%g %ss, ' % (n, names[int(c)])  # add to string

                bbox_xywh = []
                confs = []
                
                # Adapt detections to deep sort input format
                for *xyxy, conf, cls in det:
                    x_c, y_c, bbox_w, bbox_h = self.bbox_rel(*xyxy)
                    obj = [x_c, y_c, bbox_w, bbox_h]
                    bbox_xywh.append(obj)
                    confs.append([conf.item()])

                xywhs = torch.Tensor(bbox_xywh)
                confss = torch.Tensor(confs)

                # Pass detections to deepsort
                outputs = self.deepsort.update(xywhs, confss, color)

                # draw boxes for visualization
                if len(outputs) > 0:
                    bbox_xyxy = outputs[:, :4]
                    identities = outputs[:, -1]
                    self.draw_boxes(color, bbox_xyxy, identities)

                # Write MOT compliant results to file
                if save_txt and len(outputs) != 0:
                    for j, output in enumerate(outputs):
                        bbox_left = output[0]
                        bbox_top = output[1]
                        bbox_w = output[2]
                        bbox_h = output[3]
                        identity = output[-1]
                        with open(txt_path, 'a') as f:
                            f.write(('%g ' * 10 + '\n') % (frame_idx, identity, bbox_left,
            else:
                self.deepsort.increment_ages()

            # Stream results
            if view_img:
                cv2.imshow(color)
                if cv2.waitKey(1) == ord('q'):  # q to quit
                    raise StopIteration

    def bbox_rel(self,*xyxy):
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

    def compute_color_for_labels(self,label):
        """
        Simple function that adds fixed color depending on the class
        """
        color = [int((p * (label ** 2 - label + 1)) % 255) for p in palette]
        return tuple(color)

    def draw_boxes(self,img, bbox, identities=None, offset=(0, 0)):
        for i, box in enumerate(bbox):
            x1, y1, x2, y2 = [int(i) for i in box]
            x1 += offset[0]
            x2 += offset[0]
            y1 += offset[1]
            y2 += offset[1]
            # box text and bar
            id = int(identities[i]) if identities is not None else 0
            color = self.compute_color_for_labels(id)
            label = '{}{:d}'.format("", id)
            t_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_PLAIN, 2, 2)[0]
            cv2.rectangle(img, (x1, y1), (x2, y2), color, 3)
            cv2.rectangle(
                img, (x1, y1), (x1 + t_size[0] + 3, y1 + t_size[1] + 4), color, -1)
            cv2.putText(img, label, (x1, y1 +
                                    t_size[1] + 4), cv2.FONT_HERSHEY_PLAIN, 2, [255, 255, 255], 2)
        return img


if __name__=='__main__':

    rospy.init("watching")

    watch_out = Watching()

    rospy.spin()

    return


import sys
sys.path.insert(0, './yolov5')

from yolov5.utils.datasets import LoadImages, LoadStreams
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
import pyrealsense2 as rs


pipeline = rs.pipeline()

config = rs.config()
config.enable_stream(rs.stream.color,640,480,rs.format.bgr8,30)
config.enable_stream(rs.stream.depth,640,480,rs.format.z16,30)

pipeline.start(config)

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


def draw_boxes(img, bbox, identities=None, offset=(0, 0)):
    for i, box in enumerate(bbox):
        x1, y1, x2, y2 = [int(i) for i in box]
        x1 += offset[0]
        x2 += offset[0]
        y1 += offset[1]
        y2 += offset[1]
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

def letterbox(img, new_shape=(640, 640), color=(114, 114, 114), auto=True, scaleFill=False, scaleup=True, stride=32):
    # Resize and pad image while meeting stride-multiple constraints
    shape = img.shape[:2]  # current shape [height, width]
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)

    # Scale ratio (new / old)
    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
    if not scaleup:  # only scale down, do not scale up (for better test mAP)
        r = min(r, 1.0)

    # Compute padding
    ratio = r, r  # width, height ratios
    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]  # wh padding
    if auto:  # minimum rectangle
        dw, dh = np.mod(dw, stride), np.mod(dh, stride)  # wh padding
    elif scaleFill:  # stretch
        dw, dh = 0.0, 0.0
        new_unpad = (new_shape[1], new_shape[0])
        ratio = new_shape[1] / shape[1], new_shape[0] / shape[0]  # width, height ratios

    dw /= 2  # divide padding into 2 sides
    dh /= 2

    if shape[::-1] != new_unpad:  # resize
        img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)  # add border
    return img, ratio, (dw, dh)

class LoadRealsense:  # for inference
    def __init__(self, pipe='0', img_size=640, stride=32):
        self.img_size = img_size
        self.stride = stride

        self.pipe = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color,640,480,rs.format.bgr8,30)
        self.pipe.start(config)


    def __iter__(self):
        self.count = -1
        return self

    def __next__(self):
        self.count += 1
        if cv2.waitKey(1) == ord('q'):  # q to quit
            pipe.stop()
            cv2.destroyAllWindows()
            raise StopIteration


        frame = self.pipe.wait_for_frames()
        color_frame = frame.get_color_frame()

        img0 = np.asanyarray(color_frame.get_data())

        

        # Padded resize
        img = letterbox(img0, self.img_size, stride=self.stride)[0]

        # Convert
        img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, to 3x416x416
        img = np.ascontiguousarray(img)

        return img_path, img, img0, None

    def __len__(self):
        return 0
        
def detect(opt, save_img=False):
    out, source, weights, view_img, save_txt, imgsz = \
        opt.output, opt.source, opt.weights, opt.view_img, opt.save_txt, opt.img_size
    webcam = source == '0' or source.startswith(
        'rtsp') or source.startswith('http') or source.endswith('.txt')

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
    if os.path.exists(out):
        shutil.rmtree(out)  # delete output folder
    os.makedirs(out)  # make new output folder
    half = device.type != 'cpu'  # half precision only supported on CUDA

    # Load model
    model = torch.load(weights, map_location=device)[
        'model'].float()  # load to FP32
    model.to(device).eval()
    if half:
        model.half()  # to FP16

    # # Set Dataloader
    # vid_path, vid_writer = None, None
    # if webcam:
    #     view_img = True
    #     cudnn.benchmark = True  # set True to speed up constant image size inference
    #     dataset = LoadStreams(source, img_size=imgsz)
    # else:
    #     view_img = True
    #     save_img = True
    #     dataset = LoadImages(source, img_size=imgsz)

    # Get names and colors
    names = model.module.names if hasattr(model, 'module') else model.names

    # Run inference
    t0 = time.time()
    img = torch.zeros((1, 3, imgsz, imgsz), device=device)  # init img
    # run once
    _ = model(img.half() if half else img) if device.type != 'cpu' else None

    save_path = str(Path(out))
    txt_path = str(Path(out)) + '/results.txt'

    global pipeline
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        print("type_of_rs:",type(color_frame))
        color_image = np.asanyarray(color_frame.get_data())
        color_image1 = letterbox(color_image)[0]
        print("shape of origin",color_image1.shape)
        img = torch.from_numpy(color_image1).to(device)
        img = img.half() if half else img.float()  # uint8 to fp16/32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0
        if img.ndimension() == 3:
            img = img.unsqueeze(0)
        print("type_of_img:",type(img))
        print("shape of img",img.shape)
        pred = model(img, augment=opt.augment)[0]
        pred = non_max_suppression(
            pred, opt.conf_thres, opt.iou_thres, classes=opt.classes, agnostic=opt.agnostic_nms)

        print("pred:{0} \n {1}".format(type(pred),pred))
        for det in pred:
            if det is not None and len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(
                    img.shape[2:], det[:, :4], color_image.shape).round()

                # Print results
                for c in det[:, -1].unique():
                    n = (det[:, -1] == c).sum()  # detections per class
                    s += '%g %ss, ' % (n, names[int(c)])  # add to string

                bbox_xywh = []
                confs = []
                # Adapt detections to deep sort input format
                for *xyxy, conf, cls in det:
                    x_c, y_c, bbox_w, bbox_h = bbox_rel(*xyxy)
                    obj = [x_c, y_c, bbox_w, bbox_h]
                    bbox_xywh.append(obj)
                    confs.append([conf.item()])

                xywhs = torch.Tensor(bbox_xywh)
                confss = torch.Tensor(confs)

                # Pass detections to deepsort
                outputs = deepsort.update(xywhs, confss, color_image)

                # draw boxes for visualization
                if len(outputs) > 0:
                    bbox_xyxy = outputs[:, :4]
                    identities = outputs[:, -1]
                    draw_boxes(color_image, bbox_xyxy, identities)

            else:
                deepsort.increment_ages()

            # Stream results
            if view_img:
                cv2.imshow(p, color_image)
                if cv2.waitKey(1) == ord('q'):  # q to quit
                    raise StopIteration

    # for frame_idx, (path, img, im0s, vid_cap) in enumerate(dataset):
    #     img = torch.from_numpy(img).to(device)
    #     img = img.half() if half else img.float()  # uint8 to fp16/32
    #     img /= 255.0  # 0 - 255 to 0.0 - 1.0
    #     if img.ndimension() == 3:
    #         img = img.unsqueeze(0)

    #     # Inference
    #     t1 = time_synchronized()
    #     pred = model(img, augment=opt.augment)[0]

    #     # Apply NMS
    #     pred = non_max_suppression(
    #         pred, opt.conf_thres, opt.iou_thres, classes=opt.classes, agnostic=opt.agnostic_nms)
    #     t2 = time_synchronized()

    #     # Process detections
    #     for i, det in enumerate(pred):  # detections per image
    #         if webcam:  # batch_size >= 1
    #             p, s, im0 = path[i], '%g: ' % i, im0s[i].copy()
    #         else:
    #             p, s, im0 = path, '', im0s

    #         s += '%gx%g ' % img.shape[2:]  # print string
    #         save_path = str(Path(out) / Path(p).name)

    #         if det is not None and len(det):
    #             # Rescale boxes from img_size to im0 size
    #             det[:, :4] = scale_coords(
    #                 img.shape[2:], det[:, :4], im0.shape).round()

    #             # Print results
    #             for c in det[:, -1].unique():
    #                 n = (det[:, -1] == c).sum()  # detections per class
    #                 s += '%g %ss, ' % (n, names[int(c)])  # add to string

    #             bbox_xywh = []
    #             confs = []

    #             # Adapt detections to deep sort input format
    #             for *xyxy, conf, cls in det:
    #                 x_c, y_c, bbox_w, bbox_h = bbox_rel(*xyxy)
    #                 obj = [x_c, y_c, bbox_w, bbox_h]
    #                 bbox_xywh.append(obj)
    #                 confs.append([conf.item()])

    #             xywhs = torch.Tensor(bbox_xywh)
    #             confss = torch.Tensor(confs)

    #             # Pass detections to deepsort
    #             outputs = deepsort.update(xywhs, confss, im0)

    #             # draw boxes for visualization
    #             if len(outputs) > 0:
    #                 bbox_xyxy = outputs[:, :4]
    #                 identities = outputs[:, -1]
    #                 draw_boxes(im0, bbox_xyxy, identities)

    #             # Write MOT compliant results to file
    #             if save_txt and len(outputs) != 0:
    #                 for j, output in enumerate(outputs):
    #                     bbox_left = output[0]
    #                     bbox_top = output[1]
    #                     bbox_w = output[2]
    #                     bbox_h = output[3]
    #                     identity = output[-1]
    #                     with open(txt_path, 'a') as f:
    #                         f.write(('%g ' * 10 + '\n') % (frame_idx, identity, bbox_left,
    #                                                        bbox_top, bbox_w, bbox_h, -1, -1, -1, -1))  # label format

    #         else:
    #             deepsort.increment_ages()

    #         # Print time (inference + NMS)
    #         print('%sDone. (%.3fs)' % (s, t2 - t1))

    #         # Stream results
    #         if view_img:
    #             cv2.imshow(p, im0)
    #             if cv2.waitKey(1) == ord('q'):  # q to quit
    #                 raise StopIteration

    #         # Save results (image with detections)
    #         if save_img:
    #             print('saving img!')
    #             if dataset.mode == 'images':
    #                 cv2.imwrite(save_path, im0)
    #             else:
    #                 print('saving video!')
    #                 if vid_path != save_path:  # new video
    #                     vid_path = save_path
    #                     if isinstance(vid_writer, cv2.VideoWriter):
    #                         vid_writer.release()  # release previous video writer

    #                     fps = vid_cap.get(cv2.CAP_PROP_FPS)
    #                     w = int(vid_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    #                     h = int(vid_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    #                     vid_writer = cv2.VideoWriter(
    #                         save_path, cv2.VideoWriter_fourcc(*opt.fourcc), fps, (w, h))
    #                 vid_writer.write(im0)

    # if save_txt or save_img:
    #     print('Results saved to %s' % os.getcwd() + os.sep + out)
    #     if platform == 'darwin':  # MacOS
    #         os.system('open ' + save_path)

    # print('Done. (%.3fs)' % (time.time() - t0))


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

    with torch.no_grad():
        detect(args)
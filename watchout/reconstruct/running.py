from Watchout import *
import rospy

from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import Odometry
import message_filters


rospy.init_node('watchout')
print("fds")


watchout = Watchout()


def callback(scan,odom):
    print("fsdfsdf")
    pointcloud = pc2.read_points_list(scan,field_names=('x','y'),skip_nans=True) 
    watchout.translation=[odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z]
    watchout.rotation = [odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z,odom.pose.pose.orientation.w]
    
    path,img,im0,depth = Watchout.dataset.load()
    watchout.thistime = rospy.Time.now()
    img = torch.from_numpy(img).to(watchout.device)
    img = img.half() if watchout.half else img.float()  # uint8 to fp16/32
    img /= 255.0  # 0 - 255 to 0.0 - 1.0
    if img.ndimension() == 3:
        img = img.unsqueeze(0)

    # Inference
    t1 = time_synchronized()
    pred = watchout.model(img, augment=watchout.augment)[0]

    # Apply NMS
    # [xyxy, conf, cls] n*6
    pred = non_max_suppression(pred,
                                watchout.conf_thres,
                                watchout.iou_thres,
                                classes=watchout.classes,
                                agnostic=watchout.agnostic_nms)

    t2 = time_synchronized()

    # Print time (inference + NMS)
    print('Done. (%.3fs)' % (t2 - t1))

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
            outputs = watchout.deepsort.update(xywhs, confss, im0)
            pos_now = (0, 0, 0, 0, 0)
            # draw boxes for visualization
            if len(outputs) > 0:
                bbox_xyxy = outputs[:, :4]
                identities = outputs[:, -1]
                draw_boxes(im0, bbox_xyxy, identities)

                blocklist = watchout.twodbox(depth, bbox_xyxy, identities)
                vx, vy, w, f = To_1.Vis_and_deside(vis=vis, pos_now=pos_now, pos_end=pos_end,
                                                    blocklist=blocklist)
                # vx, vy, w, f, id_ = To_2.Vis_and_deside(vis=vis, pos_now=pos_now, pos_end=pos_end, blocklist=blocklist,id_=id_)

            else:
                blocklist = []
                vx, vy, w, f = To_1.Vis_and_deside(vis=vis, pos_now=pos_now, pos_end=pos_end,
                                                    blocklist=blocklist)
                # vx, vy, w, f, id_ = To_2.Vis_and_deside(vis=vis, pos_now=pos_now, pos_end=pos_end, blocklist=blocklist,id_=id_)
                vel = Twist()
                vel.linear.x = vx
                vel.linear.y = vy
                watchout.publisher.publish(vel)
        else:
            watchout.deepsort.increment_ages()

        # Stream results
        if watchout.view_img:
            cv2.imshow('watchout', im0)
            if cv2.waitKey(1) == ord('q') or rospy.is_shutdown():  # q to quit
                print('Done. (%.3fs)' % (time.time() - t0))
                raise StopIteration

        watchout.lasttime = watchout.thistime    

lasersub = message_filters.Subscriber('/PointClouds',PointCloud2)
odomsub = message_filters.Subscriber('/aft_mapped_to_init',Odometry)

sub = message_filters.TimeSynchronizer([o,odomsub],10)
sub.registerCallback(callback)
print("fsdfsdf")

rospy.spin()
# if __name__=='__main__':



    # with torch.no_grad():
    #     vis, pos_end = To_1.init(mapsize=150, scale=15)
    #     # vis, pos_end, id_  = To_2.init(mapsize=150, scale=15)
    #     for frame_idx, (path, img, im0, depth) in enumerate(watchout.dataset):
    #         t0 = time.time()
    #         watchout.thistime = rospy.Time.now()
    #         img = torch.from_numpy(img).to(watchout.device)
    #         img = img.half() if watchout.half else img.float()  # uint8 to fp16/32
    #         img /= 255.0  # 0 - 255 to 0.0 - 1.0
    #         if img.ndimension() == 3:
    #             img = img.unsqueeze(0)

    #         # Inference
    #         t1 = time_synchronized()
    #         pred = watchout.model(img, augment=watchout.augment)[0]

    #         # Apply NMS
    #         # [xyxy, conf, cls] n*6
    #         pred = non_max_suppression(pred,
    #                                    watchout.conf_thres,
    #                                    watchout.iou_thres,
    #                                    classes=watchout.classes,
    #                                    agnostic=watchout.agnostic_nms)

    #         t2 = time_synchronized()

    #         # Print time (inference + NMS)
    #         print('Done. (%.3fs)' % (t2 - t1))

    #         # Process detections
    #         for i, det in enumerate(pred):  # detections per image

    #             im0 = im0.copy()

    #             if det is not None and len(det):

    #                 # Rescale boxes from img_size to im0 size 即处理 xyxy
    #                 det[:, :4] = scale_coords(
    #                     img.shape[2:], det[:, :4], im0.shape).round()

    #                 bbox_xywh = []
    #                 confs = []

    #                 # Adapt detections to deep sort input format
    #                 # deepsort的输入类型为 [cx_2d,cy_2d,w,h,confidence]
    #                 for *xyxy, conf, cls in det:
    #                     x_c, y_c, bbox_w, bbox_h = bbox_rel(*xyxy)
    #                     obj = [x_c, y_c, bbox_w, bbox_h]
    #                     bbox_xywh.append(obj)
    #                     confs.append([conf.item()])

    #                 xywhs = torch.Tensor(bbox_xywh)
    #                 confss = torch.Tensor(confs)

    #                 # Pass detections to deepsort
    #                 # outputs : [x1,y1,x2,y2,id]
    #                 outputs = watchout.deepsort.update(xywhs, confss, im0)
    #                 pos_now = (0, 0, 0, 0, 0)
    #                 # draw boxes for visualization
    #                 if len(outputs) > 0:
    #                     bbox_xyxy = outputs[:, :4]
    #                     identities = outputs[:, -1]
    #                     draw_boxes(im0, bbox_xyxy, identities)

    #                     blocklist = watchout.twodbox(depth, bbox_xyxy, identities)
    #                     vx, vy, w, f = To_1.Vis_and_deside(vis=vis, pos_now=pos_now, pos_end=pos_end,
    #                                                        blocklist=blocklist)
    #                     # vx, vy, w, f, id_ = To_2.Vis_and_deside(vis=vis, pos_now=pos_now, pos_end=pos_end, blocklist=blocklist,id_=id_)

    #                 else:
    #                     blocklist = []
    #                     vx, vy, w, f = To_1.Vis_and_deside(vis=vis, pos_now=pos_now, pos_end=pos_end,
    #                                                        blocklist=blocklist)
    #                     # vx, vy, w, f, id_ = To_2.Vis_and_deside(vis=vis, pos_now=pos_now, pos_end=pos_end, blocklist=blocklist,id_=id_)
    #                     vel = Twist()
    #                     vel.linear.x = vx
    #                     vel.linear.y = vy
    #                     watchout.publisher.publish(vel)
    #             else:
    #                 watchout.deepsort.increment_ages()

    #             # Stream results
    #             if watchout.view_img:
    #                 cv2.imshow('watchout', im0)
    #                 if cv2.waitKey(1) == ord('q') or rospy.is_shutdown():  # q to quit
    #                     print('Done. (%.3fs)' % (time.time() - t0))
    #                     raise StopIteration

    #             watchout.lasttime = watchout.thistime



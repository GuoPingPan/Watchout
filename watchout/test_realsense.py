import cv2
import pyrealsense2 as rs
import numpy as np
from numba import jit,cuda
import numba
import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import MarkerArray,Marker
import time
import rospy
from six.moves import xrange
lines = [[0,1],[1,3],[3,2],[2,0],
        [0,4],[2,6],[1,5],[3,7],
        [4,5],[5,7],[7,6],[6,4]]

idcenvel = []
fx = 609.2713012695312
cx = 316.67022705078125
fy = 608.010498046875
cy = 244.8178253173828 
K = np.array([[1.0/fx,0,-cx/fx],
                [0,1.0/fy,-cy/fy],
                [0.0 , 0.0, 1.0]])

def test9():
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color,640,480,rs.format.bgr8,30)
    cfg.enable_stream(rs.stream.depth,640,480,rs.format.z16,30)
    pipe.start(cfg)

    align = rs.align(rs.stream.color)

    for i in range(5):
        pipe.wait_for_frames()

    frameset = pipe.wait_for_frames()
    frame = align.process(frameset)


    depth = np.asanyarray((frame.get_depth_frame()).get_data())
    a = []
    for u in range(10,100):
        for v in range(10,100):
            # print('\n depth  ',depth[v,u])
            # print('\n depthscale ' ,depth[v,u]*0.001)
            uvd = np.array([u*depth[v,u]*0.001,v*depth[v,u]*0.001,depth[v,u]*0.001]).T
            # print('\n uvd   ',uvd)
            xyz = K.dot(uvd).T
            xyz1 = [xyz[2],xyz[0],xyz[1]]
            print('\n xyz',xyz1)
            a.append(xyz1)

    f = np.array(a)
    print(f[:,0])
    print('xmax xmin',np.max(f[:,0]),np.min(f[:,0]))
    print(f[:,0].max(),f[:,0].min())
    print('ymax zmin',f[1].max(),f[1].min())
    print('zmax zmin',f[2].max(),f[2].min())
    color = np.asanyarray(frameset.get_color_frame().get_data())
    print(color.shape)
    # color1 = color.copy()[:,:,::-1]
    # print(color.shape)
    cv2.rectangle(color, (10, 10), (100, 100),  3)
    cv2.imshow('color',color)
    cv2.waitKey(0)

# t1 = time.time()
test9()
# print('\nuse_time',time.time()-t1)



def test7():
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.depth,640,480,rs.format.z16,30)
    cfg.enable_stream(rs.stream.color,640,480,rs.format.rgb8,30)
    pipe.start(cfg)
    aligned_to_color = rs.align(rs.stream.color)

    
    pc = rs.pointcloud()
    for i in range(5):
        pipe.wait_for_frames()
    frame = pipe.wait_for_frames()
    frame = aligned_to_color.process(frame)

    depth_frame = frame.get_depth_frame()
    color_frame = frame.get_color_frame()

    pc.map_to(color_frame)

    # cv2.imshow('fsaf',np.asanyarray(depth_frame.get_data()))
    # cv2.waitKey(0)
    
    points =  pc.calculate(depth_frame)
    print('\npoint',type(points),points)


    vtx = np.asanyarray(points.get_vertices())
    print('\nvtx',vtx.shape,vtx)
    npy = np.zeros((len(vtx),3),float)
    for i in range(len(vtx)):
        npy[i][0] = float(vtx[i][0])
        npy[i][1] = float(vtx[i][1])
        npy[i][2] = float(vtx[i][2])
    print('\nnpy',npy.shape,npy)


# t1 = time.time()
# test7()
# print('\nuse_time',time.time()-t1)

@jit(nopython=True)
def test5():
    for i in range(200,456):
        for j in range(200,620):
            print(i+j)

    
def test6():
    for i in range(200,456):
        for j in range(200,620):
            print(i+j)


def test8():
    for i in xrange(480):
        for j in xrange(640):
            print(i,j)

# t1 = time.time()
# test5()
# print('time',time.time()-t1)

# t1 = time.time()
# test5()
# t2 = time.time()
# print('total 1',t2-t1)
# t1 = time.time()
# test6()
# t2 = time.time()
# print('total 2',t2-t1)

def create_box(depth_img,box,offset=(0,0)):
    global lines,idcenvel
    uv1 = []
    for u in range(box[0],box[2]+1):
        for v in range(box[1],box[3]):
            if depth_img[u,v] > 10:
                continue
            else:
                depth = np.float(depth_img[u,v])
                uv1.append([u*depth,v*depth,depth])
    # 3*n
    uvd = np.array(uv1).T    
    # n*3
    K = np.random.rand(9).reshape(3,-1)
    yzx = K.dot(uvd).T

    print(yzx.shape)
    cx = yzx[2].mean()
    cy = yzx[0].mean()

    xmax = yzx[2].max()
    xmin = yzx[2].min()
    ymax = yzx[0].max()
    ymin = yzx[0].min()
    zmax = yzx[1].max()
    zmin = yzx[1].min()

    points = [Point(xmin,ymin,zmin),Point(xmax,ymin,zmin),
                Point(xmin,ymax,zmin),Point(xmax,ymax,zmin),
                Point(xmin,ymin,zmax),Point(xmax,ymin,zmax),
                Point(xmin,ymax,zmax),Point(xmax,ymax,zmax)]


    marker = Marker()
    marker.header.frame_id = 'map'
    marker.header.stamp = rospy.Time.now()
    marker.action = Marker.ADD
    marker.type = Marker.LINE_LIST

    marker.lifetime = rospy.Duration(0)

    marker.color.r = 1
    marker.color.g = 0
    marker.color.b = 0

    marker.color.a = 1
    marker.scale.x = 0.2
    marker.points = []

    for line in lines:
        marker.points.append(points[line[0]])
        marker.points.append(points[line[1]])

    return cx , cy , marker

# rospy.init_node('publisher3d')

# pub = rospy.Publisher('marker',MarkerArray,queue_size=1)


def PublishBbox(depth_img,bbox,time,identities=None,offset=(0,0)):
    global K,depth_thres,idcenvel,pub
    print(idcenvel)
    markerarray = MarkerArray()
    dt = (time - rospy.Time.now()).to_sec()
    idcv_tmp = []
    for i,id_ in enumerate(identities):
        marker = Marker()
        cx,cy,marker = create_box(depth_img,bbox[i],offset)
        marker.id = id_
        markerarray.markers.append(marker)
        flag = 0
        for idcv in idcenvel:
            if id_ == idcv[0]:

                vx = (cx - idcv[1])/dt
                vy = (cx - idcv[2])/dt

                idcv_tmp.append([id_,cx,cy,vx,vy])
                flag = 1
                break

        if not flag:
            vx=vy=0
            idcv_tmp.append([id_,cx,cy,vx,vy])
        
    print(idcv_tmp)
    idcenvel = idcv_tmp

    pub.publish(markerarray)



def test4():
    while not rospy.is_shutdown():
        time = rospy.Time.now()

        depth_image = np.random.uniform(5,11,(480,640))
        print(depth_image.shape)
        identities = [5,4]
        bbox=[[4,4,100,100],[8,8,203,203]]
        PublishBbox(depth_image,bbox,time,identities)

# test4()

def create_boxes(depth_img,bbox,identities=None,offset=(0,0)):
    K = np.random.rand(9).reshape(3,-1)
    for i,box in enumerate(bbox):
        uv1 = []
        for u in range(box[0],box[2]+1):
            for v in range(box[1],box[3]+1):
                # if depth_img[u,v] > 0.6:
                #     continue
                # else:
                print(u,v)

                uv1.append([u*depth_img[u,v],
                v*depth_img[u,v],
                depth_img[u,v]])

        uvd = np.array(uv1).T
        print(uvd.shape)

        xyz = K.dot(uvd).T
        print(xyz.shape)
        break
def test3():
    img = np.random.rand(640*480).reshape(480,-1)
    print(img)
    bbox = [[10,10,100,100],[6,6,40,40]]
    create_boxes(img,bbox=bbox)



def test2():
    pipe = rs.pipeline()
    cfg = rs.config()
    # cfg.enable_device()
    cfg.enable_stream(rs.stream.color,640,480,rs.format.rgb8,30)
    cfg.enable_stream(rs.stream.depth,640,480,rs.format.z16,30)
    profile = pipe.start(cfg)
    
    for x in range(5):
        pipe.wait_for_frames()

    frameset = pipe.wait_for_frames()
    color_frame = frameset.get_color_frame()
    depth_frame = frameset.get_depth_frame()
   
    # get profile param
    sensor = profile.get_device().first_depth_sensor()
    # print('sensor ',sensor)
    # print('scale',  sensor.get_depth_scale())
    
    #rgb
    color = np.asanyarray(color_frame.get_data())
    color = color[:,:,::-1]
    # cv2.imshow('color',color)
    # cv2.waitKey(0)
    # print('rgb:',color.shape)
    
    #depth
    depth = np.asanyarray(depth_frame.get_data())
    # print('\n depth',depth)
    # depth1 = depth[:,:,np.newaxis]
    # print('\n depth1',depth1)
    # depth2 = np.expand_dims(depth,axis=-1)
    # print('\n depth2',depth2)

    # cv2.imshow('depth',depth)
    # cv2.waitKey(0)

    # print('depth:',depth.shape)

    # two = np.hstack((color,depth))
    # cv2.imshow('two',two)
    # cv2.waitKey(0)

    colorizer = rs.colorizer()
    colorizer_depth = np.asanyarray(colorizer.colorize(depth_frame).get_data())
    # cv2.imshow('colorized_depth',colorizer_depth)
    # cv2.waitKey(0)
   
    # print('colorized_depth:',colorizer_depth.shape)

    #aligen
    # _init__(self: pyrealsense2.align, align_to: pyrealsense2.stream) → None

    # To perform alignment of a depth image to the other, set the align_to parameter with the other stream type. To perform alignment of a non depth image to a depth image, set the align_to parameter to RS2_STREAM_DEPTH. Camera calibration and frame’s stream type are determined on the fly, according to the first valid frameset passed to process().

    align = rs.align(rs.stream.color)
    alignframe = align.process(frameset)

    aligned_depth_frame = alignframe.get_depth_frame()
    aligned_color_frame = alignframe.get_color_frame()
    aligned_depth = np.asanyarray(aligned_depth_frame.get_data())
    aligned_color = np.asanyarray(aligned_color_frame.get_data())

    # cv2.imshow('aligned_depth',aligned_depth)
    # cv2.waitKey(0)
    
    # cv2.imshow('aligned_color',aligned_color)
    # cv2.waitKey(0)
    # print('aligned_color',aligned_color.shape)

    colorizer_align_depth = np.asanyarray(colorizer.colorize(aligned_depth_frame).get_data())
    
    # colorizer_depth = p.asanyarray(colorizer.colorize(depth_frame).get_data)
    print(color.shape)
    print(aligned_color.shape)
    print(colorizer_depth.shape)
    print(colorizer_align_depth.shape)
    # allpicture = np.hstack((color,aligned_color,colorizer_depth,colorizer_align_depth))

    # cv2.imshow('all',allpicture)
    cv2.imshow('color',color)
    cv2.waitKey(0)
    
    cv2.imshow('align_color',aligned_color)
    cv2.waitKey(0)
    cv2.imshow('depth',colorizer_depth)
    cv2.waitKey(0)
    cv2.imshow('align_depth',colorizer_align_depth)
    cv2.waitKey(0)

# test2()

def test1():
    pipe = rs.pipeline()

    config = rs.config()

    config.enable_stream(rs.stream.color,640,480,rs.format.bgr8,30)
    config.enable_stream(rs.stream.depth,640,480,rs.format.z16,30)
    config.enable_device()
    pipe.start(config)

    cap = cv2.VideoCapture(0)

    cap.set(cv2.CAP_PROP_BUFFERSIZE,3)

    while True:
        
        ret_val,img0 = cap.read()
        print("shape of cap:",img0.shape)

        frame = pipe.wait_for_frames()
        img1 = frame.get_color_frame()
        print("shape of img1 origin",img1)
        print("type of img1 origin",type(img1))

        img1 = np.asanyarray(img1.get_data())
        print("shape of img1",img1.shape)

        if cv2.waitKey(1)==ord('q'):
            cap.release()
            raise StopIteration




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

        print("origin shape img",img.shape)

        # Convert
        img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, to 3x416x416
        img = np.ascontiguousarray(img)

        print("convert shape img",img.shape)

        return img_path, img, img0, None

    def __len__(self):
        return 0
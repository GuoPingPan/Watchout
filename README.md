# Watchout
This project modifies yolov5+deepsort to be Watchout and combine with APF to make path planning.

这个项目是完成小车的一个跟踪功能，其中利用人工势场法来完成Path Planning @抗 的杰出工作，然后识别跟踪部分是利用yolov5+deepsort改成了watchout模块来完成。

## Structure
- can_work: 是跟踪代码可以运行的部分
- Demo1 & demo12 & percetion & Perception_Job： 使用雷达做导航的部分
- reconstruct: 将Watchout编写成一个module
- 其他文件是开发watchout的不同版本

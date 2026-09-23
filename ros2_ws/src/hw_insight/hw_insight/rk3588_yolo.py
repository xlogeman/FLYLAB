#!/usr/bin/env python3

import os
import sys
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
from hw_interface.msg import HWSimpleKeyboardInfo
from sensor_msgs.msg import Image, Range
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory 
from geometry_msgs.msg import PointStamped
import tf2_ros
import tf2_geometry_msgs
import numpy as np
import math
import message_filters
import time

from rknn.api import RKNN


# 物体置信度和物体类型可能性阈值
OBJ_THRESH = 0.60

# 同类物体检测框重合区域百分比阈值，重合区域超过该阈值则认为是检测到了同一个物体，只保留置信度最高的那一个
NMS_THRESH = 0.45

# YOLOv5模型处理的图片长和宽的像素个数
IMG_SIZE = 640

# 是否可视化检测结果
SHOW_RESULT = True

# 80种物体类型定义
CLASSES = ("person", "bicycle", "car", "motorbike ", "aeroplane ", "bus ", "train", "truck ", "boat", "traffic light",
           "fire hydrant", "stop sign ", "parking meter", "bench", "bird", "cat", "dog ", "horse ", "sheep", "cow", "elephant",
           "bear", "zebra ", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite",
           "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork", "knife ",
           "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza ", "donut", "cake", "chair", "sofa",
           "pottedplant", "bed", "diningtable", "toilet ", "tvmonitor", "laptop	", "mouse	", "remote ", "keyboard ", "cell phone", "microwave ",
           "oven ", "toaster", "sink", "refrigerator ", "book", "clock", "vase", "scissors ", "teddy bear ", "hair drier", "toothbrush ")


class RK3588YOLO(Node):
    # =================================================================================================================
    # 初始化rknn
    # =================================================================================================================
    def rknn_init(self):
        # onnx和rknn模型的位置，以及其他的路径和参数
        self.onnx_model_path = '/home/hw/ros/hw-ros2/ros2/src/hw_insight/yolo/yolov5s_relu_hw_download.onnx'
        self.rknn_model_path = '/home/hw/ros/hw-ros2/ros2/src/hw_insight/yolo/yolov5s_relu.rknn'
        #self.rknn_model_path = '/home/hw/ros/hw-ros2/ros2/src/hw_insight/yolo/yolov5n_hw_download.rknn'
        #self.rknn_model_path = '/home/hw/ros/hw-ros2/ros2/src/hw_insight/yolo/yolov5s_relu_hw_download.rknn'
        self.do_quantization = True     # 是否要进行量化
        # 量化使用的数据，需要换成airism场景的各种图片，且分辨率要是640x640
        self.dataset_path = '/home/hw/ros/hw-ros2/ros2/src/hw_insight/yolo/dataset.txt'

        # 读取NPU核心设置参数，用来设置要绑定的NPU核心
        self.declare_parameter('npu_core', 0)
        self.npu_core = self.get_parameter('npu_core').value

        # 创建RKNN对象，打开详细信息显示
        self.rknn = RKNN(verbose=False)
        # 设置默认均值和默认归一化系数，设置平台为3588
        self.rknn.config(mean_values=[[0, 0, 0]], std_values=[[255, 255, 255]], target_platform='rk3588')

        # 计算RKNN模型推理的平均时间
        self.rknn_total_time = 0.0
        self.rknn_total_count = 0
        self.rknn_total_anv = 0.0

        # 计算整个图片处理过程的平均时间
        self.run_total_time = 0.0
        self.run_total_count = 0
        self.run_total_anv = 0.0


    # 没有rknn模型的情况下，从onnk开始构造rknn模型，然后导出
    def rknn_build(self):
        # 加载onnx模型
        ret = self.rknn.load_onnx(model=self.onnx_model_path)
        if ret != 0:
            self.get_logger().info('Load model failed!')
            exit(ret)

        # 从onnx构造rknn模型，开启量化，并选择量化用到的图片集合
        ret = self.rknn.build(do_quantization=self.do_quantization, dataset=self.dataset_path)
        if ret != 0:
            self.get_logger().info('Build model failed!')
            exit(ret)

        # 导出rknn模型到文件，后续可直接load该文件，不用重新build
        ret = self.rknn.export_rknn(self.rknn_model_path)
        if ret != 0:
            self.get_logger().info('Export rknn model failed!')
            exit(ret)

        # 设置运行环境
        ret = self.rknn.init_runtime(target='rk3588', core_mask=RKNN.NPU_CORE_0_1_2)
        if ret != 0:
            self.get_logger().info('Init runtime environment failed!')
            exit(ret)


    # 已有rknn模型的情况下，直接load
    def rknn_load(self):
        # 加载RKNN模型
        ret = self.rknn.load_rknn(self.rknn_model_path)
        if ret != 0:
            self.get_logger().info('Load RKNN model failed!')
            exit(ret)
        
        # 设置运行环境，根据npu_core参数选择推理所用的NPU核心
        if self.npu_core == 0 :
            ret = self.rknn.init_runtime(target='rk3588', core_mask=RKNN.NPU_CORE_0)
        elif self.npu_core == 1 :
            ret = self.rknn.init_runtime(target='rk3588', core_mask=RKNN.NPU_CORE_1)
        elif self.npu_core == 2 :
            ret = self.rknn.init_runtime(target='rk3588', core_mask=RKNN.NPU_CORE_2)
        else:
            ret = self.rknn.init_runtime(target='rk3588', core_mask=RKNN.NPU_CORE_0_1_2)
        if ret != 0:
            self.get_logger().info('Init runtime environment failed!')
            exit(ret)

    # =================================================================================================================
    # 使用NPU运行YOLO识别图片 
    # =================================================================================================================
    # 运行RKNN YOLO模型，得到最终结果，和函数 results = self.model(img, classes=[0], conf=0.6, verbose=False) 功能一致
    # 入参img是从self.bridge.imgmsg_to_cv2(rgb_msg, "rgb8")转换得到的图片
    def rknn_run_yolo(self, img: np.ndarray):
        # 增加image的维度，从hwc变成nhwc：
        # N=Batch size(批次)    H=Height(高度)    W=Width(宽度)    C=Channels(通道数)
        img2 = np.expand_dims(img, 0)
        # 使用模型进行推理，得到推理结果outputs。输入data_format是nhwc格式，而输出outputs是nchw格式

        # 使用RKNN模型进行推理，并计算其单次耗时和平均耗时
        start_time = time.perf_counter()
        outputs = self.rknn.inference(inputs=[img2], data_format=['nhwc'])
        end_time = time.perf_counter()
        execution_time = (end_time - start_time) * 1000.0
        self.rknn_total_time = self.rknn_total_time + execution_time
        self.rknn_total_count = self.rknn_total_count + 1
        self.rknn_total_anv = self.rknn_total_time / self.rknn_total_count
        self.get_logger().info(f"rknn.inference={execution_time:.2f}ms, anverage={self.rknn_total_anv:.2f}ms")

        # 由于YOLOv5使用3个检测头，因此outputs中包含有3个检测头的数据，分别放到input0_data，input1_data，input2_data
        # 三个output均是nchw格式。c=channel包含所有锚点信息，由于是3个锚点，每种锚点有80种物体，还有x,y,w,h,conf5个信息
        # 因此一个锚点就有3x(5+80) = 255，所以c的大小是255，而n表示批次，固定为1
        input0_data = outputs[0]    # [1,255,80,80]
        input1_data = outputs[1]    # [1,255,40,40]
        input2_data = outputs[2]    # [1,255,20,20]

        # 将 [1,255,20,20] 先去掉n得到[255,20,20]，再将255按照锚点分开得到[3,85,20,20]
        # 另外两个检测头类似
        input0_data = input0_data.reshape([3, -1]+list(input0_data.shape[-2:]))
        input1_data = input1_data.reshape([3, -1]+list(input1_data.shape[-2:]))
        input2_data = input2_data.reshape([3, -1]+list(input2_data.shape[-2:]))

        # 将[3,85,20,20]变成[20,20,3,85]。 另外两个检测头类似
        # 并将三个检测头数据合并到一个list中
        input_data = list()
        input_data.append(np.transpose(input0_data, (2, 3, 0, 1)))
        input_data.append(np.transpose(input1_data, (2, 3, 0, 1)))
        input_data.append(np.transpose(input2_data, (2, 3, 0, 1)))

        # 解析输出数据，得到最终的物体检测框，物体类型，置信度数据
        boxes, classes, scores = self.yolov5_post_process(input_data)
        return boxes, classes, scores


    # =================================================================================================================
    # YOLOv5输出结果解析流程 (基于640x640输入)
    # =================================================================================================================
    # 1. **网格化预测**: YOLOv5将输入图像看作是由多个网格组成的棋盘。其核心任务是，为每一个网格单元，预测是否有物体的“中心点”落入其中，并进一步预测该物体的具体类别、位置和尺寸。
    
    # 2. **多尺度检测头**: 为了检测不同大小的物体，YOLOv5使用了3个检测头，它们在不同尺寸的特征图上工作：
    #    - **小物体检测头**: 使用8像素的步长（Stride），在80x80的特征图上进行预测。
    #    - **中物体检测头**: 使用16像素的步长，在40x40的特征图上进行预测。
    #    - **大物体检测头**: 使用32像素的步长，在20x20的特征图上进行预测。
    
    # 3. **锚点机制**: 在每个网格单元上，都预设了3个不同形状和大小的锚点框（Anchor Box）作为预测基准。3个检测头共使用9种不同的锚点。
    
    # 4. **原始输出**: 因此，对于一张640x640的图像，YOLOv5会一次性并行生成 (80*80 + 40*40 + 20*20) * 3 = 25,200个原始预测。
    #    每个预测都是一个包含85个值的向量：[tx, ty, tw, th, conf, p_class_0, ..., p_class_79]
    #    - `tx, ty`: 中心点在网格内的相对偏移量。通过解码（加上网格坐标并乘以步长）可得到像素级坐标。
    #    - `tw, th`: 宽高相对于锚点尺寸的缩放因子。通过解码（乘以锚点宽高）可得到像素级宽高。
    #    - `conf`: 物体置信度，表示该位置存在一个物体的可能性。
    #    - `p_class_n`: 80个类别的条件概率。
    
    # 5. **后处理-过滤**: 接下来进入后处理的过滤阶段。
    #    - 首先，使用预设的物体置信度阈值（OBJ_THRESH）进行两轮筛选：第一轮，过滤掉那些conf值过低的预测；第二轮，在幸存者中，再过滤掉其最大类别概率（prob_max）也低于OBJ_THRESH的预测。
    #    - 对于通过两轮筛选的候选框，我们执行两个关键操作：
    #        a. **计算最终置信度**: final_score = conf * prob_max。
    #        b. **确定物体类别**: 记录下prob_max对应的类别索引值（例如，0代表人，2代表车）。
    #    - 此时，我们得到三个独立的、长度大大缩减的数组：一个包含边界框坐标的数组，一个包含最佳类别索引的数组，以及一个包含最终置信度的数组。

    # 6. **后处理-NMS**: 最后，进入非极大值抑制（NMS）阶段。
    #    - 算法将所有候选框按类别进行分组，然后在每个类别内部独立操作：找到所有重叠度（IOU）大于NMS阈值（NMS_THRESH）的框，并只保留该组中最终置信度（final_score）最高的那一个，抑制掉其他所有框。
    
    # 7. **最终结果**: 经过NMS后，剩下的就是最终的、高质量的检测结果。
    # =================================================================================================================
    def yolov5_post_process(self, input_data):
        # 三个检测头对应的9种锚点框大小
        anchors = [[10, 13], [16, 30], [33, 23], [30, 61], [62, 45],[59, 119], [116, 90], [156, 198], [373, 326]]
        # 区分每个检测头使用哪三个锚点。第一个检测头使用anchors数值中索引为0,1,2的锚点框。依次类推
        masks = [[0, 1, 2], [3, 4, 5], [6, 7, 8]]

        boxes, classes, scores = [], [], []
        # masks数组有3个元素(检测头对应的锚点框大小数据)，input_data数组也有3个元素(3个检测头的数据)，将它们一一对应取出来进行处理
        for input, mask in zip(input_data, masks):
            # 依次处理每个检测头的数据，得到物体检测框的位置和大小，物体置信度，不同类型物体的可能性
            b, c, s = self.process(input, mask, anchors)
            # 使用OBJ_THRESH对数据进行筛选，找出物体置信度和物体可能性均大于OBJ_THRESH的数据，并计算最终置信度得分
            b, c, s = self.filter_boxes(b, c, s)
            # 将每个检测头过滤后的数据记录下来
            boxes.append(b)
            classes.append(c)
            scores.append(s)

        # 在这里，boxes, classes, scores 都是Python的list，每个list中包含3个Numpy数组（分别来自3个检测头）
        # 下一步，我们将这些零散的数据合并成一个单一的、巨大的Numpy数组，为后续的批量处理做准备。
        # 将3个 [F_head_1, 4], [F_head_2, 4], [F_head_2, 4] 的boxes数组，合并成一个 [F_head_1+F_head_2+F_head_3, 4] 大数组。
        boxes = np.concatenate(boxes)
        # 将3个长度为 F_head_1, F_head_2, F_head_3 的 classes 和 scores 一维数组，分别合并成长度为 F_head_1+F_head_2+F_head_3 的两个一维大数组。
        classes = np.concatenate(classes)
        scores = np.concatenate(scores)
        
        # 将所有边界框的坐标格式，从 [中心点x, 中心点y, 宽, 高] 转换为 [左上角x, 左上角y, 右下角x, 右下角y]。
        # 这种 [x1, y1, x2, y2] 的格式对于后续计算IOU（在NMS中）更为方便。
        boxes = self.xywh2xyxy(boxes)

        # 针对每个物体类型，找到重合区域大于阈值的检测框，只保留其中最终置信度最高的那一个
        nboxes, nclasses, nscores = [], [], []
        for c in set(classes):  # 取出一个class
            inds = np.where(classes == c)   # 找到和该class相同的所有数据，记录索引值到数组inds
            b = boxes[inds]                 # 用索引数组inds提取出所有相关数据，形成新的数组b,c,s
            c = classes[inds]
            s = scores[inds]
            # 经过非极大值抑制过滤后，剩下的数据保存在KEEP中
            keep = self.nms_boxes(b, s)
            # 转存到nboxes，nclasses，nscores中
            nboxes.append(b[keep])
            nclasses.append(c[keep])
            nscores.append(s[keep])
    
        if not nclasses and not nscores:
            return None, None, None
    
        # 将list转换为数组，并返回
        boxes = np.concatenate(nboxes)
        classes = np.concatenate(nclasses)
        scores = np.concatenate(nscores)
        return boxes, classes, scores



    # 解析一个检测头的数据，转换成边框，类型和置信度。检测头有三个，yolov5_post_process会依次调用本函数来处理
    # 三个检测头分别是 80x80(在特征图上划分成6400个格子), 40x40(划分成1600个格子), 20x20(划分成400个格子)
    # 每个格子里有3个锚点，分别对应3中不同大小的检测框。以20x20的网络为例，格子配合锚点总共就有1200个锚点框
    # 每个锚点框会给出一个物体置信度box_confidence，表示当前格子存在物体的可能性，是一个单独的数，在0~1之间
    # 每个锚点框里还有一个物体可能性数组box_class_probs，针对CLASSES定义的物体类型，给出每种类型的可能性。所有可能性加起来大约等于1(根据算法不同有些许区别)
    # 将大于OBJ_THRESH的box_confidence和box_class_probs中最大的那个值乘起来，得到的就是这个锚点框的最终置信度
    def process(self, input, mask, anchors):
        # 取出一个检测头对应的3个锚点(先验框)，anchors维度是[3,2]表示[3个锚点,长和宽]，值表示长度或者宽度
        anchors = [anchors[i] for i in mask]
        # 读取出网格的高度和宽度，以20x20检测头为例，就是grid_h=20, grid_w=20
        grid_h, grid_w = map(int, input.shape[0:2])
        # 读取出每个网络下每个锚点对应的物体置信度。有400个格子，3个锚点，所以box_confidence维度是[20,20,3]表示[网格高,网格宽,锚点]，值表示物体置信度
        box_confidence = input[..., 4]
        # 增加一个维度，box_confidence维度变成[20,20,3,1]表示[网格高,网格宽,锚点,其他], 值仍然表示物体置信度
        box_confidence = np.expand_dims(box_confidence, axis=-1)
        # 读取出每种物体的可能性，目前总共有80种物体类型，再配合上400x3个锚点，所以box_class_probs维度是[20,20,3,80]表示[网格高,网格宽,锚点,物体类型]，值表示不同类型物体的可能性
        box_class_probs = input[..., 5:]
        # 计算物体中心点的相对偏移量，每个锚点框对应一个偏移量,维度是[20,20,3,2]，表示[网格高,网格宽,锚点,XY偏移量], 值表示物体中心坐标相对于网格坐标的X或Y偏移量
        box_xy = input[..., :2]*2 - 0.5

        # 生成一个网格矩阵grid，维度是 [20,20,3,2] 表示[长，宽，锚点，XY坐标]，值表示一个网格的X或Y坐标
        col = np.tile(np.arange(0, grid_w), grid_w).reshape(-1, grid_w)
        row = np.tile(np.arange(0, grid_h).reshape(-1, 1), grid_h)
        col = col.reshape(grid_h, grid_w, 1, 1).repeat(3, axis=-2)
        row = row.reshape(grid_h, grid_w, 1, 1).repeat(3, axis=-2)
        grid = np.concatenate((col, row), axis=-1)

        # box_xy 和 grid 的维度完全相同。grid记录的是网格XY坐标，box_xy记录的是物体相对于网格坐标的XY偏移量
        # 两者相加后得到物体的中心在特征图上的绝对坐标。然后再乘以步长就得到物体中心在原始图片上的像素绝对坐标
        # 下面计算完成后，box_xy的维度仍然是[20,20,3,2]，表示[网格高,网格宽,锚点,XY坐标]，值表示物体中心的像素级绝对坐标的XY值
        box_xy += grid
        box_xy *= int(IMG_SIZE/grid_h)

        # 计算每个物体对应的长宽缩放因子，然后乘以锚点的长宽，得到物体的实际像素长宽
        # 下面计算完成后，box_wh 的维度是 [20,20,3,2], 表示[网格高,网格宽,锚点,长宽]，值表示物体检测框的像素级长宽
        box_wh = pow(input[..., 2:4]*2, 2)
        box_wh = box_wh * anchors

        # 将box_xy和box_wh合并，就得到每个锚点框对应的物体的像素级中心坐标和像素级长宽。维度变成[20,20,3,4]
        box = np.concatenate((box_xy, box_wh), axis=-1)

        # 返回每个锚点对应的：物体方框位置和大小[20,20,3,4]，物体置信度[20,20,3,1]，每种类型置信度[20,20,3,80]
        return box, box_confidence, box_class_probs


    # 根据物体置信度和每种物体可能性，按照阈值筛选出符合条件的数据
    # 注意，下面所有的分析均针对20x20的检测头。其他40x40和80x80的检测头类似。
    def filter_boxes(self, boxes, box_confidences, box_class_probs):
        # 1多维数据扁平化
        # 将boxes从[20,20,3,4]变成[1200,4]，表示[所有网格中的所有锚点框, 检测框位置和大小]，值表示物体检测框的像素级 x,y,w,h
        boxes = boxes.reshape(-1, 4)
        # 将box_confidences从[20,20,3,1]变成[1200]，每个值表示一个锚点框的物体置信度
        box_confidences = box_confidences.reshape(-1)
        # 将box_class_probs从[20,20,3,80]变成[1200, 80]，表示[所有网格中的所有锚点框, 所有物体类型]，值表示一个物体类型的可能性
        box_class_probs = box_class_probs.reshape(-1, box_class_probs.shape[-1])

        # 2筛选物体置信度
        # 在box_confidences中找出所有物体置信度大于OBJ_THRESH的元素，并将它们的索引记录到数组_box_pos中
        _box_pos = np.where(box_confidences >= OBJ_THRESH)
        # 使用_box_pos中记录的索引值，对boxes，box_confidences，box_class_probs进行第一次筛选，会让它们的长度大幅减少
        # 也就是之前维度中的1200这个值，会变成一个比较小的值F1，F1的大小就是_box_pos数组的长度
        boxes = boxes[_box_pos]                         # 变成了[F1, 4]
        box_confidences = box_confidences[_box_pos]     # 变成了[F1]
        box_class_probs = box_class_probs[_box_pos]     # 变成了[F1, 80]

        # 3筛选物体可能性
        # 在上一轮筛选后的box_class_probs中，找到其中概率最大的那个物体类型的得分和索引，分别记录在class_max_score和classes中
        class_max_score = np.max(box_class_probs, axis=-1)  # class_max_score维度是[F1]，值表示每个锚点框里，在80个物体类型中，最大可能性的值
        classes = np.argmax(box_class_probs, axis=-1)       # classes维度是[F1]，值表示最大可能性的物体在80个物体类型中的索引值，就是用来记录最大可能性的物体是什么类型的物体
        # 从class_max_score中找出所有其值大于OBJ_THRESH的索引值，并存放在_class_pos中，class_po的长度是F2，大部分时候都会比F1要小
        _class_pos = np.where(class_max_score >= OBJ_THRESH)

        # 使用_class_pos对数据进行第二次筛选，经过本次筛选，剩下的数据从F1进一步减少到F2
        boxes = boxes[_class_pos]       # 变成了[F2, 4]
        classes = classes[_class_pos]   # 变成了[F2]，值表示最大可能性的物体在80个物体类别中的索引
        scores = (class_max_score* box_confidences)[_class_pos] # 维度[F2]，将物体置信度和最大物体可能性相乘，得到最终的物体置信度

        # 返回检测框位置大小[F2,4], 物体类型索引[F2], 最终置信度得分[F2]
        return boxes, classes, scores


    # 将 中心+长宽 转换为 左上+右下
    def xywh2xyxy(self, x):
        y = np.copy(x)
        y[:, 0] = x[:, 0] - x[:, 2] / 2  # top left x
        y[:, 1] = x[:, 1] - x[:, 3] / 2  # top left y
        y[:, 2] = x[:, 0] + x[:, 2] / 2  # bottom right x
        y[:, 3] = x[:, 1] + x[:, 3] / 2  # bottom right y
        return y


    # 非极大值抑制。针对一个特定类别，找到所有重合区域大于阈值的检测框，只保留其中最终置信度最高的那一个
    def nms_boxes(self, boxes, scores):
        # 提前和计算出左上坐标以及宽高
        x = boxes[:, 0]
        y = boxes[:, 1]
        w = boxes[:, 2] - boxes[:, 0]
        h = boxes[:, 3] - boxes[:, 1]

        # 计算检测框面积
        areas = w * h
        # 按照置信度的降序排序
        order = scores.argsort()[::-1]

        # 反复取出置信度最高的那个，然后过滤掉所有重合面积比例大于NMS_THRESH的数据，反复如此直到order中没有数据，将最终结果存放在keep中
        keep = []
        while order.size > 0:
            # 取出当前置信度最高的那个检测框
            i = order[0]
            keep.append(i)

            # 计算其他所有的检测框和最高置信度检测框的重合度
            xx1 = np.maximum(x[i], x[order[1:]])
            yy1 = np.maximum(y[i], y[order[1:]])
            xx2 = np.minimum(x[i] + w[i], x[order[1:]] + w[order[1:]])
            yy2 = np.minimum(y[i] + h[i], y[order[1:]] + h[order[1:]])
            # 计算出重合部分的面积inter
            w1 = np.maximum(0.0, xx2 - xx1 + 0.00001)
            h1 = np.maximum(0.0, yy2 - yy1 + 0.00001)
            inter = w1 * h1
            # 计算出交并总面积(areas[i] + areas[order[1:]] - inter)，然后用inter除以该面积得到重合率
            ovr = inter / (areas[i] + areas[order[1:]] - inter)
            # 取出所有重合率小于阈值NMS_THRESH进行下一轮筛选
            inds = np.where(ovr <= NMS_THRESH)[0]
            order = order[inds + 1]
        keep = np.array(keep)
        return keep


    # 在原始图片上画出检测框
    def draw(self, image, boxes, scores, classes):
        for box, score, cl in zip(boxes, scores, classes):
            # 从box中提取方框坐标
            x1, y1, x2, y2 = box
            # 转换为整数
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)    
            # cv2.rectangle 的两个点分别是左上角(x1, y1)和右下角(x2, y2)
            cv2.rectangle(image, (x1, y1), (x2, y2), (255, 0, 0), 2) # Blue box
            # putText的坐标是文本的左下角，把它对应在检测框的左上角
            label_text = f'{CLASSES[cl]} {score:.2f}'
            cv2.putText(image, label_text,
                        (x1, y1 - 10), # 在左上角x坐标，y坐标向上移动10个像素
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 0, 255), 2)
            # 打印信息
            #self.get_logger().info(f"{CLASSES[cl]:^12} {score:^12.3f} [{x1:>4}, {y1:>4}, {x2:>4}, {y2:>4}]")



    # =================================================================================================================
    # 入口函数，主流程
    # =================================================================================================================
    def __init__(self) -> None:
        super().__init__('rk3588_yolo')

        # 初始化rknn
        self.rknn_init()
        #self.rknn_build()
        self.rknn_load()

        # 读取相机名字参数
        self.declare_parameter('camera_name', "CameraDepth1")
        self.camera_name = self.get_parameter('camera_name').get_parameter_value().string_value

        # 订阅相机的RGB图片和深度图片。RGB图片用来给YOLO识别物体，深度图片用来根据YOLO识别的结果计算坐标值
        self.rgb_sub = message_filters.Subscriber(self, Image, f'/airsim_node/PX4/{self.camera_name}/Scene')
        self.depth_sub = message_filters.Subscriber(self, Image, f'/airsim_node/PX4/{self.camera_name}/DepthPlanar')
        # 创建一个近似时间同步器，确保RGB图片和DEPTH图片的同步
        # 参数：订阅者列表，队列大小，时间戳容差（秒）
        self.time_synchronizer = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], 2, 0.01) 
        # 注册同步后的回调函数
        self.time_synchronizer.registerCallback(self.sync_callback)

        # CvBridge用来在RGB和ROS的image格式间进行转换
        self.bridge = CvBridge()

        # 用来发布YOLO识别后，带识别方框的RGB图片
        self.yolo_pub = self.create_publisher(Image, '/yolo/output', 1)
        # 用来发布识别到的人的位置信息，基于PX4_odom的坐标
        self.point_pub = self.create_publisher(PointStamped, '/yolo/person_position', 1)

        # TF2 buffer + listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info(f"Camera={self.camera_name}, YOLOv5.RKNN Started.")


    # 同步回调函数，它会同时接收到 rgb_msg 和 depth_msg
    def sync_callback(self, rgb_msg, depth_msg):
        start_time = time.perf_counter()
        #这个回调函数只在接收到时间戳相近的RGB和深度图时才被触发。
        try:
            # 将ROS图像消息转换为OpenCV图像
            img = self.bridge.imgmsg_to_cv2(rgb_msg, "rgb8")
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Failed to convert images: {e}")
            return

        # 使用YOLOv5模型进行目标检测
        all_boxes, all_classes, all_scores = self.rknn_run_yolo(img)

        # 如果没有检测到任何物体，则返回
        if all_boxes is None or len(all_boxes) == 0:
            #if SHOW_RESULT :
            #    self.yolo_pub.publish(rgb_msg)
            return

        # 筛选出所有类别为person(class_id = 0)的检测结果
        TARGET_CLASS_ID = 0
        person_indices = np.where(all_classes == TARGET_CLASS_ID)[0]

        # 如果检测结果不含person，则直接返回
        if len(person_indices) == 0:
            #self.get_logger().info("No person detected in the frame.")
            #if SHOW_RESULT:
            #    self.yolo_pub.publish(rgb_msg) # 同样发布原始图像
            return
        
        # 从所有person中，找到置信度最高的那一个
        # 把person 的分数提取出来
        person_scores = all_scores[person_indices]
        # 然后，在这些 person 的分数中找到最大值的索引
        # 这个 max_score_index 是相对于 person_scores 这个子数组的索引
        max_score_index_in_persons = np.argmax(person_scores)
        if person_scores[max_score_index_in_persons] < OBJ_THRESH : # 如果最终置信度小于阈值，则直接返回
            return
        # 最后，用这个子数组的索引，去找到它在原始 person_indices 数组中的位置，
        # 从而得到那个最高分 person 在 all_boxes, all_classes, all_scores 中的最终索引
        best_person_original_index = person_indices[max_score_index_in_persons]
        # 提取出这唯一一个、最好的person的信息
        # 注意：我们用 [index:index+1] 或 np.newaxis 来保持数组的二维/一维形状，
        # 目的是兼容之前用ultralytics的YOLO的检测结果格式
        final_box = all_boxes[best_person_original_index:best_person_original_index+1]
        final_class = all_classes[best_person_original_index:best_person_original_index+1]
        final_score = all_scores[best_person_original_index:best_person_original_index+1]
        self.get_logger().info(f"Found best person with score: {final_score[0]:.2f}")

        # 可视化检测结果
        if SHOW_RESULT :
            img_result = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            self.draw(img_result, final_box, final_score, final_class)
            out_msg = self.bridge.cv2_to_imgmsg(img_result, "bgr8")
            out_msg.header = rgb_msg.header # 保持header一致
            self.yolo_pub.publish(out_msg)

        # 计算目标位置并发布
        # 只处理第一个检测到的目标
        x1, y1, x2, y2 = map(int, final_box[0])

        # 计算目标中心点像素坐标
        u = (x1 + x2) // 2
        v = (y1 + y2) // 2

        # 检查像素坐标是否在图像范围内
        if not (0 <= v < depth_image.shape[0] and 0 <= u < depth_image.shape[1]):
            self.get_logger().warn("Pixel coordinate out of depth image bounds.")
            return

        # 从同步好的深度图中获取深度值
        depth = depth_image[v, u]
        
        # 检查深度值是否有效
        if depth == 0 or np.isinf(depth) or np.isnan(depth):
            self.get_logger().warn(f"Invalid depth value at ({u},{v}): {depth}")
            return

        #self.get_logger().info(f"Detected person at pixel ({u}, {v}) with depth: {depth:.2f} meters")
        # 相机内参, 分辨率 640x640，水平视场角 FOV = 120° (后续考虑从 CameraInfo 话题获取，这里暂时硬编码)
        HFOV_deg = 120.0
        HFOV_rad = math.radians(HFOV_deg)
        W = 640.0
        H = 640.0
        # 根据公式 fx = W / (2 * tan(FOV/2)) 计算焦距（像素）
        fx = W / (2 * math.tan(HFOV_rad / 2))
        fy = fx  # AirSim像素是方形的
        cx = W / 2
        cy = H / 2

        # 像素坐标到相机坐标系转换
        x_cam = (u - cx) * depth / fx
        y_cam = (v - cy) * depth / fy
        z_cam = float(depth)

        # 创建 PointStamped 消息
        point_camera = PointStamped()
        point_camera.header = rgb_msg.header
        point_camera.point.x = x_cam
        point_camera.point.y = y_cam
        point_camera.point.z = z_cam

        try:
            # 将点从相机坐标系转换到PX4_odom坐标系
            target_frame = 'PX4_odom'
            point_odom = self.tf_buffer.transform(
                point_camera,
                target_frame,
                timeout=rclpy.duration.Duration(seconds=0.01) # 设置一个合理的超时
            )

            #self.get_logger().info(f"Person position in {target_frame}: x={point_odom.point.x:.2f}, y={point_odom.point.y:.2f}, z={point_odom.point.z:.2f}")
            self.point_pub.publish(point_odom)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f"TF transform failed: {e}")

        end_time = time.perf_counter()
        execution_time = (end_time - start_time) * 1000.0
        self.run_total_time = self.run_total_time + execution_time
        self.run_total_count = self.run_total_count + 1
        self.run_total_anv = self.run_total_time / self.run_total_count
        self.get_logger().info(f"total time = {execution_time:.2f}ms, anverage = {self.run_total_anv:.2f}ms\n")


def main(args=None) -> None:
    print('Starting YOLO test1 node...')
    rclpy.init(args=args)
    rk3588_yolo = RK3588YOLO()
    rclpy.spin(rk3588_yolo)
    rk3588_yolo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)

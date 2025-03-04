#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-
# publisher.py
# MBJC

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point 
from std_msgs.msg import Float64
from color_detection.msg import ColorBlock


class image_converter:
    def __init__(self):    
        # 创建cv_bridge，声明图像的发布者和订阅者
        self.image_pub = rospy.Publisher("cv_bridge_image", Image, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/cafe_table/camera1/image_raw", Image, self.callback)
        # self.point_pub = rospy.Publisher('/point', Point, queue_size=10)
        # self.float64_pub = rospy.Publisher('/float64', Float64, queue_size=10)
        self.center_pub = rospy.Publisher("/color_block", ColorBlock, queue_size=1)
        self.color_block = ColorBlock()
    def callback(self,data):
        cx=0
        cy=0
        theta=0
        # 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
        try:
            frame0 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print (e)
        frame=frame0.copy()
        frame=frame[0:350,0:350]
    

        font = cv2.FONT_HERSHEY_SIMPLEX
        lower_red = np.array([0, 100, 100])
        higher_red = np.array([15, 255, 255])
        lower_green = np.array([40, 70, 70])  # 绿色阈值下界
        higher_green = np.array([90, 255, 255])  # 绿色阈值上界

        lower_blue = np.array([100, 100, 100])
        higher_blue = np.array([124, 255, 255])

        lower_purple = np.array([125, 80, 80])
        higher_purple = np.array([255, 255, 255])

        lower_yellow = np.array([25, 100, 100])
        higher_yellow = np.array([35, 255, 255])

        # lower_orange = np.array([9, 100, 150])
        # higher_orange = np.array([24, 255, 255])
        
        img_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_red = cv2.inRange(img_hsv, lower_red, higher_red)  # 可以认为是过滤出红色部分，获得红色的掩膜,去掉背景
        mask_green = cv2.inRange(img_hsv, lower_green, higher_green)  # 获得绿色部分掩膜
        mask_blue = cv2.inRange(img_hsv, lower_blue, higher_blue)  # 获得蓝色部分掩膜
        mask_purple = cv2.inRange(img_hsv,lower_purple,higher_purple)  # 获得紫色部分掩膜
        mask_yellow = cv2.inRange(img_hsv,lower_yellow,higher_yellow)  # 获得色黄部分掩膜
        # mask_orange = cv2.inRange(img_hsv, lower_orange, higher_orange)

        # 中值滤波(把数字图像中的一点的值用该点的邻域各点的中值代替，让周围像素值接近真实值，
        # 从而消除孤立的噪声点)
        mask_red = cv2.medianBlur(mask_red, 7)
        mask_green = cv2.medianBlur(mask_green, 7)
        mask_blue = cv2.medianBlur(mask_blue, 7)
        mask_purple = cv2.medianBlur(mask_purple, 7)
        mask_yellow = cv2.medianBlur(mask_yellow, 7)
        # mask_orange = cv2.medianBlur(mask_orange, 7)

        # 高斯滤波
        mask_red = cv2.GaussianBlur(mask_red, (5, 5), 0)
        mask_green = cv2.GaussianBlur(mask_green, (5, 5), 0)
        mask_blue = cv2.GaussianBlur(mask_blue, (5, 5), 0)
        mask_purple = cv2.GaussianBlur(mask_purple, (5, 5), 0)
        mask_yellow = cv2.GaussianBlur(mask_yellow, (5, 5), 0)
        # mask_orange = cv2.GaussianBlur(mask_orange, (5, 5), 0)

        # mask = cv2.bitwise_or(mask_red, mask_red)  # 三部分掩膜进行按位或运算

        cnts1, hierarchy1 = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        cnts2, hierarchy2 = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        cnts3, hierarchy3 = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        cnts4, hierarchy4 = cv2.findContours(mask_purple, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        cnts5, hierarchy5 = cv2.findContours(mask_yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        # cnts6, hierarchy6 = cv2.findContours(mask_orange, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)



        if(cnts1 or cnts2 or cnts3 or cnts4 or cnts5):
            for cnt in cnts1:  # Red
                (x, y, w, h) = cv2.boundingRect(cnt)
                rect = cv2.minAreaRect(cnt)  # 得到最小外接矩阵的（中心（x，y）（宽高），旋转角度）
                area = cv2.contourArea(cnt)  # 获得物体面积
                if (area < 500):  # 小于2000就跳过
                    continue
                box = cv2.boxPoints(rect)  # 获取最小外界矩形的4个顶点坐标
                box = np.int0(box)  # 取整
                cx = int(rect[0][0])  # 获取中心点x坐标
                cy = int(rect[0][1])  # 获取中心点y坐标

                self.color_block.color = "red"

                print('red:', cx, cy)

                # 获取旋转角度
                theta = cv2.minAreaRect(cnt)[2]
                if abs(theta) <= 45:
                    print('图片旋转角度为%s.' % theta)
                frame = cv2.drawContours(frame, [box], 0, (0, 0, 255), 3)  # 画旋转方框
                frame = cv2.line(frame, (cx + 5, cy), (cx - 5, cy), (0, 0, 255), 2)
                frame = cv2.line(frame, (cx, cy + 5), (cx, cy - 5), (0, 0, 255), 2)
                # cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
                # cv2.rectangle(frame, (x2, y2), (x2 + w2, y2 + h2), (0, 255, 255), 2)
                cv2.putText(frame, "Red", (x, y - 5), font, 0.5, (0, 0, 255), 2)

            for cnt in cnts2:  # green
                (x, y, w, h) = cv2.boundingRect(cnt)  # 返回矩阵四个点
                rect = cv2.minAreaRect(cnt)  # 得到最小外接矩阵的（中心（x，y）（宽高），旋转角度）
                area = cv2.contourArea(cnt)  # 获得物体面积
                if (area < 500):  # 小于2000就跳过
                    continue
                box = cv2.boxPoints(rect)  # 获取最小外界矩形的4个顶点坐标
                box = np.int0(box)  # 取整
                cx = int(rect[0][0])  # 获取中心点x坐标
                cy = int(rect[0][1])  # 获取中心点y坐标
                self.color_block.color = "green"
                print('green:', cx, cy)
                # 获取旋转角度
                theta = cv2.minAreaRect(cnt)[2]
                if abs(theta) <= 45:
                    print('图片旋转角度为%s.' % theta)
                frame = cv2.drawContours(frame, [box], 0, (0, 255, 0), 3)  # 画旋转方框
                frame = cv2.line(frame, (cx + 5, cy), (cx - 5, cy), (0, 255, 0), 2)
                frame = cv2.line(frame, (cx, cy + 5), (cx, cy - 5), (0, 255, 0), 2)
                # cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
                cv2.putText(frame, "Green", (x, y - 5), font, 0.5, (0, 255, 0), 2)

            for cnt in cnts3:  # blue
                (x, y, w, h) = cv2.boundingRect(cnt)
                rect = cv2.minAreaRect(cnt)  # 得到最小外接矩阵的（中心（x，y）（宽高），旋转角度）
                area = cv2.contourArea(cnt)  # 获得物体面积
                if (area < 500):  # 小于2000就跳过
                    continue
                box = cv2.boxPoints(rect)  # 获取最小外界矩形的4个顶点坐标
                box = np.int0(box)  # 取整
                cx = int(rect[0][0])  # 获取中心点x坐标
                cy = int(rect[0][1])  # 获取中心点y坐标
                self.color_block.color = "blue"
                print('blue:', cx, cy)
                # 获取旋转角度
                theta = cv2.minAreaRect(cnt)[2]
                if abs(theta) <= 45:
                    print('图片旋转角度为%s.' % theta)
                frame = cv2.drawContours(frame, [box], 0, (255, 0, 0), 3)  # 画旋转方框
                frame = cv2.line(frame, (cx + 5, cy), (cx - 5, cy), (255, 0, 0), 2)
                frame = cv2.line(frame, (cx, cy + 5), (cx, cy - 5), (255, 0, 0), 2)
                # cv2.rectangle(frame, (x4, y4), (x4 + w4, y4 + h4), (0, 255, 255), 2)
                cv2.putText(frame, "blue", (x, y - 5), font, 0.5, (255, 0, 0), 2)

            for cnt in cnts4:  # purple
                (x, y, w, h) = cv2.boundingRect(cnt)
                rect = cv2.minAreaRect(cnt)  # 得到最小外接矩阵的（中心（x，y）（宽高），旋转角度）
                area = cv2.contourArea(cnt)  # 获得物体面积
                if (area < 500):  # 小于2000就跳过
                    continue
                box = cv2.boxPoints(rect)  # 获取最小外界矩形的4个顶点坐标
                box = np.int0(box)  # 取整
                cx = int(rect[0][0])  # 获取中心点x坐标
                cy = int(rect[0][1])  # 获取中心点y坐标

                self.color_block.color = "purple"
                print('purple:', cx, cy)
                # 获取旋转角度
                theta = cv2.minAreaRect(cnt)[2]
                if abs(theta) <= 45:
                    print('图片旋转角度为%s.' % theta)
                frame = cv2.drawContours(frame, [box], 0, (240, 32, 160), 3)  # 画旋转方框
                frame = cv2.line(frame, (cx + 5, cy), (cx - 5, cy), (240, 32, 160), 2)  # 绘制中心点坐标
                frame = cv2.line(frame, (cx, cy + 5), (cx, cy - 5), (240, 32, 160), 2)
                # cv2.rectangle(frame, (x5, y5), (x5 + w5, y5 + h5), (0, 255, 255), 2)
                cv2.putText(frame, "purple", (x, y - 5), font, 0.5, (240, 32, 160), 2)

            for cnt in cnts5:  # yellow
                (x, y, w, h) = cv2.boundingRect(cnt)
                rect = cv2.minAreaRect(cnt)  # 得到最小外接矩阵的（中心（x，y）（宽高），旋转角度）
                area = cv2.contourArea(cnt)  # 获得物体面积
                if (area < 500):  # 小于2000就跳过
                    continue
                box = cv2.boxPoints(rect)  # 获取最小外界矩形的4个顶点坐标
                box = np.int0(box)  # 取整
                cx = int(rect[0][0])  # 获取中心点x坐标
                cy = int(rect[0][1])  # 获取中心点y坐标
                self.color_block.color = "yellow"
                print('yellow:', cx, cy)

                # 获取旋转角度
                theta = cv2.minAreaRect(cnt)[2]
                if abs(theta) <= 45:
                    print('图片旋转角度为%s.' % theta)

                frame = cv2.drawContours(frame, [box], 0, (0, 255, 255), 3)  # 画旋转方框
                frame = cv2.line(frame, (cx + 5, cy), (cx - 5, cy), (0, 255, 255), 2)
                frame = cv2.line(frame, (cx, cy + 5), (cx, cy - 5), (0, 255, 255), 2)
                # cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
                cv2.putText(frame, "yellow", (x, y - 5), font, 0.5, (0, 255, 255), 2)

            # for cnt in cnts6:  # orange
            #     (x, y, w, h) = cv2.boundingRect(cnt)
            #     rect = cv2.minAreaRect(cnt)  # 得到最小外接矩阵的（中心（x，y）（宽高），旋转角度）
            #     area = cv2.contourArea(cnt)  # 获得物体面积
            #     if (area < 500):  # 小于2000就跳过
            #         continue
            #     box = cv2.boxPoints(rect)  # 获取最小外界矩形的4个顶点坐标
            #     box = np.int0(box)  # 取整
            #     cx = int(rect[0][0])  # 获取中心点x坐标
            #     cy = int(rect[0][1])  # 获取中心点y坐标
            #     self.color_block.color = "yellow"
            #     print('orange:', cx, cy)

                #     # 获取旋转角度
                # theta = cv2.minAreaRect(cnt)[2]
                # if abs(theta) <= 45:
                #     print('图片旋转角度为%s.' % theta)
                frame = cv2.drawContours(frame, [box], 0, (0, 165, 255), 3)  # 画旋转方框
                frame = cv2.line(frame, (cx + 5, cy), (cx - 5, cy), (0, 165, 255), 2)
                frame = cv2.line(frame, (cx, cy + 5), (cx, cy - 5), (0, 165, 255), 2)
                # cv2.rectangle(frame, (x4, y4), (x4 + w4, y4 + h4), (0, 255, 255), 2)
                cv2.putText(frame, "orange", (x, y - 5), font, 0.5, (0, 165, 255), 2)
        else:
            self.color_block.color = "None"
            print("未检测到物体")
            pass
#========-----------------------------------------------
        # point_msg = Point()
        # point_msg.x = float(cy) / 1000-0.009
        # point_msg.y = float(cx) / 1000-0.4
        # point_msg.z = 0.16

        # float64_msg = Float64()
        # float64_msg.data=theta
        center = Point()
      
        # center.x = float(cy) / 1000-0.011
        # center.y = float(cx) / 1000-0.405
        #center.y = float(cx) / 1000-0.405



        center.x = -(float(cx) / 1000+1)
        center.y = float(cy) / 1000+1
        center.z = 0.0


        self.color_block.center = center
        self.color_block.angle=theta
        
#-----------------------------------------------========

        # 发布消息
        # self.point_pub.publish(point_msg)
        # rospy.loginfo("Publsh point command[%0.2fm, %0.2fm,%0.2fm]", 
        #                             point_msg.x, point_msg.y, point_msg.z)
        # # 发布消息
        # self.float64_pub.publish(float64_msg)

        self.center_pub.publish(self.color_block)
        rospy.loginfo("Publsh point command[%0.2fm, %0.2fm,%0.2fm]", 
                                    self.color_block.center.x, 
                                    self.color_block.center.y, 
                                    self.color_block.center.z)

        rospy.loginfo("Publsh angle command[%0.2fdu]", self.color_block.angle)


        
 #---------------------------------------------------------------------------------------------------------
       
        # 获取图像大小
        height, width, channels = frame.shape

        frame1=frame0.copy()
        frame1[0:350,0:350]=frame
        # 添加水印，显示图像大小
        watermark = "Size: {}x{}".format(width, height)
        cv2.putText(frame, watermark, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        
        rectangles = [
            {"points": [(130, 160), (319, 160), (319, 280), (130, 280)], "text": "pick"},
            {"points": [(370, 285), (565, 285), (570, 415), (370, 415)], "text": "place"}
        ]

        # 在图像上绘制两个矩形框和文字
        for rectangle in rectangles:
            points = rectangle["points"]
            text = rectangle["text"]

            for point in points:
                cv2.circle(frame1, point, 5, (0, 255, 0), -1)  # 画绿色圆点
            cv2.polylines(frame1, [np.array(points)], isClosed=True, color=(0, 255, 0), thickness=2)  # 画绿色矩形框

            # 在矩形框左上角打印文字
            cv2.putText(frame1, text, (points[0][0], points[0][1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)


        # 定义要截取的区域的坐标
        pick_x,  pick_y,   pick_w,   pick_h = 130, 140, 200, 150  #  抓取的坐标和大小
        place_x, place_y,  place_w,  place_h = 360, 265, 210, 160  # 放置的坐标和大小

        # 使用图像切片截取区域
        cropped_pick = frame1[pick_y:pick_y+pick_h, pick_x:pick_x+pick_w]

        cropped_place = frame1[place_y:place_y+place_h, place_x:place_x+place_w]
        frame0[0:350,0:350]=frame

        # 绘制x轴
        cv2.line(frame0, (0, frame0.shape[0] // 2), (frame0.shape[1], frame0.shape[0] // 2), (0, 0, 255), 2)
        # 绘制y轴
        cv2.line(frame0, (frame0.shape[1] // 2, 0), (frame0.shape[1] // 2, frame0.shape[0]), (0, 255, 0), 2)
       
        # 显示Opencv格式的图像
        # cv2.namedWindow("Pick and Place",cv2.WINDOW_FREERATIO)
        # cv2.imshow('Pick and Place', frame1)
  

        # # 显示Opencv格式的图像    显示截取  抓取的图像
        # cv2.namedWindow("Cropped Pick Image",cv2.WINDOW_FREERATIO)
        # cv2.imshow('Cropped Pick Image', cropped_pick)

        #         # 显示Opencv格式的图像    显示截取  抓取的图像
        # cv2.namedWindow("Cropped Place Image",cv2.WINDOW_FREERATIO)
        # cv2.imshow('Cropped Place Image', cropped_place)
 #-----------------------------------------------------------------------------------------------

        cv2.namedWindow("校准",cv2.WINDOW_FREERATIO)
        cv2.imshow("校准", frame0)
        cv2.waitKey(3)




        cv2.namedWindow("处理_方块图像",cv2.WINDOW_FREERATIO)
        cv2.imshow("处理_方块图像", frame)
        cv2.waitKey(3)
        # cv2.moveWindow("Pick and Place", 0, 0)
        # cv2.moveWindow("Cropped Pick Image", 500, 0)
        # cv2.moveWindow("Cropped Place Image",  1000, 0)
        # cv2.moveWindow("校准", 3 * frame1.shape[1], 0)


        cv2.imwrite("./test.jpg", frame)

        # 再将opencv格式额数据转换成ros image格式的数据发布
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
        except CvBridgeError as e:
            print (e)

if __name__ == '__main__':
    try:
        # 初始化ros节点
        rospy.init_node("cv_bridge_test")
        rospy.loginfo("Starting cv_bridge_test node")
        image_converter()
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down cv_bridge_test node.")
        cv2.destroyAllWindows()

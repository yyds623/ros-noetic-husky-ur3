#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math
from std_msgs.msg import Header
import time

class NavigationMetricsCalculator:
    def __init__(self):
        rospy.init_node('navigation_metrics_calculator', anonymous=True, log_level=rospy.INFO)
        self.path_length = 0.0
        self.last_pose = None
        self.global_plan_time = None
        self.local_plan_time = None

        # 订阅全局规划路径
        self.global_plan_sub = rospy.Subscriber('/move_base/RRTstarPlannerROS/plan', Path, self.global_plan_callback)
        
        # 订阅局部规划路径
        self.local_plan_sub = rospy.Subscriber('/move_base/DWAPlannerROS/local_plan', Path, self.local_plan_callback)
        
        # 定时发布路径长度信息
        self.rate = rospy.Rate(10)  # 10 Hz

    def global_plan_callback(self, path_msg):
        # 记录全局规划路径的时间戳
        self.global_plan_time = rospy.Time.now()
        rospy.loginfo("全局规划路径接收时间：%f", self.global_plan_time.to_sec())
        self.calculate_and_print_metrics(path_msg)

    def local_plan_callback(self, path_msg):
        # 记录局部规划路径的时间戳
        self.local_plan_time = rospy.Time.now()
        rospy.loginfo("局部规划路径接收时间：%f", self.local_plan_time.to_sec())
        self.calculate_and_print_metrics(path_msg)

    def calculate_and_print_metrics(self, path_msg):
        # 计算并打印路径长度和规划时间差
        for pose_stamped in path_msg.poses:
            if self.last_pose is not None:
                dx = pose_stamped.pose.position.x - self.last_pose.pose.position.x
                dy = pose_stamped.pose.position.y - self.last_pose.pose.position.y
                dz = pose_stamped.pose.position.z - self.last_pose.pose.position.z
                segment_length = math.sqrt(dx**2 + dy**2 + dz**2)
                self.path_length += segment_length
            self.last_pose = pose_stamped
        rospy.loginfo("路径长度：%.2f 米", self.path_length)
        
        # 计算时间差
        if self.global_plan_time is not None and self.local_plan_time is not None:
            time_diff = self.local_plan_time - self.global_plan_time
            rospy.loginfo("全局规划与局部规划时间差：%f 秒", time_diff.to_sec())

    def run(self):
        # 循环发布路径长度信息
        while not rospy.is_shutdown():
            rospy.loginfo("当前路径长度：%.2f 米", self.path_length)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        nav_metrics_calculator = NavigationMetricsCalculator()
        nav_metrics_calculator.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("程序中断")
    except Exception as e:
        rospy.logerr(f"发生错误：{e}")
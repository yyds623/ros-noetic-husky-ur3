#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
from geometry_msgs.msg import PoseStamped
import moveit_commander

# 初始化ROS节点
rospy.init_node('pose_subscriber')

# 创建TransformListener
listener = tf.TransformListener()

def pose_callback(data):
    # 打印消息信息
    print("Header Frame ID:", data.header.frame_id)
    print("Position: x={}, y={}, z={}".format(data.pose.position.x, data.pose.position.y, data.pose.position.z))
    print("Orientation: x={}, y={}, z={}, w={}".format(data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w))
    print("")

    try:
        # 转换姿态到指定坐标系
        # (trans, rot) = listener.lookupTransform("base_link", "camera_link", rospy.Time(0))
        # print("Transformed Pose to base_link:")
        # print("Translation 平移: x={}, y={}, z={}".format(trans[0], trans[1], trans[2]))
        # print("Rotation 旋转: x={}, y={}, z={}, w={}".format(rot[0], rot[1], rot[2], rot[3]))

        # 初始化机械臂控制
        arm = moveit_commander.MoveGroupCommander('ur3_manipulator')
        end_effector_link = arm.get_end_effector_link()
        reference_frame = 'ee_link'
        arm.set_pose_reference_frame(reference_frame)

        arm.set_planner_id("RRTstar")  # 尝试使用不同的规划器

        arm.allow_replanning(True)
        arm.set_planning_time(5.0)

        arm.set_goal_position_tolerance(0.01)
        arm.set_goal_orientation_tolerance(0.01)

        # 控制机械臂回到初始化位置
        arm.set_named_target('front_view')
        arm.go()
        rospy.sleep(2)

        # 设置目标位姿
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now()
        target_pose.pose = data.pose
        
        print("target_pose.pose.position__AAA",target_pose.pose)

        # 设置机械臂初始状态
        arm.set_start_state_to_current_state()

        # 设置机械臂终端运动目标位姿
        arm.set_pose_target(target_pose, end_effector_link)

        # 规划运动路径
        plan_success,traj,planning_time,error_code = arm.plan()

        # traj = arm.plan()

        # 执行运动
        arm.execute(traj)
        rospy.sleep(1)

        # 控制机械臂回到初始化位置
        arm.set_named_target('front_view')
        arm.go()

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print("Failed to transform pose.")

if __name__ == '__main__':
    # 订阅话题
    rospy.Subscriber('/aruco_single/pose', PoseStamped, pose_callback,queue_size=1)

    # 等待ROS节点初始化完成
    rospy.wait_for_message('/aruco_single/pose1', PoseStamped)

    # 启动 ROS 节点
    rospy.spin()

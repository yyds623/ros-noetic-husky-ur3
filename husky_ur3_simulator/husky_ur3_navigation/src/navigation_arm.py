#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import moveit_commander
import geometry_msgs.msg
import tf
import sys
import time
from copy import deepcopy


def plan_and_execute_cartesian_path(arm, waypoints, eef_step=0.01, jump_threshold=0.0, maxtries=100):


    waypoints = deepcopy(waypoints)


    fraction = 0.0   # 路径规划覆盖率
    attempts = 0     # 已经尝试规划次数
# maxtries = 100   #最大尝试规划次数

    # 设置机器臂当前的状态作为运动初始状态
    arm.set_start_state_to_current_state()

    # 尝试规划一条笛卡尔空间下的路径，依次通过所有路点
    while fraction < 1.0 and attempts < maxtries:
        # 规划路径，fraction返回1代表规划成功
        (plan, fraction) = arm.compute_cartesian_path (
                                waypoints,   # waypoint poses
                                eef_step,    # eef_step
                                jump_threshold,  # jump_threshold
                                True)        # avoid_collisions           
        # 尝试次数累加
        attempts += 1  
        # 打印运动规划进程
        if attempts % 10 == 0:
            rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
    # 如果路径规划成功（覆盖率100%）,则开始控制机械臂运动
    if 1 >= fraction >= 0.02:
        rospy.loginfo("覆盖率为" + str(fraction) + " Path computed SuCcessfully. 开始执行.")
        arm.execute(plan, wait=True)
        rospy.loginfo("Path execution complete.")
        return True
    # 如果路径规划失败，则打印失败信息
    else:
        rospy.loginfo("在经过尝试最大" + str(maxtries) + "次后路径规划失败,路径覆盖占比仅为 " + str(fraction))
        return False



class MoveVoguiBase(object):
    def __init__(self):
        # 初始化ROS通信节点
        rospy.init_node('move_base_test_client', anonymous=True, log_level=rospy.INFO)
        rospy.loginfo("Waiting for move_base action server...")
        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base action server.")
        
        # 初始化moveit_commander
        self.moveit = moveit_commander.RobotCommander()
        self.arm = moveit_commander.MoveGroupCommander('ur3_manipulator')

        # 设置POI目标点
        self.poi_goals = self.set_poi_goals()

        # 初始化时间统计字典
        self.times = {poi: 0.0 for poi in self.poi_goals.keys()}






    def set_poi_goals(self):
        # 定义POI目标点的具体位置和方向
        # 注意：位置和方向数据需要根据实际环境进行设置
        poi_goals = {
            "home": self.create_poi_goal(3.143111, -2.69011, 0.1322, 0.0, 0.0, -0.0224237328117, 0.999748556491),
            "place_ZhongTu": self.create_poi_goal(8.870, -2.1388, 0.2515, 0.0, 0.0, 0.69822, 0.715),
            "place": self.create_poi_goal(9.0608, 0.4655, 0.0, 0.0, 0.0, 0.73855, 0.67412),
            "zhuaqu": self.create_poi_goal(19.01, 0.737, 0.2515, 0.0, 0.0, 0.69822, 0.715)
        }
        return poi_goals

    def create_poi_goal(self, x, y, z, orientation_x, orientation_y, orientation_z, orientation_w):
        # 创建MoveBaseGoal对象
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = z
        goal.target_pose.pose.orientation.x = orientation_x
        goal.target_pose.pose.orientation.y = orientation_y
        goal.target_pose.pose.orientation.z = orientation_z
        goal.target_pose.pose.orientation.w = orientation_w
        return goal

    def control_arm(self):
        # 移动机械臂到front_view位置
        # self.arm.set_named_target('front_view')
        self.arm.set_named_target('home')


        # 定义目标位置和方向的Pose对象
        # waypoints = []
        # pose = geometry_msgs.msg.Pose()
        # pose.position.x = 0.4573668301851815
        # pose.position.y = 0.11241492910721473
        # pose.position.z = 0.575681496524733
        # pose.orientation.x = 1.6014374767546583e-06
        # pose.orientation.y = -0.9997998410520037
        # pose.orientation.z = -0.0003976940927680866
        # pose.orientation.w = 0.020002991507078592
        # waypoints.append(pose)  # 将Pose对象添加到waypoints列表中

        # # 使用deepcopy来避免对原始waypoints的修改
        # waypoints = deepcopy(waypoints)
        # arm = moveit_commander.MoveGroupCommander('ur3_manipulator')

        # plan_and_execute_cartesian_path(arm, waypoints, eef_step=0.01, jump_threshold=0.0, maxtries=100) # type: ignore
        self.arm.go()
        rospy.loginfo("Arm moved to home1 position.")

    def move_to_poi(self, name_poi):
        # 移动到指定的POI
        start_time = time.time()  # 记录开始时间
        self.client.send_goal(self.poi_goals[name_poi])
        self.client.wait_for_result()
        end_time = time.time()  # 记录结束时间
        self.times[name_poi] += end_time - start_time  # 更新时间统计
        rospy.loginfo(f"Time taken to reach {name_poi}: {end_time - start_time:.2f} seconds.")

    def start_ask_loop(self):
        # 开始询问用户想要移动到哪个POI
        selection = "START"
        while selection != "Q" and not rospy.is_shutdown():
            rospy.loginfo("#############################")
            rospy.loginfo("Select Point of Interest")
            rospy.loginfo("[0] HOME POI")
            rospy.loginfo("[1] PLACE_ZHONGTU POI")
            rospy.loginfo("[2] PLACE POI")
            rospy.loginfo("[3] ZHUAQU POI")
            rospy.loginfo("[Q] Quit")
            selection = input("Enter your choice: ")
            try:
                selection = int(selection)
                if selection in [0, 1, 2, 3]:
                    self.move_to_poi(list(self.poi_goals.keys())[selection])
                elif selection == ord("Q"):
                    break
                else:
                    rospy.logerr("Invalid selection. Please choose a valid POI or 'Q' to quit.")
            except ValueError:
                rospy.logerr("Invalid input. Please enter a number.")

    def display_times(self):
        # 打印每个POI的运动时间
        rospy.loginfo("\nTotal movement times for each POI:")
        for poi, time in self.times.items():
            rospy.loginfo(f"{poi}: {time:.2f} seconds")

if __name__ == '__main__':
    try:
        mv_obj = MoveVoguiBase()

        # 测量机械臂移动到front_view的时间
        start_time = time.time()
        mv_obj.control_arm()
        front_view_time = time.time() - start_time
        rospy.loginfo(f"Time taken to move arm to home1: {front_view_time:.2f} seconds")

        # 开始询问循环，允许用户选择POI
        mv_obj.start_ask_loop()

        # 打印每个POI的运动时间
        mv_obj.display_times()

    except rospy.ROSInterruptException:
        rospy.loginfo("Program interrupted before completion")
    except Exception as e:
        rospy.logerr(f"An error occurred: {e}")
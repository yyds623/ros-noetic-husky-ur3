#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult
import moveit_commander
import sys  # 添加这行
import time
import std_msgs.msg
from std_msgs.msg import Float64

from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
import math

class MoveVoguiBase(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    
        # 定义各个POI点
        ## HOME ##  出发点0
        home_poi = MoveBaseGoal()
        home_poi.target_pose.header.frame_id = "map"
        home_poi.target_pose.header.stamp = rospy.Time.now()
        home_poi.target_pose.pose.position.x = 3.143111
        home_poi.target_pose.pose.position.y = -2.69011
        home_poi.target_pose.pose.position.z = 0.1322
        home_poi.target_pose.pose.orientation.x = 0.0
        home_poi.target_pose.pose.orientation.y = 0.0
        home_poi.target_pose.pose.orientation.z = -0.0224237328117
        home_poi.target_pose.pose.orientation.w = 0.999748556491

        ## PLACE ##   中途点
        place_poi = MoveBaseGoal()
        place_poi.target_pose.header.frame_id = "map"
        place_poi.target_pose.header.stamp = rospy.Time.now()
        place_poi.target_pose.pose.position.x = 8.870
        place_poi.target_pose.pose.position.y = -2.1388
        place_poi.target_pose.pose.position.z = 0.2515
        place_poi.target_pose.pose.orientation.x = 0.0
        place_poi.target_pose.pose.orientation.y = 0.0
        place_poi.target_pose.pose.orientation.z = 0.69822
        place_poi.target_pose.pose.orientation.w = 0.715

        ## PLACE ##   抓取点
        place_poi1 = MoveBaseGoal()
        place_poi1.target_pose.header.frame_id = "map"
        place_poi1.target_pose.header.stamp = rospy.Time.now()
        place_poi1.target_pose.pose.position.x = 8.7893
        place_poi1.target_pose.pose.position.y = 0.16051
        place_poi1.target_pose.pose.position.z = 0.2515
        place_poi1.target_pose.pose.orientation.x = 0.0
        place_poi1.target_pose.pose.orientation.y = 0.0
        place_poi1.target_pose.pose.orientation.z = 0.69822
        place_poi1.target_pose.pose.orientation.w = 0.715

        ## PLACE ##   最终点
        place_poi2 = MoveBaseGoal()
        place_poi2.target_pose.header.frame_id = "map"
        place_poi2.target_pose.header.stamp = rospy.Time.now()
        place_poi2.target_pose.pose.position.x = 19.01
        place_poi2.target_pose.pose.position.y = 0.737
        place_poi2.target_pose.pose.position.z = 0.2515
        place_poi2.target_pose.pose.orientation.x = 0.0
        place_poi2.target_pose.pose.orientation.y = 0.0
        place_poi2.target_pose.pose.orientation.z = 0.69822
        place_poi2.target_pose.pose.orientation.w = 0.715

        self.keys_list = ["home", "ZhongTu", "grad", "place"]
        self.values_list = [home_poi, place_poi, place_poi1, place_poi2]
        zip_iterator = zip(self.keys_list, self.values_list)
        self.poi_dict = dict(zip_iterator)

        rospy.loginfo("Waiting for move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("Move_base action server is ready!")

    def start_ask_loop(self):
        selection = "START"
        while selection != "Q":
            rospy.loginfo("#############################")
            rospy.loginfo("Select Point of Interest")
            rospy.loginfo("[0] HOME POI")
            rospy.loginfo("[1] PLACE_Pick_ZhongTU POI")
            rospy.loginfo("[2] Pick POI")
            rospy.loginfo("[3] PLACE POI")
            rospy.loginfo("[Q] Quit")
            selection = input(">> ")
            self.select_action(selection)

    def select_action(self, selection):
        try:
            int_selection = int(selection)
            if int_selection < len(self.keys_list):
                poi_name = self.keys_list[int_selection]
                self.move_to_poi(poi_name)
            else:
                rospy.logerr("Not a valid int: " + str(int_selection))
        except ValueError:
            rospy.logerr("Not an int: " + str(selection))

    def move_arm_to_joints(self, joints):
        arm = moveit_commander.MoveGroupCommander('ur3_manipulator')
        current_position = arm.get_current_pose().pose
        if current_position != joints:
            arm.set_joint_value_target(joints)
            arm.go(wait=True)
            rospy.loginfo("机械臂移动到指定关节角度。")

    def move_arm_to_front_view(self):
        arm = moveit_commander.MoveGroupCommander('ur3_manipulator')
        arm.set_named_target('front_view')
        arm.go(wait=True)
        rospy.loginfo("机械臂移动到 front_view 点位。")

    def gripper_state(self, state):
        pub = rospy.Publisher("/rh_p12_rn_position/command", std_msgs.msg.Float64, queue_size=2)
        rospy.sleep(1)
        msg_gripper = std_msgs.msg.Float64()
        msg_gripper.data = state
        pub.publish(msg_gripper)
        if state == 0:
            rospy.loginfo("夹爪状态: 张开")
        elif 0 < state <= 1:
            rospy.loginfo("夹爪状态: 闭合")
        rospy.sleep(1)

    def move_to_poi(self, name_poi):
        rospy.logwarn("Selected POI = " + str(name_poi))
        if name_poi in self.poi_dict:
            goal_poi = self.poi_dict[name_poi]
            rospy.loginfo("Sending Goal for POI: " + str(name_poi))
            self.client.send_goal(goal_poi)
            rospy.loginfo("Goal for POI: " + str(name_poi) + "... SENT")
            result = self.client.wait_for_result()
            if result:
                rospy.loginfo("Goal successful! Result: " + str(result))
                if name_poi in ["place", "grad"]:
                    joints = [1.5699816879014499, -1.0168987916553878, 0.5408123722620539, -1.5700797802710849, -1.5699665572582333, 3.1414221899719923]
                    self.move_arm_to_joints(joints)
                    if name_poi == "grad":
                        rospy.sleep(1)
                        self.gripper_state(0.65)
                        self.move_arm_to_front_view()

                    else:
                        self.gripper_state(0.0)
                        self.move_arm_to_front_view()

            else:
                rospy.logerr("Goal failed! Result: " + str(result))
        else:
            rospy.logerr("Point of Interest " + str(name_poi) + " is not in the Database.")

if __name__ == '__main__':
    rospy.init_node('move_base_client_gripper_controller')
    mv_obj = MoveVoguiBase()
    current_position = None
    if current_position != 'front_view':
        start_time = time.time()
        mv_obj.move_arm_to_front_view()
        current_position = 'front_view'
        end_time = time.time()
        time_taken = end_time - start_time
        rospy.loginfo("从启动到机械臂移动到 front_view 的时间为：%f 秒" % time_taken)
    mv_obj.start_ask_loop()

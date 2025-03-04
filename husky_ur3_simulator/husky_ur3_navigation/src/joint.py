#!/usr/bin/env python

import rospy, sys
import moveit_commander
import std_msgs.msg
from std_msgs.msg import Float64
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
import math
from color_detection.msg import ColorBlock

def gripper_state(state):
    rospy.init_node('gripper_controller')  # 初始化ROS节点

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

arm = moveit_commander.MoveGroupCommander('ur3_manipulator')
# rospy.loginfo("point: x:%0.2fm, y:%0.2fm, z:%0.2fm", msg.center.x, msg.center.y, msg.center.z)
reference_frame = 'ur3_base_link'
arm.set_pose_reference_frame(reference_frame)
end_effector_link = arm.get_end_effector_link()
arm.allow_replanning(True)
arm.set_planner_id("RRTConnectkConfigDefault")
arm.set_planning_time(5.0)
arm.set_goal_position_tolerance(0.001)
arm.set_goal_orientation_tolerance(0.001)
arm.set_max_acceleration_scaling_factor(0.5)
arm.set_max_velocity_scaling_factor(0.5)


# arm.set_named_target('home')
# arm.go()
gripper_state(0.0)
arm.set_named_target('front_view')
arm.go()
rospy.sleep(1)

# orientation1 = quaternion_from_euler(math.pi/2, -(math.pi/2), 0)
# target_pose = PoseStamped()
# target_pose.header.frame_id = reference_frame
# target_pose.header.stamp = rospy.Time.now()

# target_pose.pose.position.x = msg.center.x
# target_pose.pose.position.y = msg.center.y
# target_pose.pose.position.z = msg.center.z
# target_pose.pose.orientation.x = orientation1[0]
# target_pose.pose.orientation.y = orientation1[1]
# target_pose.pose.orientation.z = orientation1[2]
# target_pose.pose.orientation.w = orientation1[3]
# arm.set_start_state_to_current_state()
# arm.set_pose_target(target_pose, end_effector_link)

# print("target_pose.pose.position__",target_pose.pose)
# plan_success,traj,planning_time,error_code = arm.plan()

#traj = arm.plan()

joints = [1.5699816879014499, -1.0168987916553878, 0.5408123722620539, -1.5700797802710849, -1.5699665572582333, 3.1414221899719923]

arm.set_joint_value_target(joints)  #设置关节值作为目标值

plan_success,traj,planning_time,error_code = arm.plan()

arm.execute(traj)
rospy.sleep(1)
rospy.loginfo("移动到桌面")
gripper_state(0.65)

# joints1 = [1.569885571557843, -0.7967611736573028, -0.00013855256919548253, 
#   -1.2242519692067821, -1.570107603833356, 3.141463543209932]



# arm.set_joint_value_target(joints1)  #设置关节值作为目标值

# plan_success,traj,planning_time,error_code = arm.plan()

# arm.execute(traj)
# rospy.sleep(1)
# gripper_state(0.65)

# rospy.loginfo("移动到桌面")


moveit_commander.roscpp_shutdown()
moveit_commander.os._exit(0)

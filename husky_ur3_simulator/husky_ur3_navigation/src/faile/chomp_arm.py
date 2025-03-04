#!/usr/bin/env python

import rospy
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose
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
    rospy.sleep(0.5)

def main():
    # 初始化ROS通信
    rospy.init_node('chomp_cartesian_planning')

    # 创建MoveGroupCommander对象
    arm = MoveGroupCommander('ur3_manipulator')
    end_effector_link = arm.get_end_effector_link()
    # 设置目标姿态
    arm.set_named_target('front_view')



 # 获取当前位姿数据最为机械臂运动的起始位姿
    start_pose = arm.get_current_pose(end_effector_link).pose 

    waypoints = []
    wpose = deepcopy(start_pose)#拷贝对象
    wpose.position.z += 0.1
    waypoints.append(deepcopy(wpose)) #将wpose偏移Z轴后的点加入路点列表  2

 # 设置CHOMP规划器参数
    arm.set_planning_time(10.0)  # 规划时间
    arm.set_planner_id("CHOMPPlanner")  # 使用CHOMP算法
    arm.set_max_velocity_scaling_factor(0.5)  # 最大速度因子
    arm.set_max_acceleration_scaling_factor(0.5)  # 最大加速度因子

    plan_and_execute_cartesian_path(arm, waypoints, eef_step=0.01, jump_threshold=0.0, maxtries=100)







    # 等待用户输入后退出
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
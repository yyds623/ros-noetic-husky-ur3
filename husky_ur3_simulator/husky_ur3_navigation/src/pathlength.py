import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math

class PathLengthCalculator:
    def __init__(self):
        rospy.init_node('path_length_calculator')
        self.path_length = 0.0
        self.last_pose = None
        rospy.Subscriber('/move_base/DWAPlannerROS/global_plan', Path, self.path_callback)

    def path_callback(self, path_msg):
        for pose_stamped in path_msg.poses:
            if self.last_pose is not None:
                dx = pose_stamped.pose.position.x - self.last_pose.pose.position.x
                dy = pose_stamped.pose.position.y - self.last_pose.pose.position.y
                dz = pose_stamped.pose.position.z - self.last_pose.pose.position.z
                segment_length = math.sqrt(dx**2 + dy**2 + dz**2)
                self.path_length += segment_length
            self.last_pose = pose_stamped

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            rospy.loginfo("Path length: %.2f meters" % self.path_length)
            rate.sleep()

if __name__ == '__main__':
    path_calculator = PathLengthCalculator()
    path_calculator.run()




















# import rospy
# from nav_msgs.msg import Path
# from std_msgs.msg import Header
# import time

# class NavigationLogger(object):
#     def __init__(self):
#         self.global_plan_sub = rospy.Subscriber('/move_base/RRTstarPlannerROS/plan', Path, self.global_plan_callback)
#         self.local_plan_sub = rospy.Subscriber('/move_base/DWAPlannerROS/local_plan', Path, self.local_plan_callback)
#         self.global_plan_time = None
#         self.local_plan_time = None
    
#     def global_plan_callback(self, msg):
#         # 记录全局规划路径的时间戳
#         self.global_plan_time = rospy.Time.now()
#         rospy.loginfo("Global plan received at time: %f", self.global_plan_time.to_sec())
#         self.calculate_time_difference()

#     def local_plan_callback(self, msg):
#         # 记录局部规划路径的时间戳
#         self.local_plan_time = rospy.Time.now()
#         rospy.loginfo("Local plan received at time: %f", self.local_plan_time.to_sec())
#         self.calculate_time_difference()

#     def calculate_time_difference(self):
#         # 计算时间差
#         if self.global_plan_time is not None and self.local_plan_time is not None:
#             time_diff = self.local_plan_time - self.global_plan_time
#             rospy.loginfo("Time difference between global and local plan: %f seconds", time_diff.to_sec())

# if __name__ == '__main__':
#     rospy.init_node('navigation_logger')
#     navigation_logger = NavigationLogger()
#     rospy.spin()



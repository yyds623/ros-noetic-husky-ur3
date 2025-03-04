#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def publish_velocity(linear_x, angular_z, duration):
    rospy.init_node('velocity_publisher', anonymous=True)
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=100)
    rate = rospy.Rate(10) # 发布频率为10Hz

    start_time = rospy.Time.now()
    while (rospy.Time.now() - start_time).to_sec() < duration and not rospy.is_shutdown():
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        velocity_publisher.publish(twist)
        rate.sleep()

    # 发布时间到期后停止机器人
    twist = Twist()
    velocity_publisher.publish(twist)

if __name__ == '__main__':
    try:
        # 设置线速度、角速度和持续时间
        linear_x = 0.5  # 设置线速度为0.5 m/s
        angular_z = 0.0  # 设置角速度为0.2 rad/s
        duration = 100  # 设置持续时间为10秒
        publish_velocity(linear_x, angular_z, duration)
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from time import time
import math
import tf_conversions

class Core0Odom:
    def __init__(self):
        rospy.init_node('Core0_odom_odom')
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(0.05), self.update)
        # self.static_target_tf = TransformBroadcaster()
        self.static_target_tf = StaticTransformBroadcaster()
        self.cmd_vel_sub = rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_cb)

        self.dx = 0
        self.dy = 0
        self.dr = 0
        self.r = 0

        self.x = 0
        self.y = 0
        self.heading = 0
        self.dt = 0
        self.timestamp = rospy.Time.now()

        self.odom_msg_ = Odometry()
        self.odom_msg_.header.frame_id = "odom"
        self.odom_msg_.child_frame_id = "base_link"
        self.odom_msg_.pose.pose.orientation.x = 0.0
        self.odom_msg_.pose.pose.orientation.y = 0.0
        self.odom_msg_.pose.pose.orientation.z = 0.0
        self.odom_msg_.pose.pose.orientation.w = 1.0

        ############### For tf broadcaster ####################3
        self.br_ = TransformBroadcaster()
        self.transform_stamped_ = TransformStamped()
        self.transform_stamped_.header.frame_id = "odom"
        self.transform_stamped_.child_frame_id = "base_link"

    def cmd_vel_cb(self, msg):
        self.dx = msg.linear.x
        self.dy = msg.linear.y
        self.dr = msg.angular.z
        print(self.dx)
        print(self.dr)
        now = rospy.Time.now()
        print(now)
        print("second", now.to_sec(),"sec")
        self.dt = (now.to_sec() - self.timestamp.to_sec())
        print("Different time",self.dt)

        if self.dt > 0.0001:
            self.integrate_exact(self.dx * self.dt, self.dr * self.dt)
            print("multiplying linear",self.dx * self.dt)
            print("multiplying anguler",self.dr * self.dt)
        self.timestamp = now

    def integrate_runge_kutta2(self, linear, angular):
        direction = self.heading + angular * 0.5

        # Runge-Kutta 2nd order integration:
        self.x += linear * math.cos(direction)
        self.y += linear * math.sin(direction)
        self.heading += angular

    def integrate_exact(self, linear, angular):
        if abs(angular) < 1e-6:
            self.integrate_runge_kutta2(linear, angular)
        else:
            # Exact integration (should solve problems when angular is zero):
            heading_old = self.heading
            r = linear / angular
            self.heading += angular
            self.x += r * (math.sin(self.heading) - math.sin(heading_old))
            self.y += -r * (math.cos(self.heading) - math.cos(heading_old))
    
    def update(self,event):
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, self.heading)
        # self.odom_msg_.header.stamp = rospy.Time.now()
        self.odom_msg_.pose.pose.position.x = self.x
        self.odom_msg_.pose.pose.position.y = self.y
        self.odom_msg_.pose.pose.orientation.x = q[0]
        self.odom_msg_.pose.pose.orientation.y = q[1]
        self.odom_msg_.pose.pose.orientation.z = q[2]
        self.odom_msg_.pose.pose.orientation.w = q[3]

        self.odom_msg_.pose.covariance[0] = 0.001
        self.odom_msg_.pose.covariance[7] = 0.001
        self.odom_msg_.pose.covariance[14] = 1000000.0
        self.odom_msg_.pose.covariance[21] = 1000000.0
        self.odom_msg_.pose.covariance[28] = 1000000.0
        self.odom_msg_.pose.covariance[35] = 0.03

        if self.dt > 0.0:
            self.odom_msg_.twist.twist.linear.x = self.x / self.dt
            # print("linear odom message",self.x / self.dt)
            self.odom_msg_.twist.twist.angular.z = self.heading / self.dt
            # print("anguler odom message",self.heading / self.dt)
        else:
            self.odom_msg_.twist.twist.linear.x = 0.0
            self.odom_msg_.twist.twist.angular.z = 0.0

        self.odom_msg_.twist.covariance = self.odom_msg_.pose.covariance

        print(self.odom_msg_)
        self.odom_pub.publish(self.odom_msg_)

        ######### Tf genertion #################
        self.transform_stamped_.header.stamp = rospy.Time.now()
        self.transform_stamped_.transform.translation.x = self.x
        self.transform_stamped_.transform.translation.y = self.y

        self.transform_stamped_.transform.rotation.x = q[0]
        self.transform_stamped_.transform.rotation.y = q[1]
        self.transform_stamped_.transform.rotation.z = q[2]
        self.transform_stamped_.transform.rotation.w = q[3]
        
        # self.br_.sendTransform(self.transform_stamped_)
        self.static_target_tf.sendTransform(self.transform_stamped_)
    # def update(self, event):
    #     orientation = quaternion_from_euler(0.0, 0.0, self.heading)

    #     odom = Odometry()
    #     odom.header.stamp = rospy.Time.now()
    #     odom.header.frame_id = 'odom'
    #     odom.child_frame_id = 'base_footprint'

    #     odom.pose.pose.position.x = self.x
    #     odom.pose.pose.position.y = self.y
    #     odom.pose.pose.position.z = 0.0

    #     odom.pose.pose.orientation.x = orientation[0]
    #     odom.pose.pose.orientation.y = orientation[1]
    #     odom.pose.pose.orientation.z = orientation[2]
    #     odom.pose.pose.orientation.w = orientation[3]

    #     odom.pose.covariance[0] = 0.001
    #     odom.pose.covariance[7] = 0.001
    #     odom.pose.covariance[14] = 1000000.0
    #     odom.pose.covariance[21] = 1000000.0
    #     odom.pose.covariance[28] = 1000000.0
    #     odom.pose.covariance[35] = 0.03

    #     if self.dt > 0.0:
    #         odom.twist.twist.linear.x = self.x / self.dt
    #         odom.twist.twist.angular.z = self.heading / self.dt
    #     else:
    #         odom.twist.twist.linear.x = 0.0
    #         odom.twist.twist.angular.z = 0.0

    #     odom.twist.covariance = odom.pose.covariance

    #     self.odom_pub.publish(odom)

        

    

if __name__ == '__main__':
    try:
        core0_odom = Core0Odom()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

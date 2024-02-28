import rospy
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
# from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
import tf_conversions
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class AmrOdom():
    def __init__(self):
        # super().__init__('amr_odom')

        # self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        self.timestamp = rospy.Time()

        # Parameters
        self.Wt = 0.415  # m
        self.wheel_radius = 0.0485  # Replace with your robot's wheel radius in meters

        # Robot body frame information
        self.x_ = 0.0
        self.y_ = 0.0
        self.heading_ = 0.0

        self.linear_ = 0.0
        self.linear_y = 0.0
        self.angular_ = 0.0


        # my edited code *********************************************
        self.odom_msg_ = Odometry()
        self.odom_msg_.header.frame_id = "odom"
        self.odom_msg_.child_frame_id = "base_footprint"
        self.odom_msg_.pose.pose.orientation.x = 0.0
        self.odom_msg_.pose.pose.orientation.y = 0.0
        self.odom_msg_.pose.pose.orientation.z = 0.0
        self.odom_msg_.pose.pose.orientation.w = 1.0

        self.br_ = TransformBroadcaster()
        self.transform_stamped_ = TransformStamped()
        self.transform_stamped_.header.frame_id = "odom"
        self.transform_stamped_.child_frame_id = "base_footprint"


        # self.pose_covariance = []

        # Publishers and Subscribers
        # self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.odom_pub = rospy.Publisher('odom', Odometry, 10)
        
        self.rpm_sub = rospy.Subscriber('motor_rpm', Float32MultiArray, self.rpm_callback)

    def rpm_callback(self, msg):
        # Extract RPM values for left and right motors
        rpm_right = msg.data[0]
        rpm_left = msg.data[1]
        print("New mssage received ", msg.data)
        # Call the function to update velocities and integrate odometry
        now = rospy.Time.now()
        self.get_velocities_diff_drive(rpm_left, rpm_right, now)

        # Update and publish odometry
        self.odom_update1()

    # def reset_odometry(self):
    #     self.x_ = 0.0
    #     self.y_ = 0.0
    #     self.heading_ = 0.0

    # def odom_update(self):
    #     now = rospy.Time.now()
    #     odom = Odometry()
    #     odom.header.stamp = now
    #     odom.header.frame_id = "odom"
    #     odom.child_frame_id = "base_link"
    #     odom.pose.pose.position.x = self.x_
    #     odom.pose.pose.position.y = self.y_
    #     odom.pose.pose.position.z = 0.0

    #     quaternion = quaternion_from_euler(0.0, 0.0, self.heading_)

    #     odom.pose.pose.orientation.x = quaternion[0]
    #     odom.pose.pose.orientation.y = quaternion[1]
    #     odom.pose.pose.orientation.z = quaternion[2]
    #     odom.pose.pose.orientation.w = quaternion[3]

    #     odom.twist.twist.linear.x = self.linear_
    #     odom.twist.twist.linear.y = self.linear_y
    #     odom.twist.twist.angular.z = self.angular_
    #     print(odom)
    #     self.odom_pub.publish(odom)


        # my edited odom update code *****************************************************

    def odom_update1(self):
        # odometry publish
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, self.heading_)
        self.odom_msg_.header.stamp = rospy.Time.now()
        self.odom_msg_.pose.pose.position.x = self.x_
        self.odom_msg_.pose.pose.position.y = self.y_
        self.odom_msg_.pose.pose.orientation.x = q[0]
        self.odom_msg_.pose.pose.orientation.y = q[1]
        self.odom_msg_.pose.pose.orientation.z = q[2]
        self.odom_msg_.pose.pose.orientation.w = q[3]

        self.odom_msg_.twist.twist.linear.x = self.linear_
        self.odom_msg_.twist.twist.angular.z = self.angular_

        # print(self.odom_msg_)

        self.odom_pub.publish(self.odom_msg_)

        # TF
        self.transform_stamped_.transform.translation.x = self.x_
        self.transform_stamped_.transform.translation.y = self.y_

        self.transform_stamped_.transform.rotation.x = q[0]
        self.transform_stamped_.transform.rotation.y = q[1]
        self.transform_stamped_.transform.rotation.z = q[2]
        self.transform_stamped_.transform.rotation.w = q[3]
        self.transform_stamped_.header.stamp = rospy.Time.now()
        self.br_.sendTransform(self.transform_stamped_)


    def get_velocities_diff_drive(self, rpm_l, rpm_r, time):
        d = self.Wt / 2.0
        linear_rpm = (rpm_r + rpm_l) / 2.0
        angular_rpm = (rpm_r - rpm_l) / (2.0 * d)

        body_linear_vel = self.get_vel_from_rpm(linear_rpm)
        body_angular_vel = self.get_vel_from_rpm(angular_rpm)
        print("Body linear vel",body_linear_vel)
        print("Body anguler vel",body_angular_vel)

        self.linear_ = body_linear_vel
        self.angular_ = body_angular_vel

        self.update_open_loop(self.linear_, self.angular_, time)

    def get_vel_from_rpm(self, rpm):
        linear_vel = (2 * math.pi * self.wheel_radius * rpm) / 60.0
        return linear_vel

    def cartesian_from_polar(self, ro, th):
        x = ro * math.cos(th)
        y = ro * math.sin(th)
        return x, y

    def update_open_loop(self, linear, angular, time):
        # Save last linear and angular velocity
        self.linear_ = linear
        self.angular_ = angular

        # Integrate odometry
        dt = time.to_sec() - self.timestamp.to_sec()
        self.timestamp = time
        self.integrate_exact(linear * dt, angular * dt)

    def integrate_xy(self, linear_x, linear_y, angular):
        delta_x = linear_x * math.cos(self.heading_) - linear_y * math.sin(self.heading_)
        delta_y = linear_x * math.sin(self.heading_) + linear_y * math.cos(self.heading_)

        self.x_ += delta_x
        self.y_ += delta_y
        self.heading_ += angular

    def integrate_runge_kutta2(self, linear, angular):
        direction = self.heading_ + angular * 0.5

        # Runge-Kutta 2nd order integration
        self.x_ += linear * math.cos(direction)
        self.y_ += linear * math.sin(direction)
        self.heading_ += angular

    def integrate_exact(self, linear, angular):
        if abs(angular) < 1e-6:
            self.integrate_runge_kutta2(linear, angular)
        else:
            # Exact integration (solve problems when angular is zero)
            heading_old = self.heading_
            r = linear / angular
            self.heading_ += angular
            self.x_ += r * (math.sin(self.heading_) - math.sin(heading_old))
            self.y_ += -r * (math.cos(self.heading_) - math.cos(heading_old))


# if __name__ == '__main__':
#     rospy.init_node('amr_odom_node')
#     amr_odom = AmrOdom()
#     rospy.spin()

#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math

from twist_controller import Controller
from geometry_msgs.msg import PoseStamped

from sensor_msgs.msg import Image

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)


        #initial section
        self.current_vel = None
        self.curr_ang_vel=None
        self.dbw_enabled = None
        self.linear_vel = None
        self.angular_vel = None
        self.throttle = 0
        self.steering =0
        self.brake = 0

        #for sync
        self.get_first_image = False

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',BrakeCmd, queue_size=1)

        # TODO: Create `Controller` object
        # self.controller = Controller(<Arguments you wish to provide>)
        # pass the parameter to the controller class in twist_controller.py
        # just simply pass all of param we get above

        #init a controller object and save insdie the sefl.controller
        self.controller = Controller(
                                    vehicle_mass = vehicle_mass,
                                    fuel_capacity = fuel_capacity,
                                    brake_deadband = brake_deadband,
                                    decel_limit = decel_limit,
                                    accel_limit = accel_limit,
                                    wheel_radius = wheel_radius,
                                    wheel_base = wheel_base,
                                    steer_ratio = steer_ratio,
                                    max_lat_accel = max_lat_accel,
                                    max_steer_angle = max_steer_angle
                                    )

        # TODO: Subscribe to all the topics you need to
        '''
            use command $ rostopic list
            /base_waypoints
            /current_pose
            /current_velocity
            /final_waypoints
            /image_color
            /rosout
            /rosout_agg
            /tf
            /tf_static
            /traffic_waypoint
            /twist_cmd
            /vehicle/brake_cmd
            /vehicle/brake_report
            /vehicle/dbw_enabled
            /vehicle/lidar
            /vehicle/obstacle
            /vehicle/obstacle_points
            /vehicle/steering_cmd
            /vehicle/steering_report
            /vehicle/throttle_cmd
            /vehicle/throttle_report
            /vehicle/traffic_lights

        '''
        rospy.Subscriber('/vehicle/dbw_enabled',Bool,self.dbw_enabled_cb)
        rospy.Subscriber('/twist_cmd',TwistStamped,self.twist_cb)
        rospy.Subscriber('/current_velocity',TwistStamped,self.velocity_ang_cb)

        #rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)

        #for sync img and car move
        rospy.Subscriber('/image_color', Image, self.get_first_image_cb)


        self.loop()

    def loop(self):
        rate = rospy.Rate(50) # MUST be 50Hz here!!!, for drive by wire, it works as 50Hz, if lower than 20hz, it will shut down
        while not rospy.is_shutdown():
            # TODO: Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled
            # throttle, brake, steering = self.controller.control(<proposed linear velocity>,
            #                                                     <proposed angular velocity>,
            #                                                     <current linear velocity>,
            #                                                     <dbw status>,
            #                                                     <any other argument you need>)
            # if <dbw is enabled>:
            #   self.publish(throttle, brake, steer)


            if not None in (self.current_vel, self.linear_vel, self.angular_vel):
                self.throttle, self.brake, self.steering = self.controller.control(self.current_vel,
                                                                                    self.curr_ang_vel,
                                                                                    self.dbw_enabled,
                                                                                    self.linear_vel,
                                                                                    self.angular_vel,
                                                                                    self.get_first_image)



            if self.dbw_enabled:
                rospy.logwarn("dbw_enabled: {0}, and publishing".format(self.dbw_enabled))
                #rospy.logwarn("get_first_image: {0}, and publishing".format(self.get_first_image))

                self.publish(self.throttle, self.brake, self.steering)

            rate.sleep()

    def dbw_enabled_cb(self,msg):
        self.dbw_enabled = msg.data


    def get_first_image_cb(self, msg):

        rospy.logwarn("[dbw_node] get_first_image_cb INININININININININININ")

        self.get_first_image = True

    def twist_cb(self,msg):
        '''
        cmd: rosmsg info geometry_msgs/TwistStamped
        std_msgs/Header header
          uint32 seq
          time stamp
          string frame_id
        geometry_msgs/Twist twist
          geometry_msgs/Vector3 linear
            float64 x -->this
            float64 y
            float64 z
          geometry_msgs/Vector3 angular
            float64 x
            float64 y
            float64 z -->this

        '''
        #target vel
        self.linear_vel = msg.twist.linear.x
        self.angular_vel = msg.twist.angular.z


    def velocity_ang_cb(self,msg):
        self.current_vel = msg.twist.linear.x
        self.curr_ang_vel = msg.twist.angular.z
        #rospy.logwarn("current_vel: {0}".format(self.current_vel))


    def publish(self, throttle, brake, steer):

        #if we want to force the car just move forward,
        # set throttle to 1, others are 0


        #throttle 0~1
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        #brake in units of torque (N*m)
        # can be computed using the desired acceleration, weight of the vehicle, and wheel radius.
        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)


if __name__ == '__main__':
    DBWNode()

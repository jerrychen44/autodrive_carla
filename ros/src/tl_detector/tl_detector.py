#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml

import numpy as np
from scipy.spatial import KDTree

'''
This python file processes the incoming traffic light data and camera images.
It uses the light classifier to get a color prediction, and publishes the location of any upcoming red lights.
'''

##############################################
'''
Your task for this portion of the project can be broken into two steps:

1.Use the vehicle's location and the (x, y) coordinates for traffic lights to
find the nearest visible traffic light ahead of the vehicle. This takes place in
the "process_traffic_lights" method of tl_detector.py.

You will want to use the get_closest_waypoint method to find the closest waypoints
to the vehicle and lights. Using these waypoint indices, you can determine
which light is ahead of the vehicle along the list of waypoints.


2. Use the camera image data to classify the color of the traffic light.
The core functionality of this step takes place in the "get_light_state" method of tl_detector.py.
There are a number of approaches you could take for this task.
One of the simpler approaches is to train a deep learning classifier to classify the
entire image as containing either a red light, yellow light, green light, or no light.
**One resource that's available to you is the traffic light's position in 3D space via the vehicle/traffic_lights topic.
'''
###############################################


STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.camera_image = None
        self.lights = []

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0


        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        #self.tmp = 0
        #self.dbw_enabled = None

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)#can be used to determine the vehicle's location.
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)#provides the complete list of waypoints.

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        #provides the (x, y, z) coordinates of all traffic lights.
        # the red, yellow, green info only works on the simulator
        # in real carla car, we need determine the color from below image_color and our own classifier.
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)

        #which provides an image stream from the car's camera.
        #These images are used to determine the color of upcoming traffic lights
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)


        # providing the permanent (x, y) coordinates for each traffic light's stop
        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
        self.is_sim = self.config["is_sim"]
        rospy.logwarn("[tl_det] is_sim {}".format(self.is_sim))

        #The node should publish the index of the waypoint for nearest upcoming red light's stop line to a single topic:
        '''
        For example, if waypoints is the complete list of waypoints,
        and an upcoming red light's stop line is nearest to waypoints[12],
        then 12 should be published /traffic_waypoint. This index can later be used by the waypoint
        updater node to set the target velocity for waypoints[12] to 0 and smoothly decrease the
        vehicle velocity in the waypoints leading up to waypoints[12].
        '''
        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)



        #sub7 = rospy.Subscriber('/vehicle/dbw_enabled',Bool,self.dbw_enabled_cb)




        rospy.spin()



    #def dbw_enabled_cb(self,msg):
    #    self.dbw_enabled = msg.data

    def pose_cb(self, msg):
        #current car position
        #rospy.logwarn("dl_detector pose_cb")

        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        #the same with waypoint_updater.py
        #self.base_waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            #later on , we can use the waypoint_tree to
            #find the closetest point to the car with KD_tree. from n - > log(n)
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        rospy.logwarn("[tl_det] image_cb INININININININININININ")

        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        #if we encounter the yellow - > red , or red -> green,
        #we hope the state can be stable
        if self.state != state:
            self.state_count = 0
            rospy.logwarn("[tl_det] state switch, {} -> {}, state_count {}".format(self.state,state,self.state_count))
            self.state = state

        elif self.state_count >= STATE_COUNT_THRESHOLD: #which means the state is stabled.
            self.last_state = self.state
            #in our case, we only care about the RED light, others light we can keep move.
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            rospy.logwarn("[tl_det] state_count >= STATE_COUNT_THRESHOLD, use light_wp {}, state_count {}".format(light_wp,self.state_count))

            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            rospy.logwarn("[tl_det] state_count <= STATE_COUNT_THRESHOLD, use last_wp {}, state_count {}".format(self.last_wp,self.state_count))
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))

        self.state_count += 1

    def get_closest_waypoint(self, x,y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
        closest_idx = self.waypoint_tree.query([x,y],1)[1]


        #0530 marked
        '''
        #check if closeset is ahead or behind vehicle
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx-1]

        #equation for hyperplane through closest_coord
        # change the list to vect
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x,y])

        val = np.dot(cl_vect - prev_vect, pos_vect - cl_vect)

        # the closest point now behind the car, we will force
        # use the next point as closest point
        if val > 0:
            closest_idx = (closest_idx +1) % len(self.waypoints_2d)
        '''
        return closest_idx

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        '''
        uint8 UNKNOWN=4
        uint8 GREEN=2
        uint8 YELLOW=1
        uint8 RED=0
        '''

        light_status=["RED","YELLOW","GREEN","UNKNOWN3","UNKNOWN4"]

        rospy.logwarn("[tl_det] is_sim {}".format(self.is_sim))
        if self.is_sim:
            #for simulator testing, just return the light state simulator pass to you directly
            rospy.logwarn("[tl_det] light_status, get_light_state: {0}".format(light_status[light.state]))
            return light.state
        else:

            #####################
            # mark first, for real car, we need a classification
            ####################
            if(not self.has_image):
                self.prev_light_loc = None
                return False
            #raw data to RGB8bit
            rospy.logwarn("[tl_det] Get image BGR 8bit")

            cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
            state = self.light_classifier.get_classification(cv_image)


            #Get classification
            rospy.logwarn("[tl_det] has_image, get_light_state: {0}".format(light_status[state]))
            return state


    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        closest_light = None
        line_wp_idx = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            #find the car poistion in waht waypoint
            car_position = self.get_closest_waypoint(self.pose.pose.position.x, self.pose.pose.position.y)

            #TODO find the closest visible traffic light (if one exists)
            # diff inital a max distance here, just like we find the min value as usual
            diff = len(self.waypoints.waypoints)
            for i, light in enumerate(self.lights):
                #Get setop each line waypoint index
                line = stop_line_positions[i]
                #this time, find the closest waypoint for traffic light
                temp_wp_idx = self.get_closest_waypoint(line[0],line[1])

                #check which is traffic light x,y clostest with car position and also ahead the car
                d= temp_wp_idx - car_position
                #rospy.logwarn("d: {0}".format(d))
                #rospy.logwarn("diff: {0}".format(diff))

                if d >= 0 and d < diff:
                    diff = d
                    closest_light = light
                    line_wp_idx = temp_wp_idx

                #rospy.logwarn("finial diff: {0}".format(diff))
                #rospy.logwarn("finial line_wp_idx: {0}".format(line_wp_idx))
        else:
            rospy.logwarn("[tl_det] self.pose != True")


        if closest_light:
            rospy.logwarn("[tl_det] get closest_light, to get_light_state")

            state = self.get_light_state(closest_light)
            return line_wp_idx, state
        else:
            rospy.logwarn("[tl_det] closest_light == None, return light_wp = -1, TrafficLight.UNKNOWN")


        # should we marked it or not ?
        #self.waypoints = None
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')

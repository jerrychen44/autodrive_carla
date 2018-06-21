#!/usr/bin/env python
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree


import math
from std_msgs.msg import Int32

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 #200, Number of waypoints we will publish. You can change this number
MAX_DECEL = 0.5

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)



        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        # add for get traffic waypoint
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)





        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below

        self.pose =None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None
        #add to combine the traffic light waypoint
        self.base_lane = None
        self.stopline_wp_idx = -1



        #rospy.spin()
        self.loop()

    #for control the publish frequence
    # we will publish the wapoints to wapoint_follower
    def loop(self):
        rate=rospy.Rate(50)# 50hz, change to 30 later in driver by wire
        while not rospy.is_shutdown():
            # if there are pose data in base_waypoits and the rospy not shutdown....
            if self.pose and self.base_waypoints:
                '''
                #first version, move to publish_waypoionts
                #Get closest waypoint
                closest_waypoint_idx = self.get_closest_waypoint_idx()
                self.publish_waypoints(closest_waypoint_idx)
                '''
                self.publish_waypoints()
            rate.sleep()


    def get_closest_waypoint_idx(self):
        #get thte coordinate of our car
        x=self.pose.pose.position.x
        y=self.pose.pose.position.y
        # get the index of this x,y in KDtree
        closest_idx = self.waypoint_tree.query([x,y],1)[1]

        # we hopt the new waypoints start IN FRONT of
        # the car...., we recheck below
        # If the clost x,y behind the car, we just ignore it, and
        # cut if off, and take next one point to replace it.

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

        return closest_idx

    #def publish_waypoints(self,closest_idx):#old version
    def publish_waypoints(self):
        '''
        #first version of publish_waypoints
        lane = Lane()#create a new lane msg
        #header is not important, optinonal
        lane.header = self.base_waypoints.header
        lane.waypoints = self.base_waypoints.waypoints[closest_idx: closest_idx + LOOKAHEAD_WPS]
        self.final_waypoints_pub.publish(lane)

        '''

        final_lane = self.generate_lane()
        self.final_waypoints_pub.publish(final_lane)

    def generate_lane(self):
        lane = Lane()

        #Get closest waypoint ahead the car
        closest_idx = self.get_closest_waypoint_idx()
        farthest_idx = closest_idx + LOOKAHEAD_WPS
        base_waypoints = self.base_waypoints.waypoints[closest_idx: farthest_idx]

        if self.stopline_wp_idx == -1 or (self.stopline_wp_idx >= farthest_idx):
            #in this case, which means we didn't detect the traffic light in our looking up waypoint list
            lane.waypoints = base_waypoints
        else:
            #there is a stopline wp indx , and inside our current waypoint list
            rospy.logwarn("[waypoint_updater] got a stopline waypoint, need decelerate")
            lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_idx)

        return lane


    def decelerate_waypoints(self, waypoints, closest_idx):
        #we don't directy modify the original base_waypoint
        #it will loss the info, when we come back the same place, that will mass up.
        rospy.logwarn("[waypoint_updater] -2 stop_idx")

        # we creating a new wapoint list
        temp = []
        #walk through the all waypoints[closest_waypoint_idx: farthest_idx]
        for i , wp in enumerate(waypoints):

            p = Waypoint()
            p.pose = wp.pose# use original base_waypoints pose

            # -2 is to let the car's nose can align with the traffic light
            # stop_idx, how much idx ahead from  the car
            stop_idx = max(self.stopline_wp_idx - closest_idx -2, 0)
            #get the distance between waypoints[i] and waypoints[stop_idx]
            dist = self.distance(waypoints, i ,stop_idx)
            #use sqrt the let the vel dropping really sharp when the
            #dist is really close the traffic light
            vel = math.sqrt(2*MAX_DECEL*dist)
            if vel < 1:
                vel = 0

            #assign the vel for each wapoint
            #if the vel small then original vel, then we use the small one.
            p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
            temp.append(p)
        return temp



    def pose_cb(self, msg):
        # TODO: Implement
        # be called frequencetly, save the current car position
        self.pose = msg
        pass

    def waypoints_cb(self, waypoints_in):
        # TODO: Implement
        # get base waypoints with a singel call back

        #just save the waypoints, waypoints will not change
        self.base_waypoints = waypoints_in
        #if we don't have waypoints_2d, create one
        # we hope the sefl.waypoints_2d has initialzed before
        # the susbscriber, which means this cb fun
        # will be called before the self.waypoints_2d = None
        # then the KD tree will get some unpredictable points.
        if not self.waypoints_2d:
            '''
            check the command

            msg: Lane
            Header header
            Waypoint[] waypoints
            msg: waypoints
            geometry_msgs/PoseStamped pose
            geometry_msgs/TwistStamped twist

            msg: pose
            rosmsg info geometry_msgs/PoseStamped
            std_msgs/Header header
              uint32 seq
              time stamp
              string frame_id
            geometry_msgs/Pose pose
              geometry_msgs/Point position
                float64 x
                float64 y
                float64 z
              geometry_msgs/Quaternion orientation
                float64 x
                float64 y
                float64 z
                float64 w

            '''

            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints_in.waypoints]
            #later on , we can use the waypoint_tree to
            #find the closetest point to the car with KD_tree. from n - > log(n)
            self.waypoint_tree = KDTree(self.waypoints_2d)


        pass

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.stopline_wp_idx = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')

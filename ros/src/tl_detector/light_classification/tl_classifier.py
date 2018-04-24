from styx_msgs.msg import TrafficLight
import rospy
import cv2
from utils.utils import get_yolo_boxes, makedirs
from utils.bbox import draw_boxes
from keras.models import load_model
import numpy as np
import time
import os
#from keras.backend import clear_session


'''
This file contains the TLClassifier class. You can use this class to implement traffic light classification.
For example, the get_classification method can take a camera image as input
and return an ID corresponding to the color state of the traffic light in the image.
Note that it is not required for you to use this class.
It only exists to help you break down the classification problem into more manageable chunks.
Also note that Carla currently has TensorFlow 1.3.0 installed.
If you are using TensorFlow, please be sure to test your code with this version before submission.
'''


'''
Helper Tool in the Simulator
In order to help you acquire an accurate ground truth data source for the traffic light classifier,
the Udacity simulator publishes the current color state of all traffic lights in the simulator to
the /vehicle/traffic_lights topic in addition to the light location. This state can be used to generate
classified images or subbed into your solution to help you work on another single component of the node.
The state component of the topic won't be available when running your solution in real life so don't rely
on it in the final submission.
However, you can still reference this topic in real life to get the 3D world position of the traffic light.
'''

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        rospy.logwarn("[traffic classifier] TLClassifier INIT")
        #clear_session()

        ###############################
        #   Set some parameter
        ###############################
        self.net_h = 416
        self.net_w = 416 # a multiple of 32, the smaller the faster
        self.obj_thresh = 0.6
        self.nms_thresh = 0.35

        ###############################
        #   Load the model
        ###############################
        os.environ['CUDA_VISIBLE_DEVICES'] = "0"
        pwd = os.getcwd()

        self.infer_model = load_model(pwd+"/light_classification/model/tl.h5")
        #workaround for keras deploy issue
        self.infer_model.predict(np.zeros((1, self.net_h, self.net_w, 3)))

        self.model_anchors = [17,18, 28,24, 36,34, 42,44, 56,51, 72,66, 90,95, 92,154, 139,281]
        self.model_labels = ["red","yellow","green"]

        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction

        '''
        uint8 UNKNOWN=4
        uint8 GREEN=2
        uint8 YELLOW=1
        uint8 RED=0
        '''



        tStart = time.time()
        # predict the bounding boxes
        boxes = get_yolo_boxes(self.infer_model, [image], self.net_h, self.net_w, self.model_anchors, self.obj_thresh, self.nms_thresh)[0]

        tEnd = time.time()
        rospy.logwarn ("It cost {} sec".format (tEnd - tStart))


        #debug
        state_prob = -99
        state = -99
        state_idx = -99

        for w in range(len(boxes)):
            rospy.logwarn("[Jerry] boxes {}, {}, {}".format(boxes[w].classes,boxes[w].score,boxes[w].label))
            state_idx = 0
            for class_prob in boxes[w].classes:
                rospy.logwarn("[Jerry] class_prob {}, state_prob {}".format(class_prob,state_prob))

                if class_prob > state_prob and class_prob != 0.0:
                    state_prob = class_prob
                    state = state_idx
                    rospy.logwarn("[Jerry] state {}".format(state))


                state_idx = state_idx +1

        if state == -99:
            state = TrafficLight.UNKNOWN


        #draw bounding boxes on the image using labels
        #draw_boxes(image, boxes, self.model_labels, self.obj_thresh)

        # write the image with bounding boxes to file
        #cv2.imwrite("output_"+str(tEnd)+'.png', np.uint8(image))

        #rospy.logwarn("[traffic classifier] state: {}".format(state))
        return state

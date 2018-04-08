from styx_msgs.msg import TrafficLight
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
        return TrafficLight.UNKNOWN

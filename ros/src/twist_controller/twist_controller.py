from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

'''
You can use this class to implement vehicle control.
For example, the control method can take twist data as input
and return throttle, brake, and steering values. Within this class,
you can import and use the provided pid.py and lowpass.py if needed for acceleration,
and yaw_controller.py for steering.
Note that it is not required for you to use these, and you are free to write and import other controllers.
'''
class Controller(object):
    def __init__(self,
                vehicle_mass ,
                fuel_capacity ,
                brake_deadband ,
                decel_limit ,
                accel_limit ,
                wheel_radius ,
                wheel_base ,
                steer_ratio ,
                max_lat_accel ,
                max_steer_angle
                ):
        # TODO: Implement
        min_speed = 0.1
        self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)

        #original
        #original
        #kp = 0.3
        #ki = 0.1
        #kd = 0.
        #mn = 0. #Minimum throttle value
        #mx = 0.2#Maximum throttle value
        #self.throttle_controller = PID(kp,ki,kd,mn,mx)

        #0530, add steering and throttle controller

        ###################
        # Throttle
        ###################
        #highway ok
        #testing lot ok, need use 10kms
        throttle_kp = 0.3
        throttle_ki = 0.1
        throttle_kd = 0.005

        #testing lot ok, need use 10kms
        #throttle_kp = 0.8
        #throttle_ki = 0.002
        #throttle_kd = 0.3



        #not good in testing log
        #throttle_kp = 0.8
        #throttle_ki = 0.00
        #throttle_kd = 0.06



        min_throttle = 0.0 # Minimum throttle value
        max_throttle = 0.2 # Maximum throttle value (should be OK for simulation, not for Carla!)
        #max_throttle = 0.5*accel_limit
        #self.throttle_controller =PID(2.0, 0.4, 0.1, min_throttle, max_throttle)
        self.throttle_controller = PID(throttle_kp, throttle_ki, throttle_kd, min_throttle, max_throttle)


        ###################
        # Steering
        ###################
        #highway ok
        #testing lot ok, need use 10kms
        steer_kp = 0.4
        steer_ki = 0.1
        steer_kd = 0.005

        #testing lot ok, need use 10kms
        #steer_kp = 1.15
        #steer_ki = 0.001
        #steer_kd = 0.1



        #not good in testing log
        #steer_kp = 0.5
        #steer_ki = 0.00
        #steer_kd = 0.2

        min_steer = -max_steer_angle
        max_steer = max_steer_angle
        self.steering_controller = PID(steer_kp, steer_ki, steer_kd, min_steer, max_steer)



        #is for filter out the high frequency noise of volcity
        tau = 0.5 # 1/(2pi*tau) = cutoff frequency
        ts = 0.02 #Sample time
        self.vel_lpf = LowPassFilter(tau,ts)

        # catch the input
        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.max_lat_accel = max_lat_accel
        self.max_steer_angle = max_steer_angle

        self.last_time = rospy.get_time()

        #pass

    # this fun will be call 50Hz, from dbw_node
    def control(self,
                current_vel,
                curr_ang_vel,
                dbw_enabled,
                linear_vel,
                angular_vel,
                get_first_image
                        ):
        # TODO: Change the arg, kwarg list to suit your needs


        if not dbw_enabled:
            rospy.logwarn("dbw_enabled: {0}, reset controller".format(dbw_enabled))

            #we also need to reset the PID controller to prevent
            #the error accumulating.
            # If we don't reset that, when we put dbw_enable back on , the PID will
            # use the wrong error to do something wrong behavior. ## because there is a integral term in PID.
            self.steering_controller.reset()
            self.throttle_controller.reset()
            return 0.,0.,0.



        if get_first_image == False:
            rospy.logwarn("get_first_image: {0}, return 0.,450.,0.".format(get_first_image))
            return 0.,450.,0.

        #so, if dbw is enable

        current_vel = self.vel_lpf.filt(current_vel)

        rospy.logwarn("Target vel: {0}".format(linear_vel))
        rospy.logwarn("Target angular vel: {0}\n".format(angular_vel))

        rospy.logwarn("current_vel: {0}".format(current_vel))
        rospy.logwarn("current angular_vel: {0}".format(curr_ang_vel))

        rospy.logwarn("filtered vel: {0}".format(self.vel_lpf.get()))




        ##############
        # time
        ##############
        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time



        ############
        # steering
        ############
        #use Yaw controller to get the steering
        steering_base = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)
        #rospy.logwarn("steering: {0}\n".format(steering))
        #0530
        angular_vel_error = angular_vel - curr_ang_vel
        steering_correction = self.steering_controller.step(angular_vel_error, sample_time)
        steering_total = steering_base + steering_correction
        steering = max(min(self.max_steer_angle, steering_total), -self.max_steer_angle)
        ##############
        # throttle
        ##############
        #get the vel diff erro between the the vel we want to be (linear_vel) v.s. the current_vel
        # the linear_vel comes from twist_cmd, which is the waypoint_follower published, is the target vel
        # from waypoint updater
        vel_error = linear_vel - current_vel
        self.last_vel = current_vel
        #use PID controller to get the proper throttle under the vel_error and the sample time
        throttle = self.throttle_controller.step(vel_error, sample_time)
        brake = 0




        ################
        #some constrain
        ################
        # if the target vel is 0, and we are very slow <0.1, set the throttle to 0 directly to stop
        if linear_vel ==0. and current_vel < 0.1:
            rospy.logwarn("[twist_controller] vel is really small, directly to STOP")
            throttle = 0
            brake = 450 #N*m to hold the car in place if we are stopped at a light. Acceleration 1m/s^2

        # vel_erro < 0 means the current car vel too fast than we want, and we still put small throttle
        # we directly stop to add throttle, and get the real brake, because this time
        # the car still moving, we need the real brake result from the vel, and mass, whell radius
        elif throttle < 0.1 and vel_error <0:
        #elif vel_error <0:

            rospy.logwarn("[twist_controller] slow down")

            throttle =0
            #decel is our desired deacceleration
            decel = max(vel_error, self.decel_limit)
            # vehicle_mass in kilograms, wheel_radius in meters
            # use abs is because the simluator, the de-acceleration should be negative, but
            # the simulator get the positive number as the brake value
            brake = abs(decel) * self.vehicle_mass * self.wheel_radius # Torque N*m


        # Return throttle(min is 0.1 m/s), brake, steer

        #hard code to keep car move stright
        #throttle = 1.0
        #brake = 0
        #steering = 0

        rospy.logwarn("[output] throttle: {0}".format(throttle))
        rospy.logwarn("[output] brake: {0}".format(brake))
        rospy.logwarn("[output] steering: {0}".format(steering))


        return throttle, brake, steering

#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#                                                                               #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#   You may not use this file except in compliance with the License.            #
#   You may obtain a copy of the License at                                     #
#                                                                               #
#       http://www.apache.org/licenses/LICENSE-2.0                              #
#                                                                               #
#   Unless required by applicable law or agreed to in writing, software         #
#   distributed under the License is distributed on an "AS IS" BASIS,           #
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.    #
#   See the License for the specific language governing permissions and         #
#   limitations under the License.                                              #
#################################################################################

"""
cmdvel_to_servo_node.py
This module decides the action messages (servo control messages specifically angle
and throttle) to be sent out after converting the cmd_vel.

The node defines:
    cmdvel_subscriber: A subscriber to the /cmd_vel (twist messages) published by
                       ROS2 Nav stack.
    action_publisher: A publisher to publish the action (angle and throttle values).
    set_max_speed_service: A service to dynamically set MAX_SPEED_PCT representing
                           the max speed percentage scale as per request.
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import (QoSProfile,
                       QoSHistoryPolicy,
                       QoSReliabilityPolicy)
from std_srvs.srv import Trigger

import geometry_msgs.msg
from deepracer_interfaces_pkg.msg import ServoCtrlMsg
from std_msgs.msg import String
from deepracer_interfaces_pkg.srv import SetMaxSpeedSrv
from cmdvel_to_servo_pkg import constants
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import math
import threading

#----------------------------------------------------------------------------#
import csv
import os
from datetime import datetime


class CmdvelToServoNode(Node):
    """Node responsible for deciding the action messages (servo control messages specifically angle
       and throttle) to be sent out after converting the cmd_vel.
    """

    def __init__(self, qos_profile):
        """Create a CmdvelToServoNode.
        """
        super().__init__('cmdvel_to_servo_node')
        self.get_logger().info("cmdvel_to_servo_node started.")

        # Subscription to cmd_vel.
        self.cmdvel_subscriber = \
            self.create_subscription(geometry_msgs.msg.Twist,
                                     constants.CMDVEL_TOPIC,
                                     self.on_cmd_vel,
                                     qos_profile)
        

        # Publisher to publish action (angle and throttle).
        self.action_publisher = self.create_publisher(ServoCtrlMsg,
                                                      constants.ACTION_PUBLISH_TOPIC,
                                                      qos_profile)
        

        # Service to dynamically set MAX_SPEED_PCT.
        self.set_max_speed_service = self.create_service(SetMaxSpeedSrv,
                                                         constants.SET_MAX_SPEED_SERVICE_NAME,
                                                         self.set_max_speed_cb)


        # Flag to activate/deactivate IMU use
        # 0: imu deactivated
        # 1: imu activated
        self._imu_in_use_flag = 1
        
       
        if(self._imu_in_use_flag == 1):
            ### Si la imu está siendo implementada agrega las funcionalidades necesarias para la comunicación
            # -------------------------- ADDED FOR IMU INTEGRATION ----------------------------- #
            # Subscription to zero_motion topic
            self.zero_motion_subscriber = self.create_subscription(Odometry,
                                                               constants.ODOM_MSG_TOPIC,
                                                               self.zeromotion_cb,
                                                               qos_profile)
            
            self.imu_accel_subscriber = self.create_subscription(Imu,
                                                            constants.IMU_MSG_TOPIC,
                                                            self.imu_accel_cb,
                                                            qos_profile)
            
            # Service_client to request motion state of DR for the first time
            get_motion_state_cb_group = ReentrantCallbackGroup()
            self.get_motion_state_client = self.create_client(Trigger,
                                                          constants.GET_MOTION_STATE_CLIENT_NAME,
                                                          callback_group=get_motion_state_cb_group)
            
            while not self.get_motion_state_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('get_motion_state_service not available, waiting again...')
            self.get_motion_state_req = Trigger.Request()

            # Zero Motion handler
            self.zero_motion_status = 0   # data recieved from imu
            self._first_MO = True         # flag for service request
            self.no_motion_state = 0      # DR movement status 
            # regulates the increment in the target speed for the engines until the vehicule move
            self._n = 0 #1                    
            self.slope = 0.1
            self._first_time = True
            self._peak = 0

            # Imu acceleration
            self.imu_accel_x = 0
            self.imu_accel_y = 0
        
            # Timer to check motion state of the robot
            self.motion_state_timer = self.create_timer(0.01, self.check_motion_state) # o poner en 1

            # --------------------------------------- END ----------------------------------------- #
        else:
            ### Si la imu no está siendo implementada agrega contador para el arranque
            self.peak_cnt = 0
            self.max_time_peak = 4
            

        # Linear velocity in X received on command (m/s) -- Nav2.
        self.target_linear = 0.0
        # Angular velocity in Z received on command (rad/s) -- Nav2.
        self.target_rot = 0.0
        # Target throttle and streering to publish
        self.target_steer_n2 = 0.0
        self.target_speed_n2 = 0.0
        # Max speed pct for throttle output to be rescaled with respect to.
        self.max_speed_pct = constants.MAX_SPEED_PCT

        ## ---------------------------- FILE ------------------------------ ##
        #self.data_counter = 0
        #self.file_path = os.path.join('/home/deepracer/deepracer_files/navigation_files/', 'nav2dataXservodata.txt')
        #self.dt = open(self.file_path, 'w', newline='')
        #self.csv_ = csv.writer(self.dt)
        #self.csv_.writerow(['   Nav2  ',' To Servo ', ' Counter peak '])
        ## ---------------------------------------------------------------- ##
        
        self.lock = threading.Lock()

   
    def send_request(self):
        """Send request for get_motion_state_client"""
        self.future = self.get_motion_state_client.call_async(self.get_motion_state_req)
        self.future.add_done_callback(self.handle_response_cb)  # Add callback to handle response

    def handle_response_cb(self, future):
        """Callback for managing the response of the service get_motion_state_service
        Args: 
            future: get_motion_state_service response
        """
        try:
            response = future.result()
            #self.get_logger().info(f"Service response: {response}")
            if response.success:
                self._first_MO = False
                self.setInitialMotionState(response.success)
                #self.get_logger().info(f'Initial no motion state: {response.success}')
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
        

    def set_max_speed_cb(self, req, res):
        """Callback which dynamically sets the max_speed_pct.
        Args:
            req (SetMaxSpeedSrv.Request): Request object with the updated
                                                  max speed percentage.
            res (SetMaxSpeedSrv.Response): Response object with error(int) flag
                                                   indicating successful max speed pct
                                                   update.
        Returns:
            SetMaxSpeedSrv.Response: Response object with error(int) flag indicating
                                             successful max speed pct update.
        """
        with self.lock:
            try:
                self.max_speed_pct = req.max_speed_pct
                self.get_logger().info(f"Incoming request: max_speed_pct: {req.max_speed_pct}")
                res.error = 0
            except Exception as ex:
                self.get_logger().error(f"Failed set max speed pct: {ex}")
                res.error = 1
        return res

    def get_rescaled_manual_speed(self, categorized_throttle, max_speed_pct):
        """Return the non linearly rescaled speed value based on the max_speed_pct.
        Args:
            categorized_throttle (float): Float value ranging from -1.0 to 1.0.
            max_speed_pct (float): Float value ranging from 0.0 to 1.0 taken as input
                                   from maximum speed input.
        Returns:
            float: Categorized value of the input speed.
        """
        # return 0.0 if categorized_throttle or maximum speed pct is 0.0.
        if categorized_throttle == 0.0 or max_speed_pct == 0.0:
            return 0.0

        # Get the parameter value to calculate the coefficients a, b in the equation y=ax^2+bx
        # The lower the update_speed_scale_value parameter, higher the impact on the
        # final mapped_speed.
        # Hence the update_speed_scale_value parameter is inversely associated with max_speed_pct
        # and bounded by MANUAL_SPEED_SCALE_BOUNDS.
        # Ex: max_speed_pct = 0.5; update_speed_scale_value = 3
        #     max_speed_pct = 1.0; update_speed_scale_value = 1
        # Lower the update_speed_scale_value: categorized_throttle value gets mapped to
        # higher possible values.
        #   Example: update_speed_scale_value = 1.0;
        #            categorized_throttle = 0.8 ==> mapped_speed = 0.992
        # Higher the update_speed_scale_value: categorized_throttle value gets mapped to
        # lower possible values.
        #   Example: update_speed_scale_value = 3.0;
        #            categorized_throttle = 0.8 ==> mapped_speed = 0.501

        inverse_max_speed_pct = (1 - max_speed_pct)

        update_speed_scale_value = \
            constants.MANUAL_SPEED_SCALE_BOUNDS[0] + \
            inverse_max_speed_pct * \
            (constants.MANUAL_SPEED_SCALE_BOUNDS[1] - constants.MANUAL_SPEED_SCALE_BOUNDS[0])

        if update_speed_scale_value < 0.0:
            self.get_logger().info("The update_speed_scale_value is negative, taking absolute value.")
            update_speed_scale_value = abs(update_speed_scale_value)
        speed_mapping_coefficients = dict()

        # Recreate the mapping coefficients for the non-linear equation ax^2 + bx based on
        # the update_speed_scale_value.
        # These coefficents map the [update_speed_scale_value, update_speed_scale_value/2]
        # values to DEFAULT_SPEED_SCALE values [1.0, 0.8].
        speed_mapping_coefficients["a"] = \
            (1.0 / update_speed_scale_value**2) * \
            (2.0 * constants.DEFAULT_SPEED_SCALES[0] - 4.0 * constants.DEFAULT_SPEED_SCALES[1])
        speed_mapping_coefficients["b"] = \
            (1.0 / update_speed_scale_value) * \
            (4.0 * constants.DEFAULT_SPEED_SCALES[1] - constants.DEFAULT_SPEED_SCALES[0])
        return math.copysign(1.0, categorized_throttle) * \
            (speed_mapping_coefficients["a"] * abs(categorized_throttle)**2 +
             speed_mapping_coefficients["b"] * abs(categorized_throttle))

    def imu_accel_cb(self, msg):
        self.imu_accel_x = msg.linear_acceleration.x
        self.imu_accel_y = msg.linear_acceleration.y

    def zeromotion_cb(self, msg):
        """Callback on receiving zero motion update from IMU node
        Args:
           msg (Odometry): Odometry message"""
        self.zero_motion_status = msg.pose.covariance[0]
        self.no_motion_state  = self.zero_motion_status
        #self.get_logger().info('Updated motion state ZM: {:+.1f}'.format(self.zero_motion_status))

    def check_motion_state(self):

        if(self._first_MO):
           self.send_request()
           return

        target_throttle = self.target_speed_n2
        target_angle = self.target_steer_n2


        # If robot's speed value is below threshold value sends 0 as throttle value
        if(abs(target_throttle) == 0.0):  #or abs(target_throttle) == 0.1):
            #self._n = 0
            self.action_publish(0.0, 0.0)
            return

        # Checks if the robot is not moving even though a not-zero speed value was sent to the motor
        # if this condition is true, then updates the speed value to be published.
        # Otherwise, if the robot is moving when a not-zero speed was sent publishes again the angle and
        # throttle values calculate by plan_action
        if(self.no_motion_state and (abs(target_throttle) != 0.0)):

            self._n = self._n + 1

            if(target_throttle >= 0):
                # Si la velocidad es positiva toma el mínimo entre la velocidad y 1
                target_throttle = min(target_throttle + self.slope*self._n, 1.0)
            else:
                # Si la velocidad es negativa toma el máximo entre la velocidad y -1
                target_throttle = max(-1.0, target_throttle - self.slope*self._n)

            
            self.action_publish(target_angle, target_throttle)

        # If the robot is moving does not resets _n but does not change the angle and throttle values
        # already published
        if(self.no_motion_state == 0):
            if(self._first_time):
                self._peak = self._n
                self._first_time = False
            self._n = self._peak
            
            if(target_throttle < 0.6):
                target_throttle = 0.6
                self.action_publish(target_angle, target_throttle)
            else:
                self.action_publish(target_angle, target_throttle)

        ## ------------------------------------------------------------------------------------ ##    
        self.get_logger().info('Nav2 setted throttle: {:+4f}'.format(self.target_speed_n2))
        self.get_logger().info('Calculated throttle: {:+4f}'.format(target_throttle))
        #self.get_logger().info('Peak: {:+4f}'.format(self._peak))
        #self.get_logger().info('N: {:+4f}'.format(self._n))
        self.get_logger().info('Acceleration X: {:+4f}'.format(self.imu_accel_x))
        self.get_logger().info('Acceleration Y: {:+4f}'.format(self.imu_accel_y))
        self.get_logger().info('No_motion_state: {:+4f}'.format(self.no_motion_state))
        ## ------------------------------------------------------------------------------------ ##
        

    def setInitialMotionState(self, initial_mo_state):
        """Sets initial motion state of the robot
        Args:
            initial_mo_state: First motion state of the robot after power on
        """
        self.no_motion_state  = initial_mo_state
        self.get_logger().info(f'Set initial no motion state: {initial_mo_state}')

        
    def on_cmd_vel(self, msg):
        """Callback on receiving a velocity update from ROS2 Nav stack.
        Args:
            msg: (geometry_msgs.msg.Twist): Geometry twist message.
        """
        try:
            # Linear position recieved from nav
            self.target_linear = msg.linear.x
            # Angular position recieved from nav
            self.target_rot = msg.angular.z

            # Nav2 messege
            #self.get_logger().info('Nav2 mssg')
            #self.get_logger().info('target rot: {:+.2f}'.format(self.target_rot))
            #self.get_logger().info('target linear: {:+.5f}'.format(self.target_linear))

            # PWM action "calculated"
            self.target_steer_n2, self.target_speed_n2 = self.plan_action()
            #self.get_logger().info('PWM linear: {:+.2f}'.format(self.target_speed_n2))

            # Publish PWM values into the servo topic
            self.action_publish(self.target_steer_n2, self.target_speed_n2)
            
        except Exception as ex:
            self.get_logger().error(f"Failed to publish action: {ex}")
            self.action_publish(constants.ActionValues.DEFAULT_OUTPUT, constants.ActionValues.DEFAULT_OUTPUT)

    def get_mapped_throttle(self, target_linear_clamped):
        """Get the mapped throttle value for DR servo scaled between [-1, 1].
        Args:
            target_linear_clamped (float): Clamped linear velocity between MAX_SPEED 
                                           and MIN_SPEED supported by DeepRacer.

        Returns:
            (float): Throttle value mapped wrt DeepRacer.
        """
        target_linear_pct = abs(target_linear_clamped / constants.VehicleNav2Dynamics.MAX_SPEED)
        if target_linear_pct >= constants.VehicleNav2Dynamics.MAX_THROTTLE_RATIO:
            return constants.ActionValues.MAX_THROTTLE_OUTPUT
        elif target_linear_pct >= constants.VehicleNav2Dynamics.MID_THROTTLE_RATIO:
            return constants.ActionValues.MID_THROTTLE_OUTPUT
        elif target_linear_pct >= constants.VehicleNav2Dynamics.MIN_THROTTLE_RATIO:
            return constants.ActionValues.MIN_THROTTLE_OUTPUT
        else:
            return constants.ActionValues.DEFAULT_OUTPUT

    def get_mapped_steering(self, target_rot_clamped):
        """Get the mapped steering value for DR servo scaled between [-1, 1].
        Args:
            target_rot_clamped (float): Clamped rotation between MAX_STEER
                                        and MIN_STEER supported by DeepRacer.
        Returns:
            (float): Angle value mapped wrt DeepRacer.
        """
        target_rot_pct = abs(target_rot_clamped / constants.VehicleNav2Dynamics.MAX_STEER)
        if target_rot_pct >= constants.VehicleNav2Dynamics.MAX_STEERING_RATIO:
            return constants.ActionValues.MAX_STEERING_OUTPUT
        elif target_rot_pct >= constants.VehicleNav2Dynamics.MID_STEERING_RATIO:
            return constants.ActionValues.MID_STEERING_OUTPUT
        elif target_rot_pct >= constants.VehicleNav2Dynamics.MIN_STEERING_RATIO:
            return constants.ActionValues.MIN_STEERING_OUTPUT
        else:
            return constants.ActionValues.DEFAULT_OUTPUT

    def plan_action(self):
        """Calculate the target steering and throttle.
        
        Returns:
            steering (float): Angle value to be published to servo.
            throttle (float): Throttle value to be published to servo.
        """

        # THROTTLE
        #----------
        # Clamping the linear velocity between MAX_SPEED and MIN_SPEED supported by DeepRacer.
        #self.get_logger().info("Target Linear: %f"%self.target_linear)
        target_linear_clamped = max(min(self.target_linear, constants.VehicleNav2Dynamics.MAX_SPEED),
                                            constants.VehicleNav2Dynamics.MIN_SPEED)
        #self.get_logger().info("Target Linear Clamped: %f"%target_linear_clamped)
        
        # Get the throttle values mapped wrt DeepRacer servo.
        target_throttle_mapped = self.get_mapped_throttle(target_linear_clamped)
        #self.get_logger().info("Target Throttle Clamped: %f"%target_throttle_mapped)
        
        # Set the direction.
        target_throttle_signed = target_linear_clamped #* math.copysign(1.0, self.target_linear) #### !!!!!!!VER SIGNO!!!!!!! ####
        #self.get_logger().info("Target Throttle Signed: %f"%target_throttle_signed)

        throttle = self.get_rescaled_manual_speed(target_throttle_signed, self.max_speed_pct)

        # Si no se está usando la IMU
        if(self._imu_in_use_flag == 0):
            if (self.target_linear == 0):
                # If nav2 vel == 0 --> throttle  = 0
                throttle = 0.0;
                self.peak_cnt = 0
            else:
                # Get rescaled throttle.
                if ( self.peak_cnt >= self.max_time_peak ):
                    throttle = throttle #self.get_rescaled_manual_speed(target_throttle_signed, self.max_speed_pct)
                else:
                    throttle = math.copysign(1.0, self.target_linear)
                    self.peak_cnt += 1
        
        #self.get_logger().info("Throttle to send: %f"%throttle)
        
        
        # ROTATION
        #----------
        # Clamping the rotation between MAX_STEER and MIN_STEER supported by DeepRacer.
        target_rot_clamped = max(min(self.target_rot, constants.VehicleNav2Dynamics.MAX_STEER),
                                        constants.VehicleNav2Dynamics.MIN_STEER)
        
        # Get the steering angle mapped wrt DeepRacer servo.
        #target_steering_mapped = self.get_mapped_steering(self.target_rot)
        
        # Set the direction.
        steering = target_rot_clamped * math.copysign(1.0, self.target_linear)


        # self.get_logger().inf("Peak counter: %f"%self.peak_cnt)

        #if (self.data_counter < 30 and self._imu_in_use_flag == 0):
            # Saves 30 data points
            #self.csv_.writerow([self.target_linear, throttle, self.peak_cnt])
            #self.data_counter += 1
            #if(self.data_counter == 30):
            #    self.dt.close()
                
        return steering, throttle

    def action_publish(self, target_steer, target_speed):
        """Function publishes the action and sends it to servo.

        Args:
            target_steer (float): Angle value to be published to servo.
            target_speed (float): Throttle value to be published to servo.
        """
        result = ServoCtrlMsg()
        result.angle, result.throttle = target_steer, target_speed
        #self.get_logger().info(f"Publishing to servo: Steering {target_steer} | Throttle {target_speed}")
        
        self.action_publisher.publish(result)


def main(args=None):
    rclpy.init(args=args)
    qos = QoSProfile(reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                     depth=1,
                     history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST)
    cmdvel_to_servo_node = CmdvelToServoNode(qos)
    executor = MultiThreadedExecutor()
    rclpy.spin(cmdvel_to_servo_node, executor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cmdvel_to_servo_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

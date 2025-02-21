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

import csv
import os
import time


class CmdvelToServoNode(Node):
    """Node responsible for deciding the action messages (servo control messages specifically angle
       and throttle) to be sent out after converting the cmd_vel.
    """

    def __init__(self, qos_profile):
        """Create a CmdvelToServoNode.
        """
        super().__init__('cmdvel_to_servo_node')
        self.get_logger().info("cmdvel_to_servo_node started.")

        # Cmd_vel topic subscriber
        self.cmdvel_subscriber = \
            self.create_subscription(geometry_msgs.msg.Twist,
                                     constants.CMDVEL_TOPIC,
                                     self.on_cmd_vel,
                                     qos_profile)
        

        # Action publisher (angle and throttle).
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
            
            # Zero_motion topic subscriber (no motion data)
            self.zero_motion_subscriber = self.create_subscription(Odometry,
                                                               constants.ODOM_MSG_TOPIC,
                                                               self.zeromotion_cb,
                                                               qos_profile)

            # Imu topic subscriber (accelerometer and gyroscope data)
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
            self.zero_motion_status = 0   # zero motion status recieved from IMU
            self._first_MO = True         # flag for first zero motion request
            self.no_motion_state = 0      # DR current motion status: 0 - DR is moving; 1 - DR is not moving 
            self.stop_counter = 0         # counter to stop the car with stop sequence

            # Imu acceleration data
            self.imu_accel_x = 0
            self.imu_accel_y = 0
            self.imu_accel_z = 0
        
            # Timer to check motion state of the robot
            self.motion_state_timer = self.create_timer(0.01, self.check_motion_state)
            self.have_sent_last = False
    

        
        # Nav2 commands    
        # Linear velocity in X received on command (m/s)
        self.target_linear = 0.0
        # Angular velocity in Z received on command (rad/s) 
        self.target_rot = 0.0
        
        # Target throttle and streering calculated based on Nav2 command.
        self.target_steer_n2 = 0.0
        self.target_speed_n2 = 0.0
       
        # Last throttle value sent to servo. 
        self.target_throttle_ant = 0
        
        # Max speed pct for throttle output to be rescaled with respect to.
        self.max_speed_pct = constants.MAX_SPEED_PCT
        
        self.file_path = os.path.join('/home/deepracer/deepracer_nav2_ws/deepracer_files/navigation_files/', 'navigation_data_test2.csv')
        self._write_file = False

        
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
        """Callback for IMU acceletarion data
        Args:
            msg: 3 dimensional acceletarion
        """
        self.imu_accel_x = msg.linear_acceleration.x
        self.imu_accel_y = msg.linear_acceleration.y
        self.imu_accel_z = msg.linear_acceleration.z

    def zeromotion_cb(self, msg):
        """Callback on receiving zero motion update from IMU node
        Args:
           msg (Odometry): Odometry message"""
        self.zero_motion_status = msg.pose.covariance[0]
        self.no_motion_state  = self.zero_motion_status
        

    def check_motion_state(self):
        """Timer callback. Corrects the speed being published on the servo topic according to
        the state of motion of the robot
        """

        # Sends request to get motion state of the robot for the first time
        if(self._first_MO):
           self.send_request()
           return

        ## STOP
        # If the robot is moving and a speed command under 0.3 was calculated then a stop sequence will be executed
        # Value 0.3 is used as threshold to prevent collision cause by the innertia carried by the vehicle
        if(self.no_motion_state == 0 and (abs(self.target_speed_n2) <= 0.3)):
            if(self.target_throttle_ant!=0):
                if(self.stop_counter<10):
                    if(self.target_throttle_ant>0):
                        self.action_publish(0.0, -0.75)
                        self.get_logger().info("DECELERATION P - Throttle to send: %f"%-0.75)
                        self.stop_counter += 1
                        return
                    else:
                        self.action_publish(0.0, 0.75)
                        self.get_logger().info("DECELERATION N - Throttle to send: %f"%0.75)
                        self.stop_counter += 1
                        return  

                self.stop_counter = 0
                self.target_throttle_ant = 0.0

            self.get_logger().info("DECELERATION FINAL - Throttle to send: %f"%0.0)
            self.action_publish(0.0, 0.0)
            self.target_throttle_ant = 0.0
            return

        ## START
        # If the robot is not moving and a speed command greater than 0.3 has been sent, an initial peak
        # to start the motor is sent once.
        # have_sent_last is used as a flag to indicate the initial peak has been sent
        if(self.have_sent_last == False and self.no_motion_state == 1 and self.target_speed_n2 > 0.30):
            tbs = 1.0 * math.copysign(1.0, self.target_speed_n2)
            self.action_publish(self.target_steer_n2, 1.0 * math.copysign(1.0, self.target_speed_n2))
            self.get_logger().info("PEAK - Throttle to send: %f"%tbs)
            self.have_sent_last = True
            self.target_throttle_ant = 1.0 * math.copysign(1.0, self.target_speed_n2)

        ## NORMAL NAVIGATION
        # If the start peak has already been published, the robot is moving and the calculated current speed is
        # above the threshold, cruising speed is published
        if(self.have_sent_last == True and self.no_motion_state == 0):
            tbs = 0.75 * math.copysign(1.0, self.target_speed_n2)
            self.action_publish(self.target_steer_n2, 0.75 * math.copysign(1.0, self.target_speed_n2))
            self.get_logger().info("AFTER PEAK - Throttle to send: %f"%tbs)
            self.have_sent_last = False
            self.target_throttle_ant = 0.75 * math.copysign(1.0, self.target_speed_n2)
        

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

            # PWM action "calculated"
            self.target_steer_n2, self.target_speed_n2 = self.plan_action()

            throttle_to_send = 0.0
            # Throttle/speed to be published is corrected according to the absolute value of the calculated speed
            # and the direction of the movement.
            # If the calculated speed is above 0.3 the actual published speed is going to be the minimum speed needed
            # for a smooth navigation.
            if(abs(self.target_speed_n2) > 0.3):
                throttle_to_send = 0.75 * math.copysign(1.0, self.target_speed_n2)  
                self.target_throttle_ant = throttle_to_send

            self.get_logger().info("ONCMD - Throttle to send: %f"%throttle_to_send)
            self.action_publish(self.target_steer_n2, throttle_to_send)

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
        target_linear_clamped = max(min(self.target_linear, constants.VehicleNav2Dynamics.MAX_SPEED),
                                            constants.VehicleNav2Dynamics.MIN_SPEED)
        
        # Get the throttle values mapped wrt DeepRacer servo.
        target_throttle_mapped = self.get_mapped_throttle(target_linear_clamped)
        
        # Set the direction.
        target_throttle_signed = target_linear_clamped 

        throttle = self.get_rescaled_manual_speed(target_throttle_signed, self.max_speed_pct)
        
        #self.get_logger().info("NAV2 - Throttle to send: %f"%throttle)
        
        
        # ROTATION
        #----------
        # Clamping the rotation between MAX_STEER and MIN_STEER supported by DeepRacer.
        target_rot_clamped = max(min(self.target_rot, constants.VehicleNav2Dynamics.MAX_STEER),
                                        constants.VehicleNav2Dynamics.MIN_STEER)
        
        # Get the steering angle mapped wrt DeepRacer servo.
        #target_steering_mapped = self.get_mapped_steering(self.target_rot)
        
        # Set the direction.
        steering = target_rot_clamped * math.copysign(1.0, self.target_linear)

                
        return steering, throttle

    def action_publish(self, target_steer, target_speed):
        """Function publishes the action and sends it to servo.

        Args:
            target_steer (float): Angle value to be published to servo.
            target_speed (float): Throttle value to be published to servo.
        """
        result = ServoCtrlMsg()
        result.angle, result.throttle = target_steer, target_speed

        timestamp = self.get_clock().now().to_msg()
        if(self._write_file):
            with open(self.file_path, mode='a', newline='') as file:
                writer = csv.writer(file)
                if file.tell() == 0:
                    writer.writerow(['time', 'no_motion' , 'nav2_linear', 'nav2_throttle', 'throttle_sent'])
                writer.writerow([timestamp, self.no_motion_state, self.target_linear, self.target_speed_n2, target_speed])
        
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

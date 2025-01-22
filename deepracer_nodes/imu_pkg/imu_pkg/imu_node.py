#!/usr/bin/env python

#################################################################################
#   Copyright Lars Ludvigsen. All Rights Reserved.          #
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
imu_node.py

"""

import math
import threading
from imu_pkg import (BMI160)
from imu_pkg import (I2CID)
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Trigger


from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3, Pose
from imu_pkg import (constants)

from rcl_interfaces.msg import ParameterDescriptor, ParameterType


class IMUNode(Node):
    """Node responsible for collecting the camera and LiDAR messages and publishing them
       at the rate of the camera sensor.
    """

    def __init__(self):
        """Create a IMUNode.
        """
        super().__init__("imu_node")
        self.get_logger().info("IMU node initializing.")
        self.stop_queue = threading.Event()

        self.declare_parameter('bus_id', constants.I2C_BUS_ID,
                               ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))
        self.declare_parameter('address', constants.BMI160_ADDR,
                               ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))
        self.declare_parameter('publish_rate', constants.IMU_MSG_RATE,
                               ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))
        self.declare_parameter('zero_motion_odometer', True,
                               ParameterDescriptor(type=ParameterType.PARAMETER_BOOL))

        self._bus_id = self.get_parameter('bus_id').value
        self._address = self.get_parameter('address').value
        self._publish_rate = self.get_parameter('publish_rate').value
        self._zero_motion = self.get_parameter('zero_motion_odometer').value
        self.nomotion_state = 1; #flag to detect a change in zero motion status

        self.get_logger().info("zero motion value {:+.0f}".format(self._zero_motion))

        self.get_logger().info("Connecting to IMU at bus {} address {}".format(self._bus_id, self._address))

        # Publisher that sends combined sensor messages with IMU acceleration and gyroscope data.
        self.imu_message_pub_cb_grp = ReentrantCallbackGroup()
        self.imu_message_publisher = self.create_publisher(Imu,
                                                           constants.IMU_MSG_TOPIC,
                                                           1,
                                                           callback_group=self.imu_message_pub_cb_grp)

        if self._zero_motion:
            self.odom_message_pub_cb_grp = ReentrantCallbackGroup()
            self.odom_message_publisher = self.create_publisher(Odometry,
                                                                constants.ODOM_MSG_TOPIC,
                                                                1,
                                                                callback_group=self.odom_message_pub_cb_grp)
            

        # Service to send motion state of the DR for the first time    
        self.get_motion_state_service = self.create_service(Trigger,
                                                           constants.GET_MOTION_STATE_SERVICE_NAME,
                                                           self.get_motion_state_cb)
        self.get_logger().info('Service get_motion_state_service ready')

        # Heartbeat timer.
        self.timer_count = 0
        self.timer = self.create_timer(5.0, self.timer_callback)

        self.initial_time = self.get_clock().now().to_msg().sec

        self.get_logger().info("IMU node created.")

    def get_motion_state_cb(self, request, response):
        """Get motion state service callback.
        Args:
            request (Trigger.Request): Request object: bool data 
            response (Trigger.Response.Success): Response object with motion 
                                                   state of the vehicle: bool -- 1 if it is not
                                                   moving.
            response (Trigger.Response.Message): Response object with motion 
                                                   state of the vehicle: string -- motion state
        Returns:
            Trigger.Response: Response object with motion state of the vehicle
        """

        self.nomotion_state = self.sensor.getIntZeroMotionStatus()
        #self.get_logger().info("Incoming request: get DR motion state")

        if(self.nomotion_state):
            response.success = True
            response.message = "DR is not moving"
        else:
            response.success = False
            response.message = "DR is moving" 
        
        return response

    def timer_callback(self):
        """Heartbeat function to keep the node alive.
        """
        self.get_logger().debug(f"Timer heartbeat {self.timer_count}")
        self.timer_count += 1

    def __enter__(self):
        """Called when the node object is created using the 'with' statement.
        Returns:
           IMUNode : self object returned.
        """
        try:

            # self.get_logger().info(f"Trying to initialize the sensor at {constants.BMI160_ADDR}
            # on bus {constants.I2C_BUS_ID}")
            I2C_ID = I2CID.I2C_ID()
            I2C_BUS_ID = I2C_ID.getId(1)
            
            self.sensor = BMI160.IMU_BMI160(I2C_BUS_ID)  # Depends on changes to library

            # configure sampling rate and filter
            self.sensor.set_accel_rate(6)   # 100Hz
            self.sensor.setAccelDLPFMode(0)

            # Defining the Range for Accelerometer and Gyroscope
            ##### PROBAR CONFIGURAR EL RANGO EN 2G PARA EL ACELERÓMETRO #####
            self.sensor.setFullScaleAccelRange(constants.DEF_ACCEL_RANGE_4G, constants.ACCEL_RANGE_4G_FLOAT)
            self.sensor.setFullScaleGyroRange(constants.DEF_GYRO_RANGE_250, constants.GYRO_RANGE_250_FLOAT)
            
            ## Calibrating Accelerometer - assuming that it stands on 'flat ground'.
            ## Gravity points downwards, hence Z should be calibrated to -1.
            self.sensor.setAccelOffsetEnabled(True)
            
            self.sensor.autoCalibrateXAccelOffset(0)
            self.sensor.autoCalibrateYAccelOffset(0)
            self.sensor.autoCalibrateZAccelOffset(-1)
            
            ## Enable standing still check
            if self._zero_motion:
                self.sensor.setZeroMotionDetectionDuration(0x00)
                self.sensor.setZeroMotionDetectionThreshold(0x02)
                self.sensor.setIntZeroMotionEnabled(True)

        except Exception as ex:
            self.get_logger().info(f"Failed to create IMU monitor: {ex}")
            self.observer = None
            raise ex

        self.get_logger().info('Initialization and calibration of IMU sensor done.')

        self.thread = threading.Thread(target=self.processor)
        self.thread.start()

        # Start IMU event monitor.
        return self

    def __exit__(self, ExcType, ExcValue, Traceback):
        """Called when the object is destroyed.
        """
        self.get_logger().info('Exiting.')
        self.stop_queue.set()
        self.rate.destroy()
        self.thread.join()

    def processor(self):

        self.get_logger().info(f"Publishing messages at {self._publish_rate} Hz.")

        if self._zero_motion:
            self.get_logger().info(f"Publishing zero-motion odometry.")

        self.rate = self.create_rate(self._publish_rate)

        while not self.stop_queue.is_set() and rclpy.ok():
            try:
                self.publish_imu_message()
                self.rate.sleep()
            except Exception as ex:
                self.get_logger().error(f"Failed to create IMU message: {ex}")

    def publish_imu_message(self):
        """Publish the sensor message when we get new data for the slowest sensor(LiDAR).
        """
        try:
            imu_msg = Imu()
            data = self.sensor.get_accel_gyro()
            
            # if zero motion is active publish odometry information in topic
            if self._zero_motion:
                no_motion = self.sensor.getIntZeroMotionStatus() # zero motion current state
                #self.get_logger().info(f'no motion: {self.sensor.getIntZeroMotionStatus()}')
            
                odom_msg = Odometry()
                odom_msg.header.stamp = self.get_clock().now().to_msg()
                odom_msg.header.frame_id = 'odom'
                odom_msg.child_frame_id = 'base_link'
                odom_msg.pose.pose = Pose()
                odom_msg.pose.covariance = constants.EMPTY_ARRAY_36
                odom_msg.pose.covariance[0] = no_motion
                odom_msg.twist.twist.linear = Vector3()
                odom_msg.twist.covariance = constants.EMPTY_ARRAY_36

              
               # self.get_logger().info('no motion state: {:+.1f}'.format(self.nomotion_state))
               # self.nomotion_state = no_motion
                
                # Sends new data only when there is a change in the state of motion of the robot
                if(abs(no_motion-self.nomotion_state) != 0):
                    #self.get_logger().info('no motion: {:+.1f}'.format(no_motion))
                    #self.get_logger().info('no motion state: {:+.1f}'.format(self.nomotion_state))
                    self.nomotion_state = no_motion
                    self.odom_message_publisher.publish(odom_msg)
                    #self.initial_time = self.get_clock().now().to_msg().sec


                    
            # fetch all gyro values - return in rad / sec
            gyro = Vector3()
            # swap x and y
            gyro.x =float(data[3])
            # swap x and y
            gyro.y = float(data[4])
            # upside-down
            gyro.z = float(data[5])


            # fetch all accel values - return in m/s²
            accel = Vector3()
            # swap x and y
            accel.x = float(data[0])
            # swap x and y
            accel.y = float(data[1])
            # upside-down
            accel.z = float(data[2])

            
            imu_msg.angular_velocity = gyro
            imu_msg.angular_velocity_covariance = constants.COVAR_ARRAY_9

            imu_msg.linear_acceleration = accel
            imu_msg.linear_acceleration_covariance = constants.COVAR_ARRAY_9

            imu_msg.orientation_covariance = constants.EMPTY_ARRAY_9
            imu_msg.orientation_covariance[0] = -1.0

            # add header
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'base_link'

            #self.get_logger().info('ax: {:+.3f}'.format(accel.x))
            #self.get_logger().info('ay: {:+.3f}'.format(accel.y))
            #self.get_logger().info('az: {:+.0f}'.format(accel.z))

            #self.get_logger().info('gx: {:+.0f}'.format(gyro.x))
            #self.get_logger().info('gy: {:+.0f}'.format(gyro.y))
            #self.get_logger().info('gz: {:+.0f}'.format(gyro.z))

            #self.get_logger().info(str(data))

            self.imu_message_publisher.publish(imu_msg)
        

        except Exception as ex:
            self.get_logger().error(f"Error in publishing sensor message: {ex}")



def main(args=None):

    try:
        rclpy.init(args=args)
        with IMUNode() as imu_node:
            executor = MultiThreadedExecutor()
            rclpy.spin(imu_node, executor)
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        imu_node.destroy_node()
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()

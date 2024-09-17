from smbus2 import SMBus, i2c_msg
import sys, getopt 
from time import sleep
import os
from imu_pkg import (constants)

class BMI160_CONSTANTS():
  BMI160_DEVICE_ADDRESS = 0x68

  BMI160_REGA_USR_CHIP_ID           = 0x00
  BMI160_REGA_USR_ACC_CONF_ADDR     = 0x40 
  BMI160_REGA_USR_ACC_RANGE_ADDR    = 0x41
  BMI160_REGA_USR_GYR_CONF_ADDR     = 0x42
  BMI160_REGA_USR_GYR_RANGE_ADDR    = 0x43
  
  BMI160_REGA_CMD_CMD_ADDR          =   0x7e
  BMI160_REGA_CMD_EXT_MODE_ADDR     =   0x7f
  
  CMD_SOFT_RESET_REG      = 0xb6
  
  CMD_PMU_ACC_SUSPEND     = 0x10
  CMD_PMU_ACC_NORMAL      = 0x11
  CMD_PMU_ACC_LP1         = 0x12
  CMD_PMU_ACC_LP2         = 0x13
  CMD_PMU_GYRO_SUSPEND    = 0x14
  CMD_PMU_GYRO_NORMAL     = 0x15
  CMD_PMU_GYRO_FASTSTART  = 0x17
  
  BMI160_USER_DATA_14_ADDR = 0X12 # accel x 
  BMI160_USER_DATA_15_ADDR = 0X13 # accel x 
  BMI160_USER_DATA_16_ADDR = 0X14 # accel y 
  BMI160_USER_DATA_17_ADDR = 0X15 # accel y 
  BMI160_USER_DATA_18_ADDR = 0X16 # accel z 
  BMI160_USER_DATA_19_ADDR = 0X17 # accel z 
  
  BMI160_USER_DATA_8_ADDR  = 0X0C # gyro x 
  BMI160_USER_DATA_9_ADDR  = 0X0D # gyro x 
  BMI160_USER_DATA_10_ADDR = 0X0E # gyro y 
  BMI160_USER_DATA_11_ADDR = 0X0F # gyro y
  BMI160_USER_DATA_12_ADDR = 0X10 # gyro z
  BMI160_USER_DATA_13_ADDR = 0X11 # gyro z
  
  BMI160_RA_FOC_CONF       = 0x69
  BMI160_STATUS_FOC_RDY    = 3
  BMI160_RA_STATUS         = 0x1B
  BMI160_CMD_START_FOC     = 0x03
  
  BMI160_FOC_ACC_X_BIT     = 4
  BMI160_FOC_ACC_Y_BIT     = 2
  BMI160_FOC_ACC_Z_BIT     = 0
  
  BMI160_RA_OFFSET_0       = 0x71
  BMI160_RA_OFFSET_1       = 0x72
  BMI160_RA_OFFSET_2       = 0x73
  BMI160_RA_OFFSET_3       = 0x74
  BMI160_RA_OFFSET_4       = 0x75
  BMI160_RA_OFFSET_5       = 0x76
  BMI160_RA_OFFSET_6       = 0x77
  
  BMI160_FOC_GYR_EN        = 6
  BMI160_ACC_OFFSET_EN     = 6
  BMI160_GYR_OFFSET_EN     = 7

class IMU_BMI160(BMI160_CONSTANTS):
  
  def __init__(self, BUS):
    self.bus = SMBus(BUS)

    ###########################    START UP   #############################
    ## Power on and prepare for general usage.
    # This will activate the device and take it out of sleep mode (which must be done
    # after start-up). This function also sets both the accelerometer and the gyroscope
    # to default range settings, namely +/- 2g and +/- 250 degrees/sec.

    # Issue a soft-reset to bring the device into a clean state
    self._reg_write(constants.REG_CMD, constants.COMM_SOFT_RESET)
    sleep(0.001)

    # Issue a dummy-read to force the device into I2C comms mode
    self._reg_read(0x7F)
    sleep(0.001)

    # Power up the accelerometer
    self._reg_write(constants.REG_CMD, constants.COMM_ACC_MODE_NORMAL)
    # Wait for power-up to complete
    while (1 != self.reg_read_bits(constants.REG_PMU_STATUS, constants.DEF_ACC_PMU_STATUS_BIT, constants.DEF_ACC_PMU_STATUS_LEN)):
      pass
    sleep(0.001)
    
    print(self.reg_read_bits(constants.REG_PMU_STATUS, constants.DEF_ACC_PMU_STATUS_BIT, constants.DEF_ACC_PMU_STATUS_LEN))
    
    # Power up the gyroscope
    self._reg_write(constants.REG_CMD, constants.COMM_GYR_MODE_NORMAL)
    sleep(0.001)
    # Wait for power-up to complete
    while (1 != self.reg_read_bits(constants.REG_PMU_STATUS, constants.DEF_GYR_PMU_STATUS_BIT, constants.DEF_GYR_PMU_STATUS_LEN)):
      sleep(0.2)
      pass
    sleep(0.001)

    self.setFullScaleGyroRange(constants.DEF_GYRO_RANGE_250, 250.0)
    self.setFullScaleAccelRange(constants.DEF_ACCEL_RANGE_2G, 2.0)

    # Only PIN1 interrupts currently supported - map all interrupts to PIN1
    #self._reg_write(constants.REG_INT_MAP_0, 0xFF)
    #self._reg_write(constants.REG_INT_MAP_1, 0xF0)
    #self._reg_write(constants.REG_INT_MAP_2, 0x00)

    #########################################################################
    
    self.acc_x = 0
    self.acc_y = 0
    self.acc_z = 0
    self.acc_x_raw = 0
    self.acc_y_raw = 0
    self.acc_z_raw = 0
    
    self.gyro_x = 0
    self.gyro_y = 0
    self.gyro_z = 0
    self.gyro_x_raw = 0
    self.gyro_y_raw = 0
    self.gyro_z_raw = 0

    self.accel_range = 0
    self.gyro_range  = 0
    
    self.x_accelOffset = 0
    self.y_accelOffset = 0
    self.z_accelOffset = 0
    self.chipid = self.bus.read_byte_data(self.BMI160_DEVICE_ADDRESS, self.BMI160_REGA_USR_CHIP_ID)
    
    print("---------")
    if self.chipid == 0xD1 :
      print("chip id is 0x%X, BMI160" % self.chipid)
    else :
      print("Exit")
      sys.exit()
    print("---------" )
      
    #chip init
    #self.bus.write_byte_data(self.BMI160_DEVICE_ADDRESS, self.BMI160_REGA_USR_ACC_CONF_ADDR, 0x28)
    #self.bus.write_byte_data(self.BMI160_DEVICE_ADDRESS, self.BMI160_REGA_USR_ACC_RANGE_ADDR, 0x3)
    #self.bus.write_byte_data(self.BMI160_DEVICE_ADDRESS, self.BMI160_REGA_USR_GYR_CONF_ADDR, 0x28)
    #self.bus.write_byte_data(self.BMI160_DEVICE_ADDRESS, self.BMI160_REGA_USR_GYR_RANGE_ADDR, 0x0)
    #
    ##command register
    #self.bus.write_byte_data(self.BMI160_DEVICE_ADDRESS, self.BMI160_REGA_CMD_CMD_ADDR, self.CMD_SOFT_RESET_REG)

   # self.calibration()

  def reg_read_bits(self, reg, pos, len):
    b = self.bus.read_byte_data(self.BMI160_DEVICE_ADDRESS, reg)
    mask = (1 << len) - 1
    b >>= pos
    b &= mask
    return b;

  def reg_write_bits( self, reg, data, pos, len):
    b = self.bus.read_byte_data(self.BMI160_DEVICE_ADDRESS, reg)
    mask = ((1 << len) - 1) << pos
    data <<= pos; # shift data into correct position
    data &= mask; # zero all non-important bits in data
    b &= ~(mask); # zero all important bits in existing byte
    b |= data; # combine data with existing byte
    self.bus.write_byte_data(self.BMI160_DEVICE_ADDRESS, reg, b)

  def enable_accel(self):
      #op_mode set to 0 and go to normal mode
      sleep(0.1)
      self.bus.write_byte_data(self.BMI160_DEVICE_ADDRESS, self.BMI160_REGA_CMD_CMD_ADDR, CMD_PMU_ACC_NORMAL)
      sleep(0.1)
      return;

  def read_accel(self):
    acc_value = [ 0, 0, 0, 0, 0, 0]
    
    #read acc xyz
    acc_value = self.bus.read_i2c_block_data(self.BMI160_DEVICE_ADDRESS, self.BMI160_USER_DATA_14_ADDR, 6)
    
    self.acc_x_raw =  (acc_value[1] << 8) | acc_value[0]
    self.acc_y_raw =  (acc_value[3] << 8) | acc_value[2]
    self.acc_z_raw =  (acc_value[5] << 8) | acc_value[4]
    
    if(self.acc_x_raw > 0x7fff) :
      self.acc_x_raw = -(0xffff - self.acc_x_raw + 1) 
      
    if(self.acc_y_raw > 0x7fff) :
      self.acc_y_raw = -(0xffff - self.acc_y_raw + 1) 

    if(self.acc_z_raw > 0x7fff) :
      self.acc_z_raw = -(0xffff - self.acc_z_raw + 1) 
          
    self.acc_x = (self.acc_x_raw * self.accel_range) / (0x8000)
    self.acc_y = (self.acc_y_raw * self.accel_range) / (0x8000)
    self.acc_z = (self.acc_z_raw * self.accel_range) / (0x8000)

    #self.acc_x = self.acc_x_raw/0x0001
    #self.acc_y = self.acc_y_raw/0x0001
    #self.acc_z = self.acc_z_raw/0x0001

    return

  def enable_gyro(self):
    #op_mode set to 0 and go to normal mode
    sleep(0.1)
    self.bus.write_byte_data(self.BMI160_DEVICE_ADDRESS, self.BMI160_REGA_CMD_CMD_ADDR, CMD_PMU_GYRO_NORMAL)
    sleep(0.1)
    return;

  def read_gyro(self):
    gyro_value = [ 0, 0, 0, 0, 0, 0]

    #read gyro xyz
    gyro_value = self.bus.read_i2c_block_data(self.BMI160_DEVICE_ADDRESS, self.BMI160_USER_DATA_8_ADDR, 6)
    
    self.gyro_x_raw =  (gyro_value[1] << 8) | gyro_value[0]
    self.gyro_y_raw =  (gyro_value[3] << 8) | gyro_value[2]
    self.gyro_z_raw =  (gyro_value[5] << 8) | gyro_value[4]
    
    if(self.gyro_x_raw > 0x7fff) :
      self.gyro_x_raw = -(0xffff - self.gyro_x_raw + 1) 
      
    if(self.gyro_y_raw > 0x7fff) :
      self.gyro_y_raw = -(0xffff - self.gyro_y_raw + 1) 

    if(self.gyro_z_raw > 0x7fff) :
      self.gyro_z_raw = -(0xffff - self.gyro_z_raw + 1) 

      self.gyro_x = (self.gyro_x_raw * self.gyro_range) / 0x8000
      self.gyro_y = (self.gyro_y_raw * self.gyro_range) / 0x8000
      self.gyro_z = (self.gyro_z_raw * self.gyro_range) / 0x8000

      return;

  def getAccelOffsetEnabled(self):
    accelStatus = self.reg_read_bits(self.BMI160_RA_OFFSET_6 , self.BMI160_ACC_OFFSET_EN, 1)
    print("status %x" %( accelStatus ))
    return;

  ## Set accelerometer offset compensation enabled value.
  # @see getXAccelOffset()
  # @see registers.OFFSET_6
  def setAccelOffsetEnabled(self, enabled):
    self.reg_write_bits(self.BMI160_RA_OFFSET_6, enabled, self.BMI160_ACC_OFFSET_EN, 1)
    return;

  def autoCalibrateXAccelOffset(self, target):
    if (target == 1):
      foc_conf = (0x1 << self.BMI160_FOC_ACC_X_BIT);
    elif (target == -1):
      foc_conf = (0x2 << self.BMI160_FOC_ACC_X_BIT);
    elif (target == 0):
      foc_conf = (0x3 << self.BMI160_FOC_ACC_X_BIT);
    else:
      return;  #Invalid target value 

    self.bus.write_byte_data(self.BMI160_DEVICE_ADDRESS, self.BMI160_RA_FOC_CONF, foc_conf)
    self.bus.write_byte_data(self.BMI160_DEVICE_ADDRESS, self.BMI160_REGA_CMD_CMD_ADDR, self.BMI160_CMD_START_FOC)

    while True:
      ra_status = self.bus.read_byte_data(self.BMI160_DEVICE_ADDRESS, self.BMI160_RA_STATUS)
      if((ra_status & 0x08) != 0):
        break
    return;

  def autoCalibrateYAccelOffset(self, target):
    if (target == 1):
      foc_conf = (0x1 << self.BMI160_FOC_ACC_Y_BIT);
    elif (target == -1):
      foc_conf = (0x2 << self.BMI160_FOC_ACC_Y_BIT);
    elif (target == 0):
      foc_conf = (0x3 << self.BMI160_FOC_ACC_Y_BIT);
    else:
      return;  #Invalid target value 

    self.bus.write_byte_data(self.BMI160_DEVICE_ADDRESS, self.BMI160_RA_FOC_CONF, foc_conf)
    self.bus.write_byte_data(self.BMI160_DEVICE_ADDRESS, self.BMI160_REGA_CMD_CMD_ADDR, self.BMI160_CMD_START_FOC)

    while True:
      ra_status = self.bus.read_byte_data(self.BMI160_DEVICE_ADDRESS, self.BMI160_RA_STATUS)
      if((ra_status & 0x08) != 0):
        break
    return;

  def autoCalibrateZAccelOffset(self, target):
    if (target == 1):
      foc_conf = (0x1 << self.BMI160_FOC_ACC_Z_BIT);
    elif (target == -1):
      foc_conf = (0x2 << self.BMI160_FOC_ACC_Z_BIT);
    elif (target == 0):
      foc_conf = (0x3 << self.BMI160_FOC_ACC_Z_BIT);
    else:
      return;  #Invalid target value 

    self.bus.write_byte_data(self.BMI160_DEVICE_ADDRESS, self.BMI160_RA_FOC_CONF, foc_conf)
    self.bus.write_byte_data(self.BMI160_DEVICE_ADDRESS, self.BMI160_REGA_CMD_CMD_ADDR, self.BMI160_CMD_START_FOC)

    while True:
      ra_status = self.bus.read_byte_data(self.BMI160_DEVICE_ADDRESS, self.BMI160_RA_STATUS)
      if((ra_status & 0x8) != 0):
        break
    return;

  def getAccelOffset(self):

    print("Internal sensor offsets AFTER calibration...")
    self.x_accelOffset = self.bus.read_byte_data(self.BMI160_DEVICE_ADDRESS, self.BMI160_RA_OFFSET_0)
    self.y_accelOffset = self.bus.read_byte_data(self.BMI160_DEVICE_ADDRESS, self.BMI160_RA_OFFSET_1)
    self.z_accelOffset = self.bus.read_byte_data(self.BMI160_DEVICE_ADDRESS, self.BMI160_RA_OFFSET_2)

    if(self.x_accelOffset > 0x7f) :
      self.x_accelOffset = -(0xff - self.x_accelOffset + 1)

    if(self.y_accelOffset > 0x7f) :
      self.y_accelOffset = -(0xff - self.y_accelOffset + 1)

    if(self.z_accelOffset > 0x7f) :
      self.z_accelOffset = -(0xff - self.z_accelOffset + 1)

    print("x_accelOffset %d y_accelOffset %d z_accelOffset %d" % (self.x_accelOffset, self.y_accelOffset, self.z_accelOffset))
    return;

  def autoCalibrateGyroOffset(self):
    foc_conf = (1 << self.BMI160_FOC_GYR_EN);
    self.bus.write_byte_data(self.BMI160_DEVICE_ADDRESS, self.BMI160_RA_FOC_CONF, foc_conf)
    self.bus.write_byte_data(self.BMI160_DEVICE_ADDRESS, self.BMI160_REGA_CMD_CMD_ADDR, self.BMI160_CMD_START_FOC) 

    while True:
      ra_status = self.bus.read_byte_data(self.BMI160_DEVICE_ADDRESS, self.BMI160_RA_STATUS)
      if((ra_status & 0x8) != 0):
        break
    return;

  def sign_extend(self, value, bits):
    sign_bit = 1 << (bits - 1)
    return (value & (sign_bit - 1)) - (value & sign_bit)

  def getGyroOffset(self):
    x_offset = self.bus.read_byte_data(self.BMI160_DEVICE_ADDRESS, self.BMI160_RA_OFFSET_3) 
    x_offset |=  (self.reg_read_bits(self.BMI160_RA_OFFSET_6, 0, 2)) << 8  #Get OFFSET_6 bit 0 bit 1 for off_gry_x<9:8>
    x_gyroOffset = self.sign_extend(x_offset, 10)

    y_offset = self.bus.read_byte_data(self.BMI160_DEVICE_ADDRESS, self.BMI160_RA_OFFSET_4)
    y_offset |=  (self.reg_read_bits(self.BMI160_RA_OFFSET_6, 2, 2)) << 8 #Get OFFSET_6 bit 2 bit 3 for off_gry_y<9:8>
    y_gyroOffset = self.sign_extend(y_offset, 10)

    z_offset = self.bus.read_byte_data(self.BMI160_DEVICE_ADDRESS, self.BMI160_RA_OFFSET_5)
    z_offset |=  (self.reg_read_bits(self.BMI160_RA_OFFSET_6, 4, 2)) << 8 #Get OFFSET_6 bit 4 bit 5 for off_gry_z<9:8>
    z_gyroOffset = self.sign_extend(z_offset, 10)

    print("x_gyroOffset %d y_gyroOffset %d z_gyroOffset %d" % (x_gyroOffset, y_gyroOffset, z_gyroOffset))
    
    return;

  def getGyroOffsetEnabled(self):
    gyroStatus = self.reg_read_bits(self.BMI160_RA_OFFSET_6 , self.BMI160_GYR_OFFSET_EN, 1)
    print("GyroOffsetEnabled %x" %( gyroStatus ))
    return;

  def setGyroOffsetEnabled(self, enabled):
    self.reg_write_bits(self.BMI160_RA_OFFSET_6, enabled, self.BMI160_GYR_OFFSET_EN, 1)
    return;

  def show_accel_gyro(self):
    self.enable_accel()
    self.enable_gyro()

  # Zero Motion is detected when the difference between the value of
  # consecutive accelerometer measurements for each axis remains smaller than
  # this Motion detection threshold. This condition triggers the Zero Motion
  # interrupt if the condition is maintained for a time duration 
  # specified in the int_slo_no_mot_dur field of the INT_MOTION[0] register (@see
  # registers.INT_MOTION_0), and clears the interrupt when the condition is
  # then absent for the same duration.
  #
  # For more details on the Zero Motion detection interrupt, see Section 2.6.9 of
  # the BMI160 Data Sheet.
  #
  # @return Current zero motion detection acceleration threshold value
  # @see getZeroMotionDetectionDuration()
  # @see registers.INT_MOTION_2
  def getZeroMotionDetectionThreshold(self):
    return self._reg_read(constants.REG_INT_MOTION_2)

  ## Set zero motion detection event acceleration threshold.
  # @param threshold New zero motion detection acceleration threshold value
  # @see getZeroMotionDetectionThreshold()
  # @see registers.INT_MOTION_2
  def setZeroMotionDetectionThreshold(self, threshold):
    self._reg_write(constants.REG_INT_MOTION_2, threshold)

  ## Get zero motion detection event duration threshold.
  # This register configures the duration time for Zero Motion interrupt
  # generation. A time range between 1.28s and 430.08s can be selected, but the
  # granularity of the timing reduces as the duration increases:
  #
  # <pre>
  # Duration           | Granularity
  # -------------------+----------------
  # [1.28 - 20.48]s    |  1.28s
  # [25.6 - 102.4]s    |  5.12s
  # [112.64 - 430.08]s | 10.24s
  # </pre>
  #
  # The Zero Motion interrupt is triggered when the Zero Motion condition is
  # maintained for the duration specified in this register.
  #
  # For more details on the Zero Motion detection interrupt, see Section 2.6.9 of
  # the BMI160 Data Sheet.
  #
  # @return Current zero motion detection duration threshold value
  #         @see BMI160ZeroMotionDuration for a list of possible values
  # @see getZeroMotionDetectionThreshold()
  # @see registers.INT_MOTION_0
  # @see BMI160ZeroMotionDuration
  def getZeroMotionDetectionDuration(self):
    return self.reg_read_bits(constants.REG_INT_MOTION_0, constants.DEF_NOMOTION_DUR_BIT, constants.DEF_NOMOTION_DUR_LEN)

  ## Set zero motion detection event duration threshold.
  #
  # This must be called at least once to enable zero-motion detection.
  #
  # @param duration New zero motion detection duration threshold value
  #        @see BMI160ZeroMotionDuration for a list of valid values
  # @see getZeroMotionDetectionDuration()
  # @see registers.INT_MOTION_0
  # @see BMI160ZeroMotionDuration
  def setZeroMotionDetectionDuration(self, duration):
    self.reg_write_bits(constants.REG_INT_MOTION_0, duration, constants.DEF_NOMOTION_DUR_BIT, constants.DEF_NOMOTION_DUR_LEN)

  ## Get Zero Motion Detection interrupt status.
  # This bit automatically sets to 1 when a Zero Motion Detection condition
  # is present, and clears when the condition is no longer present.
  #
  # For more details on the Motion detection interrupt, see Section 2.6.9 of the
  # BMI160 Data Sheet.
  #
  # @return Current interrupt status
  # @see registers.INT_STATUS_1
  # @see definitions.NOMOTION_INT_BIT
  def getIntZeroMotionStatus(self):
    return 0 != (self.reg_read_bits(constants.REG_INT_STATUS_1, constants.DEF_NOMOTION_INT_BIT, 1))
        
  ## Get Zero Motion Detection interrupt enabled status.
  # Will be set 0 for disabled, 1 for enabled.
  # @return Current interrupt enabled status
  # @see registers.INT_EN_2
  # @see definitions.NOMOTION_EN_BIT
  #*/
  def getIntZeroMotionEnabled(self):
    return 0 != (self.reg_read_bits(constants.REG_INT_EN_2, constants.DEF_NOMOTION_EN_BIT, constants.DEF_NOMOTION_EN_LEN))

  ## Set Zero Motion Detection interrupt enabled status.
  # @param enabled New interrupt enabled status
  # @see getIntZeroMotionEnabled()
  # @see registers.INT_EN_2
  # @see definitions.NOMOTION_EN_BIT
  # @see registers.INT_MOTION_3
  #*/
  def setIntZeroMotionEnabled(self, enabled):
    if (enabled):
      # Select No-Motion detection mode
      self.reg_write_bits(constants.REG_INT_MOTION_3, 0x1, constants.DEF_NOMOTION_SEL_BIT, constants.DEF_NOMOTION_SEL_LEN)
    # Enable for all 3 axes
    self.reg_write_bits(constants.REG_INT_EN_2, 0x7 if enabled else 0x0, constants.DEF_NOMOTION_EN_BIT, constants.DEF_NOMOTION_EN_LEN)

  ## Set full-scale accelerometer range.
  # @param range New full-scale accelerometer range setting
  # @see getFullScaleAccelRange()
  # @see BMI160AccelRange
  def setFullScaleAccelRange(self, range, real):
    self.reg_write_bits(constants.REG_ACCEL_RANGE, range, constants.DEF_ACCEL_RANGE_SEL_BIT, constants.DEF_ACCEL_RANGE_SEL_LEN)
    self.accel_range = real

  ## Set full-scale gyroscope range.
  # @param range New full-scale gyroscope range value
  # @see getFullScaleGyroRange()
  def setFullScaleGyroRange(self, range, real):
    self.reg_write_bits(constants.REG_GYRO_RANGE, range, constants.DEF_GYRO_RANGE_SEL_BIT, constants.DEF_GYRO_RANGE_SEL_LEN)
    self.gyro_range = real

  ## Set accelerometer output data rate.
  # @param rate New output data rate
  # @see get_accel_rate()
  # @see registers.ACCEL_CONF
  def set_accel_rate(self, rate):
    self.reg_write_bits(constants.REG_ACCEL_CONF, rate, constants.DEF_ACCEL_RATE_SEL_BIT, constants.DEF_ACCEL_RATE_SEL_LEN)

  ## Set accelerometer digital low-pass filter configuration.
  # @param mode New DLFP configuration setting
  # @see getAccelDLPFMode()
  def setAccelDLPFMode(self, mode):
    return self.reg_write_bits(constants.REG_ACCEL_CONF, mode, constants.DEF_ACCEL_DLPF_SEL_BIT, constants.DEF_ACCEL_DLPF_SEL_LEN)

  def calibration(self):

    print("Starting Acceleration calibration and enabling offset compensation...")
    self.autoCalibrateXAccelOffset(0)
    self.autoCalibrateYAccelOffset(0)
    self.autoCalibrateZAccelOffset(1)
    self.getAccelOffset()
    print("Done")

    print("Starting Gyroscope calibration and enabling offset compensation...")
    self.autoCalibrateGyroOffset()
    self.getGyroOffset()
    print("Done")

    self.setGyroOffsetEnabled(1)
    self.setAccelOffsetEnabled(1)

  def get_accel_gyro (self, show = False):
    #acValues = self.read_accel()
    self.read_accel()
    self.read_gyro()

    if (show):
      print("Apply Platform Matrix")
      print("===============================================================")
      print("self.gyro x_raw = %d, y_raw = %d z_raw = %d" % (-self.gyro_x_raw, self.gyro_y_raw, -self.gyro_z_raw))
      print("self.gyro x = %d, y = %d z = %d" % (-self.gyro_x, self.gyro_y, -self.gyro_z))
      print("===============================================================")
      print("self.accel x_raw = %d, y_raw = %d z_raw = %d" % (-self.acc_x_raw, self.acc_y_raw, -self.acc_z_raw))
      print("self.accel x = %d, y = %d z = %d" % (-self.acc_x, self.acc_y, -self.acc_z))
      print("===============================================================")

    #return acValues;  
    return [self.acc_x,self.acc_y,self.acc_z,self.gyro_x,self.gyro_y,self.gyro_z];
    

  def _reg_write(self, reg, data):
    write = i2c_msg.write(self.BMI160_DEVICE_ADDRESS, bytes([reg, data]))
    self.bus.i2c_rdwr(write)

  def _reg_read(self, reg):
    return self._regs_read(reg, 1)[0]

  def _regs_read(self, reg, n):
    write = i2c_msg.write(self.BMI160_DEVICE_ADDRESS, [reg])
    sleep(0.000002)
    read = i2c_msg.read(self.BMI160_DEVICE_ADDRESS, n)
    self.bus.i2c_rdwr(write, read)
    result = list(read)
    #print('< ', result)
    return result

#imu = IMU_BMI160()
#print(imu.get_accel_gyro())

#show_accel_gyro()





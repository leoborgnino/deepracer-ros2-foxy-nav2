I2C_BUS_ID = 2 #1
BMI160_ADDR = 0x68

IMU_MSG_TOPIC = "data_raw"
ODOM_MSG_TOPIC = "odom_zero"
GET_MOTION_STATE_SERVICE_NAME = "/imu_pkg/get_motion_state_service"
IMU_MSG_RATE = 25
ACCEL_RANGE_4G_FLOAT = 4.0
ACCEL_RANGE_8G_FLOAT = 8.0
ACCEL_RANGE_16G_FLOAT = 16.0
GYRO_RANGE_250_FLOAT = 250.0
CONVERSION_MASK_16BIT_FLOAT = 0x8000
GRAVITY_CONSTANT = 9.80665
EMPTY_ARRAY_9 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
COVAR_ARRAY_9 = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
EMPTY_ARRAY_36 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

## DEFINITIONS ##
## bit field offsets and lengths
DEF_NOMOTION_INT_BIT    = 7
DEF_NOMOTION_EN_BIT     = 0
DEF_NOMOTION_EN_LEN     = 3 
DEF_NOMOTION_DUR_BIT    = 2
DEF_NOMOTION_DUR_LEN    = 6
DEF_NOMOTION_SEL_LEN    = 1
DEF_NOMOTION_SEL_BIT    = 0
DEF_ACC_PMU_STATUS_BIT  = 4
DEF_ACC_PMU_STATUS_LEN  = 2
DEF_GYR_PMU_STATUS_BIT  = 2
DEF_GYR_PMU_STATUS_LEN  = 2
DEF_GYRO_RANGE_SEL_BIT  = 0
DEF_GYRO_RANGE_SEL_LEN  = 3
DEF_GYRO_RATE_SEL_BIT   = 0
DEF_GYRO_RATE_SEL_LEN   = 4
DEF_GYRO_DLPF_SEL_BIT   = 4
DEF_GYRO_DLPF_SEL_LEN   = 2
DEF_ACCEL_DLPF_SEL_BIT  = 4
DEF_ACCEL_DLPF_SEL_LEN  = 3
DEF_ACCEL_RANGE_SEL_BIT = 0
DEF_ACCEL_RANGE_SEL_LEN = 4
DEF_ACCEL_RATE_SEL_BIT  = 0    # added
DEF_ACCEL_RATE_SEL_LEN  = 4    # added

## Gyroscope Sensitivity Range options
# see setFullScaleGyroRange()
DEF_GYRO_RANGE_2000     = 0    # +/- 2000 degrees/second
DEF_GYRO_RANGE_1000     = 1    # +/- 1000 degrees/second
DEF_GYRO_RANGE_500      = 2    # +/-  500 degrees/second
DEF_GYRO_RANGE_250      = 3    # +/-  250 degrees/second
DEF_GYRO_RANGE_125      = 4    # +/-  125 degrees/second

## Accelerometer Sensitivity Range options
# see setFullScaleAccelRange()
DEF_ACCEL_RANGE_2G      = 0X03 # +/-  2g range
DEF_ACCEL_RANGE_4G      = 0X05 # +/-  4g range
DEF_ACCEL_RANGE_8G      = 0X08 # +/-  8g range
DEF_ACCEL_RANGE_16G     = 0X0C # +/- 16g range

DEF_FOC_ACC_Z_BIT       = 0
DEF_FOC_ACC_Z_LEN       = 2
DEF_FOC_ACC_Y_BIT       = 2
DEF_FOC_ACC_Y_LEN       = 2
DEF_FOC_ACC_X_BIT       = 4
DEF_FOC_ACC_X_LEN       = 2
DEF_FOC_GYR_EN          = 6

## COMMANDS ##
# command definitions
COMM_START_FOC       = 0x03
COMM_ACC_MODE_NORMAL = 0x11
COMM_GYR_MODE_NORMAL = 0x15
COMM_FIFO_FLUSH      = 0xB0
COMM_INT_RESET       = 0xB1
COMM_STEP_CNT_CLR    = 0xB2
COMM_SOFT_RESET      = 0xB6

## REGISTERS ##
# register definitions
REG_CHIP_ID           = 0x00
REG_PMU_STATUS        = 0x03
REG_GYRO_X_L          = 0x0C
REG_GYRO_X_H          = 0x0D
REG_GYRO_Y_L          = 0x0E
REG_GYRO_Y_H          = 0x0F
REG_GYRO_Z_L          = 0x10
REG_GYRO_Z_H          = 0x11
REG_ACCEL_X_L         = 0x12
REG_ACCEL_X_H         = 0x13
REG_ACCEL_Y_L         = 0x14
REG_ACCEL_Y_H         = 0x15
REG_ACCEL_Z_L         = 0x16
REG_ACCEL_Z_H         = 0x17
REG_STATUS            = 0x1B
REG_INT_STATUS_0      = 0x1C
REG_INT_STATUS_1      = 0x1D
REG_INT_STATUS_2      = 0x1E
REG_INT_STATUS_3      = 0x1F
REG_TEMP_L            = 0x20
REG_TEMP_H            = 0x21
REG_FIFO_LENGTH_0     = 0x22
REG_FIFO_LENGTH_1     = 0x23
REG_FIFO_DATA         = 0x24
REG_ACCEL_CONF        = 0X40
REG_ACCEL_RANGE       = 0X41
REG_GYRO_CONF         = 0X42
REG_GYRO_RANGE        = 0X43
REG_FIFO_CONFIG_0     = 0x46
REG_FIFO_CONFIG_1     = 0x47
REG_INT_EN_0          = 0x50
REG_INT_EN_1          = 0x51
REG_INT_EN_2          = 0x52
REG_INT_OUT_CTRL      = 0x53
REG_INT_LATCH         = 0x54
REG_INT_MAP_0         = 0x55
REG_INT_MAP_1         = 0x56
REG_INT_MAP_2         = 0x57
REG_INT_LOWHIGH_0     = 0x5A
REG_INT_LOWHIGH_1     = 0x5B
REG_INT_LOWHIGH_2     = 0x5C
REG_INT_LOWHIGH_3     = 0x5D
REG_INT_LOWHIGH_4     = 0x5E
REG_INT_MOTION_0      = 0x5F
REG_INT_MOTION_1      = 0x60
REG_INT_MOTION_2      = 0x61
REG_INT_MOTION_3      = 0x62
REG_INT_TAP_0         = 0x63
REG_INT_TAP_1         = 0x64
REG_FOC_CONF          = 0x69
REG_OFFSET_0          = 0x71
REG_OFFSET_1          = 0x72
REG_OFFSET_2          = 0x73
REG_OFFSET_3          = 0x74
REG_OFFSET_4          = 0x75
REG_OFFSET_5          = 0x76
REG_OFFSET_6          = 0x77
REG_STEP_CNT_L        = 0x78
REG_STEP_CNT_H        = 0x79
REG_STEP_CONF_0       = 0x7A
REG_STEP_CONF_1       = 0x7B
REG_STEP_CONF_0_NOR   = 0x15
REG_STEP_CONF_0_SEN   = 0x2D
REG_STEP_CONF_0_ROB   = 0x1D
REG_STEP_CONF_1_NOR   = 0x03
REG_STEP_CONF_1_SEN   = 0x00
REG_STEP_CONF_1_ROB   = 0x07
REG_CMD               = 0x7E

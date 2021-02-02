//////////////////////////////////////////////////
//
// NOTE: do not edit,
// this file created automatically by mavgen.go
//
//////////////////////////////////////////////////

package ualberta

import (
	"encoding/binary"
	"fmt"
	mavlink "github.com/asmyasnikov/go-mavlink/generated/mavlink2"
	"math"
)

// UALBERTA_AUTOPILOT_MODE type. Available autopilot modes for ualberta uav
type UALBERTA_AUTOPILOT_MODE int

const (
	// MODE_MANUAL_DIRECT enum. Raw input pulse widts sent to output
	MODE_MANUAL_DIRECT UALBERTA_AUTOPILOT_MODE = 1
	// MODE_MANUAL_SCALED enum. Inputs are normalized using calibration, the converted back to raw pulse widths for output
	MODE_MANUAL_SCALED UALBERTA_AUTOPILOT_MODE = 2
	// MODE_AUTO_PID_ATT enum. dfsdfs
	MODE_AUTO_PID_ATT UALBERTA_AUTOPILOT_MODE = 3
	// MODE_AUTO_PID_VEL enum. dfsfds
	MODE_AUTO_PID_VEL UALBERTA_AUTOPILOT_MODE = 4
	// MODE_AUTO_PID_POS enum. dfsdfsdfs
	MODE_AUTO_PID_POS UALBERTA_AUTOPILOT_MODE = 5
)

// UALBERTA_NAV_MODE type. Navigation filter mode
type UALBERTA_NAV_MODE int

const (
	// NAV_AHRS_INIT enum
	NAV_AHRS_INIT UALBERTA_NAV_MODE = 1
	// NAV_AHRS enum. AHRS mode
	NAV_AHRS UALBERTA_NAV_MODE = 2
	// NAV_INS_GPS_INIT enum. INS/GPS initialization mode
	NAV_INS_GPS_INIT UALBERTA_NAV_MODE = 3
	// NAV_INS_GPS enum. INS/GPS mode
	NAV_INS_GPS UALBERTA_NAV_MODE = 4
)

// UALBERTA_PILOT_MODE type. Mode currently commanded by pilot
type UALBERTA_PILOT_MODE int

const (
	// PILOT_MANUAL enum. sdf
	PILOT_MANUAL UALBERTA_PILOT_MODE = 1
	// PILOT_AUTO enum. dfs
	PILOT_AUTO UALBERTA_PILOT_MODE = 2
	// PILOT_ROTO enum. Rotomotion mode
	PILOT_ROTO UALBERTA_PILOT_MODE = 3
)

// FIRMWARE_VERSION_TYPE type. These values define the type of firmware release.  These values indicate the first version or release of this type.  For example the first alpha release would be 64, the second would be 65.
type FIRMWARE_VERSION_TYPE int

const (
	// FIRMWARE_VERSION_TYPE_DEV enum. development release
	FIRMWARE_VERSION_TYPE_DEV FIRMWARE_VERSION_TYPE = 0
	// FIRMWARE_VERSION_TYPE_ALPHA enum. alpha release
	FIRMWARE_VERSION_TYPE_ALPHA FIRMWARE_VERSION_TYPE = 64
	// FIRMWARE_VERSION_TYPE_BETA enum. beta release
	FIRMWARE_VERSION_TYPE_BETA FIRMWARE_VERSION_TYPE = 128
	// FIRMWARE_VERSION_TYPE_RC enum. release candidate
	FIRMWARE_VERSION_TYPE_RC FIRMWARE_VERSION_TYPE = 192
	// FIRMWARE_VERSION_TYPE_OFFICIAL enum. official stable release
	FIRMWARE_VERSION_TYPE_OFFICIAL FIRMWARE_VERSION_TYPE = 255
)

// HL_FAILURE_FLAG type. Flags to report failure cases over the high latency telemtry.
type HL_FAILURE_FLAG int

const (
	// HL_FAILURE_FLAG_GPS enum. GPS failure
	HL_FAILURE_FLAG_GPS HL_FAILURE_FLAG = 1
	// HL_FAILURE_FLAG_DIFFERENTIAL_PRESSURE enum. Differential pressure sensor failure
	HL_FAILURE_FLAG_DIFFERENTIAL_PRESSURE HL_FAILURE_FLAG = 2
	// HL_FAILURE_FLAG_ABSOLUTE_PRESSURE enum. Absolute pressure sensor failure
	HL_FAILURE_FLAG_ABSOLUTE_PRESSURE HL_FAILURE_FLAG = 4
	// HL_FAILURE_FLAG_3D_ACCEL enum. Accelerometer sensor failure
	HL_FAILURE_FLAG_3D_ACCEL HL_FAILURE_FLAG = 8
	// HL_FAILURE_FLAG_3D_GYRO enum. Gyroscope sensor failure
	HL_FAILURE_FLAG_3D_GYRO HL_FAILURE_FLAG = 16
	// HL_FAILURE_FLAG_3D_MAG enum. Magnetometer sensor failure
	HL_FAILURE_FLAG_3D_MAG HL_FAILURE_FLAG = 32
	// HL_FAILURE_FLAG_TERRAIN enum. Terrain subsystem failure
	HL_FAILURE_FLAG_TERRAIN HL_FAILURE_FLAG = 64
	// HL_FAILURE_FLAG_BATTERY enum. Battery failure/critical low battery
	HL_FAILURE_FLAG_BATTERY HL_FAILURE_FLAG = 128
	// HL_FAILURE_FLAG_RC_RECEIVER enum. RC receiver failure/no rc connection
	HL_FAILURE_FLAG_RC_RECEIVER HL_FAILURE_FLAG = 256
	// HL_FAILURE_FLAG_OFFBOARD_LINK enum. Offboard link failure
	HL_FAILURE_FLAG_OFFBOARD_LINK HL_FAILURE_FLAG = 512
	// HL_FAILURE_FLAG_ENGINE enum. Engine failure
	HL_FAILURE_FLAG_ENGINE HL_FAILURE_FLAG = 1024
	// HL_FAILURE_FLAG_GEOFENCE enum. Geofence violation
	HL_FAILURE_FLAG_GEOFENCE HL_FAILURE_FLAG = 2048
	// HL_FAILURE_FLAG_ESTIMATOR enum. Estimator failure, for example measurement rejection or large variances
	HL_FAILURE_FLAG_ESTIMATOR HL_FAILURE_FLAG = 4096
	// HL_FAILURE_FLAG_MISSION enum. Mission failure
	HL_FAILURE_FLAG_MISSION HL_FAILURE_FLAG = 8192
)

// MAV_GOTO type. Actions that may be specified in MAV_CMD_OVERRIDE_GOTO to override mission execution.
type MAV_GOTO int

const (
	// MAV_GOTO_DO_HOLD enum. Hold at the current position
	MAV_GOTO_DO_HOLD MAV_GOTO = 0
	// MAV_GOTO_DO_CONTINUE enum. Continue with the next item in mission execution
	MAV_GOTO_DO_CONTINUE MAV_GOTO = 1
	// MAV_GOTO_HOLD_AT_CURRENT_POSITION enum. Hold at the current position of the system
	MAV_GOTO_HOLD_AT_CURRENT_POSITION MAV_GOTO = 2
	// MAV_GOTO_HOLD_AT_SPECIFIED_POSITION enum. Hold at the position specified in the parameters of the DO_HOLD action
	MAV_GOTO_HOLD_AT_SPECIFIED_POSITION MAV_GOTO = 3
)

// MAV_MODE type. These defines are predefined OR-combined mode flags. There is no need to use values from this enum, but it                simplifies the use of the mode flags. Note that manual input is enabled in all modes as a safety override.
type MAV_MODE int

const (
	// MAV_MODE_PREFLIGHT enum. System is not ready to fly, booting, calibrating, etc. No flag is set
	MAV_MODE_PREFLIGHT MAV_MODE = 0
	// MAV_MODE_STABILIZE_DISARMED enum. System is allowed to be active, under assisted RC control
	MAV_MODE_STABILIZE_DISARMED MAV_MODE = 80
	// MAV_MODE_STABILIZE_ARMED enum. System is allowed to be active, under assisted RC control
	MAV_MODE_STABILIZE_ARMED MAV_MODE = 208
	// MAV_MODE_MANUAL_DISARMED enum. System is allowed to be active, under manual (RC) control, no stabilization
	MAV_MODE_MANUAL_DISARMED MAV_MODE = 64
	// MAV_MODE_MANUAL_ARMED enum. System is allowed to be active, under manual (RC) control, no stabilization
	MAV_MODE_MANUAL_ARMED MAV_MODE = 192
	// MAV_MODE_GUIDED_DISARMED enum. System is allowed to be active, under autonomous control, manual setpoint
	MAV_MODE_GUIDED_DISARMED MAV_MODE = 88
	// MAV_MODE_GUIDED_ARMED enum. System is allowed to be active, under autonomous control, manual setpoint
	MAV_MODE_GUIDED_ARMED MAV_MODE = 216
	// MAV_MODE_AUTO_DISARMED enum. System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard and not pre-programmed by waypoints)
	MAV_MODE_AUTO_DISARMED MAV_MODE = 92
	// MAV_MODE_AUTO_ARMED enum. System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard and not pre-programmed by waypoints)
	MAV_MODE_AUTO_ARMED MAV_MODE = 220
	// MAV_MODE_TEST_DISARMED enum. UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only
	MAV_MODE_TEST_DISARMED MAV_MODE = 66
	// MAV_MODE_TEST_ARMED enum. UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only
	MAV_MODE_TEST_ARMED MAV_MODE = 194
)

// MAV_SYS_STATUS_SENSOR type. These encode the sensors whose status is sent as part of the SYS_STATUS message.
type MAV_SYS_STATUS_SENSOR int

const (
	// MAV_SYS_STATUS_SENSOR_3D_GYRO enum. 0x01 3D gyro
	MAV_SYS_STATUS_SENSOR_3D_GYRO MAV_SYS_STATUS_SENSOR = 1
	// MAV_SYS_STATUS_SENSOR_3D_ACCEL enum. 0x02 3D accelerometer
	MAV_SYS_STATUS_SENSOR_3D_ACCEL MAV_SYS_STATUS_SENSOR = 2
	// MAV_SYS_STATUS_SENSOR_3D_MAG enum. 0x04 3D magnetometer
	MAV_SYS_STATUS_SENSOR_3D_MAG MAV_SYS_STATUS_SENSOR = 4
	// MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE enum. 0x08 absolute pressure
	MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE MAV_SYS_STATUS_SENSOR = 8
	// MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE enum. 0x10 differential pressure
	MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE MAV_SYS_STATUS_SENSOR = 16
	// MAV_SYS_STATUS_SENSOR_GPS enum. 0x20 GPS
	MAV_SYS_STATUS_SENSOR_GPS MAV_SYS_STATUS_SENSOR = 32
	// MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW enum. 0x40 optical flow
	MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW MAV_SYS_STATUS_SENSOR = 64
	// MAV_SYS_STATUS_SENSOR_VISION_POSITION enum. 0x80 computer vision position
	MAV_SYS_STATUS_SENSOR_VISION_POSITION MAV_SYS_STATUS_SENSOR = 128
	// MAV_SYS_STATUS_SENSOR_LASER_POSITION enum. 0x100 laser based position
	MAV_SYS_STATUS_SENSOR_LASER_POSITION MAV_SYS_STATUS_SENSOR = 256
	// MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH enum. 0x200 external ground truth (Vicon or Leica)
	MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH MAV_SYS_STATUS_SENSOR = 512
	// MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL enum. 0x400 3D angular rate control
	MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL MAV_SYS_STATUS_SENSOR = 1024
	// MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION enum. 0x800 attitude stabilization
	MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION MAV_SYS_STATUS_SENSOR = 2048
	// MAV_SYS_STATUS_SENSOR_YAW_POSITION enum. 0x1000 yaw position
	MAV_SYS_STATUS_SENSOR_YAW_POSITION MAV_SYS_STATUS_SENSOR = 4096
	// MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL enum. 0x2000 z/altitude control
	MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL MAV_SYS_STATUS_SENSOR = 8192
	// MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL enum. 0x4000 x/y position control
	MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL MAV_SYS_STATUS_SENSOR = 16384
	// MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS enum. 0x8000 motor outputs / control
	MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS MAV_SYS_STATUS_SENSOR = 32768
	// MAV_SYS_STATUS_SENSOR_RC_RECEIVER enum. 0x10000 rc receiver
	MAV_SYS_STATUS_SENSOR_RC_RECEIVER MAV_SYS_STATUS_SENSOR = 65536
	// MAV_SYS_STATUS_SENSOR_3D_GYRO2 enum. 0x20000 2nd 3D gyro
	MAV_SYS_STATUS_SENSOR_3D_GYRO2 MAV_SYS_STATUS_SENSOR = 131072
	// MAV_SYS_STATUS_SENSOR_3D_ACCEL2 enum. 0x40000 2nd 3D accelerometer
	MAV_SYS_STATUS_SENSOR_3D_ACCEL2 MAV_SYS_STATUS_SENSOR = 262144
	// MAV_SYS_STATUS_SENSOR_3D_MAG2 enum. 0x80000 2nd 3D magnetometer
	MAV_SYS_STATUS_SENSOR_3D_MAG2 MAV_SYS_STATUS_SENSOR = 524288
	// MAV_SYS_STATUS_GEOFENCE enum. 0x100000 geofence
	MAV_SYS_STATUS_GEOFENCE MAV_SYS_STATUS_SENSOR = 1048576
	// MAV_SYS_STATUS_AHRS enum. 0x200000 AHRS subsystem health
	MAV_SYS_STATUS_AHRS MAV_SYS_STATUS_SENSOR = 2097152
	// MAV_SYS_STATUS_TERRAIN enum. 0x400000 Terrain subsystem health
	MAV_SYS_STATUS_TERRAIN MAV_SYS_STATUS_SENSOR = 4194304
	// MAV_SYS_STATUS_REVERSE_MOTOR enum. 0x800000 Motors are reversed
	MAV_SYS_STATUS_REVERSE_MOTOR MAV_SYS_STATUS_SENSOR = 8388608
	// MAV_SYS_STATUS_LOGGING enum. 0x1000000 Logging
	MAV_SYS_STATUS_LOGGING MAV_SYS_STATUS_SENSOR = 16777216
	// MAV_SYS_STATUS_SENSOR_BATTERY enum. 0x2000000 Battery
	MAV_SYS_STATUS_SENSOR_BATTERY MAV_SYS_STATUS_SENSOR = 33554432
	// MAV_SYS_STATUS_SENSOR_PROXIMITY enum. 0x4000000 Proximity
	MAV_SYS_STATUS_SENSOR_PROXIMITY MAV_SYS_STATUS_SENSOR = 67108864
	// MAV_SYS_STATUS_SENSOR_SATCOM enum. 0x8000000 Satellite Communication
	MAV_SYS_STATUS_SENSOR_SATCOM MAV_SYS_STATUS_SENSOR = 134217728
	// MAV_SYS_STATUS_PREARM_CHECK enum. 0x10000000 pre-arm check status. Always healthy when armed
	MAV_SYS_STATUS_PREARM_CHECK MAV_SYS_STATUS_SENSOR = 268435456
	// MAV_SYS_STATUS_OBSTACLE_AVOIDANCE enum. 0x20000000 Avoidance/collision prevention
	MAV_SYS_STATUS_OBSTACLE_AVOIDANCE MAV_SYS_STATUS_SENSOR = 536870912
)

// MAV_FRAME type
type MAV_FRAME int

const (
	// MAV_FRAME_GLOBAL enum. Global (WGS84) coordinate frame + MSL altitude. First value / x: latitude, second value / y: longitude, third value / z: positive altitude over mean sea level (MSL)
	MAV_FRAME_GLOBAL MAV_FRAME = 0
	// MAV_FRAME_LOCAL_NED enum. Local coordinate frame, Z-down (x: North, y: East, z: Down)
	MAV_FRAME_LOCAL_NED MAV_FRAME = 1
	// MAV_FRAME_MISSION enum. NOT a coordinate frame, indicates a mission command
	MAV_FRAME_MISSION MAV_FRAME = 2
	// MAV_FRAME_GLOBAL_RELATIVE_ALT enum. Global (WGS84) coordinate frame + altitude relative to the home position. First value / x: latitude, second value / y: longitude, third value / z: positive altitude with 0 being at the altitude of the home location
	MAV_FRAME_GLOBAL_RELATIVE_ALT MAV_FRAME = 3
	// MAV_FRAME_LOCAL_ENU enum. Local coordinate frame, Z-up (x: East, y: North, z: Up)
	MAV_FRAME_LOCAL_ENU MAV_FRAME = 4
	// MAV_FRAME_GLOBAL_INT enum. Global (WGS84) coordinate frame (scaled) + MSL altitude. First value / x: latitude in degrees*1.0e-7, second value / y: longitude in degrees*1.0e-7, third value / z: positive altitude over mean sea level (MSL)
	MAV_FRAME_GLOBAL_INT MAV_FRAME = 5
	// MAV_FRAME_GLOBAL_RELATIVE_ALT_INT enum. Global (WGS84) coordinate frame (scaled) + altitude relative to the home position. First value / x: latitude in degrees*10e-7, second value / y: longitude in degrees*10e-7, third value / z: positive altitude with 0 being at the altitude of the home location
	MAV_FRAME_GLOBAL_RELATIVE_ALT_INT MAV_FRAME = 6
	// MAV_FRAME_LOCAL_OFFSET_NED enum. Offset to the current local frame. Anything expressed in this frame should be added to the current local frame position
	MAV_FRAME_LOCAL_OFFSET_NED MAV_FRAME = 7
	// MAV_FRAME_BODY_NED enum. Setpoint in body NED frame. This makes sense if all position control is externalized - e.g. useful to command 2 m/s^2 acceleration to the right
	MAV_FRAME_BODY_NED MAV_FRAME = 8
	// MAV_FRAME_BODY_OFFSET_NED enum. Offset in body NED frame. This makes sense if adding setpoints to the current flight path, to avoid an obstacle - e.g. useful to command 2 m/s^2 acceleration to the east
	MAV_FRAME_BODY_OFFSET_NED MAV_FRAME = 9
	// MAV_FRAME_GLOBAL_TERRAIN_ALT enum. Global (WGS84) coordinate frame with AGL altitude (at the waypoint coordinate). First value / x: latitude in degrees, second value / y: longitude in degrees, third value / z: positive altitude in meters with 0 being at ground level in terrain model
	MAV_FRAME_GLOBAL_TERRAIN_ALT MAV_FRAME = 10
	// MAV_FRAME_GLOBAL_TERRAIN_ALT_INT enum. Global (WGS84) coordinate frame (scaled) with AGL altitude (at the waypoint coordinate). First value / x: latitude in degrees*10e-7, second value / y: longitude in degrees*10e-7, third value / z: positive altitude in meters with 0 being at ground level in terrain model
	MAV_FRAME_GLOBAL_TERRAIN_ALT_INT MAV_FRAME = 11
	// MAV_FRAME_BODY_FRD enum. Body fixed frame of reference, Z-down (x: Forward, y: Right, z: Down)
	MAV_FRAME_BODY_FRD MAV_FRAME = 12
	// MAV_FRAME_RESERVED_13 enum. MAV_FRAME_BODY_FLU - Body fixed frame of reference, Z-up (x: Forward, y: Left, z: Up)
	MAV_FRAME_RESERVED_13 MAV_FRAME = 13
	// MAV_FRAME_RESERVED_14 enum. MAV_FRAME_MOCAP_NED - Odometry local coordinate frame of data given by a motion capture system, Z-down (x: North, y: East, z: Down)
	MAV_FRAME_RESERVED_14 MAV_FRAME = 14
	// MAV_FRAME_RESERVED_15 enum. MAV_FRAME_MOCAP_ENU - Odometry local coordinate frame of data given by a motion capture system, Z-up (x: East, y: North, z: Up)
	MAV_FRAME_RESERVED_15 MAV_FRAME = 15
	// MAV_FRAME_RESERVED_16 enum. MAV_FRAME_VISION_NED - Odometry local coordinate frame of data given by a vision estimation system, Z-down (x: North, y: East, z: Down)
	MAV_FRAME_RESERVED_16 MAV_FRAME = 16
	// MAV_FRAME_RESERVED_17 enum. MAV_FRAME_VISION_ENU - Odometry local coordinate frame of data given by a vision estimation system, Z-up (x: East, y: North, z: Up)
	MAV_FRAME_RESERVED_17 MAV_FRAME = 17
	// MAV_FRAME_RESERVED_18 enum. MAV_FRAME_ESTIM_NED - Odometry local coordinate frame of data given by an estimator running onboard the vehicle, Z-down (x: North, y: East, z: Down)
	MAV_FRAME_RESERVED_18 MAV_FRAME = 18
	// MAV_FRAME_RESERVED_19 enum. MAV_FRAME_ESTIM_ENU - Odometry local coordinate frame of data given by an estimator running onboard the vehicle, Z-up (x: East, y: North, z: Up)
	MAV_FRAME_RESERVED_19 MAV_FRAME = 19
	// MAV_FRAME_LOCAL_FRD enum. Forward, Right, Down coordinate frame. This is a local frame with Z-down and arbitrary F/R alignment (i.e. not aligned with NED/earth frame)
	MAV_FRAME_LOCAL_FRD MAV_FRAME = 20
	// MAV_FRAME_LOCAL_FLU enum. Forward, Left, Up coordinate frame. This is a local frame with Z-up and arbitrary F/L alignment (i.e. not aligned with ENU/earth frame)
	MAV_FRAME_LOCAL_FLU MAV_FRAME = 21
)

// MAVLINK_DATA_STREAM_TYPE type
type MAVLINK_DATA_STREAM_TYPE int

const (
	// MAVLINK_DATA_STREAM_IMG_JPEG enum
	MAVLINK_DATA_STREAM_IMG_JPEG MAVLINK_DATA_STREAM_TYPE = 0
	// MAVLINK_DATA_STREAM_IMG_BMP enum
	MAVLINK_DATA_STREAM_IMG_BMP MAVLINK_DATA_STREAM_TYPE = 1
	// MAVLINK_DATA_STREAM_IMG_RAW8U enum
	MAVLINK_DATA_STREAM_IMG_RAW8U MAVLINK_DATA_STREAM_TYPE = 2
	// MAVLINK_DATA_STREAM_IMG_RAW32U enum
	MAVLINK_DATA_STREAM_IMG_RAW32U MAVLINK_DATA_STREAM_TYPE = 3
	// MAVLINK_DATA_STREAM_IMG_PGM enum
	MAVLINK_DATA_STREAM_IMG_PGM MAVLINK_DATA_STREAM_TYPE = 4
	// MAVLINK_DATA_STREAM_IMG_PNG enum
	MAVLINK_DATA_STREAM_IMG_PNG MAVLINK_DATA_STREAM_TYPE = 5
)

// FENCE_ACTION type
type FENCE_ACTION int

const (
	// FENCE_ACTION_NONE enum. Disable fenced mode
	FENCE_ACTION_NONE FENCE_ACTION = 0
	// FENCE_ACTION_GUIDED enum. Switched to guided mode to return point (fence point 0)
	FENCE_ACTION_GUIDED FENCE_ACTION = 1
	// FENCE_ACTION_REPORT enum. Report fence breach, but don't take action
	FENCE_ACTION_REPORT FENCE_ACTION = 2
	// FENCE_ACTION_GUIDED_THR_PASS enum. Switched to guided mode to return point (fence point 0) with manual throttle control
	FENCE_ACTION_GUIDED_THR_PASS FENCE_ACTION = 3
	// FENCE_ACTION_RTL enum. Switch to RTL (return to launch) mode and head for the return point
	FENCE_ACTION_RTL FENCE_ACTION = 4
)

// FENCE_BREACH type
type FENCE_BREACH int

const (
	// FENCE_BREACH_NONE enum. No last fence breach
	FENCE_BREACH_NONE FENCE_BREACH = 0
	// FENCE_BREACH_MINALT enum. Breached minimum altitude
	FENCE_BREACH_MINALT FENCE_BREACH = 1
	// FENCE_BREACH_MAXALT enum. Breached maximum altitude
	FENCE_BREACH_MAXALT FENCE_BREACH = 2
	// FENCE_BREACH_BOUNDARY enum. Breached fence boundary
	FENCE_BREACH_BOUNDARY FENCE_BREACH = 3
)

// FENCE_MITIGATE type. Actions being taken to mitigate/prevent fence breach
type FENCE_MITIGATE int

const (
	// FENCE_MITIGATE_UNKNOWN enum. Unknown
	FENCE_MITIGATE_UNKNOWN FENCE_MITIGATE = 0
	// FENCE_MITIGATE_NONE enum. No actions being taken
	FENCE_MITIGATE_NONE FENCE_MITIGATE = 1
	// FENCE_MITIGATE_VEL_LIMIT enum. Velocity limiting active to prevent breach
	FENCE_MITIGATE_VEL_LIMIT FENCE_MITIGATE = 2
)

// MAV_MOUNT_MODE type. Enumeration of possible mount operation modes. This message is used by obsolete/deprecated gimbal messages.
type MAV_MOUNT_MODE int

const (
	// MAV_MOUNT_MODE_RETRACT enum. Load and keep safe position (Roll,Pitch,Yaw) from permant memory and stop stabilization
	MAV_MOUNT_MODE_RETRACT MAV_MOUNT_MODE = 0
	// MAV_MOUNT_MODE_NEUTRAL enum. Load and keep neutral position (Roll,Pitch,Yaw) from permanent memory
	MAV_MOUNT_MODE_NEUTRAL MAV_MOUNT_MODE = 1
	// MAV_MOUNT_MODE_MAVLINK_TARGETING enum. Load neutral position and start MAVLink Roll,Pitch,Yaw control with stabilization
	MAV_MOUNT_MODE_MAVLINK_TARGETING MAV_MOUNT_MODE = 2
	// MAV_MOUNT_MODE_RC_TARGETING enum. Load neutral position and start RC Roll,Pitch,Yaw control with stabilization
	MAV_MOUNT_MODE_RC_TARGETING MAV_MOUNT_MODE = 3
	// MAV_MOUNT_MODE_GPS_POINT enum. Load neutral position and start to point to Lat,Lon,Alt
	MAV_MOUNT_MODE_GPS_POINT MAV_MOUNT_MODE = 4
	// MAV_MOUNT_MODE_SYSID_TARGET enum. Gimbal tracks system with specified system ID
	MAV_MOUNT_MODE_SYSID_TARGET MAV_MOUNT_MODE = 5
	// MAV_MOUNT_MODE_HOME_LOCATION enum. Gimbal tracks home location
	MAV_MOUNT_MODE_HOME_LOCATION MAV_MOUNT_MODE = 6
)

// GIMBAL_DEVICE_CAP_FLAGS type. Gimbal device (low level) capability flags (bitmap)
type GIMBAL_DEVICE_CAP_FLAGS int

const (
	// GIMBAL_DEVICE_CAP_FLAGS_HAS_RETRACT enum. Gimbal device supports a retracted position
	GIMBAL_DEVICE_CAP_FLAGS_HAS_RETRACT GIMBAL_DEVICE_CAP_FLAGS = 1
	// GIMBAL_DEVICE_CAP_FLAGS_HAS_NEUTRAL enum. Gimbal device supports a horizontal, forward looking position, stabilized
	GIMBAL_DEVICE_CAP_FLAGS_HAS_NEUTRAL GIMBAL_DEVICE_CAP_FLAGS = 2
	// GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_AXIS enum. Gimbal device supports rotating around roll axis
	GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_AXIS GIMBAL_DEVICE_CAP_FLAGS = 4
	// GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_FOLLOW enum. Gimbal device supports to follow a roll angle relative to the vehicle
	GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_FOLLOW GIMBAL_DEVICE_CAP_FLAGS = 8
	// GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_LOCK enum. Gimbal device supports locking to an roll angle (generally that's the default with roll stabilized)
	GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_LOCK GIMBAL_DEVICE_CAP_FLAGS = 16
	// GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS enum. Gimbal device supports rotating around pitch axis
	GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS GIMBAL_DEVICE_CAP_FLAGS = 32
	// GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_FOLLOW enum. Gimbal device supports to follow a pitch angle relative to the vehicle
	GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_FOLLOW GIMBAL_DEVICE_CAP_FLAGS = 64
	// GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_LOCK enum. Gimbal device supports locking to an pitch angle (generally that's the default with pitch stabilized)
	GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_LOCK GIMBAL_DEVICE_CAP_FLAGS = 128
	// GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_AXIS enum. Gimbal device supports rotating around yaw axis
	GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_AXIS GIMBAL_DEVICE_CAP_FLAGS = 256
	// GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_FOLLOW enum. Gimbal device supports to follow a yaw angle relative to the vehicle (generally that's the default)
	GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_FOLLOW GIMBAL_DEVICE_CAP_FLAGS = 512
	// GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_LOCK enum. Gimbal device supports locking to an absolute heading (often this is an option available)
	GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_LOCK GIMBAL_DEVICE_CAP_FLAGS = 1024
	// GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_INFINITE_YAW enum. Gimbal device supports yawing/panning infinetely (e.g. using slip disk)
	GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_INFINITE_YAW GIMBAL_DEVICE_CAP_FLAGS = 2048
)

// GIMBAL_MANAGER_CAP_FLAGS type. Gimbal manager high level capability flags (bitmap). The first 16 bits are identical to the GIMBAL_DEVICE_CAP_FLAGS which are identical with GIMBAL_DEVICE_FLAGS. However, the gimbal manager does not need to copy the flags from the gimbal but can also enhance the capabilities and thus add flags.
type GIMBAL_MANAGER_CAP_FLAGS int

const (
	// GIMBAL_MANAGER_CAP_FLAGS_HAS_RETRACT enum. Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_RETRACT
	GIMBAL_MANAGER_CAP_FLAGS_HAS_RETRACT GIMBAL_MANAGER_CAP_FLAGS = 1
	// GIMBAL_MANAGER_CAP_FLAGS_HAS_NEUTRAL enum. Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_NEUTRAL
	GIMBAL_MANAGER_CAP_FLAGS_HAS_NEUTRAL GIMBAL_MANAGER_CAP_FLAGS = 2
	// GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_AXIS enum. Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_AXIS
	GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_AXIS GIMBAL_MANAGER_CAP_FLAGS = 4
	// GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_FOLLOW enum. Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_FOLLOW
	GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_FOLLOW GIMBAL_MANAGER_CAP_FLAGS = 8
	// GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_LOCK enum. Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_LOCK
	GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_LOCK GIMBAL_MANAGER_CAP_FLAGS = 16
	// GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_AXIS enum. Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS
	GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_AXIS GIMBAL_MANAGER_CAP_FLAGS = 32
	// GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_FOLLOW enum. Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_FOLLOW
	GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_FOLLOW GIMBAL_MANAGER_CAP_FLAGS = 64
	// GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_LOCK enum. Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_LOCK
	GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_LOCK GIMBAL_MANAGER_CAP_FLAGS = 128
	// GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_AXIS enum. Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_AXIS
	GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_AXIS GIMBAL_MANAGER_CAP_FLAGS = 256
	// GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_FOLLOW enum. Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_FOLLOW
	GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_FOLLOW GIMBAL_MANAGER_CAP_FLAGS = 512
	// GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_LOCK enum. Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_LOCK
	GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_LOCK GIMBAL_MANAGER_CAP_FLAGS = 1024
	// GIMBAL_MANAGER_CAP_FLAGS_SUPPORTS_INFINITE_YAW enum. Based on GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_INFINITE_YAW
	GIMBAL_MANAGER_CAP_FLAGS_SUPPORTS_INFINITE_YAW GIMBAL_MANAGER_CAP_FLAGS = 2048
	// GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_LOCAL enum. Gimbal manager supports to point to a local position
	GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_LOCAL GIMBAL_MANAGER_CAP_FLAGS = 65536
	// GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_GLOBAL enum. Gimbal manager supports to point to a global latitude, longitude, altitude position
	GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_GLOBAL GIMBAL_MANAGER_CAP_FLAGS = 131072
)

// GIMBAL_DEVICE_FLAGS type. Flags for gimbal device (lower level) operation.
type GIMBAL_DEVICE_FLAGS int

const (
	// GIMBAL_DEVICE_FLAGS_RETRACT enum. Set to retracted safe position (no stabilization), takes presedence over all other flags
	GIMBAL_DEVICE_FLAGS_RETRACT GIMBAL_DEVICE_FLAGS = 1
	// GIMBAL_DEVICE_FLAGS_NEUTRAL enum. Set to neutral position (horizontal, forward looking, with stabiliziation), takes presedence over all other flags except RETRACT
	GIMBAL_DEVICE_FLAGS_NEUTRAL GIMBAL_DEVICE_FLAGS = 2
	// GIMBAL_DEVICE_FLAGS_ROLL_LOCK enum. Lock roll angle to absolute angle relative to horizon (not relative to drone). This is generally the default with a stabilizing gimbal
	GIMBAL_DEVICE_FLAGS_ROLL_LOCK GIMBAL_DEVICE_FLAGS = 4
	// GIMBAL_DEVICE_FLAGS_PITCH_LOCK enum. Lock pitch angle to absolute angle relative to horizon (not relative to drone). This is generally the default
	GIMBAL_DEVICE_FLAGS_PITCH_LOCK GIMBAL_DEVICE_FLAGS = 8
	// GIMBAL_DEVICE_FLAGS_YAW_LOCK enum. Lock yaw angle to absolute angle relative to North (not relative to drone). If this flag is set, the quaternion is in the Earth frame with the x-axis pointing North (yaw absolute). If this flag is not set, the quaternion frame is in the Earth frame rotated so that the x-axis is pointing forward (yaw relative to vehicle)
	GIMBAL_DEVICE_FLAGS_YAW_LOCK GIMBAL_DEVICE_FLAGS = 16
)

// GIMBAL_MANAGER_FLAGS type. Flags for high level gimbal manager operation The first 16 bytes are identical to the GIMBAL_DEVICE_FLAGS.
type GIMBAL_MANAGER_FLAGS int

const (
	// GIMBAL_MANAGER_FLAGS_RETRACT enum. Based on GIMBAL_DEVICE_FLAGS_RETRACT
	GIMBAL_MANAGER_FLAGS_RETRACT GIMBAL_MANAGER_FLAGS = 1
	// GIMBAL_MANAGER_FLAGS_NEUTRAL enum. Based on GIMBAL_DEVICE_FLAGS_NEUTRAL
	GIMBAL_MANAGER_FLAGS_NEUTRAL GIMBAL_MANAGER_FLAGS = 2
	// GIMBAL_MANAGER_FLAGS_ROLL_LOCK enum. Based on GIMBAL_DEVICE_FLAGS_ROLL_LOCK
	GIMBAL_MANAGER_FLAGS_ROLL_LOCK GIMBAL_MANAGER_FLAGS = 4
	// GIMBAL_MANAGER_FLAGS_PITCH_LOCK enum. Based on GIMBAL_DEVICE_FLAGS_PITCH_LOCK
	GIMBAL_MANAGER_FLAGS_PITCH_LOCK GIMBAL_MANAGER_FLAGS = 8
	// GIMBAL_MANAGER_FLAGS_YAW_LOCK enum. Based on GIMBAL_DEVICE_FLAGS_YAW_LOCK
	GIMBAL_MANAGER_FLAGS_YAW_LOCK GIMBAL_MANAGER_FLAGS = 16
)

// GIMBAL_DEVICE_ERROR_FLAGS type. Gimbal device (low level) error flags (bitmap, 0 means no error)
type GIMBAL_DEVICE_ERROR_FLAGS int

const (
	// GIMBAL_DEVICE_ERROR_FLAGS_AT_ROLL_LIMIT enum. Gimbal device is limited by hardware roll limit
	GIMBAL_DEVICE_ERROR_FLAGS_AT_ROLL_LIMIT GIMBAL_DEVICE_ERROR_FLAGS = 1
	// GIMBAL_DEVICE_ERROR_FLAGS_AT_PITCH_LIMIT enum. Gimbal device is limited by hardware pitch limit
	GIMBAL_DEVICE_ERROR_FLAGS_AT_PITCH_LIMIT GIMBAL_DEVICE_ERROR_FLAGS = 2
	// GIMBAL_DEVICE_ERROR_FLAGS_AT_YAW_LIMIT enum. Gimbal device is limited by hardware yaw limit
	GIMBAL_DEVICE_ERROR_FLAGS_AT_YAW_LIMIT GIMBAL_DEVICE_ERROR_FLAGS = 4
	// GIMBAL_DEVICE_ERROR_FLAGS_ENCODER_ERROR enum. There is an error with the gimbal encoders
	GIMBAL_DEVICE_ERROR_FLAGS_ENCODER_ERROR GIMBAL_DEVICE_ERROR_FLAGS = 8
	// GIMBAL_DEVICE_ERROR_FLAGS_POWER_ERROR enum. There is an error with the gimbal power source
	GIMBAL_DEVICE_ERROR_FLAGS_POWER_ERROR GIMBAL_DEVICE_ERROR_FLAGS = 16
	// GIMBAL_DEVICE_ERROR_FLAGS_MOTOR_ERROR enum. There is an error with the gimbal motor's
	GIMBAL_DEVICE_ERROR_FLAGS_MOTOR_ERROR GIMBAL_DEVICE_ERROR_FLAGS = 32
	// GIMBAL_DEVICE_ERROR_FLAGS_SOFTWARE_ERROR enum. There is an error with the gimbal's software
	GIMBAL_DEVICE_ERROR_FLAGS_SOFTWARE_ERROR GIMBAL_DEVICE_ERROR_FLAGS = 64
	// GIMBAL_DEVICE_ERROR_FLAGS_COMMS_ERROR enum. There is an error with the gimbal's communication
	GIMBAL_DEVICE_ERROR_FLAGS_COMMS_ERROR GIMBAL_DEVICE_ERROR_FLAGS = 128
	// GIMBAL_DEVICE_ERROR_FLAGS_CALIBRATION_RUNNING enum. Gimbal is currently calibrating
	GIMBAL_DEVICE_ERROR_FLAGS_CALIBRATION_RUNNING GIMBAL_DEVICE_ERROR_FLAGS = 256
)

// GRIPPER_ACTIONS type. Gripper actions.
type GRIPPER_ACTIONS int

const (
	// GRIPPER_ACTION_RELEASE enum. Gripper release cargo
	GRIPPER_ACTION_RELEASE GRIPPER_ACTIONS = 0
	// GRIPPER_ACTION_GRAB enum. Gripper grab onto cargo
	GRIPPER_ACTION_GRAB GRIPPER_ACTIONS = 1
)

// WINCH_ACTIONS type. Winch actions.
type WINCH_ACTIONS int

const (
	// WINCH_RELAXED enum. Relax winch
	WINCH_RELAXED WINCH_ACTIONS = 0
	// WINCH_RELATIVE_LENGTH_CONTROL enum. Wind or unwind specified length of cable, optionally using specified rate
	WINCH_RELATIVE_LENGTH_CONTROL WINCH_ACTIONS = 1
	// WINCH_RATE_CONTROL enum. Wind or unwind cable at specified rate
	WINCH_RATE_CONTROL WINCH_ACTIONS = 2
)

// UAVCAN_NODE_HEALTH type. Generalized UAVCAN node health
type UAVCAN_NODE_HEALTH int

const (
	// UAVCAN_NODE_HEALTH_OK enum. The node is functioning properly
	UAVCAN_NODE_HEALTH_OK UAVCAN_NODE_HEALTH = 0
	// UAVCAN_NODE_HEALTH_WARNING enum. A critical parameter went out of range or the node has encountered a minor failure
	UAVCAN_NODE_HEALTH_WARNING UAVCAN_NODE_HEALTH = 1
	// UAVCAN_NODE_HEALTH_ERROR enum. The node has encountered a major failure
	UAVCAN_NODE_HEALTH_ERROR UAVCAN_NODE_HEALTH = 2
	// UAVCAN_NODE_HEALTH_CRITICAL enum. The node has suffered a fatal malfunction
	UAVCAN_NODE_HEALTH_CRITICAL UAVCAN_NODE_HEALTH = 3
)

// UAVCAN_NODE_MODE type. Generalized UAVCAN node mode
type UAVCAN_NODE_MODE int

const (
	// UAVCAN_NODE_MODE_OPERATIONAL enum. The node is performing its primary functions
	UAVCAN_NODE_MODE_OPERATIONAL UAVCAN_NODE_MODE = 0
	// UAVCAN_NODE_MODE_INITIALIZATION enum. The node is initializing; this mode is entered immediately after startup
	UAVCAN_NODE_MODE_INITIALIZATION UAVCAN_NODE_MODE = 1
	// UAVCAN_NODE_MODE_MAINTENANCE enum. The node is under maintenance
	UAVCAN_NODE_MODE_MAINTENANCE UAVCAN_NODE_MODE = 2
	// UAVCAN_NODE_MODE_SOFTWARE_UPDATE enum. The node is in the process of updating its software
	UAVCAN_NODE_MODE_SOFTWARE_UPDATE UAVCAN_NODE_MODE = 3
	// UAVCAN_NODE_MODE_OFFLINE enum. The node is no longer available online
	UAVCAN_NODE_MODE_OFFLINE UAVCAN_NODE_MODE = 7
)

// ESC_CONNECTION_TYPE type. Indicates the ESC connection type.
type ESC_CONNECTION_TYPE int

const (
	// ESC_CONNECTION_TYPE_PPM enum. Traditional PPM ESC
	ESC_CONNECTION_TYPE_PPM ESC_CONNECTION_TYPE = 0
	// ESC_CONNECTION_TYPE_SERIAL enum. Serial Bus connected ESC
	ESC_CONNECTION_TYPE_SERIAL ESC_CONNECTION_TYPE = 1
	// ESC_CONNECTION_TYPE_ONESHOT enum. One Shot PPM ESC
	ESC_CONNECTION_TYPE_ONESHOT ESC_CONNECTION_TYPE = 2
	// ESC_CONNECTION_TYPE_I2C enum. I2C ESC
	ESC_CONNECTION_TYPE_I2C ESC_CONNECTION_TYPE = 3
	// ESC_CONNECTION_TYPE_CAN enum. CAN-Bus ESC
	ESC_CONNECTION_TYPE_CAN ESC_CONNECTION_TYPE = 4
	// ESC_CONNECTION_TYPE_DSHOT enum. DShot ESC
	ESC_CONNECTION_TYPE_DSHOT ESC_CONNECTION_TYPE = 5
)

// ESC_FAILURE_FLAGS type. Flags to report ESC failures.
type ESC_FAILURE_FLAGS int

const (
	// ESC_FAILURE_NONE enum. No ESC failure
	ESC_FAILURE_NONE ESC_FAILURE_FLAGS = 0
	// ESC_FAILURE_OVER_CURRENT enum. Over current failure
	ESC_FAILURE_OVER_CURRENT ESC_FAILURE_FLAGS = 1
	// ESC_FAILURE_OVER_VOLTAGE enum. Over voltage failure
	ESC_FAILURE_OVER_VOLTAGE ESC_FAILURE_FLAGS = 2
	// ESC_FAILURE_OVER_TEMPERATURE enum. Over temperature failure
	ESC_FAILURE_OVER_TEMPERATURE ESC_FAILURE_FLAGS = 4
	// ESC_FAILURE_OVER_RPM enum. Over RPM failure
	ESC_FAILURE_OVER_RPM ESC_FAILURE_FLAGS = 8
	// ESC_FAILURE_INCONSISTENT_CMD enum. Inconsistent command failure i.e. out of bounds
	ESC_FAILURE_INCONSISTENT_CMD ESC_FAILURE_FLAGS = 16
	// ESC_FAILURE_MOTOR_STUCK enum. Motor stuck failure
	ESC_FAILURE_MOTOR_STUCK ESC_FAILURE_FLAGS = 32
	// ESC_FAILURE_GENERIC enum. Generic ESC failure
	ESC_FAILURE_GENERIC ESC_FAILURE_FLAGS = 64
)

// STORAGE_STATUS type. Flags to indicate the status of camera storage.
type STORAGE_STATUS int

const (
	// STORAGE_STATUS_EMPTY enum. Storage is missing (no microSD card loaded for example.)
	STORAGE_STATUS_EMPTY STORAGE_STATUS = 0
	// STORAGE_STATUS_UNFORMATTED enum. Storage present but unformatted
	STORAGE_STATUS_UNFORMATTED STORAGE_STATUS = 1
	// STORAGE_STATUS_READY enum. Storage present and ready
	STORAGE_STATUS_READY STORAGE_STATUS = 2
	// STORAGE_STATUS_NOT_SUPPORTED enum. Camera does not supply storage status information. Capacity information in STORAGE_INFORMATION fields will be ignored
	STORAGE_STATUS_NOT_SUPPORTED STORAGE_STATUS = 3
)

// STORAGE_TYPE type. Flags to indicate the type of storage.
type STORAGE_TYPE int

const (
	// STORAGE_TYPE_UNKNOWN enum. Storage type is not known
	STORAGE_TYPE_UNKNOWN STORAGE_TYPE = 0
	// STORAGE_TYPE_USB_STICK enum. Storage type is USB device
	STORAGE_TYPE_USB_STICK STORAGE_TYPE = 1
	// STORAGE_TYPE_SD enum. Storage type is SD card
	STORAGE_TYPE_SD STORAGE_TYPE = 2
	// STORAGE_TYPE_MICROSD enum. Storage type is microSD card
	STORAGE_TYPE_MICROSD STORAGE_TYPE = 3
	// STORAGE_TYPE_CF enum. Storage type is CFast
	STORAGE_TYPE_CF STORAGE_TYPE = 4
	// STORAGE_TYPE_CFE enum. Storage type is CFexpress
	STORAGE_TYPE_CFE STORAGE_TYPE = 5
	// STORAGE_TYPE_XQD enum. Storage type is XQD
	STORAGE_TYPE_XQD STORAGE_TYPE = 6
	// STORAGE_TYPE_HD enum. Storage type is HD mass storage type
	STORAGE_TYPE_HD STORAGE_TYPE = 7
	// STORAGE_TYPE_OTHER enum. Storage type is other, not listed type
	STORAGE_TYPE_OTHER STORAGE_TYPE = 254
)

// ORBIT_YAW_BEHAVIOUR type. Yaw behaviour during orbit flight.
type ORBIT_YAW_BEHAVIOUR int

const (
	// ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TO_CIRCLE_CENTER enum. Vehicle front points to the center (default)
	ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TO_CIRCLE_CENTER ORBIT_YAW_BEHAVIOUR = 0
	// ORBIT_YAW_BEHAVIOUR_HOLD_INITIAL_HEADING enum. Vehicle front holds heading when message received
	ORBIT_YAW_BEHAVIOUR_HOLD_INITIAL_HEADING ORBIT_YAW_BEHAVIOUR = 1
	// ORBIT_YAW_BEHAVIOUR_UNCONTROLLED enum. Yaw uncontrolled
	ORBIT_YAW_BEHAVIOUR_UNCONTROLLED ORBIT_YAW_BEHAVIOUR = 2
	// ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TANGENT_TO_CIRCLE enum. Vehicle front follows flight path (tangential to circle)
	ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TANGENT_TO_CIRCLE ORBIT_YAW_BEHAVIOUR = 3
	// ORBIT_YAW_BEHAVIOUR_RC_CONTROLLED enum. Yaw controlled by RC input
	ORBIT_YAW_BEHAVIOUR_RC_CONTROLLED ORBIT_YAW_BEHAVIOUR = 4
)

// WIFI_CONFIG_AP_RESPONSE type. Possible responses from a WIFI_CONFIG_AP message.
type WIFI_CONFIG_AP_RESPONSE int

const (
	// WIFI_CONFIG_AP_RESPONSE_UNDEFINED enum. Undefined response. Likely an indicative of a system that doesn't support this request
	WIFI_CONFIG_AP_RESPONSE_UNDEFINED WIFI_CONFIG_AP_RESPONSE = 0
	// WIFI_CONFIG_AP_RESPONSE_ACCEPTED enum. Changes accepted
	WIFI_CONFIG_AP_RESPONSE_ACCEPTED WIFI_CONFIG_AP_RESPONSE = 1
	// WIFI_CONFIG_AP_RESPONSE_REJECTED enum. Changes rejected
	WIFI_CONFIG_AP_RESPONSE_REJECTED WIFI_CONFIG_AP_RESPONSE = 2
	// WIFI_CONFIG_AP_RESPONSE_MODE_ERROR enum. Invalid Mode
	WIFI_CONFIG_AP_RESPONSE_MODE_ERROR WIFI_CONFIG_AP_RESPONSE = 3
	// WIFI_CONFIG_AP_RESPONSE_SSID_ERROR enum. Invalid SSID
	WIFI_CONFIG_AP_RESPONSE_SSID_ERROR WIFI_CONFIG_AP_RESPONSE = 4
	// WIFI_CONFIG_AP_RESPONSE_PASSWORD_ERROR enum. Invalid Password
	WIFI_CONFIG_AP_RESPONSE_PASSWORD_ERROR WIFI_CONFIG_AP_RESPONSE = 5
)

// CELLULAR_CONFIG_RESPONSE type. Possible responses from a CELLULAR_CONFIG message.
type CELLULAR_CONFIG_RESPONSE int

const (
	// CELLULAR_CONFIG_RESPONSE_ACCEPTED enum. Changes accepted
	CELLULAR_CONFIG_RESPONSE_ACCEPTED CELLULAR_CONFIG_RESPONSE = 0
	// CELLULAR_CONFIG_RESPONSE_APN_ERROR enum. Invalid APN
	CELLULAR_CONFIG_RESPONSE_APN_ERROR CELLULAR_CONFIG_RESPONSE = 1
	// CELLULAR_CONFIG_RESPONSE_PIN_ERROR enum. Invalid PIN
	CELLULAR_CONFIG_RESPONSE_PIN_ERROR CELLULAR_CONFIG_RESPONSE = 2
	// CELLULAR_CONFIG_RESPONSE_REJECTED enum. Changes rejected
	CELLULAR_CONFIG_RESPONSE_REJECTED CELLULAR_CONFIG_RESPONSE = 3
	// CELLULAR_CONFIG_BLOCKED_PUK_REQUIRED enum. PUK is required to unblock SIM card
	CELLULAR_CONFIG_BLOCKED_PUK_REQUIRED CELLULAR_CONFIG_RESPONSE = 4
)

// WIFI_CONFIG_AP_MODE type. WiFi Mode.
type WIFI_CONFIG_AP_MODE int

const (
	// WIFI_CONFIG_AP_MODE_UNDEFINED enum. WiFi mode is undefined
	WIFI_CONFIG_AP_MODE_UNDEFINED WIFI_CONFIG_AP_MODE = 0
	// WIFI_CONFIG_AP_MODE_AP enum. WiFi configured as an access point
	WIFI_CONFIG_AP_MODE_AP WIFI_CONFIG_AP_MODE = 1
	// WIFI_CONFIG_AP_MODE_STATION enum. WiFi configured as a station connected to an existing local WiFi network
	WIFI_CONFIG_AP_MODE_STATION WIFI_CONFIG_AP_MODE = 2
	// WIFI_CONFIG_AP_MODE_DISABLED enum. WiFi disabled
	WIFI_CONFIG_AP_MODE_DISABLED WIFI_CONFIG_AP_MODE = 3
)

// COMP_METADATA_TYPE type. Possible values for COMPONENT_INFORMATION.comp_metadata_type.
type COMP_METADATA_TYPE int

const (
	// COMP_METADATA_TYPE_VERSION enum. Version information which also includes information on other optional supported COMP_METADATA_TYPE's. Must be supported. Only downloadable from vehicle
	COMP_METADATA_TYPE_VERSION COMP_METADATA_TYPE = 0
	// COMP_METADATA_TYPE_PARAMETER enum. Parameter meta data
	COMP_METADATA_TYPE_PARAMETER COMP_METADATA_TYPE = 1
	// COMP_METADATA_TYPE_COMMANDS enum. Meta data which specifies the commands the vehicle supports. (WIP)
	COMP_METADATA_TYPE_COMMANDS COMP_METADATA_TYPE = 2
)

// PARAM_TRANSACTION_TRANSPORT type. Possible transport layers to set and get parameters via mavlink during a parameter transaction.
type PARAM_TRANSACTION_TRANSPORT int

const (
	// PARAM_TRANSACTION_TRANSPORT_PARAM enum. Transaction over param transport
	PARAM_TRANSACTION_TRANSPORT_PARAM PARAM_TRANSACTION_TRANSPORT = 0
	// PARAM_TRANSACTION_TRANSPORT_PARAM_EXT enum. Transaction over param_ext transport
	PARAM_TRANSACTION_TRANSPORT_PARAM_EXT PARAM_TRANSACTION_TRANSPORT = 1
)

// PARAM_TRANSACTION_ACTION type. Possible parameter transaction actions.
type PARAM_TRANSACTION_ACTION int

const (
	// PARAM_TRANSACTION_ACTION_START enum. Commit the current parameter transaction
	PARAM_TRANSACTION_ACTION_START PARAM_TRANSACTION_ACTION = 0
	// PARAM_TRANSACTION_ACTION_COMMIT enum. Commit the current parameter transaction
	PARAM_TRANSACTION_ACTION_COMMIT PARAM_TRANSACTION_ACTION = 1
	// PARAM_TRANSACTION_ACTION_CANCEL enum. Cancel the current parameter transaction
	PARAM_TRANSACTION_ACTION_CANCEL PARAM_TRANSACTION_ACTION = 2
)

// MAV_CMD type. Commands to be executed by the MAV. They can be executed on user request, or as part of a mission script. If the action is used in a mission, the parameter mapping to the waypoint/mission message is as follows: Param 1, Param 2, Param 3, Param 4, X: Param 5, Y:Param 6, Z:Param 7. This command list is similar what ARINC 424 is for commercial aircraft: A data format how to interpret waypoint/mission data. NaN and INT32_MAX may be used in float/integer params (respectively) to indicate optional/default values (e.g. to use the component's current yaw or latitude rather than a specific value). See https://mavlink.io/en/guide/xml_schema.html#MAV_CMD for information about the structure of the MAV_CMD entries
type MAV_CMD int

const (
	// MAV_CMD_NAV_WAYPOINT enum. Navigate to waypoint. Params: 1) Hold time. (ignored by fixed wing, time to stay at waypoint for rotary wing); 2) Acceptance radius (if the sphere with this radius is hit, the waypoint counts as reached); 3) 0 to pass through the WP, if &gt; 0 radius to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.; 4) Desired yaw angle at waypoint (rotary wing). NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).; 5) Latitude; 6) Longitude; 7) Altitude;
	MAV_CMD_NAV_WAYPOINT MAV_CMD = 16
	// MAV_CMD_NAV_LOITER_UNLIM enum. Loiter around this waypoint an unlimited amount of time. Params: 1) Empty; 2) Empty; 3) Loiter radius around waypoint for forward-only moving vehicles (not multicopters). If positive loiter clockwise, else counter-clockwise; 4) Desired yaw angle. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).; 5) Latitude; 6) Longitude; 7) Altitude;
	MAV_CMD_NAV_LOITER_UNLIM MAV_CMD = 17
	// MAV_CMD_NAV_LOITER_TURNS enum. Loiter around this waypoint for X turns. Params: 1) Number of turns.; 2) Leave loiter circle only once heading towards the next waypoint (0 = False); 3) Loiter radius around waypoint for forward-only moving vehicles (not multicopters). If positive loiter clockwise, else counter-clockwise; 4) Loiter circle exit location and/or path to next waypoint ("xtrack") for forward-only moving vehicles (not multicopters). 0 for the vehicle to converge towards the center xtrack when it leaves the loiter (the line between the centers of the current and next waypoint), 1 to converge to the direct line between the location that the vehicle exits the loiter radius and the next waypoint. Otherwise the angle (in degrees) between the tangent of the loiter circle and the center xtrack at which the vehicle must leave the loiter (and converge to the center xtrack). NaN to use the current system default xtrack behaviour.; 5) Latitude; 6) Longitude; 7) Altitude;
	MAV_CMD_NAV_LOITER_TURNS MAV_CMD = 18
	// MAV_CMD_NAV_LOITER_TIME enum. Loiter at the specified latitude, longitude and altitude for a certain amount of time. Multicopter vehicles stop at the point (within a vehicle-specific acceptance radius). Forward-only moving vehicles (e.g. fixed-wing) circle the point with the specified radius/direction. If the Heading Required parameter (2) is non-zero forward moving aircraft will only leave the loiter circle once heading towards the next waypoint. Params: 1) Loiter time (only starts once Lat, Lon and Alt is reached).; 2) Leave loiter circle only once heading towards the next waypoint (0 = False); 3) Loiter radius around waypoint for forward-only moving vehicles (not multicopters). If positive loiter clockwise, else counter-clockwise.; 4) Loiter circle exit location and/or path to next waypoint ("xtrack") for forward-only moving vehicles (not multicopters). 0 for the vehicle to converge towards the center xtrack when it leaves the loiter (the line between the centers of the current and next waypoint), 1 to converge to the direct line between the location that the vehicle exits the loiter radius and the next waypoint. Otherwise the angle (in degrees) between the tangent of the loiter circle and the center xtrack at which the vehicle must leave the loiter (and converge to the center xtrack). NaN to use the current system default xtrack behaviour.; 5) Latitude; 6) Longitude; 7) Altitude;
	MAV_CMD_NAV_LOITER_TIME MAV_CMD = 19
	// MAV_CMD_NAV_RETURN_TO_LAUNCH enum. Return to launch location. Params: 1) Empty; 2) Empty; 3) Empty; 4) Empty; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_NAV_RETURN_TO_LAUNCH MAV_CMD = 20
	// MAV_CMD_NAV_LAND enum. Land at location. Params: 1) Minimum target altitude if landing is aborted (0 = undefined/use system default).; 2) Precision land mode.; 3) Empty; 4) Desired yaw angle. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).; 5) Latitude.; 6) Longitude.; 7) Landing altitude (ground level in current frame).;
	MAV_CMD_NAV_LAND MAV_CMD = 21
	// MAV_CMD_NAV_TAKEOFF enum. Takeoff from ground / hand. Vehicles that support multiple takeoff modes (e.g. VTOL quadplane) should take off using the currently configured mode. Params: 1) Minimum pitch (if airspeed sensor present), desired pitch without sensor; 2) Empty; 3) Empty; 4) Yaw angle (if magnetometer present), ignored without magnetometer. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).; 5) Latitude; 6) Longitude; 7) Altitude;
	MAV_CMD_NAV_TAKEOFF MAV_CMD = 22
	// MAV_CMD_NAV_LAND_LOCAL enum. Land at local position (local frame only). Params: 1) Landing target number (if available); 2) Maximum accepted offset from desired landing position - computed magnitude from spherical coordinates: d = sqrt(x^2 + y^2 + z^2), which gives the maximum accepted distance between the desired landing position and the position where the vehicle is about to land; 3) Landing descend rate; 4) Desired yaw angle; 5) Y-axis position; 6) X-axis position; 7) Z-axis / ground level position;
	MAV_CMD_NAV_LAND_LOCAL MAV_CMD = 23
	// MAV_CMD_NAV_TAKEOFF_LOCAL enum. Takeoff from local position (local frame only). Params: 1) Minimum pitch (if airspeed sensor present), desired pitch without sensor; 2) Empty; 3) Takeoff ascend rate; 4) Yaw angle (if magnetometer or another yaw estimation source present), ignored without one of these; 5) Y-axis position; 6) X-axis position; 7) Z-axis position;
	MAV_CMD_NAV_TAKEOFF_LOCAL MAV_CMD = 24
	// MAV_CMD_NAV_FOLLOW enum. Vehicle following, i.e. this waypoint represents the position of a moving vehicle. Params: 1) Following logic to use (e.g. loitering or sinusoidal following) - depends on specific autopilot implementation; 2) Ground speed of vehicle to be followed; 3) Radius around waypoint. If positive loiter clockwise, else counter-clockwise; 4) Desired yaw angle.; 5) Latitude; 6) Longitude; 7) Altitude;
	MAV_CMD_NAV_FOLLOW MAV_CMD = 25
	// MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT enum. Continue on the current course and climb/descend to specified altitude.  When the altitude is reached continue to the next command (i.e., don't proceed to the next command until the desired altitude is reached. Params: 1) Climb or Descend (0 = Neutral, command completes when within 5m of this command's altitude, 1 = Climbing, command completes when at or above this command's altitude, 2 = Descending, command completes when at or below this command's altitude.; 2) Empty; 3) Empty; 4) Empty; 5) Empty; 6) Empty; 7) Desired altitude;
	MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT MAV_CMD = 30
	// MAV_CMD_NAV_LOITER_TO_ALT enum. Begin loiter at the specified Latitude and Longitude.  If Lat=Lon=0, then loiter at the current position.  Don't consider the navigation command complete (don't leave loiter) until the altitude has been reached. Additionally, if the Heading Required parameter is non-zero the aircraft will not leave the loiter until heading toward the next waypoint. Params: 1) Leave loiter circle only once heading towards the next waypoint (0 = False); 2) Loiter radius around waypoint for forward-only moving vehicles (not multicopters). If positive loiter clockwise, negative counter-clockwise, 0 means no change to standard loiter.; 3) Empty; 4) Loiter circle exit location and/or path to next waypoint ("xtrack") for forward-only moving vehicles (not multicopters). 0 for the vehicle to converge towards the center xtrack when it leaves the loiter (the line between the centers of the current and next waypoint), 1 to converge to the direct line between the location that the vehicle exits the loiter radius and the next waypoint. Otherwise the angle (in degrees) between the tangent of the loiter circle and the center xtrack at which the vehicle must leave the loiter (and converge to the center xtrack). NaN to use the current system default xtrack behaviour.; 5) Latitude; 6) Longitude; 7) Altitude;
	MAV_CMD_NAV_LOITER_TO_ALT MAV_CMD = 31
	// MAV_CMD_DO_FOLLOW enum. Begin following a target. Params: 1) System ID (of the FOLLOW_TARGET beacon). Send 0 to disable follow-me and return to the default position hold mode.; 2) Reserved; 3) Reserved; 4) Altitude mode: 0: Keep current altitude, 1: keep altitude difference to target, 2: go to a fixed altitude above home.; 5) Altitude above home. (used if mode=2); 6) Reserved; 7) Time to land in which the MAV should go to the default position hold mode after a message RX timeout.;
	MAV_CMD_DO_FOLLOW MAV_CMD = 32
	// MAV_CMD_DO_FOLLOW_REPOSITION enum. Reposition the MAV after a follow target command has been sent. Params: 1) Camera q1 (where 0 is on the ray from the camera to the tracking device); 2) Camera q2; 3) Camera q3; 4) Camera q4; 5) altitude offset from target; 6) X offset from target; 7) Y offset from target;
	MAV_CMD_DO_FOLLOW_REPOSITION MAV_CMD = 33
	// MAV_CMD_DO_ORBIT enum. Start orbiting on the circumference of a circle defined by the parameters. Setting any value NaN results in using defaults. Params: 1) Radius of the circle. positive: Orbit clockwise. negative: Orbit counter-clockwise.; 2) Tangential Velocity. NaN: Vehicle configuration default.; 3) Yaw behavior of the vehicle.; 4) Reserved (e.g. for dynamic center beacon options); 5) Center point latitude (if no MAV_FRAME specified) / X coordinate according to MAV_FRAME. NaN: Use current vehicle position or current center if already orbiting.; 6) Center point longitude (if no MAV_FRAME specified) / Y coordinate according to MAV_FRAME. NaN: Use current vehicle position or current center if already orbiting.; 7) Center point altitude (MSL) (if no MAV_FRAME specified) / Z coordinate according to MAV_FRAME. NaN: Use current vehicle position or current center if already orbiting.;
	MAV_CMD_DO_ORBIT MAV_CMD = 34
	// MAV_CMD_NAV_ROI enum. Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicle's control system to control the vehicle attitude and the attitude of various sensors such as cameras. Params: 1) Region of interest mode.; 2) Waypoint index/ target ID. (see MAV_ROI enum); 3) ROI index (allows a vehicle to manage multiple ROI's); 4) Empty; 5) x the location of the fixed ROI (see MAV_FRAME); 6) y; 7) z;
	MAV_CMD_NAV_ROI MAV_CMD = 80
	// MAV_CMD_NAV_PATHPLANNING enum. Control autonomous path planning on the MAV. Params: 1) 0: Disable local obstacle avoidance / local path planning (without resetting map), 1: Enable local path planning, 2: Enable and reset local path planning; 2) 0: Disable full path planning (without resetting map), 1: Enable, 2: Enable and reset map/occupancy grid, 3: Enable and reset planned route, but not occupancy grid; 3) Empty; 4) Yaw angle at goal; 5) Latitude/X of goal; 6) Longitude/Y of goal; 7) Altitude/Z of goal;
	MAV_CMD_NAV_PATHPLANNING MAV_CMD = 81
	// MAV_CMD_NAV_SPLINE_WAYPOINT enum. Navigate to waypoint using a spline path. Params: 1) Hold time. (ignored by fixed wing, time to stay at waypoint for rotary wing); 2) Empty; 3) Empty; 4) Empty; 5) Latitude/X of goal; 6) Longitude/Y of goal; 7) Altitude/Z of goal;
	MAV_CMD_NAV_SPLINE_WAYPOINT MAV_CMD = 82
	// MAV_CMD_NAV_VTOL_TAKEOFF enum. Takeoff from ground using VTOL mode, and transition to forward flight with specified heading. The command should be ignored by vehicles that dont support both VTOL and fixed-wing flight (multicopters, boats,etc.). Params: 1) Empty; 2) Front transition heading.; 3) Empty; 4) Yaw angle. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).; 5) Latitude; 6) Longitude; 7) Altitude;
	MAV_CMD_NAV_VTOL_TAKEOFF MAV_CMD = 84
	// MAV_CMD_NAV_VTOL_LAND enum. Land using VTOL mode. Params: 1) Empty; 2) Empty; 3) Approach altitude (with the same reference as the Altitude field). NaN if unspecified.; 4) Yaw angle. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).; 5) Latitude; 6) Longitude; 7) Altitude (ground level);
	MAV_CMD_NAV_VTOL_LAND MAV_CMD = 85
	// MAV_CMD_NAV_GUIDED_ENABLE enum. hand control over to an external controller. Params: 1) On / Off (&gt; 0.5f on); 2) Empty; 3) Empty; 4) Empty; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_NAV_GUIDED_ENABLE MAV_CMD = 92
	// MAV_CMD_NAV_DELAY enum. Delay the next navigation command a number of seconds or until a specified time. Params: 1) Delay (-1 to enable time-of-day fields); 2) hour (24h format, UTC, -1 to ignore); 3) minute (24h format, UTC, -1 to ignore); 4) second (24h format, UTC, -1 to ignore); 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_NAV_DELAY MAV_CMD = 93
	// MAV_CMD_NAV_PAYLOAD_PLACE enum. Descend and place payload. Vehicle moves to specified location, descends until it detects a hanging payload has reached the ground, and then releases the payload. If ground is not detected before the reaching the maximum descent value (param1), the command will complete without releasing the payload. Params: 1) Maximum distance to descend.; 2) Empty; 3) Empty; 4) Empty; 5) Latitude; 6) Longitude; 7) Altitude;
	MAV_CMD_NAV_PAYLOAD_PLACE MAV_CMD = 94
	// MAV_CMD_NAV_LAST enum. NOP - This command is only used to mark the upper limit of the NAV/ACTION commands in the enumeration. Params: 1) Empty; 2) Empty; 3) Empty; 4) Empty; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_NAV_LAST MAV_CMD = 95
	// MAV_CMD_CONDITION_DELAY enum. Delay mission state machine. Params: 1) Delay; 2) Empty; 3) Empty; 4) Empty; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_CONDITION_DELAY MAV_CMD = 112
	// MAV_CMD_CONDITION_CHANGE_ALT enum. Ascend/descend to target altitude at specified rate. Delay mission state machine until desired altitude reached. Params: 1) Descent / Ascend rate.; 2) Empty; 3) Empty; 4) Empty; 5) Empty; 6) Empty; 7) Target Altitude;
	MAV_CMD_CONDITION_CHANGE_ALT MAV_CMD = 113
	// MAV_CMD_CONDITION_DISTANCE enum. Delay mission state machine until within desired distance of next NAV point. Params: 1) Distance.; 2) Empty; 3) Empty; 4) Empty; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_CONDITION_DISTANCE MAV_CMD = 114
	// MAV_CMD_CONDITION_YAW enum. Reach a certain target angle. Params: 1) target angle, 0 is north; 2) angular speed; 3) direction: -1: counter clockwise, 1: clockwise; 4) 0: absolute angle, 1: relative offset; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_CONDITION_YAW MAV_CMD = 115
	// MAV_CMD_CONDITION_LAST enum. NOP - This command is only used to mark the upper limit of the CONDITION commands in the enumeration. Params: 1) Empty; 2) Empty; 3) Empty; 4) Empty; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_CONDITION_LAST MAV_CMD = 159
	// MAV_CMD_DO_SET_MODE enum. Set system mode. Params: 1) Mode; 2) Custom mode - this is system specific, please refer to the individual autopilot specifications for details.; 3) Custom sub mode - this is system specific, please refer to the individual autopilot specifications for details.; 4) Empty; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_DO_SET_MODE MAV_CMD = 176
	// MAV_CMD_DO_JUMP enum. Jump to the desired command in the mission list.  Repeat this action only the specified number of times. Params: 1) Sequence number; 2) Repeat count; 3) Empty; 4) Empty; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_DO_JUMP MAV_CMD = 177
	// MAV_CMD_DO_CHANGE_SPEED enum. Change speed and/or throttle set points. Params: 1) Speed type (0=Airspeed, 1=Ground Speed, 2=Climb Speed, 3=Descent Speed); 2) Speed (-1 indicates no change); 3) Throttle (-1 indicates no change); 4) 0: absolute, 1: relative; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_DO_CHANGE_SPEED MAV_CMD = 178
	// MAV_CMD_DO_SET_HOME enum. Changes the home location either to the current location or a specified location. Params: 1) Use current (1=use current location, 0=use specified location); 2) Empty; 3) Empty; 4) Yaw angle. NaN to use default heading; 5) Latitude; 6) Longitude; 7) Altitude;
	MAV_CMD_DO_SET_HOME MAV_CMD = 179
	// MAV_CMD_DO_SET_PARAMETER enum. Set a system parameter.  Caution!  Use of this command requires knowledge of the numeric enumeration value of the parameter. Params: 1) Parameter number; 2) Parameter value; 3) Empty; 4) Empty; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_DO_SET_PARAMETER MAV_CMD = 180
	// MAV_CMD_DO_SET_RELAY enum. Set a relay to a condition. Params: 1) Relay instance number.; 2) Setting. (1=on, 0=off, others possible depending on system hardware); 3) Empty; 4) Empty; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_DO_SET_RELAY MAV_CMD = 181
	// MAV_CMD_DO_REPEAT_RELAY enum. Cycle a relay on and off for a desired number of cycles with a desired period. Params: 1) Relay instance number.; 2) Cycle count.; 3) Cycle time.; 4) Empty; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_DO_REPEAT_RELAY MAV_CMD = 182
	// MAV_CMD_DO_SET_SERVO enum. Set a servo to a desired PWM value. Params: 1) Servo instance number.; 2) Pulse Width Modulation.; 3) Empty; 4) Empty; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_DO_SET_SERVO MAV_CMD = 183
	// MAV_CMD_DO_REPEAT_SERVO enum. Cycle a between its nominal setting and a desired PWM for a desired number of cycles with a desired period. Params: 1) Servo instance number.; 2) Pulse Width Modulation.; 3) Cycle count.; 4) Cycle time.; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_DO_REPEAT_SERVO MAV_CMD = 184
	// MAV_CMD_DO_FLIGHTTERMINATION enum. Terminate flight immediately. Params: 1) Flight termination activated if &gt; 0.5; 2) Empty; 3) Empty; 4) Empty; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_DO_FLIGHTTERMINATION MAV_CMD = 185
	// MAV_CMD_DO_CHANGE_ALTITUDE enum. Change altitude set point. Params: 1) Altitude; 2) Frame of new altitude.; 3) Empty; 4) Empty; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_DO_CHANGE_ALTITUDE MAV_CMD = 186
	// MAV_CMD_DO_SET_ACTUATOR enum. Sets actuators (e.g. servos) to a desired value. The actuator numbers are mapped to specific outputs (e.g. on any MAIN or AUX PWM or UAVCAN) using a flight-stack specific mechanism (i.e. a parameter). Params: 1) Actuator 1 value, scaled from [-1 to 1]. NaN to ignore.; 2) Actuator 2 value, scaled from [-1 to 1]. NaN to ignore.; 3) Actuator 3 value, scaled from [-1 to 1]. NaN to ignore.; 4) Actuator 4 value, scaled from [-1 to 1]. NaN to ignore.; 5) Actuator 5 value, scaled from [-1 to 1]. NaN to ignore.; 6) Actuator 6 value, scaled from [-1 to 1]. NaN to ignore.; 7) Index of actuator set (i.e if set to 1, Actuator 1 becomes Actuator 7);
	MAV_CMD_DO_SET_ACTUATOR MAV_CMD = 187
	// MAV_CMD_DO_LAND_START enum. Mission command to perform a landing. This is used as a marker in a mission to tell the autopilot where a sequence of mission items that represents a landing starts. It may also be sent via a COMMAND_LONG to trigger a landing, in which case the nearest (geographically) landing sequence in the mission will be used. The Latitude/Longitude is optional, and may be set to 0 if not needed. If specified then it will be used to help find the closest landing sequence. Params: 1) Empty; 2) Empty; 3) Empty; 4) Empty; 5) Latitude; 6) Longitude; 7) Empty;
	MAV_CMD_DO_LAND_START MAV_CMD = 189
	// MAV_CMD_DO_RALLY_LAND enum. Mission command to perform a landing from a rally point. Params: 1) Break altitude; 2) Landing speed; 3) Empty; 4) Empty; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_DO_RALLY_LAND MAV_CMD = 190
	// MAV_CMD_DO_GO_AROUND enum. Mission command to safely abort an autonomous landing. Params: 1) Altitude; 2) Empty; 3) Empty; 4) Empty; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_DO_GO_AROUND MAV_CMD = 191
	// MAV_CMD_DO_REPOSITION enum. Reposition the vehicle to a specific WGS84 global position. Params: 1) Ground speed, less than 0 (-1) for default; 2) Bitmask of option flags.; 3) Reserved; 4) Yaw heading. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.). For planes indicates loiter direction (0: clockwise, 1: counter clockwise); 5) Latitude; 6) Longitude; 7) Altitude;
	MAV_CMD_DO_REPOSITION MAV_CMD = 192
	// MAV_CMD_DO_PAUSE_CONTINUE enum. If in a GPS controlled position mode, hold the current position or continue. Params: 1) 0: Pause current mission or reposition command, hold current position. 1: Continue mission. A VTOL capable vehicle should enter hover mode (multicopter and VTOL planes). A plane should loiter with the default loiter radius.; 2) Reserved; 3) Reserved; 4) Reserved; 5) Reserved; 6) Reserved; 7) Reserved;
	MAV_CMD_DO_PAUSE_CONTINUE MAV_CMD = 193
	// MAV_CMD_DO_SET_REVERSE enum. Set moving direction to forward or reverse. Params: 1) Direction (0=Forward, 1=Reverse); 2) Empty; 3) Empty; 4) Empty; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_DO_SET_REVERSE MAV_CMD = 194
	// MAV_CMD_DO_SET_ROI_LOCATION enum. Sets the region of interest (ROI) to a location. This can then be used by the vehicle's control system to control the vehicle attitude and the attitude of various sensors such as cameras. This command can be sent to a gimbal manager but not to a gimbal device. A gimbal is not to react to this message. Params: 1) Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals).; 2) Empty; 3) Empty; 4) Empty; 5) Latitude of ROI location; 6) Longitude of ROI location; 7) Altitude of ROI location;
	MAV_CMD_DO_SET_ROI_LOCATION MAV_CMD = 195
	// MAV_CMD_DO_SET_ROI_WPNEXT_OFFSET enum. Sets the region of interest (ROI) to be toward next waypoint, with optional pitch/roll/yaw offset. This can then be used by the vehicle's control system to control the vehicle attitude and the attitude of various sensors such as cameras. This command can be sent to a gimbal manager but not to a gimbal device. A gimbal device is not to react to this message. Params: 1) Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals).; 2) Empty; 3) Empty; 4) Empty; 5) Pitch offset from next waypoint, positive pitching up; 6) roll offset from next waypoint, positive rolling to the right; 7) yaw offset from next waypoint, positive yawing to the right;
	MAV_CMD_DO_SET_ROI_WPNEXT_OFFSET MAV_CMD = 196
	// MAV_CMD_DO_SET_ROI_NONE enum. Cancels any previous ROI command returning the vehicle/sensors to default flight characteristics. This can then be used by the vehicle's control system to control the vehicle attitude and the attitude of various sensors such as cameras. This command can be sent to a gimbal manager but not to a gimbal device. A gimbal device is not to react to this message. After this command the gimbal manager should go back to manual input if available, and otherwise assume a neutral position. Params: 1) Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals).; 2) Empty; 3) Empty; 4) Empty; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_DO_SET_ROI_NONE MAV_CMD = 197
	// MAV_CMD_DO_SET_ROI_SYSID enum. Mount tracks system with specified system ID. Determination of target vehicle position may be done with GLOBAL_POSITION_INT or any other means. This command can be sent to a gimbal manager but not to a gimbal device. A gimbal device is not to react to this message. Params: 1) System ID; 2) Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals).;
	MAV_CMD_DO_SET_ROI_SYSID MAV_CMD = 198
	// MAV_CMD_DO_CONTROL_VIDEO enum. Control onboard camera system. Params: 1) Camera ID (-1 for all); 2) Transmission: 0: disabled, 1: enabled compressed, 2: enabled raw; 3) Transmission mode: 0: video stream, &gt;0: single images every n seconds; 4) Recording: 0: disabled, 1: enabled compressed, 2: enabled raw; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_DO_CONTROL_VIDEO MAV_CMD = 200
	// MAV_CMD_DO_SET_ROI enum. Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicle's control system to control the vehicle attitude and the attitude of various sensors such as cameras. Params: 1) Region of interest mode.; 2) Waypoint index/ target ID (depends on param 1).; 3) Region of interest index. (allows a vehicle to manage multiple ROI's); 4) Empty; 5) MAV_ROI_WPNEXT: pitch offset from next waypoint, MAV_ROI_LOCATION: latitude; 6) MAV_ROI_WPNEXT: roll offset from next waypoint, MAV_ROI_LOCATION: longitude; 7) MAV_ROI_WPNEXT: yaw offset from next waypoint, MAV_ROI_LOCATION: altitude;
	MAV_CMD_DO_SET_ROI MAV_CMD = 201
	// MAV_CMD_DO_DIGICAM_CONFIGURE enum. Configure digital camera. This is a fallback message for systems that have not yet implemented PARAM_EXT_XXX messages and camera definition files (see https://mavlink.io/en/services/camera_def.html ). Params: 1) Modes: P, TV, AV, M, Etc.; 2) Shutter speed: Divisor number for one second.; 3) Aperture: F stop number.; 4) ISO number e.g. 80, 100, 200, Etc.; 5) Exposure type enumerator.; 6) Command Identity.; 7) Main engine cut-off time before camera trigger. (0 means no cut-off);
	MAV_CMD_DO_DIGICAM_CONFIGURE MAV_CMD = 202
	// MAV_CMD_DO_DIGICAM_CONTROL enum. Control digital camera. This is a fallback message for systems that have not yet implemented PARAM_EXT_XXX messages and camera definition files (see https://mavlink.io/en/services/camera_def.html ). Params: 1) Session control e.g. show/hide lens; 2) Zoom's absolute position; 3) Zooming step value to offset zoom from the current position; 4) Focus Locking, Unlocking or Re-locking; 5) Shooting Command; 6) Command Identity; 7) Test shot identifier. If set to 1, image will only be captured, but not counted towards internal frame count.;
	MAV_CMD_DO_DIGICAM_CONTROL MAV_CMD = 203
	// MAV_CMD_DO_MOUNT_CONFIGURE enum. Mission command to configure a camera or antenna mount. Params: 1) Mount operation mode; 2) stabilize roll? (1 = yes, 0 = no); 3) stabilize pitch? (1 = yes, 0 = no); 4) stabilize yaw? (1 = yes, 0 = no); 5) roll input (0 = angle body frame, 1 = angular rate, 2 = angle absolute frame); 6) pitch input (0 = angle body frame, 1 = angular rate, 2 = angle absolute frame); 7) yaw input (0 = angle body frame, 1 = angular rate, 2 = angle absolute frame);
	MAV_CMD_DO_MOUNT_CONFIGURE MAV_CMD = 204
	// MAV_CMD_DO_MOUNT_CONTROL enum. Mission command to control a camera or antenna mount. Params: 1) pitch depending on mount mode (degrees or degrees/second depending on pitch input).; 2) roll depending on mount mode (degrees or degrees/second depending on roll input).; 3) yaw depending on mount mode (degrees or degrees/second depending on yaw input).; 4) altitude depending on mount mode.; 5) latitude, set if appropriate mount mode.; 6) longitude, set if appropriate mount mode.; 7) Mount mode.;
	MAV_CMD_DO_MOUNT_CONTROL MAV_CMD = 205
	// MAV_CMD_DO_SET_CAM_TRIGG_DIST enum. Mission command to set camera trigger distance for this flight. The camera is triggered each time this distance is exceeded. This command can also be used to set the shutter integration time for the camera. Params: 1) Camera trigger distance. 0 to stop triggering.; 2) Camera shutter integration time. -1 or 0 to ignore; 3) Trigger camera once immediately. (0 = no trigger, 1 = trigger); 4) Empty; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_DO_SET_CAM_TRIGG_DIST MAV_CMD = 206
	// MAV_CMD_DO_FENCE_ENABLE enum. Mission command to enable the geofence. Params: 1) enable? (0=disable, 1=enable, 2=disable_floor_only); 2) Empty; 3) Empty; 4) Empty; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_DO_FENCE_ENABLE MAV_CMD = 207
	// MAV_CMD_DO_PARACHUTE enum. Mission item/command to release a parachute or enable/disable auto release. Params: 1) Action; 2) Empty; 3) Empty; 4) Empty; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_DO_PARACHUTE MAV_CMD = 208
	// MAV_CMD_DO_MOTOR_TEST enum. Mission command to perform motor test. Params: 1) Motor instance number. (from 1 to max number of motors on the vehicle); 2) Throttle type.; 3) Throttle.; 4) Timeout.; 5) Motor count. (number of motors to test to test in sequence, waiting for the timeout above between them; 0=1 motor, 1=1 motor, 2=2 motors...); 6) Motor test order.; 7) Empty;
	MAV_CMD_DO_MOTOR_TEST MAV_CMD = 209
	// MAV_CMD_DO_INVERTED_FLIGHT enum. Change to/from inverted flight. Params: 1) Inverted flight. (0=normal, 1=inverted); 2) Empty; 3) Empty; 4) Empty; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_DO_INVERTED_FLIGHT MAV_CMD = 210
	// MAV_CMD_DO_GRIPPER enum. Mission command to operate a gripper. Params: 1) Gripper instance number.; 2) Gripper action to perform.; 3) Empty; 4) Empty; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_DO_GRIPPER MAV_CMD = 211
	// MAV_CMD_DO_AUTOTUNE_ENABLE enum. Enable/disable autotune. Params: 1) Enable (1: enable, 0:disable).; 2) Empty.; 3) Empty.; 4) Empty.; 5) Empty.; 6) Empty.; 7) Empty.;
	MAV_CMD_DO_AUTOTUNE_ENABLE MAV_CMD = 212
	// MAV_CMD_NAV_SET_YAW_SPEED enum. Sets a desired vehicle turn angle and speed change. Params: 1) Yaw angle to adjust steering by.; 2) Speed.; 3) Final angle. (0=absolute, 1=relative); 4) Empty; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_NAV_SET_YAW_SPEED MAV_CMD = 213
	// MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL enum. Mission command to set camera trigger interval for this flight. If triggering is enabled, the camera is triggered each time this interval expires. This command can also be used to set the shutter integration time for the camera. Params: 1) Camera trigger cycle time. -1 or 0 to ignore.; 2) Camera shutter integration time. Should be less than trigger cycle time. -1 or 0 to ignore.; 3) Empty; 4) Empty; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL MAV_CMD = 214
	// MAV_CMD_DO_MOUNT_CONTROL_QUAT enum. Mission command to control a camera or antenna mount, using a quaternion as reference. Params: 1) quaternion param q1, w (1 in null-rotation); 2) quaternion param q2, x (0 in null-rotation); 3) quaternion param q3, y (0 in null-rotation); 4) quaternion param q4, z (0 in null-rotation); 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_DO_MOUNT_CONTROL_QUAT MAV_CMD = 220
	// MAV_CMD_DO_GUIDED_MASTER enum. set id of master controller. Params: 1) System ID; 2) Component ID; 3) Empty; 4) Empty; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_DO_GUIDED_MASTER MAV_CMD = 221
	// MAV_CMD_DO_GUIDED_LIMITS enum. Set limits for external control. Params: 1) Timeout - maximum time that external controller will be allowed to control vehicle. 0 means no timeout.; 2) Altitude (MSL) min - if vehicle moves below this alt, the command will be aborted and the mission will continue. 0 means no lower altitude limit.; 3) Altitude (MSL) max - if vehicle moves above this alt, the command will be aborted and the mission will continue. 0 means no upper altitude limit.; 4) Horizontal move limit - if vehicle moves more than this distance from its location at the moment the command was executed, the command will be aborted and the mission will continue. 0 means no horizontal move limit.; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_DO_GUIDED_LIMITS MAV_CMD = 222
	// MAV_CMD_DO_ENGINE_CONTROL enum. Control vehicle engine. This is interpreted by the vehicles engine controller to change the target engine state. It is intended for vehicles with internal combustion engines. Params: 1) 0: Stop engine, 1:Start Engine; 2) 0: Warm start, 1:Cold start. Controls use of choke where applicable; 3) Height delay. This is for commanding engine start only after the vehicle has gained the specified height. Used in VTOL vehicles during takeoff to start engine after the aircraft is off the ground. Zero for no delay.; 4) Empty; 5) Empty; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_DO_ENGINE_CONTROL MAV_CMD = 223
	// MAV_CMD_DO_SET_MISSION_CURRENT enum. Set the mission item with sequence number seq as current item. This means that the MAV will continue to this mission item on the shortest path (not following the mission items in-between). Params: 1) Mission sequence value to set; 2) Empty; 3) Empty; 4) Empty; 5) Empty; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_DO_SET_MISSION_CURRENT MAV_CMD = 224
	// MAV_CMD_DO_LAST enum. NOP - This command is only used to mark the upper limit of the DO commands in the enumeration. Params: 1) Empty; 2) Empty; 3) Empty; 4) Empty; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_DO_LAST MAV_CMD = 240
	// MAV_CMD_PREFLIGHT_CALIBRATION enum. Trigger calibration. This command will be only accepted if in pre-flight mode. Except for Temperature Calibration, only one sensor should be set in a single message and all others should be zero. Params: 1) 1: gyro calibration, 3: gyro temperature calibration; 2) 1: magnetometer calibration; 3) 1: ground pressure calibration; 4) 1: radio RC calibration, 2: RC trim calibration; 5) 1: accelerometer calibration, 2: board level calibration, 3: accelerometer temperature calibration, 4: simple accelerometer calibration; 6) 1: APM: compass/motor interference calibration (PX4: airspeed calibration, deprecated), 2: airspeed calibration; 7) 1: ESC calibration, 3: barometer temperature calibration;
	MAV_CMD_PREFLIGHT_CALIBRATION MAV_CMD = 241
	// MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS enum. Set sensor offsets. This command will be only accepted if in pre-flight mode. Params: 1) Sensor to adjust the offsets for: 0: gyros, 1: accelerometer, 2: magnetometer, 3: barometer, 4: optical flow, 5: second magnetometer, 6: third magnetometer; 2) X axis offset (or generic dimension 1), in the sensor's raw units; 3) Y axis offset (or generic dimension 2), in the sensor's raw units; 4) Z axis offset (or generic dimension 3), in the sensor's raw units; 5) Generic dimension 4, in the sensor's raw units; 6) Generic dimension 5, in the sensor's raw units; 7) Generic dimension 6, in the sensor's raw units;
	MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS MAV_CMD = 242
	// MAV_CMD_PREFLIGHT_UAVCAN enum. Trigger UAVCAN configuration (actuator ID assignment and direction mapping). Note that this maps to the legacy UAVCAN v0 function UAVCAN_ENUMERATE, which is intended to be executed just once during initial vehicle configuration (it is not a normal pre-flight command and has been poorly named). Params: 1) 1: Trigger actuator ID assignment and direction mapping. 0: Cancel command.; 2) Reserved; 3) Reserved; 4) Reserved; 5) Reserved; 6) Reserved; 7) Reserved;
	MAV_CMD_PREFLIGHT_UAVCAN MAV_CMD = 243
	// MAV_CMD_PREFLIGHT_STORAGE enum. Request storage of different parameter values and logs. This command will be only accepted if in pre-flight mode. Params: 1) Parameter storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM, 2: Reset to defaults; 2) Mission storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM, 2: Reset to defaults; 3) Onboard logging: 0: Ignore, 1: Start default rate logging, -1: Stop logging, &gt; 1: logging rate (e.g. set to 1000 for 1000 Hz logging); 4) Reserved; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_PREFLIGHT_STORAGE MAV_CMD = 245
	// MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN enum. Request the reboot or shutdown of system components. Params: 1) 0: Do nothing for autopilot, 1: Reboot autopilot, 2: Shutdown autopilot, 3: Reboot autopilot and keep it in the bootloader until upgraded.; 2) 0: Do nothing for onboard computer, 1: Reboot onboard computer, 2: Shutdown onboard computer, 3: Reboot onboard computer and keep it in the bootloader until upgraded.; 3) WIP: 0: Do nothing for camera, 1: Reboot onboard camera, 2: Shutdown onboard camera, 3: Reboot onboard camera and keep it in the bootloader until upgraded; 4) WIP: 0: Do nothing for mount (e.g. gimbal), 1: Reboot mount, 2: Shutdown mount, 3: Reboot mount and keep it in the bootloader until upgraded; 5) Reserved (set to 0); 6) Reserved (set to 0); 7) WIP: ID (e.g. camera ID -1 for all IDs);
	MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN MAV_CMD = 246
	// MAV_CMD_DO_UPGRADE enum. Request a target system to start an upgrade of one (or all) of its components. For example, the command might be sent to a companion computer to cause it to upgrade a connected flight controller. The system doing the upgrade will report progress using the normal command protocol sequence for a long running operation. Command protocol information: https://mavlink.io/en/services/command.html. Params: 1) Component id of the component to be upgraded. If set to 0, all components should be upgraded.; 2) 0: Do not reboot component after the action is executed, 1: Reboot component after the action is executed.; 3) Reserved; 4) Reserved; 5) Reserved; 6) Reserved; 7) WIP: upgrade progress report rate (can be used for more granular control).;
	MAV_CMD_DO_UPGRADE MAV_CMD = 247
	// MAV_CMD_OVERRIDE_GOTO enum. Override current mission with command to pause mission, pause mission and move to position, continue/resume mission. When param 1 indicates that the mission is paused (MAV_GOTO_DO_HOLD), param 2 defines whether it holds in place or moves to another position. Params: 1) MAV_GOTO_DO_HOLD: pause mission and either hold or move to specified position (depending on param2), MAV_GOTO_DO_CONTINUE: resume mission.; 2) MAV_GOTO_HOLD_AT_CURRENT_POSITION: hold at current position, MAV_GOTO_HOLD_AT_SPECIFIED_POSITION: hold at specified position.; 3) Coordinate frame of hold point.; 4) Desired yaw angle.; 5) Latitude/X position.; 6) Longitude/Y position.; 7) Altitude/Z position.;
	MAV_CMD_OVERRIDE_GOTO MAV_CMD = 252
	// MAV_CMD_OBLIQUE_SURVEY enum. Mission command to set a Camera Auto Mount Pivoting Oblique Survey (Replaces CAM_TRIGG_DIST for this purpose). The camera is triggered each time this distance is exceeded, then the mount moves to the next position. Params 4~6 set-up the angle limits and number of positions for oblique survey, where mount-enabled vehicles automatically roll the camera between shots to emulate an oblique camera setup (providing an increased HFOV). This command can also be used to set the shutter integration time for the camera. Params: 1) Camera trigger distance. 0 to stop triggering.; 2) Camera shutter integration time. 0 to ignore; 3) The minimum interval in which the camera is capable of taking subsequent pictures repeatedly. 0 to ignore.; 4) Total number of roll positions at which the camera will capture photos (images captures spread evenly across the limits defined by param5).; 5) Angle limits that the camera can be rolled to left and right of center.; 6) Fixed pitch angle that the camera will hold in oblique mode if the mount is actuated in the pitch axis.; 7) Empty;
	MAV_CMD_OBLIQUE_SURVEY MAV_CMD = 260
	// MAV_CMD_MISSION_START enum. start running a mission. Params: 1) first_item: the first mission item to run; 2) last_item:  the last mission item to run (after this item is run, the mission ends);
	MAV_CMD_MISSION_START MAV_CMD = 300
	// MAV_CMD_COMPONENT_ARM_DISARM enum. Arms / Disarms a component. Params: 1) 0: disarm, 1: arm; 2) 0: arm-disarm unless prevented by safety checks (i.e. when landed), 21196: force arming/disarming (e.g. allow arming to override preflight checks and disarming in flight);
	MAV_CMD_COMPONENT_ARM_DISARM MAV_CMD = 400
	// MAV_CMD_ILLUMINATOR_ON_OFF enum. Turns illuminators ON/OFF. An illuminator is a light source that is used for lighting up dark areas external to the sytstem: e.g. a torch or searchlight (as opposed to a light source for illuminating the system itself, e.g. an indicator light). Params: 1) 0: Illuminators OFF, 1: Illuminators ON;
	MAV_CMD_ILLUMINATOR_ON_OFF MAV_CMD = 405
	// MAV_CMD_GET_HOME_POSITION enum. Request the home position from the vehicle. Params: 1) Reserved; 2) Reserved; 3) Reserved; 4) Reserved; 5) Reserved; 6) Reserved; 7) Reserved;
	MAV_CMD_GET_HOME_POSITION MAV_CMD = 410
	// MAV_CMD_INJECT_FAILURE enum. Inject artificial failure for testing purposes. Note that autopilots should implement an additional protection before accepting this command such as a specific param setting. Params: 1) The unit which is affected by the failure.; 2) The type how the failure manifests itself.; 3) Instance affected by failure (0 to signal all).;
	MAV_CMD_INJECT_FAILURE MAV_CMD = 420
	// MAV_CMD_START_RX_PAIR enum. Starts receiver pairing. Params: 1) 0:Spektrum.; 2) RC type.;
	MAV_CMD_START_RX_PAIR MAV_CMD = 500
	// MAV_CMD_GET_MESSAGE_INTERVAL enum. Request the interval between messages for a particular MAVLink message ID. The receiver should ACK the command and then emit its response in a MESSAGE_INTERVAL message. Params: 1) The MAVLink message ID;
	MAV_CMD_GET_MESSAGE_INTERVAL MAV_CMD = 510
	// MAV_CMD_SET_MESSAGE_INTERVAL enum. Set the interval between messages for a particular MAVLink message ID. This interface replaces REQUEST_DATA_STREAM. Params: 1) The MAVLink message ID; 2) The interval between two messages. Set to -1 to disable and 0 to request default rate.; 7) Target address of message stream (if message has target address fields). 0: Flight-stack default (recommended), 1: address of requestor, 2: broadcast.;
	MAV_CMD_SET_MESSAGE_INTERVAL MAV_CMD = 511
	// MAV_CMD_REQUEST_MESSAGE enum. Request the target system(s) emit a single instance of a specified message (i.e. a "one-shot" version of MAV_CMD_SET_MESSAGE_INTERVAL). Params: 1) The MAVLink message ID of the requested message.; 2) Use for index ID, if required. Otherwise, the use of this parameter (if any) must be defined in the requested message. By default assumed not used (0).; 3) The use of this parameter (if any), must be defined in the requested message. By default assumed not used (0).; 4) The use of this parameter (if any), must be defined in the requested message. By default assumed not used (0).; 5) The use of this parameter (if any), must be defined in the requested message. By default assumed not used (0).; 6) The use of this parameter (if any), must be defined in the requested message. By default assumed not used (0).; 7) Target address for requested message (if message has target address fields). 0: Flight-stack default, 1: address of requestor, 2: broadcast.;
	MAV_CMD_REQUEST_MESSAGE MAV_CMD = 512
	// MAV_CMD_REQUEST_PROTOCOL_VERSION enum. Request MAVLink protocol version compatibility. All receivers should ACK the command and then emit their capabilities in an PROTOCOL_VERSION message. Params: 1) 1: Request supported protocol versions by all nodes on the network; 2) Reserved (all remaining params);
	MAV_CMD_REQUEST_PROTOCOL_VERSION MAV_CMD = 519
	// MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES enum. Request autopilot capabilities. The receiver should ACK the command and then emit its capabilities in an AUTOPILOT_VERSION message. Params: 1) 1: Request autopilot version; 2) Reserved (all remaining params);
	MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES MAV_CMD = 520
	// MAV_CMD_REQUEST_CAMERA_INFORMATION enum. Request camera information (CAMERA_INFORMATION). Params: 1) 0: No action 1: Request camera capabilities; 2) Reserved (all remaining params);
	MAV_CMD_REQUEST_CAMERA_INFORMATION MAV_CMD = 521
	// MAV_CMD_REQUEST_CAMERA_SETTINGS enum. Request camera settings (CAMERA_SETTINGS). Params: 1) 0: No Action 1: Request camera settings; 2) Reserved (all remaining params);
	MAV_CMD_REQUEST_CAMERA_SETTINGS MAV_CMD = 522
	// MAV_CMD_REQUEST_STORAGE_INFORMATION enum. Request storage information (STORAGE_INFORMATION). Use the command's target_component to target a specific component's storage. Params: 1) Storage ID (0 for all, 1 for first, 2 for second, etc.); 2) 0: No Action 1: Request storage information; 3) Reserved (all remaining params);
	MAV_CMD_REQUEST_STORAGE_INFORMATION MAV_CMD = 525
	// MAV_CMD_STORAGE_FORMAT enum. Format a storage medium. Once format is complete, a STORAGE_INFORMATION message is sent. Use the command's target_component to target a specific component's storage. Params: 1) Storage ID (1 for first, 2 for second, etc.); 2) Format storage (and reset image log). 0: No action 1: Format storage; 3) Reset Image Log (without formatting storage medium). This will reset CAMERA_CAPTURE_STATUS.image_count and CAMERA_IMAGE_CAPTURED.image_index. 0: No action 1: Reset Image Log; 4) Reserved (all remaining params);
	MAV_CMD_STORAGE_FORMAT MAV_CMD = 526
	// MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS enum. Request camera capture status (CAMERA_CAPTURE_STATUS). Params: 1) 0: No Action 1: Request camera capture status; 2) Reserved (all remaining params);
	MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS MAV_CMD = 527
	// MAV_CMD_REQUEST_FLIGHT_INFORMATION enum. Request flight information (FLIGHT_INFORMATION). Params: 1) 1: Request flight information; 2) Reserved (all remaining params);
	MAV_CMD_REQUEST_FLIGHT_INFORMATION MAV_CMD = 528
	// MAV_CMD_RESET_CAMERA_SETTINGS enum. Reset all camera settings to Factory Default. Params: 1) 0: No Action 1: Reset all settings; 2) Reserved (all remaining params);
	MAV_CMD_RESET_CAMERA_SETTINGS MAV_CMD = 529
	// MAV_CMD_SET_CAMERA_MODE enum. Set camera running mode. Use NaN for reserved values. GCS will send a MAV_CMD_REQUEST_VIDEO_STREAM_STATUS command after a mode change if the camera supports video streaming. Params: 1) Reserved (Set to 0); 2) Camera mode; 3) ; 4) ; 7) ;
	MAV_CMD_SET_CAMERA_MODE MAV_CMD = 530
	// MAV_CMD_SET_CAMERA_ZOOM enum. Set camera zoom. Camera must respond with a CAMERA_SETTINGS message (on success). Params: 1) Zoom type; 2) Zoom value. The range of valid values depend on the zoom type.; 3) ; 4) ; 7) ;
	MAV_CMD_SET_CAMERA_ZOOM MAV_CMD = 531
	// MAV_CMD_SET_CAMERA_FOCUS enum. Set camera focus. Camera must respond with a CAMERA_SETTINGS message (on success). Params: 1) Focus type; 2) Focus value; 3) ; 4) ; 7) ;
	MAV_CMD_SET_CAMERA_FOCUS MAV_CMD = 532
	// MAV_CMD_JUMP_TAG enum. Tagged jump target. Can be jumped to with MAV_CMD_DO_JUMP_TAG. Params: 1) Tag.;
	MAV_CMD_JUMP_TAG MAV_CMD = 600
	// MAV_CMD_DO_JUMP_TAG enum. Jump to the matching tag in the mission list. Repeat this action for the specified number of times. A mission should contain a single matching tag for each jump. If this is not the case then a jump to a missing tag should complete the mission, and a jump where there are multiple matching tags should always select the one with the lowest mission sequence number. Params: 1) Target tag to jump to.; 2) Repeat count.;
	MAV_CMD_DO_JUMP_TAG MAV_CMD = 601
	// MAV_CMD_PARAM_TRANSACTION enum. Request to start or end a parameter transaction. Multiple kinds of transport layers can be used to exchange parameters in the transaction (param, param_ext and mavftp). The command response can either be a success/failure or an in progress in case the receiving side takes some time to apply the parameters. Params: 1) Action to be performed (start, commit, cancel, etc.); 2) Possible transport layers to set and get parameters via mavlink during a parameter transaction.; 3) Identifier for a specific transaction.;
	MAV_CMD_PARAM_TRANSACTION MAV_CMD = 900
	// MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW enum. High level setpoint to be sent to a gimbal manager to set a gimbal attitude. It is possible to set combinations of the values below. E.g. an angle as well as a desired angular rate can be used to get to this angle at a certain angular rate, or an angular rate only will result in continuous turning. NaN is to be used to signal unset. Note: a gimbal is never to react to this command but only the gimbal manager. Params: 1) Pitch angle (positive to pitch up, relative to vehicle for FOLLOW mode, relative to world horizon for LOCK mode).; 2) Yaw angle (positive to yaw to the right, relative to vehicle for FOLLOW mode, absolute to North for LOCK mode).; 3) Pitch rate (positive to pitch up).; 4) Yaw rate (positive to yaw to the right).; 5) Gimbal manager flags to use.; 7) Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals).;
	MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW MAV_CMD = 1000
	// MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE enum. Gimbal configuration to set which sysid/compid is in primary and secondary control. Params: 1) Sysid for primary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for missions where the own sysid is still unknown), -3: remove control if currently in control).; 2) Compid for primary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for missions where the own sysid is still unknown), -3: remove control if currently in control).; 3) Sysid for secondary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for missions where the own sysid is still unknown), -3: remove control if currently in control).; 4) Compid for secondary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for missions where the own sysid is still unknown), -3: remove control if currently in control).; 7) Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals).;
	MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE MAV_CMD = 1001
	// MAV_CMD_IMAGE_START_CAPTURE enum. Start image capture sequence. Sends CAMERA_IMAGE_CAPTURED after each capture. Use NaN for reserved values. Params: 1) Reserved (Set to 0); 2) Desired elapsed time between two consecutive pictures (in seconds). Minimum values depend on hardware (typically greater than 2 seconds).; 3) Total number of images to capture. 0 to capture forever/until MAV_CMD_IMAGE_STOP_CAPTURE.; 4) Capture sequence number starting from 1. This is only valid for single-capture (param3 == 1), otherwise set to 0. Increment the capture ID for each capture command to prevent double captures when a command is re-transmitted.; 5) ; 6) ; 7) ;
	MAV_CMD_IMAGE_START_CAPTURE MAV_CMD = 2000
	// MAV_CMD_IMAGE_STOP_CAPTURE enum. Stop image capture sequence Use NaN for reserved values. Params: 1) Reserved (Set to 0); 2) ; 3) ; 4) ; 7) ;
	MAV_CMD_IMAGE_STOP_CAPTURE MAV_CMD = 2001
	// MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE enum. Re-request a CAMERA_IMAGE_CAPTURED message. Params: 1) Sequence number for missing CAMERA_IMAGE_CAPTURED message; 2) ; 3) ; 4) ; 7) ;
	MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE MAV_CMD = 2002
	// MAV_CMD_DO_TRIGGER_CONTROL enum. Enable or disable on-board camera triggering system. Params: 1) Trigger enable/disable (0 for disable, 1 for start), -1 to ignore; 2) 1 to reset the trigger sequence, -1 or 0 to ignore; 3) 1 to pause triggering, but without switching the camera off or retracting it. -1 to ignore;
	MAV_CMD_DO_TRIGGER_CONTROL MAV_CMD = 2003
	// MAV_CMD_CAMERA_TRACK_POINT enum. If the camera supports point visual tracking (CAMERA_CAP_FLAGS_HAS_TRACKING_POINT is set), this command allows to initiate the tracking. Params: 1) Point to track x value (normalized 0..1, 0 is left, 1 is right).; 2) Point to track y value (normalized 0..1, 0 is top, 1 is bottom).; 3) Point radius (normalized 0..1, 0 is image left, 1 is image right).;
	MAV_CMD_CAMERA_TRACK_POINT MAV_CMD = 2004
	// MAV_CMD_CAMERA_TRACK_RECTANGLE enum. If the camera supports rectangle visual tracking (CAMERA_CAP_FLAGS_HAS_TRACKING_RECTANGLE is set), this command allows to initiate the tracking. Params: 1) Top left corner of rectangle x value (normalized 0..1, 0 is left, 1 is right).; 2) Top left corner of rectangle y value (normalized 0..1, 0 is top, 1 is bottom).; 3) Bottom right corner of rectangle x value (normalized 0..1, 0 is left, 1 is right).; 4) Bottom right corner of rectangle y value (normalized 0..1, 0 is top, 1 is bottom).;
	MAV_CMD_CAMERA_TRACK_RECTANGLE MAV_CMD = 2005
	// MAV_CMD_CAMERA_STOP_TRACKING enum. Stops ongoing tracking
	MAV_CMD_CAMERA_STOP_TRACKING MAV_CMD = 2010
	// MAV_CMD_VIDEO_START_CAPTURE enum. Starts video capture (recording). Params: 1) Video Stream ID (0 for all streams); 2) Frequency CAMERA_CAPTURE_STATUS messages should be sent while recording (0 for no messages, otherwise frequency); 3) ; 4) ; 5) ; 6) ; 7) ;
	MAV_CMD_VIDEO_START_CAPTURE MAV_CMD = 2500
	// MAV_CMD_VIDEO_STOP_CAPTURE enum. Stop the current video capture (recording). Params: 1) Video Stream ID (0 for all streams); 2) ; 3) ; 4) ; 5) ; 6) ; 7) ;
	MAV_CMD_VIDEO_STOP_CAPTURE MAV_CMD = 2501
	// MAV_CMD_VIDEO_START_STREAMING enum. Start video streaming. Params: 1) Video Stream ID (0 for all streams, 1 for first, 2 for second, etc.);
	MAV_CMD_VIDEO_START_STREAMING MAV_CMD = 2502
	// MAV_CMD_VIDEO_STOP_STREAMING enum. Stop the given video stream. Params: 1) Video Stream ID (0 for all streams, 1 for first, 2 for second, etc.);
	MAV_CMD_VIDEO_STOP_STREAMING MAV_CMD = 2503
	// MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION enum. Request video stream information (VIDEO_STREAM_INFORMATION). Params: 1) Video Stream ID (0 for all streams, 1 for first, 2 for second, etc.);
	MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION MAV_CMD = 2504
	// MAV_CMD_REQUEST_VIDEO_STREAM_STATUS enum. Request video stream status (VIDEO_STREAM_STATUS). Params: 1) Video Stream ID (0 for all streams, 1 for first, 2 for second, etc.);
	MAV_CMD_REQUEST_VIDEO_STREAM_STATUS MAV_CMD = 2505
	// MAV_CMD_LOGGING_START enum. Request to start streaming logging data over MAVLink (see also LOGGING_DATA message). Params: 1) Format: 0: ULog; 2) Reserved (set to 0); 3) Reserved (set to 0); 4) Reserved (set to 0); 5) Reserved (set to 0); 6) Reserved (set to 0); 7) Reserved (set to 0);
	MAV_CMD_LOGGING_START MAV_CMD = 2510
	// MAV_CMD_LOGGING_STOP enum. Request to stop streaming log data over MAVLink. Params: 1) Reserved (set to 0); 2) Reserved (set to 0); 3) Reserved (set to 0); 4) Reserved (set to 0); 5) Reserved (set to 0); 6) Reserved (set to 0); 7) Reserved (set to 0);
	MAV_CMD_LOGGING_STOP MAV_CMD = 2511
	// MAV_CMD_AIRFRAME_CONFIGURATION enum. Params: 1) Landing gear ID (default: 0, -1 for all); 2) Landing gear position (Down: 0, Up: 1, NaN for no change); 3) ; 4) ; 5) ; 6) ; 7) ;
	MAV_CMD_AIRFRAME_CONFIGURATION MAV_CMD = 2520
	// MAV_CMD_CONTROL_HIGH_LATENCY enum. Request to start/stop transmitting over the high latency telemetry. Params: 1) Control transmission over high latency telemetry (0: stop, 1: start); 2) Empty; 3) Empty; 4) Empty; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_CONTROL_HIGH_LATENCY MAV_CMD = 2600
	// MAV_CMD_PANORAMA_CREATE enum. Create a panorama at the current position. Params: 1) Viewing angle horizontal of the panorama (+- 0.5 the total angle); 2) Viewing angle vertical of panorama.; 3) Speed of the horizontal rotation.; 4) Speed of the vertical rotation.;
	MAV_CMD_PANORAMA_CREATE MAV_CMD = 2800
	// MAV_CMD_DO_VTOL_TRANSITION enum. Request VTOL transition. Params: 1) The target VTOL state. Only MAV_VTOL_STATE_MC and MAV_VTOL_STATE_FW can be used.;
	MAV_CMD_DO_VTOL_TRANSITION MAV_CMD = 3000
	// MAV_CMD_ARM_AUTHORIZATION_REQUEST enum. Request authorization to arm the vehicle to a external entity, the arm authorizer is responsible to request all data that is needs from the vehicle before authorize or deny the request. If approved the progress of command_ack message should be set with period of time that this authorization is valid in seconds or in case it was denied it should be set with one of the reasons in ARM_AUTH_DENIED_REASON. Params: 1) Vehicle system id, this way ground station can request arm authorization on behalf of any vehicle;
	MAV_CMD_ARM_AUTHORIZATION_REQUEST MAV_CMD = 3001
	// MAV_CMD_SET_GUIDED_SUBMODE_STANDARD enum. This command sets the submode to standard guided when vehicle is in guided mode. The vehicle holds position and altitude and the user can input the desired velocities along all three axes
	MAV_CMD_SET_GUIDED_SUBMODE_STANDARD MAV_CMD = 4000
	// MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE enum. This command sets submode circle when vehicle is in guided mode. Vehicle flies along a circle facing the center of the circle. The user can input the velocity along the circle and change the radius. If no input is given the vehicle will hold position. Params: 1) Radius of desired circle in CIRCLE_MODE; 2) User defined; 3) User defined; 4) User defined; 5) Target latitude of center of circle in CIRCLE_MODE; 6) Target longitude of center of circle in CIRCLE_MODE;
	MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE MAV_CMD = 4001
	// MAV_CMD_CONDITION_GATE enum. Delay mission state machine until gate has been reached. Params: 1) Geometry: 0: orthogonal to path between previous and next waypoint.; 2) Altitude: 0: ignore altitude; 3) Empty; 4) Empty; 5) Latitude; 6) Longitude; 7) Altitude;
	MAV_CMD_CONDITION_GATE MAV_CMD = 4501
	// MAV_CMD_NAV_FENCE_RETURN_POINT enum. Fence return point. There can only be one fence return point. Params: 1) Reserved; 2) Reserved; 3) Reserved; 4) Reserved; 5) Latitude; 6) Longitude; 7) Altitude;
	MAV_CMD_NAV_FENCE_RETURN_POINT MAV_CMD = 5000
	// MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION enum. Fence vertex for an inclusion polygon (the polygon must not be self-intersecting). The vehicle must stay within this area. Minimum of 3 vertices required. Params: 1) Polygon vertex count; 2) Vehicle must be inside ALL inclusion zones in a single group, vehicle must be inside at least one group, must be the same for all points in each polygon; 3) Reserved; 4) Reserved; 5) Latitude; 6) Longitude; 7) Reserved;
	MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION MAV_CMD = 5001
	// MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION enum. Fence vertex for an exclusion polygon (the polygon must not be self-intersecting). The vehicle must stay outside this area. Minimum of 3 vertices required. Params: 1) Polygon vertex count; 2) Reserved; 3) Reserved; 4) Reserved; 5) Latitude; 6) Longitude; 7) Reserved;
	MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION MAV_CMD = 5002
	// MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION enum. Circular fence area. The vehicle must stay inside this area. Params: 1) Radius.; 2) Vehicle must be inside ALL inclusion zones in a single group, vehicle must be inside at least one group; 3) Reserved; 4) Reserved; 5) Latitude; 6) Longitude; 7) Reserved;
	MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION MAV_CMD = 5003
	// MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION enum. Circular fence area. The vehicle must stay outside this area. Params: 1) Radius.; 2) Reserved; 3) Reserved; 4) Reserved; 5) Latitude; 6) Longitude; 7) Reserved;
	MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION MAV_CMD = 5004
	// MAV_CMD_NAV_RALLY_POINT enum. Rally point. You can have multiple rally points defined. Params: 1) Reserved; 2) Reserved; 3) Reserved; 4) Reserved; 5) Latitude; 6) Longitude; 7) Altitude;
	MAV_CMD_NAV_RALLY_POINT MAV_CMD = 5100
	// MAV_CMD_UAVCAN_GET_NODE_INFO enum. Commands the vehicle to respond with a sequence of messages UAVCAN_NODE_INFO, one message per every UAVCAN node that is online. Note that some of the response messages can be lost, which the receiver can detect easily by checking whether every received UAVCAN_NODE_STATUS has a matching message UAVCAN_NODE_INFO received earlier; if not, this command should be sent again in order to request re-transmission of the node information messages. Params: 1) Reserved (set to 0); 2) Reserved (set to 0); 3) Reserved (set to 0); 4) Reserved (set to 0); 5) Reserved (set to 0); 6) Reserved (set to 0); 7) Reserved (set to 0);
	MAV_CMD_UAVCAN_GET_NODE_INFO MAV_CMD = 5200
	// MAV_CMD_PAYLOAD_PREPARE_DEPLOY enum. Deploy payload on a Lat / Lon / Alt position. This includes the navigation to reach the required release position and velocity. Params: 1) Operation mode. 0: prepare single payload deploy (overwriting previous requests), but do not execute it. 1: execute payload deploy immediately (rejecting further deploy commands during execution, but allowing abort). 2: add payload deploy to existing deployment list.; 2) Desired approach vector in compass heading. A negative value indicates the system can define the approach vector at will.; 3) Desired ground speed at release time. This can be overridden by the airframe in case it needs to meet minimum airspeed. A negative value indicates the system can define the ground speed at will.; 4) Minimum altitude clearance to the release position. A negative value indicates the system can define the clearance at will.; 5) Latitude. Note, if used in MISSION_ITEM (deprecated) the units are degrees (unscaled); 6) Longitude. Note, if used in MISSION_ITEM (deprecated) the units are degrees (unscaled); 7) Altitude (MSL);
	MAV_CMD_PAYLOAD_PREPARE_DEPLOY MAV_CMD = 30001
	// MAV_CMD_PAYLOAD_CONTROL_DEPLOY enum. Control the payload deployment. Params: 1) Operation mode. 0: Abort deployment, continue normal mission. 1: switch to payload deployment mode. 100: delete first payload deployment request. 101: delete all payload deployment requests.; 2) Reserved; 3) Reserved; 4) Reserved; 5) Reserved; 6) Reserved; 7) Reserved;
	MAV_CMD_PAYLOAD_CONTROL_DEPLOY MAV_CMD = 30002
	// MAV_CMD_FIXED_MAG_CAL_YAW enum. Magnetometer calibration based on provided known yaw. This allows for fast calibration using WMM field tables in the vehicle, given only the known yaw of the vehicle. If Latitude and longitude are both zero then use the current vehicle location. Params: 1) Yaw of vehicle in earth frame.; 2) CompassMask, 0 for all.; 3) Latitude.; 4) Longitude.; 5) Empty.; 6) Empty.; 7) Empty.;
	MAV_CMD_FIXED_MAG_CAL_YAW MAV_CMD = 42006
	// MAV_CMD_DO_WINCH enum. Command to operate winch. Params: 1) Winch instance number.; 2) Action to perform.; 3) Length of cable to release (negative to wind).; 4) Release rate (negative to wind).; 5) Empty.; 6) Empty.; 7) Empty.;
	MAV_CMD_DO_WINCH MAV_CMD = 42600
	// MAV_CMD_WAYPOINT_USER_1 enum. User defined waypoint item. Ground Station will show the Vehicle as flying through this item. Params: 1) User defined; 2) User defined; 3) User defined; 4) User defined; 5) Latitude unscaled; 6) Longitude unscaled; 7) Altitude (MSL);
	MAV_CMD_WAYPOINT_USER_1 MAV_CMD = 31000
	// MAV_CMD_WAYPOINT_USER_2 enum. User defined waypoint item. Ground Station will show the Vehicle as flying through this item. Params: 1) User defined; 2) User defined; 3) User defined; 4) User defined; 5) Latitude unscaled; 6) Longitude unscaled; 7) Altitude (MSL);
	MAV_CMD_WAYPOINT_USER_2 MAV_CMD = 31001
	// MAV_CMD_WAYPOINT_USER_3 enum. User defined waypoint item. Ground Station will show the Vehicle as flying through this item. Params: 1) User defined; 2) User defined; 3) User defined; 4) User defined; 5) Latitude unscaled; 6) Longitude unscaled; 7) Altitude (MSL);
	MAV_CMD_WAYPOINT_USER_3 MAV_CMD = 31002
	// MAV_CMD_WAYPOINT_USER_4 enum. User defined waypoint item. Ground Station will show the Vehicle as flying through this item. Params: 1) User defined; 2) User defined; 3) User defined; 4) User defined; 5) Latitude unscaled; 6) Longitude unscaled; 7) Altitude (MSL);
	MAV_CMD_WAYPOINT_USER_4 MAV_CMD = 31003
	// MAV_CMD_WAYPOINT_USER_5 enum. User defined waypoint item. Ground Station will show the Vehicle as flying through this item. Params: 1) User defined; 2) User defined; 3) User defined; 4) User defined; 5) Latitude unscaled; 6) Longitude unscaled; 7) Altitude (MSL);
	MAV_CMD_WAYPOINT_USER_5 MAV_CMD = 31004
	// MAV_CMD_SPATIAL_USER_1 enum. User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. Params: 1) User defined; 2) User defined; 3) User defined; 4) User defined; 5) Latitude unscaled; 6) Longitude unscaled; 7) Altitude (MSL);
	MAV_CMD_SPATIAL_USER_1 MAV_CMD = 31005
	// MAV_CMD_SPATIAL_USER_2 enum. User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. Params: 1) User defined; 2) User defined; 3) User defined; 4) User defined; 5) Latitude unscaled; 6) Longitude unscaled; 7) Altitude (MSL);
	MAV_CMD_SPATIAL_USER_2 MAV_CMD = 31006
	// MAV_CMD_SPATIAL_USER_3 enum. User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. Params: 1) User defined; 2) User defined; 3) User defined; 4) User defined; 5) Latitude unscaled; 6) Longitude unscaled; 7) Altitude (MSL);
	MAV_CMD_SPATIAL_USER_3 MAV_CMD = 31007
	// MAV_CMD_SPATIAL_USER_4 enum. User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. Params: 1) User defined; 2) User defined; 3) User defined; 4) User defined; 5) Latitude unscaled; 6) Longitude unscaled; 7) Altitude (MSL);
	MAV_CMD_SPATIAL_USER_4 MAV_CMD = 31008
	// MAV_CMD_SPATIAL_USER_5 enum. User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. Params: 1) User defined; 2) User defined; 3) User defined; 4) User defined; 5) Latitude unscaled; 6) Longitude unscaled; 7) Altitude (MSL);
	MAV_CMD_SPATIAL_USER_5 MAV_CMD = 31009
	// MAV_CMD_USER_1 enum. User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. Params: 1) User defined; 2) User defined; 3) User defined; 4) User defined; 5) User defined; 6) User defined; 7) User defined;
	MAV_CMD_USER_1 MAV_CMD = 31010
	// MAV_CMD_USER_2 enum. User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. Params: 1) User defined; 2) User defined; 3) User defined; 4) User defined; 5) User defined; 6) User defined; 7) User defined;
	MAV_CMD_USER_2 MAV_CMD = 31011
	// MAV_CMD_USER_3 enum. User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. Params: 1) User defined; 2) User defined; 3) User defined; 4) User defined; 5) User defined; 6) User defined; 7) User defined;
	MAV_CMD_USER_3 MAV_CMD = 31012
	// MAV_CMD_USER_4 enum. User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. Params: 1) User defined; 2) User defined; 3) User defined; 4) User defined; 5) User defined; 6) User defined; 7) User defined;
	MAV_CMD_USER_4 MAV_CMD = 31013
	// MAV_CMD_USER_5 enum. User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. Params: 1) User defined; 2) User defined; 3) User defined; 4) User defined; 5) User defined; 6) User defined; 7) User defined;
	MAV_CMD_USER_5 MAV_CMD = 31014
)

// MAV_DATA_STREAM type. A data stream is not a fixed set of messages, but rather a      recommendation to the autopilot software. Individual autopilots may or may not obey      the recommended messages.
type MAV_DATA_STREAM int

const (
	// MAV_DATA_STREAM_ALL enum. Enable all data streams
	MAV_DATA_STREAM_ALL MAV_DATA_STREAM = 0
	// MAV_DATA_STREAM_RAW_SENSORS enum. Enable IMU_RAW, GPS_RAW, GPS_STATUS packets
	MAV_DATA_STREAM_RAW_SENSORS MAV_DATA_STREAM = 1
	// MAV_DATA_STREAM_EXTENDED_STATUS enum. Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS
	MAV_DATA_STREAM_EXTENDED_STATUS MAV_DATA_STREAM = 2
	// MAV_DATA_STREAM_RC_CHANNELS enum. Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW
	MAV_DATA_STREAM_RC_CHANNELS MAV_DATA_STREAM = 3
	// MAV_DATA_STREAM_RAW_CONTROLLER enum. Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT
	MAV_DATA_STREAM_RAW_CONTROLLER MAV_DATA_STREAM = 4
	// MAV_DATA_STREAM_POSITION enum. Enable LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT messages
	MAV_DATA_STREAM_POSITION MAV_DATA_STREAM = 6
	// MAV_DATA_STREAM_EXTRA1 enum. Dependent on the autopilot
	MAV_DATA_STREAM_EXTRA1 MAV_DATA_STREAM = 10
	// MAV_DATA_STREAM_EXTRA2 enum. Dependent on the autopilot
	MAV_DATA_STREAM_EXTRA2 MAV_DATA_STREAM = 11
	// MAV_DATA_STREAM_EXTRA3 enum. Dependent on the autopilot
	MAV_DATA_STREAM_EXTRA3 MAV_DATA_STREAM = 12
)

// MAV_ROI type. The ROI (region of interest) for the vehicle. This can be                 be used by the vehicle for camera/vehicle attitude alignment (see                 MAV_CMD_NAV_ROI).
type MAV_ROI int

const (
	// MAV_ROI_NONE enum. No region of interest
	MAV_ROI_NONE MAV_ROI = 0
	// MAV_ROI_WPNEXT enum. Point toward next waypoint, with optional pitch/roll/yaw offset
	MAV_ROI_WPNEXT MAV_ROI = 1
	// MAV_ROI_WPINDEX enum. Point toward given waypoint
	MAV_ROI_WPINDEX MAV_ROI = 2
	// MAV_ROI_LOCATION enum. Point toward fixed location
	MAV_ROI_LOCATION MAV_ROI = 3
	// MAV_ROI_TARGET enum. Point toward of given id
	MAV_ROI_TARGET MAV_ROI = 4
)

// MAV_CMD_ACK type. ACK / NACK / ERROR values as a result of MAV_CMDs and for mission item transmission.
type MAV_CMD_ACK int

const (
	// MAV_CMD_ACK_OK enum. Command / mission item is ok
	MAV_CMD_ACK_OK MAV_CMD_ACK = 0
	// MAV_CMD_ACK_ERR_FAIL enum. Generic error message if none of the other reasons fails or if no detailed error reporting is implemented
	MAV_CMD_ACK_ERR_FAIL MAV_CMD_ACK = 1
	// MAV_CMD_ACK_ERR_ACCESS_DENIED enum. The system is refusing to accept this command from this source / communication partner
	MAV_CMD_ACK_ERR_ACCESS_DENIED MAV_CMD_ACK = 2
	// MAV_CMD_ACK_ERR_NOT_SUPPORTED enum. Command or mission item is not supported, other commands would be accepted
	MAV_CMD_ACK_ERR_NOT_SUPPORTED MAV_CMD_ACK = 3
	// MAV_CMD_ACK_ERR_COORDINATE_FRAME_NOT_SUPPORTED enum. The coordinate frame of this command / mission item is not supported
	MAV_CMD_ACK_ERR_COORDINATE_FRAME_NOT_SUPPORTED MAV_CMD_ACK = 4
	// MAV_CMD_ACK_ERR_COORDINATES_OUT_OF_RANGE enum. The coordinate frame of this command is ok, but he coordinate values exceed the safety limits of this system. This is a generic error, please use the more specific error messages below if possible
	MAV_CMD_ACK_ERR_COORDINATES_OUT_OF_RANGE MAV_CMD_ACK = 5
	// MAV_CMD_ACK_ERR_X_LAT_OUT_OF_RANGE enum. The X or latitude value is out of range
	MAV_CMD_ACK_ERR_X_LAT_OUT_OF_RANGE MAV_CMD_ACK = 6
	// MAV_CMD_ACK_ERR_Y_LON_OUT_OF_RANGE enum. The Y or longitude value is out of range
	MAV_CMD_ACK_ERR_Y_LON_OUT_OF_RANGE MAV_CMD_ACK = 7
	// MAV_CMD_ACK_ERR_Z_ALT_OUT_OF_RANGE enum. The Z or altitude value is out of range
	MAV_CMD_ACK_ERR_Z_ALT_OUT_OF_RANGE MAV_CMD_ACK = 8
)

// MAV_PARAM_TYPE type. Specifies the datatype of a MAVLink parameter.
type MAV_PARAM_TYPE int

const (
	// MAV_PARAM_TYPE_UINT8 enum. 8-bit unsigned integer
	MAV_PARAM_TYPE_UINT8 MAV_PARAM_TYPE = 1
	// MAV_PARAM_TYPE_INT8 enum. 8-bit signed integer
	MAV_PARAM_TYPE_INT8 MAV_PARAM_TYPE = 2
	// MAV_PARAM_TYPE_UINT16 enum. 16-bit unsigned integer
	MAV_PARAM_TYPE_UINT16 MAV_PARAM_TYPE = 3
	// MAV_PARAM_TYPE_INT16 enum. 16-bit signed integer
	MAV_PARAM_TYPE_INT16 MAV_PARAM_TYPE = 4
	// MAV_PARAM_TYPE_UINT32 enum. 32-bit unsigned integer
	MAV_PARAM_TYPE_UINT32 MAV_PARAM_TYPE = 5
	// MAV_PARAM_TYPE_INT32 enum. 32-bit signed integer
	MAV_PARAM_TYPE_INT32 MAV_PARAM_TYPE = 6
	// MAV_PARAM_TYPE_UINT64 enum. 64-bit unsigned integer
	MAV_PARAM_TYPE_UINT64 MAV_PARAM_TYPE = 7
	// MAV_PARAM_TYPE_INT64 enum. 64-bit signed integer
	MAV_PARAM_TYPE_INT64 MAV_PARAM_TYPE = 8
	// MAV_PARAM_TYPE_REAL32 enum. 32-bit floating-point
	MAV_PARAM_TYPE_REAL32 MAV_PARAM_TYPE = 9
	// MAV_PARAM_TYPE_REAL64 enum. 64-bit floating-point
	MAV_PARAM_TYPE_REAL64 MAV_PARAM_TYPE = 10
)

// MAV_PARAM_EXT_TYPE type. Specifies the datatype of a MAVLink extended parameter.
type MAV_PARAM_EXT_TYPE int

const (
	// MAV_PARAM_EXT_TYPE_UINT8 enum. 8-bit unsigned integer
	MAV_PARAM_EXT_TYPE_UINT8 MAV_PARAM_EXT_TYPE = 1
	// MAV_PARAM_EXT_TYPE_INT8 enum. 8-bit signed integer
	MAV_PARAM_EXT_TYPE_INT8 MAV_PARAM_EXT_TYPE = 2
	// MAV_PARAM_EXT_TYPE_UINT16 enum. 16-bit unsigned integer
	MAV_PARAM_EXT_TYPE_UINT16 MAV_PARAM_EXT_TYPE = 3
	// MAV_PARAM_EXT_TYPE_INT16 enum. 16-bit signed integer
	MAV_PARAM_EXT_TYPE_INT16 MAV_PARAM_EXT_TYPE = 4
	// MAV_PARAM_EXT_TYPE_UINT32 enum. 32-bit unsigned integer
	MAV_PARAM_EXT_TYPE_UINT32 MAV_PARAM_EXT_TYPE = 5
	// MAV_PARAM_EXT_TYPE_INT32 enum. 32-bit signed integer
	MAV_PARAM_EXT_TYPE_INT32 MAV_PARAM_EXT_TYPE = 6
	// MAV_PARAM_EXT_TYPE_UINT64 enum. 64-bit unsigned integer
	MAV_PARAM_EXT_TYPE_UINT64 MAV_PARAM_EXT_TYPE = 7
	// MAV_PARAM_EXT_TYPE_INT64 enum. 64-bit signed integer
	MAV_PARAM_EXT_TYPE_INT64 MAV_PARAM_EXT_TYPE = 8
	// MAV_PARAM_EXT_TYPE_REAL32 enum. 32-bit floating-point
	MAV_PARAM_EXT_TYPE_REAL32 MAV_PARAM_EXT_TYPE = 9
	// MAV_PARAM_EXT_TYPE_REAL64 enum. 64-bit floating-point
	MAV_PARAM_EXT_TYPE_REAL64 MAV_PARAM_EXT_TYPE = 10
	// MAV_PARAM_EXT_TYPE_CUSTOM enum. Custom Type
	MAV_PARAM_EXT_TYPE_CUSTOM MAV_PARAM_EXT_TYPE = 11
)

// MAV_RESULT type. Result from a MAVLink command (MAV_CMD)
type MAV_RESULT int

const (
	// MAV_RESULT_ACCEPTED enum. Command is valid (is supported and has valid parameters), and was executed
	MAV_RESULT_ACCEPTED MAV_RESULT = 0
	// MAV_RESULT_TEMPORARILY_REJECTED enum. Command is valid, but cannot be executed at this time. This is used to indicate a problem that should be fixed just by waiting (e.g. a state machine is busy, can't arm because have not got GPS lock, etc.). Retrying later should work
	MAV_RESULT_TEMPORARILY_REJECTED MAV_RESULT = 1
	// MAV_RESULT_DENIED enum. Command is invalid (is supported but has invalid parameters). Retrying same command and parameters will not work
	MAV_RESULT_DENIED MAV_RESULT = 2
	// MAV_RESULT_UNSUPPORTED enum. Command is not supported (unknown)
	MAV_RESULT_UNSUPPORTED MAV_RESULT = 3
	// MAV_RESULT_FAILED enum. Command is valid, but execution has failed. This is used to indicate any non-temporary or unexpected problem, i.e. any problem that must be fixed before the command can succeed/be retried. For example, attempting to write a file when out of memory, attempting to arm when sensors are not calibrated, etc
	MAV_RESULT_FAILED MAV_RESULT = 4
	// MAV_RESULT_IN_PROGRESS enum. Command is valid and is being executed. This will be followed by further progress updates, i.e. the component may send further COMMAND_ACK messages with result MAV_RESULT_IN_PROGRESS (at a rate decided by the implementation), and must terminate by sending a COMMAND_ACK message with final result of the operation. The COMMAND_ACK.progress field can be used to indicate the progress of the operation
	MAV_RESULT_IN_PROGRESS MAV_RESULT = 5
	// MAV_RESULT_CANCELLED enum. Command has been cancelled (as a result of receiving a COMMAND_CANCEL message)
	MAV_RESULT_CANCELLED MAV_RESULT = 6
)

// MAV_MISSION_RESULT type. Result of mission operation (in a MISSION_ACK message).
type MAV_MISSION_RESULT int

const (
	// MAV_MISSION_ACCEPTED enum. mission accepted OK
	MAV_MISSION_ACCEPTED MAV_MISSION_RESULT = 0
	// MAV_MISSION_ERROR enum. Generic error / not accepting mission commands at all right now
	MAV_MISSION_ERROR MAV_MISSION_RESULT = 1
	// MAV_MISSION_UNSUPPORTED_FRAME enum. Coordinate frame is not supported
	MAV_MISSION_UNSUPPORTED_FRAME MAV_MISSION_RESULT = 2
	// MAV_MISSION_UNSUPPORTED enum. Command is not supported
	MAV_MISSION_UNSUPPORTED MAV_MISSION_RESULT = 3
	// MAV_MISSION_NO_SPACE enum. Mission items exceed storage space
	MAV_MISSION_NO_SPACE MAV_MISSION_RESULT = 4
	// MAV_MISSION_INVALID enum. One of the parameters has an invalid value
	MAV_MISSION_INVALID MAV_MISSION_RESULT = 5
	// MAV_MISSION_INVALID_PARAM1 enum. param1 has an invalid value
	MAV_MISSION_INVALID_PARAM1 MAV_MISSION_RESULT = 6
	// MAV_MISSION_INVALID_PARAM2 enum. param2 has an invalid value
	MAV_MISSION_INVALID_PARAM2 MAV_MISSION_RESULT = 7
	// MAV_MISSION_INVALID_PARAM3 enum. param3 has an invalid value
	MAV_MISSION_INVALID_PARAM3 MAV_MISSION_RESULT = 8
	// MAV_MISSION_INVALID_PARAM4 enum. param4 has an invalid value
	MAV_MISSION_INVALID_PARAM4 MAV_MISSION_RESULT = 9
	// MAV_MISSION_INVALID_PARAM5_X enum. x / param5 has an invalid value
	MAV_MISSION_INVALID_PARAM5_X MAV_MISSION_RESULT = 10
	// MAV_MISSION_INVALID_PARAM6_Y enum. y / param6 has an invalid value
	MAV_MISSION_INVALID_PARAM6_Y MAV_MISSION_RESULT = 11
	// MAV_MISSION_INVALID_PARAM7 enum. z / param7 has an invalid value
	MAV_MISSION_INVALID_PARAM7 MAV_MISSION_RESULT = 12
	// MAV_MISSION_INVALID_SEQUENCE enum. Mission item received out of sequence
	MAV_MISSION_INVALID_SEQUENCE MAV_MISSION_RESULT = 13
	// MAV_MISSION_DENIED enum. Not accepting any mission commands from this communication partner
	MAV_MISSION_DENIED MAV_MISSION_RESULT = 14
	// MAV_MISSION_OPERATION_CANCELLED enum. Current mission operation cancelled (e.g. mission upload, mission download)
	MAV_MISSION_OPERATION_CANCELLED MAV_MISSION_RESULT = 15
)

// MAV_SEVERITY type. Indicates the severity level, generally used for status messages to indicate their relative urgency. Based on RFC-5424 using expanded definitions at: http://www.kiwisyslog.com/kb/info:-syslog-message-levels/.
type MAV_SEVERITY int

const (
	// MAV_SEVERITY_EMERGENCY enum. System is unusable. This is a "panic" condition
	MAV_SEVERITY_EMERGENCY MAV_SEVERITY = 0
	// MAV_SEVERITY_ALERT enum. Action should be taken immediately. Indicates error in non-critical systems
	MAV_SEVERITY_ALERT MAV_SEVERITY = 1
	// MAV_SEVERITY_CRITICAL enum. Action must be taken immediately. Indicates failure in a primary system
	MAV_SEVERITY_CRITICAL MAV_SEVERITY = 2
	// MAV_SEVERITY_ERROR enum. Indicates an error in secondary/redundant systems
	MAV_SEVERITY_ERROR MAV_SEVERITY = 3
	// MAV_SEVERITY_WARNING enum. Indicates about a possible future error if this is not resolved within a given timeframe. Example would be a low battery warning
	MAV_SEVERITY_WARNING MAV_SEVERITY = 4
	// MAV_SEVERITY_NOTICE enum. An unusual event has occurred, though not an error condition. This should be investigated for the root cause
	MAV_SEVERITY_NOTICE MAV_SEVERITY = 5
	// MAV_SEVERITY_INFO enum. Normal operational messages. Useful for logging. No action is required for these messages
	MAV_SEVERITY_INFO MAV_SEVERITY = 6
	// MAV_SEVERITY_DEBUG enum. Useful non-operational messages that can assist in debugging. These should not occur during normal operation
	MAV_SEVERITY_DEBUG MAV_SEVERITY = 7
)

// MAV_POWER_STATUS type. Power supply status flags (bitmask)
type MAV_POWER_STATUS int

const (
	// MAV_POWER_STATUS_BRICK_VALID enum. main brick power supply valid
	MAV_POWER_STATUS_BRICK_VALID MAV_POWER_STATUS = 1
	// MAV_POWER_STATUS_SERVO_VALID enum. main servo power supply valid for FMU
	MAV_POWER_STATUS_SERVO_VALID MAV_POWER_STATUS = 2
	// MAV_POWER_STATUS_USB_CONNECTED enum. USB power is connected
	MAV_POWER_STATUS_USB_CONNECTED MAV_POWER_STATUS = 4
	// MAV_POWER_STATUS_PERIPH_OVERCURRENT enum. peripheral supply is in over-current state
	MAV_POWER_STATUS_PERIPH_OVERCURRENT MAV_POWER_STATUS = 8
	// MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT enum. hi-power peripheral supply is in over-current state
	MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT MAV_POWER_STATUS = 16
	// MAV_POWER_STATUS_CHANGED enum. Power status has changed since boot
	MAV_POWER_STATUS_CHANGED MAV_POWER_STATUS = 32
)

// SERIAL_CONTROL_DEV type. SERIAL_CONTROL device types
type SERIAL_CONTROL_DEV int

const (
	// SERIAL_CONTROL_DEV_TELEM1 enum. First telemetry port
	SERIAL_CONTROL_DEV_TELEM1 SERIAL_CONTROL_DEV = 0
	// SERIAL_CONTROL_DEV_TELEM2 enum. Second telemetry port
	SERIAL_CONTROL_DEV_TELEM2 SERIAL_CONTROL_DEV = 1
	// SERIAL_CONTROL_DEV_GPS1 enum. First GPS port
	SERIAL_CONTROL_DEV_GPS1 SERIAL_CONTROL_DEV = 2
	// SERIAL_CONTROL_DEV_GPS2 enum. Second GPS port
	SERIAL_CONTROL_DEV_GPS2 SERIAL_CONTROL_DEV = 3
	// SERIAL_CONTROL_DEV_SHELL enum. system shell
	SERIAL_CONTROL_DEV_SHELL SERIAL_CONTROL_DEV = 10
	// SERIAL_CONTROL_SERIAL0 enum. SERIAL0
	SERIAL_CONTROL_SERIAL0 SERIAL_CONTROL_DEV = 100
	// SERIAL_CONTROL_SERIAL1 enum. SERIAL1
	SERIAL_CONTROL_SERIAL1 SERIAL_CONTROL_DEV = 101
	// SERIAL_CONTROL_SERIAL2 enum. SERIAL2
	SERIAL_CONTROL_SERIAL2 SERIAL_CONTROL_DEV = 102
	// SERIAL_CONTROL_SERIAL3 enum. SERIAL3
	SERIAL_CONTROL_SERIAL3 SERIAL_CONTROL_DEV = 103
	// SERIAL_CONTROL_SERIAL4 enum. SERIAL4
	SERIAL_CONTROL_SERIAL4 SERIAL_CONTROL_DEV = 104
	// SERIAL_CONTROL_SERIAL5 enum. SERIAL5
	SERIAL_CONTROL_SERIAL5 SERIAL_CONTROL_DEV = 105
	// SERIAL_CONTROL_SERIAL6 enum. SERIAL6
	SERIAL_CONTROL_SERIAL6 SERIAL_CONTROL_DEV = 106
	// SERIAL_CONTROL_SERIAL7 enum. SERIAL7
	SERIAL_CONTROL_SERIAL7 SERIAL_CONTROL_DEV = 107
	// SERIAL_CONTROL_SERIAL8 enum. SERIAL8
	SERIAL_CONTROL_SERIAL8 SERIAL_CONTROL_DEV = 108
	// SERIAL_CONTROL_SERIAL9 enum. SERIAL9
	SERIAL_CONTROL_SERIAL9 SERIAL_CONTROL_DEV = 109
)

// SERIAL_CONTROL_FLAG type. SERIAL_CONTROL flags (bitmask)
type SERIAL_CONTROL_FLAG int

const (
	// SERIAL_CONTROL_FLAG_REPLY enum. Set if this is a reply
	SERIAL_CONTROL_FLAG_REPLY SERIAL_CONTROL_FLAG = 1
	// SERIAL_CONTROL_FLAG_RESPOND enum. Set if the sender wants the receiver to send a response as another SERIAL_CONTROL message
	SERIAL_CONTROL_FLAG_RESPOND SERIAL_CONTROL_FLAG = 2
	// SERIAL_CONTROL_FLAG_EXCLUSIVE enum. Set if access to the serial port should be removed from whatever driver is currently using it, giving exclusive access to the SERIAL_CONTROL protocol. The port can be handed back by sending a request without this flag set
	SERIAL_CONTROL_FLAG_EXCLUSIVE SERIAL_CONTROL_FLAG = 4
	// SERIAL_CONTROL_FLAG_BLOCKING enum. Block on writes to the serial port
	SERIAL_CONTROL_FLAG_BLOCKING SERIAL_CONTROL_FLAG = 8
	// SERIAL_CONTROL_FLAG_MULTI enum. Send multiple replies until port is drained
	SERIAL_CONTROL_FLAG_MULTI SERIAL_CONTROL_FLAG = 16
)

// MAV_DISTANCE_SENSOR type. Enumeration of distance sensor types
type MAV_DISTANCE_SENSOR int

const (
	// MAV_DISTANCE_SENSOR_LASER enum. Laser rangefinder, e.g. LightWare SF02/F or PulsedLight units
	MAV_DISTANCE_SENSOR_LASER MAV_DISTANCE_SENSOR = 0
	// MAV_DISTANCE_SENSOR_ULTRASOUND enum. Ultrasound rangefinder, e.g. MaxBotix units
	MAV_DISTANCE_SENSOR_ULTRASOUND MAV_DISTANCE_SENSOR = 1
	// MAV_DISTANCE_SENSOR_INFRARED enum. Infrared rangefinder, e.g. Sharp units
	MAV_DISTANCE_SENSOR_INFRARED MAV_DISTANCE_SENSOR = 2
	// MAV_DISTANCE_SENSOR_RADAR enum. Radar type, e.g. uLanding units
	MAV_DISTANCE_SENSOR_RADAR MAV_DISTANCE_SENSOR = 3
	// MAV_DISTANCE_SENSOR_UNKNOWN enum. Broken or unknown type, e.g. analog units
	MAV_DISTANCE_SENSOR_UNKNOWN MAV_DISTANCE_SENSOR = 4
)

// MAV_SENSOR_ORIENTATION type. Enumeration of sensor orientation, according to its rotations
type MAV_SENSOR_ORIENTATION int

const (
	// MAV_SENSOR_ROTATION_NONE enum. Roll: 0, Pitch: 0, Yaw: 0
	MAV_SENSOR_ROTATION_NONE MAV_SENSOR_ORIENTATION = 0
	// MAV_SENSOR_ROTATION_YAW_45 enum. Roll: 0, Pitch: 0, Yaw: 45
	MAV_SENSOR_ROTATION_YAW_45 MAV_SENSOR_ORIENTATION = 1
	// MAV_SENSOR_ROTATION_YAW_90 enum. Roll: 0, Pitch: 0, Yaw: 90
	MAV_SENSOR_ROTATION_YAW_90 MAV_SENSOR_ORIENTATION = 2
	// MAV_SENSOR_ROTATION_YAW_135 enum. Roll: 0, Pitch: 0, Yaw: 135
	MAV_SENSOR_ROTATION_YAW_135 MAV_SENSOR_ORIENTATION = 3
	// MAV_SENSOR_ROTATION_YAW_180 enum. Roll: 0, Pitch: 0, Yaw: 180
	MAV_SENSOR_ROTATION_YAW_180 MAV_SENSOR_ORIENTATION = 4
	// MAV_SENSOR_ROTATION_YAW_225 enum. Roll: 0, Pitch: 0, Yaw: 225
	MAV_SENSOR_ROTATION_YAW_225 MAV_SENSOR_ORIENTATION = 5
	// MAV_SENSOR_ROTATION_YAW_270 enum. Roll: 0, Pitch: 0, Yaw: 270
	MAV_SENSOR_ROTATION_YAW_270 MAV_SENSOR_ORIENTATION = 6
	// MAV_SENSOR_ROTATION_YAW_315 enum. Roll: 0, Pitch: 0, Yaw: 315
	MAV_SENSOR_ROTATION_YAW_315 MAV_SENSOR_ORIENTATION = 7
	// MAV_SENSOR_ROTATION_ROLL_180 enum. Roll: 180, Pitch: 0, Yaw: 0
	MAV_SENSOR_ROTATION_ROLL_180 MAV_SENSOR_ORIENTATION = 8
	// MAV_SENSOR_ROTATION_ROLL_180_YAW_45 enum. Roll: 180, Pitch: 0, Yaw: 45
	MAV_SENSOR_ROTATION_ROLL_180_YAW_45 MAV_SENSOR_ORIENTATION = 9
	// MAV_SENSOR_ROTATION_ROLL_180_YAW_90 enum. Roll: 180, Pitch: 0, Yaw: 90
	MAV_SENSOR_ROTATION_ROLL_180_YAW_90 MAV_SENSOR_ORIENTATION = 10
	// MAV_SENSOR_ROTATION_ROLL_180_YAW_135 enum. Roll: 180, Pitch: 0, Yaw: 135
	MAV_SENSOR_ROTATION_ROLL_180_YAW_135 MAV_SENSOR_ORIENTATION = 11
	// MAV_SENSOR_ROTATION_PITCH_180 enum. Roll: 0, Pitch: 180, Yaw: 0
	MAV_SENSOR_ROTATION_PITCH_180 MAV_SENSOR_ORIENTATION = 12
	// MAV_SENSOR_ROTATION_ROLL_180_YAW_225 enum. Roll: 180, Pitch: 0, Yaw: 225
	MAV_SENSOR_ROTATION_ROLL_180_YAW_225 MAV_SENSOR_ORIENTATION = 13
	// MAV_SENSOR_ROTATION_ROLL_180_YAW_270 enum. Roll: 180, Pitch: 0, Yaw: 270
	MAV_SENSOR_ROTATION_ROLL_180_YAW_270 MAV_SENSOR_ORIENTATION = 14
	// MAV_SENSOR_ROTATION_ROLL_180_YAW_315 enum. Roll: 180, Pitch: 0, Yaw: 315
	MAV_SENSOR_ROTATION_ROLL_180_YAW_315 MAV_SENSOR_ORIENTATION = 15
	// MAV_SENSOR_ROTATION_ROLL_90 enum. Roll: 90, Pitch: 0, Yaw: 0
	MAV_SENSOR_ROTATION_ROLL_90 MAV_SENSOR_ORIENTATION = 16
	// MAV_SENSOR_ROTATION_ROLL_90_YAW_45 enum. Roll: 90, Pitch: 0, Yaw: 45
	MAV_SENSOR_ROTATION_ROLL_90_YAW_45 MAV_SENSOR_ORIENTATION = 17
	// MAV_SENSOR_ROTATION_ROLL_90_YAW_90 enum. Roll: 90, Pitch: 0, Yaw: 90
	MAV_SENSOR_ROTATION_ROLL_90_YAW_90 MAV_SENSOR_ORIENTATION = 18
	// MAV_SENSOR_ROTATION_ROLL_90_YAW_135 enum. Roll: 90, Pitch: 0, Yaw: 135
	MAV_SENSOR_ROTATION_ROLL_90_YAW_135 MAV_SENSOR_ORIENTATION = 19
	// MAV_SENSOR_ROTATION_ROLL_270 enum. Roll: 270, Pitch: 0, Yaw: 0
	MAV_SENSOR_ROTATION_ROLL_270 MAV_SENSOR_ORIENTATION = 20
	// MAV_SENSOR_ROTATION_ROLL_270_YAW_45 enum. Roll: 270, Pitch: 0, Yaw: 45
	MAV_SENSOR_ROTATION_ROLL_270_YAW_45 MAV_SENSOR_ORIENTATION = 21
	// MAV_SENSOR_ROTATION_ROLL_270_YAW_90 enum. Roll: 270, Pitch: 0, Yaw: 90
	MAV_SENSOR_ROTATION_ROLL_270_YAW_90 MAV_SENSOR_ORIENTATION = 22
	// MAV_SENSOR_ROTATION_ROLL_270_YAW_135 enum. Roll: 270, Pitch: 0, Yaw: 135
	MAV_SENSOR_ROTATION_ROLL_270_YAW_135 MAV_SENSOR_ORIENTATION = 23
	// MAV_SENSOR_ROTATION_PITCH_90 enum. Roll: 0, Pitch: 90, Yaw: 0
	MAV_SENSOR_ROTATION_PITCH_90 MAV_SENSOR_ORIENTATION = 24
	// MAV_SENSOR_ROTATION_PITCH_270 enum. Roll: 0, Pitch: 270, Yaw: 0
	MAV_SENSOR_ROTATION_PITCH_270 MAV_SENSOR_ORIENTATION = 25
	// MAV_SENSOR_ROTATION_PITCH_180_YAW_90 enum. Roll: 0, Pitch: 180, Yaw: 90
	MAV_SENSOR_ROTATION_PITCH_180_YAW_90 MAV_SENSOR_ORIENTATION = 26
	// MAV_SENSOR_ROTATION_PITCH_180_YAW_270 enum. Roll: 0, Pitch: 180, Yaw: 270
	MAV_SENSOR_ROTATION_PITCH_180_YAW_270 MAV_SENSOR_ORIENTATION = 27
	// MAV_SENSOR_ROTATION_ROLL_90_PITCH_90 enum. Roll: 90, Pitch: 90, Yaw: 0
	MAV_SENSOR_ROTATION_ROLL_90_PITCH_90 MAV_SENSOR_ORIENTATION = 28
	// MAV_SENSOR_ROTATION_ROLL_180_PITCH_90 enum. Roll: 180, Pitch: 90, Yaw: 0
	MAV_SENSOR_ROTATION_ROLL_180_PITCH_90 MAV_SENSOR_ORIENTATION = 29
	// MAV_SENSOR_ROTATION_ROLL_270_PITCH_90 enum. Roll: 270, Pitch: 90, Yaw: 0
	MAV_SENSOR_ROTATION_ROLL_270_PITCH_90 MAV_SENSOR_ORIENTATION = 30
	// MAV_SENSOR_ROTATION_ROLL_90_PITCH_180 enum. Roll: 90, Pitch: 180, Yaw: 0
	MAV_SENSOR_ROTATION_ROLL_90_PITCH_180 MAV_SENSOR_ORIENTATION = 31
	// MAV_SENSOR_ROTATION_ROLL_270_PITCH_180 enum. Roll: 270, Pitch: 180, Yaw: 0
	MAV_SENSOR_ROTATION_ROLL_270_PITCH_180 MAV_SENSOR_ORIENTATION = 32
	// MAV_SENSOR_ROTATION_ROLL_90_PITCH_270 enum. Roll: 90, Pitch: 270, Yaw: 0
	MAV_SENSOR_ROTATION_ROLL_90_PITCH_270 MAV_SENSOR_ORIENTATION = 33
	// MAV_SENSOR_ROTATION_ROLL_180_PITCH_270 enum. Roll: 180, Pitch: 270, Yaw: 0
	MAV_SENSOR_ROTATION_ROLL_180_PITCH_270 MAV_SENSOR_ORIENTATION = 34
	// MAV_SENSOR_ROTATION_ROLL_270_PITCH_270 enum. Roll: 270, Pitch: 270, Yaw: 0
	MAV_SENSOR_ROTATION_ROLL_270_PITCH_270 MAV_SENSOR_ORIENTATION = 35
	// MAV_SENSOR_ROTATION_ROLL_90_PITCH_180_YAW_90 enum. Roll: 90, Pitch: 180, Yaw: 90
	MAV_SENSOR_ROTATION_ROLL_90_PITCH_180_YAW_90 MAV_SENSOR_ORIENTATION = 36
	// MAV_SENSOR_ROTATION_ROLL_90_YAW_270 enum. Roll: 90, Pitch: 0, Yaw: 270
	MAV_SENSOR_ROTATION_ROLL_90_YAW_270 MAV_SENSOR_ORIENTATION = 37
	// MAV_SENSOR_ROTATION_ROLL_90_PITCH_68_YAW_293 enum. Roll: 90, Pitch: 68, Yaw: 293
	MAV_SENSOR_ROTATION_ROLL_90_PITCH_68_YAW_293 MAV_SENSOR_ORIENTATION = 38
	// MAV_SENSOR_ROTATION_PITCH_315 enum. Pitch: 315
	MAV_SENSOR_ROTATION_PITCH_315 MAV_SENSOR_ORIENTATION = 39
	// MAV_SENSOR_ROTATION_ROLL_90_PITCH_315 enum. Roll: 90, Pitch: 315
	MAV_SENSOR_ROTATION_ROLL_90_PITCH_315 MAV_SENSOR_ORIENTATION = 40
	// MAV_SENSOR_ROTATION_CUSTOM enum. Custom orientation
	MAV_SENSOR_ROTATION_CUSTOM MAV_SENSOR_ORIENTATION = 100
)

// MAV_PROTOCOL_CAPABILITY type. Bitmask of (optional) autopilot capabilities (64 bit). If a bit is set, the autopilot supports this capability.
type MAV_PROTOCOL_CAPABILITY int

const (
	// MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT enum. Autopilot supports MISSION float message type
	MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT MAV_PROTOCOL_CAPABILITY = 1
	// MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT enum. Autopilot supports the new param float message type
	MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT MAV_PROTOCOL_CAPABILITY = 2
	// MAV_PROTOCOL_CAPABILITY_MISSION_INT enum. Autopilot supports MISSION_ITEM_INT scaled integer message type
	MAV_PROTOCOL_CAPABILITY_MISSION_INT MAV_PROTOCOL_CAPABILITY = 4
	// MAV_PROTOCOL_CAPABILITY_COMMAND_INT enum. Autopilot supports COMMAND_INT scaled integer message type
	MAV_PROTOCOL_CAPABILITY_COMMAND_INT MAV_PROTOCOL_CAPABILITY = 8
	// MAV_PROTOCOL_CAPABILITY_PARAM_UNION enum. Autopilot supports the new param union message type
	MAV_PROTOCOL_CAPABILITY_PARAM_UNION MAV_PROTOCOL_CAPABILITY = 16
	// MAV_PROTOCOL_CAPABILITY_FTP enum. Autopilot supports the new FILE_TRANSFER_PROTOCOL message type
	MAV_PROTOCOL_CAPABILITY_FTP MAV_PROTOCOL_CAPABILITY = 32
	// MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET enum. Autopilot supports commanding attitude offboard
	MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET MAV_PROTOCOL_CAPABILITY = 64
	// MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED enum. Autopilot supports commanding position and velocity targets in local NED frame
	MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED MAV_PROTOCOL_CAPABILITY = 128
	// MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT enum. Autopilot supports commanding position and velocity targets in global scaled integers
	MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT MAV_PROTOCOL_CAPABILITY = 256
	// MAV_PROTOCOL_CAPABILITY_TERRAIN enum. Autopilot supports terrain protocol / data handling
	MAV_PROTOCOL_CAPABILITY_TERRAIN MAV_PROTOCOL_CAPABILITY = 512
	// MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET enum. Autopilot supports direct actuator control
	MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET MAV_PROTOCOL_CAPABILITY = 1024
	// MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION enum. Autopilot supports the flight termination command
	MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION MAV_PROTOCOL_CAPABILITY = 2048
	// MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION enum. Autopilot supports onboard compass calibration
	MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION MAV_PROTOCOL_CAPABILITY = 4096
	// MAV_PROTOCOL_CAPABILITY_MAVLINK2 enum. Autopilot supports MAVLink version 2
	MAV_PROTOCOL_CAPABILITY_MAVLINK2 MAV_PROTOCOL_CAPABILITY = 8192
	// MAV_PROTOCOL_CAPABILITY_MISSION_FENCE enum. Autopilot supports mission fence protocol
	MAV_PROTOCOL_CAPABILITY_MISSION_FENCE MAV_PROTOCOL_CAPABILITY = 16384
	// MAV_PROTOCOL_CAPABILITY_MISSION_RALLY enum. Autopilot supports mission rally point protocol
	MAV_PROTOCOL_CAPABILITY_MISSION_RALLY MAV_PROTOCOL_CAPABILITY = 32768
	// MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION enum. Autopilot supports the flight information protocol
	MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION MAV_PROTOCOL_CAPABILITY = 65536
)

// MAV_MISSION_TYPE type. Type of mission items being requested/sent in mission protocol.
type MAV_MISSION_TYPE int

const (
	// MAV_MISSION_TYPE_MISSION enum. Items are mission commands for main mission
	MAV_MISSION_TYPE_MISSION MAV_MISSION_TYPE = 0
	// MAV_MISSION_TYPE_FENCE enum. Specifies GeoFence area(s). Items are MAV_CMD_NAV_FENCE_ GeoFence items
	MAV_MISSION_TYPE_FENCE MAV_MISSION_TYPE = 1
	// MAV_MISSION_TYPE_RALLY enum. Specifies the rally points for the vehicle. Rally points are alternative RTL points. Items are MAV_CMD_NAV_RALLY_POINT rally point items
	MAV_MISSION_TYPE_RALLY MAV_MISSION_TYPE = 2
	// MAV_MISSION_TYPE_ALL enum. Only used in MISSION_CLEAR_ALL to clear all mission types
	MAV_MISSION_TYPE_ALL MAV_MISSION_TYPE = 255
)

// MAV_ESTIMATOR_TYPE type. Enumeration of estimator types
type MAV_ESTIMATOR_TYPE int

const (
	// MAV_ESTIMATOR_TYPE_UNKNOWN enum. Unknown type of the estimator
	MAV_ESTIMATOR_TYPE_UNKNOWN MAV_ESTIMATOR_TYPE = 0
	// MAV_ESTIMATOR_TYPE_NAIVE enum. This is a naive estimator without any real covariance feedback
	MAV_ESTIMATOR_TYPE_NAIVE MAV_ESTIMATOR_TYPE = 1
	// MAV_ESTIMATOR_TYPE_VISION enum. Computer vision based estimate. Might be up to scale
	MAV_ESTIMATOR_TYPE_VISION MAV_ESTIMATOR_TYPE = 2
	// MAV_ESTIMATOR_TYPE_VIO enum. Visual-inertial estimate
	MAV_ESTIMATOR_TYPE_VIO MAV_ESTIMATOR_TYPE = 3
	// MAV_ESTIMATOR_TYPE_GPS enum. Plain GPS estimate
	MAV_ESTIMATOR_TYPE_GPS MAV_ESTIMATOR_TYPE = 4
	// MAV_ESTIMATOR_TYPE_GPS_INS enum. Estimator integrating GPS and inertial sensing
	MAV_ESTIMATOR_TYPE_GPS_INS MAV_ESTIMATOR_TYPE = 5
	// MAV_ESTIMATOR_TYPE_MOCAP enum. Estimate from external motion capturing system
	MAV_ESTIMATOR_TYPE_MOCAP MAV_ESTIMATOR_TYPE = 6
	// MAV_ESTIMATOR_TYPE_LIDAR enum. Estimator based on lidar sensor input
	MAV_ESTIMATOR_TYPE_LIDAR MAV_ESTIMATOR_TYPE = 7
	// MAV_ESTIMATOR_TYPE_AUTOPILOT enum. Estimator on autopilot
	MAV_ESTIMATOR_TYPE_AUTOPILOT MAV_ESTIMATOR_TYPE = 8
)

// MAV_BATTERY_TYPE type. Enumeration of battery types
type MAV_BATTERY_TYPE int

const (
	// MAV_BATTERY_TYPE_UNKNOWN enum. Not specified
	MAV_BATTERY_TYPE_UNKNOWN MAV_BATTERY_TYPE = 0
	// MAV_BATTERY_TYPE_LIPO enum. Lithium polymer battery
	MAV_BATTERY_TYPE_LIPO MAV_BATTERY_TYPE = 1
	// MAV_BATTERY_TYPE_LIFE enum. Lithium-iron-phosphate battery
	MAV_BATTERY_TYPE_LIFE MAV_BATTERY_TYPE = 2
	// MAV_BATTERY_TYPE_LION enum. Lithium-ION battery
	MAV_BATTERY_TYPE_LION MAV_BATTERY_TYPE = 3
	// MAV_BATTERY_TYPE_NIMH enum. Nickel metal hydride battery
	MAV_BATTERY_TYPE_NIMH MAV_BATTERY_TYPE = 4
)

// MAV_BATTERY_FUNCTION type. Enumeration of battery functions
type MAV_BATTERY_FUNCTION int

const (
	// MAV_BATTERY_FUNCTION_UNKNOWN enum. Battery function is unknown
	MAV_BATTERY_FUNCTION_UNKNOWN MAV_BATTERY_FUNCTION = 0
	// MAV_BATTERY_FUNCTION_ALL enum. Battery supports all flight systems
	MAV_BATTERY_FUNCTION_ALL MAV_BATTERY_FUNCTION = 1
	// MAV_BATTERY_FUNCTION_PROPULSION enum. Battery for the propulsion system
	MAV_BATTERY_FUNCTION_PROPULSION MAV_BATTERY_FUNCTION = 2
	// MAV_BATTERY_FUNCTION_AVIONICS enum. Avionics battery
	MAV_BATTERY_FUNCTION_AVIONICS MAV_BATTERY_FUNCTION = 3
	// MAV_BATTERY_TYPE_PAYLOAD enum. Payload battery
	MAV_BATTERY_TYPE_PAYLOAD MAV_BATTERY_FUNCTION = 4
)

// MAV_BATTERY_CHARGE_STATE type. Enumeration for battery charge states.
type MAV_BATTERY_CHARGE_STATE int

const (
	// MAV_BATTERY_CHARGE_STATE_UNDEFINED enum. Low battery state is not provided
	MAV_BATTERY_CHARGE_STATE_UNDEFINED MAV_BATTERY_CHARGE_STATE = 0
	// MAV_BATTERY_CHARGE_STATE_OK enum. Battery is not in low state. Normal operation
	MAV_BATTERY_CHARGE_STATE_OK MAV_BATTERY_CHARGE_STATE = 1
	// MAV_BATTERY_CHARGE_STATE_LOW enum. Battery state is low, warn and monitor close
	MAV_BATTERY_CHARGE_STATE_LOW MAV_BATTERY_CHARGE_STATE = 2
	// MAV_BATTERY_CHARGE_STATE_CRITICAL enum. Battery state is critical, return or abort immediately
	MAV_BATTERY_CHARGE_STATE_CRITICAL MAV_BATTERY_CHARGE_STATE = 3
	// MAV_BATTERY_CHARGE_STATE_EMERGENCY enum. Battery state is too low for ordinary abort sequence. Perform fastest possible emergency stop to prevent damage
	MAV_BATTERY_CHARGE_STATE_EMERGENCY MAV_BATTERY_CHARGE_STATE = 4
	// MAV_BATTERY_CHARGE_STATE_FAILED enum. Battery failed, damage unavoidable. Possible causes (faults) are listed in MAV_BATTERY_FAULT
	MAV_BATTERY_CHARGE_STATE_FAILED MAV_BATTERY_CHARGE_STATE = 5
	// MAV_BATTERY_CHARGE_STATE_UNHEALTHY enum. Battery is diagnosed to be defective or an error occurred, usage is discouraged / prohibited. Possible causes (faults) are listed in MAV_BATTERY_FAULT
	MAV_BATTERY_CHARGE_STATE_UNHEALTHY MAV_BATTERY_CHARGE_STATE = 6
	// MAV_BATTERY_CHARGE_STATE_CHARGING enum. Battery is charging
	MAV_BATTERY_CHARGE_STATE_CHARGING MAV_BATTERY_CHARGE_STATE = 7
)

// MAV_BATTERY_MODE type. Battery mode. Note, the normal operation mode (i.e. when flying) should be reported as MAV_BATTERY_MODE_UNKNOWN to allow message trimming in normal flight.
type MAV_BATTERY_MODE int

const (
	// MAV_BATTERY_MODE_UNKNOWN enum. Battery mode not supported/unknown battery mode/normal operation
	MAV_BATTERY_MODE_UNKNOWN MAV_BATTERY_MODE = 0
	// MAV_BATTERY_MODE_AUTO_DISCHARGING enum. Battery is auto discharging (towards storage level)
	MAV_BATTERY_MODE_AUTO_DISCHARGING MAV_BATTERY_MODE = 1
	// MAV_BATTERY_MODE_HOT_SWAP enum. Battery in hot-swap mode (current limited to prevent spikes that might damage sensitive electrical circuits)
	MAV_BATTERY_MODE_HOT_SWAP MAV_BATTERY_MODE = 2
)

// MAV_BATTERY_FAULT type. Smart battery supply status/fault flags (bitmask) for health indication. The battery must also report either MAV_BATTERY_CHARGE_STATE_FAILED or MAV_BATTERY_CHARGE_STATE_UNHEALTHY if any of these are set.
type MAV_BATTERY_FAULT int

const (
	// MAV_BATTERY_FAULT_DEEP_DISCHARGE enum. Battery has deep discharged
	MAV_BATTERY_FAULT_DEEP_DISCHARGE MAV_BATTERY_FAULT = 1
	// MAV_BATTERY_FAULT_SPIKES enum. Voltage spikes
	MAV_BATTERY_FAULT_SPIKES MAV_BATTERY_FAULT = 2
	// MAV_BATTERY_FAULT_CELL_FAIL enum. One or more cells have failed. Battery should also report MAV_BATTERY_CHARGE_STATE_FAILE (and should not be used)
	MAV_BATTERY_FAULT_CELL_FAIL MAV_BATTERY_FAULT = 4
	// MAV_BATTERY_FAULT_OVER_CURRENT enum. Over-current fault
	MAV_BATTERY_FAULT_OVER_CURRENT MAV_BATTERY_FAULT = 8
	// MAV_BATTERY_FAULT_OVER_TEMPERATURE enum. Over-temperature fault
	MAV_BATTERY_FAULT_OVER_TEMPERATURE MAV_BATTERY_FAULT = 16
	// MAV_BATTERY_FAULT_UNDER_TEMPERATURE enum. Under-temperature fault
	MAV_BATTERY_FAULT_UNDER_TEMPERATURE MAV_BATTERY_FAULT = 32
	// MAV_BATTERY_FAULT_INCOMPATIBLE_VOLTAGE enum. Vehicle voltage is not compatible with this battery (batteries on same power rail should have similar voltage)
	MAV_BATTERY_FAULT_INCOMPATIBLE_VOLTAGE MAV_BATTERY_FAULT = 64
)

// MAV_GENERATOR_STATUS_FLAG type. Flags to report status/failure cases for a power generator (used in GENERATOR_STATUS). Note that FAULTS are conditions that cause the generator to fail. Warnings are conditions that require attention before the next use (they indicate the system is not operating properly).
type MAV_GENERATOR_STATUS_FLAG int

const (
	// MAV_GENERATOR_STATUS_FLAG_OFF enum. Generator is off
	MAV_GENERATOR_STATUS_FLAG_OFF MAV_GENERATOR_STATUS_FLAG = 1
	// MAV_GENERATOR_STATUS_FLAG_READY enum. Generator is ready to start generating power
	MAV_GENERATOR_STATUS_FLAG_READY MAV_GENERATOR_STATUS_FLAG = 2
	// MAV_GENERATOR_STATUS_FLAG_GENERATING enum. Generator is generating power
	MAV_GENERATOR_STATUS_FLAG_GENERATING MAV_GENERATOR_STATUS_FLAG = 4
	// MAV_GENERATOR_STATUS_FLAG_CHARGING enum. Generator is charging the batteries (generating enough power to charge and provide the load)
	MAV_GENERATOR_STATUS_FLAG_CHARGING MAV_GENERATOR_STATUS_FLAG = 8
	// MAV_GENERATOR_STATUS_FLAG_REDUCED_POWER enum. Generator is operating at a reduced maximum power
	MAV_GENERATOR_STATUS_FLAG_REDUCED_POWER MAV_GENERATOR_STATUS_FLAG = 16
	// MAV_GENERATOR_STATUS_FLAG_MAXPOWER enum. Generator is providing the maximum output
	MAV_GENERATOR_STATUS_FLAG_MAXPOWER MAV_GENERATOR_STATUS_FLAG = 32
	// MAV_GENERATOR_STATUS_FLAG_OVERTEMP_WARNING enum. Generator is near the maximum operating temperature, cooling is insufficient
	MAV_GENERATOR_STATUS_FLAG_OVERTEMP_WARNING MAV_GENERATOR_STATUS_FLAG = 64
	// MAV_GENERATOR_STATUS_FLAG_OVERTEMP_FAULT enum. Generator hit the maximum operating temperature and shutdown
	MAV_GENERATOR_STATUS_FLAG_OVERTEMP_FAULT MAV_GENERATOR_STATUS_FLAG = 128
	// MAV_GENERATOR_STATUS_FLAG_ELECTRONICS_OVERTEMP_WARNING enum. Power electronics are near the maximum operating temperature, cooling is insufficient
	MAV_GENERATOR_STATUS_FLAG_ELECTRONICS_OVERTEMP_WARNING MAV_GENERATOR_STATUS_FLAG = 256
	// MAV_GENERATOR_STATUS_FLAG_ELECTRONICS_OVERTEMP_FAULT enum. Power electronics hit the maximum operating temperature and shutdown
	MAV_GENERATOR_STATUS_FLAG_ELECTRONICS_OVERTEMP_FAULT MAV_GENERATOR_STATUS_FLAG = 512
	// MAV_GENERATOR_STATUS_FLAG_ELECTRONICS_FAULT enum. Power electronics experienced a fault and shutdown
	MAV_GENERATOR_STATUS_FLAG_ELECTRONICS_FAULT MAV_GENERATOR_STATUS_FLAG = 1024
	// MAV_GENERATOR_STATUS_FLAG_POWERSOURCE_FAULT enum. The power source supplying the generator failed e.g. mechanical generator stopped, tether is no longer providing power, solar cell is in shade, hydrogen reaction no longer happening
	MAV_GENERATOR_STATUS_FLAG_POWERSOURCE_FAULT MAV_GENERATOR_STATUS_FLAG = 2048
	// MAV_GENERATOR_STATUS_FLAG_COMMUNICATION_WARNING enum. Generator controller having communication problems
	MAV_GENERATOR_STATUS_FLAG_COMMUNICATION_WARNING MAV_GENERATOR_STATUS_FLAG = 4096
	// MAV_GENERATOR_STATUS_FLAG_COOLING_WARNING enum. Power electronic or generator cooling system error
	MAV_GENERATOR_STATUS_FLAG_COOLING_WARNING MAV_GENERATOR_STATUS_FLAG = 8192
	// MAV_GENERATOR_STATUS_FLAG_POWER_RAIL_FAULT enum. Generator controller power rail experienced a fault
	MAV_GENERATOR_STATUS_FLAG_POWER_RAIL_FAULT MAV_GENERATOR_STATUS_FLAG = 16384
	// MAV_GENERATOR_STATUS_FLAG_OVERCURRENT_FAULT enum. Generator controller exceeded the overcurrent threshold and shutdown to prevent damage
	MAV_GENERATOR_STATUS_FLAG_OVERCURRENT_FAULT MAV_GENERATOR_STATUS_FLAG = 32768
	// MAV_GENERATOR_STATUS_FLAG_BATTERY_OVERCHARGE_CURRENT_FAULT enum. Generator controller detected a high current going into the batteries and shutdown to prevent battery damage
	MAV_GENERATOR_STATUS_FLAG_BATTERY_OVERCHARGE_CURRENT_FAULT MAV_GENERATOR_STATUS_FLAG = 65536
	// MAV_GENERATOR_STATUS_FLAG_OVERVOLTAGE_FAULT enum. Generator controller exceeded it's overvoltage threshold and shutdown to prevent it exceeding the voltage rating
	MAV_GENERATOR_STATUS_FLAG_OVERVOLTAGE_FAULT MAV_GENERATOR_STATUS_FLAG = 131072
	// MAV_GENERATOR_STATUS_FLAG_BATTERY_UNDERVOLT_FAULT enum. Batteries are under voltage (generator will not start)
	MAV_GENERATOR_STATUS_FLAG_BATTERY_UNDERVOLT_FAULT MAV_GENERATOR_STATUS_FLAG = 262144
	// MAV_GENERATOR_STATUS_FLAG_START_INHIBITED enum. Generator start is inhibited by e.g. a safety switch
	MAV_GENERATOR_STATUS_FLAG_START_INHIBITED MAV_GENERATOR_STATUS_FLAG = 524288
	// MAV_GENERATOR_STATUS_FLAG_MAINTENANCE_REQUIRED enum. Generator requires maintenance
	MAV_GENERATOR_STATUS_FLAG_MAINTENANCE_REQUIRED MAV_GENERATOR_STATUS_FLAG = 1048576
	// MAV_GENERATOR_STATUS_FLAG_WARMING_UP enum. Generator is not ready to generate yet
	MAV_GENERATOR_STATUS_FLAG_WARMING_UP MAV_GENERATOR_STATUS_FLAG = 2097152
	// MAV_GENERATOR_STATUS_FLAG_IDLE enum. Generator is idle
	MAV_GENERATOR_STATUS_FLAG_IDLE MAV_GENERATOR_STATUS_FLAG = 4194304
)

// MAV_VTOL_STATE type. Enumeration of VTOL states
type MAV_VTOL_STATE int

const (
	// MAV_VTOL_STATE_UNDEFINED enum. MAV is not configured as VTOL
	MAV_VTOL_STATE_UNDEFINED MAV_VTOL_STATE = 0
	// MAV_VTOL_STATE_TRANSITION_TO_FW enum. VTOL is in transition from multicopter to fixed-wing
	MAV_VTOL_STATE_TRANSITION_TO_FW MAV_VTOL_STATE = 1
	// MAV_VTOL_STATE_TRANSITION_TO_MC enum. VTOL is in transition from fixed-wing to multicopter
	MAV_VTOL_STATE_TRANSITION_TO_MC MAV_VTOL_STATE = 2
	// MAV_VTOL_STATE_MC enum. VTOL is in multicopter state
	MAV_VTOL_STATE_MC MAV_VTOL_STATE = 3
	// MAV_VTOL_STATE_FW enum. VTOL is in fixed-wing state
	MAV_VTOL_STATE_FW MAV_VTOL_STATE = 4
)

// MAV_LANDED_STATE type. Enumeration of landed detector states
type MAV_LANDED_STATE int

const (
	// MAV_LANDED_STATE_UNDEFINED enum. MAV landed state is unknown
	MAV_LANDED_STATE_UNDEFINED MAV_LANDED_STATE = 0
	// MAV_LANDED_STATE_ON_GROUND enum. MAV is landed (on ground)
	MAV_LANDED_STATE_ON_GROUND MAV_LANDED_STATE = 1
	// MAV_LANDED_STATE_IN_AIR enum. MAV is in air
	MAV_LANDED_STATE_IN_AIR MAV_LANDED_STATE = 2
	// MAV_LANDED_STATE_TAKEOFF enum. MAV currently taking off
	MAV_LANDED_STATE_TAKEOFF MAV_LANDED_STATE = 3
	// MAV_LANDED_STATE_LANDING enum. MAV currently landing
	MAV_LANDED_STATE_LANDING MAV_LANDED_STATE = 4
)

// ADSB_ALTITUDE_TYPE type. Enumeration of the ADSB altimeter types
type ADSB_ALTITUDE_TYPE int

const (
	// ADSB_ALTITUDE_TYPE_PRESSURE_QNH enum. Altitude reported from a Baro source using QNH reference
	ADSB_ALTITUDE_TYPE_PRESSURE_QNH ADSB_ALTITUDE_TYPE = 0
	// ADSB_ALTITUDE_TYPE_GEOMETRIC enum. Altitude reported from a GNSS source
	ADSB_ALTITUDE_TYPE_GEOMETRIC ADSB_ALTITUDE_TYPE = 1
)

// ADSB_EMITTER_TYPE type. ADSB classification for the type of vehicle emitting the transponder signal
type ADSB_EMITTER_TYPE int

const (
	// ADSB_EMITTER_TYPE_NO_INFO enum
	ADSB_EMITTER_TYPE_NO_INFO ADSB_EMITTER_TYPE = 0
	// ADSB_EMITTER_TYPE_LIGHT enum
	ADSB_EMITTER_TYPE_LIGHT ADSB_EMITTER_TYPE = 1
	// ADSB_EMITTER_TYPE_SMALL enum
	ADSB_EMITTER_TYPE_SMALL ADSB_EMITTER_TYPE = 2
	// ADSB_EMITTER_TYPE_LARGE enum
	ADSB_EMITTER_TYPE_LARGE ADSB_EMITTER_TYPE = 3
	// ADSB_EMITTER_TYPE_HIGH_VORTEX_LARGE enum
	ADSB_EMITTER_TYPE_HIGH_VORTEX_LARGE ADSB_EMITTER_TYPE = 4
	// ADSB_EMITTER_TYPE_HEAVY enum
	ADSB_EMITTER_TYPE_HEAVY ADSB_EMITTER_TYPE = 5
	// ADSB_EMITTER_TYPE_HIGHLY_MANUV enum
	ADSB_EMITTER_TYPE_HIGHLY_MANUV ADSB_EMITTER_TYPE = 6
	// ADSB_EMITTER_TYPE_ROTOCRAFT enum
	ADSB_EMITTER_TYPE_ROTOCRAFT ADSB_EMITTER_TYPE = 7
	// ADSB_EMITTER_TYPE_UNASSIGNED enum
	ADSB_EMITTER_TYPE_UNASSIGNED ADSB_EMITTER_TYPE = 8
	// ADSB_EMITTER_TYPE_GLIDER enum
	ADSB_EMITTER_TYPE_GLIDER ADSB_EMITTER_TYPE = 9
	// ADSB_EMITTER_TYPE_LIGHTER_AIR enum
	ADSB_EMITTER_TYPE_LIGHTER_AIR ADSB_EMITTER_TYPE = 10
	// ADSB_EMITTER_TYPE_PARACHUTE enum
	ADSB_EMITTER_TYPE_PARACHUTE ADSB_EMITTER_TYPE = 11
	// ADSB_EMITTER_TYPE_ULTRA_LIGHT enum
	ADSB_EMITTER_TYPE_ULTRA_LIGHT ADSB_EMITTER_TYPE = 12
	// ADSB_EMITTER_TYPE_UNASSIGNED2 enum
	ADSB_EMITTER_TYPE_UNASSIGNED2 ADSB_EMITTER_TYPE = 13
	// ADSB_EMITTER_TYPE_UAV enum
	ADSB_EMITTER_TYPE_UAV ADSB_EMITTER_TYPE = 14
	// ADSB_EMITTER_TYPE_SPACE enum
	ADSB_EMITTER_TYPE_SPACE ADSB_EMITTER_TYPE = 15
	// ADSB_EMITTER_TYPE_UNASSGINED3 enum
	ADSB_EMITTER_TYPE_UNASSGINED3 ADSB_EMITTER_TYPE = 16
	// ADSB_EMITTER_TYPE_EMERGENCY_SURFACE enum
	ADSB_EMITTER_TYPE_EMERGENCY_SURFACE ADSB_EMITTER_TYPE = 17
	// ADSB_EMITTER_TYPE_SERVICE_SURFACE enum
	ADSB_EMITTER_TYPE_SERVICE_SURFACE ADSB_EMITTER_TYPE = 18
	// ADSB_EMITTER_TYPE_POINT_OBSTACLE enum
	ADSB_EMITTER_TYPE_POINT_OBSTACLE ADSB_EMITTER_TYPE = 19
)

// ADSB_FLAGS type. These flags indicate status such as data validity of each data source. Set = data valid
type ADSB_FLAGS int

const (
	// ADSB_FLAGS_VALID_COORDS enum
	ADSB_FLAGS_VALID_COORDS ADSB_FLAGS = 1
	// ADSB_FLAGS_VALID_ALTITUDE enum
	ADSB_FLAGS_VALID_ALTITUDE ADSB_FLAGS = 2
	// ADSB_FLAGS_VALID_HEADING enum
	ADSB_FLAGS_VALID_HEADING ADSB_FLAGS = 4
	// ADSB_FLAGS_VALID_VELOCITY enum
	ADSB_FLAGS_VALID_VELOCITY ADSB_FLAGS = 8
	// ADSB_FLAGS_VALID_CALLSIGN enum
	ADSB_FLAGS_VALID_CALLSIGN ADSB_FLAGS = 16
	// ADSB_FLAGS_VALID_SQUAWK enum
	ADSB_FLAGS_VALID_SQUAWK ADSB_FLAGS = 32
	// ADSB_FLAGS_SIMULATED enum
	ADSB_FLAGS_SIMULATED ADSB_FLAGS = 64
	// ADSB_FLAGS_VERTICAL_VELOCITY_VALID enum
	ADSB_FLAGS_VERTICAL_VELOCITY_VALID ADSB_FLAGS = 128
	// ADSB_FLAGS_BARO_VALID enum
	ADSB_FLAGS_BARO_VALID ADSB_FLAGS = 256
	// ADSB_FLAGS_SOURCE_UAT enum
	ADSB_FLAGS_SOURCE_UAT ADSB_FLAGS = 32768
)

// MAV_DO_REPOSITION_FLAGS type. Bitmap of options for the MAV_CMD_DO_REPOSITION
type MAV_DO_REPOSITION_FLAGS int

const (
	// MAV_DO_REPOSITION_FLAGS_CHANGE_MODE enum. The aircraft should immediately transition into guided. This should not be set for follow me applications
	MAV_DO_REPOSITION_FLAGS_CHANGE_MODE MAV_DO_REPOSITION_FLAGS = 1
)

// ESTIMATOR_STATUS_FLAGS type. Flags in ESTIMATOR_STATUS message
type ESTIMATOR_STATUS_FLAGS int

const (
	// ESTIMATOR_ATTITUDE enum. True if the attitude estimate is good
	ESTIMATOR_ATTITUDE ESTIMATOR_STATUS_FLAGS = 1
	// ESTIMATOR_VELOCITY_HORIZ enum. True if the horizontal velocity estimate is good
	ESTIMATOR_VELOCITY_HORIZ ESTIMATOR_STATUS_FLAGS = 2
	// ESTIMATOR_VELOCITY_VERT enum. True if the  vertical velocity estimate is good
	ESTIMATOR_VELOCITY_VERT ESTIMATOR_STATUS_FLAGS = 4
	// ESTIMATOR_POS_HORIZ_REL enum. True if the horizontal position (relative) estimate is good
	ESTIMATOR_POS_HORIZ_REL ESTIMATOR_STATUS_FLAGS = 8
	// ESTIMATOR_POS_HORIZ_ABS enum. True if the horizontal position (absolute) estimate is good
	ESTIMATOR_POS_HORIZ_ABS ESTIMATOR_STATUS_FLAGS = 16
	// ESTIMATOR_POS_VERT_ABS enum. True if the vertical position (absolute) estimate is good
	ESTIMATOR_POS_VERT_ABS ESTIMATOR_STATUS_FLAGS = 32
	// ESTIMATOR_POS_VERT_AGL enum. True if the vertical position (above ground) estimate is good
	ESTIMATOR_POS_VERT_AGL ESTIMATOR_STATUS_FLAGS = 64
	// ESTIMATOR_CONST_POS_MODE enum. True if the EKF is in a constant position mode and is not using external measurements (eg GPS or optical flow)
	ESTIMATOR_CONST_POS_MODE ESTIMATOR_STATUS_FLAGS = 128
	// ESTIMATOR_PRED_POS_HORIZ_REL enum. True if the EKF has sufficient data to enter a mode that will provide a (relative) position estimate
	ESTIMATOR_PRED_POS_HORIZ_REL ESTIMATOR_STATUS_FLAGS = 256
	// ESTIMATOR_PRED_POS_HORIZ_ABS enum. True if the EKF has sufficient data to enter a mode that will provide a (absolute) position estimate
	ESTIMATOR_PRED_POS_HORIZ_ABS ESTIMATOR_STATUS_FLAGS = 512
	// ESTIMATOR_GPS_GLITCH enum. True if the EKF has detected a GPS glitch
	ESTIMATOR_GPS_GLITCH ESTIMATOR_STATUS_FLAGS = 1024
	// ESTIMATOR_ACCEL_ERROR enum. True if the EKF has detected bad accelerometer data
	ESTIMATOR_ACCEL_ERROR ESTIMATOR_STATUS_FLAGS = 2048
)

// MOTOR_TEST_ORDER type
type MOTOR_TEST_ORDER int

const (
	// MOTOR_TEST_ORDER_DEFAULT enum. default autopilot motor test method
	MOTOR_TEST_ORDER_DEFAULT MOTOR_TEST_ORDER = 0
	// MOTOR_TEST_ORDER_SEQUENCE enum. motor numbers are specified as their index in a predefined vehicle-specific sequence
	MOTOR_TEST_ORDER_SEQUENCE MOTOR_TEST_ORDER = 1
	// MOTOR_TEST_ORDER_BOARD enum. motor numbers are specified as the output as labeled on the board
	MOTOR_TEST_ORDER_BOARD MOTOR_TEST_ORDER = 2
)

// MOTOR_TEST_THROTTLE_TYPE type
type MOTOR_TEST_THROTTLE_TYPE int

const (
	// MOTOR_TEST_THROTTLE_PERCENT enum. throttle as a percentage from 0 ~ 100
	MOTOR_TEST_THROTTLE_PERCENT MOTOR_TEST_THROTTLE_TYPE = 0
	// MOTOR_TEST_THROTTLE_PWM enum. throttle as an absolute PWM value (normally in range of 1000~2000)
	MOTOR_TEST_THROTTLE_PWM MOTOR_TEST_THROTTLE_TYPE = 1
	// MOTOR_TEST_THROTTLE_PILOT enum. throttle pass-through from pilot's transmitter
	MOTOR_TEST_THROTTLE_PILOT MOTOR_TEST_THROTTLE_TYPE = 2
	// MOTOR_TEST_COMPASS_CAL enum. per-motor compass calibration test
	MOTOR_TEST_COMPASS_CAL MOTOR_TEST_THROTTLE_TYPE = 3
)

// GPS_INPUT_IGNORE_FLAGS type
type GPS_INPUT_IGNORE_FLAGS int

const (
	// GPS_INPUT_IGNORE_FLAG_ALT enum. ignore altitude field
	GPS_INPUT_IGNORE_FLAG_ALT GPS_INPUT_IGNORE_FLAGS = 1
	// GPS_INPUT_IGNORE_FLAG_HDOP enum. ignore hdop field
	GPS_INPUT_IGNORE_FLAG_HDOP GPS_INPUT_IGNORE_FLAGS = 2
	// GPS_INPUT_IGNORE_FLAG_VDOP enum. ignore vdop field
	GPS_INPUT_IGNORE_FLAG_VDOP GPS_INPUT_IGNORE_FLAGS = 4
	// GPS_INPUT_IGNORE_FLAG_VEL_HORIZ enum. ignore horizontal velocity field (vn and ve)
	GPS_INPUT_IGNORE_FLAG_VEL_HORIZ GPS_INPUT_IGNORE_FLAGS = 8
	// GPS_INPUT_IGNORE_FLAG_VEL_VERT enum. ignore vertical velocity field (vd)
	GPS_INPUT_IGNORE_FLAG_VEL_VERT GPS_INPUT_IGNORE_FLAGS = 16
	// GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY enum. ignore speed accuracy field
	GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY GPS_INPUT_IGNORE_FLAGS = 32
	// GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY enum. ignore horizontal accuracy field
	GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY GPS_INPUT_IGNORE_FLAGS = 64
	// GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY enum. ignore vertical accuracy field
	GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY GPS_INPUT_IGNORE_FLAGS = 128
)

// MAV_COLLISION_ACTION type. Possible actions an aircraft can take to avoid a collision.
type MAV_COLLISION_ACTION int

const (
	// MAV_COLLISION_ACTION_NONE enum. Ignore any potential collisions
	MAV_COLLISION_ACTION_NONE MAV_COLLISION_ACTION = 0
	// MAV_COLLISION_ACTION_REPORT enum. Report potential collision
	MAV_COLLISION_ACTION_REPORT MAV_COLLISION_ACTION = 1
	// MAV_COLLISION_ACTION_ASCEND_OR_DESCEND enum. Ascend or Descend to avoid threat
	MAV_COLLISION_ACTION_ASCEND_OR_DESCEND MAV_COLLISION_ACTION = 2
	// MAV_COLLISION_ACTION_MOVE_HORIZONTALLY enum. Move horizontally to avoid threat
	MAV_COLLISION_ACTION_MOVE_HORIZONTALLY MAV_COLLISION_ACTION = 3
	// MAV_COLLISION_ACTION_MOVE_PERPENDICULAR enum. Aircraft to move perpendicular to the collision's velocity vector
	MAV_COLLISION_ACTION_MOVE_PERPENDICULAR MAV_COLLISION_ACTION = 4
	// MAV_COLLISION_ACTION_RTL enum. Aircraft to fly directly back to its launch point
	MAV_COLLISION_ACTION_RTL MAV_COLLISION_ACTION = 5
	// MAV_COLLISION_ACTION_HOVER enum. Aircraft to stop in place
	MAV_COLLISION_ACTION_HOVER MAV_COLLISION_ACTION = 6
)

// MAV_COLLISION_THREAT_LEVEL type. Aircraft-rated danger from this threat.
type MAV_COLLISION_THREAT_LEVEL int

const (
	// MAV_COLLISION_THREAT_LEVEL_NONE enum. Not a threat
	MAV_COLLISION_THREAT_LEVEL_NONE MAV_COLLISION_THREAT_LEVEL = 0
	// MAV_COLLISION_THREAT_LEVEL_LOW enum. Craft is mildly concerned about this threat
	MAV_COLLISION_THREAT_LEVEL_LOW MAV_COLLISION_THREAT_LEVEL = 1
	// MAV_COLLISION_THREAT_LEVEL_HIGH enum. Craft is panicking, and may take actions to avoid threat
	MAV_COLLISION_THREAT_LEVEL_HIGH MAV_COLLISION_THREAT_LEVEL = 2
)

// MAV_COLLISION_SRC type. Source of information about this collision.
type MAV_COLLISION_SRC int

const (
	// MAV_COLLISION_SRC_ADSB enum. ID field references ADSB_VEHICLE packets
	MAV_COLLISION_SRC_ADSB MAV_COLLISION_SRC = 0
	// MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT enum. ID field references MAVLink SRC ID
	MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT MAV_COLLISION_SRC = 1
)

// GPS_FIX_TYPE type. Type of GPS fix
type GPS_FIX_TYPE int

const (
	// GPS_FIX_TYPE_NO_GPS enum. No GPS connected
	GPS_FIX_TYPE_NO_GPS GPS_FIX_TYPE = 0
	// GPS_FIX_TYPE_NO_FIX enum. No position information, GPS is connected
	GPS_FIX_TYPE_NO_FIX GPS_FIX_TYPE = 1
	// GPS_FIX_TYPE_2D_FIX enum. 2D position
	GPS_FIX_TYPE_2D_FIX GPS_FIX_TYPE = 2
	// GPS_FIX_TYPE_3D_FIX enum. 3D position
	GPS_FIX_TYPE_3D_FIX GPS_FIX_TYPE = 3
	// GPS_FIX_TYPE_DGPS enum. DGPS/SBAS aided 3D position
	GPS_FIX_TYPE_DGPS GPS_FIX_TYPE = 4
	// GPS_FIX_TYPE_RTK_FLOAT enum. RTK float, 3D position
	GPS_FIX_TYPE_RTK_FLOAT GPS_FIX_TYPE = 5
	// GPS_FIX_TYPE_RTK_FIXED enum. RTK Fixed, 3D position
	GPS_FIX_TYPE_RTK_FIXED GPS_FIX_TYPE = 6
	// GPS_FIX_TYPE_STATIC enum. Static fixed, typically used for base stations
	GPS_FIX_TYPE_STATIC GPS_FIX_TYPE = 7
	// GPS_FIX_TYPE_PPP enum. PPP, 3D position
	GPS_FIX_TYPE_PPP GPS_FIX_TYPE = 8
)

// RTK_BASELINE_COORDINATE_SYSTEM type. RTK GPS baseline coordinate system, used for RTK corrections
type RTK_BASELINE_COORDINATE_SYSTEM int

const (
	// RTK_BASELINE_COORDINATE_SYSTEM_ECEF enum. Earth-centered, Earth-fixed
	RTK_BASELINE_COORDINATE_SYSTEM_ECEF RTK_BASELINE_COORDINATE_SYSTEM = 0
	// RTK_BASELINE_COORDINATE_SYSTEM_NED enum. RTK basestation centered, north, east, down
	RTK_BASELINE_COORDINATE_SYSTEM_NED RTK_BASELINE_COORDINATE_SYSTEM = 1
)

// LANDING_TARGET_TYPE type. Type of landing target
type LANDING_TARGET_TYPE int

const (
	// LANDING_TARGET_TYPE_LIGHT_BEACON enum. Landing target signaled by light beacon (ex: IR-LOCK)
	LANDING_TARGET_TYPE_LIGHT_BEACON LANDING_TARGET_TYPE = 0
	// LANDING_TARGET_TYPE_RADIO_BEACON enum. Landing target signaled by radio beacon (ex: ILS, NDB)
	LANDING_TARGET_TYPE_RADIO_BEACON LANDING_TARGET_TYPE = 1
	// LANDING_TARGET_TYPE_VISION_FIDUCIAL enum. Landing target represented by a fiducial marker (ex: ARTag)
	LANDING_TARGET_TYPE_VISION_FIDUCIAL LANDING_TARGET_TYPE = 2
	// LANDING_TARGET_TYPE_VISION_OTHER enum. Landing target represented by a pre-defined visual shape/feature (ex: X-marker, H-marker, square)
	LANDING_TARGET_TYPE_VISION_OTHER LANDING_TARGET_TYPE = 3
)

// VTOL_TRANSITION_HEADING type. Direction of VTOL transition
type VTOL_TRANSITION_HEADING int

const (
	// VTOL_TRANSITION_HEADING_VEHICLE_DEFAULT enum. Respect the heading configuration of the vehicle
	VTOL_TRANSITION_HEADING_VEHICLE_DEFAULT VTOL_TRANSITION_HEADING = 0
	// VTOL_TRANSITION_HEADING_NEXT_WAYPOINT enum. Use the heading pointing towards the next waypoint
	VTOL_TRANSITION_HEADING_NEXT_WAYPOINT VTOL_TRANSITION_HEADING = 1
	// VTOL_TRANSITION_HEADING_TAKEOFF enum. Use the heading on takeoff (while sitting on the ground)
	VTOL_TRANSITION_HEADING_TAKEOFF VTOL_TRANSITION_HEADING = 2
	// VTOL_TRANSITION_HEADING_SPECIFIED enum. Use the specified heading in parameter 4
	VTOL_TRANSITION_HEADING_SPECIFIED VTOL_TRANSITION_HEADING = 3
	// VTOL_TRANSITION_HEADING_ANY enum. Use the current heading when reaching takeoff altitude (potentially facing the wind when weather-vaning is active)
	VTOL_TRANSITION_HEADING_ANY VTOL_TRANSITION_HEADING = 4
)

// CAMERA_CAP_FLAGS type. Camera capability flags (Bitmap)
type CAMERA_CAP_FLAGS int

const (
	// CAMERA_CAP_FLAGS_CAPTURE_VIDEO enum. Camera is able to record video
	CAMERA_CAP_FLAGS_CAPTURE_VIDEO CAMERA_CAP_FLAGS = 1
	// CAMERA_CAP_FLAGS_CAPTURE_IMAGE enum. Camera is able to capture images
	CAMERA_CAP_FLAGS_CAPTURE_IMAGE CAMERA_CAP_FLAGS = 2
	// CAMERA_CAP_FLAGS_HAS_MODES enum. Camera has separate Video and Image/Photo modes (MAV_CMD_SET_CAMERA_MODE)
	CAMERA_CAP_FLAGS_HAS_MODES CAMERA_CAP_FLAGS = 4
	// CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE enum. Camera can capture images while in video mode
	CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE CAMERA_CAP_FLAGS = 8
	// CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE enum. Camera can capture videos while in Photo/Image mode
	CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE CAMERA_CAP_FLAGS = 16
	// CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE enum. Camera has image survey mode (MAV_CMD_SET_CAMERA_MODE)
	CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE CAMERA_CAP_FLAGS = 32
	// CAMERA_CAP_FLAGS_HAS_BASIC_ZOOM enum. Camera has basic zoom control (MAV_CMD_SET_CAMERA_ZOOM)
	CAMERA_CAP_FLAGS_HAS_BASIC_ZOOM CAMERA_CAP_FLAGS = 64
	// CAMERA_CAP_FLAGS_HAS_BASIC_FOCUS enum. Camera has basic focus control (MAV_CMD_SET_CAMERA_FOCUS)
	CAMERA_CAP_FLAGS_HAS_BASIC_FOCUS CAMERA_CAP_FLAGS = 128
	// CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM enum. Camera has video streaming capabilities (request VIDEO_STREAM_INFORMATION with MAV_CMD_REQUEST_MESSAGE for video streaming info)
	CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM CAMERA_CAP_FLAGS = 256
	// CAMERA_CAP_FLAGS_HAS_TRACKING_POINT enum. Camera supports tracking of a point on the camera view
	CAMERA_CAP_FLAGS_HAS_TRACKING_POINT CAMERA_CAP_FLAGS = 512
	// CAMERA_CAP_FLAGS_HAS_TRACKING_RECTANGLE enum. Camera supports tracking of a selection rectangle on the camera view
	CAMERA_CAP_FLAGS_HAS_TRACKING_RECTANGLE CAMERA_CAP_FLAGS = 1024
	// CAMERA_CAP_FLAGS_HAS_TRACKING_GEO_STATUS enum. Camera supports tracking geo status (CAMERA_TRACKING_GEO_STATUS)
	CAMERA_CAP_FLAGS_HAS_TRACKING_GEO_STATUS CAMERA_CAP_FLAGS = 2048
)

// VIDEO_STREAM_STATUS_FLAGS type. Stream status flags (Bitmap)
type VIDEO_STREAM_STATUS_FLAGS int

const (
	// VIDEO_STREAM_STATUS_FLAGS_RUNNING enum. Stream is active (running)
	VIDEO_STREAM_STATUS_FLAGS_RUNNING VIDEO_STREAM_STATUS_FLAGS = 1
	// VIDEO_STREAM_STATUS_FLAGS_THERMAL enum. Stream is thermal imaging
	VIDEO_STREAM_STATUS_FLAGS_THERMAL VIDEO_STREAM_STATUS_FLAGS = 2
)

// VIDEO_STREAM_TYPE type. Video stream types
type VIDEO_STREAM_TYPE int

const (
	// VIDEO_STREAM_TYPE_RTSP enum. Stream is RTSP
	VIDEO_STREAM_TYPE_RTSP VIDEO_STREAM_TYPE = 0
	// VIDEO_STREAM_TYPE_RTPUDP enum. Stream is RTP UDP (URI gives the port number)
	VIDEO_STREAM_TYPE_RTPUDP VIDEO_STREAM_TYPE = 1
	// VIDEO_STREAM_TYPE_TCP_MPEG enum. Stream is MPEG on TCP
	VIDEO_STREAM_TYPE_TCP_MPEG VIDEO_STREAM_TYPE = 2
	// VIDEO_STREAM_TYPE_MPEG_TS_H264 enum. Stream is h.264 on MPEG TS (URI gives the port number)
	VIDEO_STREAM_TYPE_MPEG_TS_H264 VIDEO_STREAM_TYPE = 3
)

// CAMERA_TRACKING_STATUS_FLAGS type. Camera tracking status flags
type CAMERA_TRACKING_STATUS_FLAGS int

const (
	// CAMERA_TRACKING_STATUS_FLAGS_IDLE enum. Camera is not tracking
	CAMERA_TRACKING_STATUS_FLAGS_IDLE CAMERA_TRACKING_STATUS_FLAGS = 0
	// CAMERA_TRACKING_STATUS_FLAGS_ACTIVE enum. Camera is tracking
	CAMERA_TRACKING_STATUS_FLAGS_ACTIVE CAMERA_TRACKING_STATUS_FLAGS = 1
	// CAMERA_TRACKING_STATUS_FLAGS_ERROR enum. Camera tracking in error state
	CAMERA_TRACKING_STATUS_FLAGS_ERROR CAMERA_TRACKING_STATUS_FLAGS = 2
)

// CAMERA_TRACKING_MODE type. Camera tracking modes
type CAMERA_TRACKING_MODE int

const (
	// CAMERA_TRACKING_NONE enum. Not tracking
	CAMERA_TRACKING_NONE CAMERA_TRACKING_MODE = 0
	// CAMERA_TRACKING_POINT enum. Target is a point
	CAMERA_TRACKING_POINT CAMERA_TRACKING_MODE = 1
	// CAMERA_TRACKING_RECTANGLE enum. Target is a rectangle
	CAMERA_TRACKING_RECTANGLE CAMERA_TRACKING_MODE = 2
)

// CAMERA_TRACKING_TARGET_DATA type. Camera tracking target data (shows where tracked target is within image)
type CAMERA_TRACKING_TARGET_DATA int

const (
	// CAMERA_TRACKING_TARGET_NONE enum. No target data
	CAMERA_TRACKING_TARGET_NONE CAMERA_TRACKING_TARGET_DATA = 0
	// CAMERA_TRACKING_TARGET_EMBEDDED enum. Target data embedded in image data (proprietary)
	CAMERA_TRACKING_TARGET_EMBEDDED CAMERA_TRACKING_TARGET_DATA = 1
	// CAMERA_TRACKING_TARGET_RENDERED enum. Target data rendered in image
	CAMERA_TRACKING_TARGET_RENDERED CAMERA_TRACKING_TARGET_DATA = 2
	// CAMERA_TRACKING_TARGET_IN_STATUS enum. Target data within status message (Point or Rectangle)
	CAMERA_TRACKING_TARGET_IN_STATUS CAMERA_TRACKING_TARGET_DATA = 4
)

// CAMERA_ZOOM_TYPE type. Zoom types for MAV_CMD_SET_CAMERA_ZOOM
type CAMERA_ZOOM_TYPE int

const (
	// ZOOM_TYPE_STEP enum. Zoom one step increment (-1 for wide, 1 for tele)
	ZOOM_TYPE_STEP CAMERA_ZOOM_TYPE = 0
	// ZOOM_TYPE_CONTINUOUS enum. Continuous zoom up/down until stopped (-1 for wide, 1 for tele, 0 to stop zooming)
	ZOOM_TYPE_CONTINUOUS CAMERA_ZOOM_TYPE = 1
	// ZOOM_TYPE_RANGE enum. Zoom value as proportion of full camera range (a value between 0.0 and 100.0)
	ZOOM_TYPE_RANGE CAMERA_ZOOM_TYPE = 2
	// ZOOM_TYPE_FOCAL_LENGTH enum. Zoom value/variable focal length in milimetres. Note that there is no message to get the valid zoom range of the camera, so this can type can only be used for cameras where the zoom range is known (implying that this cannot reliably be used in a GCS for an arbitrary camera)
	ZOOM_TYPE_FOCAL_LENGTH CAMERA_ZOOM_TYPE = 3
)

// SET_FOCUS_TYPE type. Focus types for MAV_CMD_SET_CAMERA_FOCUS
type SET_FOCUS_TYPE int

const (
	// FOCUS_TYPE_STEP enum. Focus one step increment (-1 for focusing in, 1 for focusing out towards infinity)
	FOCUS_TYPE_STEP SET_FOCUS_TYPE = 0
	// FOCUS_TYPE_CONTINUOUS enum. Continuous focus up/down until stopped (-1 for focusing in, 1 for focusing out towards infinity, 0 to stop focusing)
	FOCUS_TYPE_CONTINUOUS SET_FOCUS_TYPE = 1
	// FOCUS_TYPE_RANGE enum. Focus value as proportion of full camera focus range (a value between 0.0 and 100.0)
	FOCUS_TYPE_RANGE SET_FOCUS_TYPE = 2
	// FOCUS_TYPE_METERS enum. Focus value in metres. Note that there is no message to get the valid focus range of the camera, so this can type can only be used for cameras where the range is known (implying that this cannot reliably be used in a GCS for an arbitrary camera)
	FOCUS_TYPE_METERS SET_FOCUS_TYPE = 3
)

// PARAM_ACK type. Result from PARAM_EXT_SET message (or a PARAM_SET within a transaction).
type PARAM_ACK int

const (
	// PARAM_ACK_ACCEPTED enum. Parameter value ACCEPTED and SET
	PARAM_ACK_ACCEPTED PARAM_ACK = 0
	// PARAM_ACK_VALUE_UNSUPPORTED enum. Parameter value UNKNOWN/UNSUPPORTED
	PARAM_ACK_VALUE_UNSUPPORTED PARAM_ACK = 1
	// PARAM_ACK_FAILED enum. Parameter failed to set
	PARAM_ACK_FAILED PARAM_ACK = 2
	// PARAM_ACK_IN_PROGRESS enum. Parameter value received but not yet set/accepted. A subsequent PARAM_ACK_TRANSACTION or PARAM_EXT_ACK with the final result will follow once operation is completed. This is returned immediately for parameters that take longer to set, indicating taht the the parameter was recieved and does not need to be resent
	PARAM_ACK_IN_PROGRESS PARAM_ACK = 3
)

// CAMERA_MODE type. Camera Modes.
type CAMERA_MODE int

const (
	// CAMERA_MODE_IMAGE enum. Camera is in image/photo capture mode
	CAMERA_MODE_IMAGE CAMERA_MODE = 0
	// CAMERA_MODE_VIDEO enum. Camera is in video capture mode
	CAMERA_MODE_VIDEO CAMERA_MODE = 1
	// CAMERA_MODE_IMAGE_SURVEY enum. Camera is in image survey capture mode. It allows for camera controller to do specific settings for surveys
	CAMERA_MODE_IMAGE_SURVEY CAMERA_MODE = 2
)

// MAV_ARM_AUTH_DENIED_REASON type
type MAV_ARM_AUTH_DENIED_REASON int

const (
	// MAV_ARM_AUTH_DENIED_REASON_GENERIC enum. Not a specific reason
	MAV_ARM_AUTH_DENIED_REASON_GENERIC MAV_ARM_AUTH_DENIED_REASON = 0
	// MAV_ARM_AUTH_DENIED_REASON_NONE enum. Authorizer will send the error as string to GCS
	MAV_ARM_AUTH_DENIED_REASON_NONE MAV_ARM_AUTH_DENIED_REASON = 1
	// MAV_ARM_AUTH_DENIED_REASON_INVALID_WAYPOINT enum. At least one waypoint have a invalid value
	MAV_ARM_AUTH_DENIED_REASON_INVALID_WAYPOINT MAV_ARM_AUTH_DENIED_REASON = 2
	// MAV_ARM_AUTH_DENIED_REASON_TIMEOUT enum. Timeout in the authorizer process(in case it depends on network)
	MAV_ARM_AUTH_DENIED_REASON_TIMEOUT MAV_ARM_AUTH_DENIED_REASON = 3
	// MAV_ARM_AUTH_DENIED_REASON_AIRSPACE_IN_USE enum. Airspace of the mission in use by another vehicle, second result parameter can have the waypoint id that caused it to be denied
	MAV_ARM_AUTH_DENIED_REASON_AIRSPACE_IN_USE MAV_ARM_AUTH_DENIED_REASON = 4
	// MAV_ARM_AUTH_DENIED_REASON_BAD_WEATHER enum. Weather is not good to fly
	MAV_ARM_AUTH_DENIED_REASON_BAD_WEATHER MAV_ARM_AUTH_DENIED_REASON = 5
)

// RC_TYPE type. RC type
type RC_TYPE int

const (
	// RC_TYPE_SPEKTRUM_DSM2 enum. Spektrum DSM2
	RC_TYPE_SPEKTRUM_DSM2 RC_TYPE = 0
	// RC_TYPE_SPEKTRUM_DSMX enum. Spektrum DSMX
	RC_TYPE_SPEKTRUM_DSMX RC_TYPE = 1
)

// POSITION_TARGET_TYPEMASK type. Bitmap to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 9 is set the floats afx afy afz should be interpreted as force instead of acceleration.
type POSITION_TARGET_TYPEMASK int

const (
	// POSITION_TARGET_TYPEMASK_X_IGNORE enum. Ignore position x
	POSITION_TARGET_TYPEMASK_X_IGNORE POSITION_TARGET_TYPEMASK = 1
	// POSITION_TARGET_TYPEMASK_Y_IGNORE enum. Ignore position y
	POSITION_TARGET_TYPEMASK_Y_IGNORE POSITION_TARGET_TYPEMASK = 2
	// POSITION_TARGET_TYPEMASK_Z_IGNORE enum. Ignore position z
	POSITION_TARGET_TYPEMASK_Z_IGNORE POSITION_TARGET_TYPEMASK = 4
	// POSITION_TARGET_TYPEMASK_VX_IGNORE enum. Ignore velocity x
	POSITION_TARGET_TYPEMASK_VX_IGNORE POSITION_TARGET_TYPEMASK = 8
	// POSITION_TARGET_TYPEMASK_VY_IGNORE enum. Ignore velocity y
	POSITION_TARGET_TYPEMASK_VY_IGNORE POSITION_TARGET_TYPEMASK = 16
	// POSITION_TARGET_TYPEMASK_VZ_IGNORE enum. Ignore velocity z
	POSITION_TARGET_TYPEMASK_VZ_IGNORE POSITION_TARGET_TYPEMASK = 32
	// POSITION_TARGET_TYPEMASK_AX_IGNORE enum. Ignore acceleration x
	POSITION_TARGET_TYPEMASK_AX_IGNORE POSITION_TARGET_TYPEMASK = 64
	// POSITION_TARGET_TYPEMASK_AY_IGNORE enum. Ignore acceleration y
	POSITION_TARGET_TYPEMASK_AY_IGNORE POSITION_TARGET_TYPEMASK = 128
	// POSITION_TARGET_TYPEMASK_AZ_IGNORE enum. Ignore acceleration z
	POSITION_TARGET_TYPEMASK_AZ_IGNORE POSITION_TARGET_TYPEMASK = 256
	// POSITION_TARGET_TYPEMASK_FORCE_SET enum. Use force instead of acceleration
	POSITION_TARGET_TYPEMASK_FORCE_SET POSITION_TARGET_TYPEMASK = 512
	// POSITION_TARGET_TYPEMASK_YAW_IGNORE enum. Ignore yaw
	POSITION_TARGET_TYPEMASK_YAW_IGNORE POSITION_TARGET_TYPEMASK = 1024
	// POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE enum. Ignore yaw rate
	POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE POSITION_TARGET_TYPEMASK = 2048
)

// ATTITUDE_TARGET_TYPEMASK type. Bitmap to indicate which dimensions should be ignored by the vehicle: a value of 0b00000000 indicates that none of the setpoint dimensions should be ignored.
type ATTITUDE_TARGET_TYPEMASK int

const (
	// ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE enum. Ignore body roll rate
	ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE ATTITUDE_TARGET_TYPEMASK = 1
	// ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE enum. Ignore body pitch rate
	ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE ATTITUDE_TARGET_TYPEMASK = 2
	// ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE enum. Ignore body yaw rate
	ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE ATTITUDE_TARGET_TYPEMASK = 4
	// ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE enum. Ignore throttle
	ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE ATTITUDE_TARGET_TYPEMASK = 64
	// ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE enum. Ignore attitude
	ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE ATTITUDE_TARGET_TYPEMASK = 128
)

// UTM_FLIGHT_STATE type. Airborne status of UAS.
type UTM_FLIGHT_STATE int

const (
	// UTM_FLIGHT_STATE_UNKNOWN enum. The flight state can't be determined
	UTM_FLIGHT_STATE_UNKNOWN UTM_FLIGHT_STATE = 1
	// UTM_FLIGHT_STATE_GROUND enum. UAS on ground
	UTM_FLIGHT_STATE_GROUND UTM_FLIGHT_STATE = 2
	// UTM_FLIGHT_STATE_AIRBORNE enum. UAS airborne
	UTM_FLIGHT_STATE_AIRBORNE UTM_FLIGHT_STATE = 3
	// UTM_FLIGHT_STATE_EMERGENCY enum. UAS is in an emergency flight state
	UTM_FLIGHT_STATE_EMERGENCY UTM_FLIGHT_STATE = 16
	// UTM_FLIGHT_STATE_NOCTRL enum. UAS has no active controls
	UTM_FLIGHT_STATE_NOCTRL UTM_FLIGHT_STATE = 32
)

// UTM_DATA_AVAIL_FLAGS type. Flags for the global position report.
type UTM_DATA_AVAIL_FLAGS int

const (
	// UTM_DATA_AVAIL_FLAGS_TIME_VALID enum. The field time contains valid data
	UTM_DATA_AVAIL_FLAGS_TIME_VALID UTM_DATA_AVAIL_FLAGS = 1
	// UTM_DATA_AVAIL_FLAGS_UAS_ID_AVAILABLE enum. The field uas_id contains valid data
	UTM_DATA_AVAIL_FLAGS_UAS_ID_AVAILABLE UTM_DATA_AVAIL_FLAGS = 2
	// UTM_DATA_AVAIL_FLAGS_POSITION_AVAILABLE enum. The fields lat, lon and h_acc contain valid data
	UTM_DATA_AVAIL_FLAGS_POSITION_AVAILABLE UTM_DATA_AVAIL_FLAGS = 4
	// UTM_DATA_AVAIL_FLAGS_ALTITUDE_AVAILABLE enum. The fields alt and v_acc contain valid data
	UTM_DATA_AVAIL_FLAGS_ALTITUDE_AVAILABLE UTM_DATA_AVAIL_FLAGS = 8
	// UTM_DATA_AVAIL_FLAGS_RELATIVE_ALTITUDE_AVAILABLE enum. The field relative_alt contains valid data
	UTM_DATA_AVAIL_FLAGS_RELATIVE_ALTITUDE_AVAILABLE UTM_DATA_AVAIL_FLAGS = 16
	// UTM_DATA_AVAIL_FLAGS_HORIZONTAL_VELO_AVAILABLE enum. The fields vx and vy contain valid data
	UTM_DATA_AVAIL_FLAGS_HORIZONTAL_VELO_AVAILABLE UTM_DATA_AVAIL_FLAGS = 32
	// UTM_DATA_AVAIL_FLAGS_VERTICAL_VELO_AVAILABLE enum. The field vz contains valid data
	UTM_DATA_AVAIL_FLAGS_VERTICAL_VELO_AVAILABLE UTM_DATA_AVAIL_FLAGS = 64
	// UTM_DATA_AVAIL_FLAGS_NEXT_WAYPOINT_AVAILABLE enum. The fields next_lat, next_lon and next_alt contain valid data
	UTM_DATA_AVAIL_FLAGS_NEXT_WAYPOINT_AVAILABLE UTM_DATA_AVAIL_FLAGS = 128
)

// CELLULAR_NETWORK_RADIO_TYPE type. Cellular network radio type
type CELLULAR_NETWORK_RADIO_TYPE int

const (
	// CELLULAR_NETWORK_RADIO_TYPE_NONE enum
	CELLULAR_NETWORK_RADIO_TYPE_NONE CELLULAR_NETWORK_RADIO_TYPE = 0
	// CELLULAR_NETWORK_RADIO_TYPE_GSM enum
	CELLULAR_NETWORK_RADIO_TYPE_GSM CELLULAR_NETWORK_RADIO_TYPE = 1
	// CELLULAR_NETWORK_RADIO_TYPE_CDMA enum
	CELLULAR_NETWORK_RADIO_TYPE_CDMA CELLULAR_NETWORK_RADIO_TYPE = 2
	// CELLULAR_NETWORK_RADIO_TYPE_WCDMA enum
	CELLULAR_NETWORK_RADIO_TYPE_WCDMA CELLULAR_NETWORK_RADIO_TYPE = 3
	// CELLULAR_NETWORK_RADIO_TYPE_LTE enum
	CELLULAR_NETWORK_RADIO_TYPE_LTE CELLULAR_NETWORK_RADIO_TYPE = 4
)

// CELLULAR_STATUS_FLAG type. These flags encode the cellular network status
type CELLULAR_STATUS_FLAG int

const (
	// CELLULAR_STATUS_FLAG_UNKNOWN enum. State unknown or not reportable
	CELLULAR_STATUS_FLAG_UNKNOWN CELLULAR_STATUS_FLAG = 0
	// CELLULAR_STATUS_FLAG_FAILED enum. Modem is unusable
	CELLULAR_STATUS_FLAG_FAILED CELLULAR_STATUS_FLAG = 1
	// CELLULAR_STATUS_FLAG_INITIALIZING enum. Modem is being initialized
	CELLULAR_STATUS_FLAG_INITIALIZING CELLULAR_STATUS_FLAG = 2
	// CELLULAR_STATUS_FLAG_LOCKED enum. Modem is locked
	CELLULAR_STATUS_FLAG_LOCKED CELLULAR_STATUS_FLAG = 3
	// CELLULAR_STATUS_FLAG_DISABLED enum. Modem is not enabled and is powered down
	CELLULAR_STATUS_FLAG_DISABLED CELLULAR_STATUS_FLAG = 4
	// CELLULAR_STATUS_FLAG_DISABLING enum. Modem is currently transitioning to the CELLULAR_STATUS_FLAG_DISABLED state
	CELLULAR_STATUS_FLAG_DISABLING CELLULAR_STATUS_FLAG = 5
	// CELLULAR_STATUS_FLAG_ENABLING enum. Modem is currently transitioning to the CELLULAR_STATUS_FLAG_ENABLED state
	CELLULAR_STATUS_FLAG_ENABLING CELLULAR_STATUS_FLAG = 6
	// CELLULAR_STATUS_FLAG_ENABLED enum. Modem is enabled and powered on but not registered with a network provider and not available for data connections
	CELLULAR_STATUS_FLAG_ENABLED CELLULAR_STATUS_FLAG = 7
	// CELLULAR_STATUS_FLAG_SEARCHING enum. Modem is searching for a network provider to register
	CELLULAR_STATUS_FLAG_SEARCHING CELLULAR_STATUS_FLAG = 8
	// CELLULAR_STATUS_FLAG_REGISTERED enum. Modem is registered with a network provider, and data connections and messaging may be available for use
	CELLULAR_STATUS_FLAG_REGISTERED CELLULAR_STATUS_FLAG = 9
	// CELLULAR_STATUS_FLAG_DISCONNECTING enum. Modem is disconnecting and deactivating the last active packet data bearer. This state will not be entered if more than one packet data bearer is active and one of the active bearers is deactivated
	CELLULAR_STATUS_FLAG_DISCONNECTING CELLULAR_STATUS_FLAG = 10
	// CELLULAR_STATUS_FLAG_CONNECTING enum. Modem is activating and connecting the first packet data bearer. Subsequent bearer activations when another bearer is already active do not cause this state to be entered
	CELLULAR_STATUS_FLAG_CONNECTING CELLULAR_STATUS_FLAG = 11
	// CELLULAR_STATUS_FLAG_CONNECTED enum. One or more packet data bearers is active and connected
	CELLULAR_STATUS_FLAG_CONNECTED CELLULAR_STATUS_FLAG = 12
)

// CELLULAR_NETWORK_FAILED_REASON type. These flags are used to diagnose the failure state of CELLULAR_STATUS
type CELLULAR_NETWORK_FAILED_REASON int

const (
	// CELLULAR_NETWORK_FAILED_REASON_NONE enum. No error
	CELLULAR_NETWORK_FAILED_REASON_NONE CELLULAR_NETWORK_FAILED_REASON = 0
	// CELLULAR_NETWORK_FAILED_REASON_UNKNOWN enum. Error state is unknown
	CELLULAR_NETWORK_FAILED_REASON_UNKNOWN CELLULAR_NETWORK_FAILED_REASON = 1
	// CELLULAR_NETWORK_FAILED_REASON_SIM_MISSING enum. SIM is required for the modem but missing
	CELLULAR_NETWORK_FAILED_REASON_SIM_MISSING CELLULAR_NETWORK_FAILED_REASON = 2
	// CELLULAR_NETWORK_FAILED_REASON_SIM_ERROR enum. SIM is available, but not usuable for connection
	CELLULAR_NETWORK_FAILED_REASON_SIM_ERROR CELLULAR_NETWORK_FAILED_REASON = 3
)

// PRECISION_LAND_MODE type. Precision land modes (used in MAV_CMD_NAV_LAND).
type PRECISION_LAND_MODE int

const (
	// PRECISION_LAND_MODE_DISABLED enum. Normal (non-precision) landing
	PRECISION_LAND_MODE_DISABLED PRECISION_LAND_MODE = 0
	// PRECISION_LAND_MODE_OPPORTUNISTIC enum. Use precision landing if beacon detected when land command accepted, otherwise land normally
	PRECISION_LAND_MODE_OPPORTUNISTIC PRECISION_LAND_MODE = 1
	// PRECISION_LAND_MODE_REQUIRED enum. Use precision landing, searching for beacon if not found when land command accepted (land normally if beacon cannot be found)
	PRECISION_LAND_MODE_REQUIRED PRECISION_LAND_MODE = 2
)

// PARACHUTE_ACTION type. Parachute actions. Trigger release and enable/disable auto-release.
type PARACHUTE_ACTION int

const (
	// PARACHUTE_DISABLE enum. Disable auto-release of parachute (i.e. release triggered by crash detectors)
	PARACHUTE_DISABLE PARACHUTE_ACTION = 0
	// PARACHUTE_ENABLE enum. Enable auto-release of parachute
	PARACHUTE_ENABLE PARACHUTE_ACTION = 1
	// PARACHUTE_RELEASE enum. Release parachute and kill motors
	PARACHUTE_RELEASE PARACHUTE_ACTION = 2
)

// MAV_TUNNEL_PAYLOAD_TYPE type
type MAV_TUNNEL_PAYLOAD_TYPE int

const (
	// MAV_TUNNEL_PAYLOAD_TYPE_UNKNOWN enum. Encoding of payload unknown
	MAV_TUNNEL_PAYLOAD_TYPE_UNKNOWN MAV_TUNNEL_PAYLOAD_TYPE = 0
	// MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED0 enum. Registered for STorM32 gimbal controller
	MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED0 MAV_TUNNEL_PAYLOAD_TYPE = 200
	// MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED1 enum. Registered for STorM32 gimbal controller
	MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED1 MAV_TUNNEL_PAYLOAD_TYPE = 201
	// MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED2 enum. Registered for STorM32 gimbal controller
	MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED2 MAV_TUNNEL_PAYLOAD_TYPE = 202
	// MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED3 enum. Registered for STorM32 gimbal controller
	MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED3 MAV_TUNNEL_PAYLOAD_TYPE = 203
	// MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED4 enum. Registered for STorM32 gimbal controller
	MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED4 MAV_TUNNEL_PAYLOAD_TYPE = 204
	// MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED5 enum. Registered for STorM32 gimbal controller
	MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED5 MAV_TUNNEL_PAYLOAD_TYPE = 205
	// MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED6 enum. Registered for STorM32 gimbal controller
	MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED6 MAV_TUNNEL_PAYLOAD_TYPE = 206
	// MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED7 enum. Registered for STorM32 gimbal controller
	MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED7 MAV_TUNNEL_PAYLOAD_TYPE = 207
	// MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED8 enum. Registered for STorM32 gimbal controller
	MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED8 MAV_TUNNEL_PAYLOAD_TYPE = 208
	// MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED9 enum. Registered for STorM32 gimbal controller
	MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED9 MAV_TUNNEL_PAYLOAD_TYPE = 209
)

// MAV_ODID_ID_TYPE type
type MAV_ODID_ID_TYPE int

const (
	// MAV_ODID_ID_TYPE_NONE enum. No type defined
	MAV_ODID_ID_TYPE_NONE MAV_ODID_ID_TYPE = 0
	// MAV_ODID_ID_TYPE_SERIAL_NUMBER enum. Manufacturer Serial Number (ANSI/CTA-2063 format)
	MAV_ODID_ID_TYPE_SERIAL_NUMBER MAV_ODID_ID_TYPE = 1
	// MAV_ODID_ID_TYPE_CAA_REGISTRATION_ID enum. CAA (Civil Aviation Authority) registered ID. Format: [ICAO Country Code].[CAA Assigned ID]
	MAV_ODID_ID_TYPE_CAA_REGISTRATION_ID MAV_ODID_ID_TYPE = 2
	// MAV_ODID_ID_TYPE_UTM_ASSIGNED_UUID enum. UTM (Unmanned Traffic Management) assigned UUID (RFC4122)
	MAV_ODID_ID_TYPE_UTM_ASSIGNED_UUID MAV_ODID_ID_TYPE = 3
)

// MAV_ODID_UA_TYPE type
type MAV_ODID_UA_TYPE int

const (
	// MAV_ODID_UA_TYPE_NONE enum. No UA (Unmanned Aircraft) type defined
	MAV_ODID_UA_TYPE_NONE MAV_ODID_UA_TYPE = 0
	// MAV_ODID_UA_TYPE_AEROPLANE enum. Aeroplane/Airplane. Fixed wing
	MAV_ODID_UA_TYPE_AEROPLANE MAV_ODID_UA_TYPE = 1
	// MAV_ODID_UA_TYPE_HELICOPTER_OR_MULTIROTOR enum. Helicopter or multirotor
	MAV_ODID_UA_TYPE_HELICOPTER_OR_MULTIROTOR MAV_ODID_UA_TYPE = 2
	// MAV_ODID_UA_TYPE_GYROPLANE enum. Gyroplane
	MAV_ODID_UA_TYPE_GYROPLANE MAV_ODID_UA_TYPE = 3
	// MAV_ODID_UA_TYPE_HYBRID_LIFT enum. VTOL (Vertical Take-Off and Landing). Fixed wing aircraft that can take off vertically
	MAV_ODID_UA_TYPE_HYBRID_LIFT MAV_ODID_UA_TYPE = 4
	// MAV_ODID_UA_TYPE_ORNITHOPTER enum. Ornithopter
	MAV_ODID_UA_TYPE_ORNITHOPTER MAV_ODID_UA_TYPE = 5
	// MAV_ODID_UA_TYPE_GLIDER enum. Glider
	MAV_ODID_UA_TYPE_GLIDER MAV_ODID_UA_TYPE = 6
	// MAV_ODID_UA_TYPE_KITE enum. Kite
	MAV_ODID_UA_TYPE_KITE MAV_ODID_UA_TYPE = 7
	// MAV_ODID_UA_TYPE_FREE_BALLOON enum. Free Balloon
	MAV_ODID_UA_TYPE_FREE_BALLOON MAV_ODID_UA_TYPE = 8
	// MAV_ODID_UA_TYPE_CAPTIVE_BALLOON enum. Captive Balloon
	MAV_ODID_UA_TYPE_CAPTIVE_BALLOON MAV_ODID_UA_TYPE = 9
	// MAV_ODID_UA_TYPE_AIRSHIP enum. Airship. E.g. a blimp
	MAV_ODID_UA_TYPE_AIRSHIP MAV_ODID_UA_TYPE = 10
	// MAV_ODID_UA_TYPE_FREE_FALL_PARACHUTE enum. Free Fall/Parachute (unpowered)
	MAV_ODID_UA_TYPE_FREE_FALL_PARACHUTE MAV_ODID_UA_TYPE = 11
	// MAV_ODID_UA_TYPE_ROCKET enum. Rocket
	MAV_ODID_UA_TYPE_ROCKET MAV_ODID_UA_TYPE = 12
	// MAV_ODID_UA_TYPE_TETHERED_POWERED_AIRCRAFT enum. Tethered powered aircraft
	MAV_ODID_UA_TYPE_TETHERED_POWERED_AIRCRAFT MAV_ODID_UA_TYPE = 13
	// MAV_ODID_UA_TYPE_GROUND_OBSTACLE enum. Ground Obstacle
	MAV_ODID_UA_TYPE_GROUND_OBSTACLE MAV_ODID_UA_TYPE = 14
	// MAV_ODID_UA_TYPE_OTHER enum. Other type of aircraft not listed earlier
	MAV_ODID_UA_TYPE_OTHER MAV_ODID_UA_TYPE = 15
)

// MAV_ODID_STATUS type
type MAV_ODID_STATUS int

const (
	// MAV_ODID_STATUS_UNDECLARED enum. The status of the (UA) Unmanned Aircraft is undefined
	MAV_ODID_STATUS_UNDECLARED MAV_ODID_STATUS = 0
	// MAV_ODID_STATUS_GROUND enum. The UA is on the ground
	MAV_ODID_STATUS_GROUND MAV_ODID_STATUS = 1
	// MAV_ODID_STATUS_AIRBORNE enum. The UA is in the air
	MAV_ODID_STATUS_AIRBORNE MAV_ODID_STATUS = 2
	// MAV_ODID_STATUS_EMERGENCY enum. The UA is having an emergency
	MAV_ODID_STATUS_EMERGENCY MAV_ODID_STATUS = 3
)

// MAV_ODID_HEIGHT_REF type
type MAV_ODID_HEIGHT_REF int

const (
	// MAV_ODID_HEIGHT_REF_OVER_TAKEOFF enum. The height field is relative to the take-off location
	MAV_ODID_HEIGHT_REF_OVER_TAKEOFF MAV_ODID_HEIGHT_REF = 0
	// MAV_ODID_HEIGHT_REF_OVER_GROUND enum. The height field is relative to ground
	MAV_ODID_HEIGHT_REF_OVER_GROUND MAV_ODID_HEIGHT_REF = 1
)

// MAV_ODID_HOR_ACC type
type MAV_ODID_HOR_ACC int

const (
	// MAV_ODID_HOR_ACC_UNKNOWN enum. The horizontal accuracy is unknown
	MAV_ODID_HOR_ACC_UNKNOWN MAV_ODID_HOR_ACC = 0
	// MAV_ODID_HOR_ACC_10NM enum. The horizontal accuracy is smaller than 10 Nautical Miles. 18.52 km
	MAV_ODID_HOR_ACC_10NM MAV_ODID_HOR_ACC = 1
	// MAV_ODID_HOR_ACC_4NM enum. The horizontal accuracy is smaller than 4 Nautical Miles. 7.408 km
	MAV_ODID_HOR_ACC_4NM MAV_ODID_HOR_ACC = 2
	// MAV_ODID_HOR_ACC_2NM enum. The horizontal accuracy is smaller than 2 Nautical Miles. 3.704 km
	MAV_ODID_HOR_ACC_2NM MAV_ODID_HOR_ACC = 3
	// MAV_ODID_HOR_ACC_1NM enum. The horizontal accuracy is smaller than 1 Nautical Miles. 1.852 km
	MAV_ODID_HOR_ACC_1NM MAV_ODID_HOR_ACC = 4
	// MAV_ODID_HOR_ACC_0_5NM enum. The horizontal accuracy is smaller than 0.5 Nautical Miles. 926 m
	MAV_ODID_HOR_ACC_0_5NM MAV_ODID_HOR_ACC = 5
	// MAV_ODID_HOR_ACC_0_3NM enum. The horizontal accuracy is smaller than 0.3 Nautical Miles. 555.6 m
	MAV_ODID_HOR_ACC_0_3NM MAV_ODID_HOR_ACC = 6
	// MAV_ODID_HOR_ACC_0_1NM enum. The horizontal accuracy is smaller than 0.1 Nautical Miles. 185.2 m
	MAV_ODID_HOR_ACC_0_1NM MAV_ODID_HOR_ACC = 7
	// MAV_ODID_HOR_ACC_0_05NM enum. The horizontal accuracy is smaller than 0.05 Nautical Miles. 92.6 m
	MAV_ODID_HOR_ACC_0_05NM MAV_ODID_HOR_ACC = 8
	// MAV_ODID_HOR_ACC_30_METER enum. The horizontal accuracy is smaller than 30 meter
	MAV_ODID_HOR_ACC_30_METER MAV_ODID_HOR_ACC = 9
	// MAV_ODID_HOR_ACC_10_METER enum. The horizontal accuracy is smaller than 10 meter
	MAV_ODID_HOR_ACC_10_METER MAV_ODID_HOR_ACC = 10
	// MAV_ODID_HOR_ACC_3_METER enum. The horizontal accuracy is smaller than 3 meter
	MAV_ODID_HOR_ACC_3_METER MAV_ODID_HOR_ACC = 11
	// MAV_ODID_HOR_ACC_1_METER enum. The horizontal accuracy is smaller than 1 meter
	MAV_ODID_HOR_ACC_1_METER MAV_ODID_HOR_ACC = 12
)

// MAV_ODID_VER_ACC type
type MAV_ODID_VER_ACC int

const (
	// MAV_ODID_VER_ACC_UNKNOWN enum. The vertical accuracy is unknown
	MAV_ODID_VER_ACC_UNKNOWN MAV_ODID_VER_ACC = 0
	// MAV_ODID_VER_ACC_150_METER enum. The vertical accuracy is smaller than 150 meter
	MAV_ODID_VER_ACC_150_METER MAV_ODID_VER_ACC = 1
	// MAV_ODID_VER_ACC_45_METER enum. The vertical accuracy is smaller than 45 meter
	MAV_ODID_VER_ACC_45_METER MAV_ODID_VER_ACC = 2
	// MAV_ODID_VER_ACC_25_METER enum. The vertical accuracy is smaller than 25 meter
	MAV_ODID_VER_ACC_25_METER MAV_ODID_VER_ACC = 3
	// MAV_ODID_VER_ACC_10_METER enum. The vertical accuracy is smaller than 10 meter
	MAV_ODID_VER_ACC_10_METER MAV_ODID_VER_ACC = 4
	// MAV_ODID_VER_ACC_3_METER enum. The vertical accuracy is smaller than 3 meter
	MAV_ODID_VER_ACC_3_METER MAV_ODID_VER_ACC = 5
	// MAV_ODID_VER_ACC_1_METER enum. The vertical accuracy is smaller than 1 meter
	MAV_ODID_VER_ACC_1_METER MAV_ODID_VER_ACC = 6
)

// MAV_ODID_SPEED_ACC type
type MAV_ODID_SPEED_ACC int

const (
	// MAV_ODID_SPEED_ACC_UNKNOWN enum. The speed accuracy is unknown
	MAV_ODID_SPEED_ACC_UNKNOWN MAV_ODID_SPEED_ACC = 0
	// MAV_ODID_SPEED_ACC_10_METERS_PER_SECOND enum. The speed accuracy is smaller than 10 meters per second
	MAV_ODID_SPEED_ACC_10_METERS_PER_SECOND MAV_ODID_SPEED_ACC = 1
	// MAV_ODID_SPEED_ACC_3_METERS_PER_SECOND enum. The speed accuracy is smaller than 3 meters per second
	MAV_ODID_SPEED_ACC_3_METERS_PER_SECOND MAV_ODID_SPEED_ACC = 2
	// MAV_ODID_SPEED_ACC_1_METERS_PER_SECOND enum. The speed accuracy is smaller than 1 meters per second
	MAV_ODID_SPEED_ACC_1_METERS_PER_SECOND MAV_ODID_SPEED_ACC = 3
	// MAV_ODID_SPEED_ACC_0_3_METERS_PER_SECOND enum. The speed accuracy is smaller than 0.3 meters per second
	MAV_ODID_SPEED_ACC_0_3_METERS_PER_SECOND MAV_ODID_SPEED_ACC = 4
)

// MAV_ODID_TIME_ACC type
type MAV_ODID_TIME_ACC int

const (
	// MAV_ODID_TIME_ACC_UNKNOWN enum. The timestamp accuracy is unknown
	MAV_ODID_TIME_ACC_UNKNOWN MAV_ODID_TIME_ACC = 0
	// MAV_ODID_TIME_ACC_0_1_SECOND enum. The timestamp accuracy is smaller than or equal to 0.1 second
	MAV_ODID_TIME_ACC_0_1_SECOND MAV_ODID_TIME_ACC = 1
	// MAV_ODID_TIME_ACC_0_2_SECOND enum. The timestamp accuracy is smaller than or equal to 0.2 second
	MAV_ODID_TIME_ACC_0_2_SECOND MAV_ODID_TIME_ACC = 2
	// MAV_ODID_TIME_ACC_0_3_SECOND enum. The timestamp accuracy is smaller than or equal to 0.3 second
	MAV_ODID_TIME_ACC_0_3_SECOND MAV_ODID_TIME_ACC = 3
	// MAV_ODID_TIME_ACC_0_4_SECOND enum. The timestamp accuracy is smaller than or equal to 0.4 second
	MAV_ODID_TIME_ACC_0_4_SECOND MAV_ODID_TIME_ACC = 4
	// MAV_ODID_TIME_ACC_0_5_SECOND enum. The timestamp accuracy is smaller than or equal to 0.5 second
	MAV_ODID_TIME_ACC_0_5_SECOND MAV_ODID_TIME_ACC = 5
	// MAV_ODID_TIME_ACC_0_6_SECOND enum. The timestamp accuracy is smaller than or equal to 0.6 second
	MAV_ODID_TIME_ACC_0_6_SECOND MAV_ODID_TIME_ACC = 6
	// MAV_ODID_TIME_ACC_0_7_SECOND enum. The timestamp accuracy is smaller than or equal to 0.7 second
	MAV_ODID_TIME_ACC_0_7_SECOND MAV_ODID_TIME_ACC = 7
	// MAV_ODID_TIME_ACC_0_8_SECOND enum. The timestamp accuracy is smaller than or equal to 0.8 second
	MAV_ODID_TIME_ACC_0_8_SECOND MAV_ODID_TIME_ACC = 8
	// MAV_ODID_TIME_ACC_0_9_SECOND enum. The timestamp accuracy is smaller than or equal to 0.9 second
	MAV_ODID_TIME_ACC_0_9_SECOND MAV_ODID_TIME_ACC = 9
	// MAV_ODID_TIME_ACC_1_0_SECOND enum. The timestamp accuracy is smaller than or equal to 1.0 second
	MAV_ODID_TIME_ACC_1_0_SECOND MAV_ODID_TIME_ACC = 10
	// MAV_ODID_TIME_ACC_1_1_SECOND enum. The timestamp accuracy is smaller than or equal to 1.1 second
	MAV_ODID_TIME_ACC_1_1_SECOND MAV_ODID_TIME_ACC = 11
	// MAV_ODID_TIME_ACC_1_2_SECOND enum. The timestamp accuracy is smaller than or equal to 1.2 second
	MAV_ODID_TIME_ACC_1_2_SECOND MAV_ODID_TIME_ACC = 12
	// MAV_ODID_TIME_ACC_1_3_SECOND enum. The timestamp accuracy is smaller than or equal to 1.3 second
	MAV_ODID_TIME_ACC_1_3_SECOND MAV_ODID_TIME_ACC = 13
	// MAV_ODID_TIME_ACC_1_4_SECOND enum. The timestamp accuracy is smaller than or equal to 1.4 second
	MAV_ODID_TIME_ACC_1_4_SECOND MAV_ODID_TIME_ACC = 14
	// MAV_ODID_TIME_ACC_1_5_SECOND enum. The timestamp accuracy is smaller than or equal to 1.5 second
	MAV_ODID_TIME_ACC_1_5_SECOND MAV_ODID_TIME_ACC = 15
)

// MAV_ODID_AUTH_TYPE type
type MAV_ODID_AUTH_TYPE int

const (
	// MAV_ODID_AUTH_TYPE_NONE enum. No authentication type is specified
	MAV_ODID_AUTH_TYPE_NONE MAV_ODID_AUTH_TYPE = 0
	// MAV_ODID_AUTH_TYPE_UAS_ID_SIGNATURE enum. Signature for the UAS (Unmanned Aircraft System) ID
	MAV_ODID_AUTH_TYPE_UAS_ID_SIGNATURE MAV_ODID_AUTH_TYPE = 1
	// MAV_ODID_AUTH_TYPE_OPERATOR_ID_SIGNATURE enum. Signature for the Operator ID
	MAV_ODID_AUTH_TYPE_OPERATOR_ID_SIGNATURE MAV_ODID_AUTH_TYPE = 2
	// MAV_ODID_AUTH_TYPE_MESSAGE_SET_SIGNATURE enum. Signature for the entire message set
	MAV_ODID_AUTH_TYPE_MESSAGE_SET_SIGNATURE MAV_ODID_AUTH_TYPE = 3
	// MAV_ODID_AUTH_TYPE_NETWORK_REMOTE_ID enum. Authentication is provided by Network Remote ID
	MAV_ODID_AUTH_TYPE_NETWORK_REMOTE_ID MAV_ODID_AUTH_TYPE = 4
)

// MAV_ODID_DESC_TYPE type
type MAV_ODID_DESC_TYPE int

const (
	// MAV_ODID_DESC_TYPE_TEXT enum. Free-form text description of the purpose of the flight
	MAV_ODID_DESC_TYPE_TEXT MAV_ODID_DESC_TYPE = 0
)

// MAV_ODID_OPERATOR_LOCATION_TYPE type
type MAV_ODID_OPERATOR_LOCATION_TYPE int

const (
	// MAV_ODID_OPERATOR_LOCATION_TYPE_TAKEOFF enum. The location of the operator is the same as the take-off location
	MAV_ODID_OPERATOR_LOCATION_TYPE_TAKEOFF MAV_ODID_OPERATOR_LOCATION_TYPE = 0
	// MAV_ODID_OPERATOR_LOCATION_TYPE_LIVE_GNSS enum. The location of the operator is based on live GNSS data
	MAV_ODID_OPERATOR_LOCATION_TYPE_LIVE_GNSS MAV_ODID_OPERATOR_LOCATION_TYPE = 1
	// MAV_ODID_OPERATOR_LOCATION_TYPE_FIXED enum. The location of the operator is a fixed location
	MAV_ODID_OPERATOR_LOCATION_TYPE_FIXED MAV_ODID_OPERATOR_LOCATION_TYPE = 2
)

// MAV_ODID_CLASSIFICATION_TYPE type
type MAV_ODID_CLASSIFICATION_TYPE int

const (
	// MAV_ODID_CLASSIFICATION_TYPE_UNDECLARED enum. The classification type for the UA is undeclared
	MAV_ODID_CLASSIFICATION_TYPE_UNDECLARED MAV_ODID_CLASSIFICATION_TYPE = 0
	// MAV_ODID_CLASSIFICATION_TYPE_EU enum. The classification type for the UA follows EU (European Union) specifications
	MAV_ODID_CLASSIFICATION_TYPE_EU MAV_ODID_CLASSIFICATION_TYPE = 1
)

// MAV_ODID_CATEGORY_EU type
type MAV_ODID_CATEGORY_EU int

const (
	// MAV_ODID_CATEGORY_EU_UNDECLARED enum. The category for the UA, according to the EU specification, is undeclared
	MAV_ODID_CATEGORY_EU_UNDECLARED MAV_ODID_CATEGORY_EU = 0
	// MAV_ODID_CATEGORY_EU_OPEN enum. The category for the UA, according to the EU specification, is the Open category
	MAV_ODID_CATEGORY_EU_OPEN MAV_ODID_CATEGORY_EU = 1
	// MAV_ODID_CATEGORY_EU_SPECIFIC enum. The category for the UA, according to the EU specification, is the Specific category
	MAV_ODID_CATEGORY_EU_SPECIFIC MAV_ODID_CATEGORY_EU = 2
	// MAV_ODID_CATEGORY_EU_CERTIFIED enum. The category for the UA, according to the EU specification, is the Certified category
	MAV_ODID_CATEGORY_EU_CERTIFIED MAV_ODID_CATEGORY_EU = 3
)

// MAV_ODID_CLASS_EU type
type MAV_ODID_CLASS_EU int

const (
	// MAV_ODID_CLASS_EU_UNDECLARED enum. The class for the UA, according to the EU specification, is undeclared
	MAV_ODID_CLASS_EU_UNDECLARED MAV_ODID_CLASS_EU = 0
	// MAV_ODID_CLASS_EU_CLASS_0 enum. The class for the UA, according to the EU specification, is Class 0
	MAV_ODID_CLASS_EU_CLASS_0 MAV_ODID_CLASS_EU = 1
	// MAV_ODID_CLASS_EU_CLASS_1 enum. The class for the UA, according to the EU specification, is Class 1
	MAV_ODID_CLASS_EU_CLASS_1 MAV_ODID_CLASS_EU = 2
	// MAV_ODID_CLASS_EU_CLASS_2 enum. The class for the UA, according to the EU specification, is Class 2
	MAV_ODID_CLASS_EU_CLASS_2 MAV_ODID_CLASS_EU = 3
	// MAV_ODID_CLASS_EU_CLASS_3 enum. The class for the UA, according to the EU specification, is Class 3
	MAV_ODID_CLASS_EU_CLASS_3 MAV_ODID_CLASS_EU = 4
	// MAV_ODID_CLASS_EU_CLASS_4 enum. The class for the UA, according to the EU specification, is Class 4
	MAV_ODID_CLASS_EU_CLASS_4 MAV_ODID_CLASS_EU = 5
	// MAV_ODID_CLASS_EU_CLASS_5 enum. The class for the UA, according to the EU specification, is Class 5
	MAV_ODID_CLASS_EU_CLASS_5 MAV_ODID_CLASS_EU = 6
	// MAV_ODID_CLASS_EU_CLASS_6 enum. The class for the UA, according to the EU specification, is Class 6
	MAV_ODID_CLASS_EU_CLASS_6 MAV_ODID_CLASS_EU = 7
)

// MAV_ODID_OPERATOR_ID_TYPE type
type MAV_ODID_OPERATOR_ID_TYPE int

const (
	// MAV_ODID_OPERATOR_ID_TYPE_CAA enum. CAA (Civil Aviation Authority) registered operator ID
	MAV_ODID_OPERATOR_ID_TYPE_CAA MAV_ODID_OPERATOR_ID_TYPE = 0
)

// TUNE_FORMAT type. Tune formats (used for vehicle buzzer/tone generation).
type TUNE_FORMAT int

const (
	// TUNE_FORMAT_QBASIC1_1 enum. Format is QBasic 1.1 Play: https://www.qbasic.net/en/reference/qb11/Statement/PLAY-006.htm
	TUNE_FORMAT_QBASIC1_1 TUNE_FORMAT = 1
	// TUNE_FORMAT_MML_MODERN enum. Format is Modern Music Markup Language (MML): https://en.wikipedia.org/wiki/Music_Macro_Language#Modern_MML
	TUNE_FORMAT_MML_MODERN TUNE_FORMAT = 2
)

// COMPONENT_CAP_FLAGS type. Component capability flags (Bitmap)
type COMPONENT_CAP_FLAGS int

const (
	// COMPONENT_CAP_FLAGS_PARAM enum. Component has parameters, and supports the parameter protocol (PARAM messages)
	COMPONENT_CAP_FLAGS_PARAM COMPONENT_CAP_FLAGS = 1
	// COMPONENT_CAP_FLAGS_PARAM_EXT enum. Component has parameters, and supports the extended parameter protocol (PARAM_EXT messages)
	COMPONENT_CAP_FLAGS_PARAM_EXT COMPONENT_CAP_FLAGS = 2
)

// AIS_TYPE type. Type of AIS vessel, enum duplicated from AIS standard, https://gpsd.gitlab.io/gpsd/AIVDM.html
type AIS_TYPE int

const (
	// AIS_TYPE_UNKNOWN enum. Not available (default)
	AIS_TYPE_UNKNOWN AIS_TYPE = 0
	// AIS_TYPE_RESERVED_1 enum
	AIS_TYPE_RESERVED_1 AIS_TYPE = 1
	// AIS_TYPE_RESERVED_2 enum
	AIS_TYPE_RESERVED_2 AIS_TYPE = 2
	// AIS_TYPE_RESERVED_3 enum
	AIS_TYPE_RESERVED_3 AIS_TYPE = 3
	// AIS_TYPE_RESERVED_4 enum
	AIS_TYPE_RESERVED_4 AIS_TYPE = 4
	// AIS_TYPE_RESERVED_5 enum
	AIS_TYPE_RESERVED_5 AIS_TYPE = 5
	// AIS_TYPE_RESERVED_6 enum
	AIS_TYPE_RESERVED_6 AIS_TYPE = 6
	// AIS_TYPE_RESERVED_7 enum
	AIS_TYPE_RESERVED_7 AIS_TYPE = 7
	// AIS_TYPE_RESERVED_8 enum
	AIS_TYPE_RESERVED_8 AIS_TYPE = 8
	// AIS_TYPE_RESERVED_9 enum
	AIS_TYPE_RESERVED_9 AIS_TYPE = 9
	// AIS_TYPE_RESERVED_10 enum
	AIS_TYPE_RESERVED_10 AIS_TYPE = 10
	// AIS_TYPE_RESERVED_11 enum
	AIS_TYPE_RESERVED_11 AIS_TYPE = 11
	// AIS_TYPE_RESERVED_12 enum
	AIS_TYPE_RESERVED_12 AIS_TYPE = 12
	// AIS_TYPE_RESERVED_13 enum
	AIS_TYPE_RESERVED_13 AIS_TYPE = 13
	// AIS_TYPE_RESERVED_14 enum
	AIS_TYPE_RESERVED_14 AIS_TYPE = 14
	// AIS_TYPE_RESERVED_15 enum
	AIS_TYPE_RESERVED_15 AIS_TYPE = 15
	// AIS_TYPE_RESERVED_16 enum
	AIS_TYPE_RESERVED_16 AIS_TYPE = 16
	// AIS_TYPE_RESERVED_17 enum
	AIS_TYPE_RESERVED_17 AIS_TYPE = 17
	// AIS_TYPE_RESERVED_18 enum
	AIS_TYPE_RESERVED_18 AIS_TYPE = 18
	// AIS_TYPE_RESERVED_19 enum
	AIS_TYPE_RESERVED_19 AIS_TYPE = 19
	// AIS_TYPE_WIG enum. Wing In Ground effect
	AIS_TYPE_WIG AIS_TYPE = 20
	// AIS_TYPE_WIG_HAZARDOUS_A enum
	AIS_TYPE_WIG_HAZARDOUS_A AIS_TYPE = 21
	// AIS_TYPE_WIG_HAZARDOUS_B enum
	AIS_TYPE_WIG_HAZARDOUS_B AIS_TYPE = 22
	// AIS_TYPE_WIG_HAZARDOUS_C enum
	AIS_TYPE_WIG_HAZARDOUS_C AIS_TYPE = 23
	// AIS_TYPE_WIG_HAZARDOUS_D enum
	AIS_TYPE_WIG_HAZARDOUS_D AIS_TYPE = 24
	// AIS_TYPE_WIG_RESERVED_1 enum
	AIS_TYPE_WIG_RESERVED_1 AIS_TYPE = 25
	// AIS_TYPE_WIG_RESERVED_2 enum
	AIS_TYPE_WIG_RESERVED_2 AIS_TYPE = 26
	// AIS_TYPE_WIG_RESERVED_3 enum
	AIS_TYPE_WIG_RESERVED_3 AIS_TYPE = 27
	// AIS_TYPE_WIG_RESERVED_4 enum
	AIS_TYPE_WIG_RESERVED_4 AIS_TYPE = 28
	// AIS_TYPE_WIG_RESERVED_5 enum
	AIS_TYPE_WIG_RESERVED_5 AIS_TYPE = 29
	// AIS_TYPE_FISHING enum
	AIS_TYPE_FISHING AIS_TYPE = 30
	// AIS_TYPE_TOWING enum
	AIS_TYPE_TOWING AIS_TYPE = 31
	// AIS_TYPE_TOWING_LARGE enum. Towing: length exceeds 200m or breadth exceeds 25m
	AIS_TYPE_TOWING_LARGE AIS_TYPE = 32
	// AIS_TYPE_DREDGING enum. Dredging or other underwater ops
	AIS_TYPE_DREDGING AIS_TYPE = 33
	// AIS_TYPE_DIVING enum
	AIS_TYPE_DIVING AIS_TYPE = 34
	// AIS_TYPE_MILITARY enum
	AIS_TYPE_MILITARY AIS_TYPE = 35
	// AIS_TYPE_SAILING enum
	AIS_TYPE_SAILING AIS_TYPE = 36
	// AIS_TYPE_PLEASURE enum
	AIS_TYPE_PLEASURE AIS_TYPE = 37
	// AIS_TYPE_RESERVED_20 enum
	AIS_TYPE_RESERVED_20 AIS_TYPE = 38
	// AIS_TYPE_RESERVED_21 enum
	AIS_TYPE_RESERVED_21 AIS_TYPE = 39
	// AIS_TYPE_HSC enum. High Speed Craft
	AIS_TYPE_HSC AIS_TYPE = 40
	// AIS_TYPE_HSC_HAZARDOUS_A enum
	AIS_TYPE_HSC_HAZARDOUS_A AIS_TYPE = 41
	// AIS_TYPE_HSC_HAZARDOUS_B enum
	AIS_TYPE_HSC_HAZARDOUS_B AIS_TYPE = 42
	// AIS_TYPE_HSC_HAZARDOUS_C enum
	AIS_TYPE_HSC_HAZARDOUS_C AIS_TYPE = 43
	// AIS_TYPE_HSC_HAZARDOUS_D enum
	AIS_TYPE_HSC_HAZARDOUS_D AIS_TYPE = 44
	// AIS_TYPE_HSC_RESERVED_1 enum
	AIS_TYPE_HSC_RESERVED_1 AIS_TYPE = 45
	// AIS_TYPE_HSC_RESERVED_2 enum
	AIS_TYPE_HSC_RESERVED_2 AIS_TYPE = 46
	// AIS_TYPE_HSC_RESERVED_3 enum
	AIS_TYPE_HSC_RESERVED_3 AIS_TYPE = 47
	// AIS_TYPE_HSC_RESERVED_4 enum
	AIS_TYPE_HSC_RESERVED_4 AIS_TYPE = 48
	// AIS_TYPE_HSC_UNKNOWN enum
	AIS_TYPE_HSC_UNKNOWN AIS_TYPE = 49
	// AIS_TYPE_PILOT enum
	AIS_TYPE_PILOT AIS_TYPE = 50
	// AIS_TYPE_SAR enum. Search And Rescue vessel
	AIS_TYPE_SAR AIS_TYPE = 51
	// AIS_TYPE_TUG enum
	AIS_TYPE_TUG AIS_TYPE = 52
	// AIS_TYPE_PORT_TENDER enum
	AIS_TYPE_PORT_TENDER AIS_TYPE = 53
	// AIS_TYPE_ANTI_POLLUTION enum. Anti-pollution equipment
	AIS_TYPE_ANTI_POLLUTION AIS_TYPE = 54
	// AIS_TYPE_LAW_ENFORCEMENT enum
	AIS_TYPE_LAW_ENFORCEMENT AIS_TYPE = 55
	// AIS_TYPE_SPARE_LOCAL_1 enum
	AIS_TYPE_SPARE_LOCAL_1 AIS_TYPE = 56
	// AIS_TYPE_SPARE_LOCAL_2 enum
	AIS_TYPE_SPARE_LOCAL_2 AIS_TYPE = 57
	// AIS_TYPE_MEDICAL_TRANSPORT enum
	AIS_TYPE_MEDICAL_TRANSPORT AIS_TYPE = 58
	// AIS_TYPE_NONECOMBATANT enum. Noncombatant ship according to RR Resolution No. 18
	AIS_TYPE_NONECOMBATANT AIS_TYPE = 59
	// AIS_TYPE_PASSENGER enum
	AIS_TYPE_PASSENGER AIS_TYPE = 60
	// AIS_TYPE_PASSENGER_HAZARDOUS_A enum
	AIS_TYPE_PASSENGER_HAZARDOUS_A AIS_TYPE = 61
	// AIS_TYPE_PASSENGER_HAZARDOUS_B enum
	AIS_TYPE_PASSENGER_HAZARDOUS_B AIS_TYPE = 62
	// AIS_TYPE_AIS_TYPE_PASSENGER_HAZARDOUS_C enum
	AIS_TYPE_AIS_TYPE_PASSENGER_HAZARDOUS_C AIS_TYPE = 63
	// AIS_TYPE_PASSENGER_HAZARDOUS_D enum
	AIS_TYPE_PASSENGER_HAZARDOUS_D AIS_TYPE = 64
	// AIS_TYPE_PASSENGER_RESERVED_1 enum
	AIS_TYPE_PASSENGER_RESERVED_1 AIS_TYPE = 65
	// AIS_TYPE_PASSENGER_RESERVED_2 enum
	AIS_TYPE_PASSENGER_RESERVED_2 AIS_TYPE = 66
	// AIS_TYPE_PASSENGER_RESERVED_3 enum
	AIS_TYPE_PASSENGER_RESERVED_3 AIS_TYPE = 67
	// AIS_TYPE_AIS_TYPE_PASSENGER_RESERVED_4 enum
	AIS_TYPE_AIS_TYPE_PASSENGER_RESERVED_4 AIS_TYPE = 68
	// AIS_TYPE_PASSENGER_UNKNOWN enum
	AIS_TYPE_PASSENGER_UNKNOWN AIS_TYPE = 69
	// AIS_TYPE_CARGO enum
	AIS_TYPE_CARGO AIS_TYPE = 70
	// AIS_TYPE_CARGO_HAZARDOUS_A enum
	AIS_TYPE_CARGO_HAZARDOUS_A AIS_TYPE = 71
	// AIS_TYPE_CARGO_HAZARDOUS_B enum
	AIS_TYPE_CARGO_HAZARDOUS_B AIS_TYPE = 72
	// AIS_TYPE_CARGO_HAZARDOUS_C enum
	AIS_TYPE_CARGO_HAZARDOUS_C AIS_TYPE = 73
	// AIS_TYPE_CARGO_HAZARDOUS_D enum
	AIS_TYPE_CARGO_HAZARDOUS_D AIS_TYPE = 74
	// AIS_TYPE_CARGO_RESERVED_1 enum
	AIS_TYPE_CARGO_RESERVED_1 AIS_TYPE = 75
	// AIS_TYPE_CARGO_RESERVED_2 enum
	AIS_TYPE_CARGO_RESERVED_2 AIS_TYPE = 76
	// AIS_TYPE_CARGO_RESERVED_3 enum
	AIS_TYPE_CARGO_RESERVED_3 AIS_TYPE = 77
	// AIS_TYPE_CARGO_RESERVED_4 enum
	AIS_TYPE_CARGO_RESERVED_4 AIS_TYPE = 78
	// AIS_TYPE_CARGO_UNKNOWN enum
	AIS_TYPE_CARGO_UNKNOWN AIS_TYPE = 79
	// AIS_TYPE_TANKER enum
	AIS_TYPE_TANKER AIS_TYPE = 80
	// AIS_TYPE_TANKER_HAZARDOUS_A enum
	AIS_TYPE_TANKER_HAZARDOUS_A AIS_TYPE = 81
	// AIS_TYPE_TANKER_HAZARDOUS_B enum
	AIS_TYPE_TANKER_HAZARDOUS_B AIS_TYPE = 82
	// AIS_TYPE_TANKER_HAZARDOUS_C enum
	AIS_TYPE_TANKER_HAZARDOUS_C AIS_TYPE = 83
	// AIS_TYPE_TANKER_HAZARDOUS_D enum
	AIS_TYPE_TANKER_HAZARDOUS_D AIS_TYPE = 84
	// AIS_TYPE_TANKER_RESERVED_1 enum
	AIS_TYPE_TANKER_RESERVED_1 AIS_TYPE = 85
	// AIS_TYPE_TANKER_RESERVED_2 enum
	AIS_TYPE_TANKER_RESERVED_2 AIS_TYPE = 86
	// AIS_TYPE_TANKER_RESERVED_3 enum
	AIS_TYPE_TANKER_RESERVED_3 AIS_TYPE = 87
	// AIS_TYPE_TANKER_RESERVED_4 enum
	AIS_TYPE_TANKER_RESERVED_4 AIS_TYPE = 88
	// AIS_TYPE_TANKER_UNKNOWN enum
	AIS_TYPE_TANKER_UNKNOWN AIS_TYPE = 89
	// AIS_TYPE_OTHER enum
	AIS_TYPE_OTHER AIS_TYPE = 90
	// AIS_TYPE_OTHER_HAZARDOUS_A enum
	AIS_TYPE_OTHER_HAZARDOUS_A AIS_TYPE = 91
	// AIS_TYPE_OTHER_HAZARDOUS_B enum
	AIS_TYPE_OTHER_HAZARDOUS_B AIS_TYPE = 92
	// AIS_TYPE_OTHER_HAZARDOUS_C enum
	AIS_TYPE_OTHER_HAZARDOUS_C AIS_TYPE = 93
	// AIS_TYPE_OTHER_HAZARDOUS_D enum
	AIS_TYPE_OTHER_HAZARDOUS_D AIS_TYPE = 94
	// AIS_TYPE_OTHER_RESERVED_1 enum
	AIS_TYPE_OTHER_RESERVED_1 AIS_TYPE = 95
	// AIS_TYPE_OTHER_RESERVED_2 enum
	AIS_TYPE_OTHER_RESERVED_2 AIS_TYPE = 96
	// AIS_TYPE_OTHER_RESERVED_3 enum
	AIS_TYPE_OTHER_RESERVED_3 AIS_TYPE = 97
	// AIS_TYPE_OTHER_RESERVED_4 enum
	AIS_TYPE_OTHER_RESERVED_4 AIS_TYPE = 98
	// AIS_TYPE_OTHER_UNKNOWN enum
	AIS_TYPE_OTHER_UNKNOWN AIS_TYPE = 99
)

// AIS_NAV_STATUS type. Navigational status of AIS vessel, enum duplicated from AIS standard, https://gpsd.gitlab.io/gpsd/AIVDM.html
type AIS_NAV_STATUS int

const (
	// UNDER_WAY enum. Under way using engine
	UNDER_WAY AIS_NAV_STATUS = 0
	// AIS_NAV_ANCHORED enum
	AIS_NAV_ANCHORED AIS_NAV_STATUS = 1
	// AIS_NAV_UN_COMMANDED enum
	AIS_NAV_UN_COMMANDED AIS_NAV_STATUS = 2
	// AIS_NAV_RESTRICTED_MANOEUVERABILITY enum
	AIS_NAV_RESTRICTED_MANOEUVERABILITY AIS_NAV_STATUS = 3
	// AIS_NAV_DRAUGHT_CONSTRAINED enum
	AIS_NAV_DRAUGHT_CONSTRAINED AIS_NAV_STATUS = 4
	// AIS_NAV_MOORED enum
	AIS_NAV_MOORED AIS_NAV_STATUS = 5
	// AIS_NAV_AGROUND enum
	AIS_NAV_AGROUND AIS_NAV_STATUS = 6
	// AIS_NAV_FISHING enum
	AIS_NAV_FISHING AIS_NAV_STATUS = 7
	// AIS_NAV_SAILING enum
	AIS_NAV_SAILING AIS_NAV_STATUS = 8
	// AIS_NAV_RESERVED_HSC enum
	AIS_NAV_RESERVED_HSC AIS_NAV_STATUS = 9
	// AIS_NAV_RESERVED_WIG enum
	AIS_NAV_RESERVED_WIG AIS_NAV_STATUS = 10
	// AIS_NAV_RESERVED_1 enum
	AIS_NAV_RESERVED_1 AIS_NAV_STATUS = 11
	// AIS_NAV_RESERVED_2 enum
	AIS_NAV_RESERVED_2 AIS_NAV_STATUS = 12
	// AIS_NAV_RESERVED_3 enum
	AIS_NAV_RESERVED_3 AIS_NAV_STATUS = 13
	// AIS_NAV_AIS_SART enum. Search And Rescue Transponder
	AIS_NAV_AIS_SART AIS_NAV_STATUS = 14
	// AIS_NAV_UNKNOWN enum. Not available (default)
	AIS_NAV_UNKNOWN AIS_NAV_STATUS = 15
)

// AIS_FLAGS type. These flags are used in the AIS_VESSEL.fields bitmask to indicate validity of data in the other message fields. When set, the data is valid.
type AIS_FLAGS int

const (
	// AIS_FLAGS_POSITION_ACCURACY enum. 1 = Position accuracy less than 10m, 0 = position accuracy greater than 10m
	AIS_FLAGS_POSITION_ACCURACY AIS_FLAGS = 1
	// AIS_FLAGS_VALID_COG enum
	AIS_FLAGS_VALID_COG AIS_FLAGS = 2
	// AIS_FLAGS_VALID_VELOCITY enum
	AIS_FLAGS_VALID_VELOCITY AIS_FLAGS = 4
	// AIS_FLAGS_HIGH_VELOCITY enum. 1 = Velocity over 52.5765m/s (102.2 knots)
	AIS_FLAGS_HIGH_VELOCITY AIS_FLAGS = 8
	// AIS_FLAGS_VALID_TURN_RATE enum
	AIS_FLAGS_VALID_TURN_RATE AIS_FLAGS = 16
	// AIS_FLAGS_TURN_RATE_SIGN_ONLY enum. Only the sign of the returned turn rate value is valid, either greater than 5deg/30s or less than -5deg/30s
	AIS_FLAGS_TURN_RATE_SIGN_ONLY AIS_FLAGS = 32
	// AIS_FLAGS_VALID_DIMENSIONS enum
	AIS_FLAGS_VALID_DIMENSIONS AIS_FLAGS = 64
	// AIS_FLAGS_LARGE_BOW_DIMENSION enum. Distance to bow is larger than 511m
	AIS_FLAGS_LARGE_BOW_DIMENSION AIS_FLAGS = 128
	// AIS_FLAGS_LARGE_STERN_DIMENSION enum. Distance to stern is larger than 511m
	AIS_FLAGS_LARGE_STERN_DIMENSION AIS_FLAGS = 256
	// AIS_FLAGS_LARGE_PORT_DIMENSION enum. Distance to port side is larger than 63m
	AIS_FLAGS_LARGE_PORT_DIMENSION AIS_FLAGS = 512
	// AIS_FLAGS_LARGE_STARBOARD_DIMENSION enum. Distance to starboard side is larger than 63m
	AIS_FLAGS_LARGE_STARBOARD_DIMENSION AIS_FLAGS = 1024
	// AIS_FLAGS_VALID_CALLSIGN enum
	AIS_FLAGS_VALID_CALLSIGN AIS_FLAGS = 2048
	// AIS_FLAGS_VALID_NAME enum
	AIS_FLAGS_VALID_NAME AIS_FLAGS = 4096
)

// FAILURE_UNIT type. List of possible units where failures can be injected.
type FAILURE_UNIT int

const (
	// FAILURE_UNIT_SENSOR_GYRO enum
	FAILURE_UNIT_SENSOR_GYRO FAILURE_UNIT = 0
	// FAILURE_UNIT_SENSOR_ACCEL enum
	FAILURE_UNIT_SENSOR_ACCEL FAILURE_UNIT = 1
	// FAILURE_UNIT_SENSOR_MAG enum
	FAILURE_UNIT_SENSOR_MAG FAILURE_UNIT = 2
	// FAILURE_UNIT_SENSOR_BARO enum
	FAILURE_UNIT_SENSOR_BARO FAILURE_UNIT = 3
	// FAILURE_UNIT_SENSOR_GPS enum
	FAILURE_UNIT_SENSOR_GPS FAILURE_UNIT = 4
	// FAILURE_UNIT_SENSOR_OPTICAL_FLOW enum
	FAILURE_UNIT_SENSOR_OPTICAL_FLOW FAILURE_UNIT = 5
	// FAILURE_UNIT_SENSOR_VIO enum
	FAILURE_UNIT_SENSOR_VIO FAILURE_UNIT = 6
	// FAILURE_UNIT_SENSOR_DISTANCE_SENSOR enum
	FAILURE_UNIT_SENSOR_DISTANCE_SENSOR FAILURE_UNIT = 7
	// FAILURE_UNIT_SENSOR_AIRSPEED enum
	FAILURE_UNIT_SENSOR_AIRSPEED FAILURE_UNIT = 8
	// FAILURE_UNIT_SYSTEM_BATTERY enum
	FAILURE_UNIT_SYSTEM_BATTERY FAILURE_UNIT = 100
	// FAILURE_UNIT_SYSTEM_MOTOR enum
	FAILURE_UNIT_SYSTEM_MOTOR FAILURE_UNIT = 101
	// FAILURE_UNIT_SYSTEM_SERVO enum
	FAILURE_UNIT_SYSTEM_SERVO FAILURE_UNIT = 102
	// FAILURE_UNIT_SYSTEM_AVOIDANCE enum
	FAILURE_UNIT_SYSTEM_AVOIDANCE FAILURE_UNIT = 103
	// FAILURE_UNIT_SYSTEM_RC_SIGNAL enum
	FAILURE_UNIT_SYSTEM_RC_SIGNAL FAILURE_UNIT = 104
	// FAILURE_UNIT_SYSTEM_MAVLINK_SIGNAL enum
	FAILURE_UNIT_SYSTEM_MAVLINK_SIGNAL FAILURE_UNIT = 105
)

// FAILURE_TYPE type. List of possible failure type to inject.
type FAILURE_TYPE int

const (
	// FAILURE_TYPE_OK enum. No failure injected, used to reset a previous failure
	FAILURE_TYPE_OK FAILURE_TYPE = 0
	// FAILURE_TYPE_OFF enum. Sets unit off, so completely non-responsive
	FAILURE_TYPE_OFF FAILURE_TYPE = 1
	// FAILURE_TYPE_STUCK enum. Unit is stuck e.g. keeps reporting the same value
	FAILURE_TYPE_STUCK FAILURE_TYPE = 2
	// FAILURE_TYPE_GARBAGE enum. Unit is reporting complete garbage
	FAILURE_TYPE_GARBAGE FAILURE_TYPE = 3
	// FAILURE_TYPE_WRONG enum. Unit is consistently wrong
	FAILURE_TYPE_WRONG FAILURE_TYPE = 4
	// FAILURE_TYPE_SLOW enum. Unit is slow, so e.g. reporting at slower than expected rate
	FAILURE_TYPE_SLOW FAILURE_TYPE = 5
	// FAILURE_TYPE_DELAYED enum. Data of unit is delayed in time
	FAILURE_TYPE_DELAYED FAILURE_TYPE = 6
	// FAILURE_TYPE_INTERMITTENT enum. Unit is sometimes working, sometimes not
	FAILURE_TYPE_INTERMITTENT FAILURE_TYPE = 7
)

// MAV_WINCH_STATUS_FLAG type. Winch status flags used in WINCH_STATUS
type MAV_WINCH_STATUS_FLAG int

const (
	// MAV_WINCH_STATUS_HEALTHY enum. Winch is healthy
	MAV_WINCH_STATUS_HEALTHY MAV_WINCH_STATUS_FLAG = 1
	// MAV_WINCH_STATUS_FULLY_RETRACTED enum. Winch thread is fully retracted
	MAV_WINCH_STATUS_FULLY_RETRACTED MAV_WINCH_STATUS_FLAG = 2
	// MAV_WINCH_STATUS_MOVING enum. Winch motor is moving
	MAV_WINCH_STATUS_MOVING MAV_WINCH_STATUS_FLAG = 4
	// MAV_WINCH_STATUS_CLUTCH_ENGAGED enum. Winch clutch is engaged allowing motor to move freely
	MAV_WINCH_STATUS_CLUTCH_ENGAGED MAV_WINCH_STATUS_FLAG = 8
)

// MAG_CAL_STATUS type
type MAG_CAL_STATUS int

const (
	// MAG_CAL_NOT_STARTED enum
	MAG_CAL_NOT_STARTED MAG_CAL_STATUS = 0
	// MAG_CAL_WAITING_TO_START enum
	MAG_CAL_WAITING_TO_START MAG_CAL_STATUS = 1
	// MAG_CAL_RUNNING_STEP_ONE enum
	MAG_CAL_RUNNING_STEP_ONE MAG_CAL_STATUS = 2
	// MAG_CAL_RUNNING_STEP_TWO enum
	MAG_CAL_RUNNING_STEP_TWO MAG_CAL_STATUS = 3
	// MAG_CAL_SUCCESS enum
	MAG_CAL_SUCCESS MAG_CAL_STATUS = 4
	// MAG_CAL_FAILED enum
	MAG_CAL_FAILED MAG_CAL_STATUS = 5
	// MAG_CAL_BAD_ORIENTATION enum
	MAG_CAL_BAD_ORIENTATION MAG_CAL_STATUS = 6
	// MAG_CAL_BAD_RADIUS enum
	MAG_CAL_BAD_RADIUS MAG_CAL_STATUS = 7
)

// MAV_AUTOPILOT type. Micro air vehicle / autopilot classes. This identifies the individual model.
type MAV_AUTOPILOT int

const (
	// MAV_AUTOPILOT_GENERIC enum. Generic autopilot, full support for everything
	MAV_AUTOPILOT_GENERIC MAV_AUTOPILOT = 0
	// MAV_AUTOPILOT_RESERVED enum. Reserved for future use
	MAV_AUTOPILOT_RESERVED MAV_AUTOPILOT = 1
	// MAV_AUTOPILOT_SLUGS enum. SLUGS autopilot, http://slugsuav.soe.ucsc.edu
	MAV_AUTOPILOT_SLUGS MAV_AUTOPILOT = 2
	// MAV_AUTOPILOT_ARDUPILOTMEGA enum. ArduPilot - Plane/Copter/Rover/Sub/Tracker, https://ardupilot.org
	MAV_AUTOPILOT_ARDUPILOTMEGA MAV_AUTOPILOT = 3
	// MAV_AUTOPILOT_OPENPILOT enum. OpenPilot, http://openpilot.org
	MAV_AUTOPILOT_OPENPILOT MAV_AUTOPILOT = 4
	// MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY enum. Generic autopilot only supporting simple waypoints
	MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY MAV_AUTOPILOT = 5
	// MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY enum. Generic autopilot supporting waypoints and other simple navigation commands
	MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY MAV_AUTOPILOT = 6
	// MAV_AUTOPILOT_GENERIC_MISSION_FULL enum. Generic autopilot supporting the full mission command set
	MAV_AUTOPILOT_GENERIC_MISSION_FULL MAV_AUTOPILOT = 7
	// MAV_AUTOPILOT_INVALID enum. No valid autopilot, e.g. a GCS or other MAVLink component
	MAV_AUTOPILOT_INVALID MAV_AUTOPILOT = 8
	// MAV_AUTOPILOT_PPZ enum. PPZ UAV - http://nongnu.org/paparazzi
	MAV_AUTOPILOT_PPZ MAV_AUTOPILOT = 9
	// MAV_AUTOPILOT_UDB enum. UAV Dev Board
	MAV_AUTOPILOT_UDB MAV_AUTOPILOT = 10
	// MAV_AUTOPILOT_FP enum. FlexiPilot
	MAV_AUTOPILOT_FP MAV_AUTOPILOT = 11
	// MAV_AUTOPILOT_PX4 enum. PX4 Autopilot - http://px4.io/
	MAV_AUTOPILOT_PX4 MAV_AUTOPILOT = 12
	// MAV_AUTOPILOT_SMACCMPILOT enum. SMACCMPilot - http://smaccmpilot.org
	MAV_AUTOPILOT_SMACCMPILOT MAV_AUTOPILOT = 13
	// MAV_AUTOPILOT_AUTOQUAD enum. AutoQuad -- http://autoquad.org
	MAV_AUTOPILOT_AUTOQUAD MAV_AUTOPILOT = 14
	// MAV_AUTOPILOT_ARMAZILA enum. Armazila -- http://armazila.com
	MAV_AUTOPILOT_ARMAZILA MAV_AUTOPILOT = 15
	// MAV_AUTOPILOT_AEROB enum. Aerob -- http://aerob.ru
	MAV_AUTOPILOT_AEROB MAV_AUTOPILOT = 16
	// MAV_AUTOPILOT_ASLUAV enum. ASLUAV autopilot -- http://www.asl.ethz.ch
	MAV_AUTOPILOT_ASLUAV MAV_AUTOPILOT = 17
	// MAV_AUTOPILOT_SMARTAP enum. SmartAP Autopilot - http://sky-drones.com
	MAV_AUTOPILOT_SMARTAP MAV_AUTOPILOT = 18
	// MAV_AUTOPILOT_AIRRAILS enum. AirRails - http://uaventure.com
	MAV_AUTOPILOT_AIRRAILS MAV_AUTOPILOT = 19
)

// MAV_TYPE type. MAVLINK component type reported in HEARTBEAT message. Flight controllers must report the type of the vehicle on which they are mounted (e.g. MAV_TYPE_OCTOROTOR). All other components must report a value appropriate for their type (e.g. a camera must use MAV_TYPE_CAMERA).
type MAV_TYPE int

const (
	// MAV_TYPE_GENERIC enum. Generic micro air vehicle
	MAV_TYPE_GENERIC MAV_TYPE = 0
	// MAV_TYPE_FIXED_WING enum. Fixed wing aircraft
	MAV_TYPE_FIXED_WING MAV_TYPE = 1
	// MAV_TYPE_QUADROTOR enum. Quadrotor
	MAV_TYPE_QUADROTOR MAV_TYPE = 2
	// MAV_TYPE_COAXIAL enum. Coaxial helicopter
	MAV_TYPE_COAXIAL MAV_TYPE = 3
	// MAV_TYPE_HELICOPTER enum. Normal helicopter with tail rotor
	MAV_TYPE_HELICOPTER MAV_TYPE = 4
	// MAV_TYPE_ANTENNA_TRACKER enum. Ground installation
	MAV_TYPE_ANTENNA_TRACKER MAV_TYPE = 5
	// MAV_TYPE_GCS enum. Operator control unit / ground control station
	MAV_TYPE_GCS MAV_TYPE = 6
	// MAV_TYPE_AIRSHIP enum. Airship, controlled
	MAV_TYPE_AIRSHIP MAV_TYPE = 7
	// MAV_TYPE_FREE_BALLOON enum. Free balloon, uncontrolled
	MAV_TYPE_FREE_BALLOON MAV_TYPE = 8
	// MAV_TYPE_ROCKET enum. Rocket
	MAV_TYPE_ROCKET MAV_TYPE = 9
	// MAV_TYPE_GROUND_ROVER enum. Ground rover
	MAV_TYPE_GROUND_ROVER MAV_TYPE = 10
	// MAV_TYPE_SURFACE_BOAT enum. Surface vessel, boat, ship
	MAV_TYPE_SURFACE_BOAT MAV_TYPE = 11
	// MAV_TYPE_SUBMARINE enum. Submarine
	MAV_TYPE_SUBMARINE MAV_TYPE = 12
	// MAV_TYPE_HEXAROTOR enum. Hexarotor
	MAV_TYPE_HEXAROTOR MAV_TYPE = 13
	// MAV_TYPE_OCTOROTOR enum. Octorotor
	MAV_TYPE_OCTOROTOR MAV_TYPE = 14
	// MAV_TYPE_TRICOPTER enum. Tricopter
	MAV_TYPE_TRICOPTER MAV_TYPE = 15
	// MAV_TYPE_FLAPPING_WING enum. Flapping wing
	MAV_TYPE_FLAPPING_WING MAV_TYPE = 16
	// MAV_TYPE_KITE enum. Kite
	MAV_TYPE_KITE MAV_TYPE = 17
	// MAV_TYPE_ONBOARD_CONTROLLER enum. Onboard companion controller
	MAV_TYPE_ONBOARD_CONTROLLER MAV_TYPE = 18
	// MAV_TYPE_VTOL_DUOROTOR enum. Two-rotor VTOL using control surfaces in vertical operation in addition. Tailsitter
	MAV_TYPE_VTOL_DUOROTOR MAV_TYPE = 19
	// MAV_TYPE_VTOL_QUADROTOR enum. Quad-rotor VTOL using a V-shaped quad config in vertical operation. Tailsitter
	MAV_TYPE_VTOL_QUADROTOR MAV_TYPE = 20
	// MAV_TYPE_VTOL_TILTROTOR enum. Tiltrotor VTOL
	MAV_TYPE_VTOL_TILTROTOR MAV_TYPE = 21
	// MAV_TYPE_VTOL_RESERVED2 enum. VTOL reserved 2
	MAV_TYPE_VTOL_RESERVED2 MAV_TYPE = 22
	// MAV_TYPE_VTOL_RESERVED3 enum. VTOL reserved 3
	MAV_TYPE_VTOL_RESERVED3 MAV_TYPE = 23
	// MAV_TYPE_VTOL_RESERVED4 enum. VTOL reserved 4
	MAV_TYPE_VTOL_RESERVED4 MAV_TYPE = 24
	// MAV_TYPE_VTOL_RESERVED5 enum. VTOL reserved 5
	MAV_TYPE_VTOL_RESERVED5 MAV_TYPE = 25
	// MAV_TYPE_GIMBAL enum. Gimbal
	MAV_TYPE_GIMBAL MAV_TYPE = 26
	// MAV_TYPE_ADSB enum. ADSB system
	MAV_TYPE_ADSB MAV_TYPE = 27
	// MAV_TYPE_PARAFOIL enum. Steerable, nonrigid airfoil
	MAV_TYPE_PARAFOIL MAV_TYPE = 28
	// MAV_TYPE_DODECAROTOR enum. Dodecarotor
	MAV_TYPE_DODECAROTOR MAV_TYPE = 29
	// MAV_TYPE_CAMERA enum. Camera
	MAV_TYPE_CAMERA MAV_TYPE = 30
	// MAV_TYPE_CHARGING_STATION enum. Charging station
	MAV_TYPE_CHARGING_STATION MAV_TYPE = 31
	// MAV_TYPE_FLARM enum. FLARM collision avoidance system
	MAV_TYPE_FLARM MAV_TYPE = 32
	// MAV_TYPE_SERVO enum. Servo
	MAV_TYPE_SERVO MAV_TYPE = 33
	// MAV_TYPE_ODID enum. Open Drone ID. See https://mavlink.io/en/services/opendroneid.html
	MAV_TYPE_ODID MAV_TYPE = 34
	// MAV_TYPE_DECAROTOR enum. Decarotor
	MAV_TYPE_DECAROTOR MAV_TYPE = 35
)

// MAV_MODE_FLAG type. These flags encode the MAV mode.
type MAV_MODE_FLAG int

const (
	// MAV_MODE_FLAG_SAFETY_ARMED enum. 0b10000000 MAV safety set to armed. Motors are enabled / running / can start. Ready to fly. Additional note: this flag is to be ignore when sent in the command MAV_CMD_DO_SET_MODE and MAV_CMD_COMPONENT_ARM_DISARM shall be used instead. The flag can still be used to report the armed state
	MAV_MODE_FLAG_SAFETY_ARMED MAV_MODE_FLAG = 128
	// MAV_MODE_FLAG_MANUAL_INPUT_ENABLED enum. 0b01000000 remote control input is enabled
	MAV_MODE_FLAG_MANUAL_INPUT_ENABLED MAV_MODE_FLAG = 64
	// MAV_MODE_FLAG_HIL_ENABLED enum. 0b00100000 hardware in the loop simulation. All motors / actuators are blocked, but internal software is full operational
	MAV_MODE_FLAG_HIL_ENABLED MAV_MODE_FLAG = 32
	// MAV_MODE_FLAG_STABILIZE_ENABLED enum. 0b00010000 system stabilizes electronically its attitude (and optionally position). It needs however further control inputs to move around
	MAV_MODE_FLAG_STABILIZE_ENABLED MAV_MODE_FLAG = 16
	// MAV_MODE_FLAG_GUIDED_ENABLED enum. 0b00001000 guided mode enabled, system flies waypoints / mission items
	MAV_MODE_FLAG_GUIDED_ENABLED MAV_MODE_FLAG = 8
	// MAV_MODE_FLAG_AUTO_ENABLED enum. 0b00000100 autonomous mode enabled, system finds its own goal positions. Guided flag can be set or not, depends on the actual implementation
	MAV_MODE_FLAG_AUTO_ENABLED MAV_MODE_FLAG = 4
	// MAV_MODE_FLAG_TEST_ENABLED enum. 0b00000010 system has a test mode enabled. This flag is intended for temporary system tests and should not be used for stable implementations
	MAV_MODE_FLAG_TEST_ENABLED MAV_MODE_FLAG = 2
	// MAV_MODE_FLAG_CUSTOM_MODE_ENABLED enum. 0b00000001 Reserved for future use
	MAV_MODE_FLAG_CUSTOM_MODE_ENABLED MAV_MODE_FLAG = 1
)

// MAV_MODE_FLAG_DECODE_POSITION type. These values encode the bit positions of the decode position. These values can be used to read the value of a flag bit by combining the base_mode variable with AND with the flag position value. The result will be either 0 or 1, depending on if the flag is set or not.
type MAV_MODE_FLAG_DECODE_POSITION int

const (
	// MAV_MODE_FLAG_DECODE_POSITION_SAFETY enum. First bit:  10000000
	MAV_MODE_FLAG_DECODE_POSITION_SAFETY MAV_MODE_FLAG_DECODE_POSITION = 128
	// MAV_MODE_FLAG_DECODE_POSITION_MANUAL enum. Second bit: 01000000
	MAV_MODE_FLAG_DECODE_POSITION_MANUAL MAV_MODE_FLAG_DECODE_POSITION = 64
	// MAV_MODE_FLAG_DECODE_POSITION_HIL enum. Third bit:  00100000
	MAV_MODE_FLAG_DECODE_POSITION_HIL MAV_MODE_FLAG_DECODE_POSITION = 32
	// MAV_MODE_FLAG_DECODE_POSITION_STABILIZE enum. Fourth bit: 00010000
	MAV_MODE_FLAG_DECODE_POSITION_STABILIZE MAV_MODE_FLAG_DECODE_POSITION = 16
	// MAV_MODE_FLAG_DECODE_POSITION_GUIDED enum. Fifth bit:  00001000
	MAV_MODE_FLAG_DECODE_POSITION_GUIDED MAV_MODE_FLAG_DECODE_POSITION = 8
	// MAV_MODE_FLAG_DECODE_POSITION_AUTO enum. Sixth bit:   00000100
	MAV_MODE_FLAG_DECODE_POSITION_AUTO MAV_MODE_FLAG_DECODE_POSITION = 4
	// MAV_MODE_FLAG_DECODE_POSITION_TEST enum. Seventh bit: 00000010
	MAV_MODE_FLAG_DECODE_POSITION_TEST MAV_MODE_FLAG_DECODE_POSITION = 2
	// MAV_MODE_FLAG_DECODE_POSITION_CUSTOM_MODE enum. Eighth bit: 00000001
	MAV_MODE_FLAG_DECODE_POSITION_CUSTOM_MODE MAV_MODE_FLAG_DECODE_POSITION = 1
)

// MAV_STATE type
type MAV_STATE int

const (
	// MAV_STATE_UNINIT enum. Uninitialized system, state is unknown
	MAV_STATE_UNINIT MAV_STATE = 0
	// MAV_STATE_BOOT enum. System is booting up
	MAV_STATE_BOOT MAV_STATE = 1
	// MAV_STATE_CALIBRATING enum. System is calibrating and not flight-ready
	MAV_STATE_CALIBRATING MAV_STATE = 2
	// MAV_STATE_STANDBY enum. System is grounded and on standby. It can be launched any time
	MAV_STATE_STANDBY MAV_STATE = 3
	// MAV_STATE_ACTIVE enum. System is active and might be already airborne. Motors are engaged
	MAV_STATE_ACTIVE MAV_STATE = 4
	// MAV_STATE_CRITICAL enum. System is in a non-normal flight mode. It can however still navigate
	MAV_STATE_CRITICAL MAV_STATE = 5
	// MAV_STATE_EMERGENCY enum. System is in a non-normal flight mode. It lost control over parts or over the whole airframe. It is in mayday and going down
	MAV_STATE_EMERGENCY MAV_STATE = 6
	// MAV_STATE_POWEROFF enum. System just initialized its power-down sequence, will shut down now
	MAV_STATE_POWEROFF MAV_STATE = 7
	// MAV_STATE_FLIGHT_TERMINATION enum. System is terminating itself
	MAV_STATE_FLIGHT_TERMINATION MAV_STATE = 8
)

// MAV_COMPONENT type. Component ids (values) for the different types and instances of onboard hardware/software that might make up a MAVLink system (autopilot, cameras, servos, GPS systems, avoidance systems etc.).       Components must use the appropriate ID in their source address when sending messages. Components can also use IDs to determine if they are the intended recipient of an incoming message. The MAV_COMP_ID_ALL value is used to indicate messages that must be processed by all components.       When creating new entries, components that can have multiple instances (e.g. cameras, servos etc.) should be allocated sequential values. An appropriate number of values should be left free after these components to allow the number of instances to be expanded.
type MAV_COMPONENT int

const (
	// MAV_COMP_ID_ALL enum. Target id (target_component) used to broadcast messages to all components of the receiving system. Components should attempt to process messages with this component ID and forward to components on any other interfaces. Note: This is not a valid *source* component id for a message
	MAV_COMP_ID_ALL MAV_COMPONENT = 0
	// MAV_COMP_ID_AUTOPILOT1 enum. System flight controller component ("autopilot"). Only one autopilot is expected in a particular system
	MAV_COMP_ID_AUTOPILOT1 MAV_COMPONENT = 1
	// MAV_COMP_ID_USER1 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER1 MAV_COMPONENT = 25
	// MAV_COMP_ID_USER2 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER2 MAV_COMPONENT = 26
	// MAV_COMP_ID_USER3 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER3 MAV_COMPONENT = 27
	// MAV_COMP_ID_USER4 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER4 MAV_COMPONENT = 28
	// MAV_COMP_ID_USER5 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER5 MAV_COMPONENT = 29
	// MAV_COMP_ID_USER6 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER6 MAV_COMPONENT = 30
	// MAV_COMP_ID_USER7 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER7 MAV_COMPONENT = 31
	// MAV_COMP_ID_USER8 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER8 MAV_COMPONENT = 32
	// MAV_COMP_ID_USER9 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER9 MAV_COMPONENT = 33
	// MAV_COMP_ID_USER10 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER10 MAV_COMPONENT = 34
	// MAV_COMP_ID_USER11 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER11 MAV_COMPONENT = 35
	// MAV_COMP_ID_USER12 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER12 MAV_COMPONENT = 36
	// MAV_COMP_ID_USER13 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER13 MAV_COMPONENT = 37
	// MAV_COMP_ID_USER14 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER14 MAV_COMPONENT = 38
	// MAV_COMP_ID_USER15 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER15 MAV_COMPONENT = 39
	// MAV_COMP_ID_USER16 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER16 MAV_COMPONENT = 40
	// MAV_COMP_ID_USER17 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER17 MAV_COMPONENT = 41
	// MAV_COMP_ID_USER18 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER18 MAV_COMPONENT = 42
	// MAV_COMP_ID_USER19 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER19 MAV_COMPONENT = 43
	// MAV_COMP_ID_USER20 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER20 MAV_COMPONENT = 44
	// MAV_COMP_ID_USER21 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER21 MAV_COMPONENT = 45
	// MAV_COMP_ID_USER22 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER22 MAV_COMPONENT = 46
	// MAV_COMP_ID_USER23 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER23 MAV_COMPONENT = 47
	// MAV_COMP_ID_USER24 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER24 MAV_COMPONENT = 48
	// MAV_COMP_ID_USER25 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER25 MAV_COMPONENT = 49
	// MAV_COMP_ID_USER26 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER26 MAV_COMPONENT = 50
	// MAV_COMP_ID_USER27 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER27 MAV_COMPONENT = 51
	// MAV_COMP_ID_USER28 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER28 MAV_COMPONENT = 52
	// MAV_COMP_ID_USER29 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER29 MAV_COMPONENT = 53
	// MAV_COMP_ID_USER30 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER30 MAV_COMPONENT = 54
	// MAV_COMP_ID_USER31 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER31 MAV_COMPONENT = 55
	// MAV_COMP_ID_USER32 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER32 MAV_COMPONENT = 56
	// MAV_COMP_ID_USER33 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER33 MAV_COMPONENT = 57
	// MAV_COMP_ID_USER34 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER34 MAV_COMPONENT = 58
	// MAV_COMP_ID_USER35 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER35 MAV_COMPONENT = 59
	// MAV_COMP_ID_USER36 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER36 MAV_COMPONENT = 60
	// MAV_COMP_ID_USER37 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER37 MAV_COMPONENT = 61
	// MAV_COMP_ID_USER38 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER38 MAV_COMPONENT = 62
	// MAV_COMP_ID_USER39 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER39 MAV_COMPONENT = 63
	// MAV_COMP_ID_USER40 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER40 MAV_COMPONENT = 64
	// MAV_COMP_ID_USER41 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER41 MAV_COMPONENT = 65
	// MAV_COMP_ID_USER42 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER42 MAV_COMPONENT = 66
	// MAV_COMP_ID_USER43 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER43 MAV_COMPONENT = 67
	// MAV_COMP_ID_TELEMETRY_RADIO enum. Telemetry radio (e.g. SiK radio, or other component that emits RADIO_STATUS messages)
	MAV_COMP_ID_TELEMETRY_RADIO MAV_COMPONENT = 68
	// MAV_COMP_ID_USER45 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER45 MAV_COMPONENT = 69
	// MAV_COMP_ID_USER46 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER46 MAV_COMPONENT = 70
	// MAV_COMP_ID_USER47 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER47 MAV_COMPONENT = 71
	// MAV_COMP_ID_USER48 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER48 MAV_COMPONENT = 72
	// MAV_COMP_ID_USER49 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER49 MAV_COMPONENT = 73
	// MAV_COMP_ID_USER50 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER50 MAV_COMPONENT = 74
	// MAV_COMP_ID_USER51 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER51 MAV_COMPONENT = 75
	// MAV_COMP_ID_USER52 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER52 MAV_COMPONENT = 76
	// MAV_COMP_ID_USER53 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER53 MAV_COMPONENT = 77
	// MAV_COMP_ID_USER54 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER54 MAV_COMPONENT = 78
	// MAV_COMP_ID_USER55 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER55 MAV_COMPONENT = 79
	// MAV_COMP_ID_USER56 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER56 MAV_COMPONENT = 80
	// MAV_COMP_ID_USER57 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER57 MAV_COMPONENT = 81
	// MAV_COMP_ID_USER58 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER58 MAV_COMPONENT = 82
	// MAV_COMP_ID_USER59 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER59 MAV_COMPONENT = 83
	// MAV_COMP_ID_USER60 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER60 MAV_COMPONENT = 84
	// MAV_COMP_ID_USER61 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER61 MAV_COMPONENT = 85
	// MAV_COMP_ID_USER62 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER62 MAV_COMPONENT = 86
	// MAV_COMP_ID_USER63 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER63 MAV_COMPONENT = 87
	// MAV_COMP_ID_USER64 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER64 MAV_COMPONENT = 88
	// MAV_COMP_ID_USER65 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER65 MAV_COMPONENT = 89
	// MAV_COMP_ID_USER66 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER66 MAV_COMPONENT = 90
	// MAV_COMP_ID_USER67 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER67 MAV_COMPONENT = 91
	// MAV_COMP_ID_USER68 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER68 MAV_COMPONENT = 92
	// MAV_COMP_ID_USER69 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER69 MAV_COMPONENT = 93
	// MAV_COMP_ID_USER70 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER70 MAV_COMPONENT = 94
	// MAV_COMP_ID_USER71 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER71 MAV_COMPONENT = 95
	// MAV_COMP_ID_USER72 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER72 MAV_COMPONENT = 96
	// MAV_COMP_ID_USER73 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER73 MAV_COMPONENT = 97
	// MAV_COMP_ID_USER74 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER74 MAV_COMPONENT = 98
	// MAV_COMP_ID_USER75 enum. Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER75 MAV_COMPONENT = 99
	// MAV_COMP_ID_CAMERA enum. Camera #1
	MAV_COMP_ID_CAMERA MAV_COMPONENT = 100
	// MAV_COMP_ID_CAMERA2 enum. Camera #2
	MAV_COMP_ID_CAMERA2 MAV_COMPONENT = 101
	// MAV_COMP_ID_CAMERA3 enum. Camera #3
	MAV_COMP_ID_CAMERA3 MAV_COMPONENT = 102
	// MAV_COMP_ID_CAMERA4 enum. Camera #4
	MAV_COMP_ID_CAMERA4 MAV_COMPONENT = 103
	// MAV_COMP_ID_CAMERA5 enum. Camera #5
	MAV_COMP_ID_CAMERA5 MAV_COMPONENT = 104
	// MAV_COMP_ID_CAMERA6 enum. Camera #6
	MAV_COMP_ID_CAMERA6 MAV_COMPONENT = 105
	// MAV_COMP_ID_SERVO1 enum. Servo #1
	MAV_COMP_ID_SERVO1 MAV_COMPONENT = 140
	// MAV_COMP_ID_SERVO2 enum. Servo #2
	MAV_COMP_ID_SERVO2 MAV_COMPONENT = 141
	// MAV_COMP_ID_SERVO3 enum. Servo #3
	MAV_COMP_ID_SERVO3 MAV_COMPONENT = 142
	// MAV_COMP_ID_SERVO4 enum. Servo #4
	MAV_COMP_ID_SERVO4 MAV_COMPONENT = 143
	// MAV_COMP_ID_SERVO5 enum. Servo #5
	MAV_COMP_ID_SERVO5 MAV_COMPONENT = 144
	// MAV_COMP_ID_SERVO6 enum. Servo #6
	MAV_COMP_ID_SERVO6 MAV_COMPONENT = 145
	// MAV_COMP_ID_SERVO7 enum. Servo #7
	MAV_COMP_ID_SERVO7 MAV_COMPONENT = 146
	// MAV_COMP_ID_SERVO8 enum. Servo #8
	MAV_COMP_ID_SERVO8 MAV_COMPONENT = 147
	// MAV_COMP_ID_SERVO9 enum. Servo #9
	MAV_COMP_ID_SERVO9 MAV_COMPONENT = 148
	// MAV_COMP_ID_SERVO10 enum. Servo #10
	MAV_COMP_ID_SERVO10 MAV_COMPONENT = 149
	// MAV_COMP_ID_SERVO11 enum. Servo #11
	MAV_COMP_ID_SERVO11 MAV_COMPONENT = 150
	// MAV_COMP_ID_SERVO12 enum. Servo #12
	MAV_COMP_ID_SERVO12 MAV_COMPONENT = 151
	// MAV_COMP_ID_SERVO13 enum. Servo #13
	MAV_COMP_ID_SERVO13 MAV_COMPONENT = 152
	// MAV_COMP_ID_SERVO14 enum. Servo #14
	MAV_COMP_ID_SERVO14 MAV_COMPONENT = 153
	// MAV_COMP_ID_GIMBAL enum. Gimbal #1
	MAV_COMP_ID_GIMBAL MAV_COMPONENT = 154
	// MAV_COMP_ID_LOG enum. Logging component
	MAV_COMP_ID_LOG MAV_COMPONENT = 155
	// MAV_COMP_ID_ADSB enum. Automatic Dependent Surveillance-Broadcast (ADS-B) component
	MAV_COMP_ID_ADSB MAV_COMPONENT = 156
	// MAV_COMP_ID_OSD enum. On Screen Display (OSD) devices for video links
	MAV_COMP_ID_OSD MAV_COMPONENT = 157
	// MAV_COMP_ID_PERIPHERAL enum. Generic autopilot peripheral component ID. Meant for devices that do not implement the parameter microservice
	MAV_COMP_ID_PERIPHERAL MAV_COMPONENT = 158
	// MAV_COMP_ID_QX1_GIMBAL enum. Gimbal ID for QX1
	MAV_COMP_ID_QX1_GIMBAL MAV_COMPONENT = 159
	// MAV_COMP_ID_FLARM enum. FLARM collision alert component
	MAV_COMP_ID_FLARM MAV_COMPONENT = 160
	// MAV_COMP_ID_GIMBAL2 enum. Gimbal #2
	MAV_COMP_ID_GIMBAL2 MAV_COMPONENT = 171
	// MAV_COMP_ID_GIMBAL3 enum. Gimbal #3
	MAV_COMP_ID_GIMBAL3 MAV_COMPONENT = 172
	// MAV_COMP_ID_GIMBAL4 enum. Gimbal #4
	MAV_COMP_ID_GIMBAL4 MAV_COMPONENT = 173
	// MAV_COMP_ID_GIMBAL5 enum. Gimbal #5
	MAV_COMP_ID_GIMBAL5 MAV_COMPONENT = 174
	// MAV_COMP_ID_GIMBAL6 enum. Gimbal #6
	MAV_COMP_ID_GIMBAL6 MAV_COMPONENT = 175
	// MAV_COMP_ID_MISSIONPLANNER enum. Component that can generate/supply a mission flight plan (e.g. GCS or developer API)
	MAV_COMP_ID_MISSIONPLANNER MAV_COMPONENT = 190
	// MAV_COMP_ID_ONBOARD_COMPUTER enum. Component that lives on the onboard computer (companion computer) and has some generic functionalities, such as settings system parameters and monitoring the status of some processes that don't directly speak mavlink and so on
	MAV_COMP_ID_ONBOARD_COMPUTER MAV_COMPONENT = 191
	// MAV_COMP_ID_PATHPLANNER enum. Component that finds an optimal path between points based on a certain constraint (e.g. minimum snap, shortest path, cost, etc.)
	MAV_COMP_ID_PATHPLANNER MAV_COMPONENT = 195
	// MAV_COMP_ID_OBSTACLE_AVOIDANCE enum. Component that plans a collision free path between two points
	MAV_COMP_ID_OBSTACLE_AVOIDANCE MAV_COMPONENT = 196
	// MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY enum. Component that provides position estimates using VIO techniques
	MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY MAV_COMPONENT = 197
	// MAV_COMP_ID_PAIRING_MANAGER enum. Component that manages pairing of vehicle and GCS
	MAV_COMP_ID_PAIRING_MANAGER MAV_COMPONENT = 198
	// MAV_COMP_ID_IMU enum. Inertial Measurement Unit (IMU) #1
	MAV_COMP_ID_IMU MAV_COMPONENT = 200
	// MAV_COMP_ID_IMU_2 enum. Inertial Measurement Unit (IMU) #2
	MAV_COMP_ID_IMU_2 MAV_COMPONENT = 201
	// MAV_COMP_ID_IMU_3 enum. Inertial Measurement Unit (IMU) #3
	MAV_COMP_ID_IMU_3 MAV_COMPONENT = 202
	// MAV_COMP_ID_GPS enum. GPS #1
	MAV_COMP_ID_GPS MAV_COMPONENT = 220
	// MAV_COMP_ID_GPS2 enum. GPS #2
	MAV_COMP_ID_GPS2 MAV_COMPONENT = 221
	// MAV_COMP_ID_ODID_TXRX_1 enum. Open Drone ID transmitter/receiver (Bluetooth/WiFi/Internet)
	MAV_COMP_ID_ODID_TXRX_1 MAV_COMPONENT = 236
	// MAV_COMP_ID_ODID_TXRX_2 enum. Open Drone ID transmitter/receiver (Bluetooth/WiFi/Internet)
	MAV_COMP_ID_ODID_TXRX_2 MAV_COMPONENT = 237
	// MAV_COMP_ID_ODID_TXRX_3 enum. Open Drone ID transmitter/receiver (Bluetooth/WiFi/Internet)
	MAV_COMP_ID_ODID_TXRX_3 MAV_COMPONENT = 238
	// MAV_COMP_ID_UDP_BRIDGE enum. Component to bridge MAVLink to UDP (i.e. from a UART)
	MAV_COMP_ID_UDP_BRIDGE MAV_COMPONENT = 240
	// MAV_COMP_ID_UART_BRIDGE enum. Component to bridge to UART (i.e. from UDP)
	MAV_COMP_ID_UART_BRIDGE MAV_COMPONENT = 241
	// MAV_COMP_ID_TUNNEL_NODE enum. Component handling TUNNEL messages (e.g. vendor specific GUI of a component)
	MAV_COMP_ID_TUNNEL_NODE MAV_COMPONENT = 242
	// MAV_COMP_ID_SYSTEM_CONTROL enum. Component for handling system messages (e.g. to ARM, takeoff, etc.)
	MAV_COMP_ID_SYSTEM_CONTROL MAV_COMPONENT = 250
)

// NavFilterBias struct (generated typeinfo)
// Accelerometer and Gyro biases from the navigation filter
type NavFilterBias struct {
	Usec   uint64  // Timestamp (microseconds)
	Accel0 float32 // b_f[0]
	Accel1 float32 // b_f[1]
	Accel2 float32 // b_f[2]
	Gyro0  float32 // b_f[0]
	Gyro1  float32 // b_f[1]
	Gyro2  float32 // b_f[2]
}

// MsgID (generated function)
func (m *NavFilterBias) MsgID() mavlink.MessageID {
	return MSG_ID_NAV_FILTER_BIAS
}

// String (generated function)
func (m *NavFilterBias) String() string {
	return fmt.Sprintf(
		"&ualberta.NavFilterBias{ Usec: %+v, Accel0: %+v, Accel1: %+v, Accel2: %+v, Gyro0: %+v, Gyro1: %+v, Gyro2: %+v }",
		m.Usec,
		m.Accel0,
		m.Accel1,
		m.Accel2,
		m.Gyro0,
		m.Gyro1,
		m.Gyro2,
	)
}

// Pack (generated function)
func (m *NavFilterBias) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 32)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.Usec))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.Accel0))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.Accel1))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(m.Accel2))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(m.Gyro0))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(m.Gyro1))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(m.Gyro2))
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *NavFilterBias) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 32 {
		payload = append(payload, mavlink.ZeroTail[:32-len(p.Payload)]...)
	}
	m.Usec = uint64(binary.LittleEndian.Uint64(payload[0:]))
	m.Accel0 = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.Accel1 = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.Accel2 = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	m.Gyro0 = math.Float32frombits(binary.LittleEndian.Uint32(payload[20:]))
	m.Gyro1 = math.Float32frombits(binary.LittleEndian.Uint32(payload[24:]))
	m.Gyro2 = math.Float32frombits(binary.LittleEndian.Uint32(payload[28:]))
	return nil
}

// RadioCalibration struct (generated typeinfo)
// Complete set of calibration parameters for the radio
type RadioCalibration struct {
	Aileron  [3]uint16 // Aileron setpoints: left, center, right
	Elevator [3]uint16 // Elevator setpoints: nose down, center, nose up
	Rudder   [3]uint16 // Rudder setpoints: nose left, center, nose right
	Gyro     [2]uint16 // Tail gyro mode/gain setpoints: heading hold, rate mode
	Pitch    [5]uint16 // Pitch curve setpoints (every 25%)
	Throttle [5]uint16 // Throttle curve setpoints (every 25%)
}

// MsgID (generated function)
func (m *RadioCalibration) MsgID() mavlink.MessageID {
	return MSG_ID_RADIO_CALIBRATION
}

// String (generated function)
func (m *RadioCalibration) String() string {
	return fmt.Sprintf(
		"&ualberta.RadioCalibration{ Aileron: %+v, Elevator: %+v, Rudder: %+v, Gyro: %+v, Pitch: %+v, Throttle: %+v }",
		m.Aileron,
		m.Elevator,
		m.Rudder,
		m.Gyro,
		m.Pitch,
		m.Throttle,
	)
}

// Pack (generated function)
func (m *RadioCalibration) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 42)
	for i, v := range m.Aileron {
		binary.LittleEndian.PutUint16(payload[0+i*2:], uint16(v))
	}
	for i, v := range m.Elevator {
		binary.LittleEndian.PutUint16(payload[6+i*2:], uint16(v))
	}
	for i, v := range m.Rudder {
		binary.LittleEndian.PutUint16(payload[12+i*2:], uint16(v))
	}
	for i, v := range m.Gyro {
		binary.LittleEndian.PutUint16(payload[18+i*2:], uint16(v))
	}
	for i, v := range m.Pitch {
		binary.LittleEndian.PutUint16(payload[22+i*2:], uint16(v))
	}
	for i, v := range m.Throttle {
		binary.LittleEndian.PutUint16(payload[32+i*2:], uint16(v))
	}
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *RadioCalibration) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 42 {
		payload = append(payload, mavlink.ZeroTail[:42-len(p.Payload)]...)
	}
	for i := 0; i < len(m.Aileron); i++ {
		m.Aileron[i] = uint16(binary.LittleEndian.Uint16(payload[0+i*2:]))
	}
	for i := 0; i < len(m.Elevator); i++ {
		m.Elevator[i] = uint16(binary.LittleEndian.Uint16(payload[6+i*2:]))
	}
	for i := 0; i < len(m.Rudder); i++ {
		m.Rudder[i] = uint16(binary.LittleEndian.Uint16(payload[12+i*2:]))
	}
	for i := 0; i < len(m.Gyro); i++ {
		m.Gyro[i] = uint16(binary.LittleEndian.Uint16(payload[18+i*2:]))
	}
	for i := 0; i < len(m.Pitch); i++ {
		m.Pitch[i] = uint16(binary.LittleEndian.Uint16(payload[22+i*2:]))
	}
	for i := 0; i < len(m.Throttle); i++ {
		m.Throttle[i] = uint16(binary.LittleEndian.Uint16(payload[32+i*2:]))
	}
	return nil
}

// UalbertaSysStatus struct (generated typeinfo)
// System status specific to ualberta uav
type UalbertaSysStatus struct {
	Mode    uint8 // System mode, see UALBERTA_AUTOPILOT_MODE ENUM
	NavMode uint8 // Navigation mode, see UALBERTA_NAV_MODE ENUM
	Pilot   uint8 // Pilot mode, see UALBERTA_PILOT_MODE
}

// MsgID (generated function)
func (m *UalbertaSysStatus) MsgID() mavlink.MessageID {
	return MSG_ID_UALBERTA_SYS_STATUS
}

// String (generated function)
func (m *UalbertaSysStatus) String() string {
	return fmt.Sprintf(
		"&ualberta.UalbertaSysStatus{ Mode: %+v, NavMode: %+v, Pilot: %+v }",
		m.Mode,
		m.NavMode,
		m.Pilot,
	)
}

// Pack (generated function)
func (m *UalbertaSysStatus) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 3)
	payload[0] = byte(m.Mode)
	payload[1] = byte(m.NavMode)
	payload[2] = byte(m.Pilot)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *UalbertaSysStatus) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 3 {
		payload = append(payload, mavlink.ZeroTail[:3-len(p.Payload)]...)
	}
	m.Mode = uint8(payload[0])
	m.NavMode = uint8(payload[1])
	m.Pilot = uint8(payload[2])
	return nil
}

// SysStatus struct (generated typeinfo)
// The general system state. If the system is following the MAVLink standard, the system state is mainly defined by three orthogonal states/modes: The system mode, which is either LOCKED (motors shut down and locked), MANUAL (system under RC control), GUIDED (system with autonomous position control, position setpoint controlled manually) or AUTO (system guided by path/waypoint planner). The NAV_MODE defined the current flight state: LIFTOFF (often an open-loop maneuver), LANDING, WAYPOINTS or VECTOR. This represents the internal navigation state machine. The system status shows whether the system is currently active or not and if an emergency occurred. During the CRITICAL and EMERGENCY states the MAV is still considered to be active, but should start emergency procedures autonomously. After a failure occurred it should first move from active to critical to allow manual intervention and then move to emergency after a certain timeout.
type SysStatus struct {
	OnboardControlSensorsPresent MAV_SYS_STATUS_SENSOR `gotype:"uint32"` // Bitmap showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present.
	OnboardControlSensorsEnabled MAV_SYS_STATUS_SENSOR `gotype:"uint32"` // Bitmap showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of 1: enabled.
	OnboardControlSensorsHealth  MAV_SYS_STATUS_SENSOR `gotype:"uint32"` // Bitmap showing which onboard controllers and sensors have an error (or are operational). Value of 0: error. Value of 1: healthy.
	Load                         uint16                // Maximum usage in percent of the mainloop time. Values: [0-1000] - should always be below 1000
	VoltageBattery               uint16                // Battery voltage, UINT16_MAX: Voltage not sent by autopilot
	CurrentBattery               int16                 // Battery current, -1: Current not sent by autopilot
	DropRateComm                 uint16                // Communication drop rate, (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
	ErrorsComm                   uint16                // Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
	ErrorsCount1                 uint16                // Autopilot-specific errors
	ErrorsCount2                 uint16                // Autopilot-specific errors
	ErrorsCount3                 uint16                // Autopilot-specific errors
	ErrorsCount4                 uint16                // Autopilot-specific errors
	BatteryRemaining             int8                  // Battery energy remaining, -1: Battery remaining energy not sent by autopilot
}

// MsgID (generated function)
func (m *SysStatus) MsgID() mavlink.MessageID {
	return MSG_ID_SYS_STATUS
}

// String (generated function)
func (m *SysStatus) String() string {
	return fmt.Sprintf(
		"&common.SysStatus{ OnboardControlSensorsPresent: %+v, OnboardControlSensorsEnabled: %+v, OnboardControlSensorsHealth: %+v, Load: %+v, VoltageBattery: %+v, CurrentBattery: %+v, DropRateComm: %+v, ErrorsComm: %+v, ErrorsCount1: %+v, ErrorsCount2: %+v, ErrorsCount3: %+v, ErrorsCount4: %+v, BatteryRemaining: %+v }",
		m.OnboardControlSensorsPresent,
		m.OnboardControlSensorsEnabled,
		m.OnboardControlSensorsHealth,
		m.Load,
		m.VoltageBattery,
		m.CurrentBattery,
		m.DropRateComm,
		m.ErrorsComm,
		m.ErrorsCount1,
		m.ErrorsCount2,
		m.ErrorsCount3,
		m.ErrorsCount4,
		m.BatteryRemaining,
	)
}

// Pack (generated function)
func (m *SysStatus) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 31)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.OnboardControlSensorsPresent))
	binary.LittleEndian.PutUint32(payload[4:], uint32(m.OnboardControlSensorsEnabled))
	binary.LittleEndian.PutUint32(payload[8:], uint32(m.OnboardControlSensorsHealth))
	binary.LittleEndian.PutUint16(payload[12:], uint16(m.Load))
	binary.LittleEndian.PutUint16(payload[14:], uint16(m.VoltageBattery))
	binary.LittleEndian.PutUint16(payload[16:], uint16(m.CurrentBattery))
	binary.LittleEndian.PutUint16(payload[18:], uint16(m.DropRateComm))
	binary.LittleEndian.PutUint16(payload[20:], uint16(m.ErrorsComm))
	binary.LittleEndian.PutUint16(payload[22:], uint16(m.ErrorsCount1))
	binary.LittleEndian.PutUint16(payload[24:], uint16(m.ErrorsCount2))
	binary.LittleEndian.PutUint16(payload[26:], uint16(m.ErrorsCount3))
	binary.LittleEndian.PutUint16(payload[28:], uint16(m.ErrorsCount4))
	payload[30] = byte(m.BatteryRemaining)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *SysStatus) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 31 {
		payload = append(payload, mavlink.ZeroTail[:31-len(p.Payload)]...)
	}
	m.OnboardControlSensorsPresent = MAV_SYS_STATUS_SENSOR(binary.LittleEndian.Uint32(payload[0:]))
	m.OnboardControlSensorsEnabled = MAV_SYS_STATUS_SENSOR(binary.LittleEndian.Uint32(payload[4:]))
	m.OnboardControlSensorsHealth = MAV_SYS_STATUS_SENSOR(binary.LittleEndian.Uint32(payload[8:]))
	m.Load = uint16(binary.LittleEndian.Uint16(payload[12:]))
	m.VoltageBattery = uint16(binary.LittleEndian.Uint16(payload[14:]))
	m.CurrentBattery = int16(binary.LittleEndian.Uint16(payload[16:]))
	m.DropRateComm = uint16(binary.LittleEndian.Uint16(payload[18:]))
	m.ErrorsComm = uint16(binary.LittleEndian.Uint16(payload[20:]))
	m.ErrorsCount1 = uint16(binary.LittleEndian.Uint16(payload[22:]))
	m.ErrorsCount2 = uint16(binary.LittleEndian.Uint16(payload[24:]))
	m.ErrorsCount3 = uint16(binary.LittleEndian.Uint16(payload[26:]))
	m.ErrorsCount4 = uint16(binary.LittleEndian.Uint16(payload[28:]))
	m.BatteryRemaining = int8(payload[30])
	return nil
}

// SystemTime struct (generated typeinfo)
// The system time is the time of the master clock, typically the computer clock of the main onboard computer.
type SystemTime struct {
	TimeUnixUsec uint64 // Timestamp (UNIX epoch time).
	TimeBootMs   uint32 // Timestamp (time since system boot).
}

// MsgID (generated function)
func (m *SystemTime) MsgID() mavlink.MessageID {
	return MSG_ID_SYSTEM_TIME
}

// String (generated function)
func (m *SystemTime) String() string {
	return fmt.Sprintf(
		"&common.SystemTime{ TimeUnixUsec: %+v, TimeBootMs: %+v }",
		m.TimeUnixUsec,
		m.TimeBootMs,
	)
}

// Pack (generated function)
func (m *SystemTime) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 12)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.TimeUnixUsec))
	binary.LittleEndian.PutUint32(payload[8:], uint32(m.TimeBootMs))
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *SystemTime) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 12 {
		payload = append(payload, mavlink.ZeroTail[:12-len(p.Payload)]...)
	}
	m.TimeUnixUsec = uint64(binary.LittleEndian.Uint64(payload[0:]))
	m.TimeBootMs = uint32(binary.LittleEndian.Uint32(payload[8:]))
	return nil
}

// Ping struct (generated typeinfo)
// A ping message either requesting or responding to a ping. This allows to measure the system latencies, including serial port, radio modem and UDP connections. The ping microservice is documented at https://mavlink.io/en/services/ping.html
type Ping struct {
	TimeUsec        uint64 // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
	Seq             uint32 // PING sequence
	TargetSystem    uint8  // 0: request ping from all receiving systems. If greater than 0: message is a ping response and number is the system id of the requesting system
	TargetComponent uint8  // 0: request ping from all receiving components. If greater than 0: message is a ping response and number is the component id of the requesting component.
}

// MsgID (generated function)
func (m *Ping) MsgID() mavlink.MessageID {
	return MSG_ID_PING
}

// String (generated function)
func (m *Ping) String() string {
	return fmt.Sprintf(
		"&common.Ping{ TimeUsec: %+v, Seq: %+v, TargetSystem: %+v, TargetComponent: %+v }",
		m.TimeUsec,
		m.Seq,
		m.TargetSystem,
		m.TargetComponent,
	)
}

// Pack (generated function)
func (m *Ping) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 14)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.TimeUsec))
	binary.LittleEndian.PutUint32(payload[8:], uint32(m.Seq))
	payload[12] = byte(m.TargetSystem)
	payload[13] = byte(m.TargetComponent)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *Ping) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 14 {
		payload = append(payload, mavlink.ZeroTail[:14-len(p.Payload)]...)
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(payload[0:]))
	m.Seq = uint32(binary.LittleEndian.Uint32(payload[8:]))
	m.TargetSystem = uint8(payload[12])
	m.TargetComponent = uint8(payload[13])
	return nil
}

// ChangeOperatorControl struct (generated typeinfo)
// Request to control this MAV
type ChangeOperatorControl struct {
	TargetSystem   uint8    // System the GCS requests control for
	ControlRequest uint8    // 0: request control of this MAV, 1: Release control of this MAV
	Version        uint8    // 0: key as plaintext, 1-255: future, different hashing/encryption variants. The GCS should in general use the safest mode possible initially and then gradually move down the encryption level if it gets a NACK message indicating an encryption mismatch.
	Passkey        [25]byte // Password / Key, depending on version plaintext or encrypted. 25 or less characters, NULL terminated. The characters may involve A-Z, a-z, 0-9, and "!?,.-"
}

// MsgID (generated function)
func (m *ChangeOperatorControl) MsgID() mavlink.MessageID {
	return MSG_ID_CHANGE_OPERATOR_CONTROL
}

// String (generated function)
func (m *ChangeOperatorControl) String() string {
	return fmt.Sprintf(
		"&common.ChangeOperatorControl{ TargetSystem: %+v, ControlRequest: %+v, Version: %+v, Passkey: %0X (\"%s\") }",
		m.TargetSystem,
		m.ControlRequest,
		m.Version,
		m.Passkey, string(m.Passkey[:]),
	)
}

// Pack (generated function)
func (m *ChangeOperatorControl) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 28)
	payload[0] = byte(m.TargetSystem)
	payload[1] = byte(m.ControlRequest)
	payload[2] = byte(m.Version)
	copy(payload[3:], m.Passkey[:])
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ChangeOperatorControl) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 28 {
		payload = append(payload, mavlink.ZeroTail[:28-len(p.Payload)]...)
	}
	m.TargetSystem = uint8(payload[0])
	m.ControlRequest = uint8(payload[1])
	m.Version = uint8(payload[2])
	copy(m.Passkey[:], payload[3:28])
	return nil
}

// ChangeOperatorControlAck struct (generated typeinfo)
// Accept / deny control of this MAV
type ChangeOperatorControlAck struct {
	GcsSystemID    uint8 // ID of the GCS this message
	ControlRequest uint8 // 0: request control of this MAV, 1: Release control of this MAV
	Ack            uint8 // 0: ACK, 1: NACK: Wrong passkey, 2: NACK: Unsupported passkey encryption method, 3: NACK: Already under control
}

// MsgID (generated function)
func (m *ChangeOperatorControlAck) MsgID() mavlink.MessageID {
	return MSG_ID_CHANGE_OPERATOR_CONTROL_ACK
}

// String (generated function)
func (m *ChangeOperatorControlAck) String() string {
	return fmt.Sprintf(
		"&common.ChangeOperatorControlAck{ GcsSystemID: %+v, ControlRequest: %+v, Ack: %+v }",
		m.GcsSystemID,
		m.ControlRequest,
		m.Ack,
	)
}

// Pack (generated function)
func (m *ChangeOperatorControlAck) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 3)
	payload[0] = byte(m.GcsSystemID)
	payload[1] = byte(m.ControlRequest)
	payload[2] = byte(m.Ack)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ChangeOperatorControlAck) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 3 {
		payload = append(payload, mavlink.ZeroTail[:3-len(p.Payload)]...)
	}
	m.GcsSystemID = uint8(payload[0])
	m.ControlRequest = uint8(payload[1])
	m.Ack = uint8(payload[2])
	return nil
}

// AuthKey struct (generated typeinfo)
// Emit an encrypted signature / key identifying this system. PLEASE NOTE: This protocol has been kept simple, so transmitting the key requires an encrypted channel for true safety.
type AuthKey struct {
	Key [32]byte // key
}

// MsgID (generated function)
func (m *AuthKey) MsgID() mavlink.MessageID {
	return MSG_ID_AUTH_KEY
}

// String (generated function)
func (m *AuthKey) String() string {
	return fmt.Sprintf(
		"&common.AuthKey{ Key: %0X (\"%s\") }",
		m.Key, string(m.Key[:]),
	)
}

// Pack (generated function)
func (m *AuthKey) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 32)
	copy(payload[0:], m.Key[:])
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *AuthKey) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 32 {
		payload = append(payload, mavlink.ZeroTail[:32-len(p.Payload)]...)
	}
	copy(m.Key[:], payload[0:32])
	return nil
}

// LinkNodeStatus struct (generated typeinfo)
// Status generated in each node in the communication chain and injected into MAVLink stream.
type LinkNodeStatus struct {
	Timestamp        uint64 // Timestamp (time since system boot).
	TxRate           uint32 // Transmit rate
	RxRate           uint32 // Receive rate
	MessagesSent     uint32 // Messages sent
	MessagesReceived uint32 // Messages received (estimated from counting seq)
	MessagesLost     uint32 // Messages lost (estimated from counting seq)
	RxParseErr       uint16 // Number of bytes that could not be parsed correctly.
	TxOverflows      uint16 // Transmit buffer overflows. This number wraps around as it reaches UINT16_MAX
	RxOverflows      uint16 // Receive buffer overflows. This number wraps around as it reaches UINT16_MAX
	TxBuf            uint8  // Remaining free transmit buffer space
	RxBuf            uint8  // Remaining free receive buffer space
}

// MsgID (generated function)
func (m *LinkNodeStatus) MsgID() mavlink.MessageID {
	return MSG_ID_LINK_NODE_STATUS
}

// String (generated function)
func (m *LinkNodeStatus) String() string {
	return fmt.Sprintf(
		"&common.LinkNodeStatus{ Timestamp: %+v, TxRate: %+v, RxRate: %+v, MessagesSent: %+v, MessagesReceived: %+v, MessagesLost: %+v, RxParseErr: %+v, TxOverflows: %+v, RxOverflows: %+v, TxBuf: %+v, RxBuf: %+v }",
		m.Timestamp,
		m.TxRate,
		m.RxRate,
		m.MessagesSent,
		m.MessagesReceived,
		m.MessagesLost,
		m.RxParseErr,
		m.TxOverflows,
		m.RxOverflows,
		m.TxBuf,
		m.RxBuf,
	)
}

// Pack (generated function)
func (m *LinkNodeStatus) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 36)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.Timestamp))
	binary.LittleEndian.PutUint32(payload[8:], uint32(m.TxRate))
	binary.LittleEndian.PutUint32(payload[12:], uint32(m.RxRate))
	binary.LittleEndian.PutUint32(payload[16:], uint32(m.MessagesSent))
	binary.LittleEndian.PutUint32(payload[20:], uint32(m.MessagesReceived))
	binary.LittleEndian.PutUint32(payload[24:], uint32(m.MessagesLost))
	binary.LittleEndian.PutUint16(payload[28:], uint16(m.RxParseErr))
	binary.LittleEndian.PutUint16(payload[30:], uint16(m.TxOverflows))
	binary.LittleEndian.PutUint16(payload[32:], uint16(m.RxOverflows))
	payload[34] = byte(m.TxBuf)
	payload[35] = byte(m.RxBuf)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *LinkNodeStatus) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 36 {
		payload = append(payload, mavlink.ZeroTail[:36-len(p.Payload)]...)
	}
	m.Timestamp = uint64(binary.LittleEndian.Uint64(payload[0:]))
	m.TxRate = uint32(binary.LittleEndian.Uint32(payload[8:]))
	m.RxRate = uint32(binary.LittleEndian.Uint32(payload[12:]))
	m.MessagesSent = uint32(binary.LittleEndian.Uint32(payload[16:]))
	m.MessagesReceived = uint32(binary.LittleEndian.Uint32(payload[20:]))
	m.MessagesLost = uint32(binary.LittleEndian.Uint32(payload[24:]))
	m.RxParseErr = uint16(binary.LittleEndian.Uint16(payload[28:]))
	m.TxOverflows = uint16(binary.LittleEndian.Uint16(payload[30:]))
	m.RxOverflows = uint16(binary.LittleEndian.Uint16(payload[32:]))
	m.TxBuf = uint8(payload[34])
	m.RxBuf = uint8(payload[35])
	return nil
}

// SetMode struct (generated typeinfo)
// Set the system mode, as defined by enum MAV_MODE. There is no target component id as the mode is by definition for the overall aircraft, not only for one component.
type SetMode struct {
	CustomMode   uint32   // The new autopilot-specific mode. This field can be ignored by an autopilot.
	TargetSystem uint8    // The system setting the mode
	BaseMode     MAV_MODE `gotype:"uint8"` // The new base mode.
}

// MsgID (generated function)
func (m *SetMode) MsgID() mavlink.MessageID {
	return MSG_ID_SET_MODE
}

// String (generated function)
func (m *SetMode) String() string {
	return fmt.Sprintf(
		"&common.SetMode{ CustomMode: %+v, TargetSystem: %+v, BaseMode: %+v }",
		m.CustomMode,
		m.TargetSystem,
		m.BaseMode,
	)
}

// Pack (generated function)
func (m *SetMode) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 6)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.CustomMode))
	payload[4] = byte(m.TargetSystem)
	payload[5] = byte(m.BaseMode)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *SetMode) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 6 {
		payload = append(payload, mavlink.ZeroTail[:6-len(p.Payload)]...)
	}
	m.CustomMode = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.TargetSystem = uint8(payload[4])
	m.BaseMode = MAV_MODE(payload[5])
	return nil
}

// ParamAckTransaction struct (generated typeinfo)
// Response from a PARAM_SET message when it is used in a transaction.
type ParamAckTransaction struct {
	ParamValue      float32        // Parameter value (new value if PARAM_ACCEPTED, current value otherwise)
	TargetSystem    uint8          // Id of system that sent PARAM_SET message.
	TargetComponent uint8          // Id of system that sent PARAM_SET message.
	ParamID         [16]byte       // Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
	ParamType       MAV_PARAM_TYPE `gotype:"uint8"` // Parameter type.
	ParamResult     PARAM_ACK      `gotype:"uint8"` // Result code.
}

// MsgID (generated function)
func (m *ParamAckTransaction) MsgID() mavlink.MessageID {
	return MSG_ID_PARAM_ACK_TRANSACTION
}

// String (generated function)
func (m *ParamAckTransaction) String() string {
	return fmt.Sprintf(
		"&common.ParamAckTransaction{ ParamValue: %+v, TargetSystem: %+v, TargetComponent: %+v, ParamID: %0X (\"%s\"), ParamType: %+v, ParamResult: %+v }",
		m.ParamValue,
		m.TargetSystem,
		m.TargetComponent,
		m.ParamID, string(m.ParamID[:]),
		m.ParamType,
		m.ParamResult,
	)
}

// Pack (generated function)
func (m *ParamAckTransaction) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 24)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(m.ParamValue))
	payload[4] = byte(m.TargetSystem)
	payload[5] = byte(m.TargetComponent)
	copy(payload[6:], m.ParamID[:])
	payload[22] = byte(m.ParamType)
	payload[23] = byte(m.ParamResult)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ParamAckTransaction) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 24 {
		payload = append(payload, mavlink.ZeroTail[:24-len(p.Payload)]...)
	}
	m.ParamValue = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.TargetSystem = uint8(payload[4])
	m.TargetComponent = uint8(payload[5])
	copy(m.ParamID[:], payload[6:22])
	m.ParamType = MAV_PARAM_TYPE(payload[22])
	m.ParamResult = PARAM_ACK(payload[23])
	return nil
}

// ParamRequestRead struct (generated typeinfo)
// Request to read the onboard parameter with the param_id string id. Onboard parameters are stored as key[const char*] -> value[float]. This allows to send a parameter to any other component (such as the GCS) without the need of previous knowledge of possible parameter names. Thus the same GCS can store different parameters for different autopilots. See also https://mavlink.io/en/services/parameter.html for a full documentation of QGroundControl and IMU code.
type ParamRequestRead struct {
	ParamIndex      int16    // Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignored)
	TargetSystem    uint8    // System ID
	TargetComponent uint8    // Component ID
	ParamID         [16]byte // Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
}

// MsgID (generated function)
func (m *ParamRequestRead) MsgID() mavlink.MessageID {
	return MSG_ID_PARAM_REQUEST_READ
}

// String (generated function)
func (m *ParamRequestRead) String() string {
	return fmt.Sprintf(
		"&common.ParamRequestRead{ ParamIndex: %+v, TargetSystem: %+v, TargetComponent: %+v, ParamID: %0X (\"%s\") }",
		m.ParamIndex,
		m.TargetSystem,
		m.TargetComponent,
		m.ParamID, string(m.ParamID[:]),
	)
}

// Pack (generated function)
func (m *ParamRequestRead) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 20)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.ParamIndex))
	payload[2] = byte(m.TargetSystem)
	payload[3] = byte(m.TargetComponent)
	copy(payload[4:], m.ParamID[:])
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ParamRequestRead) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 20 {
		payload = append(payload, mavlink.ZeroTail[:20-len(p.Payload)]...)
	}
	m.ParamIndex = int16(binary.LittleEndian.Uint16(payload[0:]))
	m.TargetSystem = uint8(payload[2])
	m.TargetComponent = uint8(payload[3])
	copy(m.ParamID[:], payload[4:20])
	return nil
}

// ParamRequestList struct (generated typeinfo)
// Request all parameters of this component. After this request, all parameters are emitted. The parameter microservice is documented at https://mavlink.io/en/services/parameter.html
type ParamRequestList struct {
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
}

// MsgID (generated function)
func (m *ParamRequestList) MsgID() mavlink.MessageID {
	return MSG_ID_PARAM_REQUEST_LIST
}

// String (generated function)
func (m *ParamRequestList) String() string {
	return fmt.Sprintf(
		"&common.ParamRequestList{ TargetSystem: %+v, TargetComponent: %+v }",
		m.TargetSystem,
		m.TargetComponent,
	)
}

// Pack (generated function)
func (m *ParamRequestList) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 2)
	payload[0] = byte(m.TargetSystem)
	payload[1] = byte(m.TargetComponent)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ParamRequestList) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 2 {
		payload = append(payload, mavlink.ZeroTail[:2-len(p.Payload)]...)
	}
	m.TargetSystem = uint8(payload[0])
	m.TargetComponent = uint8(payload[1])
	return nil
}

// ParamValue struct (generated typeinfo)
// Emit the value of a onboard parameter. The inclusion of param_count and param_index in the message allows the recipient to keep track of received parameters and allows him to re-request missing parameters after a loss or timeout. The parameter microservice is documented at https://mavlink.io/en/services/parameter.html
type ParamValue struct {
	ParamValue float32        // Onboard parameter value
	ParamCount uint16         // Total number of onboard parameters
	ParamIndex uint16         // Index of this onboard parameter
	ParamID    [16]byte       // Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
	ParamType  MAV_PARAM_TYPE `gotype:"uint8"` // Onboard parameter type.
}

// MsgID (generated function)
func (m *ParamValue) MsgID() mavlink.MessageID {
	return MSG_ID_PARAM_VALUE
}

// String (generated function)
func (m *ParamValue) String() string {
	return fmt.Sprintf(
		"&common.ParamValue{ ParamValue: %+v, ParamCount: %+v, ParamIndex: %+v, ParamID: %0X (\"%s\"), ParamType: %+v }",
		m.ParamValue,
		m.ParamCount,
		m.ParamIndex,
		m.ParamID, string(m.ParamID[:]),
		m.ParamType,
	)
}

// Pack (generated function)
func (m *ParamValue) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 25)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(m.ParamValue))
	binary.LittleEndian.PutUint16(payload[4:], uint16(m.ParamCount))
	binary.LittleEndian.PutUint16(payload[6:], uint16(m.ParamIndex))
	copy(payload[8:], m.ParamID[:])
	payload[24] = byte(m.ParamType)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ParamValue) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 25 {
		payload = append(payload, mavlink.ZeroTail[:25-len(p.Payload)]...)
	}
	m.ParamValue = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.ParamCount = uint16(binary.LittleEndian.Uint16(payload[4:]))
	m.ParamIndex = uint16(binary.LittleEndian.Uint16(payload[6:]))
	copy(m.ParamID[:], payload[8:24])
	m.ParamType = MAV_PARAM_TYPE(payload[24])
	return nil
}

// ParamSet struct (generated typeinfo)
// Set a parameter value (write new value to permanent storage).
//         The receiving component should acknowledge the new parameter value by broadcasting a PARAM_VALUE message (broadcasting ensures that multiple GCS all have an up-to-date list of all parameters). If the sending GCS did not receive a PARAM_VALUE within its timeout time, it should re-send the PARAM_SET message. The parameter microservice is documented at https://mavlink.io/en/services/parameter.html.
//         PARAM_SET may also be called within the context of a transaction (started with MAV_CMD_PARAM_TRANSACTION). Within a transaction the receiving component should respond with PARAM_ACK_TRANSACTION to the setter component (instead of broadcasting PARAM_VALUE), and PARAM_SET should be re-sent if this is ACK not received.
type ParamSet struct {
	ParamValue      float32        // Onboard parameter value
	TargetSystem    uint8          // System ID
	TargetComponent uint8          // Component ID
	ParamID         [16]byte       // Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
	ParamType       MAV_PARAM_TYPE `gotype:"uint8"` // Onboard parameter type.
}

// MsgID (generated function)
func (m *ParamSet) MsgID() mavlink.MessageID {
	return MSG_ID_PARAM_SET
}

// String (generated function)
func (m *ParamSet) String() string {
	return fmt.Sprintf(
		"&common.ParamSet{ ParamValue: %+v, TargetSystem: %+v, TargetComponent: %+v, ParamID: %0X (\"%s\"), ParamType: %+v }",
		m.ParamValue,
		m.TargetSystem,
		m.TargetComponent,
		m.ParamID, string(m.ParamID[:]),
		m.ParamType,
	)
}

// Pack (generated function)
func (m *ParamSet) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 23)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(m.ParamValue))
	payload[4] = byte(m.TargetSystem)
	payload[5] = byte(m.TargetComponent)
	copy(payload[6:], m.ParamID[:])
	payload[22] = byte(m.ParamType)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ParamSet) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 23 {
		payload = append(payload, mavlink.ZeroTail[:23-len(p.Payload)]...)
	}
	m.ParamValue = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.TargetSystem = uint8(payload[4])
	m.TargetComponent = uint8(payload[5])
	copy(m.ParamID[:], payload[6:22])
	m.ParamType = MAV_PARAM_TYPE(payload[22])
	return nil
}

// GpsRawInt struct (generated typeinfo)
// The global position, as returned by the Global Positioning System (GPS). This is
//                 NOT the global position estimate of the system, but rather a RAW sensor value. See message GLOBAL_POSITION for the global position estimate.
type GpsRawInt struct {
	TimeUsec          uint64       // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
	Lat               int32        // Latitude (WGS84, EGM96 ellipsoid)
	Lon               int32        // Longitude (WGS84, EGM96 ellipsoid)
	Alt               int32        // Altitude (MSL). Positive for up. Note that virtually all GPS modules provide the MSL altitude in addition to the WGS84 altitude.
	Eph               uint16       // GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX
	Epv               uint16       // GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX
	Vel               uint16       // GPS ground speed. If unknown, set to: UINT16_MAX
	Cog               uint16       // Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
	FixType           GPS_FIX_TYPE `gotype:"uint8"` // GPS fix type.
	SatellitesVisible uint8        // Number of satellites visible. If unknown, set to 255
}

// MsgID (generated function)
func (m *GpsRawInt) MsgID() mavlink.MessageID {
	return MSG_ID_GPS_RAW_INT
}

// String (generated function)
func (m *GpsRawInt) String() string {
	return fmt.Sprintf(
		"&common.GpsRawInt{ TimeUsec: %+v, Lat: %+v, Lon: %+v, Alt: %+v, Eph: %+v, Epv: %+v, Vel: %+v, Cog: %+v, FixType: %+v, SatellitesVisible: %+v }",
		m.TimeUsec,
		m.Lat,
		m.Lon,
		m.Alt,
		m.Eph,
		m.Epv,
		m.Vel,
		m.Cog,
		m.FixType,
		m.SatellitesVisible,
	)
}

// Pack (generated function)
func (m *GpsRawInt) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 30)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.TimeUsec))
	binary.LittleEndian.PutUint32(payload[8:], uint32(m.Lat))
	binary.LittleEndian.PutUint32(payload[12:], uint32(m.Lon))
	binary.LittleEndian.PutUint32(payload[16:], uint32(m.Alt))
	binary.LittleEndian.PutUint16(payload[20:], uint16(m.Eph))
	binary.LittleEndian.PutUint16(payload[22:], uint16(m.Epv))
	binary.LittleEndian.PutUint16(payload[24:], uint16(m.Vel))
	binary.LittleEndian.PutUint16(payload[26:], uint16(m.Cog))
	payload[28] = byte(m.FixType)
	payload[29] = byte(m.SatellitesVisible)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *GpsRawInt) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 30 {
		payload = append(payload, mavlink.ZeroTail[:30-len(p.Payload)]...)
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(payload[0:]))
	m.Lat = int32(binary.LittleEndian.Uint32(payload[8:]))
	m.Lon = int32(binary.LittleEndian.Uint32(payload[12:]))
	m.Alt = int32(binary.LittleEndian.Uint32(payload[16:]))
	m.Eph = uint16(binary.LittleEndian.Uint16(payload[20:]))
	m.Epv = uint16(binary.LittleEndian.Uint16(payload[22:]))
	m.Vel = uint16(binary.LittleEndian.Uint16(payload[24:]))
	m.Cog = uint16(binary.LittleEndian.Uint16(payload[26:]))
	m.FixType = GPS_FIX_TYPE(payload[28])
	m.SatellitesVisible = uint8(payload[29])
	return nil
}

// GpsStatus struct (generated typeinfo)
// The positioning status, as reported by GPS. This message is intended to display status information about each satellite visible to the receiver. See message GLOBAL_POSITION for the global position estimate. This message can contain information for up to 20 satellites.
type GpsStatus struct {
	SatellitesVisible  uint8     // Number of satellites visible
	SatellitePrn       [20]uint8 // Global satellite ID
	SatelliteUsed      [20]uint8 // 0: Satellite not used, 1: used for localization
	SatelliteElevation [20]uint8 // Elevation (0: right on top of receiver, 90: on the horizon) of satellite
	SatelliteAzimuth   [20]uint8 // Direction of satellite, 0: 0 deg, 255: 360 deg.
	SatelliteSnr       [20]uint8 // Signal to noise ratio of satellite
}

// MsgID (generated function)
func (m *GpsStatus) MsgID() mavlink.MessageID {
	return MSG_ID_GPS_STATUS
}

// String (generated function)
func (m *GpsStatus) String() string {
	return fmt.Sprintf(
		"&common.GpsStatus{ SatellitesVisible: %+v, SatellitePrn: %0X (\"%s\"), SatelliteUsed: %0X (\"%s\"), SatelliteElevation: %0X (\"%s\"), SatelliteAzimuth: %0X (\"%s\"), SatelliteSnr: %0X (\"%s\") }",
		m.SatellitesVisible,
		m.SatellitePrn, string(m.SatellitePrn[:]),
		m.SatelliteUsed, string(m.SatelliteUsed[:]),
		m.SatelliteElevation, string(m.SatelliteElevation[:]),
		m.SatelliteAzimuth, string(m.SatelliteAzimuth[:]),
		m.SatelliteSnr, string(m.SatelliteSnr[:]),
	)
}

// Pack (generated function)
func (m *GpsStatus) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 101)
	payload[0] = byte(m.SatellitesVisible)
	copy(payload[1:], m.SatellitePrn[:])
	copy(payload[21:], m.SatelliteUsed[:])
	copy(payload[41:], m.SatelliteElevation[:])
	copy(payload[61:], m.SatelliteAzimuth[:])
	copy(payload[81:], m.SatelliteSnr[:])
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *GpsStatus) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 101 {
		payload = append(payload, mavlink.ZeroTail[:101-len(p.Payload)]...)
	}
	m.SatellitesVisible = uint8(payload[0])
	copy(m.SatellitePrn[:], payload[1:21])
	copy(m.SatelliteUsed[:], payload[21:41])
	copy(m.SatelliteElevation[:], payload[41:61])
	copy(m.SatelliteAzimuth[:], payload[61:81])
	copy(m.SatelliteSnr[:], payload[81:101])
	return nil
}

// ScaledImu struct (generated typeinfo)
// The RAW IMU readings for the usual 9DOF sensor setup. This message should contain the scaled values to the described units
type ScaledImu struct {
	TimeBootMs uint32 // Timestamp (time since system boot).
	Xacc       int16  // X acceleration
	Yacc       int16  // Y acceleration
	Zacc       int16  // Z acceleration
	Xgyro      int16  // Angular speed around X axis
	Ygyro      int16  // Angular speed around Y axis
	Zgyro      int16  // Angular speed around Z axis
	Xmag       int16  // X Magnetic field
	Ymag       int16  // Y Magnetic field
	Zmag       int16  // Z Magnetic field
}

// MsgID (generated function)
func (m *ScaledImu) MsgID() mavlink.MessageID {
	return MSG_ID_SCALED_IMU
}

// String (generated function)
func (m *ScaledImu) String() string {
	return fmt.Sprintf(
		"&common.ScaledImu{ TimeBootMs: %+v, Xacc: %+v, Yacc: %+v, Zacc: %+v, Xgyro: %+v, Ygyro: %+v, Zgyro: %+v, Xmag: %+v, Ymag: %+v, Zmag: %+v }",
		m.TimeBootMs,
		m.Xacc,
		m.Yacc,
		m.Zacc,
		m.Xgyro,
		m.Ygyro,
		m.Zgyro,
		m.Xmag,
		m.Ymag,
		m.Zmag,
	)
}

// Pack (generated function)
func (m *ScaledImu) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 22)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.TimeBootMs))
	binary.LittleEndian.PutUint16(payload[4:], uint16(m.Xacc))
	binary.LittleEndian.PutUint16(payload[6:], uint16(m.Yacc))
	binary.LittleEndian.PutUint16(payload[8:], uint16(m.Zacc))
	binary.LittleEndian.PutUint16(payload[10:], uint16(m.Xgyro))
	binary.LittleEndian.PutUint16(payload[12:], uint16(m.Ygyro))
	binary.LittleEndian.PutUint16(payload[14:], uint16(m.Zgyro))
	binary.LittleEndian.PutUint16(payload[16:], uint16(m.Xmag))
	binary.LittleEndian.PutUint16(payload[18:], uint16(m.Ymag))
	binary.LittleEndian.PutUint16(payload[20:], uint16(m.Zmag))
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ScaledImu) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 22 {
		payload = append(payload, mavlink.ZeroTail[:22-len(p.Payload)]...)
	}
	m.TimeBootMs = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.Xacc = int16(binary.LittleEndian.Uint16(payload[4:]))
	m.Yacc = int16(binary.LittleEndian.Uint16(payload[6:]))
	m.Zacc = int16(binary.LittleEndian.Uint16(payload[8:]))
	m.Xgyro = int16(binary.LittleEndian.Uint16(payload[10:]))
	m.Ygyro = int16(binary.LittleEndian.Uint16(payload[12:]))
	m.Zgyro = int16(binary.LittleEndian.Uint16(payload[14:]))
	m.Xmag = int16(binary.LittleEndian.Uint16(payload[16:]))
	m.Ymag = int16(binary.LittleEndian.Uint16(payload[18:]))
	m.Zmag = int16(binary.LittleEndian.Uint16(payload[20:]))
	return nil
}

// RawImu struct (generated typeinfo)
// The RAW IMU readings for a 9DOF sensor, which is identified by the id (default IMU1). This message should always contain the true raw values without any scaling to allow data capture and system debugging.
type RawImu struct {
	TimeUsec uint64 // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
	Xacc     int16  // X acceleration (raw)
	Yacc     int16  // Y acceleration (raw)
	Zacc     int16  // Z acceleration (raw)
	Xgyro    int16  // Angular speed around X axis (raw)
	Ygyro    int16  // Angular speed around Y axis (raw)
	Zgyro    int16  // Angular speed around Z axis (raw)
	Xmag     int16  // X Magnetic field (raw)
	Ymag     int16  // Y Magnetic field (raw)
	Zmag     int16  // Z Magnetic field (raw)
}

// MsgID (generated function)
func (m *RawImu) MsgID() mavlink.MessageID {
	return MSG_ID_RAW_IMU
}

// String (generated function)
func (m *RawImu) String() string {
	return fmt.Sprintf(
		"&common.RawImu{ TimeUsec: %+v, Xacc: %+v, Yacc: %+v, Zacc: %+v, Xgyro: %+v, Ygyro: %+v, Zgyro: %+v, Xmag: %+v, Ymag: %+v, Zmag: %+v }",
		m.TimeUsec,
		m.Xacc,
		m.Yacc,
		m.Zacc,
		m.Xgyro,
		m.Ygyro,
		m.Zgyro,
		m.Xmag,
		m.Ymag,
		m.Zmag,
	)
}

// Pack (generated function)
func (m *RawImu) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 26)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.TimeUsec))
	binary.LittleEndian.PutUint16(payload[8:], uint16(m.Xacc))
	binary.LittleEndian.PutUint16(payload[10:], uint16(m.Yacc))
	binary.LittleEndian.PutUint16(payload[12:], uint16(m.Zacc))
	binary.LittleEndian.PutUint16(payload[14:], uint16(m.Xgyro))
	binary.LittleEndian.PutUint16(payload[16:], uint16(m.Ygyro))
	binary.LittleEndian.PutUint16(payload[18:], uint16(m.Zgyro))
	binary.LittleEndian.PutUint16(payload[20:], uint16(m.Xmag))
	binary.LittleEndian.PutUint16(payload[22:], uint16(m.Ymag))
	binary.LittleEndian.PutUint16(payload[24:], uint16(m.Zmag))
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *RawImu) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 26 {
		payload = append(payload, mavlink.ZeroTail[:26-len(p.Payload)]...)
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(payload[0:]))
	m.Xacc = int16(binary.LittleEndian.Uint16(payload[8:]))
	m.Yacc = int16(binary.LittleEndian.Uint16(payload[10:]))
	m.Zacc = int16(binary.LittleEndian.Uint16(payload[12:]))
	m.Xgyro = int16(binary.LittleEndian.Uint16(payload[14:]))
	m.Ygyro = int16(binary.LittleEndian.Uint16(payload[16:]))
	m.Zgyro = int16(binary.LittleEndian.Uint16(payload[18:]))
	m.Xmag = int16(binary.LittleEndian.Uint16(payload[20:]))
	m.Ymag = int16(binary.LittleEndian.Uint16(payload[22:]))
	m.Zmag = int16(binary.LittleEndian.Uint16(payload[24:]))
	return nil
}

// RawPressure struct (generated typeinfo)
// The RAW pressure readings for the typical setup of one absolute pressure and one differential pressure sensor. The sensor values should be the raw, UNSCALED ADC values.
type RawPressure struct {
	TimeUsec    uint64 // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
	PressAbs    int16  // Absolute pressure (raw)
	PressDiff1  int16  // Differential pressure 1 (raw, 0 if nonexistent)
	PressDiff2  int16  // Differential pressure 2 (raw, 0 if nonexistent)
	Temperature int16  // Raw Temperature measurement (raw)
}

// MsgID (generated function)
func (m *RawPressure) MsgID() mavlink.MessageID {
	return MSG_ID_RAW_PRESSURE
}

// String (generated function)
func (m *RawPressure) String() string {
	return fmt.Sprintf(
		"&common.RawPressure{ TimeUsec: %+v, PressAbs: %+v, PressDiff1: %+v, PressDiff2: %+v, Temperature: %+v }",
		m.TimeUsec,
		m.PressAbs,
		m.PressDiff1,
		m.PressDiff2,
		m.Temperature,
	)
}

// Pack (generated function)
func (m *RawPressure) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 16)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.TimeUsec))
	binary.LittleEndian.PutUint16(payload[8:], uint16(m.PressAbs))
	binary.LittleEndian.PutUint16(payload[10:], uint16(m.PressDiff1))
	binary.LittleEndian.PutUint16(payload[12:], uint16(m.PressDiff2))
	binary.LittleEndian.PutUint16(payload[14:], uint16(m.Temperature))
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *RawPressure) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 16 {
		payload = append(payload, mavlink.ZeroTail[:16-len(p.Payload)]...)
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(payload[0:]))
	m.PressAbs = int16(binary.LittleEndian.Uint16(payload[8:]))
	m.PressDiff1 = int16(binary.LittleEndian.Uint16(payload[10:]))
	m.PressDiff2 = int16(binary.LittleEndian.Uint16(payload[12:]))
	m.Temperature = int16(binary.LittleEndian.Uint16(payload[14:]))
	return nil
}

// ScaledPressure struct (generated typeinfo)
// The pressure readings for the typical setup of one absolute and differential pressure sensor. The units are as specified in each field.
type ScaledPressure struct {
	TimeBootMs  uint32  // Timestamp (time since system boot).
	PressAbs    float32 // Absolute pressure
	PressDiff   float32 // Differential pressure 1
	Temperature int16   // Absolute pressure temperature
}

// MsgID (generated function)
func (m *ScaledPressure) MsgID() mavlink.MessageID {
	return MSG_ID_SCALED_PRESSURE
}

// String (generated function)
func (m *ScaledPressure) String() string {
	return fmt.Sprintf(
		"&common.ScaledPressure{ TimeBootMs: %+v, PressAbs: %+v, PressDiff: %+v, Temperature: %+v }",
		m.TimeBootMs,
		m.PressAbs,
		m.PressDiff,
		m.Temperature,
	)
}

// Pack (generated function)
func (m *ScaledPressure) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 14)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.TimeBootMs))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(m.PressAbs))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.PressDiff))
	binary.LittleEndian.PutUint16(payload[12:], uint16(m.Temperature))
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ScaledPressure) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 14 {
		payload = append(payload, mavlink.ZeroTail[:14-len(p.Payload)]...)
	}
	m.TimeBootMs = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.PressAbs = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	m.PressDiff = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.Temperature = int16(binary.LittleEndian.Uint16(payload[12:]))
	return nil
}

// Attitude struct (generated typeinfo)
// The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right).
type Attitude struct {
	TimeBootMs uint32  // Timestamp (time since system boot).
	Roll       float32 // Roll angle (-pi..+pi)
	Pitch      float32 // Pitch angle (-pi..+pi)
	Yaw        float32 // Yaw angle (-pi..+pi)
	Rollspeed  float32 // Roll angular speed
	Pitchspeed float32 // Pitch angular speed
	Yawspeed   float32 // Yaw angular speed
}

// MsgID (generated function)
func (m *Attitude) MsgID() mavlink.MessageID {
	return MSG_ID_ATTITUDE
}

// String (generated function)
func (m *Attitude) String() string {
	return fmt.Sprintf(
		"&common.Attitude{ TimeBootMs: %+v, Roll: %+v, Pitch: %+v, Yaw: %+v, Rollspeed: %+v, Pitchspeed: %+v, Yawspeed: %+v }",
		m.TimeBootMs,
		m.Roll,
		m.Pitch,
		m.Yaw,
		m.Rollspeed,
		m.Pitchspeed,
		m.Yawspeed,
	)
}

// Pack (generated function)
func (m *Attitude) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 28)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.TimeBootMs))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(m.Roll))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.Pitch))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.Yaw))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(m.Rollspeed))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(m.Pitchspeed))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(m.Yawspeed))
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *Attitude) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 28 {
		payload = append(payload, mavlink.ZeroTail[:28-len(p.Payload)]...)
	}
	m.TimeBootMs = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.Roll = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	m.Pitch = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.Yaw = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.Rollspeed = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	m.Pitchspeed = math.Float32frombits(binary.LittleEndian.Uint32(payload[20:]))
	m.Yawspeed = math.Float32frombits(binary.LittleEndian.Uint32(payload[24:]))
	return nil
}

// AttitudeQuaternion struct (generated typeinfo)
// The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right), expressed as quaternion. Quaternion order is w, x, y, z and a zero rotation would be expressed as (1 0 0 0).
type AttitudeQuaternion struct {
	TimeBootMs uint32  // Timestamp (time since system boot).
	Q1         float32 // Quaternion component 1, w (1 in null-rotation)
	Q2         float32 // Quaternion component 2, x (0 in null-rotation)
	Q3         float32 // Quaternion component 3, y (0 in null-rotation)
	Q4         float32 // Quaternion component 4, z (0 in null-rotation)
	Rollspeed  float32 // Roll angular speed
	Pitchspeed float32 // Pitch angular speed
	Yawspeed   float32 // Yaw angular speed
}

// MsgID (generated function)
func (m *AttitudeQuaternion) MsgID() mavlink.MessageID {
	return MSG_ID_ATTITUDE_QUATERNION
}

// String (generated function)
func (m *AttitudeQuaternion) String() string {
	return fmt.Sprintf(
		"&common.AttitudeQuaternion{ TimeBootMs: %+v, Q1: %+v, Q2: %+v, Q3: %+v, Q4: %+v, Rollspeed: %+v, Pitchspeed: %+v, Yawspeed: %+v }",
		m.TimeBootMs,
		m.Q1,
		m.Q2,
		m.Q3,
		m.Q4,
		m.Rollspeed,
		m.Pitchspeed,
		m.Yawspeed,
	)
}

// Pack (generated function)
func (m *AttitudeQuaternion) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 32)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.TimeBootMs))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(m.Q1))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.Q2))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.Q3))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(m.Q4))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(m.Rollspeed))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(m.Pitchspeed))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(m.Yawspeed))
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *AttitudeQuaternion) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 32 {
		payload = append(payload, mavlink.ZeroTail[:32-len(p.Payload)]...)
	}
	m.TimeBootMs = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.Q1 = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	m.Q2 = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.Q3 = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.Q4 = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	m.Rollspeed = math.Float32frombits(binary.LittleEndian.Uint32(payload[20:]))
	m.Pitchspeed = math.Float32frombits(binary.LittleEndian.Uint32(payload[24:]))
	m.Yawspeed = math.Float32frombits(binary.LittleEndian.Uint32(payload[28:]))
	return nil
}

// LocalPositionNed struct (generated typeinfo)
// The filtered local position (e.g. fused computer vision and accelerometers). Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down convention)
type LocalPositionNed struct {
	TimeBootMs uint32  // Timestamp (time since system boot).
	X          float32 // X Position
	Y          float32 // Y Position
	Z          float32 // Z Position
	Vx         float32 // X Speed
	Vy         float32 // Y Speed
	Vz         float32 // Z Speed
}

// MsgID (generated function)
func (m *LocalPositionNed) MsgID() mavlink.MessageID {
	return MSG_ID_LOCAL_POSITION_NED
}

// String (generated function)
func (m *LocalPositionNed) String() string {
	return fmt.Sprintf(
		"&common.LocalPositionNed{ TimeBootMs: %+v, X: %+v, Y: %+v, Z: %+v, Vx: %+v, Vy: %+v, Vz: %+v }",
		m.TimeBootMs,
		m.X,
		m.Y,
		m.Z,
		m.Vx,
		m.Vy,
		m.Vz,
	)
}

// Pack (generated function)
func (m *LocalPositionNed) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 28)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.TimeBootMs))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(m.X))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.Y))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.Z))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(m.Vx))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(m.Vy))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(m.Vz))
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *LocalPositionNed) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 28 {
		payload = append(payload, mavlink.ZeroTail[:28-len(p.Payload)]...)
	}
	m.TimeBootMs = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.X = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	m.Y = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.Z = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.Vx = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	m.Vy = math.Float32frombits(binary.LittleEndian.Uint32(payload[20:]))
	m.Vz = math.Float32frombits(binary.LittleEndian.Uint32(payload[24:]))
	return nil
}

// GlobalPositionInt struct (generated typeinfo)
// The filtered global position (e.g. fused GPS and accelerometers). The position is in GPS-frame (right-handed, Z-up). It
//                is designed as scaled integer message since the resolution of float is not sufficient.
type GlobalPositionInt struct {
	TimeBootMs  uint32 // Timestamp (time since system boot).
	Lat         int32  // Latitude, expressed
	Lon         int32  // Longitude, expressed
	Alt         int32  // Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL.
	RelativeAlt int32  // Altitude above ground
	Vx          int16  // Ground X Speed (Latitude, positive north)
	Vy          int16  // Ground Y Speed (Longitude, positive east)
	Vz          int16  // Ground Z Speed (Altitude, positive down)
	Hdg         uint16 // Vehicle heading (yaw angle), 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
}

// MsgID (generated function)
func (m *GlobalPositionInt) MsgID() mavlink.MessageID {
	return MSG_ID_GLOBAL_POSITION_INT
}

// String (generated function)
func (m *GlobalPositionInt) String() string {
	return fmt.Sprintf(
		"&common.GlobalPositionInt{ TimeBootMs: %+v, Lat: %+v, Lon: %+v, Alt: %+v, RelativeAlt: %+v, Vx: %+v, Vy: %+v, Vz: %+v, Hdg: %+v }",
		m.TimeBootMs,
		m.Lat,
		m.Lon,
		m.Alt,
		m.RelativeAlt,
		m.Vx,
		m.Vy,
		m.Vz,
		m.Hdg,
	)
}

// Pack (generated function)
func (m *GlobalPositionInt) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 28)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.TimeBootMs))
	binary.LittleEndian.PutUint32(payload[4:], uint32(m.Lat))
	binary.LittleEndian.PutUint32(payload[8:], uint32(m.Lon))
	binary.LittleEndian.PutUint32(payload[12:], uint32(m.Alt))
	binary.LittleEndian.PutUint32(payload[16:], uint32(m.RelativeAlt))
	binary.LittleEndian.PutUint16(payload[20:], uint16(m.Vx))
	binary.LittleEndian.PutUint16(payload[22:], uint16(m.Vy))
	binary.LittleEndian.PutUint16(payload[24:], uint16(m.Vz))
	binary.LittleEndian.PutUint16(payload[26:], uint16(m.Hdg))
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *GlobalPositionInt) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 28 {
		payload = append(payload, mavlink.ZeroTail[:28-len(p.Payload)]...)
	}
	m.TimeBootMs = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.Lat = int32(binary.LittleEndian.Uint32(payload[4:]))
	m.Lon = int32(binary.LittleEndian.Uint32(payload[8:]))
	m.Alt = int32(binary.LittleEndian.Uint32(payload[12:]))
	m.RelativeAlt = int32(binary.LittleEndian.Uint32(payload[16:]))
	m.Vx = int16(binary.LittleEndian.Uint16(payload[20:]))
	m.Vy = int16(binary.LittleEndian.Uint16(payload[22:]))
	m.Vz = int16(binary.LittleEndian.Uint16(payload[24:]))
	m.Hdg = uint16(binary.LittleEndian.Uint16(payload[26:]))
	return nil
}

// RcChannelsScaled struct (generated typeinfo)
// The scaled values of the RC channels received: (-100%) -10000, (0%) 0, (100%) 10000. Channels that are inactive should be set to UINT16_MAX.
type RcChannelsScaled struct {
	TimeBootMs  uint32 // Timestamp (time since system boot).
	Chan1Scaled int16  // RC channel 1 value scaled.
	Chan2Scaled int16  // RC channel 2 value scaled.
	Chan3Scaled int16  // RC channel 3 value scaled.
	Chan4Scaled int16  // RC channel 4 value scaled.
	Chan5Scaled int16  // RC channel 5 value scaled.
	Chan6Scaled int16  // RC channel 6 value scaled.
	Chan7Scaled int16  // RC channel 7 value scaled.
	Chan8Scaled int16  // RC channel 8 value scaled.
	Port        uint8  // Servo output port (set of 8 outputs = 1 port). Flight stacks running on Pixhawk should use: 0 = MAIN, 1 = AUX.
	Rssi        uint8  // Receive signal strength indicator in device-dependent units/scale. Values: [0-254], 255: invalid/unknown.
}

// MsgID (generated function)
func (m *RcChannelsScaled) MsgID() mavlink.MessageID {
	return MSG_ID_RC_CHANNELS_SCALED
}

// String (generated function)
func (m *RcChannelsScaled) String() string {
	return fmt.Sprintf(
		"&common.RcChannelsScaled{ TimeBootMs: %+v, Chan1Scaled: %+v, Chan2Scaled: %+v, Chan3Scaled: %+v, Chan4Scaled: %+v, Chan5Scaled: %+v, Chan6Scaled: %+v, Chan7Scaled: %+v, Chan8Scaled: %+v, Port: %+v, Rssi: %+v }",
		m.TimeBootMs,
		m.Chan1Scaled,
		m.Chan2Scaled,
		m.Chan3Scaled,
		m.Chan4Scaled,
		m.Chan5Scaled,
		m.Chan6Scaled,
		m.Chan7Scaled,
		m.Chan8Scaled,
		m.Port,
		m.Rssi,
	)
}

// Pack (generated function)
func (m *RcChannelsScaled) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 22)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.TimeBootMs))
	binary.LittleEndian.PutUint16(payload[4:], uint16(m.Chan1Scaled))
	binary.LittleEndian.PutUint16(payload[6:], uint16(m.Chan2Scaled))
	binary.LittleEndian.PutUint16(payload[8:], uint16(m.Chan3Scaled))
	binary.LittleEndian.PutUint16(payload[10:], uint16(m.Chan4Scaled))
	binary.LittleEndian.PutUint16(payload[12:], uint16(m.Chan5Scaled))
	binary.LittleEndian.PutUint16(payload[14:], uint16(m.Chan6Scaled))
	binary.LittleEndian.PutUint16(payload[16:], uint16(m.Chan7Scaled))
	binary.LittleEndian.PutUint16(payload[18:], uint16(m.Chan8Scaled))
	payload[20] = byte(m.Port)
	payload[21] = byte(m.Rssi)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *RcChannelsScaled) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 22 {
		payload = append(payload, mavlink.ZeroTail[:22-len(p.Payload)]...)
	}
	m.TimeBootMs = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.Chan1Scaled = int16(binary.LittleEndian.Uint16(payload[4:]))
	m.Chan2Scaled = int16(binary.LittleEndian.Uint16(payload[6:]))
	m.Chan3Scaled = int16(binary.LittleEndian.Uint16(payload[8:]))
	m.Chan4Scaled = int16(binary.LittleEndian.Uint16(payload[10:]))
	m.Chan5Scaled = int16(binary.LittleEndian.Uint16(payload[12:]))
	m.Chan6Scaled = int16(binary.LittleEndian.Uint16(payload[14:]))
	m.Chan7Scaled = int16(binary.LittleEndian.Uint16(payload[16:]))
	m.Chan8Scaled = int16(binary.LittleEndian.Uint16(payload[18:]))
	m.Port = uint8(payload[20])
	m.Rssi = uint8(payload[21])
	return nil
}

// RcChannelsRaw struct (generated typeinfo)
// The RAW values of the RC channels received. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. A value of UINT16_MAX implies the channel is unused. Individual receivers/transmitters might violate this specification.
type RcChannelsRaw struct {
	TimeBootMs uint32 // Timestamp (time since system boot).
	Chan1Raw   uint16 // RC channel 1 value.
	Chan2Raw   uint16 // RC channel 2 value.
	Chan3Raw   uint16 // RC channel 3 value.
	Chan4Raw   uint16 // RC channel 4 value.
	Chan5Raw   uint16 // RC channel 5 value.
	Chan6Raw   uint16 // RC channel 6 value.
	Chan7Raw   uint16 // RC channel 7 value.
	Chan8Raw   uint16 // RC channel 8 value.
	Port       uint8  // Servo output port (set of 8 outputs = 1 port). Flight stacks running on Pixhawk should use: 0 = MAIN, 1 = AUX.
	Rssi       uint8  // Receive signal strength indicator in device-dependent units/scale. Values: [0-254], 255: invalid/unknown.
}

// MsgID (generated function)
func (m *RcChannelsRaw) MsgID() mavlink.MessageID {
	return MSG_ID_RC_CHANNELS_RAW
}

// String (generated function)
func (m *RcChannelsRaw) String() string {
	return fmt.Sprintf(
		"&common.RcChannelsRaw{ TimeBootMs: %+v, Chan1Raw: %+v, Chan2Raw: %+v, Chan3Raw: %+v, Chan4Raw: %+v, Chan5Raw: %+v, Chan6Raw: %+v, Chan7Raw: %+v, Chan8Raw: %+v, Port: %+v, Rssi: %+v }",
		m.TimeBootMs,
		m.Chan1Raw,
		m.Chan2Raw,
		m.Chan3Raw,
		m.Chan4Raw,
		m.Chan5Raw,
		m.Chan6Raw,
		m.Chan7Raw,
		m.Chan8Raw,
		m.Port,
		m.Rssi,
	)
}

// Pack (generated function)
func (m *RcChannelsRaw) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 22)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.TimeBootMs))
	binary.LittleEndian.PutUint16(payload[4:], uint16(m.Chan1Raw))
	binary.LittleEndian.PutUint16(payload[6:], uint16(m.Chan2Raw))
	binary.LittleEndian.PutUint16(payload[8:], uint16(m.Chan3Raw))
	binary.LittleEndian.PutUint16(payload[10:], uint16(m.Chan4Raw))
	binary.LittleEndian.PutUint16(payload[12:], uint16(m.Chan5Raw))
	binary.LittleEndian.PutUint16(payload[14:], uint16(m.Chan6Raw))
	binary.LittleEndian.PutUint16(payload[16:], uint16(m.Chan7Raw))
	binary.LittleEndian.PutUint16(payload[18:], uint16(m.Chan8Raw))
	payload[20] = byte(m.Port)
	payload[21] = byte(m.Rssi)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *RcChannelsRaw) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 22 {
		payload = append(payload, mavlink.ZeroTail[:22-len(p.Payload)]...)
	}
	m.TimeBootMs = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.Chan1Raw = uint16(binary.LittleEndian.Uint16(payload[4:]))
	m.Chan2Raw = uint16(binary.LittleEndian.Uint16(payload[6:]))
	m.Chan3Raw = uint16(binary.LittleEndian.Uint16(payload[8:]))
	m.Chan4Raw = uint16(binary.LittleEndian.Uint16(payload[10:]))
	m.Chan5Raw = uint16(binary.LittleEndian.Uint16(payload[12:]))
	m.Chan6Raw = uint16(binary.LittleEndian.Uint16(payload[14:]))
	m.Chan7Raw = uint16(binary.LittleEndian.Uint16(payload[16:]))
	m.Chan8Raw = uint16(binary.LittleEndian.Uint16(payload[18:]))
	m.Port = uint8(payload[20])
	m.Rssi = uint8(payload[21])
	return nil
}

// ServoOutputRaw struct (generated typeinfo)
// Superseded by ACTUATOR_OUTPUT_STATUS. The RAW values of the servo outputs (for RC input from the remote, use the RC_CHANNELS messages). The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%.
type ServoOutputRaw struct {
	TimeUsec  uint32 // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
	Servo1Raw uint16 // Servo output 1 value
	Servo2Raw uint16 // Servo output 2 value
	Servo3Raw uint16 // Servo output 3 value
	Servo4Raw uint16 // Servo output 4 value
	Servo5Raw uint16 // Servo output 5 value
	Servo6Raw uint16 // Servo output 6 value
	Servo7Raw uint16 // Servo output 7 value
	Servo8Raw uint16 // Servo output 8 value
	Port      uint8  // Servo output port (set of 8 outputs = 1 port). Flight stacks running on Pixhawk should use: 0 = MAIN, 1 = AUX.
}

// MsgID (generated function)
func (m *ServoOutputRaw) MsgID() mavlink.MessageID {
	return MSG_ID_SERVO_OUTPUT_RAW
}

// String (generated function)
func (m *ServoOutputRaw) String() string {
	return fmt.Sprintf(
		"&common.ServoOutputRaw{ TimeUsec: %+v, Servo1Raw: %+v, Servo2Raw: %+v, Servo3Raw: %+v, Servo4Raw: %+v, Servo5Raw: %+v, Servo6Raw: %+v, Servo7Raw: %+v, Servo8Raw: %+v, Port: %+v }",
		m.TimeUsec,
		m.Servo1Raw,
		m.Servo2Raw,
		m.Servo3Raw,
		m.Servo4Raw,
		m.Servo5Raw,
		m.Servo6Raw,
		m.Servo7Raw,
		m.Servo8Raw,
		m.Port,
	)
}

// Pack (generated function)
func (m *ServoOutputRaw) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 21)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.TimeUsec))
	binary.LittleEndian.PutUint16(payload[4:], uint16(m.Servo1Raw))
	binary.LittleEndian.PutUint16(payload[6:], uint16(m.Servo2Raw))
	binary.LittleEndian.PutUint16(payload[8:], uint16(m.Servo3Raw))
	binary.LittleEndian.PutUint16(payload[10:], uint16(m.Servo4Raw))
	binary.LittleEndian.PutUint16(payload[12:], uint16(m.Servo5Raw))
	binary.LittleEndian.PutUint16(payload[14:], uint16(m.Servo6Raw))
	binary.LittleEndian.PutUint16(payload[16:], uint16(m.Servo7Raw))
	binary.LittleEndian.PutUint16(payload[18:], uint16(m.Servo8Raw))
	payload[20] = byte(m.Port)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ServoOutputRaw) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 21 {
		payload = append(payload, mavlink.ZeroTail[:21-len(p.Payload)]...)
	}
	m.TimeUsec = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.Servo1Raw = uint16(binary.LittleEndian.Uint16(payload[4:]))
	m.Servo2Raw = uint16(binary.LittleEndian.Uint16(payload[6:]))
	m.Servo3Raw = uint16(binary.LittleEndian.Uint16(payload[8:]))
	m.Servo4Raw = uint16(binary.LittleEndian.Uint16(payload[10:]))
	m.Servo5Raw = uint16(binary.LittleEndian.Uint16(payload[12:]))
	m.Servo6Raw = uint16(binary.LittleEndian.Uint16(payload[14:]))
	m.Servo7Raw = uint16(binary.LittleEndian.Uint16(payload[16:]))
	m.Servo8Raw = uint16(binary.LittleEndian.Uint16(payload[18:]))
	m.Port = uint8(payload[20])
	return nil
}

// MissionRequestPartialList struct (generated typeinfo)
// Request a partial list of mission items from the system/component. https://mavlink.io/en/services/mission.html. If start and end index are the same, just send one waypoint.
type MissionRequestPartialList struct {
	StartIndex      int16 // Start index
	EndIndex        int16 // End index, -1 by default (-1: send list to end). Else a valid index of the list
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
}

// MsgID (generated function)
func (m *MissionRequestPartialList) MsgID() mavlink.MessageID {
	return MSG_ID_MISSION_REQUEST_PARTIAL_LIST
}

// String (generated function)
func (m *MissionRequestPartialList) String() string {
	return fmt.Sprintf(
		"&common.MissionRequestPartialList{ StartIndex: %+v, EndIndex: %+v, TargetSystem: %+v, TargetComponent: %+v }",
		m.StartIndex,
		m.EndIndex,
		m.TargetSystem,
		m.TargetComponent,
	)
}

// Pack (generated function)
func (m *MissionRequestPartialList) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 6)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.StartIndex))
	binary.LittleEndian.PutUint16(payload[2:], uint16(m.EndIndex))
	payload[4] = byte(m.TargetSystem)
	payload[5] = byte(m.TargetComponent)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *MissionRequestPartialList) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 6 {
		payload = append(payload, mavlink.ZeroTail[:6-len(p.Payload)]...)
	}
	m.StartIndex = int16(binary.LittleEndian.Uint16(payload[0:]))
	m.EndIndex = int16(binary.LittleEndian.Uint16(payload[2:]))
	m.TargetSystem = uint8(payload[4])
	m.TargetComponent = uint8(payload[5])
	return nil
}

// MissionWritePartialList struct (generated typeinfo)
// This message is sent to the MAV to write a partial list. If start index == end index, only one item will be transmitted / updated. If the start index is NOT 0 and above the current list size, this request should be REJECTED!
type MissionWritePartialList struct {
	StartIndex      int16 // Start index. Must be smaller / equal to the largest index of the current onboard list.
	EndIndex        int16 // End index, equal or greater than start index.
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
}

// MsgID (generated function)
func (m *MissionWritePartialList) MsgID() mavlink.MessageID {
	return MSG_ID_MISSION_WRITE_PARTIAL_LIST
}

// String (generated function)
func (m *MissionWritePartialList) String() string {
	return fmt.Sprintf(
		"&common.MissionWritePartialList{ StartIndex: %+v, EndIndex: %+v, TargetSystem: %+v, TargetComponent: %+v }",
		m.StartIndex,
		m.EndIndex,
		m.TargetSystem,
		m.TargetComponent,
	)
}

// Pack (generated function)
func (m *MissionWritePartialList) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 6)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.StartIndex))
	binary.LittleEndian.PutUint16(payload[2:], uint16(m.EndIndex))
	payload[4] = byte(m.TargetSystem)
	payload[5] = byte(m.TargetComponent)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *MissionWritePartialList) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 6 {
		payload = append(payload, mavlink.ZeroTail[:6-len(p.Payload)]...)
	}
	m.StartIndex = int16(binary.LittleEndian.Uint16(payload[0:]))
	m.EndIndex = int16(binary.LittleEndian.Uint16(payload[2:]))
	m.TargetSystem = uint8(payload[4])
	m.TargetComponent = uint8(payload[5])
	return nil
}

// MissionItem struct (generated typeinfo)
// Message encoding a mission item. This message is emitted to announce
//                 the presence of a mission item and to set a mission item on the system. The mission item can be either in x, y, z meters (type: LOCAL) or x:lat, y:lon, z:altitude. Local frame is Z-down, right handed (NED), global frame is Z-up, right handed (ENU). NaN may be used to indicate an optional/default value (e.g. to use the system's current latitude or yaw rather than a specific value). See also https://mavlink.io/en/services/mission.html.
type MissionItem struct {
	Param1          float32   // PARAM1, see MAV_CMD enum
	Param2          float32   // PARAM2, see MAV_CMD enum
	Param3          float32   // PARAM3, see MAV_CMD enum
	Param4          float32   // PARAM4, see MAV_CMD enum
	X               float32   // PARAM5 / local: X coordinate, global: latitude
	Y               float32   // PARAM6 / local: Y coordinate, global: longitude
	Z               float32   // PARAM7 / local: Z coordinate, global: altitude (relative or absolute, depending on frame).
	Seq             uint16    // Sequence
	Command         MAV_CMD   `gotype:"uint16"` // The scheduled action for the waypoint.
	TargetSystem    uint8     // System ID
	TargetComponent uint8     // Component ID
	Frame           MAV_FRAME `gotype:"uint8"` // The coordinate system of the waypoint.
	Current         uint8     // false:0, true:1
	Autocontinue    uint8     // Autocontinue to next waypoint
}

// MsgID (generated function)
func (m *MissionItem) MsgID() mavlink.MessageID {
	return MSG_ID_MISSION_ITEM
}

// String (generated function)
func (m *MissionItem) String() string {
	return fmt.Sprintf(
		"&common.MissionItem{ Param1: %+v, Param2: %+v, Param3: %+v, Param4: %+v, X: %+v, Y: %+v, Z: %+v, Seq: %+v, Command: %+v, TargetSystem: %+v, TargetComponent: %+v, Frame: %+v, Current: %+v, Autocontinue: %+v }",
		m.Param1,
		m.Param2,
		m.Param3,
		m.Param4,
		m.X,
		m.Y,
		m.Z,
		m.Seq,
		m.Command,
		m.TargetSystem,
		m.TargetComponent,
		m.Frame,
		m.Current,
		m.Autocontinue,
	)
}

// Pack (generated function)
func (m *MissionItem) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 37)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(m.Param1))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(m.Param2))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.Param3))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.Param4))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(m.X))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(m.Y))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(m.Z))
	binary.LittleEndian.PutUint16(payload[28:], uint16(m.Seq))
	binary.LittleEndian.PutUint16(payload[30:], uint16(m.Command))
	payload[32] = byte(m.TargetSystem)
	payload[33] = byte(m.TargetComponent)
	payload[34] = byte(m.Frame)
	payload[35] = byte(m.Current)
	payload[36] = byte(m.Autocontinue)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *MissionItem) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 37 {
		payload = append(payload, mavlink.ZeroTail[:37-len(p.Payload)]...)
	}
	m.Param1 = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.Param2 = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	m.Param3 = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.Param4 = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.X = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	m.Y = math.Float32frombits(binary.LittleEndian.Uint32(payload[20:]))
	m.Z = math.Float32frombits(binary.LittleEndian.Uint32(payload[24:]))
	m.Seq = uint16(binary.LittleEndian.Uint16(payload[28:]))
	m.Command = MAV_CMD(binary.LittleEndian.Uint16(payload[30:]))
	m.TargetSystem = uint8(payload[32])
	m.TargetComponent = uint8(payload[33])
	m.Frame = MAV_FRAME(payload[34])
	m.Current = uint8(payload[35])
	m.Autocontinue = uint8(payload[36])
	return nil
}

// MissionRequest struct (generated typeinfo)
// Request the information of the mission item with the sequence number seq. The response of the system to this message should be a MISSION_ITEM message. https://mavlink.io/en/services/mission.html
type MissionRequest struct {
	Seq             uint16 // Sequence
	TargetSystem    uint8  // System ID
	TargetComponent uint8  // Component ID
}

// MsgID (generated function)
func (m *MissionRequest) MsgID() mavlink.MessageID {
	return MSG_ID_MISSION_REQUEST
}

// String (generated function)
func (m *MissionRequest) String() string {
	return fmt.Sprintf(
		"&common.MissionRequest{ Seq: %+v, TargetSystem: %+v, TargetComponent: %+v }",
		m.Seq,
		m.TargetSystem,
		m.TargetComponent,
	)
}

// Pack (generated function)
func (m *MissionRequest) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 4)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.Seq))
	payload[2] = byte(m.TargetSystem)
	payload[3] = byte(m.TargetComponent)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *MissionRequest) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 4 {
		payload = append(payload, mavlink.ZeroTail[:4-len(p.Payload)]...)
	}
	m.Seq = uint16(binary.LittleEndian.Uint16(payload[0:]))
	m.TargetSystem = uint8(payload[2])
	m.TargetComponent = uint8(payload[3])
	return nil
}

// MissionSetCurrent struct (generated typeinfo)
// Set the mission item with sequence number seq as current item. This means that the MAV will continue to this mission item on the shortest path (not following the mission items in-between).
type MissionSetCurrent struct {
	Seq             uint16 // Sequence
	TargetSystem    uint8  // System ID
	TargetComponent uint8  // Component ID
}

// MsgID (generated function)
func (m *MissionSetCurrent) MsgID() mavlink.MessageID {
	return MSG_ID_MISSION_SET_CURRENT
}

// String (generated function)
func (m *MissionSetCurrent) String() string {
	return fmt.Sprintf(
		"&common.MissionSetCurrent{ Seq: %+v, TargetSystem: %+v, TargetComponent: %+v }",
		m.Seq,
		m.TargetSystem,
		m.TargetComponent,
	)
}

// Pack (generated function)
func (m *MissionSetCurrent) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 4)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.Seq))
	payload[2] = byte(m.TargetSystem)
	payload[3] = byte(m.TargetComponent)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *MissionSetCurrent) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 4 {
		payload = append(payload, mavlink.ZeroTail[:4-len(p.Payload)]...)
	}
	m.Seq = uint16(binary.LittleEndian.Uint16(payload[0:]))
	m.TargetSystem = uint8(payload[2])
	m.TargetComponent = uint8(payload[3])
	return nil
}

// MissionCurrent struct (generated typeinfo)
// Message that announces the sequence number of the current active mission item. The MAV will fly towards this mission item.
type MissionCurrent struct {
	Seq uint16 // Sequence
}

// MsgID (generated function)
func (m *MissionCurrent) MsgID() mavlink.MessageID {
	return MSG_ID_MISSION_CURRENT
}

// String (generated function)
func (m *MissionCurrent) String() string {
	return fmt.Sprintf(
		"&common.MissionCurrent{ Seq: %+v }",
		m.Seq,
	)
}

// Pack (generated function)
func (m *MissionCurrent) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 2)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.Seq))
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *MissionCurrent) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 2 {
		payload = append(payload, mavlink.ZeroTail[:2-len(p.Payload)]...)
	}
	m.Seq = uint16(binary.LittleEndian.Uint16(payload[0:]))
	return nil
}

// MissionRequestList struct (generated typeinfo)
// Request the overall list of mission items from the system/component.
type MissionRequestList struct {
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
}

// MsgID (generated function)
func (m *MissionRequestList) MsgID() mavlink.MessageID {
	return MSG_ID_MISSION_REQUEST_LIST
}

// String (generated function)
func (m *MissionRequestList) String() string {
	return fmt.Sprintf(
		"&common.MissionRequestList{ TargetSystem: %+v, TargetComponent: %+v }",
		m.TargetSystem,
		m.TargetComponent,
	)
}

// Pack (generated function)
func (m *MissionRequestList) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 2)
	payload[0] = byte(m.TargetSystem)
	payload[1] = byte(m.TargetComponent)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *MissionRequestList) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 2 {
		payload = append(payload, mavlink.ZeroTail[:2-len(p.Payload)]...)
	}
	m.TargetSystem = uint8(payload[0])
	m.TargetComponent = uint8(payload[1])
	return nil
}

// MissionCount struct (generated typeinfo)
// This message is emitted as response to MISSION_REQUEST_LIST by the MAV and to initiate a write transaction. The GCS can then request the individual mission item based on the knowledge of the total number of waypoints.
type MissionCount struct {
	Count           uint16 // Number of mission items in the sequence
	TargetSystem    uint8  // System ID
	TargetComponent uint8  // Component ID
}

// MsgID (generated function)
func (m *MissionCount) MsgID() mavlink.MessageID {
	return MSG_ID_MISSION_COUNT
}

// String (generated function)
func (m *MissionCount) String() string {
	return fmt.Sprintf(
		"&common.MissionCount{ Count: %+v, TargetSystem: %+v, TargetComponent: %+v }",
		m.Count,
		m.TargetSystem,
		m.TargetComponent,
	)
}

// Pack (generated function)
func (m *MissionCount) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 4)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.Count))
	payload[2] = byte(m.TargetSystem)
	payload[3] = byte(m.TargetComponent)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *MissionCount) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 4 {
		payload = append(payload, mavlink.ZeroTail[:4-len(p.Payload)]...)
	}
	m.Count = uint16(binary.LittleEndian.Uint16(payload[0:]))
	m.TargetSystem = uint8(payload[2])
	m.TargetComponent = uint8(payload[3])
	return nil
}

// MissionClearAll struct (generated typeinfo)
// Delete all mission items at once.
type MissionClearAll struct {
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
}

// MsgID (generated function)
func (m *MissionClearAll) MsgID() mavlink.MessageID {
	return MSG_ID_MISSION_CLEAR_ALL
}

// String (generated function)
func (m *MissionClearAll) String() string {
	return fmt.Sprintf(
		"&common.MissionClearAll{ TargetSystem: %+v, TargetComponent: %+v }",
		m.TargetSystem,
		m.TargetComponent,
	)
}

// Pack (generated function)
func (m *MissionClearAll) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 2)
	payload[0] = byte(m.TargetSystem)
	payload[1] = byte(m.TargetComponent)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *MissionClearAll) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 2 {
		payload = append(payload, mavlink.ZeroTail[:2-len(p.Payload)]...)
	}
	m.TargetSystem = uint8(payload[0])
	m.TargetComponent = uint8(payload[1])
	return nil
}

// MissionItemReached struct (generated typeinfo)
// A certain mission item has been reached. The system will either hold this position (or circle on the orbit) or (if the autocontinue on the WP was set) continue to the next waypoint.
type MissionItemReached struct {
	Seq uint16 // Sequence
}

// MsgID (generated function)
func (m *MissionItemReached) MsgID() mavlink.MessageID {
	return MSG_ID_MISSION_ITEM_REACHED
}

// String (generated function)
func (m *MissionItemReached) String() string {
	return fmt.Sprintf(
		"&common.MissionItemReached{ Seq: %+v }",
		m.Seq,
	)
}

// Pack (generated function)
func (m *MissionItemReached) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 2)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.Seq))
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *MissionItemReached) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 2 {
		payload = append(payload, mavlink.ZeroTail[:2-len(p.Payload)]...)
	}
	m.Seq = uint16(binary.LittleEndian.Uint16(payload[0:]))
	return nil
}

// MissionAck struct (generated typeinfo)
// Acknowledgment message during waypoint handling. The type field states if this message is a positive ack (type=0) or if an error happened (type=non-zero).
type MissionAck struct {
	TargetSystem    uint8              // System ID
	TargetComponent uint8              // Component ID
	Type            MAV_MISSION_RESULT `gotype:"uint8"` // Mission result.
}

// MsgID (generated function)
func (m *MissionAck) MsgID() mavlink.MessageID {
	return MSG_ID_MISSION_ACK
}

// String (generated function)
func (m *MissionAck) String() string {
	return fmt.Sprintf(
		"&common.MissionAck{ TargetSystem: %+v, TargetComponent: %+v, Type: %+v }",
		m.TargetSystem,
		m.TargetComponent,
		m.Type,
	)
}

// Pack (generated function)
func (m *MissionAck) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 3)
	payload[0] = byte(m.TargetSystem)
	payload[1] = byte(m.TargetComponent)
	payload[2] = byte(m.Type)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *MissionAck) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 3 {
		payload = append(payload, mavlink.ZeroTail[:3-len(p.Payload)]...)
	}
	m.TargetSystem = uint8(payload[0])
	m.TargetComponent = uint8(payload[1])
	m.Type = MAV_MISSION_RESULT(payload[2])
	return nil
}

// SetGpsGlobalOrigin struct (generated typeinfo)
// Sets the GPS co-ordinates of the vehicle local origin (0,0,0) position. Vehicle should emit GPS_GLOBAL_ORIGIN irrespective of whether the origin is changed. This enables transform between the local coordinate frame and the global (GPS) coordinate frame, which may be necessary when (for example) indoor and outdoor settings are connected and the MAV should move from in- to outdoor.
type SetGpsGlobalOrigin struct {
	Latitude     int32 // Latitude (WGS84)
	Longitude    int32 // Longitude (WGS84)
	Altitude     int32 // Altitude (MSL). Positive for up.
	TargetSystem uint8 // System ID
}

// MsgID (generated function)
func (m *SetGpsGlobalOrigin) MsgID() mavlink.MessageID {
	return MSG_ID_SET_GPS_GLOBAL_ORIGIN
}

// String (generated function)
func (m *SetGpsGlobalOrigin) String() string {
	return fmt.Sprintf(
		"&common.SetGpsGlobalOrigin{ Latitude: %+v, Longitude: %+v, Altitude: %+v, TargetSystem: %+v }",
		m.Latitude,
		m.Longitude,
		m.Altitude,
		m.TargetSystem,
	)
}

// Pack (generated function)
func (m *SetGpsGlobalOrigin) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 13)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.Latitude))
	binary.LittleEndian.PutUint32(payload[4:], uint32(m.Longitude))
	binary.LittleEndian.PutUint32(payload[8:], uint32(m.Altitude))
	payload[12] = byte(m.TargetSystem)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *SetGpsGlobalOrigin) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 13 {
		payload = append(payload, mavlink.ZeroTail[:13-len(p.Payload)]...)
	}
	m.Latitude = int32(binary.LittleEndian.Uint32(payload[0:]))
	m.Longitude = int32(binary.LittleEndian.Uint32(payload[4:]))
	m.Altitude = int32(binary.LittleEndian.Uint32(payload[8:]))
	m.TargetSystem = uint8(payload[12])
	return nil
}

// GpsGlobalOrigin struct (generated typeinfo)
// Publishes the GPS co-ordinates of the vehicle local origin (0,0,0) position. Emitted whenever a new GPS-Local position mapping is requested or set - e.g. following SET_GPS_GLOBAL_ORIGIN message.
type GpsGlobalOrigin struct {
	Latitude  int32 // Latitude (WGS84)
	Longitude int32 // Longitude (WGS84)
	Altitude  int32 // Altitude (MSL). Positive for up.
}

// MsgID (generated function)
func (m *GpsGlobalOrigin) MsgID() mavlink.MessageID {
	return MSG_ID_GPS_GLOBAL_ORIGIN
}

// String (generated function)
func (m *GpsGlobalOrigin) String() string {
	return fmt.Sprintf(
		"&common.GpsGlobalOrigin{ Latitude: %+v, Longitude: %+v, Altitude: %+v }",
		m.Latitude,
		m.Longitude,
		m.Altitude,
	)
}

// Pack (generated function)
func (m *GpsGlobalOrigin) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 12)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.Latitude))
	binary.LittleEndian.PutUint32(payload[4:], uint32(m.Longitude))
	binary.LittleEndian.PutUint32(payload[8:], uint32(m.Altitude))
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *GpsGlobalOrigin) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 12 {
		payload = append(payload, mavlink.ZeroTail[:12-len(p.Payload)]...)
	}
	m.Latitude = int32(binary.LittleEndian.Uint32(payload[0:]))
	m.Longitude = int32(binary.LittleEndian.Uint32(payload[4:]))
	m.Altitude = int32(binary.LittleEndian.Uint32(payload[8:]))
	return nil
}

// ParamMapRc struct (generated typeinfo)
// Bind a RC channel to a parameter. The parameter should change according to the RC channel value.
type ParamMapRc struct {
	ParamValue0             float32  // Initial parameter value
	Scale                   float32  // Scale, maps the RC range [-1, 1] to a parameter value
	ParamValueMin           float32  // Minimum param value. The protocol does not define if this overwrites an onboard minimum value. (Depends on implementation)
	ParamValueMax           float32  // Maximum param value. The protocol does not define if this overwrites an onboard maximum value. (Depends on implementation)
	ParamIndex              int16    // Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignored), send -2 to disable any existing map for this rc_channel_index.
	TargetSystem            uint8    // System ID
	TargetComponent         uint8    // Component ID
	ParamID                 [16]byte // Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
	ParameterRcChannelIndex uint8    // Index of parameter RC channel. Not equal to the RC channel id. Typically corresponds to a potentiometer-knob on the RC.
}

// MsgID (generated function)
func (m *ParamMapRc) MsgID() mavlink.MessageID {
	return MSG_ID_PARAM_MAP_RC
}

// String (generated function)
func (m *ParamMapRc) String() string {
	return fmt.Sprintf(
		"&common.ParamMapRc{ ParamValue0: %+v, Scale: %+v, ParamValueMin: %+v, ParamValueMax: %+v, ParamIndex: %+v, TargetSystem: %+v, TargetComponent: %+v, ParamID: %0X (\"%s\"), ParameterRcChannelIndex: %+v }",
		m.ParamValue0,
		m.Scale,
		m.ParamValueMin,
		m.ParamValueMax,
		m.ParamIndex,
		m.TargetSystem,
		m.TargetComponent,
		m.ParamID, string(m.ParamID[:]),
		m.ParameterRcChannelIndex,
	)
}

// Pack (generated function)
func (m *ParamMapRc) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 37)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(m.ParamValue0))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(m.Scale))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.ParamValueMin))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.ParamValueMax))
	binary.LittleEndian.PutUint16(payload[16:], uint16(m.ParamIndex))
	payload[18] = byte(m.TargetSystem)
	payload[19] = byte(m.TargetComponent)
	copy(payload[20:], m.ParamID[:])
	payload[36] = byte(m.ParameterRcChannelIndex)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ParamMapRc) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 37 {
		payload = append(payload, mavlink.ZeroTail[:37-len(p.Payload)]...)
	}
	m.ParamValue0 = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.Scale = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	m.ParamValueMin = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.ParamValueMax = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.ParamIndex = int16(binary.LittleEndian.Uint16(payload[16:]))
	m.TargetSystem = uint8(payload[18])
	m.TargetComponent = uint8(payload[19])
	copy(m.ParamID[:], payload[20:36])
	m.ParameterRcChannelIndex = uint8(payload[36])
	return nil
}

// MissionRequestInt struct (generated typeinfo)
// Request the information of the mission item with the sequence number seq. The response of the system to this message should be a MISSION_ITEM_INT message. https://mavlink.io/en/services/mission.html
type MissionRequestInt struct {
	Seq             uint16 // Sequence
	TargetSystem    uint8  // System ID
	TargetComponent uint8  // Component ID
}

// MsgID (generated function)
func (m *MissionRequestInt) MsgID() mavlink.MessageID {
	return MSG_ID_MISSION_REQUEST_INT
}

// String (generated function)
func (m *MissionRequestInt) String() string {
	return fmt.Sprintf(
		"&common.MissionRequestInt{ Seq: %+v, TargetSystem: %+v, TargetComponent: %+v }",
		m.Seq,
		m.TargetSystem,
		m.TargetComponent,
	)
}

// Pack (generated function)
func (m *MissionRequestInt) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 4)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.Seq))
	payload[2] = byte(m.TargetSystem)
	payload[3] = byte(m.TargetComponent)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *MissionRequestInt) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 4 {
		payload = append(payload, mavlink.ZeroTail[:4-len(p.Payload)]...)
	}
	m.Seq = uint16(binary.LittleEndian.Uint16(payload[0:]))
	m.TargetSystem = uint8(payload[2])
	m.TargetComponent = uint8(payload[3])
	return nil
}

// MissionChanged struct (generated typeinfo)
// A broadcast message to notify any ground station or SDK if a mission, geofence or safe points have changed on the vehicle.
type MissionChanged struct {
	StartIndex   int16            // Start index for partial mission change (-1 for all items).
	EndIndex     int16            // End index of a partial mission change. -1 is a synonym for the last mission item (i.e. selects all items from start_index). Ignore field if start_index=-1.
	OriginSysid  uint8            // System ID of the author of the new mission.
	OriginCompid MAV_COMPONENT    `gotype:"uint8"` // Compnent ID of the author of the new mission.
	MissionType  MAV_MISSION_TYPE `gotype:"uint8"` // Mission type.
}

// MsgID (generated function)
func (m *MissionChanged) MsgID() mavlink.MessageID {
	return MSG_ID_MISSION_CHANGED
}

// String (generated function)
func (m *MissionChanged) String() string {
	return fmt.Sprintf(
		"&common.MissionChanged{ StartIndex: %+v, EndIndex: %+v, OriginSysid: %+v, OriginCompid: %+v, MissionType: %+v }",
		m.StartIndex,
		m.EndIndex,
		m.OriginSysid,
		m.OriginCompid,
		m.MissionType,
	)
}

// Pack (generated function)
func (m *MissionChanged) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 7)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.StartIndex))
	binary.LittleEndian.PutUint16(payload[2:], uint16(m.EndIndex))
	payload[4] = byte(m.OriginSysid)
	payload[5] = byte(m.OriginCompid)
	payload[6] = byte(m.MissionType)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *MissionChanged) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 7 {
		payload = append(payload, mavlink.ZeroTail[:7-len(p.Payload)]...)
	}
	m.StartIndex = int16(binary.LittleEndian.Uint16(payload[0:]))
	m.EndIndex = int16(binary.LittleEndian.Uint16(payload[2:]))
	m.OriginSysid = uint8(payload[4])
	m.OriginCompid = MAV_COMPONENT(payload[5])
	m.MissionType = MAV_MISSION_TYPE(payload[6])
	return nil
}

// SafetySetAllowedArea struct (generated typeinfo)
// Set a safety zone (volume), which is defined by two corners of a cube. This message can be used to tell the MAV which setpoints/waypoints to accept and which to reject. Safety areas are often enforced by national or competition regulations.
type SafetySetAllowedArea struct {
	P1x             float32   // x position 1 / Latitude 1
	P1y             float32   // y position 1 / Longitude 1
	P1z             float32   // z position 1 / Altitude 1
	P2x             float32   // x position 2 / Latitude 2
	P2y             float32   // y position 2 / Longitude 2
	P2z             float32   // z position 2 / Altitude 2
	TargetSystem    uint8     // System ID
	TargetComponent uint8     // Component ID
	Frame           MAV_FRAME `gotype:"uint8"` // Coordinate frame. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down.
}

// MsgID (generated function)
func (m *SafetySetAllowedArea) MsgID() mavlink.MessageID {
	return MSG_ID_SAFETY_SET_ALLOWED_AREA
}

// String (generated function)
func (m *SafetySetAllowedArea) String() string {
	return fmt.Sprintf(
		"&common.SafetySetAllowedArea{ P1x: %+v, P1y: %+v, P1z: %+v, P2x: %+v, P2y: %+v, P2z: %+v, TargetSystem: %+v, TargetComponent: %+v, Frame: %+v }",
		m.P1x,
		m.P1y,
		m.P1z,
		m.P2x,
		m.P2y,
		m.P2z,
		m.TargetSystem,
		m.TargetComponent,
		m.Frame,
	)
}

// Pack (generated function)
func (m *SafetySetAllowedArea) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 27)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(m.P1x))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(m.P1y))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.P1z))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.P2x))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(m.P2y))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(m.P2z))
	payload[24] = byte(m.TargetSystem)
	payload[25] = byte(m.TargetComponent)
	payload[26] = byte(m.Frame)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *SafetySetAllowedArea) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 27 {
		payload = append(payload, mavlink.ZeroTail[:27-len(p.Payload)]...)
	}
	m.P1x = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.P1y = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	m.P1z = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.P2x = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.P2y = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	m.P2z = math.Float32frombits(binary.LittleEndian.Uint32(payload[20:]))
	m.TargetSystem = uint8(payload[24])
	m.TargetComponent = uint8(payload[25])
	m.Frame = MAV_FRAME(payload[26])
	return nil
}

// SafetyAllowedArea struct (generated typeinfo)
// Read out the safety zone the MAV currently assumes.
type SafetyAllowedArea struct {
	P1x   float32   // x position 1 / Latitude 1
	P1y   float32   // y position 1 / Longitude 1
	P1z   float32   // z position 1 / Altitude 1
	P2x   float32   // x position 2 / Latitude 2
	P2y   float32   // y position 2 / Longitude 2
	P2z   float32   // z position 2 / Altitude 2
	Frame MAV_FRAME `gotype:"uint8"` // Coordinate frame. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down.
}

// MsgID (generated function)
func (m *SafetyAllowedArea) MsgID() mavlink.MessageID {
	return MSG_ID_SAFETY_ALLOWED_AREA
}

// String (generated function)
func (m *SafetyAllowedArea) String() string {
	return fmt.Sprintf(
		"&common.SafetyAllowedArea{ P1x: %+v, P1y: %+v, P1z: %+v, P2x: %+v, P2y: %+v, P2z: %+v, Frame: %+v }",
		m.P1x,
		m.P1y,
		m.P1z,
		m.P2x,
		m.P2y,
		m.P2z,
		m.Frame,
	)
}

// Pack (generated function)
func (m *SafetyAllowedArea) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 25)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(m.P1x))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(m.P1y))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.P1z))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.P2x))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(m.P2y))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(m.P2z))
	payload[24] = byte(m.Frame)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *SafetyAllowedArea) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 25 {
		payload = append(payload, mavlink.ZeroTail[:25-len(p.Payload)]...)
	}
	m.P1x = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.P1y = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	m.P1z = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.P2x = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.P2y = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	m.P2z = math.Float32frombits(binary.LittleEndian.Uint32(payload[20:]))
	m.Frame = MAV_FRAME(payload[24])
	return nil
}

// AttitudeQuaternionCov struct (generated typeinfo)
// The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right), expressed as quaternion. Quaternion order is w, x, y, z and a zero rotation would be expressed as (1 0 0 0).
type AttitudeQuaternionCov struct {
	TimeUsec   uint64     // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
	Q          [4]float32 // Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation)
	Rollspeed  float32    // Roll angular speed
	Pitchspeed float32    // Pitch angular speed
	Yawspeed   float32    // Yaw angular speed
	Covariance [9]float32 // Row-major representation of a 3x3 attitude covariance matrix (states: roll, pitch, yaw; first three entries are the first ROW, next three entries are the second row, etc.). If unknown, assign NaN value to first element in the array.
}

// MsgID (generated function)
func (m *AttitudeQuaternionCov) MsgID() mavlink.MessageID {
	return MSG_ID_ATTITUDE_QUATERNION_COV
}

// String (generated function)
func (m *AttitudeQuaternionCov) String() string {
	return fmt.Sprintf(
		"&common.AttitudeQuaternionCov{ TimeUsec: %+v, Q: %+v, Rollspeed: %+v, Pitchspeed: %+v, Yawspeed: %+v, Covariance: %+v }",
		m.TimeUsec,
		m.Q,
		m.Rollspeed,
		m.Pitchspeed,
		m.Yawspeed,
		m.Covariance,
	)
}

// Pack (generated function)
func (m *AttitudeQuaternionCov) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 72)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.TimeUsec))
	for i, v := range m.Q {
		binary.LittleEndian.PutUint32(payload[8+i*4:], math.Float32bits(v))
	}
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(m.Rollspeed))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(m.Pitchspeed))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(m.Yawspeed))
	for i, v := range m.Covariance {
		binary.LittleEndian.PutUint32(payload[36+i*4:], math.Float32bits(v))
	}
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *AttitudeQuaternionCov) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 72 {
		payload = append(payload, mavlink.ZeroTail[:72-len(p.Payload)]...)
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(payload[0:]))
	for i := 0; i < len(m.Q); i++ {
		m.Q[i] = math.Float32frombits(binary.LittleEndian.Uint32(payload[8+i*4:]))
	}
	m.Rollspeed = math.Float32frombits(binary.LittleEndian.Uint32(payload[24:]))
	m.Pitchspeed = math.Float32frombits(binary.LittleEndian.Uint32(payload[28:]))
	m.Yawspeed = math.Float32frombits(binary.LittleEndian.Uint32(payload[32:]))
	for i := 0; i < len(m.Covariance); i++ {
		m.Covariance[i] = math.Float32frombits(binary.LittleEndian.Uint32(payload[36+i*4:]))
	}
	return nil
}

// NavControllerOutput struct (generated typeinfo)
// The state of the fixed wing navigation and position controller.
type NavControllerOutput struct {
	NavRoll       float32 // Current desired roll
	NavPitch      float32 // Current desired pitch
	AltError      float32 // Current altitude error
	AspdError     float32 // Current airspeed error
	XtrackError   float32 // Current crosstrack error on x-y plane
	NavBearing    int16   // Current desired heading
	TargetBearing int16   // Bearing to current waypoint/target
	WpDist        uint16  // Distance to active waypoint
}

// MsgID (generated function)
func (m *NavControllerOutput) MsgID() mavlink.MessageID {
	return MSG_ID_NAV_CONTROLLER_OUTPUT
}

// String (generated function)
func (m *NavControllerOutput) String() string {
	return fmt.Sprintf(
		"&common.NavControllerOutput{ NavRoll: %+v, NavPitch: %+v, AltError: %+v, AspdError: %+v, XtrackError: %+v, NavBearing: %+v, TargetBearing: %+v, WpDist: %+v }",
		m.NavRoll,
		m.NavPitch,
		m.AltError,
		m.AspdError,
		m.XtrackError,
		m.NavBearing,
		m.TargetBearing,
		m.WpDist,
	)
}

// Pack (generated function)
func (m *NavControllerOutput) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 26)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(m.NavRoll))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(m.NavPitch))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.AltError))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.AspdError))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(m.XtrackError))
	binary.LittleEndian.PutUint16(payload[20:], uint16(m.NavBearing))
	binary.LittleEndian.PutUint16(payload[22:], uint16(m.TargetBearing))
	binary.LittleEndian.PutUint16(payload[24:], uint16(m.WpDist))
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *NavControllerOutput) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 26 {
		payload = append(payload, mavlink.ZeroTail[:26-len(p.Payload)]...)
	}
	m.NavRoll = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.NavPitch = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	m.AltError = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.AspdError = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.XtrackError = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	m.NavBearing = int16(binary.LittleEndian.Uint16(payload[20:]))
	m.TargetBearing = int16(binary.LittleEndian.Uint16(payload[22:]))
	m.WpDist = uint16(binary.LittleEndian.Uint16(payload[24:]))
	return nil
}

// GlobalPositionIntCov struct (generated typeinfo)
// The filtered global position (e.g. fused GPS and accelerometers). The position is in GPS-frame (right-handed, Z-up). It  is designed as scaled integer message since the resolution of float is not sufficient. NOTE: This message is intended for onboard networks / companion computers and higher-bandwidth links and optimized for accuracy and completeness. Please use the GLOBAL_POSITION_INT message for a minimal subset.
type GlobalPositionIntCov struct {
	TimeUsec      uint64             // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
	Lat           int32              // Latitude
	Lon           int32              // Longitude
	Alt           int32              // Altitude in meters above MSL
	RelativeAlt   int32              // Altitude above ground
	Vx            float32            // Ground X Speed (Latitude)
	Vy            float32            // Ground Y Speed (Longitude)
	Vz            float32            // Ground Z Speed (Altitude)
	Covariance    [36]float32        // Row-major representation of a 6x6 position and velocity 6x6 cross-covariance matrix (states: lat, lon, alt, vx, vy, vz; first six entries are the first ROW, next six entries are the second row, etc.). If unknown, assign NaN value to first element in the array.
	EstimatorType MAV_ESTIMATOR_TYPE `gotype:"uint8"` // Class id of the estimator this estimate originated from.
}

// MsgID (generated function)
func (m *GlobalPositionIntCov) MsgID() mavlink.MessageID {
	return MSG_ID_GLOBAL_POSITION_INT_COV
}

// String (generated function)
func (m *GlobalPositionIntCov) String() string {
	return fmt.Sprintf(
		"&common.GlobalPositionIntCov{ TimeUsec: %+v, Lat: %+v, Lon: %+v, Alt: %+v, RelativeAlt: %+v, Vx: %+v, Vy: %+v, Vz: %+v, Covariance: %+v, EstimatorType: %+v }",
		m.TimeUsec,
		m.Lat,
		m.Lon,
		m.Alt,
		m.RelativeAlt,
		m.Vx,
		m.Vy,
		m.Vz,
		m.Covariance,
		m.EstimatorType,
	)
}

// Pack (generated function)
func (m *GlobalPositionIntCov) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 181)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.TimeUsec))
	binary.LittleEndian.PutUint32(payload[8:], uint32(m.Lat))
	binary.LittleEndian.PutUint32(payload[12:], uint32(m.Lon))
	binary.LittleEndian.PutUint32(payload[16:], uint32(m.Alt))
	binary.LittleEndian.PutUint32(payload[20:], uint32(m.RelativeAlt))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(m.Vx))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(m.Vy))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(m.Vz))
	for i, v := range m.Covariance {
		binary.LittleEndian.PutUint32(payload[36+i*4:], math.Float32bits(v))
	}
	payload[180] = byte(m.EstimatorType)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *GlobalPositionIntCov) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 181 {
		payload = append(payload, mavlink.ZeroTail[:181-len(p.Payload)]...)
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(payload[0:]))
	m.Lat = int32(binary.LittleEndian.Uint32(payload[8:]))
	m.Lon = int32(binary.LittleEndian.Uint32(payload[12:]))
	m.Alt = int32(binary.LittleEndian.Uint32(payload[16:]))
	m.RelativeAlt = int32(binary.LittleEndian.Uint32(payload[20:]))
	m.Vx = math.Float32frombits(binary.LittleEndian.Uint32(payload[24:]))
	m.Vy = math.Float32frombits(binary.LittleEndian.Uint32(payload[28:]))
	m.Vz = math.Float32frombits(binary.LittleEndian.Uint32(payload[32:]))
	for i := 0; i < len(m.Covariance); i++ {
		m.Covariance[i] = math.Float32frombits(binary.LittleEndian.Uint32(payload[36+i*4:]))
	}
	m.EstimatorType = MAV_ESTIMATOR_TYPE(payload[180])
	return nil
}

// LocalPositionNedCov struct (generated typeinfo)
// The filtered local position (e.g. fused computer vision and accelerometers). Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down convention)
type LocalPositionNedCov struct {
	TimeUsec      uint64             // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
	X             float32            // X Position
	Y             float32            // Y Position
	Z             float32            // Z Position
	Vx            float32            // X Speed
	Vy            float32            // Y Speed
	Vz            float32            // Z Speed
	Ax            float32            // X Acceleration
	Ay            float32            // Y Acceleration
	Az            float32            // Z Acceleration
	Covariance    [45]float32        // Row-major representation of position, velocity and acceleration 9x9 cross-covariance matrix upper right triangle (states: x, y, z, vx, vy, vz, ax, ay, az; first nine entries are the first ROW, next eight entries are the second row, etc.). If unknown, assign NaN value to first element in the array.
	EstimatorType MAV_ESTIMATOR_TYPE `gotype:"uint8"` // Class id of the estimator this estimate originated from.
}

// MsgID (generated function)
func (m *LocalPositionNedCov) MsgID() mavlink.MessageID {
	return MSG_ID_LOCAL_POSITION_NED_COV
}

// String (generated function)
func (m *LocalPositionNedCov) String() string {
	return fmt.Sprintf(
		"&common.LocalPositionNedCov{ TimeUsec: %+v, X: %+v, Y: %+v, Z: %+v, Vx: %+v, Vy: %+v, Vz: %+v, Ax: %+v, Ay: %+v, Az: %+v, Covariance: %+v, EstimatorType: %+v }",
		m.TimeUsec,
		m.X,
		m.Y,
		m.Z,
		m.Vx,
		m.Vy,
		m.Vz,
		m.Ax,
		m.Ay,
		m.Az,
		m.Covariance,
		m.EstimatorType,
	)
}

// Pack (generated function)
func (m *LocalPositionNedCov) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 225)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.TimeUsec))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.X))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.Y))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(m.Z))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(m.Vx))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(m.Vy))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(m.Vz))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(m.Ax))
	binary.LittleEndian.PutUint32(payload[36:], math.Float32bits(m.Ay))
	binary.LittleEndian.PutUint32(payload[40:], math.Float32bits(m.Az))
	for i, v := range m.Covariance {
		binary.LittleEndian.PutUint32(payload[44+i*4:], math.Float32bits(v))
	}
	payload[224] = byte(m.EstimatorType)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *LocalPositionNedCov) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 225 {
		payload = append(payload, mavlink.ZeroTail[:225-len(p.Payload)]...)
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(payload[0:]))
	m.X = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.Y = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.Z = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	m.Vx = math.Float32frombits(binary.LittleEndian.Uint32(payload[20:]))
	m.Vy = math.Float32frombits(binary.LittleEndian.Uint32(payload[24:]))
	m.Vz = math.Float32frombits(binary.LittleEndian.Uint32(payload[28:]))
	m.Ax = math.Float32frombits(binary.LittleEndian.Uint32(payload[32:]))
	m.Ay = math.Float32frombits(binary.LittleEndian.Uint32(payload[36:]))
	m.Az = math.Float32frombits(binary.LittleEndian.Uint32(payload[40:]))
	for i := 0; i < len(m.Covariance); i++ {
		m.Covariance[i] = math.Float32frombits(binary.LittleEndian.Uint32(payload[44+i*4:]))
	}
	m.EstimatorType = MAV_ESTIMATOR_TYPE(payload[224])
	return nil
}

// RcChannels struct (generated typeinfo)
// The PPM values of the RC channels received. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%.  A value of UINT16_MAX implies the channel is unused. Individual receivers/transmitters might violate this specification.
type RcChannels struct {
	TimeBootMs uint32 // Timestamp (time since system boot).
	Chan1Raw   uint16 // RC channel 1 value.
	Chan2Raw   uint16 // RC channel 2 value.
	Chan3Raw   uint16 // RC channel 3 value.
	Chan4Raw   uint16 // RC channel 4 value.
	Chan5Raw   uint16 // RC channel 5 value.
	Chan6Raw   uint16 // RC channel 6 value.
	Chan7Raw   uint16 // RC channel 7 value.
	Chan8Raw   uint16 // RC channel 8 value.
	Chan9Raw   uint16 // RC channel 9 value.
	Chan10Raw  uint16 // RC channel 10 value.
	Chan11Raw  uint16 // RC channel 11 value.
	Chan12Raw  uint16 // RC channel 12 value.
	Chan13Raw  uint16 // RC channel 13 value.
	Chan14Raw  uint16 // RC channel 14 value.
	Chan15Raw  uint16 // RC channel 15 value.
	Chan16Raw  uint16 // RC channel 16 value.
	Chan17Raw  uint16 // RC channel 17 value.
	Chan18Raw  uint16 // RC channel 18 value.
	Chancount  uint8  // Total number of RC channels being received. This can be larger than 18, indicating that more channels are available but not given in this message. This value should be 0 when no RC channels are available.
	Rssi       uint8  // Receive signal strength indicator in device-dependent units/scale. Values: [0-254], 255: invalid/unknown.
}

// MsgID (generated function)
func (m *RcChannels) MsgID() mavlink.MessageID {
	return MSG_ID_RC_CHANNELS
}

// String (generated function)
func (m *RcChannels) String() string {
	return fmt.Sprintf(
		"&common.RcChannels{ TimeBootMs: %+v, Chan1Raw: %+v, Chan2Raw: %+v, Chan3Raw: %+v, Chan4Raw: %+v, Chan5Raw: %+v, Chan6Raw: %+v, Chan7Raw: %+v, Chan8Raw: %+v, Chan9Raw: %+v, Chan10Raw: %+v, Chan11Raw: %+v, Chan12Raw: %+v, Chan13Raw: %+v, Chan14Raw: %+v, Chan15Raw: %+v, Chan16Raw: %+v, Chan17Raw: %+v, Chan18Raw: %+v, Chancount: %+v, Rssi: %+v }",
		m.TimeBootMs,
		m.Chan1Raw,
		m.Chan2Raw,
		m.Chan3Raw,
		m.Chan4Raw,
		m.Chan5Raw,
		m.Chan6Raw,
		m.Chan7Raw,
		m.Chan8Raw,
		m.Chan9Raw,
		m.Chan10Raw,
		m.Chan11Raw,
		m.Chan12Raw,
		m.Chan13Raw,
		m.Chan14Raw,
		m.Chan15Raw,
		m.Chan16Raw,
		m.Chan17Raw,
		m.Chan18Raw,
		m.Chancount,
		m.Rssi,
	)
}

// Pack (generated function)
func (m *RcChannels) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 42)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.TimeBootMs))
	binary.LittleEndian.PutUint16(payload[4:], uint16(m.Chan1Raw))
	binary.LittleEndian.PutUint16(payload[6:], uint16(m.Chan2Raw))
	binary.LittleEndian.PutUint16(payload[8:], uint16(m.Chan3Raw))
	binary.LittleEndian.PutUint16(payload[10:], uint16(m.Chan4Raw))
	binary.LittleEndian.PutUint16(payload[12:], uint16(m.Chan5Raw))
	binary.LittleEndian.PutUint16(payload[14:], uint16(m.Chan6Raw))
	binary.LittleEndian.PutUint16(payload[16:], uint16(m.Chan7Raw))
	binary.LittleEndian.PutUint16(payload[18:], uint16(m.Chan8Raw))
	binary.LittleEndian.PutUint16(payload[20:], uint16(m.Chan9Raw))
	binary.LittleEndian.PutUint16(payload[22:], uint16(m.Chan10Raw))
	binary.LittleEndian.PutUint16(payload[24:], uint16(m.Chan11Raw))
	binary.LittleEndian.PutUint16(payload[26:], uint16(m.Chan12Raw))
	binary.LittleEndian.PutUint16(payload[28:], uint16(m.Chan13Raw))
	binary.LittleEndian.PutUint16(payload[30:], uint16(m.Chan14Raw))
	binary.LittleEndian.PutUint16(payload[32:], uint16(m.Chan15Raw))
	binary.LittleEndian.PutUint16(payload[34:], uint16(m.Chan16Raw))
	binary.LittleEndian.PutUint16(payload[36:], uint16(m.Chan17Raw))
	binary.LittleEndian.PutUint16(payload[38:], uint16(m.Chan18Raw))
	payload[40] = byte(m.Chancount)
	payload[41] = byte(m.Rssi)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *RcChannels) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 42 {
		payload = append(payload, mavlink.ZeroTail[:42-len(p.Payload)]...)
	}
	m.TimeBootMs = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.Chan1Raw = uint16(binary.LittleEndian.Uint16(payload[4:]))
	m.Chan2Raw = uint16(binary.LittleEndian.Uint16(payload[6:]))
	m.Chan3Raw = uint16(binary.LittleEndian.Uint16(payload[8:]))
	m.Chan4Raw = uint16(binary.LittleEndian.Uint16(payload[10:]))
	m.Chan5Raw = uint16(binary.LittleEndian.Uint16(payload[12:]))
	m.Chan6Raw = uint16(binary.LittleEndian.Uint16(payload[14:]))
	m.Chan7Raw = uint16(binary.LittleEndian.Uint16(payload[16:]))
	m.Chan8Raw = uint16(binary.LittleEndian.Uint16(payload[18:]))
	m.Chan9Raw = uint16(binary.LittleEndian.Uint16(payload[20:]))
	m.Chan10Raw = uint16(binary.LittleEndian.Uint16(payload[22:]))
	m.Chan11Raw = uint16(binary.LittleEndian.Uint16(payload[24:]))
	m.Chan12Raw = uint16(binary.LittleEndian.Uint16(payload[26:]))
	m.Chan13Raw = uint16(binary.LittleEndian.Uint16(payload[28:]))
	m.Chan14Raw = uint16(binary.LittleEndian.Uint16(payload[30:]))
	m.Chan15Raw = uint16(binary.LittleEndian.Uint16(payload[32:]))
	m.Chan16Raw = uint16(binary.LittleEndian.Uint16(payload[34:]))
	m.Chan17Raw = uint16(binary.LittleEndian.Uint16(payload[36:]))
	m.Chan18Raw = uint16(binary.LittleEndian.Uint16(payload[38:]))
	m.Chancount = uint8(payload[40])
	m.Rssi = uint8(payload[41])
	return nil
}

// RequestDataStream struct (generated typeinfo)
// Request a data stream.
type RequestDataStream struct {
	ReqMessageRate  uint16 // The requested message rate
	TargetSystem    uint8  // The target requested to send the message stream.
	TargetComponent uint8  // The target requested to send the message stream.
	ReqStreamID     uint8  // The ID of the requested data stream
	StartStop       uint8  // 1 to start sending, 0 to stop sending.
}

// MsgID (generated function)
func (m *RequestDataStream) MsgID() mavlink.MessageID {
	return MSG_ID_REQUEST_DATA_STREAM
}

// String (generated function)
func (m *RequestDataStream) String() string {
	return fmt.Sprintf(
		"&common.RequestDataStream{ ReqMessageRate: %+v, TargetSystem: %+v, TargetComponent: %+v, ReqStreamID: %+v, StartStop: %+v }",
		m.ReqMessageRate,
		m.TargetSystem,
		m.TargetComponent,
		m.ReqStreamID,
		m.StartStop,
	)
}

// Pack (generated function)
func (m *RequestDataStream) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 6)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.ReqMessageRate))
	payload[2] = byte(m.TargetSystem)
	payload[3] = byte(m.TargetComponent)
	payload[4] = byte(m.ReqStreamID)
	payload[5] = byte(m.StartStop)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *RequestDataStream) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 6 {
		payload = append(payload, mavlink.ZeroTail[:6-len(p.Payload)]...)
	}
	m.ReqMessageRate = uint16(binary.LittleEndian.Uint16(payload[0:]))
	m.TargetSystem = uint8(payload[2])
	m.TargetComponent = uint8(payload[3])
	m.ReqStreamID = uint8(payload[4])
	m.StartStop = uint8(payload[5])
	return nil
}

// DataStream struct (generated typeinfo)
// Data stream status information.
type DataStream struct {
	MessageRate uint16 // The message rate
	StreamID    uint8  // The ID of the requested data stream
	OnOff       uint8  // 1 stream is enabled, 0 stream is stopped.
}

// MsgID (generated function)
func (m *DataStream) MsgID() mavlink.MessageID {
	return MSG_ID_DATA_STREAM
}

// String (generated function)
func (m *DataStream) String() string {
	return fmt.Sprintf(
		"&common.DataStream{ MessageRate: %+v, StreamID: %+v, OnOff: %+v }",
		m.MessageRate,
		m.StreamID,
		m.OnOff,
	)
}

// Pack (generated function)
func (m *DataStream) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 4)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.MessageRate))
	payload[2] = byte(m.StreamID)
	payload[3] = byte(m.OnOff)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *DataStream) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 4 {
		payload = append(payload, mavlink.ZeroTail[:4-len(p.Payload)]...)
	}
	m.MessageRate = uint16(binary.LittleEndian.Uint16(payload[0:]))
	m.StreamID = uint8(payload[2])
	m.OnOff = uint8(payload[3])
	return nil
}

// ManualControl struct (generated typeinfo)
// This message provides an API for manually controlling the vehicle using standard joystick axes nomenclature, along with a joystick-like input device. Unused axes can be disabled an buttons are also transmit as boolean values of their
type ManualControl struct {
	X       int16  // X-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to forward(1000)-backward(-1000) movement on a joystick and the pitch of a vehicle.
	Y       int16  // Y-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to left(-1000)-right(1000) movement on a joystick and the roll of a vehicle.
	Z       int16  // Z-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to a separate slider movement with maximum being 1000 and minimum being -1000 on a joystick and the thrust of a vehicle. Positive values are positive thrust, negative values are negative thrust.
	R       int16  // R-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to a twisting of the joystick, with counter-clockwise being 1000 and clockwise being -1000, and the yaw of a vehicle.
	Buttons uint16 // A bitfield corresponding to the joystick buttons' current state, 1 for pressed, 0 for released. The lowest bit corresponds to Button 1.
	Target  uint8  // The system to be controlled.
}

// MsgID (generated function)
func (m *ManualControl) MsgID() mavlink.MessageID {
	return MSG_ID_MANUAL_CONTROL
}

// String (generated function)
func (m *ManualControl) String() string {
	return fmt.Sprintf(
		"&common.ManualControl{ X: %+v, Y: %+v, Z: %+v, R: %+v, Buttons: %+v, Target: %+v }",
		m.X,
		m.Y,
		m.Z,
		m.R,
		m.Buttons,
		m.Target,
	)
}

// Pack (generated function)
func (m *ManualControl) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 11)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.X))
	binary.LittleEndian.PutUint16(payload[2:], uint16(m.Y))
	binary.LittleEndian.PutUint16(payload[4:], uint16(m.Z))
	binary.LittleEndian.PutUint16(payload[6:], uint16(m.R))
	binary.LittleEndian.PutUint16(payload[8:], uint16(m.Buttons))
	payload[10] = byte(m.Target)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ManualControl) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 11 {
		payload = append(payload, mavlink.ZeroTail[:11-len(p.Payload)]...)
	}
	m.X = int16(binary.LittleEndian.Uint16(payload[0:]))
	m.Y = int16(binary.LittleEndian.Uint16(payload[2:]))
	m.Z = int16(binary.LittleEndian.Uint16(payload[4:]))
	m.R = int16(binary.LittleEndian.Uint16(payload[6:]))
	m.Buttons = uint16(binary.LittleEndian.Uint16(payload[8:]))
	m.Target = uint8(payload[10])
	return nil
}

// RcChannelsOverride struct (generated typeinfo)
// The RAW values of the RC channels sent to the MAV to override info received from the RC radio. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual receivers/transmitters might violate this specification.  Note carefully the semantic differences between the first 8 channels and the subsequent channels
type RcChannelsOverride struct {
	Chan1Raw        uint16 // RC channel 1 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this channel back to the RC radio.
	Chan2Raw        uint16 // RC channel 2 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this channel back to the RC radio.
	Chan3Raw        uint16 // RC channel 3 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this channel back to the RC radio.
	Chan4Raw        uint16 // RC channel 4 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this channel back to the RC radio.
	Chan5Raw        uint16 // RC channel 5 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this channel back to the RC radio.
	Chan6Raw        uint16 // RC channel 6 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this channel back to the RC radio.
	Chan7Raw        uint16 // RC channel 7 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this channel back to the RC radio.
	Chan8Raw        uint16 // RC channel 8 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this channel back to the RC radio.
	TargetSystem    uint8  // System ID
	TargetComponent uint8  // Component ID
}

// MsgID (generated function)
func (m *RcChannelsOverride) MsgID() mavlink.MessageID {
	return MSG_ID_RC_CHANNELS_OVERRIDE
}

// String (generated function)
func (m *RcChannelsOverride) String() string {
	return fmt.Sprintf(
		"&common.RcChannelsOverride{ Chan1Raw: %+v, Chan2Raw: %+v, Chan3Raw: %+v, Chan4Raw: %+v, Chan5Raw: %+v, Chan6Raw: %+v, Chan7Raw: %+v, Chan8Raw: %+v, TargetSystem: %+v, TargetComponent: %+v }",
		m.Chan1Raw,
		m.Chan2Raw,
		m.Chan3Raw,
		m.Chan4Raw,
		m.Chan5Raw,
		m.Chan6Raw,
		m.Chan7Raw,
		m.Chan8Raw,
		m.TargetSystem,
		m.TargetComponent,
	)
}

// Pack (generated function)
func (m *RcChannelsOverride) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 18)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.Chan1Raw))
	binary.LittleEndian.PutUint16(payload[2:], uint16(m.Chan2Raw))
	binary.LittleEndian.PutUint16(payload[4:], uint16(m.Chan3Raw))
	binary.LittleEndian.PutUint16(payload[6:], uint16(m.Chan4Raw))
	binary.LittleEndian.PutUint16(payload[8:], uint16(m.Chan5Raw))
	binary.LittleEndian.PutUint16(payload[10:], uint16(m.Chan6Raw))
	binary.LittleEndian.PutUint16(payload[12:], uint16(m.Chan7Raw))
	binary.LittleEndian.PutUint16(payload[14:], uint16(m.Chan8Raw))
	payload[16] = byte(m.TargetSystem)
	payload[17] = byte(m.TargetComponent)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *RcChannelsOverride) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 18 {
		payload = append(payload, mavlink.ZeroTail[:18-len(p.Payload)]...)
	}
	m.Chan1Raw = uint16(binary.LittleEndian.Uint16(payload[0:]))
	m.Chan2Raw = uint16(binary.LittleEndian.Uint16(payload[2:]))
	m.Chan3Raw = uint16(binary.LittleEndian.Uint16(payload[4:]))
	m.Chan4Raw = uint16(binary.LittleEndian.Uint16(payload[6:]))
	m.Chan5Raw = uint16(binary.LittleEndian.Uint16(payload[8:]))
	m.Chan6Raw = uint16(binary.LittleEndian.Uint16(payload[10:]))
	m.Chan7Raw = uint16(binary.LittleEndian.Uint16(payload[12:]))
	m.Chan8Raw = uint16(binary.LittleEndian.Uint16(payload[14:]))
	m.TargetSystem = uint8(payload[16])
	m.TargetComponent = uint8(payload[17])
	return nil
}

// MissionItemInt struct (generated typeinfo)
// Message encoding a mission item. This message is emitted to announce
//                 the presence of a mission item and to set a mission item on the system. The mission item can be either in x, y, z meters (type: LOCAL) or x:lat, y:lon, z:altitude. Local frame is Z-down, right handed (NED), global frame is Z-up, right handed (ENU). NaN or INT32_MAX may be used in float/integer params (respectively) to indicate optional/default values (e.g. to use the component's current latitude, yaw rather than a specific value). See also https://mavlink.io/en/services/mission.html.
type MissionItemInt struct {
	Param1          float32   // PARAM1, see MAV_CMD enum
	Param2          float32   // PARAM2, see MAV_CMD enum
	Param3          float32   // PARAM3, see MAV_CMD enum
	Param4          float32   // PARAM4, see MAV_CMD enum
	X               int32     // PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7
	Y               int32     // PARAM6 / y position: local: x position in meters * 1e4, global: longitude in degrees *10^7
	Z               float32   // PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame.
	Seq             uint16    // Waypoint ID (sequence number). Starts at zero. Increases monotonically for each waypoint, no gaps in the sequence (0,1,2,3,4).
	Command         MAV_CMD   `gotype:"uint16"` // The scheduled action for the waypoint.
	TargetSystem    uint8     // System ID
	TargetComponent uint8     // Component ID
	Frame           MAV_FRAME `gotype:"uint8"` // The coordinate system of the waypoint.
	Current         uint8     // false:0, true:1
	Autocontinue    uint8     // Autocontinue to next waypoint
}

// MsgID (generated function)
func (m *MissionItemInt) MsgID() mavlink.MessageID {
	return MSG_ID_MISSION_ITEM_INT
}

// String (generated function)
func (m *MissionItemInt) String() string {
	return fmt.Sprintf(
		"&common.MissionItemInt{ Param1: %+v, Param2: %+v, Param3: %+v, Param4: %+v, X: %+v, Y: %+v, Z: %+v, Seq: %+v, Command: %+v, TargetSystem: %+v, TargetComponent: %+v, Frame: %+v, Current: %+v, Autocontinue: %+v }",
		m.Param1,
		m.Param2,
		m.Param3,
		m.Param4,
		m.X,
		m.Y,
		m.Z,
		m.Seq,
		m.Command,
		m.TargetSystem,
		m.TargetComponent,
		m.Frame,
		m.Current,
		m.Autocontinue,
	)
}

// Pack (generated function)
func (m *MissionItemInt) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 37)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(m.Param1))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(m.Param2))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.Param3))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.Param4))
	binary.LittleEndian.PutUint32(payload[16:], uint32(m.X))
	binary.LittleEndian.PutUint32(payload[20:], uint32(m.Y))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(m.Z))
	binary.LittleEndian.PutUint16(payload[28:], uint16(m.Seq))
	binary.LittleEndian.PutUint16(payload[30:], uint16(m.Command))
	payload[32] = byte(m.TargetSystem)
	payload[33] = byte(m.TargetComponent)
	payload[34] = byte(m.Frame)
	payload[35] = byte(m.Current)
	payload[36] = byte(m.Autocontinue)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *MissionItemInt) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 37 {
		payload = append(payload, mavlink.ZeroTail[:37-len(p.Payload)]...)
	}
	m.Param1 = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.Param2 = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	m.Param3 = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.Param4 = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.X = int32(binary.LittleEndian.Uint32(payload[16:]))
	m.Y = int32(binary.LittleEndian.Uint32(payload[20:]))
	m.Z = math.Float32frombits(binary.LittleEndian.Uint32(payload[24:]))
	m.Seq = uint16(binary.LittleEndian.Uint16(payload[28:]))
	m.Command = MAV_CMD(binary.LittleEndian.Uint16(payload[30:]))
	m.TargetSystem = uint8(payload[32])
	m.TargetComponent = uint8(payload[33])
	m.Frame = MAV_FRAME(payload[34])
	m.Current = uint8(payload[35])
	m.Autocontinue = uint8(payload[36])
	return nil
}

// VfrHud struct (generated typeinfo)
// Metrics typically displayed on a HUD for fixed wing aircraft.
type VfrHud struct {
	Airspeed    float32 // Vehicle speed in form appropriate for vehicle type. For standard aircraft this is typically calibrated airspeed (CAS) or indicated airspeed (IAS) - either of which can be used by a pilot to estimate stall speed.
	Groundspeed float32 // Current ground speed.
	Alt         float32 // Current altitude (MSL).
	Climb       float32 // Current climb rate.
	Heading     int16   // Current heading in compass units (0-360, 0=north).
	Throttle    uint16  // Current throttle setting (0 to 100).
}

// MsgID (generated function)
func (m *VfrHud) MsgID() mavlink.MessageID {
	return MSG_ID_VFR_HUD
}

// String (generated function)
func (m *VfrHud) String() string {
	return fmt.Sprintf(
		"&common.VfrHud{ Airspeed: %+v, Groundspeed: %+v, Alt: %+v, Climb: %+v, Heading: %+v, Throttle: %+v }",
		m.Airspeed,
		m.Groundspeed,
		m.Alt,
		m.Climb,
		m.Heading,
		m.Throttle,
	)
}

// Pack (generated function)
func (m *VfrHud) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 20)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(m.Airspeed))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(m.Groundspeed))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.Alt))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.Climb))
	binary.LittleEndian.PutUint16(payload[16:], uint16(m.Heading))
	binary.LittleEndian.PutUint16(payload[18:], uint16(m.Throttle))
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *VfrHud) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 20 {
		payload = append(payload, mavlink.ZeroTail[:20-len(p.Payload)]...)
	}
	m.Airspeed = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.Groundspeed = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	m.Alt = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.Climb = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.Heading = int16(binary.LittleEndian.Uint16(payload[16:]))
	m.Throttle = uint16(binary.LittleEndian.Uint16(payload[18:]))
	return nil
}

// CommandInt struct (generated typeinfo)
// Message encoding a command with parameters as scaled integers. Scaling depends on the actual command value. The command microservice is documented at https://mavlink.io/en/services/command.html
type CommandInt struct {
	Param1          float32   // PARAM1, see MAV_CMD enum
	Param2          float32   // PARAM2, see MAV_CMD enum
	Param3          float32   // PARAM3, see MAV_CMD enum
	Param4          float32   // PARAM4, see MAV_CMD enum
	X               int32     // PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7
	Y               int32     // PARAM6 / local: y position in meters * 1e4, global: longitude in degrees * 10^7
	Z               float32   // PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame).
	Command         MAV_CMD   `gotype:"uint16"` // The scheduled action for the mission item.
	TargetSystem    uint8     // System ID
	TargetComponent uint8     // Component ID
	Frame           MAV_FRAME `gotype:"uint8"` // The coordinate system of the COMMAND.
	Current         uint8     // Not used.
	Autocontinue    uint8     // Not used (set 0).
}

// MsgID (generated function)
func (m *CommandInt) MsgID() mavlink.MessageID {
	return MSG_ID_COMMAND_INT
}

// String (generated function)
func (m *CommandInt) String() string {
	return fmt.Sprintf(
		"&common.CommandInt{ Param1: %+v, Param2: %+v, Param3: %+v, Param4: %+v, X: %+v, Y: %+v, Z: %+v, Command: %+v, TargetSystem: %+v, TargetComponent: %+v, Frame: %+v, Current: %+v, Autocontinue: %+v }",
		m.Param1,
		m.Param2,
		m.Param3,
		m.Param4,
		m.X,
		m.Y,
		m.Z,
		m.Command,
		m.TargetSystem,
		m.TargetComponent,
		m.Frame,
		m.Current,
		m.Autocontinue,
	)
}

// Pack (generated function)
func (m *CommandInt) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 35)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(m.Param1))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(m.Param2))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.Param3))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.Param4))
	binary.LittleEndian.PutUint32(payload[16:], uint32(m.X))
	binary.LittleEndian.PutUint32(payload[20:], uint32(m.Y))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(m.Z))
	binary.LittleEndian.PutUint16(payload[28:], uint16(m.Command))
	payload[30] = byte(m.TargetSystem)
	payload[31] = byte(m.TargetComponent)
	payload[32] = byte(m.Frame)
	payload[33] = byte(m.Current)
	payload[34] = byte(m.Autocontinue)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *CommandInt) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 35 {
		payload = append(payload, mavlink.ZeroTail[:35-len(p.Payload)]...)
	}
	m.Param1 = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.Param2 = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	m.Param3 = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.Param4 = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.X = int32(binary.LittleEndian.Uint32(payload[16:]))
	m.Y = int32(binary.LittleEndian.Uint32(payload[20:]))
	m.Z = math.Float32frombits(binary.LittleEndian.Uint32(payload[24:]))
	m.Command = MAV_CMD(binary.LittleEndian.Uint16(payload[28:]))
	m.TargetSystem = uint8(payload[30])
	m.TargetComponent = uint8(payload[31])
	m.Frame = MAV_FRAME(payload[32])
	m.Current = uint8(payload[33])
	m.Autocontinue = uint8(payload[34])
	return nil
}

// CommandLong struct (generated typeinfo)
// Send a command with up to seven parameters to the MAV. The command microservice is documented at https://mavlink.io/en/services/command.html
type CommandLong struct {
	Param1          float32 // Parameter 1 (for the specific command).
	Param2          float32 // Parameter 2 (for the specific command).
	Param3          float32 // Parameter 3 (for the specific command).
	Param4          float32 // Parameter 4 (for the specific command).
	Param5          float32 // Parameter 5 (for the specific command).
	Param6          float32 // Parameter 6 (for the specific command).
	Param7          float32 // Parameter 7 (for the specific command).
	Command         MAV_CMD `gotype:"uint16"` // Command ID (of command to send).
	TargetSystem    uint8   // System which should execute the command
	TargetComponent uint8   // Component which should execute the command, 0 for all components
	Confirmation    uint8   // 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
}

// MsgID (generated function)
func (m *CommandLong) MsgID() mavlink.MessageID {
	return MSG_ID_COMMAND_LONG
}

// String (generated function)
func (m *CommandLong) String() string {
	return fmt.Sprintf(
		"&common.CommandLong{ Param1: %+v, Param2: %+v, Param3: %+v, Param4: %+v, Param5: %+v, Param6: %+v, Param7: %+v, Command: %+v, TargetSystem: %+v, TargetComponent: %+v, Confirmation: %+v }",
		m.Param1,
		m.Param2,
		m.Param3,
		m.Param4,
		m.Param5,
		m.Param6,
		m.Param7,
		m.Command,
		m.TargetSystem,
		m.TargetComponent,
		m.Confirmation,
	)
}

// Pack (generated function)
func (m *CommandLong) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 33)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(m.Param1))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(m.Param2))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.Param3))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.Param4))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(m.Param5))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(m.Param6))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(m.Param7))
	binary.LittleEndian.PutUint16(payload[28:], uint16(m.Command))
	payload[30] = byte(m.TargetSystem)
	payload[31] = byte(m.TargetComponent)
	payload[32] = byte(m.Confirmation)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *CommandLong) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 33 {
		payload = append(payload, mavlink.ZeroTail[:33-len(p.Payload)]...)
	}
	m.Param1 = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.Param2 = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	m.Param3 = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.Param4 = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.Param5 = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	m.Param6 = math.Float32frombits(binary.LittleEndian.Uint32(payload[20:]))
	m.Param7 = math.Float32frombits(binary.LittleEndian.Uint32(payload[24:]))
	m.Command = MAV_CMD(binary.LittleEndian.Uint16(payload[28:]))
	m.TargetSystem = uint8(payload[30])
	m.TargetComponent = uint8(payload[31])
	m.Confirmation = uint8(payload[32])
	return nil
}

// CommandAck struct (generated typeinfo)
// Report status of a command. Includes feedback whether the command was executed. The command microservice is documented at https://mavlink.io/en/services/command.html
type CommandAck struct {
	Command MAV_CMD    `gotype:"uint16"` // Command ID (of acknowledged command).
	Result  MAV_RESULT `gotype:"uint8"`  // Result of command.
}

// MsgID (generated function)
func (m *CommandAck) MsgID() mavlink.MessageID {
	return MSG_ID_COMMAND_ACK
}

// String (generated function)
func (m *CommandAck) String() string {
	return fmt.Sprintf(
		"&common.CommandAck{ Command: %+v, Result: %+v }",
		m.Command,
		m.Result,
	)
}

// Pack (generated function)
func (m *CommandAck) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 3)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.Command))
	payload[2] = byte(m.Result)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *CommandAck) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 3 {
		payload = append(payload, mavlink.ZeroTail[:3-len(p.Payload)]...)
	}
	m.Command = MAV_CMD(binary.LittleEndian.Uint16(payload[0:]))
	m.Result = MAV_RESULT(payload[2])
	return nil
}

// CommandCancel struct (generated typeinfo)
// Cancel a long running command. The target system should respond with a COMMAND_ACK to the original command with result=MAV_RESULT_CANCELLED if the long running process was cancelled. If it has already completed, the cancel action can be ignored. The cancel action can be retried until some sort of acknowledgement to the original command has been received. The command microservice is documented at https://mavlink.io/en/services/command.html
type CommandCancel struct {
	Command         MAV_CMD `gotype:"uint16"` // Command ID (of command to cancel).
	TargetSystem    uint8   // System executing long running command. Should not be broadcast (0).
	TargetComponent uint8   // Component executing long running command.
}

// MsgID (generated function)
func (m *CommandCancel) MsgID() mavlink.MessageID {
	return MSG_ID_COMMAND_CANCEL
}

// String (generated function)
func (m *CommandCancel) String() string {
	return fmt.Sprintf(
		"&common.CommandCancel{ Command: %+v, TargetSystem: %+v, TargetComponent: %+v }",
		m.Command,
		m.TargetSystem,
		m.TargetComponent,
	)
}

// Pack (generated function)
func (m *CommandCancel) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 4)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.Command))
	payload[2] = byte(m.TargetSystem)
	payload[3] = byte(m.TargetComponent)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *CommandCancel) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 4 {
		payload = append(payload, mavlink.ZeroTail[:4-len(p.Payload)]...)
	}
	m.Command = MAV_CMD(binary.LittleEndian.Uint16(payload[0:]))
	m.TargetSystem = uint8(payload[2])
	m.TargetComponent = uint8(payload[3])
	return nil
}

// ManualSetpoint struct (generated typeinfo)
// Setpoint in roll, pitch, yaw and thrust from the operator
type ManualSetpoint struct {
	TimeBootMs           uint32  // Timestamp (time since system boot).
	Roll                 float32 // Desired roll rate
	Pitch                float32 // Desired pitch rate
	Yaw                  float32 // Desired yaw rate
	Thrust               float32 // Collective thrust, normalized to 0 .. 1
	ModeSwitch           uint8   // Flight mode switch position, 0.. 255
	ManualOverrideSwitch uint8   // Override mode switch position, 0.. 255
}

// MsgID (generated function)
func (m *ManualSetpoint) MsgID() mavlink.MessageID {
	return MSG_ID_MANUAL_SETPOINT
}

// String (generated function)
func (m *ManualSetpoint) String() string {
	return fmt.Sprintf(
		"&common.ManualSetpoint{ TimeBootMs: %+v, Roll: %+v, Pitch: %+v, Yaw: %+v, Thrust: %+v, ModeSwitch: %+v, ManualOverrideSwitch: %+v }",
		m.TimeBootMs,
		m.Roll,
		m.Pitch,
		m.Yaw,
		m.Thrust,
		m.ModeSwitch,
		m.ManualOverrideSwitch,
	)
}

// Pack (generated function)
func (m *ManualSetpoint) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 22)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.TimeBootMs))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(m.Roll))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.Pitch))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.Yaw))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(m.Thrust))
	payload[20] = byte(m.ModeSwitch)
	payload[21] = byte(m.ManualOverrideSwitch)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ManualSetpoint) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 22 {
		payload = append(payload, mavlink.ZeroTail[:22-len(p.Payload)]...)
	}
	m.TimeBootMs = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.Roll = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	m.Pitch = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.Yaw = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.Thrust = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	m.ModeSwitch = uint8(payload[20])
	m.ManualOverrideSwitch = uint8(payload[21])
	return nil
}

// SetAttitudeTarget struct (generated typeinfo)
// Sets a desired vehicle attitude. Used by an external controller to command the vehicle (manual controller or other system).
type SetAttitudeTarget struct {
	TimeBootMs      uint32                   // Timestamp (time since system boot).
	Q               [4]float32               // Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
	BodyRollRate    float32                  // Body roll rate
	BodyPitchRate   float32                  // Body pitch rate
	BodyYawRate     float32                  // Body yaw rate
	Thrust          float32                  // Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)
	TargetSystem    uint8                    // System ID
	TargetComponent uint8                    // Component ID
	TypeMask        ATTITUDE_TARGET_TYPEMASK `gotype:"uint8"` // Bitmap to indicate which dimensions should be ignored by the vehicle.
}

// MsgID (generated function)
func (m *SetAttitudeTarget) MsgID() mavlink.MessageID {
	return MSG_ID_SET_ATTITUDE_TARGET
}

// String (generated function)
func (m *SetAttitudeTarget) String() string {
	return fmt.Sprintf(
		"&common.SetAttitudeTarget{ TimeBootMs: %+v, Q: %+v, BodyRollRate: %+v, BodyPitchRate: %+v, BodyYawRate: %+v, Thrust: %+v, TargetSystem: %+v, TargetComponent: %+v, TypeMask: %+v }",
		m.TimeBootMs,
		m.Q,
		m.BodyRollRate,
		m.BodyPitchRate,
		m.BodyYawRate,
		m.Thrust,
		m.TargetSystem,
		m.TargetComponent,
		m.TypeMask,
	)
}

// Pack (generated function)
func (m *SetAttitudeTarget) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 39)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.TimeBootMs))
	for i, v := range m.Q {
		binary.LittleEndian.PutUint32(payload[4+i*4:], math.Float32bits(v))
	}
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(m.BodyRollRate))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(m.BodyPitchRate))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(m.BodyYawRate))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(m.Thrust))
	payload[36] = byte(m.TargetSystem)
	payload[37] = byte(m.TargetComponent)
	payload[38] = byte(m.TypeMask)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *SetAttitudeTarget) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 39 {
		payload = append(payload, mavlink.ZeroTail[:39-len(p.Payload)]...)
	}
	m.TimeBootMs = uint32(binary.LittleEndian.Uint32(payload[0:]))
	for i := 0; i < len(m.Q); i++ {
		m.Q[i] = math.Float32frombits(binary.LittleEndian.Uint32(payload[4+i*4:]))
	}
	m.BodyRollRate = math.Float32frombits(binary.LittleEndian.Uint32(payload[20:]))
	m.BodyPitchRate = math.Float32frombits(binary.LittleEndian.Uint32(payload[24:]))
	m.BodyYawRate = math.Float32frombits(binary.LittleEndian.Uint32(payload[28:]))
	m.Thrust = math.Float32frombits(binary.LittleEndian.Uint32(payload[32:]))
	m.TargetSystem = uint8(payload[36])
	m.TargetComponent = uint8(payload[37])
	m.TypeMask = ATTITUDE_TARGET_TYPEMASK(payload[38])
	return nil
}

// AttitudeTarget struct (generated typeinfo)
// Reports the current commanded attitude of the vehicle as specified by the autopilot. This should match the commands sent in a SET_ATTITUDE_TARGET message if the vehicle is being controlled this way.
type AttitudeTarget struct {
	TimeBootMs    uint32                   // Timestamp (time since system boot).
	Q             [4]float32               // Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
	BodyRollRate  float32                  // Body roll rate
	BodyPitchRate float32                  // Body pitch rate
	BodyYawRate   float32                  // Body yaw rate
	Thrust        float32                  // Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)
	TypeMask      ATTITUDE_TARGET_TYPEMASK `gotype:"uint8"` // Bitmap to indicate which dimensions should be ignored by the vehicle.
}

// MsgID (generated function)
func (m *AttitudeTarget) MsgID() mavlink.MessageID {
	return MSG_ID_ATTITUDE_TARGET
}

// String (generated function)
func (m *AttitudeTarget) String() string {
	return fmt.Sprintf(
		"&common.AttitudeTarget{ TimeBootMs: %+v, Q: %+v, BodyRollRate: %+v, BodyPitchRate: %+v, BodyYawRate: %+v, Thrust: %+v, TypeMask: %+v }",
		m.TimeBootMs,
		m.Q,
		m.BodyRollRate,
		m.BodyPitchRate,
		m.BodyYawRate,
		m.Thrust,
		m.TypeMask,
	)
}

// Pack (generated function)
func (m *AttitudeTarget) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 37)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.TimeBootMs))
	for i, v := range m.Q {
		binary.LittleEndian.PutUint32(payload[4+i*4:], math.Float32bits(v))
	}
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(m.BodyRollRate))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(m.BodyPitchRate))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(m.BodyYawRate))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(m.Thrust))
	payload[36] = byte(m.TypeMask)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *AttitudeTarget) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 37 {
		payload = append(payload, mavlink.ZeroTail[:37-len(p.Payload)]...)
	}
	m.TimeBootMs = uint32(binary.LittleEndian.Uint32(payload[0:]))
	for i := 0; i < len(m.Q); i++ {
		m.Q[i] = math.Float32frombits(binary.LittleEndian.Uint32(payload[4+i*4:]))
	}
	m.BodyRollRate = math.Float32frombits(binary.LittleEndian.Uint32(payload[20:]))
	m.BodyPitchRate = math.Float32frombits(binary.LittleEndian.Uint32(payload[24:]))
	m.BodyYawRate = math.Float32frombits(binary.LittleEndian.Uint32(payload[28:]))
	m.Thrust = math.Float32frombits(binary.LittleEndian.Uint32(payload[32:]))
	m.TypeMask = ATTITUDE_TARGET_TYPEMASK(payload[36])
	return nil
}

// SetPositionTargetLocalNed struct (generated typeinfo)
// Sets a desired vehicle position in a local north-east-down coordinate frame. Used by an external controller to command the vehicle (manual controller or other system).
type SetPositionTargetLocalNed struct {
	TimeBootMs      uint32                   // Timestamp (time since system boot).
	X               float32                  // X Position in NED frame
	Y               float32                  // Y Position in NED frame
	Z               float32                  // Z Position in NED frame (note, altitude is negative in NED)
	Vx              float32                  // X velocity in NED frame
	Vy              float32                  // Y velocity in NED frame
	Vz              float32                  // Z velocity in NED frame
	Afx             float32                  // X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Afy             float32                  // Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Afz             float32                  // Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Yaw             float32                  // yaw setpoint
	YawRate         float32                  // yaw rate setpoint
	TypeMask        POSITION_TARGET_TYPEMASK `gotype:"uint16"` // Bitmap to indicate which dimensions should be ignored by the vehicle.
	TargetSystem    uint8                    // System ID
	TargetComponent uint8                    // Component ID
	CoordinateFrame MAV_FRAME                `gotype:"uint8"` // Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED = 9
}

// MsgID (generated function)
func (m *SetPositionTargetLocalNed) MsgID() mavlink.MessageID {
	return MSG_ID_SET_POSITION_TARGET_LOCAL_NED
}

// String (generated function)
func (m *SetPositionTargetLocalNed) String() string {
	return fmt.Sprintf(
		"&common.SetPositionTargetLocalNed{ TimeBootMs: %+v, X: %+v, Y: %+v, Z: %+v, Vx: %+v, Vy: %+v, Vz: %+v, Afx: %+v, Afy: %+v, Afz: %+v, Yaw: %+v, YawRate: %+v, TypeMask: %+v, TargetSystem: %+v, TargetComponent: %+v, CoordinateFrame: %+v }",
		m.TimeBootMs,
		m.X,
		m.Y,
		m.Z,
		m.Vx,
		m.Vy,
		m.Vz,
		m.Afx,
		m.Afy,
		m.Afz,
		m.Yaw,
		m.YawRate,
		m.TypeMask,
		m.TargetSystem,
		m.TargetComponent,
		m.CoordinateFrame,
	)
}

// Pack (generated function)
func (m *SetPositionTargetLocalNed) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 53)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.TimeBootMs))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(m.X))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.Y))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.Z))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(m.Vx))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(m.Vy))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(m.Vz))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(m.Afx))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(m.Afy))
	binary.LittleEndian.PutUint32(payload[36:], math.Float32bits(m.Afz))
	binary.LittleEndian.PutUint32(payload[40:], math.Float32bits(m.Yaw))
	binary.LittleEndian.PutUint32(payload[44:], math.Float32bits(m.YawRate))
	binary.LittleEndian.PutUint16(payload[48:], uint16(m.TypeMask))
	payload[50] = byte(m.TargetSystem)
	payload[51] = byte(m.TargetComponent)
	payload[52] = byte(m.CoordinateFrame)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *SetPositionTargetLocalNed) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 53 {
		payload = append(payload, mavlink.ZeroTail[:53-len(p.Payload)]...)
	}
	m.TimeBootMs = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.X = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	m.Y = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.Z = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.Vx = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	m.Vy = math.Float32frombits(binary.LittleEndian.Uint32(payload[20:]))
	m.Vz = math.Float32frombits(binary.LittleEndian.Uint32(payload[24:]))
	m.Afx = math.Float32frombits(binary.LittleEndian.Uint32(payload[28:]))
	m.Afy = math.Float32frombits(binary.LittleEndian.Uint32(payload[32:]))
	m.Afz = math.Float32frombits(binary.LittleEndian.Uint32(payload[36:]))
	m.Yaw = math.Float32frombits(binary.LittleEndian.Uint32(payload[40:]))
	m.YawRate = math.Float32frombits(binary.LittleEndian.Uint32(payload[44:]))
	m.TypeMask = POSITION_TARGET_TYPEMASK(binary.LittleEndian.Uint16(payload[48:]))
	m.TargetSystem = uint8(payload[50])
	m.TargetComponent = uint8(payload[51])
	m.CoordinateFrame = MAV_FRAME(payload[52])
	return nil
}

// PositionTargetLocalNed struct (generated typeinfo)
// Reports the current commanded vehicle position, velocity, and acceleration as specified by the autopilot. This should match the commands sent in SET_POSITION_TARGET_LOCAL_NED if the vehicle is being controlled this way.
type PositionTargetLocalNed struct {
	TimeBootMs      uint32                   // Timestamp (time since system boot).
	X               float32                  // X Position in NED frame
	Y               float32                  // Y Position in NED frame
	Z               float32                  // Z Position in NED frame (note, altitude is negative in NED)
	Vx              float32                  // X velocity in NED frame
	Vy              float32                  // Y velocity in NED frame
	Vz              float32                  // Z velocity in NED frame
	Afx             float32                  // X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Afy             float32                  // Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Afz             float32                  // Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Yaw             float32                  // yaw setpoint
	YawRate         float32                  // yaw rate setpoint
	TypeMask        POSITION_TARGET_TYPEMASK `gotype:"uint16"` // Bitmap to indicate which dimensions should be ignored by the vehicle.
	CoordinateFrame MAV_FRAME                `gotype:"uint8"`  // Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED = 9
}

// MsgID (generated function)
func (m *PositionTargetLocalNed) MsgID() mavlink.MessageID {
	return MSG_ID_POSITION_TARGET_LOCAL_NED
}

// String (generated function)
func (m *PositionTargetLocalNed) String() string {
	return fmt.Sprintf(
		"&common.PositionTargetLocalNed{ TimeBootMs: %+v, X: %+v, Y: %+v, Z: %+v, Vx: %+v, Vy: %+v, Vz: %+v, Afx: %+v, Afy: %+v, Afz: %+v, Yaw: %+v, YawRate: %+v, TypeMask: %+v, CoordinateFrame: %+v }",
		m.TimeBootMs,
		m.X,
		m.Y,
		m.Z,
		m.Vx,
		m.Vy,
		m.Vz,
		m.Afx,
		m.Afy,
		m.Afz,
		m.Yaw,
		m.YawRate,
		m.TypeMask,
		m.CoordinateFrame,
	)
}

// Pack (generated function)
func (m *PositionTargetLocalNed) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 51)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.TimeBootMs))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(m.X))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.Y))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.Z))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(m.Vx))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(m.Vy))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(m.Vz))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(m.Afx))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(m.Afy))
	binary.LittleEndian.PutUint32(payload[36:], math.Float32bits(m.Afz))
	binary.LittleEndian.PutUint32(payload[40:], math.Float32bits(m.Yaw))
	binary.LittleEndian.PutUint32(payload[44:], math.Float32bits(m.YawRate))
	binary.LittleEndian.PutUint16(payload[48:], uint16(m.TypeMask))
	payload[50] = byte(m.CoordinateFrame)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *PositionTargetLocalNed) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 51 {
		payload = append(payload, mavlink.ZeroTail[:51-len(p.Payload)]...)
	}
	m.TimeBootMs = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.X = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	m.Y = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.Z = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.Vx = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	m.Vy = math.Float32frombits(binary.LittleEndian.Uint32(payload[20:]))
	m.Vz = math.Float32frombits(binary.LittleEndian.Uint32(payload[24:]))
	m.Afx = math.Float32frombits(binary.LittleEndian.Uint32(payload[28:]))
	m.Afy = math.Float32frombits(binary.LittleEndian.Uint32(payload[32:]))
	m.Afz = math.Float32frombits(binary.LittleEndian.Uint32(payload[36:]))
	m.Yaw = math.Float32frombits(binary.LittleEndian.Uint32(payload[40:]))
	m.YawRate = math.Float32frombits(binary.LittleEndian.Uint32(payload[44:]))
	m.TypeMask = POSITION_TARGET_TYPEMASK(binary.LittleEndian.Uint16(payload[48:]))
	m.CoordinateFrame = MAV_FRAME(payload[50])
	return nil
}

// SetPositionTargetGlobalInt struct (generated typeinfo)
// Sets a desired vehicle position, velocity, and/or acceleration in a global coordinate system (WGS84). Used by an external controller to command the vehicle (manual controller or other system).
type SetPositionTargetGlobalInt struct {
	TimeBootMs      uint32                   // Timestamp (time since system boot). The rationale for the timestamp in the setpoint is to allow the system to compensate for the transport delay of the setpoint. This allows the system to compensate processing latency.
	LatInt          int32                    // X Position in WGS84 frame
	LonInt          int32                    // Y Position in WGS84 frame
	Alt             float32                  // Altitude (MSL, Relative to home, or AGL - depending on frame)
	Vx              float32                  // X velocity in NED frame
	Vy              float32                  // Y velocity in NED frame
	Vz              float32                  // Z velocity in NED frame
	Afx             float32                  // X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Afy             float32                  // Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Afz             float32                  // Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Yaw             float32                  // yaw setpoint
	YawRate         float32                  // yaw rate setpoint
	TypeMask        POSITION_TARGET_TYPEMASK `gotype:"uint16"` // Bitmap to indicate which dimensions should be ignored by the vehicle.
	TargetSystem    uint8                    // System ID
	TargetComponent uint8                    // Component ID
	CoordinateFrame MAV_FRAME                `gotype:"uint8"` // Valid options are: MAV_FRAME_GLOBAL_INT = 5, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT = 11
}

// MsgID (generated function)
func (m *SetPositionTargetGlobalInt) MsgID() mavlink.MessageID {
	return MSG_ID_SET_POSITION_TARGET_GLOBAL_INT
}

// String (generated function)
func (m *SetPositionTargetGlobalInt) String() string {
	return fmt.Sprintf(
		"&common.SetPositionTargetGlobalInt{ TimeBootMs: %+v, LatInt: %+v, LonInt: %+v, Alt: %+v, Vx: %+v, Vy: %+v, Vz: %+v, Afx: %+v, Afy: %+v, Afz: %+v, Yaw: %+v, YawRate: %+v, TypeMask: %+v, TargetSystem: %+v, TargetComponent: %+v, CoordinateFrame: %+v }",
		m.TimeBootMs,
		m.LatInt,
		m.LonInt,
		m.Alt,
		m.Vx,
		m.Vy,
		m.Vz,
		m.Afx,
		m.Afy,
		m.Afz,
		m.Yaw,
		m.YawRate,
		m.TypeMask,
		m.TargetSystem,
		m.TargetComponent,
		m.CoordinateFrame,
	)
}

// Pack (generated function)
func (m *SetPositionTargetGlobalInt) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 53)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.TimeBootMs))
	binary.LittleEndian.PutUint32(payload[4:], uint32(m.LatInt))
	binary.LittleEndian.PutUint32(payload[8:], uint32(m.LonInt))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.Alt))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(m.Vx))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(m.Vy))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(m.Vz))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(m.Afx))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(m.Afy))
	binary.LittleEndian.PutUint32(payload[36:], math.Float32bits(m.Afz))
	binary.LittleEndian.PutUint32(payload[40:], math.Float32bits(m.Yaw))
	binary.LittleEndian.PutUint32(payload[44:], math.Float32bits(m.YawRate))
	binary.LittleEndian.PutUint16(payload[48:], uint16(m.TypeMask))
	payload[50] = byte(m.TargetSystem)
	payload[51] = byte(m.TargetComponent)
	payload[52] = byte(m.CoordinateFrame)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *SetPositionTargetGlobalInt) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 53 {
		payload = append(payload, mavlink.ZeroTail[:53-len(p.Payload)]...)
	}
	m.TimeBootMs = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.LatInt = int32(binary.LittleEndian.Uint32(payload[4:]))
	m.LonInt = int32(binary.LittleEndian.Uint32(payload[8:]))
	m.Alt = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.Vx = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	m.Vy = math.Float32frombits(binary.LittleEndian.Uint32(payload[20:]))
	m.Vz = math.Float32frombits(binary.LittleEndian.Uint32(payload[24:]))
	m.Afx = math.Float32frombits(binary.LittleEndian.Uint32(payload[28:]))
	m.Afy = math.Float32frombits(binary.LittleEndian.Uint32(payload[32:]))
	m.Afz = math.Float32frombits(binary.LittleEndian.Uint32(payload[36:]))
	m.Yaw = math.Float32frombits(binary.LittleEndian.Uint32(payload[40:]))
	m.YawRate = math.Float32frombits(binary.LittleEndian.Uint32(payload[44:]))
	m.TypeMask = POSITION_TARGET_TYPEMASK(binary.LittleEndian.Uint16(payload[48:]))
	m.TargetSystem = uint8(payload[50])
	m.TargetComponent = uint8(payload[51])
	m.CoordinateFrame = MAV_FRAME(payload[52])
	return nil
}

// PositionTargetGlobalInt struct (generated typeinfo)
// Reports the current commanded vehicle position, velocity, and acceleration as specified by the autopilot. This should match the commands sent in SET_POSITION_TARGET_GLOBAL_INT if the vehicle is being controlled this way.
type PositionTargetGlobalInt struct {
	TimeBootMs      uint32                   // Timestamp (time since system boot). The rationale for the timestamp in the setpoint is to allow the system to compensate for the transport delay of the setpoint. This allows the system to compensate processing latency.
	LatInt          int32                    // X Position in WGS84 frame
	LonInt          int32                    // Y Position in WGS84 frame
	Alt             float32                  // Altitude (MSL, AGL or relative to home altitude, depending on frame)
	Vx              float32                  // X velocity in NED frame
	Vy              float32                  // Y velocity in NED frame
	Vz              float32                  // Z velocity in NED frame
	Afx             float32                  // X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Afy             float32                  // Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Afz             float32                  // Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Yaw             float32                  // yaw setpoint
	YawRate         float32                  // yaw rate setpoint
	TypeMask        POSITION_TARGET_TYPEMASK `gotype:"uint16"` // Bitmap to indicate which dimensions should be ignored by the vehicle.
	CoordinateFrame MAV_FRAME                `gotype:"uint8"`  // Valid options are: MAV_FRAME_GLOBAL_INT = 5, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT = 11
}

// MsgID (generated function)
func (m *PositionTargetGlobalInt) MsgID() mavlink.MessageID {
	return MSG_ID_POSITION_TARGET_GLOBAL_INT
}

// String (generated function)
func (m *PositionTargetGlobalInt) String() string {
	return fmt.Sprintf(
		"&common.PositionTargetGlobalInt{ TimeBootMs: %+v, LatInt: %+v, LonInt: %+v, Alt: %+v, Vx: %+v, Vy: %+v, Vz: %+v, Afx: %+v, Afy: %+v, Afz: %+v, Yaw: %+v, YawRate: %+v, TypeMask: %+v, CoordinateFrame: %+v }",
		m.TimeBootMs,
		m.LatInt,
		m.LonInt,
		m.Alt,
		m.Vx,
		m.Vy,
		m.Vz,
		m.Afx,
		m.Afy,
		m.Afz,
		m.Yaw,
		m.YawRate,
		m.TypeMask,
		m.CoordinateFrame,
	)
}

// Pack (generated function)
func (m *PositionTargetGlobalInt) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 51)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.TimeBootMs))
	binary.LittleEndian.PutUint32(payload[4:], uint32(m.LatInt))
	binary.LittleEndian.PutUint32(payload[8:], uint32(m.LonInt))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.Alt))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(m.Vx))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(m.Vy))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(m.Vz))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(m.Afx))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(m.Afy))
	binary.LittleEndian.PutUint32(payload[36:], math.Float32bits(m.Afz))
	binary.LittleEndian.PutUint32(payload[40:], math.Float32bits(m.Yaw))
	binary.LittleEndian.PutUint32(payload[44:], math.Float32bits(m.YawRate))
	binary.LittleEndian.PutUint16(payload[48:], uint16(m.TypeMask))
	payload[50] = byte(m.CoordinateFrame)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *PositionTargetGlobalInt) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 51 {
		payload = append(payload, mavlink.ZeroTail[:51-len(p.Payload)]...)
	}
	m.TimeBootMs = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.LatInt = int32(binary.LittleEndian.Uint32(payload[4:]))
	m.LonInt = int32(binary.LittleEndian.Uint32(payload[8:]))
	m.Alt = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.Vx = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	m.Vy = math.Float32frombits(binary.LittleEndian.Uint32(payload[20:]))
	m.Vz = math.Float32frombits(binary.LittleEndian.Uint32(payload[24:]))
	m.Afx = math.Float32frombits(binary.LittleEndian.Uint32(payload[28:]))
	m.Afy = math.Float32frombits(binary.LittleEndian.Uint32(payload[32:]))
	m.Afz = math.Float32frombits(binary.LittleEndian.Uint32(payload[36:]))
	m.Yaw = math.Float32frombits(binary.LittleEndian.Uint32(payload[40:]))
	m.YawRate = math.Float32frombits(binary.LittleEndian.Uint32(payload[44:]))
	m.TypeMask = POSITION_TARGET_TYPEMASK(binary.LittleEndian.Uint16(payload[48:]))
	m.CoordinateFrame = MAV_FRAME(payload[50])
	return nil
}

// LocalPositionNedSystemGlobalOffset struct (generated typeinfo)
// The offset in X, Y, Z and yaw between the LOCAL_POSITION_NED messages of MAV X and the global coordinate frame in NED coordinates. Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down convention)
type LocalPositionNedSystemGlobalOffset struct {
	TimeBootMs uint32  // Timestamp (time since system boot).
	X          float32 // X Position
	Y          float32 // Y Position
	Z          float32 // Z Position
	Roll       float32 // Roll
	Pitch      float32 // Pitch
	Yaw        float32 // Yaw
}

// MsgID (generated function)
func (m *LocalPositionNedSystemGlobalOffset) MsgID() mavlink.MessageID {
	return MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET
}

// String (generated function)
func (m *LocalPositionNedSystemGlobalOffset) String() string {
	return fmt.Sprintf(
		"&common.LocalPositionNedSystemGlobalOffset{ TimeBootMs: %+v, X: %+v, Y: %+v, Z: %+v, Roll: %+v, Pitch: %+v, Yaw: %+v }",
		m.TimeBootMs,
		m.X,
		m.Y,
		m.Z,
		m.Roll,
		m.Pitch,
		m.Yaw,
	)
}

// Pack (generated function)
func (m *LocalPositionNedSystemGlobalOffset) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 28)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.TimeBootMs))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(m.X))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.Y))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.Z))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(m.Roll))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(m.Pitch))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(m.Yaw))
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *LocalPositionNedSystemGlobalOffset) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 28 {
		payload = append(payload, mavlink.ZeroTail[:28-len(p.Payload)]...)
	}
	m.TimeBootMs = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.X = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	m.Y = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.Z = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.Roll = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	m.Pitch = math.Float32frombits(binary.LittleEndian.Uint32(payload[20:]))
	m.Yaw = math.Float32frombits(binary.LittleEndian.Uint32(payload[24:]))
	return nil
}

// HilState struct (generated typeinfo)
// Sent from simulation to autopilot. This packet is useful for high throughput applications such as hardware in the loop simulations.
type HilState struct {
	TimeUsec   uint64  // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
	Roll       float32 // Roll angle
	Pitch      float32 // Pitch angle
	Yaw        float32 // Yaw angle
	Rollspeed  float32 // Body frame roll / phi angular speed
	Pitchspeed float32 // Body frame pitch / theta angular speed
	Yawspeed   float32 // Body frame yaw / psi angular speed
	Lat        int32   // Latitude
	Lon        int32   // Longitude
	Alt        int32   // Altitude
	Vx         int16   // Ground X Speed (Latitude)
	Vy         int16   // Ground Y Speed (Longitude)
	Vz         int16   // Ground Z Speed (Altitude)
	Xacc       int16   // X acceleration
	Yacc       int16   // Y acceleration
	Zacc       int16   // Z acceleration
}

// MsgID (generated function)
func (m *HilState) MsgID() mavlink.MessageID {
	return MSG_ID_HIL_STATE
}

// String (generated function)
func (m *HilState) String() string {
	return fmt.Sprintf(
		"&common.HilState{ TimeUsec: %+v, Roll: %+v, Pitch: %+v, Yaw: %+v, Rollspeed: %+v, Pitchspeed: %+v, Yawspeed: %+v, Lat: %+v, Lon: %+v, Alt: %+v, Vx: %+v, Vy: %+v, Vz: %+v, Xacc: %+v, Yacc: %+v, Zacc: %+v }",
		m.TimeUsec,
		m.Roll,
		m.Pitch,
		m.Yaw,
		m.Rollspeed,
		m.Pitchspeed,
		m.Yawspeed,
		m.Lat,
		m.Lon,
		m.Alt,
		m.Vx,
		m.Vy,
		m.Vz,
		m.Xacc,
		m.Yacc,
		m.Zacc,
	)
}

// Pack (generated function)
func (m *HilState) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 56)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.TimeUsec))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.Roll))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.Pitch))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(m.Yaw))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(m.Rollspeed))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(m.Pitchspeed))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(m.Yawspeed))
	binary.LittleEndian.PutUint32(payload[32:], uint32(m.Lat))
	binary.LittleEndian.PutUint32(payload[36:], uint32(m.Lon))
	binary.LittleEndian.PutUint32(payload[40:], uint32(m.Alt))
	binary.LittleEndian.PutUint16(payload[44:], uint16(m.Vx))
	binary.LittleEndian.PutUint16(payload[46:], uint16(m.Vy))
	binary.LittleEndian.PutUint16(payload[48:], uint16(m.Vz))
	binary.LittleEndian.PutUint16(payload[50:], uint16(m.Xacc))
	binary.LittleEndian.PutUint16(payload[52:], uint16(m.Yacc))
	binary.LittleEndian.PutUint16(payload[54:], uint16(m.Zacc))
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *HilState) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 56 {
		payload = append(payload, mavlink.ZeroTail[:56-len(p.Payload)]...)
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(payload[0:]))
	m.Roll = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.Pitch = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.Yaw = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	m.Rollspeed = math.Float32frombits(binary.LittleEndian.Uint32(payload[20:]))
	m.Pitchspeed = math.Float32frombits(binary.LittleEndian.Uint32(payload[24:]))
	m.Yawspeed = math.Float32frombits(binary.LittleEndian.Uint32(payload[28:]))
	m.Lat = int32(binary.LittleEndian.Uint32(payload[32:]))
	m.Lon = int32(binary.LittleEndian.Uint32(payload[36:]))
	m.Alt = int32(binary.LittleEndian.Uint32(payload[40:]))
	m.Vx = int16(binary.LittleEndian.Uint16(payload[44:]))
	m.Vy = int16(binary.LittleEndian.Uint16(payload[46:]))
	m.Vz = int16(binary.LittleEndian.Uint16(payload[48:]))
	m.Xacc = int16(binary.LittleEndian.Uint16(payload[50:]))
	m.Yacc = int16(binary.LittleEndian.Uint16(payload[52:]))
	m.Zacc = int16(binary.LittleEndian.Uint16(payload[54:]))
	return nil
}

// HilControls struct (generated typeinfo)
// Sent from autopilot to simulation. Hardware in the loop control outputs
type HilControls struct {
	TimeUsec      uint64   // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
	RollAilerons  float32  // Control output -1 .. 1
	PitchElevator float32  // Control output -1 .. 1
	YawRudder     float32  // Control output -1 .. 1
	Throttle      float32  // Throttle 0 .. 1
	Aux1          float32  // Aux 1, -1 .. 1
	Aux2          float32  // Aux 2, -1 .. 1
	Aux3          float32  // Aux 3, -1 .. 1
	Aux4          float32  // Aux 4, -1 .. 1
	Mode          MAV_MODE `gotype:"uint8"` // System mode.
	NavMode       uint8    // Navigation mode (MAV_NAV_MODE)
}

// MsgID (generated function)
func (m *HilControls) MsgID() mavlink.MessageID {
	return MSG_ID_HIL_CONTROLS
}

// String (generated function)
func (m *HilControls) String() string {
	return fmt.Sprintf(
		"&common.HilControls{ TimeUsec: %+v, RollAilerons: %+v, PitchElevator: %+v, YawRudder: %+v, Throttle: %+v, Aux1: %+v, Aux2: %+v, Aux3: %+v, Aux4: %+v, Mode: %+v, NavMode: %+v }",
		m.TimeUsec,
		m.RollAilerons,
		m.PitchElevator,
		m.YawRudder,
		m.Throttle,
		m.Aux1,
		m.Aux2,
		m.Aux3,
		m.Aux4,
		m.Mode,
		m.NavMode,
	)
}

// Pack (generated function)
func (m *HilControls) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 42)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.TimeUsec))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.RollAilerons))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.PitchElevator))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(m.YawRudder))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(m.Throttle))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(m.Aux1))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(m.Aux2))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(m.Aux3))
	binary.LittleEndian.PutUint32(payload[36:], math.Float32bits(m.Aux4))
	payload[40] = byte(m.Mode)
	payload[41] = byte(m.NavMode)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *HilControls) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 42 {
		payload = append(payload, mavlink.ZeroTail[:42-len(p.Payload)]...)
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(payload[0:]))
	m.RollAilerons = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.PitchElevator = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.YawRudder = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	m.Throttle = math.Float32frombits(binary.LittleEndian.Uint32(payload[20:]))
	m.Aux1 = math.Float32frombits(binary.LittleEndian.Uint32(payload[24:]))
	m.Aux2 = math.Float32frombits(binary.LittleEndian.Uint32(payload[28:]))
	m.Aux3 = math.Float32frombits(binary.LittleEndian.Uint32(payload[32:]))
	m.Aux4 = math.Float32frombits(binary.LittleEndian.Uint32(payload[36:]))
	m.Mode = MAV_MODE(payload[40])
	m.NavMode = uint8(payload[41])
	return nil
}

// HilRcInputsRaw struct (generated typeinfo)
// Sent from simulation to autopilot. The RAW values of the RC channels received. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual receivers/transmitters might violate this specification.
type HilRcInputsRaw struct {
	TimeUsec  uint64 // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
	Chan1Raw  uint16 // RC channel 1 value
	Chan2Raw  uint16 // RC channel 2 value
	Chan3Raw  uint16 // RC channel 3 value
	Chan4Raw  uint16 // RC channel 4 value
	Chan5Raw  uint16 // RC channel 5 value
	Chan6Raw  uint16 // RC channel 6 value
	Chan7Raw  uint16 // RC channel 7 value
	Chan8Raw  uint16 // RC channel 8 value
	Chan9Raw  uint16 // RC channel 9 value
	Chan10Raw uint16 // RC channel 10 value
	Chan11Raw uint16 // RC channel 11 value
	Chan12Raw uint16 // RC channel 12 value
	Rssi      uint8  // Receive signal strength indicator in device-dependent units/scale. Values: [0-254], 255: invalid/unknown.
}

// MsgID (generated function)
func (m *HilRcInputsRaw) MsgID() mavlink.MessageID {
	return MSG_ID_HIL_RC_INPUTS_RAW
}

// String (generated function)
func (m *HilRcInputsRaw) String() string {
	return fmt.Sprintf(
		"&common.HilRcInputsRaw{ TimeUsec: %+v, Chan1Raw: %+v, Chan2Raw: %+v, Chan3Raw: %+v, Chan4Raw: %+v, Chan5Raw: %+v, Chan6Raw: %+v, Chan7Raw: %+v, Chan8Raw: %+v, Chan9Raw: %+v, Chan10Raw: %+v, Chan11Raw: %+v, Chan12Raw: %+v, Rssi: %+v }",
		m.TimeUsec,
		m.Chan1Raw,
		m.Chan2Raw,
		m.Chan3Raw,
		m.Chan4Raw,
		m.Chan5Raw,
		m.Chan6Raw,
		m.Chan7Raw,
		m.Chan8Raw,
		m.Chan9Raw,
		m.Chan10Raw,
		m.Chan11Raw,
		m.Chan12Raw,
		m.Rssi,
	)
}

// Pack (generated function)
func (m *HilRcInputsRaw) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 33)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.TimeUsec))
	binary.LittleEndian.PutUint16(payload[8:], uint16(m.Chan1Raw))
	binary.LittleEndian.PutUint16(payload[10:], uint16(m.Chan2Raw))
	binary.LittleEndian.PutUint16(payload[12:], uint16(m.Chan3Raw))
	binary.LittleEndian.PutUint16(payload[14:], uint16(m.Chan4Raw))
	binary.LittleEndian.PutUint16(payload[16:], uint16(m.Chan5Raw))
	binary.LittleEndian.PutUint16(payload[18:], uint16(m.Chan6Raw))
	binary.LittleEndian.PutUint16(payload[20:], uint16(m.Chan7Raw))
	binary.LittleEndian.PutUint16(payload[22:], uint16(m.Chan8Raw))
	binary.LittleEndian.PutUint16(payload[24:], uint16(m.Chan9Raw))
	binary.LittleEndian.PutUint16(payload[26:], uint16(m.Chan10Raw))
	binary.LittleEndian.PutUint16(payload[28:], uint16(m.Chan11Raw))
	binary.LittleEndian.PutUint16(payload[30:], uint16(m.Chan12Raw))
	payload[32] = byte(m.Rssi)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *HilRcInputsRaw) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 33 {
		payload = append(payload, mavlink.ZeroTail[:33-len(p.Payload)]...)
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(payload[0:]))
	m.Chan1Raw = uint16(binary.LittleEndian.Uint16(payload[8:]))
	m.Chan2Raw = uint16(binary.LittleEndian.Uint16(payload[10:]))
	m.Chan3Raw = uint16(binary.LittleEndian.Uint16(payload[12:]))
	m.Chan4Raw = uint16(binary.LittleEndian.Uint16(payload[14:]))
	m.Chan5Raw = uint16(binary.LittleEndian.Uint16(payload[16:]))
	m.Chan6Raw = uint16(binary.LittleEndian.Uint16(payload[18:]))
	m.Chan7Raw = uint16(binary.LittleEndian.Uint16(payload[20:]))
	m.Chan8Raw = uint16(binary.LittleEndian.Uint16(payload[22:]))
	m.Chan9Raw = uint16(binary.LittleEndian.Uint16(payload[24:]))
	m.Chan10Raw = uint16(binary.LittleEndian.Uint16(payload[26:]))
	m.Chan11Raw = uint16(binary.LittleEndian.Uint16(payload[28:]))
	m.Chan12Raw = uint16(binary.LittleEndian.Uint16(payload[30:]))
	m.Rssi = uint8(payload[32])
	return nil
}

// HilActuatorControls struct (generated typeinfo)
// Sent from autopilot to simulation. Hardware in the loop control outputs (replacement for HIL_CONTROLS)
type HilActuatorControls struct {
	TimeUsec uint64        // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
	Flags    uint64        // Flags as bitfield, 1: indicate simulation using lockstep.
	Controls [16]float32   // Control outputs -1 .. 1. Channel assignment depends on the simulated hardware.
	Mode     MAV_MODE_FLAG `gotype:"uint8"` // System mode. Includes arming state.
}

// MsgID (generated function)
func (m *HilActuatorControls) MsgID() mavlink.MessageID {
	return MSG_ID_HIL_ACTUATOR_CONTROLS
}

// String (generated function)
func (m *HilActuatorControls) String() string {
	return fmt.Sprintf(
		"&common.HilActuatorControls{ TimeUsec: %+v, Flags: %+v, Controls: %+v, Mode: %+v }",
		m.TimeUsec,
		m.Flags,
		m.Controls,
		m.Mode,
	)
}

// Pack (generated function)
func (m *HilActuatorControls) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 81)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.TimeUsec))
	binary.LittleEndian.PutUint64(payload[8:], uint64(m.Flags))
	for i, v := range m.Controls {
		binary.LittleEndian.PutUint32(payload[16+i*4:], math.Float32bits(v))
	}
	payload[80] = byte(m.Mode)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *HilActuatorControls) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 81 {
		payload = append(payload, mavlink.ZeroTail[:81-len(p.Payload)]...)
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(payload[0:]))
	m.Flags = uint64(binary.LittleEndian.Uint64(payload[8:]))
	for i := 0; i < len(m.Controls); i++ {
		m.Controls[i] = math.Float32frombits(binary.LittleEndian.Uint32(payload[16+i*4:]))
	}
	m.Mode = MAV_MODE_FLAG(payload[80])
	return nil
}

// OpticalFlow struct (generated typeinfo)
// Optical flow from a flow sensor (e.g. optical mouse sensor)
type OpticalFlow struct {
	TimeUsec       uint64  // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
	FlowCompMX     float32 // Flow in x-sensor direction, angular-speed compensated
	FlowCompMY     float32 // Flow in y-sensor direction, angular-speed compensated
	GroundDistance float32 // Ground distance. Positive value: distance known. Negative value: Unknown distance
	FlowX          int16   // Flow in x-sensor direction
	FlowY          int16   // Flow in y-sensor direction
	SensorID       uint8   // Sensor ID
	Quality        uint8   // Optical flow quality / confidence. 0: bad, 255: maximum quality
}

// MsgID (generated function)
func (m *OpticalFlow) MsgID() mavlink.MessageID {
	return MSG_ID_OPTICAL_FLOW
}

// String (generated function)
func (m *OpticalFlow) String() string {
	return fmt.Sprintf(
		"&common.OpticalFlow{ TimeUsec: %+v, FlowCompMX: %+v, FlowCompMY: %+v, GroundDistance: %+v, FlowX: %+v, FlowY: %+v, SensorID: %+v, Quality: %+v }",
		m.TimeUsec,
		m.FlowCompMX,
		m.FlowCompMY,
		m.GroundDistance,
		m.FlowX,
		m.FlowY,
		m.SensorID,
		m.Quality,
	)
}

// Pack (generated function)
func (m *OpticalFlow) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 26)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.TimeUsec))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.FlowCompMX))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.FlowCompMY))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(m.GroundDistance))
	binary.LittleEndian.PutUint16(payload[20:], uint16(m.FlowX))
	binary.LittleEndian.PutUint16(payload[22:], uint16(m.FlowY))
	payload[24] = byte(m.SensorID)
	payload[25] = byte(m.Quality)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *OpticalFlow) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 26 {
		payload = append(payload, mavlink.ZeroTail[:26-len(p.Payload)]...)
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(payload[0:]))
	m.FlowCompMX = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.FlowCompMY = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.GroundDistance = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	m.FlowX = int16(binary.LittleEndian.Uint16(payload[20:]))
	m.FlowY = int16(binary.LittleEndian.Uint16(payload[22:]))
	m.SensorID = uint8(payload[24])
	m.Quality = uint8(payload[25])
	return nil
}

// GlobalVisionPositionEstimate struct (generated typeinfo)
// Global position/attitude estimate from a vision source.
type GlobalVisionPositionEstimate struct {
	Usec  uint64  // Timestamp (UNIX time or since system boot)
	X     float32 // Global X position
	Y     float32 // Global Y position
	Z     float32 // Global Z position
	Roll  float32 // Roll angle
	Pitch float32 // Pitch angle
	Yaw   float32 // Yaw angle
}

// MsgID (generated function)
func (m *GlobalVisionPositionEstimate) MsgID() mavlink.MessageID {
	return MSG_ID_GLOBAL_VISION_POSITION_ESTIMATE
}

// String (generated function)
func (m *GlobalVisionPositionEstimate) String() string {
	return fmt.Sprintf(
		"&common.GlobalVisionPositionEstimate{ Usec: %+v, X: %+v, Y: %+v, Z: %+v, Roll: %+v, Pitch: %+v, Yaw: %+v }",
		m.Usec,
		m.X,
		m.Y,
		m.Z,
		m.Roll,
		m.Pitch,
		m.Yaw,
	)
}

// Pack (generated function)
func (m *GlobalVisionPositionEstimate) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 32)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.Usec))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.X))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.Y))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(m.Z))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(m.Roll))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(m.Pitch))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(m.Yaw))
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *GlobalVisionPositionEstimate) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 32 {
		payload = append(payload, mavlink.ZeroTail[:32-len(p.Payload)]...)
	}
	m.Usec = uint64(binary.LittleEndian.Uint64(payload[0:]))
	m.X = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.Y = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.Z = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	m.Roll = math.Float32frombits(binary.LittleEndian.Uint32(payload[20:]))
	m.Pitch = math.Float32frombits(binary.LittleEndian.Uint32(payload[24:]))
	m.Yaw = math.Float32frombits(binary.LittleEndian.Uint32(payload[28:]))
	return nil
}

// VisionPositionEstimate struct (generated typeinfo)
// Local position/attitude estimate from a vision source.
type VisionPositionEstimate struct {
	Usec  uint64  // Timestamp (UNIX time or time since system boot)
	X     float32 // Local X position
	Y     float32 // Local Y position
	Z     float32 // Local Z position
	Roll  float32 // Roll angle
	Pitch float32 // Pitch angle
	Yaw   float32 // Yaw angle
}

// MsgID (generated function)
func (m *VisionPositionEstimate) MsgID() mavlink.MessageID {
	return MSG_ID_VISION_POSITION_ESTIMATE
}

// String (generated function)
func (m *VisionPositionEstimate) String() string {
	return fmt.Sprintf(
		"&common.VisionPositionEstimate{ Usec: %+v, X: %+v, Y: %+v, Z: %+v, Roll: %+v, Pitch: %+v, Yaw: %+v }",
		m.Usec,
		m.X,
		m.Y,
		m.Z,
		m.Roll,
		m.Pitch,
		m.Yaw,
	)
}

// Pack (generated function)
func (m *VisionPositionEstimate) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 32)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.Usec))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.X))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.Y))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(m.Z))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(m.Roll))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(m.Pitch))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(m.Yaw))
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *VisionPositionEstimate) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 32 {
		payload = append(payload, mavlink.ZeroTail[:32-len(p.Payload)]...)
	}
	m.Usec = uint64(binary.LittleEndian.Uint64(payload[0:]))
	m.X = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.Y = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.Z = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	m.Roll = math.Float32frombits(binary.LittleEndian.Uint32(payload[20:]))
	m.Pitch = math.Float32frombits(binary.LittleEndian.Uint32(payload[24:]))
	m.Yaw = math.Float32frombits(binary.LittleEndian.Uint32(payload[28:]))
	return nil
}

// VisionSpeedEstimate struct (generated typeinfo)
// Speed estimate from a vision source.
type VisionSpeedEstimate struct {
	Usec uint64  // Timestamp (UNIX time or time since system boot)
	X    float32 // Global X speed
	Y    float32 // Global Y speed
	Z    float32 // Global Z speed
}

// MsgID (generated function)
func (m *VisionSpeedEstimate) MsgID() mavlink.MessageID {
	return MSG_ID_VISION_SPEED_ESTIMATE
}

// String (generated function)
func (m *VisionSpeedEstimate) String() string {
	return fmt.Sprintf(
		"&common.VisionSpeedEstimate{ Usec: %+v, X: %+v, Y: %+v, Z: %+v }",
		m.Usec,
		m.X,
		m.Y,
		m.Z,
	)
}

// Pack (generated function)
func (m *VisionSpeedEstimate) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 20)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.Usec))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.X))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.Y))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(m.Z))
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *VisionSpeedEstimate) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 20 {
		payload = append(payload, mavlink.ZeroTail[:20-len(p.Payload)]...)
	}
	m.Usec = uint64(binary.LittleEndian.Uint64(payload[0:]))
	m.X = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.Y = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.Z = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	return nil
}

// ViconPositionEstimate struct (generated typeinfo)
// Global position estimate from a Vicon motion system source.
type ViconPositionEstimate struct {
	Usec  uint64  // Timestamp (UNIX time or time since system boot)
	X     float32 // Global X position
	Y     float32 // Global Y position
	Z     float32 // Global Z position
	Roll  float32 // Roll angle
	Pitch float32 // Pitch angle
	Yaw   float32 // Yaw angle
}

// MsgID (generated function)
func (m *ViconPositionEstimate) MsgID() mavlink.MessageID {
	return MSG_ID_VICON_POSITION_ESTIMATE
}

// String (generated function)
func (m *ViconPositionEstimate) String() string {
	return fmt.Sprintf(
		"&common.ViconPositionEstimate{ Usec: %+v, X: %+v, Y: %+v, Z: %+v, Roll: %+v, Pitch: %+v, Yaw: %+v }",
		m.Usec,
		m.X,
		m.Y,
		m.Z,
		m.Roll,
		m.Pitch,
		m.Yaw,
	)
}

// Pack (generated function)
func (m *ViconPositionEstimate) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 32)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.Usec))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.X))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.Y))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(m.Z))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(m.Roll))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(m.Pitch))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(m.Yaw))
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ViconPositionEstimate) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 32 {
		payload = append(payload, mavlink.ZeroTail[:32-len(p.Payload)]...)
	}
	m.Usec = uint64(binary.LittleEndian.Uint64(payload[0:]))
	m.X = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.Y = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.Z = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	m.Roll = math.Float32frombits(binary.LittleEndian.Uint32(payload[20:]))
	m.Pitch = math.Float32frombits(binary.LittleEndian.Uint32(payload[24:]))
	m.Yaw = math.Float32frombits(binary.LittleEndian.Uint32(payload[28:]))
	return nil
}

// HighresImu struct (generated typeinfo)
// The IMU readings in SI units in NED body frame
type HighresImu struct {
	TimeUsec      uint64  // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
	Xacc          float32 // X acceleration
	Yacc          float32 // Y acceleration
	Zacc          float32 // Z acceleration
	Xgyro         float32 // Angular speed around X axis
	Ygyro         float32 // Angular speed around Y axis
	Zgyro         float32 // Angular speed around Z axis
	Xmag          float32 // X Magnetic field
	Ymag          float32 // Y Magnetic field
	Zmag          float32 // Z Magnetic field
	AbsPressure   float32 // Absolute pressure
	DiffPressure  float32 // Differential pressure
	PressureAlt   float32 // Altitude calculated from pressure
	Temperature   float32 // Temperature
	FieldsUpdated uint16  // Bitmap for fields that have updated since last message, bit 0 = xacc, bit 12: temperature
}

// MsgID (generated function)
func (m *HighresImu) MsgID() mavlink.MessageID {
	return MSG_ID_HIGHRES_IMU
}

// String (generated function)
func (m *HighresImu) String() string {
	return fmt.Sprintf(
		"&common.HighresImu{ TimeUsec: %+v, Xacc: %+v, Yacc: %+v, Zacc: %+v, Xgyro: %+v, Ygyro: %+v, Zgyro: %+v, Xmag: %+v, Ymag: %+v, Zmag: %+v, AbsPressure: %+v, DiffPressure: %+v, PressureAlt: %+v, Temperature: %+v, FieldsUpdated: %+v }",
		m.TimeUsec,
		m.Xacc,
		m.Yacc,
		m.Zacc,
		m.Xgyro,
		m.Ygyro,
		m.Zgyro,
		m.Xmag,
		m.Ymag,
		m.Zmag,
		m.AbsPressure,
		m.DiffPressure,
		m.PressureAlt,
		m.Temperature,
		m.FieldsUpdated,
	)
}

// Pack (generated function)
func (m *HighresImu) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 62)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.TimeUsec))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.Xacc))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.Yacc))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(m.Zacc))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(m.Xgyro))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(m.Ygyro))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(m.Zgyro))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(m.Xmag))
	binary.LittleEndian.PutUint32(payload[36:], math.Float32bits(m.Ymag))
	binary.LittleEndian.PutUint32(payload[40:], math.Float32bits(m.Zmag))
	binary.LittleEndian.PutUint32(payload[44:], math.Float32bits(m.AbsPressure))
	binary.LittleEndian.PutUint32(payload[48:], math.Float32bits(m.DiffPressure))
	binary.LittleEndian.PutUint32(payload[52:], math.Float32bits(m.PressureAlt))
	binary.LittleEndian.PutUint32(payload[56:], math.Float32bits(m.Temperature))
	binary.LittleEndian.PutUint16(payload[60:], uint16(m.FieldsUpdated))
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *HighresImu) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 62 {
		payload = append(payload, mavlink.ZeroTail[:62-len(p.Payload)]...)
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(payload[0:]))
	m.Xacc = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.Yacc = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.Zacc = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	m.Xgyro = math.Float32frombits(binary.LittleEndian.Uint32(payload[20:]))
	m.Ygyro = math.Float32frombits(binary.LittleEndian.Uint32(payload[24:]))
	m.Zgyro = math.Float32frombits(binary.LittleEndian.Uint32(payload[28:]))
	m.Xmag = math.Float32frombits(binary.LittleEndian.Uint32(payload[32:]))
	m.Ymag = math.Float32frombits(binary.LittleEndian.Uint32(payload[36:]))
	m.Zmag = math.Float32frombits(binary.LittleEndian.Uint32(payload[40:]))
	m.AbsPressure = math.Float32frombits(binary.LittleEndian.Uint32(payload[44:]))
	m.DiffPressure = math.Float32frombits(binary.LittleEndian.Uint32(payload[48:]))
	m.PressureAlt = math.Float32frombits(binary.LittleEndian.Uint32(payload[52:]))
	m.Temperature = math.Float32frombits(binary.LittleEndian.Uint32(payload[56:]))
	m.FieldsUpdated = uint16(binary.LittleEndian.Uint16(payload[60:]))
	return nil
}

// OpticalFlowRad struct (generated typeinfo)
// Optical flow from an angular rate flow sensor (e.g. PX4FLOW or mouse sensor)
type OpticalFlowRad struct {
	TimeUsec            uint64  // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
	IntegrationTimeUs   uint32  // Integration time. Divide integrated_x and integrated_y by the integration time to obtain average flow. The integration time also indicates the.
	IntegratedX         float32 // Flow around X axis (Sensor RH rotation about the X axis induces a positive flow. Sensor linear motion along the positive Y axis induces a negative flow.)
	IntegratedY         float32 // Flow around Y axis (Sensor RH rotation about the Y axis induces a positive flow. Sensor linear motion along the positive X axis induces a positive flow.)
	IntegratedXgyro     float32 // RH rotation around X axis
	IntegratedYgyro     float32 // RH rotation around Y axis
	IntegratedZgyro     float32 // RH rotation around Z axis
	TimeDeltaDistanceUs uint32  // Time since the distance was sampled.
	Distance            float32 // Distance to the center of the flow field. Positive value (including zero): distance known. Negative value: Unknown distance.
	Temperature         int16   // Temperature
	SensorID            uint8   // Sensor ID
	Quality             uint8   // Optical flow quality / confidence. 0: no valid flow, 255: maximum quality
}

// MsgID (generated function)
func (m *OpticalFlowRad) MsgID() mavlink.MessageID {
	return MSG_ID_OPTICAL_FLOW_RAD
}

// String (generated function)
func (m *OpticalFlowRad) String() string {
	return fmt.Sprintf(
		"&common.OpticalFlowRad{ TimeUsec: %+v, IntegrationTimeUs: %+v, IntegratedX: %+v, IntegratedY: %+v, IntegratedXgyro: %+v, IntegratedYgyro: %+v, IntegratedZgyro: %+v, TimeDeltaDistanceUs: %+v, Distance: %+v, Temperature: %+v, SensorID: %+v, Quality: %+v }",
		m.TimeUsec,
		m.IntegrationTimeUs,
		m.IntegratedX,
		m.IntegratedY,
		m.IntegratedXgyro,
		m.IntegratedYgyro,
		m.IntegratedZgyro,
		m.TimeDeltaDistanceUs,
		m.Distance,
		m.Temperature,
		m.SensorID,
		m.Quality,
	)
}

// Pack (generated function)
func (m *OpticalFlowRad) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 44)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.TimeUsec))
	binary.LittleEndian.PutUint32(payload[8:], uint32(m.IntegrationTimeUs))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.IntegratedX))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(m.IntegratedY))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(m.IntegratedXgyro))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(m.IntegratedYgyro))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(m.IntegratedZgyro))
	binary.LittleEndian.PutUint32(payload[32:], uint32(m.TimeDeltaDistanceUs))
	binary.LittleEndian.PutUint32(payload[36:], math.Float32bits(m.Distance))
	binary.LittleEndian.PutUint16(payload[40:], uint16(m.Temperature))
	payload[42] = byte(m.SensorID)
	payload[43] = byte(m.Quality)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *OpticalFlowRad) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 44 {
		payload = append(payload, mavlink.ZeroTail[:44-len(p.Payload)]...)
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(payload[0:]))
	m.IntegrationTimeUs = uint32(binary.LittleEndian.Uint32(payload[8:]))
	m.IntegratedX = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.IntegratedY = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	m.IntegratedXgyro = math.Float32frombits(binary.LittleEndian.Uint32(payload[20:]))
	m.IntegratedYgyro = math.Float32frombits(binary.LittleEndian.Uint32(payload[24:]))
	m.IntegratedZgyro = math.Float32frombits(binary.LittleEndian.Uint32(payload[28:]))
	m.TimeDeltaDistanceUs = uint32(binary.LittleEndian.Uint32(payload[32:]))
	m.Distance = math.Float32frombits(binary.LittleEndian.Uint32(payload[36:]))
	m.Temperature = int16(binary.LittleEndian.Uint16(payload[40:]))
	m.SensorID = uint8(payload[42])
	m.Quality = uint8(payload[43])
	return nil
}

// HilSensor struct (generated typeinfo)
// The IMU readings in SI units in NED body frame
type HilSensor struct {
	TimeUsec      uint64  // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
	Xacc          float32 // X acceleration
	Yacc          float32 // Y acceleration
	Zacc          float32 // Z acceleration
	Xgyro         float32 // Angular speed around X axis in body frame
	Ygyro         float32 // Angular speed around Y axis in body frame
	Zgyro         float32 // Angular speed around Z axis in body frame
	Xmag          float32 // X Magnetic field
	Ymag          float32 // Y Magnetic field
	Zmag          float32 // Z Magnetic field
	AbsPressure   float32 // Absolute pressure
	DiffPressure  float32 // Differential pressure (airspeed)
	PressureAlt   float32 // Altitude calculated from pressure
	Temperature   float32 // Temperature
	FieldsUpdated uint32  // Bitmap for fields that have updated since last message, bit 0 = xacc, bit 12: temperature, bit 31: full reset of attitude/position/velocities/etc was performed in sim.
}

// MsgID (generated function)
func (m *HilSensor) MsgID() mavlink.MessageID {
	return MSG_ID_HIL_SENSOR
}

// String (generated function)
func (m *HilSensor) String() string {
	return fmt.Sprintf(
		"&common.HilSensor{ TimeUsec: %+v, Xacc: %+v, Yacc: %+v, Zacc: %+v, Xgyro: %+v, Ygyro: %+v, Zgyro: %+v, Xmag: %+v, Ymag: %+v, Zmag: %+v, AbsPressure: %+v, DiffPressure: %+v, PressureAlt: %+v, Temperature: %+v, FieldsUpdated: %+v }",
		m.TimeUsec,
		m.Xacc,
		m.Yacc,
		m.Zacc,
		m.Xgyro,
		m.Ygyro,
		m.Zgyro,
		m.Xmag,
		m.Ymag,
		m.Zmag,
		m.AbsPressure,
		m.DiffPressure,
		m.PressureAlt,
		m.Temperature,
		m.FieldsUpdated,
	)
}

// Pack (generated function)
func (m *HilSensor) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 64)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.TimeUsec))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.Xacc))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.Yacc))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(m.Zacc))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(m.Xgyro))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(m.Ygyro))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(m.Zgyro))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(m.Xmag))
	binary.LittleEndian.PutUint32(payload[36:], math.Float32bits(m.Ymag))
	binary.LittleEndian.PutUint32(payload[40:], math.Float32bits(m.Zmag))
	binary.LittleEndian.PutUint32(payload[44:], math.Float32bits(m.AbsPressure))
	binary.LittleEndian.PutUint32(payload[48:], math.Float32bits(m.DiffPressure))
	binary.LittleEndian.PutUint32(payload[52:], math.Float32bits(m.PressureAlt))
	binary.LittleEndian.PutUint32(payload[56:], math.Float32bits(m.Temperature))
	binary.LittleEndian.PutUint32(payload[60:], uint32(m.FieldsUpdated))
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *HilSensor) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 64 {
		payload = append(payload, mavlink.ZeroTail[:64-len(p.Payload)]...)
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(payload[0:]))
	m.Xacc = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.Yacc = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.Zacc = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	m.Xgyro = math.Float32frombits(binary.LittleEndian.Uint32(payload[20:]))
	m.Ygyro = math.Float32frombits(binary.LittleEndian.Uint32(payload[24:]))
	m.Zgyro = math.Float32frombits(binary.LittleEndian.Uint32(payload[28:]))
	m.Xmag = math.Float32frombits(binary.LittleEndian.Uint32(payload[32:]))
	m.Ymag = math.Float32frombits(binary.LittleEndian.Uint32(payload[36:]))
	m.Zmag = math.Float32frombits(binary.LittleEndian.Uint32(payload[40:]))
	m.AbsPressure = math.Float32frombits(binary.LittleEndian.Uint32(payload[44:]))
	m.DiffPressure = math.Float32frombits(binary.LittleEndian.Uint32(payload[48:]))
	m.PressureAlt = math.Float32frombits(binary.LittleEndian.Uint32(payload[52:]))
	m.Temperature = math.Float32frombits(binary.LittleEndian.Uint32(payload[56:]))
	m.FieldsUpdated = uint32(binary.LittleEndian.Uint32(payload[60:]))
	return nil
}

// SimState struct (generated typeinfo)
// Status of simulation environment, if used
type SimState struct {
	Q1         float32 // True attitude quaternion component 1, w (1 in null-rotation)
	Q2         float32 // True attitude quaternion component 2, x (0 in null-rotation)
	Q3         float32 // True attitude quaternion component 3, y (0 in null-rotation)
	Q4         float32 // True attitude quaternion component 4, z (0 in null-rotation)
	Roll       float32 // Attitude roll expressed as Euler angles, not recommended except for human-readable outputs
	Pitch      float32 // Attitude pitch expressed as Euler angles, not recommended except for human-readable outputs
	Yaw        float32 // Attitude yaw expressed as Euler angles, not recommended except for human-readable outputs
	Xacc       float32 // X acceleration
	Yacc       float32 // Y acceleration
	Zacc       float32 // Z acceleration
	Xgyro      float32 // Angular speed around X axis
	Ygyro      float32 // Angular speed around Y axis
	Zgyro      float32 // Angular speed around Z axis
	Lat        float32 // Latitude
	Lon        float32 // Longitude
	Alt        float32 // Altitude
	StdDevHorz float32 // Horizontal position standard deviation
	StdDevVert float32 // Vertical position standard deviation
	Vn         float32 // True velocity in north direction in earth-fixed NED frame
	Ve         float32 // True velocity in east direction in earth-fixed NED frame
	Vd         float32 // True velocity in down direction in earth-fixed NED frame
}

// MsgID (generated function)
func (m *SimState) MsgID() mavlink.MessageID {
	return MSG_ID_SIM_STATE
}

// String (generated function)
func (m *SimState) String() string {
	return fmt.Sprintf(
		"&common.SimState{ Q1: %+v, Q2: %+v, Q3: %+v, Q4: %+v, Roll: %+v, Pitch: %+v, Yaw: %+v, Xacc: %+v, Yacc: %+v, Zacc: %+v, Xgyro: %+v, Ygyro: %+v, Zgyro: %+v, Lat: %+v, Lon: %+v, Alt: %+v, StdDevHorz: %+v, StdDevVert: %+v, Vn: %+v, Ve: %+v, Vd: %+v }",
		m.Q1,
		m.Q2,
		m.Q3,
		m.Q4,
		m.Roll,
		m.Pitch,
		m.Yaw,
		m.Xacc,
		m.Yacc,
		m.Zacc,
		m.Xgyro,
		m.Ygyro,
		m.Zgyro,
		m.Lat,
		m.Lon,
		m.Alt,
		m.StdDevHorz,
		m.StdDevVert,
		m.Vn,
		m.Ve,
		m.Vd,
	)
}

// Pack (generated function)
func (m *SimState) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 84)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(m.Q1))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(m.Q2))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.Q3))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.Q4))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(m.Roll))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(m.Pitch))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(m.Yaw))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(m.Xacc))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(m.Yacc))
	binary.LittleEndian.PutUint32(payload[36:], math.Float32bits(m.Zacc))
	binary.LittleEndian.PutUint32(payload[40:], math.Float32bits(m.Xgyro))
	binary.LittleEndian.PutUint32(payload[44:], math.Float32bits(m.Ygyro))
	binary.LittleEndian.PutUint32(payload[48:], math.Float32bits(m.Zgyro))
	binary.LittleEndian.PutUint32(payload[52:], math.Float32bits(m.Lat))
	binary.LittleEndian.PutUint32(payload[56:], math.Float32bits(m.Lon))
	binary.LittleEndian.PutUint32(payload[60:], math.Float32bits(m.Alt))
	binary.LittleEndian.PutUint32(payload[64:], math.Float32bits(m.StdDevHorz))
	binary.LittleEndian.PutUint32(payload[68:], math.Float32bits(m.StdDevVert))
	binary.LittleEndian.PutUint32(payload[72:], math.Float32bits(m.Vn))
	binary.LittleEndian.PutUint32(payload[76:], math.Float32bits(m.Ve))
	binary.LittleEndian.PutUint32(payload[80:], math.Float32bits(m.Vd))
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *SimState) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 84 {
		payload = append(payload, mavlink.ZeroTail[:84-len(p.Payload)]...)
	}
	m.Q1 = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.Q2 = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	m.Q3 = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.Q4 = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.Roll = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	m.Pitch = math.Float32frombits(binary.LittleEndian.Uint32(payload[20:]))
	m.Yaw = math.Float32frombits(binary.LittleEndian.Uint32(payload[24:]))
	m.Xacc = math.Float32frombits(binary.LittleEndian.Uint32(payload[28:]))
	m.Yacc = math.Float32frombits(binary.LittleEndian.Uint32(payload[32:]))
	m.Zacc = math.Float32frombits(binary.LittleEndian.Uint32(payload[36:]))
	m.Xgyro = math.Float32frombits(binary.LittleEndian.Uint32(payload[40:]))
	m.Ygyro = math.Float32frombits(binary.LittleEndian.Uint32(payload[44:]))
	m.Zgyro = math.Float32frombits(binary.LittleEndian.Uint32(payload[48:]))
	m.Lat = math.Float32frombits(binary.LittleEndian.Uint32(payload[52:]))
	m.Lon = math.Float32frombits(binary.LittleEndian.Uint32(payload[56:]))
	m.Alt = math.Float32frombits(binary.LittleEndian.Uint32(payload[60:]))
	m.StdDevHorz = math.Float32frombits(binary.LittleEndian.Uint32(payload[64:]))
	m.StdDevVert = math.Float32frombits(binary.LittleEndian.Uint32(payload[68:]))
	m.Vn = math.Float32frombits(binary.LittleEndian.Uint32(payload[72:]))
	m.Ve = math.Float32frombits(binary.LittleEndian.Uint32(payload[76:]))
	m.Vd = math.Float32frombits(binary.LittleEndian.Uint32(payload[80:]))
	return nil
}

// RadioStatus struct (generated typeinfo)
// Status generated by radio and injected into MAVLink stream.
type RadioStatus struct {
	Rxerrors uint16 // Count of radio packet receive errors (since boot).
	Fixed    uint16 // Count of error corrected radio packets (since boot).
	Rssi     uint8  // Local (message sender) recieved signal strength indication in device-dependent units/scale. Values: [0-254], 255: invalid/unknown.
	Remrssi  uint8  // Remote (message receiver) signal strength indication in device-dependent units/scale. Values: [0-254], 255: invalid/unknown.
	Txbuf    uint8  // Remaining free transmitter buffer space.
	Noise    uint8  // Local background noise level. These are device dependent RSSI values (scale as approx 2x dB on SiK radios). Values: [0-254], 255: invalid/unknown.
	Remnoise uint8  // Remote background noise level. These are device dependent RSSI values (scale as approx 2x dB on SiK radios). Values: [0-254], 255: invalid/unknown.
}

// MsgID (generated function)
func (m *RadioStatus) MsgID() mavlink.MessageID {
	return MSG_ID_RADIO_STATUS
}

// String (generated function)
func (m *RadioStatus) String() string {
	return fmt.Sprintf(
		"&common.RadioStatus{ Rxerrors: %+v, Fixed: %+v, Rssi: %+v, Remrssi: %+v, Txbuf: %+v, Noise: %+v, Remnoise: %+v }",
		m.Rxerrors,
		m.Fixed,
		m.Rssi,
		m.Remrssi,
		m.Txbuf,
		m.Noise,
		m.Remnoise,
	)
}

// Pack (generated function)
func (m *RadioStatus) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 9)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.Rxerrors))
	binary.LittleEndian.PutUint16(payload[2:], uint16(m.Fixed))
	payload[4] = byte(m.Rssi)
	payload[5] = byte(m.Remrssi)
	payload[6] = byte(m.Txbuf)
	payload[7] = byte(m.Noise)
	payload[8] = byte(m.Remnoise)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *RadioStatus) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 9 {
		payload = append(payload, mavlink.ZeroTail[:9-len(p.Payload)]...)
	}
	m.Rxerrors = uint16(binary.LittleEndian.Uint16(payload[0:]))
	m.Fixed = uint16(binary.LittleEndian.Uint16(payload[2:]))
	m.Rssi = uint8(payload[4])
	m.Remrssi = uint8(payload[5])
	m.Txbuf = uint8(payload[6])
	m.Noise = uint8(payload[7])
	m.Remnoise = uint8(payload[8])
	return nil
}

// FileTransferProtocol struct (generated typeinfo)
// File transfer message
type FileTransferProtocol struct {
	TargetNetwork   uint8      // Network ID (0 for broadcast)
	TargetSystem    uint8      // System ID (0 for broadcast)
	TargetComponent uint8      // Component ID (0 for broadcast)
	Payload         [251]uint8 // Variable length payload. The length is defined by the remaining message length when subtracting the header and other fields.  The entire content of this block is opaque unless you understand any the encoding message_type.  The particular encoding used can be extension specific and might not always be documented as part of the mavlink specification.
}

// MsgID (generated function)
func (m *FileTransferProtocol) MsgID() mavlink.MessageID {
	return MSG_ID_FILE_TRANSFER_PROTOCOL
}

// String (generated function)
func (m *FileTransferProtocol) String() string {
	return fmt.Sprintf(
		"&common.FileTransferProtocol{ TargetNetwork: %+v, TargetSystem: %+v, TargetComponent: %+v, Payload: %0X (\"%s\") }",
		m.TargetNetwork,
		m.TargetSystem,
		m.TargetComponent,
		m.Payload, string(m.Payload[:]),
	)
}

// Pack (generated function)
func (m *FileTransferProtocol) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 254)
	payload[0] = byte(m.TargetNetwork)
	payload[1] = byte(m.TargetSystem)
	payload[2] = byte(m.TargetComponent)
	copy(payload[3:], m.Payload[:])
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *FileTransferProtocol) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 254 {
		payload = append(payload, mavlink.ZeroTail[:254-len(p.Payload)]...)
	}
	m.TargetNetwork = uint8(payload[0])
	m.TargetSystem = uint8(payload[1])
	m.TargetComponent = uint8(payload[2])
	copy(m.Payload[:], payload[3:254])
	return nil
}

// Timesync struct (generated typeinfo)
// Time synchronization message.
type Timesync struct {
	Tc1 int64 // Time sync timestamp 1
	Ts1 int64 // Time sync timestamp 2
}

// MsgID (generated function)
func (m *Timesync) MsgID() mavlink.MessageID {
	return MSG_ID_TIMESYNC
}

// String (generated function)
func (m *Timesync) String() string {
	return fmt.Sprintf(
		"&common.Timesync{ Tc1: %+v, Ts1: %+v }",
		m.Tc1,
		m.Ts1,
	)
}

// Pack (generated function)
func (m *Timesync) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 16)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.Tc1))
	binary.LittleEndian.PutUint64(payload[8:], uint64(m.Ts1))
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *Timesync) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 16 {
		payload = append(payload, mavlink.ZeroTail[:16-len(p.Payload)]...)
	}
	m.Tc1 = int64(binary.LittleEndian.Uint64(payload[0:]))
	m.Ts1 = int64(binary.LittleEndian.Uint64(payload[8:]))
	return nil
}

// CameraTrigger struct (generated typeinfo)
// Camera-IMU triggering and synchronisation message.
type CameraTrigger struct {
	TimeUsec uint64 // Timestamp for image frame (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
	Seq      uint32 // Image frame sequence
}

// MsgID (generated function)
func (m *CameraTrigger) MsgID() mavlink.MessageID {
	return MSG_ID_CAMERA_TRIGGER
}

// String (generated function)
func (m *CameraTrigger) String() string {
	return fmt.Sprintf(
		"&common.CameraTrigger{ TimeUsec: %+v, Seq: %+v }",
		m.TimeUsec,
		m.Seq,
	)
}

// Pack (generated function)
func (m *CameraTrigger) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 12)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.TimeUsec))
	binary.LittleEndian.PutUint32(payload[8:], uint32(m.Seq))
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *CameraTrigger) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 12 {
		payload = append(payload, mavlink.ZeroTail[:12-len(p.Payload)]...)
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(payload[0:]))
	m.Seq = uint32(binary.LittleEndian.Uint32(payload[8:]))
	return nil
}

// HilGps struct (generated typeinfo)
// The global position, as returned by the Global Positioning System (GPS). This is
//                  NOT the global position estimate of the sytem, but rather a RAW sensor value. See message GLOBAL_POSITION for the global position estimate.
type HilGps struct {
	TimeUsec          uint64 // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
	Lat               int32  // Latitude (WGS84)
	Lon               int32  // Longitude (WGS84)
	Alt               int32  // Altitude (MSL). Positive for up.
	Eph               uint16 // GPS HDOP horizontal dilution of position. If unknown, set to: 65535
	Epv               uint16 // GPS VDOP vertical dilution of position. If unknown, set to: 65535
	Vel               uint16 // GPS ground speed. If unknown, set to: 65535
	Vn                int16  // GPS velocity in north direction in earth-fixed NED frame
	Ve                int16  // GPS velocity in east direction in earth-fixed NED frame
	Vd                int16  // GPS velocity in down direction in earth-fixed NED frame
	Cog               uint16 // Course over ground (NOT heading, but direction of movement), 0.0..359.99 degrees. If unknown, set to: 65535
	FixType           uint8  // 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
	SatellitesVisible uint8  // Number of satellites visible. If unknown, set to 255
}

// MsgID (generated function)
func (m *HilGps) MsgID() mavlink.MessageID {
	return MSG_ID_HIL_GPS
}

// String (generated function)
func (m *HilGps) String() string {
	return fmt.Sprintf(
		"&common.HilGps{ TimeUsec: %+v, Lat: %+v, Lon: %+v, Alt: %+v, Eph: %+v, Epv: %+v, Vel: %+v, Vn: %+v, Ve: %+v, Vd: %+v, Cog: %+v, FixType: %+v, SatellitesVisible: %+v }",
		m.TimeUsec,
		m.Lat,
		m.Lon,
		m.Alt,
		m.Eph,
		m.Epv,
		m.Vel,
		m.Vn,
		m.Ve,
		m.Vd,
		m.Cog,
		m.FixType,
		m.SatellitesVisible,
	)
}

// Pack (generated function)
func (m *HilGps) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 36)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.TimeUsec))
	binary.LittleEndian.PutUint32(payload[8:], uint32(m.Lat))
	binary.LittleEndian.PutUint32(payload[12:], uint32(m.Lon))
	binary.LittleEndian.PutUint32(payload[16:], uint32(m.Alt))
	binary.LittleEndian.PutUint16(payload[20:], uint16(m.Eph))
	binary.LittleEndian.PutUint16(payload[22:], uint16(m.Epv))
	binary.LittleEndian.PutUint16(payload[24:], uint16(m.Vel))
	binary.LittleEndian.PutUint16(payload[26:], uint16(m.Vn))
	binary.LittleEndian.PutUint16(payload[28:], uint16(m.Ve))
	binary.LittleEndian.PutUint16(payload[30:], uint16(m.Vd))
	binary.LittleEndian.PutUint16(payload[32:], uint16(m.Cog))
	payload[34] = byte(m.FixType)
	payload[35] = byte(m.SatellitesVisible)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *HilGps) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 36 {
		payload = append(payload, mavlink.ZeroTail[:36-len(p.Payload)]...)
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(payload[0:]))
	m.Lat = int32(binary.LittleEndian.Uint32(payload[8:]))
	m.Lon = int32(binary.LittleEndian.Uint32(payload[12:]))
	m.Alt = int32(binary.LittleEndian.Uint32(payload[16:]))
	m.Eph = uint16(binary.LittleEndian.Uint16(payload[20:]))
	m.Epv = uint16(binary.LittleEndian.Uint16(payload[22:]))
	m.Vel = uint16(binary.LittleEndian.Uint16(payload[24:]))
	m.Vn = int16(binary.LittleEndian.Uint16(payload[26:]))
	m.Ve = int16(binary.LittleEndian.Uint16(payload[28:]))
	m.Vd = int16(binary.LittleEndian.Uint16(payload[30:]))
	m.Cog = uint16(binary.LittleEndian.Uint16(payload[32:]))
	m.FixType = uint8(payload[34])
	m.SatellitesVisible = uint8(payload[35])
	return nil
}

// HilOpticalFlow struct (generated typeinfo)
// Simulated optical flow from a flow sensor (e.g. PX4FLOW or optical mouse sensor)
type HilOpticalFlow struct {
	TimeUsec            uint64  // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
	IntegrationTimeUs   uint32  // Integration time. Divide integrated_x and integrated_y by the integration time to obtain average flow. The integration time also indicates the.
	IntegratedX         float32 // Flow in radians around X axis (Sensor RH rotation about the X axis induces a positive flow. Sensor linear motion along the positive Y axis induces a negative flow.)
	IntegratedY         float32 // Flow in radians around Y axis (Sensor RH rotation about the Y axis induces a positive flow. Sensor linear motion along the positive X axis induces a positive flow.)
	IntegratedXgyro     float32 // RH rotation around X axis
	IntegratedYgyro     float32 // RH rotation around Y axis
	IntegratedZgyro     float32 // RH rotation around Z axis
	TimeDeltaDistanceUs uint32  // Time since the distance was sampled.
	Distance            float32 // Distance to the center of the flow field. Positive value (including zero): distance known. Negative value: Unknown distance.
	Temperature         int16   // Temperature
	SensorID            uint8   // Sensor ID
	Quality             uint8   // Optical flow quality / confidence. 0: no valid flow, 255: maximum quality
}

// MsgID (generated function)
func (m *HilOpticalFlow) MsgID() mavlink.MessageID {
	return MSG_ID_HIL_OPTICAL_FLOW
}

// String (generated function)
func (m *HilOpticalFlow) String() string {
	return fmt.Sprintf(
		"&common.HilOpticalFlow{ TimeUsec: %+v, IntegrationTimeUs: %+v, IntegratedX: %+v, IntegratedY: %+v, IntegratedXgyro: %+v, IntegratedYgyro: %+v, IntegratedZgyro: %+v, TimeDeltaDistanceUs: %+v, Distance: %+v, Temperature: %+v, SensorID: %+v, Quality: %+v }",
		m.TimeUsec,
		m.IntegrationTimeUs,
		m.IntegratedX,
		m.IntegratedY,
		m.IntegratedXgyro,
		m.IntegratedYgyro,
		m.IntegratedZgyro,
		m.TimeDeltaDistanceUs,
		m.Distance,
		m.Temperature,
		m.SensorID,
		m.Quality,
	)
}

// Pack (generated function)
func (m *HilOpticalFlow) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 44)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.TimeUsec))
	binary.LittleEndian.PutUint32(payload[8:], uint32(m.IntegrationTimeUs))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.IntegratedX))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(m.IntegratedY))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(m.IntegratedXgyro))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(m.IntegratedYgyro))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(m.IntegratedZgyro))
	binary.LittleEndian.PutUint32(payload[32:], uint32(m.TimeDeltaDistanceUs))
	binary.LittleEndian.PutUint32(payload[36:], math.Float32bits(m.Distance))
	binary.LittleEndian.PutUint16(payload[40:], uint16(m.Temperature))
	payload[42] = byte(m.SensorID)
	payload[43] = byte(m.Quality)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *HilOpticalFlow) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 44 {
		payload = append(payload, mavlink.ZeroTail[:44-len(p.Payload)]...)
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(payload[0:]))
	m.IntegrationTimeUs = uint32(binary.LittleEndian.Uint32(payload[8:]))
	m.IntegratedX = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.IntegratedY = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	m.IntegratedXgyro = math.Float32frombits(binary.LittleEndian.Uint32(payload[20:]))
	m.IntegratedYgyro = math.Float32frombits(binary.LittleEndian.Uint32(payload[24:]))
	m.IntegratedZgyro = math.Float32frombits(binary.LittleEndian.Uint32(payload[28:]))
	m.TimeDeltaDistanceUs = uint32(binary.LittleEndian.Uint32(payload[32:]))
	m.Distance = math.Float32frombits(binary.LittleEndian.Uint32(payload[36:]))
	m.Temperature = int16(binary.LittleEndian.Uint16(payload[40:]))
	m.SensorID = uint8(payload[42])
	m.Quality = uint8(payload[43])
	return nil
}

// HilStateQuaternion struct (generated typeinfo)
// Sent from simulation to autopilot, avoids in contrast to HIL_STATE singularities. This packet is useful for high throughput applications such as hardware in the loop simulations.
type HilStateQuaternion struct {
	TimeUsec           uint64     // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
	AttitudeQuaternion [4]float32 // Vehicle attitude expressed as normalized quaternion in w, x, y, z order (with 1 0 0 0 being the null-rotation)
	Rollspeed          float32    // Body frame roll / phi angular speed
	Pitchspeed         float32    // Body frame pitch / theta angular speed
	Yawspeed           float32    // Body frame yaw / psi angular speed
	Lat                int32      // Latitude
	Lon                int32      // Longitude
	Alt                int32      // Altitude
	Vx                 int16      // Ground X Speed (Latitude)
	Vy                 int16      // Ground Y Speed (Longitude)
	Vz                 int16      // Ground Z Speed (Altitude)
	IndAirspeed        uint16     // Indicated airspeed
	TrueAirspeed       uint16     // True airspeed
	Xacc               int16      // X acceleration
	Yacc               int16      // Y acceleration
	Zacc               int16      // Z acceleration
}

// MsgID (generated function)
func (m *HilStateQuaternion) MsgID() mavlink.MessageID {
	return MSG_ID_HIL_STATE_QUATERNION
}

// String (generated function)
func (m *HilStateQuaternion) String() string {
	return fmt.Sprintf(
		"&common.HilStateQuaternion{ TimeUsec: %+v, AttitudeQuaternion: %+v, Rollspeed: %+v, Pitchspeed: %+v, Yawspeed: %+v, Lat: %+v, Lon: %+v, Alt: %+v, Vx: %+v, Vy: %+v, Vz: %+v, IndAirspeed: %+v, TrueAirspeed: %+v, Xacc: %+v, Yacc: %+v, Zacc: %+v }",
		m.TimeUsec,
		m.AttitudeQuaternion,
		m.Rollspeed,
		m.Pitchspeed,
		m.Yawspeed,
		m.Lat,
		m.Lon,
		m.Alt,
		m.Vx,
		m.Vy,
		m.Vz,
		m.IndAirspeed,
		m.TrueAirspeed,
		m.Xacc,
		m.Yacc,
		m.Zacc,
	)
}

// Pack (generated function)
func (m *HilStateQuaternion) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 64)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.TimeUsec))
	for i, v := range m.AttitudeQuaternion {
		binary.LittleEndian.PutUint32(payload[8+i*4:], math.Float32bits(v))
	}
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(m.Rollspeed))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(m.Pitchspeed))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(m.Yawspeed))
	binary.LittleEndian.PutUint32(payload[36:], uint32(m.Lat))
	binary.LittleEndian.PutUint32(payload[40:], uint32(m.Lon))
	binary.LittleEndian.PutUint32(payload[44:], uint32(m.Alt))
	binary.LittleEndian.PutUint16(payload[48:], uint16(m.Vx))
	binary.LittleEndian.PutUint16(payload[50:], uint16(m.Vy))
	binary.LittleEndian.PutUint16(payload[52:], uint16(m.Vz))
	binary.LittleEndian.PutUint16(payload[54:], uint16(m.IndAirspeed))
	binary.LittleEndian.PutUint16(payload[56:], uint16(m.TrueAirspeed))
	binary.LittleEndian.PutUint16(payload[58:], uint16(m.Xacc))
	binary.LittleEndian.PutUint16(payload[60:], uint16(m.Yacc))
	binary.LittleEndian.PutUint16(payload[62:], uint16(m.Zacc))
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *HilStateQuaternion) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 64 {
		payload = append(payload, mavlink.ZeroTail[:64-len(p.Payload)]...)
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(payload[0:]))
	for i := 0; i < len(m.AttitudeQuaternion); i++ {
		m.AttitudeQuaternion[i] = math.Float32frombits(binary.LittleEndian.Uint32(payload[8+i*4:]))
	}
	m.Rollspeed = math.Float32frombits(binary.LittleEndian.Uint32(payload[24:]))
	m.Pitchspeed = math.Float32frombits(binary.LittleEndian.Uint32(payload[28:]))
	m.Yawspeed = math.Float32frombits(binary.LittleEndian.Uint32(payload[32:]))
	m.Lat = int32(binary.LittleEndian.Uint32(payload[36:]))
	m.Lon = int32(binary.LittleEndian.Uint32(payload[40:]))
	m.Alt = int32(binary.LittleEndian.Uint32(payload[44:]))
	m.Vx = int16(binary.LittleEndian.Uint16(payload[48:]))
	m.Vy = int16(binary.LittleEndian.Uint16(payload[50:]))
	m.Vz = int16(binary.LittleEndian.Uint16(payload[52:]))
	m.IndAirspeed = uint16(binary.LittleEndian.Uint16(payload[54:]))
	m.TrueAirspeed = uint16(binary.LittleEndian.Uint16(payload[56:]))
	m.Xacc = int16(binary.LittleEndian.Uint16(payload[58:]))
	m.Yacc = int16(binary.LittleEndian.Uint16(payload[60:]))
	m.Zacc = int16(binary.LittleEndian.Uint16(payload[62:]))
	return nil
}

// ScaledImu2 struct (generated typeinfo)
// The RAW IMU readings for secondary 9DOF sensor setup. This message should contain the scaled values to the described units
type ScaledImu2 struct {
	TimeBootMs uint32 // Timestamp (time since system boot).
	Xacc       int16  // X acceleration
	Yacc       int16  // Y acceleration
	Zacc       int16  // Z acceleration
	Xgyro      int16  // Angular speed around X axis
	Ygyro      int16  // Angular speed around Y axis
	Zgyro      int16  // Angular speed around Z axis
	Xmag       int16  // X Magnetic field
	Ymag       int16  // Y Magnetic field
	Zmag       int16  // Z Magnetic field
}

// MsgID (generated function)
func (m *ScaledImu2) MsgID() mavlink.MessageID {
	return MSG_ID_SCALED_IMU2
}

// String (generated function)
func (m *ScaledImu2) String() string {
	return fmt.Sprintf(
		"&common.ScaledImu2{ TimeBootMs: %+v, Xacc: %+v, Yacc: %+v, Zacc: %+v, Xgyro: %+v, Ygyro: %+v, Zgyro: %+v, Xmag: %+v, Ymag: %+v, Zmag: %+v }",
		m.TimeBootMs,
		m.Xacc,
		m.Yacc,
		m.Zacc,
		m.Xgyro,
		m.Ygyro,
		m.Zgyro,
		m.Xmag,
		m.Ymag,
		m.Zmag,
	)
}

// Pack (generated function)
func (m *ScaledImu2) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 22)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.TimeBootMs))
	binary.LittleEndian.PutUint16(payload[4:], uint16(m.Xacc))
	binary.LittleEndian.PutUint16(payload[6:], uint16(m.Yacc))
	binary.LittleEndian.PutUint16(payload[8:], uint16(m.Zacc))
	binary.LittleEndian.PutUint16(payload[10:], uint16(m.Xgyro))
	binary.LittleEndian.PutUint16(payload[12:], uint16(m.Ygyro))
	binary.LittleEndian.PutUint16(payload[14:], uint16(m.Zgyro))
	binary.LittleEndian.PutUint16(payload[16:], uint16(m.Xmag))
	binary.LittleEndian.PutUint16(payload[18:], uint16(m.Ymag))
	binary.LittleEndian.PutUint16(payload[20:], uint16(m.Zmag))
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ScaledImu2) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 22 {
		payload = append(payload, mavlink.ZeroTail[:22-len(p.Payload)]...)
	}
	m.TimeBootMs = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.Xacc = int16(binary.LittleEndian.Uint16(payload[4:]))
	m.Yacc = int16(binary.LittleEndian.Uint16(payload[6:]))
	m.Zacc = int16(binary.LittleEndian.Uint16(payload[8:]))
	m.Xgyro = int16(binary.LittleEndian.Uint16(payload[10:]))
	m.Ygyro = int16(binary.LittleEndian.Uint16(payload[12:]))
	m.Zgyro = int16(binary.LittleEndian.Uint16(payload[14:]))
	m.Xmag = int16(binary.LittleEndian.Uint16(payload[16:]))
	m.Ymag = int16(binary.LittleEndian.Uint16(payload[18:]))
	m.Zmag = int16(binary.LittleEndian.Uint16(payload[20:]))
	return nil
}

// LogRequestList struct (generated typeinfo)
// Request a list of available logs. On some systems calling this may stop on-board logging until LOG_REQUEST_END is called. If there are no log files available this request shall be answered with one LOG_ENTRY message with id = 0 and num_logs = 0.
type LogRequestList struct {
	Start           uint16 // First log id (0 for first available)
	End             uint16 // Last log id (0xffff for last available)
	TargetSystem    uint8  // System ID
	TargetComponent uint8  // Component ID
}

// MsgID (generated function)
func (m *LogRequestList) MsgID() mavlink.MessageID {
	return MSG_ID_LOG_REQUEST_LIST
}

// String (generated function)
func (m *LogRequestList) String() string {
	return fmt.Sprintf(
		"&common.LogRequestList{ Start: %+v, End: %+v, TargetSystem: %+v, TargetComponent: %+v }",
		m.Start,
		m.End,
		m.TargetSystem,
		m.TargetComponent,
	)
}

// Pack (generated function)
func (m *LogRequestList) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 6)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.Start))
	binary.LittleEndian.PutUint16(payload[2:], uint16(m.End))
	payload[4] = byte(m.TargetSystem)
	payload[5] = byte(m.TargetComponent)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *LogRequestList) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 6 {
		payload = append(payload, mavlink.ZeroTail[:6-len(p.Payload)]...)
	}
	m.Start = uint16(binary.LittleEndian.Uint16(payload[0:]))
	m.End = uint16(binary.LittleEndian.Uint16(payload[2:]))
	m.TargetSystem = uint8(payload[4])
	m.TargetComponent = uint8(payload[5])
	return nil
}

// LogEntry struct (generated typeinfo)
// Reply to LOG_REQUEST_LIST
type LogEntry struct {
	TimeUtc    uint32 // UTC timestamp of log since 1970, or 0 if not available
	Size       uint32 // Size of the log (may be approximate)
	ID         uint16 // Log id
	NumLogs    uint16 // Total number of logs
	LastLogNum uint16 // High log number
}

// MsgID (generated function)
func (m *LogEntry) MsgID() mavlink.MessageID {
	return MSG_ID_LOG_ENTRY
}

// String (generated function)
func (m *LogEntry) String() string {
	return fmt.Sprintf(
		"&common.LogEntry{ TimeUtc: %+v, Size: %+v, ID: %+v, NumLogs: %+v, LastLogNum: %+v }",
		m.TimeUtc,
		m.Size,
		m.ID,
		m.NumLogs,
		m.LastLogNum,
	)
}

// Pack (generated function)
func (m *LogEntry) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 14)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.TimeUtc))
	binary.LittleEndian.PutUint32(payload[4:], uint32(m.Size))
	binary.LittleEndian.PutUint16(payload[8:], uint16(m.ID))
	binary.LittleEndian.PutUint16(payload[10:], uint16(m.NumLogs))
	binary.LittleEndian.PutUint16(payload[12:], uint16(m.LastLogNum))
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *LogEntry) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 14 {
		payload = append(payload, mavlink.ZeroTail[:14-len(p.Payload)]...)
	}
	m.TimeUtc = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.Size = uint32(binary.LittleEndian.Uint32(payload[4:]))
	m.ID = uint16(binary.LittleEndian.Uint16(payload[8:]))
	m.NumLogs = uint16(binary.LittleEndian.Uint16(payload[10:]))
	m.LastLogNum = uint16(binary.LittleEndian.Uint16(payload[12:]))
	return nil
}

// LogRequestData struct (generated typeinfo)
// Request a chunk of a log
type LogRequestData struct {
	Ofs             uint32 // Offset into the log
	Count           uint32 // Number of bytes
	ID              uint16 // Log id (from LOG_ENTRY reply)
	TargetSystem    uint8  // System ID
	TargetComponent uint8  // Component ID
}

// MsgID (generated function)
func (m *LogRequestData) MsgID() mavlink.MessageID {
	return MSG_ID_LOG_REQUEST_DATA
}

// String (generated function)
func (m *LogRequestData) String() string {
	return fmt.Sprintf(
		"&common.LogRequestData{ Ofs: %+v, Count: %+v, ID: %+v, TargetSystem: %+v, TargetComponent: %+v }",
		m.Ofs,
		m.Count,
		m.ID,
		m.TargetSystem,
		m.TargetComponent,
	)
}

// Pack (generated function)
func (m *LogRequestData) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 12)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.Ofs))
	binary.LittleEndian.PutUint32(payload[4:], uint32(m.Count))
	binary.LittleEndian.PutUint16(payload[8:], uint16(m.ID))
	payload[10] = byte(m.TargetSystem)
	payload[11] = byte(m.TargetComponent)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *LogRequestData) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 12 {
		payload = append(payload, mavlink.ZeroTail[:12-len(p.Payload)]...)
	}
	m.Ofs = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.Count = uint32(binary.LittleEndian.Uint32(payload[4:]))
	m.ID = uint16(binary.LittleEndian.Uint16(payload[8:]))
	m.TargetSystem = uint8(payload[10])
	m.TargetComponent = uint8(payload[11])
	return nil
}

// LogData struct (generated typeinfo)
// Reply to LOG_REQUEST_DATA
type LogData struct {
	Ofs   uint32    // Offset into the log
	ID    uint16    // Log id (from LOG_ENTRY reply)
	Count uint8     // Number of bytes (zero for end of log)
	Data  [90]uint8 // log data
}

// MsgID (generated function)
func (m *LogData) MsgID() mavlink.MessageID {
	return MSG_ID_LOG_DATA
}

// String (generated function)
func (m *LogData) String() string {
	return fmt.Sprintf(
		"&common.LogData{ Ofs: %+v, ID: %+v, Count: %+v, Data: %0X (\"%s\") }",
		m.Ofs,
		m.ID,
		m.Count,
		m.Data, string(m.Data[:]),
	)
}

// Pack (generated function)
func (m *LogData) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 97)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.Ofs))
	binary.LittleEndian.PutUint16(payload[4:], uint16(m.ID))
	payload[6] = byte(m.Count)
	copy(payload[7:], m.Data[:])
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *LogData) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 97 {
		payload = append(payload, mavlink.ZeroTail[:97-len(p.Payload)]...)
	}
	m.Ofs = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.ID = uint16(binary.LittleEndian.Uint16(payload[4:]))
	m.Count = uint8(payload[6])
	copy(m.Data[:], payload[7:97])
	return nil
}

// LogErase struct (generated typeinfo)
// Erase all logs
type LogErase struct {
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
}

// MsgID (generated function)
func (m *LogErase) MsgID() mavlink.MessageID {
	return MSG_ID_LOG_ERASE
}

// String (generated function)
func (m *LogErase) String() string {
	return fmt.Sprintf(
		"&common.LogErase{ TargetSystem: %+v, TargetComponent: %+v }",
		m.TargetSystem,
		m.TargetComponent,
	)
}

// Pack (generated function)
func (m *LogErase) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 2)
	payload[0] = byte(m.TargetSystem)
	payload[1] = byte(m.TargetComponent)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *LogErase) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 2 {
		payload = append(payload, mavlink.ZeroTail[:2-len(p.Payload)]...)
	}
	m.TargetSystem = uint8(payload[0])
	m.TargetComponent = uint8(payload[1])
	return nil
}

// LogRequestEnd struct (generated typeinfo)
// Stop log transfer and resume normal logging
type LogRequestEnd struct {
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
}

// MsgID (generated function)
func (m *LogRequestEnd) MsgID() mavlink.MessageID {
	return MSG_ID_LOG_REQUEST_END
}

// String (generated function)
func (m *LogRequestEnd) String() string {
	return fmt.Sprintf(
		"&common.LogRequestEnd{ TargetSystem: %+v, TargetComponent: %+v }",
		m.TargetSystem,
		m.TargetComponent,
	)
}

// Pack (generated function)
func (m *LogRequestEnd) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 2)
	payload[0] = byte(m.TargetSystem)
	payload[1] = byte(m.TargetComponent)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *LogRequestEnd) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 2 {
		payload = append(payload, mavlink.ZeroTail[:2-len(p.Payload)]...)
	}
	m.TargetSystem = uint8(payload[0])
	m.TargetComponent = uint8(payload[1])
	return nil
}

// GpsInjectData struct (generated typeinfo)
// Data for injecting into the onboard GPS (used for DGPS)
type GpsInjectData struct {
	TargetSystem    uint8      // System ID
	TargetComponent uint8      // Component ID
	Len             uint8      // Data length
	Data            [110]uint8 // Raw data (110 is enough for 12 satellites of RTCMv2)
}

// MsgID (generated function)
func (m *GpsInjectData) MsgID() mavlink.MessageID {
	return MSG_ID_GPS_INJECT_DATA
}

// String (generated function)
func (m *GpsInjectData) String() string {
	return fmt.Sprintf(
		"&common.GpsInjectData{ TargetSystem: %+v, TargetComponent: %+v, Len: %+v, Data: %0X (\"%s\") }",
		m.TargetSystem,
		m.TargetComponent,
		m.Len,
		m.Data, string(m.Data[:]),
	)
}

// Pack (generated function)
func (m *GpsInjectData) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 113)
	payload[0] = byte(m.TargetSystem)
	payload[1] = byte(m.TargetComponent)
	payload[2] = byte(m.Len)
	copy(payload[3:], m.Data[:])
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *GpsInjectData) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 113 {
		payload = append(payload, mavlink.ZeroTail[:113-len(p.Payload)]...)
	}
	m.TargetSystem = uint8(payload[0])
	m.TargetComponent = uint8(payload[1])
	m.Len = uint8(payload[2])
	copy(m.Data[:], payload[3:113])
	return nil
}

// Gps2Raw struct (generated typeinfo)
// Second GPS data.
type Gps2Raw struct {
	TimeUsec          uint64       // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
	Lat               int32        // Latitude (WGS84)
	Lon               int32        // Longitude (WGS84)
	Alt               int32        // Altitude (MSL). Positive for up.
	DgpsAge           uint32       // Age of DGPS info
	Eph               uint16       // GPS HDOP horizontal dilution of position. If unknown, set to: UINT16_MAX
	Epv               uint16       // GPS VDOP vertical dilution of position. If unknown, set to: UINT16_MAX
	Vel               uint16       // GPS ground speed. If unknown, set to: UINT16_MAX
	Cog               uint16       // Course over ground (NOT heading, but direction of movement): 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
	FixType           GPS_FIX_TYPE `gotype:"uint8"` // GPS fix type.
	SatellitesVisible uint8        // Number of satellites visible. If unknown, set to 255
	DgpsNumch         uint8        // Number of DGPS satellites
}

// MsgID (generated function)
func (m *Gps2Raw) MsgID() mavlink.MessageID {
	return MSG_ID_GPS2_RAW
}

// String (generated function)
func (m *Gps2Raw) String() string {
	return fmt.Sprintf(
		"&common.Gps2Raw{ TimeUsec: %+v, Lat: %+v, Lon: %+v, Alt: %+v, DgpsAge: %+v, Eph: %+v, Epv: %+v, Vel: %+v, Cog: %+v, FixType: %+v, SatellitesVisible: %+v, DgpsNumch: %+v }",
		m.TimeUsec,
		m.Lat,
		m.Lon,
		m.Alt,
		m.DgpsAge,
		m.Eph,
		m.Epv,
		m.Vel,
		m.Cog,
		m.FixType,
		m.SatellitesVisible,
		m.DgpsNumch,
	)
}

// Pack (generated function)
func (m *Gps2Raw) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 35)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.TimeUsec))
	binary.LittleEndian.PutUint32(payload[8:], uint32(m.Lat))
	binary.LittleEndian.PutUint32(payload[12:], uint32(m.Lon))
	binary.LittleEndian.PutUint32(payload[16:], uint32(m.Alt))
	binary.LittleEndian.PutUint32(payload[20:], uint32(m.DgpsAge))
	binary.LittleEndian.PutUint16(payload[24:], uint16(m.Eph))
	binary.LittleEndian.PutUint16(payload[26:], uint16(m.Epv))
	binary.LittleEndian.PutUint16(payload[28:], uint16(m.Vel))
	binary.LittleEndian.PutUint16(payload[30:], uint16(m.Cog))
	payload[32] = byte(m.FixType)
	payload[33] = byte(m.SatellitesVisible)
	payload[34] = byte(m.DgpsNumch)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *Gps2Raw) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 35 {
		payload = append(payload, mavlink.ZeroTail[:35-len(p.Payload)]...)
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(payload[0:]))
	m.Lat = int32(binary.LittleEndian.Uint32(payload[8:]))
	m.Lon = int32(binary.LittleEndian.Uint32(payload[12:]))
	m.Alt = int32(binary.LittleEndian.Uint32(payload[16:]))
	m.DgpsAge = uint32(binary.LittleEndian.Uint32(payload[20:]))
	m.Eph = uint16(binary.LittleEndian.Uint16(payload[24:]))
	m.Epv = uint16(binary.LittleEndian.Uint16(payload[26:]))
	m.Vel = uint16(binary.LittleEndian.Uint16(payload[28:]))
	m.Cog = uint16(binary.LittleEndian.Uint16(payload[30:]))
	m.FixType = GPS_FIX_TYPE(payload[32])
	m.SatellitesVisible = uint8(payload[33])
	m.DgpsNumch = uint8(payload[34])
	return nil
}

// PowerStatus struct (generated typeinfo)
// Power supply status
type PowerStatus struct {
	Vcc    uint16           // 5V rail voltage.
	Vservo uint16           // Servo rail voltage.
	Flags  MAV_POWER_STATUS `gotype:"uint16"` // Bitmap of power supply status flags.
}

// MsgID (generated function)
func (m *PowerStatus) MsgID() mavlink.MessageID {
	return MSG_ID_POWER_STATUS
}

// String (generated function)
func (m *PowerStatus) String() string {
	return fmt.Sprintf(
		"&common.PowerStatus{ Vcc: %+v, Vservo: %+v, Flags: %+v }",
		m.Vcc,
		m.Vservo,
		m.Flags,
	)
}

// Pack (generated function)
func (m *PowerStatus) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 6)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.Vcc))
	binary.LittleEndian.PutUint16(payload[2:], uint16(m.Vservo))
	binary.LittleEndian.PutUint16(payload[4:], uint16(m.Flags))
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *PowerStatus) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 6 {
		payload = append(payload, mavlink.ZeroTail[:6-len(p.Payload)]...)
	}
	m.Vcc = uint16(binary.LittleEndian.Uint16(payload[0:]))
	m.Vservo = uint16(binary.LittleEndian.Uint16(payload[2:]))
	m.Flags = MAV_POWER_STATUS(binary.LittleEndian.Uint16(payload[4:]))
	return nil
}

// SerialControl struct (generated typeinfo)
// Control a serial port. This can be used for raw access to an onboard serial peripheral such as a GPS or telemetry radio. It is designed to make it possible to update the devices firmware via MAVLink messages or change the devices settings. A message with zero bytes can be used to change just the baudrate.
type SerialControl struct {
	Baudrate uint32              // Baudrate of transfer. Zero means no change.
	Timeout  uint16              // Timeout for reply data
	Device   SERIAL_CONTROL_DEV  `gotype:"uint8"` // Serial control device type.
	Flags    SERIAL_CONTROL_FLAG `gotype:"uint8"` // Bitmap of serial control flags.
	Count    uint8               // how many bytes in this transfer
	Data     [70]uint8           // serial data
}

// MsgID (generated function)
func (m *SerialControl) MsgID() mavlink.MessageID {
	return MSG_ID_SERIAL_CONTROL
}

// String (generated function)
func (m *SerialControl) String() string {
	return fmt.Sprintf(
		"&common.SerialControl{ Baudrate: %+v, Timeout: %+v, Device: %+v, Flags: %+v, Count: %+v, Data: %0X (\"%s\") }",
		m.Baudrate,
		m.Timeout,
		m.Device,
		m.Flags,
		m.Count,
		m.Data, string(m.Data[:]),
	)
}

// Pack (generated function)
func (m *SerialControl) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 79)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.Baudrate))
	binary.LittleEndian.PutUint16(payload[4:], uint16(m.Timeout))
	payload[6] = byte(m.Device)
	payload[7] = byte(m.Flags)
	payload[8] = byte(m.Count)
	copy(payload[9:], m.Data[:])
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *SerialControl) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 79 {
		payload = append(payload, mavlink.ZeroTail[:79-len(p.Payload)]...)
	}
	m.Baudrate = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.Timeout = uint16(binary.LittleEndian.Uint16(payload[4:]))
	m.Device = SERIAL_CONTROL_DEV(payload[6])
	m.Flags = SERIAL_CONTROL_FLAG(payload[7])
	m.Count = uint8(payload[8])
	copy(m.Data[:], payload[9:79])
	return nil
}

// GpsRtk struct (generated typeinfo)
// RTK GPS data. Gives information on the relative baseline calculation the GPS is reporting
type GpsRtk struct {
	TimeLastBaselineMs uint32                         // Time since boot of last baseline message received.
	Tow                uint32                         // GPS Time of Week of last baseline
	BaselineAMm        int32                          // Current baseline in ECEF x or NED north component.
	BaselineBMm        int32                          // Current baseline in ECEF y or NED east component.
	BaselineCMm        int32                          // Current baseline in ECEF z or NED down component.
	Accuracy           uint32                         // Current estimate of baseline accuracy.
	IarNumHypotheses   int32                          // Current number of integer ambiguity hypotheses.
	Wn                 uint16                         // GPS Week Number of last baseline
	RtkReceiverID      uint8                          // Identification of connected RTK receiver.
	RtkHealth          uint8                          // GPS-specific health report for RTK data.
	RtkRate            uint8                          // Rate of baseline messages being received by GPS
	Nsats              uint8                          // Current number of sats used for RTK calculation.
	BaselineCoordsType RTK_BASELINE_COORDINATE_SYSTEM `gotype:"uint8"` // Coordinate system of baseline
}

// MsgID (generated function)
func (m *GpsRtk) MsgID() mavlink.MessageID {
	return MSG_ID_GPS_RTK
}

// String (generated function)
func (m *GpsRtk) String() string {
	return fmt.Sprintf(
		"&common.GpsRtk{ TimeLastBaselineMs: %+v, Tow: %+v, BaselineAMm: %+v, BaselineBMm: %+v, BaselineCMm: %+v, Accuracy: %+v, IarNumHypotheses: %+v, Wn: %+v, RtkReceiverID: %+v, RtkHealth: %+v, RtkRate: %+v, Nsats: %+v, BaselineCoordsType: %+v }",
		m.TimeLastBaselineMs,
		m.Tow,
		m.BaselineAMm,
		m.BaselineBMm,
		m.BaselineCMm,
		m.Accuracy,
		m.IarNumHypotheses,
		m.Wn,
		m.RtkReceiverID,
		m.RtkHealth,
		m.RtkRate,
		m.Nsats,
		m.BaselineCoordsType,
	)
}

// Pack (generated function)
func (m *GpsRtk) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 35)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.TimeLastBaselineMs))
	binary.LittleEndian.PutUint32(payload[4:], uint32(m.Tow))
	binary.LittleEndian.PutUint32(payload[8:], uint32(m.BaselineAMm))
	binary.LittleEndian.PutUint32(payload[12:], uint32(m.BaselineBMm))
	binary.LittleEndian.PutUint32(payload[16:], uint32(m.BaselineCMm))
	binary.LittleEndian.PutUint32(payload[20:], uint32(m.Accuracy))
	binary.LittleEndian.PutUint32(payload[24:], uint32(m.IarNumHypotheses))
	binary.LittleEndian.PutUint16(payload[28:], uint16(m.Wn))
	payload[30] = byte(m.RtkReceiverID)
	payload[31] = byte(m.RtkHealth)
	payload[32] = byte(m.RtkRate)
	payload[33] = byte(m.Nsats)
	payload[34] = byte(m.BaselineCoordsType)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *GpsRtk) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 35 {
		payload = append(payload, mavlink.ZeroTail[:35-len(p.Payload)]...)
	}
	m.TimeLastBaselineMs = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.Tow = uint32(binary.LittleEndian.Uint32(payload[4:]))
	m.BaselineAMm = int32(binary.LittleEndian.Uint32(payload[8:]))
	m.BaselineBMm = int32(binary.LittleEndian.Uint32(payload[12:]))
	m.BaselineCMm = int32(binary.LittleEndian.Uint32(payload[16:]))
	m.Accuracy = uint32(binary.LittleEndian.Uint32(payload[20:]))
	m.IarNumHypotheses = int32(binary.LittleEndian.Uint32(payload[24:]))
	m.Wn = uint16(binary.LittleEndian.Uint16(payload[28:]))
	m.RtkReceiverID = uint8(payload[30])
	m.RtkHealth = uint8(payload[31])
	m.RtkRate = uint8(payload[32])
	m.Nsats = uint8(payload[33])
	m.BaselineCoordsType = RTK_BASELINE_COORDINATE_SYSTEM(payload[34])
	return nil
}

// Gps2Rtk struct (generated typeinfo)
// RTK GPS data. Gives information on the relative baseline calculation the GPS is reporting
type Gps2Rtk struct {
	TimeLastBaselineMs uint32                         // Time since boot of last baseline message received.
	Tow                uint32                         // GPS Time of Week of last baseline
	BaselineAMm        int32                          // Current baseline in ECEF x or NED north component.
	BaselineBMm        int32                          // Current baseline in ECEF y or NED east component.
	BaselineCMm        int32                          // Current baseline in ECEF z or NED down component.
	Accuracy           uint32                         // Current estimate of baseline accuracy.
	IarNumHypotheses   int32                          // Current number of integer ambiguity hypotheses.
	Wn                 uint16                         // GPS Week Number of last baseline
	RtkReceiverID      uint8                          // Identification of connected RTK receiver.
	RtkHealth          uint8                          // GPS-specific health report for RTK data.
	RtkRate            uint8                          // Rate of baseline messages being received by GPS
	Nsats              uint8                          // Current number of sats used for RTK calculation.
	BaselineCoordsType RTK_BASELINE_COORDINATE_SYSTEM `gotype:"uint8"` // Coordinate system of baseline
}

// MsgID (generated function)
func (m *Gps2Rtk) MsgID() mavlink.MessageID {
	return MSG_ID_GPS2_RTK
}

// String (generated function)
func (m *Gps2Rtk) String() string {
	return fmt.Sprintf(
		"&common.Gps2Rtk{ TimeLastBaselineMs: %+v, Tow: %+v, BaselineAMm: %+v, BaselineBMm: %+v, BaselineCMm: %+v, Accuracy: %+v, IarNumHypotheses: %+v, Wn: %+v, RtkReceiverID: %+v, RtkHealth: %+v, RtkRate: %+v, Nsats: %+v, BaselineCoordsType: %+v }",
		m.TimeLastBaselineMs,
		m.Tow,
		m.BaselineAMm,
		m.BaselineBMm,
		m.BaselineCMm,
		m.Accuracy,
		m.IarNumHypotheses,
		m.Wn,
		m.RtkReceiverID,
		m.RtkHealth,
		m.RtkRate,
		m.Nsats,
		m.BaselineCoordsType,
	)
}

// Pack (generated function)
func (m *Gps2Rtk) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 35)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.TimeLastBaselineMs))
	binary.LittleEndian.PutUint32(payload[4:], uint32(m.Tow))
	binary.LittleEndian.PutUint32(payload[8:], uint32(m.BaselineAMm))
	binary.LittleEndian.PutUint32(payload[12:], uint32(m.BaselineBMm))
	binary.LittleEndian.PutUint32(payload[16:], uint32(m.BaselineCMm))
	binary.LittleEndian.PutUint32(payload[20:], uint32(m.Accuracy))
	binary.LittleEndian.PutUint32(payload[24:], uint32(m.IarNumHypotheses))
	binary.LittleEndian.PutUint16(payload[28:], uint16(m.Wn))
	payload[30] = byte(m.RtkReceiverID)
	payload[31] = byte(m.RtkHealth)
	payload[32] = byte(m.RtkRate)
	payload[33] = byte(m.Nsats)
	payload[34] = byte(m.BaselineCoordsType)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *Gps2Rtk) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 35 {
		payload = append(payload, mavlink.ZeroTail[:35-len(p.Payload)]...)
	}
	m.TimeLastBaselineMs = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.Tow = uint32(binary.LittleEndian.Uint32(payload[4:]))
	m.BaselineAMm = int32(binary.LittleEndian.Uint32(payload[8:]))
	m.BaselineBMm = int32(binary.LittleEndian.Uint32(payload[12:]))
	m.BaselineCMm = int32(binary.LittleEndian.Uint32(payload[16:]))
	m.Accuracy = uint32(binary.LittleEndian.Uint32(payload[20:]))
	m.IarNumHypotheses = int32(binary.LittleEndian.Uint32(payload[24:]))
	m.Wn = uint16(binary.LittleEndian.Uint16(payload[28:]))
	m.RtkReceiverID = uint8(payload[30])
	m.RtkHealth = uint8(payload[31])
	m.RtkRate = uint8(payload[32])
	m.Nsats = uint8(payload[33])
	m.BaselineCoordsType = RTK_BASELINE_COORDINATE_SYSTEM(payload[34])
	return nil
}

// ScaledImu3 struct (generated typeinfo)
// The RAW IMU readings for 3rd 9DOF sensor setup. This message should contain the scaled values to the described units
type ScaledImu3 struct {
	TimeBootMs uint32 // Timestamp (time since system boot).
	Xacc       int16  // X acceleration
	Yacc       int16  // Y acceleration
	Zacc       int16  // Z acceleration
	Xgyro      int16  // Angular speed around X axis
	Ygyro      int16  // Angular speed around Y axis
	Zgyro      int16  // Angular speed around Z axis
	Xmag       int16  // X Magnetic field
	Ymag       int16  // Y Magnetic field
	Zmag       int16  // Z Magnetic field
}

// MsgID (generated function)
func (m *ScaledImu3) MsgID() mavlink.MessageID {
	return MSG_ID_SCALED_IMU3
}

// String (generated function)
func (m *ScaledImu3) String() string {
	return fmt.Sprintf(
		"&common.ScaledImu3{ TimeBootMs: %+v, Xacc: %+v, Yacc: %+v, Zacc: %+v, Xgyro: %+v, Ygyro: %+v, Zgyro: %+v, Xmag: %+v, Ymag: %+v, Zmag: %+v }",
		m.TimeBootMs,
		m.Xacc,
		m.Yacc,
		m.Zacc,
		m.Xgyro,
		m.Ygyro,
		m.Zgyro,
		m.Xmag,
		m.Ymag,
		m.Zmag,
	)
}

// Pack (generated function)
func (m *ScaledImu3) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 22)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.TimeBootMs))
	binary.LittleEndian.PutUint16(payload[4:], uint16(m.Xacc))
	binary.LittleEndian.PutUint16(payload[6:], uint16(m.Yacc))
	binary.LittleEndian.PutUint16(payload[8:], uint16(m.Zacc))
	binary.LittleEndian.PutUint16(payload[10:], uint16(m.Xgyro))
	binary.LittleEndian.PutUint16(payload[12:], uint16(m.Ygyro))
	binary.LittleEndian.PutUint16(payload[14:], uint16(m.Zgyro))
	binary.LittleEndian.PutUint16(payload[16:], uint16(m.Xmag))
	binary.LittleEndian.PutUint16(payload[18:], uint16(m.Ymag))
	binary.LittleEndian.PutUint16(payload[20:], uint16(m.Zmag))
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ScaledImu3) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 22 {
		payload = append(payload, mavlink.ZeroTail[:22-len(p.Payload)]...)
	}
	m.TimeBootMs = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.Xacc = int16(binary.LittleEndian.Uint16(payload[4:]))
	m.Yacc = int16(binary.LittleEndian.Uint16(payload[6:]))
	m.Zacc = int16(binary.LittleEndian.Uint16(payload[8:]))
	m.Xgyro = int16(binary.LittleEndian.Uint16(payload[10:]))
	m.Ygyro = int16(binary.LittleEndian.Uint16(payload[12:]))
	m.Zgyro = int16(binary.LittleEndian.Uint16(payload[14:]))
	m.Xmag = int16(binary.LittleEndian.Uint16(payload[16:]))
	m.Ymag = int16(binary.LittleEndian.Uint16(payload[18:]))
	m.Zmag = int16(binary.LittleEndian.Uint16(payload[20:]))
	return nil
}

// DataTransmissionHandshake struct (generated typeinfo)
// Handshake message to initiate, control and stop image streaming when using the Image Transmission Protocol: https://mavlink.io/en/services/image_transmission.html.
type DataTransmissionHandshake struct {
	Size       uint32                   // total data size (set on ACK only).
	Width      uint16                   // Width of a matrix or image.
	Height     uint16                   // Height of a matrix or image.
	Packets    uint16                   // Number of packets being sent (set on ACK only).
	Type       MAVLINK_DATA_STREAM_TYPE `gotype:"uint8"` // Type of requested/acknowledged data.
	Payload    uint8                    // Payload size per packet (normally 253 byte, see DATA field size in message ENCAPSULATED_DATA) (set on ACK only).
	JpgQuality uint8                    // JPEG quality. Values: [1-100].
}

// MsgID (generated function)
func (m *DataTransmissionHandshake) MsgID() mavlink.MessageID {
	return MSG_ID_DATA_TRANSMISSION_HANDSHAKE
}

// String (generated function)
func (m *DataTransmissionHandshake) String() string {
	return fmt.Sprintf(
		"&common.DataTransmissionHandshake{ Size: %+v, Width: %+v, Height: %+v, Packets: %+v, Type: %+v, Payload: %+v, JpgQuality: %+v }",
		m.Size,
		m.Width,
		m.Height,
		m.Packets,
		m.Type,
		m.Payload,
		m.JpgQuality,
	)
}

// Pack (generated function)
func (m *DataTransmissionHandshake) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 13)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.Size))
	binary.LittleEndian.PutUint16(payload[4:], uint16(m.Width))
	binary.LittleEndian.PutUint16(payload[6:], uint16(m.Height))
	binary.LittleEndian.PutUint16(payload[8:], uint16(m.Packets))
	payload[10] = byte(m.Type)
	payload[11] = byte(m.Payload)
	payload[12] = byte(m.JpgQuality)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *DataTransmissionHandshake) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 13 {
		payload = append(payload, mavlink.ZeroTail[:13-len(p.Payload)]...)
	}
	m.Size = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.Width = uint16(binary.LittleEndian.Uint16(payload[4:]))
	m.Height = uint16(binary.LittleEndian.Uint16(payload[6:]))
	m.Packets = uint16(binary.LittleEndian.Uint16(payload[8:]))
	m.Type = MAVLINK_DATA_STREAM_TYPE(payload[10])
	m.Payload = uint8(payload[11])
	m.JpgQuality = uint8(payload[12])
	return nil
}

// EncapsulatedData struct (generated typeinfo)
// Data packet for images sent using the Image Transmission Protocol: https://mavlink.io/en/services/image_transmission.html.
type EncapsulatedData struct {
	Seqnr uint16     // sequence number (starting with 0 on every transmission)
	Data  [253]uint8 // image data bytes
}

// MsgID (generated function)
func (m *EncapsulatedData) MsgID() mavlink.MessageID {
	return MSG_ID_ENCAPSULATED_DATA
}

// String (generated function)
func (m *EncapsulatedData) String() string {
	return fmt.Sprintf(
		"&common.EncapsulatedData{ Seqnr: %+v, Data: %0X (\"%s\") }",
		m.Seqnr,
		m.Data, string(m.Data[:]),
	)
}

// Pack (generated function)
func (m *EncapsulatedData) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 255)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.Seqnr))
	copy(payload[2:], m.Data[:])
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *EncapsulatedData) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 255 {
		payload = append(payload, mavlink.ZeroTail[:255-len(p.Payload)]...)
	}
	m.Seqnr = uint16(binary.LittleEndian.Uint16(payload[0:]))
	copy(m.Data[:], payload[2:255])
	return nil
}

// DistanceSensor struct (generated typeinfo)
// Distance sensor information for an onboard rangefinder.
type DistanceSensor struct {
	TimeBootMs      uint32                 // Timestamp (time since system boot).
	MinDistance     uint16                 // Minimum distance the sensor can measure
	MaxDistance     uint16                 // Maximum distance the sensor can measure
	CurrentDistance uint16                 // Current distance reading
	Type            MAV_DISTANCE_SENSOR    `gotype:"uint8"` // Type of distance sensor.
	ID              uint8                  // Onboard ID of the sensor
	Orientation     MAV_SENSOR_ORIENTATION `gotype:"uint8"` // Direction the sensor faces. downward-facing: ROTATION_PITCH_270, upward-facing: ROTATION_PITCH_90, backward-facing: ROTATION_PITCH_180, forward-facing: ROTATION_NONE, left-facing: ROTATION_YAW_90, right-facing: ROTATION_YAW_270
	Covariance      uint8                  // Measurement variance. Max standard deviation is 6cm. 255 if unknown.
}

// MsgID (generated function)
func (m *DistanceSensor) MsgID() mavlink.MessageID {
	return MSG_ID_DISTANCE_SENSOR
}

// String (generated function)
func (m *DistanceSensor) String() string {
	return fmt.Sprintf(
		"&common.DistanceSensor{ TimeBootMs: %+v, MinDistance: %+v, MaxDistance: %+v, CurrentDistance: %+v, Type: %+v, ID: %+v, Orientation: %+v, Covariance: %+v }",
		m.TimeBootMs,
		m.MinDistance,
		m.MaxDistance,
		m.CurrentDistance,
		m.Type,
		m.ID,
		m.Orientation,
		m.Covariance,
	)
}

// Pack (generated function)
func (m *DistanceSensor) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 14)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.TimeBootMs))
	binary.LittleEndian.PutUint16(payload[4:], uint16(m.MinDistance))
	binary.LittleEndian.PutUint16(payload[6:], uint16(m.MaxDistance))
	binary.LittleEndian.PutUint16(payload[8:], uint16(m.CurrentDistance))
	payload[10] = byte(m.Type)
	payload[11] = byte(m.ID)
	payload[12] = byte(m.Orientation)
	payload[13] = byte(m.Covariance)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *DistanceSensor) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 14 {
		payload = append(payload, mavlink.ZeroTail[:14-len(p.Payload)]...)
	}
	m.TimeBootMs = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.MinDistance = uint16(binary.LittleEndian.Uint16(payload[4:]))
	m.MaxDistance = uint16(binary.LittleEndian.Uint16(payload[6:]))
	m.CurrentDistance = uint16(binary.LittleEndian.Uint16(payload[8:]))
	m.Type = MAV_DISTANCE_SENSOR(payload[10])
	m.ID = uint8(payload[11])
	m.Orientation = MAV_SENSOR_ORIENTATION(payload[12])
	m.Covariance = uint8(payload[13])
	return nil
}

// TerrainRequest struct (generated typeinfo)
// Request for terrain data and terrain status. See terrain protocol docs: https://mavlink.io/en/services/terrain.html
type TerrainRequest struct {
	Mask        uint64 // Bitmask of requested 4x4 grids (row major 8x7 array of grids, 56 bits)
	Lat         int32  // Latitude of SW corner of first grid
	Lon         int32  // Longitude of SW corner of first grid
	GridSpacing uint16 // Grid spacing
}

// MsgID (generated function)
func (m *TerrainRequest) MsgID() mavlink.MessageID {
	return MSG_ID_TERRAIN_REQUEST
}

// String (generated function)
func (m *TerrainRequest) String() string {
	return fmt.Sprintf(
		"&common.TerrainRequest{ Mask: %+v, Lat: %+v, Lon: %+v, GridSpacing: %+v }",
		m.Mask,
		m.Lat,
		m.Lon,
		m.GridSpacing,
	)
}

// Pack (generated function)
func (m *TerrainRequest) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 18)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.Mask))
	binary.LittleEndian.PutUint32(payload[8:], uint32(m.Lat))
	binary.LittleEndian.PutUint32(payload[12:], uint32(m.Lon))
	binary.LittleEndian.PutUint16(payload[16:], uint16(m.GridSpacing))
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *TerrainRequest) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 18 {
		payload = append(payload, mavlink.ZeroTail[:18-len(p.Payload)]...)
	}
	m.Mask = uint64(binary.LittleEndian.Uint64(payload[0:]))
	m.Lat = int32(binary.LittleEndian.Uint32(payload[8:]))
	m.Lon = int32(binary.LittleEndian.Uint32(payload[12:]))
	m.GridSpacing = uint16(binary.LittleEndian.Uint16(payload[16:]))
	return nil
}

// TerrainData struct (generated typeinfo)
// Terrain data sent from GCS. The lat/lon and grid_spacing must be the same as a lat/lon from a TERRAIN_REQUEST. See terrain protocol docs: https://mavlink.io/en/services/terrain.html
type TerrainData struct {
	Lat         int32     // Latitude of SW corner of first grid
	Lon         int32     // Longitude of SW corner of first grid
	GridSpacing uint16    // Grid spacing
	Data        [16]int16 // Terrain data MSL
	Gridbit     uint8     // bit within the terrain request mask
}

// MsgID (generated function)
func (m *TerrainData) MsgID() mavlink.MessageID {
	return MSG_ID_TERRAIN_DATA
}

// String (generated function)
func (m *TerrainData) String() string {
	return fmt.Sprintf(
		"&common.TerrainData{ Lat: %+v, Lon: %+v, GridSpacing: %+v, Data: %+v, Gridbit: %+v }",
		m.Lat,
		m.Lon,
		m.GridSpacing,
		m.Data,
		m.Gridbit,
	)
}

// Pack (generated function)
func (m *TerrainData) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 43)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.Lat))
	binary.LittleEndian.PutUint32(payload[4:], uint32(m.Lon))
	binary.LittleEndian.PutUint16(payload[8:], uint16(m.GridSpacing))
	for i, v := range m.Data {
		binary.LittleEndian.PutUint16(payload[10+i*2:], uint16(v))
	}
	payload[42] = byte(m.Gridbit)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *TerrainData) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 43 {
		payload = append(payload, mavlink.ZeroTail[:43-len(p.Payload)]...)
	}
	m.Lat = int32(binary.LittleEndian.Uint32(payload[0:]))
	m.Lon = int32(binary.LittleEndian.Uint32(payload[4:]))
	m.GridSpacing = uint16(binary.LittleEndian.Uint16(payload[8:]))
	for i := 0; i < len(m.Data); i++ {
		m.Data[i] = int16(binary.LittleEndian.Uint16(payload[10+i*2:]))
	}
	m.Gridbit = uint8(payload[42])
	return nil
}

// TerrainCheck struct (generated typeinfo)
// Request that the vehicle report terrain height at the given location (expected response is a TERRAIN_REPORT). Used by GCS to check if vehicle has all terrain data needed for a mission.
type TerrainCheck struct {
	Lat int32 // Latitude
	Lon int32 // Longitude
}

// MsgID (generated function)
func (m *TerrainCheck) MsgID() mavlink.MessageID {
	return MSG_ID_TERRAIN_CHECK
}

// String (generated function)
func (m *TerrainCheck) String() string {
	return fmt.Sprintf(
		"&common.TerrainCheck{ Lat: %+v, Lon: %+v }",
		m.Lat,
		m.Lon,
	)
}

// Pack (generated function)
func (m *TerrainCheck) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 8)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.Lat))
	binary.LittleEndian.PutUint32(payload[4:], uint32(m.Lon))
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *TerrainCheck) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 8 {
		payload = append(payload, mavlink.ZeroTail[:8-len(p.Payload)]...)
	}
	m.Lat = int32(binary.LittleEndian.Uint32(payload[0:]))
	m.Lon = int32(binary.LittleEndian.Uint32(payload[4:]))
	return nil
}

// TerrainReport struct (generated typeinfo)
// Streamed from drone to report progress of terrain map download (initiated by TERRAIN_REQUEST), or sent as a response to a TERRAIN_CHECK request. See terrain protocol docs: https://mavlink.io/en/services/terrain.html
type TerrainReport struct {
	Lat           int32   // Latitude
	Lon           int32   // Longitude
	TerrainHeight float32 // Terrain height MSL
	CurrentHeight float32 // Current vehicle height above lat/lon terrain height
	Spacing       uint16  // grid spacing (zero if terrain at this location unavailable)
	Pending       uint16  // Number of 4x4 terrain blocks waiting to be received or read from disk
	Loaded        uint16  // Number of 4x4 terrain blocks in memory
}

// MsgID (generated function)
func (m *TerrainReport) MsgID() mavlink.MessageID {
	return MSG_ID_TERRAIN_REPORT
}

// String (generated function)
func (m *TerrainReport) String() string {
	return fmt.Sprintf(
		"&common.TerrainReport{ Lat: %+v, Lon: %+v, TerrainHeight: %+v, CurrentHeight: %+v, Spacing: %+v, Pending: %+v, Loaded: %+v }",
		m.Lat,
		m.Lon,
		m.TerrainHeight,
		m.CurrentHeight,
		m.Spacing,
		m.Pending,
		m.Loaded,
	)
}

// Pack (generated function)
func (m *TerrainReport) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 22)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.Lat))
	binary.LittleEndian.PutUint32(payload[4:], uint32(m.Lon))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.TerrainHeight))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.CurrentHeight))
	binary.LittleEndian.PutUint16(payload[16:], uint16(m.Spacing))
	binary.LittleEndian.PutUint16(payload[18:], uint16(m.Pending))
	binary.LittleEndian.PutUint16(payload[20:], uint16(m.Loaded))
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *TerrainReport) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 22 {
		payload = append(payload, mavlink.ZeroTail[:22-len(p.Payload)]...)
	}
	m.Lat = int32(binary.LittleEndian.Uint32(payload[0:]))
	m.Lon = int32(binary.LittleEndian.Uint32(payload[4:]))
	m.TerrainHeight = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.CurrentHeight = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.Spacing = uint16(binary.LittleEndian.Uint16(payload[16:]))
	m.Pending = uint16(binary.LittleEndian.Uint16(payload[18:]))
	m.Loaded = uint16(binary.LittleEndian.Uint16(payload[20:]))
	return nil
}

// ScaledPressure2 struct (generated typeinfo)
// Barometer readings for 2nd barometer
type ScaledPressure2 struct {
	TimeBootMs  uint32  // Timestamp (time since system boot).
	PressAbs    float32 // Absolute pressure
	PressDiff   float32 // Differential pressure
	Temperature int16   // Absolute pressure temperature
}

// MsgID (generated function)
func (m *ScaledPressure2) MsgID() mavlink.MessageID {
	return MSG_ID_SCALED_PRESSURE2
}

// String (generated function)
func (m *ScaledPressure2) String() string {
	return fmt.Sprintf(
		"&common.ScaledPressure2{ TimeBootMs: %+v, PressAbs: %+v, PressDiff: %+v, Temperature: %+v }",
		m.TimeBootMs,
		m.PressAbs,
		m.PressDiff,
		m.Temperature,
	)
}

// Pack (generated function)
func (m *ScaledPressure2) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 14)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.TimeBootMs))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(m.PressAbs))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.PressDiff))
	binary.LittleEndian.PutUint16(payload[12:], uint16(m.Temperature))
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ScaledPressure2) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 14 {
		payload = append(payload, mavlink.ZeroTail[:14-len(p.Payload)]...)
	}
	m.TimeBootMs = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.PressAbs = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	m.PressDiff = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.Temperature = int16(binary.LittleEndian.Uint16(payload[12:]))
	return nil
}

// AttPosMocap struct (generated typeinfo)
// Motion capture attitude and position
type AttPosMocap struct {
	TimeUsec uint64     // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
	Q        [4]float32 // Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
	X        float32    // X position (NED)
	Y        float32    // Y position (NED)
	Z        float32    // Z position (NED)
}

// MsgID (generated function)
func (m *AttPosMocap) MsgID() mavlink.MessageID {
	return MSG_ID_ATT_POS_MOCAP
}

// String (generated function)
func (m *AttPosMocap) String() string {
	return fmt.Sprintf(
		"&common.AttPosMocap{ TimeUsec: %+v, Q: %+v, X: %+v, Y: %+v, Z: %+v }",
		m.TimeUsec,
		m.Q,
		m.X,
		m.Y,
		m.Z,
	)
}

// Pack (generated function)
func (m *AttPosMocap) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 36)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.TimeUsec))
	for i, v := range m.Q {
		binary.LittleEndian.PutUint32(payload[8+i*4:], math.Float32bits(v))
	}
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(m.X))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(m.Y))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(m.Z))
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *AttPosMocap) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 36 {
		payload = append(payload, mavlink.ZeroTail[:36-len(p.Payload)]...)
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(payload[0:]))
	for i := 0; i < len(m.Q); i++ {
		m.Q[i] = math.Float32frombits(binary.LittleEndian.Uint32(payload[8+i*4:]))
	}
	m.X = math.Float32frombits(binary.LittleEndian.Uint32(payload[24:]))
	m.Y = math.Float32frombits(binary.LittleEndian.Uint32(payload[28:]))
	m.Z = math.Float32frombits(binary.LittleEndian.Uint32(payload[32:]))
	return nil
}

// SetActuatorControlTarget struct (generated typeinfo)
// Set the vehicle attitude and body angular rates.
type SetActuatorControlTarget struct {
	TimeUsec        uint64     // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
	Controls        [8]float32 // Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0): (index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through mixer to repurpose them as generic outputs.
	GroupMlx        uint8      // Actuator group. The "_mlx" indicates this is a multi-instance message and a MAVLink parser should use this field to difference between instances.
	TargetSystem    uint8      // System ID
	TargetComponent uint8      // Component ID
}

// MsgID (generated function)
func (m *SetActuatorControlTarget) MsgID() mavlink.MessageID {
	return MSG_ID_SET_ACTUATOR_CONTROL_TARGET
}

// String (generated function)
func (m *SetActuatorControlTarget) String() string {
	return fmt.Sprintf(
		"&common.SetActuatorControlTarget{ TimeUsec: %+v, Controls: %+v, GroupMlx: %+v, TargetSystem: %+v, TargetComponent: %+v }",
		m.TimeUsec,
		m.Controls,
		m.GroupMlx,
		m.TargetSystem,
		m.TargetComponent,
	)
}

// Pack (generated function)
func (m *SetActuatorControlTarget) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 43)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.TimeUsec))
	for i, v := range m.Controls {
		binary.LittleEndian.PutUint32(payload[8+i*4:], math.Float32bits(v))
	}
	payload[40] = byte(m.GroupMlx)
	payload[41] = byte(m.TargetSystem)
	payload[42] = byte(m.TargetComponent)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *SetActuatorControlTarget) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 43 {
		payload = append(payload, mavlink.ZeroTail[:43-len(p.Payload)]...)
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(payload[0:]))
	for i := 0; i < len(m.Controls); i++ {
		m.Controls[i] = math.Float32frombits(binary.LittleEndian.Uint32(payload[8+i*4:]))
	}
	m.GroupMlx = uint8(payload[40])
	m.TargetSystem = uint8(payload[41])
	m.TargetComponent = uint8(payload[42])
	return nil
}

// ActuatorControlTarget struct (generated typeinfo)
// Set the vehicle attitude and body angular rates.
type ActuatorControlTarget struct {
	TimeUsec uint64     // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
	Controls [8]float32 // Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0): (index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through mixer to repurpose them as generic outputs.
	GroupMlx uint8      // Actuator group. The "_mlx" indicates this is a multi-instance message and a MAVLink parser should use this field to difference between instances.
}

// MsgID (generated function)
func (m *ActuatorControlTarget) MsgID() mavlink.MessageID {
	return MSG_ID_ACTUATOR_CONTROL_TARGET
}

// String (generated function)
func (m *ActuatorControlTarget) String() string {
	return fmt.Sprintf(
		"&common.ActuatorControlTarget{ TimeUsec: %+v, Controls: %+v, GroupMlx: %+v }",
		m.TimeUsec,
		m.Controls,
		m.GroupMlx,
	)
}

// Pack (generated function)
func (m *ActuatorControlTarget) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 41)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.TimeUsec))
	for i, v := range m.Controls {
		binary.LittleEndian.PutUint32(payload[8+i*4:], math.Float32bits(v))
	}
	payload[40] = byte(m.GroupMlx)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ActuatorControlTarget) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 41 {
		payload = append(payload, mavlink.ZeroTail[:41-len(p.Payload)]...)
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(payload[0:]))
	for i := 0; i < len(m.Controls); i++ {
		m.Controls[i] = math.Float32frombits(binary.LittleEndian.Uint32(payload[8+i*4:]))
	}
	m.GroupMlx = uint8(payload[40])
	return nil
}

// Altitude struct (generated typeinfo)
// The current system altitude.
type Altitude struct {
	TimeUsec          uint64  // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
	AltitudeMonotonic float32 // This altitude measure is initialized on system boot and monotonic (it is never reset, but represents the local altitude change). The only guarantee on this field is that it will never be reset and is consistent within a flight. The recommended value for this field is the uncorrected barometric altitude at boot time. This altitude will also drift and vary between flights.
	AltitudeAmsl      float32 // This altitude measure is strictly above mean sea level and might be non-monotonic (it might reset on events like GPS lock or when a new QNH value is set). It should be the altitude to which global altitude waypoints are compared to. Note that it is *not* the GPS altitude, however, most GPS modules already output MSL by default and not the WGS84 altitude.
	AltitudeLocal     float32 // This is the local altitude in the local coordinate frame. It is not the altitude above home, but in reference to the coordinate origin (0, 0, 0). It is up-positive.
	AltitudeRelative  float32 // This is the altitude above the home position. It resets on each change of the current home position.
	AltitudeTerrain   float32 // This is the altitude above terrain. It might be fed by a terrain database or an altimeter. Values smaller than -1000 should be interpreted as unknown.
	BottomClearance   float32 // This is not the altitude, but the clear space below the system according to the fused clearance estimate. It generally should max out at the maximum range of e.g. the laser altimeter. It is generally a moving target. A negative value indicates no measurement available.
}

// MsgID (generated function)
func (m *Altitude) MsgID() mavlink.MessageID {
	return MSG_ID_ALTITUDE
}

// String (generated function)
func (m *Altitude) String() string {
	return fmt.Sprintf(
		"&common.Altitude{ TimeUsec: %+v, AltitudeMonotonic: %+v, AltitudeAmsl: %+v, AltitudeLocal: %+v, AltitudeRelative: %+v, AltitudeTerrain: %+v, BottomClearance: %+v }",
		m.TimeUsec,
		m.AltitudeMonotonic,
		m.AltitudeAmsl,
		m.AltitudeLocal,
		m.AltitudeRelative,
		m.AltitudeTerrain,
		m.BottomClearance,
	)
}

// Pack (generated function)
func (m *Altitude) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 32)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.TimeUsec))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.AltitudeMonotonic))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.AltitudeAmsl))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(m.AltitudeLocal))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(m.AltitudeRelative))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(m.AltitudeTerrain))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(m.BottomClearance))
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *Altitude) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 32 {
		payload = append(payload, mavlink.ZeroTail[:32-len(p.Payload)]...)
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(payload[0:]))
	m.AltitudeMonotonic = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.AltitudeAmsl = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.AltitudeLocal = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	m.AltitudeRelative = math.Float32frombits(binary.LittleEndian.Uint32(payload[20:]))
	m.AltitudeTerrain = math.Float32frombits(binary.LittleEndian.Uint32(payload[24:]))
	m.BottomClearance = math.Float32frombits(binary.LittleEndian.Uint32(payload[28:]))
	return nil
}

// ResourceRequest struct (generated typeinfo)
// The autopilot is requesting a resource (file, binary, other type of data)
type ResourceRequest struct {
	RequestID    uint8      // Request ID. This ID should be re-used when sending back URI contents
	URIType      uint8      // The type of requested URI. 0 = a file via URL. 1 = a UAVCAN binary
	URI          [120]uint8 // The requested unique resource identifier (URI). It is not necessarily a straight domain name (depends on the URI type enum)
	TransferType uint8      // The way the autopilot wants to receive the URI. 0 = MAVLink FTP. 1 = binary stream.
	Storage      [120]uint8 // The storage path the autopilot wants the URI to be stored in. Will only be valid if the transfer_type has a storage associated (e.g. MAVLink FTP).
}

// MsgID (generated function)
func (m *ResourceRequest) MsgID() mavlink.MessageID {
	return MSG_ID_RESOURCE_REQUEST
}

// String (generated function)
func (m *ResourceRequest) String() string {
	return fmt.Sprintf(
		"&common.ResourceRequest{ RequestID: %+v, URIType: %+v, URI: %0X (\"%s\"), TransferType: %+v, Storage: %0X (\"%s\") }",
		m.RequestID,
		m.URIType,
		m.URI, string(m.URI[:]),
		m.TransferType,
		m.Storage, string(m.Storage[:]),
	)
}

// Pack (generated function)
func (m *ResourceRequest) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 243)
	payload[0] = byte(m.RequestID)
	payload[1] = byte(m.URIType)
	copy(payload[2:], m.URI[:])
	payload[122] = byte(m.TransferType)
	copy(payload[123:], m.Storage[:])
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ResourceRequest) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 243 {
		payload = append(payload, mavlink.ZeroTail[:243-len(p.Payload)]...)
	}
	m.RequestID = uint8(payload[0])
	m.URIType = uint8(payload[1])
	copy(m.URI[:], payload[2:122])
	m.TransferType = uint8(payload[122])
	copy(m.Storage[:], payload[123:243])
	return nil
}

// ScaledPressure3 struct (generated typeinfo)
// Barometer readings for 3rd barometer
type ScaledPressure3 struct {
	TimeBootMs  uint32  // Timestamp (time since system boot).
	PressAbs    float32 // Absolute pressure
	PressDiff   float32 // Differential pressure
	Temperature int16   // Absolute pressure temperature
}

// MsgID (generated function)
func (m *ScaledPressure3) MsgID() mavlink.MessageID {
	return MSG_ID_SCALED_PRESSURE3
}

// String (generated function)
func (m *ScaledPressure3) String() string {
	return fmt.Sprintf(
		"&common.ScaledPressure3{ TimeBootMs: %+v, PressAbs: %+v, PressDiff: %+v, Temperature: %+v }",
		m.TimeBootMs,
		m.PressAbs,
		m.PressDiff,
		m.Temperature,
	)
}

// Pack (generated function)
func (m *ScaledPressure3) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 14)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.TimeBootMs))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(m.PressAbs))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.PressDiff))
	binary.LittleEndian.PutUint16(payload[12:], uint16(m.Temperature))
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ScaledPressure3) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 14 {
		payload = append(payload, mavlink.ZeroTail[:14-len(p.Payload)]...)
	}
	m.TimeBootMs = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.PressAbs = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	m.PressDiff = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.Temperature = int16(binary.LittleEndian.Uint16(payload[12:]))
	return nil
}

// FollowTarget struct (generated typeinfo)
// Current motion information from a designated system
type FollowTarget struct {
	Timestamp       uint64     // Timestamp (time since system boot).
	CustomState     uint64     // button states or switches of a tracker device
	Lat             int32      // Latitude (WGS84)
	Lon             int32      // Longitude (WGS84)
	Alt             float32    // Altitude (MSL)
	Vel             [3]float32 // target velocity (0,0,0) for unknown
	Acc             [3]float32 // linear target acceleration (0,0,0) for unknown
	AttitudeQ       [4]float32 // (1 0 0 0 for unknown)
	Rates           [3]float32 // (0 0 0 for unknown)
	PositionCov     [3]float32 // eph epv
	EstCapabilities uint8      // bit positions for tracker reporting capabilities (POS = 0, VEL = 1, ACCEL = 2, ATT + RATES = 3)
}

// MsgID (generated function)
func (m *FollowTarget) MsgID() mavlink.MessageID {
	return MSG_ID_FOLLOW_TARGET
}

// String (generated function)
func (m *FollowTarget) String() string {
	return fmt.Sprintf(
		"&common.FollowTarget{ Timestamp: %+v, CustomState: %+v, Lat: %+v, Lon: %+v, Alt: %+v, Vel: %+v, Acc: %+v, AttitudeQ: %+v, Rates: %+v, PositionCov: %+v, EstCapabilities: %+v }",
		m.Timestamp,
		m.CustomState,
		m.Lat,
		m.Lon,
		m.Alt,
		m.Vel,
		m.Acc,
		m.AttitudeQ,
		m.Rates,
		m.PositionCov,
		m.EstCapabilities,
	)
}

// Pack (generated function)
func (m *FollowTarget) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 93)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.Timestamp))
	binary.LittleEndian.PutUint64(payload[8:], uint64(m.CustomState))
	binary.LittleEndian.PutUint32(payload[16:], uint32(m.Lat))
	binary.LittleEndian.PutUint32(payload[20:], uint32(m.Lon))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(m.Alt))
	for i, v := range m.Vel {
		binary.LittleEndian.PutUint32(payload[28+i*4:], math.Float32bits(v))
	}
	for i, v := range m.Acc {
		binary.LittleEndian.PutUint32(payload[40+i*4:], math.Float32bits(v))
	}
	for i, v := range m.AttitudeQ {
		binary.LittleEndian.PutUint32(payload[52+i*4:], math.Float32bits(v))
	}
	for i, v := range m.Rates {
		binary.LittleEndian.PutUint32(payload[68+i*4:], math.Float32bits(v))
	}
	for i, v := range m.PositionCov {
		binary.LittleEndian.PutUint32(payload[80+i*4:], math.Float32bits(v))
	}
	payload[92] = byte(m.EstCapabilities)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *FollowTarget) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 93 {
		payload = append(payload, mavlink.ZeroTail[:93-len(p.Payload)]...)
	}
	m.Timestamp = uint64(binary.LittleEndian.Uint64(payload[0:]))
	m.CustomState = uint64(binary.LittleEndian.Uint64(payload[8:]))
	m.Lat = int32(binary.LittleEndian.Uint32(payload[16:]))
	m.Lon = int32(binary.LittleEndian.Uint32(payload[20:]))
	m.Alt = math.Float32frombits(binary.LittleEndian.Uint32(payload[24:]))
	for i := 0; i < len(m.Vel); i++ {
		m.Vel[i] = math.Float32frombits(binary.LittleEndian.Uint32(payload[28+i*4:]))
	}
	for i := 0; i < len(m.Acc); i++ {
		m.Acc[i] = math.Float32frombits(binary.LittleEndian.Uint32(payload[40+i*4:]))
	}
	for i := 0; i < len(m.AttitudeQ); i++ {
		m.AttitudeQ[i] = math.Float32frombits(binary.LittleEndian.Uint32(payload[52+i*4:]))
	}
	for i := 0; i < len(m.Rates); i++ {
		m.Rates[i] = math.Float32frombits(binary.LittleEndian.Uint32(payload[68+i*4:]))
	}
	for i := 0; i < len(m.PositionCov); i++ {
		m.PositionCov[i] = math.Float32frombits(binary.LittleEndian.Uint32(payload[80+i*4:]))
	}
	m.EstCapabilities = uint8(payload[92])
	return nil
}

// ControlSystemState struct (generated typeinfo)
// The smoothed, monotonic system state used to feed the control loops of the system.
type ControlSystemState struct {
	TimeUsec    uint64     // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
	XAcc        float32    // X acceleration in body frame
	YAcc        float32    // Y acceleration in body frame
	ZAcc        float32    // Z acceleration in body frame
	XVel        float32    // X velocity in body frame
	YVel        float32    // Y velocity in body frame
	ZVel        float32    // Z velocity in body frame
	XPos        float32    // X position in local frame
	YPos        float32    // Y position in local frame
	ZPos        float32    // Z position in local frame
	Airspeed    float32    // Airspeed, set to -1 if unknown
	VelVariance [3]float32 // Variance of body velocity estimate
	PosVariance [3]float32 // Variance in local position
	Q           [4]float32 // The attitude, represented as Quaternion
	RollRate    float32    // Angular rate in roll axis
	PitchRate   float32    // Angular rate in pitch axis
	YawRate     float32    // Angular rate in yaw axis
}

// MsgID (generated function)
func (m *ControlSystemState) MsgID() mavlink.MessageID {
	return MSG_ID_CONTROL_SYSTEM_STATE
}

// String (generated function)
func (m *ControlSystemState) String() string {
	return fmt.Sprintf(
		"&common.ControlSystemState{ TimeUsec: %+v, XAcc: %+v, YAcc: %+v, ZAcc: %+v, XVel: %+v, YVel: %+v, ZVel: %+v, XPos: %+v, YPos: %+v, ZPos: %+v, Airspeed: %+v, VelVariance: %+v, PosVariance: %+v, Q: %+v, RollRate: %+v, PitchRate: %+v, YawRate: %+v }",
		m.TimeUsec,
		m.XAcc,
		m.YAcc,
		m.ZAcc,
		m.XVel,
		m.YVel,
		m.ZVel,
		m.XPos,
		m.YPos,
		m.ZPos,
		m.Airspeed,
		m.VelVariance,
		m.PosVariance,
		m.Q,
		m.RollRate,
		m.PitchRate,
		m.YawRate,
	)
}

// Pack (generated function)
func (m *ControlSystemState) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 100)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.TimeUsec))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.XAcc))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.YAcc))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(m.ZAcc))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(m.XVel))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(m.YVel))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(m.ZVel))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(m.XPos))
	binary.LittleEndian.PutUint32(payload[36:], math.Float32bits(m.YPos))
	binary.LittleEndian.PutUint32(payload[40:], math.Float32bits(m.ZPos))
	binary.LittleEndian.PutUint32(payload[44:], math.Float32bits(m.Airspeed))
	for i, v := range m.VelVariance {
		binary.LittleEndian.PutUint32(payload[48+i*4:], math.Float32bits(v))
	}
	for i, v := range m.PosVariance {
		binary.LittleEndian.PutUint32(payload[60+i*4:], math.Float32bits(v))
	}
	for i, v := range m.Q {
		binary.LittleEndian.PutUint32(payload[72+i*4:], math.Float32bits(v))
	}
	binary.LittleEndian.PutUint32(payload[88:], math.Float32bits(m.RollRate))
	binary.LittleEndian.PutUint32(payload[92:], math.Float32bits(m.PitchRate))
	binary.LittleEndian.PutUint32(payload[96:], math.Float32bits(m.YawRate))
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ControlSystemState) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 100 {
		payload = append(payload, mavlink.ZeroTail[:100-len(p.Payload)]...)
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(payload[0:]))
	m.XAcc = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.YAcc = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.ZAcc = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	m.XVel = math.Float32frombits(binary.LittleEndian.Uint32(payload[20:]))
	m.YVel = math.Float32frombits(binary.LittleEndian.Uint32(payload[24:]))
	m.ZVel = math.Float32frombits(binary.LittleEndian.Uint32(payload[28:]))
	m.XPos = math.Float32frombits(binary.LittleEndian.Uint32(payload[32:]))
	m.YPos = math.Float32frombits(binary.LittleEndian.Uint32(payload[36:]))
	m.ZPos = math.Float32frombits(binary.LittleEndian.Uint32(payload[40:]))
	m.Airspeed = math.Float32frombits(binary.LittleEndian.Uint32(payload[44:]))
	for i := 0; i < len(m.VelVariance); i++ {
		m.VelVariance[i] = math.Float32frombits(binary.LittleEndian.Uint32(payload[48+i*4:]))
	}
	for i := 0; i < len(m.PosVariance); i++ {
		m.PosVariance[i] = math.Float32frombits(binary.LittleEndian.Uint32(payload[60+i*4:]))
	}
	for i := 0; i < len(m.Q); i++ {
		m.Q[i] = math.Float32frombits(binary.LittleEndian.Uint32(payload[72+i*4:]))
	}
	m.RollRate = math.Float32frombits(binary.LittleEndian.Uint32(payload[88:]))
	m.PitchRate = math.Float32frombits(binary.LittleEndian.Uint32(payload[92:]))
	m.YawRate = math.Float32frombits(binary.LittleEndian.Uint32(payload[96:]))
	return nil
}

// BatteryStatus struct (generated typeinfo)
// Battery information. Updates GCS with flight controller battery status. Smart batteries also use this message, but may additionally send SMART_BATTERY_INFO.
type BatteryStatus struct {
	CurrentConsumed  int32                // Consumed charge, -1: autopilot does not provide consumption estimate
	EnergyConsumed   int32                // Consumed energy, -1: autopilot does not provide energy consumption estimate
	Temperature      int16                // Temperature of the battery. INT16_MAX for unknown temperature.
	Voltages         [10]uint16           // Battery voltage of cells 1 to 10 (see voltages_ext for cells 11-14). Cells in this field above the valid cell count for this battery should have the UINT16_MAX value. If individual cell voltages are unknown or not measured for this battery, then the overall battery voltage should be filled in cell 0, with all others set to UINT16_MAX. If the voltage of the battery is greater than (UINT16_MAX - 1), then cell 0 should be set to (UINT16_MAX - 1), and cell 1 to the remaining voltage. This can be extended to multiple cells if the total voltage is greater than 2 * (UINT16_MAX - 1).
	CurrentBattery   int16                // Battery current, -1: autopilot does not measure the current
	ID               uint8                // Battery ID
	BatteryFunction  MAV_BATTERY_FUNCTION `gotype:"uint8"` // Function of the battery
	Type             MAV_BATTERY_TYPE     `gotype:"uint8"` // Type (chemistry) of the battery
	BatteryRemaining int8                 // Remaining battery energy. Values: [0-100], -1: autopilot does not estimate the remaining battery.
}

// MsgID (generated function)
func (m *BatteryStatus) MsgID() mavlink.MessageID {
	return MSG_ID_BATTERY_STATUS
}

// String (generated function)
func (m *BatteryStatus) String() string {
	return fmt.Sprintf(
		"&common.BatteryStatus{ CurrentConsumed: %+v, EnergyConsumed: %+v, Temperature: %+v, Voltages: %+v, CurrentBattery: %+v, ID: %+v, BatteryFunction: %+v, Type: %+v, BatteryRemaining: %+v }",
		m.CurrentConsumed,
		m.EnergyConsumed,
		m.Temperature,
		m.Voltages,
		m.CurrentBattery,
		m.ID,
		m.BatteryFunction,
		m.Type,
		m.BatteryRemaining,
	)
}

// Pack (generated function)
func (m *BatteryStatus) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 36)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.CurrentConsumed))
	binary.LittleEndian.PutUint32(payload[4:], uint32(m.EnergyConsumed))
	binary.LittleEndian.PutUint16(payload[8:], uint16(m.Temperature))
	for i, v := range m.Voltages {
		binary.LittleEndian.PutUint16(payload[10+i*2:], uint16(v))
	}
	binary.LittleEndian.PutUint16(payload[30:], uint16(m.CurrentBattery))
	payload[32] = byte(m.ID)
	payload[33] = byte(m.BatteryFunction)
	payload[34] = byte(m.Type)
	payload[35] = byte(m.BatteryRemaining)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *BatteryStatus) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 36 {
		payload = append(payload, mavlink.ZeroTail[:36-len(p.Payload)]...)
	}
	m.CurrentConsumed = int32(binary.LittleEndian.Uint32(payload[0:]))
	m.EnergyConsumed = int32(binary.LittleEndian.Uint32(payload[4:]))
	m.Temperature = int16(binary.LittleEndian.Uint16(payload[8:]))
	for i := 0; i < len(m.Voltages); i++ {
		m.Voltages[i] = uint16(binary.LittleEndian.Uint16(payload[10+i*2:]))
	}
	m.CurrentBattery = int16(binary.LittleEndian.Uint16(payload[30:]))
	m.ID = uint8(payload[32])
	m.BatteryFunction = MAV_BATTERY_FUNCTION(payload[33])
	m.Type = MAV_BATTERY_TYPE(payload[34])
	m.BatteryRemaining = int8(payload[35])
	return nil
}

// AutopilotVersion struct (generated typeinfo)
// Version and capability of autopilot software. This should be emitted in response to a request with MAV_CMD_REQUEST_MESSAGE.
type AutopilotVersion struct {
	Capabilities            MAV_PROTOCOL_CAPABILITY `gotype:"uint64"` // Bitmap of capabilities
	UID                     uint64                  // UID if provided by hardware (see uid2)
	FlightSwVersion         uint32                  // Firmware version number
	MiddlewareSwVersion     uint32                  // Middleware version number
	OsSwVersion             uint32                  // Operating system version number
	BoardVersion            uint32                  // HW / board version (last 8 bytes should be silicon ID, if any)
	VendorID                uint16                  // ID of the board vendor
	ProductID               uint16                  // ID of the product
	FlightCustomVersion     [8]uint8                // Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but should allow to identify the commit using the main version number even for very large code bases.
	MiddlewareCustomVersion [8]uint8                // Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but should allow to identify the commit using the main version number even for very large code bases.
	OsCustomVersion         [8]uint8                // Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but should allow to identify the commit using the main version number even for very large code bases.
}

// MsgID (generated function)
func (m *AutopilotVersion) MsgID() mavlink.MessageID {
	return MSG_ID_AUTOPILOT_VERSION
}

// String (generated function)
func (m *AutopilotVersion) String() string {
	return fmt.Sprintf(
		"&common.AutopilotVersion{ Capabilities: %+v, UID: %+v, FlightSwVersion: %+v, MiddlewareSwVersion: %+v, OsSwVersion: %+v, BoardVersion: %+v, VendorID: %+v, ProductID: %+v, FlightCustomVersion: %0X (\"%s\"), MiddlewareCustomVersion: %0X (\"%s\"), OsCustomVersion: %0X (\"%s\") }",
		m.Capabilities,
		m.UID,
		m.FlightSwVersion,
		m.MiddlewareSwVersion,
		m.OsSwVersion,
		m.BoardVersion,
		m.VendorID,
		m.ProductID,
		m.FlightCustomVersion, string(m.FlightCustomVersion[:]),
		m.MiddlewareCustomVersion, string(m.MiddlewareCustomVersion[:]),
		m.OsCustomVersion, string(m.OsCustomVersion[:]),
	)
}

// Pack (generated function)
func (m *AutopilotVersion) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 60)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.Capabilities))
	binary.LittleEndian.PutUint64(payload[8:], uint64(m.UID))
	binary.LittleEndian.PutUint32(payload[16:], uint32(m.FlightSwVersion))
	binary.LittleEndian.PutUint32(payload[20:], uint32(m.MiddlewareSwVersion))
	binary.LittleEndian.PutUint32(payload[24:], uint32(m.OsSwVersion))
	binary.LittleEndian.PutUint32(payload[28:], uint32(m.BoardVersion))
	binary.LittleEndian.PutUint16(payload[32:], uint16(m.VendorID))
	binary.LittleEndian.PutUint16(payload[34:], uint16(m.ProductID))
	copy(payload[36:], m.FlightCustomVersion[:])
	copy(payload[44:], m.MiddlewareCustomVersion[:])
	copy(payload[52:], m.OsCustomVersion[:])
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *AutopilotVersion) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 60 {
		payload = append(payload, mavlink.ZeroTail[:60-len(p.Payload)]...)
	}
	m.Capabilities = MAV_PROTOCOL_CAPABILITY(binary.LittleEndian.Uint64(payload[0:]))
	m.UID = uint64(binary.LittleEndian.Uint64(payload[8:]))
	m.FlightSwVersion = uint32(binary.LittleEndian.Uint32(payload[16:]))
	m.MiddlewareSwVersion = uint32(binary.LittleEndian.Uint32(payload[20:]))
	m.OsSwVersion = uint32(binary.LittleEndian.Uint32(payload[24:]))
	m.BoardVersion = uint32(binary.LittleEndian.Uint32(payload[28:]))
	m.VendorID = uint16(binary.LittleEndian.Uint16(payload[32:]))
	m.ProductID = uint16(binary.LittleEndian.Uint16(payload[34:]))
	copy(m.FlightCustomVersion[:], payload[36:44])
	copy(m.MiddlewareCustomVersion[:], payload[44:52])
	copy(m.OsCustomVersion[:], payload[52:60])
	return nil
}

// LandingTarget struct (generated typeinfo)
// The location of a landing target. See: https://mavlink.io/en/services/landing_target.html
type LandingTarget struct {
	TimeUsec  uint64    // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
	AngleX    float32   // X-axis angular offset of the target from the center of the image
	AngleY    float32   // Y-axis angular offset of the target from the center of the image
	Distance  float32   // Distance to the target from the vehicle
	SizeX     float32   // Size of target along x-axis
	SizeY     float32   // Size of target along y-axis
	TargetNum uint8     // The ID of the target if multiple targets are present
	Frame     MAV_FRAME `gotype:"uint8"` // Coordinate frame used for following fields.
}

// MsgID (generated function)
func (m *LandingTarget) MsgID() mavlink.MessageID {
	return MSG_ID_LANDING_TARGET
}

// String (generated function)
func (m *LandingTarget) String() string {
	return fmt.Sprintf(
		"&common.LandingTarget{ TimeUsec: %+v, AngleX: %+v, AngleY: %+v, Distance: %+v, SizeX: %+v, SizeY: %+v, TargetNum: %+v, Frame: %+v }",
		m.TimeUsec,
		m.AngleX,
		m.AngleY,
		m.Distance,
		m.SizeX,
		m.SizeY,
		m.TargetNum,
		m.Frame,
	)
}

// Pack (generated function)
func (m *LandingTarget) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 30)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.TimeUsec))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.AngleX))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.AngleY))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(m.Distance))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(m.SizeX))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(m.SizeY))
	payload[28] = byte(m.TargetNum)
	payload[29] = byte(m.Frame)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *LandingTarget) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 30 {
		payload = append(payload, mavlink.ZeroTail[:30-len(p.Payload)]...)
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(payload[0:]))
	m.AngleX = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.AngleY = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.Distance = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	m.SizeX = math.Float32frombits(binary.LittleEndian.Uint32(payload[20:]))
	m.SizeY = math.Float32frombits(binary.LittleEndian.Uint32(payload[24:]))
	m.TargetNum = uint8(payload[28])
	m.Frame = MAV_FRAME(payload[29])
	return nil
}

// FenceStatus struct (generated typeinfo)
// Status of geo-fencing. Sent in extended status stream when fencing enabled.
type FenceStatus struct {
	BreachTime   uint32       // Time (since boot) of last breach.
	BreachCount  uint16       // Number of fence breaches.
	BreachStatus uint8        // Breach status (0 if currently inside fence, 1 if outside).
	BreachType   FENCE_BREACH `gotype:"uint8"` // Last breach type.
}

// MsgID (generated function)
func (m *FenceStatus) MsgID() mavlink.MessageID {
	return MSG_ID_FENCE_STATUS
}

// String (generated function)
func (m *FenceStatus) String() string {
	return fmt.Sprintf(
		"&common.FenceStatus{ BreachTime: %+v, BreachCount: %+v, BreachStatus: %+v, BreachType: %+v }",
		m.BreachTime,
		m.BreachCount,
		m.BreachStatus,
		m.BreachType,
	)
}

// Pack (generated function)
func (m *FenceStatus) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 8)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.BreachTime))
	binary.LittleEndian.PutUint16(payload[4:], uint16(m.BreachCount))
	payload[6] = byte(m.BreachStatus)
	payload[7] = byte(m.BreachType)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *FenceStatus) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 8 {
		payload = append(payload, mavlink.ZeroTail[:8-len(p.Payload)]...)
	}
	m.BreachTime = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.BreachCount = uint16(binary.LittleEndian.Uint16(payload[4:]))
	m.BreachStatus = uint8(payload[6])
	m.BreachType = FENCE_BREACH(payload[7])
	return nil
}

// MagCalReport struct (generated typeinfo)
// Reports results of completed compass calibration. Sent until MAG_CAL_ACK received.
type MagCalReport struct {
	Fitness   float32        // RMS milligauss residuals.
	OfsX      float32        // X offset.
	OfsY      float32        // Y offset.
	OfsZ      float32        // Z offset.
	DiagX     float32        // X diagonal (matrix 11).
	DiagY     float32        // Y diagonal (matrix 22).
	DiagZ     float32        // Z diagonal (matrix 33).
	OffdiagX  float32        // X off-diagonal (matrix 12 and 21).
	OffdiagY  float32        // Y off-diagonal (matrix 13 and 31).
	OffdiagZ  float32        // Z off-diagonal (matrix 32 and 23).
	CompassID uint8          // Compass being calibrated.
	CalMask   uint8          // Bitmask of compasses being calibrated.
	CalStatus MAG_CAL_STATUS `gotype:"uint8"` // Calibration Status.
	Autosaved uint8          // 0=requires a MAV_CMD_DO_ACCEPT_MAG_CAL, 1=saved to parameters.
}

// MsgID (generated function)
func (m *MagCalReport) MsgID() mavlink.MessageID {
	return MSG_ID_MAG_CAL_REPORT
}

// String (generated function)
func (m *MagCalReport) String() string {
	return fmt.Sprintf(
		"&common.MagCalReport{ Fitness: %+v, OfsX: %+v, OfsY: %+v, OfsZ: %+v, DiagX: %+v, DiagY: %+v, DiagZ: %+v, OffdiagX: %+v, OffdiagY: %+v, OffdiagZ: %+v, CompassID: %+v, CalMask: %+v, CalStatus: %+v, Autosaved: %+v }",
		m.Fitness,
		m.OfsX,
		m.OfsY,
		m.OfsZ,
		m.DiagX,
		m.DiagY,
		m.DiagZ,
		m.OffdiagX,
		m.OffdiagY,
		m.OffdiagZ,
		m.CompassID,
		m.CalMask,
		m.CalStatus,
		m.Autosaved,
	)
}

// Pack (generated function)
func (m *MagCalReport) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 44)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(m.Fitness))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(m.OfsX))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.OfsY))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.OfsZ))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(m.DiagX))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(m.DiagY))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(m.DiagZ))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(m.OffdiagX))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(m.OffdiagY))
	binary.LittleEndian.PutUint32(payload[36:], math.Float32bits(m.OffdiagZ))
	payload[40] = byte(m.CompassID)
	payload[41] = byte(m.CalMask)
	payload[42] = byte(m.CalStatus)
	payload[43] = byte(m.Autosaved)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *MagCalReport) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 44 {
		payload = append(payload, mavlink.ZeroTail[:44-len(p.Payload)]...)
	}
	m.Fitness = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.OfsX = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	m.OfsY = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.OfsZ = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.DiagX = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	m.DiagY = math.Float32frombits(binary.LittleEndian.Uint32(payload[20:]))
	m.DiagZ = math.Float32frombits(binary.LittleEndian.Uint32(payload[24:]))
	m.OffdiagX = math.Float32frombits(binary.LittleEndian.Uint32(payload[28:]))
	m.OffdiagY = math.Float32frombits(binary.LittleEndian.Uint32(payload[32:]))
	m.OffdiagZ = math.Float32frombits(binary.LittleEndian.Uint32(payload[36:]))
	m.CompassID = uint8(payload[40])
	m.CalMask = uint8(payload[41])
	m.CalStatus = MAG_CAL_STATUS(payload[42])
	m.Autosaved = uint8(payload[43])
	return nil
}

// EfiStatus struct (generated typeinfo)
// EFI status output
type EfiStatus struct {
	EcuIndex                  float32 // ECU index
	Rpm                       float32 // RPM
	FuelConsumed              float32 // Fuel consumed
	FuelFlow                  float32 // Fuel flow rate
	EngineLoad                float32 // Engine load
	ThrottlePosition          float32 // Throttle position
	SparkDwellTime            float32 // Spark dwell time
	BarometricPressure        float32 // Barometric pressure
	IntakeManifoldPressure    float32 // Intake manifold pressure(
	IntakeManifoldTemperature float32 // Intake manifold temperature
	CylinderHeadTemperature   float32 // Cylinder head temperature
	IgnitionTiming            float32 // Ignition timing (Crank angle degrees)
	InjectionTime             float32 // Injection time
	ExhaustGasTemperature     float32 // Exhaust gas temperature
	ThrottleOut               float32 // Output throttle
	PtCompensation            float32 // Pressure/temperature compensation
	Health                    uint8   // EFI health status
}

// MsgID (generated function)
func (m *EfiStatus) MsgID() mavlink.MessageID {
	return MSG_ID_EFI_STATUS
}

// String (generated function)
func (m *EfiStatus) String() string {
	return fmt.Sprintf(
		"&common.EfiStatus{ EcuIndex: %+v, Rpm: %+v, FuelConsumed: %+v, FuelFlow: %+v, EngineLoad: %+v, ThrottlePosition: %+v, SparkDwellTime: %+v, BarometricPressure: %+v, IntakeManifoldPressure: %+v, IntakeManifoldTemperature: %+v, CylinderHeadTemperature: %+v, IgnitionTiming: %+v, InjectionTime: %+v, ExhaustGasTemperature: %+v, ThrottleOut: %+v, PtCompensation: %+v, Health: %+v }",
		m.EcuIndex,
		m.Rpm,
		m.FuelConsumed,
		m.FuelFlow,
		m.EngineLoad,
		m.ThrottlePosition,
		m.SparkDwellTime,
		m.BarometricPressure,
		m.IntakeManifoldPressure,
		m.IntakeManifoldTemperature,
		m.CylinderHeadTemperature,
		m.IgnitionTiming,
		m.InjectionTime,
		m.ExhaustGasTemperature,
		m.ThrottleOut,
		m.PtCompensation,
		m.Health,
	)
}

// Pack (generated function)
func (m *EfiStatus) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 65)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(m.EcuIndex))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(m.Rpm))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.FuelConsumed))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.FuelFlow))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(m.EngineLoad))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(m.ThrottlePosition))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(m.SparkDwellTime))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(m.BarometricPressure))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(m.IntakeManifoldPressure))
	binary.LittleEndian.PutUint32(payload[36:], math.Float32bits(m.IntakeManifoldTemperature))
	binary.LittleEndian.PutUint32(payload[40:], math.Float32bits(m.CylinderHeadTemperature))
	binary.LittleEndian.PutUint32(payload[44:], math.Float32bits(m.IgnitionTiming))
	binary.LittleEndian.PutUint32(payload[48:], math.Float32bits(m.InjectionTime))
	binary.LittleEndian.PutUint32(payload[52:], math.Float32bits(m.ExhaustGasTemperature))
	binary.LittleEndian.PutUint32(payload[56:], math.Float32bits(m.ThrottleOut))
	binary.LittleEndian.PutUint32(payload[60:], math.Float32bits(m.PtCompensation))
	payload[64] = byte(m.Health)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *EfiStatus) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 65 {
		payload = append(payload, mavlink.ZeroTail[:65-len(p.Payload)]...)
	}
	m.EcuIndex = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.Rpm = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	m.FuelConsumed = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.FuelFlow = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.EngineLoad = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	m.ThrottlePosition = math.Float32frombits(binary.LittleEndian.Uint32(payload[20:]))
	m.SparkDwellTime = math.Float32frombits(binary.LittleEndian.Uint32(payload[24:]))
	m.BarometricPressure = math.Float32frombits(binary.LittleEndian.Uint32(payload[28:]))
	m.IntakeManifoldPressure = math.Float32frombits(binary.LittleEndian.Uint32(payload[32:]))
	m.IntakeManifoldTemperature = math.Float32frombits(binary.LittleEndian.Uint32(payload[36:]))
	m.CylinderHeadTemperature = math.Float32frombits(binary.LittleEndian.Uint32(payload[40:]))
	m.IgnitionTiming = math.Float32frombits(binary.LittleEndian.Uint32(payload[44:]))
	m.InjectionTime = math.Float32frombits(binary.LittleEndian.Uint32(payload[48:]))
	m.ExhaustGasTemperature = math.Float32frombits(binary.LittleEndian.Uint32(payload[52:]))
	m.ThrottleOut = math.Float32frombits(binary.LittleEndian.Uint32(payload[56:]))
	m.PtCompensation = math.Float32frombits(binary.LittleEndian.Uint32(payload[60:]))
	m.Health = uint8(payload[64])
	return nil
}

// EstimatorStatus struct (generated typeinfo)
// Estimator status message including flags, innovation test ratios and estimated accuracies. The flags message is an integer bitmask containing information on which EKF outputs are valid. See the ESTIMATOR_STATUS_FLAGS enum definition for further information. The innovation test ratios show the magnitude of the sensor innovation divided by the innovation check threshold. Under normal operation the innovation test ratios should be below 0.5 with occasional values up to 1.0. Values greater than 1.0 should be rare under normal operation and indicate that a measurement has been rejected by the filter. The user should be notified if an innovation test ratio greater than 1.0 is recorded. Notifications for values in the range between 0.5 and 1.0 should be optional and controllable by the user.
type EstimatorStatus struct {
	TimeUsec         uint64                 // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
	VelRatio         float32                // Velocity innovation test ratio
	PosHorizRatio    float32                // Horizontal position innovation test ratio
	PosVertRatio     float32                // Vertical position innovation test ratio
	MagRatio         float32                // Magnetometer innovation test ratio
	HaglRatio        float32                // Height above terrain innovation test ratio
	TasRatio         float32                // True airspeed innovation test ratio
	PosHorizAccuracy float32                // Horizontal position 1-STD accuracy relative to the EKF local origin
	PosVertAccuracy  float32                // Vertical position 1-STD accuracy relative to the EKF local origin
	Flags            ESTIMATOR_STATUS_FLAGS `gotype:"uint16"` // Bitmap indicating which EKF outputs are valid.
}

// MsgID (generated function)
func (m *EstimatorStatus) MsgID() mavlink.MessageID {
	return MSG_ID_ESTIMATOR_STATUS
}

// String (generated function)
func (m *EstimatorStatus) String() string {
	return fmt.Sprintf(
		"&common.EstimatorStatus{ TimeUsec: %+v, VelRatio: %+v, PosHorizRatio: %+v, PosVertRatio: %+v, MagRatio: %+v, HaglRatio: %+v, TasRatio: %+v, PosHorizAccuracy: %+v, PosVertAccuracy: %+v, Flags: %+v }",
		m.TimeUsec,
		m.VelRatio,
		m.PosHorizRatio,
		m.PosVertRatio,
		m.MagRatio,
		m.HaglRatio,
		m.TasRatio,
		m.PosHorizAccuracy,
		m.PosVertAccuracy,
		m.Flags,
	)
}

// Pack (generated function)
func (m *EstimatorStatus) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 42)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.TimeUsec))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.VelRatio))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.PosHorizRatio))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(m.PosVertRatio))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(m.MagRatio))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(m.HaglRatio))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(m.TasRatio))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(m.PosHorizAccuracy))
	binary.LittleEndian.PutUint32(payload[36:], math.Float32bits(m.PosVertAccuracy))
	binary.LittleEndian.PutUint16(payload[40:], uint16(m.Flags))
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *EstimatorStatus) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 42 {
		payload = append(payload, mavlink.ZeroTail[:42-len(p.Payload)]...)
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(payload[0:]))
	m.VelRatio = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.PosHorizRatio = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.PosVertRatio = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	m.MagRatio = math.Float32frombits(binary.LittleEndian.Uint32(payload[20:]))
	m.HaglRatio = math.Float32frombits(binary.LittleEndian.Uint32(payload[24:]))
	m.TasRatio = math.Float32frombits(binary.LittleEndian.Uint32(payload[28:]))
	m.PosHorizAccuracy = math.Float32frombits(binary.LittleEndian.Uint32(payload[32:]))
	m.PosVertAccuracy = math.Float32frombits(binary.LittleEndian.Uint32(payload[36:]))
	m.Flags = ESTIMATOR_STATUS_FLAGS(binary.LittleEndian.Uint16(payload[40:]))
	return nil
}

// WindCov struct (generated typeinfo)
// Wind covariance estimate from vehicle.
type WindCov struct {
	TimeUsec      uint64  // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
	WindX         float32 // Wind in X (NED) direction
	WindY         float32 // Wind in Y (NED) direction
	WindZ         float32 // Wind in Z (NED) direction
	VarHoriz      float32 // Variability of the wind in XY. RMS of a 1 Hz lowpassed wind estimate.
	VarVert       float32 // Variability of the wind in Z. RMS of a 1 Hz lowpassed wind estimate.
	WindAlt       float32 // Altitude (MSL) that this measurement was taken at
	HorizAccuracy float32 // Horizontal speed 1-STD accuracy
	VertAccuracy  float32 // Vertical speed 1-STD accuracy
}

// MsgID (generated function)
func (m *WindCov) MsgID() mavlink.MessageID {
	return MSG_ID_WIND_COV
}

// String (generated function)
func (m *WindCov) String() string {
	return fmt.Sprintf(
		"&common.WindCov{ TimeUsec: %+v, WindX: %+v, WindY: %+v, WindZ: %+v, VarHoriz: %+v, VarVert: %+v, WindAlt: %+v, HorizAccuracy: %+v, VertAccuracy: %+v }",
		m.TimeUsec,
		m.WindX,
		m.WindY,
		m.WindZ,
		m.VarHoriz,
		m.VarVert,
		m.WindAlt,
		m.HorizAccuracy,
		m.VertAccuracy,
	)
}

// Pack (generated function)
func (m *WindCov) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 40)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.TimeUsec))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.WindX))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.WindY))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(m.WindZ))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(m.VarHoriz))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(m.VarVert))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(m.WindAlt))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(m.HorizAccuracy))
	binary.LittleEndian.PutUint32(payload[36:], math.Float32bits(m.VertAccuracy))
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *WindCov) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 40 {
		payload = append(payload, mavlink.ZeroTail[:40-len(p.Payload)]...)
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(payload[0:]))
	m.WindX = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.WindY = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.WindZ = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	m.VarHoriz = math.Float32frombits(binary.LittleEndian.Uint32(payload[20:]))
	m.VarVert = math.Float32frombits(binary.LittleEndian.Uint32(payload[24:]))
	m.WindAlt = math.Float32frombits(binary.LittleEndian.Uint32(payload[28:]))
	m.HorizAccuracy = math.Float32frombits(binary.LittleEndian.Uint32(payload[32:]))
	m.VertAccuracy = math.Float32frombits(binary.LittleEndian.Uint32(payload[36:]))
	return nil
}

// GpsInput struct (generated typeinfo)
// GPS sensor input message.  This is a raw sensor value sent by the GPS. This is NOT the global position estimate of the system.
type GpsInput struct {
	TimeUsec          uint64                 // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
	TimeWeekMs        uint32                 // GPS time (from start of GPS week)
	Lat               int32                  // Latitude (WGS84)
	Lon               int32                  // Longitude (WGS84)
	Alt               float32                // Altitude (MSL). Positive for up.
	Hdop              float32                // GPS HDOP horizontal dilution of position
	Vdop              float32                // GPS VDOP vertical dilution of position
	Vn                float32                // GPS velocity in north direction in earth-fixed NED frame
	Ve                float32                // GPS velocity in east direction in earth-fixed NED frame
	Vd                float32                // GPS velocity in down direction in earth-fixed NED frame
	SpeedAccuracy     float32                // GPS speed accuracy
	HorizAccuracy     float32                // GPS horizontal accuracy
	VertAccuracy      float32                // GPS vertical accuracy
	IgnoreFlags       GPS_INPUT_IGNORE_FLAGS `gotype:"uint16"` // Bitmap indicating which GPS input flags fields to ignore.  All other fields must be provided.
	TimeWeek          uint16                 // GPS week number
	GpsID             uint8                  // ID of the GPS for multiple GPS inputs
	FixType           uint8                  // 0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RTK
	SatellitesVisible uint8                  // Number of satellites visible.
}

// MsgID (generated function)
func (m *GpsInput) MsgID() mavlink.MessageID {
	return MSG_ID_GPS_INPUT
}

// String (generated function)
func (m *GpsInput) String() string {
	return fmt.Sprintf(
		"&common.GpsInput{ TimeUsec: %+v, TimeWeekMs: %+v, Lat: %+v, Lon: %+v, Alt: %+v, Hdop: %+v, Vdop: %+v, Vn: %+v, Ve: %+v, Vd: %+v, SpeedAccuracy: %+v, HorizAccuracy: %+v, VertAccuracy: %+v, IgnoreFlags: %+v, TimeWeek: %+v, GpsID: %+v, FixType: %+v, SatellitesVisible: %+v }",
		m.TimeUsec,
		m.TimeWeekMs,
		m.Lat,
		m.Lon,
		m.Alt,
		m.Hdop,
		m.Vdop,
		m.Vn,
		m.Ve,
		m.Vd,
		m.SpeedAccuracy,
		m.HorizAccuracy,
		m.VertAccuracy,
		m.IgnoreFlags,
		m.TimeWeek,
		m.GpsID,
		m.FixType,
		m.SatellitesVisible,
	)
}

// Pack (generated function)
func (m *GpsInput) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 63)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.TimeUsec))
	binary.LittleEndian.PutUint32(payload[8:], uint32(m.TimeWeekMs))
	binary.LittleEndian.PutUint32(payload[12:], uint32(m.Lat))
	binary.LittleEndian.PutUint32(payload[16:], uint32(m.Lon))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(m.Alt))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(m.Hdop))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(m.Vdop))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(m.Vn))
	binary.LittleEndian.PutUint32(payload[36:], math.Float32bits(m.Ve))
	binary.LittleEndian.PutUint32(payload[40:], math.Float32bits(m.Vd))
	binary.LittleEndian.PutUint32(payload[44:], math.Float32bits(m.SpeedAccuracy))
	binary.LittleEndian.PutUint32(payload[48:], math.Float32bits(m.HorizAccuracy))
	binary.LittleEndian.PutUint32(payload[52:], math.Float32bits(m.VertAccuracy))
	binary.LittleEndian.PutUint16(payload[56:], uint16(m.IgnoreFlags))
	binary.LittleEndian.PutUint16(payload[58:], uint16(m.TimeWeek))
	payload[60] = byte(m.GpsID)
	payload[61] = byte(m.FixType)
	payload[62] = byte(m.SatellitesVisible)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *GpsInput) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 63 {
		payload = append(payload, mavlink.ZeroTail[:63-len(p.Payload)]...)
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(payload[0:]))
	m.TimeWeekMs = uint32(binary.LittleEndian.Uint32(payload[8:]))
	m.Lat = int32(binary.LittleEndian.Uint32(payload[12:]))
	m.Lon = int32(binary.LittleEndian.Uint32(payload[16:]))
	m.Alt = math.Float32frombits(binary.LittleEndian.Uint32(payload[20:]))
	m.Hdop = math.Float32frombits(binary.LittleEndian.Uint32(payload[24:]))
	m.Vdop = math.Float32frombits(binary.LittleEndian.Uint32(payload[28:]))
	m.Vn = math.Float32frombits(binary.LittleEndian.Uint32(payload[32:]))
	m.Ve = math.Float32frombits(binary.LittleEndian.Uint32(payload[36:]))
	m.Vd = math.Float32frombits(binary.LittleEndian.Uint32(payload[40:]))
	m.SpeedAccuracy = math.Float32frombits(binary.LittleEndian.Uint32(payload[44:]))
	m.HorizAccuracy = math.Float32frombits(binary.LittleEndian.Uint32(payload[48:]))
	m.VertAccuracy = math.Float32frombits(binary.LittleEndian.Uint32(payload[52:]))
	m.IgnoreFlags = GPS_INPUT_IGNORE_FLAGS(binary.LittleEndian.Uint16(payload[56:]))
	m.TimeWeek = uint16(binary.LittleEndian.Uint16(payload[58:]))
	m.GpsID = uint8(payload[60])
	m.FixType = uint8(payload[61])
	m.SatellitesVisible = uint8(payload[62])
	return nil
}

// GpsRtcmData struct (generated typeinfo)
// RTCM message for injecting into the onboard GPS (used for DGPS)
type GpsRtcmData struct {
	Flags uint8      // LSB: 1 means message is fragmented, next 2 bits are the fragment ID, the remaining 5 bits are used for the sequence ID. Messages are only to be flushed to the GPS when the entire message has been reconstructed on the autopilot. The fragment ID specifies which order the fragments should be assembled into a buffer, while the sequence ID is used to detect a mismatch between different buffers. The buffer is considered fully reconstructed when either all 4 fragments are present, or all the fragments before the first fragment with a non full payload is received. This management is used to ensure that normal GPS operation doesn't corrupt RTCM data, and to recover from a unreliable transport delivery order.
	Len   uint8      // data length
	Data  [180]uint8 // RTCM message (may be fragmented)
}

// MsgID (generated function)
func (m *GpsRtcmData) MsgID() mavlink.MessageID {
	return MSG_ID_GPS_RTCM_DATA
}

// String (generated function)
func (m *GpsRtcmData) String() string {
	return fmt.Sprintf(
		"&common.GpsRtcmData{ Flags: %+v, Len: %+v, Data: %0X (\"%s\") }",
		m.Flags,
		m.Len,
		m.Data, string(m.Data[:]),
	)
}

// Pack (generated function)
func (m *GpsRtcmData) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 182)
	payload[0] = byte(m.Flags)
	payload[1] = byte(m.Len)
	copy(payload[2:], m.Data[:])
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *GpsRtcmData) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 182 {
		payload = append(payload, mavlink.ZeroTail[:182-len(p.Payload)]...)
	}
	m.Flags = uint8(payload[0])
	m.Len = uint8(payload[1])
	copy(m.Data[:], payload[2:182])
	return nil
}

// HighLatency struct (generated typeinfo)
// Message appropriate for high latency connections like Iridium
type HighLatency struct {
	CustomMode       uint32           // A bitfield for use for autopilot-specific flags.
	Latitude         int32            // Latitude
	Longitude        int32            // Longitude
	Roll             int16            // roll
	Pitch            int16            // pitch
	Heading          uint16           // heading
	HeadingSp        int16            // heading setpoint
	AltitudeAmsl     int16            // Altitude above mean sea level
	AltitudeSp       int16            // Altitude setpoint relative to the home position
	WpDistance       uint16           // distance to target
	BaseMode         MAV_MODE_FLAG    `gotype:"uint8"` // Bitmap of enabled system modes.
	LandedState      MAV_LANDED_STATE `gotype:"uint8"` // The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.
	Throttle         int8             // throttle (percentage)
	Airspeed         uint8            // airspeed
	AirspeedSp       uint8            // airspeed setpoint
	Groundspeed      uint8            // groundspeed
	ClimbRate        int8             // climb rate
	GpsNsat          uint8            // Number of satellites visible. If unknown, set to 255
	GpsFixType       GPS_FIX_TYPE     `gotype:"uint8"` // GPS Fix type.
	BatteryRemaining uint8            // Remaining battery (percentage)
	Temperature      int8             // Autopilot temperature (degrees C)
	TemperatureAir   int8             // Air temperature (degrees C) from airspeed sensor
	Failsafe         uint8            // failsafe (each bit represents a failsafe where 0=ok, 1=failsafe active (bit0:RC, bit1:batt, bit2:GPS, bit3:GCS, bit4:fence)
	WpNum            uint8            // current waypoint number
}

// MsgID (generated function)
func (m *HighLatency) MsgID() mavlink.MessageID {
	return MSG_ID_HIGH_LATENCY
}

// String (generated function)
func (m *HighLatency) String() string {
	return fmt.Sprintf(
		"&common.HighLatency{ CustomMode: %+v, Latitude: %+v, Longitude: %+v, Roll: %+v, Pitch: %+v, Heading: %+v, HeadingSp: %+v, AltitudeAmsl: %+v, AltitudeSp: %+v, WpDistance: %+v, BaseMode: %+v, LandedState: %+v, Throttle: %+v, Airspeed: %+v, AirspeedSp: %+v, Groundspeed: %+v, ClimbRate: %+v, GpsNsat: %+v, GpsFixType: %+v, BatteryRemaining: %+v, Temperature: %+v, TemperatureAir: %+v, Failsafe: %+v, WpNum: %+v }",
		m.CustomMode,
		m.Latitude,
		m.Longitude,
		m.Roll,
		m.Pitch,
		m.Heading,
		m.HeadingSp,
		m.AltitudeAmsl,
		m.AltitudeSp,
		m.WpDistance,
		m.BaseMode,
		m.LandedState,
		m.Throttle,
		m.Airspeed,
		m.AirspeedSp,
		m.Groundspeed,
		m.ClimbRate,
		m.GpsNsat,
		m.GpsFixType,
		m.BatteryRemaining,
		m.Temperature,
		m.TemperatureAir,
		m.Failsafe,
		m.WpNum,
	)
}

// Pack (generated function)
func (m *HighLatency) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 40)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.CustomMode))
	binary.LittleEndian.PutUint32(payload[4:], uint32(m.Latitude))
	binary.LittleEndian.PutUint32(payload[8:], uint32(m.Longitude))
	binary.LittleEndian.PutUint16(payload[12:], uint16(m.Roll))
	binary.LittleEndian.PutUint16(payload[14:], uint16(m.Pitch))
	binary.LittleEndian.PutUint16(payload[16:], uint16(m.Heading))
	binary.LittleEndian.PutUint16(payload[18:], uint16(m.HeadingSp))
	binary.LittleEndian.PutUint16(payload[20:], uint16(m.AltitudeAmsl))
	binary.LittleEndian.PutUint16(payload[22:], uint16(m.AltitudeSp))
	binary.LittleEndian.PutUint16(payload[24:], uint16(m.WpDistance))
	payload[26] = byte(m.BaseMode)
	payload[27] = byte(m.LandedState)
	payload[28] = byte(m.Throttle)
	payload[29] = byte(m.Airspeed)
	payload[30] = byte(m.AirspeedSp)
	payload[31] = byte(m.Groundspeed)
	payload[32] = byte(m.ClimbRate)
	payload[33] = byte(m.GpsNsat)
	payload[34] = byte(m.GpsFixType)
	payload[35] = byte(m.BatteryRemaining)
	payload[36] = byte(m.Temperature)
	payload[37] = byte(m.TemperatureAir)
	payload[38] = byte(m.Failsafe)
	payload[39] = byte(m.WpNum)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *HighLatency) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 40 {
		payload = append(payload, mavlink.ZeroTail[:40-len(p.Payload)]...)
	}
	m.CustomMode = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.Latitude = int32(binary.LittleEndian.Uint32(payload[4:]))
	m.Longitude = int32(binary.LittleEndian.Uint32(payload[8:]))
	m.Roll = int16(binary.LittleEndian.Uint16(payload[12:]))
	m.Pitch = int16(binary.LittleEndian.Uint16(payload[14:]))
	m.Heading = uint16(binary.LittleEndian.Uint16(payload[16:]))
	m.HeadingSp = int16(binary.LittleEndian.Uint16(payload[18:]))
	m.AltitudeAmsl = int16(binary.LittleEndian.Uint16(payload[20:]))
	m.AltitudeSp = int16(binary.LittleEndian.Uint16(payload[22:]))
	m.WpDistance = uint16(binary.LittleEndian.Uint16(payload[24:]))
	m.BaseMode = MAV_MODE_FLAG(payload[26])
	m.LandedState = MAV_LANDED_STATE(payload[27])
	m.Throttle = int8(payload[28])
	m.Airspeed = uint8(payload[29])
	m.AirspeedSp = uint8(payload[30])
	m.Groundspeed = uint8(payload[31])
	m.ClimbRate = int8(payload[32])
	m.GpsNsat = uint8(payload[33])
	m.GpsFixType = GPS_FIX_TYPE(payload[34])
	m.BatteryRemaining = uint8(payload[35])
	m.Temperature = int8(payload[36])
	m.TemperatureAir = int8(payload[37])
	m.Failsafe = uint8(payload[38])
	m.WpNum = uint8(payload[39])
	return nil
}

// HighLatency2 struct (generated typeinfo)
// Message appropriate for high latency connections like Iridium (version 2)
type HighLatency2 struct {
	Timestamp      uint32          // Timestamp (milliseconds since boot or Unix epoch)
	Latitude       int32           // Latitude
	Longitude      int32           // Longitude
	CustomMode     uint16          // A bitfield for use for autopilot-specific flags (2 byte version).
	Altitude       int16           // Altitude above mean sea level
	TargetAltitude int16           // Altitude setpoint
	TargetDistance uint16          // Distance to target waypoint or position
	WpNum          uint16          // Current waypoint number
	FailureFlags   HL_FAILURE_FLAG `gotype:"uint16"` // Bitmap of failure flags.
	Type           MAV_TYPE        `gotype:"uint8"`  // Type of the MAV (quadrotor, helicopter, etc.)
	Autopilot      MAV_AUTOPILOT   `gotype:"uint8"`  // Autopilot type / class. Use MAV_AUTOPILOT_INVALID for components that are not flight controllers.
	Heading        uint8           // Heading
	TargetHeading  uint8           // Heading setpoint
	Throttle       uint8           // Throttle
	Airspeed       uint8           // Airspeed
	AirspeedSp     uint8           // Airspeed setpoint
	Groundspeed    uint8           // Groundspeed
	Windspeed      uint8           // Windspeed
	WindHeading    uint8           // Wind heading
	Eph            uint8           // Maximum error horizontal position since last message
	Epv            uint8           // Maximum error vertical position since last message
	TemperatureAir int8            // Air temperature from airspeed sensor
	ClimbRate      int8            // Maximum climb rate magnitude since last message
	Battery        int8            // Battery level (-1 if field not provided).
	Custom0        int8            // Field for custom payload.
	Custom1        int8            // Field for custom payload.
	Custom2        int8            // Field for custom payload.
}

// MsgID (generated function)
func (m *HighLatency2) MsgID() mavlink.MessageID {
	return MSG_ID_HIGH_LATENCY2
}

// String (generated function)
func (m *HighLatency2) String() string {
	return fmt.Sprintf(
		"&common.HighLatency2{ Timestamp: %+v, Latitude: %+v, Longitude: %+v, CustomMode: %+v, Altitude: %+v, TargetAltitude: %+v, TargetDistance: %+v, WpNum: %+v, FailureFlags: %+v, Type: %+v, Autopilot: %+v, Heading: %+v, TargetHeading: %+v, Throttle: %+v, Airspeed: %+v, AirspeedSp: %+v, Groundspeed: %+v, Windspeed: %+v, WindHeading: %+v, Eph: %+v, Epv: %+v, TemperatureAir: %+v, ClimbRate: %+v, Battery: %+v, Custom0: %+v, Custom1: %+v, Custom2: %+v }",
		m.Timestamp,
		m.Latitude,
		m.Longitude,
		m.CustomMode,
		m.Altitude,
		m.TargetAltitude,
		m.TargetDistance,
		m.WpNum,
		m.FailureFlags,
		m.Type,
		m.Autopilot,
		m.Heading,
		m.TargetHeading,
		m.Throttle,
		m.Airspeed,
		m.AirspeedSp,
		m.Groundspeed,
		m.Windspeed,
		m.WindHeading,
		m.Eph,
		m.Epv,
		m.TemperatureAir,
		m.ClimbRate,
		m.Battery,
		m.Custom0,
		m.Custom1,
		m.Custom2,
	)
}

// Pack (generated function)
func (m *HighLatency2) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 42)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.Timestamp))
	binary.LittleEndian.PutUint32(payload[4:], uint32(m.Latitude))
	binary.LittleEndian.PutUint32(payload[8:], uint32(m.Longitude))
	binary.LittleEndian.PutUint16(payload[12:], uint16(m.CustomMode))
	binary.LittleEndian.PutUint16(payload[14:], uint16(m.Altitude))
	binary.LittleEndian.PutUint16(payload[16:], uint16(m.TargetAltitude))
	binary.LittleEndian.PutUint16(payload[18:], uint16(m.TargetDistance))
	binary.LittleEndian.PutUint16(payload[20:], uint16(m.WpNum))
	binary.LittleEndian.PutUint16(payload[22:], uint16(m.FailureFlags))
	payload[24] = byte(m.Type)
	payload[25] = byte(m.Autopilot)
	payload[26] = byte(m.Heading)
	payload[27] = byte(m.TargetHeading)
	payload[28] = byte(m.Throttle)
	payload[29] = byte(m.Airspeed)
	payload[30] = byte(m.AirspeedSp)
	payload[31] = byte(m.Groundspeed)
	payload[32] = byte(m.Windspeed)
	payload[33] = byte(m.WindHeading)
	payload[34] = byte(m.Eph)
	payload[35] = byte(m.Epv)
	payload[36] = byte(m.TemperatureAir)
	payload[37] = byte(m.ClimbRate)
	payload[38] = byte(m.Battery)
	payload[39] = byte(m.Custom0)
	payload[40] = byte(m.Custom1)
	payload[41] = byte(m.Custom2)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *HighLatency2) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 42 {
		payload = append(payload, mavlink.ZeroTail[:42-len(p.Payload)]...)
	}
	m.Timestamp = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.Latitude = int32(binary.LittleEndian.Uint32(payload[4:]))
	m.Longitude = int32(binary.LittleEndian.Uint32(payload[8:]))
	m.CustomMode = uint16(binary.LittleEndian.Uint16(payload[12:]))
	m.Altitude = int16(binary.LittleEndian.Uint16(payload[14:]))
	m.TargetAltitude = int16(binary.LittleEndian.Uint16(payload[16:]))
	m.TargetDistance = uint16(binary.LittleEndian.Uint16(payload[18:]))
	m.WpNum = uint16(binary.LittleEndian.Uint16(payload[20:]))
	m.FailureFlags = HL_FAILURE_FLAG(binary.LittleEndian.Uint16(payload[22:]))
	m.Type = MAV_TYPE(payload[24])
	m.Autopilot = MAV_AUTOPILOT(payload[25])
	m.Heading = uint8(payload[26])
	m.TargetHeading = uint8(payload[27])
	m.Throttle = uint8(payload[28])
	m.Airspeed = uint8(payload[29])
	m.AirspeedSp = uint8(payload[30])
	m.Groundspeed = uint8(payload[31])
	m.Windspeed = uint8(payload[32])
	m.WindHeading = uint8(payload[33])
	m.Eph = uint8(payload[34])
	m.Epv = uint8(payload[35])
	m.TemperatureAir = int8(payload[36])
	m.ClimbRate = int8(payload[37])
	m.Battery = int8(payload[38])
	m.Custom0 = int8(payload[39])
	m.Custom1 = int8(payload[40])
	m.Custom2 = int8(payload[41])
	return nil
}

// Vibration struct (generated typeinfo)
// Vibration levels and accelerometer clipping
type Vibration struct {
	TimeUsec   uint64  // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
	VibrationX float32 // Vibration levels on X-axis
	VibrationY float32 // Vibration levels on Y-axis
	VibrationZ float32 // Vibration levels on Z-axis
	Clipping0  uint32  // first accelerometer clipping count
	Clipping1  uint32  // second accelerometer clipping count
	Clipping2  uint32  // third accelerometer clipping count
}

// MsgID (generated function)
func (m *Vibration) MsgID() mavlink.MessageID {
	return MSG_ID_VIBRATION
}

// String (generated function)
func (m *Vibration) String() string {
	return fmt.Sprintf(
		"&common.Vibration{ TimeUsec: %+v, VibrationX: %+v, VibrationY: %+v, VibrationZ: %+v, Clipping0: %+v, Clipping1: %+v, Clipping2: %+v }",
		m.TimeUsec,
		m.VibrationX,
		m.VibrationY,
		m.VibrationZ,
		m.Clipping0,
		m.Clipping1,
		m.Clipping2,
	)
}

// Pack (generated function)
func (m *Vibration) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 32)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.TimeUsec))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.VibrationX))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.VibrationY))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(m.VibrationZ))
	binary.LittleEndian.PutUint32(payload[20:], uint32(m.Clipping0))
	binary.LittleEndian.PutUint32(payload[24:], uint32(m.Clipping1))
	binary.LittleEndian.PutUint32(payload[28:], uint32(m.Clipping2))
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *Vibration) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 32 {
		payload = append(payload, mavlink.ZeroTail[:32-len(p.Payload)]...)
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(payload[0:]))
	m.VibrationX = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.VibrationY = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.VibrationZ = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	m.Clipping0 = uint32(binary.LittleEndian.Uint32(payload[20:]))
	m.Clipping1 = uint32(binary.LittleEndian.Uint32(payload[24:]))
	m.Clipping2 = uint32(binary.LittleEndian.Uint32(payload[28:]))
	return nil
}

// HomePosition struct (generated typeinfo)
// This message can be requested by sending the MAV_CMD_GET_HOME_POSITION command. The position the system will return to and land on. The position is set automatically by the system during the takeoff in case it was not explicitly set by the operator before or after. The global and local positions encode the position in the respective coordinate frames, while the q parameter encodes the orientation of the surface. Under normal conditions it describes the heading and terrain slope, which can be used by the aircraft to adjust the approach. The approach 3D vector describes the point to which the system should fly in normal flight mode and then perform a landing sequence along the vector.
type HomePosition struct {
	Latitude  int32      // Latitude (WGS84)
	Longitude int32      // Longitude (WGS84)
	Altitude  int32      // Altitude (MSL). Positive for up.
	X         float32    // Local X position of this position in the local coordinate frame
	Y         float32    // Local Y position of this position in the local coordinate frame
	Z         float32    // Local Z position of this position in the local coordinate frame
	Q         [4]float32 // World to surface normal and heading transformation of the takeoff position. Used to indicate the heading and slope of the ground
	ApproachX float32    // Local X position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.
	ApproachY float32    // Local Y position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.
	ApproachZ float32    // Local Z position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.
}

// MsgID (generated function)
func (m *HomePosition) MsgID() mavlink.MessageID {
	return MSG_ID_HOME_POSITION
}

// String (generated function)
func (m *HomePosition) String() string {
	return fmt.Sprintf(
		"&common.HomePosition{ Latitude: %+v, Longitude: %+v, Altitude: %+v, X: %+v, Y: %+v, Z: %+v, Q: %+v, ApproachX: %+v, ApproachY: %+v, ApproachZ: %+v }",
		m.Latitude,
		m.Longitude,
		m.Altitude,
		m.X,
		m.Y,
		m.Z,
		m.Q,
		m.ApproachX,
		m.ApproachY,
		m.ApproachZ,
	)
}

// Pack (generated function)
func (m *HomePosition) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 52)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.Latitude))
	binary.LittleEndian.PutUint32(payload[4:], uint32(m.Longitude))
	binary.LittleEndian.PutUint32(payload[8:], uint32(m.Altitude))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.X))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(m.Y))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(m.Z))
	for i, v := range m.Q {
		binary.LittleEndian.PutUint32(payload[24+i*4:], math.Float32bits(v))
	}
	binary.LittleEndian.PutUint32(payload[40:], math.Float32bits(m.ApproachX))
	binary.LittleEndian.PutUint32(payload[44:], math.Float32bits(m.ApproachY))
	binary.LittleEndian.PutUint32(payload[48:], math.Float32bits(m.ApproachZ))
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *HomePosition) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 52 {
		payload = append(payload, mavlink.ZeroTail[:52-len(p.Payload)]...)
	}
	m.Latitude = int32(binary.LittleEndian.Uint32(payload[0:]))
	m.Longitude = int32(binary.LittleEndian.Uint32(payload[4:]))
	m.Altitude = int32(binary.LittleEndian.Uint32(payload[8:]))
	m.X = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.Y = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	m.Z = math.Float32frombits(binary.LittleEndian.Uint32(payload[20:]))
	for i := 0; i < len(m.Q); i++ {
		m.Q[i] = math.Float32frombits(binary.LittleEndian.Uint32(payload[24+i*4:]))
	}
	m.ApproachX = math.Float32frombits(binary.LittleEndian.Uint32(payload[40:]))
	m.ApproachY = math.Float32frombits(binary.LittleEndian.Uint32(payload[44:]))
	m.ApproachZ = math.Float32frombits(binary.LittleEndian.Uint32(payload[48:]))
	return nil
}

// SetHomePosition struct (generated typeinfo)
// The position the system will return to and land on. The position is set automatically by the system during the takeoff in case it was not explicitly set by the operator before or after. The global and local positions encode the position in the respective coordinate frames, while the q parameter encodes the orientation of the surface. Under normal conditions it describes the heading and terrain slope, which can be used by the aircraft to adjust the approach. The approach 3D vector describes the point to which the system should fly in normal flight mode and then perform a landing sequence along the vector.
type SetHomePosition struct {
	Latitude     int32      // Latitude (WGS84)
	Longitude    int32      // Longitude (WGS84)
	Altitude     int32      // Altitude (MSL). Positive for up.
	X            float32    // Local X position of this position in the local coordinate frame
	Y            float32    // Local Y position of this position in the local coordinate frame
	Z            float32    // Local Z position of this position in the local coordinate frame
	Q            [4]float32 // World to surface normal and heading transformation of the takeoff position. Used to indicate the heading and slope of the ground
	ApproachX    float32    // Local X position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.
	ApproachY    float32    // Local Y position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.
	ApproachZ    float32    // Local Z position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.
	TargetSystem uint8      // System ID.
}

// MsgID (generated function)
func (m *SetHomePosition) MsgID() mavlink.MessageID {
	return MSG_ID_SET_HOME_POSITION
}

// String (generated function)
func (m *SetHomePosition) String() string {
	return fmt.Sprintf(
		"&common.SetHomePosition{ Latitude: %+v, Longitude: %+v, Altitude: %+v, X: %+v, Y: %+v, Z: %+v, Q: %+v, ApproachX: %+v, ApproachY: %+v, ApproachZ: %+v, TargetSystem: %+v }",
		m.Latitude,
		m.Longitude,
		m.Altitude,
		m.X,
		m.Y,
		m.Z,
		m.Q,
		m.ApproachX,
		m.ApproachY,
		m.ApproachZ,
		m.TargetSystem,
	)
}

// Pack (generated function)
func (m *SetHomePosition) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 53)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.Latitude))
	binary.LittleEndian.PutUint32(payload[4:], uint32(m.Longitude))
	binary.LittleEndian.PutUint32(payload[8:], uint32(m.Altitude))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.X))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(m.Y))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(m.Z))
	for i, v := range m.Q {
		binary.LittleEndian.PutUint32(payload[24+i*4:], math.Float32bits(v))
	}
	binary.LittleEndian.PutUint32(payload[40:], math.Float32bits(m.ApproachX))
	binary.LittleEndian.PutUint32(payload[44:], math.Float32bits(m.ApproachY))
	binary.LittleEndian.PutUint32(payload[48:], math.Float32bits(m.ApproachZ))
	payload[52] = byte(m.TargetSystem)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *SetHomePosition) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 53 {
		payload = append(payload, mavlink.ZeroTail[:53-len(p.Payload)]...)
	}
	m.Latitude = int32(binary.LittleEndian.Uint32(payload[0:]))
	m.Longitude = int32(binary.LittleEndian.Uint32(payload[4:]))
	m.Altitude = int32(binary.LittleEndian.Uint32(payload[8:]))
	m.X = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.Y = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	m.Z = math.Float32frombits(binary.LittleEndian.Uint32(payload[20:]))
	for i := 0; i < len(m.Q); i++ {
		m.Q[i] = math.Float32frombits(binary.LittleEndian.Uint32(payload[24+i*4:]))
	}
	m.ApproachX = math.Float32frombits(binary.LittleEndian.Uint32(payload[40:]))
	m.ApproachY = math.Float32frombits(binary.LittleEndian.Uint32(payload[44:]))
	m.ApproachZ = math.Float32frombits(binary.LittleEndian.Uint32(payload[48:]))
	m.TargetSystem = uint8(payload[52])
	return nil
}

// MessageInterval struct (generated typeinfo)
// The interval between messages for a particular MAVLink message ID. This message is the response to the MAV_CMD_GET_MESSAGE_INTERVAL command. This interface replaces DATA_STREAM.
type MessageInterval struct {
	IntervalUs int32  // The interval between two messages. A value of -1 indicates this stream is disabled, 0 indicates it is not available, &gt; 0 indicates the interval at which it is sent.
	MessageID  uint16 // The ID of the requested MAVLink message. v1.0 is limited to 254 messages.
}

// MsgID (generated function)
func (m *MessageInterval) MsgID() mavlink.MessageID {
	return MSG_ID_MESSAGE_INTERVAL
}

// String (generated function)
func (m *MessageInterval) String() string {
	return fmt.Sprintf(
		"&common.MessageInterval{ IntervalUs: %+v, MessageID: %+v }",
		m.IntervalUs,
		m.MessageID,
	)
}

// Pack (generated function)
func (m *MessageInterval) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 6)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.IntervalUs))
	binary.LittleEndian.PutUint16(payload[4:], uint16(m.MessageID))
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *MessageInterval) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 6 {
		payload = append(payload, mavlink.ZeroTail[:6-len(p.Payload)]...)
	}
	m.IntervalUs = int32(binary.LittleEndian.Uint32(payload[0:]))
	m.MessageID = uint16(binary.LittleEndian.Uint16(payload[4:]))
	return nil
}

// ExtendedSysState struct (generated typeinfo)
// Provides state for additional features
type ExtendedSysState struct {
	VtolState   MAV_VTOL_STATE   `gotype:"uint8"` // The VTOL state if applicable. Is set to MAV_VTOL_STATE_UNDEFINED if UAV is not in VTOL configuration.
	LandedState MAV_LANDED_STATE `gotype:"uint8"` // The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.
}

// MsgID (generated function)
func (m *ExtendedSysState) MsgID() mavlink.MessageID {
	return MSG_ID_EXTENDED_SYS_STATE
}

// String (generated function)
func (m *ExtendedSysState) String() string {
	return fmt.Sprintf(
		"&common.ExtendedSysState{ VtolState: %+v, LandedState: %+v }",
		m.VtolState,
		m.LandedState,
	)
}

// Pack (generated function)
func (m *ExtendedSysState) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 2)
	payload[0] = byte(m.VtolState)
	payload[1] = byte(m.LandedState)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ExtendedSysState) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 2 {
		payload = append(payload, mavlink.ZeroTail[:2-len(p.Payload)]...)
	}
	m.VtolState = MAV_VTOL_STATE(payload[0])
	m.LandedState = MAV_LANDED_STATE(payload[1])
	return nil
}

// AdsbVehicle struct (generated typeinfo)
// The location and information of an ADSB vehicle
type AdsbVehicle struct {
	IcaoAddress  uint32             // ICAO address
	Lat          int32              // Latitude
	Lon          int32              // Longitude
	Altitude     int32              // Altitude(ASL)
	Heading      uint16             // Course over ground
	HorVelocity  uint16             // The horizontal velocity
	VerVelocity  int16              // The vertical velocity. Positive is up
	Flags        ADSB_FLAGS         `gotype:"uint16"` // Bitmap to indicate various statuses including valid data fields
	Squawk       uint16             // Squawk code
	AltitudeType ADSB_ALTITUDE_TYPE `gotype:"uint8"` // ADSB altitude type.
	Callsign     [9]byte            // The callsign, 8+null
	EmitterType  ADSB_EMITTER_TYPE  `gotype:"uint8"` // ADSB emitter type.
	Tslc         uint8              // Time since last communication in seconds
}

// MsgID (generated function)
func (m *AdsbVehicle) MsgID() mavlink.MessageID {
	return MSG_ID_ADSB_VEHICLE
}

// String (generated function)
func (m *AdsbVehicle) String() string {
	return fmt.Sprintf(
		"&common.AdsbVehicle{ IcaoAddress: %+v, Lat: %+v, Lon: %+v, Altitude: %+v, Heading: %+v, HorVelocity: %+v, VerVelocity: %+v, Flags: %+v, Squawk: %+v, AltitudeType: %+v, Callsign: %0X (\"%s\"), EmitterType: %+v, Tslc: %+v }",
		m.IcaoAddress,
		m.Lat,
		m.Lon,
		m.Altitude,
		m.Heading,
		m.HorVelocity,
		m.VerVelocity,
		m.Flags,
		m.Squawk,
		m.AltitudeType,
		m.Callsign, string(m.Callsign[:]),
		m.EmitterType,
		m.Tslc,
	)
}

// Pack (generated function)
func (m *AdsbVehicle) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 38)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.IcaoAddress))
	binary.LittleEndian.PutUint32(payload[4:], uint32(m.Lat))
	binary.LittleEndian.PutUint32(payload[8:], uint32(m.Lon))
	binary.LittleEndian.PutUint32(payload[12:], uint32(m.Altitude))
	binary.LittleEndian.PutUint16(payload[16:], uint16(m.Heading))
	binary.LittleEndian.PutUint16(payload[18:], uint16(m.HorVelocity))
	binary.LittleEndian.PutUint16(payload[20:], uint16(m.VerVelocity))
	binary.LittleEndian.PutUint16(payload[22:], uint16(m.Flags))
	binary.LittleEndian.PutUint16(payload[24:], uint16(m.Squawk))
	payload[26] = byte(m.AltitudeType)
	copy(payload[27:], m.Callsign[:])
	payload[36] = byte(m.EmitterType)
	payload[37] = byte(m.Tslc)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *AdsbVehicle) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 38 {
		payload = append(payload, mavlink.ZeroTail[:38-len(p.Payload)]...)
	}
	m.IcaoAddress = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.Lat = int32(binary.LittleEndian.Uint32(payload[4:]))
	m.Lon = int32(binary.LittleEndian.Uint32(payload[8:]))
	m.Altitude = int32(binary.LittleEndian.Uint32(payload[12:]))
	m.Heading = uint16(binary.LittleEndian.Uint16(payload[16:]))
	m.HorVelocity = uint16(binary.LittleEndian.Uint16(payload[18:]))
	m.VerVelocity = int16(binary.LittleEndian.Uint16(payload[20:]))
	m.Flags = ADSB_FLAGS(binary.LittleEndian.Uint16(payload[22:]))
	m.Squawk = uint16(binary.LittleEndian.Uint16(payload[24:]))
	m.AltitudeType = ADSB_ALTITUDE_TYPE(payload[26])
	copy(m.Callsign[:], payload[27:36])
	m.EmitterType = ADSB_EMITTER_TYPE(payload[36])
	m.Tslc = uint8(payload[37])
	return nil
}

// Collision struct (generated typeinfo)
// Information about a potential collision
type Collision struct {
	ID                     uint32                     // Unique identifier, domain based on src field
	TimeToMinimumDelta     float32                    // Estimated time until collision occurs
	AltitudeMinimumDelta   float32                    // Closest vertical distance between vehicle and object
	HorizontalMinimumDelta float32                    // Closest horizontal distance between vehicle and object
	Src                    MAV_COLLISION_SRC          `gotype:"uint8"` // Collision data source
	Action                 MAV_COLLISION_ACTION       `gotype:"uint8"` // Action that is being taken to avoid this collision
	ThreatLevel            MAV_COLLISION_THREAT_LEVEL `gotype:"uint8"` // How concerned the aircraft is about this collision
}

// MsgID (generated function)
func (m *Collision) MsgID() mavlink.MessageID {
	return MSG_ID_COLLISION
}

// String (generated function)
func (m *Collision) String() string {
	return fmt.Sprintf(
		"&common.Collision{ ID: %+v, TimeToMinimumDelta: %+v, AltitudeMinimumDelta: %+v, HorizontalMinimumDelta: %+v, Src: %+v, Action: %+v, ThreatLevel: %+v }",
		m.ID,
		m.TimeToMinimumDelta,
		m.AltitudeMinimumDelta,
		m.HorizontalMinimumDelta,
		m.Src,
		m.Action,
		m.ThreatLevel,
	)
}

// Pack (generated function)
func (m *Collision) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 19)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.ID))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(m.TimeToMinimumDelta))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.AltitudeMinimumDelta))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.HorizontalMinimumDelta))
	payload[16] = byte(m.Src)
	payload[17] = byte(m.Action)
	payload[18] = byte(m.ThreatLevel)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *Collision) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 19 {
		payload = append(payload, mavlink.ZeroTail[:19-len(p.Payload)]...)
	}
	m.ID = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.TimeToMinimumDelta = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	m.AltitudeMinimumDelta = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.HorizontalMinimumDelta = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.Src = MAV_COLLISION_SRC(payload[16])
	m.Action = MAV_COLLISION_ACTION(payload[17])
	m.ThreatLevel = MAV_COLLISION_THREAT_LEVEL(payload[18])
	return nil
}

// V2Extension struct (generated typeinfo)
// Message implementing parts of the V2 payload specs in V1 frames for transitional support.
type V2Extension struct {
	MessageType     uint16     // A code that identifies the software component that understands this message (analogous to USB device classes or mime type strings). If this code is less than 32768, it is considered a 'registered' protocol extension and the corresponding entry should be added to https://github.com/mavlink/mavlink/definition_files/extension_message_ids.xml. Software creators can register blocks of message IDs as needed (useful for GCS specific metadata, etc...). Message_types greater than 32767 are considered local experiments and should not be checked in to any widely distributed codebase.
	TargetNetwork   uint8      // Network ID (0 for broadcast)
	TargetSystem    uint8      // System ID (0 for broadcast)
	TargetComponent uint8      // Component ID (0 for broadcast)
	Payload         [249]uint8 // Variable length payload. The length must be encoded in the payload as part of the message_type protocol, e.g. by including the length as payload data, or by terminating the payload data with a non-zero marker. This is required in order to reconstruct zero-terminated payloads that are (or otherwise would be) trimmed by MAVLink 2 empty-byte truncation. The entire content of the payload block is opaque unless you understand the encoding message_type. The particular encoding used can be extension specific and might not always be documented as part of the MAVLink specification.
}

// MsgID (generated function)
func (m *V2Extension) MsgID() mavlink.MessageID {
	return MSG_ID_V2_EXTENSION
}

// String (generated function)
func (m *V2Extension) String() string {
	return fmt.Sprintf(
		"&common.V2Extension{ MessageType: %+v, TargetNetwork: %+v, TargetSystem: %+v, TargetComponent: %+v, Payload: %0X (\"%s\") }",
		m.MessageType,
		m.TargetNetwork,
		m.TargetSystem,
		m.TargetComponent,
		m.Payload, string(m.Payload[:]),
	)
}

// Pack (generated function)
func (m *V2Extension) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 254)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.MessageType))
	payload[2] = byte(m.TargetNetwork)
	payload[3] = byte(m.TargetSystem)
	payload[4] = byte(m.TargetComponent)
	copy(payload[5:], m.Payload[:])
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *V2Extension) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 254 {
		payload = append(payload, mavlink.ZeroTail[:254-len(p.Payload)]...)
	}
	m.MessageType = uint16(binary.LittleEndian.Uint16(payload[0:]))
	m.TargetNetwork = uint8(payload[2])
	m.TargetSystem = uint8(payload[3])
	m.TargetComponent = uint8(payload[4])
	copy(m.Payload[:], payload[5:254])
	return nil
}

// MemoryVect struct (generated typeinfo)
// Send raw controller memory. The use of this message is discouraged for normal packets, but a quite efficient way for testing new messages and getting experimental debug output.
type MemoryVect struct {
	Address uint16   // Starting address of the debug variables
	Ver     uint8    // Version code of the type variable. 0=unknown, type ignored and assumed int16_t. 1=as below
	Type    uint8    // Type code of the memory variables. for ver = 1: 0=16 x int16_t, 1=16 x uint16_t, 2=16 x Q15, 3=16 x 1Q14
	Value   [32]int8 // Memory contents at specified address
}

// MsgID (generated function)
func (m *MemoryVect) MsgID() mavlink.MessageID {
	return MSG_ID_MEMORY_VECT
}

// String (generated function)
func (m *MemoryVect) String() string {
	return fmt.Sprintf(
		"&common.MemoryVect{ Address: %+v, Ver: %+v, Type: %+v, Value: %+v }",
		m.Address,
		m.Ver,
		m.Type,
		m.Value,
	)
}

// Pack (generated function)
func (m *MemoryVect) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 36)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.Address))
	payload[2] = byte(m.Ver)
	payload[3] = byte(m.Type)
	for i, v := range m.Value {
		payload[4+i*1] = byte(v)
	}
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *MemoryVect) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 36 {
		payload = append(payload, mavlink.ZeroTail[:36-len(p.Payload)]...)
	}
	m.Address = uint16(binary.LittleEndian.Uint16(payload[0:]))
	m.Ver = uint8(payload[2])
	m.Type = uint8(payload[3])
	for i := 0; i < len(m.Value); i++ {
		m.Value[i] = int8(payload[4+i*1])
	}
	return nil
}

// DebugVect struct (generated typeinfo)
// To debug something using a named 3D vector.
type DebugVect struct {
	TimeUsec uint64   // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
	X        float32  // x
	Y        float32  // y
	Z        float32  // z
	Name     [10]byte // Name
}

// MsgID (generated function)
func (m *DebugVect) MsgID() mavlink.MessageID {
	return MSG_ID_DEBUG_VECT
}

// String (generated function)
func (m *DebugVect) String() string {
	return fmt.Sprintf(
		"&common.DebugVect{ TimeUsec: %+v, X: %+v, Y: %+v, Z: %+v, Name: %0X (\"%s\") }",
		m.TimeUsec,
		m.X,
		m.Y,
		m.Z,
		m.Name, string(m.Name[:]),
	)
}

// Pack (generated function)
func (m *DebugVect) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 30)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.TimeUsec))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.X))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.Y))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(m.Z))
	copy(payload[20:], m.Name[:])
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *DebugVect) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 30 {
		payload = append(payload, mavlink.ZeroTail[:30-len(p.Payload)]...)
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(payload[0:]))
	m.X = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.Y = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.Z = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	copy(m.Name[:], payload[20:30])
	return nil
}

// NamedValueFloat struct (generated typeinfo)
// Send a key-value pair as float. The use of this message is discouraged for normal packets, but a quite efficient way for testing new messages and getting experimental debug output.
type NamedValueFloat struct {
	TimeBootMs uint32   // Timestamp (time since system boot).
	Value      float32  // Floating point value
	Name       [10]byte // Name of the debug variable
}

// MsgID (generated function)
func (m *NamedValueFloat) MsgID() mavlink.MessageID {
	return MSG_ID_NAMED_VALUE_FLOAT
}

// String (generated function)
func (m *NamedValueFloat) String() string {
	return fmt.Sprintf(
		"&common.NamedValueFloat{ TimeBootMs: %+v, Value: %+v, Name: %0X (\"%s\") }",
		m.TimeBootMs,
		m.Value,
		m.Name, string(m.Name[:]),
	)
}

// Pack (generated function)
func (m *NamedValueFloat) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 18)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.TimeBootMs))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(m.Value))
	copy(payload[8:], m.Name[:])
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *NamedValueFloat) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 18 {
		payload = append(payload, mavlink.ZeroTail[:18-len(p.Payload)]...)
	}
	m.TimeBootMs = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.Value = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	copy(m.Name[:], payload[8:18])
	return nil
}

// NamedValueInt struct (generated typeinfo)
// Send a key-value pair as integer. The use of this message is discouraged for normal packets, but a quite efficient way for testing new messages and getting experimental debug output.
type NamedValueInt struct {
	TimeBootMs uint32   // Timestamp (time since system boot).
	Value      int32    // Signed integer value
	Name       [10]byte // Name of the debug variable
}

// MsgID (generated function)
func (m *NamedValueInt) MsgID() mavlink.MessageID {
	return MSG_ID_NAMED_VALUE_INT
}

// String (generated function)
func (m *NamedValueInt) String() string {
	return fmt.Sprintf(
		"&common.NamedValueInt{ TimeBootMs: %+v, Value: %+v, Name: %0X (\"%s\") }",
		m.TimeBootMs,
		m.Value,
		m.Name, string(m.Name[:]),
	)
}

// Pack (generated function)
func (m *NamedValueInt) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 18)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.TimeBootMs))
	binary.LittleEndian.PutUint32(payload[4:], uint32(m.Value))
	copy(payload[8:], m.Name[:])
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *NamedValueInt) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 18 {
		payload = append(payload, mavlink.ZeroTail[:18-len(p.Payload)]...)
	}
	m.TimeBootMs = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.Value = int32(binary.LittleEndian.Uint32(payload[4:]))
	copy(m.Name[:], payload[8:18])
	return nil
}

// Statustext struct (generated typeinfo)
// Status text message. These messages are printed in yellow in the COMM console of QGroundControl. WARNING: They consume quite some bandwidth, so use only for important status and error messages. If implemented wisely, these messages are buffered on the MCU and sent only at a limited rate (e.g. 10 Hz).
type Statustext struct {
	Severity MAV_SEVERITY `gotype:"uint8"` // Severity of status. Relies on the definitions within RFC-5424.
	Text     [50]byte     // Status text message, without null termination character
}

// MsgID (generated function)
func (m *Statustext) MsgID() mavlink.MessageID {
	return MSG_ID_STATUSTEXT
}

// String (generated function)
func (m *Statustext) String() string {
	return fmt.Sprintf(
		"&common.Statustext{ Severity: %+v, Text: %0X (\"%s\") }",
		m.Severity,
		m.Text, string(m.Text[:]),
	)
}

// Pack (generated function)
func (m *Statustext) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 51)
	payload[0] = byte(m.Severity)
	copy(payload[1:], m.Text[:])
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *Statustext) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 51 {
		payload = append(payload, mavlink.ZeroTail[:51-len(p.Payload)]...)
	}
	m.Severity = MAV_SEVERITY(payload[0])
	copy(m.Text[:], payload[1:51])
	return nil
}

// Debug struct (generated typeinfo)
// Send a debug value. The index is used to discriminate between values. These values show up in the plot of QGroundControl as DEBUG N.
type Debug struct {
	TimeBootMs uint32  // Timestamp (time since system boot).
	Value      float32 // DEBUG value
	Ind        uint8   // index of debug variable
}

// MsgID (generated function)
func (m *Debug) MsgID() mavlink.MessageID {
	return MSG_ID_DEBUG
}

// String (generated function)
func (m *Debug) String() string {
	return fmt.Sprintf(
		"&common.Debug{ TimeBootMs: %+v, Value: %+v, Ind: %+v }",
		m.TimeBootMs,
		m.Value,
		m.Ind,
	)
}

// Pack (generated function)
func (m *Debug) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 9)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.TimeBootMs))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(m.Value))
	payload[8] = byte(m.Ind)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *Debug) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 9 {
		payload = append(payload, mavlink.ZeroTail[:9-len(p.Payload)]...)
	}
	m.TimeBootMs = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.Value = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	m.Ind = uint8(payload[8])
	return nil
}

// Heartbeat struct (generated typeinfo)
// The heartbeat message shows that a system or component is present and responding. The type and autopilot fields (along with the message component id), allow the receiving system to treat further messages from this system appropriately (e.g. by laying out the user interface based on the autopilot). This microservice is documented at https://mavlink.io/en/services/heartbeat.html
type Heartbeat struct {
	CustomMode     uint32        // A bitfield for use for autopilot-specific flags
	Type           MAV_TYPE      `gotype:"uint8"` // Vehicle or component type. For a flight controller component the vehicle type (quadrotor, helicopter, etc.). For other components the component type (e.g. camera, gimbal, etc.). This should be used in preference to component id for identifying the component type.
	Autopilot      MAV_AUTOPILOT `gotype:"uint8"` // Autopilot type / class. Use MAV_AUTOPILOT_INVALID for components that are not flight controllers.
	BaseMode       MAV_MODE_FLAG `gotype:"uint8"` // System mode bitmap.
	SystemStatus   MAV_STATE     `gotype:"uint8"` // System status flag.
	MavlinkVersion uint8         // MAVLink version, not writable by user, gets added by protocol because of magic data type: uint8_t_mavlink_version
}

// MsgID (generated function)
func (m *Heartbeat) MsgID() mavlink.MessageID {
	return MSG_ID_HEARTBEAT
}

// String (generated function)
func (m *Heartbeat) String() string {
	return fmt.Sprintf(
		"&common.Heartbeat{ CustomMode: %+v, Type: %+v, Autopilot: %+v, BaseMode: %+v, SystemStatus: %+v, MavlinkVersion: %+v }",
		m.CustomMode,
		m.Type,
		m.Autopilot,
		m.BaseMode,
		m.SystemStatus,
		m.MavlinkVersion,
	)
}

// Pack (generated function)
func (m *Heartbeat) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 9)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.CustomMode))
	payload[4] = byte(m.Type)
	payload[5] = byte(m.Autopilot)
	payload[6] = byte(m.BaseMode)
	payload[7] = byte(m.SystemStatus)
	payload[8] = byte(m.MavlinkVersion)
	payloadLen := len(payload)
	for payloadLen > 1 && payload[payloadLen-1] == 0 {
		payloadLen--
	}
	payload = payload[:payloadLen]
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *Heartbeat) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 9 {
		payload = append(payload, mavlink.ZeroTail[:9-len(p.Payload)]...)
	}
	m.CustomMode = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.Type = MAV_TYPE(payload[4])
	m.Autopilot = MAV_AUTOPILOT(payload[5])
	m.BaseMode = MAV_MODE_FLAG(payload[6])
	m.SystemStatus = MAV_STATE(payload[7])
	m.MavlinkVersion = uint8(payload[8])
	return nil
}

// Message IDs
const (
	MSG_ID_NAV_FILTER_BIAS                         mavlink.MessageID = 220
	MSG_ID_RADIO_CALIBRATION                       mavlink.MessageID = 221
	MSG_ID_UALBERTA_SYS_STATUS                     mavlink.MessageID = 222
	MSG_ID_SYS_STATUS                              mavlink.MessageID = 1
	MSG_ID_SYSTEM_TIME                             mavlink.MessageID = 2
	MSG_ID_PING                                    mavlink.MessageID = 4
	MSG_ID_CHANGE_OPERATOR_CONTROL                 mavlink.MessageID = 5
	MSG_ID_CHANGE_OPERATOR_CONTROL_ACK             mavlink.MessageID = 6
	MSG_ID_AUTH_KEY                                mavlink.MessageID = 7
	MSG_ID_LINK_NODE_STATUS                        mavlink.MessageID = 8
	MSG_ID_SET_MODE                                mavlink.MessageID = 11
	MSG_ID_PARAM_ACK_TRANSACTION                   mavlink.MessageID = 19
	MSG_ID_PARAM_REQUEST_READ                      mavlink.MessageID = 20
	MSG_ID_PARAM_REQUEST_LIST                      mavlink.MessageID = 21
	MSG_ID_PARAM_VALUE                             mavlink.MessageID = 22
	MSG_ID_PARAM_SET                               mavlink.MessageID = 23
	MSG_ID_GPS_RAW_INT                             mavlink.MessageID = 24
	MSG_ID_GPS_STATUS                              mavlink.MessageID = 25
	MSG_ID_SCALED_IMU                              mavlink.MessageID = 26
	MSG_ID_RAW_IMU                                 mavlink.MessageID = 27
	MSG_ID_RAW_PRESSURE                            mavlink.MessageID = 28
	MSG_ID_SCALED_PRESSURE                         mavlink.MessageID = 29
	MSG_ID_ATTITUDE                                mavlink.MessageID = 30
	MSG_ID_ATTITUDE_QUATERNION                     mavlink.MessageID = 31
	MSG_ID_LOCAL_POSITION_NED                      mavlink.MessageID = 32
	MSG_ID_GLOBAL_POSITION_INT                     mavlink.MessageID = 33
	MSG_ID_RC_CHANNELS_SCALED                      mavlink.MessageID = 34
	MSG_ID_RC_CHANNELS_RAW                         mavlink.MessageID = 35
	MSG_ID_SERVO_OUTPUT_RAW                        mavlink.MessageID = 36
	MSG_ID_MISSION_REQUEST_PARTIAL_LIST            mavlink.MessageID = 37
	MSG_ID_MISSION_WRITE_PARTIAL_LIST              mavlink.MessageID = 38
	MSG_ID_MISSION_ITEM                            mavlink.MessageID = 39
	MSG_ID_MISSION_REQUEST                         mavlink.MessageID = 40
	MSG_ID_MISSION_SET_CURRENT                     mavlink.MessageID = 41
	MSG_ID_MISSION_CURRENT                         mavlink.MessageID = 42
	MSG_ID_MISSION_REQUEST_LIST                    mavlink.MessageID = 43
	MSG_ID_MISSION_COUNT                           mavlink.MessageID = 44
	MSG_ID_MISSION_CLEAR_ALL                       mavlink.MessageID = 45
	MSG_ID_MISSION_ITEM_REACHED                    mavlink.MessageID = 46
	MSG_ID_MISSION_ACK                             mavlink.MessageID = 47
	MSG_ID_SET_GPS_GLOBAL_ORIGIN                   mavlink.MessageID = 48
	MSG_ID_GPS_GLOBAL_ORIGIN                       mavlink.MessageID = 49
	MSG_ID_PARAM_MAP_RC                            mavlink.MessageID = 50
	MSG_ID_MISSION_REQUEST_INT                     mavlink.MessageID = 51
	MSG_ID_MISSION_CHANGED                         mavlink.MessageID = 52
	MSG_ID_SAFETY_SET_ALLOWED_AREA                 mavlink.MessageID = 54
	MSG_ID_SAFETY_ALLOWED_AREA                     mavlink.MessageID = 55
	MSG_ID_ATTITUDE_QUATERNION_COV                 mavlink.MessageID = 61
	MSG_ID_NAV_CONTROLLER_OUTPUT                   mavlink.MessageID = 62
	MSG_ID_GLOBAL_POSITION_INT_COV                 mavlink.MessageID = 63
	MSG_ID_LOCAL_POSITION_NED_COV                  mavlink.MessageID = 64
	MSG_ID_RC_CHANNELS                             mavlink.MessageID = 65
	MSG_ID_REQUEST_DATA_STREAM                     mavlink.MessageID = 66
	MSG_ID_DATA_STREAM                             mavlink.MessageID = 67
	MSG_ID_MANUAL_CONTROL                          mavlink.MessageID = 69
	MSG_ID_RC_CHANNELS_OVERRIDE                    mavlink.MessageID = 70
	MSG_ID_MISSION_ITEM_INT                        mavlink.MessageID = 73
	MSG_ID_VFR_HUD                                 mavlink.MessageID = 74
	MSG_ID_COMMAND_INT                             mavlink.MessageID = 75
	MSG_ID_COMMAND_LONG                            mavlink.MessageID = 76
	MSG_ID_COMMAND_ACK                             mavlink.MessageID = 77
	MSG_ID_COMMAND_CANCEL                          mavlink.MessageID = 80
	MSG_ID_MANUAL_SETPOINT                         mavlink.MessageID = 81
	MSG_ID_SET_ATTITUDE_TARGET                     mavlink.MessageID = 82
	MSG_ID_ATTITUDE_TARGET                         mavlink.MessageID = 83
	MSG_ID_SET_POSITION_TARGET_LOCAL_NED           mavlink.MessageID = 84
	MSG_ID_POSITION_TARGET_LOCAL_NED               mavlink.MessageID = 85
	MSG_ID_SET_POSITION_TARGET_GLOBAL_INT          mavlink.MessageID = 86
	MSG_ID_POSITION_TARGET_GLOBAL_INT              mavlink.MessageID = 87
	MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET mavlink.MessageID = 89
	MSG_ID_HIL_STATE                               mavlink.MessageID = 90
	MSG_ID_HIL_CONTROLS                            mavlink.MessageID = 91
	MSG_ID_HIL_RC_INPUTS_RAW                       mavlink.MessageID = 92
	MSG_ID_HIL_ACTUATOR_CONTROLS                   mavlink.MessageID = 93
	MSG_ID_OPTICAL_FLOW                            mavlink.MessageID = 100
	MSG_ID_GLOBAL_VISION_POSITION_ESTIMATE         mavlink.MessageID = 101
	MSG_ID_VISION_POSITION_ESTIMATE                mavlink.MessageID = 102
	MSG_ID_VISION_SPEED_ESTIMATE                   mavlink.MessageID = 103
	MSG_ID_VICON_POSITION_ESTIMATE                 mavlink.MessageID = 104
	MSG_ID_HIGHRES_IMU                             mavlink.MessageID = 105
	MSG_ID_OPTICAL_FLOW_RAD                        mavlink.MessageID = 106
	MSG_ID_HIL_SENSOR                              mavlink.MessageID = 107
	MSG_ID_SIM_STATE                               mavlink.MessageID = 108
	MSG_ID_RADIO_STATUS                            mavlink.MessageID = 109
	MSG_ID_FILE_TRANSFER_PROTOCOL                  mavlink.MessageID = 110
	MSG_ID_TIMESYNC                                mavlink.MessageID = 111
	MSG_ID_CAMERA_TRIGGER                          mavlink.MessageID = 112
	MSG_ID_HIL_GPS                                 mavlink.MessageID = 113
	MSG_ID_HIL_OPTICAL_FLOW                        mavlink.MessageID = 114
	MSG_ID_HIL_STATE_QUATERNION                    mavlink.MessageID = 115
	MSG_ID_SCALED_IMU2                             mavlink.MessageID = 116
	MSG_ID_LOG_REQUEST_LIST                        mavlink.MessageID = 117
	MSG_ID_LOG_ENTRY                               mavlink.MessageID = 118
	MSG_ID_LOG_REQUEST_DATA                        mavlink.MessageID = 119
	MSG_ID_LOG_DATA                                mavlink.MessageID = 120
	MSG_ID_LOG_ERASE                               mavlink.MessageID = 121
	MSG_ID_LOG_REQUEST_END                         mavlink.MessageID = 122
	MSG_ID_GPS_INJECT_DATA                         mavlink.MessageID = 123
	MSG_ID_GPS2_RAW                                mavlink.MessageID = 124
	MSG_ID_POWER_STATUS                            mavlink.MessageID = 125
	MSG_ID_SERIAL_CONTROL                          mavlink.MessageID = 126
	MSG_ID_GPS_RTK                                 mavlink.MessageID = 127
	MSG_ID_GPS2_RTK                                mavlink.MessageID = 128
	MSG_ID_SCALED_IMU3                             mavlink.MessageID = 129
	MSG_ID_DATA_TRANSMISSION_HANDSHAKE             mavlink.MessageID = 130
	MSG_ID_ENCAPSULATED_DATA                       mavlink.MessageID = 131
	MSG_ID_DISTANCE_SENSOR                         mavlink.MessageID = 132
	MSG_ID_TERRAIN_REQUEST                         mavlink.MessageID = 133
	MSG_ID_TERRAIN_DATA                            mavlink.MessageID = 134
	MSG_ID_TERRAIN_CHECK                           mavlink.MessageID = 135
	MSG_ID_TERRAIN_REPORT                          mavlink.MessageID = 136
	MSG_ID_SCALED_PRESSURE2                        mavlink.MessageID = 137
	MSG_ID_ATT_POS_MOCAP                           mavlink.MessageID = 138
	MSG_ID_SET_ACTUATOR_CONTROL_TARGET             mavlink.MessageID = 139
	MSG_ID_ACTUATOR_CONTROL_TARGET                 mavlink.MessageID = 140
	MSG_ID_ALTITUDE                                mavlink.MessageID = 141
	MSG_ID_RESOURCE_REQUEST                        mavlink.MessageID = 142
	MSG_ID_SCALED_PRESSURE3                        mavlink.MessageID = 143
	MSG_ID_FOLLOW_TARGET                           mavlink.MessageID = 144
	MSG_ID_CONTROL_SYSTEM_STATE                    mavlink.MessageID = 146
	MSG_ID_BATTERY_STATUS                          mavlink.MessageID = 147
	MSG_ID_AUTOPILOT_VERSION                       mavlink.MessageID = 148
	MSG_ID_LANDING_TARGET                          mavlink.MessageID = 149
	MSG_ID_FENCE_STATUS                            mavlink.MessageID = 162
	MSG_ID_MAG_CAL_REPORT                          mavlink.MessageID = 192
	MSG_ID_EFI_STATUS                              mavlink.MessageID = 225
	MSG_ID_ESTIMATOR_STATUS                        mavlink.MessageID = 230
	MSG_ID_WIND_COV                                mavlink.MessageID = 231
	MSG_ID_GPS_INPUT                               mavlink.MessageID = 232
	MSG_ID_GPS_RTCM_DATA                           mavlink.MessageID = 233
	MSG_ID_HIGH_LATENCY                            mavlink.MessageID = 234
	MSG_ID_HIGH_LATENCY2                           mavlink.MessageID = 235
	MSG_ID_VIBRATION                               mavlink.MessageID = 241
	MSG_ID_HOME_POSITION                           mavlink.MessageID = 242
	MSG_ID_SET_HOME_POSITION                       mavlink.MessageID = 243
	MSG_ID_MESSAGE_INTERVAL                        mavlink.MessageID = 244
	MSG_ID_EXTENDED_SYS_STATE                      mavlink.MessageID = 245
	MSG_ID_ADSB_VEHICLE                            mavlink.MessageID = 246
	MSG_ID_COLLISION                               mavlink.MessageID = 247
	MSG_ID_V2_EXTENSION                            mavlink.MessageID = 248
	MSG_ID_MEMORY_VECT                             mavlink.MessageID = 249
	MSG_ID_DEBUG_VECT                              mavlink.MessageID = 250
	MSG_ID_NAMED_VALUE_FLOAT                       mavlink.MessageID = 251
	MSG_ID_NAMED_VALUE_INT                         mavlink.MessageID = 252
	MSG_ID_STATUSTEXT                              mavlink.MessageID = 253
	MSG_ID_DEBUG                                   mavlink.MessageID = 254
	MSG_ID_HEARTBEAT                               mavlink.MessageID = 0
)

func init() {
	mavlink.Register(MSG_ID_NAV_FILTER_BIAS, "MSG_ID_NAV_FILTER_BIAS", 34, func(p *mavlink.Packet) mavlink.Message {
		msg := new(NavFilterBias)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_RADIO_CALIBRATION, "MSG_ID_RADIO_CALIBRATION", 71, func(p *mavlink.Packet) mavlink.Message {
		msg := new(RadioCalibration)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_UALBERTA_SYS_STATUS, "MSG_ID_UALBERTA_SYS_STATUS", 15, func(p *mavlink.Packet) mavlink.Message {
		msg := new(UalbertaSysStatus)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_SYS_STATUS, "MSG_ID_SYS_STATUS", 124, func(p *mavlink.Packet) mavlink.Message {
		msg := new(SysStatus)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_SYSTEM_TIME, "MSG_ID_SYSTEM_TIME", 137, func(p *mavlink.Packet) mavlink.Message {
		msg := new(SystemTime)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_PING, "MSG_ID_PING", 237, func(p *mavlink.Packet) mavlink.Message {
		msg := new(Ping)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_CHANGE_OPERATOR_CONTROL, "MSG_ID_CHANGE_OPERATOR_CONTROL", 217, func(p *mavlink.Packet) mavlink.Message {
		msg := new(ChangeOperatorControl)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_CHANGE_OPERATOR_CONTROL_ACK, "MSG_ID_CHANGE_OPERATOR_CONTROL_ACK", 104, func(p *mavlink.Packet) mavlink.Message {
		msg := new(ChangeOperatorControlAck)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_AUTH_KEY, "MSG_ID_AUTH_KEY", 119, func(p *mavlink.Packet) mavlink.Message {
		msg := new(AuthKey)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_LINK_NODE_STATUS, "MSG_ID_LINK_NODE_STATUS", 117, func(p *mavlink.Packet) mavlink.Message {
		msg := new(LinkNodeStatus)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_SET_MODE, "MSG_ID_SET_MODE", 89, func(p *mavlink.Packet) mavlink.Message {
		msg := new(SetMode)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_PARAM_ACK_TRANSACTION, "MSG_ID_PARAM_ACK_TRANSACTION", 137, func(p *mavlink.Packet) mavlink.Message {
		msg := new(ParamAckTransaction)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_PARAM_REQUEST_READ, "MSG_ID_PARAM_REQUEST_READ", 214, func(p *mavlink.Packet) mavlink.Message {
		msg := new(ParamRequestRead)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_PARAM_REQUEST_LIST, "MSG_ID_PARAM_REQUEST_LIST", 159, func(p *mavlink.Packet) mavlink.Message {
		msg := new(ParamRequestList)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_PARAM_VALUE, "MSG_ID_PARAM_VALUE", 220, func(p *mavlink.Packet) mavlink.Message {
		msg := new(ParamValue)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_PARAM_SET, "MSG_ID_PARAM_SET", 168, func(p *mavlink.Packet) mavlink.Message {
		msg := new(ParamSet)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_GPS_RAW_INT, "MSG_ID_GPS_RAW_INT", 24, func(p *mavlink.Packet) mavlink.Message {
		msg := new(GpsRawInt)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_GPS_STATUS, "MSG_ID_GPS_STATUS", 23, func(p *mavlink.Packet) mavlink.Message {
		msg := new(GpsStatus)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_SCALED_IMU, "MSG_ID_SCALED_IMU", 170, func(p *mavlink.Packet) mavlink.Message {
		msg := new(ScaledImu)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_RAW_IMU, "MSG_ID_RAW_IMU", 144, func(p *mavlink.Packet) mavlink.Message {
		msg := new(RawImu)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_RAW_PRESSURE, "MSG_ID_RAW_PRESSURE", 67, func(p *mavlink.Packet) mavlink.Message {
		msg := new(RawPressure)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_SCALED_PRESSURE, "MSG_ID_SCALED_PRESSURE", 115, func(p *mavlink.Packet) mavlink.Message {
		msg := new(ScaledPressure)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_ATTITUDE, "MSG_ID_ATTITUDE", 39, func(p *mavlink.Packet) mavlink.Message {
		msg := new(Attitude)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_ATTITUDE_QUATERNION, "MSG_ID_ATTITUDE_QUATERNION", 246, func(p *mavlink.Packet) mavlink.Message {
		msg := new(AttitudeQuaternion)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_LOCAL_POSITION_NED, "MSG_ID_LOCAL_POSITION_NED", 185, func(p *mavlink.Packet) mavlink.Message {
		msg := new(LocalPositionNed)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_GLOBAL_POSITION_INT, "MSG_ID_GLOBAL_POSITION_INT", 104, func(p *mavlink.Packet) mavlink.Message {
		msg := new(GlobalPositionInt)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_RC_CHANNELS_SCALED, "MSG_ID_RC_CHANNELS_SCALED", 237, func(p *mavlink.Packet) mavlink.Message {
		msg := new(RcChannelsScaled)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_RC_CHANNELS_RAW, "MSG_ID_RC_CHANNELS_RAW", 244, func(p *mavlink.Packet) mavlink.Message {
		msg := new(RcChannelsRaw)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_SERVO_OUTPUT_RAW, "MSG_ID_SERVO_OUTPUT_RAW", 222, func(p *mavlink.Packet) mavlink.Message {
		msg := new(ServoOutputRaw)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_MISSION_REQUEST_PARTIAL_LIST, "MSG_ID_MISSION_REQUEST_PARTIAL_LIST", 212, func(p *mavlink.Packet) mavlink.Message {
		msg := new(MissionRequestPartialList)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_MISSION_WRITE_PARTIAL_LIST, "MSG_ID_MISSION_WRITE_PARTIAL_LIST", 9, func(p *mavlink.Packet) mavlink.Message {
		msg := new(MissionWritePartialList)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_MISSION_ITEM, "MSG_ID_MISSION_ITEM", 254, func(p *mavlink.Packet) mavlink.Message {
		msg := new(MissionItem)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_MISSION_REQUEST, "MSG_ID_MISSION_REQUEST", 230, func(p *mavlink.Packet) mavlink.Message {
		msg := new(MissionRequest)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_MISSION_SET_CURRENT, "MSG_ID_MISSION_SET_CURRENT", 28, func(p *mavlink.Packet) mavlink.Message {
		msg := new(MissionSetCurrent)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_MISSION_CURRENT, "MSG_ID_MISSION_CURRENT", 28, func(p *mavlink.Packet) mavlink.Message {
		msg := new(MissionCurrent)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_MISSION_REQUEST_LIST, "MSG_ID_MISSION_REQUEST_LIST", 132, func(p *mavlink.Packet) mavlink.Message {
		msg := new(MissionRequestList)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_MISSION_COUNT, "MSG_ID_MISSION_COUNT", 221, func(p *mavlink.Packet) mavlink.Message {
		msg := new(MissionCount)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_MISSION_CLEAR_ALL, "MSG_ID_MISSION_CLEAR_ALL", 232, func(p *mavlink.Packet) mavlink.Message {
		msg := new(MissionClearAll)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_MISSION_ITEM_REACHED, "MSG_ID_MISSION_ITEM_REACHED", 11, func(p *mavlink.Packet) mavlink.Message {
		msg := new(MissionItemReached)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_MISSION_ACK, "MSG_ID_MISSION_ACK", 153, func(p *mavlink.Packet) mavlink.Message {
		msg := new(MissionAck)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_SET_GPS_GLOBAL_ORIGIN, "MSG_ID_SET_GPS_GLOBAL_ORIGIN", 41, func(p *mavlink.Packet) mavlink.Message {
		msg := new(SetGpsGlobalOrigin)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_GPS_GLOBAL_ORIGIN, "MSG_ID_GPS_GLOBAL_ORIGIN", 39, func(p *mavlink.Packet) mavlink.Message {
		msg := new(GpsGlobalOrigin)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_PARAM_MAP_RC, "MSG_ID_PARAM_MAP_RC", 78, func(p *mavlink.Packet) mavlink.Message {
		msg := new(ParamMapRc)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_MISSION_REQUEST_INT, "MSG_ID_MISSION_REQUEST_INT", 196, func(p *mavlink.Packet) mavlink.Message {
		msg := new(MissionRequestInt)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_MISSION_CHANGED, "MSG_ID_MISSION_CHANGED", 132, func(p *mavlink.Packet) mavlink.Message {
		msg := new(MissionChanged)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_SAFETY_SET_ALLOWED_AREA, "MSG_ID_SAFETY_SET_ALLOWED_AREA", 15, func(p *mavlink.Packet) mavlink.Message {
		msg := new(SafetySetAllowedArea)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_SAFETY_ALLOWED_AREA, "MSG_ID_SAFETY_ALLOWED_AREA", 3, func(p *mavlink.Packet) mavlink.Message {
		msg := new(SafetyAllowedArea)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_ATTITUDE_QUATERNION_COV, "MSG_ID_ATTITUDE_QUATERNION_COV", 167, func(p *mavlink.Packet) mavlink.Message {
		msg := new(AttitudeQuaternionCov)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_NAV_CONTROLLER_OUTPUT, "MSG_ID_NAV_CONTROLLER_OUTPUT", 183, func(p *mavlink.Packet) mavlink.Message {
		msg := new(NavControllerOutput)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_GLOBAL_POSITION_INT_COV, "MSG_ID_GLOBAL_POSITION_INT_COV", 119, func(p *mavlink.Packet) mavlink.Message {
		msg := new(GlobalPositionIntCov)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_LOCAL_POSITION_NED_COV, "MSG_ID_LOCAL_POSITION_NED_COV", 191, func(p *mavlink.Packet) mavlink.Message {
		msg := new(LocalPositionNedCov)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_RC_CHANNELS, "MSG_ID_RC_CHANNELS", 118, func(p *mavlink.Packet) mavlink.Message {
		msg := new(RcChannels)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_REQUEST_DATA_STREAM, "MSG_ID_REQUEST_DATA_STREAM", 148, func(p *mavlink.Packet) mavlink.Message {
		msg := new(RequestDataStream)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_DATA_STREAM, "MSG_ID_DATA_STREAM", 21, func(p *mavlink.Packet) mavlink.Message {
		msg := new(DataStream)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_MANUAL_CONTROL, "MSG_ID_MANUAL_CONTROL", 243, func(p *mavlink.Packet) mavlink.Message {
		msg := new(ManualControl)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_RC_CHANNELS_OVERRIDE, "MSG_ID_RC_CHANNELS_OVERRIDE", 124, func(p *mavlink.Packet) mavlink.Message {
		msg := new(RcChannelsOverride)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_MISSION_ITEM_INT, "MSG_ID_MISSION_ITEM_INT", 38, func(p *mavlink.Packet) mavlink.Message {
		msg := new(MissionItemInt)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_VFR_HUD, "MSG_ID_VFR_HUD", 20, func(p *mavlink.Packet) mavlink.Message {
		msg := new(VfrHud)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_COMMAND_INT, "MSG_ID_COMMAND_INT", 158, func(p *mavlink.Packet) mavlink.Message {
		msg := new(CommandInt)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_COMMAND_LONG, "MSG_ID_COMMAND_LONG", 152, func(p *mavlink.Packet) mavlink.Message {
		msg := new(CommandLong)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_COMMAND_ACK, "MSG_ID_COMMAND_ACK", 143, func(p *mavlink.Packet) mavlink.Message {
		msg := new(CommandAck)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_COMMAND_CANCEL, "MSG_ID_COMMAND_CANCEL", 14, func(p *mavlink.Packet) mavlink.Message {
		msg := new(CommandCancel)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_MANUAL_SETPOINT, "MSG_ID_MANUAL_SETPOINT", 106, func(p *mavlink.Packet) mavlink.Message {
		msg := new(ManualSetpoint)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_SET_ATTITUDE_TARGET, "MSG_ID_SET_ATTITUDE_TARGET", 49, func(p *mavlink.Packet) mavlink.Message {
		msg := new(SetAttitudeTarget)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_ATTITUDE_TARGET, "MSG_ID_ATTITUDE_TARGET", 22, func(p *mavlink.Packet) mavlink.Message {
		msg := new(AttitudeTarget)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_SET_POSITION_TARGET_LOCAL_NED, "MSG_ID_SET_POSITION_TARGET_LOCAL_NED", 143, func(p *mavlink.Packet) mavlink.Message {
		msg := new(SetPositionTargetLocalNed)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_POSITION_TARGET_LOCAL_NED, "MSG_ID_POSITION_TARGET_LOCAL_NED", 140, func(p *mavlink.Packet) mavlink.Message {
		msg := new(PositionTargetLocalNed)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_SET_POSITION_TARGET_GLOBAL_INT, "MSG_ID_SET_POSITION_TARGET_GLOBAL_INT", 5, func(p *mavlink.Packet) mavlink.Message {
		msg := new(SetPositionTargetGlobalInt)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_POSITION_TARGET_GLOBAL_INT, "MSG_ID_POSITION_TARGET_GLOBAL_INT", 150, func(p *mavlink.Packet) mavlink.Message {
		msg := new(PositionTargetGlobalInt)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET, "MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET", 231, func(p *mavlink.Packet) mavlink.Message {
		msg := new(LocalPositionNedSystemGlobalOffset)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_HIL_STATE, "MSG_ID_HIL_STATE", 183, func(p *mavlink.Packet) mavlink.Message {
		msg := new(HilState)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_HIL_CONTROLS, "MSG_ID_HIL_CONTROLS", 63, func(p *mavlink.Packet) mavlink.Message {
		msg := new(HilControls)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_HIL_RC_INPUTS_RAW, "MSG_ID_HIL_RC_INPUTS_RAW", 54, func(p *mavlink.Packet) mavlink.Message {
		msg := new(HilRcInputsRaw)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_HIL_ACTUATOR_CONTROLS, "MSG_ID_HIL_ACTUATOR_CONTROLS", 47, func(p *mavlink.Packet) mavlink.Message {
		msg := new(HilActuatorControls)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_OPTICAL_FLOW, "MSG_ID_OPTICAL_FLOW", 175, func(p *mavlink.Packet) mavlink.Message {
		msg := new(OpticalFlow)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_GLOBAL_VISION_POSITION_ESTIMATE, "MSG_ID_GLOBAL_VISION_POSITION_ESTIMATE", 102, func(p *mavlink.Packet) mavlink.Message {
		msg := new(GlobalVisionPositionEstimate)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_VISION_POSITION_ESTIMATE, "MSG_ID_VISION_POSITION_ESTIMATE", 158, func(p *mavlink.Packet) mavlink.Message {
		msg := new(VisionPositionEstimate)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_VISION_SPEED_ESTIMATE, "MSG_ID_VISION_SPEED_ESTIMATE", 208, func(p *mavlink.Packet) mavlink.Message {
		msg := new(VisionSpeedEstimate)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_VICON_POSITION_ESTIMATE, "MSG_ID_VICON_POSITION_ESTIMATE", 56, func(p *mavlink.Packet) mavlink.Message {
		msg := new(ViconPositionEstimate)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_HIGHRES_IMU, "MSG_ID_HIGHRES_IMU", 93, func(p *mavlink.Packet) mavlink.Message {
		msg := new(HighresImu)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_OPTICAL_FLOW_RAD, "MSG_ID_OPTICAL_FLOW_RAD", 138, func(p *mavlink.Packet) mavlink.Message {
		msg := new(OpticalFlowRad)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_HIL_SENSOR, "MSG_ID_HIL_SENSOR", 108, func(p *mavlink.Packet) mavlink.Message {
		msg := new(HilSensor)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_SIM_STATE, "MSG_ID_SIM_STATE", 32, func(p *mavlink.Packet) mavlink.Message {
		msg := new(SimState)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_RADIO_STATUS, "MSG_ID_RADIO_STATUS", 185, func(p *mavlink.Packet) mavlink.Message {
		msg := new(RadioStatus)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_FILE_TRANSFER_PROTOCOL, "MSG_ID_FILE_TRANSFER_PROTOCOL", 84, func(p *mavlink.Packet) mavlink.Message {
		msg := new(FileTransferProtocol)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_TIMESYNC, "MSG_ID_TIMESYNC", 34, func(p *mavlink.Packet) mavlink.Message {
		msg := new(Timesync)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_CAMERA_TRIGGER, "MSG_ID_CAMERA_TRIGGER", 174, func(p *mavlink.Packet) mavlink.Message {
		msg := new(CameraTrigger)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_HIL_GPS, "MSG_ID_HIL_GPS", 124, func(p *mavlink.Packet) mavlink.Message {
		msg := new(HilGps)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_HIL_OPTICAL_FLOW, "MSG_ID_HIL_OPTICAL_FLOW", 237, func(p *mavlink.Packet) mavlink.Message {
		msg := new(HilOpticalFlow)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_HIL_STATE_QUATERNION, "MSG_ID_HIL_STATE_QUATERNION", 4, func(p *mavlink.Packet) mavlink.Message {
		msg := new(HilStateQuaternion)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_SCALED_IMU2, "MSG_ID_SCALED_IMU2", 76, func(p *mavlink.Packet) mavlink.Message {
		msg := new(ScaledImu2)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_LOG_REQUEST_LIST, "MSG_ID_LOG_REQUEST_LIST", 128, func(p *mavlink.Packet) mavlink.Message {
		msg := new(LogRequestList)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_LOG_ENTRY, "MSG_ID_LOG_ENTRY", 56, func(p *mavlink.Packet) mavlink.Message {
		msg := new(LogEntry)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_LOG_REQUEST_DATA, "MSG_ID_LOG_REQUEST_DATA", 116, func(p *mavlink.Packet) mavlink.Message {
		msg := new(LogRequestData)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_LOG_DATA, "MSG_ID_LOG_DATA", 134, func(p *mavlink.Packet) mavlink.Message {
		msg := new(LogData)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_LOG_ERASE, "MSG_ID_LOG_ERASE", 237, func(p *mavlink.Packet) mavlink.Message {
		msg := new(LogErase)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_LOG_REQUEST_END, "MSG_ID_LOG_REQUEST_END", 203, func(p *mavlink.Packet) mavlink.Message {
		msg := new(LogRequestEnd)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_GPS_INJECT_DATA, "MSG_ID_GPS_INJECT_DATA", 250, func(p *mavlink.Packet) mavlink.Message {
		msg := new(GpsInjectData)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_GPS2_RAW, "MSG_ID_GPS2_RAW", 87, func(p *mavlink.Packet) mavlink.Message {
		msg := new(Gps2Raw)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_POWER_STATUS, "MSG_ID_POWER_STATUS", 203, func(p *mavlink.Packet) mavlink.Message {
		msg := new(PowerStatus)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_SERIAL_CONTROL, "MSG_ID_SERIAL_CONTROL", 220, func(p *mavlink.Packet) mavlink.Message {
		msg := new(SerialControl)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_GPS_RTK, "MSG_ID_GPS_RTK", 25, func(p *mavlink.Packet) mavlink.Message {
		msg := new(GpsRtk)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_GPS2_RTK, "MSG_ID_GPS2_RTK", 226, func(p *mavlink.Packet) mavlink.Message {
		msg := new(Gps2Rtk)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_SCALED_IMU3, "MSG_ID_SCALED_IMU3", 46, func(p *mavlink.Packet) mavlink.Message {
		msg := new(ScaledImu3)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_DATA_TRANSMISSION_HANDSHAKE, "MSG_ID_DATA_TRANSMISSION_HANDSHAKE", 29, func(p *mavlink.Packet) mavlink.Message {
		msg := new(DataTransmissionHandshake)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_ENCAPSULATED_DATA, "MSG_ID_ENCAPSULATED_DATA", 223, func(p *mavlink.Packet) mavlink.Message {
		msg := new(EncapsulatedData)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_DISTANCE_SENSOR, "MSG_ID_DISTANCE_SENSOR", 85, func(p *mavlink.Packet) mavlink.Message {
		msg := new(DistanceSensor)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_TERRAIN_REQUEST, "MSG_ID_TERRAIN_REQUEST", 6, func(p *mavlink.Packet) mavlink.Message {
		msg := new(TerrainRequest)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_TERRAIN_DATA, "MSG_ID_TERRAIN_DATA", 229, func(p *mavlink.Packet) mavlink.Message {
		msg := new(TerrainData)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_TERRAIN_CHECK, "MSG_ID_TERRAIN_CHECK", 203, func(p *mavlink.Packet) mavlink.Message {
		msg := new(TerrainCheck)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_TERRAIN_REPORT, "MSG_ID_TERRAIN_REPORT", 1, func(p *mavlink.Packet) mavlink.Message {
		msg := new(TerrainReport)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_SCALED_PRESSURE2, "MSG_ID_SCALED_PRESSURE2", 195, func(p *mavlink.Packet) mavlink.Message {
		msg := new(ScaledPressure2)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_ATT_POS_MOCAP, "MSG_ID_ATT_POS_MOCAP", 109, func(p *mavlink.Packet) mavlink.Message {
		msg := new(AttPosMocap)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_SET_ACTUATOR_CONTROL_TARGET, "MSG_ID_SET_ACTUATOR_CONTROL_TARGET", 168, func(p *mavlink.Packet) mavlink.Message {
		msg := new(SetActuatorControlTarget)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_ACTUATOR_CONTROL_TARGET, "MSG_ID_ACTUATOR_CONTROL_TARGET", 181, func(p *mavlink.Packet) mavlink.Message {
		msg := new(ActuatorControlTarget)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_ALTITUDE, "MSG_ID_ALTITUDE", 47, func(p *mavlink.Packet) mavlink.Message {
		msg := new(Altitude)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_RESOURCE_REQUEST, "MSG_ID_RESOURCE_REQUEST", 72, func(p *mavlink.Packet) mavlink.Message {
		msg := new(ResourceRequest)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_SCALED_PRESSURE3, "MSG_ID_SCALED_PRESSURE3", 131, func(p *mavlink.Packet) mavlink.Message {
		msg := new(ScaledPressure3)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_FOLLOW_TARGET, "MSG_ID_FOLLOW_TARGET", 127, func(p *mavlink.Packet) mavlink.Message {
		msg := new(FollowTarget)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_CONTROL_SYSTEM_STATE, "MSG_ID_CONTROL_SYSTEM_STATE", 103, func(p *mavlink.Packet) mavlink.Message {
		msg := new(ControlSystemState)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_BATTERY_STATUS, "MSG_ID_BATTERY_STATUS", 154, func(p *mavlink.Packet) mavlink.Message {
		msg := new(BatteryStatus)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_AUTOPILOT_VERSION, "MSG_ID_AUTOPILOT_VERSION", 178, func(p *mavlink.Packet) mavlink.Message {
		msg := new(AutopilotVersion)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_LANDING_TARGET, "MSG_ID_LANDING_TARGET", 200, func(p *mavlink.Packet) mavlink.Message {
		msg := new(LandingTarget)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_FENCE_STATUS, "MSG_ID_FENCE_STATUS", 189, func(p *mavlink.Packet) mavlink.Message {
		msg := new(FenceStatus)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_MAG_CAL_REPORT, "MSG_ID_MAG_CAL_REPORT", 36, func(p *mavlink.Packet) mavlink.Message {
		msg := new(MagCalReport)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_EFI_STATUS, "MSG_ID_EFI_STATUS", 208, func(p *mavlink.Packet) mavlink.Message {
		msg := new(EfiStatus)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_ESTIMATOR_STATUS, "MSG_ID_ESTIMATOR_STATUS", 163, func(p *mavlink.Packet) mavlink.Message {
		msg := new(EstimatorStatus)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_WIND_COV, "MSG_ID_WIND_COV", 105, func(p *mavlink.Packet) mavlink.Message {
		msg := new(WindCov)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_GPS_INPUT, "MSG_ID_GPS_INPUT", 151, func(p *mavlink.Packet) mavlink.Message {
		msg := new(GpsInput)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_GPS_RTCM_DATA, "MSG_ID_GPS_RTCM_DATA", 35, func(p *mavlink.Packet) mavlink.Message {
		msg := new(GpsRtcmData)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_HIGH_LATENCY, "MSG_ID_HIGH_LATENCY", 150, func(p *mavlink.Packet) mavlink.Message {
		msg := new(HighLatency)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_HIGH_LATENCY2, "MSG_ID_HIGH_LATENCY2", 179, func(p *mavlink.Packet) mavlink.Message {
		msg := new(HighLatency2)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_VIBRATION, "MSG_ID_VIBRATION", 90, func(p *mavlink.Packet) mavlink.Message {
		msg := new(Vibration)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_HOME_POSITION, "MSG_ID_HOME_POSITION", 104, func(p *mavlink.Packet) mavlink.Message {
		msg := new(HomePosition)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_SET_HOME_POSITION, "MSG_ID_SET_HOME_POSITION", 85, func(p *mavlink.Packet) mavlink.Message {
		msg := new(SetHomePosition)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_MESSAGE_INTERVAL, "MSG_ID_MESSAGE_INTERVAL", 95, func(p *mavlink.Packet) mavlink.Message {
		msg := new(MessageInterval)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_EXTENDED_SYS_STATE, "MSG_ID_EXTENDED_SYS_STATE", 130, func(p *mavlink.Packet) mavlink.Message {
		msg := new(ExtendedSysState)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_ADSB_VEHICLE, "MSG_ID_ADSB_VEHICLE", 184, func(p *mavlink.Packet) mavlink.Message {
		msg := new(AdsbVehicle)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_COLLISION, "MSG_ID_COLLISION", 81, func(p *mavlink.Packet) mavlink.Message {
		msg := new(Collision)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_V2_EXTENSION, "MSG_ID_V2_EXTENSION", 8, func(p *mavlink.Packet) mavlink.Message {
		msg := new(V2Extension)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_MEMORY_VECT, "MSG_ID_MEMORY_VECT", 204, func(p *mavlink.Packet) mavlink.Message {
		msg := new(MemoryVect)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_DEBUG_VECT, "MSG_ID_DEBUG_VECT", 49, func(p *mavlink.Packet) mavlink.Message {
		msg := new(DebugVect)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_NAMED_VALUE_FLOAT, "MSG_ID_NAMED_VALUE_FLOAT", 170, func(p *mavlink.Packet) mavlink.Message {
		msg := new(NamedValueFloat)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_NAMED_VALUE_INT, "MSG_ID_NAMED_VALUE_INT", 44, func(p *mavlink.Packet) mavlink.Message {
		msg := new(NamedValueInt)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_STATUSTEXT, "MSG_ID_STATUSTEXT", 83, func(p *mavlink.Packet) mavlink.Message {
		msg := new(Statustext)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_DEBUG, "MSG_ID_DEBUG", 46, func(p *mavlink.Packet) mavlink.Message {
		msg := new(Debug)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_HEARTBEAT, "MSG_ID_HEARTBEAT", 50, func(p *mavlink.Packet) mavlink.Message {
		msg := new(Heartbeat)
		msg.Unpack(p)
		return msg
	})
}

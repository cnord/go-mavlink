//////////////////////////////////////////////////
//
// NOTE: do not edit,
// this file created automatically by mavgen.go
//
//////////////////////////////////////////////////

package common

import (
	mavlink ".."
	"encoding/binary"
	"fmt"
	"math"
)

// FirmwareVersionType (generated enum)
// These values define the type of firmware release.  These values indicate the first version or release of this type.  For example the first alpha release would be 64, the second would be 65.
const (
	FIRMWARE_VERSION_TYPE_DEV      = 0   // development release
	FIRMWARE_VERSION_TYPE_ALPHA    = 64  // alpha release
	FIRMWARE_VERSION_TYPE_BETA     = 128 // beta release
	FIRMWARE_VERSION_TYPE_RC       = 192 // release candidate
	FIRMWARE_VERSION_TYPE_OFFICIAL = 255 // official stable release
)

// HlFailureFlag (generated enum)
// Flags to report failure cases over the high latency telemtry.
const (
	HL_FAILURE_FLAG_GPS                   = 1    // GPS failure
	HL_FAILURE_FLAG_DIFFERENTIAL_PRESSURE = 2    // Differential pressure sensor failure
	HL_FAILURE_FLAG_ABSOLUTE_PRESSURE     = 4    // Absolute pressure sensor failure
	HL_FAILURE_FLAG_3D_ACCEL              = 8    // Accelerometer sensor failure
	HL_FAILURE_FLAG_3D_GYRO               = 16   // Gyroscope sensor failure
	HL_FAILURE_FLAG_3D_MAG                = 32   // Magnetometer sensor failure
	HL_FAILURE_FLAG_TERRAIN               = 64   // Terrain subsystem failure
	HL_FAILURE_FLAG_BATTERY               = 128  // Battery failure/critical low battery
	HL_FAILURE_FLAG_RC_RECEIVER           = 256  // RC receiver failure/no rc connection
	HL_FAILURE_FLAG_OFFBOARD_LINK         = 512  // Offboard link failure
	HL_FAILURE_FLAG_ENGINE                = 1024 // Engine failure
	HL_FAILURE_FLAG_GEOFENCE              = 2048 // Geofence violation
	HL_FAILURE_FLAG_ESTIMATOR             = 4096 // Estimator failure, for example measurement rejection or large variances
	HL_FAILURE_FLAG_MISSION               = 8192 // Mission failure
)

// MavGoto (generated enum)
// Actions that may be specified in MAV_CMD_OVERRIDE_GOTO to override mission execution.
const (
	MAV_GOTO_DO_HOLD                    = 0 // Hold at the current position
	MAV_GOTO_DO_CONTINUE                = 1 // Continue with the next item in mission execution
	MAV_GOTO_HOLD_AT_CURRENT_POSITION   = 2 // Hold at the current position of the system
	MAV_GOTO_HOLD_AT_SPECIFIED_POSITION = 3 // Hold at the position specified in the parameters of the DO_HOLD action
)

// MavMode (generated enum)
// These defines are predefined OR-combined mode flags. There is no need to use values from this enum, but it                simplifies the use of the mode flags. Note that manual input is enabled in all modes as a safety override.
const (
	MAV_MODE_PREFLIGHT          = 0   // System is not ready to fly, booting, calibrating, etc. No flag is set
	MAV_MODE_STABILIZE_DISARMED = 80  // System is allowed to be active, under assisted RC control
	MAV_MODE_STABILIZE_ARMED    = 208 // System is allowed to be active, under assisted RC control
	MAV_MODE_MANUAL_DISARMED    = 64  // System is allowed to be active, under manual (RC) control, no stabilization
	MAV_MODE_MANUAL_ARMED       = 192 // System is allowed to be active, under manual (RC) control, no stabilization
	MAV_MODE_GUIDED_DISARMED    = 88  // System is allowed to be active, under autonomous control, manual setpoint
	MAV_MODE_GUIDED_ARMED       = 216 // System is allowed to be active, under autonomous control, manual setpoint
	MAV_MODE_AUTO_DISARMED      = 92  // System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard and not pre-programmed by waypoints)
	MAV_MODE_AUTO_ARMED         = 220 // System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard and not pre-programmed by waypoints)
	MAV_MODE_TEST_DISARMED      = 66  // UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only
	MAV_MODE_TEST_ARMED         = 194 // UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only
)

// MavSysStatusSensor (generated enum)
// These encode the sensors whose status is sent as part of the SYS_STATUS message.
const (
	MAV_SYS_STATUS_SENSOR_3D_GYRO                = 1         // 0x01 3D gyro
	MAV_SYS_STATUS_SENSOR_3D_ACCEL               = 2         // 0x02 3D accelerometer
	MAV_SYS_STATUS_SENSOR_3D_MAG                 = 4         // 0x04 3D magnetometer
	MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE      = 8         // 0x08 absolute pressure
	MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE  = 16        // 0x10 differential pressure
	MAV_SYS_STATUS_SENSOR_GPS                    = 32        // 0x20 GPS
	MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW           = 64        // 0x40 optical flow
	MAV_SYS_STATUS_SENSOR_VISION_POSITION        = 128       // 0x80 computer vision position
	MAV_SYS_STATUS_SENSOR_LASER_POSITION         = 256       // 0x100 laser based position
	MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH  = 512       // 0x200 external ground truth (Vicon or Leica)
	MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL   = 1024      // 0x400 3D angular rate control
	MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION = 2048      // 0x800 attitude stabilization
	MAV_SYS_STATUS_SENSOR_YAW_POSITION           = 4096      // 0x1000 yaw position
	MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL     = 8192      // 0x2000 z/altitude control
	MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL    = 16384     // 0x4000 x/y position control
	MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS          = 32768     // 0x8000 motor outputs / control
	MAV_SYS_STATUS_SENSOR_RC_RECEIVER            = 65536     // 0x10000 rc receiver
	MAV_SYS_STATUS_SENSOR_3D_GYRO2               = 131072    // 0x20000 2nd 3D gyro
	MAV_SYS_STATUS_SENSOR_3D_ACCEL2              = 262144    // 0x40000 2nd 3D accelerometer
	MAV_SYS_STATUS_SENSOR_3D_MAG2                = 524288    // 0x80000 2nd 3D magnetometer
	MAV_SYS_STATUS_GEOFENCE                      = 1048576   // 0x100000 geofence
	MAV_SYS_STATUS_AHRS                          = 2097152   // 0x200000 AHRS subsystem health
	MAV_SYS_STATUS_TERRAIN                       = 4194304   // 0x400000 Terrain subsystem health
	MAV_SYS_STATUS_REVERSE_MOTOR                 = 8388608   // 0x800000 Motors are reversed
	MAV_SYS_STATUS_LOGGING                       = 16777216  // 0x1000000 Logging
	MAV_SYS_STATUS_SENSOR_BATTERY                = 33554432  // 0x2000000 Battery
	MAV_SYS_STATUS_SENSOR_PROXIMITY              = 67108864  // 0x4000000 Proximity
	MAV_SYS_STATUS_SENSOR_SATCOM                 = 134217728 // 0x8000000 Satellite Communication
	MAV_SYS_STATUS_PREARM_CHECK                  = 268435456 // 0x10000000 pre-arm check status. Always healthy when armed
	MAV_SYS_STATUS_OBSTACLE_AVOIDANCE            = 536870912 // 0x20000000 Avoidance/collision prevention
)

// MavFrame (generated enum)
//
const (
	MAV_FRAME_GLOBAL                  = 0  // Global (WGS84) coordinate frame + MSL altitude. First value / x: latitude, second value / y: longitude, third value / z: positive altitude over mean sea level (MSL)
	MAV_FRAME_LOCAL_NED               = 1  // Local coordinate frame, Z-down (x: North, y: East, z: Down)
	MAV_FRAME_MISSION                 = 2  // NOT a coordinate frame, indicates a mission command
	MAV_FRAME_GLOBAL_RELATIVE_ALT     = 3  // Global (WGS84) coordinate frame + altitude relative to the home position. First value / x: latitude, second value / y: longitude, third value / z: positive altitude with 0 being at the altitude of the home location
	MAV_FRAME_LOCAL_ENU               = 4  // Local coordinate frame, Z-up (x: East, y: North, z: Up)
	MAV_FRAME_GLOBAL_INT              = 5  // Global (WGS84) coordinate frame (scaled) + MSL altitude. First value / x: latitude in degrees*1.0e-7, second value / y: longitude in degrees*1.0e-7, third value / z: positive altitude over mean sea level (MSL)
	MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6  // Global (WGS84) coordinate frame (scaled) + altitude relative to the home position. First value / x: latitude in degrees*10e-7, second value / y: longitude in degrees*10e-7, third value / z: positive altitude with 0 being at the altitude of the home location
	MAV_FRAME_LOCAL_OFFSET_NED        = 7  // Offset to the current local frame. Anything expressed in this frame should be added to the current local frame position
	MAV_FRAME_BODY_NED                = 8  // Setpoint in body NED frame. This makes sense if all position control is externalized - e.g. useful to command 2 m/s^2 acceleration to the right
	MAV_FRAME_BODY_OFFSET_NED         = 9  // Offset in body NED frame. This makes sense if adding setpoints to the current flight path, to avoid an obstacle - e.g. useful to command 2 m/s^2 acceleration to the east
	MAV_FRAME_GLOBAL_TERRAIN_ALT      = 10 // Global (WGS84) coordinate frame with AGL altitude (at the waypoint coordinate). First value / x: latitude in degrees, second value / y: longitude in degrees, third value / z: positive altitude in meters with 0 being at ground level in terrain model
	MAV_FRAME_GLOBAL_TERRAIN_ALT_INT  = 11 // Global (WGS84) coordinate frame (scaled) with AGL altitude (at the waypoint coordinate). First value / x: latitude in degrees*10e-7, second value / y: longitude in degrees*10e-7, third value / z: positive altitude in meters with 0 being at ground level in terrain model
	MAV_FRAME_BODY_FRD                = 12 // Body fixed frame of reference, Z-down (x: Forward, y: Right, z: Down)
	MAV_FRAME_RESERVED_13             = 13 // MAV_FRAME_BODY_FLU - Body fixed frame of reference, Z-up (x: Forward, y: Left, z: Up)
	MAV_FRAME_RESERVED_14             = 14 // MAV_FRAME_MOCAP_NED - Odometry local coordinate frame of data given by a motion capture system, Z-down (x: North, y: East, z: Down)
	MAV_FRAME_RESERVED_15             = 15 // MAV_FRAME_MOCAP_ENU - Odometry local coordinate frame of data given by a motion capture system, Z-up (x: East, y: North, z: Up)
	MAV_FRAME_RESERVED_16             = 16 // MAV_FRAME_VISION_NED - Odometry local coordinate frame of data given by a vision estimation system, Z-down (x: North, y: East, z: Down)
	MAV_FRAME_RESERVED_17             = 17 // MAV_FRAME_VISION_ENU - Odometry local coordinate frame of data given by a vision estimation system, Z-up (x: East, y: North, z: Up)
	MAV_FRAME_RESERVED_18             = 18 // MAV_FRAME_ESTIM_NED - Odometry local coordinate frame of data given by an estimator running onboard the vehicle, Z-down (x: North, y: East, z: Down)
	MAV_FRAME_RESERVED_19             = 19 // MAV_FRAME_ESTIM_ENU - Odometry local coordinate frame of data given by an estimator running onboard the vehicle, Z-up (x: East, y: North, z: Up)
	MAV_FRAME_LOCAL_FRD               = 20 // Forward, Right, Down coordinate frame. This is a local frame with Z-down and arbitrary F/R alignment (i.e. not aligned with NED/earth frame)
	MAV_FRAME_LOCAL_FLU               = 21 // Forward, Left, Up coordinate frame. This is a local frame with Z-up and arbitrary F/L alignment (i.e. not aligned with ENU/earth frame)
)

// MavlinkDataStreamType (generated enum)
//
const (
	MAVLINK_DATA_STREAM_IMG_JPEG   = 0 //
	MAVLINK_DATA_STREAM_IMG_BMP    = 1 //
	MAVLINK_DATA_STREAM_IMG_RAW8U  = 2 //
	MAVLINK_DATA_STREAM_IMG_RAW32U = 3 //
	MAVLINK_DATA_STREAM_IMG_PGM    = 4 //
	MAVLINK_DATA_STREAM_IMG_PNG    = 5 //
)

// FenceAction (generated enum)
//
const (
	FENCE_ACTION_NONE            = 0 // Disable fenced mode
	FENCE_ACTION_GUIDED          = 1 // Switched to guided mode to return point (fence point 0)
	FENCE_ACTION_REPORT          = 2 // Report fence breach, but don't take action
	FENCE_ACTION_GUIDED_THR_PASS = 3 // Switched to guided mode to return point (fence point 0) with manual throttle control
	FENCE_ACTION_RTL             = 4 // Switch to RTL (return to launch) mode and head for the return point
)

// FenceBreach (generated enum)
//
const (
	FENCE_BREACH_NONE     = 0 // No last fence breach
	FENCE_BREACH_MINALT   = 1 // Breached minimum altitude
	FENCE_BREACH_MAXALT   = 2 // Breached maximum altitude
	FENCE_BREACH_BOUNDARY = 3 // Breached fence boundary
)

// FenceMitigate (generated enum)
// Actions being taken to mitigate/prevent fence breach
const (
	FENCE_MITIGATE_UNKNOWN   = 0 // Unknown
	FENCE_MITIGATE_NONE      = 1 // No actions being taken
	FENCE_MITIGATE_VEL_LIMIT = 2 // Velocity limiting active to prevent breach
)

// MavMountMode (generated enum)
// Enumeration of possible mount operation modes. This message is used by obsolete/deprecated gimbal messages.
const (
	MAV_MOUNT_MODE_RETRACT           = 0 // Load and keep safe position (Roll,Pitch,Yaw) from permant memory and stop stabilization
	MAV_MOUNT_MODE_NEUTRAL           = 1 // Load and keep neutral position (Roll,Pitch,Yaw) from permanent memory
	MAV_MOUNT_MODE_MAVLINK_TARGETING = 2 // Load neutral position and start MAVLink Roll,Pitch,Yaw control with stabilization
	MAV_MOUNT_MODE_RC_TARGETING      = 3 // Load neutral position and start RC Roll,Pitch,Yaw control with stabilization
	MAV_MOUNT_MODE_GPS_POINT         = 4 // Load neutral position and start to point to Lat,Lon,Alt
	MAV_MOUNT_MODE_SYSID_TARGET      = 5 // Gimbal tracks system with specified system ID
	MAV_MOUNT_MODE_HOME_LOCATION     = 6 // Gimbal tracks home location
)

// GimbalDeviceCapFlags (generated enum)
// Gimbal device (low level) capability flags (bitmap)
const (
	GIMBAL_DEVICE_CAP_FLAGS_HAS_RETRACT           = 1    // Gimbal device supports a retracted position
	GIMBAL_DEVICE_CAP_FLAGS_HAS_NEUTRAL           = 2    // Gimbal device supports a horizontal, forward looking position, stabilized
	GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_AXIS         = 4    // Gimbal device supports rotating around roll axis
	GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_FOLLOW       = 8    // Gimbal device supports to follow a roll angle relative to the vehicle
	GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_LOCK         = 16   // Gimbal device supports locking to an roll angle (generally that's the default with roll stabilized)
	GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS        = 32   // Gimbal device supports rotating around pitch axis
	GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_FOLLOW      = 64   // Gimbal device supports to follow a pitch angle relative to the vehicle
	GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_LOCK        = 128  // Gimbal device supports locking to an pitch angle (generally that's the default with pitch stabilized)
	GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_AXIS          = 256  // Gimbal device supports rotating around yaw axis
	GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_FOLLOW        = 512  // Gimbal device supports to follow a yaw angle relative to the vehicle (generally that's the default)
	GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_LOCK          = 1024 // Gimbal device supports locking to an absolute heading (often this is an option available)
	GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_INFINITE_YAW = 2048 // Gimbal device supports yawing/panning infinetely (e.g. using slip disk)
)

// GimbalManagerCapFlags (generated enum)
// Gimbal manager high level capability flags (bitmap). The first 16 bits are identical to the GIMBAL_DEVICE_CAP_FLAGS which are identical with GIMBAL_DEVICE_FLAGS. However, the gimbal manager does not need to copy the flags from the gimbal but can also enhance the capabilities and thus add flags.
const (
	GIMBAL_MANAGER_CAP_FLAGS_HAS_RETRACT               = 1      // Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_RETRACT
	GIMBAL_MANAGER_CAP_FLAGS_HAS_NEUTRAL               = 2      // Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_NEUTRAL
	GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_AXIS             = 4      // Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_AXIS
	GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_FOLLOW           = 8      // Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_FOLLOW
	GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_LOCK             = 16     // Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_LOCK
	GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_AXIS            = 32     // Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS
	GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_FOLLOW          = 64     // Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_FOLLOW
	GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_LOCK            = 128    // Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_LOCK
	GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_AXIS              = 256    // Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_AXIS
	GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_FOLLOW            = 512    // Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_FOLLOW
	GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_LOCK              = 1024   // Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_LOCK
	GIMBAL_MANAGER_CAP_FLAGS_SUPPORTS_INFINITE_YAW     = 2048   // Based on GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_INFINITE_YAW
	GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_LOCAL  = 65536  // Gimbal manager supports to point to a local position
	GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_GLOBAL = 131072 // Gimbal manager supports to point to a global latitude, longitude, altitude position
)

// GimbalDeviceFlags (generated enum)
// Flags for gimbal device (lower level) operation.
const (
	GIMBAL_DEVICE_FLAGS_RETRACT    = 1  // Set to retracted safe position (no stabilization), takes presedence over all other flags
	GIMBAL_DEVICE_FLAGS_NEUTRAL    = 2  // Set to neutral position (horizontal, forward looking, with stabiliziation), takes presedence over all other flags except RETRACT
	GIMBAL_DEVICE_FLAGS_ROLL_LOCK  = 4  // Lock roll angle to absolute angle relative to horizon (not relative to drone). This is generally the default with a stabilizing gimbal
	GIMBAL_DEVICE_FLAGS_PITCH_LOCK = 8  // Lock pitch angle to absolute angle relative to horizon (not relative to drone). This is generally the default
	GIMBAL_DEVICE_FLAGS_YAW_LOCK   = 16 // Lock yaw angle to absolute angle relative to North (not relative to drone). If this flag is set, the quaternion is in the Earth frame with the x-axis pointing North (yaw absolute). If this flag is not set, the quaternion frame is in the Earth frame rotated so that the x-axis is pointing forward (yaw relative to vehicle)
)

// GimbalManagerFlags (generated enum)
// Flags for high level gimbal manager operation The first 16 bytes are identical to the GIMBAL_DEVICE_FLAGS.
const (
	GIMBAL_MANAGER_FLAGS_RETRACT    = 1  // Based on GIMBAL_DEVICE_FLAGS_RETRACT
	GIMBAL_MANAGER_FLAGS_NEUTRAL    = 2  // Based on GIMBAL_DEVICE_FLAGS_NEUTRAL
	GIMBAL_MANAGER_FLAGS_ROLL_LOCK  = 4  // Based on GIMBAL_DEVICE_FLAGS_ROLL_LOCK
	GIMBAL_MANAGER_FLAGS_PITCH_LOCK = 8  // Based on GIMBAL_DEVICE_FLAGS_PITCH_LOCK
	GIMBAL_MANAGER_FLAGS_YAW_LOCK   = 16 // Based on GIMBAL_DEVICE_FLAGS_YAW_LOCK
)

// GimbalDeviceErrorFlags (generated enum)
// Gimbal device (low level) error flags (bitmap, 0 means no error)
const (
	GIMBAL_DEVICE_ERROR_FLAGS_AT_ROLL_LIMIT       = 1   // Gimbal device is limited by hardware roll limit
	GIMBAL_DEVICE_ERROR_FLAGS_AT_PITCH_LIMIT      = 2   // Gimbal device is limited by hardware pitch limit
	GIMBAL_DEVICE_ERROR_FLAGS_AT_YAW_LIMIT        = 4   // Gimbal device is limited by hardware yaw limit
	GIMBAL_DEVICE_ERROR_FLAGS_ENCODER_ERROR       = 8   // There is an error with the gimbal encoders
	GIMBAL_DEVICE_ERROR_FLAGS_POWER_ERROR         = 16  // There is an error with the gimbal power source
	GIMBAL_DEVICE_ERROR_FLAGS_MOTOR_ERROR         = 32  // There is an error with the gimbal motor's
	GIMBAL_DEVICE_ERROR_FLAGS_SOFTWARE_ERROR      = 64  // There is an error with the gimbal's software
	GIMBAL_DEVICE_ERROR_FLAGS_COMMS_ERROR         = 128 // There is an error with the gimbal's communication
	GIMBAL_DEVICE_ERROR_FLAGS_CALIBRATION_RUNNING = 256 // Gimbal is currently calibrating
)

// GripperActions (generated enum)
// Gripper actions.
const (
	GRIPPER_ACTION_RELEASE = 0 // Gripper release cargo
	GRIPPER_ACTION_GRAB    = 1 // Gripper grab onto cargo
)

// WinchActions (generated enum)
// Winch actions.
const (
	WINCH_RELAXED                 = 0 // Relax winch
	WINCH_RELATIVE_LENGTH_CONTROL = 1 // Wind or unwind specified length of cable, optionally using specified rate
	WINCH_RATE_CONTROL            = 2 // Wind or unwind cable at specified rate
)

// UavcanNodeHealth (generated enum)
// Generalized UAVCAN node health
const (
	UAVCAN_NODE_HEALTH_OK       = 0 // The node is functioning properly
	UAVCAN_NODE_HEALTH_WARNING  = 1 // A critical parameter went out of range or the node has encountered a minor failure
	UAVCAN_NODE_HEALTH_ERROR    = 2 // The node has encountered a major failure
	UAVCAN_NODE_HEALTH_CRITICAL = 3 // The node has suffered a fatal malfunction
)

// UavcanNodeMode (generated enum)
// Generalized UAVCAN node mode
const (
	UAVCAN_NODE_MODE_OPERATIONAL     = 0 // The node is performing its primary functions
	UAVCAN_NODE_MODE_INITIALIZATION  = 1 // The node is initializing; this mode is entered immediately after startup
	UAVCAN_NODE_MODE_MAINTENANCE     = 2 // The node is under maintenance
	UAVCAN_NODE_MODE_SOFTWARE_UPDATE = 3 // The node is in the process of updating its software
	UAVCAN_NODE_MODE_OFFLINE         = 7 // The node is no longer available online
)

// EscConnectionType (generated enum)
// Indicates the ESC connection type.
const (
	ESC_CONNECTION_TYPE_PPM     = 0 // Traditional PPM ESC
	ESC_CONNECTION_TYPE_SERIAL  = 1 // Serial Bus connected ESC
	ESC_CONNECTION_TYPE_ONESHOT = 2 // One Shot PPM ESC
	ESC_CONNECTION_TYPE_I2C     = 3 // I2C ESC
	ESC_CONNECTION_TYPE_CAN     = 4 // CAN-Bus ESC
	ESC_CONNECTION_TYPE_DSHOT   = 5 // DShot ESC
)

// EscFailureFlags (generated enum)
// Flags to report ESC failures.
const (
	ESC_FAILURE_NONE             = 0  // No ESC failure
	ESC_FAILURE_OVER_CURRENT     = 1  // Over current failure
	ESC_FAILURE_OVER_VOLTAGE     = 2  // Over voltage failure
	ESC_FAILURE_OVER_TEMPERATURE = 4  // Over temperature failure
	ESC_FAILURE_OVER_RPM         = 8  // Over RPM failure
	ESC_FAILURE_INCONSISTENT_CMD = 16 // Inconsistent command failure i.e. out of bounds
	ESC_FAILURE_MOTOR_STUCK      = 32 // Motor stuck failure
	ESC_FAILURE_GENERIC          = 64 // Generic ESC failure
)

// StorageStatus (generated enum)
// Flags to indicate the status of camera storage.
const (
	STORAGE_STATUS_EMPTY         = 0 // Storage is missing (no microSD card loaded for example.)
	STORAGE_STATUS_UNFORMATTED   = 1 // Storage present but unformatted
	STORAGE_STATUS_READY         = 2 // Storage present and ready
	STORAGE_STATUS_NOT_SUPPORTED = 3 // Camera does not supply storage status information. Capacity information in STORAGE_INFORMATION fields will be ignored
)

// StorageType (generated enum)
// Flags to indicate the type of storage.
const (
	STORAGE_TYPE_UNKNOWN   = 0   // Storage type is not known
	STORAGE_TYPE_USB_STICK = 1   // Storage type is USB device
	STORAGE_TYPE_SD        = 2   // Storage type is SD card
	STORAGE_TYPE_MICROSD   = 3   // Storage type is microSD card
	STORAGE_TYPE_CF        = 4   // Storage type is CFast
	STORAGE_TYPE_CFE       = 5   // Storage type is CFexpress
	STORAGE_TYPE_XQD       = 6   // Storage type is XQD
	STORAGE_TYPE_HD        = 7   // Storage type is HD mass storage type
	STORAGE_TYPE_OTHER     = 254 // Storage type is other, not listed type
)

// OrbitYawBehaviour (generated enum)
// Yaw behaviour during orbit flight.
const (
	ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TO_CIRCLE_CENTER  = 0 // Vehicle front points to the center (default)
	ORBIT_YAW_BEHAVIOUR_HOLD_INITIAL_HEADING         = 1 // Vehicle front holds heading when message received
	ORBIT_YAW_BEHAVIOUR_UNCONTROLLED                 = 2 // Yaw uncontrolled
	ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TANGENT_TO_CIRCLE = 3 // Vehicle front follows flight path (tangential to circle)
	ORBIT_YAW_BEHAVIOUR_RC_CONTROLLED                = 4 // Yaw controlled by RC input
)

// WifiConfigApResponse (generated enum)
// Possible responses from a WIFI_CONFIG_AP message.
const (
	WIFI_CONFIG_AP_RESPONSE_UNDEFINED      = 0 // Undefined response. Likely an indicative of a system that doesn't support this request
	WIFI_CONFIG_AP_RESPONSE_ACCEPTED       = 1 // Changes accepted
	WIFI_CONFIG_AP_RESPONSE_REJECTED       = 2 // Changes rejected
	WIFI_CONFIG_AP_RESPONSE_MODE_ERROR     = 3 // Invalid Mode
	WIFI_CONFIG_AP_RESPONSE_SSID_ERROR     = 4 // Invalid SSID
	WIFI_CONFIG_AP_RESPONSE_PASSWORD_ERROR = 5 // Invalid Password
)

// CellularConfigResponse (generated enum)
// Possible responses from a CELLULAR_CONFIG message.
const (
	CELLULAR_CONFIG_RESPONSE_ACCEPTED    = 0 // Changes accepted
	CELLULAR_CONFIG_RESPONSE_APN_ERROR   = 1 // Invalid APN
	CELLULAR_CONFIG_RESPONSE_PIN_ERROR   = 2 // Invalid PIN
	CELLULAR_CONFIG_RESPONSE_REJECTED    = 3 // Changes rejected
	CELLULAR_CONFIG_BLOCKED_PUK_REQUIRED = 4 // PUK is required to unblock SIM card
)

// WifiConfigApMode (generated enum)
// WiFi Mode.
const (
	WIFI_CONFIG_AP_MODE_UNDEFINED = 0 // WiFi mode is undefined
	WIFI_CONFIG_AP_MODE_AP        = 1 // WiFi configured as an access point
	WIFI_CONFIG_AP_MODE_STATION   = 2 // WiFi configured as a station connected to an existing local WiFi network
	WIFI_CONFIG_AP_MODE_DISABLED  = 3 // WiFi disabled
)

// CompMetadataType (generated enum)
// Possible values for COMPONENT_INFORMATION.comp_metadata_type.
const (
	COMP_METADATA_TYPE_VERSION   = 0 // Version information which also includes information on other optional supported COMP_METADATA_TYPE's. Must be supported. Only downloadable from vehicle
	COMP_METADATA_TYPE_PARAMETER = 1 // Parameter meta data
	COMP_METADATA_TYPE_COMMANDS  = 2 // Meta data which specifies the commands the vehicle supports. (WIP)
)

// ParamTransactionTransport (generated enum)
// Possible transport layers to set and get parameters via mavlink during a parameter transaction.
const (
	PARAM_TRANSACTION_TRANSPORT_PARAM     = 0 // Transaction over param transport
	PARAM_TRANSACTION_TRANSPORT_PARAM_EXT = 1 // Transaction over param_ext transport
)

// ParamTransactionAction (generated enum)
// Possible parameter transaction actions.
const (
	PARAM_TRANSACTION_ACTION_START  = 0 // Commit the current parameter transaction
	PARAM_TRANSACTION_ACTION_COMMIT = 1 // Commit the current parameter transaction
	PARAM_TRANSACTION_ACTION_CANCEL = 2 // Cancel the current parameter transaction
)

// MavCmd (generated enum)
// Commands to be executed by the MAV. They can be executed on user request, or as part of a mission script. If the action is used in a mission, the parameter mapping to the waypoint/mission message is as follows: Param 1, Param 2, Param 3, Param 4, X: Param 5, Y:Param 6, Z:Param 7. This command list is similar what ARINC 424 is for commercial aircraft: A data format how to interpret waypoint/mission data. NaN and INT32_MAX may be used in float/integer params (respectively) to indicate optional/default values (e.g. to use the component's current yaw or latitude rather than a specific value). See https://mavlink.io/en/guide/xml_schema.html#MAV_CMD for information about the structure of the MAV_CMD entries
const (
	MAV_CMD_NAV_WAYPOINT                       = 16    // Navigate to waypoint. Params: 1) Hold time. (ignored by fixed wing, time to stay at waypoint for rotary wing); 2) Acceptance radius (if the sphere with this radius is hit, the waypoint counts as reached); 3) 0 to pass through the WP, if &gt; 0 radius to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.; 4) Desired yaw angle at waypoint (rotary wing). NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).; 5) Latitude; 6) Longitude; 7) Altitude;
	MAV_CMD_NAV_LOITER_UNLIM                   = 17    // Loiter around this waypoint an unlimited amount of time. Params: 1) Empty; 2) Empty; 3) Loiter radius around waypoint for forward-only moving vehicles (not multicopters). If positive loiter clockwise, else counter-clockwise; 4) Desired yaw angle. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).; 5) Latitude; 6) Longitude; 7) Altitude;
	MAV_CMD_NAV_LOITER_TURNS                   = 18    // Loiter around this waypoint for X turns. Params: 1) Number of turns.; 2) Leave loiter circle only once heading towards the next waypoint (0 = False); 3) Loiter radius around waypoint for forward-only moving vehicles (not multicopters). If positive loiter clockwise, else counter-clockwise; 4) Loiter circle exit location and/or path to next waypoint ("xtrack") for forward-only moving vehicles (not multicopters). 0 for the vehicle to converge towards the center xtrack when it leaves the loiter (the line between the centers of the current and next waypoint), 1 to converge to the direct line between the location that the vehicle exits the loiter radius and the next waypoint. Otherwise the angle (in degrees) between the tangent of the loiter circle and the center xtrack at which the vehicle must leave the loiter (and converge to the center xtrack). NaN to use the current system default xtrack behaviour.; 5) Latitude; 6) Longitude; 7) Altitude;
	MAV_CMD_NAV_LOITER_TIME                    = 19    // Loiter at the specified latitude, longitude and altitude for a certain amount of time. Multicopter vehicles stop at the point (within a vehicle-specific acceptance radius). Forward-only moving vehicles (e.g. fixed-wing) circle the point with the specified radius/direction. If the Heading Required parameter (2) is non-zero forward moving aircraft will only leave the loiter circle once heading towards the next waypoint. Params: 1) Loiter time (only starts once Lat, Lon and Alt is reached).; 2) Leave loiter circle only once heading towards the next waypoint (0 = False); 3) Loiter radius around waypoint for forward-only moving vehicles (not multicopters). If positive loiter clockwise, else counter-clockwise.; 4) Loiter circle exit location and/or path to next waypoint ("xtrack") for forward-only moving vehicles (not multicopters). 0 for the vehicle to converge towards the center xtrack when it leaves the loiter (the line between the centers of the current and next waypoint), 1 to converge to the direct line between the location that the vehicle exits the loiter radius and the next waypoint. Otherwise the angle (in degrees) between the tangent of the loiter circle and the center xtrack at which the vehicle must leave the loiter (and converge to the center xtrack). NaN to use the current system default xtrack behaviour.; 5) Latitude; 6) Longitude; 7) Altitude;
	MAV_CMD_NAV_RETURN_TO_LAUNCH               = 20    // Return to launch location. Params: 1) Empty; 2) Empty; 3) Empty; 4) Empty; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_NAV_LAND                           = 21    // Land at location. Params: 1) Minimum target altitude if landing is aborted (0 = undefined/use system default).; 2) Precision land mode.; 3) Empty; 4) Desired yaw angle. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).; 5) Latitude.; 6) Longitude.; 7) Landing altitude (ground level in current frame).;
	MAV_CMD_NAV_TAKEOFF                        = 22    // Takeoff from ground / hand. Vehicles that support multiple takeoff modes (e.g. VTOL quadplane) should take off using the currently configured mode. Params: 1) Minimum pitch (if airspeed sensor present), desired pitch without sensor; 2) Empty; 3) Empty; 4) Yaw angle (if magnetometer present), ignored without magnetometer. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).; 5) Latitude; 6) Longitude; 7) Altitude;
	MAV_CMD_NAV_LAND_LOCAL                     = 23    // Land at local position (local frame only). Params: 1) Landing target number (if available); 2) Maximum accepted offset from desired landing position - computed magnitude from spherical coordinates: d = sqrt(x^2 + y^2 + z^2), which gives the maximum accepted distance between the desired landing position and the position where the vehicle is about to land; 3) Landing descend rate; 4) Desired yaw angle; 5) Y-axis position; 6) X-axis position; 7) Z-axis / ground level position;
	MAV_CMD_NAV_TAKEOFF_LOCAL                  = 24    // Takeoff from local position (local frame only). Params: 1) Minimum pitch (if airspeed sensor present), desired pitch without sensor; 2) Empty; 3) Takeoff ascend rate; 4) Yaw angle (if magnetometer or another yaw estimation source present), ignored without one of these; 5) Y-axis position; 6) X-axis position; 7) Z-axis position;
	MAV_CMD_NAV_FOLLOW                         = 25    // Vehicle following, i.e. this waypoint represents the position of a moving vehicle. Params: 1) Following logic to use (e.g. loitering or sinusoidal following) - depends on specific autopilot implementation; 2) Ground speed of vehicle to be followed; 3) Radius around waypoint. If positive loiter clockwise, else counter-clockwise; 4) Desired yaw angle.; 5) Latitude; 6) Longitude; 7) Altitude;
	MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT        = 30    // Continue on the current course and climb/descend to specified altitude.  When the altitude is reached continue to the next command (i.e., don't proceed to the next command until the desired altitude is reached. Params: 1) Climb or Descend (0 = Neutral, command completes when within 5m of this command's altitude, 1 = Climbing, command completes when at or above this command's altitude, 2 = Descending, command completes when at or below this command's altitude.; 2) Empty; 3) Empty; 4) Empty; 5) Empty; 6) Empty; 7) Desired altitude;
	MAV_CMD_NAV_LOITER_TO_ALT                  = 31    // Begin loiter at the specified Latitude and Longitude.  If Lat=Lon=0, then loiter at the current position.  Don't consider the navigation command complete (don't leave loiter) until the altitude has been reached. Additionally, if the Heading Required parameter is non-zero the aircraft will not leave the loiter until heading toward the next waypoint. Params: 1) Leave loiter circle only once heading towards the next waypoint (0 = False); 2) Loiter radius around waypoint for forward-only moving vehicles (not multicopters). If positive loiter clockwise, negative counter-clockwise, 0 means no change to standard loiter.; 3) Empty; 4) Loiter circle exit location and/or path to next waypoint ("xtrack") for forward-only moving vehicles (not multicopters). 0 for the vehicle to converge towards the center xtrack when it leaves the loiter (the line between the centers of the current and next waypoint), 1 to converge to the direct line between the location that the vehicle exits the loiter radius and the next waypoint. Otherwise the angle (in degrees) between the tangent of the loiter circle and the center xtrack at which the vehicle must leave the loiter (and converge to the center xtrack). NaN to use the current system default xtrack behaviour.; 5) Latitude; 6) Longitude; 7) Altitude;
	MAV_CMD_DO_FOLLOW                          = 32    // Begin following a target. Params: 1) System ID (of the FOLLOW_TARGET beacon). Send 0 to disable follow-me and return to the default position hold mode.; 2) Reserved; 3) Reserved; 4) Altitude mode: 0: Keep current altitude, 1: keep altitude difference to target, 2: go to a fixed altitude above home.; 5) Altitude above home. (used if mode=2); 6) Reserved; 7) Time to land in which the MAV should go to the default position hold mode after a message RX timeout.;
	MAV_CMD_DO_FOLLOW_REPOSITION               = 33    // Reposition the MAV after a follow target command has been sent. Params: 1) Camera q1 (where 0 is on the ray from the camera to the tracking device); 2) Camera q2; 3) Camera q3; 4) Camera q4; 5) altitude offset from target; 6) X offset from target; 7) Y offset from target;
	MAV_CMD_DO_ORBIT                           = 34    // Start orbiting on the circumference of a circle defined by the parameters. Setting any value NaN results in using defaults. Params: 1) Radius of the circle. positive: Orbit clockwise. negative: Orbit counter-clockwise.; 2) Tangential Velocity. NaN: Vehicle configuration default.; 3) Yaw behavior of the vehicle.; 4) Reserved (e.g. for dynamic center beacon options); 5) Center point latitude (if no MAV_FRAME specified) / X coordinate according to MAV_FRAME. NaN: Use current vehicle position or current center if already orbiting.; 6) Center point longitude (if no MAV_FRAME specified) / Y coordinate according to MAV_FRAME. NaN: Use current vehicle position or current center if already orbiting.; 7) Center point altitude (MSL) (if no MAV_FRAME specified) / Z coordinate according to MAV_FRAME. NaN: Use current vehicle position or current center if already orbiting.;
	MAV_CMD_NAV_ROI                            = 80    // Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicle's control system to control the vehicle attitude and the attitude of various sensors such as cameras. Params: 1) Region of interest mode.; 2) Waypoint index/ target ID. (see MAV_ROI enum); 3) ROI index (allows a vehicle to manage multiple ROI's); 4) Empty; 5) x the location of the fixed ROI (see MAV_FRAME); 6) y; 7) z;
	MAV_CMD_NAV_PATHPLANNING                   = 81    // Control autonomous path planning on the MAV. Params: 1) 0: Disable local obstacle avoidance / local path planning (without resetting map), 1: Enable local path planning, 2: Enable and reset local path planning; 2) 0: Disable full path planning (without resetting map), 1: Enable, 2: Enable and reset map/occupancy grid, 3: Enable and reset planned route, but not occupancy grid; 3) Empty; 4) Yaw angle at goal; 5) Latitude/X of goal; 6) Longitude/Y of goal; 7) Altitude/Z of goal;
	MAV_CMD_NAV_SPLINE_WAYPOINT                = 82    // Navigate to waypoint using a spline path. Params: 1) Hold time. (ignored by fixed wing, time to stay at waypoint for rotary wing); 2) Empty; 3) Empty; 4) Empty; 5) Latitude/X of goal; 6) Longitude/Y of goal; 7) Altitude/Z of goal;
	MAV_CMD_NAV_VTOL_TAKEOFF                   = 84    // Takeoff from ground using VTOL mode, and transition to forward flight with specified heading. The command should be ignored by vehicles that dont support both VTOL and fixed-wing flight (multicopters, boats,etc.). Params: 1) Empty; 2) Front transition heading.; 3) Empty; 4) Yaw angle. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).; 5) Latitude; 6) Longitude; 7) Altitude;
	MAV_CMD_NAV_VTOL_LAND                      = 85    // Land using VTOL mode. Params: 1) Empty; 2) Empty; 3) Approach altitude (with the same reference as the Altitude field). NaN if unspecified.; 4) Yaw angle. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).; 5) Latitude; 6) Longitude; 7) Altitude (ground level);
	MAV_CMD_NAV_GUIDED_ENABLE                  = 92    // hand control over to an external controller. Params: 1) On / Off (&gt; 0.5f on); 2) Empty; 3) Empty; 4) Empty; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_NAV_DELAY                          = 93    // Delay the next navigation command a number of seconds or until a specified time. Params: 1) Delay (-1 to enable time-of-day fields); 2) hour (24h format, UTC, -1 to ignore); 3) minute (24h format, UTC, -1 to ignore); 4) second (24h format, UTC, -1 to ignore); 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_NAV_PAYLOAD_PLACE                  = 94    // Descend and place payload. Vehicle moves to specified location, descends until it detects a hanging payload has reached the ground, and then releases the payload. If ground is not detected before the reaching the maximum descent value (param1), the command will complete without releasing the payload. Params: 1) Maximum distance to descend.; 2) Empty; 3) Empty; 4) Empty; 5) Latitude; 6) Longitude; 7) Altitude;
	MAV_CMD_NAV_LAST                           = 95    // NOP - This command is only used to mark the upper limit of the NAV/ACTION commands in the enumeration. Params: 1) Empty; 2) Empty; 3) Empty; 4) Empty; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_CONDITION_DELAY                    = 112   // Delay mission state machine. Params: 1) Delay; 2) Empty; 3) Empty; 4) Empty; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_CONDITION_CHANGE_ALT               = 113   // Ascend/descend to target altitude at specified rate. Delay mission state machine until desired altitude reached. Params: 1) Descent / Ascend rate.; 2) Empty; 3) Empty; 4) Empty; 5) Empty; 6) Empty; 7) Target Altitude;
	MAV_CMD_CONDITION_DISTANCE                 = 114   // Delay mission state machine until within desired distance of next NAV point. Params: 1) Distance.; 2) Empty; 3) Empty; 4) Empty; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_CONDITION_YAW                      = 115   // Reach a certain target angle. Params: 1) target angle, 0 is north; 2) angular speed; 3) direction: -1: counter clockwise, 1: clockwise; 4) 0: absolute angle, 1: relative offset; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_CONDITION_LAST                     = 159   // NOP - This command is only used to mark the upper limit of the CONDITION commands in the enumeration. Params: 1) Empty; 2) Empty; 3) Empty; 4) Empty; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_DO_SET_MODE                        = 176   // Set system mode. Params: 1) Mode; 2) Custom mode - this is system specific, please refer to the individual autopilot specifications for details.; 3) Custom sub mode - this is system specific, please refer to the individual autopilot specifications for details.; 4) Empty; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_DO_JUMP                            = 177   // Jump to the desired command in the mission list.  Repeat this action only the specified number of times. Params: 1) Sequence number; 2) Repeat count; 3) Empty; 4) Empty; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_DO_CHANGE_SPEED                    = 178   // Change speed and/or throttle set points. Params: 1) Speed type (0=Airspeed, 1=Ground Speed, 2=Climb Speed, 3=Descent Speed); 2) Speed (-1 indicates no change); 3) Throttle (-1 indicates no change); 4) 0: absolute, 1: relative; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_DO_SET_HOME                        = 179   // Changes the home location either to the current location or a specified location. Params: 1) Use current (1=use current location, 0=use specified location); 2) Empty; 3) Empty; 4) Yaw angle. NaN to use default heading; 5) Latitude; 6) Longitude; 7) Altitude;
	MAV_CMD_DO_SET_PARAMETER                   = 180   // Set a system parameter.  Caution!  Use of this command requires knowledge of the numeric enumeration value of the parameter. Params: 1) Parameter number; 2) Parameter value; 3) Empty; 4) Empty; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_DO_SET_RELAY                       = 181   // Set a relay to a condition. Params: 1) Relay instance number.; 2) Setting. (1=on, 0=off, others possible depending on system hardware); 3) Empty; 4) Empty; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_DO_REPEAT_RELAY                    = 182   // Cycle a relay on and off for a desired number of cycles with a desired period. Params: 1) Relay instance number.; 2) Cycle count.; 3) Cycle time.; 4) Empty; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_DO_SET_SERVO                       = 183   // Set a servo to a desired PWM value. Params: 1) Servo instance number.; 2) Pulse Width Modulation.; 3) Empty; 4) Empty; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_DO_REPEAT_SERVO                    = 184   // Cycle a between its nominal setting and a desired PWM for a desired number of cycles with a desired period. Params: 1) Servo instance number.; 2) Pulse Width Modulation.; 3) Cycle count.; 4) Cycle time.; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_DO_FLIGHTTERMINATION               = 185   // Terminate flight immediately. Params: 1) Flight termination activated if &gt; 0.5; 2) Empty; 3) Empty; 4) Empty; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_DO_CHANGE_ALTITUDE                 = 186   // Change altitude set point. Params: 1) Altitude; 2) Frame of new altitude.; 3) Empty; 4) Empty; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_DO_SET_ACTUATOR                    = 187   // Sets actuators (e.g. servos) to a desired value. The actuator numbers are mapped to specific outputs (e.g. on any MAIN or AUX PWM or UAVCAN) using a flight-stack specific mechanism (i.e. a parameter). Params: 1) Actuator 1 value, scaled from [-1 to 1]. NaN to ignore.; 2) Actuator 2 value, scaled from [-1 to 1]. NaN to ignore.; 3) Actuator 3 value, scaled from [-1 to 1]. NaN to ignore.; 4) Actuator 4 value, scaled from [-1 to 1]. NaN to ignore.; 5) Actuator 5 value, scaled from [-1 to 1]. NaN to ignore.; 6) Actuator 6 value, scaled from [-1 to 1]. NaN to ignore.; 7) Index of actuator set (i.e if set to 1, Actuator 1 becomes Actuator 7);
	MAV_CMD_DO_LAND_START                      = 189   // Mission command to perform a landing. This is used as a marker in a mission to tell the autopilot where a sequence of mission items that represents a landing starts. It may also be sent via a COMMAND_LONG to trigger a landing, in which case the nearest (geographically) landing sequence in the mission will be used. The Latitude/Longitude is optional, and may be set to 0 if not needed. If specified then it will be used to help find the closest landing sequence. Params: 1) Empty; 2) Empty; 3) Empty; 4) Empty; 5) Latitude; 6) Longitude; 7) Empty;
	MAV_CMD_DO_RALLY_LAND                      = 190   // Mission command to perform a landing from a rally point. Params: 1) Break altitude; 2) Landing speed; 3) Empty; 4) Empty; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_DO_GO_AROUND                       = 191   // Mission command to safely abort an autonomous landing. Params: 1) Altitude; 2) Empty; 3) Empty; 4) Empty; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_DO_REPOSITION                      = 192   // Reposition the vehicle to a specific WGS84 global position. Params: 1) Ground speed, less than 0 (-1) for default; 2) Bitmask of option flags.; 3) Reserved; 4) Yaw heading. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.). For planes indicates loiter direction (0: clockwise, 1: counter clockwise); 5) Latitude; 6) Longitude; 7) Altitude;
	MAV_CMD_DO_PAUSE_CONTINUE                  = 193   // If in a GPS controlled position mode, hold the current position or continue. Params: 1) 0: Pause current mission or reposition command, hold current position. 1: Continue mission. A VTOL capable vehicle should enter hover mode (multicopter and VTOL planes). A plane should loiter with the default loiter radius.; 2) Reserved; 3) Reserved; 4) Reserved; 5) Reserved; 6) Reserved; 7) Reserved;
	MAV_CMD_DO_SET_REVERSE                     = 194   // Set moving direction to forward or reverse. Params: 1) Direction (0=Forward, 1=Reverse); 2) Empty; 3) Empty; 4) Empty; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_DO_SET_ROI_LOCATION                = 195   // Sets the region of interest (ROI) to a location. This can then be used by the vehicle's control system to control the vehicle attitude and the attitude of various sensors such as cameras. This command can be sent to a gimbal manager but not to a gimbal device. A gimbal is not to react to this message. Params: 1) Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals).; 2) Empty; 3) Empty; 4) Empty; 5) Latitude of ROI location; 6) Longitude of ROI location; 7) Altitude of ROI location;
	MAV_CMD_DO_SET_ROI_WPNEXT_OFFSET           = 196   // Sets the region of interest (ROI) to be toward next waypoint, with optional pitch/roll/yaw offset. This can then be used by the vehicle's control system to control the vehicle attitude and the attitude of various sensors such as cameras. This command can be sent to a gimbal manager but not to a gimbal device. A gimbal device is not to react to this message. Params: 1) Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals).; 2) Empty; 3) Empty; 4) Empty; 5) Pitch offset from next waypoint, positive pitching up; 6) roll offset from next waypoint, positive rolling to the right; 7) yaw offset from next waypoint, positive yawing to the right;
	MAV_CMD_DO_SET_ROI_NONE                    = 197   // Cancels any previous ROI command returning the vehicle/sensors to default flight characteristics. This can then be used by the vehicle's control system to control the vehicle attitude and the attitude of various sensors such as cameras. This command can be sent to a gimbal manager but not to a gimbal device. A gimbal device is not to react to this message. After this command the gimbal manager should go back to manual input if available, and otherwise assume a neutral position. Params: 1) Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals).; 2) Empty; 3) Empty; 4) Empty; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_DO_SET_ROI_SYSID                   = 198   // Mount tracks system with specified system ID. Determination of target vehicle position may be done with GLOBAL_POSITION_INT or any other means. This command can be sent to a gimbal manager but not to a gimbal device. A gimbal device is not to react to this message. Params: 1) System ID; 2) Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals).;
	MAV_CMD_DO_CONTROL_VIDEO                   = 200   // Control onboard camera system. Params: 1) Camera ID (-1 for all); 2) Transmission: 0: disabled, 1: enabled compressed, 2: enabled raw; 3) Transmission mode: 0: video stream, &gt;0: single images every n seconds; 4) Recording: 0: disabled, 1: enabled compressed, 2: enabled raw; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_DO_SET_ROI                         = 201   // Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicle's control system to control the vehicle attitude and the attitude of various sensors such as cameras. Params: 1) Region of interest mode.; 2) Waypoint index/ target ID (depends on param 1).; 3) Region of interest index. (allows a vehicle to manage multiple ROI's); 4) Empty; 5) MAV_ROI_WPNEXT: pitch offset from next waypoint, MAV_ROI_LOCATION: latitude; 6) MAV_ROI_WPNEXT: roll offset from next waypoint, MAV_ROI_LOCATION: longitude; 7) MAV_ROI_WPNEXT: yaw offset from next waypoint, MAV_ROI_LOCATION: altitude;
	MAV_CMD_DO_DIGICAM_CONFIGURE               = 202   // Configure digital camera. This is a fallback message for systems that have not yet implemented PARAM_EXT_XXX messages and camera definition files (see https://mavlink.io/en/services/camera_def.html ). Params: 1) Modes: P, TV, AV, M, Etc.; 2) Shutter speed: Divisor number for one second.; 3) Aperture: F stop number.; 4) ISO number e.g. 80, 100, 200, Etc.; 5) Exposure type enumerator.; 6) Command Identity.; 7) Main engine cut-off time before camera trigger. (0 means no cut-off);
	MAV_CMD_DO_DIGICAM_CONTROL                 = 203   // Control digital camera. This is a fallback message for systems that have not yet implemented PARAM_EXT_XXX messages and camera definition files (see https://mavlink.io/en/services/camera_def.html ). Params: 1) Session control e.g. show/hide lens; 2) Zoom's absolute position; 3) Zooming step value to offset zoom from the current position; 4) Focus Locking, Unlocking or Re-locking; 5) Shooting Command; 6) Command Identity; 7) Test shot identifier. If set to 1, image will only be captured, but not counted towards internal frame count.;
	MAV_CMD_DO_MOUNT_CONFIGURE                 = 204   // Mission command to configure a camera or antenna mount. Params: 1) Mount operation mode; 2) stabilize roll? (1 = yes, 0 = no); 3) stabilize pitch? (1 = yes, 0 = no); 4) stabilize yaw? (1 = yes, 0 = no); 5) roll input (0 = angle body frame, 1 = angular rate, 2 = angle absolute frame); 6) pitch input (0 = angle body frame, 1 = angular rate, 2 = angle absolute frame); 7) yaw input (0 = angle body frame, 1 = angular rate, 2 = angle absolute frame);
	MAV_CMD_DO_MOUNT_CONTROL                   = 205   // Mission command to control a camera or antenna mount. Params: 1) pitch depending on mount mode (degrees or degrees/second depending on pitch input).; 2) roll depending on mount mode (degrees or degrees/second depending on roll input).; 3) yaw depending on mount mode (degrees or degrees/second depending on yaw input).; 4) altitude depending on mount mode.; 5) latitude, set if appropriate mount mode.; 6) longitude, set if appropriate mount mode.; 7) Mount mode.;
	MAV_CMD_DO_SET_CAM_TRIGG_DIST              = 206   // Mission command to set camera trigger distance for this flight. The camera is triggered each time this distance is exceeded. This command can also be used to set the shutter integration time for the camera. Params: 1) Camera trigger distance. 0 to stop triggering.; 2) Camera shutter integration time. -1 or 0 to ignore; 3) Trigger camera once immediately. (0 = no trigger, 1 = trigger); 4) Empty; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_DO_FENCE_ENABLE                    = 207   // Mission command to enable the geofence. Params: 1) enable? (0=disable, 1=enable, 2=disable_floor_only); 2) Empty; 3) Empty; 4) Empty; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_DO_PARACHUTE                       = 208   // Mission item/command to release a parachute or enable/disable auto release. Params: 1) Action; 2) Empty; 3) Empty; 4) Empty; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_DO_MOTOR_TEST                      = 209   // Mission command to perform motor test. Params: 1) Motor instance number. (from 1 to max number of motors on the vehicle); 2) Throttle type.; 3) Throttle.; 4) Timeout.; 5) Motor count. (number of motors to test to test in sequence, waiting for the timeout above between them; 0=1 motor, 1=1 motor, 2=2 motors...); 6) Motor test order.; 7) Empty;
	MAV_CMD_DO_INVERTED_FLIGHT                 = 210   // Change to/from inverted flight. Params: 1) Inverted flight. (0=normal, 1=inverted); 2) Empty; 3) Empty; 4) Empty; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_DO_GRIPPER                         = 211   // Mission command to operate a gripper. Params: 1) Gripper instance number.; 2) Gripper action to perform.; 3) Empty; 4) Empty; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_DO_AUTOTUNE_ENABLE                 = 212   // Enable/disable autotune. Params: 1) Enable (1: enable, 0:disable).; 2) Empty.; 3) Empty.; 4) Empty.; 5) Empty.; 6) Empty.; 7) Empty.;
	MAV_CMD_NAV_SET_YAW_SPEED                  = 213   // Sets a desired vehicle turn angle and speed change. Params: 1) Yaw angle to adjust steering by.; 2) Speed.; 3) Final angle. (0=absolute, 1=relative); 4) Empty; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL          = 214   // Mission command to set camera trigger interval for this flight. If triggering is enabled, the camera is triggered each time this interval expires. This command can also be used to set the shutter integration time for the camera. Params: 1) Camera trigger cycle time. -1 or 0 to ignore.; 2) Camera shutter integration time. Should be less than trigger cycle time. -1 or 0 to ignore.; 3) Empty; 4) Empty; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_DO_MOUNT_CONTROL_QUAT              = 220   // Mission command to control a camera or antenna mount, using a quaternion as reference. Params: 1) quaternion param q1, w (1 in null-rotation); 2) quaternion param q2, x (0 in null-rotation); 3) quaternion param q3, y (0 in null-rotation); 4) quaternion param q4, z (0 in null-rotation); 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_DO_GUIDED_MASTER                   = 221   // set id of master controller. Params: 1) System ID; 2) Component ID; 3) Empty; 4) Empty; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_DO_GUIDED_LIMITS                   = 222   // Set limits for external control. Params: 1) Timeout - maximum time that external controller will be allowed to control vehicle. 0 means no timeout.; 2) Altitude (MSL) min - if vehicle moves below this alt, the command will be aborted and the mission will continue. 0 means no lower altitude limit.; 3) Altitude (MSL) max - if vehicle moves above this alt, the command will be aborted and the mission will continue. 0 means no upper altitude limit.; 4) Horizontal move limit - if vehicle moves more than this distance from its location at the moment the command was executed, the command will be aborted and the mission will continue. 0 means no horizontal move limit.; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_DO_ENGINE_CONTROL                  = 223   // Control vehicle engine. This is interpreted by the vehicles engine controller to change the target engine state. It is intended for vehicles with internal combustion engines. Params: 1) 0: Stop engine, 1:Start Engine; 2) 0: Warm start, 1:Cold start. Controls use of choke where applicable; 3) Height delay. This is for commanding engine start only after the vehicle has gained the specified height. Used in VTOL vehicles during takeoff to start engine after the aircraft is off the ground. Zero for no delay.; 4) Empty; 5) Empty; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_DO_SET_MISSION_CURRENT             = 224   // Set the mission item with sequence number seq as current item. This means that the MAV will continue to this mission item on the shortest path (not following the mission items in-between). Params: 1) Mission sequence value to set; 2) Empty; 3) Empty; 4) Empty; 5) Empty; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_DO_LAST                            = 240   // NOP - This command is only used to mark the upper limit of the DO commands in the enumeration. Params: 1) Empty; 2) Empty; 3) Empty; 4) Empty; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_PREFLIGHT_CALIBRATION              = 241   // Trigger calibration. This command will be only accepted if in pre-flight mode. Except for Temperature Calibration, only one sensor should be set in a single message and all others should be zero. Params: 1) 1: gyro calibration, 3: gyro temperature calibration; 2) 1: magnetometer calibration; 3) 1: ground pressure calibration; 4) 1: radio RC calibration, 2: RC trim calibration; 5) 1: accelerometer calibration, 2: board level calibration, 3: accelerometer temperature calibration, 4: simple accelerometer calibration; 6) 1: APM: compass/motor interference calibration (PX4: airspeed calibration, deprecated), 2: airspeed calibration; 7) 1: ESC calibration, 3: barometer temperature calibration;
	MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS       = 242   // Set sensor offsets. This command will be only accepted if in pre-flight mode. Params: 1) Sensor to adjust the offsets for: 0: gyros, 1: accelerometer, 2: magnetometer, 3: barometer, 4: optical flow, 5: second magnetometer, 6: third magnetometer; 2) X axis offset (or generic dimension 1), in the sensor's raw units; 3) Y axis offset (or generic dimension 2), in the sensor's raw units; 4) Z axis offset (or generic dimension 3), in the sensor's raw units; 5) Generic dimension 4, in the sensor's raw units; 6) Generic dimension 5, in the sensor's raw units; 7) Generic dimension 6, in the sensor's raw units;
	MAV_CMD_PREFLIGHT_UAVCAN                   = 243   // Trigger UAVCAN configuration (actuator ID assignment and direction mapping). Note that this maps to the legacy UAVCAN v0 function UAVCAN_ENUMERATE, which is intended to be executed just once during initial vehicle configuration (it is not a normal pre-flight command and has been poorly named). Params: 1) 1: Trigger actuator ID assignment and direction mapping. 0: Cancel command.; 2) Reserved; 3) Reserved; 4) Reserved; 5) Reserved; 6) Reserved; 7) Reserved;
	MAV_CMD_PREFLIGHT_STORAGE                  = 245   // Request storage of different parameter values and logs. This command will be only accepted if in pre-flight mode. Params: 1) Parameter storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM, 2: Reset to defaults; 2) Mission storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM, 2: Reset to defaults; 3) Onboard logging: 0: Ignore, 1: Start default rate logging, -1: Stop logging, &gt; 1: logging rate (e.g. set to 1000 for 1000 Hz logging); 4) Reserved; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN          = 246   // Request the reboot or shutdown of system components. Params: 1) 0: Do nothing for autopilot, 1: Reboot autopilot, 2: Shutdown autopilot, 3: Reboot autopilot and keep it in the bootloader until upgraded.; 2) 0: Do nothing for onboard computer, 1: Reboot onboard computer, 2: Shutdown onboard computer, 3: Reboot onboard computer and keep it in the bootloader until upgraded.; 3) WIP: 0: Do nothing for camera, 1: Reboot onboard camera, 2: Shutdown onboard camera, 3: Reboot onboard camera and keep it in the bootloader until upgraded; 4) WIP: 0: Do nothing for mount (e.g. gimbal), 1: Reboot mount, 2: Shutdown mount, 3: Reboot mount and keep it in the bootloader until upgraded; 5) Reserved (set to 0); 6) Reserved (set to 0); 7) WIP: ID (e.g. camera ID -1 for all IDs);
	MAV_CMD_DO_UPGRADE                         = 247   // Request a target system to start an upgrade of one (or all) of its components. For example, the command might be sent to a companion computer to cause it to upgrade a connected flight controller. The system doing the upgrade will report progress using the normal command protocol sequence for a long running operation. Command protocol information: https://mavlink.io/en/services/command.html. Params: 1) Component id of the component to be upgraded. If set to 0, all components should be upgraded.; 2) 0: Do not reboot component after the action is executed, 1: Reboot component after the action is executed.; 3) Reserved; 4) Reserved; 5) Reserved; 6) Reserved; 7) WIP: upgrade progress report rate (can be used for more granular control).;
	MAV_CMD_OVERRIDE_GOTO                      = 252   // Override current mission with command to pause mission, pause mission and move to position, continue/resume mission. When param 1 indicates that the mission is paused (MAV_GOTO_DO_HOLD), param 2 defines whether it holds in place or moves to another position. Params: 1) MAV_GOTO_DO_HOLD: pause mission and either hold or move to specified position (depending on param2), MAV_GOTO_DO_CONTINUE: resume mission.; 2) MAV_GOTO_HOLD_AT_CURRENT_POSITION: hold at current position, MAV_GOTO_HOLD_AT_SPECIFIED_POSITION: hold at specified position.; 3) Coordinate frame of hold point.; 4) Desired yaw angle.; 5) Latitude/X position.; 6) Longitude/Y position.; 7) Altitude/Z position.;
	MAV_CMD_OBLIQUE_SURVEY                     = 260   // Mission command to set a Camera Auto Mount Pivoting Oblique Survey (Replaces CAM_TRIGG_DIST for this purpose). The camera is triggered each time this distance is exceeded, then the mount moves to the next position. Params 4~6 set-up the angle limits and number of positions for oblique survey, where mount-enabled vehicles automatically roll the camera between shots to emulate an oblique camera setup (providing an increased HFOV). This command can also be used to set the shutter integration time for the camera. Params: 1) Camera trigger distance. 0 to stop triggering.; 2) Camera shutter integration time. 0 to ignore; 3) The minimum interval in which the camera is capable of taking subsequent pictures repeatedly. 0 to ignore.; 4) Total number of roll positions at which the camera will capture photos (images captures spread evenly across the limits defined by param5).; 5) Angle limits that the camera can be rolled to left and right of center.; 6) Fixed pitch angle that the camera will hold in oblique mode if the mount is actuated in the pitch axis.; 7) Empty;
	MAV_CMD_MISSION_START                      = 300   // start running a mission. Params: 1) first_item: the first mission item to run; 2) last_item:  the last mission item to run (after this item is run, the mission ends);
	MAV_CMD_COMPONENT_ARM_DISARM               = 400   // Arms / Disarms a component. Params: 1) 0: disarm, 1: arm; 2) 0: arm-disarm unless prevented by safety checks (i.e. when landed), 21196: force arming/disarming (e.g. allow arming to override preflight checks and disarming in flight);
	MAV_CMD_ILLUMINATOR_ON_OFF                 = 405   // Turns illuminators ON/OFF. An illuminator is a light source that is used for lighting up dark areas external to the sytstem: e.g. a torch or searchlight (as opposed to a light source for illuminating the system itself, e.g. an indicator light). Params: 1) 0: Illuminators OFF, 1: Illuminators ON;
	MAV_CMD_GET_HOME_POSITION                  = 410   // Request the home position from the vehicle. Params: 1) Reserved; 2) Reserved; 3) Reserved; 4) Reserved; 5) Reserved; 6) Reserved; 7) Reserved;
	MAV_CMD_INJECT_FAILURE                     = 420   // Inject artificial failure for testing purposes. Note that autopilots should implement an additional protection before accepting this command such as a specific param setting. Params: 1) The unit which is affected by the failure.; 2) The type how the failure manifests itself.; 3) Instance affected by failure (0 to signal all).;
	MAV_CMD_START_RX_PAIR                      = 500   // Starts receiver pairing. Params: 1) 0:Spektrum.; 2) RC type.;
	MAV_CMD_GET_MESSAGE_INTERVAL               = 510   // Request the interval between messages for a particular MAVLink message ID. The receiver should ACK the command and then emit its response in a MESSAGE_INTERVAL message. Params: 1) The MAVLink message ID;
	MAV_CMD_SET_MESSAGE_INTERVAL               = 511   // Set the interval between messages for a particular MAVLink message ID. This interface replaces REQUEST_DATA_STREAM. Params: 1) The MAVLink message ID; 2) The interval between two messages. Set to -1 to disable and 0 to request default rate.; 7) Target address of message stream (if message has target address fields). 0: Flight-stack default (recommended), 1: address of requestor, 2: broadcast.;
	MAV_CMD_REQUEST_MESSAGE                    = 512   // Request the target system(s) emit a single instance of a specified message (i.e. a "one-shot" version of MAV_CMD_SET_MESSAGE_INTERVAL). Params: 1) The MAVLink message ID of the requested message.; 2) Use for index ID, if required. Otherwise, the use of this parameter (if any) must be defined in the requested message. By default assumed not used (0).; 3) The use of this parameter (if any), must be defined in the requested message. By default assumed not used (0).; 4) The use of this parameter (if any), must be defined in the requested message. By default assumed not used (0).; 5) The use of this parameter (if any), must be defined in the requested message. By default assumed not used (0).; 6) The use of this parameter (if any), must be defined in the requested message. By default assumed not used (0).; 7) Target address for requested message (if message has target address fields). 0: Flight-stack default, 1: address of requestor, 2: broadcast.;
	MAV_CMD_REQUEST_PROTOCOL_VERSION           = 519   // Request MAVLink protocol version compatibility. All receivers should ACK the command and then emit their capabilities in an PROTOCOL_VERSION message. Params: 1) 1: Request supported protocol versions by all nodes on the network; 2) Reserved (all remaining params);
	MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES     = 520   // Request autopilot capabilities. The receiver should ACK the command and then emit its capabilities in an AUTOPILOT_VERSION message. Params: 1) 1: Request autopilot version; 2) Reserved (all remaining params);
	MAV_CMD_REQUEST_CAMERA_INFORMATION         = 521   // Request camera information (CAMERA_INFORMATION). Params: 1) 0: No action 1: Request camera capabilities; 2) Reserved (all remaining params);
	MAV_CMD_REQUEST_CAMERA_SETTINGS            = 522   // Request camera settings (CAMERA_SETTINGS). Params: 1) 0: No Action 1: Request camera settings; 2) Reserved (all remaining params);
	MAV_CMD_REQUEST_STORAGE_INFORMATION        = 525   // Request storage information (STORAGE_INFORMATION). Use the command's target_component to target a specific component's storage. Params: 1) Storage ID (0 for all, 1 for first, 2 for second, etc.); 2) 0: No Action 1: Request storage information; 3) Reserved (all remaining params);
	MAV_CMD_STORAGE_FORMAT                     = 526   // Format a storage medium. Once format is complete, a STORAGE_INFORMATION message is sent. Use the command's target_component to target a specific component's storage. Params: 1) Storage ID (1 for first, 2 for second, etc.); 2) Format storage (and reset image log). 0: No action 1: Format storage; 3) Reset Image Log (without formatting storage medium). This will reset CAMERA_CAPTURE_STATUS.image_count and CAMERA_IMAGE_CAPTURED.image_index. 0: No action 1: Reset Image Log; 4) Reserved (all remaining params);
	MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS      = 527   // Request camera capture status (CAMERA_CAPTURE_STATUS). Params: 1) 0: No Action 1: Request camera capture status; 2) Reserved (all remaining params);
	MAV_CMD_REQUEST_FLIGHT_INFORMATION         = 528   // Request flight information (FLIGHT_INFORMATION). Params: 1) 1: Request flight information; 2) Reserved (all remaining params);
	MAV_CMD_RESET_CAMERA_SETTINGS              = 529   // Reset all camera settings to Factory Default. Params: 1) 0: No Action 1: Reset all settings; 2) Reserved (all remaining params);
	MAV_CMD_SET_CAMERA_MODE                    = 530   // Set camera running mode. Use NaN for reserved values. GCS will send a MAV_CMD_REQUEST_VIDEO_STREAM_STATUS command after a mode change if the camera supports video streaming. Params: 1) Reserved (Set to 0); 2) Camera mode; 3) ; 4) ; 7) ;
	MAV_CMD_SET_CAMERA_ZOOM                    = 531   // Set camera zoom. Camera must respond with a CAMERA_SETTINGS message (on success). Params: 1) Zoom type; 2) Zoom value. The range of valid values depend on the zoom type.; 3) ; 4) ; 7) ;
	MAV_CMD_SET_CAMERA_FOCUS                   = 532   // Set camera focus. Camera must respond with a CAMERA_SETTINGS message (on success). Params: 1) Focus type; 2) Focus value; 3) ; 4) ; 7) ;
	MAV_CMD_JUMP_TAG                           = 600   // Tagged jump target. Can be jumped to with MAV_CMD_DO_JUMP_TAG. Params: 1) Tag.;
	MAV_CMD_DO_JUMP_TAG                        = 601   // Jump to the matching tag in the mission list. Repeat this action for the specified number of times. A mission should contain a single matching tag for each jump. If this is not the case then a jump to a missing tag should complete the mission, and a jump where there are multiple matching tags should always select the one with the lowest mission sequence number. Params: 1) Target tag to jump to.; 2) Repeat count.;
	MAV_CMD_PARAM_TRANSACTION                  = 900   // Request to start or end a parameter transaction. Multiple kinds of transport layers can be used to exchange parameters in the transaction (param, param_ext and mavftp). The command response can either be a success/failure or an in progress in case the receiving side takes some time to apply the parameters. Params: 1) Action to be performed (start, commit, cancel, etc.); 2) Possible transport layers to set and get parameters via mavlink during a parameter transaction.; 3) Identifier for a specific transaction.;
	MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW         = 1000  // High level setpoint to be sent to a gimbal manager to set a gimbal attitude. It is possible to set combinations of the values below. E.g. an angle as well as a desired angular rate can be used to get to this angle at a certain angular rate, or an angular rate only will result in continuous turning. NaN is to be used to signal unset. Note: a gimbal is never to react to this command but only the gimbal manager. Params: 1) Pitch angle (positive to pitch up, relative to vehicle for FOLLOW mode, relative to world horizon for LOCK mode).; 2) Yaw angle (positive to yaw to the right, relative to vehicle for FOLLOW mode, absolute to North for LOCK mode).; 3) Pitch rate (positive to pitch up).; 4) Yaw rate (positive to yaw to the right).; 5) Gimbal manager flags to use.; 7) Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals).;
	MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE        = 1001  // Gimbal configuration to set which sysid/compid is in primary and secondary control. Params: 1) Sysid for primary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for missions where the own sysid is still unknown), -3: remove control if currently in control).; 2) Compid for primary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for missions where the own sysid is still unknown), -3: remove control if currently in control).; 3) Sysid for secondary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for missions where the own sysid is still unknown), -3: remove control if currently in control).; 4) Compid for secondary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for missions where the own sysid is still unknown), -3: remove control if currently in control).; 7) Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals).;
	MAV_CMD_IMAGE_START_CAPTURE                = 2000  // Start image capture sequence. Sends CAMERA_IMAGE_CAPTURED after each capture. Use NaN for reserved values. Params: 1) Reserved (Set to 0); 2) Desired elapsed time between two consecutive pictures (in seconds). Minimum values depend on hardware (typically greater than 2 seconds).; 3) Total number of images to capture. 0 to capture forever/until MAV_CMD_IMAGE_STOP_CAPTURE.; 4) Capture sequence number starting from 1. This is only valid for single-capture (param3 == 1), otherwise set to 0. Increment the capture ID for each capture command to prevent double captures when a command is re-transmitted.; 5) ; 6) ; 7) ;
	MAV_CMD_IMAGE_STOP_CAPTURE                 = 2001  // Stop image capture sequence Use NaN for reserved values. Params: 1) Reserved (Set to 0); 2) ; 3) ; 4) ; 7) ;
	MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE       = 2002  // Re-request a CAMERA_IMAGE_CAPTURED message. Params: 1) Sequence number for missing CAMERA_IMAGE_CAPTURED message; 2) ; 3) ; 4) ; 7) ;
	MAV_CMD_DO_TRIGGER_CONTROL                 = 2003  // Enable or disable on-board camera triggering system. Params: 1) Trigger enable/disable (0 for disable, 1 for start), -1 to ignore; 2) 1 to reset the trigger sequence, -1 or 0 to ignore; 3) 1 to pause triggering, but without switching the camera off or retracting it. -1 to ignore;
	MAV_CMD_CAMERA_TRACK_POINT                 = 2004  // If the camera supports point visual tracking (CAMERA_CAP_FLAGS_HAS_TRACKING_POINT is set), this command allows to initiate the tracking. Params: 1) Point to track x value (normalized 0..1, 0 is left, 1 is right).; 2) Point to track y value (normalized 0..1, 0 is top, 1 is bottom).; 3) Point radius (normalized 0..1, 0 is image left, 1 is image right).;
	MAV_CMD_CAMERA_TRACK_RECTANGLE             = 2005  // If the camera supports rectangle visual tracking (CAMERA_CAP_FLAGS_HAS_TRACKING_RECTANGLE is set), this command allows to initiate the tracking. Params: 1) Top left corner of rectangle x value (normalized 0..1, 0 is left, 1 is right).; 2) Top left corner of rectangle y value (normalized 0..1, 0 is top, 1 is bottom).; 3) Bottom right corner of rectangle x value (normalized 0..1, 0 is left, 1 is right).; 4) Bottom right corner of rectangle y value (normalized 0..1, 0 is top, 1 is bottom).;
	MAV_CMD_CAMERA_STOP_TRACKING               = 2010  // Stops ongoing tracking
	MAV_CMD_VIDEO_START_CAPTURE                = 2500  // Starts video capture (recording). Params: 1) Video Stream ID (0 for all streams); 2) Frequency CAMERA_CAPTURE_STATUS messages should be sent while recording (0 for no messages, otherwise frequency); 3) ; 4) ; 5) ; 6) ; 7) ;
	MAV_CMD_VIDEO_STOP_CAPTURE                 = 2501  // Stop the current video capture (recording). Params: 1) Video Stream ID (0 for all streams); 2) ; 3) ; 4) ; 5) ; 6) ; 7) ;
	MAV_CMD_VIDEO_START_STREAMING              = 2502  // Start video streaming. Params: 1) Video Stream ID (0 for all streams, 1 for first, 2 for second, etc.);
	MAV_CMD_VIDEO_STOP_STREAMING               = 2503  // Stop the given video stream. Params: 1) Video Stream ID (0 for all streams, 1 for first, 2 for second, etc.);
	MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION   = 2504  // Request video stream information (VIDEO_STREAM_INFORMATION). Params: 1) Video Stream ID (0 for all streams, 1 for first, 2 for second, etc.);
	MAV_CMD_REQUEST_VIDEO_STREAM_STATUS        = 2505  // Request video stream status (VIDEO_STREAM_STATUS). Params: 1) Video Stream ID (0 for all streams, 1 for first, 2 for second, etc.);
	MAV_CMD_LOGGING_START                      = 2510  // Request to start streaming logging data over MAVLink (see also LOGGING_DATA message). Params: 1) Format: 0: ULog; 2) Reserved (set to 0); 3) Reserved (set to 0); 4) Reserved (set to 0); 5) Reserved (set to 0); 6) Reserved (set to 0); 7) Reserved (set to 0);
	MAV_CMD_LOGGING_STOP                       = 2511  // Request to stop streaming log data over MAVLink. Params: 1) Reserved (set to 0); 2) Reserved (set to 0); 3) Reserved (set to 0); 4) Reserved (set to 0); 5) Reserved (set to 0); 6) Reserved (set to 0); 7) Reserved (set to 0);
	MAV_CMD_AIRFRAME_CONFIGURATION             = 2520  // Params: 1) Landing gear ID (default: 0, -1 for all); 2) Landing gear position (Down: 0, Up: 1, NaN for no change); 3) ; 4) ; 5) ; 6) ; 7) ;
	MAV_CMD_CONTROL_HIGH_LATENCY               = 2600  // Request to start/stop transmitting over the high latency telemetry. Params: 1) Control transmission over high latency telemetry (0: stop, 1: start); 2) Empty; 3) Empty; 4) Empty; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_PANORAMA_CREATE                    = 2800  // Create a panorama at the current position. Params: 1) Viewing angle horizontal of the panorama (+- 0.5 the total angle); 2) Viewing angle vertical of panorama.; 3) Speed of the horizontal rotation.; 4) Speed of the vertical rotation.;
	MAV_CMD_DO_VTOL_TRANSITION                 = 3000  // Request VTOL transition. Params: 1) The target VTOL state. Only MAV_VTOL_STATE_MC and MAV_VTOL_STATE_FW can be used.;
	MAV_CMD_ARM_AUTHORIZATION_REQUEST          = 3001  // Request authorization to arm the vehicle to a external entity, the arm authorizer is responsible to request all data that is needs from the vehicle before authorize or deny the request. If approved the progress of command_ack message should be set with period of time that this authorization is valid in seconds or in case it was denied it should be set with one of the reasons in ARM_AUTH_DENIED_REASON. Params: 1) Vehicle system id, this way ground station can request arm authorization on behalf of any vehicle;
	MAV_CMD_SET_GUIDED_SUBMODE_STANDARD        = 4000  // This command sets the submode to standard guided when vehicle is in guided mode. The vehicle holds position and altitude and the user can input the desired velocities along all three axes
	MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE          = 4001  // This command sets submode circle when vehicle is in guided mode. Vehicle flies along a circle facing the center of the circle. The user can input the velocity along the circle and change the radius. If no input is given the vehicle will hold position. Params: 1) Radius of desired circle in CIRCLE_MODE; 2) User defined; 3) User defined; 4) User defined; 5) Target latitude of center of circle in CIRCLE_MODE; 6) Target longitude of center of circle in CIRCLE_MODE;
	MAV_CMD_CONDITION_GATE                     = 4501  // Delay mission state machine until gate has been reached. Params: 1) Geometry: 0: orthogonal to path between previous and next waypoint.; 2) Altitude: 0: ignore altitude; 3) Empty; 4) Empty; 5) Latitude; 6) Longitude; 7) Altitude;
	MAV_CMD_NAV_FENCE_RETURN_POINT             = 5000  // Fence return point. There can only be one fence return point. Params: 1) Reserved; 2) Reserved; 3) Reserved; 4) Reserved; 5) Latitude; 6) Longitude; 7) Altitude;
	MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION = 5001  // Fence vertex for an inclusion polygon (the polygon must not be self-intersecting). The vehicle must stay within this area. Minimum of 3 vertices required. Params: 1) Polygon vertex count; 2) Vehicle must be inside ALL inclusion zones in a single group, vehicle must be inside at least one group, must be the same for all points in each polygon; 3) Reserved; 4) Reserved; 5) Latitude; 6) Longitude; 7) Reserved;
	MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION = 5002  // Fence vertex for an exclusion polygon (the polygon must not be self-intersecting). The vehicle must stay outside this area. Minimum of 3 vertices required. Params: 1) Polygon vertex count; 2) Reserved; 3) Reserved; 4) Reserved; 5) Latitude; 6) Longitude; 7) Reserved;
	MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION         = 5003  // Circular fence area. The vehicle must stay inside this area. Params: 1) Radius.; 2) Vehicle must be inside ALL inclusion zones in a single group, vehicle must be inside at least one group; 3) Reserved; 4) Reserved; 5) Latitude; 6) Longitude; 7) Reserved;
	MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION         = 5004  // Circular fence area. The vehicle must stay outside this area. Params: 1) Radius.; 2) Reserved; 3) Reserved; 4) Reserved; 5) Latitude; 6) Longitude; 7) Reserved;
	MAV_CMD_NAV_RALLY_POINT                    = 5100  // Rally point. You can have multiple rally points defined. Params: 1) Reserved; 2) Reserved; 3) Reserved; 4) Reserved; 5) Latitude; 6) Longitude; 7) Altitude;
	MAV_CMD_UAVCAN_GET_NODE_INFO               = 5200  // Commands the vehicle to respond with a sequence of messages UAVCAN_NODE_INFO, one message per every UAVCAN node that is online. Note that some of the response messages can be lost, which the receiver can detect easily by checking whether every received UAVCAN_NODE_STATUS has a matching message UAVCAN_NODE_INFO received earlier; if not, this command should be sent again in order to request re-transmission of the node information messages. Params: 1) Reserved (set to 0); 2) Reserved (set to 0); 3) Reserved (set to 0); 4) Reserved (set to 0); 5) Reserved (set to 0); 6) Reserved (set to 0); 7) Reserved (set to 0);
	MAV_CMD_PAYLOAD_PREPARE_DEPLOY             = 30001 // Deploy payload on a Lat / Lon / Alt position. This includes the navigation to reach the required release position and velocity. Params: 1) Operation mode. 0: prepare single payload deploy (overwriting previous requests), but do not execute it. 1: execute payload deploy immediately (rejecting further deploy commands during execution, but allowing abort). 2: add payload deploy to existing deployment list.; 2) Desired approach vector in compass heading. A negative value indicates the system can define the approach vector at will.; 3) Desired ground speed at release time. This can be overridden by the airframe in case it needs to meet minimum airspeed. A negative value indicates the system can define the ground speed at will.; 4) Minimum altitude clearance to the release position. A negative value indicates the system can define the clearance at will.; 5) Latitude. Note, if used in MISSION_ITEM (deprecated) the units are degrees (unscaled); 6) Longitude. Note, if used in MISSION_ITEM (deprecated) the units are degrees (unscaled); 7) Altitude (MSL);
	MAV_CMD_PAYLOAD_CONTROL_DEPLOY             = 30002 // Control the payload deployment. Params: 1) Operation mode. 0: Abort deployment, continue normal mission. 1: switch to payload deployment mode. 100: delete first payload deployment request. 101: delete all payload deployment requests.; 2) Reserved; 3) Reserved; 4) Reserved; 5) Reserved; 6) Reserved; 7) Reserved;
	MAV_CMD_FIXED_MAG_CAL_YAW                  = 42006 // Magnetometer calibration based on provided known yaw. This allows for fast calibration using WMM field tables in the vehicle, given only the known yaw of the vehicle. If Latitude and longitude are both zero then use the current vehicle location. Params: 1) Yaw of vehicle in earth frame.; 2) CompassMask, 0 for all.; 3) Latitude.; 4) Longitude.; 5) Empty.; 6) Empty.; 7) Empty.;
	MAV_CMD_DO_WINCH                           = 42600 // Command to operate winch. Params: 1) Winch instance number.; 2) Action to perform.; 3) Length of cable to release (negative to wind).; 4) Release rate (negative to wind).; 5) Empty.; 6) Empty.; 7) Empty.;
	MAV_CMD_WAYPOINT_USER_1                    = 31000 // User defined waypoint item. Ground Station will show the Vehicle as flying through this item. Params: 1) User defined; 2) User defined; 3) User defined; 4) User defined; 5) Latitude unscaled; 6) Longitude unscaled; 7) Altitude (MSL);
	MAV_CMD_WAYPOINT_USER_2                    = 31001 // User defined waypoint item. Ground Station will show the Vehicle as flying through this item. Params: 1) User defined; 2) User defined; 3) User defined; 4) User defined; 5) Latitude unscaled; 6) Longitude unscaled; 7) Altitude (MSL);
	MAV_CMD_WAYPOINT_USER_3                    = 31002 // User defined waypoint item. Ground Station will show the Vehicle as flying through this item. Params: 1) User defined; 2) User defined; 3) User defined; 4) User defined; 5) Latitude unscaled; 6) Longitude unscaled; 7) Altitude (MSL);
	MAV_CMD_WAYPOINT_USER_4                    = 31003 // User defined waypoint item. Ground Station will show the Vehicle as flying through this item. Params: 1) User defined; 2) User defined; 3) User defined; 4) User defined; 5) Latitude unscaled; 6) Longitude unscaled; 7) Altitude (MSL);
	MAV_CMD_WAYPOINT_USER_5                    = 31004 // User defined waypoint item. Ground Station will show the Vehicle as flying through this item. Params: 1) User defined; 2) User defined; 3) User defined; 4) User defined; 5) Latitude unscaled; 6) Longitude unscaled; 7) Altitude (MSL);
	MAV_CMD_SPATIAL_USER_1                     = 31005 // User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. Params: 1) User defined; 2) User defined; 3) User defined; 4) User defined; 5) Latitude unscaled; 6) Longitude unscaled; 7) Altitude (MSL);
	MAV_CMD_SPATIAL_USER_2                     = 31006 // User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. Params: 1) User defined; 2) User defined; 3) User defined; 4) User defined; 5) Latitude unscaled; 6) Longitude unscaled; 7) Altitude (MSL);
	MAV_CMD_SPATIAL_USER_3                     = 31007 // User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. Params: 1) User defined; 2) User defined; 3) User defined; 4) User defined; 5) Latitude unscaled; 6) Longitude unscaled; 7) Altitude (MSL);
	MAV_CMD_SPATIAL_USER_4                     = 31008 // User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. Params: 1) User defined; 2) User defined; 3) User defined; 4) User defined; 5) Latitude unscaled; 6) Longitude unscaled; 7) Altitude (MSL);
	MAV_CMD_SPATIAL_USER_5                     = 31009 // User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. Params: 1) User defined; 2) User defined; 3) User defined; 4) User defined; 5) Latitude unscaled; 6) Longitude unscaled; 7) Altitude (MSL);
	MAV_CMD_USER_1                             = 31010 // User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. Params: 1) User defined; 2) User defined; 3) User defined; 4) User defined; 5) User defined; 6) User defined; 7) User defined;
	MAV_CMD_USER_2                             = 31011 // User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. Params: 1) User defined; 2) User defined; 3) User defined; 4) User defined; 5) User defined; 6) User defined; 7) User defined;
	MAV_CMD_USER_3                             = 31012 // User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. Params: 1) User defined; 2) User defined; 3) User defined; 4) User defined; 5) User defined; 6) User defined; 7) User defined;
	MAV_CMD_USER_4                             = 31013 // User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. Params: 1) User defined; 2) User defined; 3) User defined; 4) User defined; 5) User defined; 6) User defined; 7) User defined;
	MAV_CMD_USER_5                             = 31014 // User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. Params: 1) User defined; 2) User defined; 3) User defined; 4) User defined; 5) User defined; 6) User defined; 7) User defined;
)

// MavDataStream (generated enum)
// A data stream is not a fixed set of messages, but rather a      recommendation to the autopilot software. Individual autopilots may or may not obey      the recommended messages.
const (
	MAV_DATA_STREAM_ALL             = 0  // Enable all data streams
	MAV_DATA_STREAM_RAW_SENSORS     = 1  // Enable IMU_RAW, GPS_RAW, GPS_STATUS packets
	MAV_DATA_STREAM_EXTENDED_STATUS = 2  // Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS
	MAV_DATA_STREAM_RC_CHANNELS     = 3  // Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW
	MAV_DATA_STREAM_RAW_CONTROLLER  = 4  // Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT
	MAV_DATA_STREAM_POSITION        = 6  // Enable LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT messages
	MAV_DATA_STREAM_EXTRA1          = 10 // Dependent on the autopilot
	MAV_DATA_STREAM_EXTRA2          = 11 // Dependent on the autopilot
	MAV_DATA_STREAM_EXTRA3          = 12 // Dependent on the autopilot
)

// MavRoi (generated enum)
// The ROI (region of interest) for the vehicle. This can be                 be used by the vehicle for camera/vehicle attitude alignment (see                 MAV_CMD_NAV_ROI).
const (
	MAV_ROI_NONE     = 0 // No region of interest
	MAV_ROI_WPNEXT   = 1 // Point toward next waypoint, with optional pitch/roll/yaw offset
	MAV_ROI_WPINDEX  = 2 // Point toward given waypoint
	MAV_ROI_LOCATION = 3 // Point toward fixed location
	MAV_ROI_TARGET   = 4 // Point toward of given id
)

// MavCmdAck (generated enum)
// ACK / NACK / ERROR values as a result of MAV_CMDs and for mission item transmission.
const (
	MAV_CMD_ACK_OK                                 = 0 // Command / mission item is ok
	MAV_CMD_ACK_ERR_FAIL                           = 1 // Generic error message if none of the other reasons fails or if no detailed error reporting is implemented
	MAV_CMD_ACK_ERR_ACCESS_DENIED                  = 2 // The system is refusing to accept this command from this source / communication partner
	MAV_CMD_ACK_ERR_NOT_SUPPORTED                  = 3 // Command or mission item is not supported, other commands would be accepted
	MAV_CMD_ACK_ERR_COORDINATE_FRAME_NOT_SUPPORTED = 4 // The coordinate frame of this command / mission item is not supported
	MAV_CMD_ACK_ERR_COORDINATES_OUT_OF_RANGE       = 5 // The coordinate frame of this command is ok, but he coordinate values exceed the safety limits of this system. This is a generic error, please use the more specific error messages below if possible
	MAV_CMD_ACK_ERR_X_LAT_OUT_OF_RANGE             = 6 // The X or latitude value is out of range
	MAV_CMD_ACK_ERR_Y_LON_OUT_OF_RANGE             = 7 // The Y or longitude value is out of range
	MAV_CMD_ACK_ERR_Z_ALT_OUT_OF_RANGE             = 8 // The Z or altitude value is out of range
)

// MavParamType (generated enum)
// Specifies the datatype of a MAVLink parameter.
const (
	MAV_PARAM_TYPE_UINT8  = 1  // 8-bit unsigned integer
	MAV_PARAM_TYPE_INT8   = 2  // 8-bit signed integer
	MAV_PARAM_TYPE_UINT16 = 3  // 16-bit unsigned integer
	MAV_PARAM_TYPE_INT16  = 4  // 16-bit signed integer
	MAV_PARAM_TYPE_UINT32 = 5  // 32-bit unsigned integer
	MAV_PARAM_TYPE_INT32  = 6  // 32-bit signed integer
	MAV_PARAM_TYPE_UINT64 = 7  // 64-bit unsigned integer
	MAV_PARAM_TYPE_INT64  = 8  // 64-bit signed integer
	MAV_PARAM_TYPE_REAL32 = 9  // 32-bit floating-point
	MAV_PARAM_TYPE_REAL64 = 10 // 64-bit floating-point
)

// MavParamExtType (generated enum)
// Specifies the datatype of a MAVLink extended parameter.
const (
	MAV_PARAM_EXT_TYPE_UINT8  = 1  // 8-bit unsigned integer
	MAV_PARAM_EXT_TYPE_INT8   = 2  // 8-bit signed integer
	MAV_PARAM_EXT_TYPE_UINT16 = 3  // 16-bit unsigned integer
	MAV_PARAM_EXT_TYPE_INT16  = 4  // 16-bit signed integer
	MAV_PARAM_EXT_TYPE_UINT32 = 5  // 32-bit unsigned integer
	MAV_PARAM_EXT_TYPE_INT32  = 6  // 32-bit signed integer
	MAV_PARAM_EXT_TYPE_UINT64 = 7  // 64-bit unsigned integer
	MAV_PARAM_EXT_TYPE_INT64  = 8  // 64-bit signed integer
	MAV_PARAM_EXT_TYPE_REAL32 = 9  // 32-bit floating-point
	MAV_PARAM_EXT_TYPE_REAL64 = 10 // 64-bit floating-point
	MAV_PARAM_EXT_TYPE_CUSTOM = 11 // Custom Type
)

// MavResult (generated enum)
// Result from a MAVLink command (MAV_CMD)
const (
	MAV_RESULT_ACCEPTED             = 0 // Command is valid (is supported and has valid parameters), and was executed
	MAV_RESULT_TEMPORARILY_REJECTED = 1 // Command is valid, but cannot be executed at this time. This is used to indicate a problem that should be fixed just by waiting (e.g. a state machine is busy, can't arm because have not got GPS lock, etc.). Retrying later should work
	MAV_RESULT_DENIED               = 2 // Command is invalid (is supported but has invalid parameters). Retrying same command and parameters will not work
	MAV_RESULT_UNSUPPORTED          = 3 // Command is not supported (unknown)
	MAV_RESULT_FAILED               = 4 // Command is valid, but execution has failed. This is used to indicate any non-temporary or unexpected problem, i.e. any problem that must be fixed before the command can succeed/be retried. For example, attempting to write a file when out of memory, attempting to arm when sensors are not calibrated, etc
	MAV_RESULT_IN_PROGRESS          = 5 // Command is valid and is being executed. This will be followed by further progress updates, i.e. the component may send further COMMAND_ACK messages with result MAV_RESULT_IN_PROGRESS (at a rate decided by the implementation), and must terminate by sending a COMMAND_ACK message with final result of the operation. The COMMAND_ACK.progress field can be used to indicate the progress of the operation
	MAV_RESULT_CANCELLED            = 6 // Command has been cancelled (as a result of receiving a COMMAND_CANCEL message)
)

// MavMissionResult (generated enum)
// Result of mission operation (in a MISSION_ACK message).
const (
	MAV_MISSION_ACCEPTED            = 0  // mission accepted OK
	MAV_MISSION_ERROR               = 1  // Generic error / not accepting mission commands at all right now
	MAV_MISSION_UNSUPPORTED_FRAME   = 2  // Coordinate frame is not supported
	MAV_MISSION_UNSUPPORTED         = 3  // Command is not supported
	MAV_MISSION_NO_SPACE            = 4  // Mission items exceed storage space
	MAV_MISSION_INVALID             = 5  // One of the parameters has an invalid value
	MAV_MISSION_INVALID_PARAM1      = 6  // param1 has an invalid value
	MAV_MISSION_INVALID_PARAM2      = 7  // param2 has an invalid value
	MAV_MISSION_INVALID_PARAM3      = 8  // param3 has an invalid value
	MAV_MISSION_INVALID_PARAM4      = 9  // param4 has an invalid value
	MAV_MISSION_INVALID_PARAM5_X    = 10 // x / param5 has an invalid value
	MAV_MISSION_INVALID_PARAM6_Y    = 11 // y / param6 has an invalid value
	MAV_MISSION_INVALID_PARAM7      = 12 // z / param7 has an invalid value
	MAV_MISSION_INVALID_SEQUENCE    = 13 // Mission item received out of sequence
	MAV_MISSION_DENIED              = 14 // Not accepting any mission commands from this communication partner
	MAV_MISSION_OPERATION_CANCELLED = 15 // Current mission operation cancelled (e.g. mission upload, mission download)
)

// MavSeverity (generated enum)
// Indicates the severity level, generally used for status messages to indicate their relative urgency. Based on RFC-5424 using expanded definitions at: http://www.kiwisyslog.com/kb/info:-syslog-message-levels/.
const (
	MAV_SEVERITY_EMERGENCY = 0 // System is unusable. This is a "panic" condition
	MAV_SEVERITY_ALERT     = 1 // Action should be taken immediately. Indicates error in non-critical systems
	MAV_SEVERITY_CRITICAL  = 2 // Action must be taken immediately. Indicates failure in a primary system
	MAV_SEVERITY_ERROR     = 3 // Indicates an error in secondary/redundant systems
	MAV_SEVERITY_WARNING   = 4 // Indicates about a possible future error if this is not resolved within a given timeframe. Example would be a low battery warning
	MAV_SEVERITY_NOTICE    = 5 // An unusual event has occurred, though not an error condition. This should be investigated for the root cause
	MAV_SEVERITY_INFO      = 6 // Normal operational messages. Useful for logging. No action is required for these messages
	MAV_SEVERITY_DEBUG     = 7 // Useful non-operational messages that can assist in debugging. These should not occur during normal operation
)

// MavPowerStatus (generated enum)
// Power supply status flags (bitmask)
const (
	MAV_POWER_STATUS_BRICK_VALID                = 1  // main brick power supply valid
	MAV_POWER_STATUS_SERVO_VALID                = 2  // main servo power supply valid for FMU
	MAV_POWER_STATUS_USB_CONNECTED              = 4  // USB power is connected
	MAV_POWER_STATUS_PERIPH_OVERCURRENT         = 8  // peripheral supply is in over-current state
	MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT = 16 // hi-power peripheral supply is in over-current state
	MAV_POWER_STATUS_CHANGED                    = 32 // Power status has changed since boot
)

// SerialControlDev (generated enum)
// SERIAL_CONTROL device types
const (
	SERIAL_CONTROL_DEV_TELEM1 = 0   // First telemetry port
	SERIAL_CONTROL_DEV_TELEM2 = 1   // Second telemetry port
	SERIAL_CONTROL_DEV_GPS1   = 2   // First GPS port
	SERIAL_CONTROL_DEV_GPS2   = 3   // Second GPS port
	SERIAL_CONTROL_DEV_SHELL  = 10  // system shell
	SERIAL_CONTROL_SERIAL0    = 100 // SERIAL0
	SERIAL_CONTROL_SERIAL1    = 101 // SERIAL1
	SERIAL_CONTROL_SERIAL2    = 102 // SERIAL2
	SERIAL_CONTROL_SERIAL3    = 103 // SERIAL3
	SERIAL_CONTROL_SERIAL4    = 104 // SERIAL4
	SERIAL_CONTROL_SERIAL5    = 105 // SERIAL5
	SERIAL_CONTROL_SERIAL6    = 106 // SERIAL6
	SERIAL_CONTROL_SERIAL7    = 107 // SERIAL7
	SERIAL_CONTROL_SERIAL8    = 108 // SERIAL8
	SERIAL_CONTROL_SERIAL9    = 109 // SERIAL9
)

// SerialControlFlag (generated enum)
// SERIAL_CONTROL flags (bitmask)
const (
	SERIAL_CONTROL_FLAG_REPLY     = 1  // Set if this is a reply
	SERIAL_CONTROL_FLAG_RESPOND   = 2  // Set if the sender wants the receiver to send a response as another SERIAL_CONTROL message
	SERIAL_CONTROL_FLAG_EXCLUSIVE = 4  // Set if access to the serial port should be removed from whatever driver is currently using it, giving exclusive access to the SERIAL_CONTROL protocol. The port can be handed back by sending a request without this flag set
	SERIAL_CONTROL_FLAG_BLOCKING  = 8  // Block on writes to the serial port
	SERIAL_CONTROL_FLAG_MULTI     = 16 // Send multiple replies until port is drained
)

// MavDistanceSensor (generated enum)
// Enumeration of distance sensor types
const (
	MAV_DISTANCE_SENSOR_LASER      = 0 // Laser rangefinder, e.g. LightWare SF02/F or PulsedLight units
	MAV_DISTANCE_SENSOR_ULTRASOUND = 1 // Ultrasound rangefinder, e.g. MaxBotix units
	MAV_DISTANCE_SENSOR_INFRARED   = 2 // Infrared rangefinder, e.g. Sharp units
	MAV_DISTANCE_SENSOR_RADAR      = 3 // Radar type, e.g. uLanding units
	MAV_DISTANCE_SENSOR_UNKNOWN    = 4 // Broken or unknown type, e.g. analog units
)

// MavSensorOrientation (generated enum)
// Enumeration of sensor orientation, according to its rotations
const (
	MAV_SENSOR_ROTATION_NONE                     = 0   // Roll: 0, Pitch: 0, Yaw: 0
	MAV_SENSOR_ROTATION_YAW_45                   = 1   // Roll: 0, Pitch: 0, Yaw: 45
	MAV_SENSOR_ROTATION_YAW_90                   = 2   // Roll: 0, Pitch: 0, Yaw: 90
	MAV_SENSOR_ROTATION_YAW_135                  = 3   // Roll: 0, Pitch: 0, Yaw: 135
	MAV_SENSOR_ROTATION_YAW_180                  = 4   // Roll: 0, Pitch: 0, Yaw: 180
	MAV_SENSOR_ROTATION_YAW_225                  = 5   // Roll: 0, Pitch: 0, Yaw: 225
	MAV_SENSOR_ROTATION_YAW_270                  = 6   // Roll: 0, Pitch: 0, Yaw: 270
	MAV_SENSOR_ROTATION_YAW_315                  = 7   // Roll: 0, Pitch: 0, Yaw: 315
	MAV_SENSOR_ROTATION_ROLL_180                 = 8   // Roll: 180, Pitch: 0, Yaw: 0
	MAV_SENSOR_ROTATION_ROLL_180_YAW_45          = 9   // Roll: 180, Pitch: 0, Yaw: 45
	MAV_SENSOR_ROTATION_ROLL_180_YAW_90          = 10  // Roll: 180, Pitch: 0, Yaw: 90
	MAV_SENSOR_ROTATION_ROLL_180_YAW_135         = 11  // Roll: 180, Pitch: 0, Yaw: 135
	MAV_SENSOR_ROTATION_PITCH_180                = 12  // Roll: 0, Pitch: 180, Yaw: 0
	MAV_SENSOR_ROTATION_ROLL_180_YAW_225         = 13  // Roll: 180, Pitch: 0, Yaw: 225
	MAV_SENSOR_ROTATION_ROLL_180_YAW_270         = 14  // Roll: 180, Pitch: 0, Yaw: 270
	MAV_SENSOR_ROTATION_ROLL_180_YAW_315         = 15  // Roll: 180, Pitch: 0, Yaw: 315
	MAV_SENSOR_ROTATION_ROLL_90                  = 16  // Roll: 90, Pitch: 0, Yaw: 0
	MAV_SENSOR_ROTATION_ROLL_90_YAW_45           = 17  // Roll: 90, Pitch: 0, Yaw: 45
	MAV_SENSOR_ROTATION_ROLL_90_YAW_90           = 18  // Roll: 90, Pitch: 0, Yaw: 90
	MAV_SENSOR_ROTATION_ROLL_90_YAW_135          = 19  // Roll: 90, Pitch: 0, Yaw: 135
	MAV_SENSOR_ROTATION_ROLL_270                 = 20  // Roll: 270, Pitch: 0, Yaw: 0
	MAV_SENSOR_ROTATION_ROLL_270_YAW_45          = 21  // Roll: 270, Pitch: 0, Yaw: 45
	MAV_SENSOR_ROTATION_ROLL_270_YAW_90          = 22  // Roll: 270, Pitch: 0, Yaw: 90
	MAV_SENSOR_ROTATION_ROLL_270_YAW_135         = 23  // Roll: 270, Pitch: 0, Yaw: 135
	MAV_SENSOR_ROTATION_PITCH_90                 = 24  // Roll: 0, Pitch: 90, Yaw: 0
	MAV_SENSOR_ROTATION_PITCH_270                = 25  // Roll: 0, Pitch: 270, Yaw: 0
	MAV_SENSOR_ROTATION_PITCH_180_YAW_90         = 26  // Roll: 0, Pitch: 180, Yaw: 90
	MAV_SENSOR_ROTATION_PITCH_180_YAW_270        = 27  // Roll: 0, Pitch: 180, Yaw: 270
	MAV_SENSOR_ROTATION_ROLL_90_PITCH_90         = 28  // Roll: 90, Pitch: 90, Yaw: 0
	MAV_SENSOR_ROTATION_ROLL_180_PITCH_90        = 29  // Roll: 180, Pitch: 90, Yaw: 0
	MAV_SENSOR_ROTATION_ROLL_270_PITCH_90        = 30  // Roll: 270, Pitch: 90, Yaw: 0
	MAV_SENSOR_ROTATION_ROLL_90_PITCH_180        = 31  // Roll: 90, Pitch: 180, Yaw: 0
	MAV_SENSOR_ROTATION_ROLL_270_PITCH_180       = 32  // Roll: 270, Pitch: 180, Yaw: 0
	MAV_SENSOR_ROTATION_ROLL_90_PITCH_270        = 33  // Roll: 90, Pitch: 270, Yaw: 0
	MAV_SENSOR_ROTATION_ROLL_180_PITCH_270       = 34  // Roll: 180, Pitch: 270, Yaw: 0
	MAV_SENSOR_ROTATION_ROLL_270_PITCH_270       = 35  // Roll: 270, Pitch: 270, Yaw: 0
	MAV_SENSOR_ROTATION_ROLL_90_PITCH_180_YAW_90 = 36  // Roll: 90, Pitch: 180, Yaw: 90
	MAV_SENSOR_ROTATION_ROLL_90_YAW_270          = 37  // Roll: 90, Pitch: 0, Yaw: 270
	MAV_SENSOR_ROTATION_ROLL_90_PITCH_68_YAW_293 = 38  // Roll: 90, Pitch: 68, Yaw: 293
	MAV_SENSOR_ROTATION_PITCH_315                = 39  // Pitch: 315
	MAV_SENSOR_ROTATION_ROLL_90_PITCH_315        = 40  // Roll: 90, Pitch: 315
	MAV_SENSOR_ROTATION_CUSTOM                   = 100 // Custom orientation
)

// MavProtocolCapability (generated enum)
// Bitmask of (optional) autopilot capabilities (64 bit). If a bit is set, the autopilot supports this capability.
const (
	MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT                  = 1     // Autopilot supports MISSION float message type
	MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT                    = 2     // Autopilot supports the new param float message type
	MAV_PROTOCOL_CAPABILITY_MISSION_INT                    = 4     // Autopilot supports MISSION_ITEM_INT scaled integer message type
	MAV_PROTOCOL_CAPABILITY_COMMAND_INT                    = 8     // Autopilot supports COMMAND_INT scaled integer message type
	MAV_PROTOCOL_CAPABILITY_PARAM_UNION                    = 16    // Autopilot supports the new param union message type
	MAV_PROTOCOL_CAPABILITY_FTP                            = 32    // Autopilot supports the new FILE_TRANSFER_PROTOCOL message type
	MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET            = 64    // Autopilot supports commanding attitude offboard
	MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED  = 128   // Autopilot supports commanding position and velocity targets in local NED frame
	MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT = 256   // Autopilot supports commanding position and velocity targets in global scaled integers
	MAV_PROTOCOL_CAPABILITY_TERRAIN                        = 512   // Autopilot supports terrain protocol / data handling
	MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET            = 1024  // Autopilot supports direct actuator control
	MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION             = 2048  // Autopilot supports the flight termination command
	MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION            = 4096  // Autopilot supports onboard compass calibration
	MAV_PROTOCOL_CAPABILITY_MAVLINK2                       = 8192  // Autopilot supports MAVLink version 2
	MAV_PROTOCOL_CAPABILITY_MISSION_FENCE                  = 16384 // Autopilot supports mission fence protocol
	MAV_PROTOCOL_CAPABILITY_MISSION_RALLY                  = 32768 // Autopilot supports mission rally point protocol
	MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION             = 65536 // Autopilot supports the flight information protocol
)

// MavMissionType (generated enum)
// Type of mission items being requested/sent in mission protocol.
const (
	MAV_MISSION_TYPE_MISSION = 0   // Items are mission commands for main mission
	MAV_MISSION_TYPE_FENCE   = 1   // Specifies GeoFence area(s). Items are MAV_CMD_NAV_FENCE_ GeoFence items
	MAV_MISSION_TYPE_RALLY   = 2   // Specifies the rally points for the vehicle. Rally points are alternative RTL points. Items are MAV_CMD_NAV_RALLY_POINT rally point items
	MAV_MISSION_TYPE_ALL     = 255 // Only used in MISSION_CLEAR_ALL to clear all mission types
)

// MavEstimatorType (generated enum)
// Enumeration of estimator types
const (
	MAV_ESTIMATOR_TYPE_UNKNOWN   = 0 // Unknown type of the estimator
	MAV_ESTIMATOR_TYPE_NAIVE     = 1 // This is a naive estimator without any real covariance feedback
	MAV_ESTIMATOR_TYPE_VISION    = 2 // Computer vision based estimate. Might be up to scale
	MAV_ESTIMATOR_TYPE_VIO       = 3 // Visual-inertial estimate
	MAV_ESTIMATOR_TYPE_GPS       = 4 // Plain GPS estimate
	MAV_ESTIMATOR_TYPE_GPS_INS   = 5 // Estimator integrating GPS and inertial sensing
	MAV_ESTIMATOR_TYPE_MOCAP     = 6 // Estimate from external motion capturing system
	MAV_ESTIMATOR_TYPE_LIDAR     = 7 // Estimator based on lidar sensor input
	MAV_ESTIMATOR_TYPE_AUTOPILOT = 8 // Estimator on autopilot
)

// MavBatteryType (generated enum)
// Enumeration of battery types
const (
	MAV_BATTERY_TYPE_UNKNOWN = 0 // Not specified
	MAV_BATTERY_TYPE_LIPO    = 1 // Lithium polymer battery
	MAV_BATTERY_TYPE_LIFE    = 2 // Lithium-iron-phosphate battery
	MAV_BATTERY_TYPE_LION    = 3 // Lithium-ION battery
	MAV_BATTERY_TYPE_NIMH    = 4 // Nickel metal hydride battery
)

// MavBatteryFunction (generated enum)
// Enumeration of battery functions
const (
	MAV_BATTERY_FUNCTION_UNKNOWN    = 0 // Battery function is unknown
	MAV_BATTERY_FUNCTION_ALL        = 1 // Battery supports all flight systems
	MAV_BATTERY_FUNCTION_PROPULSION = 2 // Battery for the propulsion system
	MAV_BATTERY_FUNCTION_AVIONICS   = 3 // Avionics battery
	MAV_BATTERY_TYPE_PAYLOAD        = 4 // Payload battery
)

// MavBatteryChargeState (generated enum)
// Enumeration for battery charge states.
const (
	MAV_BATTERY_CHARGE_STATE_UNDEFINED = 0 // Low battery state is not provided
	MAV_BATTERY_CHARGE_STATE_OK        = 1 // Battery is not in low state. Normal operation
	MAV_BATTERY_CHARGE_STATE_LOW       = 2 // Battery state is low, warn and monitor close
	MAV_BATTERY_CHARGE_STATE_CRITICAL  = 3 // Battery state is critical, return or abort immediately
	MAV_BATTERY_CHARGE_STATE_EMERGENCY = 4 // Battery state is too low for ordinary abort sequence. Perform fastest possible emergency stop to prevent damage
	MAV_BATTERY_CHARGE_STATE_FAILED    = 5 // Battery failed, damage unavoidable. Possible causes (faults) are listed in MAV_BATTERY_FAULT
	MAV_BATTERY_CHARGE_STATE_UNHEALTHY = 6 // Battery is diagnosed to be defective or an error occurred, usage is discouraged / prohibited. Possible causes (faults) are listed in MAV_BATTERY_FAULT
	MAV_BATTERY_CHARGE_STATE_CHARGING  = 7 // Battery is charging
)

// MavBatteryMode (generated enum)
// Battery mode. Note, the normal operation mode (i.e. when flying) should be reported as MAV_BATTERY_MODE_UNKNOWN to allow message trimming in normal flight.
const (
	MAV_BATTERY_MODE_UNKNOWN          = 0 // Battery mode not supported/unknown battery mode/normal operation
	MAV_BATTERY_MODE_AUTO_DISCHARGING = 1 // Battery is auto discharging (towards storage level)
	MAV_BATTERY_MODE_HOT_SWAP         = 2 // Battery in hot-swap mode (current limited to prevent spikes that might damage sensitive electrical circuits)
)

// MavBatteryFault (generated enum)
// Smart battery supply status/fault flags (bitmask) for health indication. The battery must also report either MAV_BATTERY_CHARGE_STATE_FAILED or MAV_BATTERY_CHARGE_STATE_UNHEALTHY if any of these are set.
const (
	MAV_BATTERY_FAULT_DEEP_DISCHARGE       = 1  // Battery has deep discharged
	MAV_BATTERY_FAULT_SPIKES               = 2  // Voltage spikes
	MAV_BATTERY_FAULT_CELL_FAIL            = 4  // One or more cells have failed. Battery should also report MAV_BATTERY_CHARGE_STATE_FAILE (and should not be used)
	MAV_BATTERY_FAULT_OVER_CURRENT         = 8  // Over-current fault
	MAV_BATTERY_FAULT_OVER_TEMPERATURE     = 16 // Over-temperature fault
	MAV_BATTERY_FAULT_UNDER_TEMPERATURE    = 32 // Under-temperature fault
	MAV_BATTERY_FAULT_INCOMPATIBLE_VOLTAGE = 64 // Vehicle voltage is not compatible with this battery (batteries on same power rail should have similar voltage)
)

// MavGeneratorStatusFlag (generated enum)
// Flags to report status/failure cases for a power generator (used in GENERATOR_STATUS). Note that FAULTS are conditions that cause the generator to fail. Warnings are conditions that require attention before the next use (they indicate the system is not operating properly).
const (
	MAV_GENERATOR_STATUS_FLAG_OFF                              = 1       // Generator is off
	MAV_GENERATOR_STATUS_FLAG_READY                            = 2       // Generator is ready to start generating power
	MAV_GENERATOR_STATUS_FLAG_GENERATING                       = 4       // Generator is generating power
	MAV_GENERATOR_STATUS_FLAG_CHARGING                         = 8       // Generator is charging the batteries (generating enough power to charge and provide the load)
	MAV_GENERATOR_STATUS_FLAG_REDUCED_POWER                    = 16      // Generator is operating at a reduced maximum power
	MAV_GENERATOR_STATUS_FLAG_MAXPOWER                         = 32      // Generator is providing the maximum output
	MAV_GENERATOR_STATUS_FLAG_OVERTEMP_WARNING                 = 64      // Generator is near the maximum operating temperature, cooling is insufficient
	MAV_GENERATOR_STATUS_FLAG_OVERTEMP_FAULT                   = 128     // Generator hit the maximum operating temperature and shutdown
	MAV_GENERATOR_STATUS_FLAG_ELECTRONICS_OVERTEMP_WARNING     = 256     // Power electronics are near the maximum operating temperature, cooling is insufficient
	MAV_GENERATOR_STATUS_FLAG_ELECTRONICS_OVERTEMP_FAULT       = 512     // Power electronics hit the maximum operating temperature and shutdown
	MAV_GENERATOR_STATUS_FLAG_ELECTRONICS_FAULT                = 1024    // Power electronics experienced a fault and shutdown
	MAV_GENERATOR_STATUS_FLAG_POWERSOURCE_FAULT                = 2048    // The power source supplying the generator failed e.g. mechanical generator stopped, tether is no longer providing power, solar cell is in shade, hydrogen reaction no longer happening
	MAV_GENERATOR_STATUS_FLAG_COMMUNICATION_WARNING            = 4096    // Generator controller having communication problems
	MAV_GENERATOR_STATUS_FLAG_COOLING_WARNING                  = 8192    // Power electronic or generator cooling system error
	MAV_GENERATOR_STATUS_FLAG_POWER_RAIL_FAULT                 = 16384   // Generator controller power rail experienced a fault
	MAV_GENERATOR_STATUS_FLAG_OVERCURRENT_FAULT                = 32768   // Generator controller exceeded the overcurrent threshold and shutdown to prevent damage
	MAV_GENERATOR_STATUS_FLAG_BATTERY_OVERCHARGE_CURRENT_FAULT = 65536   // Generator controller detected a high current going into the batteries and shutdown to prevent battery damage
	MAV_GENERATOR_STATUS_FLAG_OVERVOLTAGE_FAULT                = 131072  // Generator controller exceeded it's overvoltage threshold and shutdown to prevent it exceeding the voltage rating
	MAV_GENERATOR_STATUS_FLAG_BATTERY_UNDERVOLT_FAULT          = 262144  // Batteries are under voltage (generator will not start)
	MAV_GENERATOR_STATUS_FLAG_START_INHIBITED                  = 524288  // Generator start is inhibited by e.g. a safety switch
	MAV_GENERATOR_STATUS_FLAG_MAINTENANCE_REQUIRED             = 1048576 // Generator requires maintenance
	MAV_GENERATOR_STATUS_FLAG_WARMING_UP                       = 2097152 // Generator is not ready to generate yet
	MAV_GENERATOR_STATUS_FLAG_IDLE                             = 4194304 // Generator is idle
)

// MavVtolState (generated enum)
// Enumeration of VTOL states
const (
	MAV_VTOL_STATE_UNDEFINED        = 0 // MAV is not configured as VTOL
	MAV_VTOL_STATE_TRANSITION_TO_FW = 1 // VTOL is in transition from multicopter to fixed-wing
	MAV_VTOL_STATE_TRANSITION_TO_MC = 2 // VTOL is in transition from fixed-wing to multicopter
	MAV_VTOL_STATE_MC               = 3 // VTOL is in multicopter state
	MAV_VTOL_STATE_FW               = 4 // VTOL is in fixed-wing state
)

// MavLandedState (generated enum)
// Enumeration of landed detector states
const (
	MAV_LANDED_STATE_UNDEFINED = 0 // MAV landed state is unknown
	MAV_LANDED_STATE_ON_GROUND = 1 // MAV is landed (on ground)
	MAV_LANDED_STATE_IN_AIR    = 2 // MAV is in air
	MAV_LANDED_STATE_TAKEOFF   = 3 // MAV currently taking off
	MAV_LANDED_STATE_LANDING   = 4 // MAV currently landing
)

// AdsbAltitudeType (generated enum)
// Enumeration of the ADSB altimeter types
const (
	ADSB_ALTITUDE_TYPE_PRESSURE_QNH = 0 // Altitude reported from a Baro source using QNH reference
	ADSB_ALTITUDE_TYPE_GEOMETRIC    = 1 // Altitude reported from a GNSS source
)

// AdsbEmitterType (generated enum)
// ADSB classification for the type of vehicle emitting the transponder signal
const (
	ADSB_EMITTER_TYPE_NO_INFO           = 0  //
	ADSB_EMITTER_TYPE_LIGHT             = 1  //
	ADSB_EMITTER_TYPE_SMALL             = 2  //
	ADSB_EMITTER_TYPE_LARGE             = 3  //
	ADSB_EMITTER_TYPE_HIGH_VORTEX_LARGE = 4  //
	ADSB_EMITTER_TYPE_HEAVY             = 5  //
	ADSB_EMITTER_TYPE_HIGHLY_MANUV      = 6  //
	ADSB_EMITTER_TYPE_ROTOCRAFT         = 7  //
	ADSB_EMITTER_TYPE_UNASSIGNED        = 8  //
	ADSB_EMITTER_TYPE_GLIDER            = 9  //
	ADSB_EMITTER_TYPE_LIGHTER_AIR       = 10 //
	ADSB_EMITTER_TYPE_PARACHUTE         = 11 //
	ADSB_EMITTER_TYPE_ULTRA_LIGHT       = 12 //
	ADSB_EMITTER_TYPE_UNASSIGNED2       = 13 //
	ADSB_EMITTER_TYPE_UAV               = 14 //
	ADSB_EMITTER_TYPE_SPACE             = 15 //
	ADSB_EMITTER_TYPE_UNASSGINED3       = 16 //
	ADSB_EMITTER_TYPE_EMERGENCY_SURFACE = 17 //
	ADSB_EMITTER_TYPE_SERVICE_SURFACE   = 18 //
	ADSB_EMITTER_TYPE_POINT_OBSTACLE    = 19 //
)

// AdsbFlags (generated enum)
// These flags indicate status such as data validity of each data source. Set = data valid
const (
	ADSB_FLAGS_VALID_COORDS            = 1     //
	ADSB_FLAGS_VALID_ALTITUDE          = 2     //
	ADSB_FLAGS_VALID_HEADING           = 4     //
	ADSB_FLAGS_VALID_VELOCITY          = 8     //
	ADSB_FLAGS_VALID_CALLSIGN          = 16    //
	ADSB_FLAGS_VALID_SQUAWK            = 32    //
	ADSB_FLAGS_SIMULATED               = 64    //
	ADSB_FLAGS_VERTICAL_VELOCITY_VALID = 128   //
	ADSB_FLAGS_BARO_VALID              = 256   //
	ADSB_FLAGS_SOURCE_UAT              = 32768 //
)

// MavDoRepositionFlags (generated enum)
// Bitmap of options for the MAV_CMD_DO_REPOSITION
const (
	MAV_DO_REPOSITION_FLAGS_CHANGE_MODE = 1 // The aircraft should immediately transition into guided. This should not be set for follow me applications
)

// EstimatorStatusFlags (generated enum)
// Flags in ESTIMATOR_STATUS message
const (
	ESTIMATOR_ATTITUDE           = 1    // True if the attitude estimate is good
	ESTIMATOR_VELOCITY_HORIZ     = 2    // True if the horizontal velocity estimate is good
	ESTIMATOR_VELOCITY_VERT      = 4    // True if the  vertical velocity estimate is good
	ESTIMATOR_POS_HORIZ_REL      = 8    // True if the horizontal position (relative) estimate is good
	ESTIMATOR_POS_HORIZ_ABS      = 16   // True if the horizontal position (absolute) estimate is good
	ESTIMATOR_POS_VERT_ABS       = 32   // True if the vertical position (absolute) estimate is good
	ESTIMATOR_POS_VERT_AGL       = 64   // True if the vertical position (above ground) estimate is good
	ESTIMATOR_CONST_POS_MODE     = 128  // True if the EKF is in a constant position mode and is not using external measurements (eg GPS or optical flow)
	ESTIMATOR_PRED_POS_HORIZ_REL = 256  // True if the EKF has sufficient data to enter a mode that will provide a (relative) position estimate
	ESTIMATOR_PRED_POS_HORIZ_ABS = 512  // True if the EKF has sufficient data to enter a mode that will provide a (absolute) position estimate
	ESTIMATOR_GPS_GLITCH         = 1024 // True if the EKF has detected a GPS glitch
	ESTIMATOR_ACCEL_ERROR        = 2048 // True if the EKF has detected bad accelerometer data
)

// MotorTestOrder (generated enum)
//
const (
	MOTOR_TEST_ORDER_DEFAULT  = 0 // default autopilot motor test method
	MOTOR_TEST_ORDER_SEQUENCE = 1 // motor numbers are specified as their index in a predefined vehicle-specific sequence
	MOTOR_TEST_ORDER_BOARD    = 2 // motor numbers are specified as the output as labeled on the board
)

// MotorTestThrottleType (generated enum)
//
const (
	MOTOR_TEST_THROTTLE_PERCENT = 0 // throttle as a percentage from 0 ~ 100
	MOTOR_TEST_THROTTLE_PWM     = 1 // throttle as an absolute PWM value (normally in range of 1000~2000)
	MOTOR_TEST_THROTTLE_PILOT   = 2 // throttle pass-through from pilot's transmitter
	MOTOR_TEST_COMPASS_CAL      = 3 // per-motor compass calibration test
)

// GpsInputIgnoreFlags (generated enum)
//
const (
	GPS_INPUT_IGNORE_FLAG_ALT                 = 1   // ignore altitude field
	GPS_INPUT_IGNORE_FLAG_HDOP                = 2   // ignore hdop field
	GPS_INPUT_IGNORE_FLAG_VDOP                = 4   // ignore vdop field
	GPS_INPUT_IGNORE_FLAG_VEL_HORIZ           = 8   // ignore horizontal velocity field (vn and ve)
	GPS_INPUT_IGNORE_FLAG_VEL_VERT            = 16  // ignore vertical velocity field (vd)
	GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY      = 32  // ignore speed accuracy field
	GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY = 64  // ignore horizontal accuracy field
	GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY   = 128 // ignore vertical accuracy field
)

// MavCollisionAction (generated enum)
// Possible actions an aircraft can take to avoid a collision.
const (
	MAV_COLLISION_ACTION_NONE               = 0 // Ignore any potential collisions
	MAV_COLLISION_ACTION_REPORT             = 1 // Report potential collision
	MAV_COLLISION_ACTION_ASCEND_OR_DESCEND  = 2 // Ascend or Descend to avoid threat
	MAV_COLLISION_ACTION_MOVE_HORIZONTALLY  = 3 // Move horizontally to avoid threat
	MAV_COLLISION_ACTION_MOVE_PERPENDICULAR = 4 // Aircraft to move perpendicular to the collision's velocity vector
	MAV_COLLISION_ACTION_RTL                = 5 // Aircraft to fly directly back to its launch point
	MAV_COLLISION_ACTION_HOVER              = 6 // Aircraft to stop in place
)

// MavCollisionThreatLevel (generated enum)
// Aircraft-rated danger from this threat.
const (
	MAV_COLLISION_THREAT_LEVEL_NONE = 0 // Not a threat
	MAV_COLLISION_THREAT_LEVEL_LOW  = 1 // Craft is mildly concerned about this threat
	MAV_COLLISION_THREAT_LEVEL_HIGH = 2 // Craft is panicking, and may take actions to avoid threat
)

// MavCollisionSrc (generated enum)
// Source of information about this collision.
const (
	MAV_COLLISION_SRC_ADSB                   = 0 // ID field references ADSB_VEHICLE packets
	MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT = 1 // ID field references MAVLink SRC ID
)

// GpsFixType (generated enum)
// Type of GPS fix
const (
	GPS_FIX_TYPE_NO_GPS    = 0 // No GPS connected
	GPS_FIX_TYPE_NO_FIX    = 1 // No position information, GPS is connected
	GPS_FIX_TYPE_2D_FIX    = 2 // 2D position
	GPS_FIX_TYPE_3D_FIX    = 3 // 3D position
	GPS_FIX_TYPE_DGPS      = 4 // DGPS/SBAS aided 3D position
	GPS_FIX_TYPE_RTK_FLOAT = 5 // RTK float, 3D position
	GPS_FIX_TYPE_RTK_FIXED = 6 // RTK Fixed, 3D position
	GPS_FIX_TYPE_STATIC    = 7 // Static fixed, typically used for base stations
	GPS_FIX_TYPE_PPP       = 8 // PPP, 3D position
)

// RtkBaselineCoordinateSystem (generated enum)
// RTK GPS baseline coordinate system, used for RTK corrections
const (
	RTK_BASELINE_COORDINATE_SYSTEM_ECEF = 0 // Earth-centered, Earth-fixed
	RTK_BASELINE_COORDINATE_SYSTEM_NED  = 1 // RTK basestation centered, north, east, down
)

// LandingTargetType (generated enum)
// Type of landing target
const (
	LANDING_TARGET_TYPE_LIGHT_BEACON    = 0 // Landing target signaled by light beacon (ex: IR-LOCK)
	LANDING_TARGET_TYPE_RADIO_BEACON    = 1 // Landing target signaled by radio beacon (ex: ILS, NDB)
	LANDING_TARGET_TYPE_VISION_FIDUCIAL = 2 // Landing target represented by a fiducial marker (ex: ARTag)
	LANDING_TARGET_TYPE_VISION_OTHER    = 3 // Landing target represented by a pre-defined visual shape/feature (ex: X-marker, H-marker, square)
)

// VtolTransitionHeading (generated enum)
// Direction of VTOL transition
const (
	VTOL_TRANSITION_HEADING_VEHICLE_DEFAULT = 0 // Respect the heading configuration of the vehicle
	VTOL_TRANSITION_HEADING_NEXT_WAYPOINT   = 1 // Use the heading pointing towards the next waypoint
	VTOL_TRANSITION_HEADING_TAKEOFF         = 2 // Use the heading on takeoff (while sitting on the ground)
	VTOL_TRANSITION_HEADING_SPECIFIED       = 3 // Use the specified heading in parameter 4
	VTOL_TRANSITION_HEADING_ANY             = 4 // Use the current heading when reaching takeoff altitude (potentially facing the wind when weather-vaning is active)
)

// CameraCapFlags (generated enum)
// Camera capability flags (Bitmap)
const (
	CAMERA_CAP_FLAGS_CAPTURE_VIDEO                   = 1    // Camera is able to record video
	CAMERA_CAP_FLAGS_CAPTURE_IMAGE                   = 2    // Camera is able to capture images
	CAMERA_CAP_FLAGS_HAS_MODES                       = 4    // Camera has separate Video and Image/Photo modes (MAV_CMD_SET_CAMERA_MODE)
	CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE = 8    // Camera can capture images while in video mode
	CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE = 16   // Camera can capture videos while in Photo/Image mode
	CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE           = 32   // Camera has image survey mode (MAV_CMD_SET_CAMERA_MODE)
	CAMERA_CAP_FLAGS_HAS_BASIC_ZOOM                  = 64   // Camera has basic zoom control (MAV_CMD_SET_CAMERA_ZOOM)
	CAMERA_CAP_FLAGS_HAS_BASIC_FOCUS                 = 128  // Camera has basic focus control (MAV_CMD_SET_CAMERA_FOCUS)
	CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM                = 256  // Camera has video streaming capabilities (request VIDEO_STREAM_INFORMATION with MAV_CMD_REQUEST_MESSAGE for video streaming info)
	CAMERA_CAP_FLAGS_HAS_TRACKING_POINT              = 512  // Camera supports tracking of a point on the camera view
	CAMERA_CAP_FLAGS_HAS_TRACKING_RECTANGLE          = 1024 // Camera supports tracking of a selection rectangle on the camera view
	CAMERA_CAP_FLAGS_HAS_TRACKING_GEO_STATUS         = 2048 // Camera supports tracking geo status (CAMERA_TRACKING_GEO_STATUS)
)

// VideoStreamStatusFlags (generated enum)
// Stream status flags (Bitmap)
const (
	VIDEO_STREAM_STATUS_FLAGS_RUNNING = 1 // Stream is active (running)
	VIDEO_STREAM_STATUS_FLAGS_THERMAL = 2 // Stream is thermal imaging
)

// VideoStreamType (generated enum)
// Video stream types
const (
	VIDEO_STREAM_TYPE_RTSP         = 0 // Stream is RTSP
	VIDEO_STREAM_TYPE_RTPUDP       = 1 // Stream is RTP UDP (URI gives the port number)
	VIDEO_STREAM_TYPE_TCP_MPEG     = 2 // Stream is MPEG on TCP
	VIDEO_STREAM_TYPE_MPEG_TS_H264 = 3 // Stream is h.264 on MPEG TS (URI gives the port number)
)

// CameraTrackingStatusFlags (generated enum)
// Camera tracking status flags
const (
	CAMERA_TRACKING_STATUS_FLAGS_IDLE   = 0 // Camera is not tracking
	CAMERA_TRACKING_STATUS_FLAGS_ACTIVE = 1 // Camera is tracking
	CAMERA_TRACKING_STATUS_FLAGS_ERROR  = 2 // Camera tracking in error state
)

// CameraTrackingMode (generated enum)
// Camera tracking modes
const (
	CAMERA_TRACKING_NONE      = 0 // Not tracking
	CAMERA_TRACKING_POINT     = 1 // Target is a point
	CAMERA_TRACKING_RECTANGLE = 2 // Target is a rectangle
)

// CameraTrackingTargetData (generated enum)
// Camera tracking target data (shows where tracked target is within image)
const (
	CAMERA_TRACKING_TARGET_NONE      = 0 // No target data
	CAMERA_TRACKING_TARGET_EMBEDDED  = 1 // Target data embedded in image data (proprietary)
	CAMERA_TRACKING_TARGET_RENDERED  = 2 // Target data rendered in image
	CAMERA_TRACKING_TARGET_IN_STATUS = 4 // Target data within status message (Point or Rectangle)
)

// CameraZoomType (generated enum)
// Zoom types for MAV_CMD_SET_CAMERA_ZOOM
const (
	ZOOM_TYPE_STEP         = 0 // Zoom one step increment (-1 for wide, 1 for tele)
	ZOOM_TYPE_CONTINUOUS   = 1 // Continuous zoom up/down until stopped (-1 for wide, 1 for tele, 0 to stop zooming)
	ZOOM_TYPE_RANGE        = 2 // Zoom value as proportion of full camera range (a value between 0.0 and 100.0)
	ZOOM_TYPE_FOCAL_LENGTH = 3 // Zoom value/variable focal length in milimetres. Note that there is no message to get the valid zoom range of the camera, so this can type can only be used for cameras where the zoom range is known (implying that this cannot reliably be used in a GCS for an arbitrary camera)
)

// SetFocusType (generated enum)
// Focus types for MAV_CMD_SET_CAMERA_FOCUS
const (
	FOCUS_TYPE_STEP       = 0 // Focus one step increment (-1 for focusing in, 1 for focusing out towards infinity)
	FOCUS_TYPE_CONTINUOUS = 1 // Continuous focus up/down until stopped (-1 for focusing in, 1 for focusing out towards infinity, 0 to stop focusing)
	FOCUS_TYPE_RANGE      = 2 // Focus value as proportion of full camera focus range (a value between 0.0 and 100.0)
	FOCUS_TYPE_METERS     = 3 // Focus value in metres. Note that there is no message to get the valid focus range of the camera, so this can type can only be used for cameras where the range is known (implying that this cannot reliably be used in a GCS for an arbitrary camera)
)

// ParamAck (generated enum)
// Result from PARAM_EXT_SET message (or a PARAM_SET within a transaction).
const (
	PARAM_ACK_ACCEPTED          = 0 // Parameter value ACCEPTED and SET
	PARAM_ACK_VALUE_UNSUPPORTED = 1 // Parameter value UNKNOWN/UNSUPPORTED
	PARAM_ACK_FAILED            = 2 // Parameter failed to set
	PARAM_ACK_IN_PROGRESS       = 3 // Parameter value received but not yet set/accepted. A subsequent PARAM_ACK_TRANSACTION or PARAM_EXT_ACK with the final result will follow once operation is completed. This is returned immediately for parameters that take longer to set, indicating taht the the parameter was recieved and does not need to be resent
)

// CameraMode (generated enum)
// Camera Modes.
const (
	CAMERA_MODE_IMAGE        = 0 // Camera is in image/photo capture mode
	CAMERA_MODE_VIDEO        = 1 // Camera is in video capture mode
	CAMERA_MODE_IMAGE_SURVEY = 2 // Camera is in image survey capture mode. It allows for camera controller to do specific settings for surveys
)

// MavArmAuthDeniedReason (generated enum)
//
const (
	MAV_ARM_AUTH_DENIED_REASON_GENERIC          = 0 // Not a specific reason
	MAV_ARM_AUTH_DENIED_REASON_NONE             = 1 // Authorizer will send the error as string to GCS
	MAV_ARM_AUTH_DENIED_REASON_INVALID_WAYPOINT = 2 // At least one waypoint have a invalid value
	MAV_ARM_AUTH_DENIED_REASON_TIMEOUT          = 3 // Timeout in the authorizer process(in case it depends on network)
	MAV_ARM_AUTH_DENIED_REASON_AIRSPACE_IN_USE  = 4 // Airspace of the mission in use by another vehicle, second result parameter can have the waypoint id that caused it to be denied
	MAV_ARM_AUTH_DENIED_REASON_BAD_WEATHER      = 5 // Weather is not good to fly
)

// RcType (generated enum)
// RC type
const (
	RC_TYPE_SPEKTRUM_DSM2 = 0 // Spektrum DSM2
	RC_TYPE_SPEKTRUM_DSMX = 1 // Spektrum DSMX
)

// PositionTargetTypemask (generated enum)
// Bitmap to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 9 is set the floats afx afy afz should be interpreted as force instead of acceleration.
const (
	POSITION_TARGET_TYPEMASK_X_IGNORE        = 1    // Ignore position x
	POSITION_TARGET_TYPEMASK_Y_IGNORE        = 2    // Ignore position y
	POSITION_TARGET_TYPEMASK_Z_IGNORE        = 4    // Ignore position z
	POSITION_TARGET_TYPEMASK_VX_IGNORE       = 8    // Ignore velocity x
	POSITION_TARGET_TYPEMASK_VY_IGNORE       = 16   // Ignore velocity y
	POSITION_TARGET_TYPEMASK_VZ_IGNORE       = 32   // Ignore velocity z
	POSITION_TARGET_TYPEMASK_AX_IGNORE       = 64   // Ignore acceleration x
	POSITION_TARGET_TYPEMASK_AY_IGNORE       = 128  // Ignore acceleration y
	POSITION_TARGET_TYPEMASK_AZ_IGNORE       = 256  // Ignore acceleration z
	POSITION_TARGET_TYPEMASK_FORCE_SET       = 512  // Use force instead of acceleration
	POSITION_TARGET_TYPEMASK_YAW_IGNORE      = 1024 // Ignore yaw
	POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE = 2048 // Ignore yaw rate
)

// AttitudeTargetTypemask (generated enum)
// Bitmap to indicate which dimensions should be ignored by the vehicle: a value of 0b00000000 indicates that none of the setpoint dimensions should be ignored.
const (
	ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE  = 1   // Ignore body roll rate
	ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE = 2   // Ignore body pitch rate
	ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE   = 4   // Ignore body yaw rate
	ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE        = 64  // Ignore throttle
	ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE        = 128 // Ignore attitude
)

// UtmFlightState (generated enum)
// Airborne status of UAS.
const (
	UTM_FLIGHT_STATE_UNKNOWN   = 1  // The flight state can't be determined
	UTM_FLIGHT_STATE_GROUND    = 2  // UAS on ground
	UTM_FLIGHT_STATE_AIRBORNE  = 3  // UAS airborne
	UTM_FLIGHT_STATE_EMERGENCY = 16 // UAS is in an emergency flight state
	UTM_FLIGHT_STATE_NOCTRL    = 32 // UAS has no active controls
)

// UtmDataAvailFlags (generated enum)
// Flags for the global position report.
const (
	UTM_DATA_AVAIL_FLAGS_TIME_VALID                  = 1   // The field time contains valid data
	UTM_DATA_AVAIL_FLAGS_UAS_ID_AVAILABLE            = 2   // The field uas_id contains valid data
	UTM_DATA_AVAIL_FLAGS_POSITION_AVAILABLE          = 4   // The fields lat, lon and h_acc contain valid data
	UTM_DATA_AVAIL_FLAGS_ALTITUDE_AVAILABLE          = 8   // The fields alt and v_acc contain valid data
	UTM_DATA_AVAIL_FLAGS_RELATIVE_ALTITUDE_AVAILABLE = 16  // The field relative_alt contains valid data
	UTM_DATA_AVAIL_FLAGS_HORIZONTAL_VELO_AVAILABLE   = 32  // The fields vx and vy contain valid data
	UTM_DATA_AVAIL_FLAGS_VERTICAL_VELO_AVAILABLE     = 64  // The field vz contains valid data
	UTM_DATA_AVAIL_FLAGS_NEXT_WAYPOINT_AVAILABLE     = 128 // The fields next_lat, next_lon and next_alt contain valid data
)

// CellularNetworkRadioType (generated enum)
// Cellular network radio type
const (
	CELLULAR_NETWORK_RADIO_TYPE_NONE  = 0 //
	CELLULAR_NETWORK_RADIO_TYPE_GSM   = 1 //
	CELLULAR_NETWORK_RADIO_TYPE_CDMA  = 2 //
	CELLULAR_NETWORK_RADIO_TYPE_WCDMA = 3 //
	CELLULAR_NETWORK_RADIO_TYPE_LTE   = 4 //
)

// CellularStatusFlag (generated enum)
// These flags encode the cellular network status
const (
	CELLULAR_STATUS_FLAG_UNKNOWN       = 0  // State unknown or not reportable
	CELLULAR_STATUS_FLAG_FAILED        = 1  // Modem is unusable
	CELLULAR_STATUS_FLAG_INITIALIZING  = 2  // Modem is being initialized
	CELLULAR_STATUS_FLAG_LOCKED        = 3  // Modem is locked
	CELLULAR_STATUS_FLAG_DISABLED      = 4  // Modem is not enabled and is powered down
	CELLULAR_STATUS_FLAG_DISABLING     = 5  // Modem is currently transitioning to the CELLULAR_STATUS_FLAG_DISABLED state
	CELLULAR_STATUS_FLAG_ENABLING      = 6  // Modem is currently transitioning to the CELLULAR_STATUS_FLAG_ENABLED state
	CELLULAR_STATUS_FLAG_ENABLED       = 7  // Modem is enabled and powered on but not registered with a network provider and not available for data connections
	CELLULAR_STATUS_FLAG_SEARCHING     = 8  // Modem is searching for a network provider to register
	CELLULAR_STATUS_FLAG_REGISTERED    = 9  // Modem is registered with a network provider, and data connections and messaging may be available for use
	CELLULAR_STATUS_FLAG_DISCONNECTING = 10 // Modem is disconnecting and deactivating the last active packet data bearer. This state will not be entered if more than one packet data bearer is active and one of the active bearers is deactivated
	CELLULAR_STATUS_FLAG_CONNECTING    = 11 // Modem is activating and connecting the first packet data bearer. Subsequent bearer activations when another bearer is already active do not cause this state to be entered
	CELLULAR_STATUS_FLAG_CONNECTED     = 12 // One or more packet data bearers is active and connected
)

// CellularNetworkFailedReason (generated enum)
// These flags are used to diagnose the failure state of CELLULAR_STATUS
const (
	CELLULAR_NETWORK_FAILED_REASON_NONE        = 0 // No error
	CELLULAR_NETWORK_FAILED_REASON_UNKNOWN     = 1 // Error state is unknown
	CELLULAR_NETWORK_FAILED_REASON_SIM_MISSING = 2 // SIM is required for the modem but missing
	CELLULAR_NETWORK_FAILED_REASON_SIM_ERROR   = 3 // SIM is available, but not usuable for connection
)

// PrecisionLandMode (generated enum)
// Precision land modes (used in MAV_CMD_NAV_LAND).
const (
	PRECISION_LAND_MODE_DISABLED      = 0 // Normal (non-precision) landing
	PRECISION_LAND_MODE_OPPORTUNISTIC = 1 // Use precision landing if beacon detected when land command accepted, otherwise land normally
	PRECISION_LAND_MODE_REQUIRED      = 2 // Use precision landing, searching for beacon if not found when land command accepted (land normally if beacon cannot be found)
)

// ParachuteAction (generated enum)
// Parachute actions. Trigger release and enable/disable auto-release.
const (
	PARACHUTE_DISABLE = 0 // Disable auto-release of parachute (i.e. release triggered by crash detectors)
	PARACHUTE_ENABLE  = 1 // Enable auto-release of parachute
	PARACHUTE_RELEASE = 2 // Release parachute and kill motors
)

// MavTunnelPayloadType (generated enum)
//
const (
	MAV_TUNNEL_PAYLOAD_TYPE_UNKNOWN           = 0   // Encoding of payload unknown
	MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED0 = 200 // Registered for STorM32 gimbal controller
	MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED1 = 201 // Registered for STorM32 gimbal controller
	MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED2 = 202 // Registered for STorM32 gimbal controller
	MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED3 = 203 // Registered for STorM32 gimbal controller
	MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED4 = 204 // Registered for STorM32 gimbal controller
	MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED5 = 205 // Registered for STorM32 gimbal controller
	MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED6 = 206 // Registered for STorM32 gimbal controller
	MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED7 = 207 // Registered for STorM32 gimbal controller
	MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED8 = 208 // Registered for STorM32 gimbal controller
	MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED9 = 209 // Registered for STorM32 gimbal controller
)

// MavOdidIDType (generated enum)
//
const (
	MAV_ODID_ID_TYPE_NONE                = 0 // No type defined
	MAV_ODID_ID_TYPE_SERIAL_NUMBER       = 1 // Manufacturer Serial Number (ANSI/CTA-2063 format)
	MAV_ODID_ID_TYPE_CAA_REGISTRATION_ID = 2 // CAA (Civil Aviation Authority) registered ID. Format: [ICAO Country Code].[CAA Assigned ID]
	MAV_ODID_ID_TYPE_UTM_ASSIGNED_UUID   = 3 // UTM (Unmanned Traffic Management) assigned UUID (RFC4122)
)

// MavOdidUaType (generated enum)
//
const (
	MAV_ODID_UA_TYPE_NONE                      = 0  // No UA (Unmanned Aircraft) type defined
	MAV_ODID_UA_TYPE_AEROPLANE                 = 1  // Aeroplane/Airplane. Fixed wing
	MAV_ODID_UA_TYPE_HELICOPTER_OR_MULTIROTOR  = 2  // Helicopter or multirotor
	MAV_ODID_UA_TYPE_GYROPLANE                 = 3  // Gyroplane
	MAV_ODID_UA_TYPE_HYBRID_LIFT               = 4  // VTOL (Vertical Take-Off and Landing). Fixed wing aircraft that can take off vertically
	MAV_ODID_UA_TYPE_ORNITHOPTER               = 5  // Ornithopter
	MAV_ODID_UA_TYPE_GLIDER                    = 6  // Glider
	MAV_ODID_UA_TYPE_KITE                      = 7  // Kite
	MAV_ODID_UA_TYPE_FREE_BALLOON              = 8  // Free Balloon
	MAV_ODID_UA_TYPE_CAPTIVE_BALLOON           = 9  // Captive Balloon
	MAV_ODID_UA_TYPE_AIRSHIP                   = 10 // Airship. E.g. a blimp
	MAV_ODID_UA_TYPE_FREE_FALL_PARACHUTE       = 11 // Free Fall/Parachute (unpowered)
	MAV_ODID_UA_TYPE_ROCKET                    = 12 // Rocket
	MAV_ODID_UA_TYPE_TETHERED_POWERED_AIRCRAFT = 13 // Tethered powered aircraft
	MAV_ODID_UA_TYPE_GROUND_OBSTACLE           = 14 // Ground Obstacle
	MAV_ODID_UA_TYPE_OTHER                     = 15 // Other type of aircraft not listed earlier
)

// MavOdidStatus (generated enum)
//
const (
	MAV_ODID_STATUS_UNDECLARED = 0 // The status of the (UA) Unmanned Aircraft is undefined
	MAV_ODID_STATUS_GROUND     = 1 // The UA is on the ground
	MAV_ODID_STATUS_AIRBORNE   = 2 // The UA is in the air
	MAV_ODID_STATUS_EMERGENCY  = 3 // The UA is having an emergency
)

// MavOdidHeightRef (generated enum)
//
const (
	MAV_ODID_HEIGHT_REF_OVER_TAKEOFF = 0 // The height field is relative to the take-off location
	MAV_ODID_HEIGHT_REF_OVER_GROUND  = 1 // The height field is relative to ground
)

// MavOdidHorAcc (generated enum)
//
const (
	MAV_ODID_HOR_ACC_UNKNOWN  = 0  // The horizontal accuracy is unknown
	MAV_ODID_HOR_ACC_10NM     = 1  // The horizontal accuracy is smaller than 10 Nautical Miles. 18.52 km
	MAV_ODID_HOR_ACC_4NM      = 2  // The horizontal accuracy is smaller than 4 Nautical Miles. 7.408 km
	MAV_ODID_HOR_ACC_2NM      = 3  // The horizontal accuracy is smaller than 2 Nautical Miles. 3.704 km
	MAV_ODID_HOR_ACC_1NM      = 4  // The horizontal accuracy is smaller than 1 Nautical Miles. 1.852 km
	MAV_ODID_HOR_ACC_0_5NM    = 5  // The horizontal accuracy is smaller than 0.5 Nautical Miles. 926 m
	MAV_ODID_HOR_ACC_0_3NM    = 6  // The horizontal accuracy is smaller than 0.3 Nautical Miles. 555.6 m
	MAV_ODID_HOR_ACC_0_1NM    = 7  // The horizontal accuracy is smaller than 0.1 Nautical Miles. 185.2 m
	MAV_ODID_HOR_ACC_0_05NM   = 8  // The horizontal accuracy is smaller than 0.05 Nautical Miles. 92.6 m
	MAV_ODID_HOR_ACC_30_METER = 9  // The horizontal accuracy is smaller than 30 meter
	MAV_ODID_HOR_ACC_10_METER = 10 // The horizontal accuracy is smaller than 10 meter
	MAV_ODID_HOR_ACC_3_METER  = 11 // The horizontal accuracy is smaller than 3 meter
	MAV_ODID_HOR_ACC_1_METER  = 12 // The horizontal accuracy is smaller than 1 meter
)

// MavOdidVerAcc (generated enum)
//
const (
	MAV_ODID_VER_ACC_UNKNOWN   = 0 // The vertical accuracy is unknown
	MAV_ODID_VER_ACC_150_METER = 1 // The vertical accuracy is smaller than 150 meter
	MAV_ODID_VER_ACC_45_METER  = 2 // The vertical accuracy is smaller than 45 meter
	MAV_ODID_VER_ACC_25_METER  = 3 // The vertical accuracy is smaller than 25 meter
	MAV_ODID_VER_ACC_10_METER  = 4 // The vertical accuracy is smaller than 10 meter
	MAV_ODID_VER_ACC_3_METER   = 5 // The vertical accuracy is smaller than 3 meter
	MAV_ODID_VER_ACC_1_METER   = 6 // The vertical accuracy is smaller than 1 meter
)

// MavOdidSpeedAcc (generated enum)
//
const (
	MAV_ODID_SPEED_ACC_UNKNOWN               = 0 // The speed accuracy is unknown
	MAV_ODID_SPEED_ACC_10_METERS_PER_SECOND  = 1 // The speed accuracy is smaller than 10 meters per second
	MAV_ODID_SPEED_ACC_3_METERS_PER_SECOND   = 2 // The speed accuracy is smaller than 3 meters per second
	MAV_ODID_SPEED_ACC_1_METERS_PER_SECOND   = 3 // The speed accuracy is smaller than 1 meters per second
	MAV_ODID_SPEED_ACC_0_3_METERS_PER_SECOND = 4 // The speed accuracy is smaller than 0.3 meters per second
)

// MavOdidTimeAcc (generated enum)
//
const (
	MAV_ODID_TIME_ACC_UNKNOWN    = 0  // The timestamp accuracy is unknown
	MAV_ODID_TIME_ACC_0_1_SECOND = 1  // The timestamp accuracy is smaller than or equal to 0.1 second
	MAV_ODID_TIME_ACC_0_2_SECOND = 2  // The timestamp accuracy is smaller than or equal to 0.2 second
	MAV_ODID_TIME_ACC_0_3_SECOND = 3  // The timestamp accuracy is smaller than or equal to 0.3 second
	MAV_ODID_TIME_ACC_0_4_SECOND = 4  // The timestamp accuracy is smaller than or equal to 0.4 second
	MAV_ODID_TIME_ACC_0_5_SECOND = 5  // The timestamp accuracy is smaller than or equal to 0.5 second
	MAV_ODID_TIME_ACC_0_6_SECOND = 6  // The timestamp accuracy is smaller than or equal to 0.6 second
	MAV_ODID_TIME_ACC_0_7_SECOND = 7  // The timestamp accuracy is smaller than or equal to 0.7 second
	MAV_ODID_TIME_ACC_0_8_SECOND = 8  // The timestamp accuracy is smaller than or equal to 0.8 second
	MAV_ODID_TIME_ACC_0_9_SECOND = 9  // The timestamp accuracy is smaller than or equal to 0.9 second
	MAV_ODID_TIME_ACC_1_0_SECOND = 10 // The timestamp accuracy is smaller than or equal to 1.0 second
	MAV_ODID_TIME_ACC_1_1_SECOND = 11 // The timestamp accuracy is smaller than or equal to 1.1 second
	MAV_ODID_TIME_ACC_1_2_SECOND = 12 // The timestamp accuracy is smaller than or equal to 1.2 second
	MAV_ODID_TIME_ACC_1_3_SECOND = 13 // The timestamp accuracy is smaller than or equal to 1.3 second
	MAV_ODID_TIME_ACC_1_4_SECOND = 14 // The timestamp accuracy is smaller than or equal to 1.4 second
	MAV_ODID_TIME_ACC_1_5_SECOND = 15 // The timestamp accuracy is smaller than or equal to 1.5 second
)

// MavOdidAuthType (generated enum)
//
const (
	MAV_ODID_AUTH_TYPE_NONE                  = 0 // No authentication type is specified
	MAV_ODID_AUTH_TYPE_UAS_ID_SIGNATURE      = 1 // Signature for the UAS (Unmanned Aircraft System) ID
	MAV_ODID_AUTH_TYPE_OPERATOR_ID_SIGNATURE = 2 // Signature for the Operator ID
	MAV_ODID_AUTH_TYPE_MESSAGE_SET_SIGNATURE = 3 // Signature for the entire message set
	MAV_ODID_AUTH_TYPE_NETWORK_REMOTE_ID     = 4 // Authentication is provided by Network Remote ID
)

// MavOdidDescType (generated enum)
//
const (
	MAV_ODID_DESC_TYPE_TEXT = 0 // Free-form text description of the purpose of the flight
)

// MavOdidOperatorLocationType (generated enum)
//
const (
	MAV_ODID_OPERATOR_LOCATION_TYPE_TAKEOFF   = 0 // The location of the operator is the same as the take-off location
	MAV_ODID_OPERATOR_LOCATION_TYPE_LIVE_GNSS = 1 // The location of the operator is based on live GNSS data
	MAV_ODID_OPERATOR_LOCATION_TYPE_FIXED     = 2 // The location of the operator is a fixed location
)

// MavOdidClassificationType (generated enum)
//
const (
	MAV_ODID_CLASSIFICATION_TYPE_UNDECLARED = 0 // The classification type for the UA is undeclared
	MAV_ODID_CLASSIFICATION_TYPE_EU         = 1 // The classification type for the UA follows EU (European Union) specifications
)

// MavOdidCategoryEu (generated enum)
//
const (
	MAV_ODID_CATEGORY_EU_UNDECLARED = 0 // The category for the UA, according to the EU specification, is undeclared
	MAV_ODID_CATEGORY_EU_OPEN       = 1 // The category for the UA, according to the EU specification, is the Open category
	MAV_ODID_CATEGORY_EU_SPECIFIC   = 2 // The category for the UA, according to the EU specification, is the Specific category
	MAV_ODID_CATEGORY_EU_CERTIFIED  = 3 // The category for the UA, according to the EU specification, is the Certified category
)

// MavOdidClassEu (generated enum)
//
const (
	MAV_ODID_CLASS_EU_UNDECLARED = 0 // The class for the UA, according to the EU specification, is undeclared
	MAV_ODID_CLASS_EU_CLASS_0    = 1 // The class for the UA, according to the EU specification, is Class 0
	MAV_ODID_CLASS_EU_CLASS_1    = 2 // The class for the UA, according to the EU specification, is Class 1
	MAV_ODID_CLASS_EU_CLASS_2    = 3 // The class for the UA, according to the EU specification, is Class 2
	MAV_ODID_CLASS_EU_CLASS_3    = 4 // The class for the UA, according to the EU specification, is Class 3
	MAV_ODID_CLASS_EU_CLASS_4    = 5 // The class for the UA, according to the EU specification, is Class 4
	MAV_ODID_CLASS_EU_CLASS_5    = 6 // The class for the UA, according to the EU specification, is Class 5
	MAV_ODID_CLASS_EU_CLASS_6    = 7 // The class for the UA, according to the EU specification, is Class 6
)

// MavOdidOperatorIDType (generated enum)
//
const (
	MAV_ODID_OPERATOR_ID_TYPE_CAA = 0 // CAA (Civil Aviation Authority) registered operator ID
)

// TuneFormat (generated enum)
// Tune formats (used for vehicle buzzer/tone generation).
const (
	TUNE_FORMAT_QBASIC1_1  = 1 // Format is QBasic 1.1 Play: https://www.qbasic.net/en/reference/qb11/Statement/PLAY-006.htm
	TUNE_FORMAT_MML_MODERN = 2 // Format is Modern Music Markup Language (MML): https://en.wikipedia.org/wiki/Music_Macro_Language#Modern_MML
)

// ComponentCapFlags (generated enum)
// Component capability flags (Bitmap)
const (
	COMPONENT_CAP_FLAGS_PARAM     = 1 // Component has parameters, and supports the parameter protocol (PARAM messages)
	COMPONENT_CAP_FLAGS_PARAM_EXT = 2 // Component has parameters, and supports the extended parameter protocol (PARAM_EXT messages)
)

// AisType (generated enum)
// Type of AIS vessel, enum duplicated from AIS standard, https://gpsd.gitlab.io/gpsd/AIVDM.html
const (
	AIS_TYPE_UNKNOWN                        = 0  // Not available (default)
	AIS_TYPE_RESERVED_1                     = 1  //
	AIS_TYPE_RESERVED_2                     = 2  //
	AIS_TYPE_RESERVED_3                     = 3  //
	AIS_TYPE_RESERVED_4                     = 4  //
	AIS_TYPE_RESERVED_5                     = 5  //
	AIS_TYPE_RESERVED_6                     = 6  //
	AIS_TYPE_RESERVED_7                     = 7  //
	AIS_TYPE_RESERVED_8                     = 8  //
	AIS_TYPE_RESERVED_9                     = 9  //
	AIS_TYPE_RESERVED_10                    = 10 //
	AIS_TYPE_RESERVED_11                    = 11 //
	AIS_TYPE_RESERVED_12                    = 12 //
	AIS_TYPE_RESERVED_13                    = 13 //
	AIS_TYPE_RESERVED_14                    = 14 //
	AIS_TYPE_RESERVED_15                    = 15 //
	AIS_TYPE_RESERVED_16                    = 16 //
	AIS_TYPE_RESERVED_17                    = 17 //
	AIS_TYPE_RESERVED_18                    = 18 //
	AIS_TYPE_RESERVED_19                    = 19 //
	AIS_TYPE_WIG                            = 20 // Wing In Ground effect
	AIS_TYPE_WIG_HAZARDOUS_A                = 21 //
	AIS_TYPE_WIG_HAZARDOUS_B                = 22 //
	AIS_TYPE_WIG_HAZARDOUS_C                = 23 //
	AIS_TYPE_WIG_HAZARDOUS_D                = 24 //
	AIS_TYPE_WIG_RESERVED_1                 = 25 //
	AIS_TYPE_WIG_RESERVED_2                 = 26 //
	AIS_TYPE_WIG_RESERVED_3                 = 27 //
	AIS_TYPE_WIG_RESERVED_4                 = 28 //
	AIS_TYPE_WIG_RESERVED_5                 = 29 //
	AIS_TYPE_FISHING                        = 30 //
	AIS_TYPE_TOWING                         = 31 //
	AIS_TYPE_TOWING_LARGE                   = 32 // Towing: length exceeds 200m or breadth exceeds 25m
	AIS_TYPE_DREDGING                       = 33 // Dredging or other underwater ops
	AIS_TYPE_DIVING                         = 34 //
	AIS_TYPE_MILITARY                       = 35 //
	AIS_TYPE_SAILING                        = 36 //
	AIS_TYPE_PLEASURE                       = 37 //
	AIS_TYPE_RESERVED_20                    = 38 //
	AIS_TYPE_RESERVED_21                    = 39 //
	AIS_TYPE_HSC                            = 40 // High Speed Craft
	AIS_TYPE_HSC_HAZARDOUS_A                = 41 //
	AIS_TYPE_HSC_HAZARDOUS_B                = 42 //
	AIS_TYPE_HSC_HAZARDOUS_C                = 43 //
	AIS_TYPE_HSC_HAZARDOUS_D                = 44 //
	AIS_TYPE_HSC_RESERVED_1                 = 45 //
	AIS_TYPE_HSC_RESERVED_2                 = 46 //
	AIS_TYPE_HSC_RESERVED_3                 = 47 //
	AIS_TYPE_HSC_RESERVED_4                 = 48 //
	AIS_TYPE_HSC_UNKNOWN                    = 49 //
	AIS_TYPE_PILOT                          = 50 //
	AIS_TYPE_SAR                            = 51 // Search And Rescue vessel
	AIS_TYPE_TUG                            = 52 //
	AIS_TYPE_PORT_TENDER                    = 53 //
	AIS_TYPE_ANTI_POLLUTION                 = 54 // Anti-pollution equipment
	AIS_TYPE_LAW_ENFORCEMENT                = 55 //
	AIS_TYPE_SPARE_LOCAL_1                  = 56 //
	AIS_TYPE_SPARE_LOCAL_2                  = 57 //
	AIS_TYPE_MEDICAL_TRANSPORT              = 58 //
	AIS_TYPE_NONECOMBATANT                  = 59 // Noncombatant ship according to RR Resolution No. 18
	AIS_TYPE_PASSENGER                      = 60 //
	AIS_TYPE_PASSENGER_HAZARDOUS_A          = 61 //
	AIS_TYPE_PASSENGER_HAZARDOUS_B          = 62 //
	AIS_TYPE_AIS_TYPE_PASSENGER_HAZARDOUS_C = 63 //
	AIS_TYPE_PASSENGER_HAZARDOUS_D          = 64 //
	AIS_TYPE_PASSENGER_RESERVED_1           = 65 //
	AIS_TYPE_PASSENGER_RESERVED_2           = 66 //
	AIS_TYPE_PASSENGER_RESERVED_3           = 67 //
	AIS_TYPE_AIS_TYPE_PASSENGER_RESERVED_4  = 68 //
	AIS_TYPE_PASSENGER_UNKNOWN              = 69 //
	AIS_TYPE_CARGO                          = 70 //
	AIS_TYPE_CARGO_HAZARDOUS_A              = 71 //
	AIS_TYPE_CARGO_HAZARDOUS_B              = 72 //
	AIS_TYPE_CARGO_HAZARDOUS_C              = 73 //
	AIS_TYPE_CARGO_HAZARDOUS_D              = 74 //
	AIS_TYPE_CARGO_RESERVED_1               = 75 //
	AIS_TYPE_CARGO_RESERVED_2               = 76 //
	AIS_TYPE_CARGO_RESERVED_3               = 77 //
	AIS_TYPE_CARGO_RESERVED_4               = 78 //
	AIS_TYPE_CARGO_UNKNOWN                  = 79 //
	AIS_TYPE_TANKER                         = 80 //
	AIS_TYPE_TANKER_HAZARDOUS_A             = 81 //
	AIS_TYPE_TANKER_HAZARDOUS_B             = 82 //
	AIS_TYPE_TANKER_HAZARDOUS_C             = 83 //
	AIS_TYPE_TANKER_HAZARDOUS_D             = 84 //
	AIS_TYPE_TANKER_RESERVED_1              = 85 //
	AIS_TYPE_TANKER_RESERVED_2              = 86 //
	AIS_TYPE_TANKER_RESERVED_3              = 87 //
	AIS_TYPE_TANKER_RESERVED_4              = 88 //
	AIS_TYPE_TANKER_UNKNOWN                 = 89 //
	AIS_TYPE_OTHER                          = 90 //
	AIS_TYPE_OTHER_HAZARDOUS_A              = 91 //
	AIS_TYPE_OTHER_HAZARDOUS_B              = 92 //
	AIS_TYPE_OTHER_HAZARDOUS_C              = 93 //
	AIS_TYPE_OTHER_HAZARDOUS_D              = 94 //
	AIS_TYPE_OTHER_RESERVED_1               = 95 //
	AIS_TYPE_OTHER_RESERVED_2               = 96 //
	AIS_TYPE_OTHER_RESERVED_3               = 97 //
	AIS_TYPE_OTHER_RESERVED_4               = 98 //
	AIS_TYPE_OTHER_UNKNOWN                  = 99 //
)

// AisNavStatus (generated enum)
// Navigational status of AIS vessel, enum duplicated from AIS standard, https://gpsd.gitlab.io/gpsd/AIVDM.html
const (
	UNDER_WAY                           = 0  // Under way using engine
	AIS_NAV_ANCHORED                    = 1  //
	AIS_NAV_UN_COMMANDED                = 2  //
	AIS_NAV_RESTRICTED_MANOEUVERABILITY = 3  //
	AIS_NAV_DRAUGHT_CONSTRAINED         = 4  //
	AIS_NAV_MOORED                      = 5  //
	AIS_NAV_AGROUND                     = 6  //
	AIS_NAV_FISHING                     = 7  //
	AIS_NAV_SAILING                     = 8  //
	AIS_NAV_RESERVED_HSC                = 9  //
	AIS_NAV_RESERVED_WIG                = 10 //
	AIS_NAV_RESERVED_1                  = 11 //
	AIS_NAV_RESERVED_2                  = 12 //
	AIS_NAV_RESERVED_3                  = 13 //
	AIS_NAV_AIS_SART                    = 14 // Search And Rescue Transponder
	AIS_NAV_UNKNOWN                     = 15 // Not available (default)
)

// AisFlags (generated enum)
// These flags are used in the AIS_VESSEL.fields bitmask to indicate validity of data in the other message fields. When set, the data is valid.
const (
	AIS_FLAGS_POSITION_ACCURACY         = 1    // 1 = Position accuracy less than 10m, 0 = position accuracy greater than 10m
	AIS_FLAGS_VALID_COG                 = 2    //
	AIS_FLAGS_VALID_VELOCITY            = 4    //
	AIS_FLAGS_HIGH_VELOCITY             = 8    // 1 = Velocity over 52.5765m/s (102.2 knots)
	AIS_FLAGS_VALID_TURN_RATE           = 16   //
	AIS_FLAGS_TURN_RATE_SIGN_ONLY       = 32   // Only the sign of the returned turn rate value is valid, either greater than 5deg/30s or less than -5deg/30s
	AIS_FLAGS_VALID_DIMENSIONS          = 64   //
	AIS_FLAGS_LARGE_BOW_DIMENSION       = 128  // Distance to bow is larger than 511m
	AIS_FLAGS_LARGE_STERN_DIMENSION     = 256  // Distance to stern is larger than 511m
	AIS_FLAGS_LARGE_PORT_DIMENSION      = 512  // Distance to port side is larger than 63m
	AIS_FLAGS_LARGE_STARBOARD_DIMENSION = 1024 // Distance to starboard side is larger than 63m
	AIS_FLAGS_VALID_CALLSIGN            = 2048 //
	AIS_FLAGS_VALID_NAME                = 4096 //
)

// FailureUnit (generated enum)
// List of possible units where failures can be injected.
const (
	FAILURE_UNIT_SENSOR_GYRO            = 0   //
	FAILURE_UNIT_SENSOR_ACCEL           = 1   //
	FAILURE_UNIT_SENSOR_MAG             = 2   //
	FAILURE_UNIT_SENSOR_BARO            = 3   //
	FAILURE_UNIT_SENSOR_GPS             = 4   //
	FAILURE_UNIT_SENSOR_OPTICAL_FLOW    = 5   //
	FAILURE_UNIT_SENSOR_VIO             = 6   //
	FAILURE_UNIT_SENSOR_DISTANCE_SENSOR = 7   //
	FAILURE_UNIT_SENSOR_AIRSPEED        = 8   //
	FAILURE_UNIT_SYSTEM_BATTERY         = 100 //
	FAILURE_UNIT_SYSTEM_MOTOR           = 101 //
	FAILURE_UNIT_SYSTEM_SERVO           = 102 //
	FAILURE_UNIT_SYSTEM_AVOIDANCE       = 103 //
	FAILURE_UNIT_SYSTEM_RC_SIGNAL       = 104 //
	FAILURE_UNIT_SYSTEM_MAVLINK_SIGNAL  = 105 //
)

// FailureType (generated enum)
// List of possible failure type to inject.
const (
	FAILURE_TYPE_OK           = 0 // No failure injected, used to reset a previous failure
	FAILURE_TYPE_OFF          = 1 // Sets unit off, so completely non-responsive
	FAILURE_TYPE_STUCK        = 2 // Unit is stuck e.g. keeps reporting the same value
	FAILURE_TYPE_GARBAGE      = 3 // Unit is reporting complete garbage
	FAILURE_TYPE_WRONG        = 4 // Unit is consistently wrong
	FAILURE_TYPE_SLOW         = 5 // Unit is slow, so e.g. reporting at slower than expected rate
	FAILURE_TYPE_DELAYED      = 6 // Data of unit is delayed in time
	FAILURE_TYPE_INTERMITTENT = 7 // Unit is sometimes working, sometimes not
)

// MavWinchStatusFlag (generated enum)
// Winch status flags used in WINCH_STATUS
const (
	MAV_WINCH_STATUS_HEALTHY         = 1 // Winch is healthy
	MAV_WINCH_STATUS_FULLY_RETRACTED = 2 // Winch thread is fully retracted
	MAV_WINCH_STATUS_MOVING          = 4 // Winch motor is moving
	MAV_WINCH_STATUS_CLUTCH_ENGAGED  = 8 // Winch clutch is engaged allowing motor to move freely
)

// MagCalStatus (generated enum)
//
const (
	MAG_CAL_NOT_STARTED      = 0 //
	MAG_CAL_WAITING_TO_START = 1 //
	MAG_CAL_RUNNING_STEP_ONE = 2 //
	MAG_CAL_RUNNING_STEP_TWO = 3 //
	MAG_CAL_SUCCESS          = 4 //
	MAG_CAL_FAILED           = 5 //
	MAG_CAL_BAD_ORIENTATION  = 6 //
	MAG_CAL_BAD_RADIUS       = 7 //
)

// SysStatus struct (generated typeinfo)
// The general system state. If the system is following the MAVLink standard, the system state is mainly defined by three orthogonal states/modes: The system mode, which is either LOCKED (motors shut down and locked), MANUAL (system under RC control), GUIDED (system with autonomous position control, position setpoint controlled manually) or AUTO (system guided by path/waypoint planner). The NAV_MODE defined the current flight state: LIFTOFF (often an open-loop maneuver), LANDING, WAYPOINTS or VECTOR. This represents the internal navigation state machine. The system status shows whether the system is currently active or not and if an emergency occurred. During the CRITICAL and EMERGENCY states the MAV is still considered to be active, but should start emergency procedures autonomously. After a failure occurred it should first move from active to critical to allow manual intervention and then move to emergency after a certain timeout.
type SysStatus struct {
	OnboardControlSensorsPresent uint32 // Bitmap showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present.
	OnboardControlSensorsEnabled uint32 // Bitmap showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of 1: enabled.
	OnboardControlSensorsHealth  uint32 // Bitmap showing which onboard controllers and sensors have an error (or are operational). Value of 0: error. Value of 1: healthy.
	Load                         uint16 // Maximum usage in percent of the mainloop time. Values: [0-1000] - should always be below 1000
	VoltageBattery               uint16 // Battery voltage, UINT16_MAX: Voltage not sent by autopilot
	CurrentBattery               int16  // Battery current, -1: Current not sent by autopilot
	DropRateComm                 uint16 // Communication drop rate, (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
	ErrorsComm                   uint16 // Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
	ErrorsCount1                 uint16 // Autopilot-specific errors
	ErrorsCount2                 uint16 // Autopilot-specific errors
	ErrorsCount3                 uint16 // Autopilot-specific errors
	ErrorsCount4                 uint16 // Autopilot-specific errors
	BatteryRemaining             int8   // Battery energy remaining, -1: Battery remaining energy not sent by autopilot
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *SysStatus) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 31 {
		return mavlink.ErrPayloadTooSmall
	}
	m.OnboardControlSensorsPresent = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.OnboardControlSensorsEnabled = uint32(binary.LittleEndian.Uint32(payload[4:]))
	m.OnboardControlSensorsHealth = uint32(binary.LittleEndian.Uint32(payload[8:]))
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *SystemTime) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 12 {
		return mavlink.ErrPayloadTooSmall
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *Ping) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 14 {
		return mavlink.ErrPayloadTooSmall
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
		"&common.ChangeOperatorControl{ TargetSystem: %+v, ControlRequest: %+v, Version: %+v, Passkey: %0b (\"%s\") }",
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ChangeOperatorControl) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 28 {
		return mavlink.ErrPayloadTooSmall
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ChangeOperatorControlAck) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 3 {
		return mavlink.ErrPayloadTooSmall
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
		"&common.AuthKey{ Key: %0b (\"%s\") }",
		m.Key, string(m.Key[:]),
	)
}

// Pack (generated function)
func (m *AuthKey) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 32)
	copy(payload[0:], m.Key[:])
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *AuthKey) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 32 {
		return mavlink.ErrPayloadTooSmall
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *LinkNodeStatus) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 36 {
		return mavlink.ErrPayloadTooSmall
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
	CustomMode   uint32 // The new autopilot-specific mode. This field can be ignored by an autopilot.
	TargetSystem uint8  // The system setting the mode
	BaseMode     uint8  // The new base mode.
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *SetMode) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 6 {
		return mavlink.ErrPayloadTooSmall
	}
	m.CustomMode = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.TargetSystem = uint8(payload[4])
	m.BaseMode = uint8(payload[5])
	return nil
}

// ParamAckTransaction struct (generated typeinfo)
// Response from a PARAM_SET message when it is used in a transaction.
type ParamAckTransaction struct {
	ParamValue      float32  // Parameter value (new value if PARAM_ACCEPTED, current value otherwise)
	TargetSystem    uint8    // Id of system that sent PARAM_SET message.
	TargetComponent uint8    // Id of system that sent PARAM_SET message.
	ParamID         [16]byte // Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
	ParamType       uint8    // Parameter type.
	ParamResult     uint8    // Result code.
}

// MsgID (generated function)
func (m *ParamAckTransaction) MsgID() mavlink.MessageID {
	return MSG_ID_PARAM_ACK_TRANSACTION
}

// String (generated function)
func (m *ParamAckTransaction) String() string {
	return fmt.Sprintf(
		"&common.ParamAckTransaction{ ParamValue: %+v, TargetSystem: %+v, TargetComponent: %+v, ParamID: %0b (\"%s\"), ParamType: %+v, ParamResult: %+v }",
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ParamAckTransaction) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 24 {
		return mavlink.ErrPayloadTooSmall
	}
	m.ParamValue = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.TargetSystem = uint8(payload[4])
	m.TargetComponent = uint8(payload[5])
	copy(m.ParamID[:], payload[6:22])
	m.ParamType = uint8(payload[22])
	m.ParamResult = uint8(payload[23])
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
		"&common.ParamRequestRead{ ParamIndex: %+v, TargetSystem: %+v, TargetComponent: %+v, ParamID: %0b (\"%s\") }",
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ParamRequestRead) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 20 {
		return mavlink.ErrPayloadTooSmall
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ParamRequestList) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 2 {
		return mavlink.ErrPayloadTooSmall
	}
	m.TargetSystem = uint8(payload[0])
	m.TargetComponent = uint8(payload[1])
	return nil
}

// ParamValue struct (generated typeinfo)
// Emit the value of a onboard parameter. The inclusion of param_count and param_index in the message allows the recipient to keep track of received parameters and allows him to re-request missing parameters after a loss or timeout. The parameter microservice is documented at https://mavlink.io/en/services/parameter.html
type ParamValue struct {
	ParamValue float32  // Onboard parameter value
	ParamCount uint16   // Total number of onboard parameters
	ParamIndex uint16   // Index of this onboard parameter
	ParamID    [16]byte // Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
	ParamType  uint8    // Onboard parameter type.
}

// MsgID (generated function)
func (m *ParamValue) MsgID() mavlink.MessageID {
	return MSG_ID_PARAM_VALUE
}

// String (generated function)
func (m *ParamValue) String() string {
	return fmt.Sprintf(
		"&common.ParamValue{ ParamValue: %+v, ParamCount: %+v, ParamIndex: %+v, ParamID: %0b (\"%s\"), ParamType: %+v }",
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ParamValue) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 25 {
		return mavlink.ErrPayloadTooSmall
	}
	m.ParamValue = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.ParamCount = uint16(binary.LittleEndian.Uint16(payload[4:]))
	m.ParamIndex = uint16(binary.LittleEndian.Uint16(payload[6:]))
	copy(m.ParamID[:], payload[8:24])
	m.ParamType = uint8(payload[24])
	return nil
}

// ParamSet struct (generated typeinfo)
// Set a parameter value (write new value to permanent storage).
//         The receiving component should acknowledge the new parameter value by broadcasting a PARAM_VALUE message (broadcasting ensures that multiple GCS all have an up-to-date list of all parameters). If the sending GCS did not receive a PARAM_VALUE within its timeout time, it should re-send the PARAM_SET message. The parameter microservice is documented at https://mavlink.io/en/services/parameter.html.
//         PARAM_SET may also be called within the context of a transaction (started with MAV_CMD_PARAM_TRANSACTION). Within a transaction the receiving component should respond with PARAM_ACK_TRANSACTION to the setter component (instead of broadcasting PARAM_VALUE), and PARAM_SET should be re-sent if this is ACK not received.
type ParamSet struct {
	ParamValue      float32  // Onboard parameter value
	TargetSystem    uint8    // System ID
	TargetComponent uint8    // Component ID
	ParamID         [16]byte // Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
	ParamType       uint8    // Onboard parameter type.
}

// MsgID (generated function)
func (m *ParamSet) MsgID() mavlink.MessageID {
	return MSG_ID_PARAM_SET
}

// String (generated function)
func (m *ParamSet) String() string {
	return fmt.Sprintf(
		"&common.ParamSet{ ParamValue: %+v, TargetSystem: %+v, TargetComponent: %+v, ParamID: %0b (\"%s\"), ParamType: %+v }",
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ParamSet) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 23 {
		return mavlink.ErrPayloadTooSmall
	}
	m.ParamValue = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.TargetSystem = uint8(payload[4])
	m.TargetComponent = uint8(payload[5])
	copy(m.ParamID[:], payload[6:22])
	m.ParamType = uint8(payload[22])
	return nil
}

// GpsRawInt struct (generated typeinfo)
// The global position, as returned by the Global Positioning System (GPS). This is
//                 NOT the global position estimate of the system, but rather a RAW sensor value. See message GLOBAL_POSITION for the global position estimate.
type GpsRawInt struct {
	TimeUsec          uint64 // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
	Lat               int32  // Latitude (WGS84, EGM96 ellipsoid)
	Lon               int32  // Longitude (WGS84, EGM96 ellipsoid)
	Alt               int32  // Altitude (MSL). Positive for up. Note that virtually all GPS modules provide the MSL altitude in addition to the WGS84 altitude.
	Eph               uint16 // GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX
	Epv               uint16 // GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX
	Vel               uint16 // GPS ground speed. If unknown, set to: UINT16_MAX
	Cog               uint16 // Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
	FixType           uint8  // GPS fix type.
	SatellitesVisible uint8  // Number of satellites visible. If unknown, set to 255
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *GpsRawInt) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 30 {
		return mavlink.ErrPayloadTooSmall
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(payload[0:]))
	m.Lat = int32(binary.LittleEndian.Uint32(payload[8:]))
	m.Lon = int32(binary.LittleEndian.Uint32(payload[12:]))
	m.Alt = int32(binary.LittleEndian.Uint32(payload[16:]))
	m.Eph = uint16(binary.LittleEndian.Uint16(payload[20:]))
	m.Epv = uint16(binary.LittleEndian.Uint16(payload[22:]))
	m.Vel = uint16(binary.LittleEndian.Uint16(payload[24:]))
	m.Cog = uint16(binary.LittleEndian.Uint16(payload[26:]))
	m.FixType = uint8(payload[28])
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
		"&common.GpsStatus{ SatellitesVisible: %+v, SatellitePrn: %0b (\"%s\"), SatelliteUsed: %0b (\"%s\"), SatelliteElevation: %0b (\"%s\"), SatelliteAzimuth: %0b (\"%s\"), SatelliteSnr: %0b (\"%s\") }",
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *GpsStatus) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 101 {
		return mavlink.ErrPayloadTooSmall
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ScaledImu) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 22 {
		return mavlink.ErrPayloadTooSmall
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *RawImu) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 26 {
		return mavlink.ErrPayloadTooSmall
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *RawPressure) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 16 {
		return mavlink.ErrPayloadTooSmall
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ScaledPressure) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 14 {
		return mavlink.ErrPayloadTooSmall
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *Attitude) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 28 {
		return mavlink.ErrPayloadTooSmall
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *AttitudeQuaternion) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 32 {
		return mavlink.ErrPayloadTooSmall
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *LocalPositionNed) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 28 {
		return mavlink.ErrPayloadTooSmall
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *GlobalPositionInt) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 28 {
		return mavlink.ErrPayloadTooSmall
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *RcChannelsScaled) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 22 {
		return mavlink.ErrPayloadTooSmall
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *RcChannelsRaw) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 22 {
		return mavlink.ErrPayloadTooSmall
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ServoOutputRaw) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 21 {
		return mavlink.ErrPayloadTooSmall
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *MissionRequestPartialList) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 6 {
		return mavlink.ErrPayloadTooSmall
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *MissionWritePartialList) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 6 {
		return mavlink.ErrPayloadTooSmall
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
	Param1          float32 // PARAM1, see MAV_CMD enum
	Param2          float32 // PARAM2, see MAV_CMD enum
	Param3          float32 // PARAM3, see MAV_CMD enum
	Param4          float32 // PARAM4, see MAV_CMD enum
	X               float32 // PARAM5 / local: X coordinate, global: latitude
	Y               float32 // PARAM6 / local: Y coordinate, global: longitude
	Z               float32 // PARAM7 / local: Z coordinate, global: altitude (relative or absolute, depending on frame).
	Seq             uint16  // Sequence
	Command         uint16  // The scheduled action for the waypoint.
	TargetSystem    uint8   // System ID
	TargetComponent uint8   // Component ID
	Frame           uint8   // The coordinate system of the waypoint.
	Current         uint8   // false:0, true:1
	Autocontinue    uint8   // Autocontinue to next waypoint
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *MissionItem) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 37 {
		return mavlink.ErrPayloadTooSmall
	}
	m.Param1 = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.Param2 = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	m.Param3 = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.Param4 = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.X = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	m.Y = math.Float32frombits(binary.LittleEndian.Uint32(payload[20:]))
	m.Z = math.Float32frombits(binary.LittleEndian.Uint32(payload[24:]))
	m.Seq = uint16(binary.LittleEndian.Uint16(payload[28:]))
	m.Command = uint16(binary.LittleEndian.Uint16(payload[30:]))
	m.TargetSystem = uint8(payload[32])
	m.TargetComponent = uint8(payload[33])
	m.Frame = uint8(payload[34])
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *MissionRequest) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 4 {
		return mavlink.ErrPayloadTooSmall
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *MissionSetCurrent) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 4 {
		return mavlink.ErrPayloadTooSmall
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *MissionCurrent) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 2 {
		return mavlink.ErrPayloadTooSmall
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *MissionRequestList) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 2 {
		return mavlink.ErrPayloadTooSmall
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *MissionCount) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 4 {
		return mavlink.ErrPayloadTooSmall
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *MissionClearAll) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 2 {
		return mavlink.ErrPayloadTooSmall
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *MissionItemReached) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 2 {
		return mavlink.ErrPayloadTooSmall
	}
	m.Seq = uint16(binary.LittleEndian.Uint16(payload[0:]))
	return nil
}

// MissionAck struct (generated typeinfo)
// Acknowledgment message during waypoint handling. The type field states if this message is a positive ack (type=0) or if an error happened (type=non-zero).
type MissionAck struct {
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
	Type            uint8 // Mission result.
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *MissionAck) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 3 {
		return mavlink.ErrPayloadTooSmall
	}
	m.TargetSystem = uint8(payload[0])
	m.TargetComponent = uint8(payload[1])
	m.Type = uint8(payload[2])
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *SetGpsGlobalOrigin) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 13 {
		return mavlink.ErrPayloadTooSmall
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *GpsGlobalOrigin) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 12 {
		return mavlink.ErrPayloadTooSmall
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
		"&common.ParamMapRc{ ParamValue0: %+v, Scale: %+v, ParamValueMin: %+v, ParamValueMax: %+v, ParamIndex: %+v, TargetSystem: %+v, TargetComponent: %+v, ParamID: %0b (\"%s\"), ParameterRcChannelIndex: %+v }",
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ParamMapRc) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 37 {
		return mavlink.ErrPayloadTooSmall
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *MissionRequestInt) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 4 {
		return mavlink.ErrPayloadTooSmall
	}
	m.Seq = uint16(binary.LittleEndian.Uint16(payload[0:]))
	m.TargetSystem = uint8(payload[2])
	m.TargetComponent = uint8(payload[3])
	return nil
}

// MissionChanged struct (generated typeinfo)
// A broadcast message to notify any ground station or SDK if a mission, geofence or safe points have changed on the vehicle.
type MissionChanged struct {
	StartIndex   int16 // Start index for partial mission change (-1 for all items).
	EndIndex     int16 // End index of a partial mission change. -1 is a synonym for the last mission item (i.e. selects all items from start_index). Ignore field if start_index=-1.
	OriginSysid  uint8 // System ID of the author of the new mission.
	OriginCompid uint8 // Compnent ID of the author of the new mission.
	MissionType  uint8 // Mission type.
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *MissionChanged) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 7 {
		return mavlink.ErrPayloadTooSmall
	}
	m.StartIndex = int16(binary.LittleEndian.Uint16(payload[0:]))
	m.EndIndex = int16(binary.LittleEndian.Uint16(payload[2:]))
	m.OriginSysid = uint8(payload[4])
	m.OriginCompid = uint8(payload[5])
	m.MissionType = uint8(payload[6])
	return nil
}

// SafetySetAllowedArea struct (generated typeinfo)
// Set a safety zone (volume), which is defined by two corners of a cube. This message can be used to tell the MAV which setpoints/waypoints to accept and which to reject. Safety areas are often enforced by national or competition regulations.
type SafetySetAllowedArea struct {
	P1x             float32 // x position 1 / Latitude 1
	P1y             float32 // y position 1 / Longitude 1
	P1z             float32 // z position 1 / Altitude 1
	P2x             float32 // x position 2 / Latitude 2
	P2y             float32 // y position 2 / Longitude 2
	P2z             float32 // z position 2 / Altitude 2
	TargetSystem    uint8   // System ID
	TargetComponent uint8   // Component ID
	Frame           uint8   // Coordinate frame. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down.
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *SafetySetAllowedArea) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 27 {
		return mavlink.ErrPayloadTooSmall
	}
	m.P1x = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.P1y = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	m.P1z = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.P2x = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.P2y = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	m.P2z = math.Float32frombits(binary.LittleEndian.Uint32(payload[20:]))
	m.TargetSystem = uint8(payload[24])
	m.TargetComponent = uint8(payload[25])
	m.Frame = uint8(payload[26])
	return nil
}

// SafetyAllowedArea struct (generated typeinfo)
// Read out the safety zone the MAV currently assumes.
type SafetyAllowedArea struct {
	P1x   float32 // x position 1 / Latitude 1
	P1y   float32 // y position 1 / Longitude 1
	P1z   float32 // z position 1 / Altitude 1
	P2x   float32 // x position 2 / Latitude 2
	P2y   float32 // y position 2 / Longitude 2
	P2z   float32 // z position 2 / Altitude 2
	Frame uint8   // Coordinate frame. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down.
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *SafetyAllowedArea) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 25 {
		return mavlink.ErrPayloadTooSmall
	}
	m.P1x = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.P1y = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	m.P1z = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.P2x = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.P2y = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	m.P2z = math.Float32frombits(binary.LittleEndian.Uint32(payload[20:]))
	m.Frame = uint8(payload[24])
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *AttitudeQuaternionCov) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 72 {
		return mavlink.ErrPayloadTooSmall
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *NavControllerOutput) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 26 {
		return mavlink.ErrPayloadTooSmall
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
	TimeUsec      uint64      // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
	Lat           int32       // Latitude
	Lon           int32       // Longitude
	Alt           int32       // Altitude in meters above MSL
	RelativeAlt   int32       // Altitude above ground
	Vx            float32     // Ground X Speed (Latitude)
	Vy            float32     // Ground Y Speed (Longitude)
	Vz            float32     // Ground Z Speed (Altitude)
	Covariance    [36]float32 // Row-major representation of a 6x6 position and velocity 6x6 cross-covariance matrix (states: lat, lon, alt, vx, vy, vz; first six entries are the first ROW, next six entries are the second row, etc.). If unknown, assign NaN value to first element in the array.
	EstimatorType uint8       // Class id of the estimator this estimate originated from.
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *GlobalPositionIntCov) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 181 {
		return mavlink.ErrPayloadTooSmall
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
	m.EstimatorType = uint8(payload[180])
	return nil
}

// LocalPositionNedCov struct (generated typeinfo)
// The filtered local position (e.g. fused computer vision and accelerometers). Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down convention)
type LocalPositionNedCov struct {
	TimeUsec      uint64      // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
	X             float32     // X Position
	Y             float32     // Y Position
	Z             float32     // Z Position
	Vx            float32     // X Speed
	Vy            float32     // Y Speed
	Vz            float32     // Z Speed
	Ax            float32     // X Acceleration
	Ay            float32     // Y Acceleration
	Az            float32     // Z Acceleration
	Covariance    [45]float32 // Row-major representation of position, velocity and acceleration 9x9 cross-covariance matrix upper right triangle (states: x, y, z, vx, vy, vz, ax, ay, az; first nine entries are the first ROW, next eight entries are the second row, etc.). If unknown, assign NaN value to first element in the array.
	EstimatorType uint8       // Class id of the estimator this estimate originated from.
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *LocalPositionNedCov) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 225 {
		return mavlink.ErrPayloadTooSmall
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
	m.EstimatorType = uint8(payload[224])
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *RcChannels) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 42 {
		return mavlink.ErrPayloadTooSmall
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *RequestDataStream) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 6 {
		return mavlink.ErrPayloadTooSmall
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *DataStream) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 4 {
		return mavlink.ErrPayloadTooSmall
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ManualControl) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 11 {
		return mavlink.ErrPayloadTooSmall
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *RcChannelsOverride) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 18 {
		return mavlink.ErrPayloadTooSmall
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
	Param1          float32 // PARAM1, see MAV_CMD enum
	Param2          float32 // PARAM2, see MAV_CMD enum
	Param3          float32 // PARAM3, see MAV_CMD enum
	Param4          float32 // PARAM4, see MAV_CMD enum
	X               int32   // PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7
	Y               int32   // PARAM6 / y position: local: x position in meters * 1e4, global: longitude in degrees *10^7
	Z               float32 // PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame.
	Seq             uint16  // Waypoint ID (sequence number). Starts at zero. Increases monotonically for each waypoint, no gaps in the sequence (0,1,2,3,4).
	Command         uint16  // The scheduled action for the waypoint.
	TargetSystem    uint8   // System ID
	TargetComponent uint8   // Component ID
	Frame           uint8   // The coordinate system of the waypoint.
	Current         uint8   // false:0, true:1
	Autocontinue    uint8   // Autocontinue to next waypoint
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *MissionItemInt) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 37 {
		return mavlink.ErrPayloadTooSmall
	}
	m.Param1 = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.Param2 = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	m.Param3 = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.Param4 = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.X = int32(binary.LittleEndian.Uint32(payload[16:]))
	m.Y = int32(binary.LittleEndian.Uint32(payload[20:]))
	m.Z = math.Float32frombits(binary.LittleEndian.Uint32(payload[24:]))
	m.Seq = uint16(binary.LittleEndian.Uint16(payload[28:]))
	m.Command = uint16(binary.LittleEndian.Uint16(payload[30:]))
	m.TargetSystem = uint8(payload[32])
	m.TargetComponent = uint8(payload[33])
	m.Frame = uint8(payload[34])
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *VfrHud) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 20 {
		return mavlink.ErrPayloadTooSmall
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
	Param1          float32 // PARAM1, see MAV_CMD enum
	Param2          float32 // PARAM2, see MAV_CMD enum
	Param3          float32 // PARAM3, see MAV_CMD enum
	Param4          float32 // PARAM4, see MAV_CMD enum
	X               int32   // PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7
	Y               int32   // PARAM6 / local: y position in meters * 1e4, global: longitude in degrees * 10^7
	Z               float32 // PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame).
	Command         uint16  // The scheduled action for the mission item.
	TargetSystem    uint8   // System ID
	TargetComponent uint8   // Component ID
	Frame           uint8   // The coordinate system of the COMMAND.
	Current         uint8   // Not used.
	Autocontinue    uint8   // Not used (set 0).
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *CommandInt) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 35 {
		return mavlink.ErrPayloadTooSmall
	}
	m.Param1 = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.Param2 = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	m.Param3 = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.Param4 = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.X = int32(binary.LittleEndian.Uint32(payload[16:]))
	m.Y = int32(binary.LittleEndian.Uint32(payload[20:]))
	m.Z = math.Float32frombits(binary.LittleEndian.Uint32(payload[24:]))
	m.Command = uint16(binary.LittleEndian.Uint16(payload[28:]))
	m.TargetSystem = uint8(payload[30])
	m.TargetComponent = uint8(payload[31])
	m.Frame = uint8(payload[32])
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
	Command         uint16  // Command ID (of command to send).
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *CommandLong) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 33 {
		return mavlink.ErrPayloadTooSmall
	}
	m.Param1 = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.Param2 = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	m.Param3 = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.Param4 = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.Param5 = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	m.Param6 = math.Float32frombits(binary.LittleEndian.Uint32(payload[20:]))
	m.Param7 = math.Float32frombits(binary.LittleEndian.Uint32(payload[24:]))
	m.Command = uint16(binary.LittleEndian.Uint16(payload[28:]))
	m.TargetSystem = uint8(payload[30])
	m.TargetComponent = uint8(payload[31])
	m.Confirmation = uint8(payload[32])
	return nil
}

// CommandAck struct (generated typeinfo)
// Report status of a command. Includes feedback whether the command was executed. The command microservice is documented at https://mavlink.io/en/services/command.html
type CommandAck struct {
	Command uint16 // Command ID (of acknowledged command).
	Result  uint8  // Result of command.
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *CommandAck) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 3 {
		return mavlink.ErrPayloadTooSmall
	}
	m.Command = uint16(binary.LittleEndian.Uint16(payload[0:]))
	m.Result = uint8(payload[2])
	return nil
}

// CommandCancel struct (generated typeinfo)
// Cancel a long running command. The target system should respond with a COMMAND_ACK to the original command with result=MAV_RESULT_CANCELLED if the long running process was cancelled. If it has already completed, the cancel action can be ignored. The cancel action can be retried until some sort of acknowledgement to the original command has been received. The command microservice is documented at https://mavlink.io/en/services/command.html
type CommandCancel struct {
	Command         uint16 // Command ID (of command to cancel).
	TargetSystem    uint8  // System executing long running command. Should not be broadcast (0).
	TargetComponent uint8  // Component executing long running command.
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *CommandCancel) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 4 {
		return mavlink.ErrPayloadTooSmall
	}
	m.Command = uint16(binary.LittleEndian.Uint16(payload[0:]))
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ManualSetpoint) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 22 {
		return mavlink.ErrPayloadTooSmall
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
	TimeBootMs      uint32     // Timestamp (time since system boot).
	Q               [4]float32 // Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
	BodyRollRate    float32    // Body roll rate
	BodyPitchRate   float32    // Body pitch rate
	BodyYawRate     float32    // Body yaw rate
	Thrust          float32    // Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)
	TargetSystem    uint8      // System ID
	TargetComponent uint8      // Component ID
	TypeMask        uint8      // Bitmap to indicate which dimensions should be ignored by the vehicle.
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *SetAttitudeTarget) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 39 {
		return mavlink.ErrPayloadTooSmall
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
	m.TypeMask = uint8(payload[38])
	return nil
}

// AttitudeTarget struct (generated typeinfo)
// Reports the current commanded attitude of the vehicle as specified by the autopilot. This should match the commands sent in a SET_ATTITUDE_TARGET message if the vehicle is being controlled this way.
type AttitudeTarget struct {
	TimeBootMs    uint32     // Timestamp (time since system boot).
	Q             [4]float32 // Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
	BodyRollRate  float32    // Body roll rate
	BodyPitchRate float32    // Body pitch rate
	BodyYawRate   float32    // Body yaw rate
	Thrust        float32    // Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)
	TypeMask      uint8      // Bitmap to indicate which dimensions should be ignored by the vehicle.
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *AttitudeTarget) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 37 {
		return mavlink.ErrPayloadTooSmall
	}
	m.TimeBootMs = uint32(binary.LittleEndian.Uint32(payload[0:]))
	for i := 0; i < len(m.Q); i++ {
		m.Q[i] = math.Float32frombits(binary.LittleEndian.Uint32(payload[4+i*4:]))
	}
	m.BodyRollRate = math.Float32frombits(binary.LittleEndian.Uint32(payload[20:]))
	m.BodyPitchRate = math.Float32frombits(binary.LittleEndian.Uint32(payload[24:]))
	m.BodyYawRate = math.Float32frombits(binary.LittleEndian.Uint32(payload[28:]))
	m.Thrust = math.Float32frombits(binary.LittleEndian.Uint32(payload[32:]))
	m.TypeMask = uint8(payload[36])
	return nil
}

// SetPositionTargetLocalNed struct (generated typeinfo)
// Sets a desired vehicle position in a local north-east-down coordinate frame. Used by an external controller to command the vehicle (manual controller or other system).
type SetPositionTargetLocalNed struct {
	TimeBootMs      uint32  // Timestamp (time since system boot).
	X               float32 // X Position in NED frame
	Y               float32 // Y Position in NED frame
	Z               float32 // Z Position in NED frame (note, altitude is negative in NED)
	Vx              float32 // X velocity in NED frame
	Vy              float32 // Y velocity in NED frame
	Vz              float32 // Z velocity in NED frame
	Afx             float32 // X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Afy             float32 // Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Afz             float32 // Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Yaw             float32 // yaw setpoint
	YawRate         float32 // yaw rate setpoint
	TypeMask        uint16  // Bitmap to indicate which dimensions should be ignored by the vehicle.
	TargetSystem    uint8   // System ID
	TargetComponent uint8   // Component ID
	CoordinateFrame uint8   // Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED = 9
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *SetPositionTargetLocalNed) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 53 {
		return mavlink.ErrPayloadTooSmall
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
	m.TypeMask = uint16(binary.LittleEndian.Uint16(payload[48:]))
	m.TargetSystem = uint8(payload[50])
	m.TargetComponent = uint8(payload[51])
	m.CoordinateFrame = uint8(payload[52])
	return nil
}

// PositionTargetLocalNed struct (generated typeinfo)
// Reports the current commanded vehicle position, velocity, and acceleration as specified by the autopilot. This should match the commands sent in SET_POSITION_TARGET_LOCAL_NED if the vehicle is being controlled this way.
type PositionTargetLocalNed struct {
	TimeBootMs      uint32  // Timestamp (time since system boot).
	X               float32 // X Position in NED frame
	Y               float32 // Y Position in NED frame
	Z               float32 // Z Position in NED frame (note, altitude is negative in NED)
	Vx              float32 // X velocity in NED frame
	Vy              float32 // Y velocity in NED frame
	Vz              float32 // Z velocity in NED frame
	Afx             float32 // X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Afy             float32 // Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Afz             float32 // Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Yaw             float32 // yaw setpoint
	YawRate         float32 // yaw rate setpoint
	TypeMask        uint16  // Bitmap to indicate which dimensions should be ignored by the vehicle.
	CoordinateFrame uint8   // Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED = 9
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *PositionTargetLocalNed) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 51 {
		return mavlink.ErrPayloadTooSmall
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
	m.TypeMask = uint16(binary.LittleEndian.Uint16(payload[48:]))
	m.CoordinateFrame = uint8(payload[50])
	return nil
}

// SetPositionTargetGlobalInt struct (generated typeinfo)
// Sets a desired vehicle position, velocity, and/or acceleration in a global coordinate system (WGS84). Used by an external controller to command the vehicle (manual controller or other system).
type SetPositionTargetGlobalInt struct {
	TimeBootMs      uint32  // Timestamp (time since system boot). The rationale for the timestamp in the setpoint is to allow the system to compensate for the transport delay of the setpoint. This allows the system to compensate processing latency.
	LatInt          int32   // X Position in WGS84 frame
	LonInt          int32   // Y Position in WGS84 frame
	Alt             float32 // Altitude (MSL, Relative to home, or AGL - depending on frame)
	Vx              float32 // X velocity in NED frame
	Vy              float32 // Y velocity in NED frame
	Vz              float32 // Z velocity in NED frame
	Afx             float32 // X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Afy             float32 // Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Afz             float32 // Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Yaw             float32 // yaw setpoint
	YawRate         float32 // yaw rate setpoint
	TypeMask        uint16  // Bitmap to indicate which dimensions should be ignored by the vehicle.
	TargetSystem    uint8   // System ID
	TargetComponent uint8   // Component ID
	CoordinateFrame uint8   // Valid options are: MAV_FRAME_GLOBAL_INT = 5, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT = 11
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *SetPositionTargetGlobalInt) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 53 {
		return mavlink.ErrPayloadTooSmall
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
	m.TypeMask = uint16(binary.LittleEndian.Uint16(payload[48:]))
	m.TargetSystem = uint8(payload[50])
	m.TargetComponent = uint8(payload[51])
	m.CoordinateFrame = uint8(payload[52])
	return nil
}

// PositionTargetGlobalInt struct (generated typeinfo)
// Reports the current commanded vehicle position, velocity, and acceleration as specified by the autopilot. This should match the commands sent in SET_POSITION_TARGET_GLOBAL_INT if the vehicle is being controlled this way.
type PositionTargetGlobalInt struct {
	TimeBootMs      uint32  // Timestamp (time since system boot). The rationale for the timestamp in the setpoint is to allow the system to compensate for the transport delay of the setpoint. This allows the system to compensate processing latency.
	LatInt          int32   // X Position in WGS84 frame
	LonInt          int32   // Y Position in WGS84 frame
	Alt             float32 // Altitude (MSL, AGL or relative to home altitude, depending on frame)
	Vx              float32 // X velocity in NED frame
	Vy              float32 // Y velocity in NED frame
	Vz              float32 // Z velocity in NED frame
	Afx             float32 // X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Afy             float32 // Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Afz             float32 // Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Yaw             float32 // yaw setpoint
	YawRate         float32 // yaw rate setpoint
	TypeMask        uint16  // Bitmap to indicate which dimensions should be ignored by the vehicle.
	CoordinateFrame uint8   // Valid options are: MAV_FRAME_GLOBAL_INT = 5, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT = 11
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *PositionTargetGlobalInt) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 51 {
		return mavlink.ErrPayloadTooSmall
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
	m.TypeMask = uint16(binary.LittleEndian.Uint16(payload[48:]))
	m.CoordinateFrame = uint8(payload[50])
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *LocalPositionNedSystemGlobalOffset) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 28 {
		return mavlink.ErrPayloadTooSmall
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *HilState) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 56 {
		return mavlink.ErrPayloadTooSmall
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
	TimeUsec      uint64  // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
	RollAilerons  float32 // Control output -1 .. 1
	PitchElevator float32 // Control output -1 .. 1
	YawRudder     float32 // Control output -1 .. 1
	Throttle      float32 // Throttle 0 .. 1
	Aux1          float32 // Aux 1, -1 .. 1
	Aux2          float32 // Aux 2, -1 .. 1
	Aux3          float32 // Aux 3, -1 .. 1
	Aux4          float32 // Aux 4, -1 .. 1
	Mode          uint8   // System mode.
	NavMode       uint8   // Navigation mode (MAV_NAV_MODE)
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *HilControls) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 42 {
		return mavlink.ErrPayloadTooSmall
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
	m.Mode = uint8(payload[40])
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *HilRcInputsRaw) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 33 {
		return mavlink.ErrPayloadTooSmall
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
	TimeUsec uint64      // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
	Flags    uint64      // Flags as bitfield, 1: indicate simulation using lockstep.
	Controls [16]float32 // Control outputs -1 .. 1. Channel assignment depends on the simulated hardware.
	Mode     uint8       // System mode. Includes arming state.
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *HilActuatorControls) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 81 {
		return mavlink.ErrPayloadTooSmall
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(payload[0:]))
	m.Flags = uint64(binary.LittleEndian.Uint64(payload[8:]))
	for i := 0; i < len(m.Controls); i++ {
		m.Controls[i] = math.Float32frombits(binary.LittleEndian.Uint32(payload[16+i*4:]))
	}
	m.Mode = uint8(payload[80])
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *OpticalFlow) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 26 {
		return mavlink.ErrPayloadTooSmall
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *GlobalVisionPositionEstimate) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 32 {
		return mavlink.ErrPayloadTooSmall
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *VisionPositionEstimate) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 32 {
		return mavlink.ErrPayloadTooSmall
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *VisionSpeedEstimate) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 20 {
		return mavlink.ErrPayloadTooSmall
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ViconPositionEstimate) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 32 {
		return mavlink.ErrPayloadTooSmall
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *HighresImu) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 62 {
		return mavlink.ErrPayloadTooSmall
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *OpticalFlowRad) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 44 {
		return mavlink.ErrPayloadTooSmall
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *HilSensor) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 64 {
		return mavlink.ErrPayloadTooSmall
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *SimState) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 84 {
		return mavlink.ErrPayloadTooSmall
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *RadioStatus) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 9 {
		return mavlink.ErrPayloadTooSmall
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
		"&common.FileTransferProtocol{ TargetNetwork: %+v, TargetSystem: %+v, TargetComponent: %+v, Payload: %0b (\"%s\") }",
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *FileTransferProtocol) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 254 {
		return mavlink.ErrPayloadTooSmall
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *Timesync) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 16 {
		return mavlink.ErrPayloadTooSmall
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *CameraTrigger) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 12 {
		return mavlink.ErrPayloadTooSmall
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *HilGps) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 36 {
		return mavlink.ErrPayloadTooSmall
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *HilOpticalFlow) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 44 {
		return mavlink.ErrPayloadTooSmall
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *HilStateQuaternion) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 64 {
		return mavlink.ErrPayloadTooSmall
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ScaledImu2) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 22 {
		return mavlink.ErrPayloadTooSmall
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *LogRequestList) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 6 {
		return mavlink.ErrPayloadTooSmall
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *LogEntry) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 14 {
		return mavlink.ErrPayloadTooSmall
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *LogRequestData) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 12 {
		return mavlink.ErrPayloadTooSmall
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
		"&common.LogData{ Ofs: %+v, ID: %+v, Count: %+v, Data: %0b (\"%s\") }",
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *LogData) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 97 {
		return mavlink.ErrPayloadTooSmall
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *LogErase) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 2 {
		return mavlink.ErrPayloadTooSmall
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *LogRequestEnd) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 2 {
		return mavlink.ErrPayloadTooSmall
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
		"&common.GpsInjectData{ TargetSystem: %+v, TargetComponent: %+v, Len: %+v, Data: %0b (\"%s\") }",
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *GpsInjectData) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 113 {
		return mavlink.ErrPayloadTooSmall
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
	TimeUsec          uint64 // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
	Lat               int32  // Latitude (WGS84)
	Lon               int32  // Longitude (WGS84)
	Alt               int32  // Altitude (MSL). Positive for up.
	DgpsAge           uint32 // Age of DGPS info
	Eph               uint16 // GPS HDOP horizontal dilution of position. If unknown, set to: UINT16_MAX
	Epv               uint16 // GPS VDOP vertical dilution of position. If unknown, set to: UINT16_MAX
	Vel               uint16 // GPS ground speed. If unknown, set to: UINT16_MAX
	Cog               uint16 // Course over ground (NOT heading, but direction of movement): 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
	FixType           uint8  // GPS fix type.
	SatellitesVisible uint8  // Number of satellites visible. If unknown, set to 255
	DgpsNumch         uint8  // Number of DGPS satellites
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *Gps2Raw) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 35 {
		return mavlink.ErrPayloadTooSmall
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
	m.FixType = uint8(payload[32])
	m.SatellitesVisible = uint8(payload[33])
	m.DgpsNumch = uint8(payload[34])
	return nil
}

// PowerStatus struct (generated typeinfo)
// Power supply status
type PowerStatus struct {
	Vcc    uint16 // 5V rail voltage.
	Vservo uint16 // Servo rail voltage.
	Flags  uint16 // Bitmap of power supply status flags.
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *PowerStatus) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 6 {
		return mavlink.ErrPayloadTooSmall
	}
	m.Vcc = uint16(binary.LittleEndian.Uint16(payload[0:]))
	m.Vservo = uint16(binary.LittleEndian.Uint16(payload[2:]))
	m.Flags = uint16(binary.LittleEndian.Uint16(payload[4:]))
	return nil
}

// SerialControl struct (generated typeinfo)
// Control a serial port. This can be used for raw access to an onboard serial peripheral such as a GPS or telemetry radio. It is designed to make it possible to update the devices firmware via MAVLink messages or change the devices settings. A message with zero bytes can be used to change just the baudrate.
type SerialControl struct {
	Baudrate uint32    // Baudrate of transfer. Zero means no change.
	Timeout  uint16    // Timeout for reply data
	Device   uint8     // Serial control device type.
	Flags    uint8     // Bitmap of serial control flags.
	Count    uint8     // how many bytes in this transfer
	Data     [70]uint8 // serial data
}

// MsgID (generated function)
func (m *SerialControl) MsgID() mavlink.MessageID {
	return MSG_ID_SERIAL_CONTROL
}

// String (generated function)
func (m *SerialControl) String() string {
	return fmt.Sprintf(
		"&common.SerialControl{ Baudrate: %+v, Timeout: %+v, Device: %+v, Flags: %+v, Count: %+v, Data: %0b (\"%s\") }",
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *SerialControl) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 79 {
		return mavlink.ErrPayloadTooSmall
	}
	m.Baudrate = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.Timeout = uint16(binary.LittleEndian.Uint16(payload[4:]))
	m.Device = uint8(payload[6])
	m.Flags = uint8(payload[7])
	m.Count = uint8(payload[8])
	copy(m.Data[:], payload[9:79])
	return nil
}

// GpsRtk struct (generated typeinfo)
// RTK GPS data. Gives information on the relative baseline calculation the GPS is reporting
type GpsRtk struct {
	TimeLastBaselineMs uint32 // Time since boot of last baseline message received.
	Tow                uint32 // GPS Time of Week of last baseline
	BaselineAMm        int32  // Current baseline in ECEF x or NED north component.
	BaselineBMm        int32  // Current baseline in ECEF y or NED east component.
	BaselineCMm        int32  // Current baseline in ECEF z or NED down component.
	Accuracy           uint32 // Current estimate of baseline accuracy.
	IarNumHypotheses   int32  // Current number of integer ambiguity hypotheses.
	Wn                 uint16 // GPS Week Number of last baseline
	RtkReceiverID      uint8  // Identification of connected RTK receiver.
	RtkHealth          uint8  // GPS-specific health report for RTK data.
	RtkRate            uint8  // Rate of baseline messages being received by GPS
	Nsats              uint8  // Current number of sats used for RTK calculation.
	BaselineCoordsType uint8  // Coordinate system of baseline
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *GpsRtk) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 35 {
		return mavlink.ErrPayloadTooSmall
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
	m.BaselineCoordsType = uint8(payload[34])
	return nil
}

// Gps2Rtk struct (generated typeinfo)
// RTK GPS data. Gives information on the relative baseline calculation the GPS is reporting
type Gps2Rtk struct {
	TimeLastBaselineMs uint32 // Time since boot of last baseline message received.
	Tow                uint32 // GPS Time of Week of last baseline
	BaselineAMm        int32  // Current baseline in ECEF x or NED north component.
	BaselineBMm        int32  // Current baseline in ECEF y or NED east component.
	BaselineCMm        int32  // Current baseline in ECEF z or NED down component.
	Accuracy           uint32 // Current estimate of baseline accuracy.
	IarNumHypotheses   int32  // Current number of integer ambiguity hypotheses.
	Wn                 uint16 // GPS Week Number of last baseline
	RtkReceiverID      uint8  // Identification of connected RTK receiver.
	RtkHealth          uint8  // GPS-specific health report for RTK data.
	RtkRate            uint8  // Rate of baseline messages being received by GPS
	Nsats              uint8  // Current number of sats used for RTK calculation.
	BaselineCoordsType uint8  // Coordinate system of baseline
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *Gps2Rtk) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 35 {
		return mavlink.ErrPayloadTooSmall
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
	m.BaselineCoordsType = uint8(payload[34])
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ScaledImu3) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 22 {
		return mavlink.ErrPayloadTooSmall
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
	Size       uint32 // total data size (set on ACK only).
	Width      uint16 // Width of a matrix or image.
	Height     uint16 // Height of a matrix or image.
	Packets    uint16 // Number of packets being sent (set on ACK only).
	Type       uint8  // Type of requested/acknowledged data.
	Payload    uint8  // Payload size per packet (normally 253 byte, see DATA field size in message ENCAPSULATED_DATA) (set on ACK only).
	JpgQuality uint8  // JPEG quality. Values: [1-100].
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *DataTransmissionHandshake) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 13 {
		return mavlink.ErrPayloadTooSmall
	}
	m.Size = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.Width = uint16(binary.LittleEndian.Uint16(payload[4:]))
	m.Height = uint16(binary.LittleEndian.Uint16(payload[6:]))
	m.Packets = uint16(binary.LittleEndian.Uint16(payload[8:]))
	m.Type = uint8(payload[10])
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
		"&common.EncapsulatedData{ Seqnr: %+v, Data: %0b (\"%s\") }",
		m.Seqnr,
		m.Data, string(m.Data[:]),
	)
}

// Pack (generated function)
func (m *EncapsulatedData) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 255)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.Seqnr))
	copy(payload[2:], m.Data[:])
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *EncapsulatedData) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 255 {
		return mavlink.ErrPayloadTooSmall
	}
	m.Seqnr = uint16(binary.LittleEndian.Uint16(payload[0:]))
	copy(m.Data[:], payload[2:255])
	return nil
}

// DistanceSensor struct (generated typeinfo)
// Distance sensor information for an onboard rangefinder.
type DistanceSensor struct {
	TimeBootMs      uint32 // Timestamp (time since system boot).
	MinDistance     uint16 // Minimum distance the sensor can measure
	MaxDistance     uint16 // Maximum distance the sensor can measure
	CurrentDistance uint16 // Current distance reading
	Type            uint8  // Type of distance sensor.
	ID              uint8  // Onboard ID of the sensor
	Orientation     uint8  // Direction the sensor faces. downward-facing: ROTATION_PITCH_270, upward-facing: ROTATION_PITCH_90, backward-facing: ROTATION_PITCH_180, forward-facing: ROTATION_NONE, left-facing: ROTATION_YAW_90, right-facing: ROTATION_YAW_270
	Covariance      uint8  // Measurement variance. Max standard deviation is 6cm. 255 if unknown.
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *DistanceSensor) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 14 {
		return mavlink.ErrPayloadTooSmall
	}
	m.TimeBootMs = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.MinDistance = uint16(binary.LittleEndian.Uint16(payload[4:]))
	m.MaxDistance = uint16(binary.LittleEndian.Uint16(payload[6:]))
	m.CurrentDistance = uint16(binary.LittleEndian.Uint16(payload[8:]))
	m.Type = uint8(payload[10])
	m.ID = uint8(payload[11])
	m.Orientation = uint8(payload[12])
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *TerrainRequest) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 18 {
		return mavlink.ErrPayloadTooSmall
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *TerrainData) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 43 {
		return mavlink.ErrPayloadTooSmall
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *TerrainCheck) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 8 {
		return mavlink.ErrPayloadTooSmall
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *TerrainReport) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 22 {
		return mavlink.ErrPayloadTooSmall
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ScaledPressure2) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 14 {
		return mavlink.ErrPayloadTooSmall
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *AttPosMocap) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 36 {
		return mavlink.ErrPayloadTooSmall
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *SetActuatorControlTarget) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 43 {
		return mavlink.ErrPayloadTooSmall
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ActuatorControlTarget) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 41 {
		return mavlink.ErrPayloadTooSmall
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *Altitude) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 32 {
		return mavlink.ErrPayloadTooSmall
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
		"&common.ResourceRequest{ RequestID: %+v, URIType: %+v, URI: %0b (\"%s\"), TransferType: %+v, Storage: %0b (\"%s\") }",
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ResourceRequest) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 243 {
		return mavlink.ErrPayloadTooSmall
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ScaledPressure3) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 14 {
		return mavlink.ErrPayloadTooSmall
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *FollowTarget) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 93 {
		return mavlink.ErrPayloadTooSmall
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ControlSystemState) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 100 {
		return mavlink.ErrPayloadTooSmall
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
	CurrentConsumed  int32      // Consumed charge, -1: autopilot does not provide consumption estimate
	EnergyConsumed   int32      // Consumed energy, -1: autopilot does not provide energy consumption estimate
	Temperature      int16      // Temperature of the battery. INT16_MAX for unknown temperature.
	Voltages         [10]uint16 // Battery voltage of cells 1 to 10 (see voltages_ext for cells 11-14). Cells in this field above the valid cell count for this battery should have the UINT16_MAX value. If individual cell voltages are unknown or not measured for this battery, then the overall battery voltage should be filled in cell 0, with all others set to UINT16_MAX. If the voltage of the battery is greater than (UINT16_MAX - 1), then cell 0 should be set to (UINT16_MAX - 1), and cell 1 to the remaining voltage. This can be extended to multiple cells if the total voltage is greater than 2 * (UINT16_MAX - 1).
	CurrentBattery   int16      // Battery current, -1: autopilot does not measure the current
	ID               uint8      // Battery ID
	BatteryFunction  uint8      // Function of the battery
	Type             uint8      // Type (chemistry) of the battery
	BatteryRemaining int8       // Remaining battery energy. Values: [0-100], -1: autopilot does not estimate the remaining battery.
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *BatteryStatus) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 36 {
		return mavlink.ErrPayloadTooSmall
	}
	m.CurrentConsumed = int32(binary.LittleEndian.Uint32(payload[0:]))
	m.EnergyConsumed = int32(binary.LittleEndian.Uint32(payload[4:]))
	m.Temperature = int16(binary.LittleEndian.Uint16(payload[8:]))
	for i := 0; i < len(m.Voltages); i++ {
		m.Voltages[i] = uint16(binary.LittleEndian.Uint16(payload[10+i*2:]))
	}
	m.CurrentBattery = int16(binary.LittleEndian.Uint16(payload[30:]))
	m.ID = uint8(payload[32])
	m.BatteryFunction = uint8(payload[33])
	m.Type = uint8(payload[34])
	m.BatteryRemaining = int8(payload[35])
	return nil
}

// AutopilotVersion struct (generated typeinfo)
// Version and capability of autopilot software. This should be emitted in response to a request with MAV_CMD_REQUEST_MESSAGE.
type AutopilotVersion struct {
	Capabilities            uint64   // Bitmap of capabilities
	UID                     uint64   // UID if provided by hardware (see uid2)
	FlightSwVersion         uint32   // Firmware version number
	MiddlewareSwVersion     uint32   // Middleware version number
	OsSwVersion             uint32   // Operating system version number
	BoardVersion            uint32   // HW / board version (last 8 bytes should be silicon ID, if any)
	VendorID                uint16   // ID of the board vendor
	ProductID               uint16   // ID of the product
	FlightCustomVersion     [8]uint8 // Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but should allow to identify the commit using the main version number even for very large code bases.
	MiddlewareCustomVersion [8]uint8 // Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but should allow to identify the commit using the main version number even for very large code bases.
	OsCustomVersion         [8]uint8 // Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but should allow to identify the commit using the main version number even for very large code bases.
}

// MsgID (generated function)
func (m *AutopilotVersion) MsgID() mavlink.MessageID {
	return MSG_ID_AUTOPILOT_VERSION
}

// String (generated function)
func (m *AutopilotVersion) String() string {
	return fmt.Sprintf(
		"&common.AutopilotVersion{ Capabilities: %+v, UID: %+v, FlightSwVersion: %+v, MiddlewareSwVersion: %+v, OsSwVersion: %+v, BoardVersion: %+v, VendorID: %+v, ProductID: %+v, FlightCustomVersion: %0b (\"%s\"), MiddlewareCustomVersion: %0b (\"%s\"), OsCustomVersion: %0b (\"%s\") }",
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *AutopilotVersion) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 60 {
		return mavlink.ErrPayloadTooSmall
	}
	m.Capabilities = uint64(binary.LittleEndian.Uint64(payload[0:]))
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
	TimeUsec  uint64  // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
	AngleX    float32 // X-axis angular offset of the target from the center of the image
	AngleY    float32 // Y-axis angular offset of the target from the center of the image
	Distance  float32 // Distance to the target from the vehicle
	SizeX     float32 // Size of target along x-axis
	SizeY     float32 // Size of target along y-axis
	TargetNum uint8   // The ID of the target if multiple targets are present
	Frame     uint8   // Coordinate frame used for following fields.
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *LandingTarget) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 30 {
		return mavlink.ErrPayloadTooSmall
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(payload[0:]))
	m.AngleX = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.AngleY = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.Distance = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	m.SizeX = math.Float32frombits(binary.LittleEndian.Uint32(payload[20:]))
	m.SizeY = math.Float32frombits(binary.LittleEndian.Uint32(payload[24:]))
	m.TargetNum = uint8(payload[28])
	m.Frame = uint8(payload[29])
	return nil
}

// FenceStatus struct (generated typeinfo)
// Status of geo-fencing. Sent in extended status stream when fencing enabled.
type FenceStatus struct {
	BreachTime   uint32 // Time (since boot) of last breach.
	BreachCount  uint16 // Number of fence breaches.
	BreachStatus uint8  // Breach status (0 if currently inside fence, 1 if outside).
	BreachType   uint8  // Last breach type.
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *FenceStatus) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 8 {
		return mavlink.ErrPayloadTooSmall
	}
	m.BreachTime = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.BreachCount = uint16(binary.LittleEndian.Uint16(payload[4:]))
	m.BreachStatus = uint8(payload[6])
	m.BreachType = uint8(payload[7])
	return nil
}

// MagCalReport struct (generated typeinfo)
// Reports results of completed compass calibration. Sent until MAG_CAL_ACK received.
type MagCalReport struct {
	Fitness   float32 // RMS milligauss residuals.
	OfsX      float32 // X offset.
	OfsY      float32 // Y offset.
	OfsZ      float32 // Z offset.
	DiagX     float32 // X diagonal (matrix 11).
	DiagY     float32 // Y diagonal (matrix 22).
	DiagZ     float32 // Z diagonal (matrix 33).
	OffdiagX  float32 // X off-diagonal (matrix 12 and 21).
	OffdiagY  float32 // Y off-diagonal (matrix 13 and 31).
	OffdiagZ  float32 // Z off-diagonal (matrix 32 and 23).
	CompassID uint8   // Compass being calibrated.
	CalMask   uint8   // Bitmask of compasses being calibrated.
	CalStatus uint8   // Calibration Status.
	Autosaved uint8   // 0=requires a MAV_CMD_DO_ACCEPT_MAG_CAL, 1=saved to parameters.
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *MagCalReport) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 44 {
		return mavlink.ErrPayloadTooSmall
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
	m.CalStatus = uint8(payload[42])
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *EfiStatus) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 65 {
		return mavlink.ErrPayloadTooSmall
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
	TimeUsec         uint64  // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
	VelRatio         float32 // Velocity innovation test ratio
	PosHorizRatio    float32 // Horizontal position innovation test ratio
	PosVertRatio     float32 // Vertical position innovation test ratio
	MagRatio         float32 // Magnetometer innovation test ratio
	HaglRatio        float32 // Height above terrain innovation test ratio
	TasRatio         float32 // True airspeed innovation test ratio
	PosHorizAccuracy float32 // Horizontal position 1-STD accuracy relative to the EKF local origin
	PosVertAccuracy  float32 // Vertical position 1-STD accuracy relative to the EKF local origin
	Flags            uint16  // Bitmap indicating which EKF outputs are valid.
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *EstimatorStatus) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 42 {
		return mavlink.ErrPayloadTooSmall
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
	m.Flags = uint16(binary.LittleEndian.Uint16(payload[40:]))
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *WindCov) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 40 {
		return mavlink.ErrPayloadTooSmall
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
	TimeUsec          uint64  // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
	TimeWeekMs        uint32  // GPS time (from start of GPS week)
	Lat               int32   // Latitude (WGS84)
	Lon               int32   // Longitude (WGS84)
	Alt               float32 // Altitude (MSL). Positive for up.
	Hdop              float32 // GPS HDOP horizontal dilution of position
	Vdop              float32 // GPS VDOP vertical dilution of position
	Vn                float32 // GPS velocity in north direction in earth-fixed NED frame
	Ve                float32 // GPS velocity in east direction in earth-fixed NED frame
	Vd                float32 // GPS velocity in down direction in earth-fixed NED frame
	SpeedAccuracy     float32 // GPS speed accuracy
	HorizAccuracy     float32 // GPS horizontal accuracy
	VertAccuracy      float32 // GPS vertical accuracy
	IgnoreFlags       uint16  // Bitmap indicating which GPS input flags fields to ignore.  All other fields must be provided.
	TimeWeek          uint16  // GPS week number
	GpsID             uint8   // ID of the GPS for multiple GPS inputs
	FixType           uint8   // 0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RTK
	SatellitesVisible uint8   // Number of satellites visible.
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *GpsInput) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 63 {
		return mavlink.ErrPayloadTooSmall
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
	m.IgnoreFlags = uint16(binary.LittleEndian.Uint16(payload[56:]))
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
		"&common.GpsRtcmData{ Flags: %+v, Len: %+v, Data: %0b (\"%s\") }",
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *GpsRtcmData) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 182 {
		return mavlink.ErrPayloadTooSmall
	}
	m.Flags = uint8(payload[0])
	m.Len = uint8(payload[1])
	copy(m.Data[:], payload[2:182])
	return nil
}

// HighLatency struct (generated typeinfo)
// Message appropriate for high latency connections like Iridium
type HighLatency struct {
	CustomMode       uint32 // A bitfield for use for autopilot-specific flags.
	Latitude         int32  // Latitude
	Longitude        int32  // Longitude
	Roll             int16  // roll
	Pitch            int16  // pitch
	Heading          uint16 // heading
	HeadingSp        int16  // heading setpoint
	AltitudeAmsl     int16  // Altitude above mean sea level
	AltitudeSp       int16  // Altitude setpoint relative to the home position
	WpDistance       uint16 // distance to target
	BaseMode         uint8  // Bitmap of enabled system modes.
	LandedState      uint8  // The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.
	Throttle         int8   // throttle (percentage)
	Airspeed         uint8  // airspeed
	AirspeedSp       uint8  // airspeed setpoint
	Groundspeed      uint8  // groundspeed
	ClimbRate        int8   // climb rate
	GpsNsat          uint8  // Number of satellites visible. If unknown, set to 255
	GpsFixType       uint8  // GPS Fix type.
	BatteryRemaining uint8  // Remaining battery (percentage)
	Temperature      int8   // Autopilot temperature (degrees C)
	TemperatureAir   int8   // Air temperature (degrees C) from airspeed sensor
	Failsafe         uint8  // failsafe (each bit represents a failsafe where 0=ok, 1=failsafe active (bit0:RC, bit1:batt, bit2:GPS, bit3:GCS, bit4:fence)
	WpNum            uint8  // current waypoint number
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *HighLatency) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 40 {
		return mavlink.ErrPayloadTooSmall
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
	m.BaseMode = uint8(payload[26])
	m.LandedState = uint8(payload[27])
	m.Throttle = int8(payload[28])
	m.Airspeed = uint8(payload[29])
	m.AirspeedSp = uint8(payload[30])
	m.Groundspeed = uint8(payload[31])
	m.ClimbRate = int8(payload[32])
	m.GpsNsat = uint8(payload[33])
	m.GpsFixType = uint8(payload[34])
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
	Timestamp      uint32 // Timestamp (milliseconds since boot or Unix epoch)
	Latitude       int32  // Latitude
	Longitude      int32  // Longitude
	CustomMode     uint16 // A bitfield for use for autopilot-specific flags (2 byte version).
	Altitude       int16  // Altitude above mean sea level
	TargetAltitude int16  // Altitude setpoint
	TargetDistance uint16 // Distance to target waypoint or position
	WpNum          uint16 // Current waypoint number
	FailureFlags   uint16 // Bitmap of failure flags.
	Type           uint8  // Type of the MAV (quadrotor, helicopter, etc.)
	Autopilot      uint8  // Autopilot type / class. Use MAV_AUTOPILOT_INVALID for components that are not flight controllers.
	Heading        uint8  // Heading
	TargetHeading  uint8  // Heading setpoint
	Throttle       uint8  // Throttle
	Airspeed       uint8  // Airspeed
	AirspeedSp     uint8  // Airspeed setpoint
	Groundspeed    uint8  // Groundspeed
	Windspeed      uint8  // Windspeed
	WindHeading    uint8  // Wind heading
	Eph            uint8  // Maximum error horizontal position since last message
	Epv            uint8  // Maximum error vertical position since last message
	TemperatureAir int8   // Air temperature from airspeed sensor
	ClimbRate      int8   // Maximum climb rate magnitude since last message
	Battery        int8   // Battery level (-1 if field not provided).
	Custom0        int8   // Field for custom payload.
	Custom1        int8   // Field for custom payload.
	Custom2        int8   // Field for custom payload.
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *HighLatency2) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 42 {
		return mavlink.ErrPayloadTooSmall
	}
	m.Timestamp = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.Latitude = int32(binary.LittleEndian.Uint32(payload[4:]))
	m.Longitude = int32(binary.LittleEndian.Uint32(payload[8:]))
	m.CustomMode = uint16(binary.LittleEndian.Uint16(payload[12:]))
	m.Altitude = int16(binary.LittleEndian.Uint16(payload[14:]))
	m.TargetAltitude = int16(binary.LittleEndian.Uint16(payload[16:]))
	m.TargetDistance = uint16(binary.LittleEndian.Uint16(payload[18:]))
	m.WpNum = uint16(binary.LittleEndian.Uint16(payload[20:]))
	m.FailureFlags = uint16(binary.LittleEndian.Uint16(payload[22:]))
	m.Type = uint8(payload[24])
	m.Autopilot = uint8(payload[25])
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *Vibration) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 32 {
		return mavlink.ErrPayloadTooSmall
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *HomePosition) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 52 {
		return mavlink.ErrPayloadTooSmall
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *SetHomePosition) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 53 {
		return mavlink.ErrPayloadTooSmall
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *MessageInterval) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 6 {
		return mavlink.ErrPayloadTooSmall
	}
	m.IntervalUs = int32(binary.LittleEndian.Uint32(payload[0:]))
	m.MessageID = uint16(binary.LittleEndian.Uint16(payload[4:]))
	return nil
}

// ExtendedSysState struct (generated typeinfo)
// Provides state for additional features
type ExtendedSysState struct {
	VtolState   uint8 // The VTOL state if applicable. Is set to MAV_VTOL_STATE_UNDEFINED if UAV is not in VTOL configuration.
	LandedState uint8 // The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ExtendedSysState) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 2 {
		return mavlink.ErrPayloadTooSmall
	}
	m.VtolState = uint8(payload[0])
	m.LandedState = uint8(payload[1])
	return nil
}

// AdsbVehicle struct (generated typeinfo)
// The location and information of an ADSB vehicle
type AdsbVehicle struct {
	IcaoAddress  uint32  // ICAO address
	Lat          int32   // Latitude
	Lon          int32   // Longitude
	Altitude     int32   // Altitude(ASL)
	Heading      uint16  // Course over ground
	HorVelocity  uint16  // The horizontal velocity
	VerVelocity  int16   // The vertical velocity. Positive is up
	Flags        uint16  // Bitmap to indicate various statuses including valid data fields
	Squawk       uint16  // Squawk code
	AltitudeType uint8   // ADSB altitude type.
	Callsign     [9]byte // The callsign, 8+null
	EmitterType  uint8   // ADSB emitter type.
	Tslc         uint8   // Time since last communication in seconds
}

// MsgID (generated function)
func (m *AdsbVehicle) MsgID() mavlink.MessageID {
	return MSG_ID_ADSB_VEHICLE
}

// String (generated function)
func (m *AdsbVehicle) String() string {
	return fmt.Sprintf(
		"&common.AdsbVehicle{ IcaoAddress: %+v, Lat: %+v, Lon: %+v, Altitude: %+v, Heading: %+v, HorVelocity: %+v, VerVelocity: %+v, Flags: %+v, Squawk: %+v, AltitudeType: %+v, Callsign: %0b (\"%s\"), EmitterType: %+v, Tslc: %+v }",
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *AdsbVehicle) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 38 {
		return mavlink.ErrPayloadTooSmall
	}
	m.IcaoAddress = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.Lat = int32(binary.LittleEndian.Uint32(payload[4:]))
	m.Lon = int32(binary.LittleEndian.Uint32(payload[8:]))
	m.Altitude = int32(binary.LittleEndian.Uint32(payload[12:]))
	m.Heading = uint16(binary.LittleEndian.Uint16(payload[16:]))
	m.HorVelocity = uint16(binary.LittleEndian.Uint16(payload[18:]))
	m.VerVelocity = int16(binary.LittleEndian.Uint16(payload[20:]))
	m.Flags = uint16(binary.LittleEndian.Uint16(payload[22:]))
	m.Squawk = uint16(binary.LittleEndian.Uint16(payload[24:]))
	m.AltitudeType = uint8(payload[26])
	copy(m.Callsign[:], payload[27:36])
	m.EmitterType = uint8(payload[36])
	m.Tslc = uint8(payload[37])
	return nil
}

// Collision struct (generated typeinfo)
// Information about a potential collision
type Collision struct {
	ID                     uint32  // Unique identifier, domain based on src field
	TimeToMinimumDelta     float32 // Estimated time until collision occurs
	AltitudeMinimumDelta   float32 // Closest vertical distance between vehicle and object
	HorizontalMinimumDelta float32 // Closest horizontal distance between vehicle and object
	Src                    uint8   // Collision data source
	Action                 uint8   // Action that is being taken to avoid this collision
	ThreatLevel            uint8   // How concerned the aircraft is about this collision
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *Collision) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 19 {
		return mavlink.ErrPayloadTooSmall
	}
	m.ID = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.TimeToMinimumDelta = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	m.AltitudeMinimumDelta = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.HorizontalMinimumDelta = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.Src = uint8(payload[16])
	m.Action = uint8(payload[17])
	m.ThreatLevel = uint8(payload[18])
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
		"&common.V2Extension{ MessageType: %+v, TargetNetwork: %+v, TargetSystem: %+v, TargetComponent: %+v, Payload: %0b (\"%s\") }",
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *V2Extension) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 254 {
		return mavlink.ErrPayloadTooSmall
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *MemoryVect) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 36 {
		return mavlink.ErrPayloadTooSmall
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
		"&common.DebugVect{ TimeUsec: %+v, X: %+v, Y: %+v, Z: %+v, Name: %0b (\"%s\") }",
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *DebugVect) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 30 {
		return mavlink.ErrPayloadTooSmall
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
		"&common.NamedValueFloat{ TimeBootMs: %+v, Value: %+v, Name: %0b (\"%s\") }",
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *NamedValueFloat) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 18 {
		return mavlink.ErrPayloadTooSmall
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
		"&common.NamedValueInt{ TimeBootMs: %+v, Value: %+v, Name: %0b (\"%s\") }",
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *NamedValueInt) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 18 {
		return mavlink.ErrPayloadTooSmall
	}
	m.TimeBootMs = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.Value = int32(binary.LittleEndian.Uint32(payload[4:]))
	copy(m.Name[:], payload[8:18])
	return nil
}

// Statustext struct (generated typeinfo)
// Status text message. These messages are printed in yellow in the COMM console of QGroundControl. WARNING: They consume quite some bandwidth, so use only for important status and error messages. If implemented wisely, these messages are buffered on the MCU and sent only at a limited rate (e.g. 10 Hz).
type Statustext struct {
	Severity uint8    // Severity of status. Relies on the definitions within RFC-5424.
	Text     [50]byte // Status text message, without null termination character
}

// MsgID (generated function)
func (m *Statustext) MsgID() mavlink.MessageID {
	return MSG_ID_STATUSTEXT
}

// String (generated function)
func (m *Statustext) String() string {
	return fmt.Sprintf(
		"&common.Statustext{ Severity: %+v, Text: %0b (\"%s\") }",
		m.Severity,
		m.Text, string(m.Text[:]),
	)
}

// Pack (generated function)
func (m *Statustext) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 51)
	payload[0] = byte(m.Severity)
	copy(payload[1:], m.Text[:])
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *Statustext) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 51 {
		return mavlink.ErrPayloadTooSmall
	}
	m.Severity = uint8(payload[0])
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *Debug) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 9 {
		return mavlink.ErrPayloadTooSmall
	}
	m.TimeBootMs = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.Value = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	m.Ind = uint8(payload[8])
	return nil
}

// Message IDs
const (
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
)

// init Common dialect
func init() {
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
}

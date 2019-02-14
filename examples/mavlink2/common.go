//////////////////////////////////////////////////
//
// NOTE: do not edit,
// this file created automatically by mavgen.go
//
//////////////////////////////////////////////////

package mavlink

import (
	"encoding/binary"
	"fmt"
	"math"
)

// MavAutopilot (generated enum)
// Micro air vehicle / autopilot classes. This identifies the individual model.
const (
	MAV_AUTOPILOT_GENERIC                                      = 0  // Generic autopilot, full support for everything
	MAV_AUTOPILOT_RESERVED                                     = 1  // Reserved for future use.
	MAV_AUTOPILOT_SLUGS                                        = 2  // SLUGS autopilot, http://slugsuav.soe.ucsc.edu
	MAV_AUTOPILOT_ARDUPILOTMEGA                                = 3  // ArduPilotMega / ArduCopter, http://diydrones.com
	MAV_AUTOPILOT_OPENPILOT                                    = 4  // OpenPilot, http://openpilot.org
	MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY                       = 5  // Generic autopilot only supporting simple waypoints
	MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY = 6  // Generic autopilot supporting waypoints and other simple navigation commands
	MAV_AUTOPILOT_GENERIC_MISSION_FULL                         = 7  // Generic autopilot supporting the full mission command set
	MAV_AUTOPILOT_INVALID                                      = 8  // No valid autopilot, e.g. a GCS or other MAVLink component
	MAV_AUTOPILOT_PPZ                                          = 9  // PPZ UAV - http://nongnu.org/paparazzi
	MAV_AUTOPILOT_UDB                                          = 10 // UAV Dev Board
	MAV_AUTOPILOT_FP                                           = 11 // FlexiPilot
	MAV_AUTOPILOT_PX4                                          = 12 // PX4 Autopilot - http://pixhawk.ethz.ch/px4/
	MAV_AUTOPILOT_SMACCMPILOT                                  = 13 // SMACCMPilot - http://smaccmpilot.org
	MAV_AUTOPILOT_AUTOQUAD                                     = 14 // AutoQuad -- http://autoquad.org
	MAV_AUTOPILOT_ARMAZILA                                     = 15 // Armazila -- http://armazila.com
	MAV_AUTOPILOT_AEROB                                        = 16 // Aerob -- http://aerob.ru
	MAV_AUTOPILOT_ASLUAV                                       = 17 // ASLUAV autopilot -- http://www.asl.ethz.ch
	MAV_AUTOPILOT_SMARTAP                                      = 18 // SmartAP Autopilot - http://sky-drones.com
)

// MavType (generated enum)
//
const (
	MAV_TYPE_GENERIC            = 0  // Generic micro air vehicle.
	MAV_TYPE_FIXED_WING         = 1  // Fixed wing aircraft.
	MAV_TYPE_QUADROTOR          = 2  // Quadrotor
	MAV_TYPE_COAXIAL            = 3  // Coaxial helicopter
	MAV_TYPE_HELICOPTER         = 4  // Normal helicopter with tail rotor.
	MAV_TYPE_ANTENNA_TRACKER    = 5  // Ground installation
	MAV_TYPE_GCS                = 6  // Operator control unit / ground control station
	MAV_TYPE_AIRSHIP            = 7  // Airship, controlled
	MAV_TYPE_FREE_BALLOON       = 8  // Free balloon, uncontrolled
	MAV_TYPE_ROCKET             = 9  // Rocket
	MAV_TYPE_GROUND_ROVER       = 10 // Ground rover
	MAV_TYPE_SURFACE_BOAT       = 11 // Surface vessel, boat, ship
	MAV_TYPE_SUBMARINE          = 12 // Submarine
	MAV_TYPE_HEXAROTOR          = 13 // Hexarotor
	MAV_TYPE_OCTOROTOR          = 14 // Octorotor
	MAV_TYPE_TRICOPTER          = 15 // Tricopter
	MAV_TYPE_FLAPPING_WING      = 16 // Flapping wing
	MAV_TYPE_KITE               = 17 // Kite
	MAV_TYPE_ONBOARD_CONTROLLER = 18 // Onboard companion controller
	MAV_TYPE_VTOL_DUOROTOR      = 19 // Two-rotor VTOL using control surfaces in vertical operation in addition. Tailsitter.
	MAV_TYPE_VTOL_QUADROTOR     = 20 // Quad-rotor VTOL using a V-shaped quad config in vertical operation. Tailsitter.
	MAV_TYPE_VTOL_TILTROTOR     = 21 // Tiltrotor VTOL
	MAV_TYPE_VTOL_RESERVED2     = 22 // VTOL reserved 2
	MAV_TYPE_VTOL_RESERVED3     = 23 // VTOL reserved 3
	MAV_TYPE_VTOL_RESERVED4     = 24 // VTOL reserved 4
	MAV_TYPE_VTOL_RESERVED5     = 25 // VTOL reserved 5
	MAV_TYPE_GIMBAL             = 26 // Onboard gimbal
	MAV_TYPE_ADSB               = 27 // Onboard ADSB peripheral
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

// MavModeFlag (generated enum)
// These flags encode the MAV mode.
const (
	MAV_MODE_FLAG_SAFETY_ARMED         = 128 // 0b10000000 MAV safety set to armed. Motors are enabled / running / can start. Ready to fly. Additional note: this flag is to be ignore when sent in the command MAV_CMD_DO_SET_MODE and MAV_CMD_COMPONENT_ARM_DISARM shall be used instead. The flag can still be used to report the armed state.
	MAV_MODE_FLAG_MANUAL_INPUT_ENABLED = 64  // 0b01000000 remote control input is enabled.
	MAV_MODE_FLAG_HIL_ENABLED          = 32  // 0b00100000 hardware in the loop simulation. All motors / actuators are blocked, but internal software is full operational.
	MAV_MODE_FLAG_STABILIZE_ENABLED    = 16  // 0b00010000 system stabilizes electronically its attitude (and optionally position). It needs however further control inputs to move around.
	MAV_MODE_FLAG_GUIDED_ENABLED       = 8   // 0b00001000 guided mode enabled, system flies MISSIONs / mission items.
	MAV_MODE_FLAG_AUTO_ENABLED         = 4   // 0b00000100 autonomous mode enabled, system finds its own goal positions. Guided flag can be set or not, depends on the actual implementation.
	MAV_MODE_FLAG_TEST_ENABLED         = 2   // 0b00000010 system has a test mode enabled. This flag is intended for temporary system tests and should not be used for stable implementations.
	MAV_MODE_FLAG_CUSTOM_MODE_ENABLED  = 1   // 0b00000001 Reserved for future use.
)

// MavModeFlagDecodePosition (generated enum)
// These values encode the bit positions of the decode position. These values can be used to read the value of a flag bit by combining the base_mode variable with AND with the flag position value. The result will be either 0 or 1, depending on if the flag is set or not.
const (
	MAV_MODE_FLAG_DECODE_POSITION_SAFETY      = 128 // First bit:  10000000
	MAV_MODE_FLAG_DECODE_POSITION_MANUAL      = 64  // Second bit: 01000000
	MAV_MODE_FLAG_DECODE_POSITION_HIL         = 32  // Third bit:  00100000
	MAV_MODE_FLAG_DECODE_POSITION_STABILIZE   = 16  // Fourth bit: 00010000
	MAV_MODE_FLAG_DECODE_POSITION_GUIDED      = 8   // Fifth bit:  00001000
	MAV_MODE_FLAG_DECODE_POSITION_AUTO        = 4   // Sixt bit:   00000100
	MAV_MODE_FLAG_DECODE_POSITION_TEST        = 2   // Seventh bit: 00000010
	MAV_MODE_FLAG_DECODE_POSITION_CUSTOM_MODE = 1   // Eighth bit: 00000001
)

// MavGoto (generated enum)
// Override command, pauses current mission execution and moves immediately to a position
const (
	MAV_GOTO_DO_HOLD                    = 0 // Hold at the current position.
	MAV_GOTO_DO_CONTINUE                = 1 // Continue with the next item in mission execution.
	MAV_GOTO_HOLD_AT_CURRENT_POSITION   = 2 // Hold at the current position of the system
	MAV_GOTO_HOLD_AT_SPECIFIED_POSITION = 3 // Hold at the position specified in the parameters of the DO_HOLD action
)

// MavMode (generated enum)
// These defines are predefined OR-combined mode flags. There is no need to use values from this enum, but it                simplifies the use of the mode flags. Note that manual input is enabled in all modes as a safety override.
const (
	MAV_MODE_PREFLIGHT          = 0   // System is not ready to fly, booting, calibrating, etc. No flag is set.
	MAV_MODE_STABILIZE_DISARMED = 80  // System is allowed to be active, under assisted RC control.
	MAV_MODE_STABILIZE_ARMED    = 208 // System is allowed to be active, under assisted RC control.
	MAV_MODE_MANUAL_DISARMED    = 64  // System is allowed to be active, under manual (RC) control, no stabilization
	MAV_MODE_MANUAL_ARMED       = 192 // System is allowed to be active, under manual (RC) control, no stabilization
	MAV_MODE_GUIDED_DISARMED    = 88  // System is allowed to be active, under autonomous control, manual setpoint
	MAV_MODE_GUIDED_ARMED       = 216 // System is allowed to be active, under autonomous control, manual setpoint
	MAV_MODE_AUTO_DISARMED      = 92  // System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard and not pre-programmed by MISSIONs)
	MAV_MODE_AUTO_ARMED         = 220 // System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard and not pre-programmed by MISSIONs)
	MAV_MODE_TEST_DISARMED      = 66  // UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only.
	MAV_MODE_TEST_ARMED         = 194 // UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only.
)

// MavState (generated enum)
//
const (
	MAV_STATE_UNINIT      = 0 // Uninitialized system, state is unknown.
	MAV_STATE_BOOT        = 1 // System is booting up.
	MAV_STATE_CALIBRATING = 2 // System is calibrating and not flight-ready.
	MAV_STATE_STANDBY     = 3 // System is grounded and on standby. It can be launched any time.
	MAV_STATE_ACTIVE      = 4 // System is active and might be already airborne. Motors are engaged.
	MAV_STATE_CRITICAL    = 5 // System is in a non-normal flight mode. It can however still navigate.
	MAV_STATE_EMERGENCY   = 6 // System is in a non-normal flight mode. It lost control over parts or over the whole airframe. It is in mayday and going down.
	MAV_STATE_POWEROFF    = 7 // System just initialized its power-down sequence, will shut down now.
)

// MavComponent (generated enum)
//
const (
	MAV_COMP_ID_ALL            = 0   //
	MAV_COMP_ID_AUTOPILOT1     = 1   //
	MAV_COMP_ID_CAMERA         = 100 //
	MAV_COMP_ID_SERVO1         = 140 //
	MAV_COMP_ID_SERVO2         = 141 //
	MAV_COMP_ID_SERVO3         = 142 //
	MAV_COMP_ID_SERVO4         = 143 //
	MAV_COMP_ID_SERVO5         = 144 //
	MAV_COMP_ID_SERVO6         = 145 //
	MAV_COMP_ID_SERVO7         = 146 //
	MAV_COMP_ID_SERVO8         = 147 //
	MAV_COMP_ID_SERVO9         = 148 //
	MAV_COMP_ID_SERVO10        = 149 //
	MAV_COMP_ID_SERVO11        = 150 //
	MAV_COMP_ID_SERVO12        = 151 //
	MAV_COMP_ID_SERVO13        = 152 //
	MAV_COMP_ID_SERVO14        = 153 //
	MAV_COMP_ID_GIMBAL         = 154 //
	MAV_COMP_ID_LOG            = 155 //
	MAV_COMP_ID_ADSB           = 156 //
	MAV_COMP_ID_OSD            = 157 // On Screen Display (OSD) devices for video links
	MAV_COMP_ID_PERIPHERAL     = 158 // Generic autopilot peripheral component ID. Meant for devices that do not implement the parameter sub-protocol
	MAV_COMP_ID_QX1_GIMBAL     = 159 //
	MAV_COMP_ID_MAPPER         = 180 //
	MAV_COMP_ID_MISSIONPLANNER = 190 //
	MAV_COMP_ID_PATHPLANNER    = 195 //
	MAV_COMP_ID_IMU            = 200 //
	MAV_COMP_ID_IMU_2          = 201 //
	MAV_COMP_ID_IMU_3          = 202 //
	MAV_COMP_ID_GPS            = 220 //
	MAV_COMP_ID_GPS2           = 221 //
	MAV_COMP_ID_UDP_BRIDGE     = 240 //
	MAV_COMP_ID_UART_BRIDGE    = 241 //
	MAV_COMP_ID_SYSTEM_CONTROL = 250 //
)

// MavSysStatusSensor (generated enum)
// These encode the sensors whose status is sent as part of the SYS_STATUS message.
const (
	MAV_SYS_STATUS_SENSOR_3D_GYRO                = 1        // 0x01 3D gyro
	MAV_SYS_STATUS_SENSOR_3D_ACCEL               = 2        // 0x02 3D accelerometer
	MAV_SYS_STATUS_SENSOR_3D_MAG                 = 4        // 0x04 3D magnetometer
	MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE      = 8        // 0x08 absolute pressure
	MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE  = 16       // 0x10 differential pressure
	MAV_SYS_STATUS_SENSOR_GPS                    = 32       // 0x20 GPS
	MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW           = 64       // 0x40 optical flow
	MAV_SYS_STATUS_SENSOR_VISION_POSITION        = 128      // 0x80 computer vision position
	MAV_SYS_STATUS_SENSOR_LASER_POSITION         = 256      // 0x100 laser based position
	MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH  = 512      // 0x200 external ground truth (Vicon or Leica)
	MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL   = 1024     // 0x400 3D angular rate control
	MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION = 2048     // 0x800 attitude stabilization
	MAV_SYS_STATUS_SENSOR_YAW_POSITION           = 4096     // 0x1000 yaw position
	MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL     = 8192     // 0x2000 z/altitude control
	MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL    = 16384    // 0x4000 x/y position control
	MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS          = 32768    // 0x8000 motor outputs / control
	MAV_SYS_STATUS_SENSOR_RC_RECEIVER            = 65536    // 0x10000 rc receiver
	MAV_SYS_STATUS_SENSOR_3D_GYRO2               = 131072   // 0x20000 2nd 3D gyro
	MAV_SYS_STATUS_SENSOR_3D_ACCEL2              = 262144   // 0x40000 2nd 3D accelerometer
	MAV_SYS_STATUS_SENSOR_3D_MAG2                = 524288   // 0x80000 2nd 3D magnetometer
	MAV_SYS_STATUS_GEOFENCE                      = 1048576  // 0x100000 geofence
	MAV_SYS_STATUS_AHRS                          = 2097152  // 0x200000 AHRS subsystem health
	MAV_SYS_STATUS_TERRAIN                       = 4194304  // 0x400000 Terrain subsystem health
	MAV_SYS_STATUS_REVERSE_MOTOR                 = 8388608  // 0x800000 Motors are reversed
	MAV_SYS_STATUS_LOGGING                       = 16777216 // 0x1000000 Logging
	MAV_SYS_STATUS_SENSOR_BATTERY                = 33554432 // 0x2000000 Battery
)

// MavFrame (generated enum)
//
const (
	MAV_FRAME_GLOBAL                  = 0  // Global coordinate frame, WGS84 coordinate system. First value / x: latitude, second value / y: longitude, third value / z: positive altitude over mean sea level (MSL)
	MAV_FRAME_LOCAL_NED               = 1  // Local coordinate frame, Z-up (x: north, y: east, z: down).
	MAV_FRAME_MISSION                 = 2  // NOT a coordinate frame, indicates a mission command.
	MAV_FRAME_GLOBAL_RELATIVE_ALT     = 3  // Global coordinate frame, WGS84 coordinate system, relative altitude over ground with respect to the home position. First value / x: latitude, second value / y: longitude, third value / z: positive altitude with 0 being at the altitude of the home location.
	MAV_FRAME_LOCAL_ENU               = 4  // Local coordinate frame, Z-down (x: east, y: north, z: up)
	MAV_FRAME_GLOBAL_INT              = 5  // Global coordinate frame, WGS84 coordinate system. First value / x: latitude in degrees*1.0e-7, second value / y: longitude in degrees*1.0e-7, third value / z: positive altitude over mean sea level (MSL)
	MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6  // Global coordinate frame, WGS84 coordinate system, relative altitude over ground with respect to the home position. First value / x: latitude in degrees*10e-7, second value / y: longitude in degrees*10e-7, third value / z: positive altitude with 0 being at the altitude of the home location.
	MAV_FRAME_LOCAL_OFFSET_NED        = 7  // Offset to the current local frame. Anything expressed in this frame should be added to the current local frame position.
	MAV_FRAME_BODY_NED                = 8  // Setpoint in body NED frame. This makes sense if all position control is externalized - e.g. useful to command 2 m/s^2 acceleration to the right.
	MAV_FRAME_BODY_OFFSET_NED         = 9  // Offset in body NED frame. This makes sense if adding setpoints to the current flight path, to avoid an obstacle - e.g. useful to command 2 m/s^2 acceleration to the east.
	MAV_FRAME_GLOBAL_TERRAIN_ALT      = 10 // Global coordinate frame with above terrain level altitude. WGS84 coordinate system, relative altitude over terrain with respect to the waypoint coordinate. First value / x: latitude in degrees, second value / y: longitude in degrees, third value / z: positive altitude in meters with 0 being at ground level in terrain model.
	MAV_FRAME_GLOBAL_TERRAIN_ALT_INT  = 11 // Global coordinate frame with above terrain level altitude. WGS84 coordinate system, relative altitude over terrain with respect to the waypoint coordinate. First value / x: latitude in degrees*10e-7, second value / y: longitude in degrees*10e-7, third value / z: positive altitude in meters with 0 being at ground level in terrain model.
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
	FENCE_ACTION_RTL             = 4 // Switch to RTL (return to launch) mode and head for the return point.
)

// FenceBreach (generated enum)
//
const (
	FENCE_BREACH_NONE     = 0 // No last fence breach
	FENCE_BREACH_MINALT   = 1 // Breached minimum altitude
	FENCE_BREACH_MAXALT   = 2 // Breached maximum altitude
	FENCE_BREACH_BOUNDARY = 3 // Breached fence boundary
)

// MavMountMode (generated enum)
// Enumeration of possible mount operation modes
const (
	MAV_MOUNT_MODE_RETRACT           = 0 // Load and keep safe position (Roll,Pitch,Yaw) from permant memory and stop stabilization
	MAV_MOUNT_MODE_NEUTRAL           = 1 // Load and keep neutral position (Roll,Pitch,Yaw) from permanent memory.
	MAV_MOUNT_MODE_MAVLINK_TARGETING = 2 // Load neutral position and start MAVLink Roll,Pitch,Yaw control with stabilization
	MAV_MOUNT_MODE_RC_TARGETING      = 3 // Load neutral position and start RC Roll,Pitch,Yaw control with stabilization
	MAV_MOUNT_MODE_GPS_POINT         = 4 // Load neutral position and start to point to Lat,Lon,Alt
)

// MavCmd (generated enum)
// Commands to be executed by the MAV. They can be executed on user request, or as part of a mission script. If the action is used in a mission, the parameter mapping to the waypoint/mission message is as follows: Param 1, Param 2, Param 3, Param 4, X: Param 5, Y:Param 6, Z:Param 7. This command list is similar what ARINC 424 is for commercial aircraft: A data format how to interpret waypoint/mission data.
const (
	MAV_CMD_NAV_WAYPOINT                       = 16    // Navigate to MISSION.
	MAV_CMD_NAV_LOITER_UNLIM                   = 17    // Loiter around this MISSION an unlimited amount of time
	MAV_CMD_NAV_LOITER_TURNS                   = 18    // Loiter around this MISSION for X turns
	MAV_CMD_NAV_LOITER_TIME                    = 19    // Loiter around this MISSION for X seconds
	MAV_CMD_NAV_RETURN_TO_LAUNCH               = 20    // Return to launch location
	MAV_CMD_NAV_LAND                           = 21    // Land at location
	MAV_CMD_NAV_TAKEOFF                        = 22    // Takeoff from ground / hand
	MAV_CMD_NAV_LAND_LOCAL                     = 23    // Land at local position (local frame only)
	MAV_CMD_NAV_TAKEOFF_LOCAL                  = 24    // Takeoff from local position (local frame only)
	MAV_CMD_NAV_FOLLOW                         = 25    // Vehicle following, i.e. this waypoint represents the position of a moving vehicle
	MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT        = 30    // Continue on the current course and climb/descend to specified altitude.  When the altitude is reached continue to the next command (i.e., don't proceed to the next command until the desired altitude is reached.
	MAV_CMD_NAV_LOITER_TO_ALT                  = 31    // Begin loiter at the specified Latitude and Longitude.  If Lat=Lon=0, then loiter at the current position.  Don't consider the navigation command complete (don't leave loiter) until the altitude has been reached.  Additionally, if the Heading Required parameter is non-zero the  aircraft will not leave the loiter until heading toward the next waypoint.
	MAV_CMD_DO_FOLLOW                          = 32    // Being following a target
	MAV_CMD_DO_FOLLOW_REPOSITION               = 33    // Reposition the MAV after a follow target command has been sent
	MAV_CMD_NAV_ROI                            = 80    // Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras.
	MAV_CMD_NAV_PATHPLANNING                   = 81    // Control autonomous path planning on the MAV.
	MAV_CMD_NAV_SPLINE_WAYPOINT                = 82    // Navigate to MISSION using a spline path.
	MAV_CMD_NAV_VTOL_TAKEOFF                   = 84    // Takeoff from ground using VTOL mode
	MAV_CMD_NAV_VTOL_LAND                      = 85    // Land using VTOL mode
	MAV_CMD_NAV_GUIDED_ENABLE                  = 92    // hand control over to an external controller
	MAV_CMD_NAV_DELAY                          = 93    // Delay the next navigation command a number of seconds or until a specified time
	MAV_CMD_NAV_PAYLOAD_PLACE                  = 94    // Descend and place payload.  Vehicle descends until it detects a hanging payload has reached the ground, the gripper is opened to release the payload
	MAV_CMD_NAV_LAST                           = 95    // NOP - This command is only used to mark the upper limit of the NAV/ACTION commands in the enumeration
	MAV_CMD_CONDITION_DELAY                    = 112   // Delay mission state machine.
	MAV_CMD_CONDITION_CHANGE_ALT               = 113   // Ascend/descend at rate.  Delay mission state machine until desired altitude reached.
	MAV_CMD_CONDITION_DISTANCE                 = 114   // Delay mission state machine until within desired distance of next NAV point.
	MAV_CMD_CONDITION_YAW                      = 115   // Reach a certain target angle.
	MAV_CMD_CONDITION_LAST                     = 159   // NOP - This command is only used to mark the upper limit of the CONDITION commands in the enumeration
	MAV_CMD_DO_SET_MODE                        = 176   // Set system mode.
	MAV_CMD_DO_JUMP                            = 177   // Jump to the desired command in the mission list.  Repeat this action only the specified number of times
	MAV_CMD_DO_CHANGE_SPEED                    = 178   // Change speed and/or throttle set points.
	MAV_CMD_DO_SET_HOME                        = 179   // Changes the home location either to the current location or a specified location.
	MAV_CMD_DO_SET_PARAMETER                   = 180   // Set a system parameter.  Caution!  Use of this command requires knowledge of the numeric enumeration value of the parameter.
	MAV_CMD_DO_SET_RELAY                       = 181   // Set a relay to a condition.
	MAV_CMD_DO_REPEAT_RELAY                    = 182   // Cycle a relay on and off for a desired number of cyles with a desired period.
	MAV_CMD_DO_SET_SERVO                       = 183   // Set a servo to a desired PWM value.
	MAV_CMD_DO_REPEAT_SERVO                    = 184   // Cycle a between its nominal setting and a desired PWM for a desired number of cycles with a desired period.
	MAV_CMD_DO_FLIGHTTERMINATION               = 185   // Terminate flight immediately
	MAV_CMD_DO_CHANGE_ALTITUDE                 = 186   // Change altitude set point.
	MAV_CMD_DO_LAND_START                      = 189   // Mission command to perform a landing. This is used as a marker in a mission to tell the autopilot where a sequence of mission items that represents a landing starts. It may also be sent via a COMMAND_LONG to trigger a landing, in which case the nearest (geographically) landing sequence in the mission will be used. The Latitude/Longitude is optional, and may be set to 0 if not needed. If specified then it will be used to help find the closest landing sequence.
	MAV_CMD_DO_RALLY_LAND                      = 190   // Mission command to perform a landing from a rally point.
	MAV_CMD_DO_GO_AROUND                       = 191   // Mission command to safely abort an autonmous landing.
	MAV_CMD_DO_REPOSITION                      = 192   // Reposition the vehicle to a specific WGS84 global position.
	MAV_CMD_DO_PAUSE_CONTINUE                  = 193   // If in a GPS controlled position mode, hold the current position or continue.
	MAV_CMD_DO_SET_REVERSE                     = 194   // Set moving direction to forward or reverse.
	MAV_CMD_DO_CONTROL_VIDEO                   = 200   // Control onboard camera system.
	MAV_CMD_DO_SET_ROI                         = 201   // Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras.
	MAV_CMD_DO_DIGICAM_CONFIGURE               = 202   // Mission command to configure an on-board camera controller system.
	MAV_CMD_DO_DIGICAM_CONTROL                 = 203   // Mission command to control an on-board camera controller system.
	MAV_CMD_DO_MOUNT_CONFIGURE                 = 204   // Mission command to configure a camera or antenna mount
	MAV_CMD_DO_MOUNT_CONTROL                   = 205   // Mission command to control a camera or antenna mount
	MAV_CMD_DO_SET_CAM_TRIGG_DIST              = 206   // Mission command to set camera trigger distance for this flight. The camera is trigerred each time this distance is exceeded. This command can also be used to set the shutter integration time for the camera.
	MAV_CMD_DO_FENCE_ENABLE                    = 207   // Mission command to enable the geofence
	MAV_CMD_DO_PARACHUTE                       = 208   // Mission command to trigger a parachute
	MAV_CMD_DO_MOTOR_TEST                      = 209   // Mission command to perform motor test
	MAV_CMD_DO_INVERTED_FLIGHT                 = 210   // Change to/from inverted flight
	MAV_CMD_NAV_SET_YAW_SPEED                  = 213   // Sets a desired vehicle turn angle and speed change
	MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL          = 214   // Mission command to set camera trigger interval for this flight. If triggering is enabled, the camera is triggered each time this interval expires. This command can also be used to set the shutter integration time for the camera.
	MAV_CMD_DO_MOUNT_CONTROL_QUAT              = 220   // Mission command to control a camera or antenna mount, using a quaternion as reference.
	MAV_CMD_DO_GUIDED_MASTER                   = 221   // set id of master controller
	MAV_CMD_DO_GUIDED_LIMITS                   = 222   // set limits for external control
	MAV_CMD_DO_ENGINE_CONTROL                  = 223   // Control vehicle engine. This is interpreted by the vehicles engine controller to change the target engine state. It is intended for vehicles with internal combustion engines
	MAV_CMD_DO_LAST                            = 240   // NOP - This command is only used to mark the upper limit of the DO commands in the enumeration
	MAV_CMD_PREFLIGHT_CALIBRATION              = 241   // Trigger calibration. This command will be only accepted if in pre-flight mode. Except for Temperature Calibration, only one sensor should be set in a single message and all others should be zero.
	MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS       = 242   // Set sensor offsets. This command will be only accepted if in pre-flight mode.
	MAV_CMD_PREFLIGHT_UAVCAN                   = 243   // Trigger UAVCAN config. This command will be only accepted if in pre-flight mode.
	MAV_CMD_PREFLIGHT_STORAGE                  = 245   // Request storage of different parameter values and logs. This command will be only accepted if in pre-flight mode.
	MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN          = 246   // Request the reboot or shutdown of system components.
	MAV_CMD_OVERRIDE_GOTO                      = 252   // Hold / continue the current action
	MAV_CMD_MISSION_START                      = 300   // start running a mission
	MAV_CMD_COMPONENT_ARM_DISARM               = 400   // Arms / Disarms a component
	MAV_CMD_GET_HOME_POSITION                  = 410   // Request the home position from the vehicle.
	MAV_CMD_START_RX_PAIR                      = 500   // Starts receiver pairing
	MAV_CMD_GET_MESSAGE_INTERVAL               = 510   // Request the interval between messages for a particular MAVLink message ID
	MAV_CMD_SET_MESSAGE_INTERVAL               = 511   // Request the interval between messages for a particular MAVLink message ID. This interface replaces REQUEST_DATA_STREAM
	MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES     = 520   // Request autopilot capabilities
	MAV_CMD_REQUEST_CAMERA_INFORMATION         = 521   // WIP: Request camera information (CAMERA_INFORMATION)
	MAV_CMD_REQUEST_CAMERA_SETTINGS            = 522   // WIP: Request camera settings (CAMERA_SETTINGS)
	MAV_CMD_SET_CAMERA_SETTINGS_1              = 523   // WIP: Set the camera settings part 1 (CAMERA_SETTINGS). Use NAN for values you don't want to change.
	MAV_CMD_SET_CAMERA_SETTINGS_2              = 524   // WIP: Set the camera settings part 2 (CAMERA_SETTINGS). Use NAN for values you don't want to change.
	MAV_CMD_REQUEST_STORAGE_INFORMATION        = 525   // WIP: Request storage information (STORAGE_INFORMATION)
	MAV_CMD_STORAGE_FORMAT                     = 526   // WIP: Format a storage medium
	MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS      = 527   // WIP: Request camera capture status (CAMERA_CAPTURE_STATUS)
	MAV_CMD_REQUEST_FLIGHT_INFORMATION         = 528   // WIP: Request flight information (FLIGHT_INFORMATION)
	MAV_CMD_RESET_CAMERA_SETTINGS              = 529   // WIP: Reset all camera settings to Factory Default (CAMERA_SETTINGS)
	MAV_CMD_SET_CAMERA_MODE                    = 530   // WIP: Set camera running mode. Use NAN for values you don't want to change.
	MAV_CMD_IMAGE_START_CAPTURE                = 2000  // WIP: Start image capture sequence. Sends CAMERA_IMAGE_CAPTURED after each capture.
	MAV_CMD_IMAGE_STOP_CAPTURE                 = 2001  // WIP: Stop image capture sequence
	MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE       = 2002  // WIP: Re-request a CAMERA_IMAGE_CAPTURE packet
	MAV_CMD_DO_TRIGGER_CONTROL                 = 2003  // Enable or disable on-board camera triggering system.
	MAV_CMD_VIDEO_START_CAPTURE                = 2500  // WIP: Starts video capture (recording)
	MAV_CMD_VIDEO_STOP_CAPTURE                 = 2501  // WIP: Stop the current video capture (recording)
	MAV_CMD_VIDEO_START_STREAMING              = 2502  // WIP: Start video streaming
	MAV_CMD_VIDEO_STOP_STREAMING               = 2503  // WIP: Stop the current video streaming
	MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION   = 2504  // WIP: Request video stream information (VIDEO_STREAM_INFORMATION)
	MAV_CMD_LOGGING_START                      = 2510  // Request to start streaming logging data over MAVLink (see also LOGGING_DATA message)
	MAV_CMD_LOGGING_STOP                       = 2511  // Request to stop streaming log data over MAVLink
	MAV_CMD_AIRFRAME_CONFIGURATION             = 2520  //
	MAV_CMD_PANORAMA_CREATE                    = 2800  // Create a panorama at the current position
	MAV_CMD_DO_VTOL_TRANSITION                 = 3000  // Request VTOL transition
	MAV_CMD_SET_GUIDED_SUBMODE_STANDARD        = 4000  // This command sets the submode to standard guided when vehicle is in guided mode. The vehicle holds position and altitude and the user can input the desired velocites along all three axes.
	MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE          = 4001  // This command sets submode circle when vehicle is in guided mode. Vehicle flies along a circle facing the center of the circle. The user can input the velocity along the circle and change the radius. If no input is given the vehicle will hold position.
	MAV_CMD_NAV_FENCE_RETURN_POINT             = 5000  // Fence return point. There can only be one fence return point.
	MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION = 5001  // Fence vertex for an inclusion polygon. The vehicle must stay within this area. Minimum of 3 vertices required.
	MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION = 5002  // Fence vertex for an exclusion polygon. The vehicle must stay outside this area. Minimum of 3 vertices required.
	MAV_CMD_NAV_RALLY_POINT                    = 5100  // Rally point. You can have multiple rally points defined.
	MAV_CMD_PAYLOAD_PREPARE_DEPLOY             = 30001 // Deploy payload on a Lat / Lon / Alt position. This includes the navigation to reach the required release position and velocity.
	MAV_CMD_PAYLOAD_CONTROL_DEPLOY             = 30002 // Control the payload deployment.
	MAV_CMD_WAYPOINT_USER_1                    = 31000 // User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
	MAV_CMD_WAYPOINT_USER_2                    = 31001 // User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
	MAV_CMD_WAYPOINT_USER_3                    = 31002 // User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
	MAV_CMD_WAYPOINT_USER_4                    = 31003 // User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
	MAV_CMD_WAYPOINT_USER_5                    = 31004 // User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
	MAV_CMD_SPATIAL_USER_1                     = 31005 // User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item.
	MAV_CMD_SPATIAL_USER_2                     = 31006 // User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item.
	MAV_CMD_SPATIAL_USER_3                     = 31007 // User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item.
	MAV_CMD_SPATIAL_USER_4                     = 31008 // User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item.
	MAV_CMD_SPATIAL_USER_5                     = 31009 // User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item.
	MAV_CMD_USER_1                             = 31010 // User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item.
	MAV_CMD_USER_2                             = 31011 // User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item.
	MAV_CMD_USER_3                             = 31012 // User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item.
	MAV_CMD_USER_4                             = 31013 // User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item.
	MAV_CMD_USER_5                             = 31014 // User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item.
)

// MavDataStream (generated enum)
// THIS INTERFACE IS DEPRECATED AS OF JULY 2015. Please use MESSAGE_INTERVAL instead. A data stream is not a fixed set of messages, but rather a      recommendation to the autopilot software. Individual autopilots may or may not obey      the recommended messages.
const (
	MAV_DATA_STREAM_ALL             = 0  // Enable all data streams
	MAV_DATA_STREAM_RAW_SENSORS     = 1  // Enable IMU_RAW, GPS_RAW, GPS_STATUS packets.
	MAV_DATA_STREAM_EXTENDED_STATUS = 2  // Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS
	MAV_DATA_STREAM_RC_CHANNELS     = 3  // Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW
	MAV_DATA_STREAM_RAW_CONTROLLER  = 4  // Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT.
	MAV_DATA_STREAM_POSITION        = 6  // Enable LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT messages.
	MAV_DATA_STREAM_EXTRA1          = 10 // Dependent on the autopilot
	MAV_DATA_STREAM_EXTRA2          = 11 // Dependent on the autopilot
	MAV_DATA_STREAM_EXTRA3          = 12 // Dependent on the autopilot
)

// MavRoi (generated enum)
//  The ROI (region of interest) for the vehicle. This can be                 be used by the vehicle for camera/vehicle attitude alignment (see                 MAV_CMD_NAV_ROI).
const (
	MAV_ROI_NONE     = 0 // No region of interest.
	MAV_ROI_WPNEXT   = 1 // Point toward next MISSION.
	MAV_ROI_WPINDEX  = 2 // Point toward given MISSION.
	MAV_ROI_LOCATION = 3 // Point toward fixed location.
	MAV_ROI_TARGET   = 4 // Point toward of given id.
)

// MavCmdAck (generated enum)
// ACK / NACK / ERROR values as a result of MAV_CMDs and for mission item transmission.
const (
	MAV_CMD_ACK_OK                                 = 0 // Command / mission item is ok.
	MAV_CMD_ACK_ERR_FAIL                           = 1 // Generic error message if none of the other reasons fails or if no detailed error reporting is implemented.
	MAV_CMD_ACK_ERR_ACCESS_DENIED                  = 2 // The system is refusing to accept this command from this source / communication partner.
	MAV_CMD_ACK_ERR_NOT_SUPPORTED                  = 3 // Command or mission item is not supported, other commands would be accepted.
	MAV_CMD_ACK_ERR_COORDINATE_FRAME_NOT_SUPPORTED = 4 // The coordinate frame of this command / mission item is not supported.
	MAV_CMD_ACK_ERR_COORDINATES_OUT_OF_RANGE       = 5 // The coordinate frame of this command is ok, but he coordinate values exceed the safety limits of this system. This is a generic error, please use the more specific error messages below if possible.
	MAV_CMD_ACK_ERR_X_LAT_OUT_OF_RANGE             = 6 // The X or latitude value is out of range.
	MAV_CMD_ACK_ERR_Y_LON_OUT_OF_RANGE             = 7 // The Y or longitude value is out of range.
	MAV_CMD_ACK_ERR_Z_ALT_OUT_OF_RANGE             = 8 // The Z or altitude value is out of range.
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

// MavResult (generated enum)
// result from a mavlink command
const (
	MAV_RESULT_ACCEPTED             = 0 // Command ACCEPTED and EXECUTED
	MAV_RESULT_TEMPORARILY_REJECTED = 1 // Command TEMPORARY REJECTED/DENIED
	MAV_RESULT_DENIED               = 2 // Command PERMANENTLY DENIED
	MAV_RESULT_UNSUPPORTED          = 3 // Command UNKNOWN/UNSUPPORTED
	MAV_RESULT_FAILED               = 4 // Command executed, but failed
	MAV_RESULT_IN_PROGRESS          = 5 // WIP: Command being executed
)

// MavMissionResult (generated enum)
// result in a mavlink mission ack
const (
	MAV_MISSION_ACCEPTED          = 0  // mission accepted OK
	MAV_MISSION_ERROR             = 1  // generic error / not accepting mission commands at all right now
	MAV_MISSION_UNSUPPORTED_FRAME = 2  // coordinate frame is not supported
	MAV_MISSION_UNSUPPORTED       = 3  // command is not supported
	MAV_MISSION_NO_SPACE          = 4  // mission item exceeds storage space
	MAV_MISSION_INVALID           = 5  // one of the parameters has an invalid value
	MAV_MISSION_INVALID_PARAM1    = 6  // param1 has an invalid value
	MAV_MISSION_INVALID_PARAM2    = 7  // param2 has an invalid value
	MAV_MISSION_INVALID_PARAM3    = 8  // param3 has an invalid value
	MAV_MISSION_INVALID_PARAM4    = 9  // param4 has an invalid value
	MAV_MISSION_INVALID_PARAM5_X  = 10 // x/param5 has an invalid value
	MAV_MISSION_INVALID_PARAM6_Y  = 11 // y/param6 has an invalid value
	MAV_MISSION_INVALID_PARAM7    = 12 // param7 has an invalid value
	MAV_MISSION_INVALID_SEQUENCE  = 13 // received waypoint out of sequence
	MAV_MISSION_DENIED            = 14 // not accepting any mission commands from this communication partner
)

// MavSeverity (generated enum)
// Indicates the severity level, generally used for status messages to indicate their relative urgency. Based on RFC-5424 using expanded definitions at: http://www.kiwisyslog.com/kb/info:-syslog-message-levels/.
const (
	MAV_SEVERITY_EMERGENCY = 0 // System is unusable. This is a "panic" condition.
	MAV_SEVERITY_ALERT     = 1 // Action should be taken immediately. Indicates error in non-critical systems.
	MAV_SEVERITY_CRITICAL  = 2 // Action must be taken immediately. Indicates failure in a primary system.
	MAV_SEVERITY_ERROR     = 3 // Indicates an error in secondary/redundant systems.
	MAV_SEVERITY_WARNING   = 4 // Indicates about a possible future error if this is not resolved within a given timeframe. Example would be a low battery warning.
	MAV_SEVERITY_NOTICE    = 5 // An unusual event has occured, though not an error condition. This should be investigated for the root cause.
	MAV_SEVERITY_INFO      = 6 // Normal operational messages. Useful for logging. No action is required for these messages.
	MAV_SEVERITY_DEBUG     = 7 // Useful non-operational messages that can assist in debugging. These should not occur during normal operation.
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
	SERIAL_CONTROL_DEV_TELEM1 = 0  // First telemetry port
	SERIAL_CONTROL_DEV_TELEM2 = 1  // Second telemetry port
	SERIAL_CONTROL_DEV_GPS1   = 2  // First GPS port
	SERIAL_CONTROL_DEV_GPS2   = 3  // Second GPS port
	SERIAL_CONTROL_DEV_SHELL  = 10 // system shell
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
)

// MavSensorOrientation (generated enum)
// Enumeration of sensor orientation, according to its rotations
const (
	MAV_SENSOR_ROTATION_NONE                       = 0  // Roll: 0, Pitch: 0, Yaw: 0
	MAV_SENSOR_ROTATION_YAW_45                     = 1  // Roll: 0, Pitch: 0, Yaw: 45
	MAV_SENSOR_ROTATION_YAW_90                     = 2  // Roll: 0, Pitch: 0, Yaw: 90
	MAV_SENSOR_ROTATION_YAW_135                    = 3  // Roll: 0, Pitch: 0, Yaw: 135
	MAV_SENSOR_ROTATION_YAW_180                    = 4  // Roll: 0, Pitch: 0, Yaw: 180
	MAV_SENSOR_ROTATION_YAW_225                    = 5  // Roll: 0, Pitch: 0, Yaw: 225
	MAV_SENSOR_ROTATION_YAW_270                    = 6  // Roll: 0, Pitch: 0, Yaw: 270
	MAV_SENSOR_ROTATION_YAW_315                    = 7  // Roll: 0, Pitch: 0, Yaw: 315
	MAV_SENSOR_ROTATION_ROLL_180                   = 8  // Roll: 180, Pitch: 0, Yaw: 0
	MAV_SENSOR_ROTATION_ROLL_180_YAW_45            = 9  // Roll: 180, Pitch: 0, Yaw: 45
	MAV_SENSOR_ROTATION_ROLL_180_YAW_90            = 10 // Roll: 180, Pitch: 0, Yaw: 90
	MAV_SENSOR_ROTATION_ROLL_180_YAW_135           = 11 // Roll: 180, Pitch: 0, Yaw: 135
	MAV_SENSOR_ROTATION_PITCH_180                  = 12 // Roll: 0, Pitch: 180, Yaw: 0
	MAV_SENSOR_ROTATION_ROLL_180_YAW_225           = 13 // Roll: 180, Pitch: 0, Yaw: 225
	MAV_SENSOR_ROTATION_ROLL_180_YAW_270           = 14 // Roll: 180, Pitch: 0, Yaw: 270
	MAV_SENSOR_ROTATION_ROLL_180_YAW_315           = 15 // Roll: 180, Pitch: 0, Yaw: 315
	MAV_SENSOR_ROTATION_ROLL_90                    = 16 // Roll: 90, Pitch: 0, Yaw: 0
	MAV_SENSOR_ROTATION_ROLL_90_YAW_45             = 17 // Roll: 90, Pitch: 0, Yaw: 45
	MAV_SENSOR_ROTATION_ROLL_90_YAW_90             = 18 // Roll: 90, Pitch: 0, Yaw: 90
	MAV_SENSOR_ROTATION_ROLL_90_YAW_135            = 19 // Roll: 90, Pitch: 0, Yaw: 135
	MAV_SENSOR_ROTATION_ROLL_270                   = 20 // Roll: 270, Pitch: 0, Yaw: 0
	MAV_SENSOR_ROTATION_ROLL_270_YAW_45            = 21 // Roll: 270, Pitch: 0, Yaw: 45
	MAV_SENSOR_ROTATION_ROLL_270_YAW_90            = 22 // Roll: 270, Pitch: 0, Yaw: 90
	MAV_SENSOR_ROTATION_ROLL_270_YAW_135           = 23 // Roll: 270, Pitch: 0, Yaw: 135
	MAV_SENSOR_ROTATION_PITCH_90                   = 24 // Roll: 0, Pitch: 90, Yaw: 0
	MAV_SENSOR_ROTATION_PITCH_270                  = 25 // Roll: 0, Pitch: 270, Yaw: 0
	MAV_SENSOR_ROTATION_PITCH_180_YAW_90           = 26 // Roll: 0, Pitch: 180, Yaw: 90
	MAV_SENSOR_ROTATION_PITCH_180_YAW_270          = 27 // Roll: 0, Pitch: 180, Yaw: 270
	MAV_SENSOR_ROTATION_ROLL_90_PITCH_90           = 28 // Roll: 90, Pitch: 90, Yaw: 0
	MAV_SENSOR_ROTATION_ROLL_180_PITCH_90          = 29 // Roll: 180, Pitch: 90, Yaw: 0
	MAV_SENSOR_ROTATION_ROLL_270_PITCH_90          = 30 // Roll: 270, Pitch: 90, Yaw: 0
	MAV_SENSOR_ROTATION_ROLL_90_PITCH_180          = 31 // Roll: 90, Pitch: 180, Yaw: 0
	MAV_SENSOR_ROTATION_ROLL_270_PITCH_180         = 32 // Roll: 270, Pitch: 180, Yaw: 0
	MAV_SENSOR_ROTATION_ROLL_90_PITCH_270          = 33 // Roll: 90, Pitch: 270, Yaw: 0
	MAV_SENSOR_ROTATION_ROLL_180_PITCH_270         = 34 // Roll: 180, Pitch: 270, Yaw: 0
	MAV_SENSOR_ROTATION_ROLL_270_PITCH_270         = 35 // Roll: 270, Pitch: 270, Yaw: 0
	MAV_SENSOR_ROTATION_ROLL_90_PITCH_180_YAW_90   = 36 // Roll: 90, Pitch: 180, Yaw: 90
	MAV_SENSOR_ROTATION_ROLL_90_YAW_270            = 37 // Roll: 90, Pitch: 0, Yaw: 270
	MAV_SENSOR_ROTATION_ROLL_315_PITCH_315_YAW_315 = 38 // Roll: 315, Pitch: 315, Yaw: 315
)

// MavProtocolCapability (generated enum)
// Bitmask of (optional) autopilot capabilities (64 bit). If a bit is set, the autopilot supports this capability.
const (
	MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT                  = 1     // Autopilot supports MISSION float message type.
	MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT                    = 2     // Autopilot supports the new param float message type.
	MAV_PROTOCOL_CAPABILITY_MISSION_INT                    = 4     // Autopilot supports MISSION_INT scaled integer message type.
	MAV_PROTOCOL_CAPABILITY_COMMAND_INT                    = 8     // Autopilot supports COMMAND_INT scaled integer message type.
	MAV_PROTOCOL_CAPABILITY_PARAM_UNION                    = 16    // Autopilot supports the new param union message type.
	MAV_PROTOCOL_CAPABILITY_FTP                            = 32    // Autopilot supports the new FILE_TRANSFER_PROTOCOL message type.
	MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET            = 64    // Autopilot supports commanding attitude offboard.
	MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED  = 128   // Autopilot supports commanding position and velocity targets in local NED frame.
	MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT = 256   // Autopilot supports commanding position and velocity targets in global scaled integers.
	MAV_PROTOCOL_CAPABILITY_TERRAIN                        = 512   // Autopilot supports terrain protocol / data handling.
	MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET            = 1024  // Autopilot supports direct actuator control.
	MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION             = 2048  // Autopilot supports the flight termination command.
	MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION            = 4096  // Autopilot supports onboard compass calibration.
	MAV_PROTOCOL_CAPABILITY_MAVLINK2                       = 8192  // Autopilot supports mavlink version 2.
	MAV_PROTOCOL_CAPABILITY_MISSION_FENCE                  = 16384 // Autopilot supports mission fence protocol.
	MAV_PROTOCOL_CAPABILITY_MISSION_RALLY                  = 32768 // Autopilot supports mission rally point protocol.
	MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION             = 65536 // Autopilot supports the flight information protocol.
)

// MavMissionType (generated enum)
// Type of mission items being requested/sent in mission protocol.
const (
	MAV_MISSION_TYPE_MISSION = 0   // Items are mission commands for main mission.
	MAV_MISSION_TYPE_FENCE   = 1   // Specifies GeoFence area(s). Items are MAV_CMD_FENCE_ GeoFence items.
	MAV_MISSION_TYPE_RALLY   = 2   // Specifies the rally points for the vehicle. Rally points are alternative RTL points. Items are MAV_CMD_RALLY_POINT rally point items.
	MAV_MISSION_TYPE_ALL     = 255 // Only used in MISSION_CLEAR_ALL to clear all mission types.
)

// MavEstimatorType (generated enum)
// Enumeration of estimator types
const (
	MAV_ESTIMATOR_TYPE_NAIVE   = 1 // This is a naive estimator without any real covariance feedback.
	MAV_ESTIMATOR_TYPE_VISION  = 2 // Computer vision based estimate. Might be up to scale.
	MAV_ESTIMATOR_TYPE_VIO     = 3 // Visual-inertial estimate.
	MAV_ESTIMATOR_TYPE_GPS     = 4 // Plain GPS estimate.
	MAV_ESTIMATOR_TYPE_GPS_INS = 5 // Estimator integrating GPS and inertial sensing.
)

// MavBatteryType (generated enum)
// Enumeration of battery types
const (
	MAV_BATTERY_TYPE_UNKNOWN = 0 // Not specified.
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
	ADSB_FLAGS_VALID_COORDS   = 1  //
	ADSB_FLAGS_VALID_ALTITUDE = 2  //
	ADSB_FLAGS_VALID_HEADING  = 4  //
	ADSB_FLAGS_VALID_VELOCITY = 8  //
	ADSB_FLAGS_VALID_CALLSIGN = 16 //
	ADSB_FLAGS_VALID_SQUAWK   = 32 //
	ADSB_FLAGS_SIMULATED      = 64 //
)

// MavDoRepositionFlags (generated enum)
// Bitmask of options for the MAV_CMD_DO_REPOSITION
const (
	MAV_DO_REPOSITION_FLAGS_CHANGE_MODE = 1 // The aircraft should immediately transition into guided. This should not be set for follow me applications
)

// EstimatorStatusFlags (generated enum)
// Flags in EKF_STATUS message
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
)

// MotorTestThrottleType (generated enum)
//
const (
	MOTOR_TEST_THROTTLE_PERCENT = 0 // throttle as a percentage from 0 ~ 100
	MOTOR_TEST_THROTTLE_PWM     = 1 // throttle as an absolute PWM value (normally in range of 1000~2000)
	MOTOR_TEST_THROTTLE_PILOT   = 2 // throttle pass-through from pilot's transmitter
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
	MAV_COLLISION_THREAT_LEVEL_HIGH = 2 // Craft is panicing, and may take actions to avoid threat
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
)

// Heartbeat struct (generated typeinfo)
// The heartbeat message shows that a system is present and responding. The type of the MAV and Autopilot hardware allow the receiving system to treat further messages from this system appropriate (e.g. by laying out the user interface based on the autopilot).
type Heartbeat struct {
	CustomMode     uint32 // A bitfield for use for autopilot-specific flags.
	Type           uint8  // Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
	Autopilot      uint8  // Autopilot type / class. defined in MAV_AUTOPILOT ENUM
	BaseMode       uint8  // System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.h
	SystemStatus   uint8  // System status flag, see MAV_STATE ENUM
	MavlinkVersion uint8  // MAVLink version, not writable by user, gets added by protocol because of magic data type: uint8_t_mavlink_version
}

// MsgID (generated function)
func (m *Heartbeat) MsgID() MessageID {
	return MSG_ID_HEARTBEAT
}

// MsgName (generated function)
func (m *Heartbeat) MsgName() string {
	return "Heartbeat"
}

// Pack (generated function)
func (m *Heartbeat) Pack(p *Packet) error {
	payload := make([]byte, 9)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.CustomMode))
	payload[4] = byte(m.Type)
	payload[5] = byte(m.Autopilot)
	payload[6] = byte(m.BaseMode)
	payload[7] = byte(m.SystemStatus)
	payload[8] = byte(m.MavlinkVersion)

	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *Heartbeat) Unpack(p *Packet) error {
	if len(p.Payload) < 9 {
		return fmt.Errorf("payload too small")
	}
	m.CustomMode = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.Type = uint8(p.Payload[4])
	m.Autopilot = uint8(p.Payload[5])
	m.BaseMode = uint8(p.Payload[6])
	m.SystemStatus = uint8(p.Payload[7])
	m.MavlinkVersion = uint8(p.Payload[8])
	return nil
}

// SysStatus struct (generated typeinfo)
// The general system state. If the system is following the MAVLink standard, the system state is mainly defined by three orthogonal states/modes: The system mode, which is either LOCKED (motors shut down and locked), MANUAL (system under RC control), GUIDED (system with autonomous position control, position setpoint controlled manually) or AUTO (system guided by path/waypoint planner). The NAV_MODE defined the current flight state: LIFTOFF (often an open-loop maneuver), LANDING, WAYPOINTS or VECTOR. This represents the internal navigation state machine. The system status shows wether the system is currently active or not and if an emergency occured. During the CRITICAL and EMERGENCY states the MAV is still considered to be active, but should start emergency procedures autonomously. After a failure occured it should first move from active to critical to allow manual intervention and then move to emergency after a certain timeout.
type SysStatus struct {
	OnboardControlSensorsPresent uint32 // Bitmask showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present. Indices defined by ENUM MAV_SYS_STATUS_SENSOR
	OnboardControlSensorsEnabled uint32 // Bitmask showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSOR
	OnboardControlSensorsHealth  uint32 // Bitmask showing which onboard controllers and sensors are operational or have an error:  Value of 0: not enabled. Value of 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSOR
	Load                         uint16 // Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 1000
	VoltageBattery               uint16 // Battery voltage, in millivolts (1 = 1 millivolt)
	CurrentBattery               int16  // Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
	DropRateComm                 uint16 // Communication drops in percent, (0%: 0, 100%: 10'000), (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
	ErrorsComm                   uint16 // Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
	ErrorsCount1                 uint16 // Autopilot-specific errors
	ErrorsCount2                 uint16 // Autopilot-specific errors
	ErrorsCount3                 uint16 // Autopilot-specific errors
	ErrorsCount4                 uint16 // Autopilot-specific errors
	BatteryRemaining             int8   // Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot estimate the remaining battery
}

// MsgID (generated function)
func (m *SysStatus) MsgID() MessageID {
	return MSG_ID_SYS_STATUS
}

// MsgName (generated function)
func (m *SysStatus) MsgName() string {
	return "SysStatus"
}

// Pack (generated function)
func (m *SysStatus) Pack(p *Packet) error {
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
func (m *SysStatus) Unpack(p *Packet) error {
	if len(p.Payload) < 31 {
		return fmt.Errorf("payload too small")
	}
	m.OnboardControlSensorsPresent = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.OnboardControlSensorsEnabled = uint32(binary.LittleEndian.Uint32(p.Payload[4:]))
	m.OnboardControlSensorsHealth = uint32(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.Load = uint16(binary.LittleEndian.Uint16(p.Payload[12:]))
	m.VoltageBattery = uint16(binary.LittleEndian.Uint16(p.Payload[14:]))
	m.CurrentBattery = int16(binary.LittleEndian.Uint16(p.Payload[16:]))
	m.DropRateComm = uint16(binary.LittleEndian.Uint16(p.Payload[18:]))
	m.ErrorsComm = uint16(binary.LittleEndian.Uint16(p.Payload[20:]))
	m.ErrorsCount1 = uint16(binary.LittleEndian.Uint16(p.Payload[22:]))
	m.ErrorsCount2 = uint16(binary.LittleEndian.Uint16(p.Payload[24:]))
	m.ErrorsCount3 = uint16(binary.LittleEndian.Uint16(p.Payload[26:]))
	m.ErrorsCount4 = uint16(binary.LittleEndian.Uint16(p.Payload[28:]))
	m.BatteryRemaining = int8(p.Payload[30])
	return nil
}

// SystemTime struct (generated typeinfo)
// The system time is the time of the master clock, typically the computer clock of the main onboard computer.
type SystemTime struct {
	TimeUnixUsec uint64 // Timestamp of the master clock in microseconds since UNIX epoch.
	TimeBootMs   uint32 // Timestamp of the component clock since boot time in milliseconds.
}

// MsgID (generated function)
func (m *SystemTime) MsgID() MessageID {
	return MSG_ID_SYSTEM_TIME
}

// MsgName (generated function)
func (m *SystemTime) MsgName() string {
	return "SystemTime"
}

// Pack (generated function)
func (m *SystemTime) Pack(p *Packet) error {
	payload := make([]byte, 12)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.TimeUnixUsec))
	binary.LittleEndian.PutUint32(payload[8:], uint32(m.TimeBootMs))

	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *SystemTime) Unpack(p *Packet) error {
	if len(p.Payload) < 12 {
		return fmt.Errorf("payload too small")
	}
	m.TimeUnixUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	m.TimeBootMs = uint32(binary.LittleEndian.Uint32(p.Payload[8:]))
	return nil
}

// Ping struct (generated typeinfo)
// A ping message either requesting or responding to a ping. This allows to measure the system latencies, including serial port, radio modem and UDP connections.
type Ping struct {
	TimeUsec        uint64 // Unix timestamp in microseconds or since system boot if smaller than MAVLink epoch (1.1.2009)
	Seq             uint32 // PING sequence
	TargetSystem    uint8  // 0: request ping from all receiving systems, if greater than 0: message is a ping response and number is the system id of the requesting system
	TargetComponent uint8  // 0: request ping from all receiving components, if greater than 0: message is a ping response and number is the system id of the requesting system
}

// MsgID (generated function)
func (m *Ping) MsgID() MessageID {
	return MSG_ID_PING
}

// MsgName (generated function)
func (m *Ping) MsgName() string {
	return "Ping"
}

// Pack (generated function)
func (m *Ping) Pack(p *Packet) error {
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
func (m *Ping) Unpack(p *Packet) error {
	if len(p.Payload) < 14 {
		return fmt.Errorf("payload too small")
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	m.Seq = uint32(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.TargetSystem = uint8(p.Payload[12])
	m.TargetComponent = uint8(p.Payload[13])
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
func (m *ChangeOperatorControl) MsgID() MessageID {
	return MSG_ID_CHANGE_OPERATOR_CONTROL
}

// MsgName (generated function)
func (m *ChangeOperatorControl) MsgName() string {
	return "ChangeOperatorControl"
}

// Pack (generated function)
func (m *ChangeOperatorControl) Pack(p *Packet) error {
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
func (m *ChangeOperatorControl) Unpack(p *Packet) error {
	if len(p.Payload) < 28 {
		return fmt.Errorf("payload too small")
	}
	m.TargetSystem = uint8(p.Payload[0])
	m.ControlRequest = uint8(p.Payload[1])
	m.Version = uint8(p.Payload[2])
	copy(m.Passkey[:], p.Payload[3:28])
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
func (m *ChangeOperatorControlAck) MsgID() MessageID {
	return MSG_ID_CHANGE_OPERATOR_CONTROL_ACK
}

// MsgName (generated function)
func (m *ChangeOperatorControlAck) MsgName() string {
	return "ChangeOperatorControlAck"
}

// Pack (generated function)
func (m *ChangeOperatorControlAck) Pack(p *Packet) error {
	payload := make([]byte, 3)
	payload[0] = byte(m.GcsSystemID)
	payload[1] = byte(m.ControlRequest)
	payload[2] = byte(m.Ack)

	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ChangeOperatorControlAck) Unpack(p *Packet) error {
	if len(p.Payload) < 3 {
		return fmt.Errorf("payload too small")
	}
	m.GcsSystemID = uint8(p.Payload[0])
	m.ControlRequest = uint8(p.Payload[1])
	m.Ack = uint8(p.Payload[2])
	return nil
}

// AuthKey struct (generated typeinfo)
// Emit an encrypted signature / key identifying this system. PLEASE NOTE: This protocol has been kept simple, so transmitting the key requires an encrypted channel for true safety.
type AuthKey struct {
	Key [32]byte // key
}

// MsgID (generated function)
func (m *AuthKey) MsgID() MessageID {
	return MSG_ID_AUTH_KEY
}

// MsgName (generated function)
func (m *AuthKey) MsgName() string {
	return "AuthKey"
}

// Pack (generated function)
func (m *AuthKey) Pack(p *Packet) error {
	payload := make([]byte, 32)
	copy(payload[0:], m.Key[:])

	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *AuthKey) Unpack(p *Packet) error {
	if len(p.Payload) < 32 {
		return fmt.Errorf("payload too small")
	}
	copy(m.Key[:], p.Payload[0:32])
	return nil
}

// SetMode struct (generated typeinfo)
// THIS INTERFACE IS DEPRECATED. USE COMMAND_LONG with MAV_CMD_DO_SET_MODE INSTEAD. Set the system mode, as defined by enum MAV_MODE. There is no target component id as the mode is by definition for the overall aircraft, not only for one component.
type SetMode struct {
	CustomMode   uint32 // The new autopilot-specific mode. This field can be ignored by an autopilot.
	TargetSystem uint8  // The system setting the mode
	BaseMode     uint8  // The new base mode
}

// MsgID (generated function)
func (m *SetMode) MsgID() MessageID {
	return MSG_ID_SET_MODE
}

// MsgName (generated function)
func (m *SetMode) MsgName() string {
	return "SetMode"
}

// Pack (generated function)
func (m *SetMode) Pack(p *Packet) error {
	payload := make([]byte, 6)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.CustomMode))
	payload[4] = byte(m.TargetSystem)
	payload[5] = byte(m.BaseMode)

	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *SetMode) Unpack(p *Packet) error {
	if len(p.Payload) < 6 {
		return fmt.Errorf("payload too small")
	}
	m.CustomMode = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.TargetSystem = uint8(p.Payload[4])
	m.BaseMode = uint8(p.Payload[5])
	return nil
}

// ParamRequestRead struct (generated typeinfo)
// Request to read the onboard parameter with the param_id string id. Onboard parameters are stored as key[const char*] -> value[float]. This allows to send a parameter to any other component (such as the GCS) without the need of previous knowledge of possible parameter names. Thus the same GCS can store different parameters for different autopilots. See also http://qgroundcontrol.org/parameter_interface for a full documentation of QGroundControl and IMU code.
type ParamRequestRead struct {
	ParamIndex      int16    // Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignored)
	TargetSystem    uint8    // System ID
	TargetComponent uint8    // Component ID
	ParamID         [16]byte // Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
}

// MsgID (generated function)
func (m *ParamRequestRead) MsgID() MessageID {
	return MSG_ID_PARAM_REQUEST_READ
}

// MsgName (generated function)
func (m *ParamRequestRead) MsgName() string {
	return "ParamRequestRead"
}

// Pack (generated function)
func (m *ParamRequestRead) Pack(p *Packet) error {
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
func (m *ParamRequestRead) Unpack(p *Packet) error {
	if len(p.Payload) < 20 {
		return fmt.Errorf("payload too small")
	}
	m.ParamIndex = int16(binary.LittleEndian.Uint16(p.Payload[0:]))
	m.TargetSystem = uint8(p.Payload[2])
	m.TargetComponent = uint8(p.Payload[3])
	copy(m.ParamID[:], p.Payload[4:20])
	return nil
}

// ParamRequestList struct (generated typeinfo)
// Request all parameters of this component. After this request, all parameters are emitted.
type ParamRequestList struct {
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
}

// MsgID (generated function)
func (m *ParamRequestList) MsgID() MessageID {
	return MSG_ID_PARAM_REQUEST_LIST
}

// MsgName (generated function)
func (m *ParamRequestList) MsgName() string {
	return "ParamRequestList"
}

// Pack (generated function)
func (m *ParamRequestList) Pack(p *Packet) error {
	payload := make([]byte, 2)
	payload[0] = byte(m.TargetSystem)
	payload[1] = byte(m.TargetComponent)

	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ParamRequestList) Unpack(p *Packet) error {
	if len(p.Payload) < 2 {
		return fmt.Errorf("payload too small")
	}
	m.TargetSystem = uint8(p.Payload[0])
	m.TargetComponent = uint8(p.Payload[1])
	return nil
}

// ParamValue struct (generated typeinfo)
// Emit the value of a onboard parameter. The inclusion of param_count and param_index in the message allows the recipient to keep track of received parameters and allows him to re-request missing parameters after a loss or timeout.
type ParamValue struct {
	ParamValue float32  // Onboard parameter value
	ParamCount uint16   // Total number of onboard parameters
	ParamIndex uint16   // Index of this onboard parameter
	ParamID    [16]byte // Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
	ParamType  uint8    // Onboard parameter type: see the MAV_PARAM_TYPE enum for supported data types.
}

// MsgID (generated function)
func (m *ParamValue) MsgID() MessageID {
	return MSG_ID_PARAM_VALUE
}

// MsgName (generated function)
func (m *ParamValue) MsgName() string {
	return "ParamValue"
}

// Pack (generated function)
func (m *ParamValue) Pack(p *Packet) error {
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
func (m *ParamValue) Unpack(p *Packet) error {
	if len(p.Payload) < 25 {
		return fmt.Errorf("payload too small")
	}
	m.ParamValue = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.ParamCount = uint16(binary.LittleEndian.Uint16(p.Payload[4:]))
	m.ParamIndex = uint16(binary.LittleEndian.Uint16(p.Payload[6:]))
	copy(m.ParamID[:], p.Payload[8:24])
	m.ParamType = uint8(p.Payload[24])
	return nil
}

// ParamSet struct (generated typeinfo)
// Set a parameter value TEMPORARILY to RAM. It will be reset to default on system reboot. Send the ACTION MAV_ACTION_STORAGE_WRITE to PERMANENTLY write the RAM contents to EEPROM. IMPORTANT: The receiving component should acknowledge the new parameter value by sending a param_value message to all communication partners. This will also ensure that multiple GCS all have an up-to-date list of all parameters. If the sending GCS did not receive a PARAM_VALUE message within its timeout time, it should re-send the PARAM_SET message.
type ParamSet struct {
	ParamValue      float32  // Onboard parameter value
	TargetSystem    uint8    // System ID
	TargetComponent uint8    // Component ID
	ParamID         [16]byte // Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
	ParamType       uint8    // Onboard parameter type: see the MAV_PARAM_TYPE enum for supported data types.
}

// MsgID (generated function)
func (m *ParamSet) MsgID() MessageID {
	return MSG_ID_PARAM_SET
}

// MsgName (generated function)
func (m *ParamSet) MsgName() string {
	return "ParamSet"
}

// Pack (generated function)
func (m *ParamSet) Pack(p *Packet) error {
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
func (m *ParamSet) Unpack(p *Packet) error {
	if len(p.Payload) < 23 {
		return fmt.Errorf("payload too small")
	}
	m.ParamValue = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.TargetSystem = uint8(p.Payload[4])
	m.TargetComponent = uint8(p.Payload[5])
	copy(m.ParamID[:], p.Payload[6:22])
	m.ParamType = uint8(p.Payload[22])
	return nil
}

// GpsRawInt struct (generated typeinfo)
// The global position, as returned by the Global Positioning System (GPS). This is
//                 NOT the global position estimate of the system, but rather a RAW sensor value. See message GLOBAL_POSITION for the global position estimate. Coordinate frame is right-handed, Z-axis up (GPS frame).
type GpsRawInt struct {
	TimeUsec          uint64 // Timestamp (microseconds since UNIX epoch or microseconds since system boot)
	Lat               int32  // Latitude (WGS84), in degrees * 1E7
	Lon               int32  // Longitude (WGS84), in degrees * 1E7
	Alt               int32  // Altitude (AMSL, NOT WGS84), in meters * 1000 (positive for up). Note that virtually all GPS modules provide the AMSL altitude in addition to the WGS84 altitude.
	Eph               uint16 // GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX
	Epv               uint16 // GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX
	Vel               uint16 // GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX
	Cog               uint16 // Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
	FixType           uint8  // See the GPS_FIX_TYPE enum.
	SatellitesVisible uint8  // Number of satellites visible. If unknown, set to 255
}

// MsgID (generated function)
func (m *GpsRawInt) MsgID() MessageID {
	return MSG_ID_GPS_RAW_INT
}

// MsgName (generated function)
func (m *GpsRawInt) MsgName() string {
	return "GpsRawInt"
}

// Pack (generated function)
func (m *GpsRawInt) Pack(p *Packet) error {
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
func (m *GpsRawInt) Unpack(p *Packet) error {
	if len(p.Payload) < 30 {
		return fmt.Errorf("payload too small")
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	m.Lat = int32(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.Lon = int32(binary.LittleEndian.Uint32(p.Payload[12:]))
	m.Alt = int32(binary.LittleEndian.Uint32(p.Payload[16:]))
	m.Eph = uint16(binary.LittleEndian.Uint16(p.Payload[20:]))
	m.Epv = uint16(binary.LittleEndian.Uint16(p.Payload[22:]))
	m.Vel = uint16(binary.LittleEndian.Uint16(p.Payload[24:]))
	m.Cog = uint16(binary.LittleEndian.Uint16(p.Payload[26:]))
	m.FixType = uint8(p.Payload[28])
	m.SatellitesVisible = uint8(p.Payload[29])
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
func (m *GpsStatus) MsgID() MessageID {
	return MSG_ID_GPS_STATUS
}

// MsgName (generated function)
func (m *GpsStatus) MsgName() string {
	return "GpsStatus"
}

// Pack (generated function)
func (m *GpsStatus) Pack(p *Packet) error {
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
func (m *GpsStatus) Unpack(p *Packet) error {
	if len(p.Payload) < 101 {
		return fmt.Errorf("payload too small")
	}
	m.SatellitesVisible = uint8(p.Payload[0])
	copy(m.SatellitePrn[:], p.Payload[1:21])
	copy(m.SatelliteUsed[:], p.Payload[21:41])
	copy(m.SatelliteElevation[:], p.Payload[41:61])
	copy(m.SatelliteAzimuth[:], p.Payload[61:81])
	copy(m.SatelliteSnr[:], p.Payload[81:101])
	return nil
}

// ScaledImu struct (generated typeinfo)
// The RAW IMU readings for the usual 9DOF sensor setup. This message should contain the scaled values to the described units
type ScaledImu struct {
	TimeBootMs uint32 // Timestamp (milliseconds since system boot)
	Xacc       int16  // X acceleration (mg)
	Yacc       int16  // Y acceleration (mg)
	Zacc       int16  // Z acceleration (mg)
	Xgyro      int16  // Angular speed around X axis (millirad /sec)
	Ygyro      int16  // Angular speed around Y axis (millirad /sec)
	Zgyro      int16  // Angular speed around Z axis (millirad /sec)
	Xmag       int16  // X Magnetic field (milli tesla)
	Ymag       int16  // Y Magnetic field (milli tesla)
	Zmag       int16  // Z Magnetic field (milli tesla)
}

// MsgID (generated function)
func (m *ScaledImu) MsgID() MessageID {
	return MSG_ID_SCALED_IMU
}

// MsgName (generated function)
func (m *ScaledImu) MsgName() string {
	return "ScaledImu"
}

// Pack (generated function)
func (m *ScaledImu) Pack(p *Packet) error {
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
func (m *ScaledImu) Unpack(p *Packet) error {
	if len(p.Payload) < 22 {
		return fmt.Errorf("payload too small")
	}
	m.TimeBootMs = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.Xacc = int16(binary.LittleEndian.Uint16(p.Payload[4:]))
	m.Yacc = int16(binary.LittleEndian.Uint16(p.Payload[6:]))
	m.Zacc = int16(binary.LittleEndian.Uint16(p.Payload[8:]))
	m.Xgyro = int16(binary.LittleEndian.Uint16(p.Payload[10:]))
	m.Ygyro = int16(binary.LittleEndian.Uint16(p.Payload[12:]))
	m.Zgyro = int16(binary.LittleEndian.Uint16(p.Payload[14:]))
	m.Xmag = int16(binary.LittleEndian.Uint16(p.Payload[16:]))
	m.Ymag = int16(binary.LittleEndian.Uint16(p.Payload[18:]))
	m.Zmag = int16(binary.LittleEndian.Uint16(p.Payload[20:]))
	return nil
}

// RawImu struct (generated typeinfo)
// The RAW IMU readings for the usual 9DOF sensor setup. This message should always contain the true raw values without any scaling to allow data capture and system debugging.
type RawImu struct {
	TimeUsec uint64 // Timestamp (microseconds since UNIX epoch or microseconds since system boot)
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
func (m *RawImu) MsgID() MessageID {
	return MSG_ID_RAW_IMU
}

// MsgName (generated function)
func (m *RawImu) MsgName() string {
	return "RawImu"
}

// Pack (generated function)
func (m *RawImu) Pack(p *Packet) error {
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
func (m *RawImu) Unpack(p *Packet) error {
	if len(p.Payload) < 26 {
		return fmt.Errorf("payload too small")
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	m.Xacc = int16(binary.LittleEndian.Uint16(p.Payload[8:]))
	m.Yacc = int16(binary.LittleEndian.Uint16(p.Payload[10:]))
	m.Zacc = int16(binary.LittleEndian.Uint16(p.Payload[12:]))
	m.Xgyro = int16(binary.LittleEndian.Uint16(p.Payload[14:]))
	m.Ygyro = int16(binary.LittleEndian.Uint16(p.Payload[16:]))
	m.Zgyro = int16(binary.LittleEndian.Uint16(p.Payload[18:]))
	m.Xmag = int16(binary.LittleEndian.Uint16(p.Payload[20:]))
	m.Ymag = int16(binary.LittleEndian.Uint16(p.Payload[22:]))
	m.Zmag = int16(binary.LittleEndian.Uint16(p.Payload[24:]))
	return nil
}

// RawPressure struct (generated typeinfo)
// The RAW pressure readings for the typical setup of one absolute pressure and one differential pressure sensor. The sensor values should be the raw, UNSCALED ADC values.
type RawPressure struct {
	TimeUsec    uint64 // Timestamp (microseconds since UNIX epoch or microseconds since system boot)
	PressAbs    int16  // Absolute pressure (raw)
	PressDiff1  int16  // Differential pressure 1 (raw, 0 if nonexistant)
	PressDiff2  int16  // Differential pressure 2 (raw, 0 if nonexistant)
	Temperature int16  // Raw Temperature measurement (raw)
}

// MsgID (generated function)
func (m *RawPressure) MsgID() MessageID {
	return MSG_ID_RAW_PRESSURE
}

// MsgName (generated function)
func (m *RawPressure) MsgName() string {
	return "RawPressure"
}

// Pack (generated function)
func (m *RawPressure) Pack(p *Packet) error {
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
func (m *RawPressure) Unpack(p *Packet) error {
	if len(p.Payload) < 16 {
		return fmt.Errorf("payload too small")
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	m.PressAbs = int16(binary.LittleEndian.Uint16(p.Payload[8:]))
	m.PressDiff1 = int16(binary.LittleEndian.Uint16(p.Payload[10:]))
	m.PressDiff2 = int16(binary.LittleEndian.Uint16(p.Payload[12:]))
	m.Temperature = int16(binary.LittleEndian.Uint16(p.Payload[14:]))
	return nil
}

// ScaledPressure struct (generated typeinfo)
// The pressure readings for the typical setup of one absolute and differential pressure sensor. The units are as specified in each field.
type ScaledPressure struct {
	TimeBootMs  uint32  // Timestamp (milliseconds since system boot)
	PressAbs    float32 // Absolute pressure (hectopascal)
	PressDiff   float32 // Differential pressure 1 (hectopascal)
	Temperature int16   // Temperature measurement (0.01 degrees celsius)
}

// MsgID (generated function)
func (m *ScaledPressure) MsgID() MessageID {
	return MSG_ID_SCALED_PRESSURE
}

// MsgName (generated function)
func (m *ScaledPressure) MsgName() string {
	return "ScaledPressure"
}

// Pack (generated function)
func (m *ScaledPressure) Pack(p *Packet) error {
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
func (m *ScaledPressure) Unpack(p *Packet) error {
	if len(p.Payload) < 14 {
		return fmt.Errorf("payload too small")
	}
	m.TimeBootMs = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.PressAbs = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	m.PressDiff = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.Temperature = int16(binary.LittleEndian.Uint16(p.Payload[12:]))
	return nil
}

// Attitude struct (generated typeinfo)
// The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right).
type Attitude struct {
	TimeBootMs uint32  // Timestamp (milliseconds since system boot)
	Roll       float32 // Roll angle (rad, -pi..+pi)
	Pitch      float32 // Pitch angle (rad, -pi..+pi)
	Yaw        float32 // Yaw angle (rad, -pi..+pi)
	Rollspeed  float32 // Roll angular speed (rad/s)
	Pitchspeed float32 // Pitch angular speed (rad/s)
	Yawspeed   float32 // Yaw angular speed (rad/s)
}

// MsgID (generated function)
func (m *Attitude) MsgID() MessageID {
	return MSG_ID_ATTITUDE
}

// MsgName (generated function)
func (m *Attitude) MsgName() string {
	return "Attitude"
}

// Pack (generated function)
func (m *Attitude) Pack(p *Packet) error {
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
func (m *Attitude) Unpack(p *Packet) error {
	if len(p.Payload) < 28 {
		return fmt.Errorf("payload too small")
	}
	m.TimeBootMs = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.Roll = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	m.Pitch = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.Yaw = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	m.Rollspeed = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	m.Pitchspeed = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	m.Yawspeed = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	return nil
}

// AttitudeQuaternion struct (generated typeinfo)
// The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right), expressed as quaternion. Quaternion order is w, x, y, z and a zero rotation would be expressed as (1 0 0 0).
type AttitudeQuaternion struct {
	TimeBootMs uint32  // Timestamp (milliseconds since system boot)
	Q1         float32 // Quaternion component 1, w (1 in null-rotation)
	Q2         float32 // Quaternion component 2, x (0 in null-rotation)
	Q3         float32 // Quaternion component 3, y (0 in null-rotation)
	Q4         float32 // Quaternion component 4, z (0 in null-rotation)
	Rollspeed  float32 // Roll angular speed (rad/s)
	Pitchspeed float32 // Pitch angular speed (rad/s)
	Yawspeed   float32 // Yaw angular speed (rad/s)
}

// MsgID (generated function)
func (m *AttitudeQuaternion) MsgID() MessageID {
	return MSG_ID_ATTITUDE_QUATERNION
}

// MsgName (generated function)
func (m *AttitudeQuaternion) MsgName() string {
	return "AttitudeQuaternion"
}

// Pack (generated function)
func (m *AttitudeQuaternion) Pack(p *Packet) error {
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
func (m *AttitudeQuaternion) Unpack(p *Packet) error {
	if len(p.Payload) < 32 {
		return fmt.Errorf("payload too small")
	}
	m.TimeBootMs = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.Q1 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	m.Q2 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.Q3 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	m.Q4 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	m.Rollspeed = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	m.Pitchspeed = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	m.Yawspeed = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	return nil
}

// LocalPositionNed struct (generated typeinfo)
// The filtered local position (e.g. fused computer vision and accelerometers). Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down convention)
type LocalPositionNed struct {
	TimeBootMs uint32  // Timestamp (milliseconds since system boot)
	X          float32 // X Position
	Y          float32 // Y Position
	Z          float32 // Z Position
	Vx         float32 // X Speed
	Vy         float32 // Y Speed
	Vz         float32 // Z Speed
}

// MsgID (generated function)
func (m *LocalPositionNed) MsgID() MessageID {
	return MSG_ID_LOCAL_POSITION_NED
}

// MsgName (generated function)
func (m *LocalPositionNed) MsgName() string {
	return "LocalPositionNed"
}

// Pack (generated function)
func (m *LocalPositionNed) Pack(p *Packet) error {
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
func (m *LocalPositionNed) Unpack(p *Packet) error {
	if len(p.Payload) < 28 {
		return fmt.Errorf("payload too small")
	}
	m.TimeBootMs = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.X = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	m.Y = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.Z = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	m.Vx = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	m.Vy = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	m.Vz = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	return nil
}

// GlobalPositionInt struct (generated typeinfo)
// The filtered global position (e.g. fused GPS and accelerometers). The position is in GPS-frame (right-handed, Z-up). It
//                is designed as scaled integer message since the resolution of float is not sufficient.
type GlobalPositionInt struct {
	TimeBootMs  uint32 // Timestamp (milliseconds since system boot)
	Lat         int32  // Latitude, expressed as degrees * 1E7
	Lon         int32  // Longitude, expressed as degrees * 1E7
	Alt         int32  // Altitude in meters, expressed as * 1000 (millimeters), AMSL (not WGS84 - note that virtually all GPS modules provide the AMSL as well)
	RelativeAlt int32  // Altitude above ground in meters, expressed as * 1000 (millimeters)
	Vx          int16  // Ground X Speed (Latitude, positive north), expressed as m/s * 100
	Vy          int16  // Ground Y Speed (Longitude, positive east), expressed as m/s * 100
	Vz          int16  // Ground Z Speed (Altitude, positive down), expressed as m/s * 100
	Hdg         uint16 // Vehicle heading (yaw angle) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
}

// MsgID (generated function)
func (m *GlobalPositionInt) MsgID() MessageID {
	return MSG_ID_GLOBAL_POSITION_INT
}

// MsgName (generated function)
func (m *GlobalPositionInt) MsgName() string {
	return "GlobalPositionInt"
}

// Pack (generated function)
func (m *GlobalPositionInt) Pack(p *Packet) error {
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
func (m *GlobalPositionInt) Unpack(p *Packet) error {
	if len(p.Payload) < 28 {
		return fmt.Errorf("payload too small")
	}
	m.TimeBootMs = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.Lat = int32(binary.LittleEndian.Uint32(p.Payload[4:]))
	m.Lon = int32(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.Alt = int32(binary.LittleEndian.Uint32(p.Payload[12:]))
	m.RelativeAlt = int32(binary.LittleEndian.Uint32(p.Payload[16:]))
	m.Vx = int16(binary.LittleEndian.Uint16(p.Payload[20:]))
	m.Vy = int16(binary.LittleEndian.Uint16(p.Payload[22:]))
	m.Vz = int16(binary.LittleEndian.Uint16(p.Payload[24:]))
	m.Hdg = uint16(binary.LittleEndian.Uint16(p.Payload[26:]))
	return nil
}

// RcChannelsScaled struct (generated typeinfo)
// The scaled values of the RC channels received. (-100%) -10000, (0%) 0, (100%) 10000. Channels that are inactive should be set to UINT16_MAX.
type RcChannelsScaled struct {
	TimeBootMs  uint32 // Timestamp (milliseconds since system boot)
	Chan1Scaled int16  // RC channel 1 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
	Chan2Scaled int16  // RC channel 2 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
	Chan3Scaled int16  // RC channel 3 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
	Chan4Scaled int16  // RC channel 4 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
	Chan5Scaled int16  // RC channel 5 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
	Chan6Scaled int16  // RC channel 6 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
	Chan7Scaled int16  // RC channel 7 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
	Chan8Scaled int16  // RC channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
	Port        uint8  // Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows for more than 8 servos.
	Rssi        uint8  // Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
}

// MsgID (generated function)
func (m *RcChannelsScaled) MsgID() MessageID {
	return MSG_ID_RC_CHANNELS_SCALED
}

// MsgName (generated function)
func (m *RcChannelsScaled) MsgName() string {
	return "RcChannelsScaled"
}

// Pack (generated function)
func (m *RcChannelsScaled) Pack(p *Packet) error {
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
func (m *RcChannelsScaled) Unpack(p *Packet) error {
	if len(p.Payload) < 22 {
		return fmt.Errorf("payload too small")
	}
	m.TimeBootMs = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.Chan1Scaled = int16(binary.LittleEndian.Uint16(p.Payload[4:]))
	m.Chan2Scaled = int16(binary.LittleEndian.Uint16(p.Payload[6:]))
	m.Chan3Scaled = int16(binary.LittleEndian.Uint16(p.Payload[8:]))
	m.Chan4Scaled = int16(binary.LittleEndian.Uint16(p.Payload[10:]))
	m.Chan5Scaled = int16(binary.LittleEndian.Uint16(p.Payload[12:]))
	m.Chan6Scaled = int16(binary.LittleEndian.Uint16(p.Payload[14:]))
	m.Chan7Scaled = int16(binary.LittleEndian.Uint16(p.Payload[16:]))
	m.Chan8Scaled = int16(binary.LittleEndian.Uint16(p.Payload[18:]))
	m.Port = uint8(p.Payload[20])
	m.Rssi = uint8(p.Payload[21])
	return nil
}

// RcChannelsRaw struct (generated typeinfo)
// The RAW values of the RC channels received. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual receivers/transmitters might violate this specification.
type RcChannelsRaw struct {
	TimeBootMs uint32 // Timestamp (milliseconds since system boot)
	Chan1Raw   uint16 // RC channel 1 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan2Raw   uint16 // RC channel 2 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan3Raw   uint16 // RC channel 3 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan4Raw   uint16 // RC channel 4 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan5Raw   uint16 // RC channel 5 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan6Raw   uint16 // RC channel 6 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan7Raw   uint16 // RC channel 7 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan8Raw   uint16 // RC channel 8 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Port       uint8  // Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows for more than 8 servos.
	Rssi       uint8  // Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
}

// MsgID (generated function)
func (m *RcChannelsRaw) MsgID() MessageID {
	return MSG_ID_RC_CHANNELS_RAW
}

// MsgName (generated function)
func (m *RcChannelsRaw) MsgName() string {
	return "RcChannelsRaw"
}

// Pack (generated function)
func (m *RcChannelsRaw) Pack(p *Packet) error {
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
func (m *RcChannelsRaw) Unpack(p *Packet) error {
	if len(p.Payload) < 22 {
		return fmt.Errorf("payload too small")
	}
	m.TimeBootMs = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.Chan1Raw = uint16(binary.LittleEndian.Uint16(p.Payload[4:]))
	m.Chan2Raw = uint16(binary.LittleEndian.Uint16(p.Payload[6:]))
	m.Chan3Raw = uint16(binary.LittleEndian.Uint16(p.Payload[8:]))
	m.Chan4Raw = uint16(binary.LittleEndian.Uint16(p.Payload[10:]))
	m.Chan5Raw = uint16(binary.LittleEndian.Uint16(p.Payload[12:]))
	m.Chan6Raw = uint16(binary.LittleEndian.Uint16(p.Payload[14:]))
	m.Chan7Raw = uint16(binary.LittleEndian.Uint16(p.Payload[16:]))
	m.Chan8Raw = uint16(binary.LittleEndian.Uint16(p.Payload[18:]))
	m.Port = uint8(p.Payload[20])
	m.Rssi = uint8(p.Payload[21])
	return nil
}

// ServoOutputRaw struct (generated typeinfo)
// The RAW values of the servo outputs (for RC input from the remote, use the RC_CHANNELS messages). The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%.
type ServoOutputRaw struct {
	TimeUsec  uint32 // Timestamp (microseconds since system boot)
	Servo1Raw uint16 // Servo output 1 value, in microseconds
	Servo2Raw uint16 // Servo output 2 value, in microseconds
	Servo3Raw uint16 // Servo output 3 value, in microseconds
	Servo4Raw uint16 // Servo output 4 value, in microseconds
	Servo5Raw uint16 // Servo output 5 value, in microseconds
	Servo6Raw uint16 // Servo output 6 value, in microseconds
	Servo7Raw uint16 // Servo output 7 value, in microseconds
	Servo8Raw uint16 // Servo output 8 value, in microseconds
	Port      uint8  // Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows to encode more than 8 servos.
}

// MsgID (generated function)
func (m *ServoOutputRaw) MsgID() MessageID {
	return MSG_ID_SERVO_OUTPUT_RAW
}

// MsgName (generated function)
func (m *ServoOutputRaw) MsgName() string {
	return "ServoOutputRaw"
}

// Pack (generated function)
func (m *ServoOutputRaw) Pack(p *Packet) error {
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
func (m *ServoOutputRaw) Unpack(p *Packet) error {
	if len(p.Payload) < 21 {
		return fmt.Errorf("payload too small")
	}
	m.TimeUsec = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.Servo1Raw = uint16(binary.LittleEndian.Uint16(p.Payload[4:]))
	m.Servo2Raw = uint16(binary.LittleEndian.Uint16(p.Payload[6:]))
	m.Servo3Raw = uint16(binary.LittleEndian.Uint16(p.Payload[8:]))
	m.Servo4Raw = uint16(binary.LittleEndian.Uint16(p.Payload[10:]))
	m.Servo5Raw = uint16(binary.LittleEndian.Uint16(p.Payload[12:]))
	m.Servo6Raw = uint16(binary.LittleEndian.Uint16(p.Payload[14:]))
	m.Servo7Raw = uint16(binary.LittleEndian.Uint16(p.Payload[16:]))
	m.Servo8Raw = uint16(binary.LittleEndian.Uint16(p.Payload[18:]))
	m.Port = uint8(p.Payload[20])
	return nil
}

// MissionRequestPartialList struct (generated typeinfo)
// Request a partial list of mission items from the system/component. http://qgroundcontrol.org/mavlink/waypoint_protocol. If start and end index are the same, just send one waypoint.
type MissionRequestPartialList struct {
	StartIndex      int16 // Start index, 0 by default
	EndIndex        int16 // End index, -1 by default (-1: send list to end). Else a valid index of the list
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
}

// MsgID (generated function)
func (m *MissionRequestPartialList) MsgID() MessageID {
	return MSG_ID_MISSION_REQUEST_PARTIAL_LIST
}

// MsgName (generated function)
func (m *MissionRequestPartialList) MsgName() string {
	return "MissionRequestPartialList"
}

// Pack (generated function)
func (m *MissionRequestPartialList) Pack(p *Packet) error {
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
func (m *MissionRequestPartialList) Unpack(p *Packet) error {
	if len(p.Payload) < 6 {
		return fmt.Errorf("payload too small")
	}
	m.StartIndex = int16(binary.LittleEndian.Uint16(p.Payload[0:]))
	m.EndIndex = int16(binary.LittleEndian.Uint16(p.Payload[2:]))
	m.TargetSystem = uint8(p.Payload[4])
	m.TargetComponent = uint8(p.Payload[5])
	return nil
}

// MissionWritePartialList struct (generated typeinfo)
// This message is sent to the MAV to write a partial list. If start index == end index, only one item will be transmitted / updated. If the start index is NOT 0 and above the current list size, this request should be REJECTED!
type MissionWritePartialList struct {
	StartIndex      int16 // Start index, 0 by default and smaller / equal to the largest index of the current onboard list.
	EndIndex        int16 // End index, equal or greater than start index.
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
}

// MsgID (generated function)
func (m *MissionWritePartialList) MsgID() MessageID {
	return MSG_ID_MISSION_WRITE_PARTIAL_LIST
}

// MsgName (generated function)
func (m *MissionWritePartialList) MsgName() string {
	return "MissionWritePartialList"
}

// Pack (generated function)
func (m *MissionWritePartialList) Pack(p *Packet) error {
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
func (m *MissionWritePartialList) Unpack(p *Packet) error {
	if len(p.Payload) < 6 {
		return fmt.Errorf("payload too small")
	}
	m.StartIndex = int16(binary.LittleEndian.Uint16(p.Payload[0:]))
	m.EndIndex = int16(binary.LittleEndian.Uint16(p.Payload[2:]))
	m.TargetSystem = uint8(p.Payload[4])
	m.TargetComponent = uint8(p.Payload[5])
	return nil
}

// MissionItem struct (generated typeinfo)
// Message encoding a mission item. This message is emitted to announce
//                 the presence of a mission item and to set a mission item on the system. The mission item can be either in x, y, z meters (type: LOCAL) or x:lat, y:lon, z:altitude. Local frame is Z-down, right handed (NED), global frame is Z-up, right handed (ENU). See also http://qgroundcontrol.org/mavlink/waypoint_protocol.
type MissionItem struct {
	Param1          float32 // PARAM1, see MAV_CMD enum
	Param2          float32 // PARAM2, see MAV_CMD enum
	Param3          float32 // PARAM3, see MAV_CMD enum
	Param4          float32 // PARAM4, see MAV_CMD enum
	X               float32 // PARAM5 / local: x position, global: latitude
	Y               float32 // PARAM6 / y position: global: longitude
	Z               float32 // PARAM7 / z position: global: altitude (relative or absolute, depending on frame.
	Seq             uint16  // Sequence
	Command         uint16  // The scheduled action for the MISSION. see MAV_CMD in common.xml MAVLink specs
	TargetSystem    uint8   // System ID
	TargetComponent uint8   // Component ID
	Frame           uint8   // The coordinate system of the MISSION. see MAV_FRAME in mavlink_types.h
	Current         uint8   // false:0, true:1
	Autocontinue    uint8   // autocontinue to next wp
}

// MsgID (generated function)
func (m *MissionItem) MsgID() MessageID {
	return MSG_ID_MISSION_ITEM
}

// MsgName (generated function)
func (m *MissionItem) MsgName() string {
	return "MissionItem"
}

// Pack (generated function)
func (m *MissionItem) Pack(p *Packet) error {
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
func (m *MissionItem) Unpack(p *Packet) error {
	if len(p.Payload) < 37 {
		return fmt.Errorf("payload too small")
	}
	m.Param1 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.Param2 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	m.Param3 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.Param4 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	m.X = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	m.Y = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	m.Z = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	m.Seq = uint16(binary.LittleEndian.Uint16(p.Payload[28:]))
	m.Command = uint16(binary.LittleEndian.Uint16(p.Payload[30:]))
	m.TargetSystem = uint8(p.Payload[32])
	m.TargetComponent = uint8(p.Payload[33])
	m.Frame = uint8(p.Payload[34])
	m.Current = uint8(p.Payload[35])
	m.Autocontinue = uint8(p.Payload[36])
	return nil
}

// MissionRequest struct (generated typeinfo)
// Request the information of the mission item with the sequence number seq. The response of the system to this message should be a MISSION_ITEM message. http://qgroundcontrol.org/mavlink/waypoint_protocol
type MissionRequest struct {
	Seq             uint16 // Sequence
	TargetSystem    uint8  // System ID
	TargetComponent uint8  // Component ID
}

// MsgID (generated function)
func (m *MissionRequest) MsgID() MessageID {
	return MSG_ID_MISSION_REQUEST
}

// MsgName (generated function)
func (m *MissionRequest) MsgName() string {
	return "MissionRequest"
}

// Pack (generated function)
func (m *MissionRequest) Pack(p *Packet) error {
	payload := make([]byte, 4)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.Seq))
	payload[2] = byte(m.TargetSystem)
	payload[3] = byte(m.TargetComponent)

	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *MissionRequest) Unpack(p *Packet) error {
	if len(p.Payload) < 4 {
		return fmt.Errorf("payload too small")
	}
	m.Seq = uint16(binary.LittleEndian.Uint16(p.Payload[0:]))
	m.TargetSystem = uint8(p.Payload[2])
	m.TargetComponent = uint8(p.Payload[3])
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
func (m *MissionSetCurrent) MsgID() MessageID {
	return MSG_ID_MISSION_SET_CURRENT
}

// MsgName (generated function)
func (m *MissionSetCurrent) MsgName() string {
	return "MissionSetCurrent"
}

// Pack (generated function)
func (m *MissionSetCurrent) Pack(p *Packet) error {
	payload := make([]byte, 4)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.Seq))
	payload[2] = byte(m.TargetSystem)
	payload[3] = byte(m.TargetComponent)

	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *MissionSetCurrent) Unpack(p *Packet) error {
	if len(p.Payload) < 4 {
		return fmt.Errorf("payload too small")
	}
	m.Seq = uint16(binary.LittleEndian.Uint16(p.Payload[0:]))
	m.TargetSystem = uint8(p.Payload[2])
	m.TargetComponent = uint8(p.Payload[3])
	return nil
}

// MissionCurrent struct (generated typeinfo)
// Message that announces the sequence number of the current active mission item. The MAV will fly towards this mission item.
type MissionCurrent struct {
	Seq uint16 // Sequence
}

// MsgID (generated function)
func (m *MissionCurrent) MsgID() MessageID {
	return MSG_ID_MISSION_CURRENT
}

// MsgName (generated function)
func (m *MissionCurrent) MsgName() string {
	return "MissionCurrent"
}

// Pack (generated function)
func (m *MissionCurrent) Pack(p *Packet) error {
	payload := make([]byte, 2)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.Seq))

	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *MissionCurrent) Unpack(p *Packet) error {
	if len(p.Payload) < 2 {
		return fmt.Errorf("payload too small")
	}
	m.Seq = uint16(binary.LittleEndian.Uint16(p.Payload[0:]))
	return nil
}

// MissionRequestList struct (generated typeinfo)
// Request the overall list of mission items from the system/component.
type MissionRequestList struct {
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
}

// MsgID (generated function)
func (m *MissionRequestList) MsgID() MessageID {
	return MSG_ID_MISSION_REQUEST_LIST
}

// MsgName (generated function)
func (m *MissionRequestList) MsgName() string {
	return "MissionRequestList"
}

// Pack (generated function)
func (m *MissionRequestList) Pack(p *Packet) error {
	payload := make([]byte, 2)
	payload[0] = byte(m.TargetSystem)
	payload[1] = byte(m.TargetComponent)

	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *MissionRequestList) Unpack(p *Packet) error {
	if len(p.Payload) < 2 {
		return fmt.Errorf("payload too small")
	}
	m.TargetSystem = uint8(p.Payload[0])
	m.TargetComponent = uint8(p.Payload[1])
	return nil
}

// MissionCount struct (generated typeinfo)
// This message is emitted as response to MISSION_REQUEST_LIST by the MAV and to initiate a write transaction. The GCS can then request the individual mission item based on the knowledge of the total number of MISSIONs.
type MissionCount struct {
	Count           uint16 // Number of mission items in the sequence
	TargetSystem    uint8  // System ID
	TargetComponent uint8  // Component ID
}

// MsgID (generated function)
func (m *MissionCount) MsgID() MessageID {
	return MSG_ID_MISSION_COUNT
}

// MsgName (generated function)
func (m *MissionCount) MsgName() string {
	return "MissionCount"
}

// Pack (generated function)
func (m *MissionCount) Pack(p *Packet) error {
	payload := make([]byte, 4)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.Count))
	payload[2] = byte(m.TargetSystem)
	payload[3] = byte(m.TargetComponent)

	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *MissionCount) Unpack(p *Packet) error {
	if len(p.Payload) < 4 {
		return fmt.Errorf("payload too small")
	}
	m.Count = uint16(binary.LittleEndian.Uint16(p.Payload[0:]))
	m.TargetSystem = uint8(p.Payload[2])
	m.TargetComponent = uint8(p.Payload[3])
	return nil
}

// MissionClearAll struct (generated typeinfo)
// Delete all mission items at once.
type MissionClearAll struct {
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
}

// MsgID (generated function)
func (m *MissionClearAll) MsgID() MessageID {
	return MSG_ID_MISSION_CLEAR_ALL
}

// MsgName (generated function)
func (m *MissionClearAll) MsgName() string {
	return "MissionClearAll"
}

// Pack (generated function)
func (m *MissionClearAll) Pack(p *Packet) error {
	payload := make([]byte, 2)
	payload[0] = byte(m.TargetSystem)
	payload[1] = byte(m.TargetComponent)

	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *MissionClearAll) Unpack(p *Packet) error {
	if len(p.Payload) < 2 {
		return fmt.Errorf("payload too small")
	}
	m.TargetSystem = uint8(p.Payload[0])
	m.TargetComponent = uint8(p.Payload[1])
	return nil
}

// MissionItemReached struct (generated typeinfo)
// A certain mission item has been reached. The system will either hold this position (or circle on the orbit) or (if the autocontinue on the WP was set) continue to the next MISSION.
type MissionItemReached struct {
	Seq uint16 // Sequence
}

// MsgID (generated function)
func (m *MissionItemReached) MsgID() MessageID {
	return MSG_ID_MISSION_ITEM_REACHED
}

// MsgName (generated function)
func (m *MissionItemReached) MsgName() string {
	return "MissionItemReached"
}

// Pack (generated function)
func (m *MissionItemReached) Pack(p *Packet) error {
	payload := make([]byte, 2)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.Seq))

	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *MissionItemReached) Unpack(p *Packet) error {
	if len(p.Payload) < 2 {
		return fmt.Errorf("payload too small")
	}
	m.Seq = uint16(binary.LittleEndian.Uint16(p.Payload[0:]))
	return nil
}

// MissionAck struct (generated typeinfo)
// Ack message during MISSION handling. The type field states if this message is a positive ack (type=0) or if an error happened (type=non-zero).
type MissionAck struct {
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
	Type            uint8 // See MAV_MISSION_RESULT enum
}

// MsgID (generated function)
func (m *MissionAck) MsgID() MessageID {
	return MSG_ID_MISSION_ACK
}

// MsgName (generated function)
func (m *MissionAck) MsgName() string {
	return "MissionAck"
}

// Pack (generated function)
func (m *MissionAck) Pack(p *Packet) error {
	payload := make([]byte, 3)
	payload[0] = byte(m.TargetSystem)
	payload[1] = byte(m.TargetComponent)
	payload[2] = byte(m.Type)

	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *MissionAck) Unpack(p *Packet) error {
	if len(p.Payload) < 3 {
		return fmt.Errorf("payload too small")
	}
	m.TargetSystem = uint8(p.Payload[0])
	m.TargetComponent = uint8(p.Payload[1])
	m.Type = uint8(p.Payload[2])
	return nil
}

// SetGpsGlobalOrigin struct (generated typeinfo)
// As local waypoints exist, the global MISSION reference allows to transform between the local coordinate frame and the global (GPS) coordinate frame. This can be necessary when e.g. in- and outdoor settings are connected and the MAV should move from in- to outdoor.
type SetGpsGlobalOrigin struct {
	Latitude     int32 // Latitude (WGS84), in degrees * 1E7
	Longitude    int32 // Longitude (WGS84, in degrees * 1E7
	Altitude     int32 // Altitude (AMSL), in meters * 1000 (positive for up)
	TargetSystem uint8 // System ID
}

// MsgID (generated function)
func (m *SetGpsGlobalOrigin) MsgID() MessageID {
	return MSG_ID_SET_GPS_GLOBAL_ORIGIN
}

// MsgName (generated function)
func (m *SetGpsGlobalOrigin) MsgName() string {
	return "SetGpsGlobalOrigin"
}

// Pack (generated function)
func (m *SetGpsGlobalOrigin) Pack(p *Packet) error {
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
func (m *SetGpsGlobalOrigin) Unpack(p *Packet) error {
	if len(p.Payload) < 13 {
		return fmt.Errorf("payload too small")
	}
	m.Latitude = int32(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.Longitude = int32(binary.LittleEndian.Uint32(p.Payload[4:]))
	m.Altitude = int32(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.TargetSystem = uint8(p.Payload[12])
	return nil
}

// GpsGlobalOrigin struct (generated typeinfo)
// Once the MAV sets a new GPS-Local correspondence, this message announces the origin (0,0,0) position
type GpsGlobalOrigin struct {
	Latitude  int32 // Latitude (WGS84), in degrees * 1E7
	Longitude int32 // Longitude (WGS84), in degrees * 1E7
	Altitude  int32 // Altitude (AMSL), in meters * 1000 (positive for up)
}

// MsgID (generated function)
func (m *GpsGlobalOrigin) MsgID() MessageID {
	return MSG_ID_GPS_GLOBAL_ORIGIN
}

// MsgName (generated function)
func (m *GpsGlobalOrigin) MsgName() string {
	return "GpsGlobalOrigin"
}

// Pack (generated function)
func (m *GpsGlobalOrigin) Pack(p *Packet) error {
	payload := make([]byte, 12)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.Latitude))
	binary.LittleEndian.PutUint32(payload[4:], uint32(m.Longitude))
	binary.LittleEndian.PutUint32(payload[8:], uint32(m.Altitude))

	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *GpsGlobalOrigin) Unpack(p *Packet) error {
	if len(p.Payload) < 12 {
		return fmt.Errorf("payload too small")
	}
	m.Latitude = int32(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.Longitude = int32(binary.LittleEndian.Uint32(p.Payload[4:]))
	m.Altitude = int32(binary.LittleEndian.Uint32(p.Payload[8:]))
	return nil
}

// ParamMapRc struct (generated typeinfo)
// Bind a RC channel to a parameter. The parameter should change accoding to the RC channel value.
type ParamMapRc struct {
	ParamValue0             float32  // Initial parameter value
	Scale                   float32  // Scale, maps the RC range [-1, 1] to a parameter value
	ParamValueMin           float32  // Minimum param value. The protocol does not define if this overwrites an onboard minimum value. (Depends on implementation)
	ParamValueMax           float32  // Maximum param value. The protocol does not define if this overwrites an onboard maximum value. (Depends on implementation)
	ParamIndex              int16    // Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignored), send -2 to disable any existing map for this rc_channel_index.
	TargetSystem            uint8    // System ID
	TargetComponent         uint8    // Component ID
	ParamID                 [16]byte // Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
	ParameterRcChannelIndex uint8    // Index of parameter RC channel. Not equal to the RC channel id. Typically correpsonds to a potentiometer-knob on the RC.
}

// MsgID (generated function)
func (m *ParamMapRc) MsgID() MessageID {
	return MSG_ID_PARAM_MAP_RC
}

// MsgName (generated function)
func (m *ParamMapRc) MsgName() string {
	return "ParamMapRc"
}

// Pack (generated function)
func (m *ParamMapRc) Pack(p *Packet) error {
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
func (m *ParamMapRc) Unpack(p *Packet) error {
	if len(p.Payload) < 37 {
		return fmt.Errorf("payload too small")
	}
	m.ParamValue0 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.Scale = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	m.ParamValueMin = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.ParamValueMax = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	m.ParamIndex = int16(binary.LittleEndian.Uint16(p.Payload[16:]))
	m.TargetSystem = uint8(p.Payload[18])
	m.TargetComponent = uint8(p.Payload[19])
	copy(m.ParamID[:], p.Payload[20:36])
	m.ParameterRcChannelIndex = uint8(p.Payload[36])
	return nil
}

// MissionRequestInt struct (generated typeinfo)
// Request the information of the mission item with the sequence number seq. The response of the system to this message should be a MISSION_ITEM_INT message. http://qgroundcontrol.org/mavlink/waypoint_protocol
type MissionRequestInt struct {
	Seq             uint16 // Sequence
	TargetSystem    uint8  // System ID
	TargetComponent uint8  // Component ID
}

// MsgID (generated function)
func (m *MissionRequestInt) MsgID() MessageID {
	return MSG_ID_MISSION_REQUEST_INT
}

// MsgName (generated function)
func (m *MissionRequestInt) MsgName() string {
	return "MissionRequestInt"
}

// Pack (generated function)
func (m *MissionRequestInt) Pack(p *Packet) error {
	payload := make([]byte, 4)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.Seq))
	payload[2] = byte(m.TargetSystem)
	payload[3] = byte(m.TargetComponent)

	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *MissionRequestInt) Unpack(p *Packet) error {
	if len(p.Payload) < 4 {
		return fmt.Errorf("payload too small")
	}
	m.Seq = uint16(binary.LittleEndian.Uint16(p.Payload[0:]))
	m.TargetSystem = uint8(p.Payload[2])
	m.TargetComponent = uint8(p.Payload[3])
	return nil
}

// SafetySetAllowedArea struct (generated typeinfo)
// Set a safety zone (volume), which is defined by two corners of a cube. This message can be used to tell the MAV which setpoints/MISSIONs to accept and which to reject. Safety areas are often enforced by national or competition regulations.
type SafetySetAllowedArea struct {
	P1x             float32 // x position 1 / Latitude 1
	P1y             float32 // y position 1 / Longitude 1
	P1z             float32 // z position 1 / Altitude 1
	P2x             float32 // x position 2 / Latitude 2
	P2y             float32 // y position 2 / Longitude 2
	P2z             float32 // z position 2 / Altitude 2
	TargetSystem    uint8   // System ID
	TargetComponent uint8   // Component ID
	Frame           uint8   // Coordinate frame, as defined by MAV_FRAME enum in mavlink_types.h. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down.
}

// MsgID (generated function)
func (m *SafetySetAllowedArea) MsgID() MessageID {
	return MSG_ID_SAFETY_SET_ALLOWED_AREA
}

// MsgName (generated function)
func (m *SafetySetAllowedArea) MsgName() string {
	return "SafetySetAllowedArea"
}

// Pack (generated function)
func (m *SafetySetAllowedArea) Pack(p *Packet) error {
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
func (m *SafetySetAllowedArea) Unpack(p *Packet) error {
	if len(p.Payload) < 27 {
		return fmt.Errorf("payload too small")
	}
	m.P1x = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.P1y = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	m.P1z = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.P2x = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	m.P2y = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	m.P2z = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	m.TargetSystem = uint8(p.Payload[24])
	m.TargetComponent = uint8(p.Payload[25])
	m.Frame = uint8(p.Payload[26])
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
	Frame uint8   // Coordinate frame, as defined by MAV_FRAME enum in mavlink_types.h. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down.
}

// MsgID (generated function)
func (m *SafetyAllowedArea) MsgID() MessageID {
	return MSG_ID_SAFETY_ALLOWED_AREA
}

// MsgName (generated function)
func (m *SafetyAllowedArea) MsgName() string {
	return "SafetyAllowedArea"
}

// Pack (generated function)
func (m *SafetyAllowedArea) Pack(p *Packet) error {
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
func (m *SafetyAllowedArea) Unpack(p *Packet) error {
	if len(p.Payload) < 25 {
		return fmt.Errorf("payload too small")
	}
	m.P1x = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.P1y = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	m.P1z = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.P2x = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	m.P2y = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	m.P2z = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	m.Frame = uint8(p.Payload[24])
	return nil
}

// AttitudeQuaternionCov struct (generated typeinfo)
// The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right), expressed as quaternion. Quaternion order is w, x, y, z and a zero rotation would be expressed as (1 0 0 0).
type AttitudeQuaternionCov struct {
	TimeUsec   uint64     // Timestamp (microseconds since system boot or since UNIX epoch)
	Q          [4]float32 // Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation)
	Rollspeed  float32    // Roll angular speed (rad/s)
	Pitchspeed float32    // Pitch angular speed (rad/s)
	Yawspeed   float32    // Yaw angular speed (rad/s)
	Covariance [9]float32 // Attitude covariance
}

// MsgID (generated function)
func (m *AttitudeQuaternionCov) MsgID() MessageID {
	return MSG_ID_ATTITUDE_QUATERNION_COV
}

// MsgName (generated function)
func (m *AttitudeQuaternionCov) MsgName() string {
	return "AttitudeQuaternionCov"
}

// Pack (generated function)
func (m *AttitudeQuaternionCov) Pack(p *Packet) error {
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
func (m *AttitudeQuaternionCov) Unpack(p *Packet) error {
	if len(p.Payload) < 72 {
		return fmt.Errorf("payload too small")
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	for i := 0; i < len(m.Q); i++ {
		m.Q[i] = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8+i*4:]))
	}
	m.Rollspeed = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	m.Pitchspeed = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	m.Yawspeed = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[32:]))
	for i := 0; i < len(m.Covariance); i++ {
		m.Covariance[i] = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[36+i*4:]))
	}
	return nil
}

// NavControllerOutput struct (generated typeinfo)
// The state of the fixed wing navigation and position controller.
type NavControllerOutput struct {
	NavRoll       float32 // Current desired roll in degrees
	NavPitch      float32 // Current desired pitch in degrees
	AltError      float32 // Current altitude error in meters
	AspdError     float32 // Current airspeed error in meters/second
	XtrackError   float32 // Current crosstrack error on x-y plane in meters
	NavBearing    int16   // Current desired heading in degrees
	TargetBearing int16   // Bearing to current MISSION/target in degrees
	WpDist        uint16  // Distance to active MISSION in meters
}

// MsgID (generated function)
func (m *NavControllerOutput) MsgID() MessageID {
	return MSG_ID_NAV_CONTROLLER_OUTPUT
}

// MsgName (generated function)
func (m *NavControllerOutput) MsgName() string {
	return "NavControllerOutput"
}

// Pack (generated function)
func (m *NavControllerOutput) Pack(p *Packet) error {
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
func (m *NavControllerOutput) Unpack(p *Packet) error {
	if len(p.Payload) < 26 {
		return fmt.Errorf("payload too small")
	}
	m.NavRoll = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.NavPitch = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	m.AltError = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.AspdError = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	m.XtrackError = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	m.NavBearing = int16(binary.LittleEndian.Uint16(p.Payload[20:]))
	m.TargetBearing = int16(binary.LittleEndian.Uint16(p.Payload[22:]))
	m.WpDist = uint16(binary.LittleEndian.Uint16(p.Payload[24:]))
	return nil
}

// GlobalPositionIntCov struct (generated typeinfo)
// The filtered global position (e.g. fused GPS and accelerometers). The position is in GPS-frame (right-handed, Z-up). It  is designed as scaled integer message since the resolution of float is not sufficient. NOTE: This message is intended for onboard networks / companion computers and higher-bandwidth links and optimized for accuracy and completeness. Please use the GLOBAL_POSITION_INT message for a minimal subset.
type GlobalPositionIntCov struct {
	TimeUsec      uint64      // Timestamp (microseconds since system boot or since UNIX epoch)
	Lat           int32       // Latitude, expressed as degrees * 1E7
	Lon           int32       // Longitude, expressed as degrees * 1E7
	Alt           int32       // Altitude in meters, expressed as * 1000 (millimeters), above MSL
	RelativeAlt   int32       // Altitude above ground in meters, expressed as * 1000 (millimeters)
	Vx            float32     // Ground X Speed (Latitude), expressed as m/s
	Vy            float32     // Ground Y Speed (Longitude), expressed as m/s
	Vz            float32     // Ground Z Speed (Altitude), expressed as m/s
	Covariance    [36]float32 // Covariance matrix (first six entries are the first ROW, next six entries are the second row, etc.)
	EstimatorType uint8       // Class id of the estimator this estimate originated from.
}

// MsgID (generated function)
func (m *GlobalPositionIntCov) MsgID() MessageID {
	return MSG_ID_GLOBAL_POSITION_INT_COV
}

// MsgName (generated function)
func (m *GlobalPositionIntCov) MsgName() string {
	return "GlobalPositionIntCov"
}

// Pack (generated function)
func (m *GlobalPositionIntCov) Pack(p *Packet) error {
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
func (m *GlobalPositionIntCov) Unpack(p *Packet) error {
	if len(p.Payload) < 181 {
		return fmt.Errorf("payload too small")
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	m.Lat = int32(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.Lon = int32(binary.LittleEndian.Uint32(p.Payload[12:]))
	m.Alt = int32(binary.LittleEndian.Uint32(p.Payload[16:]))
	m.RelativeAlt = int32(binary.LittleEndian.Uint32(p.Payload[20:]))
	m.Vx = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	m.Vy = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	m.Vz = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[32:]))
	for i := 0; i < len(m.Covariance); i++ {
		m.Covariance[i] = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[36+i*4:]))
	}
	m.EstimatorType = uint8(p.Payload[180])
	return nil
}

// LocalPositionNedCov struct (generated typeinfo)
// The filtered local position (e.g. fused computer vision and accelerometers). Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down convention)
type LocalPositionNedCov struct {
	TimeUsec      uint64      // Timestamp (microseconds since system boot or since UNIX epoch)
	X             float32     // X Position
	Y             float32     // Y Position
	Z             float32     // Z Position
	Vx            float32     // X Speed (m/s)
	Vy            float32     // Y Speed (m/s)
	Vz            float32     // Z Speed (m/s)
	Ax            float32     // X Acceleration (m/s^2)
	Ay            float32     // Y Acceleration (m/s^2)
	Az            float32     // Z Acceleration (m/s^2)
	Covariance    [45]float32 // Covariance matrix upper right triangular (first nine entries are the first ROW, next eight entries are the second row, etc.)
	EstimatorType uint8       // Class id of the estimator this estimate originated from.
}

// MsgID (generated function)
func (m *LocalPositionNedCov) MsgID() MessageID {
	return MSG_ID_LOCAL_POSITION_NED_COV
}

// MsgName (generated function)
func (m *LocalPositionNedCov) MsgName() string {
	return "LocalPositionNedCov"
}

// Pack (generated function)
func (m *LocalPositionNedCov) Pack(p *Packet) error {
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
func (m *LocalPositionNedCov) Unpack(p *Packet) error {
	if len(p.Payload) < 225 {
		return fmt.Errorf("payload too small")
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	m.X = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.Y = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	m.Z = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	m.Vx = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	m.Vy = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	m.Vz = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	m.Ax = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[32:]))
	m.Ay = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[36:]))
	m.Az = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[40:]))
	for i := 0; i < len(m.Covariance); i++ {
		m.Covariance[i] = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[44+i*4:]))
	}
	m.EstimatorType = uint8(p.Payload[224])
	return nil
}

// RcChannels struct (generated typeinfo)
// The PPM values of the RC channels received. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual receivers/transmitters might violate this specification.
type RcChannels struct {
	TimeBootMs uint32 // Timestamp (milliseconds since system boot)
	Chan1Raw   uint16 // RC channel 1 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan2Raw   uint16 // RC channel 2 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan3Raw   uint16 // RC channel 3 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan4Raw   uint16 // RC channel 4 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan5Raw   uint16 // RC channel 5 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan6Raw   uint16 // RC channel 6 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan7Raw   uint16 // RC channel 7 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan8Raw   uint16 // RC channel 8 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan9Raw   uint16 // RC channel 9 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan10Raw  uint16 // RC channel 10 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan11Raw  uint16 // RC channel 11 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan12Raw  uint16 // RC channel 12 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan13Raw  uint16 // RC channel 13 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan14Raw  uint16 // RC channel 14 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan15Raw  uint16 // RC channel 15 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan16Raw  uint16 // RC channel 16 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan17Raw  uint16 // RC channel 17 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan18Raw  uint16 // RC channel 18 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chancount  uint8  // Total number of RC channels being received. This can be larger than 18, indicating that more channels are available but not given in this message. This value should be 0 when no RC channels are available.
	Rssi       uint8  // Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
}

// MsgID (generated function)
func (m *RcChannels) MsgID() MessageID {
	return MSG_ID_RC_CHANNELS
}

// MsgName (generated function)
func (m *RcChannels) MsgName() string {
	return "RcChannels"
}

// Pack (generated function)
func (m *RcChannels) Pack(p *Packet) error {
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
func (m *RcChannels) Unpack(p *Packet) error {
	if len(p.Payload) < 42 {
		return fmt.Errorf("payload too small")
	}
	m.TimeBootMs = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.Chan1Raw = uint16(binary.LittleEndian.Uint16(p.Payload[4:]))
	m.Chan2Raw = uint16(binary.LittleEndian.Uint16(p.Payload[6:]))
	m.Chan3Raw = uint16(binary.LittleEndian.Uint16(p.Payload[8:]))
	m.Chan4Raw = uint16(binary.LittleEndian.Uint16(p.Payload[10:]))
	m.Chan5Raw = uint16(binary.LittleEndian.Uint16(p.Payload[12:]))
	m.Chan6Raw = uint16(binary.LittleEndian.Uint16(p.Payload[14:]))
	m.Chan7Raw = uint16(binary.LittleEndian.Uint16(p.Payload[16:]))
	m.Chan8Raw = uint16(binary.LittleEndian.Uint16(p.Payload[18:]))
	m.Chan9Raw = uint16(binary.LittleEndian.Uint16(p.Payload[20:]))
	m.Chan10Raw = uint16(binary.LittleEndian.Uint16(p.Payload[22:]))
	m.Chan11Raw = uint16(binary.LittleEndian.Uint16(p.Payload[24:]))
	m.Chan12Raw = uint16(binary.LittleEndian.Uint16(p.Payload[26:]))
	m.Chan13Raw = uint16(binary.LittleEndian.Uint16(p.Payload[28:]))
	m.Chan14Raw = uint16(binary.LittleEndian.Uint16(p.Payload[30:]))
	m.Chan15Raw = uint16(binary.LittleEndian.Uint16(p.Payload[32:]))
	m.Chan16Raw = uint16(binary.LittleEndian.Uint16(p.Payload[34:]))
	m.Chan17Raw = uint16(binary.LittleEndian.Uint16(p.Payload[36:]))
	m.Chan18Raw = uint16(binary.LittleEndian.Uint16(p.Payload[38:]))
	m.Chancount = uint8(p.Payload[40])
	m.Rssi = uint8(p.Payload[41])
	return nil
}

// RequestDataStream struct (generated typeinfo)
// THIS INTERFACE IS DEPRECATED. USE SET_MESSAGE_INTERVAL INSTEAD.
type RequestDataStream struct {
	ReqMessageRate  uint16 // The requested message rate
	TargetSystem    uint8  // The target requested to send the message stream.
	TargetComponent uint8  // The target requested to send the message stream.
	ReqStreamID     uint8  // The ID of the requested data stream
	StartStop       uint8  // 1 to start sending, 0 to stop sending.
}

// MsgID (generated function)
func (m *RequestDataStream) MsgID() MessageID {
	return MSG_ID_REQUEST_DATA_STREAM
}

// MsgName (generated function)
func (m *RequestDataStream) MsgName() string {
	return "RequestDataStream"
}

// Pack (generated function)
func (m *RequestDataStream) Pack(p *Packet) error {
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
func (m *RequestDataStream) Unpack(p *Packet) error {
	if len(p.Payload) < 6 {
		return fmt.Errorf("payload too small")
	}
	m.ReqMessageRate = uint16(binary.LittleEndian.Uint16(p.Payload[0:]))
	m.TargetSystem = uint8(p.Payload[2])
	m.TargetComponent = uint8(p.Payload[3])
	m.ReqStreamID = uint8(p.Payload[4])
	m.StartStop = uint8(p.Payload[5])
	return nil
}

// DataStream struct (generated typeinfo)
// THIS INTERFACE IS DEPRECATED. USE MESSAGE_INTERVAL INSTEAD.
type DataStream struct {
	MessageRate uint16 // The message rate
	StreamID    uint8  // The ID of the requested data stream
	OnOff       uint8  // 1 stream is enabled, 0 stream is stopped.
}

// MsgID (generated function)
func (m *DataStream) MsgID() MessageID {
	return MSG_ID_DATA_STREAM
}

// MsgName (generated function)
func (m *DataStream) MsgName() string {
	return "DataStream"
}

// Pack (generated function)
func (m *DataStream) Pack(p *Packet) error {
	payload := make([]byte, 4)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.MessageRate))
	payload[2] = byte(m.StreamID)
	payload[3] = byte(m.OnOff)

	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *DataStream) Unpack(p *Packet) error {
	if len(p.Payload) < 4 {
		return fmt.Errorf("payload too small")
	}
	m.MessageRate = uint16(binary.LittleEndian.Uint16(p.Payload[0:]))
	m.StreamID = uint8(p.Payload[2])
	m.OnOff = uint8(p.Payload[3])
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
func (m *ManualControl) MsgID() MessageID {
	return MSG_ID_MANUAL_CONTROL
}

// MsgName (generated function)
func (m *ManualControl) MsgName() string {
	return "ManualControl"
}

// Pack (generated function)
func (m *ManualControl) Pack(p *Packet) error {
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
func (m *ManualControl) Unpack(p *Packet) error {
	if len(p.Payload) < 11 {
		return fmt.Errorf("payload too small")
	}
	m.X = int16(binary.LittleEndian.Uint16(p.Payload[0:]))
	m.Y = int16(binary.LittleEndian.Uint16(p.Payload[2:]))
	m.Z = int16(binary.LittleEndian.Uint16(p.Payload[4:]))
	m.R = int16(binary.LittleEndian.Uint16(p.Payload[6:]))
	m.Buttons = uint16(binary.LittleEndian.Uint16(p.Payload[8:]))
	m.Target = uint8(p.Payload[10])
	return nil
}

// RcChannelsOverride struct (generated typeinfo)
// The RAW values of the RC channels sent to the MAV to override info received from the RC radio. A value of UINT16_MAX means no change to that channel. A value of 0 means control of that channel should be released back to the RC radio. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual receivers/transmitters might violate this specification.
type RcChannelsOverride struct {
	Chan1Raw        uint16 // RC channel 1 value, in microseconds. A value of UINT16_MAX means to ignore this field.
	Chan2Raw        uint16 // RC channel 2 value, in microseconds. A value of UINT16_MAX means to ignore this field.
	Chan3Raw        uint16 // RC channel 3 value, in microseconds. A value of UINT16_MAX means to ignore this field.
	Chan4Raw        uint16 // RC channel 4 value, in microseconds. A value of UINT16_MAX means to ignore this field.
	Chan5Raw        uint16 // RC channel 5 value, in microseconds. A value of UINT16_MAX means to ignore this field.
	Chan6Raw        uint16 // RC channel 6 value, in microseconds. A value of UINT16_MAX means to ignore this field.
	Chan7Raw        uint16 // RC channel 7 value, in microseconds. A value of UINT16_MAX means to ignore this field.
	Chan8Raw        uint16 // RC channel 8 value, in microseconds. A value of UINT16_MAX means to ignore this field.
	TargetSystem    uint8  // System ID
	TargetComponent uint8  // Component ID
}

// MsgID (generated function)
func (m *RcChannelsOverride) MsgID() MessageID {
	return MSG_ID_RC_CHANNELS_OVERRIDE
}

// MsgName (generated function)
func (m *RcChannelsOverride) MsgName() string {
	return "RcChannelsOverride"
}

// Pack (generated function)
func (m *RcChannelsOverride) Pack(p *Packet) error {
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
func (m *RcChannelsOverride) Unpack(p *Packet) error {
	if len(p.Payload) < 18 {
		return fmt.Errorf("payload too small")
	}
	m.Chan1Raw = uint16(binary.LittleEndian.Uint16(p.Payload[0:]))
	m.Chan2Raw = uint16(binary.LittleEndian.Uint16(p.Payload[2:]))
	m.Chan3Raw = uint16(binary.LittleEndian.Uint16(p.Payload[4:]))
	m.Chan4Raw = uint16(binary.LittleEndian.Uint16(p.Payload[6:]))
	m.Chan5Raw = uint16(binary.LittleEndian.Uint16(p.Payload[8:]))
	m.Chan6Raw = uint16(binary.LittleEndian.Uint16(p.Payload[10:]))
	m.Chan7Raw = uint16(binary.LittleEndian.Uint16(p.Payload[12:]))
	m.Chan8Raw = uint16(binary.LittleEndian.Uint16(p.Payload[14:]))
	m.TargetSystem = uint8(p.Payload[16])
	m.TargetComponent = uint8(p.Payload[17])
	return nil
}

// MissionItemInt struct (generated typeinfo)
// Message encoding a mission item. This message is emitted to announce
//                 the presence of a mission item and to set a mission item on the system. The mission item can be either in x, y, z meters (type: LOCAL) or x:lat, y:lon, z:altitude. Local frame is Z-down, right handed (NED), global frame is Z-up, right handed (ENU). See alsohttp://qgroundcontrol.org/mavlink/waypoint_protocol.
type MissionItemInt struct {
	Param1          float32 // PARAM1, see MAV_CMD enum
	Param2          float32 // PARAM2, see MAV_CMD enum
	Param3          float32 // PARAM3, see MAV_CMD enum
	Param4          float32 // PARAM4, see MAV_CMD enum
	X               int32   // PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7
	Y               int32   // PARAM6 / y position: local: x position in meters * 1e4, global: longitude in degrees *10^7
	Z               float32 // PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame.
	Seq             uint16  // Waypoint ID (sequence number). Starts at zero. Increases monotonically for each waypoint, no gaps in the sequence (0,1,2,3,4).
	Command         uint16  // The scheduled action for the MISSION. see MAV_CMD in common.xml MAVLink specs
	TargetSystem    uint8   // System ID
	TargetComponent uint8   // Component ID
	Frame           uint8   // The coordinate system of the MISSION. see MAV_FRAME in mavlink_types.h
	Current         uint8   // false:0, true:1
	Autocontinue    uint8   // autocontinue to next wp
}

// MsgID (generated function)
func (m *MissionItemInt) MsgID() MessageID {
	return MSG_ID_MISSION_ITEM_INT
}

// MsgName (generated function)
func (m *MissionItemInt) MsgName() string {
	return "MissionItemInt"
}

// Pack (generated function)
func (m *MissionItemInt) Pack(p *Packet) error {
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
func (m *MissionItemInt) Unpack(p *Packet) error {
	if len(p.Payload) < 37 {
		return fmt.Errorf("payload too small")
	}
	m.Param1 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.Param2 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	m.Param3 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.Param4 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	m.X = int32(binary.LittleEndian.Uint32(p.Payload[16:]))
	m.Y = int32(binary.LittleEndian.Uint32(p.Payload[20:]))
	m.Z = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	m.Seq = uint16(binary.LittleEndian.Uint16(p.Payload[28:]))
	m.Command = uint16(binary.LittleEndian.Uint16(p.Payload[30:]))
	m.TargetSystem = uint8(p.Payload[32])
	m.TargetComponent = uint8(p.Payload[33])
	m.Frame = uint8(p.Payload[34])
	m.Current = uint8(p.Payload[35])
	m.Autocontinue = uint8(p.Payload[36])
	return nil
}

// VfrHud struct (generated typeinfo)
// Metrics typically displayed on a HUD for fixed wing aircraft
type VfrHud struct {
	Airspeed    float32 // Current airspeed in m/s
	Groundspeed float32 // Current ground speed in m/s
	Alt         float32 // Current altitude (MSL), in meters
	Climb       float32 // Current climb rate in meters/second
	Heading     int16   // Current heading in degrees, in compass units (0..360, 0=north)
	Throttle    uint16  // Current throttle setting in integer percent, 0 to 100
}

// MsgID (generated function)
func (m *VfrHud) MsgID() MessageID {
	return MSG_ID_VFR_HUD
}

// MsgName (generated function)
func (m *VfrHud) MsgName() string {
	return "VfrHud"
}

// Pack (generated function)
func (m *VfrHud) Pack(p *Packet) error {
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
func (m *VfrHud) Unpack(p *Packet) error {
	if len(p.Payload) < 20 {
		return fmt.Errorf("payload too small")
	}
	m.Airspeed = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.Groundspeed = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	m.Alt = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.Climb = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	m.Heading = int16(binary.LittleEndian.Uint16(p.Payload[16:]))
	m.Throttle = uint16(binary.LittleEndian.Uint16(p.Payload[18:]))
	return nil
}

// CommandInt struct (generated typeinfo)
// Message encoding a command with parameters as scaled integers. Scaling depends on the actual command value.
type CommandInt struct {
	Param1          float32 // PARAM1, see MAV_CMD enum
	Param2          float32 // PARAM2, see MAV_CMD enum
	Param3          float32 // PARAM3, see MAV_CMD enum
	Param4          float32 // PARAM4, see MAV_CMD enum
	X               int32   // PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7
	Y               int32   // PARAM6 / local: y position in meters * 1e4, global: longitude in degrees * 10^7
	Z               float32 // PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame.
	Command         uint16  // The scheduled action for the mission item. see MAV_CMD in common.xml MAVLink specs
	TargetSystem    uint8   // System ID
	TargetComponent uint8   // Component ID
	Frame           uint8   // The coordinate system of the COMMAND. see MAV_FRAME in mavlink_types.h
	Current         uint8   // false:0, true:1
	Autocontinue    uint8   // autocontinue to next wp
}

// MsgID (generated function)
func (m *CommandInt) MsgID() MessageID {
	return MSG_ID_COMMAND_INT
}

// MsgName (generated function)
func (m *CommandInt) MsgName() string {
	return "CommandInt"
}

// Pack (generated function)
func (m *CommandInt) Pack(p *Packet) error {
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
func (m *CommandInt) Unpack(p *Packet) error {
	if len(p.Payload) < 35 {
		return fmt.Errorf("payload too small")
	}
	m.Param1 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.Param2 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	m.Param3 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.Param4 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	m.X = int32(binary.LittleEndian.Uint32(p.Payload[16:]))
	m.Y = int32(binary.LittleEndian.Uint32(p.Payload[20:]))
	m.Z = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	m.Command = uint16(binary.LittleEndian.Uint16(p.Payload[28:]))
	m.TargetSystem = uint8(p.Payload[30])
	m.TargetComponent = uint8(p.Payload[31])
	m.Frame = uint8(p.Payload[32])
	m.Current = uint8(p.Payload[33])
	m.Autocontinue = uint8(p.Payload[34])
	return nil
}

// CommandLong struct (generated typeinfo)
// Send a command with up to seven parameters to the MAV
type CommandLong struct {
	Param1          float32 // Parameter 1, as defined by MAV_CMD enum.
	Param2          float32 // Parameter 2, as defined by MAV_CMD enum.
	Param3          float32 // Parameter 3, as defined by MAV_CMD enum.
	Param4          float32 // Parameter 4, as defined by MAV_CMD enum.
	Param5          float32 // Parameter 5, as defined by MAV_CMD enum.
	Param6          float32 // Parameter 6, as defined by MAV_CMD enum.
	Param7          float32 // Parameter 7, as defined by MAV_CMD enum.
	Command         uint16  // Command ID, as defined by MAV_CMD enum.
	TargetSystem    uint8   // System which should execute the command
	TargetComponent uint8   // Component which should execute the command, 0 for all components
	Confirmation    uint8   // 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
}

// MsgID (generated function)
func (m *CommandLong) MsgID() MessageID {
	return MSG_ID_COMMAND_LONG
}

// MsgName (generated function)
func (m *CommandLong) MsgName() string {
	return "CommandLong"
}

// Pack (generated function)
func (m *CommandLong) Pack(p *Packet) error {
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
func (m *CommandLong) Unpack(p *Packet) error {
	if len(p.Payload) < 33 {
		return fmt.Errorf("payload too small")
	}
	m.Param1 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.Param2 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	m.Param3 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.Param4 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	m.Param5 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	m.Param6 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	m.Param7 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	m.Command = uint16(binary.LittleEndian.Uint16(p.Payload[28:]))
	m.TargetSystem = uint8(p.Payload[30])
	m.TargetComponent = uint8(p.Payload[31])
	m.Confirmation = uint8(p.Payload[32])
	return nil
}

// CommandAck struct (generated typeinfo)
// Report status of a command. Includes feedback wether the command was executed.
type CommandAck struct {
	Command uint16 // Command ID, as defined by MAV_CMD enum.
	Result  uint8  // See MAV_RESULT enum
}

// MsgID (generated function)
func (m *CommandAck) MsgID() MessageID {
	return MSG_ID_COMMAND_ACK
}

// MsgName (generated function)
func (m *CommandAck) MsgName() string {
	return "CommandAck"
}

// Pack (generated function)
func (m *CommandAck) Pack(p *Packet) error {
	payload := make([]byte, 3)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.Command))
	payload[2] = byte(m.Result)

	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *CommandAck) Unpack(p *Packet) error {
	if len(p.Payload) < 3 {
		return fmt.Errorf("payload too small")
	}
	m.Command = uint16(binary.LittleEndian.Uint16(p.Payload[0:]))
	m.Result = uint8(p.Payload[2])
	return nil
}

// ManualSetpoint struct (generated typeinfo)
// Setpoint in roll, pitch, yaw and thrust from the operator
type ManualSetpoint struct {
	TimeBootMs           uint32  // Timestamp in milliseconds since system boot
	Roll                 float32 // Desired roll rate in radians per second
	Pitch                float32 // Desired pitch rate in radians per second
	Yaw                  float32 // Desired yaw rate in radians per second
	Thrust               float32 // Collective thrust, normalized to 0 .. 1
	ModeSwitch           uint8   // Flight mode switch position, 0.. 255
	ManualOverrideSwitch uint8   // Override mode switch position, 0.. 255
}

// MsgID (generated function)
func (m *ManualSetpoint) MsgID() MessageID {
	return MSG_ID_MANUAL_SETPOINT
}

// MsgName (generated function)
func (m *ManualSetpoint) MsgName() string {
	return "ManualSetpoint"
}

// Pack (generated function)
func (m *ManualSetpoint) Pack(p *Packet) error {
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
func (m *ManualSetpoint) Unpack(p *Packet) error {
	if len(p.Payload) < 22 {
		return fmt.Errorf("payload too small")
	}
	m.TimeBootMs = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.Roll = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	m.Pitch = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.Yaw = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	m.Thrust = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	m.ModeSwitch = uint8(p.Payload[20])
	m.ManualOverrideSwitch = uint8(p.Payload[21])
	return nil
}

// SetAttitudeTarget struct (generated typeinfo)
// Sets a desired vehicle attitude. Used by an external controller to command the vehicle (manual controller or other system).
type SetAttitudeTarget struct {
	TimeBootMs      uint32     // Timestamp in milliseconds since system boot
	Q               [4]float32 // Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
	BodyRollRate    float32    // Body roll rate in radians per second
	BodyPitchRate   float32    // Body roll rate in radians per second
	BodyYawRate     float32    // Body roll rate in radians per second
	Thrust          float32    // Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)
	TargetSystem    uint8      // System ID
	TargetComponent uint8      // Component ID
	TypeMask        uint8      // Mappings: If any of these bits are set, the corresponding input should be ignored: bit 1: body roll rate, bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 6: reserved, bit 7: throttle, bit 8: attitude
}

// MsgID (generated function)
func (m *SetAttitudeTarget) MsgID() MessageID {
	return MSG_ID_SET_ATTITUDE_TARGET
}

// MsgName (generated function)
func (m *SetAttitudeTarget) MsgName() string {
	return "SetAttitudeTarget"
}

// Pack (generated function)
func (m *SetAttitudeTarget) Pack(p *Packet) error {
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
func (m *SetAttitudeTarget) Unpack(p *Packet) error {
	if len(p.Payload) < 39 {
		return fmt.Errorf("payload too small")
	}
	m.TimeBootMs = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	for i := 0; i < len(m.Q); i++ {
		m.Q[i] = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4+i*4:]))
	}
	m.BodyRollRate = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	m.BodyPitchRate = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	m.BodyYawRate = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	m.Thrust = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[32:]))
	m.TargetSystem = uint8(p.Payload[36])
	m.TargetComponent = uint8(p.Payload[37])
	m.TypeMask = uint8(p.Payload[38])
	return nil
}

// AttitudeTarget struct (generated typeinfo)
// Reports the current commanded attitude of the vehicle as specified by the autopilot. This should match the commands sent in a SET_ATTITUDE_TARGET message if the vehicle is being controlled this way.
type AttitudeTarget struct {
	TimeBootMs    uint32     // Timestamp in milliseconds since system boot
	Q             [4]float32 // Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
	BodyRollRate  float32    // Body roll rate in radians per second
	BodyPitchRate float32    // Body pitch rate in radians per second
	BodyYawRate   float32    // Body yaw rate in radians per second
	Thrust        float32    // Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)
	TypeMask      uint8      // Mappings: If any of these bits are set, the corresponding input should be ignored: bit 1: body roll rate, bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 7: reserved, bit 8: attitude
}

// MsgID (generated function)
func (m *AttitudeTarget) MsgID() MessageID {
	return MSG_ID_ATTITUDE_TARGET
}

// MsgName (generated function)
func (m *AttitudeTarget) MsgName() string {
	return "AttitudeTarget"
}

// Pack (generated function)
func (m *AttitudeTarget) Pack(p *Packet) error {
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
func (m *AttitudeTarget) Unpack(p *Packet) error {
	if len(p.Payload) < 37 {
		return fmt.Errorf("payload too small")
	}
	m.TimeBootMs = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	for i := 0; i < len(m.Q); i++ {
		m.Q[i] = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4+i*4:]))
	}
	m.BodyRollRate = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	m.BodyPitchRate = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	m.BodyYawRate = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	m.Thrust = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[32:]))
	m.TypeMask = uint8(p.Payload[36])
	return nil
}

// SetPositionTargetLocalNed struct (generated typeinfo)
// Sets a desired vehicle position in a local north-east-down coordinate frame. Used by an external controller to command the vehicle (manual controller or other system).
type SetPositionTargetLocalNed struct {
	TimeBootMs      uint32  // Timestamp in milliseconds since system boot
	X               float32 // X Position in NED frame in meters
	Y               float32 // Y Position in NED frame in meters
	Z               float32 // Z Position in NED frame in meters (note, altitude is negative in NED)
	Vx              float32 // X velocity in NED frame in meter / s
	Vy              float32 // Y velocity in NED frame in meter / s
	Vz              float32 // Z velocity in NED frame in meter / s
	Afx             float32 // X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Afy             float32 // Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Afz             float32 // Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Yaw             float32 // yaw setpoint in rad
	YawRate         float32 // yaw rate setpoint in rad/s
	TypeMask        uint16  // Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint, bit 11: yaw, bit 12: yaw rate
	TargetSystem    uint8   // System ID
	TargetComponent uint8   // Component ID
	CoordinateFrame uint8   // Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED = 9
}

// MsgID (generated function)
func (m *SetPositionTargetLocalNed) MsgID() MessageID {
	return MSG_ID_SET_POSITION_TARGET_LOCAL_NED
}

// MsgName (generated function)
func (m *SetPositionTargetLocalNed) MsgName() string {
	return "SetPositionTargetLocalNed"
}

// Pack (generated function)
func (m *SetPositionTargetLocalNed) Pack(p *Packet) error {
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
func (m *SetPositionTargetLocalNed) Unpack(p *Packet) error {
	if len(p.Payload) < 53 {
		return fmt.Errorf("payload too small")
	}
	m.TimeBootMs = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.X = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	m.Y = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.Z = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	m.Vx = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	m.Vy = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	m.Vz = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	m.Afx = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	m.Afy = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[32:]))
	m.Afz = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[36:]))
	m.Yaw = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[40:]))
	m.YawRate = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[44:]))
	m.TypeMask = uint16(binary.LittleEndian.Uint16(p.Payload[48:]))
	m.TargetSystem = uint8(p.Payload[50])
	m.TargetComponent = uint8(p.Payload[51])
	m.CoordinateFrame = uint8(p.Payload[52])
	return nil
}

// PositionTargetLocalNed struct (generated typeinfo)
// Reports the current commanded vehicle position, velocity, and acceleration as specified by the autopilot. This should match the commands sent in SET_POSITION_TARGET_LOCAL_NED if the vehicle is being controlled this way.
type PositionTargetLocalNed struct {
	TimeBootMs      uint32  // Timestamp in milliseconds since system boot
	X               float32 // X Position in NED frame in meters
	Y               float32 // Y Position in NED frame in meters
	Z               float32 // Z Position in NED frame in meters (note, altitude is negative in NED)
	Vx              float32 // X velocity in NED frame in meter / s
	Vy              float32 // Y velocity in NED frame in meter / s
	Vz              float32 // Z velocity in NED frame in meter / s
	Afx             float32 // X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Afy             float32 // Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Afz             float32 // Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Yaw             float32 // yaw setpoint in rad
	YawRate         float32 // yaw rate setpoint in rad/s
	TypeMask        uint16  // Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint, bit 11: yaw, bit 12: yaw rate
	CoordinateFrame uint8   // Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED = 9
}

// MsgID (generated function)
func (m *PositionTargetLocalNed) MsgID() MessageID {
	return MSG_ID_POSITION_TARGET_LOCAL_NED
}

// MsgName (generated function)
func (m *PositionTargetLocalNed) MsgName() string {
	return "PositionTargetLocalNed"
}

// Pack (generated function)
func (m *PositionTargetLocalNed) Pack(p *Packet) error {
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
func (m *PositionTargetLocalNed) Unpack(p *Packet) error {
	if len(p.Payload) < 51 {
		return fmt.Errorf("payload too small")
	}
	m.TimeBootMs = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.X = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	m.Y = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.Z = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	m.Vx = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	m.Vy = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	m.Vz = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	m.Afx = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	m.Afy = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[32:]))
	m.Afz = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[36:]))
	m.Yaw = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[40:]))
	m.YawRate = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[44:]))
	m.TypeMask = uint16(binary.LittleEndian.Uint16(p.Payload[48:]))
	m.CoordinateFrame = uint8(p.Payload[50])
	return nil
}

// SetPositionTargetGlobalInt struct (generated typeinfo)
// Sets a desired vehicle position, velocity, and/or acceleration in a global coordinate system (WGS84). Used by an external controller to command the vehicle (manual controller or other system).
type SetPositionTargetGlobalInt struct {
	TimeBootMs      uint32  // Timestamp in milliseconds since system boot. The rationale for the timestamp in the setpoint is to allow the system to compensate for the transport delay of the setpoint. This allows the system to compensate processing latency.
	LatInt          int32   // X Position in WGS84 frame in 1e7 * meters
	LonInt          int32   // Y Position in WGS84 frame in 1e7 * meters
	Alt             float32 // Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
	Vx              float32 // X velocity in NED frame in meter / s
	Vy              float32 // Y velocity in NED frame in meter / s
	Vz              float32 // Z velocity in NED frame in meter / s
	Afx             float32 // X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Afy             float32 // Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Afz             float32 // Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Yaw             float32 // yaw setpoint in rad
	YawRate         float32 // yaw rate setpoint in rad/s
	TypeMask        uint16  // Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint, bit 11: yaw, bit 12: yaw rate
	TargetSystem    uint8   // System ID
	TargetComponent uint8   // Component ID
	CoordinateFrame uint8   // Valid options are: MAV_FRAME_GLOBAL_INT = 5, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT = 11
}

// MsgID (generated function)
func (m *SetPositionTargetGlobalInt) MsgID() MessageID {
	return MSG_ID_SET_POSITION_TARGET_GLOBAL_INT
}

// MsgName (generated function)
func (m *SetPositionTargetGlobalInt) MsgName() string {
	return "SetPositionTargetGlobalInt"
}

// Pack (generated function)
func (m *SetPositionTargetGlobalInt) Pack(p *Packet) error {
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
func (m *SetPositionTargetGlobalInt) Unpack(p *Packet) error {
	if len(p.Payload) < 53 {
		return fmt.Errorf("payload too small")
	}
	m.TimeBootMs = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.LatInt = int32(binary.LittleEndian.Uint32(p.Payload[4:]))
	m.LonInt = int32(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.Alt = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	m.Vx = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	m.Vy = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	m.Vz = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	m.Afx = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	m.Afy = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[32:]))
	m.Afz = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[36:]))
	m.Yaw = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[40:]))
	m.YawRate = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[44:]))
	m.TypeMask = uint16(binary.LittleEndian.Uint16(p.Payload[48:]))
	m.TargetSystem = uint8(p.Payload[50])
	m.TargetComponent = uint8(p.Payload[51])
	m.CoordinateFrame = uint8(p.Payload[52])
	return nil
}

// PositionTargetGlobalInt struct (generated typeinfo)
// Reports the current commanded vehicle position, velocity, and acceleration as specified by the autopilot. This should match the commands sent in SET_POSITION_TARGET_GLOBAL_INT if the vehicle is being controlled this way.
type PositionTargetGlobalInt struct {
	TimeBootMs      uint32  // Timestamp in milliseconds since system boot. The rationale for the timestamp in the setpoint is to allow the system to compensate for the transport delay of the setpoint. This allows the system to compensate processing latency.
	LatInt          int32   // X Position in WGS84 frame in 1e7 * meters
	LonInt          int32   // Y Position in WGS84 frame in 1e7 * meters
	Alt             float32 // Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
	Vx              float32 // X velocity in NED frame in meter / s
	Vy              float32 // Y velocity in NED frame in meter / s
	Vz              float32 // Z velocity in NED frame in meter / s
	Afx             float32 // X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Afy             float32 // Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Afz             float32 // Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Yaw             float32 // yaw setpoint in rad
	YawRate         float32 // yaw rate setpoint in rad/s
	TypeMask        uint16  // Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint, bit 11: yaw, bit 12: yaw rate
	CoordinateFrame uint8   // Valid options are: MAV_FRAME_GLOBAL_INT = 5, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT = 11
}

// MsgID (generated function)
func (m *PositionTargetGlobalInt) MsgID() MessageID {
	return MSG_ID_POSITION_TARGET_GLOBAL_INT
}

// MsgName (generated function)
func (m *PositionTargetGlobalInt) MsgName() string {
	return "PositionTargetGlobalInt"
}

// Pack (generated function)
func (m *PositionTargetGlobalInt) Pack(p *Packet) error {
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
func (m *PositionTargetGlobalInt) Unpack(p *Packet) error {
	if len(p.Payload) < 51 {
		return fmt.Errorf("payload too small")
	}
	m.TimeBootMs = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.LatInt = int32(binary.LittleEndian.Uint32(p.Payload[4:]))
	m.LonInt = int32(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.Alt = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	m.Vx = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	m.Vy = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	m.Vz = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	m.Afx = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	m.Afy = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[32:]))
	m.Afz = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[36:]))
	m.Yaw = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[40:]))
	m.YawRate = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[44:]))
	m.TypeMask = uint16(binary.LittleEndian.Uint16(p.Payload[48:]))
	m.CoordinateFrame = uint8(p.Payload[50])
	return nil
}

// LocalPositionNedSystemGlobalOffset struct (generated typeinfo)
// The offset in X, Y, Z and yaw between the LOCAL_POSITION_NED messages of MAV X and the global coordinate frame in NED coordinates. Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down convention)
type LocalPositionNedSystemGlobalOffset struct {
	TimeBootMs uint32  // Timestamp (milliseconds since system boot)
	X          float32 // X Position
	Y          float32 // Y Position
	Z          float32 // Z Position
	Roll       float32 // Roll
	Pitch      float32 // Pitch
	Yaw        float32 // Yaw
}

// MsgID (generated function)
func (m *LocalPositionNedSystemGlobalOffset) MsgID() MessageID {
	return MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET
}

// MsgName (generated function)
func (m *LocalPositionNedSystemGlobalOffset) MsgName() string {
	return "LocalPositionNedSystemGlobalOffset"
}

// Pack (generated function)
func (m *LocalPositionNedSystemGlobalOffset) Pack(p *Packet) error {
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
func (m *LocalPositionNedSystemGlobalOffset) Unpack(p *Packet) error {
	if len(p.Payload) < 28 {
		return fmt.Errorf("payload too small")
	}
	m.TimeBootMs = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.X = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	m.Y = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.Z = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	m.Roll = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	m.Pitch = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	m.Yaw = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	return nil
}

// HilState struct (generated typeinfo)
// DEPRECATED PACKET! Suffers from missing airspeed fields and singularities due to Euler angles. Please use HIL_STATE_QUATERNION instead. Sent from simulation to autopilot. This packet is useful for high throughput applications such as hardware in the loop simulations.
type HilState struct {
	TimeUsec   uint64  // Timestamp (microseconds since UNIX epoch or microseconds since system boot)
	Roll       float32 // Roll angle (rad)
	Pitch      float32 // Pitch angle (rad)
	Yaw        float32 // Yaw angle (rad)
	Rollspeed  float32 // Body frame roll / phi angular speed (rad/s)
	Pitchspeed float32 // Body frame pitch / theta angular speed (rad/s)
	Yawspeed   float32 // Body frame yaw / psi angular speed (rad/s)
	Lat        int32   // Latitude, expressed as * 1E7
	Lon        int32   // Longitude, expressed as * 1E7
	Alt        int32   // Altitude in meters, expressed as * 1000 (millimeters)
	Vx         int16   // Ground X Speed (Latitude), expressed as m/s * 100
	Vy         int16   // Ground Y Speed (Longitude), expressed as m/s * 100
	Vz         int16   // Ground Z Speed (Altitude), expressed as m/s * 100
	Xacc       int16   // X acceleration (mg)
	Yacc       int16   // Y acceleration (mg)
	Zacc       int16   // Z acceleration (mg)
}

// MsgID (generated function)
func (m *HilState) MsgID() MessageID {
	return MSG_ID_HIL_STATE
}

// MsgName (generated function)
func (m *HilState) MsgName() string {
	return "HilState"
}

// Pack (generated function)
func (m *HilState) Pack(p *Packet) error {
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
func (m *HilState) Unpack(p *Packet) error {
	if len(p.Payload) < 56 {
		return fmt.Errorf("payload too small")
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	m.Roll = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.Pitch = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	m.Yaw = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	m.Rollspeed = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	m.Pitchspeed = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	m.Yawspeed = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	m.Lat = int32(binary.LittleEndian.Uint32(p.Payload[32:]))
	m.Lon = int32(binary.LittleEndian.Uint32(p.Payload[36:]))
	m.Alt = int32(binary.LittleEndian.Uint32(p.Payload[40:]))
	m.Vx = int16(binary.LittleEndian.Uint16(p.Payload[44:]))
	m.Vy = int16(binary.LittleEndian.Uint16(p.Payload[46:]))
	m.Vz = int16(binary.LittleEndian.Uint16(p.Payload[48:]))
	m.Xacc = int16(binary.LittleEndian.Uint16(p.Payload[50:]))
	m.Yacc = int16(binary.LittleEndian.Uint16(p.Payload[52:]))
	m.Zacc = int16(binary.LittleEndian.Uint16(p.Payload[54:]))
	return nil
}

// HilControls struct (generated typeinfo)
// Sent from autopilot to simulation. Hardware in the loop control outputs
type HilControls struct {
	TimeUsec      uint64  // Timestamp (microseconds since UNIX epoch or microseconds since system boot)
	RollAilerons  float32 // Control output -1 .. 1
	PitchElevator float32 // Control output -1 .. 1
	YawRudder     float32 // Control output -1 .. 1
	Throttle      float32 // Throttle 0 .. 1
	Aux1          float32 // Aux 1, -1 .. 1
	Aux2          float32 // Aux 2, -1 .. 1
	Aux3          float32 // Aux 3, -1 .. 1
	Aux4          float32 // Aux 4, -1 .. 1
	Mode          uint8   // System mode (MAV_MODE)
	NavMode       uint8   // Navigation mode (MAV_NAV_MODE)
}

// MsgID (generated function)
func (m *HilControls) MsgID() MessageID {
	return MSG_ID_HIL_CONTROLS
}

// MsgName (generated function)
func (m *HilControls) MsgName() string {
	return "HilControls"
}

// Pack (generated function)
func (m *HilControls) Pack(p *Packet) error {
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
func (m *HilControls) Unpack(p *Packet) error {
	if len(p.Payload) < 42 {
		return fmt.Errorf("payload too small")
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	m.RollAilerons = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.PitchElevator = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	m.YawRudder = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	m.Throttle = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	m.Aux1 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	m.Aux2 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	m.Aux3 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[32:]))
	m.Aux4 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[36:]))
	m.Mode = uint8(p.Payload[40])
	m.NavMode = uint8(p.Payload[41])
	return nil
}

// HilRcInputsRaw struct (generated typeinfo)
// Sent from simulation to autopilot. The RAW values of the RC channels received. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual receivers/transmitters might violate this specification.
type HilRcInputsRaw struct {
	TimeUsec  uint64 // Timestamp (microseconds since UNIX epoch or microseconds since system boot)
	Chan1Raw  uint16 // RC channel 1 value, in microseconds
	Chan2Raw  uint16 // RC channel 2 value, in microseconds
	Chan3Raw  uint16 // RC channel 3 value, in microseconds
	Chan4Raw  uint16 // RC channel 4 value, in microseconds
	Chan5Raw  uint16 // RC channel 5 value, in microseconds
	Chan6Raw  uint16 // RC channel 6 value, in microseconds
	Chan7Raw  uint16 // RC channel 7 value, in microseconds
	Chan8Raw  uint16 // RC channel 8 value, in microseconds
	Chan9Raw  uint16 // RC channel 9 value, in microseconds
	Chan10Raw uint16 // RC channel 10 value, in microseconds
	Chan11Raw uint16 // RC channel 11 value, in microseconds
	Chan12Raw uint16 // RC channel 12 value, in microseconds
	Rssi      uint8  // Receive signal strength indicator, 0: 0%, 255: 100%
}

// MsgID (generated function)
func (m *HilRcInputsRaw) MsgID() MessageID {
	return MSG_ID_HIL_RC_INPUTS_RAW
}

// MsgName (generated function)
func (m *HilRcInputsRaw) MsgName() string {
	return "HilRcInputsRaw"
}

// Pack (generated function)
func (m *HilRcInputsRaw) Pack(p *Packet) error {
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
func (m *HilRcInputsRaw) Unpack(p *Packet) error {
	if len(p.Payload) < 33 {
		return fmt.Errorf("payload too small")
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	m.Chan1Raw = uint16(binary.LittleEndian.Uint16(p.Payload[8:]))
	m.Chan2Raw = uint16(binary.LittleEndian.Uint16(p.Payload[10:]))
	m.Chan3Raw = uint16(binary.LittleEndian.Uint16(p.Payload[12:]))
	m.Chan4Raw = uint16(binary.LittleEndian.Uint16(p.Payload[14:]))
	m.Chan5Raw = uint16(binary.LittleEndian.Uint16(p.Payload[16:]))
	m.Chan6Raw = uint16(binary.LittleEndian.Uint16(p.Payload[18:]))
	m.Chan7Raw = uint16(binary.LittleEndian.Uint16(p.Payload[20:]))
	m.Chan8Raw = uint16(binary.LittleEndian.Uint16(p.Payload[22:]))
	m.Chan9Raw = uint16(binary.LittleEndian.Uint16(p.Payload[24:]))
	m.Chan10Raw = uint16(binary.LittleEndian.Uint16(p.Payload[26:]))
	m.Chan11Raw = uint16(binary.LittleEndian.Uint16(p.Payload[28:]))
	m.Chan12Raw = uint16(binary.LittleEndian.Uint16(p.Payload[30:]))
	m.Rssi = uint8(p.Payload[32])
	return nil
}

// HilActuatorControls struct (generated typeinfo)
// Sent from autopilot to simulation. Hardware in the loop control outputs (replacement for HIL_CONTROLS)
type HilActuatorControls struct {
	TimeUsec uint64      // Timestamp (microseconds since UNIX epoch or microseconds since system boot)
	Flags    uint64      // Flags as bitfield, reserved for future use.
	Controls [16]float32 // Control outputs -1 .. 1. Channel assignment depends on the simulated hardware.
	Mode     uint8       // System mode (MAV_MODE), includes arming state.
}

// MsgID (generated function)
func (m *HilActuatorControls) MsgID() MessageID {
	return MSG_ID_HIL_ACTUATOR_CONTROLS
}

// MsgName (generated function)
func (m *HilActuatorControls) MsgName() string {
	return "HilActuatorControls"
}

// Pack (generated function)
func (m *HilActuatorControls) Pack(p *Packet) error {
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
func (m *HilActuatorControls) Unpack(p *Packet) error {
	if len(p.Payload) < 81 {
		return fmt.Errorf("payload too small")
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	m.Flags = uint64(binary.LittleEndian.Uint64(p.Payload[8:]))
	for i := 0; i < len(m.Controls); i++ {
		m.Controls[i] = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16+i*4:]))
	}
	m.Mode = uint8(p.Payload[80])
	return nil
}

// OpticalFlow struct (generated typeinfo)
// Optical flow from a flow sensor (e.g. optical mouse sensor)
type OpticalFlow struct {
	TimeUsec       uint64  // Timestamp (UNIX)
	FlowCompMX     float32 // Flow in meters in x-sensor direction, angular-speed compensated
	FlowCompMY     float32 // Flow in meters in y-sensor direction, angular-speed compensated
	GroundDistance float32 // Ground distance in meters. Positive value: distance known. Negative value: Unknown distance
	FlowX          int16   // Flow in pixels * 10 in x-sensor direction (dezi-pixels)
	FlowY          int16   // Flow in pixels * 10 in y-sensor direction (dezi-pixels)
	SensorID       uint8   // Sensor ID
	Quality        uint8   // Optical flow quality / confidence. 0: bad, 255: maximum quality
}

// MsgID (generated function)
func (m *OpticalFlow) MsgID() MessageID {
	return MSG_ID_OPTICAL_FLOW
}

// MsgName (generated function)
func (m *OpticalFlow) MsgName() string {
	return "OpticalFlow"
}

// Pack (generated function)
func (m *OpticalFlow) Pack(p *Packet) error {
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
func (m *OpticalFlow) Unpack(p *Packet) error {
	if len(p.Payload) < 26 {
		return fmt.Errorf("payload too small")
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	m.FlowCompMX = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.FlowCompMY = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	m.GroundDistance = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	m.FlowX = int16(binary.LittleEndian.Uint16(p.Payload[20:]))
	m.FlowY = int16(binary.LittleEndian.Uint16(p.Payload[22:]))
	m.SensorID = uint8(p.Payload[24])
	m.Quality = uint8(p.Payload[25])
	return nil
}

// GlobalVisionPositionEstimate struct (generated typeinfo)
//
type GlobalVisionPositionEstimate struct {
	Usec  uint64  // Timestamp (microseconds, synced to UNIX time or since system boot)
	X     float32 // Global X position
	Y     float32 // Global Y position
	Z     float32 // Global Z position
	Roll  float32 // Roll angle in rad
	Pitch float32 // Pitch angle in rad
	Yaw   float32 // Yaw angle in rad
}

// MsgID (generated function)
func (m *GlobalVisionPositionEstimate) MsgID() MessageID {
	return MSG_ID_GLOBAL_VISION_POSITION_ESTIMATE
}

// MsgName (generated function)
func (m *GlobalVisionPositionEstimate) MsgName() string {
	return "GlobalVisionPositionEstimate"
}

// Pack (generated function)
func (m *GlobalVisionPositionEstimate) Pack(p *Packet) error {
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
func (m *GlobalVisionPositionEstimate) Unpack(p *Packet) error {
	if len(p.Payload) < 32 {
		return fmt.Errorf("payload too small")
	}
	m.Usec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	m.X = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.Y = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	m.Z = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	m.Roll = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	m.Pitch = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	m.Yaw = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	return nil
}

// VisionPositionEstimate struct (generated typeinfo)
//
type VisionPositionEstimate struct {
	Usec  uint64  // Timestamp (microseconds, synced to UNIX time or since system boot)
	X     float32 // Global X position
	Y     float32 // Global Y position
	Z     float32 // Global Z position
	Roll  float32 // Roll angle in rad
	Pitch float32 // Pitch angle in rad
	Yaw   float32 // Yaw angle in rad
}

// MsgID (generated function)
func (m *VisionPositionEstimate) MsgID() MessageID {
	return MSG_ID_VISION_POSITION_ESTIMATE
}

// MsgName (generated function)
func (m *VisionPositionEstimate) MsgName() string {
	return "VisionPositionEstimate"
}

// Pack (generated function)
func (m *VisionPositionEstimate) Pack(p *Packet) error {
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
func (m *VisionPositionEstimate) Unpack(p *Packet) error {
	if len(p.Payload) < 32 {
		return fmt.Errorf("payload too small")
	}
	m.Usec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	m.X = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.Y = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	m.Z = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	m.Roll = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	m.Pitch = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	m.Yaw = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	return nil
}

// VisionSpeedEstimate struct (generated typeinfo)
//
type VisionSpeedEstimate struct {
	Usec uint64  // Timestamp (microseconds, synced to UNIX time or since system boot)
	X    float32 // Global X speed
	Y    float32 // Global Y speed
	Z    float32 // Global Z speed
}

// MsgID (generated function)
func (m *VisionSpeedEstimate) MsgID() MessageID {
	return MSG_ID_VISION_SPEED_ESTIMATE
}

// MsgName (generated function)
func (m *VisionSpeedEstimate) MsgName() string {
	return "VisionSpeedEstimate"
}

// Pack (generated function)
func (m *VisionSpeedEstimate) Pack(p *Packet) error {
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
func (m *VisionSpeedEstimate) Unpack(p *Packet) error {
	if len(p.Payload) < 20 {
		return fmt.Errorf("payload too small")
	}
	m.Usec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	m.X = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.Y = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	m.Z = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	return nil
}

// ViconPositionEstimate struct (generated typeinfo)
//
type ViconPositionEstimate struct {
	Usec  uint64  // Timestamp (microseconds, synced to UNIX time or since system boot)
	X     float32 // Global X position
	Y     float32 // Global Y position
	Z     float32 // Global Z position
	Roll  float32 // Roll angle in rad
	Pitch float32 // Pitch angle in rad
	Yaw   float32 // Yaw angle in rad
}

// MsgID (generated function)
func (m *ViconPositionEstimate) MsgID() MessageID {
	return MSG_ID_VICON_POSITION_ESTIMATE
}

// MsgName (generated function)
func (m *ViconPositionEstimate) MsgName() string {
	return "ViconPositionEstimate"
}

// Pack (generated function)
func (m *ViconPositionEstimate) Pack(p *Packet) error {
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
func (m *ViconPositionEstimate) Unpack(p *Packet) error {
	if len(p.Payload) < 32 {
		return fmt.Errorf("payload too small")
	}
	m.Usec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	m.X = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.Y = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	m.Z = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	m.Roll = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	m.Pitch = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	m.Yaw = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	return nil
}

// HighresImu struct (generated typeinfo)
// The IMU readings in SI units in NED body frame
type HighresImu struct {
	TimeUsec      uint64  // Timestamp (microseconds, synced to UNIX time or since system boot)
	Xacc          float32 // X acceleration (m/s^2)
	Yacc          float32 // Y acceleration (m/s^2)
	Zacc          float32 // Z acceleration (m/s^2)
	Xgyro         float32 // Angular speed around X axis (rad / sec)
	Ygyro         float32 // Angular speed around Y axis (rad / sec)
	Zgyro         float32 // Angular speed around Z axis (rad / sec)
	Xmag          float32 // X Magnetic field (Gauss)
	Ymag          float32 // Y Magnetic field (Gauss)
	Zmag          float32 // Z Magnetic field (Gauss)
	AbsPressure   float32 // Absolute pressure in millibar
	DiffPressure  float32 // Differential pressure in millibar
	PressureAlt   float32 // Altitude calculated from pressure
	Temperature   float32 // Temperature in degrees celsius
	FieldsUpdated uint16  // Bitmask for fields that have updated since last message, bit 0 = xacc, bit 12: temperature
}

// MsgID (generated function)
func (m *HighresImu) MsgID() MessageID {
	return MSG_ID_HIGHRES_IMU
}

// MsgName (generated function)
func (m *HighresImu) MsgName() string {
	return "HighresImu"
}

// Pack (generated function)
func (m *HighresImu) Pack(p *Packet) error {
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
func (m *HighresImu) Unpack(p *Packet) error {
	if len(p.Payload) < 62 {
		return fmt.Errorf("payload too small")
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	m.Xacc = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.Yacc = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	m.Zacc = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	m.Xgyro = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	m.Ygyro = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	m.Zgyro = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	m.Xmag = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[32:]))
	m.Ymag = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[36:]))
	m.Zmag = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[40:]))
	m.AbsPressure = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[44:]))
	m.DiffPressure = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[48:]))
	m.PressureAlt = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[52:]))
	m.Temperature = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[56:]))
	m.FieldsUpdated = uint16(binary.LittleEndian.Uint16(p.Payload[60:]))
	return nil
}

// OpticalFlowRad struct (generated typeinfo)
// Optical flow from an angular rate flow sensor (e.g. PX4FLOW or mouse sensor)
type OpticalFlowRad struct {
	TimeUsec            uint64  // Timestamp (microseconds, synced to UNIX time or since system boot)
	IntegrationTimeUs   uint32  // Integration time in microseconds. Divide integrated_x and integrated_y by the integration time to obtain average flow. The integration time also indicates the.
	IntegratedX         float32 // Flow in radians around X axis (Sensor RH rotation about the X axis induces a positive flow. Sensor linear motion along the positive Y axis induces a negative flow.)
	IntegratedY         float32 // Flow in radians around Y axis (Sensor RH rotation about the Y axis induces a positive flow. Sensor linear motion along the positive X axis induces a positive flow.)
	IntegratedXgyro     float32 // RH rotation around X axis (rad)
	IntegratedYgyro     float32 // RH rotation around Y axis (rad)
	IntegratedZgyro     float32 // RH rotation around Z axis (rad)
	TimeDeltaDistanceUs uint32  // Time in microseconds since the distance was sampled.
	Distance            float32 // Distance to the center of the flow field in meters. Positive value (including zero): distance known. Negative value: Unknown distance.
	Temperature         int16   // Temperature * 100 in centi-degrees Celsius
	SensorID            uint8   // Sensor ID
	Quality             uint8   // Optical flow quality / confidence. 0: no valid flow, 255: maximum quality
}

// MsgID (generated function)
func (m *OpticalFlowRad) MsgID() MessageID {
	return MSG_ID_OPTICAL_FLOW_RAD
}

// MsgName (generated function)
func (m *OpticalFlowRad) MsgName() string {
	return "OpticalFlowRad"
}

// Pack (generated function)
func (m *OpticalFlowRad) Pack(p *Packet) error {
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
func (m *OpticalFlowRad) Unpack(p *Packet) error {
	if len(p.Payload) < 44 {
		return fmt.Errorf("payload too small")
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	m.IntegrationTimeUs = uint32(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.IntegratedX = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	m.IntegratedY = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	m.IntegratedXgyro = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	m.IntegratedYgyro = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	m.IntegratedZgyro = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	m.TimeDeltaDistanceUs = uint32(binary.LittleEndian.Uint32(p.Payload[32:]))
	m.Distance = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[36:]))
	m.Temperature = int16(binary.LittleEndian.Uint16(p.Payload[40:]))
	m.SensorID = uint8(p.Payload[42])
	m.Quality = uint8(p.Payload[43])
	return nil
}

// HilSensor struct (generated typeinfo)
// The IMU readings in SI units in NED body frame
type HilSensor struct {
	TimeUsec      uint64  // Timestamp (microseconds, synced to UNIX time or since system boot)
	Xacc          float32 // X acceleration (m/s^2)
	Yacc          float32 // Y acceleration (m/s^2)
	Zacc          float32 // Z acceleration (m/s^2)
	Xgyro         float32 // Angular speed around X axis in body frame (rad / sec)
	Ygyro         float32 // Angular speed around Y axis in body frame (rad / sec)
	Zgyro         float32 // Angular speed around Z axis in body frame (rad / sec)
	Xmag          float32 // X Magnetic field (Gauss)
	Ymag          float32 // Y Magnetic field (Gauss)
	Zmag          float32 // Z Magnetic field (Gauss)
	AbsPressure   float32 // Absolute pressure in millibar
	DiffPressure  float32 // Differential pressure (airspeed) in millibar
	PressureAlt   float32 // Altitude calculated from pressure
	Temperature   float32 // Temperature in degrees celsius
	FieldsUpdated uint32  // Bitmask for fields that have updated since last message, bit 0 = xacc, bit 12: temperature, bit 31: full reset of attitude/position/velocities/etc was performed in sim.
}

// MsgID (generated function)
func (m *HilSensor) MsgID() MessageID {
	return MSG_ID_HIL_SENSOR
}

// MsgName (generated function)
func (m *HilSensor) MsgName() string {
	return "HilSensor"
}

// Pack (generated function)
func (m *HilSensor) Pack(p *Packet) error {
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
func (m *HilSensor) Unpack(p *Packet) error {
	if len(p.Payload) < 64 {
		return fmt.Errorf("payload too small")
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	m.Xacc = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.Yacc = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	m.Zacc = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	m.Xgyro = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	m.Ygyro = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	m.Zgyro = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	m.Xmag = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[32:]))
	m.Ymag = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[36:]))
	m.Zmag = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[40:]))
	m.AbsPressure = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[44:]))
	m.DiffPressure = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[48:]))
	m.PressureAlt = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[52:]))
	m.Temperature = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[56:]))
	m.FieldsUpdated = uint32(binary.LittleEndian.Uint32(p.Payload[60:]))
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
	Xacc       float32 // X acceleration m/s/s
	Yacc       float32 // Y acceleration m/s/s
	Zacc       float32 // Z acceleration m/s/s
	Xgyro      float32 // Angular speed around X axis rad/s
	Ygyro      float32 // Angular speed around Y axis rad/s
	Zgyro      float32 // Angular speed around Z axis rad/s
	Lat        float32 // Latitude in degrees
	Lon        float32 // Longitude in degrees
	Alt        float32 // Altitude in meters
	StdDevHorz float32 // Horizontal position standard deviation
	StdDevVert float32 // Vertical position standard deviation
	Vn         float32 // True velocity in m/s in NORTH direction in earth-fixed NED frame
	Ve         float32 // True velocity in m/s in EAST direction in earth-fixed NED frame
	Vd         float32 // True velocity in m/s in DOWN direction in earth-fixed NED frame
}

// MsgID (generated function)
func (m *SimState) MsgID() MessageID {
	return MSG_ID_SIM_STATE
}

// MsgName (generated function)
func (m *SimState) MsgName() string {
	return "SimState"
}

// Pack (generated function)
func (m *SimState) Pack(p *Packet) error {
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
func (m *SimState) Unpack(p *Packet) error {
	if len(p.Payload) < 84 {
		return fmt.Errorf("payload too small")
	}
	m.Q1 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.Q2 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	m.Q3 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.Q4 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	m.Roll = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	m.Pitch = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	m.Yaw = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	m.Xacc = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	m.Yacc = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[32:]))
	m.Zacc = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[36:]))
	m.Xgyro = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[40:]))
	m.Ygyro = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[44:]))
	m.Zgyro = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[48:]))
	m.Lat = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[52:]))
	m.Lon = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[56:]))
	m.Alt = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[60:]))
	m.StdDevHorz = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[64:]))
	m.StdDevVert = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[68:]))
	m.Vn = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[72:]))
	m.Ve = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[76:]))
	m.Vd = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[80:]))
	return nil
}

// RadioStatus struct (generated typeinfo)
// Status generated by radio and injected into MAVLink stream.
type RadioStatus struct {
	Rxerrors uint16 // Receive errors
	Fixed    uint16 // Count of error corrected packets
	Rssi     uint8  // Local signal strength
	Remrssi  uint8  // Remote signal strength
	Txbuf    uint8  // Remaining free buffer space in percent.
	Noise    uint8  // Background noise level
	Remnoise uint8  // Remote background noise level
}

// MsgID (generated function)
func (m *RadioStatus) MsgID() MessageID {
	return MSG_ID_RADIO_STATUS
}

// MsgName (generated function)
func (m *RadioStatus) MsgName() string {
	return "RadioStatus"
}

// Pack (generated function)
func (m *RadioStatus) Pack(p *Packet) error {
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
func (m *RadioStatus) Unpack(p *Packet) error {
	if len(p.Payload) < 9 {
		return fmt.Errorf("payload too small")
	}
	m.Rxerrors = uint16(binary.LittleEndian.Uint16(p.Payload[0:]))
	m.Fixed = uint16(binary.LittleEndian.Uint16(p.Payload[2:]))
	m.Rssi = uint8(p.Payload[4])
	m.Remrssi = uint8(p.Payload[5])
	m.Txbuf = uint8(p.Payload[6])
	m.Noise = uint8(p.Payload[7])
	m.Remnoise = uint8(p.Payload[8])
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
func (m *FileTransferProtocol) MsgID() MessageID {
	return MSG_ID_FILE_TRANSFER_PROTOCOL
}

// MsgName (generated function)
func (m *FileTransferProtocol) MsgName() string {
	return "FileTransferProtocol"
}

// Pack (generated function)
func (m *FileTransferProtocol) Pack(p *Packet) error {
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
func (m *FileTransferProtocol) Unpack(p *Packet) error {
	if len(p.Payload) < 254 {
		return fmt.Errorf("payload too small")
	}
	m.TargetNetwork = uint8(p.Payload[0])
	m.TargetSystem = uint8(p.Payload[1])
	m.TargetComponent = uint8(p.Payload[2])
	copy(m.Payload[:], p.Payload[3:254])
	return nil
}

// Timesync struct (generated typeinfo)
// Time synchronization message.
type Timesync struct {
	Tc1 int64 // Time sync timestamp 1
	Ts1 int64 // Time sync timestamp 2
}

// MsgID (generated function)
func (m *Timesync) MsgID() MessageID {
	return MSG_ID_TIMESYNC
}

// MsgName (generated function)
func (m *Timesync) MsgName() string {
	return "Timesync"
}

// Pack (generated function)
func (m *Timesync) Pack(p *Packet) error {
	payload := make([]byte, 16)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.Tc1))
	binary.LittleEndian.PutUint64(payload[8:], uint64(m.Ts1))

	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *Timesync) Unpack(p *Packet) error {
	if len(p.Payload) < 16 {
		return fmt.Errorf("payload too small")
	}
	m.Tc1 = int64(binary.LittleEndian.Uint64(p.Payload[0:]))
	m.Ts1 = int64(binary.LittleEndian.Uint64(p.Payload[8:]))
	return nil
}

// CameraTrigger struct (generated typeinfo)
// Camera-IMU triggering and synchronisation message.
type CameraTrigger struct {
	TimeUsec uint64 // Timestamp for the image frame in microseconds
	Seq      uint32 // Image frame sequence
}

// MsgID (generated function)
func (m *CameraTrigger) MsgID() MessageID {
	return MSG_ID_CAMERA_TRIGGER
}

// MsgName (generated function)
func (m *CameraTrigger) MsgName() string {
	return "CameraTrigger"
}

// Pack (generated function)
func (m *CameraTrigger) Pack(p *Packet) error {
	payload := make([]byte, 12)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.TimeUsec))
	binary.LittleEndian.PutUint32(payload[8:], uint32(m.Seq))

	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *CameraTrigger) Unpack(p *Packet) error {
	if len(p.Payload) < 12 {
		return fmt.Errorf("payload too small")
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	m.Seq = uint32(binary.LittleEndian.Uint32(p.Payload[8:]))
	return nil
}

// HilGps struct (generated typeinfo)
// The global position, as returned by the Global Positioning System (GPS). This is
//                  NOT the global position estimate of the sytem, but rather a RAW sensor value. See message GLOBAL_POSITION for the global position estimate. Coordinate frame is right-handed, Z-axis up (GPS frame).
type HilGps struct {
	TimeUsec          uint64 // Timestamp (microseconds since UNIX epoch or microseconds since system boot)
	Lat               int32  // Latitude (WGS84), in degrees * 1E7
	Lon               int32  // Longitude (WGS84), in degrees * 1E7
	Alt               int32  // Altitude (AMSL, not WGS84), in meters * 1000 (positive for up)
	Eph               uint16 // GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
	Epv               uint16 // GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: 65535
	Vel               uint16 // GPS ground speed in cm/s. If unknown, set to: 65535
	Vn                int16  // GPS velocity in cm/s in NORTH direction in earth-fixed NED frame
	Ve                int16  // GPS velocity in cm/s in EAST direction in earth-fixed NED frame
	Vd                int16  // GPS velocity in cm/s in DOWN direction in earth-fixed NED frame
	Cog               uint16 // Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
	FixType           uint8  // 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
	SatellitesVisible uint8  // Number of satellites visible. If unknown, set to 255
}

// MsgID (generated function)
func (m *HilGps) MsgID() MessageID {
	return MSG_ID_HIL_GPS
}

// MsgName (generated function)
func (m *HilGps) MsgName() string {
	return "HilGps"
}

// Pack (generated function)
func (m *HilGps) Pack(p *Packet) error {
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
func (m *HilGps) Unpack(p *Packet) error {
	if len(p.Payload) < 36 {
		return fmt.Errorf("payload too small")
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	m.Lat = int32(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.Lon = int32(binary.LittleEndian.Uint32(p.Payload[12:]))
	m.Alt = int32(binary.LittleEndian.Uint32(p.Payload[16:]))
	m.Eph = uint16(binary.LittleEndian.Uint16(p.Payload[20:]))
	m.Epv = uint16(binary.LittleEndian.Uint16(p.Payload[22:]))
	m.Vel = uint16(binary.LittleEndian.Uint16(p.Payload[24:]))
	m.Vn = int16(binary.LittleEndian.Uint16(p.Payload[26:]))
	m.Ve = int16(binary.LittleEndian.Uint16(p.Payload[28:]))
	m.Vd = int16(binary.LittleEndian.Uint16(p.Payload[30:]))
	m.Cog = uint16(binary.LittleEndian.Uint16(p.Payload[32:]))
	m.FixType = uint8(p.Payload[34])
	m.SatellitesVisible = uint8(p.Payload[35])
	return nil
}

// HilOpticalFlow struct (generated typeinfo)
// Simulated optical flow from a flow sensor (e.g. PX4FLOW or optical mouse sensor)
type HilOpticalFlow struct {
	TimeUsec            uint64  // Timestamp (microseconds, synced to UNIX time or since system boot)
	IntegrationTimeUs   uint32  // Integration time in microseconds. Divide integrated_x and integrated_y by the integration time to obtain average flow. The integration time also indicates the.
	IntegratedX         float32 // Flow in radians around X axis (Sensor RH rotation about the X axis induces a positive flow. Sensor linear motion along the positive Y axis induces a negative flow.)
	IntegratedY         float32 // Flow in radians around Y axis (Sensor RH rotation about the Y axis induces a positive flow. Sensor linear motion along the positive X axis induces a positive flow.)
	IntegratedXgyro     float32 // RH rotation around X axis (rad)
	IntegratedYgyro     float32 // RH rotation around Y axis (rad)
	IntegratedZgyro     float32 // RH rotation around Z axis (rad)
	TimeDeltaDistanceUs uint32  // Time in microseconds since the distance was sampled.
	Distance            float32 // Distance to the center of the flow field in meters. Positive value (including zero): distance known. Negative value: Unknown distance.
	Temperature         int16   // Temperature * 100 in centi-degrees Celsius
	SensorID            uint8   // Sensor ID
	Quality             uint8   // Optical flow quality / confidence. 0: no valid flow, 255: maximum quality
}

// MsgID (generated function)
func (m *HilOpticalFlow) MsgID() MessageID {
	return MSG_ID_HIL_OPTICAL_FLOW
}

// MsgName (generated function)
func (m *HilOpticalFlow) MsgName() string {
	return "HilOpticalFlow"
}

// Pack (generated function)
func (m *HilOpticalFlow) Pack(p *Packet) error {
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
func (m *HilOpticalFlow) Unpack(p *Packet) error {
	if len(p.Payload) < 44 {
		return fmt.Errorf("payload too small")
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	m.IntegrationTimeUs = uint32(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.IntegratedX = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	m.IntegratedY = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	m.IntegratedXgyro = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	m.IntegratedYgyro = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	m.IntegratedZgyro = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	m.TimeDeltaDistanceUs = uint32(binary.LittleEndian.Uint32(p.Payload[32:]))
	m.Distance = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[36:]))
	m.Temperature = int16(binary.LittleEndian.Uint16(p.Payload[40:]))
	m.SensorID = uint8(p.Payload[42])
	m.Quality = uint8(p.Payload[43])
	return nil
}

// HilStateQuaternion struct (generated typeinfo)
// Sent from simulation to autopilot, avoids in contrast to HIL_STATE singularities. This packet is useful for high throughput applications such as hardware in the loop simulations.
type HilStateQuaternion struct {
	TimeUsec           uint64     // Timestamp (microseconds since UNIX epoch or microseconds since system boot)
	AttitudeQuaternion [4]float32 // Vehicle attitude expressed as normalized quaternion in w, x, y, z order (with 1 0 0 0 being the null-rotation)
	Rollspeed          float32    // Body frame roll / phi angular speed (rad/s)
	Pitchspeed         float32    // Body frame pitch / theta angular speed (rad/s)
	Yawspeed           float32    // Body frame yaw / psi angular speed (rad/s)
	Lat                int32      // Latitude, expressed as * 1E7
	Lon                int32      // Longitude, expressed as * 1E7
	Alt                int32      // Altitude in meters, expressed as * 1000 (millimeters)
	Vx                 int16      // Ground X Speed (Latitude), expressed as cm/s
	Vy                 int16      // Ground Y Speed (Longitude), expressed as cm/s
	Vz                 int16      // Ground Z Speed (Altitude), expressed as cm/s
	IndAirspeed        uint16     // Indicated airspeed, expressed as cm/s
	TrueAirspeed       uint16     // True airspeed, expressed as cm/s
	Xacc               int16      // X acceleration (mg)
	Yacc               int16      // Y acceleration (mg)
	Zacc               int16      // Z acceleration (mg)
}

// MsgID (generated function)
func (m *HilStateQuaternion) MsgID() MessageID {
	return MSG_ID_HIL_STATE_QUATERNION
}

// MsgName (generated function)
func (m *HilStateQuaternion) MsgName() string {
	return "HilStateQuaternion"
}

// Pack (generated function)
func (m *HilStateQuaternion) Pack(p *Packet) error {
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
func (m *HilStateQuaternion) Unpack(p *Packet) error {
	if len(p.Payload) < 64 {
		return fmt.Errorf("payload too small")
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	for i := 0; i < len(m.AttitudeQuaternion); i++ {
		m.AttitudeQuaternion[i] = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8+i*4:]))
	}
	m.Rollspeed = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	m.Pitchspeed = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	m.Yawspeed = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[32:]))
	m.Lat = int32(binary.LittleEndian.Uint32(p.Payload[36:]))
	m.Lon = int32(binary.LittleEndian.Uint32(p.Payload[40:]))
	m.Alt = int32(binary.LittleEndian.Uint32(p.Payload[44:]))
	m.Vx = int16(binary.LittleEndian.Uint16(p.Payload[48:]))
	m.Vy = int16(binary.LittleEndian.Uint16(p.Payload[50:]))
	m.Vz = int16(binary.LittleEndian.Uint16(p.Payload[52:]))
	m.IndAirspeed = uint16(binary.LittleEndian.Uint16(p.Payload[54:]))
	m.TrueAirspeed = uint16(binary.LittleEndian.Uint16(p.Payload[56:]))
	m.Xacc = int16(binary.LittleEndian.Uint16(p.Payload[58:]))
	m.Yacc = int16(binary.LittleEndian.Uint16(p.Payload[60:]))
	m.Zacc = int16(binary.LittleEndian.Uint16(p.Payload[62:]))
	return nil
}

// ScaledImu2 struct (generated typeinfo)
// The RAW IMU readings for secondary 9DOF sensor setup. This message should contain the scaled values to the described units
type ScaledImu2 struct {
	TimeBootMs uint32 // Timestamp (milliseconds since system boot)
	Xacc       int16  // X acceleration (mg)
	Yacc       int16  // Y acceleration (mg)
	Zacc       int16  // Z acceleration (mg)
	Xgyro      int16  // Angular speed around X axis (millirad /sec)
	Ygyro      int16  // Angular speed around Y axis (millirad /sec)
	Zgyro      int16  // Angular speed around Z axis (millirad /sec)
	Xmag       int16  // X Magnetic field (milli tesla)
	Ymag       int16  // Y Magnetic field (milli tesla)
	Zmag       int16  // Z Magnetic field (milli tesla)
}

// MsgID (generated function)
func (m *ScaledImu2) MsgID() MessageID {
	return MSG_ID_SCALED_IMU2
}

// MsgName (generated function)
func (m *ScaledImu2) MsgName() string {
	return "ScaledImu2"
}

// Pack (generated function)
func (m *ScaledImu2) Pack(p *Packet) error {
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
func (m *ScaledImu2) Unpack(p *Packet) error {
	if len(p.Payload) < 22 {
		return fmt.Errorf("payload too small")
	}
	m.TimeBootMs = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.Xacc = int16(binary.LittleEndian.Uint16(p.Payload[4:]))
	m.Yacc = int16(binary.LittleEndian.Uint16(p.Payload[6:]))
	m.Zacc = int16(binary.LittleEndian.Uint16(p.Payload[8:]))
	m.Xgyro = int16(binary.LittleEndian.Uint16(p.Payload[10:]))
	m.Ygyro = int16(binary.LittleEndian.Uint16(p.Payload[12:]))
	m.Zgyro = int16(binary.LittleEndian.Uint16(p.Payload[14:]))
	m.Xmag = int16(binary.LittleEndian.Uint16(p.Payload[16:]))
	m.Ymag = int16(binary.LittleEndian.Uint16(p.Payload[18:]))
	m.Zmag = int16(binary.LittleEndian.Uint16(p.Payload[20:]))
	return nil
}

// LogRequestList struct (generated typeinfo)
// Request a list of available logs. On some systems calling this may stop on-board logging until LOG_REQUEST_END is called.
type LogRequestList struct {
	Start           uint16 // First log id (0 for first available)
	End             uint16 // Last log id (0xffff for last available)
	TargetSystem    uint8  // System ID
	TargetComponent uint8  // Component ID
}

// MsgID (generated function)
func (m *LogRequestList) MsgID() MessageID {
	return MSG_ID_LOG_REQUEST_LIST
}

// MsgName (generated function)
func (m *LogRequestList) MsgName() string {
	return "LogRequestList"
}

// Pack (generated function)
func (m *LogRequestList) Pack(p *Packet) error {
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
func (m *LogRequestList) Unpack(p *Packet) error {
	if len(p.Payload) < 6 {
		return fmt.Errorf("payload too small")
	}
	m.Start = uint16(binary.LittleEndian.Uint16(p.Payload[0:]))
	m.End = uint16(binary.LittleEndian.Uint16(p.Payload[2:]))
	m.TargetSystem = uint8(p.Payload[4])
	m.TargetComponent = uint8(p.Payload[5])
	return nil
}

// LogEntry struct (generated typeinfo)
// Reply to LOG_REQUEST_LIST
type LogEntry struct {
	TimeUtc    uint32 // UTC timestamp of log in seconds since 1970, or 0 if not available
	Size       uint32 // Size of the log (may be approximate) in bytes
	ID         uint16 // Log id
	NumLogs    uint16 // Total number of logs
	LastLogNum uint16 // High log number
}

// MsgID (generated function)
func (m *LogEntry) MsgID() MessageID {
	return MSG_ID_LOG_ENTRY
}

// MsgName (generated function)
func (m *LogEntry) MsgName() string {
	return "LogEntry"
}

// Pack (generated function)
func (m *LogEntry) Pack(p *Packet) error {
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
func (m *LogEntry) Unpack(p *Packet) error {
	if len(p.Payload) < 14 {
		return fmt.Errorf("payload too small")
	}
	m.TimeUtc = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.Size = uint32(binary.LittleEndian.Uint32(p.Payload[4:]))
	m.ID = uint16(binary.LittleEndian.Uint16(p.Payload[8:]))
	m.NumLogs = uint16(binary.LittleEndian.Uint16(p.Payload[10:]))
	m.LastLogNum = uint16(binary.LittleEndian.Uint16(p.Payload[12:]))
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
func (m *LogRequestData) MsgID() MessageID {
	return MSG_ID_LOG_REQUEST_DATA
}

// MsgName (generated function)
func (m *LogRequestData) MsgName() string {
	return "LogRequestData"
}

// Pack (generated function)
func (m *LogRequestData) Pack(p *Packet) error {
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
func (m *LogRequestData) Unpack(p *Packet) error {
	if len(p.Payload) < 12 {
		return fmt.Errorf("payload too small")
	}
	m.Ofs = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.Count = uint32(binary.LittleEndian.Uint32(p.Payload[4:]))
	m.ID = uint16(binary.LittleEndian.Uint16(p.Payload[8:]))
	m.TargetSystem = uint8(p.Payload[10])
	m.TargetComponent = uint8(p.Payload[11])
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
func (m *LogData) MsgID() MessageID {
	return MSG_ID_LOG_DATA
}

// MsgName (generated function)
func (m *LogData) MsgName() string {
	return "LogData"
}

// Pack (generated function)
func (m *LogData) Pack(p *Packet) error {
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
func (m *LogData) Unpack(p *Packet) error {
	if len(p.Payload) < 97 {
		return fmt.Errorf("payload too small")
	}
	m.Ofs = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.ID = uint16(binary.LittleEndian.Uint16(p.Payload[4:]))
	m.Count = uint8(p.Payload[6])
	copy(m.Data[:], p.Payload[7:97])
	return nil
}

// LogErase struct (generated typeinfo)
// Erase all logs
type LogErase struct {
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
}

// MsgID (generated function)
func (m *LogErase) MsgID() MessageID {
	return MSG_ID_LOG_ERASE
}

// MsgName (generated function)
func (m *LogErase) MsgName() string {
	return "LogErase"
}

// Pack (generated function)
func (m *LogErase) Pack(p *Packet) error {
	payload := make([]byte, 2)
	payload[0] = byte(m.TargetSystem)
	payload[1] = byte(m.TargetComponent)

	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *LogErase) Unpack(p *Packet) error {
	if len(p.Payload) < 2 {
		return fmt.Errorf("payload too small")
	}
	m.TargetSystem = uint8(p.Payload[0])
	m.TargetComponent = uint8(p.Payload[1])
	return nil
}

// LogRequestEnd struct (generated typeinfo)
// Stop log transfer and resume normal logging
type LogRequestEnd struct {
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
}

// MsgID (generated function)
func (m *LogRequestEnd) MsgID() MessageID {
	return MSG_ID_LOG_REQUEST_END
}

// MsgName (generated function)
func (m *LogRequestEnd) MsgName() string {
	return "LogRequestEnd"
}

// Pack (generated function)
func (m *LogRequestEnd) Pack(p *Packet) error {
	payload := make([]byte, 2)
	payload[0] = byte(m.TargetSystem)
	payload[1] = byte(m.TargetComponent)

	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *LogRequestEnd) Unpack(p *Packet) error {
	if len(p.Payload) < 2 {
		return fmt.Errorf("payload too small")
	}
	m.TargetSystem = uint8(p.Payload[0])
	m.TargetComponent = uint8(p.Payload[1])
	return nil
}

// GpsInjectData struct (generated typeinfo)
// data for injecting into the onboard GPS (used for DGPS)
type GpsInjectData struct {
	TargetSystem    uint8      // System ID
	TargetComponent uint8      // Component ID
	Len             uint8      // data length
	Data            [110]uint8 // raw data (110 is enough for 12 satellites of RTCMv2)
}

// MsgID (generated function)
func (m *GpsInjectData) MsgID() MessageID {
	return MSG_ID_GPS_INJECT_DATA
}

// MsgName (generated function)
func (m *GpsInjectData) MsgName() string {
	return "GpsInjectData"
}

// Pack (generated function)
func (m *GpsInjectData) Pack(p *Packet) error {
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
func (m *GpsInjectData) Unpack(p *Packet) error {
	if len(p.Payload) < 113 {
		return fmt.Errorf("payload too small")
	}
	m.TargetSystem = uint8(p.Payload[0])
	m.TargetComponent = uint8(p.Payload[1])
	m.Len = uint8(p.Payload[2])
	copy(m.Data[:], p.Payload[3:113])
	return nil
}

// Gps2Raw struct (generated typeinfo)
// Second GPS data. Coordinate frame is right-handed, Z-axis up (GPS frame).
type Gps2Raw struct {
	TimeUsec          uint64 // Timestamp (microseconds since UNIX epoch or microseconds since system boot)
	Lat               int32  // Latitude (WGS84), in degrees * 1E7
	Lon               int32  // Longitude (WGS84), in degrees * 1E7
	Alt               int32  // Altitude (AMSL, not WGS84), in meters * 1000 (positive for up)
	DgpsAge           uint32 // Age of DGPS info
	Eph               uint16 // GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: UINT16_MAX
	Epv               uint16 // GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: UINT16_MAX
	Vel               uint16 // GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX
	Cog               uint16 // Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
	FixType           uint8  // See the GPS_FIX_TYPE enum.
	SatellitesVisible uint8  // Number of satellites visible. If unknown, set to 255
	DgpsNumch         uint8  // Number of DGPS satellites
}

// MsgID (generated function)
func (m *Gps2Raw) MsgID() MessageID {
	return MSG_ID_GPS2_RAW
}

// MsgName (generated function)
func (m *Gps2Raw) MsgName() string {
	return "Gps2Raw"
}

// Pack (generated function)
func (m *Gps2Raw) Pack(p *Packet) error {
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
func (m *Gps2Raw) Unpack(p *Packet) error {
	if len(p.Payload) < 35 {
		return fmt.Errorf("payload too small")
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	m.Lat = int32(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.Lon = int32(binary.LittleEndian.Uint32(p.Payload[12:]))
	m.Alt = int32(binary.LittleEndian.Uint32(p.Payload[16:]))
	m.DgpsAge = uint32(binary.LittleEndian.Uint32(p.Payload[20:]))
	m.Eph = uint16(binary.LittleEndian.Uint16(p.Payload[24:]))
	m.Epv = uint16(binary.LittleEndian.Uint16(p.Payload[26:]))
	m.Vel = uint16(binary.LittleEndian.Uint16(p.Payload[28:]))
	m.Cog = uint16(binary.LittleEndian.Uint16(p.Payload[30:]))
	m.FixType = uint8(p.Payload[32])
	m.SatellitesVisible = uint8(p.Payload[33])
	m.DgpsNumch = uint8(p.Payload[34])
	return nil
}

// PowerStatus struct (generated typeinfo)
// Power supply status
type PowerStatus struct {
	Vcc    uint16 // 5V rail voltage in millivolts
	Vservo uint16 // servo rail voltage in millivolts
	Flags  uint16 // power supply status flags (see MAV_POWER_STATUS enum)
}

// MsgID (generated function)
func (m *PowerStatus) MsgID() MessageID {
	return MSG_ID_POWER_STATUS
}

// MsgName (generated function)
func (m *PowerStatus) MsgName() string {
	return "PowerStatus"
}

// Pack (generated function)
func (m *PowerStatus) Pack(p *Packet) error {
	payload := make([]byte, 6)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.Vcc))
	binary.LittleEndian.PutUint16(payload[2:], uint16(m.Vservo))
	binary.LittleEndian.PutUint16(payload[4:], uint16(m.Flags))

	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *PowerStatus) Unpack(p *Packet) error {
	if len(p.Payload) < 6 {
		return fmt.Errorf("payload too small")
	}
	m.Vcc = uint16(binary.LittleEndian.Uint16(p.Payload[0:]))
	m.Vservo = uint16(binary.LittleEndian.Uint16(p.Payload[2:]))
	m.Flags = uint16(binary.LittleEndian.Uint16(p.Payload[4:]))
	return nil
}

// SerialControl struct (generated typeinfo)
// Control a serial port. This can be used for raw access to an onboard serial peripheral such as a GPS or telemetry radio. It is designed to make it possible to update the devices firmware via MAVLink messages or change the devices settings. A message with zero bytes can be used to change just the baudrate.
type SerialControl struct {
	Baudrate uint32    // Baudrate of transfer. Zero means no change.
	Timeout  uint16    // Timeout for reply data in milliseconds
	Device   uint8     // See SERIAL_CONTROL_DEV enum
	Flags    uint8     // See SERIAL_CONTROL_FLAG enum
	Count    uint8     // how many bytes in this transfer
	Data     [70]uint8 // serial data
}

// MsgID (generated function)
func (m *SerialControl) MsgID() MessageID {
	return MSG_ID_SERIAL_CONTROL
}

// MsgName (generated function)
func (m *SerialControl) MsgName() string {
	return "SerialControl"
}

// Pack (generated function)
func (m *SerialControl) Pack(p *Packet) error {
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
func (m *SerialControl) Unpack(p *Packet) error {
	if len(p.Payload) < 79 {
		return fmt.Errorf("payload too small")
	}
	m.Baudrate = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.Timeout = uint16(binary.LittleEndian.Uint16(p.Payload[4:]))
	m.Device = uint8(p.Payload[6])
	m.Flags = uint8(p.Payload[7])
	m.Count = uint8(p.Payload[8])
	copy(m.Data[:], p.Payload[9:79])
	return nil
}

// GpsRtk struct (generated typeinfo)
// RTK GPS data. Gives information on the relative baseline calculation the GPS is reporting
type GpsRtk struct {
	TimeLastBaselineMs uint32 // Time since boot of last baseline message received in ms.
	Tow                uint32 // GPS Time of Week of last baseline
	BaselineAMm        int32  // Current baseline in ECEF x or NED north component in mm.
	BaselineBMm        int32  // Current baseline in ECEF y or NED east component in mm.
	BaselineCMm        int32  // Current baseline in ECEF z or NED down component in mm.
	Accuracy           uint32 // Current estimate of baseline accuracy.
	IarNumHypotheses   int32  // Current number of integer ambiguity hypotheses.
	Wn                 uint16 // GPS Week Number of last baseline
	RtkReceiverID      uint8  // Identification of connected RTK receiver.
	RtkHealth          uint8  // GPS-specific health report for RTK data.
	RtkRate            uint8  // Rate of baseline messages being received by GPS, in HZ
	Nsats              uint8  // Current number of sats used for RTK calculation.
	BaselineCoordsType uint8  // Coordinate system of baseline. 0 == ECEF, 1 == NED
}

// MsgID (generated function)
func (m *GpsRtk) MsgID() MessageID {
	return MSG_ID_GPS_RTK
}

// MsgName (generated function)
func (m *GpsRtk) MsgName() string {
	return "GpsRtk"
}

// Pack (generated function)
func (m *GpsRtk) Pack(p *Packet) error {
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
func (m *GpsRtk) Unpack(p *Packet) error {
	if len(p.Payload) < 35 {
		return fmt.Errorf("payload too small")
	}
	m.TimeLastBaselineMs = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.Tow = uint32(binary.LittleEndian.Uint32(p.Payload[4:]))
	m.BaselineAMm = int32(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.BaselineBMm = int32(binary.LittleEndian.Uint32(p.Payload[12:]))
	m.BaselineCMm = int32(binary.LittleEndian.Uint32(p.Payload[16:]))
	m.Accuracy = uint32(binary.LittleEndian.Uint32(p.Payload[20:]))
	m.IarNumHypotheses = int32(binary.LittleEndian.Uint32(p.Payload[24:]))
	m.Wn = uint16(binary.LittleEndian.Uint16(p.Payload[28:]))
	m.RtkReceiverID = uint8(p.Payload[30])
	m.RtkHealth = uint8(p.Payload[31])
	m.RtkRate = uint8(p.Payload[32])
	m.Nsats = uint8(p.Payload[33])
	m.BaselineCoordsType = uint8(p.Payload[34])
	return nil
}

// Gps2Rtk struct (generated typeinfo)
// RTK GPS data. Gives information on the relative baseline calculation the GPS is reporting
type Gps2Rtk struct {
	TimeLastBaselineMs uint32 // Time since boot of last baseline message received in ms.
	Tow                uint32 // GPS Time of Week of last baseline
	BaselineAMm        int32  // Current baseline in ECEF x or NED north component in mm.
	BaselineBMm        int32  // Current baseline in ECEF y or NED east component in mm.
	BaselineCMm        int32  // Current baseline in ECEF z or NED down component in mm.
	Accuracy           uint32 // Current estimate of baseline accuracy.
	IarNumHypotheses   int32  // Current number of integer ambiguity hypotheses.
	Wn                 uint16 // GPS Week Number of last baseline
	RtkReceiverID      uint8  // Identification of connected RTK receiver.
	RtkHealth          uint8  // GPS-specific health report for RTK data.
	RtkRate            uint8  // Rate of baseline messages being received by GPS, in HZ
	Nsats              uint8  // Current number of sats used for RTK calculation.
	BaselineCoordsType uint8  // Coordinate system of baseline. 0 == ECEF, 1 == NED
}

// MsgID (generated function)
func (m *Gps2Rtk) MsgID() MessageID {
	return MSG_ID_GPS2_RTK
}

// MsgName (generated function)
func (m *Gps2Rtk) MsgName() string {
	return "Gps2Rtk"
}

// Pack (generated function)
func (m *Gps2Rtk) Pack(p *Packet) error {
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
func (m *Gps2Rtk) Unpack(p *Packet) error {
	if len(p.Payload) < 35 {
		return fmt.Errorf("payload too small")
	}
	m.TimeLastBaselineMs = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.Tow = uint32(binary.LittleEndian.Uint32(p.Payload[4:]))
	m.BaselineAMm = int32(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.BaselineBMm = int32(binary.LittleEndian.Uint32(p.Payload[12:]))
	m.BaselineCMm = int32(binary.LittleEndian.Uint32(p.Payload[16:]))
	m.Accuracy = uint32(binary.LittleEndian.Uint32(p.Payload[20:]))
	m.IarNumHypotheses = int32(binary.LittleEndian.Uint32(p.Payload[24:]))
	m.Wn = uint16(binary.LittleEndian.Uint16(p.Payload[28:]))
	m.RtkReceiverID = uint8(p.Payload[30])
	m.RtkHealth = uint8(p.Payload[31])
	m.RtkRate = uint8(p.Payload[32])
	m.Nsats = uint8(p.Payload[33])
	m.BaselineCoordsType = uint8(p.Payload[34])
	return nil
}

// ScaledImu3 struct (generated typeinfo)
// The RAW IMU readings for 3rd 9DOF sensor setup. This message should contain the scaled values to the described units
type ScaledImu3 struct {
	TimeBootMs uint32 // Timestamp (milliseconds since system boot)
	Xacc       int16  // X acceleration (mg)
	Yacc       int16  // Y acceleration (mg)
	Zacc       int16  // Z acceleration (mg)
	Xgyro      int16  // Angular speed around X axis (millirad /sec)
	Ygyro      int16  // Angular speed around Y axis (millirad /sec)
	Zgyro      int16  // Angular speed around Z axis (millirad /sec)
	Xmag       int16  // X Magnetic field (milli tesla)
	Ymag       int16  // Y Magnetic field (milli tesla)
	Zmag       int16  // Z Magnetic field (milli tesla)
}

// MsgID (generated function)
func (m *ScaledImu3) MsgID() MessageID {
	return MSG_ID_SCALED_IMU3
}

// MsgName (generated function)
func (m *ScaledImu3) MsgName() string {
	return "ScaledImu3"
}

// Pack (generated function)
func (m *ScaledImu3) Pack(p *Packet) error {
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
func (m *ScaledImu3) Unpack(p *Packet) error {
	if len(p.Payload) < 22 {
		return fmt.Errorf("payload too small")
	}
	m.TimeBootMs = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.Xacc = int16(binary.LittleEndian.Uint16(p.Payload[4:]))
	m.Yacc = int16(binary.LittleEndian.Uint16(p.Payload[6:]))
	m.Zacc = int16(binary.LittleEndian.Uint16(p.Payload[8:]))
	m.Xgyro = int16(binary.LittleEndian.Uint16(p.Payload[10:]))
	m.Ygyro = int16(binary.LittleEndian.Uint16(p.Payload[12:]))
	m.Zgyro = int16(binary.LittleEndian.Uint16(p.Payload[14:]))
	m.Xmag = int16(binary.LittleEndian.Uint16(p.Payload[16:]))
	m.Ymag = int16(binary.LittleEndian.Uint16(p.Payload[18:]))
	m.Zmag = int16(binary.LittleEndian.Uint16(p.Payload[20:]))
	return nil
}

// DataTransmissionHandshake struct (generated typeinfo)
//
type DataTransmissionHandshake struct {
	Size       uint32 // total data size in bytes (set on ACK only)
	Width      uint16 // Width of a matrix or image
	Height     uint16 // Height of a matrix or image
	Packets    uint16 // number of packets beeing sent (set on ACK only)
	Type       uint8  // type of requested/acknowledged data (as defined in ENUM DATA_TYPES in mavlink/include/mavlink_types.h)
	Payload    uint8  // payload size per packet (normally 253 byte, see DATA field size in message ENCAPSULATED_DATA) (set on ACK only)
	JpgQuality uint8  // JPEG quality out of [1,100]
}

// MsgID (generated function)
func (m *DataTransmissionHandshake) MsgID() MessageID {
	return MSG_ID_DATA_TRANSMISSION_HANDSHAKE
}

// MsgName (generated function)
func (m *DataTransmissionHandshake) MsgName() string {
	return "DataTransmissionHandshake"
}

// Pack (generated function)
func (m *DataTransmissionHandshake) Pack(p *Packet) error {
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
func (m *DataTransmissionHandshake) Unpack(p *Packet) error {
	if len(p.Payload) < 13 {
		return fmt.Errorf("payload too small")
	}
	m.Size = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.Width = uint16(binary.LittleEndian.Uint16(p.Payload[4:]))
	m.Height = uint16(binary.LittleEndian.Uint16(p.Payload[6:]))
	m.Packets = uint16(binary.LittleEndian.Uint16(p.Payload[8:]))
	m.Type = uint8(p.Payload[10])
	m.Payload = uint8(p.Payload[11])
	m.JpgQuality = uint8(p.Payload[12])
	return nil
}

// EncapsulatedData struct (generated typeinfo)
//
type EncapsulatedData struct {
	Seqnr uint16     // sequence number (starting with 0 on every transmission)
	Data  [253]uint8 // image data bytes
}

// MsgID (generated function)
func (m *EncapsulatedData) MsgID() MessageID {
	return MSG_ID_ENCAPSULATED_DATA
}

// MsgName (generated function)
func (m *EncapsulatedData) MsgName() string {
	return "EncapsulatedData"
}

// Pack (generated function)
func (m *EncapsulatedData) Pack(p *Packet) error {
	payload := make([]byte, 255)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.Seqnr))
	copy(payload[2:], m.Data[:])

	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *EncapsulatedData) Unpack(p *Packet) error {
	if len(p.Payload) < 255 {
		return fmt.Errorf("payload too small")
	}
	m.Seqnr = uint16(binary.LittleEndian.Uint16(p.Payload[0:]))
	copy(m.Data[:], p.Payload[2:255])
	return nil
}

// DistanceSensor struct (generated typeinfo)
//
type DistanceSensor struct {
	TimeBootMs      uint32 // Time since system boot
	MinDistance     uint16 // Minimum distance the sensor can measure in centimeters
	MaxDistance     uint16 // Maximum distance the sensor can measure in centimeters
	CurrentDistance uint16 // Current distance reading
	Type            uint8  // Type from MAV_DISTANCE_SENSOR enum.
	ID              uint8  // Onboard ID of the sensor
	Orientation     uint8  // Direction the sensor faces from MAV_SENSOR_ORIENTATION enum.
	Covariance      uint8  // Measurement covariance in centimeters, 0 for unknown / invalid readings
}

// MsgID (generated function)
func (m *DistanceSensor) MsgID() MessageID {
	return MSG_ID_DISTANCE_SENSOR
}

// MsgName (generated function)
func (m *DistanceSensor) MsgName() string {
	return "DistanceSensor"
}

// Pack (generated function)
func (m *DistanceSensor) Pack(p *Packet) error {
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
func (m *DistanceSensor) Unpack(p *Packet) error {
	if len(p.Payload) < 14 {
		return fmt.Errorf("payload too small")
	}
	m.TimeBootMs = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.MinDistance = uint16(binary.LittleEndian.Uint16(p.Payload[4:]))
	m.MaxDistance = uint16(binary.LittleEndian.Uint16(p.Payload[6:]))
	m.CurrentDistance = uint16(binary.LittleEndian.Uint16(p.Payload[8:]))
	m.Type = uint8(p.Payload[10])
	m.ID = uint8(p.Payload[11])
	m.Orientation = uint8(p.Payload[12])
	m.Covariance = uint8(p.Payload[13])
	return nil
}

// TerrainRequest struct (generated typeinfo)
// Request for terrain data and terrain status
type TerrainRequest struct {
	Mask        uint64 // Bitmask of requested 4x4 grids (row major 8x7 array of grids, 56 bits)
	Lat         int32  // Latitude of SW corner of first grid (degrees *10^7)
	Lon         int32  // Longitude of SW corner of first grid (in degrees *10^7)
	GridSpacing uint16 // Grid spacing in meters
}

// MsgID (generated function)
func (m *TerrainRequest) MsgID() MessageID {
	return MSG_ID_TERRAIN_REQUEST
}

// MsgName (generated function)
func (m *TerrainRequest) MsgName() string {
	return "TerrainRequest"
}

// Pack (generated function)
func (m *TerrainRequest) Pack(p *Packet) error {
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
func (m *TerrainRequest) Unpack(p *Packet) error {
	if len(p.Payload) < 18 {
		return fmt.Errorf("payload too small")
	}
	m.Mask = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	m.Lat = int32(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.Lon = int32(binary.LittleEndian.Uint32(p.Payload[12:]))
	m.GridSpacing = uint16(binary.LittleEndian.Uint16(p.Payload[16:]))
	return nil
}

// TerrainData struct (generated typeinfo)
// Terrain data sent from GCS. The lat/lon and grid_spacing must be the same as a lat/lon from a TERRAIN_REQUEST
type TerrainData struct {
	Lat         int32     // Latitude of SW corner of first grid (degrees *10^7)
	Lon         int32     // Longitude of SW corner of first grid (in degrees *10^7)
	GridSpacing uint16    // Grid spacing in meters
	Data        [16]int16 // Terrain data in meters AMSL
	Gridbit     uint8     // bit within the terrain request mask
}

// MsgID (generated function)
func (m *TerrainData) MsgID() MessageID {
	return MSG_ID_TERRAIN_DATA
}

// MsgName (generated function)
func (m *TerrainData) MsgName() string {
	return "TerrainData"
}

// Pack (generated function)
func (m *TerrainData) Pack(p *Packet) error {
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
func (m *TerrainData) Unpack(p *Packet) error {
	if len(p.Payload) < 43 {
		return fmt.Errorf("payload too small")
	}
	m.Lat = int32(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.Lon = int32(binary.LittleEndian.Uint32(p.Payload[4:]))
	m.GridSpacing = uint16(binary.LittleEndian.Uint16(p.Payload[8:]))
	for i := 0; i < len(m.Data); i++ {
		m.Data[i] = int16(binary.LittleEndian.Uint16(p.Payload[10+i*2:]))
	}
	m.Gridbit = uint8(p.Payload[42])
	return nil
}

// TerrainCheck struct (generated typeinfo)
// Request that the vehicle report terrain height at the given location. Used by GCS to check if vehicle has all terrain data needed for a mission.
type TerrainCheck struct {
	Lat int32 // Latitude (degrees *10^7)
	Lon int32 // Longitude (degrees *10^7)
}

// MsgID (generated function)
func (m *TerrainCheck) MsgID() MessageID {
	return MSG_ID_TERRAIN_CHECK
}

// MsgName (generated function)
func (m *TerrainCheck) MsgName() string {
	return "TerrainCheck"
}

// Pack (generated function)
func (m *TerrainCheck) Pack(p *Packet) error {
	payload := make([]byte, 8)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.Lat))
	binary.LittleEndian.PutUint32(payload[4:], uint32(m.Lon))

	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *TerrainCheck) Unpack(p *Packet) error {
	if len(p.Payload) < 8 {
		return fmt.Errorf("payload too small")
	}
	m.Lat = int32(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.Lon = int32(binary.LittleEndian.Uint32(p.Payload[4:]))
	return nil
}

// TerrainReport struct (generated typeinfo)
// Response from a TERRAIN_CHECK request
type TerrainReport struct {
	Lat           int32   // Latitude (degrees *10^7)
	Lon           int32   // Longitude (degrees *10^7)
	TerrainHeight float32 // Terrain height in meters AMSL
	CurrentHeight float32 // Current vehicle height above lat/lon terrain height (meters)
	Spacing       uint16  // grid spacing (zero if terrain at this location unavailable)
	Pending       uint16  // Number of 4x4 terrain blocks waiting to be received or read from disk
	Loaded        uint16  // Number of 4x4 terrain blocks in memory
}

// MsgID (generated function)
func (m *TerrainReport) MsgID() MessageID {
	return MSG_ID_TERRAIN_REPORT
}

// MsgName (generated function)
func (m *TerrainReport) MsgName() string {
	return "TerrainReport"
}

// Pack (generated function)
func (m *TerrainReport) Pack(p *Packet) error {
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
func (m *TerrainReport) Unpack(p *Packet) error {
	if len(p.Payload) < 22 {
		return fmt.Errorf("payload too small")
	}
	m.Lat = int32(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.Lon = int32(binary.LittleEndian.Uint32(p.Payload[4:]))
	m.TerrainHeight = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.CurrentHeight = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	m.Spacing = uint16(binary.LittleEndian.Uint16(p.Payload[16:]))
	m.Pending = uint16(binary.LittleEndian.Uint16(p.Payload[18:]))
	m.Loaded = uint16(binary.LittleEndian.Uint16(p.Payload[20:]))
	return nil
}

// ScaledPressure2 struct (generated typeinfo)
// Barometer readings for 2nd barometer
type ScaledPressure2 struct {
	TimeBootMs  uint32  // Timestamp (milliseconds since system boot)
	PressAbs    float32 // Absolute pressure (hectopascal)
	PressDiff   float32 // Differential pressure 1 (hectopascal)
	Temperature int16   // Temperature measurement (0.01 degrees celsius)
}

// MsgID (generated function)
func (m *ScaledPressure2) MsgID() MessageID {
	return MSG_ID_SCALED_PRESSURE2
}

// MsgName (generated function)
func (m *ScaledPressure2) MsgName() string {
	return "ScaledPressure2"
}

// Pack (generated function)
func (m *ScaledPressure2) Pack(p *Packet) error {
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
func (m *ScaledPressure2) Unpack(p *Packet) error {
	if len(p.Payload) < 14 {
		return fmt.Errorf("payload too small")
	}
	m.TimeBootMs = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.PressAbs = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	m.PressDiff = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.Temperature = int16(binary.LittleEndian.Uint16(p.Payload[12:]))
	return nil
}

// AttPosMocap struct (generated typeinfo)
// Motion capture attitude and position
type AttPosMocap struct {
	TimeUsec uint64     // Timestamp (micros since boot or Unix epoch)
	Q        [4]float32 // Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
	X        float32    // X position in meters (NED)
	Y        float32    // Y position in meters (NED)
	Z        float32    // Z position in meters (NED)
}

// MsgID (generated function)
func (m *AttPosMocap) MsgID() MessageID {
	return MSG_ID_ATT_POS_MOCAP
}

// MsgName (generated function)
func (m *AttPosMocap) MsgName() string {
	return "AttPosMocap"
}

// Pack (generated function)
func (m *AttPosMocap) Pack(p *Packet) error {
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
func (m *AttPosMocap) Unpack(p *Packet) error {
	if len(p.Payload) < 36 {
		return fmt.Errorf("payload too small")
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	for i := 0; i < len(m.Q); i++ {
		m.Q[i] = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8+i*4:]))
	}
	m.X = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	m.Y = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	m.Z = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[32:]))
	return nil
}

// SetActuatorControlTarget struct (generated typeinfo)
// Set the vehicle attitude and body angular rates.
type SetActuatorControlTarget struct {
	TimeUsec        uint64     // Timestamp (micros since boot or Unix epoch)
	Controls        [8]float32 // Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0): (index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through mixer to repurpose them as generic outputs.
	GroupMlx        uint8      // Actuator group. The "_mlx" indicates this is a multi-instance message and a MAVLink parser should use this field to difference between instances.
	TargetSystem    uint8      // System ID
	TargetComponent uint8      // Component ID
}

// MsgID (generated function)
func (m *SetActuatorControlTarget) MsgID() MessageID {
	return MSG_ID_SET_ACTUATOR_CONTROL_TARGET
}

// MsgName (generated function)
func (m *SetActuatorControlTarget) MsgName() string {
	return "SetActuatorControlTarget"
}

// Pack (generated function)
func (m *SetActuatorControlTarget) Pack(p *Packet) error {
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
func (m *SetActuatorControlTarget) Unpack(p *Packet) error {
	if len(p.Payload) < 43 {
		return fmt.Errorf("payload too small")
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	for i := 0; i < len(m.Controls); i++ {
		m.Controls[i] = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8+i*4:]))
	}
	m.GroupMlx = uint8(p.Payload[40])
	m.TargetSystem = uint8(p.Payload[41])
	m.TargetComponent = uint8(p.Payload[42])
	return nil
}

// ActuatorControlTarget struct (generated typeinfo)
// Set the vehicle attitude and body angular rates.
type ActuatorControlTarget struct {
	TimeUsec uint64     // Timestamp (micros since boot or Unix epoch)
	Controls [8]float32 // Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0): (index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through mixer to repurpose them as generic outputs.
	GroupMlx uint8      // Actuator group. The "_mlx" indicates this is a multi-instance message and a MAVLink parser should use this field to difference between instances.
}

// MsgID (generated function)
func (m *ActuatorControlTarget) MsgID() MessageID {
	return MSG_ID_ACTUATOR_CONTROL_TARGET
}

// MsgName (generated function)
func (m *ActuatorControlTarget) MsgName() string {
	return "ActuatorControlTarget"
}

// Pack (generated function)
func (m *ActuatorControlTarget) Pack(p *Packet) error {
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
func (m *ActuatorControlTarget) Unpack(p *Packet) error {
	if len(p.Payload) < 41 {
		return fmt.Errorf("payload too small")
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	for i := 0; i < len(m.Controls); i++ {
		m.Controls[i] = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8+i*4:]))
	}
	m.GroupMlx = uint8(p.Payload[40])
	return nil
}

// Altitude struct (generated typeinfo)
// The current system altitude.
type Altitude struct {
	TimeUsec          uint64  // Timestamp (micros since boot or Unix epoch)
	AltitudeMonotonic float32 // This altitude measure is initialized on system boot and monotonic (it is never reset, but represents the local altitude change). The only guarantee on this field is that it will never be reset and is consistent within a flight. The recommended value for this field is the uncorrected barometric altitude at boot time. This altitude will also drift and vary between flights.
	AltitudeAmsl      float32 // This altitude measure is strictly above mean sea level and might be non-monotonic (it might reset on events like GPS lock or when a new QNH value is set). It should be the altitude to which global altitude waypoints are compared to. Note that it is *not* the GPS altitude, however, most GPS modules already output AMSL by default and not the WGS84 altitude.
	AltitudeLocal     float32 // This is the local altitude in the local coordinate frame. It is not the altitude above home, but in reference to the coordinate origin (0, 0, 0). It is up-positive.
	AltitudeRelative  float32 // This is the altitude above the home position. It resets on each change of the current home position.
	AltitudeTerrain   float32 // This is the altitude above terrain. It might be fed by a terrain database or an altimeter. Values smaller than -1000 should be interpreted as unknown.
	BottomClearance   float32 // This is not the altitude, but the clear space below the system according to the fused clearance estimate. It generally should max out at the maximum range of e.g. the laser altimeter. It is generally a moving target. A negative value indicates no measurement available.
}

// MsgID (generated function)
func (m *Altitude) MsgID() MessageID {
	return MSG_ID_ALTITUDE
}

// MsgName (generated function)
func (m *Altitude) MsgName() string {
	return "Altitude"
}

// Pack (generated function)
func (m *Altitude) Pack(p *Packet) error {
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
func (m *Altitude) Unpack(p *Packet) error {
	if len(p.Payload) < 32 {
		return fmt.Errorf("payload too small")
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	m.AltitudeMonotonic = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.AltitudeAmsl = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	m.AltitudeLocal = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	m.AltitudeRelative = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	m.AltitudeTerrain = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	m.BottomClearance = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	return nil
}

// ResourceRequest struct (generated typeinfo)
// The autopilot is requesting a resource (file, binary, other type of data)
type ResourceRequest struct {
	RequestID    uint8      // Request ID. This ID should be re-used when sending back URI contents
	UriType      uint8      // The type of requested URI. 0 = a file via URL. 1 = a UAVCAN binary
	Uri          [120]uint8 // The requested unique resource identifier (URI). It is not necessarily a straight domain name (depends on the URI type enum)
	TransferType uint8      // The way the autopilot wants to receive the URI. 0 = MAVLink FTP. 1 = binary stream.
	Storage      [120]uint8 // The storage path the autopilot wants the URI to be stored in. Will only be valid if the transfer_type has a storage associated (e.g. MAVLink FTP).
}

// MsgID (generated function)
func (m *ResourceRequest) MsgID() MessageID {
	return MSG_ID_RESOURCE_REQUEST
}

// MsgName (generated function)
func (m *ResourceRequest) MsgName() string {
	return "ResourceRequest"
}

// Pack (generated function)
func (m *ResourceRequest) Pack(p *Packet) error {
	payload := make([]byte, 243)
	payload[0] = byte(m.RequestID)
	payload[1] = byte(m.UriType)
	copy(payload[2:], m.Uri[:])
	payload[122] = byte(m.TransferType)
	copy(payload[123:], m.Storage[:])

	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ResourceRequest) Unpack(p *Packet) error {
	if len(p.Payload) < 243 {
		return fmt.Errorf("payload too small")
	}
	m.RequestID = uint8(p.Payload[0])
	m.UriType = uint8(p.Payload[1])
	copy(m.Uri[:], p.Payload[2:122])
	m.TransferType = uint8(p.Payload[122])
	copy(m.Storage[:], p.Payload[123:243])
	return nil
}

// ScaledPressure3 struct (generated typeinfo)
// Barometer readings for 3rd barometer
type ScaledPressure3 struct {
	TimeBootMs  uint32  // Timestamp (milliseconds since system boot)
	PressAbs    float32 // Absolute pressure (hectopascal)
	PressDiff   float32 // Differential pressure 1 (hectopascal)
	Temperature int16   // Temperature measurement (0.01 degrees celsius)
}

// MsgID (generated function)
func (m *ScaledPressure3) MsgID() MessageID {
	return MSG_ID_SCALED_PRESSURE3
}

// MsgName (generated function)
func (m *ScaledPressure3) MsgName() string {
	return "ScaledPressure3"
}

// Pack (generated function)
func (m *ScaledPressure3) Pack(p *Packet) error {
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
func (m *ScaledPressure3) Unpack(p *Packet) error {
	if len(p.Payload) < 14 {
		return fmt.Errorf("payload too small")
	}
	m.TimeBootMs = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.PressAbs = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	m.PressDiff = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.Temperature = int16(binary.LittleEndian.Uint16(p.Payload[12:]))
	return nil
}

// FollowTarget struct (generated typeinfo)
// current motion information from a designated system
type FollowTarget struct {
	Timestamp       uint64     // Timestamp in milliseconds since system boot
	CustomState     uint64     // button states or switches of a tracker device
	Lat             int32      // Latitude (WGS84), in degrees * 1E7
	Lon             int32      // Longitude (WGS84), in degrees * 1E7
	Alt             float32    // AMSL, in meters
	Vel             [3]float32 // target velocity (0,0,0) for unknown
	Acc             [3]float32 // linear target acceleration (0,0,0) for unknown
	AttitudeQ       [4]float32 // (1 0 0 0 for unknown)
	Rates           [3]float32 // (0 0 0 for unknown)
	PositionCov     [3]float32 // eph epv
	EstCapabilities uint8      // bit positions for tracker reporting capabilities (POS = 0, VEL = 1, ACCEL = 2, ATT + RATES = 3)
}

// MsgID (generated function)
func (m *FollowTarget) MsgID() MessageID {
	return MSG_ID_FOLLOW_TARGET
}

// MsgName (generated function)
func (m *FollowTarget) MsgName() string {
	return "FollowTarget"
}

// Pack (generated function)
func (m *FollowTarget) Pack(p *Packet) error {
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
func (m *FollowTarget) Unpack(p *Packet) error {
	if len(p.Payload) < 93 {
		return fmt.Errorf("payload too small")
	}
	m.Timestamp = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	m.CustomState = uint64(binary.LittleEndian.Uint64(p.Payload[8:]))
	m.Lat = int32(binary.LittleEndian.Uint32(p.Payload[16:]))
	m.Lon = int32(binary.LittleEndian.Uint32(p.Payload[20:]))
	m.Alt = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	for i := 0; i < len(m.Vel); i++ {
		m.Vel[i] = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28+i*4:]))
	}
	for i := 0; i < len(m.Acc); i++ {
		m.Acc[i] = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[40+i*4:]))
	}
	for i := 0; i < len(m.AttitudeQ); i++ {
		m.AttitudeQ[i] = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[52+i*4:]))
	}
	for i := 0; i < len(m.Rates); i++ {
		m.Rates[i] = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[68+i*4:]))
	}
	for i := 0; i < len(m.PositionCov); i++ {
		m.PositionCov[i] = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[80+i*4:]))
	}
	m.EstCapabilities = uint8(p.Payload[92])
	return nil
}

// ControlSystemState struct (generated typeinfo)
// The smoothed, monotonic system state used to feed the control loops of the system.
type ControlSystemState struct {
	TimeUsec    uint64     // Timestamp (micros since boot or Unix epoch)
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
func (m *ControlSystemState) MsgID() MessageID {
	return MSG_ID_CONTROL_SYSTEM_STATE
}

// MsgName (generated function)
func (m *ControlSystemState) MsgName() string {
	return "ControlSystemState"
}

// Pack (generated function)
func (m *ControlSystemState) Pack(p *Packet) error {
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
func (m *ControlSystemState) Unpack(p *Packet) error {
	if len(p.Payload) < 100 {
		return fmt.Errorf("payload too small")
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	m.XAcc = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.YAcc = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	m.ZAcc = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	m.XVel = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	m.YVel = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	m.ZVel = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	m.XPos = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[32:]))
	m.YPos = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[36:]))
	m.ZPos = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[40:]))
	m.Airspeed = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[44:]))
	for i := 0; i < len(m.VelVariance); i++ {
		m.VelVariance[i] = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[48+i*4:]))
	}
	for i := 0; i < len(m.PosVariance); i++ {
		m.PosVariance[i] = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[60+i*4:]))
	}
	for i := 0; i < len(m.Q); i++ {
		m.Q[i] = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[72+i*4:]))
	}
	m.RollRate = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[88:]))
	m.PitchRate = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[92:]))
	m.YawRate = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[96:]))
	return nil
}

// BatteryStatus struct (generated typeinfo)
// Battery information
type BatteryStatus struct {
	CurrentConsumed  int32      // Consumed charge, in milliampere hours (1 = 1 mAh), -1: autopilot does not provide mAh consumption estimate
	EnergyConsumed   int32      // Consumed energy, in HectoJoules (intergrated U*I*dt)  (1 = 100 Joule), -1: autopilot does not provide energy consumption estimate
	Temperature      int16      // Temperature of the battery in centi-degrees celsius. INT16_MAX for unknown temperature.
	Voltages         [10]uint16 // Battery voltage of cells, in millivolts (1 = 1 millivolt). Cells above the valid cell count for this battery should have the UINT16_MAX value.
	CurrentBattery   int16      // Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
	ID               uint8      // Battery ID
	BatteryFunction  uint8      // Function of the battery
	Type             uint8      // Type (chemistry) of the battery
	BatteryRemaining int8       // Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot does not estimate the remaining battery
}

// MsgID (generated function)
func (m *BatteryStatus) MsgID() MessageID {
	return MSG_ID_BATTERY_STATUS
}

// MsgName (generated function)
func (m *BatteryStatus) MsgName() string {
	return "BatteryStatus"
}

// Pack (generated function)
func (m *BatteryStatus) Pack(p *Packet) error {
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
func (m *BatteryStatus) Unpack(p *Packet) error {
	if len(p.Payload) < 36 {
		return fmt.Errorf("payload too small")
	}
	m.CurrentConsumed = int32(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.EnergyConsumed = int32(binary.LittleEndian.Uint32(p.Payload[4:]))
	m.Temperature = int16(binary.LittleEndian.Uint16(p.Payload[8:]))
	for i := 0; i < len(m.Voltages); i++ {
		m.Voltages[i] = uint16(binary.LittleEndian.Uint16(p.Payload[10+i*2:]))
	}
	m.CurrentBattery = int16(binary.LittleEndian.Uint16(p.Payload[30:]))
	m.ID = uint8(p.Payload[32])
	m.BatteryFunction = uint8(p.Payload[33])
	m.Type = uint8(p.Payload[34])
	m.BatteryRemaining = int8(p.Payload[35])
	return nil
}

// AutopilotVersion struct (generated typeinfo)
// Version and capability of autopilot software
type AutopilotVersion struct {
	Capabilities            uint64   // bitmask of capabilities (see MAV_PROTOCOL_CAPABILITY enum)
	Uid                     uint64   // UID if provided by hardware
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
func (m *AutopilotVersion) MsgID() MessageID {
	return MSG_ID_AUTOPILOT_VERSION
}

// MsgName (generated function)
func (m *AutopilotVersion) MsgName() string {
	return "AutopilotVersion"
}

// Pack (generated function)
func (m *AutopilotVersion) Pack(p *Packet) error {
	payload := make([]byte, 60)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.Capabilities))
	binary.LittleEndian.PutUint64(payload[8:], uint64(m.Uid))
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
func (m *AutopilotVersion) Unpack(p *Packet) error {
	if len(p.Payload) < 60 {
		return fmt.Errorf("payload too small")
	}
	m.Capabilities = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	m.Uid = uint64(binary.LittleEndian.Uint64(p.Payload[8:]))
	m.FlightSwVersion = uint32(binary.LittleEndian.Uint32(p.Payload[16:]))
	m.MiddlewareSwVersion = uint32(binary.LittleEndian.Uint32(p.Payload[20:]))
	m.OsSwVersion = uint32(binary.LittleEndian.Uint32(p.Payload[24:]))
	m.BoardVersion = uint32(binary.LittleEndian.Uint32(p.Payload[28:]))
	m.VendorID = uint16(binary.LittleEndian.Uint16(p.Payload[32:]))
	m.ProductID = uint16(binary.LittleEndian.Uint16(p.Payload[34:]))
	copy(m.FlightCustomVersion[:], p.Payload[36:44])
	copy(m.MiddlewareCustomVersion[:], p.Payload[44:52])
	copy(m.OsCustomVersion[:], p.Payload[52:60])
	return nil
}

// LandingTarget struct (generated typeinfo)
// The location of a landing area captured from a downward facing camera
type LandingTarget struct {
	TimeUsec  uint64  // Timestamp (micros since boot or Unix epoch)
	AngleX    float32 // X-axis angular offset (in radians) of the target from the center of the image
	AngleY    float32 // Y-axis angular offset (in radians) of the target from the center of the image
	Distance  float32 // Distance to the target from the vehicle in meters
	SizeX     float32 // Size in radians of target along x-axis
	SizeY     float32 // Size in radians of target along y-axis
	TargetNum uint8   // The ID of the target if multiple targets are present
	Frame     uint8   // MAV_FRAME enum specifying the whether the following feilds are earth-frame, body-frame, etc.
}

// MsgID (generated function)
func (m *LandingTarget) MsgID() MessageID {
	return MSG_ID_LANDING_TARGET
}

// MsgName (generated function)
func (m *LandingTarget) MsgName() string {
	return "LandingTarget"
}

// Pack (generated function)
func (m *LandingTarget) Pack(p *Packet) error {
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
func (m *LandingTarget) Unpack(p *Packet) error {
	if len(p.Payload) < 30 {
		return fmt.Errorf("payload too small")
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	m.AngleX = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.AngleY = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	m.Distance = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	m.SizeX = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	m.SizeY = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	m.TargetNum = uint8(p.Payload[28])
	m.Frame = uint8(p.Payload[29])
	return nil
}

// EstimatorStatus struct (generated typeinfo)
// Estimator status message including flags, innovation test ratios and estimated accuracies. The flags message is an integer bitmask containing information on which EKF outputs are valid. See the ESTIMATOR_STATUS_FLAGS enum definition for further information. The innovaton test ratios show the magnitude of the sensor innovation divided by the innovation check threshold. Under normal operation the innovaton test ratios should be below 0.5 with occasional values up to 1.0. Values greater than 1.0 should be rare under normal operation and indicate that a measurement has been rejected by the filter. The user should be notified if an innovation test ratio greater than 1.0 is recorded. Notifications for values in the range between 0.5 and 1.0 should be optional and controllable by the user.
type EstimatorStatus struct {
	TimeUsec         uint64  // Timestamp (micros since boot or Unix epoch)
	VelRatio         float32 // Velocity innovation test ratio
	PosHorizRatio    float32 // Horizontal position innovation test ratio
	PosVertRatio     float32 // Vertical position innovation test ratio
	MagRatio         float32 // Magnetometer innovation test ratio
	HaglRatio        float32 // Height above terrain innovation test ratio
	TasRatio         float32 // True airspeed innovation test ratio
	PosHorizAccuracy float32 // Horizontal position 1-STD accuracy relative to the EKF local origin (m)
	PosVertAccuracy  float32 // Vertical position 1-STD accuracy relative to the EKF local origin (m)
	Flags            uint16  // Integer bitmask indicating which EKF outputs are valid. See definition for ESTIMATOR_STATUS_FLAGS.
}

// MsgID (generated function)
func (m *EstimatorStatus) MsgID() MessageID {
	return MSG_ID_ESTIMATOR_STATUS
}

// MsgName (generated function)
func (m *EstimatorStatus) MsgName() string {
	return "EstimatorStatus"
}

// Pack (generated function)
func (m *EstimatorStatus) Pack(p *Packet) error {
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
func (m *EstimatorStatus) Unpack(p *Packet) error {
	if len(p.Payload) < 42 {
		return fmt.Errorf("payload too small")
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	m.VelRatio = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.PosHorizRatio = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	m.PosVertRatio = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	m.MagRatio = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	m.HaglRatio = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	m.TasRatio = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	m.PosHorizAccuracy = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[32:]))
	m.PosVertAccuracy = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[36:]))
	m.Flags = uint16(binary.LittleEndian.Uint16(p.Payload[40:]))
	return nil
}

// WindCov struct (generated typeinfo)
//
type WindCov struct {
	TimeUsec      uint64  // Timestamp (micros since boot or Unix epoch)
	WindX         float32 // Wind in X (NED) direction in m/s
	WindY         float32 // Wind in Y (NED) direction in m/s
	WindZ         float32 // Wind in Z (NED) direction in m/s
	VarHoriz      float32 // Variability of the wind in XY. RMS of a 1 Hz lowpassed wind estimate.
	VarVert       float32 // Variability of the wind in Z. RMS of a 1 Hz lowpassed wind estimate.
	WindAlt       float32 // AMSL altitude (m) this measurement was taken at
	HorizAccuracy float32 // Horizontal speed 1-STD accuracy
	VertAccuracy  float32 // Vertical speed 1-STD accuracy
}

// MsgID (generated function)
func (m *WindCov) MsgID() MessageID {
	return MSG_ID_WIND_COV
}

// MsgName (generated function)
func (m *WindCov) MsgName() string {
	return "WindCov"
}

// Pack (generated function)
func (m *WindCov) Pack(p *Packet) error {
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
func (m *WindCov) Unpack(p *Packet) error {
	if len(p.Payload) < 40 {
		return fmt.Errorf("payload too small")
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	m.WindX = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.WindY = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	m.WindZ = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	m.VarHoriz = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	m.VarVert = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	m.WindAlt = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	m.HorizAccuracy = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[32:]))
	m.VertAccuracy = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[36:]))
	return nil
}

// GpsInput struct (generated typeinfo)
// GPS sensor input message.  This is a raw sensor value sent by the GPS. This is NOT the global position estimate of the sytem.
type GpsInput struct {
	TimeUsec          uint64  // Timestamp (micros since boot or Unix epoch)
	TimeWeekMs        uint32  // GPS time (milliseconds from start of GPS week)
	Lat               int32   // Latitude (WGS84), in degrees * 1E7
	Lon               int32   // Longitude (WGS84), in degrees * 1E7
	Alt               float32 // Altitude (AMSL, not WGS84), in m (positive for up)
	Hdop              float32 // GPS HDOP horizontal dilution of position in m
	Vdop              float32 // GPS VDOP vertical dilution of position in m
	Vn                float32 // GPS velocity in m/s in NORTH direction in earth-fixed NED frame
	Ve                float32 // GPS velocity in m/s in EAST direction in earth-fixed NED frame
	Vd                float32 // GPS velocity in m/s in DOWN direction in earth-fixed NED frame
	SpeedAccuracy     float32 // GPS speed accuracy in m/s
	HorizAccuracy     float32 // GPS horizontal accuracy in m
	VertAccuracy      float32 // GPS vertical accuracy in m
	IgnoreFlags       uint16  // Flags indicating which fields to ignore (see GPS_INPUT_IGNORE_FLAGS enum).  All other fields must be provided.
	TimeWeek          uint16  // GPS week number
	GpsID             uint8   // ID of the GPS for multiple GPS inputs
	FixType           uint8   // 0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RTK
	SatellitesVisible uint8   // Number of satellites visible.
}

// MsgID (generated function)
func (m *GpsInput) MsgID() MessageID {
	return MSG_ID_GPS_INPUT
}

// MsgName (generated function)
func (m *GpsInput) MsgName() string {
	return "GpsInput"
}

// Pack (generated function)
func (m *GpsInput) Pack(p *Packet) error {
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
func (m *GpsInput) Unpack(p *Packet) error {
	if len(p.Payload) < 63 {
		return fmt.Errorf("payload too small")
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	m.TimeWeekMs = uint32(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.Lat = int32(binary.LittleEndian.Uint32(p.Payload[12:]))
	m.Lon = int32(binary.LittleEndian.Uint32(p.Payload[16:]))
	m.Alt = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	m.Hdop = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	m.Vdop = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	m.Vn = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[32:]))
	m.Ve = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[36:]))
	m.Vd = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[40:]))
	m.SpeedAccuracy = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[44:]))
	m.HorizAccuracy = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[48:]))
	m.VertAccuracy = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[52:]))
	m.IgnoreFlags = uint16(binary.LittleEndian.Uint16(p.Payload[56:]))
	m.TimeWeek = uint16(binary.LittleEndian.Uint16(p.Payload[58:]))
	m.GpsID = uint8(p.Payload[60])
	m.FixType = uint8(p.Payload[61])
	m.SatellitesVisible = uint8(p.Payload[62])
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
func (m *GpsRtcmData) MsgID() MessageID {
	return MSG_ID_GPS_RTCM_DATA
}

// MsgName (generated function)
func (m *GpsRtcmData) MsgName() string {
	return "GpsRtcmData"
}

// Pack (generated function)
func (m *GpsRtcmData) Pack(p *Packet) error {
	payload := make([]byte, 182)
	payload[0] = byte(m.Flags)
	payload[1] = byte(m.Len)
	copy(payload[2:], m.Data[:])

	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *GpsRtcmData) Unpack(p *Packet) error {
	if len(p.Payload) < 182 {
		return fmt.Errorf("payload too small")
	}
	m.Flags = uint8(p.Payload[0])
	m.Len = uint8(p.Payload[1])
	copy(m.Data[:], p.Payload[2:182])
	return nil
}

// HighLatency struct (generated typeinfo)
// Message appropriate for high latency connections like Iridium
type HighLatency struct {
	CustomMode       uint32 // A bitfield for use for autopilot-specific flags.
	Latitude         int32  // Latitude, expressed as degrees * 1E7
	Longitude        int32  // Longitude, expressed as degrees * 1E7
	Roll             int16  // roll (centidegrees)
	Pitch            int16  // pitch (centidegrees)
	Heading          uint16 // heading (centidegrees)
	HeadingSp        int16  // heading setpoint (centidegrees)
	AltitudeAmsl     int16  // Altitude above mean sea level (meters)
	AltitudeSp       int16  // Altitude setpoint relative to the home position (meters)
	WpDistance       uint16 // distance to target (meters)
	BaseMode         uint8  // System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.h
	LandedState      uint8  // The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.
	Throttle         int8   // throttle (percentage)
	Airspeed         uint8  // airspeed (m/s)
	AirspeedSp       uint8  // airspeed setpoint (m/s)
	Groundspeed      uint8  // groundspeed (m/s)
	ClimbRate        int8   // climb rate (m/s)
	GpsNsat          uint8  // Number of satellites visible. If unknown, set to 255
	GpsFixType       uint8  // See the GPS_FIX_TYPE enum.
	BatteryRemaining uint8  // Remaining battery (percentage)
	Temperature      int8   // Autopilot temperature (degrees C)
	TemperatureAir   int8   // Air temperature (degrees C) from airspeed sensor
	Failsafe         uint8  // failsafe (each bit represents a failsafe where 0=ok, 1=failsafe active (bit0:RC, bit1:batt, bit2:GPS, bit3:GCS, bit4:fence)
	WpNum            uint8  // current waypoint number
}

// MsgID (generated function)
func (m *HighLatency) MsgID() MessageID {
	return MSG_ID_HIGH_LATENCY
}

// MsgName (generated function)
func (m *HighLatency) MsgName() string {
	return "HighLatency"
}

// Pack (generated function)
func (m *HighLatency) Pack(p *Packet) error {
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
func (m *HighLatency) Unpack(p *Packet) error {
	if len(p.Payload) < 40 {
		return fmt.Errorf("payload too small")
	}
	m.CustomMode = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.Latitude = int32(binary.LittleEndian.Uint32(p.Payload[4:]))
	m.Longitude = int32(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.Roll = int16(binary.LittleEndian.Uint16(p.Payload[12:]))
	m.Pitch = int16(binary.LittleEndian.Uint16(p.Payload[14:]))
	m.Heading = uint16(binary.LittleEndian.Uint16(p.Payload[16:]))
	m.HeadingSp = int16(binary.LittleEndian.Uint16(p.Payload[18:]))
	m.AltitudeAmsl = int16(binary.LittleEndian.Uint16(p.Payload[20:]))
	m.AltitudeSp = int16(binary.LittleEndian.Uint16(p.Payload[22:]))
	m.WpDistance = uint16(binary.LittleEndian.Uint16(p.Payload[24:]))
	m.BaseMode = uint8(p.Payload[26])
	m.LandedState = uint8(p.Payload[27])
	m.Throttle = int8(p.Payload[28])
	m.Airspeed = uint8(p.Payload[29])
	m.AirspeedSp = uint8(p.Payload[30])
	m.Groundspeed = uint8(p.Payload[31])
	m.ClimbRate = int8(p.Payload[32])
	m.GpsNsat = uint8(p.Payload[33])
	m.GpsFixType = uint8(p.Payload[34])
	m.BatteryRemaining = uint8(p.Payload[35])
	m.Temperature = int8(p.Payload[36])
	m.TemperatureAir = int8(p.Payload[37])
	m.Failsafe = uint8(p.Payload[38])
	m.WpNum = uint8(p.Payload[39])
	return nil
}

// Vibration struct (generated typeinfo)
// Vibration levels and accelerometer clipping
type Vibration struct {
	TimeUsec   uint64  // Timestamp (micros since boot or Unix epoch)
	VibrationX float32 // Vibration levels on X-axis
	VibrationY float32 // Vibration levels on Y-axis
	VibrationZ float32 // Vibration levels on Z-axis
	Clipping0  uint32  // first accelerometer clipping count
	Clipping1  uint32  // second accelerometer clipping count
	Clipping2  uint32  // third accelerometer clipping count
}

// MsgID (generated function)
func (m *Vibration) MsgID() MessageID {
	return MSG_ID_VIBRATION
}

// MsgName (generated function)
func (m *Vibration) MsgName() string {
	return "Vibration"
}

// Pack (generated function)
func (m *Vibration) Pack(p *Packet) error {
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
func (m *Vibration) Unpack(p *Packet) error {
	if len(p.Payload) < 32 {
		return fmt.Errorf("payload too small")
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	m.VibrationX = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.VibrationY = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	m.VibrationZ = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	m.Clipping0 = uint32(binary.LittleEndian.Uint32(p.Payload[20:]))
	m.Clipping1 = uint32(binary.LittleEndian.Uint32(p.Payload[24:]))
	m.Clipping2 = uint32(binary.LittleEndian.Uint32(p.Payload[28:]))
	return nil
}

// HomePosition struct (generated typeinfo)
// This message can be requested by sending the MAV_CMD_GET_HOME_POSITION command. The position the system will return to and land on. The position is set automatically by the system during the takeoff in case it was not explicitely set by the operator before or after. The position the system will return to and land on. The global and local positions encode the position in the respective coordinate frames, while the q parameter encodes the orientation of the surface. Under normal conditions it describes the heading and terrain slope, which can be used by the aircraft to adjust the approach. The approach 3D vector describes the point to which the system should fly in normal flight mode and then perform a landing sequence along the vector.
type HomePosition struct {
	Latitude  int32      // Latitude (WGS84), in degrees * 1E7
	Longitude int32      // Longitude (WGS84, in degrees * 1E7
	Altitude  int32      // Altitude (AMSL), in meters * 1000 (positive for up)
	X         float32    // Local X position of this position in the local coordinate frame
	Y         float32    // Local Y position of this position in the local coordinate frame
	Z         float32    // Local Z position of this position in the local coordinate frame
	Q         [4]float32 // World to surface normal and heading transformation of the takeoff position. Used to indicate the heading and slope of the ground
	ApproachX float32    // Local X position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.
	ApproachY float32    // Local Y position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.
	ApproachZ float32    // Local Z position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.
}

// MsgID (generated function)
func (m *HomePosition) MsgID() MessageID {
	return MSG_ID_HOME_POSITION
}

// MsgName (generated function)
func (m *HomePosition) MsgName() string {
	return "HomePosition"
}

// Pack (generated function)
func (m *HomePosition) Pack(p *Packet) error {
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
func (m *HomePosition) Unpack(p *Packet) error {
	if len(p.Payload) < 52 {
		return fmt.Errorf("payload too small")
	}
	m.Latitude = int32(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.Longitude = int32(binary.LittleEndian.Uint32(p.Payload[4:]))
	m.Altitude = int32(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.X = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	m.Y = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	m.Z = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	for i := 0; i < len(m.Q); i++ {
		m.Q[i] = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24+i*4:]))
	}
	m.ApproachX = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[40:]))
	m.ApproachY = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[44:]))
	m.ApproachZ = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[48:]))
	return nil
}

// SetHomePosition struct (generated typeinfo)
// The position the system will return to and land on. The position is set automatically by the system during the takeoff in case it was not explicitely set by the operator before or after. The global and local positions encode the position in the respective coordinate frames, while the q parameter encodes the orientation of the surface. Under normal conditions it describes the heading and terrain slope, which can be used by the aircraft to adjust the approach. The approach 3D vector describes the point to which the system should fly in normal flight mode and then perform a landing sequence along the vector.
type SetHomePosition struct {
	Latitude     int32      // Latitude (WGS84), in degrees * 1E7
	Longitude    int32      // Longitude (WGS84, in degrees * 1E7
	Altitude     int32      // Altitude (AMSL), in meters * 1000 (positive for up)
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
func (m *SetHomePosition) MsgID() MessageID {
	return MSG_ID_SET_HOME_POSITION
}

// MsgName (generated function)
func (m *SetHomePosition) MsgName() string {
	return "SetHomePosition"
}

// Pack (generated function)
func (m *SetHomePosition) Pack(p *Packet) error {
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
func (m *SetHomePosition) Unpack(p *Packet) error {
	if len(p.Payload) < 53 {
		return fmt.Errorf("payload too small")
	}
	m.Latitude = int32(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.Longitude = int32(binary.LittleEndian.Uint32(p.Payload[4:]))
	m.Altitude = int32(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.X = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	m.Y = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	m.Z = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	for i := 0; i < len(m.Q); i++ {
		m.Q[i] = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24+i*4:]))
	}
	m.ApproachX = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[40:]))
	m.ApproachY = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[44:]))
	m.ApproachZ = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[48:]))
	m.TargetSystem = uint8(p.Payload[52])
	return nil
}

// MessageInterval struct (generated typeinfo)
// This interface replaces DATA_STREAM
type MessageInterval struct {
	IntervalUs int32  // The interval between two messages, in microseconds. A value of -1 indicates this stream is disabled, 0 indicates it is not available, &gt; 0 indicates the interval at which it is sent.
	MessageID  uint16 // The ID of the requested MAVLink message. v1.0 is limited to 254 messages.
}

// MsgID (generated function)
func (m *MessageInterval) MsgID() MessageID {
	return MSG_ID_MESSAGE_INTERVAL
}

// MsgName (generated function)
func (m *MessageInterval) MsgName() string {
	return "MessageInterval"
}

// Pack (generated function)
func (m *MessageInterval) Pack(p *Packet) error {
	payload := make([]byte, 6)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.IntervalUs))
	binary.LittleEndian.PutUint16(payload[4:], uint16(m.MessageID))

	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *MessageInterval) Unpack(p *Packet) error {
	if len(p.Payload) < 6 {
		return fmt.Errorf("payload too small")
	}
	m.IntervalUs = int32(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.MessageID = uint16(binary.LittleEndian.Uint16(p.Payload[4:]))
	return nil
}

// ExtendedSysState struct (generated typeinfo)
// Provides state for additional features
type ExtendedSysState struct {
	VtolState   uint8 // The VTOL state if applicable. Is set to MAV_VTOL_STATE_UNDEFINED if UAV is not in VTOL configuration.
	LandedState uint8 // The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.
}

// MsgID (generated function)
func (m *ExtendedSysState) MsgID() MessageID {
	return MSG_ID_EXTENDED_SYS_STATE
}

// MsgName (generated function)
func (m *ExtendedSysState) MsgName() string {
	return "ExtendedSysState"
}

// Pack (generated function)
func (m *ExtendedSysState) Pack(p *Packet) error {
	payload := make([]byte, 2)
	payload[0] = byte(m.VtolState)
	payload[1] = byte(m.LandedState)

	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ExtendedSysState) Unpack(p *Packet) error {
	if len(p.Payload) < 2 {
		return fmt.Errorf("payload too small")
	}
	m.VtolState = uint8(p.Payload[0])
	m.LandedState = uint8(p.Payload[1])
	return nil
}

// AdsbVehicle struct (generated typeinfo)
// The location and information of an ADSB vehicle
type AdsbVehicle struct {
	IcaoAddress  uint32  // ICAO address
	Lat          int32   // Latitude, expressed as degrees * 1E7
	Lon          int32   // Longitude, expressed as degrees * 1E7
	Altitude     int32   // Altitude(ASL) in millimeters
	Heading      uint16  // Course over ground in centidegrees
	HorVelocity  uint16  // The horizontal velocity in centimeters/second
	VerVelocity  int16   // The vertical velocity in centimeters/second, positive is up
	Flags        uint16  // Flags to indicate various statuses including valid data fields
	Squawk       uint16  // Squawk code
	AltitudeType uint8   // Type from ADSB_ALTITUDE_TYPE enum
	Callsign     [9]byte // The callsign, 8+null
	EmitterType  uint8   // Type from ADSB_EMITTER_TYPE enum
	Tslc         uint8   // Time since last communication in seconds
}

// MsgID (generated function)
func (m *AdsbVehicle) MsgID() MessageID {
	return MSG_ID_ADSB_VEHICLE
}

// MsgName (generated function)
func (m *AdsbVehicle) MsgName() string {
	return "AdsbVehicle"
}

// Pack (generated function)
func (m *AdsbVehicle) Pack(p *Packet) error {
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
func (m *AdsbVehicle) Unpack(p *Packet) error {
	if len(p.Payload) < 38 {
		return fmt.Errorf("payload too small")
	}
	m.IcaoAddress = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.Lat = int32(binary.LittleEndian.Uint32(p.Payload[4:]))
	m.Lon = int32(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.Altitude = int32(binary.LittleEndian.Uint32(p.Payload[12:]))
	m.Heading = uint16(binary.LittleEndian.Uint16(p.Payload[16:]))
	m.HorVelocity = uint16(binary.LittleEndian.Uint16(p.Payload[18:]))
	m.VerVelocity = int16(binary.LittleEndian.Uint16(p.Payload[20:]))
	m.Flags = uint16(binary.LittleEndian.Uint16(p.Payload[22:]))
	m.Squawk = uint16(binary.LittleEndian.Uint16(p.Payload[24:]))
	m.AltitudeType = uint8(p.Payload[26])
	copy(m.Callsign[:], p.Payload[27:36])
	m.EmitterType = uint8(p.Payload[36])
	m.Tslc = uint8(p.Payload[37])
	return nil
}

// Collision struct (generated typeinfo)
// Information about a potential collision
type Collision struct {
	ID                     uint32  // Unique identifier, domain based on src field
	TimeToMinimumDelta     float32 // Estimated time until collision occurs (seconds)
	AltitudeMinimumDelta   float32 // Closest vertical distance in meters between vehicle and object
	HorizontalMinimumDelta float32 // Closest horizontal distance in meteres between vehicle and object
	Src                    uint8   // Collision data source
	Action                 uint8   // Action that is being taken to avoid this collision
	ThreatLevel            uint8   // How concerned the aircraft is about this collision
}

// MsgID (generated function)
func (m *Collision) MsgID() MessageID {
	return MSG_ID_COLLISION
}

// MsgName (generated function)
func (m *Collision) MsgName() string {
	return "Collision"
}

// Pack (generated function)
func (m *Collision) Pack(p *Packet) error {
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
func (m *Collision) Unpack(p *Packet) error {
	if len(p.Payload) < 19 {
		return fmt.Errorf("payload too small")
	}
	m.ID = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.TimeToMinimumDelta = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	m.AltitudeMinimumDelta = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.HorizontalMinimumDelta = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	m.Src = uint8(p.Payload[16])
	m.Action = uint8(p.Payload[17])
	m.ThreatLevel = uint8(p.Payload[18])
	return nil
}

// V2Extension struct (generated typeinfo)
// Message implementing parts of the V2 payload specs in V1 frames for transitional support.
type V2Extension struct {
	MessageType     uint16     // A code that identifies the software component that understands this message (analogous to usb device classes or mime type strings).  If this code is less than 32768, it is considered a 'registered' protocol extension and the corresponding entry should be added to https://github.com/mavlink/mavlink/extension-message-ids.xml.  Software creators can register blocks of message IDs as needed (useful for GCS specific metadata, etc...). Message_types greater than 32767 are considered local experiments and should not be checked in to any widely distributed codebase.
	TargetNetwork   uint8      // Network ID (0 for broadcast)
	TargetSystem    uint8      // System ID (0 for broadcast)
	TargetComponent uint8      // Component ID (0 for broadcast)
	Payload         [249]uint8 // Variable length payload. The length is defined by the remaining message length when subtracting the header and other fields.  The entire content of this block is opaque unless you understand any the encoding message_type.  The particular encoding used can be extension specific and might not always be documented as part of the mavlink specification.
}

// MsgID (generated function)
func (m *V2Extension) MsgID() MessageID {
	return MSG_ID_V2_EXTENSION
}

// MsgName (generated function)
func (m *V2Extension) MsgName() string {
	return "V2Extension"
}

// Pack (generated function)
func (m *V2Extension) Pack(p *Packet) error {
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
func (m *V2Extension) Unpack(p *Packet) error {
	if len(p.Payload) < 254 {
		return fmt.Errorf("payload too small")
	}
	m.MessageType = uint16(binary.LittleEndian.Uint16(p.Payload[0:]))
	m.TargetNetwork = uint8(p.Payload[2])
	m.TargetSystem = uint8(p.Payload[3])
	m.TargetComponent = uint8(p.Payload[4])
	copy(m.Payload[:], p.Payload[5:254])
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
func (m *MemoryVect) MsgID() MessageID {
	return MSG_ID_MEMORY_VECT
}

// MsgName (generated function)
func (m *MemoryVect) MsgName() string {
	return "MemoryVect"
}

// Pack (generated function)
func (m *MemoryVect) Pack(p *Packet) error {
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
func (m *MemoryVect) Unpack(p *Packet) error {
	if len(p.Payload) < 36 {
		return fmt.Errorf("payload too small")
	}
	m.Address = uint16(binary.LittleEndian.Uint16(p.Payload[0:]))
	m.Ver = uint8(p.Payload[2])
	m.Type = uint8(p.Payload[3])
	for i := 0; i < len(m.Value); i++ {
		m.Value[i] = int8(p.Payload[4+i*1])
	}
	return nil
}

// DebugVect struct (generated typeinfo)
//
type DebugVect struct {
	TimeUsec uint64   // Timestamp
	X        float32  // x
	Y        float32  // y
	Z        float32  // z
	Name     [10]byte // Name
}

// MsgID (generated function)
func (m *DebugVect) MsgID() MessageID {
	return MSG_ID_DEBUG_VECT
}

// MsgName (generated function)
func (m *DebugVect) MsgName() string {
	return "DebugVect"
}

// Pack (generated function)
func (m *DebugVect) Pack(p *Packet) error {
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
func (m *DebugVect) Unpack(p *Packet) error {
	if len(p.Payload) < 30 {
		return fmt.Errorf("payload too small")
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	m.X = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.Y = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	m.Z = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	copy(m.Name[:], p.Payload[20:30])
	return nil
}

// NamedValueFloat struct (generated typeinfo)
// Send a key-value pair as float. The use of this message is discouraged for normal packets, but a quite efficient way for testing new messages and getting experimental debug output.
type NamedValueFloat struct {
	TimeBootMs uint32   // Timestamp (milliseconds since system boot)
	Value      float32  // Floating point value
	Name       [10]byte // Name of the debug variable
}

// MsgID (generated function)
func (m *NamedValueFloat) MsgID() MessageID {
	return MSG_ID_NAMED_VALUE_FLOAT
}

// MsgName (generated function)
func (m *NamedValueFloat) MsgName() string {
	return "NamedValueFloat"
}

// Pack (generated function)
func (m *NamedValueFloat) Pack(p *Packet) error {
	payload := make([]byte, 18)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.TimeBootMs))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(m.Value))
	copy(payload[8:], m.Name[:])

	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *NamedValueFloat) Unpack(p *Packet) error {
	if len(p.Payload) < 18 {
		return fmt.Errorf("payload too small")
	}
	m.TimeBootMs = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.Value = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	copy(m.Name[:], p.Payload[8:18])
	return nil
}

// NamedValueInt struct (generated typeinfo)
// Send a key-value pair as integer. The use of this message is discouraged for normal packets, but a quite efficient way for testing new messages and getting experimental debug output.
type NamedValueInt struct {
	TimeBootMs uint32   // Timestamp (milliseconds since system boot)
	Value      int32    // Signed integer value
	Name       [10]byte // Name of the debug variable
}

// MsgID (generated function)
func (m *NamedValueInt) MsgID() MessageID {
	return MSG_ID_NAMED_VALUE_INT
}

// MsgName (generated function)
func (m *NamedValueInt) MsgName() string {
	return "NamedValueInt"
}

// Pack (generated function)
func (m *NamedValueInt) Pack(p *Packet) error {
	payload := make([]byte, 18)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.TimeBootMs))
	binary.LittleEndian.PutUint32(payload[4:], uint32(m.Value))
	copy(payload[8:], m.Name[:])

	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *NamedValueInt) Unpack(p *Packet) error {
	if len(p.Payload) < 18 {
		return fmt.Errorf("payload too small")
	}
	m.TimeBootMs = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.Value = int32(binary.LittleEndian.Uint32(p.Payload[4:]))
	copy(m.Name[:], p.Payload[8:18])
	return nil
}

// Statustext struct (generated typeinfo)
// Status text message. These messages are printed in yellow in the COMM console of QGroundControl. WARNING: They consume quite some bandwidth, so use only for important status and error messages. If implemented wisely, these messages are buffered on the MCU and sent only at a limited rate (e.g. 10 Hz).
type Statustext struct {
	Severity uint8    // Severity of status. Relies on the definitions within RFC-5424. See enum MAV_SEVERITY.
	Text     [50]byte // Status text message, without null termination character
}

// MsgID (generated function)
func (m *Statustext) MsgID() MessageID {
	return MSG_ID_STATUSTEXT
}

// MsgName (generated function)
func (m *Statustext) MsgName() string {
	return "Statustext"
}

// Pack (generated function)
func (m *Statustext) Pack(p *Packet) error {
	payload := make([]byte, 51)
	payload[0] = byte(m.Severity)
	copy(payload[1:], m.Text[:])

	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *Statustext) Unpack(p *Packet) error {
	if len(p.Payload) < 51 {
		return fmt.Errorf("payload too small")
	}
	m.Severity = uint8(p.Payload[0])
	copy(m.Text[:], p.Payload[1:51])
	return nil
}

// Debug struct (generated typeinfo)
// Send a debug value. The index is used to discriminate between values. These values show up in the plot of QGroundControl as DEBUG N.
type Debug struct {
	TimeBootMs uint32  // Timestamp (milliseconds since system boot)
	Value      float32 // DEBUG value
	Ind        uint8   // index of debug variable
}

// MsgID (generated function)
func (m *Debug) MsgID() MessageID {
	return MSG_ID_DEBUG
}

// MsgName (generated function)
func (m *Debug) MsgName() string {
	return "Debug"
}

// Pack (generated function)
func (m *Debug) Pack(p *Packet) error {
	payload := make([]byte, 9)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.TimeBootMs))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(m.Value))
	payload[8] = byte(m.Ind)

	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *Debug) Unpack(p *Packet) error {
	if len(p.Payload) < 9 {
		return fmt.Errorf("payload too small")
	}
	m.TimeBootMs = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.Value = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	m.Ind = uint8(p.Payload[8])
	return nil
}

// Message IDs
const (
	MSG_ID_HEARTBEAT                               MessageID = 0
	MSG_ID_SYS_STATUS                              MessageID = 1
	MSG_ID_SYSTEM_TIME                             MessageID = 2
	MSG_ID_PING                                    MessageID = 4
	MSG_ID_CHANGE_OPERATOR_CONTROL                 MessageID = 5
	MSG_ID_CHANGE_OPERATOR_CONTROL_ACK             MessageID = 6
	MSG_ID_AUTH_KEY                                MessageID = 7
	MSG_ID_SET_MODE                                MessageID = 11
	MSG_ID_PARAM_REQUEST_READ                      MessageID = 20
	MSG_ID_PARAM_REQUEST_LIST                      MessageID = 21
	MSG_ID_PARAM_VALUE                             MessageID = 22
	MSG_ID_PARAM_SET                               MessageID = 23
	MSG_ID_GPS_RAW_INT                             MessageID = 24
	MSG_ID_GPS_STATUS                              MessageID = 25
	MSG_ID_SCALED_IMU                              MessageID = 26
	MSG_ID_RAW_IMU                                 MessageID = 27
	MSG_ID_RAW_PRESSURE                            MessageID = 28
	MSG_ID_SCALED_PRESSURE                         MessageID = 29
	MSG_ID_ATTITUDE                                MessageID = 30
	MSG_ID_ATTITUDE_QUATERNION                     MessageID = 31
	MSG_ID_LOCAL_POSITION_NED                      MessageID = 32
	MSG_ID_GLOBAL_POSITION_INT                     MessageID = 33
	MSG_ID_RC_CHANNELS_SCALED                      MessageID = 34
	MSG_ID_RC_CHANNELS_RAW                         MessageID = 35
	MSG_ID_SERVO_OUTPUT_RAW                        MessageID = 36
	MSG_ID_MISSION_REQUEST_PARTIAL_LIST            MessageID = 37
	MSG_ID_MISSION_WRITE_PARTIAL_LIST              MessageID = 38
	MSG_ID_MISSION_ITEM                            MessageID = 39
	MSG_ID_MISSION_REQUEST                         MessageID = 40
	MSG_ID_MISSION_SET_CURRENT                     MessageID = 41
	MSG_ID_MISSION_CURRENT                         MessageID = 42
	MSG_ID_MISSION_REQUEST_LIST                    MessageID = 43
	MSG_ID_MISSION_COUNT                           MessageID = 44
	MSG_ID_MISSION_CLEAR_ALL                       MessageID = 45
	MSG_ID_MISSION_ITEM_REACHED                    MessageID = 46
	MSG_ID_MISSION_ACK                             MessageID = 47
	MSG_ID_SET_GPS_GLOBAL_ORIGIN                   MessageID = 48
	MSG_ID_GPS_GLOBAL_ORIGIN                       MessageID = 49
	MSG_ID_PARAM_MAP_RC                            MessageID = 50
	MSG_ID_MISSION_REQUEST_INT                     MessageID = 51
	MSG_ID_SAFETY_SET_ALLOWED_AREA                 MessageID = 54
	MSG_ID_SAFETY_ALLOWED_AREA                     MessageID = 55
	MSG_ID_ATTITUDE_QUATERNION_COV                 MessageID = 61
	MSG_ID_NAV_CONTROLLER_OUTPUT                   MessageID = 62
	MSG_ID_GLOBAL_POSITION_INT_COV                 MessageID = 63
	MSG_ID_LOCAL_POSITION_NED_COV                  MessageID = 64
	MSG_ID_RC_CHANNELS                             MessageID = 65
	MSG_ID_REQUEST_DATA_STREAM                     MessageID = 66
	MSG_ID_DATA_STREAM                             MessageID = 67
	MSG_ID_MANUAL_CONTROL                          MessageID = 69
	MSG_ID_RC_CHANNELS_OVERRIDE                    MessageID = 70
	MSG_ID_MISSION_ITEM_INT                        MessageID = 73
	MSG_ID_VFR_HUD                                 MessageID = 74
	MSG_ID_COMMAND_INT                             MessageID = 75
	MSG_ID_COMMAND_LONG                            MessageID = 76
	MSG_ID_COMMAND_ACK                             MessageID = 77
	MSG_ID_MANUAL_SETPOINT                         MessageID = 81
	MSG_ID_SET_ATTITUDE_TARGET                     MessageID = 82
	MSG_ID_ATTITUDE_TARGET                         MessageID = 83
	MSG_ID_SET_POSITION_TARGET_LOCAL_NED           MessageID = 84
	MSG_ID_POSITION_TARGET_LOCAL_NED               MessageID = 85
	MSG_ID_SET_POSITION_TARGET_GLOBAL_INT          MessageID = 86
	MSG_ID_POSITION_TARGET_GLOBAL_INT              MessageID = 87
	MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET MessageID = 89
	MSG_ID_HIL_STATE                               MessageID = 90
	MSG_ID_HIL_CONTROLS                            MessageID = 91
	MSG_ID_HIL_RC_INPUTS_RAW                       MessageID = 92
	MSG_ID_HIL_ACTUATOR_CONTROLS                   MessageID = 93
	MSG_ID_OPTICAL_FLOW                            MessageID = 100
	MSG_ID_GLOBAL_VISION_POSITION_ESTIMATE         MessageID = 101
	MSG_ID_VISION_POSITION_ESTIMATE                MessageID = 102
	MSG_ID_VISION_SPEED_ESTIMATE                   MessageID = 103
	MSG_ID_VICON_POSITION_ESTIMATE                 MessageID = 104
	MSG_ID_HIGHRES_IMU                             MessageID = 105
	MSG_ID_OPTICAL_FLOW_RAD                        MessageID = 106
	MSG_ID_HIL_SENSOR                              MessageID = 107
	MSG_ID_SIM_STATE                               MessageID = 108
	MSG_ID_RADIO_STATUS                            MessageID = 109
	MSG_ID_FILE_TRANSFER_PROTOCOL                  MessageID = 110
	MSG_ID_TIMESYNC                                MessageID = 111
	MSG_ID_CAMERA_TRIGGER                          MessageID = 112
	MSG_ID_HIL_GPS                                 MessageID = 113
	MSG_ID_HIL_OPTICAL_FLOW                        MessageID = 114
	MSG_ID_HIL_STATE_QUATERNION                    MessageID = 115
	MSG_ID_SCALED_IMU2                             MessageID = 116
	MSG_ID_LOG_REQUEST_LIST                        MessageID = 117
	MSG_ID_LOG_ENTRY                               MessageID = 118
	MSG_ID_LOG_REQUEST_DATA                        MessageID = 119
	MSG_ID_LOG_DATA                                MessageID = 120
	MSG_ID_LOG_ERASE                               MessageID = 121
	MSG_ID_LOG_REQUEST_END                         MessageID = 122
	MSG_ID_GPS_INJECT_DATA                         MessageID = 123
	MSG_ID_GPS2_RAW                                MessageID = 124
	MSG_ID_POWER_STATUS                            MessageID = 125
	MSG_ID_SERIAL_CONTROL                          MessageID = 126
	MSG_ID_GPS_RTK                                 MessageID = 127
	MSG_ID_GPS2_RTK                                MessageID = 128
	MSG_ID_SCALED_IMU3                             MessageID = 129
	MSG_ID_DATA_TRANSMISSION_HANDSHAKE             MessageID = 130
	MSG_ID_ENCAPSULATED_DATA                       MessageID = 131
	MSG_ID_DISTANCE_SENSOR                         MessageID = 132
	MSG_ID_TERRAIN_REQUEST                         MessageID = 133
	MSG_ID_TERRAIN_DATA                            MessageID = 134
	MSG_ID_TERRAIN_CHECK                           MessageID = 135
	MSG_ID_TERRAIN_REPORT                          MessageID = 136
	MSG_ID_SCALED_PRESSURE2                        MessageID = 137
	MSG_ID_ATT_POS_MOCAP                           MessageID = 138
	MSG_ID_SET_ACTUATOR_CONTROL_TARGET             MessageID = 139
	MSG_ID_ACTUATOR_CONTROL_TARGET                 MessageID = 140
	MSG_ID_ALTITUDE                                MessageID = 141
	MSG_ID_RESOURCE_REQUEST                        MessageID = 142
	MSG_ID_SCALED_PRESSURE3                        MessageID = 143
	MSG_ID_FOLLOW_TARGET                           MessageID = 144
	MSG_ID_CONTROL_SYSTEM_STATE                    MessageID = 146
	MSG_ID_BATTERY_STATUS                          MessageID = 147
	MSG_ID_AUTOPILOT_VERSION                       MessageID = 148
	MSG_ID_LANDING_TARGET                          MessageID = 149
	MSG_ID_ESTIMATOR_STATUS                        MessageID = 230
	MSG_ID_WIND_COV                                MessageID = 231
	MSG_ID_GPS_INPUT                               MessageID = 232
	MSG_ID_GPS_RTCM_DATA                           MessageID = 233
	MSG_ID_HIGH_LATENCY                            MessageID = 234
	MSG_ID_VIBRATION                               MessageID = 241
	MSG_ID_HOME_POSITION                           MessageID = 242
	MSG_ID_SET_HOME_POSITION                       MessageID = 243
	MSG_ID_MESSAGE_INTERVAL                        MessageID = 244
	MSG_ID_EXTENDED_SYS_STATE                      MessageID = 245
	MSG_ID_ADSB_VEHICLE                            MessageID = 246
	MSG_ID_COLLISION                               MessageID = 247
	MSG_ID_V2_EXTENSION                            MessageID = 248
	MSG_ID_MEMORY_VECT                             MessageID = 249
	MSG_ID_DEBUG_VECT                              MessageID = 250
	MSG_ID_NAMED_VALUE_FLOAT                       MessageID = 251
	MSG_ID_NAMED_VALUE_INT                         MessageID = 252
	MSG_ID_STATUSTEXT                              MessageID = 253
	MSG_ID_DEBUG                                   MessageID = 254
)

// DialectCommon is the dialect represented by common.xml
var DialectCommon = &Dialect{
	Name: "common",
	crcExtras: map[MessageID]uint8{
		MSG_ID_HEARTBEAT:                               50,
		MSG_ID_SYS_STATUS:                              124,
		MSG_ID_SYSTEM_TIME:                             137,
		MSG_ID_PING:                                    237,
		MSG_ID_CHANGE_OPERATOR_CONTROL:                 217,
		MSG_ID_CHANGE_OPERATOR_CONTROL_ACK:             104,
		MSG_ID_AUTH_KEY:                                119,
		MSG_ID_SET_MODE:                                89,
		MSG_ID_PARAM_REQUEST_READ:                      214,
		MSG_ID_PARAM_REQUEST_LIST:                      159,
		MSG_ID_PARAM_VALUE:                             220,
		MSG_ID_PARAM_SET:                               168,
		MSG_ID_GPS_RAW_INT:                             24,
		MSG_ID_GPS_STATUS:                              23,
		MSG_ID_SCALED_IMU:                              170,
		MSG_ID_RAW_IMU:                                 144,
		MSG_ID_RAW_PRESSURE:                            67,
		MSG_ID_SCALED_PRESSURE:                         115,
		MSG_ID_ATTITUDE:                                39,
		MSG_ID_ATTITUDE_QUATERNION:                     246,
		MSG_ID_LOCAL_POSITION_NED:                      185,
		MSG_ID_GLOBAL_POSITION_INT:                     104,
		MSG_ID_RC_CHANNELS_SCALED:                      237,
		MSG_ID_RC_CHANNELS_RAW:                         244,
		MSG_ID_SERVO_OUTPUT_RAW:                        222,
		MSG_ID_MISSION_REQUEST_PARTIAL_LIST:            212,
		MSG_ID_MISSION_WRITE_PARTIAL_LIST:              9,
		MSG_ID_MISSION_ITEM:                            254,
		MSG_ID_MISSION_REQUEST:                         230,
		MSG_ID_MISSION_SET_CURRENT:                     28,
		MSG_ID_MISSION_CURRENT:                         28,
		MSG_ID_MISSION_REQUEST_LIST:                    132,
		MSG_ID_MISSION_COUNT:                           221,
		MSG_ID_MISSION_CLEAR_ALL:                       232,
		MSG_ID_MISSION_ITEM_REACHED:                    11,
		MSG_ID_MISSION_ACK:                             153,
		MSG_ID_SET_GPS_GLOBAL_ORIGIN:                   41,
		MSG_ID_GPS_GLOBAL_ORIGIN:                       39,
		MSG_ID_PARAM_MAP_RC:                            78,
		MSG_ID_MISSION_REQUEST_INT:                     196,
		MSG_ID_SAFETY_SET_ALLOWED_AREA:                 15,
		MSG_ID_SAFETY_ALLOWED_AREA:                     3,
		MSG_ID_ATTITUDE_QUATERNION_COV:                 167,
		MSG_ID_NAV_CONTROLLER_OUTPUT:                   183,
		MSG_ID_GLOBAL_POSITION_INT_COV:                 119,
		MSG_ID_LOCAL_POSITION_NED_COV:                  191,
		MSG_ID_RC_CHANNELS:                             118,
		MSG_ID_REQUEST_DATA_STREAM:                     148,
		MSG_ID_DATA_STREAM:                             21,
		MSG_ID_MANUAL_CONTROL:                          243,
		MSG_ID_RC_CHANNELS_OVERRIDE:                    124,
		MSG_ID_MISSION_ITEM_INT:                        38,
		MSG_ID_VFR_HUD:                                 20,
		MSG_ID_COMMAND_INT:                             158,
		MSG_ID_COMMAND_LONG:                            152,
		MSG_ID_COMMAND_ACK:                             143,
		MSG_ID_MANUAL_SETPOINT:                         106,
		MSG_ID_SET_ATTITUDE_TARGET:                     49,
		MSG_ID_ATTITUDE_TARGET:                         22,
		MSG_ID_SET_POSITION_TARGET_LOCAL_NED:           143,
		MSG_ID_POSITION_TARGET_LOCAL_NED:               140,
		MSG_ID_SET_POSITION_TARGET_GLOBAL_INT:          5,
		MSG_ID_POSITION_TARGET_GLOBAL_INT:              150,
		MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET: 231,
		MSG_ID_HIL_STATE:                               183,
		MSG_ID_HIL_CONTROLS:                            63,
		MSG_ID_HIL_RC_INPUTS_RAW:                       54,
		MSG_ID_HIL_ACTUATOR_CONTROLS:                   47,
		MSG_ID_OPTICAL_FLOW:                            175,
		MSG_ID_GLOBAL_VISION_POSITION_ESTIMATE:         102,
		MSG_ID_VISION_POSITION_ESTIMATE:                158,
		MSG_ID_VISION_SPEED_ESTIMATE:                   208,
		MSG_ID_VICON_POSITION_ESTIMATE:                 56,
		MSG_ID_HIGHRES_IMU:                             93,
		MSG_ID_OPTICAL_FLOW_RAD:                        138,
		MSG_ID_HIL_SENSOR:                              108,
		MSG_ID_SIM_STATE:                               32,
		MSG_ID_RADIO_STATUS:                            185,
		MSG_ID_FILE_TRANSFER_PROTOCOL:                  84,
		MSG_ID_TIMESYNC:                                34,
		MSG_ID_CAMERA_TRIGGER:                          174,
		MSG_ID_HIL_GPS:                                 124,
		MSG_ID_HIL_OPTICAL_FLOW:                        237,
		MSG_ID_HIL_STATE_QUATERNION:                    4,
		MSG_ID_SCALED_IMU2:                             76,
		MSG_ID_LOG_REQUEST_LIST:                        128,
		MSG_ID_LOG_ENTRY:                               56,
		MSG_ID_LOG_REQUEST_DATA:                        116,
		MSG_ID_LOG_DATA:                                134,
		MSG_ID_LOG_ERASE:                               237,
		MSG_ID_LOG_REQUEST_END:                         203,
		MSG_ID_GPS_INJECT_DATA:                         250,
		MSG_ID_GPS2_RAW:                                87,
		MSG_ID_POWER_STATUS:                            203,
		MSG_ID_SERIAL_CONTROL:                          220,
		MSG_ID_GPS_RTK:                                 25,
		MSG_ID_GPS2_RTK:                                226,
		MSG_ID_SCALED_IMU3:                             46,
		MSG_ID_DATA_TRANSMISSION_HANDSHAKE:             29,
		MSG_ID_ENCAPSULATED_DATA:                       223,
		MSG_ID_DISTANCE_SENSOR:                         85,
		MSG_ID_TERRAIN_REQUEST:                         6,
		MSG_ID_TERRAIN_DATA:                            229,
		MSG_ID_TERRAIN_CHECK:                           203,
		MSG_ID_TERRAIN_REPORT:                          1,
		MSG_ID_SCALED_PRESSURE2:                        195,
		MSG_ID_ATT_POS_MOCAP:                           109,
		MSG_ID_SET_ACTUATOR_CONTROL_TARGET:             168,
		MSG_ID_ACTUATOR_CONTROL_TARGET:                 181,
		MSG_ID_ALTITUDE:                                47,
		MSG_ID_RESOURCE_REQUEST:                        72,
		MSG_ID_SCALED_PRESSURE3:                        131,
		MSG_ID_FOLLOW_TARGET:                           127,
		MSG_ID_CONTROL_SYSTEM_STATE:                    103,
		MSG_ID_BATTERY_STATUS:                          154,
		MSG_ID_AUTOPILOT_VERSION:                       178,
		MSG_ID_LANDING_TARGET:                          200,
		MSG_ID_ESTIMATOR_STATUS:                        163,
		MSG_ID_WIND_COV:                                105,
		MSG_ID_GPS_INPUT:                               151,
		MSG_ID_GPS_RTCM_DATA:                           35,
		MSG_ID_HIGH_LATENCY:                            150,
		MSG_ID_VIBRATION:                               90,
		MSG_ID_HOME_POSITION:                           104,
		MSG_ID_SET_HOME_POSITION:                       85,
		MSG_ID_MESSAGE_INTERVAL:                        95,
		MSG_ID_EXTENDED_SYS_STATE:                      130,
		MSG_ID_ADSB_VEHICLE:                            184,
		MSG_ID_COLLISION:                               81,
		MSG_ID_V2_EXTENSION:                            8,
		MSG_ID_MEMORY_VECT:                             204,
		MSG_ID_DEBUG_VECT:                              49,
		MSG_ID_NAMED_VALUE_FLOAT:                       170,
		MSG_ID_NAMED_VALUE_INT:                         44,
		MSG_ID_STATUSTEXT:                              83,
		MSG_ID_DEBUG:                                   46,
	},
	messageConstructorByMsgID: map[MessageID]func(*Packet) Message{
		MSG_ID_HEARTBEAT: func(pkt *Packet) Message {
			msg := new(Heartbeat)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SYS_STATUS: func(pkt *Packet) Message {
			msg := new(SysStatus)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SYSTEM_TIME: func(pkt *Packet) Message {
			msg := new(SystemTime)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_PING: func(pkt *Packet) Message {
			msg := new(Ping)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_CHANGE_OPERATOR_CONTROL: func(pkt *Packet) Message {
			msg := new(ChangeOperatorControl)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_CHANGE_OPERATOR_CONTROL_ACK: func(pkt *Packet) Message {
			msg := new(ChangeOperatorControlAck)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_AUTH_KEY: func(pkt *Packet) Message {
			msg := new(AuthKey)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SET_MODE: func(pkt *Packet) Message {
			msg := new(SetMode)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_PARAM_REQUEST_READ: func(pkt *Packet) Message {
			msg := new(ParamRequestRead)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_PARAM_REQUEST_LIST: func(pkt *Packet) Message {
			msg := new(ParamRequestList)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_PARAM_VALUE: func(pkt *Packet) Message {
			msg := new(ParamValue)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_PARAM_SET: func(pkt *Packet) Message {
			msg := new(ParamSet)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_GPS_RAW_INT: func(pkt *Packet) Message {
			msg := new(GpsRawInt)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_GPS_STATUS: func(pkt *Packet) Message {
			msg := new(GpsStatus)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SCALED_IMU: func(pkt *Packet) Message {
			msg := new(ScaledImu)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_RAW_IMU: func(pkt *Packet) Message {
			msg := new(RawImu)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_RAW_PRESSURE: func(pkt *Packet) Message {
			msg := new(RawPressure)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SCALED_PRESSURE: func(pkt *Packet) Message {
			msg := new(ScaledPressure)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_ATTITUDE: func(pkt *Packet) Message {
			msg := new(Attitude)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_ATTITUDE_QUATERNION: func(pkt *Packet) Message {
			msg := new(AttitudeQuaternion)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_LOCAL_POSITION_NED: func(pkt *Packet) Message {
			msg := new(LocalPositionNed)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_GLOBAL_POSITION_INT: func(pkt *Packet) Message {
			msg := new(GlobalPositionInt)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_RC_CHANNELS_SCALED: func(pkt *Packet) Message {
			msg := new(RcChannelsScaled)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_RC_CHANNELS_RAW: func(pkt *Packet) Message {
			msg := new(RcChannelsRaw)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SERVO_OUTPUT_RAW: func(pkt *Packet) Message {
			msg := new(ServoOutputRaw)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_MISSION_REQUEST_PARTIAL_LIST: func(pkt *Packet) Message {
			msg := new(MissionRequestPartialList)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_MISSION_WRITE_PARTIAL_LIST: func(pkt *Packet) Message {
			msg := new(MissionWritePartialList)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_MISSION_ITEM: func(pkt *Packet) Message {
			msg := new(MissionItem)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_MISSION_REQUEST: func(pkt *Packet) Message {
			msg := new(MissionRequest)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_MISSION_SET_CURRENT: func(pkt *Packet) Message {
			msg := new(MissionSetCurrent)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_MISSION_CURRENT: func(pkt *Packet) Message {
			msg := new(MissionCurrent)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_MISSION_REQUEST_LIST: func(pkt *Packet) Message {
			msg := new(MissionRequestList)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_MISSION_COUNT: func(pkt *Packet) Message {
			msg := new(MissionCount)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_MISSION_CLEAR_ALL: func(pkt *Packet) Message {
			msg := new(MissionClearAll)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_MISSION_ITEM_REACHED: func(pkt *Packet) Message {
			msg := new(MissionItemReached)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_MISSION_ACK: func(pkt *Packet) Message {
			msg := new(MissionAck)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SET_GPS_GLOBAL_ORIGIN: func(pkt *Packet) Message {
			msg := new(SetGpsGlobalOrigin)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_GPS_GLOBAL_ORIGIN: func(pkt *Packet) Message {
			msg := new(GpsGlobalOrigin)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_PARAM_MAP_RC: func(pkt *Packet) Message {
			msg := new(ParamMapRc)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_MISSION_REQUEST_INT: func(pkt *Packet) Message {
			msg := new(MissionRequestInt)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SAFETY_SET_ALLOWED_AREA: func(pkt *Packet) Message {
			msg := new(SafetySetAllowedArea)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SAFETY_ALLOWED_AREA: func(pkt *Packet) Message {
			msg := new(SafetyAllowedArea)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_ATTITUDE_QUATERNION_COV: func(pkt *Packet) Message {
			msg := new(AttitudeQuaternionCov)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_NAV_CONTROLLER_OUTPUT: func(pkt *Packet) Message {
			msg := new(NavControllerOutput)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_GLOBAL_POSITION_INT_COV: func(pkt *Packet) Message {
			msg := new(GlobalPositionIntCov)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_LOCAL_POSITION_NED_COV: func(pkt *Packet) Message {
			msg := new(LocalPositionNedCov)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_RC_CHANNELS: func(pkt *Packet) Message {
			msg := new(RcChannels)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_REQUEST_DATA_STREAM: func(pkt *Packet) Message {
			msg := new(RequestDataStream)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_DATA_STREAM: func(pkt *Packet) Message {
			msg := new(DataStream)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_MANUAL_CONTROL: func(pkt *Packet) Message {
			msg := new(ManualControl)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_RC_CHANNELS_OVERRIDE: func(pkt *Packet) Message {
			msg := new(RcChannelsOverride)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_MISSION_ITEM_INT: func(pkt *Packet) Message {
			msg := new(MissionItemInt)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_VFR_HUD: func(pkt *Packet) Message {
			msg := new(VfrHud)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_COMMAND_INT: func(pkt *Packet) Message {
			msg := new(CommandInt)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_COMMAND_LONG: func(pkt *Packet) Message {
			msg := new(CommandLong)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_COMMAND_ACK: func(pkt *Packet) Message {
			msg := new(CommandAck)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_MANUAL_SETPOINT: func(pkt *Packet) Message {
			msg := new(ManualSetpoint)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SET_ATTITUDE_TARGET: func(pkt *Packet) Message {
			msg := new(SetAttitudeTarget)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_ATTITUDE_TARGET: func(pkt *Packet) Message {
			msg := new(AttitudeTarget)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SET_POSITION_TARGET_LOCAL_NED: func(pkt *Packet) Message {
			msg := new(SetPositionTargetLocalNed)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_POSITION_TARGET_LOCAL_NED: func(pkt *Packet) Message {
			msg := new(PositionTargetLocalNed)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SET_POSITION_TARGET_GLOBAL_INT: func(pkt *Packet) Message {
			msg := new(SetPositionTargetGlobalInt)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_POSITION_TARGET_GLOBAL_INT: func(pkt *Packet) Message {
			msg := new(PositionTargetGlobalInt)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET: func(pkt *Packet) Message {
			msg := new(LocalPositionNedSystemGlobalOffset)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_HIL_STATE: func(pkt *Packet) Message {
			msg := new(HilState)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_HIL_CONTROLS: func(pkt *Packet) Message {
			msg := new(HilControls)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_HIL_RC_INPUTS_RAW: func(pkt *Packet) Message {
			msg := new(HilRcInputsRaw)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_HIL_ACTUATOR_CONTROLS: func(pkt *Packet) Message {
			msg := new(HilActuatorControls)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_OPTICAL_FLOW: func(pkt *Packet) Message {
			msg := new(OpticalFlow)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_GLOBAL_VISION_POSITION_ESTIMATE: func(pkt *Packet) Message {
			msg := new(GlobalVisionPositionEstimate)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_VISION_POSITION_ESTIMATE: func(pkt *Packet) Message {
			msg := new(VisionPositionEstimate)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_VISION_SPEED_ESTIMATE: func(pkt *Packet) Message {
			msg := new(VisionSpeedEstimate)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_VICON_POSITION_ESTIMATE: func(pkt *Packet) Message {
			msg := new(ViconPositionEstimate)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_HIGHRES_IMU: func(pkt *Packet) Message {
			msg := new(HighresImu)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_OPTICAL_FLOW_RAD: func(pkt *Packet) Message {
			msg := new(OpticalFlowRad)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_HIL_SENSOR: func(pkt *Packet) Message {
			msg := new(HilSensor)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SIM_STATE: func(pkt *Packet) Message {
			msg := new(SimState)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_RADIO_STATUS: func(pkt *Packet) Message {
			msg := new(RadioStatus)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_FILE_TRANSFER_PROTOCOL: func(pkt *Packet) Message {
			msg := new(FileTransferProtocol)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_TIMESYNC: func(pkt *Packet) Message {
			msg := new(Timesync)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_CAMERA_TRIGGER: func(pkt *Packet) Message {
			msg := new(CameraTrigger)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_HIL_GPS: func(pkt *Packet) Message {
			msg := new(HilGps)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_HIL_OPTICAL_FLOW: func(pkt *Packet) Message {
			msg := new(HilOpticalFlow)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_HIL_STATE_QUATERNION: func(pkt *Packet) Message {
			msg := new(HilStateQuaternion)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SCALED_IMU2: func(pkt *Packet) Message {
			msg := new(ScaledImu2)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_LOG_REQUEST_LIST: func(pkt *Packet) Message {
			msg := new(LogRequestList)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_LOG_ENTRY: func(pkt *Packet) Message {
			msg := new(LogEntry)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_LOG_REQUEST_DATA: func(pkt *Packet) Message {
			msg := new(LogRequestData)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_LOG_DATA: func(pkt *Packet) Message {
			msg := new(LogData)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_LOG_ERASE: func(pkt *Packet) Message {
			msg := new(LogErase)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_LOG_REQUEST_END: func(pkt *Packet) Message {
			msg := new(LogRequestEnd)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_GPS_INJECT_DATA: func(pkt *Packet) Message {
			msg := new(GpsInjectData)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_GPS2_RAW: func(pkt *Packet) Message {
			msg := new(Gps2Raw)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_POWER_STATUS: func(pkt *Packet) Message {
			msg := new(PowerStatus)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SERIAL_CONTROL: func(pkt *Packet) Message {
			msg := new(SerialControl)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_GPS_RTK: func(pkt *Packet) Message {
			msg := new(GpsRtk)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_GPS2_RTK: func(pkt *Packet) Message {
			msg := new(Gps2Rtk)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SCALED_IMU3: func(pkt *Packet) Message {
			msg := new(ScaledImu3)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_DATA_TRANSMISSION_HANDSHAKE: func(pkt *Packet) Message {
			msg := new(DataTransmissionHandshake)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_ENCAPSULATED_DATA: func(pkt *Packet) Message {
			msg := new(EncapsulatedData)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_DISTANCE_SENSOR: func(pkt *Packet) Message {
			msg := new(DistanceSensor)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_TERRAIN_REQUEST: func(pkt *Packet) Message {
			msg := new(TerrainRequest)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_TERRAIN_DATA: func(pkt *Packet) Message {
			msg := new(TerrainData)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_TERRAIN_CHECK: func(pkt *Packet) Message {
			msg := new(TerrainCheck)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_TERRAIN_REPORT: func(pkt *Packet) Message {
			msg := new(TerrainReport)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SCALED_PRESSURE2: func(pkt *Packet) Message {
			msg := new(ScaledPressure2)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_ATT_POS_MOCAP: func(pkt *Packet) Message {
			msg := new(AttPosMocap)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SET_ACTUATOR_CONTROL_TARGET: func(pkt *Packet) Message {
			msg := new(SetActuatorControlTarget)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_ACTUATOR_CONTROL_TARGET: func(pkt *Packet) Message {
			msg := new(ActuatorControlTarget)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_ALTITUDE: func(pkt *Packet) Message {
			msg := new(Altitude)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_RESOURCE_REQUEST: func(pkt *Packet) Message {
			msg := new(ResourceRequest)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SCALED_PRESSURE3: func(pkt *Packet) Message {
			msg := new(ScaledPressure3)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_FOLLOW_TARGET: func(pkt *Packet) Message {
			msg := new(FollowTarget)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_CONTROL_SYSTEM_STATE: func(pkt *Packet) Message {
			msg := new(ControlSystemState)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_BATTERY_STATUS: func(pkt *Packet) Message {
			msg := new(BatteryStatus)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_AUTOPILOT_VERSION: func(pkt *Packet) Message {
			msg := new(AutopilotVersion)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_LANDING_TARGET: func(pkt *Packet) Message {
			msg := new(LandingTarget)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_ESTIMATOR_STATUS: func(pkt *Packet) Message {
			msg := new(EstimatorStatus)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_WIND_COV: func(pkt *Packet) Message {
			msg := new(WindCov)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_GPS_INPUT: func(pkt *Packet) Message {
			msg := new(GpsInput)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_GPS_RTCM_DATA: func(pkt *Packet) Message {
			msg := new(GpsRtcmData)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_HIGH_LATENCY: func(pkt *Packet) Message {
			msg := new(HighLatency)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_VIBRATION: func(pkt *Packet) Message {
			msg := new(Vibration)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_HOME_POSITION: func(pkt *Packet) Message {
			msg := new(HomePosition)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SET_HOME_POSITION: func(pkt *Packet) Message {
			msg := new(SetHomePosition)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_MESSAGE_INTERVAL: func(pkt *Packet) Message {
			msg := new(MessageInterval)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_EXTENDED_SYS_STATE: func(pkt *Packet) Message {
			msg := new(ExtendedSysState)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_ADSB_VEHICLE: func(pkt *Packet) Message {
			msg := new(AdsbVehicle)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_COLLISION: func(pkt *Packet) Message {
			msg := new(Collision)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_V2_EXTENSION: func(pkt *Packet) Message {
			msg := new(V2Extension)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_MEMORY_VECT: func(pkt *Packet) Message {
			msg := new(MemoryVect)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_DEBUG_VECT: func(pkt *Packet) Message {
			msg := new(DebugVect)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_NAMED_VALUE_FLOAT: func(pkt *Packet) Message {
			msg := new(NamedValueFloat)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_NAMED_VALUE_INT: func(pkt *Packet) Message {
			msg := new(NamedValueInt)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_STATUSTEXT: func(pkt *Packet) Message {
			msg := new(Statustext)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_DEBUG: func(pkt *Packet) Message {
			msg := new(Debug)
			msg.Unpack(pkt)
			return msg
		},
	},
}

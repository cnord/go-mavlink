//////////////////////////////////////////////////
//
// NOTE: do not edit,
// this file created automatically by mavgen.go
//
//////////////////////////////////////////////////

package ardupilotmega

import (
	"encoding/binary"
	"fmt"
	mavlink "github.com/asmyasnikov/go-mavlink/generated/mavlink1"
	"math"
)

// AccelcalVehiclePos (generated enum)
//
const (
	ACCELCAL_VEHICLE_POS_LEVEL    = 1        //
	ACCELCAL_VEHICLE_POS_LEFT     = 2        //
	ACCELCAL_VEHICLE_POS_RIGHT    = 3        //
	ACCELCAL_VEHICLE_POS_NOSEDOWN = 4        //
	ACCELCAL_VEHICLE_POS_NOSEUP   = 5        //
	ACCELCAL_VEHICLE_POS_BACK     = 6        //
	ACCELCAL_VEHICLE_POS_SUCCESS  = 16777215 //
	ACCELCAL_VEHICLE_POS_FAILED   = 16777216 //
)

// HeadingType (generated enum)
//
const (
	HEADING_TYPE_COURSE_OVER_GROUND = 0 //
	HEADING_TYPE_HEADING            = 1 //
)

// SpeedType (generated enum)
//
const (
	SPEED_TYPE_AIRSPEED    = 0 //
	SPEED_TYPE_GROUNDSPEED = 1 //
)

// MavCmd (generated enum)
//
const (
	MAV_CMD_DO_SET_RESUME_REPEAT_DIST       = 215   // Set the distance to be repeated on mission resume. Params: 1) Distance.; 2) Empty.; 3) Empty.; 4) Empty.; 5) Empty.; 6) Empty.; 7) Empty.;
	MAV_CMD_NAV_ALTITUDE_WAIT               = 83    // Mission command to wait for an altitude or downwards vertical speed. This is meant for high altitude balloon launches, allowing the aircraft to be idle until either an altitude is reached or a negative vertical speed is reached (indicating early balloon burst). The wiggle time is how often to wiggle the control surfaces to prevent them seizing up. Params: 1) Altitude.; 2) Descent speed.; 3) How long to wiggle the control surfaces to prevent them seizing up.; 4) Empty.; 5) Empty.; 6) Empty.; 7) Empty.;
	MAV_CMD_POWER_OFF_INITIATED             = 42000 // A system wide power-off event has been initiated. Params: 1) Empty.; 2) Empty.; 3) Empty.; 4) Empty.; 5) Empty.; 6) Empty.; 7) Empty.;
	MAV_CMD_SOLO_BTN_FLY_CLICK              = 42001 // FLY button has been clicked. Params: 1) Empty.; 2) Empty.; 3) Empty.; 4) Empty.; 5) Empty.; 6) Empty.; 7) Empty.;
	MAV_CMD_SOLO_BTN_FLY_HOLD               = 42002 // FLY button has been held for 1.5 seconds. Params: 1) Takeoff altitude.; 2) Empty.; 3) Empty.; 4) Empty.; 5) Empty.; 6) Empty.; 7) Empty.;
	MAV_CMD_SOLO_BTN_PAUSE_CLICK            = 42003 // PAUSE button has been clicked. Params: 1) 1 if Solo is in a shot mode, 0 otherwise.; 2) Empty.; 3) Empty.; 4) Empty.; 5) Empty.; 6) Empty.; 7) Empty.;
	MAV_CMD_FIXED_MAG_CAL                   = 42004 // Magnetometer calibration based on fixed position         in earth field given by inclination, declination and intensity. Params: 1) Magnetic declination.; 2) Magnetic inclination.; 3) Magnetic intensity.; 4) Yaw.; 5) Empty.; 6) Empty.; 7) Empty.;
	MAV_CMD_FIXED_MAG_CAL_FIELD             = 42005 // Magnetometer calibration based on fixed expected field values. Params: 1) Field strength X.; 2) Field strength Y.; 3) Field strength Z.; 4) Empty.; 5) Empty.; 6) Empty.; 7) Empty.;
	MAV_CMD_DO_START_MAG_CAL                = 42424 // Initiate a magnetometer calibration. Params: 1) Bitmask of magnetometers to calibrate. Use 0 to calibrate all sensors that can be started (sensors may not start if disabled, unhealthy, etc.). The command will NACK if calibration does not start for a sensor explicitly specified by the bitmask.; 2) Automatically retry on failure (0=no retry, 1=retry).; 3) Save without user input (0=require input, 1=autosave).; 4) Delay.; 5) Autoreboot (0=user reboot, 1=autoreboot).; 6) Empty.; 7) Empty.;
	MAV_CMD_DO_ACCEPT_MAG_CAL               = 42425 // Accept a magnetometer calibration. Params: 1) Bitmask of magnetometers that calibration is accepted (0 means all).; 2) Empty.; 3) Empty.; 4) Empty.; 5) Empty.; 6) Empty.; 7) Empty.;
	MAV_CMD_DO_CANCEL_MAG_CAL               = 42426 // Cancel a running magnetometer calibration. Params: 1) Bitmask of magnetometers to cancel a running calibration (0 means all).; 2) Empty.; 3) Empty.; 4) Empty.; 5) Empty.; 6) Empty.; 7) Empty.;
	MAV_CMD_ACCELCAL_VEHICLE_POS            = 42429 // Used when doing accelerometer calibration. When sent to the GCS tells it what position to put the vehicle in. When sent to the vehicle says what position the vehicle is in. Params: 1) Position.; 2) Empty.; 3) Empty.; 4) Empty.; 5) Empty.; 6) Empty.; 7) Empty.;
	MAV_CMD_DO_SEND_BANNER                  = 42428 // Reply with the version banner. Params: 1) Empty.; 2) Empty.; 3) Empty.; 4) Empty.; 5) Empty.; 6) Empty.; 7) Empty.;
	MAV_CMD_SET_FACTORY_TEST_MODE           = 42427 // Command autopilot to get into factory test/diagnostic mode. Params: 1) 0: activate test mode, 1: exit test mode.; 2) Empty.; 3) Empty.; 4) Empty.; 5) Empty.; 6) Empty.; 7) Empty.;
	MAV_CMD_GIMBAL_RESET                    = 42501 // Causes the gimbal to reset and boot as if it was just powered on. Params: 1) Empty.; 2) Empty.; 3) Empty.; 4) Empty.; 5) Empty.; 6) Empty.; 7) Empty.;
	MAV_CMD_GIMBAL_AXIS_CALIBRATION_STATUS  = 42502 // Reports progress and success or failure of gimbal axis calibration procedure. Params: 1) Gimbal axis we're reporting calibration progress for.; 2) Current calibration progress for this axis.; 3) Status of the calibration.; 4) Empty.; 5) Empty.; 6) Empty.; 7) Empty.;
	MAV_CMD_GIMBAL_REQUEST_AXIS_CALIBRATION = 42503 // Starts commutation calibration on the gimbal. Params: 1) Empty.; 2) Empty.; 3) Empty.; 4) Empty.; 5) Empty.; 6) Empty.; 7) Empty.;
	MAV_CMD_GIMBAL_FULL_RESET               = 42505 // Erases gimbal application and parameters. Params: 1) Magic number.; 2) Magic number.; 3) Magic number.; 4) Magic number.; 5) Magic number.; 6) Magic number.; 7) Magic number.;
	MAV_CMD_FLASH_BOOTLOADER                = 42650 // Update the bootloader. Params: 1) Empty; 2) Empty; 3) Empty; 4) Empty; 5) Magic number - set to 290876 to actually flash; 6) Empty; 7) Empty;
	MAV_CMD_BATTERY_RESET                   = 42651 // Reset battery capacity for batteries that accumulate consumed battery via integration. Params: 1) Bitmask of batteries to reset. Least significant bit is for the first battery.; 2) Battery percentage remaining to set.;
	MAV_CMD_DEBUG_TRAP                      = 42700 // Issue a trap signal to the autopilot process, presumably to enter the debugger. Params: 1) Magic number - set to 32451 to actually trap.; 2) Empty.; 3) Empty.; 4) Empty.; 5) Empty.; 6) Empty.; 7) Empty.;
	MAV_CMD_SCRIPTING                       = 42701 // Control onboard scripting. Params: 1) Scripting command to execute;
	MAV_CMD_GUIDED_CHANGE_SPEED             = 43000 // Change flight speed at a given rate. This slews the vehicle at a controllable rate between it's previous speed and the new one. (affects GUIDED only. Outside GUIDED, aircraft ignores these commands. Designed for onboard companion-computer command-and-control, not normally operator/GCS control.). Params: 1) Airspeed or groundspeed.; 2) Target Speed; 3) Acceleration rate, 0 to take effect instantly; 4) Empty; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_GUIDED_CHANGE_ALTITUDE          = 43001 // Change target altitude at a given rate. This slews the vehicle at a controllable rate between it's previous altitude and the new one. (affects GUIDED only. Outside GUIDED, aircraft ignores these commands. Designed for onboard companion-computer command-and-control, not normally operator/GCS control.). Params: 1) Empty; 2) Empty; 3) Rate of change, toward new altitude. 0 for maximum rate change. Positive numbers only, as negative numbers will not converge on the new target alt.; 4) Empty; 5) Empty; 6) Empty; 7) Target Altitude;
	MAV_CMD_GUIDED_CHANGE_HEADING           = 43002 // Change to target heading at a given rate, overriding previous heading/s. This slews the vehicle at a controllable rate between it's previous heading and the new one. (affects GUIDED only. Exiting GUIDED returns aircraft to normal behaviour defined elsewhere. Designed for onboard companion-computer command-and-control, not normally operator/GCS control.). Params: 1) course-over-ground or raw vehicle heading.; 2) Target heading.; 3) Maximum centripetal accelearation, ie rate of change,  toward new heading.; 4) Empty; 5) Empty; 6) Empty; 7) Empty;
)

// ScriptingCmd (generated enum)
//
const (
	SCRIPTING_CMD_REPL_START = 0 // Start a REPL session
	SCRIPTING_CMD_REPL_STOP  = 1 // End a REPL session
)

// LimitsState (generated enum)
//
const (
	LIMITS_INIT       = 0 // Pre-initialization
	LIMITS_DISABLED   = 1 // Disabled
	LIMITS_ENABLED    = 2 // Checking limits
	LIMITS_TRIGGERED  = 3 // A limit has been breached
	LIMITS_RECOVERING = 4 // Taking action e.g. Return/RTL
	LIMITS_RECOVERED  = 5 // We're no longer in breach of a limit
)

// LimitModule (generated enum)
//
const (
	LIMIT_GPSLOCK  = 1 // Pre-initialization
	LIMIT_GEOFENCE = 2 // Disabled
	LIMIT_ALTITUDE = 4 // Checking limits
)

// RallyFlags (generated enum)
// Flags in RALLY_POINT message.
const (
	FAVORABLE_WIND   = 1 // Flag set when requiring favorable winds for landing
	LAND_IMMEDIATELY = 2 // Flag set when plane is to immediately descend to break altitude and land without GCS intervention. Flag not set when plane is to loiter at Rally point until commanded to land
)

// CameraStatusTypes (generated enum)
//
const (
	CAMERA_STATUS_TYPE_HEARTBEAT  = 0 // Camera heartbeat, announce camera component ID at 1Hz
	CAMERA_STATUS_TYPE_TRIGGER    = 1 // Camera image triggered
	CAMERA_STATUS_TYPE_DISCONNECT = 2 // Camera connection lost
	CAMERA_STATUS_TYPE_ERROR      = 3 // Camera unknown error
	CAMERA_STATUS_TYPE_LOWBATT    = 4 // Camera battery low. Parameter p1 shows reported voltage
	CAMERA_STATUS_TYPE_LOWSTORE   = 5 // Camera storage low. Parameter p1 shows reported shots remaining
	CAMERA_STATUS_TYPE_LOWSTOREV  = 6 // Camera storage low. Parameter p1 shows reported video minutes remaining
)

// CameraFeedbackFlags (generated enum)
//
const (
	CAMERA_FEEDBACK_PHOTO       = 0 // Shooting photos, not video
	CAMERA_FEEDBACK_VIDEO       = 1 // Shooting video, not stills
	CAMERA_FEEDBACK_BADEXPOSURE = 2 // Unable to achieve requested exposure (e.g. shutter speed too low)
	CAMERA_FEEDBACK_CLOSEDLOOP  = 3 // Closed loop feedback from camera, we know for sure it has successfully taken a picture
	CAMERA_FEEDBACK_OPENLOOP    = 4 // Open loop camera, an image trigger has been requested but we can't know for sure it has successfully taken a picture
)

// MavModeGimbal (generated enum)
//
const (
	MAV_MODE_GIMBAL_UNINITIALIZED     = 0 // Gimbal is powered on but has not started initializing yet
	MAV_MODE_GIMBAL_CALIBRATING_PITCH = 1 // Gimbal is currently running calibration on the pitch axis
	MAV_MODE_GIMBAL_CALIBRATING_ROLL  = 2 // Gimbal is currently running calibration on the roll axis
	MAV_MODE_GIMBAL_CALIBRATING_YAW   = 3 // Gimbal is currently running calibration on the yaw axis
	MAV_MODE_GIMBAL_INITIALIZED       = 4 // Gimbal has finished calibrating and initializing, but is relaxed pending reception of first rate command from copter
	MAV_MODE_GIMBAL_ACTIVE            = 5 // Gimbal is actively stabilizing
	MAV_MODE_GIMBAL_RATE_CMD_TIMEOUT  = 6 // Gimbal is relaxed because it missed more than 10 expected rate command messages in a row. Gimbal will move back to active mode when it receives a new rate command
)

// GimbalAxis (generated enum)
//
const (
	GIMBAL_AXIS_YAW   = 0 // Gimbal yaw axis
	GIMBAL_AXIS_PITCH = 1 // Gimbal pitch axis
	GIMBAL_AXIS_ROLL  = 2 // Gimbal roll axis
)

// GimbalAxisCalibrationStatus (generated enum)
//
const (
	GIMBAL_AXIS_CALIBRATION_STATUS_IN_PROGRESS = 0 // Axis calibration is in progress
	GIMBAL_AXIS_CALIBRATION_STATUS_SUCCEEDED   = 1 // Axis calibration succeeded
	GIMBAL_AXIS_CALIBRATION_STATUS_FAILED      = 2 // Axis calibration failed
)

// GimbalAxisCalibrationRequired (generated enum)
//
const (
	GIMBAL_AXIS_CALIBRATION_REQUIRED_UNKNOWN = 0 // Whether or not this axis requires calibration is unknown at this time
	GIMBAL_AXIS_CALIBRATION_REQUIRED_TRUE    = 1 // This axis requires calibration
	GIMBAL_AXIS_CALIBRATION_REQUIRED_FALSE   = 2 // This axis does not require calibration
)

// GoproHeartbeatStatus (generated enum)
//
const (
	GOPRO_HEARTBEAT_STATUS_DISCONNECTED = 0 // No GoPro connected
	GOPRO_HEARTBEAT_STATUS_INCOMPATIBLE = 1 // The detected GoPro is not HeroBus compatible
	GOPRO_HEARTBEAT_STATUS_CONNECTED    = 2 // A HeroBus compatible GoPro is connected
	GOPRO_HEARTBEAT_STATUS_ERROR        = 3 // An unrecoverable error was encountered with the connected GoPro, it may require a power cycle
)

// GoproHeartbeatFlags (generated enum)
//
const (
	GOPRO_FLAG_RECORDING = 1 // GoPro is currently recording
)

// GoproRequestStatus (generated enum)
//
const (
	GOPRO_REQUEST_SUCCESS = 0 // The write message with ID indicated succeeded
	GOPRO_REQUEST_FAILED  = 1 // The write message with ID indicated failed
)

// GoproCommand (generated enum)
//
const (
	GOPRO_COMMAND_POWER                 = 0  // (Get/Set)
	GOPRO_COMMAND_CAPTURE_MODE          = 1  // (Get/Set)
	GOPRO_COMMAND_SHUTTER               = 2  // (___/Set)
	GOPRO_COMMAND_BATTERY               = 3  // (Get/___)
	GOPRO_COMMAND_MODEL                 = 4  // (Get/___)
	GOPRO_COMMAND_VIDEO_SETTINGS        = 5  // (Get/Set)
	GOPRO_COMMAND_LOW_LIGHT             = 6  // (Get/Set)
	GOPRO_COMMAND_PHOTO_RESOLUTION      = 7  // (Get/Set)
	GOPRO_COMMAND_PHOTO_BURST_RATE      = 8  // (Get/Set)
	GOPRO_COMMAND_PROTUNE               = 9  // (Get/Set)
	GOPRO_COMMAND_PROTUNE_WHITE_BALANCE = 10 // (Get/Set) Hero 3+ Only
	GOPRO_COMMAND_PROTUNE_COLOUR        = 11 // (Get/Set) Hero 3+ Only
	GOPRO_COMMAND_PROTUNE_GAIN          = 12 // (Get/Set) Hero 3+ Only
	GOPRO_COMMAND_PROTUNE_SHARPNESS     = 13 // (Get/Set) Hero 3+ Only
	GOPRO_COMMAND_PROTUNE_EXPOSURE      = 14 // (Get/Set) Hero 3+ Only
	GOPRO_COMMAND_TIME                  = 15 // (Get/Set)
	GOPRO_COMMAND_CHARGING              = 16 // (Get/Set)
)

// GoproCaptureMode (generated enum)
//
const (
	GOPRO_CAPTURE_MODE_VIDEO      = 0   // Video mode
	GOPRO_CAPTURE_MODE_PHOTO      = 1   // Photo mode
	GOPRO_CAPTURE_MODE_BURST      = 2   // Burst mode, Hero 3+ only
	GOPRO_CAPTURE_MODE_TIME_LAPSE = 3   // Time lapse mode, Hero 3+ only
	GOPRO_CAPTURE_MODE_MULTI_SHOT = 4   // Multi shot mode, Hero 4 only
	GOPRO_CAPTURE_MODE_PLAYBACK   = 5   // Playback mode, Hero 4 only, silver only except when LCD or HDMI is connected to black
	GOPRO_CAPTURE_MODE_SETUP      = 6   // Playback mode, Hero 4 only
	GOPRO_CAPTURE_MODE_UNKNOWN    = 255 // Mode not yet known
)

// GoproResolution (generated enum)
//
const (
	GOPRO_RESOLUTION_480p            = 0  // 848 x 480 (480p)
	GOPRO_RESOLUTION_720p            = 1  // 1280 x 720 (720p)
	GOPRO_RESOLUTION_960p            = 2  // 1280 x 960 (960p)
	GOPRO_RESOLUTION_1080p           = 3  // 1920 x 1080 (1080p)
	GOPRO_RESOLUTION_1440p           = 4  // 1920 x 1440 (1440p)
	GOPRO_RESOLUTION_2_7k_17_9       = 5  // 2704 x 1440 (2.7k-17:9)
	GOPRO_RESOLUTION_2_7k_16_9       = 6  // 2704 x 1524 (2.7k-16:9)
	GOPRO_RESOLUTION_2_7k_4_3        = 7  // 2704 x 2028 (2.7k-4:3)
	GOPRO_RESOLUTION_4k_16_9         = 8  // 3840 x 2160 (4k-16:9)
	GOPRO_RESOLUTION_4k_17_9         = 9  // 4096 x 2160 (4k-17:9)
	GOPRO_RESOLUTION_720p_SUPERVIEW  = 10 // 1280 x 720 (720p-SuperView)
	GOPRO_RESOLUTION_1080p_SUPERVIEW = 11 // 1920 x 1080 (1080p-SuperView)
	GOPRO_RESOLUTION_2_7k_SUPERVIEW  = 12 // 2704 x 1520 (2.7k-SuperView)
	GOPRO_RESOLUTION_4k_SUPERVIEW    = 13 // 3840 x 2160 (4k-SuperView)
)

// GoproFrameRate (generated enum)
//
const (
	GOPRO_FRAME_RATE_12   = 0  // 12 FPS
	GOPRO_FRAME_RATE_15   = 1  // 15 FPS
	GOPRO_FRAME_RATE_24   = 2  // 24 FPS
	GOPRO_FRAME_RATE_25   = 3  // 25 FPS
	GOPRO_FRAME_RATE_30   = 4  // 30 FPS
	GOPRO_FRAME_RATE_48   = 5  // 48 FPS
	GOPRO_FRAME_RATE_50   = 6  // 50 FPS
	GOPRO_FRAME_RATE_60   = 7  // 60 FPS
	GOPRO_FRAME_RATE_80   = 8  // 80 FPS
	GOPRO_FRAME_RATE_90   = 9  // 90 FPS
	GOPRO_FRAME_RATE_100  = 10 // 100 FPS
	GOPRO_FRAME_RATE_120  = 11 // 120 FPS
	GOPRO_FRAME_RATE_240  = 12 // 240 FPS
	GOPRO_FRAME_RATE_12_5 = 13 // 12.5 FPS
)

// GoproFieldOfView (generated enum)
//
const (
	GOPRO_FIELD_OF_VIEW_WIDE   = 0 // 0x00: Wide
	GOPRO_FIELD_OF_VIEW_MEDIUM = 1 // 0x01: Medium
	GOPRO_FIELD_OF_VIEW_NARROW = 2 // 0x02: Narrow
)

// GoproVideoSettingsFlags (generated enum)
//
const (
	GOPRO_VIDEO_SETTINGS_TV_MODE = 1 // 0=NTSC, 1=PAL
)

// GoproPhotoResolution (generated enum)
//
const (
	GOPRO_PHOTO_RESOLUTION_5MP_MEDIUM = 0 // 5MP Medium
	GOPRO_PHOTO_RESOLUTION_7MP_MEDIUM = 1 // 7MP Medium
	GOPRO_PHOTO_RESOLUTION_7MP_WIDE   = 2 // 7MP Wide
	GOPRO_PHOTO_RESOLUTION_10MP_WIDE  = 3 // 10MP Wide
	GOPRO_PHOTO_RESOLUTION_12MP_WIDE  = 4 // 12MP Wide
)

// GoproProtuneWhiteBalance (generated enum)
//
const (
	GOPRO_PROTUNE_WHITE_BALANCE_AUTO  = 0 // Auto
	GOPRO_PROTUNE_WHITE_BALANCE_3000K = 1 // 3000K
	GOPRO_PROTUNE_WHITE_BALANCE_5500K = 2 // 5500K
	GOPRO_PROTUNE_WHITE_BALANCE_6500K = 3 // 6500K
	GOPRO_PROTUNE_WHITE_BALANCE_RAW   = 4 // Camera Raw
)

// GoproProtuneColour (generated enum)
//
const (
	GOPRO_PROTUNE_COLOUR_STANDARD = 0 // Auto
	GOPRO_PROTUNE_COLOUR_NEUTRAL  = 1 // Neutral
)

// GoproProtuneGain (generated enum)
//
const (
	GOPRO_PROTUNE_GAIN_400  = 0 // ISO 400
	GOPRO_PROTUNE_GAIN_800  = 1 // ISO 800 (Only Hero 4)
	GOPRO_PROTUNE_GAIN_1600 = 2 // ISO 1600
	GOPRO_PROTUNE_GAIN_3200 = 3 // ISO 3200 (Only Hero 4)
	GOPRO_PROTUNE_GAIN_6400 = 4 // ISO 6400
)

// GoproProtuneSharpness (generated enum)
//
const (
	GOPRO_PROTUNE_SHARPNESS_LOW    = 0 // Low Sharpness
	GOPRO_PROTUNE_SHARPNESS_MEDIUM = 1 // Medium Sharpness
	GOPRO_PROTUNE_SHARPNESS_HIGH   = 2 // High Sharpness
)

// GoproProtuneExposure (generated enum)
//
const (
	GOPRO_PROTUNE_EXPOSURE_NEG_5_0 = 0  // -5.0 EV (Hero 3+ Only)
	GOPRO_PROTUNE_EXPOSURE_NEG_4_5 = 1  // -4.5 EV (Hero 3+ Only)
	GOPRO_PROTUNE_EXPOSURE_NEG_4_0 = 2  // -4.0 EV (Hero 3+ Only)
	GOPRO_PROTUNE_EXPOSURE_NEG_3_5 = 3  // -3.5 EV (Hero 3+ Only)
	GOPRO_PROTUNE_EXPOSURE_NEG_3_0 = 4  // -3.0 EV (Hero 3+ Only)
	GOPRO_PROTUNE_EXPOSURE_NEG_2_5 = 5  // -2.5 EV (Hero 3+ Only)
	GOPRO_PROTUNE_EXPOSURE_NEG_2_0 = 6  // -2.0 EV
	GOPRO_PROTUNE_EXPOSURE_NEG_1_5 = 7  // -1.5 EV
	GOPRO_PROTUNE_EXPOSURE_NEG_1_0 = 8  // -1.0 EV
	GOPRO_PROTUNE_EXPOSURE_NEG_0_5 = 9  // -0.5 EV
	GOPRO_PROTUNE_EXPOSURE_ZERO    = 10 // 0.0 EV
	GOPRO_PROTUNE_EXPOSURE_POS_0_5 = 11 // +0.5 EV
	GOPRO_PROTUNE_EXPOSURE_POS_1_0 = 12 // +1.0 EV
	GOPRO_PROTUNE_EXPOSURE_POS_1_5 = 13 // +1.5 EV
	GOPRO_PROTUNE_EXPOSURE_POS_2_0 = 14 // +2.0 EV
	GOPRO_PROTUNE_EXPOSURE_POS_2_5 = 15 // +2.5 EV (Hero 3+ Only)
	GOPRO_PROTUNE_EXPOSURE_POS_3_0 = 16 // +3.0 EV (Hero 3+ Only)
	GOPRO_PROTUNE_EXPOSURE_POS_3_5 = 17 // +3.5 EV (Hero 3+ Only)
	GOPRO_PROTUNE_EXPOSURE_POS_4_0 = 18 // +4.0 EV (Hero 3+ Only)
	GOPRO_PROTUNE_EXPOSURE_POS_4_5 = 19 // +4.5 EV (Hero 3+ Only)
	GOPRO_PROTUNE_EXPOSURE_POS_5_0 = 20 // +5.0 EV (Hero 3+ Only)
)

// GoproCharging (generated enum)
//
const (
	GOPRO_CHARGING_DISABLED = 0 // Charging disabled
	GOPRO_CHARGING_ENABLED  = 1 // Charging enabled
)

// GoproModel (generated enum)
//
const (
	GOPRO_MODEL_UNKNOWN            = 0 // Unknown gopro model
	GOPRO_MODEL_HERO_3_PLUS_SILVER = 1 // Hero 3+ Silver (HeroBus not supported by GoPro)
	GOPRO_MODEL_HERO_3_PLUS_BLACK  = 2 // Hero 3+ Black
	GOPRO_MODEL_HERO_4_SILVER      = 3 // Hero 4 Silver
	GOPRO_MODEL_HERO_4_BLACK       = 4 // Hero 4 Black
)

// GoproBurstRate (generated enum)
//
const (
	GOPRO_BURST_RATE_3_IN_1_SECOND  = 0 // 3 Shots / 1 Second
	GOPRO_BURST_RATE_5_IN_1_SECOND  = 1 // 5 Shots / 1 Second
	GOPRO_BURST_RATE_10_IN_1_SECOND = 2 // 10 Shots / 1 Second
	GOPRO_BURST_RATE_10_IN_2_SECOND = 3 // 10 Shots / 2 Second
	GOPRO_BURST_RATE_10_IN_3_SECOND = 4 // 10 Shots / 3 Second (Hero 4 Only)
	GOPRO_BURST_RATE_30_IN_1_SECOND = 5 // 30 Shots / 1 Second
	GOPRO_BURST_RATE_30_IN_2_SECOND = 6 // 30 Shots / 2 Second
	GOPRO_BURST_RATE_30_IN_3_SECOND = 7 // 30 Shots / 3 Second
	GOPRO_BURST_RATE_30_IN_6_SECOND = 8 // 30 Shots / 6 Second
)

// LedControlPattern (generated enum)
//
const (
	LED_CONTROL_PATTERN_OFF            = 0   // LED patterns off (return control to regular vehicle control)
	LED_CONTROL_PATTERN_FIRMWAREUPDATE = 1   // LEDs show pattern during firmware update
	LED_CONTROL_PATTERN_CUSTOM         = 255 // Custom Pattern using custom bytes fields
)

// EkfStatusFlags (generated enum)
// Flags in EKF_STATUS message.
const (
	EKF_ATTITUDE           = 1    // Set if EKF's attitude estimate is good
	EKF_VELOCITY_HORIZ     = 2    // Set if EKF's horizontal velocity estimate is good
	EKF_VELOCITY_VERT      = 4    // Set if EKF's vertical velocity estimate is good
	EKF_POS_HORIZ_REL      = 8    // Set if EKF's horizontal position (relative) estimate is good
	EKF_POS_HORIZ_ABS      = 16   // Set if EKF's horizontal position (absolute) estimate is good
	EKF_POS_VERT_ABS       = 32   // Set if EKF's vertical position (absolute) estimate is good
	EKF_POS_VERT_AGL       = 64   // Set if EKF's vertical position (above ground) estimate is good
	EKF_CONST_POS_MODE     = 128  // EKF is in constant position mode and does not know it's absolute or relative position
	EKF_PRED_POS_HORIZ_REL = 256  // Set if EKF's predicted horizontal position (relative) estimate is good
	EKF_PRED_POS_HORIZ_ABS = 512  // Set if EKF's predicted horizontal position (absolute) estimate is good
	EKF_UNINITIALIZED      = 1024 // Set if EKF has never been healthy
)

// PidTuningAxis (generated enum)
//
const (
	PID_TUNING_ROLL    = 1 //
	PID_TUNING_PITCH   = 2 //
	PID_TUNING_YAW     = 3 //
	PID_TUNING_ACCZ    = 4 //
	PID_TUNING_STEER   = 5 //
	PID_TUNING_LANDING = 6 //
)

// MavRemoteLogDataBlockCommands (generated enum)
// Special ACK block numbers control activation of dataflash log streaming.
const (
	MAV_REMOTE_LOG_DATA_BLOCK_STOP  = 2147483645 // UAV to stop sending DataFlash blocks
	MAV_REMOTE_LOG_DATA_BLOCK_START = 2147483646 // UAV to start sending DataFlash blocks
)

// MavRemoteLogDataBlockStatuses (generated enum)
// Possible remote log data block statuses.
const (
	MAV_REMOTE_LOG_DATA_BLOCK_NACK = 0 // This block has NOT been received
	MAV_REMOTE_LOG_DATA_BLOCK_ACK  = 1 // This block has been received
)

// DeviceOpBustype (generated enum)
// Bus types for device operations.
const (
	DEVICE_OP_BUSTYPE_I2C = 0 // I2C Device operation
	DEVICE_OP_BUSTYPE_SPI = 1 // SPI Device operation
)

// DeepstallStage (generated enum)
// Deepstall flight stage.
const (
	DEEPSTALL_STAGE_FLY_TO_LANDING    = 0 // Flying to the landing point
	DEEPSTALL_STAGE_ESTIMATE_WIND     = 1 // Building an estimate of the wind
	DEEPSTALL_STAGE_WAIT_FOR_BREAKOUT = 2 // Waiting to breakout of the loiter to fly the approach
	DEEPSTALL_STAGE_FLY_TO_ARC        = 3 // Flying to the first arc point to turn around to the landing point
	DEEPSTALL_STAGE_ARC               = 4 // Turning around back to the deepstall landing point
	DEEPSTALL_STAGE_APPROACH          = 5 // Approaching the landing point
	DEEPSTALL_STAGE_LAND              = 6 // Stalling and steering towards the land point
)

// PlaneMode (generated enum)
// A mapping of plane flight modes for custom_mode field of heartbeat.
const (
	PLANE_MODE_MANUAL        = 0  //
	PLANE_MODE_CIRCLE        = 1  //
	PLANE_MODE_STABILIZE     = 2  //
	PLANE_MODE_TRAINING      = 3  //
	PLANE_MODE_ACRO          = 4  //
	PLANE_MODE_FLY_BY_WIRE_A = 5  //
	PLANE_MODE_FLY_BY_WIRE_B = 6  //
	PLANE_MODE_CRUISE        = 7  //
	PLANE_MODE_AUTOTUNE      = 8  //
	PLANE_MODE_AUTO          = 10 //
	PLANE_MODE_RTL           = 11 //
	PLANE_MODE_LOITER        = 12 //
	PLANE_MODE_TAKEOFF       = 13 //
	PLANE_MODE_AVOID_ADSB    = 14 //
	PLANE_MODE_GUIDED        = 15 //
	PLANE_MODE_INITIALIZING  = 16 //
	PLANE_MODE_QSTABILIZE    = 17 //
	PLANE_MODE_QHOVER        = 18 //
	PLANE_MODE_QLOITER       = 19 //
	PLANE_MODE_QLAND         = 20 //
	PLANE_MODE_QRTL          = 21 //
	PLANE_MODE_QAUTOTUNE     = 22 //
	PLANE_MODE_QACRO         = 23 //
)

// CopterMode (generated enum)
// A mapping of copter flight modes for custom_mode field of heartbeat.
const (
	COPTER_MODE_STABILIZE    = 0  //
	COPTER_MODE_ACRO         = 1  //
	COPTER_MODE_ALT_HOLD     = 2  //
	COPTER_MODE_AUTO         = 3  //
	COPTER_MODE_GUIDED       = 4  //
	COPTER_MODE_LOITER       = 5  //
	COPTER_MODE_RTL          = 6  //
	COPTER_MODE_CIRCLE       = 7  //
	COPTER_MODE_LAND         = 9  //
	COPTER_MODE_DRIFT        = 11 //
	COPTER_MODE_SPORT        = 13 //
	COPTER_MODE_FLIP         = 14 //
	COPTER_MODE_AUTOTUNE     = 15 //
	COPTER_MODE_POSHOLD      = 16 //
	COPTER_MODE_BRAKE        = 17 //
	COPTER_MODE_THROW        = 18 //
	COPTER_MODE_AVOID_ADSB   = 19 //
	COPTER_MODE_GUIDED_NOGPS = 20 //
	COPTER_MODE_SMART_RTL    = 21 //
	COPTER_MODE_FLOWHOLD     = 22 //
	COPTER_MODE_FOLLOW       = 23 //
	COPTER_MODE_ZIGZAG       = 24 //
	COPTER_MODE_SYSTEMID     = 25 //
	COPTER_MODE_AUTOROTATE   = 26 //
)

// SubMode (generated enum)
// A mapping of sub flight modes for custom_mode field of heartbeat.
const (
	SUB_MODE_STABILIZE = 0  //
	SUB_MODE_ACRO      = 1  //
	SUB_MODE_ALT_HOLD  = 2  //
	SUB_MODE_AUTO      = 3  //
	SUB_MODE_GUIDED    = 4  //
	SUB_MODE_CIRCLE    = 7  //
	SUB_MODE_SURFACE   = 9  //
	SUB_MODE_POSHOLD   = 16 //
	SUB_MODE_MANUAL    = 19 //
)

// RoverMode (generated enum)
// A mapping of rover flight modes for custom_mode field of heartbeat.
const (
	ROVER_MODE_MANUAL       = 0  //
	ROVER_MODE_ACRO         = 1  //
	ROVER_MODE_STEERING     = 3  //
	ROVER_MODE_HOLD         = 4  //
	ROVER_MODE_LOITER       = 5  //
	ROVER_MODE_FOLLOW       = 6  //
	ROVER_MODE_SIMPLE       = 7  //
	ROVER_MODE_AUTO         = 10 //
	ROVER_MODE_RTL          = 11 //
	ROVER_MODE_SMART_RTL    = 12 //
	ROVER_MODE_GUIDED       = 15 //
	ROVER_MODE_INITIALIZING = 16 //
)

// TrackerMode (generated enum)
// A mapping of antenna tracker flight modes for custom_mode field of heartbeat.
const (
	TRACKER_MODE_MANUAL       = 0  //
	TRACKER_MODE_STOP         = 1  //
	TRACKER_MODE_SCAN         = 2  //
	TRACKER_MODE_SERVO_TEST   = 3  //
	TRACKER_MODE_AUTO         = 10 //
	TRACKER_MODE_INITIALIZING = 16 //
)

// OsdParamConfigType (generated enum)
// The type of parameter for the OSD parameter editor.
const (
	OSD_PARAM_NONE              = 0 //
	OSD_PARAM_SERIAL_PROTOCOL   = 1 //
	OSD_PARAM_SERVO_FUNCTION    = 2 //
	OSD_PARAM_AUX_FUNCTION      = 3 //
	OSD_PARAM_FLIGHT_MODE       = 4 //
	OSD_PARAM_FAILSAFE_ACTION   = 5 //
	OSD_PARAM_FAILSAFE_ACTION_1 = 6 //
	OSD_PARAM_FAILSAFE_ACTION_2 = 7 //
	OSD_PARAM_NUM_TYPES         = 8 //
)

// OsdParamConfigError (generated enum)
// The error type for the OSD parameter editor.
const (
	OSD_PARAM_SUCCESS                 = 0 //
	OSD_PARAM_INVALID_SCREEN          = 1 //
	OSD_PARAM_INVALID_PARAMETER_INDEX = 2 //
	OSD_PARAM_INVALID_PARAMETER       = 3 //
)

// SensorOffsets struct (generated typeinfo)
// Offsets and calibrations values for hardware sensors. This makes it easier to debug the calibration process.
type SensorOffsets struct {
	MagDeclination float32 // Magnetic declination.
	RawPress       int32   // Raw pressure from barometer.
	RawTemp        int32   // Raw temperature from barometer.
	GyroCalX       float32 // Gyro X calibration.
	GyroCalY       float32 // Gyro Y calibration.
	GyroCalZ       float32 // Gyro Z calibration.
	AccelCalX      float32 // Accel X calibration.
	AccelCalY      float32 // Accel Y calibration.
	AccelCalZ      float32 // Accel Z calibration.
	MagOfsX        int16   // Magnetometer X offset.
	MagOfsY        int16   // Magnetometer Y offset.
	MagOfsZ        int16   // Magnetometer Z offset.
}

// MsgID (generated function)
func (m *SensorOffsets) MsgID() mavlink.MessageID {
	return MSG_ID_SENSOR_OFFSETS
}

// String (generated function)
func (m *SensorOffsets) String() string {
	return fmt.Sprintf(
		"&ardupilotmega.SensorOffsets{ MagDeclination: %+v, RawPress: %+v, RawTemp: %+v, GyroCalX: %+v, GyroCalY: %+v, GyroCalZ: %+v, AccelCalX: %+v, AccelCalY: %+v, AccelCalZ: %+v, MagOfsX: %+v, MagOfsY: %+v, MagOfsZ: %+v }",
		m.MagDeclination,
		m.RawPress,
		m.RawTemp,
		m.GyroCalX,
		m.GyroCalY,
		m.GyroCalZ,
		m.AccelCalX,
		m.AccelCalY,
		m.AccelCalZ,
		m.MagOfsX,
		m.MagOfsY,
		m.MagOfsZ,
	)
}

// Pack (generated function)
func (m *SensorOffsets) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 42)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(m.MagDeclination))
	binary.LittleEndian.PutUint32(payload[4:], uint32(m.RawPress))
	binary.LittleEndian.PutUint32(payload[8:], uint32(m.RawTemp))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.GyroCalX))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(m.GyroCalY))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(m.GyroCalZ))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(m.AccelCalX))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(m.AccelCalY))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(m.AccelCalZ))
	binary.LittleEndian.PutUint16(payload[36:], uint16(m.MagOfsX))
	binary.LittleEndian.PutUint16(payload[38:], uint16(m.MagOfsY))
	binary.LittleEndian.PutUint16(payload[40:], uint16(m.MagOfsZ))
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *SensorOffsets) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 42 {
		return mavlink.ErrPayloadTooSmall
	}
	m.MagDeclination = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.RawPress = int32(binary.LittleEndian.Uint32(payload[4:]))
	m.RawTemp = int32(binary.LittleEndian.Uint32(payload[8:]))
	m.GyroCalX = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.GyroCalY = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	m.GyroCalZ = math.Float32frombits(binary.LittleEndian.Uint32(payload[20:]))
	m.AccelCalX = math.Float32frombits(binary.LittleEndian.Uint32(payload[24:]))
	m.AccelCalY = math.Float32frombits(binary.LittleEndian.Uint32(payload[28:]))
	m.AccelCalZ = math.Float32frombits(binary.LittleEndian.Uint32(payload[32:]))
	m.MagOfsX = int16(binary.LittleEndian.Uint16(payload[36:]))
	m.MagOfsY = int16(binary.LittleEndian.Uint16(payload[38:]))
	m.MagOfsZ = int16(binary.LittleEndian.Uint16(payload[40:]))
	return nil
}

// SetMagOffsets struct (generated typeinfo)
// Set the magnetometer offsets
type SetMagOffsets struct {
	MagOfsX         int16 // Magnetometer X offset.
	MagOfsY         int16 // Magnetometer Y offset.
	MagOfsZ         int16 // Magnetometer Z offset.
	TargetSystem    uint8 // System ID.
	TargetComponent uint8 // Component ID.
}

// MsgID (generated function)
func (m *SetMagOffsets) MsgID() mavlink.MessageID {
	return MSG_ID_SET_MAG_OFFSETS
}

// String (generated function)
func (m *SetMagOffsets) String() string {
	return fmt.Sprintf(
		"&ardupilotmega.SetMagOffsets{ MagOfsX: %+v, MagOfsY: %+v, MagOfsZ: %+v, TargetSystem: %+v, TargetComponent: %+v }",
		m.MagOfsX,
		m.MagOfsY,
		m.MagOfsZ,
		m.TargetSystem,
		m.TargetComponent,
	)
}

// Pack (generated function)
func (m *SetMagOffsets) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 8)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.MagOfsX))
	binary.LittleEndian.PutUint16(payload[2:], uint16(m.MagOfsY))
	binary.LittleEndian.PutUint16(payload[4:], uint16(m.MagOfsZ))
	payload[6] = byte(m.TargetSystem)
	payload[7] = byte(m.TargetComponent)
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *SetMagOffsets) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 8 {
		return mavlink.ErrPayloadTooSmall
	}
	m.MagOfsX = int16(binary.LittleEndian.Uint16(payload[0:]))
	m.MagOfsY = int16(binary.LittleEndian.Uint16(payload[2:]))
	m.MagOfsZ = int16(binary.LittleEndian.Uint16(payload[4:]))
	m.TargetSystem = uint8(payload[6])
	m.TargetComponent = uint8(payload[7])
	return nil
}

// Meminfo struct (generated typeinfo)
// State of APM memory.
type Meminfo struct {
	Brkval  uint16 // Heap top.
	Freemem uint16 // Free memory.
}

// MsgID (generated function)
func (m *Meminfo) MsgID() mavlink.MessageID {
	return MSG_ID_MEMINFO
}

// String (generated function)
func (m *Meminfo) String() string {
	return fmt.Sprintf(
		"&ardupilotmega.Meminfo{ Brkval: %+v, Freemem: %+v }",
		m.Brkval,
		m.Freemem,
	)
}

// Pack (generated function)
func (m *Meminfo) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 4)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.Brkval))
	binary.LittleEndian.PutUint16(payload[2:], uint16(m.Freemem))
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *Meminfo) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 4 {
		return mavlink.ErrPayloadTooSmall
	}
	m.Brkval = uint16(binary.LittleEndian.Uint16(payload[0:]))
	m.Freemem = uint16(binary.LittleEndian.Uint16(payload[2:]))
	return nil
}

// ApAdc struct (generated typeinfo)
// Raw ADC output.
type ApAdc struct {
	Adc1 uint16 // ADC output 1.
	Adc2 uint16 // ADC output 2.
	Adc3 uint16 // ADC output 3.
	Adc4 uint16 // ADC output 4.
	Adc5 uint16 // ADC output 5.
	Adc6 uint16 // ADC output 6.
}

// MsgID (generated function)
func (m *ApAdc) MsgID() mavlink.MessageID {
	return MSG_ID_AP_ADC
}

// String (generated function)
func (m *ApAdc) String() string {
	return fmt.Sprintf(
		"&ardupilotmega.ApAdc{ Adc1: %+v, Adc2: %+v, Adc3: %+v, Adc4: %+v, Adc5: %+v, Adc6: %+v }",
		m.Adc1,
		m.Adc2,
		m.Adc3,
		m.Adc4,
		m.Adc5,
		m.Adc6,
	)
}

// Pack (generated function)
func (m *ApAdc) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 12)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.Adc1))
	binary.LittleEndian.PutUint16(payload[2:], uint16(m.Adc2))
	binary.LittleEndian.PutUint16(payload[4:], uint16(m.Adc3))
	binary.LittleEndian.PutUint16(payload[6:], uint16(m.Adc4))
	binary.LittleEndian.PutUint16(payload[8:], uint16(m.Adc5))
	binary.LittleEndian.PutUint16(payload[10:], uint16(m.Adc6))
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ApAdc) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 12 {
		return mavlink.ErrPayloadTooSmall
	}
	m.Adc1 = uint16(binary.LittleEndian.Uint16(payload[0:]))
	m.Adc2 = uint16(binary.LittleEndian.Uint16(payload[2:]))
	m.Adc3 = uint16(binary.LittleEndian.Uint16(payload[4:]))
	m.Adc4 = uint16(binary.LittleEndian.Uint16(payload[6:]))
	m.Adc5 = uint16(binary.LittleEndian.Uint16(payload[8:]))
	m.Adc6 = uint16(binary.LittleEndian.Uint16(payload[10:]))
	return nil
}

// DigicamConfigure struct (generated typeinfo)
// Configure on-board Camera Control System.
type DigicamConfigure struct {
	ExtraValue      float32 // Correspondent value to given extra_param.
	ShutterSpeed    uint16  // Divisor number //e.g. 1000 means 1/1000 (0 means ignore).
	TargetSystem    uint8   // System ID.
	TargetComponent uint8   // Component ID.
	Mode            uint8   // Mode enumeration from 1 to N //P, TV, AV, M, etc. (0 means ignore).
	Aperture        uint8   // F stop number x 10 //e.g. 28 means 2.8 (0 means ignore).
	Iso             uint8   // ISO enumeration from 1 to N //e.g. 80, 100, 200, Etc (0 means ignore).
	ExposureType    uint8   // Exposure type enumeration from 1 to N (0 means ignore).
	CommandID       uint8   // Command Identity (incremental loop: 0 to 255). //A command sent multiple times will be executed or pooled just once.
	EngineCutOff    uint8   // Main engine cut-off time before camera trigger (0 means no cut-off).
	ExtraParam      uint8   // Extra parameters enumeration (0 means ignore).
}

// MsgID (generated function)
func (m *DigicamConfigure) MsgID() mavlink.MessageID {
	return MSG_ID_DIGICAM_CONFIGURE
}

// String (generated function)
func (m *DigicamConfigure) String() string {
	return fmt.Sprintf(
		"&ardupilotmega.DigicamConfigure{ ExtraValue: %+v, ShutterSpeed: %+v, TargetSystem: %+v, TargetComponent: %+v, Mode: %+v, Aperture: %+v, Iso: %+v, ExposureType: %+v, CommandID: %+v, EngineCutOff: %+v, ExtraParam: %+v }",
		m.ExtraValue,
		m.ShutterSpeed,
		m.TargetSystem,
		m.TargetComponent,
		m.Mode,
		m.Aperture,
		m.Iso,
		m.ExposureType,
		m.CommandID,
		m.EngineCutOff,
		m.ExtraParam,
	)
}

// Pack (generated function)
func (m *DigicamConfigure) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 15)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(m.ExtraValue))
	binary.LittleEndian.PutUint16(payload[4:], uint16(m.ShutterSpeed))
	payload[6] = byte(m.TargetSystem)
	payload[7] = byte(m.TargetComponent)
	payload[8] = byte(m.Mode)
	payload[9] = byte(m.Aperture)
	payload[10] = byte(m.Iso)
	payload[11] = byte(m.ExposureType)
	payload[12] = byte(m.CommandID)
	payload[13] = byte(m.EngineCutOff)
	payload[14] = byte(m.ExtraParam)
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *DigicamConfigure) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 15 {
		return mavlink.ErrPayloadTooSmall
	}
	m.ExtraValue = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.ShutterSpeed = uint16(binary.LittleEndian.Uint16(payload[4:]))
	m.TargetSystem = uint8(payload[6])
	m.TargetComponent = uint8(payload[7])
	m.Mode = uint8(payload[8])
	m.Aperture = uint8(payload[9])
	m.Iso = uint8(payload[10])
	m.ExposureType = uint8(payload[11])
	m.CommandID = uint8(payload[12])
	m.EngineCutOff = uint8(payload[13])
	m.ExtraParam = uint8(payload[14])
	return nil
}

// DigicamControl struct (generated typeinfo)
// Control on-board Camera Control System to take shots.
type DigicamControl struct {
	ExtraValue      float32 // Correspondent value to given extra_param.
	TargetSystem    uint8   // System ID.
	TargetComponent uint8   // Component ID.
	Session         uint8   // 0: stop, 1: start or keep it up //Session control e.g. show/hide lens.
	ZoomPos         uint8   // 1 to N //Zoom's absolute position (0 means ignore).
	ZoomStep        int8    // -100 to 100 //Zooming step value to offset zoom from the current position.
	FocusLock       uint8   // 0: unlock focus or keep unlocked, 1: lock focus or keep locked, 3: re-lock focus.
	Shot            uint8   // 0: ignore, 1: shot or start filming.
	CommandID       uint8   // Command Identity (incremental loop: 0 to 255)//A command sent multiple times will be executed or pooled just once.
	ExtraParam      uint8   // Extra parameters enumeration (0 means ignore).
}

// MsgID (generated function)
func (m *DigicamControl) MsgID() mavlink.MessageID {
	return MSG_ID_DIGICAM_CONTROL
}

// String (generated function)
func (m *DigicamControl) String() string {
	return fmt.Sprintf(
		"&ardupilotmega.DigicamControl{ ExtraValue: %+v, TargetSystem: %+v, TargetComponent: %+v, Session: %+v, ZoomPos: %+v, ZoomStep: %+v, FocusLock: %+v, Shot: %+v, CommandID: %+v, ExtraParam: %+v }",
		m.ExtraValue,
		m.TargetSystem,
		m.TargetComponent,
		m.Session,
		m.ZoomPos,
		m.ZoomStep,
		m.FocusLock,
		m.Shot,
		m.CommandID,
		m.ExtraParam,
	)
}

// Pack (generated function)
func (m *DigicamControl) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 13)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(m.ExtraValue))
	payload[4] = byte(m.TargetSystem)
	payload[5] = byte(m.TargetComponent)
	payload[6] = byte(m.Session)
	payload[7] = byte(m.ZoomPos)
	payload[8] = byte(m.ZoomStep)
	payload[9] = byte(m.FocusLock)
	payload[10] = byte(m.Shot)
	payload[11] = byte(m.CommandID)
	payload[12] = byte(m.ExtraParam)
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *DigicamControl) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 13 {
		return mavlink.ErrPayloadTooSmall
	}
	m.ExtraValue = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.TargetSystem = uint8(payload[4])
	m.TargetComponent = uint8(payload[5])
	m.Session = uint8(payload[6])
	m.ZoomPos = uint8(payload[7])
	m.ZoomStep = int8(payload[8])
	m.FocusLock = uint8(payload[9])
	m.Shot = uint8(payload[10])
	m.CommandID = uint8(payload[11])
	m.ExtraParam = uint8(payload[12])
	return nil
}

// MountConfigure struct (generated typeinfo)
// Message to configure a camera mount, directional antenna, etc.
type MountConfigure struct {
	TargetSystem    uint8 // System ID.
	TargetComponent uint8 // Component ID.
	MountMode       uint8 // Mount operating mode.
	StabRoll        uint8 // (1 = yes, 0 = no).
	StabPitch       uint8 // (1 = yes, 0 = no).
	StabYaw         uint8 // (1 = yes, 0 = no).
}

// MsgID (generated function)
func (m *MountConfigure) MsgID() mavlink.MessageID {
	return MSG_ID_MOUNT_CONFIGURE
}

// String (generated function)
func (m *MountConfigure) String() string {
	return fmt.Sprintf(
		"&ardupilotmega.MountConfigure{ TargetSystem: %+v, TargetComponent: %+v, MountMode: %+v, StabRoll: %+v, StabPitch: %+v, StabYaw: %+v }",
		m.TargetSystem,
		m.TargetComponent,
		m.MountMode,
		m.StabRoll,
		m.StabPitch,
		m.StabYaw,
	)
}

// Pack (generated function)
func (m *MountConfigure) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 6)
	payload[0] = byte(m.TargetSystem)
	payload[1] = byte(m.TargetComponent)
	payload[2] = byte(m.MountMode)
	payload[3] = byte(m.StabRoll)
	payload[4] = byte(m.StabPitch)
	payload[5] = byte(m.StabYaw)
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *MountConfigure) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 6 {
		return mavlink.ErrPayloadTooSmall
	}
	m.TargetSystem = uint8(payload[0])
	m.TargetComponent = uint8(payload[1])
	m.MountMode = uint8(payload[2])
	m.StabRoll = uint8(payload[3])
	m.StabPitch = uint8(payload[4])
	m.StabYaw = uint8(payload[5])
	return nil
}

// MountControl struct (generated typeinfo)
// Message to control a camera mount, directional antenna, etc.
type MountControl struct {
	InputA          int32 // Pitch (centi-degrees) or lat (degE7), depending on mount mode.
	InputB          int32 // Roll (centi-degrees) or lon (degE7) depending on mount mode.
	InputC          int32 // Yaw (centi-degrees) or alt (cm) depending on mount mode.
	TargetSystem    uint8 // System ID.
	TargetComponent uint8 // Component ID.
	SavePosition    uint8 // If "1" it will save current trimmed position on EEPROM (just valid for NEUTRAL and LANDING).
}

// MsgID (generated function)
func (m *MountControl) MsgID() mavlink.MessageID {
	return MSG_ID_MOUNT_CONTROL
}

// String (generated function)
func (m *MountControl) String() string {
	return fmt.Sprintf(
		"&ardupilotmega.MountControl{ InputA: %+v, InputB: %+v, InputC: %+v, TargetSystem: %+v, TargetComponent: %+v, SavePosition: %+v }",
		m.InputA,
		m.InputB,
		m.InputC,
		m.TargetSystem,
		m.TargetComponent,
		m.SavePosition,
	)
}

// Pack (generated function)
func (m *MountControl) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 15)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.InputA))
	binary.LittleEndian.PutUint32(payload[4:], uint32(m.InputB))
	binary.LittleEndian.PutUint32(payload[8:], uint32(m.InputC))
	payload[12] = byte(m.TargetSystem)
	payload[13] = byte(m.TargetComponent)
	payload[14] = byte(m.SavePosition)
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *MountControl) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 15 {
		return mavlink.ErrPayloadTooSmall
	}
	m.InputA = int32(binary.LittleEndian.Uint32(payload[0:]))
	m.InputB = int32(binary.LittleEndian.Uint32(payload[4:]))
	m.InputC = int32(binary.LittleEndian.Uint32(payload[8:]))
	m.TargetSystem = uint8(payload[12])
	m.TargetComponent = uint8(payload[13])
	m.SavePosition = uint8(payload[14])
	return nil
}

// MountStatus struct (generated typeinfo)
// Message with some status from APM to GCS about camera or antenna mount.
type MountStatus struct {
	PointingA       int32 // Pitch.
	PointingB       int32 // Roll.
	PointingC       int32 // Yaw.
	TargetSystem    uint8 // System ID.
	TargetComponent uint8 // Component ID.
}

// MsgID (generated function)
func (m *MountStatus) MsgID() mavlink.MessageID {
	return MSG_ID_MOUNT_STATUS
}

// String (generated function)
func (m *MountStatus) String() string {
	return fmt.Sprintf(
		"&ardupilotmega.MountStatus{ PointingA: %+v, PointingB: %+v, PointingC: %+v, TargetSystem: %+v, TargetComponent: %+v }",
		m.PointingA,
		m.PointingB,
		m.PointingC,
		m.TargetSystem,
		m.TargetComponent,
	)
}

// Pack (generated function)
func (m *MountStatus) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 14)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.PointingA))
	binary.LittleEndian.PutUint32(payload[4:], uint32(m.PointingB))
	binary.LittleEndian.PutUint32(payload[8:], uint32(m.PointingC))
	payload[12] = byte(m.TargetSystem)
	payload[13] = byte(m.TargetComponent)
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *MountStatus) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 14 {
		return mavlink.ErrPayloadTooSmall
	}
	m.PointingA = int32(binary.LittleEndian.Uint32(payload[0:]))
	m.PointingB = int32(binary.LittleEndian.Uint32(payload[4:]))
	m.PointingC = int32(binary.LittleEndian.Uint32(payload[8:]))
	m.TargetSystem = uint8(payload[12])
	m.TargetComponent = uint8(payload[13])
	return nil
}

// FencePoint struct (generated typeinfo)
// A fence point. Used to set a point when from GCS -> MAV. Also used to return a point from MAV -> GCS.
type FencePoint struct {
	Lat             float32 // Latitude of point.
	Lng             float32 // Longitude of point.
	TargetSystem    uint8   // System ID.
	TargetComponent uint8   // Component ID.
	Idx             uint8   // Point index (first point is 1, 0 is for return point).
	Count           uint8   // Total number of points (for sanity checking).
}

// MsgID (generated function)
func (m *FencePoint) MsgID() mavlink.MessageID {
	return MSG_ID_FENCE_POINT
}

// String (generated function)
func (m *FencePoint) String() string {
	return fmt.Sprintf(
		"&ardupilotmega.FencePoint{ Lat: %+v, Lng: %+v, TargetSystem: %+v, TargetComponent: %+v, Idx: %+v, Count: %+v }",
		m.Lat,
		m.Lng,
		m.TargetSystem,
		m.TargetComponent,
		m.Idx,
		m.Count,
	)
}

// Pack (generated function)
func (m *FencePoint) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 12)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(m.Lat))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(m.Lng))
	payload[8] = byte(m.TargetSystem)
	payload[9] = byte(m.TargetComponent)
	payload[10] = byte(m.Idx)
	payload[11] = byte(m.Count)
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *FencePoint) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 12 {
		return mavlink.ErrPayloadTooSmall
	}
	m.Lat = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.Lng = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	m.TargetSystem = uint8(payload[8])
	m.TargetComponent = uint8(payload[9])
	m.Idx = uint8(payload[10])
	m.Count = uint8(payload[11])
	return nil
}

// FenceFetchPoint struct (generated typeinfo)
// Request a current fence point from MAV.
type FenceFetchPoint struct {
	TargetSystem    uint8 // System ID.
	TargetComponent uint8 // Component ID.
	Idx             uint8 // Point index (first point is 1, 0 is for return point).
}

// MsgID (generated function)
func (m *FenceFetchPoint) MsgID() mavlink.MessageID {
	return MSG_ID_FENCE_FETCH_POINT
}

// String (generated function)
func (m *FenceFetchPoint) String() string {
	return fmt.Sprintf(
		"&ardupilotmega.FenceFetchPoint{ TargetSystem: %+v, TargetComponent: %+v, Idx: %+v }",
		m.TargetSystem,
		m.TargetComponent,
		m.Idx,
	)
}

// Pack (generated function)
func (m *FenceFetchPoint) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 3)
	payload[0] = byte(m.TargetSystem)
	payload[1] = byte(m.TargetComponent)
	payload[2] = byte(m.Idx)
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *FenceFetchPoint) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 3 {
		return mavlink.ErrPayloadTooSmall
	}
	m.TargetSystem = uint8(payload[0])
	m.TargetComponent = uint8(payload[1])
	m.Idx = uint8(payload[2])
	return nil
}

// Ahrs struct (generated typeinfo)
// Status of DCM attitude estimator.
type Ahrs struct {
	Omegaix     float32 // X gyro drift estimate.
	Omegaiy     float32 // Y gyro drift estimate.
	Omegaiz     float32 // Z gyro drift estimate.
	AccelWeight float32 // Average accel_weight.
	RenormVal   float32 // Average renormalisation value.
	ErrorRp     float32 // Average error_roll_pitch value.
	ErrorYaw    float32 // Average error_yaw value.
}

// MsgID (generated function)
func (m *Ahrs) MsgID() mavlink.MessageID {
	return MSG_ID_AHRS
}

// String (generated function)
func (m *Ahrs) String() string {
	return fmt.Sprintf(
		"&ardupilotmega.Ahrs{ Omegaix: %+v, Omegaiy: %+v, Omegaiz: %+v, AccelWeight: %+v, RenormVal: %+v, ErrorRp: %+v, ErrorYaw: %+v }",
		m.Omegaix,
		m.Omegaiy,
		m.Omegaiz,
		m.AccelWeight,
		m.RenormVal,
		m.ErrorRp,
		m.ErrorYaw,
	)
}

// Pack (generated function)
func (m *Ahrs) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 28)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(m.Omegaix))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(m.Omegaiy))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.Omegaiz))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.AccelWeight))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(m.RenormVal))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(m.ErrorRp))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(m.ErrorYaw))
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *Ahrs) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 28 {
		return mavlink.ErrPayloadTooSmall
	}
	m.Omegaix = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.Omegaiy = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	m.Omegaiz = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.AccelWeight = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.RenormVal = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	m.ErrorRp = math.Float32frombits(binary.LittleEndian.Uint32(payload[20:]))
	m.ErrorYaw = math.Float32frombits(binary.LittleEndian.Uint32(payload[24:]))
	return nil
}

// Simstate struct (generated typeinfo)
// Status of simulation environment, if used.
type Simstate struct {
	Roll  float32 // Roll angle.
	Pitch float32 // Pitch angle.
	Yaw   float32 // Yaw angle.
	Xacc  float32 // X acceleration.
	Yacc  float32 // Y acceleration.
	Zacc  float32 // Z acceleration.
	Xgyro float32 // Angular speed around X axis.
	Ygyro float32 // Angular speed around Y axis.
	Zgyro float32 // Angular speed around Z axis.
	Lat   int32   // Latitude.
	Lng   int32   // Longitude.
}

// MsgID (generated function)
func (m *Simstate) MsgID() mavlink.MessageID {
	return MSG_ID_SIMSTATE
}

// String (generated function)
func (m *Simstate) String() string {
	return fmt.Sprintf(
		"&ardupilotmega.Simstate{ Roll: %+v, Pitch: %+v, Yaw: %+v, Xacc: %+v, Yacc: %+v, Zacc: %+v, Xgyro: %+v, Ygyro: %+v, Zgyro: %+v, Lat: %+v, Lng: %+v }",
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
		m.Lng,
	)
}

// Pack (generated function)
func (m *Simstate) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 44)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(m.Roll))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(m.Pitch))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.Yaw))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.Xacc))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(m.Yacc))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(m.Zacc))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(m.Xgyro))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(m.Ygyro))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(m.Zgyro))
	binary.LittleEndian.PutUint32(payload[36:], uint32(m.Lat))
	binary.LittleEndian.PutUint32(payload[40:], uint32(m.Lng))
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *Simstate) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 44 {
		return mavlink.ErrPayloadTooSmall
	}
	m.Roll = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.Pitch = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	m.Yaw = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.Xacc = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.Yacc = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	m.Zacc = math.Float32frombits(binary.LittleEndian.Uint32(payload[20:]))
	m.Xgyro = math.Float32frombits(binary.LittleEndian.Uint32(payload[24:]))
	m.Ygyro = math.Float32frombits(binary.LittleEndian.Uint32(payload[28:]))
	m.Zgyro = math.Float32frombits(binary.LittleEndian.Uint32(payload[32:]))
	m.Lat = int32(binary.LittleEndian.Uint32(payload[36:]))
	m.Lng = int32(binary.LittleEndian.Uint32(payload[40:]))
	return nil
}

// Hwstatus struct (generated typeinfo)
// Status of key hardware.
type Hwstatus struct {
	Vcc    uint16 // Board voltage.
	I2cerr uint8  // I2C error count.
}

// MsgID (generated function)
func (m *Hwstatus) MsgID() mavlink.MessageID {
	return MSG_ID_HWSTATUS
}

// String (generated function)
func (m *Hwstatus) String() string {
	return fmt.Sprintf(
		"&ardupilotmega.Hwstatus{ Vcc: %+v, I2cerr: %+v }",
		m.Vcc,
		m.I2cerr,
	)
}

// Pack (generated function)
func (m *Hwstatus) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 3)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.Vcc))
	payload[2] = byte(m.I2cerr)
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *Hwstatus) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 3 {
		return mavlink.ErrPayloadTooSmall
	}
	m.Vcc = uint16(binary.LittleEndian.Uint16(payload[0:]))
	m.I2cerr = uint8(payload[2])
	return nil
}

// Radio struct (generated typeinfo)
// Status generated by radio.
type Radio struct {
	Rxerrors uint16 // Receive errors.
	Fixed    uint16 // Count of error corrected packets.
	Rssi     uint8  // Local signal strength.
	Remrssi  uint8  // Remote signal strength.
	Txbuf    uint8  // How full the tx buffer is.
	Noise    uint8  // Background noise level.
	Remnoise uint8  // Remote background noise level.
}

// MsgID (generated function)
func (m *Radio) MsgID() mavlink.MessageID {
	return MSG_ID_RADIO
}

// String (generated function)
func (m *Radio) String() string {
	return fmt.Sprintf(
		"&ardupilotmega.Radio{ Rxerrors: %+v, Fixed: %+v, Rssi: %+v, Remrssi: %+v, Txbuf: %+v, Noise: %+v, Remnoise: %+v }",
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
func (m *Radio) Pack(p *mavlink.Packet) error {
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
func (m *Radio) Unpack(p *mavlink.Packet) error {
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

// LimitsStatus struct (generated typeinfo)
// Status of AP_Limits. Sent in extended status stream when AP_Limits is enabled.
type LimitsStatus struct {
	LastTrigger   uint32 // Time (since boot) of last breach.
	LastAction    uint32 // Time (since boot) of last recovery action.
	LastRecovery  uint32 // Time (since boot) of last successful recovery.
	LastClear     uint32 // Time (since boot) of last all-clear.
	BreachCount   uint16 // Number of fence breaches.
	LimitsState   uint8  // State of AP_Limits.
	ModsEnabled   uint8  // AP_Limit_Module bitfield of enabled modules.
	ModsRequired  uint8  // AP_Limit_Module bitfield of required modules.
	ModsTriggered uint8  // AP_Limit_Module bitfield of triggered modules.
}

// MsgID (generated function)
func (m *LimitsStatus) MsgID() mavlink.MessageID {
	return MSG_ID_LIMITS_STATUS
}

// String (generated function)
func (m *LimitsStatus) String() string {
	return fmt.Sprintf(
		"&ardupilotmega.LimitsStatus{ LastTrigger: %+v, LastAction: %+v, LastRecovery: %+v, LastClear: %+v, BreachCount: %+v, LimitsState: %+v, ModsEnabled: %+v, ModsRequired: %+v, ModsTriggered: %+v }",
		m.LastTrigger,
		m.LastAction,
		m.LastRecovery,
		m.LastClear,
		m.BreachCount,
		m.LimitsState,
		m.ModsEnabled,
		m.ModsRequired,
		m.ModsTriggered,
	)
}

// Pack (generated function)
func (m *LimitsStatus) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 22)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.LastTrigger))
	binary.LittleEndian.PutUint32(payload[4:], uint32(m.LastAction))
	binary.LittleEndian.PutUint32(payload[8:], uint32(m.LastRecovery))
	binary.LittleEndian.PutUint32(payload[12:], uint32(m.LastClear))
	binary.LittleEndian.PutUint16(payload[16:], uint16(m.BreachCount))
	payload[18] = byte(m.LimitsState)
	payload[19] = byte(m.ModsEnabled)
	payload[20] = byte(m.ModsRequired)
	payload[21] = byte(m.ModsTriggered)
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *LimitsStatus) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 22 {
		return mavlink.ErrPayloadTooSmall
	}
	m.LastTrigger = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.LastAction = uint32(binary.LittleEndian.Uint32(payload[4:]))
	m.LastRecovery = uint32(binary.LittleEndian.Uint32(payload[8:]))
	m.LastClear = uint32(binary.LittleEndian.Uint32(payload[12:]))
	m.BreachCount = uint16(binary.LittleEndian.Uint16(payload[16:]))
	m.LimitsState = uint8(payload[18])
	m.ModsEnabled = uint8(payload[19])
	m.ModsRequired = uint8(payload[20])
	m.ModsTriggered = uint8(payload[21])
	return nil
}

// Wind struct (generated typeinfo)
// Wind estimation.
type Wind struct {
	Direction float32 // Wind direction (that wind is coming from).
	Speed     float32 // Wind speed in ground plane.
	SpeedZ    float32 // Vertical wind speed.
}

// MsgID (generated function)
func (m *Wind) MsgID() mavlink.MessageID {
	return MSG_ID_WIND
}

// String (generated function)
func (m *Wind) String() string {
	return fmt.Sprintf(
		"&ardupilotmega.Wind{ Direction: %+v, Speed: %+v, SpeedZ: %+v }",
		m.Direction,
		m.Speed,
		m.SpeedZ,
	)
}

// Pack (generated function)
func (m *Wind) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 12)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(m.Direction))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(m.Speed))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.SpeedZ))
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *Wind) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 12 {
		return mavlink.ErrPayloadTooSmall
	}
	m.Direction = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.Speed = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	m.SpeedZ = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	return nil
}

// Data16 struct (generated typeinfo)
// Data packet, size 16.
type Data16 struct {
	Type uint8     // Data type.
	Len  uint8     // Data length.
	Data [16]uint8 // Raw data.
}

// MsgID (generated function)
func (m *Data16) MsgID() mavlink.MessageID {
	return MSG_ID_DATA16
}

// String (generated function)
func (m *Data16) String() string {
	return fmt.Sprintf(
		"&ardupilotmega.Data16{ Type: %+v, Len: %+v, Data: %0X (\"%s\") }",
		m.Type,
		m.Len,
		m.Data, string(m.Data[:]),
	)
}

// Pack (generated function)
func (m *Data16) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 18)
	payload[0] = byte(m.Type)
	payload[1] = byte(m.Len)
	copy(payload[2:], m.Data[:])
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *Data16) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 18 {
		return mavlink.ErrPayloadTooSmall
	}
	m.Type = uint8(payload[0])
	m.Len = uint8(payload[1])
	copy(m.Data[:], payload[2:18])
	return nil
}

// Data32 struct (generated typeinfo)
// Data packet, size 32.
type Data32 struct {
	Type uint8     // Data type.
	Len  uint8     // Data length.
	Data [32]uint8 // Raw data.
}

// MsgID (generated function)
func (m *Data32) MsgID() mavlink.MessageID {
	return MSG_ID_DATA32
}

// String (generated function)
func (m *Data32) String() string {
	return fmt.Sprintf(
		"&ardupilotmega.Data32{ Type: %+v, Len: %+v, Data: %0X (\"%s\") }",
		m.Type,
		m.Len,
		m.Data, string(m.Data[:]),
	)
}

// Pack (generated function)
func (m *Data32) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 34)
	payload[0] = byte(m.Type)
	payload[1] = byte(m.Len)
	copy(payload[2:], m.Data[:])
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *Data32) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 34 {
		return mavlink.ErrPayloadTooSmall
	}
	m.Type = uint8(payload[0])
	m.Len = uint8(payload[1])
	copy(m.Data[:], payload[2:34])
	return nil
}

// Data64 struct (generated typeinfo)
// Data packet, size 64.
type Data64 struct {
	Type uint8     // Data type.
	Len  uint8     // Data length.
	Data [64]uint8 // Raw data.
}

// MsgID (generated function)
func (m *Data64) MsgID() mavlink.MessageID {
	return MSG_ID_DATA64
}

// String (generated function)
func (m *Data64) String() string {
	return fmt.Sprintf(
		"&ardupilotmega.Data64{ Type: %+v, Len: %+v, Data: %0X (\"%s\") }",
		m.Type,
		m.Len,
		m.Data, string(m.Data[:]),
	)
}

// Pack (generated function)
func (m *Data64) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 66)
	payload[0] = byte(m.Type)
	payload[1] = byte(m.Len)
	copy(payload[2:], m.Data[:])
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *Data64) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 66 {
		return mavlink.ErrPayloadTooSmall
	}
	m.Type = uint8(payload[0])
	m.Len = uint8(payload[1])
	copy(m.Data[:], payload[2:66])
	return nil
}

// Data96 struct (generated typeinfo)
// Data packet, size 96.
type Data96 struct {
	Type uint8     // Data type.
	Len  uint8     // Data length.
	Data [96]uint8 // Raw data.
}

// MsgID (generated function)
func (m *Data96) MsgID() mavlink.MessageID {
	return MSG_ID_DATA96
}

// String (generated function)
func (m *Data96) String() string {
	return fmt.Sprintf(
		"&ardupilotmega.Data96{ Type: %+v, Len: %+v, Data: %0X (\"%s\") }",
		m.Type,
		m.Len,
		m.Data, string(m.Data[:]),
	)
}

// Pack (generated function)
func (m *Data96) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 98)
	payload[0] = byte(m.Type)
	payload[1] = byte(m.Len)
	copy(payload[2:], m.Data[:])
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *Data96) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 98 {
		return mavlink.ErrPayloadTooSmall
	}
	m.Type = uint8(payload[0])
	m.Len = uint8(payload[1])
	copy(m.Data[:], payload[2:98])
	return nil
}

// Rangefinder struct (generated typeinfo)
// Rangefinder reporting.
type Rangefinder struct {
	Distance float32 // Distance.
	Voltage  float32 // Raw voltage if available, zero otherwise.
}

// MsgID (generated function)
func (m *Rangefinder) MsgID() mavlink.MessageID {
	return MSG_ID_RANGEFINDER
}

// String (generated function)
func (m *Rangefinder) String() string {
	return fmt.Sprintf(
		"&ardupilotmega.Rangefinder{ Distance: %+v, Voltage: %+v }",
		m.Distance,
		m.Voltage,
	)
}

// Pack (generated function)
func (m *Rangefinder) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 8)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(m.Distance))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(m.Voltage))
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *Rangefinder) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 8 {
		return mavlink.ErrPayloadTooSmall
	}
	m.Distance = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.Voltage = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	return nil
}

// AirspeedAutocal struct (generated typeinfo)
// Airspeed auto-calibration.
type AirspeedAutocal struct {
	Vx           float32 // GPS velocity north.
	Vy           float32 // GPS velocity east.
	Vz           float32 // GPS velocity down.
	DiffPressure float32 // Differential pressure.
	Eas2tas      float32 // Estimated to true airspeed ratio.
	Ratio        float32 // Airspeed ratio.
	StateX       float32 // EKF state x.
	StateY       float32 // EKF state y.
	StateZ       float32 // EKF state z.
	Pax          float32 // EKF Pax.
	Pby          float32 // EKF Pby.
	Pcz          float32 // EKF Pcz.
}

// MsgID (generated function)
func (m *AirspeedAutocal) MsgID() mavlink.MessageID {
	return MSG_ID_AIRSPEED_AUTOCAL
}

// String (generated function)
func (m *AirspeedAutocal) String() string {
	return fmt.Sprintf(
		"&ardupilotmega.AirspeedAutocal{ Vx: %+v, Vy: %+v, Vz: %+v, DiffPressure: %+v, Eas2tas: %+v, Ratio: %+v, StateX: %+v, StateY: %+v, StateZ: %+v, Pax: %+v, Pby: %+v, Pcz: %+v }",
		m.Vx,
		m.Vy,
		m.Vz,
		m.DiffPressure,
		m.Eas2tas,
		m.Ratio,
		m.StateX,
		m.StateY,
		m.StateZ,
		m.Pax,
		m.Pby,
		m.Pcz,
	)
}

// Pack (generated function)
func (m *AirspeedAutocal) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 48)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(m.Vx))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(m.Vy))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.Vz))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.DiffPressure))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(m.Eas2tas))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(m.Ratio))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(m.StateX))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(m.StateY))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(m.StateZ))
	binary.LittleEndian.PutUint32(payload[36:], math.Float32bits(m.Pax))
	binary.LittleEndian.PutUint32(payload[40:], math.Float32bits(m.Pby))
	binary.LittleEndian.PutUint32(payload[44:], math.Float32bits(m.Pcz))
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *AirspeedAutocal) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 48 {
		return mavlink.ErrPayloadTooSmall
	}
	m.Vx = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.Vy = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	m.Vz = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.DiffPressure = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.Eas2tas = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	m.Ratio = math.Float32frombits(binary.LittleEndian.Uint32(payload[20:]))
	m.StateX = math.Float32frombits(binary.LittleEndian.Uint32(payload[24:]))
	m.StateY = math.Float32frombits(binary.LittleEndian.Uint32(payload[28:]))
	m.StateZ = math.Float32frombits(binary.LittleEndian.Uint32(payload[32:]))
	m.Pax = math.Float32frombits(binary.LittleEndian.Uint32(payload[36:]))
	m.Pby = math.Float32frombits(binary.LittleEndian.Uint32(payload[40:]))
	m.Pcz = math.Float32frombits(binary.LittleEndian.Uint32(payload[44:]))
	return nil
}

// RallyPoint struct (generated typeinfo)
// A rally point. Used to set a point when from GCS -> MAV. Also used to return a point from MAV -> GCS.
type RallyPoint struct {
	Lat             int32  // Latitude of point.
	Lng             int32  // Longitude of point.
	Alt             int16  // Transit / loiter altitude relative to home.
	BreakAlt        int16  // Break altitude relative to home.
	LandDir         uint16 // Heading to aim for when landing.
	TargetSystem    uint8  // System ID.
	TargetComponent uint8  // Component ID.
	Idx             uint8  // Point index (first point is 0).
	Count           uint8  // Total number of points (for sanity checking).
	Flags           uint8  // Configuration flags.
}

// MsgID (generated function)
func (m *RallyPoint) MsgID() mavlink.MessageID {
	return MSG_ID_RALLY_POINT
}

// String (generated function)
func (m *RallyPoint) String() string {
	return fmt.Sprintf(
		"&ardupilotmega.RallyPoint{ Lat: %+v, Lng: %+v, Alt: %+v, BreakAlt: %+v, LandDir: %+v, TargetSystem: %+v, TargetComponent: %+v, Idx: %+v, Count: %+v, Flags: %+v }",
		m.Lat,
		m.Lng,
		m.Alt,
		m.BreakAlt,
		m.LandDir,
		m.TargetSystem,
		m.TargetComponent,
		m.Idx,
		m.Count,
		m.Flags,
	)
}

// Pack (generated function)
func (m *RallyPoint) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 19)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.Lat))
	binary.LittleEndian.PutUint32(payload[4:], uint32(m.Lng))
	binary.LittleEndian.PutUint16(payload[8:], uint16(m.Alt))
	binary.LittleEndian.PutUint16(payload[10:], uint16(m.BreakAlt))
	binary.LittleEndian.PutUint16(payload[12:], uint16(m.LandDir))
	payload[14] = byte(m.TargetSystem)
	payload[15] = byte(m.TargetComponent)
	payload[16] = byte(m.Idx)
	payload[17] = byte(m.Count)
	payload[18] = byte(m.Flags)
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *RallyPoint) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 19 {
		return mavlink.ErrPayloadTooSmall
	}
	m.Lat = int32(binary.LittleEndian.Uint32(payload[0:]))
	m.Lng = int32(binary.LittleEndian.Uint32(payload[4:]))
	m.Alt = int16(binary.LittleEndian.Uint16(payload[8:]))
	m.BreakAlt = int16(binary.LittleEndian.Uint16(payload[10:]))
	m.LandDir = uint16(binary.LittleEndian.Uint16(payload[12:]))
	m.TargetSystem = uint8(payload[14])
	m.TargetComponent = uint8(payload[15])
	m.Idx = uint8(payload[16])
	m.Count = uint8(payload[17])
	m.Flags = uint8(payload[18])
	return nil
}

// RallyFetchPoint struct (generated typeinfo)
// Request a current rally point from MAV. MAV should respond with a RALLY_POINT message. MAV should not respond if the request is invalid.
type RallyFetchPoint struct {
	TargetSystem    uint8 // System ID.
	TargetComponent uint8 // Component ID.
	Idx             uint8 // Point index (first point is 0).
}

// MsgID (generated function)
func (m *RallyFetchPoint) MsgID() mavlink.MessageID {
	return MSG_ID_RALLY_FETCH_POINT
}

// String (generated function)
func (m *RallyFetchPoint) String() string {
	return fmt.Sprintf(
		"&ardupilotmega.RallyFetchPoint{ TargetSystem: %+v, TargetComponent: %+v, Idx: %+v }",
		m.TargetSystem,
		m.TargetComponent,
		m.Idx,
	)
}

// Pack (generated function)
func (m *RallyFetchPoint) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 3)
	payload[0] = byte(m.TargetSystem)
	payload[1] = byte(m.TargetComponent)
	payload[2] = byte(m.Idx)
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *RallyFetchPoint) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 3 {
		return mavlink.ErrPayloadTooSmall
	}
	m.TargetSystem = uint8(payload[0])
	m.TargetComponent = uint8(payload[1])
	m.Idx = uint8(payload[2])
	return nil
}

// CompassmotStatus struct (generated typeinfo)
// Status of compassmot calibration.
type CompassmotStatus struct {
	Current       float32 // Current.
	Compensationx float32 // Motor Compensation X.
	Compensationy float32 // Motor Compensation Y.
	Compensationz float32 // Motor Compensation Z.
	Throttle      uint16  // Throttle.
	Interference  uint16  // Interference.
}

// MsgID (generated function)
func (m *CompassmotStatus) MsgID() mavlink.MessageID {
	return MSG_ID_COMPASSMOT_STATUS
}

// String (generated function)
func (m *CompassmotStatus) String() string {
	return fmt.Sprintf(
		"&ardupilotmega.CompassmotStatus{ Current: %+v, Compensationx: %+v, Compensationy: %+v, Compensationz: %+v, Throttle: %+v, Interference: %+v }",
		m.Current,
		m.Compensationx,
		m.Compensationy,
		m.Compensationz,
		m.Throttle,
		m.Interference,
	)
}

// Pack (generated function)
func (m *CompassmotStatus) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 20)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(m.Current))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(m.Compensationx))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.Compensationy))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.Compensationz))
	binary.LittleEndian.PutUint16(payload[16:], uint16(m.Throttle))
	binary.LittleEndian.PutUint16(payload[18:], uint16(m.Interference))
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *CompassmotStatus) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 20 {
		return mavlink.ErrPayloadTooSmall
	}
	m.Current = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.Compensationx = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	m.Compensationy = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.Compensationz = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.Throttle = uint16(binary.LittleEndian.Uint16(payload[16:]))
	m.Interference = uint16(binary.LittleEndian.Uint16(payload[18:]))
	return nil
}

// Ahrs2 struct (generated typeinfo)
// Status of secondary AHRS filter if available.
type Ahrs2 struct {
	Roll     float32 // Roll angle.
	Pitch    float32 // Pitch angle.
	Yaw      float32 // Yaw angle.
	Altitude float32 // Altitude (MSL).
	Lat      int32   // Latitude.
	Lng      int32   // Longitude.
}

// MsgID (generated function)
func (m *Ahrs2) MsgID() mavlink.MessageID {
	return MSG_ID_AHRS2
}

// String (generated function)
func (m *Ahrs2) String() string {
	return fmt.Sprintf(
		"&ardupilotmega.Ahrs2{ Roll: %+v, Pitch: %+v, Yaw: %+v, Altitude: %+v, Lat: %+v, Lng: %+v }",
		m.Roll,
		m.Pitch,
		m.Yaw,
		m.Altitude,
		m.Lat,
		m.Lng,
	)
}

// Pack (generated function)
func (m *Ahrs2) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 24)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(m.Roll))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(m.Pitch))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.Yaw))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.Altitude))
	binary.LittleEndian.PutUint32(payload[16:], uint32(m.Lat))
	binary.LittleEndian.PutUint32(payload[20:], uint32(m.Lng))
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *Ahrs2) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 24 {
		return mavlink.ErrPayloadTooSmall
	}
	m.Roll = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.Pitch = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	m.Yaw = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.Altitude = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.Lat = int32(binary.LittleEndian.Uint32(payload[16:]))
	m.Lng = int32(binary.LittleEndian.Uint32(payload[20:]))
	return nil
}

// CameraStatus struct (generated typeinfo)
// Camera Event.
type CameraStatus struct {
	TimeUsec     uint64  // Image timestamp (since UNIX epoch, according to camera clock).
	P1           float32 // Parameter 1 (meaning depends on event_id, see CAMERA_STATUS_TYPES enum).
	P2           float32 // Parameter 2 (meaning depends on event_id, see CAMERA_STATUS_TYPES enum).
	P3           float32 // Parameter 3 (meaning depends on event_id, see CAMERA_STATUS_TYPES enum).
	P4           float32 // Parameter 4 (meaning depends on event_id, see CAMERA_STATUS_TYPES enum).
	ImgIdx       uint16  // Image index.
	TargetSystem uint8   // System ID.
	CamIdx       uint8   // Camera ID.
	EventID      uint8   // Event type.
}

// MsgID (generated function)
func (m *CameraStatus) MsgID() mavlink.MessageID {
	return MSG_ID_CAMERA_STATUS
}

// String (generated function)
func (m *CameraStatus) String() string {
	return fmt.Sprintf(
		"&ardupilotmega.CameraStatus{ TimeUsec: %+v, P1: %+v, P2: %+v, P3: %+v, P4: %+v, ImgIdx: %+v, TargetSystem: %+v, CamIdx: %+v, EventID: %+v }",
		m.TimeUsec,
		m.P1,
		m.P2,
		m.P3,
		m.P4,
		m.ImgIdx,
		m.TargetSystem,
		m.CamIdx,
		m.EventID,
	)
}

// Pack (generated function)
func (m *CameraStatus) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 29)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.TimeUsec))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.P1))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.P2))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(m.P3))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(m.P4))
	binary.LittleEndian.PutUint16(payload[24:], uint16(m.ImgIdx))
	payload[26] = byte(m.TargetSystem)
	payload[27] = byte(m.CamIdx)
	payload[28] = byte(m.EventID)
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *CameraStatus) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 29 {
		return mavlink.ErrPayloadTooSmall
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(payload[0:]))
	m.P1 = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.P2 = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.P3 = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	m.P4 = math.Float32frombits(binary.LittleEndian.Uint32(payload[20:]))
	m.ImgIdx = uint16(binary.LittleEndian.Uint16(payload[24:]))
	m.TargetSystem = uint8(payload[26])
	m.CamIdx = uint8(payload[27])
	m.EventID = uint8(payload[28])
	return nil
}

// CameraFeedback struct (generated typeinfo)
// Camera Capture Feedback.
type CameraFeedback struct {
	TimeUsec     uint64  // Image timestamp (since UNIX epoch), as passed in by CAMERA_STATUS message (or autopilot if no CCB).
	Lat          int32   // Latitude.
	Lng          int32   // Longitude.
	AltMsl       float32 // Altitude (MSL).
	AltRel       float32 // Altitude (Relative to HOME location).
	Roll         float32 // Camera Roll angle (earth frame, +-180).
	Pitch        float32 // Camera Pitch angle (earth frame, +-180).
	Yaw          float32 // Camera Yaw (earth frame, 0-360, true).
	FocLen       float32 // Focal Length.
	ImgIdx       uint16  // Image index.
	TargetSystem uint8   // System ID.
	CamIdx       uint8   // Camera ID.
	Flags        uint8   // Feedback flags.
}

// MsgID (generated function)
func (m *CameraFeedback) MsgID() mavlink.MessageID {
	return MSG_ID_CAMERA_FEEDBACK
}

// String (generated function)
func (m *CameraFeedback) String() string {
	return fmt.Sprintf(
		"&ardupilotmega.CameraFeedback{ TimeUsec: %+v, Lat: %+v, Lng: %+v, AltMsl: %+v, AltRel: %+v, Roll: %+v, Pitch: %+v, Yaw: %+v, FocLen: %+v, ImgIdx: %+v, TargetSystem: %+v, CamIdx: %+v, Flags: %+v }",
		m.TimeUsec,
		m.Lat,
		m.Lng,
		m.AltMsl,
		m.AltRel,
		m.Roll,
		m.Pitch,
		m.Yaw,
		m.FocLen,
		m.ImgIdx,
		m.TargetSystem,
		m.CamIdx,
		m.Flags,
	)
}

// Pack (generated function)
func (m *CameraFeedback) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 45)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.TimeUsec))
	binary.LittleEndian.PutUint32(payload[8:], uint32(m.Lat))
	binary.LittleEndian.PutUint32(payload[12:], uint32(m.Lng))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(m.AltMsl))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(m.AltRel))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(m.Roll))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(m.Pitch))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(m.Yaw))
	binary.LittleEndian.PutUint32(payload[36:], math.Float32bits(m.FocLen))
	binary.LittleEndian.PutUint16(payload[40:], uint16(m.ImgIdx))
	payload[42] = byte(m.TargetSystem)
	payload[43] = byte(m.CamIdx)
	payload[44] = byte(m.Flags)
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *CameraFeedback) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 45 {
		return mavlink.ErrPayloadTooSmall
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(payload[0:]))
	m.Lat = int32(binary.LittleEndian.Uint32(payload[8:]))
	m.Lng = int32(binary.LittleEndian.Uint32(payload[12:]))
	m.AltMsl = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	m.AltRel = math.Float32frombits(binary.LittleEndian.Uint32(payload[20:]))
	m.Roll = math.Float32frombits(binary.LittleEndian.Uint32(payload[24:]))
	m.Pitch = math.Float32frombits(binary.LittleEndian.Uint32(payload[28:]))
	m.Yaw = math.Float32frombits(binary.LittleEndian.Uint32(payload[32:]))
	m.FocLen = math.Float32frombits(binary.LittleEndian.Uint32(payload[36:]))
	m.ImgIdx = uint16(binary.LittleEndian.Uint16(payload[40:]))
	m.TargetSystem = uint8(payload[42])
	m.CamIdx = uint8(payload[43])
	m.Flags = uint8(payload[44])
	return nil
}

// Battery2 struct (generated typeinfo)
// 2nd Battery status
type Battery2 struct {
	Voltage        uint16 // Voltage.
	CurrentBattery int16  // Battery current, -1: autopilot does not measure the current.
}

// MsgID (generated function)
func (m *Battery2) MsgID() mavlink.MessageID {
	return MSG_ID_BATTERY2
}

// String (generated function)
func (m *Battery2) String() string {
	return fmt.Sprintf(
		"&ardupilotmega.Battery2{ Voltage: %+v, CurrentBattery: %+v }",
		m.Voltage,
		m.CurrentBattery,
	)
}

// Pack (generated function)
func (m *Battery2) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 4)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.Voltage))
	binary.LittleEndian.PutUint16(payload[2:], uint16(m.CurrentBattery))
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *Battery2) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 4 {
		return mavlink.ErrPayloadTooSmall
	}
	m.Voltage = uint16(binary.LittleEndian.Uint16(payload[0:]))
	m.CurrentBattery = int16(binary.LittleEndian.Uint16(payload[2:]))
	return nil
}

// Ahrs3 struct (generated typeinfo)
// Status of third AHRS filter if available. This is for ANU research group (Ali and Sean).
type Ahrs3 struct {
	Roll     float32 // Roll angle.
	Pitch    float32 // Pitch angle.
	Yaw      float32 // Yaw angle.
	Altitude float32 // Altitude (MSL).
	Lat      int32   // Latitude.
	Lng      int32   // Longitude.
	V1       float32 // Test variable1.
	V2       float32 // Test variable2.
	V3       float32 // Test variable3.
	V4       float32 // Test variable4.
}

// MsgID (generated function)
func (m *Ahrs3) MsgID() mavlink.MessageID {
	return MSG_ID_AHRS3
}

// String (generated function)
func (m *Ahrs3) String() string {
	return fmt.Sprintf(
		"&ardupilotmega.Ahrs3{ Roll: %+v, Pitch: %+v, Yaw: %+v, Altitude: %+v, Lat: %+v, Lng: %+v, V1: %+v, V2: %+v, V3: %+v, V4: %+v }",
		m.Roll,
		m.Pitch,
		m.Yaw,
		m.Altitude,
		m.Lat,
		m.Lng,
		m.V1,
		m.V2,
		m.V3,
		m.V4,
	)
}

// Pack (generated function)
func (m *Ahrs3) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 40)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(m.Roll))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(m.Pitch))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.Yaw))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.Altitude))
	binary.LittleEndian.PutUint32(payload[16:], uint32(m.Lat))
	binary.LittleEndian.PutUint32(payload[20:], uint32(m.Lng))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(m.V1))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(m.V2))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(m.V3))
	binary.LittleEndian.PutUint32(payload[36:], math.Float32bits(m.V4))
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *Ahrs3) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 40 {
		return mavlink.ErrPayloadTooSmall
	}
	m.Roll = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.Pitch = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	m.Yaw = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.Altitude = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.Lat = int32(binary.LittleEndian.Uint32(payload[16:]))
	m.Lng = int32(binary.LittleEndian.Uint32(payload[20:]))
	m.V1 = math.Float32frombits(binary.LittleEndian.Uint32(payload[24:]))
	m.V2 = math.Float32frombits(binary.LittleEndian.Uint32(payload[28:]))
	m.V3 = math.Float32frombits(binary.LittleEndian.Uint32(payload[32:]))
	m.V4 = math.Float32frombits(binary.LittleEndian.Uint32(payload[36:]))
	return nil
}

// AutopilotVersionRequest struct (generated typeinfo)
// Request the autopilot version from the system/component.
type AutopilotVersionRequest struct {
	TargetSystem    uint8 // System ID.
	TargetComponent uint8 // Component ID.
}

// MsgID (generated function)
func (m *AutopilotVersionRequest) MsgID() mavlink.MessageID {
	return MSG_ID_AUTOPILOT_VERSION_REQUEST
}

// String (generated function)
func (m *AutopilotVersionRequest) String() string {
	return fmt.Sprintf(
		"&ardupilotmega.AutopilotVersionRequest{ TargetSystem: %+v, TargetComponent: %+v }",
		m.TargetSystem,
		m.TargetComponent,
	)
}

// Pack (generated function)
func (m *AutopilotVersionRequest) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 2)
	payload[0] = byte(m.TargetSystem)
	payload[1] = byte(m.TargetComponent)
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *AutopilotVersionRequest) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 2 {
		return mavlink.ErrPayloadTooSmall
	}
	m.TargetSystem = uint8(payload[0])
	m.TargetComponent = uint8(payload[1])
	return nil
}

// RemoteLogDataBlock struct (generated typeinfo)
// Send a block of log data to remote location.
type RemoteLogDataBlock struct {
	Seqno           uint32     // Log data block sequence number.
	TargetSystem    uint8      // System ID.
	TargetComponent uint8      // Component ID.
	Data            [200]uint8 // Log data block.
}

// MsgID (generated function)
func (m *RemoteLogDataBlock) MsgID() mavlink.MessageID {
	return MSG_ID_REMOTE_LOG_DATA_BLOCK
}

// String (generated function)
func (m *RemoteLogDataBlock) String() string {
	return fmt.Sprintf(
		"&ardupilotmega.RemoteLogDataBlock{ Seqno: %+v, TargetSystem: %+v, TargetComponent: %+v, Data: %0X (\"%s\") }",
		m.Seqno,
		m.TargetSystem,
		m.TargetComponent,
		m.Data, string(m.Data[:]),
	)
}

// Pack (generated function)
func (m *RemoteLogDataBlock) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 206)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.Seqno))
	payload[4] = byte(m.TargetSystem)
	payload[5] = byte(m.TargetComponent)
	copy(payload[6:], m.Data[:])
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *RemoteLogDataBlock) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 206 {
		return mavlink.ErrPayloadTooSmall
	}
	m.Seqno = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.TargetSystem = uint8(payload[4])
	m.TargetComponent = uint8(payload[5])
	copy(m.Data[:], payload[6:206])
	return nil
}

// RemoteLogBlockStatus struct (generated typeinfo)
// Send Status of each log block that autopilot board might have sent.
type RemoteLogBlockStatus struct {
	Seqno           uint32 // Log data block sequence number.
	TargetSystem    uint8  // System ID.
	TargetComponent uint8  // Component ID.
	Status          uint8  // Log data block status.
}

// MsgID (generated function)
func (m *RemoteLogBlockStatus) MsgID() mavlink.MessageID {
	return MSG_ID_REMOTE_LOG_BLOCK_STATUS
}

// String (generated function)
func (m *RemoteLogBlockStatus) String() string {
	return fmt.Sprintf(
		"&ardupilotmega.RemoteLogBlockStatus{ Seqno: %+v, TargetSystem: %+v, TargetComponent: %+v, Status: %+v }",
		m.Seqno,
		m.TargetSystem,
		m.TargetComponent,
		m.Status,
	)
}

// Pack (generated function)
func (m *RemoteLogBlockStatus) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 7)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.Seqno))
	payload[4] = byte(m.TargetSystem)
	payload[5] = byte(m.TargetComponent)
	payload[6] = byte(m.Status)
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *RemoteLogBlockStatus) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 7 {
		return mavlink.ErrPayloadTooSmall
	}
	m.Seqno = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.TargetSystem = uint8(payload[4])
	m.TargetComponent = uint8(payload[5])
	m.Status = uint8(payload[6])
	return nil
}

// LedControl struct (generated typeinfo)
// Control vehicle LEDs.
type LedControl struct {
	TargetSystem    uint8     // System ID.
	TargetComponent uint8     // Component ID.
	Instance        uint8     // Instance (LED instance to control or 255 for all LEDs).
	Pattern         uint8     // Pattern (see LED_PATTERN_ENUM).
	CustomLen       uint8     // Custom Byte Length.
	CustomBytes     [24]uint8 // Custom Bytes.
}

// MsgID (generated function)
func (m *LedControl) MsgID() mavlink.MessageID {
	return MSG_ID_LED_CONTROL
}

// String (generated function)
func (m *LedControl) String() string {
	return fmt.Sprintf(
		"&ardupilotmega.LedControl{ TargetSystem: %+v, TargetComponent: %+v, Instance: %+v, Pattern: %+v, CustomLen: %+v, CustomBytes: %0X (\"%s\") }",
		m.TargetSystem,
		m.TargetComponent,
		m.Instance,
		m.Pattern,
		m.CustomLen,
		m.CustomBytes, string(m.CustomBytes[:]),
	)
}

// Pack (generated function)
func (m *LedControl) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 29)
	payload[0] = byte(m.TargetSystem)
	payload[1] = byte(m.TargetComponent)
	payload[2] = byte(m.Instance)
	payload[3] = byte(m.Pattern)
	payload[4] = byte(m.CustomLen)
	copy(payload[5:], m.CustomBytes[:])
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *LedControl) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 29 {
		return mavlink.ErrPayloadTooSmall
	}
	m.TargetSystem = uint8(payload[0])
	m.TargetComponent = uint8(payload[1])
	m.Instance = uint8(payload[2])
	m.Pattern = uint8(payload[3])
	m.CustomLen = uint8(payload[4])
	copy(m.CustomBytes[:], payload[5:29])
	return nil
}

// MagCalProgress struct (generated typeinfo)
// Reports progress of compass calibration.
type MagCalProgress struct {
	DirectionX     float32   // Body frame direction vector for display.
	DirectionY     float32   // Body frame direction vector for display.
	DirectionZ     float32   // Body frame direction vector for display.
	CompassID      uint8     // Compass being calibrated.
	CalMask        uint8     // Bitmask of compasses being calibrated.
	CalStatus      uint8     // Calibration Status.
	Attempt        uint8     // Attempt number.
	CompletionPct  uint8     // Completion percentage.
	CompletionMask [10]uint8 // Bitmask of sphere sections (see http://en.wikipedia.org/wiki/Geodesic_grid).
}

// MsgID (generated function)
func (m *MagCalProgress) MsgID() mavlink.MessageID {
	return MSG_ID_MAG_CAL_PROGRESS
}

// String (generated function)
func (m *MagCalProgress) String() string {
	return fmt.Sprintf(
		"&ardupilotmega.MagCalProgress{ DirectionX: %+v, DirectionY: %+v, DirectionZ: %+v, CompassID: %+v, CalMask: %+v, CalStatus: %+v, Attempt: %+v, CompletionPct: %+v, CompletionMask: %0X (\"%s\") }",
		m.DirectionX,
		m.DirectionY,
		m.DirectionZ,
		m.CompassID,
		m.CalMask,
		m.CalStatus,
		m.Attempt,
		m.CompletionPct,
		m.CompletionMask, string(m.CompletionMask[:]),
	)
}

// Pack (generated function)
func (m *MagCalProgress) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 27)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(m.DirectionX))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(m.DirectionY))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.DirectionZ))
	payload[12] = byte(m.CompassID)
	payload[13] = byte(m.CalMask)
	payload[14] = byte(m.CalStatus)
	payload[15] = byte(m.Attempt)
	payload[16] = byte(m.CompletionPct)
	copy(payload[17:], m.CompletionMask[:])
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *MagCalProgress) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 27 {
		return mavlink.ErrPayloadTooSmall
	}
	m.DirectionX = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.DirectionY = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	m.DirectionZ = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.CompassID = uint8(payload[12])
	m.CalMask = uint8(payload[13])
	m.CalStatus = uint8(payload[14])
	m.Attempt = uint8(payload[15])
	m.CompletionPct = uint8(payload[16])
	copy(m.CompletionMask[:], payload[17:27])
	return nil
}

// EkfStatusReport struct (generated typeinfo)
// EKF Status message including flags and variances.
type EkfStatusReport struct {
	VelocityVariance   float32 // Velocity variance.
	PosHorizVariance   float32 // Horizontal Position variance.
	PosVertVariance    float32 // Vertical Position variance.
	CompassVariance    float32 // Compass variance.
	TerrainAltVariance float32 // Terrain Altitude variance.
	Flags              uint16  // Flags.
}

// MsgID (generated function)
func (m *EkfStatusReport) MsgID() mavlink.MessageID {
	return MSG_ID_EKF_STATUS_REPORT
}

// String (generated function)
func (m *EkfStatusReport) String() string {
	return fmt.Sprintf(
		"&ardupilotmega.EkfStatusReport{ VelocityVariance: %+v, PosHorizVariance: %+v, PosVertVariance: %+v, CompassVariance: %+v, TerrainAltVariance: %+v, Flags: %+v }",
		m.VelocityVariance,
		m.PosHorizVariance,
		m.PosVertVariance,
		m.CompassVariance,
		m.TerrainAltVariance,
		m.Flags,
	)
}

// Pack (generated function)
func (m *EkfStatusReport) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 22)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(m.VelocityVariance))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(m.PosHorizVariance))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.PosVertVariance))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.CompassVariance))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(m.TerrainAltVariance))
	binary.LittleEndian.PutUint16(payload[20:], uint16(m.Flags))
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *EkfStatusReport) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 22 {
		return mavlink.ErrPayloadTooSmall
	}
	m.VelocityVariance = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.PosHorizVariance = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	m.PosVertVariance = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.CompassVariance = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.TerrainAltVariance = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	m.Flags = uint16(binary.LittleEndian.Uint16(payload[20:]))
	return nil
}

// PidTuning struct (generated typeinfo)
// PID tuning information.
type PidTuning struct {
	Desired  float32 // Desired rate.
	Achieved float32 // Achieved rate.
	Ff       float32 // FF component.
	P        float32 // P component.
	I        float32 // I component.
	D        float32 // D component.
	Axis     uint8   // Axis.
}

// MsgID (generated function)
func (m *PidTuning) MsgID() mavlink.MessageID {
	return MSG_ID_PID_TUNING
}

// String (generated function)
func (m *PidTuning) String() string {
	return fmt.Sprintf(
		"&ardupilotmega.PidTuning{ Desired: %+v, Achieved: %+v, Ff: %+v, P: %+v, I: %+v, D: %+v, Axis: %+v }",
		m.Desired,
		m.Achieved,
		m.Ff,
		m.P,
		m.I,
		m.D,
		m.Axis,
	)
}

// Pack (generated function)
func (m *PidTuning) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 25)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(m.Desired))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(m.Achieved))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.Ff))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.P))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(m.I))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(m.D))
	payload[24] = byte(m.Axis)
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *PidTuning) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 25 {
		return mavlink.ErrPayloadTooSmall
	}
	m.Desired = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.Achieved = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	m.Ff = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.P = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.I = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	m.D = math.Float32frombits(binary.LittleEndian.Uint32(payload[20:]))
	m.Axis = uint8(payload[24])
	return nil
}

// Deepstall struct (generated typeinfo)
// Deepstall path planning.
type Deepstall struct {
	LandingLat             int32   // Landing latitude.
	LandingLon             int32   // Landing longitude.
	PathLat                int32   // Final heading start point, latitude.
	PathLon                int32   // Final heading start point, longitude.
	ArcEntryLat            int32   // Arc entry point, latitude.
	ArcEntryLon            int32   // Arc entry point, longitude.
	Altitude               float32 // Altitude.
	ExpectedTravelDistance float32 // Distance the aircraft expects to travel during the deepstall.
	CrossTrackError        float32 // Deepstall cross track error (only valid when in DEEPSTALL_STAGE_LAND).
	Stage                  uint8   // Deepstall stage.
}

// MsgID (generated function)
func (m *Deepstall) MsgID() mavlink.MessageID {
	return MSG_ID_DEEPSTALL
}

// String (generated function)
func (m *Deepstall) String() string {
	return fmt.Sprintf(
		"&ardupilotmega.Deepstall{ LandingLat: %+v, LandingLon: %+v, PathLat: %+v, PathLon: %+v, ArcEntryLat: %+v, ArcEntryLon: %+v, Altitude: %+v, ExpectedTravelDistance: %+v, CrossTrackError: %+v, Stage: %+v }",
		m.LandingLat,
		m.LandingLon,
		m.PathLat,
		m.PathLon,
		m.ArcEntryLat,
		m.ArcEntryLon,
		m.Altitude,
		m.ExpectedTravelDistance,
		m.CrossTrackError,
		m.Stage,
	)
}

// Pack (generated function)
func (m *Deepstall) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 37)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.LandingLat))
	binary.LittleEndian.PutUint32(payload[4:], uint32(m.LandingLon))
	binary.LittleEndian.PutUint32(payload[8:], uint32(m.PathLat))
	binary.LittleEndian.PutUint32(payload[12:], uint32(m.PathLon))
	binary.LittleEndian.PutUint32(payload[16:], uint32(m.ArcEntryLat))
	binary.LittleEndian.PutUint32(payload[20:], uint32(m.ArcEntryLon))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(m.Altitude))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(m.ExpectedTravelDistance))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(m.CrossTrackError))
	payload[36] = byte(m.Stage)
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *Deepstall) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 37 {
		return mavlink.ErrPayloadTooSmall
	}
	m.LandingLat = int32(binary.LittleEndian.Uint32(payload[0:]))
	m.LandingLon = int32(binary.LittleEndian.Uint32(payload[4:]))
	m.PathLat = int32(binary.LittleEndian.Uint32(payload[8:]))
	m.PathLon = int32(binary.LittleEndian.Uint32(payload[12:]))
	m.ArcEntryLat = int32(binary.LittleEndian.Uint32(payload[16:]))
	m.ArcEntryLon = int32(binary.LittleEndian.Uint32(payload[20:]))
	m.Altitude = math.Float32frombits(binary.LittleEndian.Uint32(payload[24:]))
	m.ExpectedTravelDistance = math.Float32frombits(binary.LittleEndian.Uint32(payload[28:]))
	m.CrossTrackError = math.Float32frombits(binary.LittleEndian.Uint32(payload[32:]))
	m.Stage = uint8(payload[36])
	return nil
}

// GimbalReport struct (generated typeinfo)
// 3 axis gimbal measurements.
type GimbalReport struct {
	DeltaTime       float32 // Time since last update.
	DeltaAngleX     float32 // Delta angle X.
	DeltaAngleY     float32 // Delta angle Y.
	DeltaAngleZ     float32 // Delta angle X.
	DeltaVelocityX  float32 // Delta velocity X.
	DeltaVelocityY  float32 // Delta velocity Y.
	DeltaVelocityZ  float32 // Delta velocity Z.
	JointRoll       float32 // Joint ROLL.
	JointEl         float32 // Joint EL.
	JointAz         float32 // Joint AZ.
	TargetSystem    uint8   // System ID.
	TargetComponent uint8   // Component ID.
}

// MsgID (generated function)
func (m *GimbalReport) MsgID() mavlink.MessageID {
	return MSG_ID_GIMBAL_REPORT
}

// String (generated function)
func (m *GimbalReport) String() string {
	return fmt.Sprintf(
		"&ardupilotmega.GimbalReport{ DeltaTime: %+v, DeltaAngleX: %+v, DeltaAngleY: %+v, DeltaAngleZ: %+v, DeltaVelocityX: %+v, DeltaVelocityY: %+v, DeltaVelocityZ: %+v, JointRoll: %+v, JointEl: %+v, JointAz: %+v, TargetSystem: %+v, TargetComponent: %+v }",
		m.DeltaTime,
		m.DeltaAngleX,
		m.DeltaAngleY,
		m.DeltaAngleZ,
		m.DeltaVelocityX,
		m.DeltaVelocityY,
		m.DeltaVelocityZ,
		m.JointRoll,
		m.JointEl,
		m.JointAz,
		m.TargetSystem,
		m.TargetComponent,
	)
}

// Pack (generated function)
func (m *GimbalReport) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 42)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(m.DeltaTime))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(m.DeltaAngleX))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.DeltaAngleY))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.DeltaAngleZ))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(m.DeltaVelocityX))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(m.DeltaVelocityY))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(m.DeltaVelocityZ))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(m.JointRoll))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(m.JointEl))
	binary.LittleEndian.PutUint32(payload[36:], math.Float32bits(m.JointAz))
	payload[40] = byte(m.TargetSystem)
	payload[41] = byte(m.TargetComponent)
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *GimbalReport) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 42 {
		return mavlink.ErrPayloadTooSmall
	}
	m.DeltaTime = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.DeltaAngleX = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	m.DeltaAngleY = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.DeltaAngleZ = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.DeltaVelocityX = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	m.DeltaVelocityY = math.Float32frombits(binary.LittleEndian.Uint32(payload[20:]))
	m.DeltaVelocityZ = math.Float32frombits(binary.LittleEndian.Uint32(payload[24:]))
	m.JointRoll = math.Float32frombits(binary.LittleEndian.Uint32(payload[28:]))
	m.JointEl = math.Float32frombits(binary.LittleEndian.Uint32(payload[32:]))
	m.JointAz = math.Float32frombits(binary.LittleEndian.Uint32(payload[36:]))
	m.TargetSystem = uint8(payload[40])
	m.TargetComponent = uint8(payload[41])
	return nil
}

// GimbalControl struct (generated typeinfo)
// Control message for rate gimbal.
type GimbalControl struct {
	DemandedRateX   float32 // Demanded angular rate X.
	DemandedRateY   float32 // Demanded angular rate Y.
	DemandedRateZ   float32 // Demanded angular rate Z.
	TargetSystem    uint8   // System ID.
	TargetComponent uint8   // Component ID.
}

// MsgID (generated function)
func (m *GimbalControl) MsgID() mavlink.MessageID {
	return MSG_ID_GIMBAL_CONTROL
}

// String (generated function)
func (m *GimbalControl) String() string {
	return fmt.Sprintf(
		"&ardupilotmega.GimbalControl{ DemandedRateX: %+v, DemandedRateY: %+v, DemandedRateZ: %+v, TargetSystem: %+v, TargetComponent: %+v }",
		m.DemandedRateX,
		m.DemandedRateY,
		m.DemandedRateZ,
		m.TargetSystem,
		m.TargetComponent,
	)
}

// Pack (generated function)
func (m *GimbalControl) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 14)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(m.DemandedRateX))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(m.DemandedRateY))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.DemandedRateZ))
	payload[12] = byte(m.TargetSystem)
	payload[13] = byte(m.TargetComponent)
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *GimbalControl) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 14 {
		return mavlink.ErrPayloadTooSmall
	}
	m.DemandedRateX = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.DemandedRateY = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	m.DemandedRateZ = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.TargetSystem = uint8(payload[12])
	m.TargetComponent = uint8(payload[13])
	return nil
}

// GimbalTorqueCmdReport struct (generated typeinfo)
// 100 Hz gimbal torque command telemetry.
type GimbalTorqueCmdReport struct {
	RlTorqueCmd     int16 // Roll Torque Command.
	ElTorqueCmd     int16 // Elevation Torque Command.
	AzTorqueCmd     int16 // Azimuth Torque Command.
	TargetSystem    uint8 // System ID.
	TargetComponent uint8 // Component ID.
}

// MsgID (generated function)
func (m *GimbalTorqueCmdReport) MsgID() mavlink.MessageID {
	return MSG_ID_GIMBAL_TORQUE_CMD_REPORT
}

// String (generated function)
func (m *GimbalTorqueCmdReport) String() string {
	return fmt.Sprintf(
		"&ardupilotmega.GimbalTorqueCmdReport{ RlTorqueCmd: %+v, ElTorqueCmd: %+v, AzTorqueCmd: %+v, TargetSystem: %+v, TargetComponent: %+v }",
		m.RlTorqueCmd,
		m.ElTorqueCmd,
		m.AzTorqueCmd,
		m.TargetSystem,
		m.TargetComponent,
	)
}

// Pack (generated function)
func (m *GimbalTorqueCmdReport) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 8)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.RlTorqueCmd))
	binary.LittleEndian.PutUint16(payload[2:], uint16(m.ElTorqueCmd))
	binary.LittleEndian.PutUint16(payload[4:], uint16(m.AzTorqueCmd))
	payload[6] = byte(m.TargetSystem)
	payload[7] = byte(m.TargetComponent)
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *GimbalTorqueCmdReport) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 8 {
		return mavlink.ErrPayloadTooSmall
	}
	m.RlTorqueCmd = int16(binary.LittleEndian.Uint16(payload[0:]))
	m.ElTorqueCmd = int16(binary.LittleEndian.Uint16(payload[2:]))
	m.AzTorqueCmd = int16(binary.LittleEndian.Uint16(payload[4:]))
	m.TargetSystem = uint8(payload[6])
	m.TargetComponent = uint8(payload[7])
	return nil
}

// GoproHeartbeat struct (generated typeinfo)
// Heartbeat from a HeroBus attached GoPro.
type GoproHeartbeat struct {
	Status      uint8 // Status.
	CaptureMode uint8 // Current capture mode.
	Flags       uint8 // Additional status bits.
}

// MsgID (generated function)
func (m *GoproHeartbeat) MsgID() mavlink.MessageID {
	return MSG_ID_GOPRO_HEARTBEAT
}

// String (generated function)
func (m *GoproHeartbeat) String() string {
	return fmt.Sprintf(
		"&ardupilotmega.GoproHeartbeat{ Status: %+v, CaptureMode: %+v, Flags: %+v }",
		m.Status,
		m.CaptureMode,
		m.Flags,
	)
}

// Pack (generated function)
func (m *GoproHeartbeat) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 3)
	payload[0] = byte(m.Status)
	payload[1] = byte(m.CaptureMode)
	payload[2] = byte(m.Flags)
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *GoproHeartbeat) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 3 {
		return mavlink.ErrPayloadTooSmall
	}
	m.Status = uint8(payload[0])
	m.CaptureMode = uint8(payload[1])
	m.Flags = uint8(payload[2])
	return nil
}

// GoproGetRequest struct (generated typeinfo)
// Request a GOPRO_COMMAND response from the GoPro.
type GoproGetRequest struct {
	TargetSystem    uint8 // System ID.
	TargetComponent uint8 // Component ID.
	CmdID           uint8 // Command ID.
}

// MsgID (generated function)
func (m *GoproGetRequest) MsgID() mavlink.MessageID {
	return MSG_ID_GOPRO_GET_REQUEST
}

// String (generated function)
func (m *GoproGetRequest) String() string {
	return fmt.Sprintf(
		"&ardupilotmega.GoproGetRequest{ TargetSystem: %+v, TargetComponent: %+v, CmdID: %+v }",
		m.TargetSystem,
		m.TargetComponent,
		m.CmdID,
	)
}

// Pack (generated function)
func (m *GoproGetRequest) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 3)
	payload[0] = byte(m.TargetSystem)
	payload[1] = byte(m.TargetComponent)
	payload[2] = byte(m.CmdID)
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *GoproGetRequest) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 3 {
		return mavlink.ErrPayloadTooSmall
	}
	m.TargetSystem = uint8(payload[0])
	m.TargetComponent = uint8(payload[1])
	m.CmdID = uint8(payload[2])
	return nil
}

// GoproGetResponse struct (generated typeinfo)
// Response from a GOPRO_COMMAND get request.
type GoproGetResponse struct {
	CmdID  uint8    // Command ID.
	Status uint8    // Status.
	Value  [4]uint8 // Value.
}

// MsgID (generated function)
func (m *GoproGetResponse) MsgID() mavlink.MessageID {
	return MSG_ID_GOPRO_GET_RESPONSE
}

// String (generated function)
func (m *GoproGetResponse) String() string {
	return fmt.Sprintf(
		"&ardupilotmega.GoproGetResponse{ CmdID: %+v, Status: %+v, Value: %0X (\"%s\") }",
		m.CmdID,
		m.Status,
		m.Value, string(m.Value[:]),
	)
}

// Pack (generated function)
func (m *GoproGetResponse) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 6)
	payload[0] = byte(m.CmdID)
	payload[1] = byte(m.Status)
	copy(payload[2:], m.Value[:])
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *GoproGetResponse) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 6 {
		return mavlink.ErrPayloadTooSmall
	}
	m.CmdID = uint8(payload[0])
	m.Status = uint8(payload[1])
	copy(m.Value[:], payload[2:6])
	return nil
}

// GoproSetRequest struct (generated typeinfo)
// Request to set a GOPRO_COMMAND with a desired.
type GoproSetRequest struct {
	TargetSystem    uint8    // System ID.
	TargetComponent uint8    // Component ID.
	CmdID           uint8    // Command ID.
	Value           [4]uint8 // Value.
}

// MsgID (generated function)
func (m *GoproSetRequest) MsgID() mavlink.MessageID {
	return MSG_ID_GOPRO_SET_REQUEST
}

// String (generated function)
func (m *GoproSetRequest) String() string {
	return fmt.Sprintf(
		"&ardupilotmega.GoproSetRequest{ TargetSystem: %+v, TargetComponent: %+v, CmdID: %+v, Value: %0X (\"%s\") }",
		m.TargetSystem,
		m.TargetComponent,
		m.CmdID,
		m.Value, string(m.Value[:]),
	)
}

// Pack (generated function)
func (m *GoproSetRequest) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 7)
	payload[0] = byte(m.TargetSystem)
	payload[1] = byte(m.TargetComponent)
	payload[2] = byte(m.CmdID)
	copy(payload[3:], m.Value[:])
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *GoproSetRequest) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 7 {
		return mavlink.ErrPayloadTooSmall
	}
	m.TargetSystem = uint8(payload[0])
	m.TargetComponent = uint8(payload[1])
	m.CmdID = uint8(payload[2])
	copy(m.Value[:], payload[3:7])
	return nil
}

// GoproSetResponse struct (generated typeinfo)
// Response from a GOPRO_COMMAND set request.
type GoproSetResponse struct {
	CmdID  uint8 // Command ID.
	Status uint8 // Status.
}

// MsgID (generated function)
func (m *GoproSetResponse) MsgID() mavlink.MessageID {
	return MSG_ID_GOPRO_SET_RESPONSE
}

// String (generated function)
func (m *GoproSetResponse) String() string {
	return fmt.Sprintf(
		"&ardupilotmega.GoproSetResponse{ CmdID: %+v, Status: %+v }",
		m.CmdID,
		m.Status,
	)
}

// Pack (generated function)
func (m *GoproSetResponse) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 2)
	payload[0] = byte(m.CmdID)
	payload[1] = byte(m.Status)
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *GoproSetResponse) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 2 {
		return mavlink.ErrPayloadTooSmall
	}
	m.CmdID = uint8(payload[0])
	m.Status = uint8(payload[1])
	return nil
}

// Rpm struct (generated typeinfo)
// RPM sensor output.
type Rpm struct {
	Rpm1 float32 // RPM Sensor1.
	Rpm2 float32 // RPM Sensor2.
}

// MsgID (generated function)
func (m *Rpm) MsgID() mavlink.MessageID {
	return MSG_ID_RPM
}

// String (generated function)
func (m *Rpm) String() string {
	return fmt.Sprintf(
		"&ardupilotmega.Rpm{ Rpm1: %+v, Rpm2: %+v }",
		m.Rpm1,
		m.Rpm2,
	)
}

// Pack (generated function)
func (m *Rpm) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 8)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(m.Rpm1))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(m.Rpm2))
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *Rpm) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 8 {
		return mavlink.ErrPayloadTooSmall
	}
	m.Rpm1 = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.Rpm2 = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	return nil
}

// Message IDs
const (
	MSG_ID_SENSOR_OFFSETS            mavlink.MessageID = 150
	MSG_ID_SET_MAG_OFFSETS           mavlink.MessageID = 151
	MSG_ID_MEMINFO                   mavlink.MessageID = 152
	MSG_ID_AP_ADC                    mavlink.MessageID = 153
	MSG_ID_DIGICAM_CONFIGURE         mavlink.MessageID = 154
	MSG_ID_DIGICAM_CONTROL           mavlink.MessageID = 155
	MSG_ID_MOUNT_CONFIGURE           mavlink.MessageID = 156
	MSG_ID_MOUNT_CONTROL             mavlink.MessageID = 157
	MSG_ID_MOUNT_STATUS              mavlink.MessageID = 158
	MSG_ID_FENCE_POINT               mavlink.MessageID = 160
	MSG_ID_FENCE_FETCH_POINT         mavlink.MessageID = 161
	MSG_ID_AHRS                      mavlink.MessageID = 163
	MSG_ID_SIMSTATE                  mavlink.MessageID = 164
	MSG_ID_HWSTATUS                  mavlink.MessageID = 165
	MSG_ID_RADIO                     mavlink.MessageID = 166
	MSG_ID_LIMITS_STATUS             mavlink.MessageID = 167
	MSG_ID_WIND                      mavlink.MessageID = 168
	MSG_ID_DATA16                    mavlink.MessageID = 169
	MSG_ID_DATA32                    mavlink.MessageID = 170
	MSG_ID_DATA64                    mavlink.MessageID = 171
	MSG_ID_DATA96                    mavlink.MessageID = 172
	MSG_ID_RANGEFINDER               mavlink.MessageID = 173
	MSG_ID_AIRSPEED_AUTOCAL          mavlink.MessageID = 174
	MSG_ID_RALLY_POINT               mavlink.MessageID = 175
	MSG_ID_RALLY_FETCH_POINT         mavlink.MessageID = 176
	MSG_ID_COMPASSMOT_STATUS         mavlink.MessageID = 177
	MSG_ID_AHRS2                     mavlink.MessageID = 178
	MSG_ID_CAMERA_STATUS             mavlink.MessageID = 179
	MSG_ID_CAMERA_FEEDBACK           mavlink.MessageID = 180
	MSG_ID_BATTERY2                  mavlink.MessageID = 181
	MSG_ID_AHRS3                     mavlink.MessageID = 182
	MSG_ID_AUTOPILOT_VERSION_REQUEST mavlink.MessageID = 183
	MSG_ID_REMOTE_LOG_DATA_BLOCK     mavlink.MessageID = 184
	MSG_ID_REMOTE_LOG_BLOCK_STATUS   mavlink.MessageID = 185
	MSG_ID_LED_CONTROL               mavlink.MessageID = 186
	MSG_ID_MAG_CAL_PROGRESS          mavlink.MessageID = 191
	MSG_ID_EKF_STATUS_REPORT         mavlink.MessageID = 193
	MSG_ID_PID_TUNING                mavlink.MessageID = 194
	MSG_ID_DEEPSTALL                 mavlink.MessageID = 195
	MSG_ID_GIMBAL_REPORT             mavlink.MessageID = 200
	MSG_ID_GIMBAL_CONTROL            mavlink.MessageID = 201
	MSG_ID_GIMBAL_TORQUE_CMD_REPORT  mavlink.MessageID = 214
	MSG_ID_GOPRO_HEARTBEAT           mavlink.MessageID = 215
	MSG_ID_GOPRO_GET_REQUEST         mavlink.MessageID = 216
	MSG_ID_GOPRO_GET_RESPONSE        mavlink.MessageID = 217
	MSG_ID_GOPRO_SET_REQUEST         mavlink.MessageID = 218
	MSG_ID_GOPRO_SET_RESPONSE        mavlink.MessageID = 219
	MSG_ID_RPM                       mavlink.MessageID = 226
)

// init Ardupilotmega dialect
func init() {
	mavlink.Register(MSG_ID_SENSOR_OFFSETS, "MSG_ID_SENSOR_OFFSETS", 134, func(p *mavlink.Packet) mavlink.Message {
		msg := new(SensorOffsets)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_SET_MAG_OFFSETS, "MSG_ID_SET_MAG_OFFSETS", 219, func(p *mavlink.Packet) mavlink.Message {
		msg := new(SetMagOffsets)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_MEMINFO, "MSG_ID_MEMINFO", 208, func(p *mavlink.Packet) mavlink.Message {
		msg := new(Meminfo)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_AP_ADC, "MSG_ID_AP_ADC", 188, func(p *mavlink.Packet) mavlink.Message {
		msg := new(ApAdc)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_DIGICAM_CONFIGURE, "MSG_ID_DIGICAM_CONFIGURE", 84, func(p *mavlink.Packet) mavlink.Message {
		msg := new(DigicamConfigure)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_DIGICAM_CONTROL, "MSG_ID_DIGICAM_CONTROL", 22, func(p *mavlink.Packet) mavlink.Message {
		msg := new(DigicamControl)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_MOUNT_CONFIGURE, "MSG_ID_MOUNT_CONFIGURE", 19, func(p *mavlink.Packet) mavlink.Message {
		msg := new(MountConfigure)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_MOUNT_CONTROL, "MSG_ID_MOUNT_CONTROL", 21, func(p *mavlink.Packet) mavlink.Message {
		msg := new(MountControl)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_MOUNT_STATUS, "MSG_ID_MOUNT_STATUS", 134, func(p *mavlink.Packet) mavlink.Message {
		msg := new(MountStatus)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_FENCE_POINT, "MSG_ID_FENCE_POINT", 78, func(p *mavlink.Packet) mavlink.Message {
		msg := new(FencePoint)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_FENCE_FETCH_POINT, "MSG_ID_FENCE_FETCH_POINT", 68, func(p *mavlink.Packet) mavlink.Message {
		msg := new(FenceFetchPoint)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_AHRS, "MSG_ID_AHRS", 127, func(p *mavlink.Packet) mavlink.Message {
		msg := new(Ahrs)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_SIMSTATE, "MSG_ID_SIMSTATE", 154, func(p *mavlink.Packet) mavlink.Message {
		msg := new(Simstate)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_HWSTATUS, "MSG_ID_HWSTATUS", 21, func(p *mavlink.Packet) mavlink.Message {
		msg := new(Hwstatus)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_RADIO, "MSG_ID_RADIO", 21, func(p *mavlink.Packet) mavlink.Message {
		msg := new(Radio)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_LIMITS_STATUS, "MSG_ID_LIMITS_STATUS", 144, func(p *mavlink.Packet) mavlink.Message {
		msg := new(LimitsStatus)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_WIND, "MSG_ID_WIND", 1, func(p *mavlink.Packet) mavlink.Message {
		msg := new(Wind)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_DATA16, "MSG_ID_DATA16", 234, func(p *mavlink.Packet) mavlink.Message {
		msg := new(Data16)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_DATA32, "MSG_ID_DATA32", 73, func(p *mavlink.Packet) mavlink.Message {
		msg := new(Data32)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_DATA64, "MSG_ID_DATA64", 181, func(p *mavlink.Packet) mavlink.Message {
		msg := new(Data64)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_DATA96, "MSG_ID_DATA96", 22, func(p *mavlink.Packet) mavlink.Message {
		msg := new(Data96)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_RANGEFINDER, "MSG_ID_RANGEFINDER", 83, func(p *mavlink.Packet) mavlink.Message {
		msg := new(Rangefinder)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_AIRSPEED_AUTOCAL, "MSG_ID_AIRSPEED_AUTOCAL", 167, func(p *mavlink.Packet) mavlink.Message {
		msg := new(AirspeedAutocal)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_RALLY_POINT, "MSG_ID_RALLY_POINT", 138, func(p *mavlink.Packet) mavlink.Message {
		msg := new(RallyPoint)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_RALLY_FETCH_POINT, "MSG_ID_RALLY_FETCH_POINT", 234, func(p *mavlink.Packet) mavlink.Message {
		msg := new(RallyFetchPoint)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_COMPASSMOT_STATUS, "MSG_ID_COMPASSMOT_STATUS", 240, func(p *mavlink.Packet) mavlink.Message {
		msg := new(CompassmotStatus)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_AHRS2, "MSG_ID_AHRS2", 47, func(p *mavlink.Packet) mavlink.Message {
		msg := new(Ahrs2)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_CAMERA_STATUS, "MSG_ID_CAMERA_STATUS", 189, func(p *mavlink.Packet) mavlink.Message {
		msg := new(CameraStatus)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_CAMERA_FEEDBACK, "MSG_ID_CAMERA_FEEDBACK", 52, func(p *mavlink.Packet) mavlink.Message {
		msg := new(CameraFeedback)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_BATTERY2, "MSG_ID_BATTERY2", 174, func(p *mavlink.Packet) mavlink.Message {
		msg := new(Battery2)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_AHRS3, "MSG_ID_AHRS3", 229, func(p *mavlink.Packet) mavlink.Message {
		msg := new(Ahrs3)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_AUTOPILOT_VERSION_REQUEST, "MSG_ID_AUTOPILOT_VERSION_REQUEST", 85, func(p *mavlink.Packet) mavlink.Message {
		msg := new(AutopilotVersionRequest)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_REMOTE_LOG_DATA_BLOCK, "MSG_ID_REMOTE_LOG_DATA_BLOCK", 159, func(p *mavlink.Packet) mavlink.Message {
		msg := new(RemoteLogDataBlock)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_REMOTE_LOG_BLOCK_STATUS, "MSG_ID_REMOTE_LOG_BLOCK_STATUS", 186, func(p *mavlink.Packet) mavlink.Message {
		msg := new(RemoteLogBlockStatus)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_LED_CONTROL, "MSG_ID_LED_CONTROL", 72, func(p *mavlink.Packet) mavlink.Message {
		msg := new(LedControl)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_MAG_CAL_PROGRESS, "MSG_ID_MAG_CAL_PROGRESS", 92, func(p *mavlink.Packet) mavlink.Message {
		msg := new(MagCalProgress)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_EKF_STATUS_REPORT, "MSG_ID_EKF_STATUS_REPORT", 71, func(p *mavlink.Packet) mavlink.Message {
		msg := new(EkfStatusReport)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_PID_TUNING, "MSG_ID_PID_TUNING", 98, func(p *mavlink.Packet) mavlink.Message {
		msg := new(PidTuning)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_DEEPSTALL, "MSG_ID_DEEPSTALL", 120, func(p *mavlink.Packet) mavlink.Message {
		msg := new(Deepstall)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_GIMBAL_REPORT, "MSG_ID_GIMBAL_REPORT", 134, func(p *mavlink.Packet) mavlink.Message {
		msg := new(GimbalReport)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_GIMBAL_CONTROL, "MSG_ID_GIMBAL_CONTROL", 205, func(p *mavlink.Packet) mavlink.Message {
		msg := new(GimbalControl)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_GIMBAL_TORQUE_CMD_REPORT, "MSG_ID_GIMBAL_TORQUE_CMD_REPORT", 69, func(p *mavlink.Packet) mavlink.Message {
		msg := new(GimbalTorqueCmdReport)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_GOPRO_HEARTBEAT, "MSG_ID_GOPRO_HEARTBEAT", 101, func(p *mavlink.Packet) mavlink.Message {
		msg := new(GoproHeartbeat)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_GOPRO_GET_REQUEST, "MSG_ID_GOPRO_GET_REQUEST", 50, func(p *mavlink.Packet) mavlink.Message {
		msg := new(GoproGetRequest)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_GOPRO_GET_RESPONSE, "MSG_ID_GOPRO_GET_RESPONSE", 202, func(p *mavlink.Packet) mavlink.Message {
		msg := new(GoproGetResponse)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_GOPRO_SET_REQUEST, "MSG_ID_GOPRO_SET_REQUEST", 17, func(p *mavlink.Packet) mavlink.Message {
		msg := new(GoproSetRequest)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_GOPRO_SET_RESPONSE, "MSG_ID_GOPRO_SET_RESPONSE", 162, func(p *mavlink.Packet) mavlink.Message {
		msg := new(GoproSetResponse)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_RPM, "MSG_ID_RPM", 207, func(p *mavlink.Packet) mavlink.Message {
		msg := new(Rpm)
		msg.Unpack(p)
		return msg
	})
}

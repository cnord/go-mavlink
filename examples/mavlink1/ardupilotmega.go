//////////////////////////////////////////////////
//
// NOTE: do not edit,
// this file created automatically by mavgen.go
//
//////////////////////////////////////////////////

package mavlink

import (
	"encoding/binary"
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

// MavAutopilot (generated enum)
// Micro air vehicle / autopilot classes. This identifies the individual model.
const (
	MAV_AUTOPILOT_GENERIC                                      = 0  // Generic autopilot, full support for everything
	MAV_AUTOPILOT_RESERVED                                     = 1  // Reserved for future use
	MAV_AUTOPILOT_SLUGS                                        = 2  // SLUGS autopilot, http://slugsuav.soe.ucsc.edu
	MAV_AUTOPILOT_ARDUPILOTMEGA                                = 3  // ArduPilot - Plane/Copter/Rover/Sub/Tracker, https://ardupilot.org
	MAV_AUTOPILOT_OPENPILOT                                    = 4  // OpenPilot, http://openpilot.org
	MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY                       = 5  // Generic autopilot only supporting simple waypoints
	MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY = 6  // Generic autopilot supporting waypoints and other simple navigation commands
	MAV_AUTOPILOT_GENERIC_MISSION_FULL                         = 7  // Generic autopilot supporting the full mission command set
	MAV_AUTOPILOT_INVALID                                      = 8  // No valid autopilot, e.g. a GCS or other MAVLink component
	MAV_AUTOPILOT_PPZ                                          = 9  // PPZ UAV - http://nongnu.org/paparazzi
	MAV_AUTOPILOT_UDB                                          = 10 // UAV Dev Board
	MAV_AUTOPILOT_FP                                           = 11 // FlexiPilot
	MAV_AUTOPILOT_PX4                                          = 12 // PX4 Autopilot - http://px4.io/
	MAV_AUTOPILOT_SMACCMPILOT                                  = 13 // SMACCMPilot - http://smaccmpilot.org
	MAV_AUTOPILOT_AUTOQUAD                                     = 14 // AutoQuad -- http://autoquad.org
	MAV_AUTOPILOT_ARMAZILA                                     = 15 // Armazila -- http://armazila.com
	MAV_AUTOPILOT_AEROB                                        = 16 // Aerob -- http://aerob.ru
	MAV_AUTOPILOT_ASLUAV                                       = 17 // ASLUAV autopilot -- http://www.asl.ethz.ch
	MAV_AUTOPILOT_SMARTAP                                      = 18 // SmartAP Autopilot - http://sky-drones.com
	MAV_AUTOPILOT_AIRRAILS                                     = 19 // AirRails - http://uaventure.com
)

// MavType (generated enum)
// MAVLINK component type reported in HEARTBEAT message. Flight controllers must report the type of the vehicle on which they are mounted (e.g. MAV_TYPE_OCTOROTOR). All other components must report a value appropriate for their type (e.g. a camera must use MAV_TYPE_CAMERA).
const (
	MAV_TYPE_GENERIC            = 0  // Generic micro air vehicle
	MAV_TYPE_FIXED_WING         = 1  // Fixed wing aircraft
	MAV_TYPE_QUADROTOR          = 2  // Quadrotor
	MAV_TYPE_COAXIAL            = 3  // Coaxial helicopter
	MAV_TYPE_HELICOPTER         = 4  // Normal helicopter with tail rotor
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
	MAV_TYPE_VTOL_DUOROTOR      = 19 // Two-rotor VTOL using control surfaces in vertical operation in addition. Tailsitter
	MAV_TYPE_VTOL_QUADROTOR     = 20 // Quad-rotor VTOL using a V-shaped quad config in vertical operation. Tailsitter
	MAV_TYPE_VTOL_TILTROTOR     = 21 // Tiltrotor VTOL
	MAV_TYPE_VTOL_RESERVED2     = 22 // VTOL reserved 2
	MAV_TYPE_VTOL_RESERVED3     = 23 // VTOL reserved 3
	MAV_TYPE_VTOL_RESERVED4     = 24 // VTOL reserved 4
	MAV_TYPE_VTOL_RESERVED5     = 25 // VTOL reserved 5
	MAV_TYPE_GIMBAL             = 26 // Gimbal
	MAV_TYPE_ADSB               = 27 // ADSB system
	MAV_TYPE_PARAFOIL           = 28 // Steerable, nonrigid airfoil
	MAV_TYPE_DODECAROTOR        = 29 // Dodecarotor
	MAV_TYPE_CAMERA             = 30 // Camera
	MAV_TYPE_CHARGING_STATION   = 31 // Charging station
	MAV_TYPE_FLARM              = 32 // FLARM collision avoidance system
	MAV_TYPE_SERVO              = 33 // Servo
	MAV_TYPE_ODID               = 34 // Open Drone ID. See https://mavlink.io/en/services/opendroneid.html
	MAV_TYPE_DECAROTOR          = 35 // Decarotor
)

// MavModeFlag (generated enum)
// These flags encode the MAV mode.
const (
	MAV_MODE_FLAG_SAFETY_ARMED         = 128 // 0b10000000 MAV safety set to armed. Motors are enabled / running / can start. Ready to fly. Additional note: this flag is to be ignore when sent in the command MAV_CMD_DO_SET_MODE and MAV_CMD_COMPONENT_ARM_DISARM shall be used instead. The flag can still be used to report the armed state
	MAV_MODE_FLAG_MANUAL_INPUT_ENABLED = 64  // 0b01000000 remote control input is enabled
	MAV_MODE_FLAG_HIL_ENABLED          = 32  // 0b00100000 hardware in the loop simulation. All motors / actuators are blocked, but internal software is full operational
	MAV_MODE_FLAG_STABILIZE_ENABLED    = 16  // 0b00010000 system stabilizes electronically its attitude (and optionally position). It needs however further control inputs to move around
	MAV_MODE_FLAG_GUIDED_ENABLED       = 8   // 0b00001000 guided mode enabled, system flies waypoints / mission items
	MAV_MODE_FLAG_AUTO_ENABLED         = 4   // 0b00000100 autonomous mode enabled, system finds its own goal positions. Guided flag can be set or not, depends on the actual implementation
	MAV_MODE_FLAG_TEST_ENABLED         = 2   // 0b00000010 system has a test mode enabled. This flag is intended for temporary system tests and should not be used for stable implementations
	MAV_MODE_FLAG_CUSTOM_MODE_ENABLED  = 1   // 0b00000001 Reserved for future use
)

// MavModeFlagDecodePosition (generated enum)
// These values encode the bit positions of the decode position. These values can be used to read the value of a flag bit by combining the base_mode variable with AND with the flag position value. The result will be either 0 or 1, depending on if the flag is set or not.
const (
	MAV_MODE_FLAG_DECODE_POSITION_SAFETY      = 128 // First bit:  10000000
	MAV_MODE_FLAG_DECODE_POSITION_MANUAL      = 64  // Second bit: 01000000
	MAV_MODE_FLAG_DECODE_POSITION_HIL         = 32  // Third bit:  00100000
	MAV_MODE_FLAG_DECODE_POSITION_STABILIZE   = 16  // Fourth bit: 00010000
	MAV_MODE_FLAG_DECODE_POSITION_GUIDED      = 8   // Fifth bit:  00001000
	MAV_MODE_FLAG_DECODE_POSITION_AUTO        = 4   // Sixth bit:   00000100
	MAV_MODE_FLAG_DECODE_POSITION_TEST        = 2   // Seventh bit: 00000010
	MAV_MODE_FLAG_DECODE_POSITION_CUSTOM_MODE = 1   // Eighth bit: 00000001
)

// MavState (generated enum)
//
const (
	MAV_STATE_UNINIT             = 0 // Uninitialized system, state is unknown
	MAV_STATE_BOOT               = 1 // System is booting up
	MAV_STATE_CALIBRATING        = 2 // System is calibrating and not flight-ready
	MAV_STATE_STANDBY            = 3 // System is grounded and on standby. It can be launched any time
	MAV_STATE_ACTIVE             = 4 // System is active and might be already airborne. Motors are engaged
	MAV_STATE_CRITICAL           = 5 // System is in a non-normal flight mode. It can however still navigate
	MAV_STATE_EMERGENCY          = 6 // System is in a non-normal flight mode. It lost control over parts or over the whole airframe. It is in mayday and going down
	MAV_STATE_POWEROFF           = 7 // System just initialized its power-down sequence, will shut down now
	MAV_STATE_FLIGHT_TERMINATION = 8 // System is terminating itself
)

// MavComponent (generated enum)
// Component ids (values) for the different types and instances of onboard hardware/software that might make up a MAVLink system (autopilot, cameras, servos, GPS systems, avoidance systems etc.).       Components must use the appropriate ID in their source address when sending messages. Components can also use IDs to determine if they are the intended recipient of an incoming message. The MAV_COMP_ID_ALL value is used to indicate messages that must be processed by all components.       When creating new entries, components that can have multiple instances (e.g. cameras, servos etc.) should be allocated sequential values. An appropriate number of values should be left free after these components to allow the number of instances to be expanded.
const (
	MAV_COMP_ID_ALL                      = 0   // Target id (target_component) used to broadcast messages to all components of the receiving system. Components should attempt to process messages with this component ID and forward to components on any other interfaces. Note: This is not a valid *source* component id for a message
	MAV_COMP_ID_AUTOPILOT1               = 1   // System flight controller component ("autopilot"). Only one autopilot is expected in a particular system
	MAV_COMP_ID_USER1                    = 25  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER2                    = 26  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER3                    = 27  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER4                    = 28  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER5                    = 29  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER6                    = 30  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER7                    = 31  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER8                    = 32  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER9                    = 33  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER10                   = 34  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER11                   = 35  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER12                   = 36  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER13                   = 37  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER14                   = 38  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER15                   = 39  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER16                   = 40  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER17                   = 41  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER18                   = 42  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER19                   = 43  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER20                   = 44  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER21                   = 45  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER22                   = 46  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER23                   = 47  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER24                   = 48  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER25                   = 49  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER26                   = 50  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER27                   = 51  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER28                   = 52  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER29                   = 53  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER30                   = 54  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER31                   = 55  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER32                   = 56  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER33                   = 57  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER34                   = 58  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER35                   = 59  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER36                   = 60  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER37                   = 61  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER38                   = 62  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER39                   = 63  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER40                   = 64  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER41                   = 65  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER42                   = 66  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER43                   = 67  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_TELEMETRY_RADIO          = 68  // Telemetry radio (e.g. SiK radio, or other component that emits RADIO_STATUS messages)
	MAV_COMP_ID_USER45                   = 69  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER46                   = 70  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER47                   = 71  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER48                   = 72  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER49                   = 73  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER50                   = 74  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER51                   = 75  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER52                   = 76  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER53                   = 77  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER54                   = 78  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER55                   = 79  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER56                   = 80  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER57                   = 81  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER58                   = 82  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER59                   = 83  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER60                   = 84  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER61                   = 85  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER62                   = 86  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER63                   = 87  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER64                   = 88  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER65                   = 89  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER66                   = 90  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER67                   = 91  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER68                   = 92  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER69                   = 93  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER70                   = 94  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER71                   = 95  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER72                   = 96  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER73                   = 97  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER74                   = 98  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_USER75                   = 99  // Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network
	MAV_COMP_ID_CAMERA                   = 100 // Camera #1
	MAV_COMP_ID_CAMERA2                  = 101 // Camera #2
	MAV_COMP_ID_CAMERA3                  = 102 // Camera #3
	MAV_COMP_ID_CAMERA4                  = 103 // Camera #4
	MAV_COMP_ID_CAMERA5                  = 104 // Camera #5
	MAV_COMP_ID_CAMERA6                  = 105 // Camera #6
	MAV_COMP_ID_SERVO1                   = 140 // Servo #1
	MAV_COMP_ID_SERVO2                   = 141 // Servo #2
	MAV_COMP_ID_SERVO3                   = 142 // Servo #3
	MAV_COMP_ID_SERVO4                   = 143 // Servo #4
	MAV_COMP_ID_SERVO5                   = 144 // Servo #5
	MAV_COMP_ID_SERVO6                   = 145 // Servo #6
	MAV_COMP_ID_SERVO7                   = 146 // Servo #7
	MAV_COMP_ID_SERVO8                   = 147 // Servo #8
	MAV_COMP_ID_SERVO9                   = 148 // Servo #9
	MAV_COMP_ID_SERVO10                  = 149 // Servo #10
	MAV_COMP_ID_SERVO11                  = 150 // Servo #11
	MAV_COMP_ID_SERVO12                  = 151 // Servo #12
	MAV_COMP_ID_SERVO13                  = 152 // Servo #13
	MAV_COMP_ID_SERVO14                  = 153 // Servo #14
	MAV_COMP_ID_GIMBAL                   = 154 // Gimbal #1
	MAV_COMP_ID_LOG                      = 155 // Logging component
	MAV_COMP_ID_ADSB                     = 156 // Automatic Dependent Surveillance-Broadcast (ADS-B) component
	MAV_COMP_ID_OSD                      = 157 // On Screen Display (OSD) devices for video links
	MAV_COMP_ID_PERIPHERAL               = 158 // Generic autopilot peripheral component ID. Meant for devices that do not implement the parameter microservice
	MAV_COMP_ID_QX1_GIMBAL               = 159 // Gimbal ID for QX1
	MAV_COMP_ID_FLARM                    = 160 // FLARM collision alert component
	MAV_COMP_ID_GIMBAL2                  = 171 // Gimbal #2
	MAV_COMP_ID_GIMBAL3                  = 172 // Gimbal #3
	MAV_COMP_ID_GIMBAL4                  = 173 // Gimbal #4
	MAV_COMP_ID_GIMBAL5                  = 174 // Gimbal #5
	MAV_COMP_ID_GIMBAL6                  = 175 // Gimbal #6
	MAV_COMP_ID_MISSIONPLANNER           = 190 // Component that can generate/supply a mission flight plan (e.g. GCS or developer API)
	MAV_COMP_ID_ONBOARD_COMPUTER         = 191 // Component that lives on the onboard computer (companion computer) and has some generic functionalities, such as settings system parameters and monitoring the status of some processes that don't directly speak mavlink and so on
	MAV_COMP_ID_PATHPLANNER              = 195 // Component that finds an optimal path between points based on a certain constraint (e.g. minimum snap, shortest path, cost, etc.)
	MAV_COMP_ID_OBSTACLE_AVOIDANCE       = 196 // Component that plans a collision free path between two points
	MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY = 197 // Component that provides position estimates using VIO techniques
	MAV_COMP_ID_PAIRING_MANAGER          = 198 // Component that manages pairing of vehicle and GCS
	MAV_COMP_ID_IMU                      = 200 // Inertial Measurement Unit (IMU) #1
	MAV_COMP_ID_IMU_2                    = 201 // Inertial Measurement Unit (IMU) #2
	MAV_COMP_ID_IMU_3                    = 202 // Inertial Measurement Unit (IMU) #3
	MAV_COMP_ID_GPS                      = 220 // GPS #1
	MAV_COMP_ID_GPS2                     = 221 // GPS #2
	MAV_COMP_ID_ODID_TXRX_1              = 236 // Open Drone ID transmitter/receiver (Bluetooth/WiFi/Internet)
	MAV_COMP_ID_ODID_TXRX_2              = 237 // Open Drone ID transmitter/receiver (Bluetooth/WiFi/Internet)
	MAV_COMP_ID_ODID_TXRX_3              = 238 // Open Drone ID transmitter/receiver (Bluetooth/WiFi/Internet)
	MAV_COMP_ID_UDP_BRIDGE               = 240 // Component to bridge MAVLink to UDP (i.e. from a UART)
	MAV_COMP_ID_UART_BRIDGE              = 241 // Component to bridge to UART (i.e. from UDP)
	MAV_COMP_ID_TUNNEL_NODE              = 242 // Component handling TUNNEL messages (e.g. vendor specific GUI of a component)
	MAV_COMP_ID_SYSTEM_CONTROL           = 250 // Component for handling system messages (e.g. to ARM, takeoff, etc.)
)

// UavionixAdsbOutDynamicState (generated enum)
// State flags for ADS-B transponder dynamic report
const (
	UAVIONIX_ADSB_OUT_DYNAMIC_STATE_INTENT_CHANGE        = 1  //
	UAVIONIX_ADSB_OUT_DYNAMIC_STATE_AUTOPILOT_ENABLED    = 2  //
	UAVIONIX_ADSB_OUT_DYNAMIC_STATE_NICBARO_CROSSCHECKED = 4  //
	UAVIONIX_ADSB_OUT_DYNAMIC_STATE_ON_GROUND            = 8  //
	UAVIONIX_ADSB_OUT_DYNAMIC_STATE_IDENT                = 16 //
)

// UavionixAdsbOutRfSelect (generated enum)
// Transceiver RF control flags for ADS-B transponder dynamic reports
const (
	UAVIONIX_ADSB_OUT_RF_SELECT_STANDBY    = 0 //
	UAVIONIX_ADSB_OUT_RF_SELECT_RX_ENABLED = 1 //
	UAVIONIX_ADSB_OUT_RF_SELECT_TX_ENABLED = 2 //
)

// UavionixAdsbOutDynamicGpsFix (generated enum)
// Status for ADS-B transponder dynamic input
const (
	UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_NONE_0 = 0 //
	UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_NONE_1 = 1 //
	UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_2D     = 2 //
	UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_3D     = 3 //
	UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_DGPS   = 4 //
	UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_RTK    = 5 //
)

// UavionixAdsbRfHealth (generated enum)
// Status flags for ADS-B transponder dynamic output
const (
	UAVIONIX_ADSB_RF_HEALTH_INITIALIZING = 0  //
	UAVIONIX_ADSB_RF_HEALTH_OK           = 1  //
	UAVIONIX_ADSB_RF_HEALTH_FAIL_TX      = 2  //
	UAVIONIX_ADSB_RF_HEALTH_FAIL_RX      = 16 //
)

// UavionixAdsbOutCfgAircraftSize (generated enum)
// Definitions for aircraft size
const (
	UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_NO_DATA     = 0  //
	UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L15M_W23M   = 1  //
	UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L25M_W28P5M = 2  //
	UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L25_34M     = 3  //
	UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L35_33M     = 4  //
	UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L35_38M     = 5  //
	UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L45_39P5M   = 6  //
	UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L45_45M     = 7  //
	UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L55_45M     = 8  //
	UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L55_52M     = 9  //
	UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L65_59P5M   = 10 //
	UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L65_67M     = 11 //
	UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L75_W72P5M  = 12 //
	UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L75_W80M    = 13 //
	UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L85_W80M    = 14 //
	UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L85_W90M    = 15 //
)

// UavionixAdsbOutCfgGpsOffsetLat (generated enum)
// GPS lataral offset encoding
const (
	UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_NO_DATA  = 0 //
	UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_LEFT_2M  = 1 //
	UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_LEFT_4M  = 2 //
	UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_LEFT_6M  = 3 //
	UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_RIGHT_0M = 4 //
	UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_RIGHT_2M = 5 //
	UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_RIGHT_4M = 6 //
	UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_RIGHT_6M = 7 //
)

// UavionixAdsbOutCfgGpsOffsetLon (generated enum)
// GPS longitudinal offset encoding
const (
	UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_NO_DATA           = 0 //
	UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_APPLIED_BY_SENSOR = 1 //
)

// UavionixAdsbEmergencyStatus (generated enum)
// Emergency status encoding
const (
	UAVIONIX_ADSB_OUT_NO_EMERGENCY                    = 0 //
	UAVIONIX_ADSB_OUT_GENERAL_EMERGENCY               = 1 //
	UAVIONIX_ADSB_OUT_LIFEGUARD_EMERGENCY             = 2 //
	UAVIONIX_ADSB_OUT_MINIMUM_FUEL_EMERGENCY          = 3 //
	UAVIONIX_ADSB_OUT_NO_COMM_EMERGENCY               = 4 //
	UAVIONIX_ADSB_OUT_UNLAWFUL_INTERFERANCE_EMERGENCY = 5 //
	UAVIONIX_ADSB_OUT_DOWNED_AIRCRAFT_EMERGENCY       = 6 //
	UAVIONIX_ADSB_OUT_RESERVED                        = 7 //
)

// IcarousTrackBandTypes (generated enum)
//
const (
	ICAROUS_TRACK_BAND_TYPE_NONE     = 0 //
	ICAROUS_TRACK_BAND_TYPE_NEAR     = 1 //
	ICAROUS_TRACK_BAND_TYPE_RECOVERY = 2 //
)

// IcarousFmsState (generated enum)
//
const (
	ICAROUS_FMS_STATE_IDLE     = 0 //
	ICAROUS_FMS_STATE_TAKEOFF  = 1 //
	ICAROUS_FMS_STATE_CLIMB    = 2 //
	ICAROUS_FMS_STATE_CRUISE   = 3 //
	ICAROUS_FMS_STATE_APPROACH = 4 //
	ICAROUS_FMS_STATE_LAND     = 5 //
)

// ArdupilotmegaSensorOffsets struct (generated typeinfo)
// Offsets and calibrations values for hardware sensors. This makes it easier to debug the calibration process.
type ArdupilotmegaSensorOffsets struct {
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

// Dialect (generated function)
func (m *ArdupilotmegaSensorOffsets) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaSensorOffsets) MsgID() MessageID {
	return MSG_ID_SENSOR_OFFSETS
}

// CRCExtra (generated function)
func (m *ArdupilotmegaSensorOffsets) CRCExtra() uint8 {
	return 134
}

// MsgName (generated function)
func (m *ArdupilotmegaSensorOffsets) MsgName() string {
	return "SensorOffsets"
}

// String (generated function)
func (m *ArdupilotmegaSensorOffsets) String() string {
	return fmt.Sprintf("ArdupilotmegaSensorOffsets{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaSensorOffsets) Pack(p *Packet) error {
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
func (m *ArdupilotmegaSensorOffsets) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 42 {
		return errPayloadTooSmall
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

// ArdupilotmegaSetMagOffsets struct (generated typeinfo)
// Set the magnetometer offsets
type ArdupilotmegaSetMagOffsets struct {
	MagOfsX         int16 // Magnetometer X offset.
	MagOfsY         int16 // Magnetometer Y offset.
	MagOfsZ         int16 // Magnetometer Z offset.
	TargetSystem    uint8 // System ID.
	TargetComponent uint8 // Component ID.
}

// Dialect (generated function)
func (m *ArdupilotmegaSetMagOffsets) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaSetMagOffsets) MsgID() MessageID {
	return MSG_ID_SET_MAG_OFFSETS
}

// CRCExtra (generated function)
func (m *ArdupilotmegaSetMagOffsets) CRCExtra() uint8 {
	return 219
}

// MsgName (generated function)
func (m *ArdupilotmegaSetMagOffsets) MsgName() string {
	return "SetMagOffsets"
}

// String (generated function)
func (m *ArdupilotmegaSetMagOffsets) String() string {
	return fmt.Sprintf("ArdupilotmegaSetMagOffsets{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaSetMagOffsets) Pack(p *Packet) error {
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
func (m *ArdupilotmegaSetMagOffsets) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 8 {
		return errPayloadTooSmall
	}
	m.MagOfsX = int16(binary.LittleEndian.Uint16(payload[0:]))
	m.MagOfsY = int16(binary.LittleEndian.Uint16(payload[2:]))
	m.MagOfsZ = int16(binary.LittleEndian.Uint16(payload[4:]))
	m.TargetSystem = uint8(payload[6])
	m.TargetComponent = uint8(payload[7])
	return nil
}

// ArdupilotmegaMeminfo struct (generated typeinfo)
// State of APM memory.
type ArdupilotmegaMeminfo struct {
	Brkval  uint16 // Heap top.
	Freemem uint16 // Free memory.
}

// Dialect (generated function)
func (m *ArdupilotmegaMeminfo) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaMeminfo) MsgID() MessageID {
	return MSG_ID_MEMINFO
}

// CRCExtra (generated function)
func (m *ArdupilotmegaMeminfo) CRCExtra() uint8 {
	return 208
}

// MsgName (generated function)
func (m *ArdupilotmegaMeminfo) MsgName() string {
	return "Meminfo"
}

// String (generated function)
func (m *ArdupilotmegaMeminfo) String() string {
	return fmt.Sprintf("ArdupilotmegaMeminfo{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaMeminfo) Pack(p *Packet) error {
	payload := make([]byte, 4)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.Brkval))
	binary.LittleEndian.PutUint16(payload[2:], uint16(m.Freemem))
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ArdupilotmegaMeminfo) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 4 {
		return errPayloadTooSmall
	}
	m.Brkval = uint16(binary.LittleEndian.Uint16(payload[0:]))
	m.Freemem = uint16(binary.LittleEndian.Uint16(payload[2:]))
	return nil
}

// ArdupilotmegaApAdc struct (generated typeinfo)
// Raw ADC output.
type ArdupilotmegaApAdc struct {
	Adc1 uint16 // ADC output 1.
	Adc2 uint16 // ADC output 2.
	Adc3 uint16 // ADC output 3.
	Adc4 uint16 // ADC output 4.
	Adc5 uint16 // ADC output 5.
	Adc6 uint16 // ADC output 6.
}

// Dialect (generated function)
func (m *ArdupilotmegaApAdc) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaApAdc) MsgID() MessageID {
	return MSG_ID_AP_ADC
}

// CRCExtra (generated function)
func (m *ArdupilotmegaApAdc) CRCExtra() uint8 {
	return 188
}

// MsgName (generated function)
func (m *ArdupilotmegaApAdc) MsgName() string {
	return "ApAdc"
}

// String (generated function)
func (m *ArdupilotmegaApAdc) String() string {
	return fmt.Sprintf("ArdupilotmegaApAdc{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaApAdc) Pack(p *Packet) error {
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
func (m *ArdupilotmegaApAdc) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 12 {
		return errPayloadTooSmall
	}
	m.Adc1 = uint16(binary.LittleEndian.Uint16(payload[0:]))
	m.Adc2 = uint16(binary.LittleEndian.Uint16(payload[2:]))
	m.Adc3 = uint16(binary.LittleEndian.Uint16(payload[4:]))
	m.Adc4 = uint16(binary.LittleEndian.Uint16(payload[6:]))
	m.Adc5 = uint16(binary.LittleEndian.Uint16(payload[8:]))
	m.Adc6 = uint16(binary.LittleEndian.Uint16(payload[10:]))
	return nil
}

// ArdupilotmegaDigicamConfigure struct (generated typeinfo)
// Configure on-board Camera Control System.
type ArdupilotmegaDigicamConfigure struct {
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

// Dialect (generated function)
func (m *ArdupilotmegaDigicamConfigure) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaDigicamConfigure) MsgID() MessageID {
	return MSG_ID_DIGICAM_CONFIGURE
}

// CRCExtra (generated function)
func (m *ArdupilotmegaDigicamConfigure) CRCExtra() uint8 {
	return 84
}

// MsgName (generated function)
func (m *ArdupilotmegaDigicamConfigure) MsgName() string {
	return "DigicamConfigure"
}

// String (generated function)
func (m *ArdupilotmegaDigicamConfigure) String() string {
	return fmt.Sprintf("ArdupilotmegaDigicamConfigure{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaDigicamConfigure) Pack(p *Packet) error {
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
func (m *ArdupilotmegaDigicamConfigure) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 15 {
		return errPayloadTooSmall
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

// ArdupilotmegaDigicamControl struct (generated typeinfo)
// Control on-board Camera Control System to take shots.
type ArdupilotmegaDigicamControl struct {
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

// Dialect (generated function)
func (m *ArdupilotmegaDigicamControl) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaDigicamControl) MsgID() MessageID {
	return MSG_ID_DIGICAM_CONTROL
}

// CRCExtra (generated function)
func (m *ArdupilotmegaDigicamControl) CRCExtra() uint8 {
	return 22
}

// MsgName (generated function)
func (m *ArdupilotmegaDigicamControl) MsgName() string {
	return "DigicamControl"
}

// String (generated function)
func (m *ArdupilotmegaDigicamControl) String() string {
	return fmt.Sprintf("ArdupilotmegaDigicamControl{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaDigicamControl) Pack(p *Packet) error {
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
func (m *ArdupilotmegaDigicamControl) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 13 {
		return errPayloadTooSmall
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

// ArdupilotmegaMountConfigure struct (generated typeinfo)
// Message to configure a camera mount, directional antenna, etc.
type ArdupilotmegaMountConfigure struct {
	TargetSystem    uint8 // System ID.
	TargetComponent uint8 // Component ID.
	MountMode       uint8 // Mount operating mode.
	StabRoll        uint8 // (1 = yes, 0 = no).
	StabPitch       uint8 // (1 = yes, 0 = no).
	StabYaw         uint8 // (1 = yes, 0 = no).
}

// Dialect (generated function)
func (m *ArdupilotmegaMountConfigure) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaMountConfigure) MsgID() MessageID {
	return MSG_ID_MOUNT_CONFIGURE
}

// CRCExtra (generated function)
func (m *ArdupilotmegaMountConfigure) CRCExtra() uint8 {
	return 19
}

// MsgName (generated function)
func (m *ArdupilotmegaMountConfigure) MsgName() string {
	return "MountConfigure"
}

// String (generated function)
func (m *ArdupilotmegaMountConfigure) String() string {
	return fmt.Sprintf("ArdupilotmegaMountConfigure{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaMountConfigure) Pack(p *Packet) error {
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
func (m *ArdupilotmegaMountConfigure) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 6 {
		return errPayloadTooSmall
	}
	m.TargetSystem = uint8(payload[0])
	m.TargetComponent = uint8(payload[1])
	m.MountMode = uint8(payload[2])
	m.StabRoll = uint8(payload[3])
	m.StabPitch = uint8(payload[4])
	m.StabYaw = uint8(payload[5])
	return nil
}

// ArdupilotmegaMountControl struct (generated typeinfo)
// Message to control a camera mount, directional antenna, etc.
type ArdupilotmegaMountControl struct {
	InputA          int32 // Pitch (centi-degrees) or lat (degE7), depending on mount mode.
	InputB          int32 // Roll (centi-degrees) or lon (degE7) depending on mount mode.
	InputC          int32 // Yaw (centi-degrees) or alt (cm) depending on mount mode.
	TargetSystem    uint8 // System ID.
	TargetComponent uint8 // Component ID.
	SavePosition    uint8 // If "1" it will save current trimmed position on EEPROM (just valid for NEUTRAL and LANDING).
}

// Dialect (generated function)
func (m *ArdupilotmegaMountControl) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaMountControl) MsgID() MessageID {
	return MSG_ID_MOUNT_CONTROL
}

// CRCExtra (generated function)
func (m *ArdupilotmegaMountControl) CRCExtra() uint8 {
	return 21
}

// MsgName (generated function)
func (m *ArdupilotmegaMountControl) MsgName() string {
	return "MountControl"
}

// String (generated function)
func (m *ArdupilotmegaMountControl) String() string {
	return fmt.Sprintf("ArdupilotmegaMountControl{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaMountControl) Pack(p *Packet) error {
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
func (m *ArdupilotmegaMountControl) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 15 {
		return errPayloadTooSmall
	}
	m.InputA = int32(binary.LittleEndian.Uint32(payload[0:]))
	m.InputB = int32(binary.LittleEndian.Uint32(payload[4:]))
	m.InputC = int32(binary.LittleEndian.Uint32(payload[8:]))
	m.TargetSystem = uint8(payload[12])
	m.TargetComponent = uint8(payload[13])
	m.SavePosition = uint8(payload[14])
	return nil
}

// ArdupilotmegaMountStatus struct (generated typeinfo)
// Message with some status from APM to GCS about camera or antenna mount.
type ArdupilotmegaMountStatus struct {
	PointingA       int32 // Pitch.
	PointingB       int32 // Roll.
	PointingC       int32 // Yaw.
	TargetSystem    uint8 // System ID.
	TargetComponent uint8 // Component ID.
}

// Dialect (generated function)
func (m *ArdupilotmegaMountStatus) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaMountStatus) MsgID() MessageID {
	return MSG_ID_MOUNT_STATUS
}

// CRCExtra (generated function)
func (m *ArdupilotmegaMountStatus) CRCExtra() uint8 {
	return 134
}

// MsgName (generated function)
func (m *ArdupilotmegaMountStatus) MsgName() string {
	return "MountStatus"
}

// String (generated function)
func (m *ArdupilotmegaMountStatus) String() string {
	return fmt.Sprintf("ArdupilotmegaMountStatus{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaMountStatus) Pack(p *Packet) error {
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
func (m *ArdupilotmegaMountStatus) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 14 {
		return errPayloadTooSmall
	}
	m.PointingA = int32(binary.LittleEndian.Uint32(payload[0:]))
	m.PointingB = int32(binary.LittleEndian.Uint32(payload[4:]))
	m.PointingC = int32(binary.LittleEndian.Uint32(payload[8:]))
	m.TargetSystem = uint8(payload[12])
	m.TargetComponent = uint8(payload[13])
	return nil
}

// ArdupilotmegaFencePoint struct (generated typeinfo)
// A fence point. Used to set a point when from GCS -> MAV. Also used to return a point from MAV -> GCS.
type ArdupilotmegaFencePoint struct {
	Lat             float32 // Latitude of point.
	Lng             float32 // Longitude of point.
	TargetSystem    uint8   // System ID.
	TargetComponent uint8   // Component ID.
	Idx             uint8   // Point index (first point is 1, 0 is for return point).
	Count           uint8   // Total number of points (for sanity checking).
}

// Dialect (generated function)
func (m *ArdupilotmegaFencePoint) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaFencePoint) MsgID() MessageID {
	return MSG_ID_FENCE_POINT
}

// CRCExtra (generated function)
func (m *ArdupilotmegaFencePoint) CRCExtra() uint8 {
	return 78
}

// MsgName (generated function)
func (m *ArdupilotmegaFencePoint) MsgName() string {
	return "FencePoint"
}

// String (generated function)
func (m *ArdupilotmegaFencePoint) String() string {
	return fmt.Sprintf("ArdupilotmegaFencePoint{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaFencePoint) Pack(p *Packet) error {
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
func (m *ArdupilotmegaFencePoint) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 12 {
		return errPayloadTooSmall
	}
	m.Lat = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.Lng = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	m.TargetSystem = uint8(payload[8])
	m.TargetComponent = uint8(payload[9])
	m.Idx = uint8(payload[10])
	m.Count = uint8(payload[11])
	return nil
}

// ArdupilotmegaFenceFetchPoint struct (generated typeinfo)
// Request a current fence point from MAV.
type ArdupilotmegaFenceFetchPoint struct {
	TargetSystem    uint8 // System ID.
	TargetComponent uint8 // Component ID.
	Idx             uint8 // Point index (first point is 1, 0 is for return point).
}

// Dialect (generated function)
func (m *ArdupilotmegaFenceFetchPoint) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaFenceFetchPoint) MsgID() MessageID {
	return MSG_ID_FENCE_FETCH_POINT
}

// CRCExtra (generated function)
func (m *ArdupilotmegaFenceFetchPoint) CRCExtra() uint8 {
	return 68
}

// MsgName (generated function)
func (m *ArdupilotmegaFenceFetchPoint) MsgName() string {
	return "FenceFetchPoint"
}

// String (generated function)
func (m *ArdupilotmegaFenceFetchPoint) String() string {
	return fmt.Sprintf("ArdupilotmegaFenceFetchPoint{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaFenceFetchPoint) Pack(p *Packet) error {
	payload := make([]byte, 3)
	payload[0] = byte(m.TargetSystem)
	payload[1] = byte(m.TargetComponent)
	payload[2] = byte(m.Idx)
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ArdupilotmegaFenceFetchPoint) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 3 {
		return errPayloadTooSmall
	}
	m.TargetSystem = uint8(payload[0])
	m.TargetComponent = uint8(payload[1])
	m.Idx = uint8(payload[2])
	return nil
}

// ArdupilotmegaAhrs struct (generated typeinfo)
// Status of DCM attitude estimator.
type ArdupilotmegaAhrs struct {
	Omegaix     float32 // X gyro drift estimate.
	Omegaiy     float32 // Y gyro drift estimate.
	Omegaiz     float32 // Z gyro drift estimate.
	AccelWeight float32 // Average accel_weight.
	RenormVal   float32 // Average renormalisation value.
	ErrorRp     float32 // Average error_roll_pitch value.
	ErrorYaw    float32 // Average error_yaw value.
}

// Dialect (generated function)
func (m *ArdupilotmegaAhrs) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaAhrs) MsgID() MessageID {
	return MSG_ID_AHRS
}

// CRCExtra (generated function)
func (m *ArdupilotmegaAhrs) CRCExtra() uint8 {
	return 127
}

// MsgName (generated function)
func (m *ArdupilotmegaAhrs) MsgName() string {
	return "Ahrs"
}

// String (generated function)
func (m *ArdupilotmegaAhrs) String() string {
	return fmt.Sprintf("ArdupilotmegaAhrs{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaAhrs) Pack(p *Packet) error {
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
func (m *ArdupilotmegaAhrs) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 28 {
		return errPayloadTooSmall
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

// ArdupilotmegaSimstate struct (generated typeinfo)
// Status of simulation environment, if used.
type ArdupilotmegaSimstate struct {
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

// Dialect (generated function)
func (m *ArdupilotmegaSimstate) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaSimstate) MsgID() MessageID {
	return MSG_ID_SIMSTATE
}

// CRCExtra (generated function)
func (m *ArdupilotmegaSimstate) CRCExtra() uint8 {
	return 154
}

// MsgName (generated function)
func (m *ArdupilotmegaSimstate) MsgName() string {
	return "Simstate"
}

// String (generated function)
func (m *ArdupilotmegaSimstate) String() string {
	return fmt.Sprintf("ArdupilotmegaSimstate{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaSimstate) Pack(p *Packet) error {
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
func (m *ArdupilotmegaSimstate) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 44 {
		return errPayloadTooSmall
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

// ArdupilotmegaHwstatus struct (generated typeinfo)
// Status of key hardware.
type ArdupilotmegaHwstatus struct {
	Vcc    uint16 // Board voltage.
	I2cerr uint8  // I2C error count.
}

// Dialect (generated function)
func (m *ArdupilotmegaHwstatus) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaHwstatus) MsgID() MessageID {
	return MSG_ID_HWSTATUS
}

// CRCExtra (generated function)
func (m *ArdupilotmegaHwstatus) CRCExtra() uint8 {
	return 21
}

// MsgName (generated function)
func (m *ArdupilotmegaHwstatus) MsgName() string {
	return "Hwstatus"
}

// String (generated function)
func (m *ArdupilotmegaHwstatus) String() string {
	return fmt.Sprintf("ArdupilotmegaHwstatus{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaHwstatus) Pack(p *Packet) error {
	payload := make([]byte, 3)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.Vcc))
	payload[2] = byte(m.I2cerr)
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ArdupilotmegaHwstatus) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 3 {
		return errPayloadTooSmall
	}
	m.Vcc = uint16(binary.LittleEndian.Uint16(payload[0:]))
	m.I2cerr = uint8(payload[2])
	return nil
}

// ArdupilotmegaRadio struct (generated typeinfo)
// Status generated by radio.
type ArdupilotmegaRadio struct {
	Rxerrors uint16 // Receive errors.
	Fixed    uint16 // Count of error corrected packets.
	Rssi     uint8  // Local signal strength.
	Remrssi  uint8  // Remote signal strength.
	Txbuf    uint8  // How full the tx buffer is.
	Noise    uint8  // Background noise level.
	Remnoise uint8  // Remote background noise level.
}

// Dialect (generated function)
func (m *ArdupilotmegaRadio) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaRadio) MsgID() MessageID {
	return MSG_ID_RADIO
}

// CRCExtra (generated function)
func (m *ArdupilotmegaRadio) CRCExtra() uint8 {
	return 21
}

// MsgName (generated function)
func (m *ArdupilotmegaRadio) MsgName() string {
	return "Radio"
}

// String (generated function)
func (m *ArdupilotmegaRadio) String() string {
	return fmt.Sprintf("ArdupilotmegaRadio{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaRadio) Pack(p *Packet) error {
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
func (m *ArdupilotmegaRadio) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 9 {
		return errPayloadTooSmall
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

// ArdupilotmegaLimitsStatus struct (generated typeinfo)
// Status of AP_Limits. Sent in extended status stream when AP_Limits is enabled.
type ArdupilotmegaLimitsStatus struct {
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

// Dialect (generated function)
func (m *ArdupilotmegaLimitsStatus) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaLimitsStatus) MsgID() MessageID {
	return MSG_ID_LIMITS_STATUS
}

// CRCExtra (generated function)
func (m *ArdupilotmegaLimitsStatus) CRCExtra() uint8 {
	return 144
}

// MsgName (generated function)
func (m *ArdupilotmegaLimitsStatus) MsgName() string {
	return "LimitsStatus"
}

// String (generated function)
func (m *ArdupilotmegaLimitsStatus) String() string {
	return fmt.Sprintf("ArdupilotmegaLimitsStatus{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaLimitsStatus) Pack(p *Packet) error {
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
func (m *ArdupilotmegaLimitsStatus) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 22 {
		return errPayloadTooSmall
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

// ArdupilotmegaWind struct (generated typeinfo)
// Wind estimation.
type ArdupilotmegaWind struct {
	Direction float32 // Wind direction (that wind is coming from).
	Speed     float32 // Wind speed in ground plane.
	SpeedZ    float32 // Vertical wind speed.
}

// Dialect (generated function)
func (m *ArdupilotmegaWind) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaWind) MsgID() MessageID {
	return MSG_ID_WIND
}

// CRCExtra (generated function)
func (m *ArdupilotmegaWind) CRCExtra() uint8 {
	return 1
}

// MsgName (generated function)
func (m *ArdupilotmegaWind) MsgName() string {
	return "Wind"
}

// String (generated function)
func (m *ArdupilotmegaWind) String() string {
	return fmt.Sprintf("ArdupilotmegaWind{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaWind) Pack(p *Packet) error {
	payload := make([]byte, 12)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(m.Direction))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(m.Speed))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.SpeedZ))
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ArdupilotmegaWind) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 12 {
		return errPayloadTooSmall
	}
	m.Direction = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.Speed = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	m.SpeedZ = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	return nil
}

// ArdupilotmegaData16 struct (generated typeinfo)
// Data packet, size 16.
type ArdupilotmegaData16 struct {
	Type uint8     // Data type.
	Len  uint8     // Data length.
	Data [16]uint8 // Raw data.
}

// Dialect (generated function)
func (m *ArdupilotmegaData16) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaData16) MsgID() MessageID {
	return MSG_ID_DATA16
}

// CRCExtra (generated function)
func (m *ArdupilotmegaData16) CRCExtra() uint8 {
	return 234
}

// MsgName (generated function)
func (m *ArdupilotmegaData16) MsgName() string {
	return "Data16"
}

// String (generated function)
func (m *ArdupilotmegaData16) String() string {
	return fmt.Sprintf("ArdupilotmegaData16{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaData16) Pack(p *Packet) error {
	payload := make([]byte, 18)
	payload[0] = byte(m.Type)
	payload[1] = byte(m.Len)
	copy(payload[2:], m.Data[:])
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ArdupilotmegaData16) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 18 {
		return errPayloadTooSmall
	}
	m.Type = uint8(payload[0])
	m.Len = uint8(payload[1])
	copy(m.Data[:], payload[2:18])
	return nil
}

// ArdupilotmegaData32 struct (generated typeinfo)
// Data packet, size 32.
type ArdupilotmegaData32 struct {
	Type uint8     // Data type.
	Len  uint8     // Data length.
	Data [32]uint8 // Raw data.
}

// Dialect (generated function)
func (m *ArdupilotmegaData32) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaData32) MsgID() MessageID {
	return MSG_ID_DATA32
}

// CRCExtra (generated function)
func (m *ArdupilotmegaData32) CRCExtra() uint8 {
	return 73
}

// MsgName (generated function)
func (m *ArdupilotmegaData32) MsgName() string {
	return "Data32"
}

// String (generated function)
func (m *ArdupilotmegaData32) String() string {
	return fmt.Sprintf("ArdupilotmegaData32{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaData32) Pack(p *Packet) error {
	payload := make([]byte, 34)
	payload[0] = byte(m.Type)
	payload[1] = byte(m.Len)
	copy(payload[2:], m.Data[:])
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ArdupilotmegaData32) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 34 {
		return errPayloadTooSmall
	}
	m.Type = uint8(payload[0])
	m.Len = uint8(payload[1])
	copy(m.Data[:], payload[2:34])
	return nil
}

// ArdupilotmegaData64 struct (generated typeinfo)
// Data packet, size 64.
type ArdupilotmegaData64 struct {
	Type uint8     // Data type.
	Len  uint8     // Data length.
	Data [64]uint8 // Raw data.
}

// Dialect (generated function)
func (m *ArdupilotmegaData64) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaData64) MsgID() MessageID {
	return MSG_ID_DATA64
}

// CRCExtra (generated function)
func (m *ArdupilotmegaData64) CRCExtra() uint8 {
	return 181
}

// MsgName (generated function)
func (m *ArdupilotmegaData64) MsgName() string {
	return "Data64"
}

// String (generated function)
func (m *ArdupilotmegaData64) String() string {
	return fmt.Sprintf("ArdupilotmegaData64{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaData64) Pack(p *Packet) error {
	payload := make([]byte, 66)
	payload[0] = byte(m.Type)
	payload[1] = byte(m.Len)
	copy(payload[2:], m.Data[:])
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ArdupilotmegaData64) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 66 {
		return errPayloadTooSmall
	}
	m.Type = uint8(payload[0])
	m.Len = uint8(payload[1])
	copy(m.Data[:], payload[2:66])
	return nil
}

// ArdupilotmegaData96 struct (generated typeinfo)
// Data packet, size 96.
type ArdupilotmegaData96 struct {
	Type uint8     // Data type.
	Len  uint8     // Data length.
	Data [96]uint8 // Raw data.
}

// Dialect (generated function)
func (m *ArdupilotmegaData96) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaData96) MsgID() MessageID {
	return MSG_ID_DATA96
}

// CRCExtra (generated function)
func (m *ArdupilotmegaData96) CRCExtra() uint8 {
	return 22
}

// MsgName (generated function)
func (m *ArdupilotmegaData96) MsgName() string {
	return "Data96"
}

// String (generated function)
func (m *ArdupilotmegaData96) String() string {
	return fmt.Sprintf("ArdupilotmegaData96{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaData96) Pack(p *Packet) error {
	payload := make([]byte, 98)
	payload[0] = byte(m.Type)
	payload[1] = byte(m.Len)
	copy(payload[2:], m.Data[:])
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ArdupilotmegaData96) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 98 {
		return errPayloadTooSmall
	}
	m.Type = uint8(payload[0])
	m.Len = uint8(payload[1])
	copy(m.Data[:], payload[2:98])
	return nil
}

// ArdupilotmegaRangefinder struct (generated typeinfo)
// Rangefinder reporting.
type ArdupilotmegaRangefinder struct {
	Distance float32 // Distance.
	Voltage  float32 // Raw voltage if available, zero otherwise.
}

// Dialect (generated function)
func (m *ArdupilotmegaRangefinder) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaRangefinder) MsgID() MessageID {
	return MSG_ID_RANGEFINDER
}

// CRCExtra (generated function)
func (m *ArdupilotmegaRangefinder) CRCExtra() uint8 {
	return 83
}

// MsgName (generated function)
func (m *ArdupilotmegaRangefinder) MsgName() string {
	return "Rangefinder"
}

// String (generated function)
func (m *ArdupilotmegaRangefinder) String() string {
	return fmt.Sprintf("ArdupilotmegaRangefinder{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaRangefinder) Pack(p *Packet) error {
	payload := make([]byte, 8)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(m.Distance))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(m.Voltage))
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ArdupilotmegaRangefinder) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 8 {
		return errPayloadTooSmall
	}
	m.Distance = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.Voltage = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	return nil
}

// ArdupilotmegaAirspeedAutocal struct (generated typeinfo)
// Airspeed auto-calibration.
type ArdupilotmegaAirspeedAutocal struct {
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

// Dialect (generated function)
func (m *ArdupilotmegaAirspeedAutocal) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaAirspeedAutocal) MsgID() MessageID {
	return MSG_ID_AIRSPEED_AUTOCAL
}

// CRCExtra (generated function)
func (m *ArdupilotmegaAirspeedAutocal) CRCExtra() uint8 {
	return 167
}

// MsgName (generated function)
func (m *ArdupilotmegaAirspeedAutocal) MsgName() string {
	return "AirspeedAutocal"
}

// String (generated function)
func (m *ArdupilotmegaAirspeedAutocal) String() string {
	return fmt.Sprintf("ArdupilotmegaAirspeedAutocal{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaAirspeedAutocal) Pack(p *Packet) error {
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
func (m *ArdupilotmegaAirspeedAutocal) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 48 {
		return errPayloadTooSmall
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

// ArdupilotmegaRallyPoint struct (generated typeinfo)
// A rally point. Used to set a point when from GCS -> MAV. Also used to return a point from MAV -> GCS.
type ArdupilotmegaRallyPoint struct {
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

// Dialect (generated function)
func (m *ArdupilotmegaRallyPoint) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaRallyPoint) MsgID() MessageID {
	return MSG_ID_RALLY_POINT
}

// CRCExtra (generated function)
func (m *ArdupilotmegaRallyPoint) CRCExtra() uint8 {
	return 138
}

// MsgName (generated function)
func (m *ArdupilotmegaRallyPoint) MsgName() string {
	return "RallyPoint"
}

// String (generated function)
func (m *ArdupilotmegaRallyPoint) String() string {
	return fmt.Sprintf("ArdupilotmegaRallyPoint{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaRallyPoint) Pack(p *Packet) error {
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
func (m *ArdupilotmegaRallyPoint) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 19 {
		return errPayloadTooSmall
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

// ArdupilotmegaRallyFetchPoint struct (generated typeinfo)
// Request a current rally point from MAV. MAV should respond with a RALLY_POINT message. MAV should not respond if the request is invalid.
type ArdupilotmegaRallyFetchPoint struct {
	TargetSystem    uint8 // System ID.
	TargetComponent uint8 // Component ID.
	Idx             uint8 // Point index (first point is 0).
}

// Dialect (generated function)
func (m *ArdupilotmegaRallyFetchPoint) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaRallyFetchPoint) MsgID() MessageID {
	return MSG_ID_RALLY_FETCH_POINT
}

// CRCExtra (generated function)
func (m *ArdupilotmegaRallyFetchPoint) CRCExtra() uint8 {
	return 234
}

// MsgName (generated function)
func (m *ArdupilotmegaRallyFetchPoint) MsgName() string {
	return "RallyFetchPoint"
}

// String (generated function)
func (m *ArdupilotmegaRallyFetchPoint) String() string {
	return fmt.Sprintf("ArdupilotmegaRallyFetchPoint{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaRallyFetchPoint) Pack(p *Packet) error {
	payload := make([]byte, 3)
	payload[0] = byte(m.TargetSystem)
	payload[1] = byte(m.TargetComponent)
	payload[2] = byte(m.Idx)
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ArdupilotmegaRallyFetchPoint) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 3 {
		return errPayloadTooSmall
	}
	m.TargetSystem = uint8(payload[0])
	m.TargetComponent = uint8(payload[1])
	m.Idx = uint8(payload[2])
	return nil
}

// ArdupilotmegaCompassmotStatus struct (generated typeinfo)
// Status of compassmot calibration.
type ArdupilotmegaCompassmotStatus struct {
	Current       float32 // Current.
	Compensationx float32 // Motor Compensation X.
	Compensationy float32 // Motor Compensation Y.
	Compensationz float32 // Motor Compensation Z.
	Throttle      uint16  // Throttle.
	Interference  uint16  // Interference.
}

// Dialect (generated function)
func (m *ArdupilotmegaCompassmotStatus) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaCompassmotStatus) MsgID() MessageID {
	return MSG_ID_COMPASSMOT_STATUS
}

// CRCExtra (generated function)
func (m *ArdupilotmegaCompassmotStatus) CRCExtra() uint8 {
	return 240
}

// MsgName (generated function)
func (m *ArdupilotmegaCompassmotStatus) MsgName() string {
	return "CompassmotStatus"
}

// String (generated function)
func (m *ArdupilotmegaCompassmotStatus) String() string {
	return fmt.Sprintf("ArdupilotmegaCompassmotStatus{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaCompassmotStatus) Pack(p *Packet) error {
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
func (m *ArdupilotmegaCompassmotStatus) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 20 {
		return errPayloadTooSmall
	}
	m.Current = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.Compensationx = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	m.Compensationy = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.Compensationz = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.Throttle = uint16(binary.LittleEndian.Uint16(payload[16:]))
	m.Interference = uint16(binary.LittleEndian.Uint16(payload[18:]))
	return nil
}

// ArdupilotmegaAhrs2 struct (generated typeinfo)
// Status of secondary AHRS filter if available.
type ArdupilotmegaAhrs2 struct {
	Roll     float32 // Roll angle.
	Pitch    float32 // Pitch angle.
	Yaw      float32 // Yaw angle.
	Altitude float32 // Altitude (MSL).
	Lat      int32   // Latitude.
	Lng      int32   // Longitude.
}

// Dialect (generated function)
func (m *ArdupilotmegaAhrs2) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaAhrs2) MsgID() MessageID {
	return MSG_ID_AHRS2
}

// CRCExtra (generated function)
func (m *ArdupilotmegaAhrs2) CRCExtra() uint8 {
	return 47
}

// MsgName (generated function)
func (m *ArdupilotmegaAhrs2) MsgName() string {
	return "Ahrs2"
}

// String (generated function)
func (m *ArdupilotmegaAhrs2) String() string {
	return fmt.Sprintf("ArdupilotmegaAhrs2{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaAhrs2) Pack(p *Packet) error {
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
func (m *ArdupilotmegaAhrs2) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 24 {
		return errPayloadTooSmall
	}
	m.Roll = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.Pitch = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	m.Yaw = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.Altitude = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.Lat = int32(binary.LittleEndian.Uint32(payload[16:]))
	m.Lng = int32(binary.LittleEndian.Uint32(payload[20:]))
	return nil
}

// ArdupilotmegaCameraStatus struct (generated typeinfo)
// Camera Event.
type ArdupilotmegaCameraStatus struct {
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

// Dialect (generated function)
func (m *ArdupilotmegaCameraStatus) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaCameraStatus) MsgID() MessageID {
	return MSG_ID_CAMERA_STATUS
}

// CRCExtra (generated function)
func (m *ArdupilotmegaCameraStatus) CRCExtra() uint8 {
	return 189
}

// MsgName (generated function)
func (m *ArdupilotmegaCameraStatus) MsgName() string {
	return "CameraStatus"
}

// String (generated function)
func (m *ArdupilotmegaCameraStatus) String() string {
	return fmt.Sprintf("ArdupilotmegaCameraStatus{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaCameraStatus) Pack(p *Packet) error {
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
func (m *ArdupilotmegaCameraStatus) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 29 {
		return errPayloadTooSmall
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

// ArdupilotmegaCameraFeedback struct (generated typeinfo)
// Camera Capture Feedback.
type ArdupilotmegaCameraFeedback struct {
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

// Dialect (generated function)
func (m *ArdupilotmegaCameraFeedback) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaCameraFeedback) MsgID() MessageID {
	return MSG_ID_CAMERA_FEEDBACK
}

// CRCExtra (generated function)
func (m *ArdupilotmegaCameraFeedback) CRCExtra() uint8 {
	return 52
}

// MsgName (generated function)
func (m *ArdupilotmegaCameraFeedback) MsgName() string {
	return "CameraFeedback"
}

// String (generated function)
func (m *ArdupilotmegaCameraFeedback) String() string {
	return fmt.Sprintf("ArdupilotmegaCameraFeedback{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaCameraFeedback) Pack(p *Packet) error {
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
func (m *ArdupilotmegaCameraFeedback) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 45 {
		return errPayloadTooSmall
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

// ArdupilotmegaBattery2 struct (generated typeinfo)
// 2nd Battery status
type ArdupilotmegaBattery2 struct {
	Voltage        uint16 // Voltage.
	CurrentBattery int16  // Battery current, -1: autopilot does not measure the current.
}

// Dialect (generated function)
func (m *ArdupilotmegaBattery2) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaBattery2) MsgID() MessageID {
	return MSG_ID_BATTERY2
}

// CRCExtra (generated function)
func (m *ArdupilotmegaBattery2) CRCExtra() uint8 {
	return 174
}

// MsgName (generated function)
func (m *ArdupilotmegaBattery2) MsgName() string {
	return "Battery2"
}

// String (generated function)
func (m *ArdupilotmegaBattery2) String() string {
	return fmt.Sprintf("ArdupilotmegaBattery2{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaBattery2) Pack(p *Packet) error {
	payload := make([]byte, 4)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.Voltage))
	binary.LittleEndian.PutUint16(payload[2:], uint16(m.CurrentBattery))
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ArdupilotmegaBattery2) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 4 {
		return errPayloadTooSmall
	}
	m.Voltage = uint16(binary.LittleEndian.Uint16(payload[0:]))
	m.CurrentBattery = int16(binary.LittleEndian.Uint16(payload[2:]))
	return nil
}

// ArdupilotmegaAhrs3 struct (generated typeinfo)
// Status of third AHRS filter if available. This is for ANU research group (Ali and Sean).
type ArdupilotmegaAhrs3 struct {
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

// Dialect (generated function)
func (m *ArdupilotmegaAhrs3) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaAhrs3) MsgID() MessageID {
	return MSG_ID_AHRS3
}

// CRCExtra (generated function)
func (m *ArdupilotmegaAhrs3) CRCExtra() uint8 {
	return 229
}

// MsgName (generated function)
func (m *ArdupilotmegaAhrs3) MsgName() string {
	return "Ahrs3"
}

// String (generated function)
func (m *ArdupilotmegaAhrs3) String() string {
	return fmt.Sprintf("ArdupilotmegaAhrs3{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaAhrs3) Pack(p *Packet) error {
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
func (m *ArdupilotmegaAhrs3) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 40 {
		return errPayloadTooSmall
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

// ArdupilotmegaAutopilotVersionRequest struct (generated typeinfo)
// Request the autopilot version from the system/component.
type ArdupilotmegaAutopilotVersionRequest struct {
	TargetSystem    uint8 // System ID.
	TargetComponent uint8 // Component ID.
}

// Dialect (generated function)
func (m *ArdupilotmegaAutopilotVersionRequest) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaAutopilotVersionRequest) MsgID() MessageID {
	return MSG_ID_AUTOPILOT_VERSION_REQUEST
}

// CRCExtra (generated function)
func (m *ArdupilotmegaAutopilotVersionRequest) CRCExtra() uint8 {
	return 85
}

// MsgName (generated function)
func (m *ArdupilotmegaAutopilotVersionRequest) MsgName() string {
	return "AutopilotVersionRequest"
}

// String (generated function)
func (m *ArdupilotmegaAutopilotVersionRequest) String() string {
	return fmt.Sprintf("ArdupilotmegaAutopilotVersionRequest{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaAutopilotVersionRequest) Pack(p *Packet) error {
	payload := make([]byte, 2)
	payload[0] = byte(m.TargetSystem)
	payload[1] = byte(m.TargetComponent)
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ArdupilotmegaAutopilotVersionRequest) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 2 {
		return errPayloadTooSmall
	}
	m.TargetSystem = uint8(payload[0])
	m.TargetComponent = uint8(payload[1])
	return nil
}

// ArdupilotmegaRemoteLogDataBlock struct (generated typeinfo)
// Send a block of log data to remote location.
type ArdupilotmegaRemoteLogDataBlock struct {
	Seqno           uint32     // Log data block sequence number.
	TargetSystem    uint8      // System ID.
	TargetComponent uint8      // Component ID.
	Data            [200]uint8 // Log data block.
}

// Dialect (generated function)
func (m *ArdupilotmegaRemoteLogDataBlock) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaRemoteLogDataBlock) MsgID() MessageID {
	return MSG_ID_REMOTE_LOG_DATA_BLOCK
}

// CRCExtra (generated function)
func (m *ArdupilotmegaRemoteLogDataBlock) CRCExtra() uint8 {
	return 159
}

// MsgName (generated function)
func (m *ArdupilotmegaRemoteLogDataBlock) MsgName() string {
	return "RemoteLogDataBlock"
}

// String (generated function)
func (m *ArdupilotmegaRemoteLogDataBlock) String() string {
	return fmt.Sprintf("ArdupilotmegaRemoteLogDataBlock{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaRemoteLogDataBlock) Pack(p *Packet) error {
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
func (m *ArdupilotmegaRemoteLogDataBlock) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 206 {
		return errPayloadTooSmall
	}
	m.Seqno = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.TargetSystem = uint8(payload[4])
	m.TargetComponent = uint8(payload[5])
	copy(m.Data[:], payload[6:206])
	return nil
}

// ArdupilotmegaRemoteLogBlockStatus struct (generated typeinfo)
// Send Status of each log block that autopilot board might have sent.
type ArdupilotmegaRemoteLogBlockStatus struct {
	Seqno           uint32 // Log data block sequence number.
	TargetSystem    uint8  // System ID.
	TargetComponent uint8  // Component ID.
	Status          uint8  // Log data block status.
}

// Dialect (generated function)
func (m *ArdupilotmegaRemoteLogBlockStatus) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaRemoteLogBlockStatus) MsgID() MessageID {
	return MSG_ID_REMOTE_LOG_BLOCK_STATUS
}

// CRCExtra (generated function)
func (m *ArdupilotmegaRemoteLogBlockStatus) CRCExtra() uint8 {
	return 186
}

// MsgName (generated function)
func (m *ArdupilotmegaRemoteLogBlockStatus) MsgName() string {
	return "RemoteLogBlockStatus"
}

// String (generated function)
func (m *ArdupilotmegaRemoteLogBlockStatus) String() string {
	return fmt.Sprintf("ArdupilotmegaRemoteLogBlockStatus{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaRemoteLogBlockStatus) Pack(p *Packet) error {
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
func (m *ArdupilotmegaRemoteLogBlockStatus) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 7 {
		return errPayloadTooSmall
	}
	m.Seqno = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.TargetSystem = uint8(payload[4])
	m.TargetComponent = uint8(payload[5])
	m.Status = uint8(payload[6])
	return nil
}

// ArdupilotmegaLedControl struct (generated typeinfo)
// Control vehicle LEDs.
type ArdupilotmegaLedControl struct {
	TargetSystem    uint8     // System ID.
	TargetComponent uint8     // Component ID.
	Instance        uint8     // Instance (LED instance to control or 255 for all LEDs).
	Pattern         uint8     // Pattern (see LED_PATTERN_ENUM).
	CustomLen       uint8     // Custom Byte Length.
	CustomBytes     [24]uint8 // Custom Bytes.
}

// Dialect (generated function)
func (m *ArdupilotmegaLedControl) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaLedControl) MsgID() MessageID {
	return MSG_ID_LED_CONTROL
}

// CRCExtra (generated function)
func (m *ArdupilotmegaLedControl) CRCExtra() uint8 {
	return 72
}

// MsgName (generated function)
func (m *ArdupilotmegaLedControl) MsgName() string {
	return "LedControl"
}

// String (generated function)
func (m *ArdupilotmegaLedControl) String() string {
	return fmt.Sprintf("ArdupilotmegaLedControl{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaLedControl) Pack(p *Packet) error {
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
func (m *ArdupilotmegaLedControl) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 29 {
		return errPayloadTooSmall
	}
	m.TargetSystem = uint8(payload[0])
	m.TargetComponent = uint8(payload[1])
	m.Instance = uint8(payload[2])
	m.Pattern = uint8(payload[3])
	m.CustomLen = uint8(payload[4])
	copy(m.CustomBytes[:], payload[5:29])
	return nil
}

// ArdupilotmegaMagCalProgress struct (generated typeinfo)
// Reports progress of compass calibration.
type ArdupilotmegaMagCalProgress struct {
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

// Dialect (generated function)
func (m *ArdupilotmegaMagCalProgress) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaMagCalProgress) MsgID() MessageID {
	return MSG_ID_MAG_CAL_PROGRESS
}

// CRCExtra (generated function)
func (m *ArdupilotmegaMagCalProgress) CRCExtra() uint8 {
	return 92
}

// MsgName (generated function)
func (m *ArdupilotmegaMagCalProgress) MsgName() string {
	return "MagCalProgress"
}

// String (generated function)
func (m *ArdupilotmegaMagCalProgress) String() string {
	return fmt.Sprintf("ArdupilotmegaMagCalProgress{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaMagCalProgress) Pack(p *Packet) error {
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
func (m *ArdupilotmegaMagCalProgress) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 27 {
		return errPayloadTooSmall
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

// ArdupilotmegaEkfStatusReport struct (generated typeinfo)
// EKF Status message including flags and variances.
type ArdupilotmegaEkfStatusReport struct {
	VelocityVariance   float32 // Velocity variance.
	PosHorizVariance   float32 // Horizontal Position variance.
	PosVertVariance    float32 // Vertical Position variance.
	CompassVariance    float32 // Compass variance.
	TerrainAltVariance float32 // Terrain Altitude variance.
	Flags              uint16  // Flags.
}

// Dialect (generated function)
func (m *ArdupilotmegaEkfStatusReport) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaEkfStatusReport) MsgID() MessageID {
	return MSG_ID_EKF_STATUS_REPORT
}

// CRCExtra (generated function)
func (m *ArdupilotmegaEkfStatusReport) CRCExtra() uint8 {
	return 71
}

// MsgName (generated function)
func (m *ArdupilotmegaEkfStatusReport) MsgName() string {
	return "EkfStatusReport"
}

// String (generated function)
func (m *ArdupilotmegaEkfStatusReport) String() string {
	return fmt.Sprintf("ArdupilotmegaEkfStatusReport{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaEkfStatusReport) Pack(p *Packet) error {
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
func (m *ArdupilotmegaEkfStatusReport) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 22 {
		return errPayloadTooSmall
	}
	m.VelocityVariance = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.PosHorizVariance = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	m.PosVertVariance = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.CompassVariance = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.TerrainAltVariance = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	m.Flags = uint16(binary.LittleEndian.Uint16(payload[20:]))
	return nil
}

// ArdupilotmegaPidTuning struct (generated typeinfo)
// PID tuning information.
type ArdupilotmegaPidTuning struct {
	Desired  float32 // Desired rate.
	Achieved float32 // Achieved rate.
	Ff       float32 // FF component.
	P        float32 // P component.
	I        float32 // I component.
	D        float32 // D component.
	Axis     uint8   // Axis.
}

// Dialect (generated function)
func (m *ArdupilotmegaPidTuning) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaPidTuning) MsgID() MessageID {
	return MSG_ID_PID_TUNING
}

// CRCExtra (generated function)
func (m *ArdupilotmegaPidTuning) CRCExtra() uint8 {
	return 98
}

// MsgName (generated function)
func (m *ArdupilotmegaPidTuning) MsgName() string {
	return "PidTuning"
}

// String (generated function)
func (m *ArdupilotmegaPidTuning) String() string {
	return fmt.Sprintf("ArdupilotmegaPidTuning{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaPidTuning) Pack(p *Packet) error {
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
func (m *ArdupilotmegaPidTuning) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 25 {
		return errPayloadTooSmall
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

// ArdupilotmegaDeepstall struct (generated typeinfo)
// Deepstall path planning.
type ArdupilotmegaDeepstall struct {
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

// Dialect (generated function)
func (m *ArdupilotmegaDeepstall) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaDeepstall) MsgID() MessageID {
	return MSG_ID_DEEPSTALL
}

// CRCExtra (generated function)
func (m *ArdupilotmegaDeepstall) CRCExtra() uint8 {
	return 120
}

// MsgName (generated function)
func (m *ArdupilotmegaDeepstall) MsgName() string {
	return "Deepstall"
}

// String (generated function)
func (m *ArdupilotmegaDeepstall) String() string {
	return fmt.Sprintf("ArdupilotmegaDeepstall{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaDeepstall) Pack(p *Packet) error {
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
func (m *ArdupilotmegaDeepstall) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 37 {
		return errPayloadTooSmall
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

// ArdupilotmegaGimbalReport struct (generated typeinfo)
// 3 axis gimbal measurements.
type ArdupilotmegaGimbalReport struct {
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

// Dialect (generated function)
func (m *ArdupilotmegaGimbalReport) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaGimbalReport) MsgID() MessageID {
	return MSG_ID_GIMBAL_REPORT
}

// CRCExtra (generated function)
func (m *ArdupilotmegaGimbalReport) CRCExtra() uint8 {
	return 134
}

// MsgName (generated function)
func (m *ArdupilotmegaGimbalReport) MsgName() string {
	return "GimbalReport"
}

// String (generated function)
func (m *ArdupilotmegaGimbalReport) String() string {
	return fmt.Sprintf("ArdupilotmegaGimbalReport{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaGimbalReport) Pack(p *Packet) error {
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
func (m *ArdupilotmegaGimbalReport) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 42 {
		return errPayloadTooSmall
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

// ArdupilotmegaGimbalControl struct (generated typeinfo)
// Control message for rate gimbal.
type ArdupilotmegaGimbalControl struct {
	DemandedRateX   float32 // Demanded angular rate X.
	DemandedRateY   float32 // Demanded angular rate Y.
	DemandedRateZ   float32 // Demanded angular rate Z.
	TargetSystem    uint8   // System ID.
	TargetComponent uint8   // Component ID.
}

// Dialect (generated function)
func (m *ArdupilotmegaGimbalControl) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaGimbalControl) MsgID() MessageID {
	return MSG_ID_GIMBAL_CONTROL
}

// CRCExtra (generated function)
func (m *ArdupilotmegaGimbalControl) CRCExtra() uint8 {
	return 205
}

// MsgName (generated function)
func (m *ArdupilotmegaGimbalControl) MsgName() string {
	return "GimbalControl"
}

// String (generated function)
func (m *ArdupilotmegaGimbalControl) String() string {
	return fmt.Sprintf("ArdupilotmegaGimbalControl{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaGimbalControl) Pack(p *Packet) error {
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
func (m *ArdupilotmegaGimbalControl) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 14 {
		return errPayloadTooSmall
	}
	m.DemandedRateX = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.DemandedRateY = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	m.DemandedRateZ = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.TargetSystem = uint8(payload[12])
	m.TargetComponent = uint8(payload[13])
	return nil
}

// ArdupilotmegaGimbalTorqueCmdReport struct (generated typeinfo)
// 100 Hz gimbal torque command telemetry.
type ArdupilotmegaGimbalTorqueCmdReport struct {
	RlTorqueCmd     int16 // Roll Torque Command.
	ElTorqueCmd     int16 // Elevation Torque Command.
	AzTorqueCmd     int16 // Azimuth Torque Command.
	TargetSystem    uint8 // System ID.
	TargetComponent uint8 // Component ID.
}

// Dialect (generated function)
func (m *ArdupilotmegaGimbalTorqueCmdReport) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaGimbalTorqueCmdReport) MsgID() MessageID {
	return MSG_ID_GIMBAL_TORQUE_CMD_REPORT
}

// CRCExtra (generated function)
func (m *ArdupilotmegaGimbalTorqueCmdReport) CRCExtra() uint8 {
	return 69
}

// MsgName (generated function)
func (m *ArdupilotmegaGimbalTorqueCmdReport) MsgName() string {
	return "GimbalTorqueCmdReport"
}

// String (generated function)
func (m *ArdupilotmegaGimbalTorqueCmdReport) String() string {
	return fmt.Sprintf("ArdupilotmegaGimbalTorqueCmdReport{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaGimbalTorqueCmdReport) Pack(p *Packet) error {
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
func (m *ArdupilotmegaGimbalTorqueCmdReport) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 8 {
		return errPayloadTooSmall
	}
	m.RlTorqueCmd = int16(binary.LittleEndian.Uint16(payload[0:]))
	m.ElTorqueCmd = int16(binary.LittleEndian.Uint16(payload[2:]))
	m.AzTorqueCmd = int16(binary.LittleEndian.Uint16(payload[4:]))
	m.TargetSystem = uint8(payload[6])
	m.TargetComponent = uint8(payload[7])
	return nil
}

// ArdupilotmegaGoproHeartbeat struct (generated typeinfo)
// Heartbeat from a HeroBus attached GoPro.
type ArdupilotmegaGoproHeartbeat struct {
	Status      uint8 // Status.
	CaptureMode uint8 // Current capture mode.
	Flags       uint8 // Additional status bits.
}

// Dialect (generated function)
func (m *ArdupilotmegaGoproHeartbeat) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaGoproHeartbeat) MsgID() MessageID {
	return MSG_ID_GOPRO_HEARTBEAT
}

// CRCExtra (generated function)
func (m *ArdupilotmegaGoproHeartbeat) CRCExtra() uint8 {
	return 101
}

// MsgName (generated function)
func (m *ArdupilotmegaGoproHeartbeat) MsgName() string {
	return "GoproHeartbeat"
}

// String (generated function)
func (m *ArdupilotmegaGoproHeartbeat) String() string {
	return fmt.Sprintf("ArdupilotmegaGoproHeartbeat{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaGoproHeartbeat) Pack(p *Packet) error {
	payload := make([]byte, 3)
	payload[0] = byte(m.Status)
	payload[1] = byte(m.CaptureMode)
	payload[2] = byte(m.Flags)
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ArdupilotmegaGoproHeartbeat) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 3 {
		return errPayloadTooSmall
	}
	m.Status = uint8(payload[0])
	m.CaptureMode = uint8(payload[1])
	m.Flags = uint8(payload[2])
	return nil
}

// ArdupilotmegaGoproGetRequest struct (generated typeinfo)
// Request a GOPRO_COMMAND response from the GoPro.
type ArdupilotmegaGoproGetRequest struct {
	TargetSystem    uint8 // System ID.
	TargetComponent uint8 // Component ID.
	CmdID           uint8 // Command ID.
}

// Dialect (generated function)
func (m *ArdupilotmegaGoproGetRequest) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaGoproGetRequest) MsgID() MessageID {
	return MSG_ID_GOPRO_GET_REQUEST
}

// CRCExtra (generated function)
func (m *ArdupilotmegaGoproGetRequest) CRCExtra() uint8 {
	return 50
}

// MsgName (generated function)
func (m *ArdupilotmegaGoproGetRequest) MsgName() string {
	return "GoproGetRequest"
}

// String (generated function)
func (m *ArdupilotmegaGoproGetRequest) String() string {
	return fmt.Sprintf("ArdupilotmegaGoproGetRequest{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaGoproGetRequest) Pack(p *Packet) error {
	payload := make([]byte, 3)
	payload[0] = byte(m.TargetSystem)
	payload[1] = byte(m.TargetComponent)
	payload[2] = byte(m.CmdID)
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ArdupilotmegaGoproGetRequest) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 3 {
		return errPayloadTooSmall
	}
	m.TargetSystem = uint8(payload[0])
	m.TargetComponent = uint8(payload[1])
	m.CmdID = uint8(payload[2])
	return nil
}

// ArdupilotmegaGoproGetResponse struct (generated typeinfo)
// Response from a GOPRO_COMMAND get request.
type ArdupilotmegaGoproGetResponse struct {
	CmdID  uint8    // Command ID.
	Status uint8    // Status.
	Value  [4]uint8 // Value.
}

// Dialect (generated function)
func (m *ArdupilotmegaGoproGetResponse) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaGoproGetResponse) MsgID() MessageID {
	return MSG_ID_GOPRO_GET_RESPONSE
}

// CRCExtra (generated function)
func (m *ArdupilotmegaGoproGetResponse) CRCExtra() uint8 {
	return 202
}

// MsgName (generated function)
func (m *ArdupilotmegaGoproGetResponse) MsgName() string {
	return "GoproGetResponse"
}

// String (generated function)
func (m *ArdupilotmegaGoproGetResponse) String() string {
	return fmt.Sprintf("ArdupilotmegaGoproGetResponse{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaGoproGetResponse) Pack(p *Packet) error {
	payload := make([]byte, 6)
	payload[0] = byte(m.CmdID)
	payload[1] = byte(m.Status)
	copy(payload[2:], m.Value[:])
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ArdupilotmegaGoproGetResponse) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 6 {
		return errPayloadTooSmall
	}
	m.CmdID = uint8(payload[0])
	m.Status = uint8(payload[1])
	copy(m.Value[:], payload[2:6])
	return nil
}

// ArdupilotmegaGoproSetRequest struct (generated typeinfo)
// Request to set a GOPRO_COMMAND with a desired.
type ArdupilotmegaGoproSetRequest struct {
	TargetSystem    uint8    // System ID.
	TargetComponent uint8    // Component ID.
	CmdID           uint8    // Command ID.
	Value           [4]uint8 // Value.
}

// Dialect (generated function)
func (m *ArdupilotmegaGoproSetRequest) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaGoproSetRequest) MsgID() MessageID {
	return MSG_ID_GOPRO_SET_REQUEST
}

// CRCExtra (generated function)
func (m *ArdupilotmegaGoproSetRequest) CRCExtra() uint8 {
	return 17
}

// MsgName (generated function)
func (m *ArdupilotmegaGoproSetRequest) MsgName() string {
	return "GoproSetRequest"
}

// String (generated function)
func (m *ArdupilotmegaGoproSetRequest) String() string {
	return fmt.Sprintf("ArdupilotmegaGoproSetRequest{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaGoproSetRequest) Pack(p *Packet) error {
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
func (m *ArdupilotmegaGoproSetRequest) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 7 {
		return errPayloadTooSmall
	}
	m.TargetSystem = uint8(payload[0])
	m.TargetComponent = uint8(payload[1])
	m.CmdID = uint8(payload[2])
	copy(m.Value[:], payload[3:7])
	return nil
}

// ArdupilotmegaGoproSetResponse struct (generated typeinfo)
// Response from a GOPRO_COMMAND set request.
type ArdupilotmegaGoproSetResponse struct {
	CmdID  uint8 // Command ID.
	Status uint8 // Status.
}

// Dialect (generated function)
func (m *ArdupilotmegaGoproSetResponse) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaGoproSetResponse) MsgID() MessageID {
	return MSG_ID_GOPRO_SET_RESPONSE
}

// CRCExtra (generated function)
func (m *ArdupilotmegaGoproSetResponse) CRCExtra() uint8 {
	return 162
}

// MsgName (generated function)
func (m *ArdupilotmegaGoproSetResponse) MsgName() string {
	return "GoproSetResponse"
}

// String (generated function)
func (m *ArdupilotmegaGoproSetResponse) String() string {
	return fmt.Sprintf("ArdupilotmegaGoproSetResponse{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaGoproSetResponse) Pack(p *Packet) error {
	payload := make([]byte, 2)
	payload[0] = byte(m.CmdID)
	payload[1] = byte(m.Status)
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ArdupilotmegaGoproSetResponse) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 2 {
		return errPayloadTooSmall
	}
	m.CmdID = uint8(payload[0])
	m.Status = uint8(payload[1])
	return nil
}

// ArdupilotmegaRpm struct (generated typeinfo)
// RPM sensor output.
type ArdupilotmegaRpm struct {
	Rpm1 float32 // RPM Sensor1.
	Rpm2 float32 // RPM Sensor2.
}

// Dialect (generated function)
func (m *ArdupilotmegaRpm) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaRpm) MsgID() MessageID {
	return MSG_ID_RPM
}

// CRCExtra (generated function)
func (m *ArdupilotmegaRpm) CRCExtra() uint8 {
	return 207
}

// MsgName (generated function)
func (m *ArdupilotmegaRpm) MsgName() string {
	return "Rpm"
}

// String (generated function)
func (m *ArdupilotmegaRpm) String() string {
	return fmt.Sprintf("ArdupilotmegaRpm{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaRpm) Pack(p *Packet) error {
	payload := make([]byte, 8)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(m.Rpm1))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(m.Rpm2))
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ArdupilotmegaRpm) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 8 {
		return errPayloadTooSmall
	}
	m.Rpm1 = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.Rpm2 = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	return nil
}

// ArdupilotmegaSysStatus struct (generated typeinfo)
// The general system state. If the system is following the MAVLink standard, the system state is mainly defined by three orthogonal states/modes: The system mode, which is either LOCKED (motors shut down and locked), MANUAL (system under RC control), GUIDED (system with autonomous position control, position setpoint controlled manually) or AUTO (system guided by path/waypoint planner). The NAV_MODE defined the current flight state: LIFTOFF (often an open-loop maneuver), LANDING, WAYPOINTS or VECTOR. This represents the internal navigation state machine. The system status shows whether the system is currently active or not and if an emergency occurred. During the CRITICAL and EMERGENCY states the MAV is still considered to be active, but should start emergency procedures autonomously. After a failure occurred it should first move from active to critical to allow manual intervention and then move to emergency after a certain timeout.
type ArdupilotmegaSysStatus struct {
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

// Dialect (generated function)
func (m *ArdupilotmegaSysStatus) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaSysStatus) MsgID() MessageID {
	return MSG_ID_SYS_STATUS
}

// CRCExtra (generated function)
func (m *ArdupilotmegaSysStatus) CRCExtra() uint8 {
	return 124
}

// MsgName (generated function)
func (m *ArdupilotmegaSysStatus) MsgName() string {
	return "SysStatus"
}

// String (generated function)
func (m *ArdupilotmegaSysStatus) String() string {
	return fmt.Sprintf("ArdupilotmegaSysStatus{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaSysStatus) Pack(p *Packet) error {
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
func (m *ArdupilotmegaSysStatus) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 31 {
		return errPayloadTooSmall
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

// ArdupilotmegaSystemTime struct (generated typeinfo)
// The system time is the time of the master clock, typically the computer clock of the main onboard computer.
type ArdupilotmegaSystemTime struct {
	TimeUnixUsec uint64 // Timestamp (UNIX epoch time).
	TimeBootMs   uint32 // Timestamp (time since system boot).
}

// Dialect (generated function)
func (m *ArdupilotmegaSystemTime) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaSystemTime) MsgID() MessageID {
	return MSG_ID_SYSTEM_TIME
}

// CRCExtra (generated function)
func (m *ArdupilotmegaSystemTime) CRCExtra() uint8 {
	return 137
}

// MsgName (generated function)
func (m *ArdupilotmegaSystemTime) MsgName() string {
	return "SystemTime"
}

// String (generated function)
func (m *ArdupilotmegaSystemTime) String() string {
	return fmt.Sprintf("ArdupilotmegaSystemTime{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaSystemTime) Pack(p *Packet) error {
	payload := make([]byte, 12)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.TimeUnixUsec))
	binary.LittleEndian.PutUint32(payload[8:], uint32(m.TimeBootMs))
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ArdupilotmegaSystemTime) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 12 {
		return errPayloadTooSmall
	}
	m.TimeUnixUsec = uint64(binary.LittleEndian.Uint64(payload[0:]))
	m.TimeBootMs = uint32(binary.LittleEndian.Uint32(payload[8:]))
	return nil
}

// ArdupilotmegaPing struct (generated typeinfo)
// A ping message either requesting or responding to a ping. This allows to measure the system latencies, including serial port, radio modem and UDP connections. The ping microservice is documented at https://mavlink.io/en/services/ping.html
type ArdupilotmegaPing struct {
	TimeUsec        uint64 // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
	Seq             uint32 // PING sequence
	TargetSystem    uint8  // 0: request ping from all receiving systems. If greater than 0: message is a ping response and number is the system id of the requesting system
	TargetComponent uint8  // 0: request ping from all receiving components. If greater than 0: message is a ping response and number is the component id of the requesting component.
}

// Dialect (generated function)
func (m *ArdupilotmegaPing) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaPing) MsgID() MessageID {
	return MSG_ID_PING
}

// CRCExtra (generated function)
func (m *ArdupilotmegaPing) CRCExtra() uint8 {
	return 237
}

// MsgName (generated function)
func (m *ArdupilotmegaPing) MsgName() string {
	return "Ping"
}

// String (generated function)
func (m *ArdupilotmegaPing) String() string {
	return fmt.Sprintf("ArdupilotmegaPing{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaPing) Pack(p *Packet) error {
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
func (m *ArdupilotmegaPing) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 14 {
		return errPayloadTooSmall
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(payload[0:]))
	m.Seq = uint32(binary.LittleEndian.Uint32(payload[8:]))
	m.TargetSystem = uint8(payload[12])
	m.TargetComponent = uint8(payload[13])
	return nil
}

// ArdupilotmegaChangeOperatorControl struct (generated typeinfo)
// Request to control this MAV
type ArdupilotmegaChangeOperatorControl struct {
	TargetSystem   uint8    // System the GCS requests control for
	ControlRequest uint8    // 0: request control of this MAV, 1: Release control of this MAV
	Version        uint8    // 0: key as plaintext, 1-255: future, different hashing/encryption variants. The GCS should in general use the safest mode possible initially and then gradually move down the encryption level if it gets a NACK message indicating an encryption mismatch.
	Passkey        [25]byte // Password / Key, depending on version plaintext or encrypted. 25 or less characters, NULL terminated. The characters may involve A-Z, a-z, 0-9, and "!?,.-"
}

// Dialect (generated function)
func (m *ArdupilotmegaChangeOperatorControl) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaChangeOperatorControl) MsgID() MessageID {
	return MSG_ID_CHANGE_OPERATOR_CONTROL
}

// CRCExtra (generated function)
func (m *ArdupilotmegaChangeOperatorControl) CRCExtra() uint8 {
	return 217
}

// MsgName (generated function)
func (m *ArdupilotmegaChangeOperatorControl) MsgName() string {
	return "ChangeOperatorControl"
}

// String (generated function)
func (m *ArdupilotmegaChangeOperatorControl) String() string {
	return fmt.Sprintf("ArdupilotmegaChangeOperatorControl{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaChangeOperatorControl) Pack(p *Packet) error {
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
func (m *ArdupilotmegaChangeOperatorControl) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 28 {
		return errPayloadTooSmall
	}
	m.TargetSystem = uint8(payload[0])
	m.ControlRequest = uint8(payload[1])
	m.Version = uint8(payload[2])
	copy(m.Passkey[:], payload[3:28])
	return nil
}

// ArdupilotmegaChangeOperatorControlAck struct (generated typeinfo)
// Accept / deny control of this MAV
type ArdupilotmegaChangeOperatorControlAck struct {
	GcsSystemID    uint8 // ID of the GCS this message
	ControlRequest uint8 // 0: request control of this MAV, 1: Release control of this MAV
	Ack            uint8 // 0: ACK, 1: NACK: Wrong passkey, 2: NACK: Unsupported passkey encryption method, 3: NACK: Already under control
}

// Dialect (generated function)
func (m *ArdupilotmegaChangeOperatorControlAck) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaChangeOperatorControlAck) MsgID() MessageID {
	return MSG_ID_CHANGE_OPERATOR_CONTROL_ACK
}

// CRCExtra (generated function)
func (m *ArdupilotmegaChangeOperatorControlAck) CRCExtra() uint8 {
	return 104
}

// MsgName (generated function)
func (m *ArdupilotmegaChangeOperatorControlAck) MsgName() string {
	return "ChangeOperatorControlAck"
}

// String (generated function)
func (m *ArdupilotmegaChangeOperatorControlAck) String() string {
	return fmt.Sprintf("ArdupilotmegaChangeOperatorControlAck{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaChangeOperatorControlAck) Pack(p *Packet) error {
	payload := make([]byte, 3)
	payload[0] = byte(m.GcsSystemID)
	payload[1] = byte(m.ControlRequest)
	payload[2] = byte(m.Ack)
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ArdupilotmegaChangeOperatorControlAck) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 3 {
		return errPayloadTooSmall
	}
	m.GcsSystemID = uint8(payload[0])
	m.ControlRequest = uint8(payload[1])
	m.Ack = uint8(payload[2])
	return nil
}

// ArdupilotmegaAuthKey struct (generated typeinfo)
// Emit an encrypted signature / key identifying this system. PLEASE NOTE: This protocol has been kept simple, so transmitting the key requires an encrypted channel for true safety.
type ArdupilotmegaAuthKey struct {
	Key [32]byte // key
}

// Dialect (generated function)
func (m *ArdupilotmegaAuthKey) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaAuthKey) MsgID() MessageID {
	return MSG_ID_AUTH_KEY
}

// CRCExtra (generated function)
func (m *ArdupilotmegaAuthKey) CRCExtra() uint8 {
	return 119
}

// MsgName (generated function)
func (m *ArdupilotmegaAuthKey) MsgName() string {
	return "AuthKey"
}

// String (generated function)
func (m *ArdupilotmegaAuthKey) String() string {
	return fmt.Sprintf("ArdupilotmegaAuthKey{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaAuthKey) Pack(p *Packet) error {
	payload := make([]byte, 32)
	copy(payload[0:], m.Key[:])
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ArdupilotmegaAuthKey) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 32 {
		return errPayloadTooSmall
	}
	copy(m.Key[:], payload[0:32])
	return nil
}

// ArdupilotmegaLinkNodeStatus struct (generated typeinfo)
// Status generated in each node in the communication chain and injected into MAVLink stream.
type ArdupilotmegaLinkNodeStatus struct {
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

// Dialect (generated function)
func (m *ArdupilotmegaLinkNodeStatus) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaLinkNodeStatus) MsgID() MessageID {
	return MSG_ID_LINK_NODE_STATUS
}

// CRCExtra (generated function)
func (m *ArdupilotmegaLinkNodeStatus) CRCExtra() uint8 {
	return 117
}

// MsgName (generated function)
func (m *ArdupilotmegaLinkNodeStatus) MsgName() string {
	return "LinkNodeStatus"
}

// String (generated function)
func (m *ArdupilotmegaLinkNodeStatus) String() string {
	return fmt.Sprintf("ArdupilotmegaLinkNodeStatus{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaLinkNodeStatus) Pack(p *Packet) error {
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
func (m *ArdupilotmegaLinkNodeStatus) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 36 {
		return errPayloadTooSmall
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

// ArdupilotmegaSetMode struct (generated typeinfo)
// Set the system mode, as defined by enum MAV_MODE. There is no target component id as the mode is by definition for the overall aircraft, not only for one component.
type ArdupilotmegaSetMode struct {
	CustomMode   uint32 // The new autopilot-specific mode. This field can be ignored by an autopilot.
	TargetSystem uint8  // The system setting the mode
	BaseMode     uint8  // The new base mode.
}

// Dialect (generated function)
func (m *ArdupilotmegaSetMode) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaSetMode) MsgID() MessageID {
	return MSG_ID_SET_MODE
}

// CRCExtra (generated function)
func (m *ArdupilotmegaSetMode) CRCExtra() uint8 {
	return 89
}

// MsgName (generated function)
func (m *ArdupilotmegaSetMode) MsgName() string {
	return "SetMode"
}

// String (generated function)
func (m *ArdupilotmegaSetMode) String() string {
	return fmt.Sprintf("ArdupilotmegaSetMode{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaSetMode) Pack(p *Packet) error {
	payload := make([]byte, 6)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.CustomMode))
	payload[4] = byte(m.TargetSystem)
	payload[5] = byte(m.BaseMode)
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ArdupilotmegaSetMode) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 6 {
		return errPayloadTooSmall
	}
	m.CustomMode = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.TargetSystem = uint8(payload[4])
	m.BaseMode = uint8(payload[5])
	return nil
}

// ArdupilotmegaParamAckTransaction struct (generated typeinfo)
// Response from a PARAM_SET message when it is used in a transaction.
type ArdupilotmegaParamAckTransaction struct {
	ParamValue      float32  // Parameter value (new value if PARAM_ACCEPTED, current value otherwise)
	TargetSystem    uint8    // Id of system that sent PARAM_SET message.
	TargetComponent uint8    // Id of system that sent PARAM_SET message.
	ParamID         [16]byte // Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
	ParamType       uint8    // Parameter type.
	ParamResult     uint8    // Result code.
}

// Dialect (generated function)
func (m *ArdupilotmegaParamAckTransaction) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaParamAckTransaction) MsgID() MessageID {
	return MSG_ID_PARAM_ACK_TRANSACTION
}

// CRCExtra (generated function)
func (m *ArdupilotmegaParamAckTransaction) CRCExtra() uint8 {
	return 137
}

// MsgName (generated function)
func (m *ArdupilotmegaParamAckTransaction) MsgName() string {
	return "ParamAckTransaction"
}

// String (generated function)
func (m *ArdupilotmegaParamAckTransaction) String() string {
	return fmt.Sprintf("ArdupilotmegaParamAckTransaction{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaParamAckTransaction) Pack(p *Packet) error {
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
func (m *ArdupilotmegaParamAckTransaction) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 24 {
		return errPayloadTooSmall
	}
	m.ParamValue = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.TargetSystem = uint8(payload[4])
	m.TargetComponent = uint8(payload[5])
	copy(m.ParamID[:], payload[6:22])
	m.ParamType = uint8(payload[22])
	m.ParamResult = uint8(payload[23])
	return nil
}

// ArdupilotmegaParamRequestRead struct (generated typeinfo)
// Request to read the onboard parameter with the param_id string id. Onboard parameters are stored as key[const char*] -> value[float]. This allows to send a parameter to any other component (such as the GCS) without the need of previous knowledge of possible parameter names. Thus the same GCS can store different parameters for different autopilots. See also https://mavlink.io/en/services/parameter.html for a full documentation of QGroundControl and IMU code.
type ArdupilotmegaParamRequestRead struct {
	ParamIndex      int16    // Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignored)
	TargetSystem    uint8    // System ID
	TargetComponent uint8    // Component ID
	ParamID         [16]byte // Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
}

// Dialect (generated function)
func (m *ArdupilotmegaParamRequestRead) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaParamRequestRead) MsgID() MessageID {
	return MSG_ID_PARAM_REQUEST_READ
}

// CRCExtra (generated function)
func (m *ArdupilotmegaParamRequestRead) CRCExtra() uint8 {
	return 214
}

// MsgName (generated function)
func (m *ArdupilotmegaParamRequestRead) MsgName() string {
	return "ParamRequestRead"
}

// String (generated function)
func (m *ArdupilotmegaParamRequestRead) String() string {
	return fmt.Sprintf("ArdupilotmegaParamRequestRead{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaParamRequestRead) Pack(p *Packet) error {
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
func (m *ArdupilotmegaParamRequestRead) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 20 {
		return errPayloadTooSmall
	}
	m.ParamIndex = int16(binary.LittleEndian.Uint16(payload[0:]))
	m.TargetSystem = uint8(payload[2])
	m.TargetComponent = uint8(payload[3])
	copy(m.ParamID[:], payload[4:20])
	return nil
}

// ArdupilotmegaParamRequestList struct (generated typeinfo)
// Request all parameters of this component. After this request, all parameters are emitted. The parameter microservice is documented at https://mavlink.io/en/services/parameter.html
type ArdupilotmegaParamRequestList struct {
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
}

// Dialect (generated function)
func (m *ArdupilotmegaParamRequestList) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaParamRequestList) MsgID() MessageID {
	return MSG_ID_PARAM_REQUEST_LIST
}

// CRCExtra (generated function)
func (m *ArdupilotmegaParamRequestList) CRCExtra() uint8 {
	return 159
}

// MsgName (generated function)
func (m *ArdupilotmegaParamRequestList) MsgName() string {
	return "ParamRequestList"
}

// String (generated function)
func (m *ArdupilotmegaParamRequestList) String() string {
	return fmt.Sprintf("ArdupilotmegaParamRequestList{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaParamRequestList) Pack(p *Packet) error {
	payload := make([]byte, 2)
	payload[0] = byte(m.TargetSystem)
	payload[1] = byte(m.TargetComponent)
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ArdupilotmegaParamRequestList) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 2 {
		return errPayloadTooSmall
	}
	m.TargetSystem = uint8(payload[0])
	m.TargetComponent = uint8(payload[1])
	return nil
}

// ArdupilotmegaParamValue struct (generated typeinfo)
// Emit the value of a onboard parameter. The inclusion of param_count and param_index in the message allows the recipient to keep track of received parameters and allows him to re-request missing parameters after a loss or timeout. The parameter microservice is documented at https://mavlink.io/en/services/parameter.html
type ArdupilotmegaParamValue struct {
	ParamValue float32  // Onboard parameter value
	ParamCount uint16   // Total number of onboard parameters
	ParamIndex uint16   // Index of this onboard parameter
	ParamID    [16]byte // Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
	ParamType  uint8    // Onboard parameter type.
}

// Dialect (generated function)
func (m *ArdupilotmegaParamValue) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaParamValue) MsgID() MessageID {
	return MSG_ID_PARAM_VALUE
}

// CRCExtra (generated function)
func (m *ArdupilotmegaParamValue) CRCExtra() uint8 {
	return 220
}

// MsgName (generated function)
func (m *ArdupilotmegaParamValue) MsgName() string {
	return "ParamValue"
}

// String (generated function)
func (m *ArdupilotmegaParamValue) String() string {
	return fmt.Sprintf("ArdupilotmegaParamValue{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaParamValue) Pack(p *Packet) error {
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
func (m *ArdupilotmegaParamValue) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 25 {
		return errPayloadTooSmall
	}
	m.ParamValue = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.ParamCount = uint16(binary.LittleEndian.Uint16(payload[4:]))
	m.ParamIndex = uint16(binary.LittleEndian.Uint16(payload[6:]))
	copy(m.ParamID[:], payload[8:24])
	m.ParamType = uint8(payload[24])
	return nil
}

// ArdupilotmegaParamSet struct (generated typeinfo)
// Set a parameter value (write new value to permanent storage).
//         The receiving component should acknowledge the new parameter value by broadcasting a PARAM_VALUE message (broadcasting ensures that multiple GCS all have an up-to-date list of all parameters). If the sending GCS did not receive a PARAM_VALUE within its timeout time, it should re-send the PARAM_SET message. The parameter microservice is documented at https://mavlink.io/en/services/parameter.html.
//         PARAM_SET may also be called within the context of a transaction (started with MAV_CMD_PARAM_TRANSACTION). Within a transaction the receiving component should respond with PARAM_ACK_TRANSACTION to the setter component (instead of broadcasting PARAM_VALUE), and PARAM_SET should be re-sent if this is ACK not received.
type ArdupilotmegaParamSet struct {
	ParamValue      float32  // Onboard parameter value
	TargetSystem    uint8    // System ID
	TargetComponent uint8    // Component ID
	ParamID         [16]byte // Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
	ParamType       uint8    // Onboard parameter type.
}

// Dialect (generated function)
func (m *ArdupilotmegaParamSet) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaParamSet) MsgID() MessageID {
	return MSG_ID_PARAM_SET
}

// CRCExtra (generated function)
func (m *ArdupilotmegaParamSet) CRCExtra() uint8 {
	return 168
}

// MsgName (generated function)
func (m *ArdupilotmegaParamSet) MsgName() string {
	return "ParamSet"
}

// String (generated function)
func (m *ArdupilotmegaParamSet) String() string {
	return fmt.Sprintf("ArdupilotmegaParamSet{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaParamSet) Pack(p *Packet) error {
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
func (m *ArdupilotmegaParamSet) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 23 {
		return errPayloadTooSmall
	}
	m.ParamValue = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.TargetSystem = uint8(payload[4])
	m.TargetComponent = uint8(payload[5])
	copy(m.ParamID[:], payload[6:22])
	m.ParamType = uint8(payload[22])
	return nil
}

// ArdupilotmegaGpsRawInt struct (generated typeinfo)
// The global position, as returned by the Global Positioning System (GPS). This is
//                 NOT the global position estimate of the system, but rather a RAW sensor value. See message GLOBAL_POSITION for the global position estimate.
type ArdupilotmegaGpsRawInt struct {
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

// Dialect (generated function)
func (m *ArdupilotmegaGpsRawInt) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaGpsRawInt) MsgID() MessageID {
	return MSG_ID_GPS_RAW_INT
}

// CRCExtra (generated function)
func (m *ArdupilotmegaGpsRawInt) CRCExtra() uint8 {
	return 24
}

// MsgName (generated function)
func (m *ArdupilotmegaGpsRawInt) MsgName() string {
	return "GpsRawInt"
}

// String (generated function)
func (m *ArdupilotmegaGpsRawInt) String() string {
	return fmt.Sprintf("ArdupilotmegaGpsRawInt{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaGpsRawInt) Pack(p *Packet) error {
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
func (m *ArdupilotmegaGpsRawInt) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 30 {
		return errPayloadTooSmall
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

// ArdupilotmegaGpsStatus struct (generated typeinfo)
// The positioning status, as reported by GPS. This message is intended to display status information about each satellite visible to the receiver. See message GLOBAL_POSITION for the global position estimate. This message can contain information for up to 20 satellites.
type ArdupilotmegaGpsStatus struct {
	SatellitesVisible  uint8     // Number of satellites visible
	SatellitePrn       [20]uint8 // Global satellite ID
	SatelliteUsed      [20]uint8 // 0: Satellite not used, 1: used for localization
	SatelliteElevation [20]uint8 // Elevation (0: right on top of receiver, 90: on the horizon) of satellite
	SatelliteAzimuth   [20]uint8 // Direction of satellite, 0: 0 deg, 255: 360 deg.
	SatelliteSnr       [20]uint8 // Signal to noise ratio of satellite
}

// Dialect (generated function)
func (m *ArdupilotmegaGpsStatus) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaGpsStatus) MsgID() MessageID {
	return MSG_ID_GPS_STATUS
}

// CRCExtra (generated function)
func (m *ArdupilotmegaGpsStatus) CRCExtra() uint8 {
	return 23
}

// MsgName (generated function)
func (m *ArdupilotmegaGpsStatus) MsgName() string {
	return "GpsStatus"
}

// String (generated function)
func (m *ArdupilotmegaGpsStatus) String() string {
	return fmt.Sprintf("ArdupilotmegaGpsStatus{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaGpsStatus) Pack(p *Packet) error {
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
func (m *ArdupilotmegaGpsStatus) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 101 {
		return errPayloadTooSmall
	}
	m.SatellitesVisible = uint8(payload[0])
	copy(m.SatellitePrn[:], payload[1:21])
	copy(m.SatelliteUsed[:], payload[21:41])
	copy(m.SatelliteElevation[:], payload[41:61])
	copy(m.SatelliteAzimuth[:], payload[61:81])
	copy(m.SatelliteSnr[:], payload[81:101])
	return nil
}

// ArdupilotmegaScaledImu struct (generated typeinfo)
// The RAW IMU readings for the usual 9DOF sensor setup. This message should contain the scaled values to the described units
type ArdupilotmegaScaledImu struct {
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

// Dialect (generated function)
func (m *ArdupilotmegaScaledImu) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaScaledImu) MsgID() MessageID {
	return MSG_ID_SCALED_IMU
}

// CRCExtra (generated function)
func (m *ArdupilotmegaScaledImu) CRCExtra() uint8 {
	return 170
}

// MsgName (generated function)
func (m *ArdupilotmegaScaledImu) MsgName() string {
	return "ScaledImu"
}

// String (generated function)
func (m *ArdupilotmegaScaledImu) String() string {
	return fmt.Sprintf("ArdupilotmegaScaledImu{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaScaledImu) Pack(p *Packet) error {
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
func (m *ArdupilotmegaScaledImu) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 22 {
		return errPayloadTooSmall
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

// ArdupilotmegaRawImu struct (generated typeinfo)
// The RAW IMU readings for a 9DOF sensor, which is identified by the id (default IMU1). This message should always contain the true raw values without any scaling to allow data capture and system debugging.
type ArdupilotmegaRawImu struct {
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

// Dialect (generated function)
func (m *ArdupilotmegaRawImu) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaRawImu) MsgID() MessageID {
	return MSG_ID_RAW_IMU
}

// CRCExtra (generated function)
func (m *ArdupilotmegaRawImu) CRCExtra() uint8 {
	return 144
}

// MsgName (generated function)
func (m *ArdupilotmegaRawImu) MsgName() string {
	return "RawImu"
}

// String (generated function)
func (m *ArdupilotmegaRawImu) String() string {
	return fmt.Sprintf("ArdupilotmegaRawImu{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaRawImu) Pack(p *Packet) error {
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
func (m *ArdupilotmegaRawImu) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 26 {
		return errPayloadTooSmall
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

// ArdupilotmegaRawPressure struct (generated typeinfo)
// The RAW pressure readings for the typical setup of one absolute pressure and one differential pressure sensor. The sensor values should be the raw, UNSCALED ADC values.
type ArdupilotmegaRawPressure struct {
	TimeUsec    uint64 // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
	PressAbs    int16  // Absolute pressure (raw)
	PressDiff1  int16  // Differential pressure 1 (raw, 0 if nonexistent)
	PressDiff2  int16  // Differential pressure 2 (raw, 0 if nonexistent)
	Temperature int16  // Raw Temperature measurement (raw)
}

// Dialect (generated function)
func (m *ArdupilotmegaRawPressure) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaRawPressure) MsgID() MessageID {
	return MSG_ID_RAW_PRESSURE
}

// CRCExtra (generated function)
func (m *ArdupilotmegaRawPressure) CRCExtra() uint8 {
	return 67
}

// MsgName (generated function)
func (m *ArdupilotmegaRawPressure) MsgName() string {
	return "RawPressure"
}

// String (generated function)
func (m *ArdupilotmegaRawPressure) String() string {
	return fmt.Sprintf("ArdupilotmegaRawPressure{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaRawPressure) Pack(p *Packet) error {
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
func (m *ArdupilotmegaRawPressure) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 16 {
		return errPayloadTooSmall
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(payload[0:]))
	m.PressAbs = int16(binary.LittleEndian.Uint16(payload[8:]))
	m.PressDiff1 = int16(binary.LittleEndian.Uint16(payload[10:]))
	m.PressDiff2 = int16(binary.LittleEndian.Uint16(payload[12:]))
	m.Temperature = int16(binary.LittleEndian.Uint16(payload[14:]))
	return nil
}

// ArdupilotmegaScaledPressure struct (generated typeinfo)
// The pressure readings for the typical setup of one absolute and differential pressure sensor. The units are as specified in each field.
type ArdupilotmegaScaledPressure struct {
	TimeBootMs  uint32  // Timestamp (time since system boot).
	PressAbs    float32 // Absolute pressure
	PressDiff   float32 // Differential pressure 1
	Temperature int16   // Absolute pressure temperature
}

// Dialect (generated function)
func (m *ArdupilotmegaScaledPressure) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaScaledPressure) MsgID() MessageID {
	return MSG_ID_SCALED_PRESSURE
}

// CRCExtra (generated function)
func (m *ArdupilotmegaScaledPressure) CRCExtra() uint8 {
	return 115
}

// MsgName (generated function)
func (m *ArdupilotmegaScaledPressure) MsgName() string {
	return "ScaledPressure"
}

// String (generated function)
func (m *ArdupilotmegaScaledPressure) String() string {
	return fmt.Sprintf("ArdupilotmegaScaledPressure{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaScaledPressure) Pack(p *Packet) error {
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
func (m *ArdupilotmegaScaledPressure) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 14 {
		return errPayloadTooSmall
	}
	m.TimeBootMs = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.PressAbs = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	m.PressDiff = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.Temperature = int16(binary.LittleEndian.Uint16(payload[12:]))
	return nil
}

// ArdupilotmegaAttitude struct (generated typeinfo)
// The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right).
type ArdupilotmegaAttitude struct {
	TimeBootMs uint32  // Timestamp (time since system boot).
	Roll       float32 // Roll angle (-pi..+pi)
	Pitch      float32 // Pitch angle (-pi..+pi)
	Yaw        float32 // Yaw angle (-pi..+pi)
	Rollspeed  float32 // Roll angular speed
	Pitchspeed float32 // Pitch angular speed
	Yawspeed   float32 // Yaw angular speed
}

// Dialect (generated function)
func (m *ArdupilotmegaAttitude) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaAttitude) MsgID() MessageID {
	return MSG_ID_ATTITUDE
}

// CRCExtra (generated function)
func (m *ArdupilotmegaAttitude) CRCExtra() uint8 {
	return 39
}

// MsgName (generated function)
func (m *ArdupilotmegaAttitude) MsgName() string {
	return "Attitude"
}

// String (generated function)
func (m *ArdupilotmegaAttitude) String() string {
	return fmt.Sprintf("ArdupilotmegaAttitude{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaAttitude) Pack(p *Packet) error {
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
func (m *ArdupilotmegaAttitude) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 28 {
		return errPayloadTooSmall
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

// ArdupilotmegaAttitudeQuaternion struct (generated typeinfo)
// The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right), expressed as quaternion. Quaternion order is w, x, y, z and a zero rotation would be expressed as (1 0 0 0).
type ArdupilotmegaAttitudeQuaternion struct {
	TimeBootMs uint32  // Timestamp (time since system boot).
	Q1         float32 // Quaternion component 1, w (1 in null-rotation)
	Q2         float32 // Quaternion component 2, x (0 in null-rotation)
	Q3         float32 // Quaternion component 3, y (0 in null-rotation)
	Q4         float32 // Quaternion component 4, z (0 in null-rotation)
	Rollspeed  float32 // Roll angular speed
	Pitchspeed float32 // Pitch angular speed
	Yawspeed   float32 // Yaw angular speed
}

// Dialect (generated function)
func (m *ArdupilotmegaAttitudeQuaternion) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaAttitudeQuaternion) MsgID() MessageID {
	return MSG_ID_ATTITUDE_QUATERNION
}

// CRCExtra (generated function)
func (m *ArdupilotmegaAttitudeQuaternion) CRCExtra() uint8 {
	return 246
}

// MsgName (generated function)
func (m *ArdupilotmegaAttitudeQuaternion) MsgName() string {
	return "AttitudeQuaternion"
}

// String (generated function)
func (m *ArdupilotmegaAttitudeQuaternion) String() string {
	return fmt.Sprintf("ArdupilotmegaAttitudeQuaternion{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaAttitudeQuaternion) Pack(p *Packet) error {
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
func (m *ArdupilotmegaAttitudeQuaternion) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 32 {
		return errPayloadTooSmall
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

// ArdupilotmegaLocalPositionNed struct (generated typeinfo)
// The filtered local position (e.g. fused computer vision and accelerometers). Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down convention)
type ArdupilotmegaLocalPositionNed struct {
	TimeBootMs uint32  // Timestamp (time since system boot).
	X          float32 // X Position
	Y          float32 // Y Position
	Z          float32 // Z Position
	Vx         float32 // X Speed
	Vy         float32 // Y Speed
	Vz         float32 // Z Speed
}

// Dialect (generated function)
func (m *ArdupilotmegaLocalPositionNed) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaLocalPositionNed) MsgID() MessageID {
	return MSG_ID_LOCAL_POSITION_NED
}

// CRCExtra (generated function)
func (m *ArdupilotmegaLocalPositionNed) CRCExtra() uint8 {
	return 185
}

// MsgName (generated function)
func (m *ArdupilotmegaLocalPositionNed) MsgName() string {
	return "LocalPositionNed"
}

// String (generated function)
func (m *ArdupilotmegaLocalPositionNed) String() string {
	return fmt.Sprintf("ArdupilotmegaLocalPositionNed{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaLocalPositionNed) Pack(p *Packet) error {
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
func (m *ArdupilotmegaLocalPositionNed) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 28 {
		return errPayloadTooSmall
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

// ArdupilotmegaGlobalPositionInt struct (generated typeinfo)
// The filtered global position (e.g. fused GPS and accelerometers). The position is in GPS-frame (right-handed, Z-up). It
//                is designed as scaled integer message since the resolution of float is not sufficient.
type ArdupilotmegaGlobalPositionInt struct {
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

// Dialect (generated function)
func (m *ArdupilotmegaGlobalPositionInt) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaGlobalPositionInt) MsgID() MessageID {
	return MSG_ID_GLOBAL_POSITION_INT
}

// CRCExtra (generated function)
func (m *ArdupilotmegaGlobalPositionInt) CRCExtra() uint8 {
	return 104
}

// MsgName (generated function)
func (m *ArdupilotmegaGlobalPositionInt) MsgName() string {
	return "GlobalPositionInt"
}

// String (generated function)
func (m *ArdupilotmegaGlobalPositionInt) String() string {
	return fmt.Sprintf("ArdupilotmegaGlobalPositionInt{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaGlobalPositionInt) Pack(p *Packet) error {
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
func (m *ArdupilotmegaGlobalPositionInt) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 28 {
		return errPayloadTooSmall
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

// ArdupilotmegaRcChannelsScaled struct (generated typeinfo)
// The scaled values of the RC channels received: (-100%) -10000, (0%) 0, (100%) 10000. Channels that are inactive should be set to UINT16_MAX.
type ArdupilotmegaRcChannelsScaled struct {
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

// Dialect (generated function)
func (m *ArdupilotmegaRcChannelsScaled) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaRcChannelsScaled) MsgID() MessageID {
	return MSG_ID_RC_CHANNELS_SCALED
}

// CRCExtra (generated function)
func (m *ArdupilotmegaRcChannelsScaled) CRCExtra() uint8 {
	return 237
}

// MsgName (generated function)
func (m *ArdupilotmegaRcChannelsScaled) MsgName() string {
	return "RcChannelsScaled"
}

// String (generated function)
func (m *ArdupilotmegaRcChannelsScaled) String() string {
	return fmt.Sprintf("ArdupilotmegaRcChannelsScaled{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaRcChannelsScaled) Pack(p *Packet) error {
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
func (m *ArdupilotmegaRcChannelsScaled) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 22 {
		return errPayloadTooSmall
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

// ArdupilotmegaRcChannelsRaw struct (generated typeinfo)
// The RAW values of the RC channels received. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. A value of UINT16_MAX implies the channel is unused. Individual receivers/transmitters might violate this specification.
type ArdupilotmegaRcChannelsRaw struct {
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

// Dialect (generated function)
func (m *ArdupilotmegaRcChannelsRaw) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaRcChannelsRaw) MsgID() MessageID {
	return MSG_ID_RC_CHANNELS_RAW
}

// CRCExtra (generated function)
func (m *ArdupilotmegaRcChannelsRaw) CRCExtra() uint8 {
	return 244
}

// MsgName (generated function)
func (m *ArdupilotmegaRcChannelsRaw) MsgName() string {
	return "RcChannelsRaw"
}

// String (generated function)
func (m *ArdupilotmegaRcChannelsRaw) String() string {
	return fmt.Sprintf("ArdupilotmegaRcChannelsRaw{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaRcChannelsRaw) Pack(p *Packet) error {
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
func (m *ArdupilotmegaRcChannelsRaw) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 22 {
		return errPayloadTooSmall
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

// ArdupilotmegaServoOutputRaw struct (generated typeinfo)
// Superseded by ACTUATOR_OUTPUT_STATUS. The RAW values of the servo outputs (for RC input from the remote, use the RC_CHANNELS messages). The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%.
type ArdupilotmegaServoOutputRaw struct {
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

// Dialect (generated function)
func (m *ArdupilotmegaServoOutputRaw) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaServoOutputRaw) MsgID() MessageID {
	return MSG_ID_SERVO_OUTPUT_RAW
}

// CRCExtra (generated function)
func (m *ArdupilotmegaServoOutputRaw) CRCExtra() uint8 {
	return 222
}

// MsgName (generated function)
func (m *ArdupilotmegaServoOutputRaw) MsgName() string {
	return "ServoOutputRaw"
}

// String (generated function)
func (m *ArdupilotmegaServoOutputRaw) String() string {
	return fmt.Sprintf("ArdupilotmegaServoOutputRaw{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaServoOutputRaw) Pack(p *Packet) error {
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
func (m *ArdupilotmegaServoOutputRaw) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 21 {
		return errPayloadTooSmall
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

// ArdupilotmegaMissionRequestPartialList struct (generated typeinfo)
// Request a partial list of mission items from the system/component. https://mavlink.io/en/services/mission.html. If start and end index are the same, just send one waypoint.
type ArdupilotmegaMissionRequestPartialList struct {
	StartIndex      int16 // Start index
	EndIndex        int16 // End index, -1 by default (-1: send list to end). Else a valid index of the list
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
}

// Dialect (generated function)
func (m *ArdupilotmegaMissionRequestPartialList) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaMissionRequestPartialList) MsgID() MessageID {
	return MSG_ID_MISSION_REQUEST_PARTIAL_LIST
}

// CRCExtra (generated function)
func (m *ArdupilotmegaMissionRequestPartialList) CRCExtra() uint8 {
	return 212
}

// MsgName (generated function)
func (m *ArdupilotmegaMissionRequestPartialList) MsgName() string {
	return "MissionRequestPartialList"
}

// String (generated function)
func (m *ArdupilotmegaMissionRequestPartialList) String() string {
	return fmt.Sprintf("ArdupilotmegaMissionRequestPartialList{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaMissionRequestPartialList) Pack(p *Packet) error {
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
func (m *ArdupilotmegaMissionRequestPartialList) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 6 {
		return errPayloadTooSmall
	}
	m.StartIndex = int16(binary.LittleEndian.Uint16(payload[0:]))
	m.EndIndex = int16(binary.LittleEndian.Uint16(payload[2:]))
	m.TargetSystem = uint8(payload[4])
	m.TargetComponent = uint8(payload[5])
	return nil
}

// ArdupilotmegaMissionWritePartialList struct (generated typeinfo)
// This message is sent to the MAV to write a partial list. If start index == end index, only one item will be transmitted / updated. If the start index is NOT 0 and above the current list size, this request should be REJECTED!
type ArdupilotmegaMissionWritePartialList struct {
	StartIndex      int16 // Start index. Must be smaller / equal to the largest index of the current onboard list.
	EndIndex        int16 // End index, equal or greater than start index.
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
}

// Dialect (generated function)
func (m *ArdupilotmegaMissionWritePartialList) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaMissionWritePartialList) MsgID() MessageID {
	return MSG_ID_MISSION_WRITE_PARTIAL_LIST
}

// CRCExtra (generated function)
func (m *ArdupilotmegaMissionWritePartialList) CRCExtra() uint8 {
	return 9
}

// MsgName (generated function)
func (m *ArdupilotmegaMissionWritePartialList) MsgName() string {
	return "MissionWritePartialList"
}

// String (generated function)
func (m *ArdupilotmegaMissionWritePartialList) String() string {
	return fmt.Sprintf("ArdupilotmegaMissionWritePartialList{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaMissionWritePartialList) Pack(p *Packet) error {
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
func (m *ArdupilotmegaMissionWritePartialList) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 6 {
		return errPayloadTooSmall
	}
	m.StartIndex = int16(binary.LittleEndian.Uint16(payload[0:]))
	m.EndIndex = int16(binary.LittleEndian.Uint16(payload[2:]))
	m.TargetSystem = uint8(payload[4])
	m.TargetComponent = uint8(payload[5])
	return nil
}

// ArdupilotmegaMissionItem struct (generated typeinfo)
// Message encoding a mission item. This message is emitted to announce
//                 the presence of a mission item and to set a mission item on the system. The mission item can be either in x, y, z meters (type: LOCAL) or x:lat, y:lon, z:altitude. Local frame is Z-down, right handed (NED), global frame is Z-up, right handed (ENU). NaN may be used to indicate an optional/default value (e.g. to use the system's current latitude or yaw rather than a specific value). See also https://mavlink.io/en/services/mission.html.
type ArdupilotmegaMissionItem struct {
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

// Dialect (generated function)
func (m *ArdupilotmegaMissionItem) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaMissionItem) MsgID() MessageID {
	return MSG_ID_MISSION_ITEM
}

// CRCExtra (generated function)
func (m *ArdupilotmegaMissionItem) CRCExtra() uint8 {
	return 254
}

// MsgName (generated function)
func (m *ArdupilotmegaMissionItem) MsgName() string {
	return "MissionItem"
}

// String (generated function)
func (m *ArdupilotmegaMissionItem) String() string {
	return fmt.Sprintf("ArdupilotmegaMissionItem{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaMissionItem) Pack(p *Packet) error {
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
func (m *ArdupilotmegaMissionItem) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 37 {
		return errPayloadTooSmall
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

// ArdupilotmegaMissionRequest struct (generated typeinfo)
// Request the information of the mission item with the sequence number seq. The response of the system to this message should be a MISSION_ITEM message. https://mavlink.io/en/services/mission.html
type ArdupilotmegaMissionRequest struct {
	Seq             uint16 // Sequence
	TargetSystem    uint8  // System ID
	TargetComponent uint8  // Component ID
}

// Dialect (generated function)
func (m *ArdupilotmegaMissionRequest) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaMissionRequest) MsgID() MessageID {
	return MSG_ID_MISSION_REQUEST
}

// CRCExtra (generated function)
func (m *ArdupilotmegaMissionRequest) CRCExtra() uint8 {
	return 230
}

// MsgName (generated function)
func (m *ArdupilotmegaMissionRequest) MsgName() string {
	return "MissionRequest"
}

// String (generated function)
func (m *ArdupilotmegaMissionRequest) String() string {
	return fmt.Sprintf("ArdupilotmegaMissionRequest{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaMissionRequest) Pack(p *Packet) error {
	payload := make([]byte, 4)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.Seq))
	payload[2] = byte(m.TargetSystem)
	payload[3] = byte(m.TargetComponent)
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ArdupilotmegaMissionRequest) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 4 {
		return errPayloadTooSmall
	}
	m.Seq = uint16(binary.LittleEndian.Uint16(payload[0:]))
	m.TargetSystem = uint8(payload[2])
	m.TargetComponent = uint8(payload[3])
	return nil
}

// ArdupilotmegaMissionSetCurrent struct (generated typeinfo)
// Set the mission item with sequence number seq as current item. This means that the MAV will continue to this mission item on the shortest path (not following the mission items in-between).
type ArdupilotmegaMissionSetCurrent struct {
	Seq             uint16 // Sequence
	TargetSystem    uint8  // System ID
	TargetComponent uint8  // Component ID
}

// Dialect (generated function)
func (m *ArdupilotmegaMissionSetCurrent) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaMissionSetCurrent) MsgID() MessageID {
	return MSG_ID_MISSION_SET_CURRENT
}

// CRCExtra (generated function)
func (m *ArdupilotmegaMissionSetCurrent) CRCExtra() uint8 {
	return 28
}

// MsgName (generated function)
func (m *ArdupilotmegaMissionSetCurrent) MsgName() string {
	return "MissionSetCurrent"
}

// String (generated function)
func (m *ArdupilotmegaMissionSetCurrent) String() string {
	return fmt.Sprintf("ArdupilotmegaMissionSetCurrent{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaMissionSetCurrent) Pack(p *Packet) error {
	payload := make([]byte, 4)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.Seq))
	payload[2] = byte(m.TargetSystem)
	payload[3] = byte(m.TargetComponent)
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ArdupilotmegaMissionSetCurrent) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 4 {
		return errPayloadTooSmall
	}
	m.Seq = uint16(binary.LittleEndian.Uint16(payload[0:]))
	m.TargetSystem = uint8(payload[2])
	m.TargetComponent = uint8(payload[3])
	return nil
}

// ArdupilotmegaMissionCurrent struct (generated typeinfo)
// Message that announces the sequence number of the current active mission item. The MAV will fly towards this mission item.
type ArdupilotmegaMissionCurrent struct {
	Seq uint16 // Sequence
}

// Dialect (generated function)
func (m *ArdupilotmegaMissionCurrent) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaMissionCurrent) MsgID() MessageID {
	return MSG_ID_MISSION_CURRENT
}

// CRCExtra (generated function)
func (m *ArdupilotmegaMissionCurrent) CRCExtra() uint8 {
	return 28
}

// MsgName (generated function)
func (m *ArdupilotmegaMissionCurrent) MsgName() string {
	return "MissionCurrent"
}

// String (generated function)
func (m *ArdupilotmegaMissionCurrent) String() string {
	return fmt.Sprintf("ArdupilotmegaMissionCurrent{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaMissionCurrent) Pack(p *Packet) error {
	payload := make([]byte, 2)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.Seq))
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ArdupilotmegaMissionCurrent) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 2 {
		return errPayloadTooSmall
	}
	m.Seq = uint16(binary.LittleEndian.Uint16(payload[0:]))
	return nil
}

// ArdupilotmegaMissionRequestList struct (generated typeinfo)
// Request the overall list of mission items from the system/component.
type ArdupilotmegaMissionRequestList struct {
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
}

// Dialect (generated function)
func (m *ArdupilotmegaMissionRequestList) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaMissionRequestList) MsgID() MessageID {
	return MSG_ID_MISSION_REQUEST_LIST
}

// CRCExtra (generated function)
func (m *ArdupilotmegaMissionRequestList) CRCExtra() uint8 {
	return 132
}

// MsgName (generated function)
func (m *ArdupilotmegaMissionRequestList) MsgName() string {
	return "MissionRequestList"
}

// String (generated function)
func (m *ArdupilotmegaMissionRequestList) String() string {
	return fmt.Sprintf("ArdupilotmegaMissionRequestList{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaMissionRequestList) Pack(p *Packet) error {
	payload := make([]byte, 2)
	payload[0] = byte(m.TargetSystem)
	payload[1] = byte(m.TargetComponent)
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ArdupilotmegaMissionRequestList) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 2 {
		return errPayloadTooSmall
	}
	m.TargetSystem = uint8(payload[0])
	m.TargetComponent = uint8(payload[1])
	return nil
}

// ArdupilotmegaMissionCount struct (generated typeinfo)
// This message is emitted as response to MISSION_REQUEST_LIST by the MAV and to initiate a write transaction. The GCS can then request the individual mission item based on the knowledge of the total number of waypoints.
type ArdupilotmegaMissionCount struct {
	Count           uint16 // Number of mission items in the sequence
	TargetSystem    uint8  // System ID
	TargetComponent uint8  // Component ID
}

// Dialect (generated function)
func (m *ArdupilotmegaMissionCount) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaMissionCount) MsgID() MessageID {
	return MSG_ID_MISSION_COUNT
}

// CRCExtra (generated function)
func (m *ArdupilotmegaMissionCount) CRCExtra() uint8 {
	return 221
}

// MsgName (generated function)
func (m *ArdupilotmegaMissionCount) MsgName() string {
	return "MissionCount"
}

// String (generated function)
func (m *ArdupilotmegaMissionCount) String() string {
	return fmt.Sprintf("ArdupilotmegaMissionCount{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaMissionCount) Pack(p *Packet) error {
	payload := make([]byte, 4)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.Count))
	payload[2] = byte(m.TargetSystem)
	payload[3] = byte(m.TargetComponent)
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ArdupilotmegaMissionCount) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 4 {
		return errPayloadTooSmall
	}
	m.Count = uint16(binary.LittleEndian.Uint16(payload[0:]))
	m.TargetSystem = uint8(payload[2])
	m.TargetComponent = uint8(payload[3])
	return nil
}

// ArdupilotmegaMissionClearAll struct (generated typeinfo)
// Delete all mission items at once.
type ArdupilotmegaMissionClearAll struct {
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
}

// Dialect (generated function)
func (m *ArdupilotmegaMissionClearAll) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaMissionClearAll) MsgID() MessageID {
	return MSG_ID_MISSION_CLEAR_ALL
}

// CRCExtra (generated function)
func (m *ArdupilotmegaMissionClearAll) CRCExtra() uint8 {
	return 232
}

// MsgName (generated function)
func (m *ArdupilotmegaMissionClearAll) MsgName() string {
	return "MissionClearAll"
}

// String (generated function)
func (m *ArdupilotmegaMissionClearAll) String() string {
	return fmt.Sprintf("ArdupilotmegaMissionClearAll{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaMissionClearAll) Pack(p *Packet) error {
	payload := make([]byte, 2)
	payload[0] = byte(m.TargetSystem)
	payload[1] = byte(m.TargetComponent)
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ArdupilotmegaMissionClearAll) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 2 {
		return errPayloadTooSmall
	}
	m.TargetSystem = uint8(payload[0])
	m.TargetComponent = uint8(payload[1])
	return nil
}

// ArdupilotmegaMissionItemReached struct (generated typeinfo)
// A certain mission item has been reached. The system will either hold this position (or circle on the orbit) or (if the autocontinue on the WP was set) continue to the next waypoint.
type ArdupilotmegaMissionItemReached struct {
	Seq uint16 // Sequence
}

// Dialect (generated function)
func (m *ArdupilotmegaMissionItemReached) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaMissionItemReached) MsgID() MessageID {
	return MSG_ID_MISSION_ITEM_REACHED
}

// CRCExtra (generated function)
func (m *ArdupilotmegaMissionItemReached) CRCExtra() uint8 {
	return 11
}

// MsgName (generated function)
func (m *ArdupilotmegaMissionItemReached) MsgName() string {
	return "MissionItemReached"
}

// String (generated function)
func (m *ArdupilotmegaMissionItemReached) String() string {
	return fmt.Sprintf("ArdupilotmegaMissionItemReached{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaMissionItemReached) Pack(p *Packet) error {
	payload := make([]byte, 2)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.Seq))
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ArdupilotmegaMissionItemReached) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 2 {
		return errPayloadTooSmall
	}
	m.Seq = uint16(binary.LittleEndian.Uint16(payload[0:]))
	return nil
}

// ArdupilotmegaMissionAck struct (generated typeinfo)
// Acknowledgment message during waypoint handling. The type field states if this message is a positive ack (type=0) or if an error happened (type=non-zero).
type ArdupilotmegaMissionAck struct {
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
	Type            uint8 // Mission result.
}

// Dialect (generated function)
func (m *ArdupilotmegaMissionAck) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaMissionAck) MsgID() MessageID {
	return MSG_ID_MISSION_ACK
}

// CRCExtra (generated function)
func (m *ArdupilotmegaMissionAck) CRCExtra() uint8 {
	return 153
}

// MsgName (generated function)
func (m *ArdupilotmegaMissionAck) MsgName() string {
	return "MissionAck"
}

// String (generated function)
func (m *ArdupilotmegaMissionAck) String() string {
	return fmt.Sprintf("ArdupilotmegaMissionAck{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaMissionAck) Pack(p *Packet) error {
	payload := make([]byte, 3)
	payload[0] = byte(m.TargetSystem)
	payload[1] = byte(m.TargetComponent)
	payload[2] = byte(m.Type)
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ArdupilotmegaMissionAck) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 3 {
		return errPayloadTooSmall
	}
	m.TargetSystem = uint8(payload[0])
	m.TargetComponent = uint8(payload[1])
	m.Type = uint8(payload[2])
	return nil
}

// ArdupilotmegaSetGpsGlobalOrigin struct (generated typeinfo)
// Sets the GPS co-ordinates of the vehicle local origin (0,0,0) position. Vehicle should emit GPS_GLOBAL_ORIGIN irrespective of whether the origin is changed. This enables transform between the local coordinate frame and the global (GPS) coordinate frame, which may be necessary when (for example) indoor and outdoor settings are connected and the MAV should move from in- to outdoor.
type ArdupilotmegaSetGpsGlobalOrigin struct {
	Latitude     int32 // Latitude (WGS84)
	Longitude    int32 // Longitude (WGS84)
	Altitude     int32 // Altitude (MSL). Positive for up.
	TargetSystem uint8 // System ID
}

// Dialect (generated function)
func (m *ArdupilotmegaSetGpsGlobalOrigin) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaSetGpsGlobalOrigin) MsgID() MessageID {
	return MSG_ID_SET_GPS_GLOBAL_ORIGIN
}

// CRCExtra (generated function)
func (m *ArdupilotmegaSetGpsGlobalOrigin) CRCExtra() uint8 {
	return 41
}

// MsgName (generated function)
func (m *ArdupilotmegaSetGpsGlobalOrigin) MsgName() string {
	return "SetGpsGlobalOrigin"
}

// String (generated function)
func (m *ArdupilotmegaSetGpsGlobalOrigin) String() string {
	return fmt.Sprintf("ArdupilotmegaSetGpsGlobalOrigin{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaSetGpsGlobalOrigin) Pack(p *Packet) error {
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
func (m *ArdupilotmegaSetGpsGlobalOrigin) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 13 {
		return errPayloadTooSmall
	}
	m.Latitude = int32(binary.LittleEndian.Uint32(payload[0:]))
	m.Longitude = int32(binary.LittleEndian.Uint32(payload[4:]))
	m.Altitude = int32(binary.LittleEndian.Uint32(payload[8:]))
	m.TargetSystem = uint8(payload[12])
	return nil
}

// ArdupilotmegaGpsGlobalOrigin struct (generated typeinfo)
// Publishes the GPS co-ordinates of the vehicle local origin (0,0,0) position. Emitted whenever a new GPS-Local position mapping is requested or set - e.g. following SET_GPS_GLOBAL_ORIGIN message.
type ArdupilotmegaGpsGlobalOrigin struct {
	Latitude  int32 // Latitude (WGS84)
	Longitude int32 // Longitude (WGS84)
	Altitude  int32 // Altitude (MSL). Positive for up.
}

// Dialect (generated function)
func (m *ArdupilotmegaGpsGlobalOrigin) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaGpsGlobalOrigin) MsgID() MessageID {
	return MSG_ID_GPS_GLOBAL_ORIGIN
}

// CRCExtra (generated function)
func (m *ArdupilotmegaGpsGlobalOrigin) CRCExtra() uint8 {
	return 39
}

// MsgName (generated function)
func (m *ArdupilotmegaGpsGlobalOrigin) MsgName() string {
	return "GpsGlobalOrigin"
}

// String (generated function)
func (m *ArdupilotmegaGpsGlobalOrigin) String() string {
	return fmt.Sprintf("ArdupilotmegaGpsGlobalOrigin{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaGpsGlobalOrigin) Pack(p *Packet) error {
	payload := make([]byte, 12)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.Latitude))
	binary.LittleEndian.PutUint32(payload[4:], uint32(m.Longitude))
	binary.LittleEndian.PutUint32(payload[8:], uint32(m.Altitude))
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ArdupilotmegaGpsGlobalOrigin) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 12 {
		return errPayloadTooSmall
	}
	m.Latitude = int32(binary.LittleEndian.Uint32(payload[0:]))
	m.Longitude = int32(binary.LittleEndian.Uint32(payload[4:]))
	m.Altitude = int32(binary.LittleEndian.Uint32(payload[8:]))
	return nil
}

// ArdupilotmegaParamMapRc struct (generated typeinfo)
// Bind a RC channel to a parameter. The parameter should change according to the RC channel value.
type ArdupilotmegaParamMapRc struct {
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

// Dialect (generated function)
func (m *ArdupilotmegaParamMapRc) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaParamMapRc) MsgID() MessageID {
	return MSG_ID_PARAM_MAP_RC
}

// CRCExtra (generated function)
func (m *ArdupilotmegaParamMapRc) CRCExtra() uint8 {
	return 78
}

// MsgName (generated function)
func (m *ArdupilotmegaParamMapRc) MsgName() string {
	return "ParamMapRc"
}

// String (generated function)
func (m *ArdupilotmegaParamMapRc) String() string {
	return fmt.Sprintf("ArdupilotmegaParamMapRc{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaParamMapRc) Pack(p *Packet) error {
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
func (m *ArdupilotmegaParamMapRc) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 37 {
		return errPayloadTooSmall
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

// ArdupilotmegaMissionRequestInt struct (generated typeinfo)
// Request the information of the mission item with the sequence number seq. The response of the system to this message should be a MISSION_ITEM_INT message. https://mavlink.io/en/services/mission.html
type ArdupilotmegaMissionRequestInt struct {
	Seq             uint16 // Sequence
	TargetSystem    uint8  // System ID
	TargetComponent uint8  // Component ID
}

// Dialect (generated function)
func (m *ArdupilotmegaMissionRequestInt) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaMissionRequestInt) MsgID() MessageID {
	return MSG_ID_MISSION_REQUEST_INT
}

// CRCExtra (generated function)
func (m *ArdupilotmegaMissionRequestInt) CRCExtra() uint8 {
	return 196
}

// MsgName (generated function)
func (m *ArdupilotmegaMissionRequestInt) MsgName() string {
	return "MissionRequestInt"
}

// String (generated function)
func (m *ArdupilotmegaMissionRequestInt) String() string {
	return fmt.Sprintf("ArdupilotmegaMissionRequestInt{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaMissionRequestInt) Pack(p *Packet) error {
	payload := make([]byte, 4)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.Seq))
	payload[2] = byte(m.TargetSystem)
	payload[3] = byte(m.TargetComponent)
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ArdupilotmegaMissionRequestInt) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 4 {
		return errPayloadTooSmall
	}
	m.Seq = uint16(binary.LittleEndian.Uint16(payload[0:]))
	m.TargetSystem = uint8(payload[2])
	m.TargetComponent = uint8(payload[3])
	return nil
}

// ArdupilotmegaMissionChanged struct (generated typeinfo)
// A broadcast message to notify any ground station or SDK if a mission, geofence or safe points have changed on the vehicle.
type ArdupilotmegaMissionChanged struct {
	StartIndex   int16 // Start index for partial mission change (-1 for all items).
	EndIndex     int16 // End index of a partial mission change. -1 is a synonym for the last mission item (i.e. selects all items from start_index). Ignore field if start_index=-1.
	OriginSysid  uint8 // System ID of the author of the new mission.
	OriginCompid uint8 // Compnent ID of the author of the new mission.
	MissionType  uint8 // Mission type.
}

// Dialect (generated function)
func (m *ArdupilotmegaMissionChanged) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaMissionChanged) MsgID() MessageID {
	return MSG_ID_MISSION_CHANGED
}

// CRCExtra (generated function)
func (m *ArdupilotmegaMissionChanged) CRCExtra() uint8 {
	return 132
}

// MsgName (generated function)
func (m *ArdupilotmegaMissionChanged) MsgName() string {
	return "MissionChanged"
}

// String (generated function)
func (m *ArdupilotmegaMissionChanged) String() string {
	return fmt.Sprintf("ArdupilotmegaMissionChanged{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaMissionChanged) Pack(p *Packet) error {
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
func (m *ArdupilotmegaMissionChanged) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 7 {
		return errPayloadTooSmall
	}
	m.StartIndex = int16(binary.LittleEndian.Uint16(payload[0:]))
	m.EndIndex = int16(binary.LittleEndian.Uint16(payload[2:]))
	m.OriginSysid = uint8(payload[4])
	m.OriginCompid = uint8(payload[5])
	m.MissionType = uint8(payload[6])
	return nil
}

// ArdupilotmegaSafetySetAllowedArea struct (generated typeinfo)
// Set a safety zone (volume), which is defined by two corners of a cube. This message can be used to tell the MAV which setpoints/waypoints to accept and which to reject. Safety areas are often enforced by national or competition regulations.
type ArdupilotmegaSafetySetAllowedArea struct {
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

// Dialect (generated function)
func (m *ArdupilotmegaSafetySetAllowedArea) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaSafetySetAllowedArea) MsgID() MessageID {
	return MSG_ID_SAFETY_SET_ALLOWED_AREA
}

// CRCExtra (generated function)
func (m *ArdupilotmegaSafetySetAllowedArea) CRCExtra() uint8 {
	return 15
}

// MsgName (generated function)
func (m *ArdupilotmegaSafetySetAllowedArea) MsgName() string {
	return "SafetySetAllowedArea"
}

// String (generated function)
func (m *ArdupilotmegaSafetySetAllowedArea) String() string {
	return fmt.Sprintf("ArdupilotmegaSafetySetAllowedArea{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaSafetySetAllowedArea) Pack(p *Packet) error {
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
func (m *ArdupilotmegaSafetySetAllowedArea) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 27 {
		return errPayloadTooSmall
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

// ArdupilotmegaSafetyAllowedArea struct (generated typeinfo)
// Read out the safety zone the MAV currently assumes.
type ArdupilotmegaSafetyAllowedArea struct {
	P1x   float32 // x position 1 / Latitude 1
	P1y   float32 // y position 1 / Longitude 1
	P1z   float32 // z position 1 / Altitude 1
	P2x   float32 // x position 2 / Latitude 2
	P2y   float32 // y position 2 / Longitude 2
	P2z   float32 // z position 2 / Altitude 2
	Frame uint8   // Coordinate frame. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down.
}

// Dialect (generated function)
func (m *ArdupilotmegaSafetyAllowedArea) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaSafetyAllowedArea) MsgID() MessageID {
	return MSG_ID_SAFETY_ALLOWED_AREA
}

// CRCExtra (generated function)
func (m *ArdupilotmegaSafetyAllowedArea) CRCExtra() uint8 {
	return 3
}

// MsgName (generated function)
func (m *ArdupilotmegaSafetyAllowedArea) MsgName() string {
	return "SafetyAllowedArea"
}

// String (generated function)
func (m *ArdupilotmegaSafetyAllowedArea) String() string {
	return fmt.Sprintf("ArdupilotmegaSafetyAllowedArea{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaSafetyAllowedArea) Pack(p *Packet) error {
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
func (m *ArdupilotmegaSafetyAllowedArea) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 25 {
		return errPayloadTooSmall
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

// ArdupilotmegaAttitudeQuaternionCov struct (generated typeinfo)
// The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right), expressed as quaternion. Quaternion order is w, x, y, z and a zero rotation would be expressed as (1 0 0 0).
type ArdupilotmegaAttitudeQuaternionCov struct {
	TimeUsec   uint64     // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
	Q          [4]float32 // Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation)
	Rollspeed  float32    // Roll angular speed
	Pitchspeed float32    // Pitch angular speed
	Yawspeed   float32    // Yaw angular speed
	Covariance [9]float32 // Row-major representation of a 3x3 attitude covariance matrix (states: roll, pitch, yaw; first three entries are the first ROW, next three entries are the second row, etc.). If unknown, assign NaN value to first element in the array.
}

// Dialect (generated function)
func (m *ArdupilotmegaAttitudeQuaternionCov) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaAttitudeQuaternionCov) MsgID() MessageID {
	return MSG_ID_ATTITUDE_QUATERNION_COV
}

// CRCExtra (generated function)
func (m *ArdupilotmegaAttitudeQuaternionCov) CRCExtra() uint8 {
	return 167
}

// MsgName (generated function)
func (m *ArdupilotmegaAttitudeQuaternionCov) MsgName() string {
	return "AttitudeQuaternionCov"
}

// String (generated function)
func (m *ArdupilotmegaAttitudeQuaternionCov) String() string {
	return fmt.Sprintf("ArdupilotmegaAttitudeQuaternionCov{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaAttitudeQuaternionCov) Pack(p *Packet) error {
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
func (m *ArdupilotmegaAttitudeQuaternionCov) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 72 {
		return errPayloadTooSmall
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

// ArdupilotmegaNavControllerOutput struct (generated typeinfo)
// The state of the fixed wing navigation and position controller.
type ArdupilotmegaNavControllerOutput struct {
	NavRoll       float32 // Current desired roll
	NavPitch      float32 // Current desired pitch
	AltError      float32 // Current altitude error
	AspdError     float32 // Current airspeed error
	XtrackError   float32 // Current crosstrack error on x-y plane
	NavBearing    int16   // Current desired heading
	TargetBearing int16   // Bearing to current waypoint/target
	WpDist        uint16  // Distance to active waypoint
}

// Dialect (generated function)
func (m *ArdupilotmegaNavControllerOutput) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaNavControllerOutput) MsgID() MessageID {
	return MSG_ID_NAV_CONTROLLER_OUTPUT
}

// CRCExtra (generated function)
func (m *ArdupilotmegaNavControllerOutput) CRCExtra() uint8 {
	return 183
}

// MsgName (generated function)
func (m *ArdupilotmegaNavControllerOutput) MsgName() string {
	return "NavControllerOutput"
}

// String (generated function)
func (m *ArdupilotmegaNavControllerOutput) String() string {
	return fmt.Sprintf("ArdupilotmegaNavControllerOutput{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaNavControllerOutput) Pack(p *Packet) error {
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
func (m *ArdupilotmegaNavControllerOutput) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 26 {
		return errPayloadTooSmall
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

// ArdupilotmegaGlobalPositionIntCov struct (generated typeinfo)
// The filtered global position (e.g. fused GPS and accelerometers). The position is in GPS-frame (right-handed, Z-up). It  is designed as scaled integer message since the resolution of float is not sufficient. NOTE: This message is intended for onboard networks / companion computers and higher-bandwidth links and optimized for accuracy and completeness. Please use the GLOBAL_POSITION_INT message for a minimal subset.
type ArdupilotmegaGlobalPositionIntCov struct {
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

// Dialect (generated function)
func (m *ArdupilotmegaGlobalPositionIntCov) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaGlobalPositionIntCov) MsgID() MessageID {
	return MSG_ID_GLOBAL_POSITION_INT_COV
}

// CRCExtra (generated function)
func (m *ArdupilotmegaGlobalPositionIntCov) CRCExtra() uint8 {
	return 119
}

// MsgName (generated function)
func (m *ArdupilotmegaGlobalPositionIntCov) MsgName() string {
	return "GlobalPositionIntCov"
}

// String (generated function)
func (m *ArdupilotmegaGlobalPositionIntCov) String() string {
	return fmt.Sprintf("ArdupilotmegaGlobalPositionIntCov{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaGlobalPositionIntCov) Pack(p *Packet) error {
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
func (m *ArdupilotmegaGlobalPositionIntCov) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 181 {
		return errPayloadTooSmall
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

// ArdupilotmegaLocalPositionNedCov struct (generated typeinfo)
// The filtered local position (e.g. fused computer vision and accelerometers). Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down convention)
type ArdupilotmegaLocalPositionNedCov struct {
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

// Dialect (generated function)
func (m *ArdupilotmegaLocalPositionNedCov) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaLocalPositionNedCov) MsgID() MessageID {
	return MSG_ID_LOCAL_POSITION_NED_COV
}

// CRCExtra (generated function)
func (m *ArdupilotmegaLocalPositionNedCov) CRCExtra() uint8 {
	return 191
}

// MsgName (generated function)
func (m *ArdupilotmegaLocalPositionNedCov) MsgName() string {
	return "LocalPositionNedCov"
}

// String (generated function)
func (m *ArdupilotmegaLocalPositionNedCov) String() string {
	return fmt.Sprintf("ArdupilotmegaLocalPositionNedCov{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaLocalPositionNedCov) Pack(p *Packet) error {
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
func (m *ArdupilotmegaLocalPositionNedCov) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 225 {
		return errPayloadTooSmall
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

// ArdupilotmegaRcChannels struct (generated typeinfo)
// The PPM values of the RC channels received. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%.  A value of UINT16_MAX implies the channel is unused. Individual receivers/transmitters might violate this specification.
type ArdupilotmegaRcChannels struct {
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

// Dialect (generated function)
func (m *ArdupilotmegaRcChannels) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaRcChannels) MsgID() MessageID {
	return MSG_ID_RC_CHANNELS
}

// CRCExtra (generated function)
func (m *ArdupilotmegaRcChannels) CRCExtra() uint8 {
	return 118
}

// MsgName (generated function)
func (m *ArdupilotmegaRcChannels) MsgName() string {
	return "RcChannels"
}

// String (generated function)
func (m *ArdupilotmegaRcChannels) String() string {
	return fmt.Sprintf("ArdupilotmegaRcChannels{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaRcChannels) Pack(p *Packet) error {
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
func (m *ArdupilotmegaRcChannels) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 42 {
		return errPayloadTooSmall
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

// ArdupilotmegaRequestDataStream struct (generated typeinfo)
// Request a data stream.
type ArdupilotmegaRequestDataStream struct {
	ReqMessageRate  uint16 // The requested message rate
	TargetSystem    uint8  // The target requested to send the message stream.
	TargetComponent uint8  // The target requested to send the message stream.
	ReqStreamID     uint8  // The ID of the requested data stream
	StartStop       uint8  // 1 to start sending, 0 to stop sending.
}

// Dialect (generated function)
func (m *ArdupilotmegaRequestDataStream) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaRequestDataStream) MsgID() MessageID {
	return MSG_ID_REQUEST_DATA_STREAM
}

// CRCExtra (generated function)
func (m *ArdupilotmegaRequestDataStream) CRCExtra() uint8 {
	return 148
}

// MsgName (generated function)
func (m *ArdupilotmegaRequestDataStream) MsgName() string {
	return "RequestDataStream"
}

// String (generated function)
func (m *ArdupilotmegaRequestDataStream) String() string {
	return fmt.Sprintf("ArdupilotmegaRequestDataStream{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaRequestDataStream) Pack(p *Packet) error {
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
func (m *ArdupilotmegaRequestDataStream) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 6 {
		return errPayloadTooSmall
	}
	m.ReqMessageRate = uint16(binary.LittleEndian.Uint16(payload[0:]))
	m.TargetSystem = uint8(payload[2])
	m.TargetComponent = uint8(payload[3])
	m.ReqStreamID = uint8(payload[4])
	m.StartStop = uint8(payload[5])
	return nil
}

// ArdupilotmegaDataStream struct (generated typeinfo)
// Data stream status information.
type ArdupilotmegaDataStream struct {
	MessageRate uint16 // The message rate
	StreamID    uint8  // The ID of the requested data stream
	OnOff       uint8  // 1 stream is enabled, 0 stream is stopped.
}

// Dialect (generated function)
func (m *ArdupilotmegaDataStream) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaDataStream) MsgID() MessageID {
	return MSG_ID_DATA_STREAM
}

// CRCExtra (generated function)
func (m *ArdupilotmegaDataStream) CRCExtra() uint8 {
	return 21
}

// MsgName (generated function)
func (m *ArdupilotmegaDataStream) MsgName() string {
	return "DataStream"
}

// String (generated function)
func (m *ArdupilotmegaDataStream) String() string {
	return fmt.Sprintf("ArdupilotmegaDataStream{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaDataStream) Pack(p *Packet) error {
	payload := make([]byte, 4)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.MessageRate))
	payload[2] = byte(m.StreamID)
	payload[3] = byte(m.OnOff)
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ArdupilotmegaDataStream) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 4 {
		return errPayloadTooSmall
	}
	m.MessageRate = uint16(binary.LittleEndian.Uint16(payload[0:]))
	m.StreamID = uint8(payload[2])
	m.OnOff = uint8(payload[3])
	return nil
}

// ArdupilotmegaManualControl struct (generated typeinfo)
// This message provides an API for manually controlling the vehicle using standard joystick axes nomenclature, along with a joystick-like input device. Unused axes can be disabled an buttons are also transmit as boolean values of their
type ArdupilotmegaManualControl struct {
	X       int16  // X-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to forward(1000)-backward(-1000) movement on a joystick and the pitch of a vehicle.
	Y       int16  // Y-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to left(-1000)-right(1000) movement on a joystick and the roll of a vehicle.
	Z       int16  // Z-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to a separate slider movement with maximum being 1000 and minimum being -1000 on a joystick and the thrust of a vehicle. Positive values are positive thrust, negative values are negative thrust.
	R       int16  // R-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to a twisting of the joystick, with counter-clockwise being 1000 and clockwise being -1000, and the yaw of a vehicle.
	Buttons uint16 // A bitfield corresponding to the joystick buttons' current state, 1 for pressed, 0 for released. The lowest bit corresponds to Button 1.
	Target  uint8  // The system to be controlled.
}

// Dialect (generated function)
func (m *ArdupilotmegaManualControl) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaManualControl) MsgID() MessageID {
	return MSG_ID_MANUAL_CONTROL
}

// CRCExtra (generated function)
func (m *ArdupilotmegaManualControl) CRCExtra() uint8 {
	return 243
}

// MsgName (generated function)
func (m *ArdupilotmegaManualControl) MsgName() string {
	return "ManualControl"
}

// String (generated function)
func (m *ArdupilotmegaManualControl) String() string {
	return fmt.Sprintf("ArdupilotmegaManualControl{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaManualControl) Pack(p *Packet) error {
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
func (m *ArdupilotmegaManualControl) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 11 {
		return errPayloadTooSmall
	}
	m.X = int16(binary.LittleEndian.Uint16(payload[0:]))
	m.Y = int16(binary.LittleEndian.Uint16(payload[2:]))
	m.Z = int16(binary.LittleEndian.Uint16(payload[4:]))
	m.R = int16(binary.LittleEndian.Uint16(payload[6:]))
	m.Buttons = uint16(binary.LittleEndian.Uint16(payload[8:]))
	m.Target = uint8(payload[10])
	return nil
}

// ArdupilotmegaRcChannelsOverride struct (generated typeinfo)
// The RAW values of the RC channels sent to the MAV to override info received from the RC radio. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual receivers/transmitters might violate this specification.  Note carefully the semantic differences between the first 8 channels and the subsequent channels
type ArdupilotmegaRcChannelsOverride struct {
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

// Dialect (generated function)
func (m *ArdupilotmegaRcChannelsOverride) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaRcChannelsOverride) MsgID() MessageID {
	return MSG_ID_RC_CHANNELS_OVERRIDE
}

// CRCExtra (generated function)
func (m *ArdupilotmegaRcChannelsOverride) CRCExtra() uint8 {
	return 124
}

// MsgName (generated function)
func (m *ArdupilotmegaRcChannelsOverride) MsgName() string {
	return "RcChannelsOverride"
}

// String (generated function)
func (m *ArdupilotmegaRcChannelsOverride) String() string {
	return fmt.Sprintf("ArdupilotmegaRcChannelsOverride{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaRcChannelsOverride) Pack(p *Packet) error {
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
func (m *ArdupilotmegaRcChannelsOverride) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 18 {
		return errPayloadTooSmall
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

// ArdupilotmegaMissionItemInt struct (generated typeinfo)
// Message encoding a mission item. This message is emitted to announce
//                 the presence of a mission item and to set a mission item on the system. The mission item can be either in x, y, z meters (type: LOCAL) or x:lat, y:lon, z:altitude. Local frame is Z-down, right handed (NED), global frame is Z-up, right handed (ENU). NaN or INT32_MAX may be used in float/integer params (respectively) to indicate optional/default values (e.g. to use the component's current latitude, yaw rather than a specific value). See also https://mavlink.io/en/services/mission.html.
type ArdupilotmegaMissionItemInt struct {
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

// Dialect (generated function)
func (m *ArdupilotmegaMissionItemInt) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaMissionItemInt) MsgID() MessageID {
	return MSG_ID_MISSION_ITEM_INT
}

// CRCExtra (generated function)
func (m *ArdupilotmegaMissionItemInt) CRCExtra() uint8 {
	return 38
}

// MsgName (generated function)
func (m *ArdupilotmegaMissionItemInt) MsgName() string {
	return "MissionItemInt"
}

// String (generated function)
func (m *ArdupilotmegaMissionItemInt) String() string {
	return fmt.Sprintf("ArdupilotmegaMissionItemInt{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaMissionItemInt) Pack(p *Packet) error {
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
func (m *ArdupilotmegaMissionItemInt) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 37 {
		return errPayloadTooSmall
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

// ArdupilotmegaVfrHud struct (generated typeinfo)
// Metrics typically displayed on a HUD for fixed wing aircraft.
type ArdupilotmegaVfrHud struct {
	Airspeed    float32 // Vehicle speed in form appropriate for vehicle type. For standard aircraft this is typically calibrated airspeed (CAS) or indicated airspeed (IAS) - either of which can be used by a pilot to estimate stall speed.
	Groundspeed float32 // Current ground speed.
	Alt         float32 // Current altitude (MSL).
	Climb       float32 // Current climb rate.
	Heading     int16   // Current heading in compass units (0-360, 0=north).
	Throttle    uint16  // Current throttle setting (0 to 100).
}

// Dialect (generated function)
func (m *ArdupilotmegaVfrHud) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaVfrHud) MsgID() MessageID {
	return MSG_ID_VFR_HUD
}

// CRCExtra (generated function)
func (m *ArdupilotmegaVfrHud) CRCExtra() uint8 {
	return 20
}

// MsgName (generated function)
func (m *ArdupilotmegaVfrHud) MsgName() string {
	return "VfrHud"
}

// String (generated function)
func (m *ArdupilotmegaVfrHud) String() string {
	return fmt.Sprintf("ArdupilotmegaVfrHud{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaVfrHud) Pack(p *Packet) error {
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
func (m *ArdupilotmegaVfrHud) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 20 {
		return errPayloadTooSmall
	}
	m.Airspeed = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.Groundspeed = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	m.Alt = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.Climb = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.Heading = int16(binary.LittleEndian.Uint16(payload[16:]))
	m.Throttle = uint16(binary.LittleEndian.Uint16(payload[18:]))
	return nil
}

// ArdupilotmegaCommandInt struct (generated typeinfo)
// Message encoding a command with parameters as scaled integers. Scaling depends on the actual command value. The command microservice is documented at https://mavlink.io/en/services/command.html
type ArdupilotmegaCommandInt struct {
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

// Dialect (generated function)
func (m *ArdupilotmegaCommandInt) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaCommandInt) MsgID() MessageID {
	return MSG_ID_COMMAND_INT
}

// CRCExtra (generated function)
func (m *ArdupilotmegaCommandInt) CRCExtra() uint8 {
	return 158
}

// MsgName (generated function)
func (m *ArdupilotmegaCommandInt) MsgName() string {
	return "CommandInt"
}

// String (generated function)
func (m *ArdupilotmegaCommandInt) String() string {
	return fmt.Sprintf("ArdupilotmegaCommandInt{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaCommandInt) Pack(p *Packet) error {
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
func (m *ArdupilotmegaCommandInt) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 35 {
		return errPayloadTooSmall
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

// ArdupilotmegaCommandLong struct (generated typeinfo)
// Send a command with up to seven parameters to the MAV. The command microservice is documented at https://mavlink.io/en/services/command.html
type ArdupilotmegaCommandLong struct {
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

// Dialect (generated function)
func (m *ArdupilotmegaCommandLong) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaCommandLong) MsgID() MessageID {
	return MSG_ID_COMMAND_LONG
}

// CRCExtra (generated function)
func (m *ArdupilotmegaCommandLong) CRCExtra() uint8 {
	return 152
}

// MsgName (generated function)
func (m *ArdupilotmegaCommandLong) MsgName() string {
	return "CommandLong"
}

// String (generated function)
func (m *ArdupilotmegaCommandLong) String() string {
	return fmt.Sprintf("ArdupilotmegaCommandLong{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaCommandLong) Pack(p *Packet) error {
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
func (m *ArdupilotmegaCommandLong) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 33 {
		return errPayloadTooSmall
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

// ArdupilotmegaCommandAck struct (generated typeinfo)
// Report status of a command. Includes feedback whether the command was executed. The command microservice is documented at https://mavlink.io/en/services/command.html
type ArdupilotmegaCommandAck struct {
	Command uint16 // Command ID (of acknowledged command).
	Result  uint8  // Result of command.
}

// Dialect (generated function)
func (m *ArdupilotmegaCommandAck) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaCommandAck) MsgID() MessageID {
	return MSG_ID_COMMAND_ACK
}

// CRCExtra (generated function)
func (m *ArdupilotmegaCommandAck) CRCExtra() uint8 {
	return 143
}

// MsgName (generated function)
func (m *ArdupilotmegaCommandAck) MsgName() string {
	return "CommandAck"
}

// String (generated function)
func (m *ArdupilotmegaCommandAck) String() string {
	return fmt.Sprintf("ArdupilotmegaCommandAck{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaCommandAck) Pack(p *Packet) error {
	payload := make([]byte, 3)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.Command))
	payload[2] = byte(m.Result)
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ArdupilotmegaCommandAck) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 3 {
		return errPayloadTooSmall
	}
	m.Command = uint16(binary.LittleEndian.Uint16(payload[0:]))
	m.Result = uint8(payload[2])
	return nil
}

// ArdupilotmegaCommandCancel struct (generated typeinfo)
// Cancel a long running command. The target system should respond with a COMMAND_ACK to the original command with result=MAV_RESULT_CANCELLED if the long running process was cancelled. If it has already completed, the cancel action can be ignored. The cancel action can be retried until some sort of acknowledgement to the original command has been received. The command microservice is documented at https://mavlink.io/en/services/command.html
type ArdupilotmegaCommandCancel struct {
	Command         uint16 // Command ID (of command to cancel).
	TargetSystem    uint8  // System executing long running command. Should not be broadcast (0).
	TargetComponent uint8  // Component executing long running command.
}

// Dialect (generated function)
func (m *ArdupilotmegaCommandCancel) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaCommandCancel) MsgID() MessageID {
	return MSG_ID_COMMAND_CANCEL
}

// CRCExtra (generated function)
func (m *ArdupilotmegaCommandCancel) CRCExtra() uint8 {
	return 14
}

// MsgName (generated function)
func (m *ArdupilotmegaCommandCancel) MsgName() string {
	return "CommandCancel"
}

// String (generated function)
func (m *ArdupilotmegaCommandCancel) String() string {
	return fmt.Sprintf("ArdupilotmegaCommandCancel{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaCommandCancel) Pack(p *Packet) error {
	payload := make([]byte, 4)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.Command))
	payload[2] = byte(m.TargetSystem)
	payload[3] = byte(m.TargetComponent)
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ArdupilotmegaCommandCancel) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 4 {
		return errPayloadTooSmall
	}
	m.Command = uint16(binary.LittleEndian.Uint16(payload[0:]))
	m.TargetSystem = uint8(payload[2])
	m.TargetComponent = uint8(payload[3])
	return nil
}

// ArdupilotmegaManualSetpoint struct (generated typeinfo)
// Setpoint in roll, pitch, yaw and thrust from the operator
type ArdupilotmegaManualSetpoint struct {
	TimeBootMs           uint32  // Timestamp (time since system boot).
	Roll                 float32 // Desired roll rate
	Pitch                float32 // Desired pitch rate
	Yaw                  float32 // Desired yaw rate
	Thrust               float32 // Collective thrust, normalized to 0 .. 1
	ModeSwitch           uint8   // Flight mode switch position, 0.. 255
	ManualOverrideSwitch uint8   // Override mode switch position, 0.. 255
}

// Dialect (generated function)
func (m *ArdupilotmegaManualSetpoint) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaManualSetpoint) MsgID() MessageID {
	return MSG_ID_MANUAL_SETPOINT
}

// CRCExtra (generated function)
func (m *ArdupilotmegaManualSetpoint) CRCExtra() uint8 {
	return 106
}

// MsgName (generated function)
func (m *ArdupilotmegaManualSetpoint) MsgName() string {
	return "ManualSetpoint"
}

// String (generated function)
func (m *ArdupilotmegaManualSetpoint) String() string {
	return fmt.Sprintf("ArdupilotmegaManualSetpoint{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaManualSetpoint) Pack(p *Packet) error {
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
func (m *ArdupilotmegaManualSetpoint) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 22 {
		return errPayloadTooSmall
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

// ArdupilotmegaSetAttitudeTarget struct (generated typeinfo)
// Sets a desired vehicle attitude. Used by an external controller to command the vehicle (manual controller or other system).
type ArdupilotmegaSetAttitudeTarget struct {
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

// Dialect (generated function)
func (m *ArdupilotmegaSetAttitudeTarget) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaSetAttitudeTarget) MsgID() MessageID {
	return MSG_ID_SET_ATTITUDE_TARGET
}

// CRCExtra (generated function)
func (m *ArdupilotmegaSetAttitudeTarget) CRCExtra() uint8 {
	return 49
}

// MsgName (generated function)
func (m *ArdupilotmegaSetAttitudeTarget) MsgName() string {
	return "SetAttitudeTarget"
}

// String (generated function)
func (m *ArdupilotmegaSetAttitudeTarget) String() string {
	return fmt.Sprintf("ArdupilotmegaSetAttitudeTarget{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaSetAttitudeTarget) Pack(p *Packet) error {
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
func (m *ArdupilotmegaSetAttitudeTarget) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 39 {
		return errPayloadTooSmall
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

// ArdupilotmegaAttitudeTarget struct (generated typeinfo)
// Reports the current commanded attitude of the vehicle as specified by the autopilot. This should match the commands sent in a SET_ATTITUDE_TARGET message if the vehicle is being controlled this way.
type ArdupilotmegaAttitudeTarget struct {
	TimeBootMs    uint32     // Timestamp (time since system boot).
	Q             [4]float32 // Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
	BodyRollRate  float32    // Body roll rate
	BodyPitchRate float32    // Body pitch rate
	BodyYawRate   float32    // Body yaw rate
	Thrust        float32    // Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)
	TypeMask      uint8      // Bitmap to indicate which dimensions should be ignored by the vehicle.
}

// Dialect (generated function)
func (m *ArdupilotmegaAttitudeTarget) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaAttitudeTarget) MsgID() MessageID {
	return MSG_ID_ATTITUDE_TARGET
}

// CRCExtra (generated function)
func (m *ArdupilotmegaAttitudeTarget) CRCExtra() uint8 {
	return 22
}

// MsgName (generated function)
func (m *ArdupilotmegaAttitudeTarget) MsgName() string {
	return "AttitudeTarget"
}

// String (generated function)
func (m *ArdupilotmegaAttitudeTarget) String() string {
	return fmt.Sprintf("ArdupilotmegaAttitudeTarget{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaAttitudeTarget) Pack(p *Packet) error {
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
func (m *ArdupilotmegaAttitudeTarget) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 37 {
		return errPayloadTooSmall
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

// ArdupilotmegaSetPositionTargetLocalNed struct (generated typeinfo)
// Sets a desired vehicle position in a local north-east-down coordinate frame. Used by an external controller to command the vehicle (manual controller or other system).
type ArdupilotmegaSetPositionTargetLocalNed struct {
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

// Dialect (generated function)
func (m *ArdupilotmegaSetPositionTargetLocalNed) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaSetPositionTargetLocalNed) MsgID() MessageID {
	return MSG_ID_SET_POSITION_TARGET_LOCAL_NED
}

// CRCExtra (generated function)
func (m *ArdupilotmegaSetPositionTargetLocalNed) CRCExtra() uint8 {
	return 143
}

// MsgName (generated function)
func (m *ArdupilotmegaSetPositionTargetLocalNed) MsgName() string {
	return "SetPositionTargetLocalNed"
}

// String (generated function)
func (m *ArdupilotmegaSetPositionTargetLocalNed) String() string {
	return fmt.Sprintf("ArdupilotmegaSetPositionTargetLocalNed{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaSetPositionTargetLocalNed) Pack(p *Packet) error {
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
func (m *ArdupilotmegaSetPositionTargetLocalNed) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 53 {
		return errPayloadTooSmall
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

// ArdupilotmegaPositionTargetLocalNed struct (generated typeinfo)
// Reports the current commanded vehicle position, velocity, and acceleration as specified by the autopilot. This should match the commands sent in SET_POSITION_TARGET_LOCAL_NED if the vehicle is being controlled this way.
type ArdupilotmegaPositionTargetLocalNed struct {
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

// Dialect (generated function)
func (m *ArdupilotmegaPositionTargetLocalNed) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaPositionTargetLocalNed) MsgID() MessageID {
	return MSG_ID_POSITION_TARGET_LOCAL_NED
}

// CRCExtra (generated function)
func (m *ArdupilotmegaPositionTargetLocalNed) CRCExtra() uint8 {
	return 140
}

// MsgName (generated function)
func (m *ArdupilotmegaPositionTargetLocalNed) MsgName() string {
	return "PositionTargetLocalNed"
}

// String (generated function)
func (m *ArdupilotmegaPositionTargetLocalNed) String() string {
	return fmt.Sprintf("ArdupilotmegaPositionTargetLocalNed{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaPositionTargetLocalNed) Pack(p *Packet) error {
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
func (m *ArdupilotmegaPositionTargetLocalNed) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 51 {
		return errPayloadTooSmall
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

// ArdupilotmegaSetPositionTargetGlobalInt struct (generated typeinfo)
// Sets a desired vehicle position, velocity, and/or acceleration in a global coordinate system (WGS84). Used by an external controller to command the vehicle (manual controller or other system).
type ArdupilotmegaSetPositionTargetGlobalInt struct {
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

// Dialect (generated function)
func (m *ArdupilotmegaSetPositionTargetGlobalInt) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaSetPositionTargetGlobalInt) MsgID() MessageID {
	return MSG_ID_SET_POSITION_TARGET_GLOBAL_INT
}

// CRCExtra (generated function)
func (m *ArdupilotmegaSetPositionTargetGlobalInt) CRCExtra() uint8 {
	return 5
}

// MsgName (generated function)
func (m *ArdupilotmegaSetPositionTargetGlobalInt) MsgName() string {
	return "SetPositionTargetGlobalInt"
}

// String (generated function)
func (m *ArdupilotmegaSetPositionTargetGlobalInt) String() string {
	return fmt.Sprintf("ArdupilotmegaSetPositionTargetGlobalInt{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaSetPositionTargetGlobalInt) Pack(p *Packet) error {
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
func (m *ArdupilotmegaSetPositionTargetGlobalInt) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 53 {
		return errPayloadTooSmall
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

// ArdupilotmegaPositionTargetGlobalInt struct (generated typeinfo)
// Reports the current commanded vehicle position, velocity, and acceleration as specified by the autopilot. This should match the commands sent in SET_POSITION_TARGET_GLOBAL_INT if the vehicle is being controlled this way.
type ArdupilotmegaPositionTargetGlobalInt struct {
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

// Dialect (generated function)
func (m *ArdupilotmegaPositionTargetGlobalInt) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaPositionTargetGlobalInt) MsgID() MessageID {
	return MSG_ID_POSITION_TARGET_GLOBAL_INT
}

// CRCExtra (generated function)
func (m *ArdupilotmegaPositionTargetGlobalInt) CRCExtra() uint8 {
	return 150
}

// MsgName (generated function)
func (m *ArdupilotmegaPositionTargetGlobalInt) MsgName() string {
	return "PositionTargetGlobalInt"
}

// String (generated function)
func (m *ArdupilotmegaPositionTargetGlobalInt) String() string {
	return fmt.Sprintf("ArdupilotmegaPositionTargetGlobalInt{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaPositionTargetGlobalInt) Pack(p *Packet) error {
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
func (m *ArdupilotmegaPositionTargetGlobalInt) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 51 {
		return errPayloadTooSmall
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

// ArdupilotmegaLocalPositionNedSystemGlobalOffset struct (generated typeinfo)
// The offset in X, Y, Z and yaw between the LOCAL_POSITION_NED messages of MAV X and the global coordinate frame in NED coordinates. Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down convention)
type ArdupilotmegaLocalPositionNedSystemGlobalOffset struct {
	TimeBootMs uint32  // Timestamp (time since system boot).
	X          float32 // X Position
	Y          float32 // Y Position
	Z          float32 // Z Position
	Roll       float32 // Roll
	Pitch      float32 // Pitch
	Yaw        float32 // Yaw
}

// Dialect (generated function)
func (m *ArdupilotmegaLocalPositionNedSystemGlobalOffset) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaLocalPositionNedSystemGlobalOffset) MsgID() MessageID {
	return MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET
}

// CRCExtra (generated function)
func (m *ArdupilotmegaLocalPositionNedSystemGlobalOffset) CRCExtra() uint8 {
	return 231
}

// MsgName (generated function)
func (m *ArdupilotmegaLocalPositionNedSystemGlobalOffset) MsgName() string {
	return "LocalPositionNedSystemGlobalOffset"
}

// String (generated function)
func (m *ArdupilotmegaLocalPositionNedSystemGlobalOffset) String() string {
	return fmt.Sprintf("ArdupilotmegaLocalPositionNedSystemGlobalOffset{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaLocalPositionNedSystemGlobalOffset) Pack(p *Packet) error {
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
func (m *ArdupilotmegaLocalPositionNedSystemGlobalOffset) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 28 {
		return errPayloadTooSmall
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

// ArdupilotmegaHilState struct (generated typeinfo)
// Sent from simulation to autopilot. This packet is useful for high throughput applications such as hardware in the loop simulations.
type ArdupilotmegaHilState struct {
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

// Dialect (generated function)
func (m *ArdupilotmegaHilState) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaHilState) MsgID() MessageID {
	return MSG_ID_HIL_STATE
}

// CRCExtra (generated function)
func (m *ArdupilotmegaHilState) CRCExtra() uint8 {
	return 183
}

// MsgName (generated function)
func (m *ArdupilotmegaHilState) MsgName() string {
	return "HilState"
}

// String (generated function)
func (m *ArdupilotmegaHilState) String() string {
	return fmt.Sprintf("ArdupilotmegaHilState{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaHilState) Pack(p *Packet) error {
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
func (m *ArdupilotmegaHilState) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 56 {
		return errPayloadTooSmall
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

// ArdupilotmegaHilControls struct (generated typeinfo)
// Sent from autopilot to simulation. Hardware in the loop control outputs
type ArdupilotmegaHilControls struct {
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

// Dialect (generated function)
func (m *ArdupilotmegaHilControls) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaHilControls) MsgID() MessageID {
	return MSG_ID_HIL_CONTROLS
}

// CRCExtra (generated function)
func (m *ArdupilotmegaHilControls) CRCExtra() uint8 {
	return 63
}

// MsgName (generated function)
func (m *ArdupilotmegaHilControls) MsgName() string {
	return "HilControls"
}

// String (generated function)
func (m *ArdupilotmegaHilControls) String() string {
	return fmt.Sprintf("ArdupilotmegaHilControls{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaHilControls) Pack(p *Packet) error {
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
func (m *ArdupilotmegaHilControls) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 42 {
		return errPayloadTooSmall
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

// ArdupilotmegaHilRcInputsRaw struct (generated typeinfo)
// Sent from simulation to autopilot. The RAW values of the RC channels received. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual receivers/transmitters might violate this specification.
type ArdupilotmegaHilRcInputsRaw struct {
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

// Dialect (generated function)
func (m *ArdupilotmegaHilRcInputsRaw) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaHilRcInputsRaw) MsgID() MessageID {
	return MSG_ID_HIL_RC_INPUTS_RAW
}

// CRCExtra (generated function)
func (m *ArdupilotmegaHilRcInputsRaw) CRCExtra() uint8 {
	return 54
}

// MsgName (generated function)
func (m *ArdupilotmegaHilRcInputsRaw) MsgName() string {
	return "HilRcInputsRaw"
}

// String (generated function)
func (m *ArdupilotmegaHilRcInputsRaw) String() string {
	return fmt.Sprintf("ArdupilotmegaHilRcInputsRaw{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaHilRcInputsRaw) Pack(p *Packet) error {
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
func (m *ArdupilotmegaHilRcInputsRaw) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 33 {
		return errPayloadTooSmall
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

// ArdupilotmegaHilActuatorControls struct (generated typeinfo)
// Sent from autopilot to simulation. Hardware in the loop control outputs (replacement for HIL_CONTROLS)
type ArdupilotmegaHilActuatorControls struct {
	TimeUsec uint64      // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
	Flags    uint64      // Flags as bitfield, 1: indicate simulation using lockstep.
	Controls [16]float32 // Control outputs -1 .. 1. Channel assignment depends on the simulated hardware.
	Mode     uint8       // System mode. Includes arming state.
}

// Dialect (generated function)
func (m *ArdupilotmegaHilActuatorControls) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaHilActuatorControls) MsgID() MessageID {
	return MSG_ID_HIL_ACTUATOR_CONTROLS
}

// CRCExtra (generated function)
func (m *ArdupilotmegaHilActuatorControls) CRCExtra() uint8 {
	return 47
}

// MsgName (generated function)
func (m *ArdupilotmegaHilActuatorControls) MsgName() string {
	return "HilActuatorControls"
}

// String (generated function)
func (m *ArdupilotmegaHilActuatorControls) String() string {
	return fmt.Sprintf("ArdupilotmegaHilActuatorControls{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaHilActuatorControls) Pack(p *Packet) error {
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
func (m *ArdupilotmegaHilActuatorControls) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 81 {
		return errPayloadTooSmall
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(payload[0:]))
	m.Flags = uint64(binary.LittleEndian.Uint64(payload[8:]))
	for i := 0; i < len(m.Controls); i++ {
		m.Controls[i] = math.Float32frombits(binary.LittleEndian.Uint32(payload[16+i*4:]))
	}
	m.Mode = uint8(payload[80])
	return nil
}

// ArdupilotmegaOpticalFlow struct (generated typeinfo)
// Optical flow from a flow sensor (e.g. optical mouse sensor)
type ArdupilotmegaOpticalFlow struct {
	TimeUsec       uint64  // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
	FlowCompMX     float32 // Flow in x-sensor direction, angular-speed compensated
	FlowCompMY     float32 // Flow in y-sensor direction, angular-speed compensated
	GroundDistance float32 // Ground distance. Positive value: distance known. Negative value: Unknown distance
	FlowX          int16   // Flow in x-sensor direction
	FlowY          int16   // Flow in y-sensor direction
	SensorID       uint8   // Sensor ID
	Quality        uint8   // Optical flow quality / confidence. 0: bad, 255: maximum quality
}

// Dialect (generated function)
func (m *ArdupilotmegaOpticalFlow) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaOpticalFlow) MsgID() MessageID {
	return MSG_ID_OPTICAL_FLOW
}

// CRCExtra (generated function)
func (m *ArdupilotmegaOpticalFlow) CRCExtra() uint8 {
	return 175
}

// MsgName (generated function)
func (m *ArdupilotmegaOpticalFlow) MsgName() string {
	return "OpticalFlow"
}

// String (generated function)
func (m *ArdupilotmegaOpticalFlow) String() string {
	return fmt.Sprintf("ArdupilotmegaOpticalFlow{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaOpticalFlow) Pack(p *Packet) error {
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
func (m *ArdupilotmegaOpticalFlow) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 26 {
		return errPayloadTooSmall
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

// ArdupilotmegaGlobalVisionPositionEstimate struct (generated typeinfo)
// Global position/attitude estimate from a vision source.
type ArdupilotmegaGlobalVisionPositionEstimate struct {
	Usec  uint64  // Timestamp (UNIX time or since system boot)
	X     float32 // Global X position
	Y     float32 // Global Y position
	Z     float32 // Global Z position
	Roll  float32 // Roll angle
	Pitch float32 // Pitch angle
	Yaw   float32 // Yaw angle
}

// Dialect (generated function)
func (m *ArdupilotmegaGlobalVisionPositionEstimate) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaGlobalVisionPositionEstimate) MsgID() MessageID {
	return MSG_ID_GLOBAL_VISION_POSITION_ESTIMATE
}

// CRCExtra (generated function)
func (m *ArdupilotmegaGlobalVisionPositionEstimate) CRCExtra() uint8 {
	return 102
}

// MsgName (generated function)
func (m *ArdupilotmegaGlobalVisionPositionEstimate) MsgName() string {
	return "GlobalVisionPositionEstimate"
}

// String (generated function)
func (m *ArdupilotmegaGlobalVisionPositionEstimate) String() string {
	return fmt.Sprintf("ArdupilotmegaGlobalVisionPositionEstimate{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaGlobalVisionPositionEstimate) Pack(p *Packet) error {
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
func (m *ArdupilotmegaGlobalVisionPositionEstimate) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 32 {
		return errPayloadTooSmall
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

// ArdupilotmegaVisionPositionEstimate struct (generated typeinfo)
// Local position/attitude estimate from a vision source.
type ArdupilotmegaVisionPositionEstimate struct {
	Usec  uint64  // Timestamp (UNIX time or time since system boot)
	X     float32 // Local X position
	Y     float32 // Local Y position
	Z     float32 // Local Z position
	Roll  float32 // Roll angle
	Pitch float32 // Pitch angle
	Yaw   float32 // Yaw angle
}

// Dialect (generated function)
func (m *ArdupilotmegaVisionPositionEstimate) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaVisionPositionEstimate) MsgID() MessageID {
	return MSG_ID_VISION_POSITION_ESTIMATE
}

// CRCExtra (generated function)
func (m *ArdupilotmegaVisionPositionEstimate) CRCExtra() uint8 {
	return 158
}

// MsgName (generated function)
func (m *ArdupilotmegaVisionPositionEstimate) MsgName() string {
	return "VisionPositionEstimate"
}

// String (generated function)
func (m *ArdupilotmegaVisionPositionEstimate) String() string {
	return fmt.Sprintf("ArdupilotmegaVisionPositionEstimate{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaVisionPositionEstimate) Pack(p *Packet) error {
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
func (m *ArdupilotmegaVisionPositionEstimate) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 32 {
		return errPayloadTooSmall
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

// ArdupilotmegaVisionSpeedEstimate struct (generated typeinfo)
// Speed estimate from a vision source.
type ArdupilotmegaVisionSpeedEstimate struct {
	Usec uint64  // Timestamp (UNIX time or time since system boot)
	X    float32 // Global X speed
	Y    float32 // Global Y speed
	Z    float32 // Global Z speed
}

// Dialect (generated function)
func (m *ArdupilotmegaVisionSpeedEstimate) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaVisionSpeedEstimate) MsgID() MessageID {
	return MSG_ID_VISION_SPEED_ESTIMATE
}

// CRCExtra (generated function)
func (m *ArdupilotmegaVisionSpeedEstimate) CRCExtra() uint8 {
	return 208
}

// MsgName (generated function)
func (m *ArdupilotmegaVisionSpeedEstimate) MsgName() string {
	return "VisionSpeedEstimate"
}

// String (generated function)
func (m *ArdupilotmegaVisionSpeedEstimate) String() string {
	return fmt.Sprintf("ArdupilotmegaVisionSpeedEstimate{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaVisionSpeedEstimate) Pack(p *Packet) error {
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
func (m *ArdupilotmegaVisionSpeedEstimate) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 20 {
		return errPayloadTooSmall
	}
	m.Usec = uint64(binary.LittleEndian.Uint64(payload[0:]))
	m.X = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.Y = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.Z = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	return nil
}

// ArdupilotmegaViconPositionEstimate struct (generated typeinfo)
// Global position estimate from a Vicon motion system source.
type ArdupilotmegaViconPositionEstimate struct {
	Usec  uint64  // Timestamp (UNIX time or time since system boot)
	X     float32 // Global X position
	Y     float32 // Global Y position
	Z     float32 // Global Z position
	Roll  float32 // Roll angle
	Pitch float32 // Pitch angle
	Yaw   float32 // Yaw angle
}

// Dialect (generated function)
func (m *ArdupilotmegaViconPositionEstimate) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaViconPositionEstimate) MsgID() MessageID {
	return MSG_ID_VICON_POSITION_ESTIMATE
}

// CRCExtra (generated function)
func (m *ArdupilotmegaViconPositionEstimate) CRCExtra() uint8 {
	return 56
}

// MsgName (generated function)
func (m *ArdupilotmegaViconPositionEstimate) MsgName() string {
	return "ViconPositionEstimate"
}

// String (generated function)
func (m *ArdupilotmegaViconPositionEstimate) String() string {
	return fmt.Sprintf("ArdupilotmegaViconPositionEstimate{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaViconPositionEstimate) Pack(p *Packet) error {
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
func (m *ArdupilotmegaViconPositionEstimate) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 32 {
		return errPayloadTooSmall
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

// ArdupilotmegaHighresImu struct (generated typeinfo)
// The IMU readings in SI units in NED body frame
type ArdupilotmegaHighresImu struct {
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

// Dialect (generated function)
func (m *ArdupilotmegaHighresImu) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaHighresImu) MsgID() MessageID {
	return MSG_ID_HIGHRES_IMU
}

// CRCExtra (generated function)
func (m *ArdupilotmegaHighresImu) CRCExtra() uint8 {
	return 93
}

// MsgName (generated function)
func (m *ArdupilotmegaHighresImu) MsgName() string {
	return "HighresImu"
}

// String (generated function)
func (m *ArdupilotmegaHighresImu) String() string {
	return fmt.Sprintf("ArdupilotmegaHighresImu{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaHighresImu) Pack(p *Packet) error {
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
func (m *ArdupilotmegaHighresImu) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 62 {
		return errPayloadTooSmall
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

// ArdupilotmegaOpticalFlowRad struct (generated typeinfo)
// Optical flow from an angular rate flow sensor (e.g. PX4FLOW or mouse sensor)
type ArdupilotmegaOpticalFlowRad struct {
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

// Dialect (generated function)
func (m *ArdupilotmegaOpticalFlowRad) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaOpticalFlowRad) MsgID() MessageID {
	return MSG_ID_OPTICAL_FLOW_RAD
}

// CRCExtra (generated function)
func (m *ArdupilotmegaOpticalFlowRad) CRCExtra() uint8 {
	return 138
}

// MsgName (generated function)
func (m *ArdupilotmegaOpticalFlowRad) MsgName() string {
	return "OpticalFlowRad"
}

// String (generated function)
func (m *ArdupilotmegaOpticalFlowRad) String() string {
	return fmt.Sprintf("ArdupilotmegaOpticalFlowRad{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaOpticalFlowRad) Pack(p *Packet) error {
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
func (m *ArdupilotmegaOpticalFlowRad) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 44 {
		return errPayloadTooSmall
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

// ArdupilotmegaHilSensor struct (generated typeinfo)
// The IMU readings in SI units in NED body frame
type ArdupilotmegaHilSensor struct {
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

// Dialect (generated function)
func (m *ArdupilotmegaHilSensor) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaHilSensor) MsgID() MessageID {
	return MSG_ID_HIL_SENSOR
}

// CRCExtra (generated function)
func (m *ArdupilotmegaHilSensor) CRCExtra() uint8 {
	return 108
}

// MsgName (generated function)
func (m *ArdupilotmegaHilSensor) MsgName() string {
	return "HilSensor"
}

// String (generated function)
func (m *ArdupilotmegaHilSensor) String() string {
	return fmt.Sprintf("ArdupilotmegaHilSensor{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaHilSensor) Pack(p *Packet) error {
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
func (m *ArdupilotmegaHilSensor) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 64 {
		return errPayloadTooSmall
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

// ArdupilotmegaSimState struct (generated typeinfo)
// Status of simulation environment, if used
type ArdupilotmegaSimState struct {
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

// Dialect (generated function)
func (m *ArdupilotmegaSimState) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaSimState) MsgID() MessageID {
	return MSG_ID_SIM_STATE
}

// CRCExtra (generated function)
func (m *ArdupilotmegaSimState) CRCExtra() uint8 {
	return 32
}

// MsgName (generated function)
func (m *ArdupilotmegaSimState) MsgName() string {
	return "SimState"
}

// String (generated function)
func (m *ArdupilotmegaSimState) String() string {
	return fmt.Sprintf("ArdupilotmegaSimState{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaSimState) Pack(p *Packet) error {
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
func (m *ArdupilotmegaSimState) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 84 {
		return errPayloadTooSmall
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

// ArdupilotmegaRadioStatus struct (generated typeinfo)
// Status generated by radio and injected into MAVLink stream.
type ArdupilotmegaRadioStatus struct {
	Rxerrors uint16 // Count of radio packet receive errors (since boot).
	Fixed    uint16 // Count of error corrected radio packets (since boot).
	Rssi     uint8  // Local (message sender) recieved signal strength indication in device-dependent units/scale. Values: [0-254], 255: invalid/unknown.
	Remrssi  uint8  // Remote (message receiver) signal strength indication in device-dependent units/scale. Values: [0-254], 255: invalid/unknown.
	Txbuf    uint8  // Remaining free transmitter buffer space.
	Noise    uint8  // Local background noise level. These are device dependent RSSI values (scale as approx 2x dB on SiK radios). Values: [0-254], 255: invalid/unknown.
	Remnoise uint8  // Remote background noise level. These are device dependent RSSI values (scale as approx 2x dB on SiK radios). Values: [0-254], 255: invalid/unknown.
}

// Dialect (generated function)
func (m *ArdupilotmegaRadioStatus) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaRadioStatus) MsgID() MessageID {
	return MSG_ID_RADIO_STATUS
}

// CRCExtra (generated function)
func (m *ArdupilotmegaRadioStatus) CRCExtra() uint8 {
	return 185
}

// MsgName (generated function)
func (m *ArdupilotmegaRadioStatus) MsgName() string {
	return "RadioStatus"
}

// String (generated function)
func (m *ArdupilotmegaRadioStatus) String() string {
	return fmt.Sprintf("ArdupilotmegaRadioStatus{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaRadioStatus) Pack(p *Packet) error {
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
func (m *ArdupilotmegaRadioStatus) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 9 {
		return errPayloadTooSmall
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

// ArdupilotmegaFileTransferProtocol struct (generated typeinfo)
// File transfer message
type ArdupilotmegaFileTransferProtocol struct {
	TargetNetwork   uint8      // Network ID (0 for broadcast)
	TargetSystem    uint8      // System ID (0 for broadcast)
	TargetComponent uint8      // Component ID (0 for broadcast)
	Payload         [251]uint8 // Variable length payload. The length is defined by the remaining message length when subtracting the header and other fields.  The entire content of this block is opaque unless you understand any the encoding message_type.  The particular encoding used can be extension specific and might not always be documented as part of the mavlink specification.
}

// Dialect (generated function)
func (m *ArdupilotmegaFileTransferProtocol) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaFileTransferProtocol) MsgID() MessageID {
	return MSG_ID_FILE_TRANSFER_PROTOCOL
}

// CRCExtra (generated function)
func (m *ArdupilotmegaFileTransferProtocol) CRCExtra() uint8 {
	return 84
}

// MsgName (generated function)
func (m *ArdupilotmegaFileTransferProtocol) MsgName() string {
	return "FileTransferProtocol"
}

// String (generated function)
func (m *ArdupilotmegaFileTransferProtocol) String() string {
	return fmt.Sprintf("ArdupilotmegaFileTransferProtocol{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaFileTransferProtocol) Pack(p *Packet) error {
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
func (m *ArdupilotmegaFileTransferProtocol) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 254 {
		return errPayloadTooSmall
	}
	m.TargetNetwork = uint8(payload[0])
	m.TargetSystem = uint8(payload[1])
	m.TargetComponent = uint8(payload[2])
	copy(m.Payload[:], payload[3:254])
	return nil
}

// ArdupilotmegaTimesync struct (generated typeinfo)
// Time synchronization message.
type ArdupilotmegaTimesync struct {
	Tc1 int64 // Time sync timestamp 1
	Ts1 int64 // Time sync timestamp 2
}

// Dialect (generated function)
func (m *ArdupilotmegaTimesync) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaTimesync) MsgID() MessageID {
	return MSG_ID_TIMESYNC
}

// CRCExtra (generated function)
func (m *ArdupilotmegaTimesync) CRCExtra() uint8 {
	return 34
}

// MsgName (generated function)
func (m *ArdupilotmegaTimesync) MsgName() string {
	return "Timesync"
}

// String (generated function)
func (m *ArdupilotmegaTimesync) String() string {
	return fmt.Sprintf("ArdupilotmegaTimesync{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaTimesync) Pack(p *Packet) error {
	payload := make([]byte, 16)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.Tc1))
	binary.LittleEndian.PutUint64(payload[8:], uint64(m.Ts1))
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ArdupilotmegaTimesync) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 16 {
		return errPayloadTooSmall
	}
	m.Tc1 = int64(binary.LittleEndian.Uint64(payload[0:]))
	m.Ts1 = int64(binary.LittleEndian.Uint64(payload[8:]))
	return nil
}

// ArdupilotmegaCameraTrigger struct (generated typeinfo)
// Camera-IMU triggering and synchronisation message.
type ArdupilotmegaCameraTrigger struct {
	TimeUsec uint64 // Timestamp for image frame (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
	Seq      uint32 // Image frame sequence
}

// Dialect (generated function)
func (m *ArdupilotmegaCameraTrigger) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaCameraTrigger) MsgID() MessageID {
	return MSG_ID_CAMERA_TRIGGER
}

// CRCExtra (generated function)
func (m *ArdupilotmegaCameraTrigger) CRCExtra() uint8 {
	return 174
}

// MsgName (generated function)
func (m *ArdupilotmegaCameraTrigger) MsgName() string {
	return "CameraTrigger"
}

// String (generated function)
func (m *ArdupilotmegaCameraTrigger) String() string {
	return fmt.Sprintf("ArdupilotmegaCameraTrigger{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaCameraTrigger) Pack(p *Packet) error {
	payload := make([]byte, 12)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.TimeUsec))
	binary.LittleEndian.PutUint32(payload[8:], uint32(m.Seq))
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ArdupilotmegaCameraTrigger) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 12 {
		return errPayloadTooSmall
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(payload[0:]))
	m.Seq = uint32(binary.LittleEndian.Uint32(payload[8:]))
	return nil
}

// ArdupilotmegaHilGps struct (generated typeinfo)
// The global position, as returned by the Global Positioning System (GPS). This is
//                  NOT the global position estimate of the sytem, but rather a RAW sensor value. See message GLOBAL_POSITION for the global position estimate.
type ArdupilotmegaHilGps struct {
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

// Dialect (generated function)
func (m *ArdupilotmegaHilGps) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaHilGps) MsgID() MessageID {
	return MSG_ID_HIL_GPS
}

// CRCExtra (generated function)
func (m *ArdupilotmegaHilGps) CRCExtra() uint8 {
	return 124
}

// MsgName (generated function)
func (m *ArdupilotmegaHilGps) MsgName() string {
	return "HilGps"
}

// String (generated function)
func (m *ArdupilotmegaHilGps) String() string {
	return fmt.Sprintf("ArdupilotmegaHilGps{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaHilGps) Pack(p *Packet) error {
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
func (m *ArdupilotmegaHilGps) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 36 {
		return errPayloadTooSmall
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

// ArdupilotmegaHilOpticalFlow struct (generated typeinfo)
// Simulated optical flow from a flow sensor (e.g. PX4FLOW or optical mouse sensor)
type ArdupilotmegaHilOpticalFlow struct {
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

// Dialect (generated function)
func (m *ArdupilotmegaHilOpticalFlow) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaHilOpticalFlow) MsgID() MessageID {
	return MSG_ID_HIL_OPTICAL_FLOW
}

// CRCExtra (generated function)
func (m *ArdupilotmegaHilOpticalFlow) CRCExtra() uint8 {
	return 237
}

// MsgName (generated function)
func (m *ArdupilotmegaHilOpticalFlow) MsgName() string {
	return "HilOpticalFlow"
}

// String (generated function)
func (m *ArdupilotmegaHilOpticalFlow) String() string {
	return fmt.Sprintf("ArdupilotmegaHilOpticalFlow{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaHilOpticalFlow) Pack(p *Packet) error {
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
func (m *ArdupilotmegaHilOpticalFlow) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 44 {
		return errPayloadTooSmall
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

// ArdupilotmegaHilStateQuaternion struct (generated typeinfo)
// Sent from simulation to autopilot, avoids in contrast to HIL_STATE singularities. This packet is useful for high throughput applications such as hardware in the loop simulations.
type ArdupilotmegaHilStateQuaternion struct {
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

// Dialect (generated function)
func (m *ArdupilotmegaHilStateQuaternion) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaHilStateQuaternion) MsgID() MessageID {
	return MSG_ID_HIL_STATE_QUATERNION
}

// CRCExtra (generated function)
func (m *ArdupilotmegaHilStateQuaternion) CRCExtra() uint8 {
	return 4
}

// MsgName (generated function)
func (m *ArdupilotmegaHilStateQuaternion) MsgName() string {
	return "HilStateQuaternion"
}

// String (generated function)
func (m *ArdupilotmegaHilStateQuaternion) String() string {
	return fmt.Sprintf("ArdupilotmegaHilStateQuaternion{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaHilStateQuaternion) Pack(p *Packet) error {
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
func (m *ArdupilotmegaHilStateQuaternion) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 64 {
		return errPayloadTooSmall
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

// ArdupilotmegaScaledImu2 struct (generated typeinfo)
// The RAW IMU readings for secondary 9DOF sensor setup. This message should contain the scaled values to the described units
type ArdupilotmegaScaledImu2 struct {
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

// Dialect (generated function)
func (m *ArdupilotmegaScaledImu2) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaScaledImu2) MsgID() MessageID {
	return MSG_ID_SCALED_IMU2
}

// CRCExtra (generated function)
func (m *ArdupilotmegaScaledImu2) CRCExtra() uint8 {
	return 76
}

// MsgName (generated function)
func (m *ArdupilotmegaScaledImu2) MsgName() string {
	return "ScaledImu2"
}

// String (generated function)
func (m *ArdupilotmegaScaledImu2) String() string {
	return fmt.Sprintf("ArdupilotmegaScaledImu2{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaScaledImu2) Pack(p *Packet) error {
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
func (m *ArdupilotmegaScaledImu2) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 22 {
		return errPayloadTooSmall
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

// ArdupilotmegaLogRequestList struct (generated typeinfo)
// Request a list of available logs. On some systems calling this may stop on-board logging until LOG_REQUEST_END is called. If there are no log files available this request shall be answered with one LOG_ENTRY message with id = 0 and num_logs = 0.
type ArdupilotmegaLogRequestList struct {
	Start           uint16 // First log id (0 for first available)
	End             uint16 // Last log id (0xffff for last available)
	TargetSystem    uint8  // System ID
	TargetComponent uint8  // Component ID
}

// Dialect (generated function)
func (m *ArdupilotmegaLogRequestList) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaLogRequestList) MsgID() MessageID {
	return MSG_ID_LOG_REQUEST_LIST
}

// CRCExtra (generated function)
func (m *ArdupilotmegaLogRequestList) CRCExtra() uint8 {
	return 128
}

// MsgName (generated function)
func (m *ArdupilotmegaLogRequestList) MsgName() string {
	return "LogRequestList"
}

// String (generated function)
func (m *ArdupilotmegaLogRequestList) String() string {
	return fmt.Sprintf("ArdupilotmegaLogRequestList{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaLogRequestList) Pack(p *Packet) error {
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
func (m *ArdupilotmegaLogRequestList) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 6 {
		return errPayloadTooSmall
	}
	m.Start = uint16(binary.LittleEndian.Uint16(payload[0:]))
	m.End = uint16(binary.LittleEndian.Uint16(payload[2:]))
	m.TargetSystem = uint8(payload[4])
	m.TargetComponent = uint8(payload[5])
	return nil
}

// ArdupilotmegaLogEntry struct (generated typeinfo)
// Reply to LOG_REQUEST_LIST
type ArdupilotmegaLogEntry struct {
	TimeUtc    uint32 // UTC timestamp of log since 1970, or 0 if not available
	Size       uint32 // Size of the log (may be approximate)
	ID         uint16 // Log id
	NumLogs    uint16 // Total number of logs
	LastLogNum uint16 // High log number
}

// Dialect (generated function)
func (m *ArdupilotmegaLogEntry) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaLogEntry) MsgID() MessageID {
	return MSG_ID_LOG_ENTRY
}

// CRCExtra (generated function)
func (m *ArdupilotmegaLogEntry) CRCExtra() uint8 {
	return 56
}

// MsgName (generated function)
func (m *ArdupilotmegaLogEntry) MsgName() string {
	return "LogEntry"
}

// String (generated function)
func (m *ArdupilotmegaLogEntry) String() string {
	return fmt.Sprintf("ArdupilotmegaLogEntry{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaLogEntry) Pack(p *Packet) error {
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
func (m *ArdupilotmegaLogEntry) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 14 {
		return errPayloadTooSmall
	}
	m.TimeUtc = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.Size = uint32(binary.LittleEndian.Uint32(payload[4:]))
	m.ID = uint16(binary.LittleEndian.Uint16(payload[8:]))
	m.NumLogs = uint16(binary.LittleEndian.Uint16(payload[10:]))
	m.LastLogNum = uint16(binary.LittleEndian.Uint16(payload[12:]))
	return nil
}

// ArdupilotmegaLogRequestData struct (generated typeinfo)
// Request a chunk of a log
type ArdupilotmegaLogRequestData struct {
	Ofs             uint32 // Offset into the log
	Count           uint32 // Number of bytes
	ID              uint16 // Log id (from LOG_ENTRY reply)
	TargetSystem    uint8  // System ID
	TargetComponent uint8  // Component ID
}

// Dialect (generated function)
func (m *ArdupilotmegaLogRequestData) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaLogRequestData) MsgID() MessageID {
	return MSG_ID_LOG_REQUEST_DATA
}

// CRCExtra (generated function)
func (m *ArdupilotmegaLogRequestData) CRCExtra() uint8 {
	return 116
}

// MsgName (generated function)
func (m *ArdupilotmegaLogRequestData) MsgName() string {
	return "LogRequestData"
}

// String (generated function)
func (m *ArdupilotmegaLogRequestData) String() string {
	return fmt.Sprintf("ArdupilotmegaLogRequestData{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaLogRequestData) Pack(p *Packet) error {
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
func (m *ArdupilotmegaLogRequestData) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 12 {
		return errPayloadTooSmall
	}
	m.Ofs = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.Count = uint32(binary.LittleEndian.Uint32(payload[4:]))
	m.ID = uint16(binary.LittleEndian.Uint16(payload[8:]))
	m.TargetSystem = uint8(payload[10])
	m.TargetComponent = uint8(payload[11])
	return nil
}

// ArdupilotmegaLogData struct (generated typeinfo)
// Reply to LOG_REQUEST_DATA
type ArdupilotmegaLogData struct {
	Ofs   uint32    // Offset into the log
	ID    uint16    // Log id (from LOG_ENTRY reply)
	Count uint8     // Number of bytes (zero for end of log)
	Data  [90]uint8 // log data
}

// Dialect (generated function)
func (m *ArdupilotmegaLogData) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaLogData) MsgID() MessageID {
	return MSG_ID_LOG_DATA
}

// CRCExtra (generated function)
func (m *ArdupilotmegaLogData) CRCExtra() uint8 {
	return 134
}

// MsgName (generated function)
func (m *ArdupilotmegaLogData) MsgName() string {
	return "LogData"
}

// String (generated function)
func (m *ArdupilotmegaLogData) String() string {
	return fmt.Sprintf("ArdupilotmegaLogData{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaLogData) Pack(p *Packet) error {
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
func (m *ArdupilotmegaLogData) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 97 {
		return errPayloadTooSmall
	}
	m.Ofs = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.ID = uint16(binary.LittleEndian.Uint16(payload[4:]))
	m.Count = uint8(payload[6])
	copy(m.Data[:], payload[7:97])
	return nil
}

// ArdupilotmegaLogErase struct (generated typeinfo)
// Erase all logs
type ArdupilotmegaLogErase struct {
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
}

// Dialect (generated function)
func (m *ArdupilotmegaLogErase) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaLogErase) MsgID() MessageID {
	return MSG_ID_LOG_ERASE
}

// CRCExtra (generated function)
func (m *ArdupilotmegaLogErase) CRCExtra() uint8 {
	return 237
}

// MsgName (generated function)
func (m *ArdupilotmegaLogErase) MsgName() string {
	return "LogErase"
}

// String (generated function)
func (m *ArdupilotmegaLogErase) String() string {
	return fmt.Sprintf("ArdupilotmegaLogErase{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaLogErase) Pack(p *Packet) error {
	payload := make([]byte, 2)
	payload[0] = byte(m.TargetSystem)
	payload[1] = byte(m.TargetComponent)
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ArdupilotmegaLogErase) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 2 {
		return errPayloadTooSmall
	}
	m.TargetSystem = uint8(payload[0])
	m.TargetComponent = uint8(payload[1])
	return nil
}

// ArdupilotmegaLogRequestEnd struct (generated typeinfo)
// Stop log transfer and resume normal logging
type ArdupilotmegaLogRequestEnd struct {
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
}

// Dialect (generated function)
func (m *ArdupilotmegaLogRequestEnd) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaLogRequestEnd) MsgID() MessageID {
	return MSG_ID_LOG_REQUEST_END
}

// CRCExtra (generated function)
func (m *ArdupilotmegaLogRequestEnd) CRCExtra() uint8 {
	return 203
}

// MsgName (generated function)
func (m *ArdupilotmegaLogRequestEnd) MsgName() string {
	return "LogRequestEnd"
}

// String (generated function)
func (m *ArdupilotmegaLogRequestEnd) String() string {
	return fmt.Sprintf("ArdupilotmegaLogRequestEnd{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaLogRequestEnd) Pack(p *Packet) error {
	payload := make([]byte, 2)
	payload[0] = byte(m.TargetSystem)
	payload[1] = byte(m.TargetComponent)
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ArdupilotmegaLogRequestEnd) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 2 {
		return errPayloadTooSmall
	}
	m.TargetSystem = uint8(payload[0])
	m.TargetComponent = uint8(payload[1])
	return nil
}

// ArdupilotmegaGpsInjectData struct (generated typeinfo)
// Data for injecting into the onboard GPS (used for DGPS)
type ArdupilotmegaGpsInjectData struct {
	TargetSystem    uint8      // System ID
	TargetComponent uint8      // Component ID
	Len             uint8      // Data length
	Data            [110]uint8 // Raw data (110 is enough for 12 satellites of RTCMv2)
}

// Dialect (generated function)
func (m *ArdupilotmegaGpsInjectData) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaGpsInjectData) MsgID() MessageID {
	return MSG_ID_GPS_INJECT_DATA
}

// CRCExtra (generated function)
func (m *ArdupilotmegaGpsInjectData) CRCExtra() uint8 {
	return 250
}

// MsgName (generated function)
func (m *ArdupilotmegaGpsInjectData) MsgName() string {
	return "GpsInjectData"
}

// String (generated function)
func (m *ArdupilotmegaGpsInjectData) String() string {
	return fmt.Sprintf("ArdupilotmegaGpsInjectData{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaGpsInjectData) Pack(p *Packet) error {
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
func (m *ArdupilotmegaGpsInjectData) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 113 {
		return errPayloadTooSmall
	}
	m.TargetSystem = uint8(payload[0])
	m.TargetComponent = uint8(payload[1])
	m.Len = uint8(payload[2])
	copy(m.Data[:], payload[3:113])
	return nil
}

// ArdupilotmegaGps2Raw struct (generated typeinfo)
// Second GPS data.
type ArdupilotmegaGps2Raw struct {
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

// Dialect (generated function)
func (m *ArdupilotmegaGps2Raw) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaGps2Raw) MsgID() MessageID {
	return MSG_ID_GPS2_RAW
}

// CRCExtra (generated function)
func (m *ArdupilotmegaGps2Raw) CRCExtra() uint8 {
	return 87
}

// MsgName (generated function)
func (m *ArdupilotmegaGps2Raw) MsgName() string {
	return "Gps2Raw"
}

// String (generated function)
func (m *ArdupilotmegaGps2Raw) String() string {
	return fmt.Sprintf("ArdupilotmegaGps2Raw{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaGps2Raw) Pack(p *Packet) error {
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
func (m *ArdupilotmegaGps2Raw) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 35 {
		return errPayloadTooSmall
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

// ArdupilotmegaPowerStatus struct (generated typeinfo)
// Power supply status
type ArdupilotmegaPowerStatus struct {
	Vcc    uint16 // 5V rail voltage.
	Vservo uint16 // Servo rail voltage.
	Flags  uint16 // Bitmap of power supply status flags.
}

// Dialect (generated function)
func (m *ArdupilotmegaPowerStatus) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaPowerStatus) MsgID() MessageID {
	return MSG_ID_POWER_STATUS
}

// CRCExtra (generated function)
func (m *ArdupilotmegaPowerStatus) CRCExtra() uint8 {
	return 203
}

// MsgName (generated function)
func (m *ArdupilotmegaPowerStatus) MsgName() string {
	return "PowerStatus"
}

// String (generated function)
func (m *ArdupilotmegaPowerStatus) String() string {
	return fmt.Sprintf("ArdupilotmegaPowerStatus{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaPowerStatus) Pack(p *Packet) error {
	payload := make([]byte, 6)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.Vcc))
	binary.LittleEndian.PutUint16(payload[2:], uint16(m.Vservo))
	binary.LittleEndian.PutUint16(payload[4:], uint16(m.Flags))
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ArdupilotmegaPowerStatus) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 6 {
		return errPayloadTooSmall
	}
	m.Vcc = uint16(binary.LittleEndian.Uint16(payload[0:]))
	m.Vservo = uint16(binary.LittleEndian.Uint16(payload[2:]))
	m.Flags = uint16(binary.LittleEndian.Uint16(payload[4:]))
	return nil
}

// ArdupilotmegaSerialControl struct (generated typeinfo)
// Control a serial port. This can be used for raw access to an onboard serial peripheral such as a GPS or telemetry radio. It is designed to make it possible to update the devices firmware via MAVLink messages or change the devices settings. A message with zero bytes can be used to change just the baudrate.
type ArdupilotmegaSerialControl struct {
	Baudrate uint32    // Baudrate of transfer. Zero means no change.
	Timeout  uint16    // Timeout for reply data
	Device   uint8     // Serial control device type.
	Flags    uint8     // Bitmap of serial control flags.
	Count    uint8     // how many bytes in this transfer
	Data     [70]uint8 // serial data
}

// Dialect (generated function)
func (m *ArdupilotmegaSerialControl) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaSerialControl) MsgID() MessageID {
	return MSG_ID_SERIAL_CONTROL
}

// CRCExtra (generated function)
func (m *ArdupilotmegaSerialControl) CRCExtra() uint8 {
	return 220
}

// MsgName (generated function)
func (m *ArdupilotmegaSerialControl) MsgName() string {
	return "SerialControl"
}

// String (generated function)
func (m *ArdupilotmegaSerialControl) String() string {
	return fmt.Sprintf("ArdupilotmegaSerialControl{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaSerialControl) Pack(p *Packet) error {
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
func (m *ArdupilotmegaSerialControl) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 79 {
		return errPayloadTooSmall
	}
	m.Baudrate = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.Timeout = uint16(binary.LittleEndian.Uint16(payload[4:]))
	m.Device = uint8(payload[6])
	m.Flags = uint8(payload[7])
	m.Count = uint8(payload[8])
	copy(m.Data[:], payload[9:79])
	return nil
}

// ArdupilotmegaGpsRtk struct (generated typeinfo)
// RTK GPS data. Gives information on the relative baseline calculation the GPS is reporting
type ArdupilotmegaGpsRtk struct {
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

// Dialect (generated function)
func (m *ArdupilotmegaGpsRtk) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaGpsRtk) MsgID() MessageID {
	return MSG_ID_GPS_RTK
}

// CRCExtra (generated function)
func (m *ArdupilotmegaGpsRtk) CRCExtra() uint8 {
	return 25
}

// MsgName (generated function)
func (m *ArdupilotmegaGpsRtk) MsgName() string {
	return "GpsRtk"
}

// String (generated function)
func (m *ArdupilotmegaGpsRtk) String() string {
	return fmt.Sprintf("ArdupilotmegaGpsRtk{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaGpsRtk) Pack(p *Packet) error {
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
func (m *ArdupilotmegaGpsRtk) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 35 {
		return errPayloadTooSmall
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

// ArdupilotmegaGps2Rtk struct (generated typeinfo)
// RTK GPS data. Gives information on the relative baseline calculation the GPS is reporting
type ArdupilotmegaGps2Rtk struct {
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

// Dialect (generated function)
func (m *ArdupilotmegaGps2Rtk) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaGps2Rtk) MsgID() MessageID {
	return MSG_ID_GPS2_RTK
}

// CRCExtra (generated function)
func (m *ArdupilotmegaGps2Rtk) CRCExtra() uint8 {
	return 226
}

// MsgName (generated function)
func (m *ArdupilotmegaGps2Rtk) MsgName() string {
	return "Gps2Rtk"
}

// String (generated function)
func (m *ArdupilotmegaGps2Rtk) String() string {
	return fmt.Sprintf("ArdupilotmegaGps2Rtk{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaGps2Rtk) Pack(p *Packet) error {
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
func (m *ArdupilotmegaGps2Rtk) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 35 {
		return errPayloadTooSmall
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

// ArdupilotmegaScaledImu3 struct (generated typeinfo)
// The RAW IMU readings for 3rd 9DOF sensor setup. This message should contain the scaled values to the described units
type ArdupilotmegaScaledImu3 struct {
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

// Dialect (generated function)
func (m *ArdupilotmegaScaledImu3) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaScaledImu3) MsgID() MessageID {
	return MSG_ID_SCALED_IMU3
}

// CRCExtra (generated function)
func (m *ArdupilotmegaScaledImu3) CRCExtra() uint8 {
	return 46
}

// MsgName (generated function)
func (m *ArdupilotmegaScaledImu3) MsgName() string {
	return "ScaledImu3"
}

// String (generated function)
func (m *ArdupilotmegaScaledImu3) String() string {
	return fmt.Sprintf("ArdupilotmegaScaledImu3{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaScaledImu3) Pack(p *Packet) error {
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
func (m *ArdupilotmegaScaledImu3) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 22 {
		return errPayloadTooSmall
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

// ArdupilotmegaDataTransmissionHandshake struct (generated typeinfo)
// Handshake message to initiate, control and stop image streaming when using the Image Transmission Protocol: https://mavlink.io/en/services/image_transmission.html.
type ArdupilotmegaDataTransmissionHandshake struct {
	Size       uint32 // total data size (set on ACK only).
	Width      uint16 // Width of a matrix or image.
	Height     uint16 // Height of a matrix or image.
	Packets    uint16 // Number of packets being sent (set on ACK only).
	Type       uint8  // Type of requested/acknowledged data.
	Payload    uint8  // Payload size per packet (normally 253 byte, see DATA field size in message ENCAPSULATED_DATA) (set on ACK only).
	JpgQuality uint8  // JPEG quality. Values: [1-100].
}

// Dialect (generated function)
func (m *ArdupilotmegaDataTransmissionHandshake) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaDataTransmissionHandshake) MsgID() MessageID {
	return MSG_ID_DATA_TRANSMISSION_HANDSHAKE
}

// CRCExtra (generated function)
func (m *ArdupilotmegaDataTransmissionHandshake) CRCExtra() uint8 {
	return 29
}

// MsgName (generated function)
func (m *ArdupilotmegaDataTransmissionHandshake) MsgName() string {
	return "DataTransmissionHandshake"
}

// String (generated function)
func (m *ArdupilotmegaDataTransmissionHandshake) String() string {
	return fmt.Sprintf("ArdupilotmegaDataTransmissionHandshake{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaDataTransmissionHandshake) Pack(p *Packet) error {
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
func (m *ArdupilotmegaDataTransmissionHandshake) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 13 {
		return errPayloadTooSmall
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

// ArdupilotmegaEncapsulatedData struct (generated typeinfo)
// Data packet for images sent using the Image Transmission Protocol: https://mavlink.io/en/services/image_transmission.html.
type ArdupilotmegaEncapsulatedData struct {
	Seqnr uint16     // sequence number (starting with 0 on every transmission)
	Data  [253]uint8 // image data bytes
}

// Dialect (generated function)
func (m *ArdupilotmegaEncapsulatedData) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaEncapsulatedData) MsgID() MessageID {
	return MSG_ID_ENCAPSULATED_DATA
}

// CRCExtra (generated function)
func (m *ArdupilotmegaEncapsulatedData) CRCExtra() uint8 {
	return 223
}

// MsgName (generated function)
func (m *ArdupilotmegaEncapsulatedData) MsgName() string {
	return "EncapsulatedData"
}

// String (generated function)
func (m *ArdupilotmegaEncapsulatedData) String() string {
	return fmt.Sprintf("ArdupilotmegaEncapsulatedData{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaEncapsulatedData) Pack(p *Packet) error {
	payload := make([]byte, 255)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.Seqnr))
	copy(payload[2:], m.Data[:])
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ArdupilotmegaEncapsulatedData) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 255 {
		return errPayloadTooSmall
	}
	m.Seqnr = uint16(binary.LittleEndian.Uint16(payload[0:]))
	copy(m.Data[:], payload[2:255])
	return nil
}

// ArdupilotmegaDistanceSensor struct (generated typeinfo)
// Distance sensor information for an onboard rangefinder.
type ArdupilotmegaDistanceSensor struct {
	TimeBootMs      uint32 // Timestamp (time since system boot).
	MinDistance     uint16 // Minimum distance the sensor can measure
	MaxDistance     uint16 // Maximum distance the sensor can measure
	CurrentDistance uint16 // Current distance reading
	Type            uint8  // Type of distance sensor.
	ID              uint8  // Onboard ID of the sensor
	Orientation     uint8  // Direction the sensor faces. downward-facing: ROTATION_PITCH_270, upward-facing: ROTATION_PITCH_90, backward-facing: ROTATION_PITCH_180, forward-facing: ROTATION_NONE, left-facing: ROTATION_YAW_90, right-facing: ROTATION_YAW_270
	Covariance      uint8  // Measurement variance. Max standard deviation is 6cm. 255 if unknown.
}

// Dialect (generated function)
func (m *ArdupilotmegaDistanceSensor) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaDistanceSensor) MsgID() MessageID {
	return MSG_ID_DISTANCE_SENSOR
}

// CRCExtra (generated function)
func (m *ArdupilotmegaDistanceSensor) CRCExtra() uint8 {
	return 85
}

// MsgName (generated function)
func (m *ArdupilotmegaDistanceSensor) MsgName() string {
	return "DistanceSensor"
}

// String (generated function)
func (m *ArdupilotmegaDistanceSensor) String() string {
	return fmt.Sprintf("ArdupilotmegaDistanceSensor{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaDistanceSensor) Pack(p *Packet) error {
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
func (m *ArdupilotmegaDistanceSensor) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 14 {
		return errPayloadTooSmall
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

// ArdupilotmegaTerrainRequest struct (generated typeinfo)
// Request for terrain data and terrain status. See terrain protocol docs: https://mavlink.io/en/services/terrain.html
type ArdupilotmegaTerrainRequest struct {
	Mask        uint64 // Bitmask of requested 4x4 grids (row major 8x7 array of grids, 56 bits)
	Lat         int32  // Latitude of SW corner of first grid
	Lon         int32  // Longitude of SW corner of first grid
	GridSpacing uint16 // Grid spacing
}

// Dialect (generated function)
func (m *ArdupilotmegaTerrainRequest) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaTerrainRequest) MsgID() MessageID {
	return MSG_ID_TERRAIN_REQUEST
}

// CRCExtra (generated function)
func (m *ArdupilotmegaTerrainRequest) CRCExtra() uint8 {
	return 6
}

// MsgName (generated function)
func (m *ArdupilotmegaTerrainRequest) MsgName() string {
	return "TerrainRequest"
}

// String (generated function)
func (m *ArdupilotmegaTerrainRequest) String() string {
	return fmt.Sprintf("ArdupilotmegaTerrainRequest{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaTerrainRequest) Pack(p *Packet) error {
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
func (m *ArdupilotmegaTerrainRequest) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 18 {
		return errPayloadTooSmall
	}
	m.Mask = uint64(binary.LittleEndian.Uint64(payload[0:]))
	m.Lat = int32(binary.LittleEndian.Uint32(payload[8:]))
	m.Lon = int32(binary.LittleEndian.Uint32(payload[12:]))
	m.GridSpacing = uint16(binary.LittleEndian.Uint16(payload[16:]))
	return nil
}

// ArdupilotmegaTerrainData struct (generated typeinfo)
// Terrain data sent from GCS. The lat/lon and grid_spacing must be the same as a lat/lon from a TERRAIN_REQUEST. See terrain protocol docs: https://mavlink.io/en/services/terrain.html
type ArdupilotmegaTerrainData struct {
	Lat         int32     // Latitude of SW corner of first grid
	Lon         int32     // Longitude of SW corner of first grid
	GridSpacing uint16    // Grid spacing
	Data        [16]int16 // Terrain data MSL
	Gridbit     uint8     // bit within the terrain request mask
}

// Dialect (generated function)
func (m *ArdupilotmegaTerrainData) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaTerrainData) MsgID() MessageID {
	return MSG_ID_TERRAIN_DATA
}

// CRCExtra (generated function)
func (m *ArdupilotmegaTerrainData) CRCExtra() uint8 {
	return 229
}

// MsgName (generated function)
func (m *ArdupilotmegaTerrainData) MsgName() string {
	return "TerrainData"
}

// String (generated function)
func (m *ArdupilotmegaTerrainData) String() string {
	return fmt.Sprintf("ArdupilotmegaTerrainData{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaTerrainData) Pack(p *Packet) error {
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
func (m *ArdupilotmegaTerrainData) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 43 {
		return errPayloadTooSmall
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

// ArdupilotmegaTerrainCheck struct (generated typeinfo)
// Request that the vehicle report terrain height at the given location (expected response is a TERRAIN_REPORT). Used by GCS to check if vehicle has all terrain data needed for a mission.
type ArdupilotmegaTerrainCheck struct {
	Lat int32 // Latitude
	Lon int32 // Longitude
}

// Dialect (generated function)
func (m *ArdupilotmegaTerrainCheck) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaTerrainCheck) MsgID() MessageID {
	return MSG_ID_TERRAIN_CHECK
}

// CRCExtra (generated function)
func (m *ArdupilotmegaTerrainCheck) CRCExtra() uint8 {
	return 203
}

// MsgName (generated function)
func (m *ArdupilotmegaTerrainCheck) MsgName() string {
	return "TerrainCheck"
}

// String (generated function)
func (m *ArdupilotmegaTerrainCheck) String() string {
	return fmt.Sprintf("ArdupilotmegaTerrainCheck{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaTerrainCheck) Pack(p *Packet) error {
	payload := make([]byte, 8)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.Lat))
	binary.LittleEndian.PutUint32(payload[4:], uint32(m.Lon))
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ArdupilotmegaTerrainCheck) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 8 {
		return errPayloadTooSmall
	}
	m.Lat = int32(binary.LittleEndian.Uint32(payload[0:]))
	m.Lon = int32(binary.LittleEndian.Uint32(payload[4:]))
	return nil
}

// ArdupilotmegaTerrainReport struct (generated typeinfo)
// Streamed from drone to report progress of terrain map download (initiated by TERRAIN_REQUEST), or sent as a response to a TERRAIN_CHECK request. See terrain protocol docs: https://mavlink.io/en/services/terrain.html
type ArdupilotmegaTerrainReport struct {
	Lat           int32   // Latitude
	Lon           int32   // Longitude
	TerrainHeight float32 // Terrain height MSL
	CurrentHeight float32 // Current vehicle height above lat/lon terrain height
	Spacing       uint16  // grid spacing (zero if terrain at this location unavailable)
	Pending       uint16  // Number of 4x4 terrain blocks waiting to be received or read from disk
	Loaded        uint16  // Number of 4x4 terrain blocks in memory
}

// Dialect (generated function)
func (m *ArdupilotmegaTerrainReport) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaTerrainReport) MsgID() MessageID {
	return MSG_ID_TERRAIN_REPORT
}

// CRCExtra (generated function)
func (m *ArdupilotmegaTerrainReport) CRCExtra() uint8 {
	return 1
}

// MsgName (generated function)
func (m *ArdupilotmegaTerrainReport) MsgName() string {
	return "TerrainReport"
}

// String (generated function)
func (m *ArdupilotmegaTerrainReport) String() string {
	return fmt.Sprintf("ArdupilotmegaTerrainReport{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaTerrainReport) Pack(p *Packet) error {
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
func (m *ArdupilotmegaTerrainReport) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 22 {
		return errPayloadTooSmall
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

// ArdupilotmegaScaledPressure2 struct (generated typeinfo)
// Barometer readings for 2nd barometer
type ArdupilotmegaScaledPressure2 struct {
	TimeBootMs  uint32  // Timestamp (time since system boot).
	PressAbs    float32 // Absolute pressure
	PressDiff   float32 // Differential pressure
	Temperature int16   // Absolute pressure temperature
}

// Dialect (generated function)
func (m *ArdupilotmegaScaledPressure2) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaScaledPressure2) MsgID() MessageID {
	return MSG_ID_SCALED_PRESSURE2
}

// CRCExtra (generated function)
func (m *ArdupilotmegaScaledPressure2) CRCExtra() uint8 {
	return 195
}

// MsgName (generated function)
func (m *ArdupilotmegaScaledPressure2) MsgName() string {
	return "ScaledPressure2"
}

// String (generated function)
func (m *ArdupilotmegaScaledPressure2) String() string {
	return fmt.Sprintf("ArdupilotmegaScaledPressure2{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaScaledPressure2) Pack(p *Packet) error {
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
func (m *ArdupilotmegaScaledPressure2) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 14 {
		return errPayloadTooSmall
	}
	m.TimeBootMs = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.PressAbs = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	m.PressDiff = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.Temperature = int16(binary.LittleEndian.Uint16(payload[12:]))
	return nil
}

// ArdupilotmegaAttPosMocap struct (generated typeinfo)
// Motion capture attitude and position
type ArdupilotmegaAttPosMocap struct {
	TimeUsec uint64     // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
	Q        [4]float32 // Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
	X        float32    // X position (NED)
	Y        float32    // Y position (NED)
	Z        float32    // Z position (NED)
}

// Dialect (generated function)
func (m *ArdupilotmegaAttPosMocap) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaAttPosMocap) MsgID() MessageID {
	return MSG_ID_ATT_POS_MOCAP
}

// CRCExtra (generated function)
func (m *ArdupilotmegaAttPosMocap) CRCExtra() uint8 {
	return 109
}

// MsgName (generated function)
func (m *ArdupilotmegaAttPosMocap) MsgName() string {
	return "AttPosMocap"
}

// String (generated function)
func (m *ArdupilotmegaAttPosMocap) String() string {
	return fmt.Sprintf("ArdupilotmegaAttPosMocap{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaAttPosMocap) Pack(p *Packet) error {
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
func (m *ArdupilotmegaAttPosMocap) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 36 {
		return errPayloadTooSmall
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

// ArdupilotmegaSetActuatorControlTarget struct (generated typeinfo)
// Set the vehicle attitude and body angular rates.
type ArdupilotmegaSetActuatorControlTarget struct {
	TimeUsec        uint64     // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
	Controls        [8]float32 // Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0): (index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through mixer to repurpose them as generic outputs.
	GroupMlx        uint8      // Actuator group. The "_mlx" indicates this is a multi-instance message and a MAVLink parser should use this field to difference between instances.
	TargetSystem    uint8      // System ID
	TargetComponent uint8      // Component ID
}

// Dialect (generated function)
func (m *ArdupilotmegaSetActuatorControlTarget) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaSetActuatorControlTarget) MsgID() MessageID {
	return MSG_ID_SET_ACTUATOR_CONTROL_TARGET
}

// CRCExtra (generated function)
func (m *ArdupilotmegaSetActuatorControlTarget) CRCExtra() uint8 {
	return 168
}

// MsgName (generated function)
func (m *ArdupilotmegaSetActuatorControlTarget) MsgName() string {
	return "SetActuatorControlTarget"
}

// String (generated function)
func (m *ArdupilotmegaSetActuatorControlTarget) String() string {
	return fmt.Sprintf("ArdupilotmegaSetActuatorControlTarget{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaSetActuatorControlTarget) Pack(p *Packet) error {
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
func (m *ArdupilotmegaSetActuatorControlTarget) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 43 {
		return errPayloadTooSmall
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

// ArdupilotmegaActuatorControlTarget struct (generated typeinfo)
// Set the vehicle attitude and body angular rates.
type ArdupilotmegaActuatorControlTarget struct {
	TimeUsec uint64     // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
	Controls [8]float32 // Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0): (index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through mixer to repurpose them as generic outputs.
	GroupMlx uint8      // Actuator group. The "_mlx" indicates this is a multi-instance message and a MAVLink parser should use this field to difference between instances.
}

// Dialect (generated function)
func (m *ArdupilotmegaActuatorControlTarget) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaActuatorControlTarget) MsgID() MessageID {
	return MSG_ID_ACTUATOR_CONTROL_TARGET
}

// CRCExtra (generated function)
func (m *ArdupilotmegaActuatorControlTarget) CRCExtra() uint8 {
	return 181
}

// MsgName (generated function)
func (m *ArdupilotmegaActuatorControlTarget) MsgName() string {
	return "ActuatorControlTarget"
}

// String (generated function)
func (m *ArdupilotmegaActuatorControlTarget) String() string {
	return fmt.Sprintf("ArdupilotmegaActuatorControlTarget{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaActuatorControlTarget) Pack(p *Packet) error {
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
func (m *ArdupilotmegaActuatorControlTarget) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 41 {
		return errPayloadTooSmall
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(payload[0:]))
	for i := 0; i < len(m.Controls); i++ {
		m.Controls[i] = math.Float32frombits(binary.LittleEndian.Uint32(payload[8+i*4:]))
	}
	m.GroupMlx = uint8(payload[40])
	return nil
}

// ArdupilotmegaAltitude struct (generated typeinfo)
// The current system altitude.
type ArdupilotmegaAltitude struct {
	TimeUsec          uint64  // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
	AltitudeMonotonic float32 // This altitude measure is initialized on system boot and monotonic (it is never reset, but represents the local altitude change). The only guarantee on this field is that it will never be reset and is consistent within a flight. The recommended value for this field is the uncorrected barometric altitude at boot time. This altitude will also drift and vary between flights.
	AltitudeAmsl      float32 // This altitude measure is strictly above mean sea level and might be non-monotonic (it might reset on events like GPS lock or when a new QNH value is set). It should be the altitude to which global altitude waypoints are compared to. Note that it is *not* the GPS altitude, however, most GPS modules already output MSL by default and not the WGS84 altitude.
	AltitudeLocal     float32 // This is the local altitude in the local coordinate frame. It is not the altitude above home, but in reference to the coordinate origin (0, 0, 0). It is up-positive.
	AltitudeRelative  float32 // This is the altitude above the home position. It resets on each change of the current home position.
	AltitudeTerrain   float32 // This is the altitude above terrain. It might be fed by a terrain database or an altimeter. Values smaller than -1000 should be interpreted as unknown.
	BottomClearance   float32 // This is not the altitude, but the clear space below the system according to the fused clearance estimate. It generally should max out at the maximum range of e.g. the laser altimeter. It is generally a moving target. A negative value indicates no measurement available.
}

// Dialect (generated function)
func (m *ArdupilotmegaAltitude) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaAltitude) MsgID() MessageID {
	return MSG_ID_ALTITUDE
}

// CRCExtra (generated function)
func (m *ArdupilotmegaAltitude) CRCExtra() uint8 {
	return 47
}

// MsgName (generated function)
func (m *ArdupilotmegaAltitude) MsgName() string {
	return "Altitude"
}

// String (generated function)
func (m *ArdupilotmegaAltitude) String() string {
	return fmt.Sprintf("ArdupilotmegaAltitude{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaAltitude) Pack(p *Packet) error {
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
func (m *ArdupilotmegaAltitude) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 32 {
		return errPayloadTooSmall
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

// ArdupilotmegaResourceRequest struct (generated typeinfo)
// The autopilot is requesting a resource (file, binary, other type of data)
type ArdupilotmegaResourceRequest struct {
	RequestID    uint8      // Request ID. This ID should be re-used when sending back URI contents
	URIType      uint8      // The type of requested URI. 0 = a file via URL. 1 = a UAVCAN binary
	URI          [120]uint8 // The requested unique resource identifier (URI). It is not necessarily a straight domain name (depends on the URI type enum)
	TransferType uint8      // The way the autopilot wants to receive the URI. 0 = MAVLink FTP. 1 = binary stream.
	Storage      [120]uint8 // The storage path the autopilot wants the URI to be stored in. Will only be valid if the transfer_type has a storage associated (e.g. MAVLink FTP).
}

// Dialect (generated function)
func (m *ArdupilotmegaResourceRequest) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaResourceRequest) MsgID() MessageID {
	return MSG_ID_RESOURCE_REQUEST
}

// CRCExtra (generated function)
func (m *ArdupilotmegaResourceRequest) CRCExtra() uint8 {
	return 72
}

// MsgName (generated function)
func (m *ArdupilotmegaResourceRequest) MsgName() string {
	return "ResourceRequest"
}

// String (generated function)
func (m *ArdupilotmegaResourceRequest) String() string {
	return fmt.Sprintf("ArdupilotmegaResourceRequest{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaResourceRequest) Pack(p *Packet) error {
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
func (m *ArdupilotmegaResourceRequest) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 243 {
		return errPayloadTooSmall
	}
	m.RequestID = uint8(payload[0])
	m.URIType = uint8(payload[1])
	copy(m.URI[:], payload[2:122])
	m.TransferType = uint8(payload[122])
	copy(m.Storage[:], payload[123:243])
	return nil
}

// ArdupilotmegaScaledPressure3 struct (generated typeinfo)
// Barometer readings for 3rd barometer
type ArdupilotmegaScaledPressure3 struct {
	TimeBootMs  uint32  // Timestamp (time since system boot).
	PressAbs    float32 // Absolute pressure
	PressDiff   float32 // Differential pressure
	Temperature int16   // Absolute pressure temperature
}

// Dialect (generated function)
func (m *ArdupilotmegaScaledPressure3) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaScaledPressure3) MsgID() MessageID {
	return MSG_ID_SCALED_PRESSURE3
}

// CRCExtra (generated function)
func (m *ArdupilotmegaScaledPressure3) CRCExtra() uint8 {
	return 131
}

// MsgName (generated function)
func (m *ArdupilotmegaScaledPressure3) MsgName() string {
	return "ScaledPressure3"
}

// String (generated function)
func (m *ArdupilotmegaScaledPressure3) String() string {
	return fmt.Sprintf("ArdupilotmegaScaledPressure3{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaScaledPressure3) Pack(p *Packet) error {
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
func (m *ArdupilotmegaScaledPressure3) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 14 {
		return errPayloadTooSmall
	}
	m.TimeBootMs = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.PressAbs = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	m.PressDiff = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.Temperature = int16(binary.LittleEndian.Uint16(payload[12:]))
	return nil
}

// ArdupilotmegaFollowTarget struct (generated typeinfo)
// Current motion information from a designated system
type ArdupilotmegaFollowTarget struct {
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

// Dialect (generated function)
func (m *ArdupilotmegaFollowTarget) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaFollowTarget) MsgID() MessageID {
	return MSG_ID_FOLLOW_TARGET
}

// CRCExtra (generated function)
func (m *ArdupilotmegaFollowTarget) CRCExtra() uint8 {
	return 127
}

// MsgName (generated function)
func (m *ArdupilotmegaFollowTarget) MsgName() string {
	return "FollowTarget"
}

// String (generated function)
func (m *ArdupilotmegaFollowTarget) String() string {
	return fmt.Sprintf("ArdupilotmegaFollowTarget{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaFollowTarget) Pack(p *Packet) error {
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
func (m *ArdupilotmegaFollowTarget) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 93 {
		return errPayloadTooSmall
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

// ArdupilotmegaControlSystemState struct (generated typeinfo)
// The smoothed, monotonic system state used to feed the control loops of the system.
type ArdupilotmegaControlSystemState struct {
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

// Dialect (generated function)
func (m *ArdupilotmegaControlSystemState) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaControlSystemState) MsgID() MessageID {
	return MSG_ID_CONTROL_SYSTEM_STATE
}

// CRCExtra (generated function)
func (m *ArdupilotmegaControlSystemState) CRCExtra() uint8 {
	return 103
}

// MsgName (generated function)
func (m *ArdupilotmegaControlSystemState) MsgName() string {
	return "ControlSystemState"
}

// String (generated function)
func (m *ArdupilotmegaControlSystemState) String() string {
	return fmt.Sprintf("ArdupilotmegaControlSystemState{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaControlSystemState) Pack(p *Packet) error {
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
func (m *ArdupilotmegaControlSystemState) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 100 {
		return errPayloadTooSmall
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

// ArdupilotmegaBatteryStatus struct (generated typeinfo)
// Battery information. Updates GCS with flight controller battery status. Smart batteries also use this message, but may additionally send SMART_BATTERY_INFO.
type ArdupilotmegaBatteryStatus struct {
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

// Dialect (generated function)
func (m *ArdupilotmegaBatteryStatus) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaBatteryStatus) MsgID() MessageID {
	return MSG_ID_BATTERY_STATUS
}

// CRCExtra (generated function)
func (m *ArdupilotmegaBatteryStatus) CRCExtra() uint8 {
	return 154
}

// MsgName (generated function)
func (m *ArdupilotmegaBatteryStatus) MsgName() string {
	return "BatteryStatus"
}

// String (generated function)
func (m *ArdupilotmegaBatteryStatus) String() string {
	return fmt.Sprintf("ArdupilotmegaBatteryStatus{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaBatteryStatus) Pack(p *Packet) error {
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
func (m *ArdupilotmegaBatteryStatus) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 36 {
		return errPayloadTooSmall
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

// ArdupilotmegaAutopilotVersion struct (generated typeinfo)
// Version and capability of autopilot software. This should be emitted in response to a request with MAV_CMD_REQUEST_MESSAGE.
type ArdupilotmegaAutopilotVersion struct {
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

// Dialect (generated function)
func (m *ArdupilotmegaAutopilotVersion) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaAutopilotVersion) MsgID() MessageID {
	return MSG_ID_AUTOPILOT_VERSION
}

// CRCExtra (generated function)
func (m *ArdupilotmegaAutopilotVersion) CRCExtra() uint8 {
	return 178
}

// MsgName (generated function)
func (m *ArdupilotmegaAutopilotVersion) MsgName() string {
	return "AutopilotVersion"
}

// String (generated function)
func (m *ArdupilotmegaAutopilotVersion) String() string {
	return fmt.Sprintf("ArdupilotmegaAutopilotVersion{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaAutopilotVersion) Pack(p *Packet) error {
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
func (m *ArdupilotmegaAutopilotVersion) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 60 {
		return errPayloadTooSmall
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

// ArdupilotmegaLandingTarget struct (generated typeinfo)
// The location of a landing target. See: https://mavlink.io/en/services/landing_target.html
type ArdupilotmegaLandingTarget struct {
	TimeUsec  uint64  // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
	AngleX    float32 // X-axis angular offset of the target from the center of the image
	AngleY    float32 // Y-axis angular offset of the target from the center of the image
	Distance  float32 // Distance to the target from the vehicle
	SizeX     float32 // Size of target along x-axis
	SizeY     float32 // Size of target along y-axis
	TargetNum uint8   // The ID of the target if multiple targets are present
	Frame     uint8   // Coordinate frame used for following fields.
}

// Dialect (generated function)
func (m *ArdupilotmegaLandingTarget) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaLandingTarget) MsgID() MessageID {
	return MSG_ID_LANDING_TARGET
}

// CRCExtra (generated function)
func (m *ArdupilotmegaLandingTarget) CRCExtra() uint8 {
	return 200
}

// MsgName (generated function)
func (m *ArdupilotmegaLandingTarget) MsgName() string {
	return "LandingTarget"
}

// String (generated function)
func (m *ArdupilotmegaLandingTarget) String() string {
	return fmt.Sprintf("ArdupilotmegaLandingTarget{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaLandingTarget) Pack(p *Packet) error {
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
func (m *ArdupilotmegaLandingTarget) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 30 {
		return errPayloadTooSmall
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

// ArdupilotmegaFenceStatus struct (generated typeinfo)
// Status of geo-fencing. Sent in extended status stream when fencing enabled.
type ArdupilotmegaFenceStatus struct {
	BreachTime   uint32 // Time (since boot) of last breach.
	BreachCount  uint16 // Number of fence breaches.
	BreachStatus uint8  // Breach status (0 if currently inside fence, 1 if outside).
	BreachType   uint8  // Last breach type.
}

// Dialect (generated function)
func (m *ArdupilotmegaFenceStatus) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaFenceStatus) MsgID() MessageID {
	return MSG_ID_FENCE_STATUS
}

// CRCExtra (generated function)
func (m *ArdupilotmegaFenceStatus) CRCExtra() uint8 {
	return 189
}

// MsgName (generated function)
func (m *ArdupilotmegaFenceStatus) MsgName() string {
	return "FenceStatus"
}

// String (generated function)
func (m *ArdupilotmegaFenceStatus) String() string {
	return fmt.Sprintf("ArdupilotmegaFenceStatus{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaFenceStatus) Pack(p *Packet) error {
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
func (m *ArdupilotmegaFenceStatus) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 8 {
		return errPayloadTooSmall
	}
	m.BreachTime = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.BreachCount = uint16(binary.LittleEndian.Uint16(payload[4:]))
	m.BreachStatus = uint8(payload[6])
	m.BreachType = uint8(payload[7])
	return nil
}

// ArdupilotmegaMagCalReport struct (generated typeinfo)
// Reports results of completed compass calibration. Sent until MAG_CAL_ACK received.
type ArdupilotmegaMagCalReport struct {
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

// Dialect (generated function)
func (m *ArdupilotmegaMagCalReport) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaMagCalReport) MsgID() MessageID {
	return MSG_ID_MAG_CAL_REPORT
}

// CRCExtra (generated function)
func (m *ArdupilotmegaMagCalReport) CRCExtra() uint8 {
	return 36
}

// MsgName (generated function)
func (m *ArdupilotmegaMagCalReport) MsgName() string {
	return "MagCalReport"
}

// String (generated function)
func (m *ArdupilotmegaMagCalReport) String() string {
	return fmt.Sprintf("ArdupilotmegaMagCalReport{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaMagCalReport) Pack(p *Packet) error {
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
func (m *ArdupilotmegaMagCalReport) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 44 {
		return errPayloadTooSmall
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

// ArdupilotmegaEfiStatus struct (generated typeinfo)
// EFI status output
type ArdupilotmegaEfiStatus struct {
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

// Dialect (generated function)
func (m *ArdupilotmegaEfiStatus) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaEfiStatus) MsgID() MessageID {
	return MSG_ID_EFI_STATUS
}

// CRCExtra (generated function)
func (m *ArdupilotmegaEfiStatus) CRCExtra() uint8 {
	return 208
}

// MsgName (generated function)
func (m *ArdupilotmegaEfiStatus) MsgName() string {
	return "EfiStatus"
}

// String (generated function)
func (m *ArdupilotmegaEfiStatus) String() string {
	return fmt.Sprintf("ArdupilotmegaEfiStatus{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaEfiStatus) Pack(p *Packet) error {
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
func (m *ArdupilotmegaEfiStatus) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 65 {
		return errPayloadTooSmall
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

// ArdupilotmegaEstimatorStatus struct (generated typeinfo)
// Estimator status message including flags, innovation test ratios and estimated accuracies. The flags message is an integer bitmask containing information on which EKF outputs are valid. See the ESTIMATOR_STATUS_FLAGS enum definition for further information. The innovation test ratios show the magnitude of the sensor innovation divided by the innovation check threshold. Under normal operation the innovation test ratios should be below 0.5 with occasional values up to 1.0. Values greater than 1.0 should be rare under normal operation and indicate that a measurement has been rejected by the filter. The user should be notified if an innovation test ratio greater than 1.0 is recorded. Notifications for values in the range between 0.5 and 1.0 should be optional and controllable by the user.
type ArdupilotmegaEstimatorStatus struct {
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

// Dialect (generated function)
func (m *ArdupilotmegaEstimatorStatus) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaEstimatorStatus) MsgID() MessageID {
	return MSG_ID_ESTIMATOR_STATUS
}

// CRCExtra (generated function)
func (m *ArdupilotmegaEstimatorStatus) CRCExtra() uint8 {
	return 163
}

// MsgName (generated function)
func (m *ArdupilotmegaEstimatorStatus) MsgName() string {
	return "EstimatorStatus"
}

// String (generated function)
func (m *ArdupilotmegaEstimatorStatus) String() string {
	return fmt.Sprintf("ArdupilotmegaEstimatorStatus{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaEstimatorStatus) Pack(p *Packet) error {
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
func (m *ArdupilotmegaEstimatorStatus) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 42 {
		return errPayloadTooSmall
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

// ArdupilotmegaWindCov struct (generated typeinfo)
// Wind covariance estimate from vehicle.
type ArdupilotmegaWindCov struct {
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

// Dialect (generated function)
func (m *ArdupilotmegaWindCov) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaWindCov) MsgID() MessageID {
	return MSG_ID_WIND_COV
}

// CRCExtra (generated function)
func (m *ArdupilotmegaWindCov) CRCExtra() uint8 {
	return 105
}

// MsgName (generated function)
func (m *ArdupilotmegaWindCov) MsgName() string {
	return "WindCov"
}

// String (generated function)
func (m *ArdupilotmegaWindCov) String() string {
	return fmt.Sprintf("ArdupilotmegaWindCov{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaWindCov) Pack(p *Packet) error {
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
func (m *ArdupilotmegaWindCov) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 40 {
		return errPayloadTooSmall
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

// ArdupilotmegaGpsInput struct (generated typeinfo)
// GPS sensor input message.  This is a raw sensor value sent by the GPS. This is NOT the global position estimate of the system.
type ArdupilotmegaGpsInput struct {
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

// Dialect (generated function)
func (m *ArdupilotmegaGpsInput) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaGpsInput) MsgID() MessageID {
	return MSG_ID_GPS_INPUT
}

// CRCExtra (generated function)
func (m *ArdupilotmegaGpsInput) CRCExtra() uint8 {
	return 151
}

// MsgName (generated function)
func (m *ArdupilotmegaGpsInput) MsgName() string {
	return "GpsInput"
}

// String (generated function)
func (m *ArdupilotmegaGpsInput) String() string {
	return fmt.Sprintf("ArdupilotmegaGpsInput{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaGpsInput) Pack(p *Packet) error {
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
func (m *ArdupilotmegaGpsInput) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 63 {
		return errPayloadTooSmall
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

// ArdupilotmegaGpsRtcmData struct (generated typeinfo)
// RTCM message for injecting into the onboard GPS (used for DGPS)
type ArdupilotmegaGpsRtcmData struct {
	Flags uint8      // LSB: 1 means message is fragmented, next 2 bits are the fragment ID, the remaining 5 bits are used for the sequence ID. Messages are only to be flushed to the GPS when the entire message has been reconstructed on the autopilot. The fragment ID specifies which order the fragments should be assembled into a buffer, while the sequence ID is used to detect a mismatch between different buffers. The buffer is considered fully reconstructed when either all 4 fragments are present, or all the fragments before the first fragment with a non full payload is received. This management is used to ensure that normal GPS operation doesn't corrupt RTCM data, and to recover from a unreliable transport delivery order.
	Len   uint8      // data length
	Data  [180]uint8 // RTCM message (may be fragmented)
}

// Dialect (generated function)
func (m *ArdupilotmegaGpsRtcmData) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaGpsRtcmData) MsgID() MessageID {
	return MSG_ID_GPS_RTCM_DATA
}

// CRCExtra (generated function)
func (m *ArdupilotmegaGpsRtcmData) CRCExtra() uint8 {
	return 35
}

// MsgName (generated function)
func (m *ArdupilotmegaGpsRtcmData) MsgName() string {
	return "GpsRtcmData"
}

// String (generated function)
func (m *ArdupilotmegaGpsRtcmData) String() string {
	return fmt.Sprintf("ArdupilotmegaGpsRtcmData{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaGpsRtcmData) Pack(p *Packet) error {
	payload := make([]byte, 182)
	payload[0] = byte(m.Flags)
	payload[1] = byte(m.Len)
	copy(payload[2:], m.Data[:])
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ArdupilotmegaGpsRtcmData) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 182 {
		return errPayloadTooSmall
	}
	m.Flags = uint8(payload[0])
	m.Len = uint8(payload[1])
	copy(m.Data[:], payload[2:182])
	return nil
}

// ArdupilotmegaHighLatency struct (generated typeinfo)
// Message appropriate for high latency connections like Iridium
type ArdupilotmegaHighLatency struct {
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

// Dialect (generated function)
func (m *ArdupilotmegaHighLatency) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaHighLatency) MsgID() MessageID {
	return MSG_ID_HIGH_LATENCY
}

// CRCExtra (generated function)
func (m *ArdupilotmegaHighLatency) CRCExtra() uint8 {
	return 150
}

// MsgName (generated function)
func (m *ArdupilotmegaHighLatency) MsgName() string {
	return "HighLatency"
}

// String (generated function)
func (m *ArdupilotmegaHighLatency) String() string {
	return fmt.Sprintf("ArdupilotmegaHighLatency{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaHighLatency) Pack(p *Packet) error {
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
func (m *ArdupilotmegaHighLatency) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 40 {
		return errPayloadTooSmall
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

// ArdupilotmegaHighLatency2 struct (generated typeinfo)
// Message appropriate for high latency connections like Iridium (version 2)
type ArdupilotmegaHighLatency2 struct {
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

// Dialect (generated function)
func (m *ArdupilotmegaHighLatency2) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaHighLatency2) MsgID() MessageID {
	return MSG_ID_HIGH_LATENCY2
}

// CRCExtra (generated function)
func (m *ArdupilotmegaHighLatency2) CRCExtra() uint8 {
	return 179
}

// MsgName (generated function)
func (m *ArdupilotmegaHighLatency2) MsgName() string {
	return "HighLatency2"
}

// String (generated function)
func (m *ArdupilotmegaHighLatency2) String() string {
	return fmt.Sprintf("ArdupilotmegaHighLatency2{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaHighLatency2) Pack(p *Packet) error {
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
func (m *ArdupilotmegaHighLatency2) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 42 {
		return errPayloadTooSmall
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

// ArdupilotmegaVibration struct (generated typeinfo)
// Vibration levels and accelerometer clipping
type ArdupilotmegaVibration struct {
	TimeUsec   uint64  // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
	VibrationX float32 // Vibration levels on X-axis
	VibrationY float32 // Vibration levels on Y-axis
	VibrationZ float32 // Vibration levels on Z-axis
	Clipping0  uint32  // first accelerometer clipping count
	Clipping1  uint32  // second accelerometer clipping count
	Clipping2  uint32  // third accelerometer clipping count
}

// Dialect (generated function)
func (m *ArdupilotmegaVibration) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaVibration) MsgID() MessageID {
	return MSG_ID_VIBRATION
}

// CRCExtra (generated function)
func (m *ArdupilotmegaVibration) CRCExtra() uint8 {
	return 90
}

// MsgName (generated function)
func (m *ArdupilotmegaVibration) MsgName() string {
	return "Vibration"
}

// String (generated function)
func (m *ArdupilotmegaVibration) String() string {
	return fmt.Sprintf("ArdupilotmegaVibration{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaVibration) Pack(p *Packet) error {
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
func (m *ArdupilotmegaVibration) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 32 {
		return errPayloadTooSmall
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

// ArdupilotmegaHomePosition struct (generated typeinfo)
// This message can be requested by sending the MAV_CMD_GET_HOME_POSITION command. The position the system will return to and land on. The position is set automatically by the system during the takeoff in case it was not explicitly set by the operator before or after. The global and local positions encode the position in the respective coordinate frames, while the q parameter encodes the orientation of the surface. Under normal conditions it describes the heading and terrain slope, which can be used by the aircraft to adjust the approach. The approach 3D vector describes the point to which the system should fly in normal flight mode and then perform a landing sequence along the vector.
type ArdupilotmegaHomePosition struct {
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

// Dialect (generated function)
func (m *ArdupilotmegaHomePosition) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaHomePosition) MsgID() MessageID {
	return MSG_ID_HOME_POSITION
}

// CRCExtra (generated function)
func (m *ArdupilotmegaHomePosition) CRCExtra() uint8 {
	return 104
}

// MsgName (generated function)
func (m *ArdupilotmegaHomePosition) MsgName() string {
	return "HomePosition"
}

// String (generated function)
func (m *ArdupilotmegaHomePosition) String() string {
	return fmt.Sprintf("ArdupilotmegaHomePosition{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaHomePosition) Pack(p *Packet) error {
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
func (m *ArdupilotmegaHomePosition) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 52 {
		return errPayloadTooSmall
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

// ArdupilotmegaSetHomePosition struct (generated typeinfo)
// The position the system will return to and land on. The position is set automatically by the system during the takeoff in case it was not explicitly set by the operator before or after. The global and local positions encode the position in the respective coordinate frames, while the q parameter encodes the orientation of the surface. Under normal conditions it describes the heading and terrain slope, which can be used by the aircraft to adjust the approach. The approach 3D vector describes the point to which the system should fly in normal flight mode and then perform a landing sequence along the vector.
type ArdupilotmegaSetHomePosition struct {
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

// Dialect (generated function)
func (m *ArdupilotmegaSetHomePosition) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaSetHomePosition) MsgID() MessageID {
	return MSG_ID_SET_HOME_POSITION
}

// CRCExtra (generated function)
func (m *ArdupilotmegaSetHomePosition) CRCExtra() uint8 {
	return 85
}

// MsgName (generated function)
func (m *ArdupilotmegaSetHomePosition) MsgName() string {
	return "SetHomePosition"
}

// String (generated function)
func (m *ArdupilotmegaSetHomePosition) String() string {
	return fmt.Sprintf("ArdupilotmegaSetHomePosition{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaSetHomePosition) Pack(p *Packet) error {
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
func (m *ArdupilotmegaSetHomePosition) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 53 {
		return errPayloadTooSmall
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

// ArdupilotmegaMessageInterval struct (generated typeinfo)
// The interval between messages for a particular MAVLink message ID. This message is the response to the MAV_CMD_GET_MESSAGE_INTERVAL command. This interface replaces DATA_STREAM.
type ArdupilotmegaMessageInterval struct {
	IntervalUs int32  // The interval between two messages. A value of -1 indicates this stream is disabled, 0 indicates it is not available, &gt; 0 indicates the interval at which it is sent.
	MessageID  uint16 // The ID of the requested MAVLink message. v1.0 is limited to 254 messages.
}

// Dialect (generated function)
func (m *ArdupilotmegaMessageInterval) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaMessageInterval) MsgID() MessageID {
	return MSG_ID_MESSAGE_INTERVAL
}

// CRCExtra (generated function)
func (m *ArdupilotmegaMessageInterval) CRCExtra() uint8 {
	return 95
}

// MsgName (generated function)
func (m *ArdupilotmegaMessageInterval) MsgName() string {
	return "MessageInterval"
}

// String (generated function)
func (m *ArdupilotmegaMessageInterval) String() string {
	return fmt.Sprintf("ArdupilotmegaMessageInterval{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaMessageInterval) Pack(p *Packet) error {
	payload := make([]byte, 6)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.IntervalUs))
	binary.LittleEndian.PutUint16(payload[4:], uint16(m.MessageID))
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ArdupilotmegaMessageInterval) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 6 {
		return errPayloadTooSmall
	}
	m.IntervalUs = int32(binary.LittleEndian.Uint32(payload[0:]))
	m.MessageID = uint16(binary.LittleEndian.Uint16(payload[4:]))
	return nil
}

// ArdupilotmegaExtendedSysState struct (generated typeinfo)
// Provides state for additional features
type ArdupilotmegaExtendedSysState struct {
	VtolState   uint8 // The VTOL state if applicable. Is set to MAV_VTOL_STATE_UNDEFINED if UAV is not in VTOL configuration.
	LandedState uint8 // The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.
}

// Dialect (generated function)
func (m *ArdupilotmegaExtendedSysState) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaExtendedSysState) MsgID() MessageID {
	return MSG_ID_EXTENDED_SYS_STATE
}

// CRCExtra (generated function)
func (m *ArdupilotmegaExtendedSysState) CRCExtra() uint8 {
	return 130
}

// MsgName (generated function)
func (m *ArdupilotmegaExtendedSysState) MsgName() string {
	return "ExtendedSysState"
}

// String (generated function)
func (m *ArdupilotmegaExtendedSysState) String() string {
	return fmt.Sprintf("ArdupilotmegaExtendedSysState{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaExtendedSysState) Pack(p *Packet) error {
	payload := make([]byte, 2)
	payload[0] = byte(m.VtolState)
	payload[1] = byte(m.LandedState)
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ArdupilotmegaExtendedSysState) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 2 {
		return errPayloadTooSmall
	}
	m.VtolState = uint8(payload[0])
	m.LandedState = uint8(payload[1])
	return nil
}

// ArdupilotmegaAdsbVehicle struct (generated typeinfo)
// The location and information of an ADSB vehicle
type ArdupilotmegaAdsbVehicle struct {
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

// Dialect (generated function)
func (m *ArdupilotmegaAdsbVehicle) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaAdsbVehicle) MsgID() MessageID {
	return MSG_ID_ADSB_VEHICLE
}

// CRCExtra (generated function)
func (m *ArdupilotmegaAdsbVehicle) CRCExtra() uint8 {
	return 184
}

// MsgName (generated function)
func (m *ArdupilotmegaAdsbVehicle) MsgName() string {
	return "AdsbVehicle"
}

// String (generated function)
func (m *ArdupilotmegaAdsbVehicle) String() string {
	return fmt.Sprintf("ArdupilotmegaAdsbVehicle{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaAdsbVehicle) Pack(p *Packet) error {
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
func (m *ArdupilotmegaAdsbVehicle) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 38 {
		return errPayloadTooSmall
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

// ArdupilotmegaCollision struct (generated typeinfo)
// Information about a potential collision
type ArdupilotmegaCollision struct {
	ID                     uint32  // Unique identifier, domain based on src field
	TimeToMinimumDelta     float32 // Estimated time until collision occurs
	AltitudeMinimumDelta   float32 // Closest vertical distance between vehicle and object
	HorizontalMinimumDelta float32 // Closest horizontal distance between vehicle and object
	Src                    uint8   // Collision data source
	Action                 uint8   // Action that is being taken to avoid this collision
	ThreatLevel            uint8   // How concerned the aircraft is about this collision
}

// Dialect (generated function)
func (m *ArdupilotmegaCollision) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaCollision) MsgID() MessageID {
	return MSG_ID_COLLISION
}

// CRCExtra (generated function)
func (m *ArdupilotmegaCollision) CRCExtra() uint8 {
	return 81
}

// MsgName (generated function)
func (m *ArdupilotmegaCollision) MsgName() string {
	return "Collision"
}

// String (generated function)
func (m *ArdupilotmegaCollision) String() string {
	return fmt.Sprintf("ArdupilotmegaCollision{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaCollision) Pack(p *Packet) error {
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
func (m *ArdupilotmegaCollision) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 19 {
		return errPayloadTooSmall
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

// ArdupilotmegaV2Extension struct (generated typeinfo)
// Message implementing parts of the V2 payload specs in V1 frames for transitional support.
type ArdupilotmegaV2Extension struct {
	MessageType     uint16     // A code that identifies the software component that understands this message (analogous to USB device classes or mime type strings). If this code is less than 32768, it is considered a 'registered' protocol extension and the corresponding entry should be added to https://github.com/mavlink/mavlink/definition_files/extension_message_ids.xml. Software creators can register blocks of message IDs as needed (useful for GCS specific metadata, etc...). Message_types greater than 32767 are considered local experiments and should not be checked in to any widely distributed codebase.
	TargetNetwork   uint8      // Network ID (0 for broadcast)
	TargetSystem    uint8      // System ID (0 for broadcast)
	TargetComponent uint8      // Component ID (0 for broadcast)
	Payload         [249]uint8 // Variable length payload. The length must be encoded in the payload as part of the message_type protocol, e.g. by including the length as payload data, or by terminating the payload data with a non-zero marker. This is required in order to reconstruct zero-terminated payloads that are (or otherwise would be) trimmed by MAVLink 2 empty-byte truncation. The entire content of the payload block is opaque unless you understand the encoding message_type. The particular encoding used can be extension specific and might not always be documented as part of the MAVLink specification.
}

// Dialect (generated function)
func (m *ArdupilotmegaV2Extension) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaV2Extension) MsgID() MessageID {
	return MSG_ID_V2_EXTENSION
}

// CRCExtra (generated function)
func (m *ArdupilotmegaV2Extension) CRCExtra() uint8 {
	return 8
}

// MsgName (generated function)
func (m *ArdupilotmegaV2Extension) MsgName() string {
	return "V2Extension"
}

// String (generated function)
func (m *ArdupilotmegaV2Extension) String() string {
	return fmt.Sprintf("ArdupilotmegaV2Extension{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaV2Extension) Pack(p *Packet) error {
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
func (m *ArdupilotmegaV2Extension) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 254 {
		return errPayloadTooSmall
	}
	m.MessageType = uint16(binary.LittleEndian.Uint16(payload[0:]))
	m.TargetNetwork = uint8(payload[2])
	m.TargetSystem = uint8(payload[3])
	m.TargetComponent = uint8(payload[4])
	copy(m.Payload[:], payload[5:254])
	return nil
}

// ArdupilotmegaMemoryVect struct (generated typeinfo)
// Send raw controller memory. The use of this message is discouraged for normal packets, but a quite efficient way for testing new messages and getting experimental debug output.
type ArdupilotmegaMemoryVect struct {
	Address uint16   // Starting address of the debug variables
	Ver     uint8    // Version code of the type variable. 0=unknown, type ignored and assumed int16_t. 1=as below
	Type    uint8    // Type code of the memory variables. for ver = 1: 0=16 x int16_t, 1=16 x uint16_t, 2=16 x Q15, 3=16 x 1Q14
	Value   [32]int8 // Memory contents at specified address
}

// Dialect (generated function)
func (m *ArdupilotmegaMemoryVect) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaMemoryVect) MsgID() MessageID {
	return MSG_ID_MEMORY_VECT
}

// CRCExtra (generated function)
func (m *ArdupilotmegaMemoryVect) CRCExtra() uint8 {
	return 204
}

// MsgName (generated function)
func (m *ArdupilotmegaMemoryVect) MsgName() string {
	return "MemoryVect"
}

// String (generated function)
func (m *ArdupilotmegaMemoryVect) String() string {
	return fmt.Sprintf("ArdupilotmegaMemoryVect{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaMemoryVect) Pack(p *Packet) error {
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
func (m *ArdupilotmegaMemoryVect) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 36 {
		return errPayloadTooSmall
	}
	m.Address = uint16(binary.LittleEndian.Uint16(payload[0:]))
	m.Ver = uint8(payload[2])
	m.Type = uint8(payload[3])
	for i := 0; i < len(m.Value); i++ {
		m.Value[i] = int8(payload[4+i*1])
	}
	return nil
}

// ArdupilotmegaDebugVect struct (generated typeinfo)
// To debug something using a named 3D vector.
type ArdupilotmegaDebugVect struct {
	TimeUsec uint64   // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
	X        float32  // x
	Y        float32  // y
	Z        float32  // z
	Name     [10]byte // Name
}

// Dialect (generated function)
func (m *ArdupilotmegaDebugVect) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaDebugVect) MsgID() MessageID {
	return MSG_ID_DEBUG_VECT
}

// CRCExtra (generated function)
func (m *ArdupilotmegaDebugVect) CRCExtra() uint8 {
	return 49
}

// MsgName (generated function)
func (m *ArdupilotmegaDebugVect) MsgName() string {
	return "DebugVect"
}

// String (generated function)
func (m *ArdupilotmegaDebugVect) String() string {
	return fmt.Sprintf("ArdupilotmegaDebugVect{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaDebugVect) Pack(p *Packet) error {
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
func (m *ArdupilotmegaDebugVect) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 30 {
		return errPayloadTooSmall
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(payload[0:]))
	m.X = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.Y = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.Z = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	copy(m.Name[:], payload[20:30])
	return nil
}

// ArdupilotmegaNamedValueFloat struct (generated typeinfo)
// Send a key-value pair as float. The use of this message is discouraged for normal packets, but a quite efficient way for testing new messages and getting experimental debug output.
type ArdupilotmegaNamedValueFloat struct {
	TimeBootMs uint32   // Timestamp (time since system boot).
	Value      float32  // Floating point value
	Name       [10]byte // Name of the debug variable
}

// Dialect (generated function)
func (m *ArdupilotmegaNamedValueFloat) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaNamedValueFloat) MsgID() MessageID {
	return MSG_ID_NAMED_VALUE_FLOAT
}

// CRCExtra (generated function)
func (m *ArdupilotmegaNamedValueFloat) CRCExtra() uint8 {
	return 170
}

// MsgName (generated function)
func (m *ArdupilotmegaNamedValueFloat) MsgName() string {
	return "NamedValueFloat"
}

// String (generated function)
func (m *ArdupilotmegaNamedValueFloat) String() string {
	return fmt.Sprintf("ArdupilotmegaNamedValueFloat{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaNamedValueFloat) Pack(p *Packet) error {
	payload := make([]byte, 18)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.TimeBootMs))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(m.Value))
	copy(payload[8:], m.Name[:])
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ArdupilotmegaNamedValueFloat) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 18 {
		return errPayloadTooSmall
	}
	m.TimeBootMs = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.Value = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	copy(m.Name[:], payload[8:18])
	return nil
}

// ArdupilotmegaNamedValueInt struct (generated typeinfo)
// Send a key-value pair as integer. The use of this message is discouraged for normal packets, but a quite efficient way for testing new messages and getting experimental debug output.
type ArdupilotmegaNamedValueInt struct {
	TimeBootMs uint32   // Timestamp (time since system boot).
	Value      int32    // Signed integer value
	Name       [10]byte // Name of the debug variable
}

// Dialect (generated function)
func (m *ArdupilotmegaNamedValueInt) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaNamedValueInt) MsgID() MessageID {
	return MSG_ID_NAMED_VALUE_INT
}

// CRCExtra (generated function)
func (m *ArdupilotmegaNamedValueInt) CRCExtra() uint8 {
	return 44
}

// MsgName (generated function)
func (m *ArdupilotmegaNamedValueInt) MsgName() string {
	return "NamedValueInt"
}

// String (generated function)
func (m *ArdupilotmegaNamedValueInt) String() string {
	return fmt.Sprintf("ArdupilotmegaNamedValueInt{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaNamedValueInt) Pack(p *Packet) error {
	payload := make([]byte, 18)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.TimeBootMs))
	binary.LittleEndian.PutUint32(payload[4:], uint32(m.Value))
	copy(payload[8:], m.Name[:])
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ArdupilotmegaNamedValueInt) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 18 {
		return errPayloadTooSmall
	}
	m.TimeBootMs = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.Value = int32(binary.LittleEndian.Uint32(payload[4:]))
	copy(m.Name[:], payload[8:18])
	return nil
}

// ArdupilotmegaStatustext struct (generated typeinfo)
// Status text message. These messages are printed in yellow in the COMM console of QGroundControl. WARNING: They consume quite some bandwidth, so use only for important status and error messages. If implemented wisely, these messages are buffered on the MCU and sent only at a limited rate (e.g. 10 Hz).
type ArdupilotmegaStatustext struct {
	Severity uint8    // Severity of status. Relies on the definitions within RFC-5424.
	Text     [50]byte // Status text message, without null termination character
}

// Dialect (generated function)
func (m *ArdupilotmegaStatustext) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaStatustext) MsgID() MessageID {
	return MSG_ID_STATUSTEXT
}

// CRCExtra (generated function)
func (m *ArdupilotmegaStatustext) CRCExtra() uint8 {
	return 83
}

// MsgName (generated function)
func (m *ArdupilotmegaStatustext) MsgName() string {
	return "Statustext"
}

// String (generated function)
func (m *ArdupilotmegaStatustext) String() string {
	return fmt.Sprintf("ArdupilotmegaStatustext{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaStatustext) Pack(p *Packet) error {
	payload := make([]byte, 51)
	payload[0] = byte(m.Severity)
	copy(payload[1:], m.Text[:])
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ArdupilotmegaStatustext) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 51 {
		return errPayloadTooSmall
	}
	m.Severity = uint8(payload[0])
	copy(m.Text[:], payload[1:51])
	return nil
}

// ArdupilotmegaDebug struct (generated typeinfo)
// Send a debug value. The index is used to discriminate between values. These values show up in the plot of QGroundControl as DEBUG N.
type ArdupilotmegaDebug struct {
	TimeBootMs uint32  // Timestamp (time since system boot).
	Value      float32 // DEBUG value
	Ind        uint8   // index of debug variable
}

// Dialect (generated function)
func (m *ArdupilotmegaDebug) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaDebug) MsgID() MessageID {
	return MSG_ID_DEBUG
}

// CRCExtra (generated function)
func (m *ArdupilotmegaDebug) CRCExtra() uint8 {
	return 46
}

// MsgName (generated function)
func (m *ArdupilotmegaDebug) MsgName() string {
	return "Debug"
}

// String (generated function)
func (m *ArdupilotmegaDebug) String() string {
	return fmt.Sprintf("ArdupilotmegaDebug{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaDebug) Pack(p *Packet) error {
	payload := make([]byte, 9)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.TimeBootMs))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(m.Value))
	payload[8] = byte(m.Ind)
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *ArdupilotmegaDebug) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 9 {
		return errPayloadTooSmall
	}
	m.TimeBootMs = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.Value = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	m.Ind = uint8(payload[8])
	return nil
}

// ArdupilotmegaHeartbeat struct (generated typeinfo)
// The heartbeat message shows that a system or component is present and responding. The type and autopilot fields (along with the message component id), allow the receiving system to treat further messages from this system appropriately (e.g. by laying out the user interface based on the autopilot). This microservice is documented at https://mavlink.io/en/services/heartbeat.html
type ArdupilotmegaHeartbeat struct {
	CustomMode     uint32 // A bitfield for use for autopilot-specific flags
	Type           uint8  // Vehicle or component type. For a flight controller component the vehicle type (quadrotor, helicopter, etc.). For other components the component type (e.g. camera, gimbal, etc.). This should be used in preference to component id for identifying the component type.
	Autopilot      uint8  // Autopilot type / class. Use MAV_AUTOPILOT_INVALID for components that are not flight controllers.
	BaseMode       uint8  // System mode bitmap.
	SystemStatus   uint8  // System status flag.
	MavlinkVersion uint8  // MAVLink version, not writable by user, gets added by protocol because of magic data type: uint8_t_mavlink_version
}

// Dialect (generated function)
func (m *ArdupilotmegaHeartbeat) Dialect() *Dialect {
	return DialectArdupilotmega
}

// MsgID (generated function)
func (m *ArdupilotmegaHeartbeat) MsgID() MessageID {
	return MSG_ID_HEARTBEAT
}

// CRCExtra (generated function)
func (m *ArdupilotmegaHeartbeat) CRCExtra() uint8 {
	return 50
}

// MsgName (generated function)
func (m *ArdupilotmegaHeartbeat) MsgName() string {
	return "Heartbeat"
}

// String (generated function)
func (m *ArdupilotmegaHeartbeat) String() string {
	return fmt.Sprintf("ArdupilotmegaHeartbeat{&+v}", m)
}

// Pack (generated function)
func (m *ArdupilotmegaHeartbeat) Pack(p *Packet) error {
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
func (m *ArdupilotmegaHeartbeat) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 9 {
		return errPayloadTooSmall
	}
	m.CustomMode = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.Type = uint8(payload[4])
	m.Autopilot = uint8(payload[5])
	m.BaseMode = uint8(payload[6])
	m.SystemStatus = uint8(payload[7])
	m.MavlinkVersion = uint8(payload[8])
	return nil
}

// Message IDs
const (
	MSG_ID_SENSOR_OFFSETS                          MessageID = 150
	MSG_ID_SET_MAG_OFFSETS                         MessageID = 151
	MSG_ID_MEMINFO                                 MessageID = 152
	MSG_ID_AP_ADC                                  MessageID = 153
	MSG_ID_DIGICAM_CONFIGURE                       MessageID = 154
	MSG_ID_DIGICAM_CONTROL                         MessageID = 155
	MSG_ID_MOUNT_CONFIGURE                         MessageID = 156
	MSG_ID_MOUNT_CONTROL                           MessageID = 157
	MSG_ID_MOUNT_STATUS                            MessageID = 158
	MSG_ID_FENCE_POINT                             MessageID = 160
	MSG_ID_FENCE_FETCH_POINT                       MessageID = 161
	MSG_ID_AHRS                                    MessageID = 163
	MSG_ID_SIMSTATE                                MessageID = 164
	MSG_ID_HWSTATUS                                MessageID = 165
	MSG_ID_RADIO                                   MessageID = 166
	MSG_ID_LIMITS_STATUS                           MessageID = 167
	MSG_ID_WIND                                    MessageID = 168
	MSG_ID_DATA16                                  MessageID = 169
	MSG_ID_DATA32                                  MessageID = 170
	MSG_ID_DATA64                                  MessageID = 171
	MSG_ID_DATA96                                  MessageID = 172
	MSG_ID_RANGEFINDER                             MessageID = 173
	MSG_ID_AIRSPEED_AUTOCAL                        MessageID = 174
	MSG_ID_RALLY_POINT                             MessageID = 175
	MSG_ID_RALLY_FETCH_POINT                       MessageID = 176
	MSG_ID_COMPASSMOT_STATUS                       MessageID = 177
	MSG_ID_AHRS2                                   MessageID = 178
	MSG_ID_CAMERA_STATUS                           MessageID = 179
	MSG_ID_CAMERA_FEEDBACK                         MessageID = 180
	MSG_ID_BATTERY2                                MessageID = 181
	MSG_ID_AHRS3                                   MessageID = 182
	MSG_ID_AUTOPILOT_VERSION_REQUEST               MessageID = 183
	MSG_ID_REMOTE_LOG_DATA_BLOCK                   MessageID = 184
	MSG_ID_REMOTE_LOG_BLOCK_STATUS                 MessageID = 185
	MSG_ID_LED_CONTROL                             MessageID = 186
	MSG_ID_MAG_CAL_PROGRESS                        MessageID = 191
	MSG_ID_EKF_STATUS_REPORT                       MessageID = 193
	MSG_ID_PID_TUNING                              MessageID = 194
	MSG_ID_DEEPSTALL                               MessageID = 195
	MSG_ID_GIMBAL_REPORT                           MessageID = 200
	MSG_ID_GIMBAL_CONTROL                          MessageID = 201
	MSG_ID_GIMBAL_TORQUE_CMD_REPORT                MessageID = 214
	MSG_ID_GOPRO_HEARTBEAT                         MessageID = 215
	MSG_ID_GOPRO_GET_REQUEST                       MessageID = 216
	MSG_ID_GOPRO_GET_RESPONSE                      MessageID = 217
	MSG_ID_GOPRO_SET_REQUEST                       MessageID = 218
	MSG_ID_GOPRO_SET_RESPONSE                      MessageID = 219
	MSG_ID_RPM                                     MessageID = 226
	MSG_ID_SYS_STATUS                              MessageID = 1
	MSG_ID_SYSTEM_TIME                             MessageID = 2
	MSG_ID_PING                                    MessageID = 4
	MSG_ID_CHANGE_OPERATOR_CONTROL                 MessageID = 5
	MSG_ID_CHANGE_OPERATOR_CONTROL_ACK             MessageID = 6
	MSG_ID_AUTH_KEY                                MessageID = 7
	MSG_ID_LINK_NODE_STATUS                        MessageID = 8
	MSG_ID_SET_MODE                                MessageID = 11
	MSG_ID_PARAM_ACK_TRANSACTION                   MessageID = 19
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
	MSG_ID_MISSION_CHANGED                         MessageID = 52
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
	MSG_ID_COMMAND_CANCEL                          MessageID = 80
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
	MSG_ID_FENCE_STATUS                            MessageID = 162
	MSG_ID_MAG_CAL_REPORT                          MessageID = 192
	MSG_ID_EFI_STATUS                              MessageID = 225
	MSG_ID_ESTIMATOR_STATUS                        MessageID = 230
	MSG_ID_WIND_COV                                MessageID = 231
	MSG_ID_GPS_INPUT                               MessageID = 232
	MSG_ID_GPS_RTCM_DATA                           MessageID = 233
	MSG_ID_HIGH_LATENCY                            MessageID = 234
	MSG_ID_HIGH_LATENCY2                           MessageID = 235
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
	MSG_ID_HEARTBEAT                               MessageID = 0
)

// Mavlink message CRC extras
var MAVLINK_MESSAGE_CRC_EXTRAS = map[MessageID]uint8{
	MSG_ID_SENSOR_OFFSETS:                          134,
	MSG_ID_SET_MAG_OFFSETS:                         219,
	MSG_ID_MEMINFO:                                 208,
	MSG_ID_AP_ADC:                                  188,
	MSG_ID_DIGICAM_CONFIGURE:                       84,
	MSG_ID_DIGICAM_CONTROL:                         22,
	MSG_ID_MOUNT_CONFIGURE:                         19,
	MSG_ID_MOUNT_CONTROL:                           21,
	MSG_ID_MOUNT_STATUS:                            134,
	MSG_ID_FENCE_POINT:                             78,
	MSG_ID_FENCE_FETCH_POINT:                       68,
	MSG_ID_AHRS:                                    127,
	MSG_ID_SIMSTATE:                                154,
	MSG_ID_HWSTATUS:                                21,
	MSG_ID_RADIO:                                   21,
	MSG_ID_LIMITS_STATUS:                           144,
	MSG_ID_WIND:                                    1,
	MSG_ID_DATA16:                                  234,
	MSG_ID_DATA32:                                  73,
	MSG_ID_DATA64:                                  181,
	MSG_ID_DATA96:                                  22,
	MSG_ID_RANGEFINDER:                             83,
	MSG_ID_AIRSPEED_AUTOCAL:                        167,
	MSG_ID_RALLY_POINT:                             138,
	MSG_ID_RALLY_FETCH_POINT:                       234,
	MSG_ID_COMPASSMOT_STATUS:                       240,
	MSG_ID_AHRS2:                                   47,
	MSG_ID_CAMERA_STATUS:                           189,
	MSG_ID_CAMERA_FEEDBACK:                         52,
	MSG_ID_BATTERY2:                                174,
	MSG_ID_AHRS3:                                   229,
	MSG_ID_AUTOPILOT_VERSION_REQUEST:               85,
	MSG_ID_REMOTE_LOG_DATA_BLOCK:                   159,
	MSG_ID_REMOTE_LOG_BLOCK_STATUS:                 186,
	MSG_ID_LED_CONTROL:                             72,
	MSG_ID_MAG_CAL_PROGRESS:                        92,
	MSG_ID_EKF_STATUS_REPORT:                       71,
	MSG_ID_PID_TUNING:                              98,
	MSG_ID_DEEPSTALL:                               120,
	MSG_ID_GIMBAL_REPORT:                           134,
	MSG_ID_GIMBAL_CONTROL:                          205,
	MSG_ID_GIMBAL_TORQUE_CMD_REPORT:                69,
	MSG_ID_GOPRO_HEARTBEAT:                         101,
	MSG_ID_GOPRO_GET_REQUEST:                       50,
	MSG_ID_GOPRO_GET_RESPONSE:                      202,
	MSG_ID_GOPRO_SET_REQUEST:                       17,
	MSG_ID_GOPRO_SET_RESPONSE:                      162,
	MSG_ID_RPM:                                     207,
	MSG_ID_SYS_STATUS:                              124,
	MSG_ID_SYSTEM_TIME:                             137,
	MSG_ID_PING:                                    237,
	MSG_ID_CHANGE_OPERATOR_CONTROL:                 217,
	MSG_ID_CHANGE_OPERATOR_CONTROL_ACK:             104,
	MSG_ID_AUTH_KEY:                                119,
	MSG_ID_LINK_NODE_STATUS:                        117,
	MSG_ID_SET_MODE:                                89,
	MSG_ID_PARAM_ACK_TRANSACTION:                   137,
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
	MSG_ID_MISSION_CHANGED:                         132,
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
	MSG_ID_COMMAND_CANCEL:                          14,
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
	MSG_ID_FENCE_STATUS:                            189,
	MSG_ID_MAG_CAL_REPORT:                          36,
	MSG_ID_EFI_STATUS:                              208,
	MSG_ID_ESTIMATOR_STATUS:                        163,
	MSG_ID_WIND_COV:                                105,
	MSG_ID_GPS_INPUT:                               151,
	MSG_ID_GPS_RTCM_DATA:                           35,
	MSG_ID_HIGH_LATENCY:                            150,
	MSG_ID_HIGH_LATENCY2:                           179,
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
	MSG_ID_HEARTBEAT:                               50,
}

// DialectArdupilotmega is the dialect represented by ardupilotmega.xml
var DialectArdupilotmega = &Dialect{
	Name: "ardupilotmega",
	messageConstructorByMsgID: map[MessageID]func(*Packet) Message{
		MSG_ID_SENSOR_OFFSETS: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaSensorOffsets)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SET_MAG_OFFSETS: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaSetMagOffsets)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_MEMINFO: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaMeminfo)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_AP_ADC: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaApAdc)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_DIGICAM_CONFIGURE: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaDigicamConfigure)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_DIGICAM_CONTROL: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaDigicamControl)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_MOUNT_CONFIGURE: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaMountConfigure)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_MOUNT_CONTROL: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaMountControl)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_MOUNT_STATUS: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaMountStatus)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_FENCE_POINT: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaFencePoint)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_FENCE_FETCH_POINT: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaFenceFetchPoint)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_AHRS: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaAhrs)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SIMSTATE: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaSimstate)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_HWSTATUS: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaHwstatus)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_RADIO: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaRadio)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_LIMITS_STATUS: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaLimitsStatus)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_WIND: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaWind)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_DATA16: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaData16)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_DATA32: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaData32)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_DATA64: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaData64)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_DATA96: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaData96)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_RANGEFINDER: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaRangefinder)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_AIRSPEED_AUTOCAL: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaAirspeedAutocal)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_RALLY_POINT: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaRallyPoint)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_RALLY_FETCH_POINT: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaRallyFetchPoint)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_COMPASSMOT_STATUS: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaCompassmotStatus)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_AHRS2: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaAhrs2)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_CAMERA_STATUS: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaCameraStatus)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_CAMERA_FEEDBACK: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaCameraFeedback)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_BATTERY2: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaBattery2)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_AHRS3: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaAhrs3)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_AUTOPILOT_VERSION_REQUEST: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaAutopilotVersionRequest)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_REMOTE_LOG_DATA_BLOCK: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaRemoteLogDataBlock)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_REMOTE_LOG_BLOCK_STATUS: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaRemoteLogBlockStatus)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_LED_CONTROL: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaLedControl)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_MAG_CAL_PROGRESS: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaMagCalProgress)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_EKF_STATUS_REPORT: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaEkfStatusReport)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_PID_TUNING: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaPidTuning)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_DEEPSTALL: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaDeepstall)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_GIMBAL_REPORT: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaGimbalReport)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_GIMBAL_CONTROL: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaGimbalControl)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_GIMBAL_TORQUE_CMD_REPORT: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaGimbalTorqueCmdReport)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_GOPRO_HEARTBEAT: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaGoproHeartbeat)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_GOPRO_GET_REQUEST: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaGoproGetRequest)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_GOPRO_GET_RESPONSE: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaGoproGetResponse)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_GOPRO_SET_REQUEST: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaGoproSetRequest)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_GOPRO_SET_RESPONSE: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaGoproSetResponse)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_RPM: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaRpm)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SYS_STATUS: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaSysStatus)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SYSTEM_TIME: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaSystemTime)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_PING: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaPing)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_CHANGE_OPERATOR_CONTROL: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaChangeOperatorControl)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_CHANGE_OPERATOR_CONTROL_ACK: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaChangeOperatorControlAck)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_AUTH_KEY: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaAuthKey)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_LINK_NODE_STATUS: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaLinkNodeStatus)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SET_MODE: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaSetMode)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_PARAM_ACK_TRANSACTION: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaParamAckTransaction)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_PARAM_REQUEST_READ: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaParamRequestRead)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_PARAM_REQUEST_LIST: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaParamRequestList)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_PARAM_VALUE: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaParamValue)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_PARAM_SET: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaParamSet)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_GPS_RAW_INT: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaGpsRawInt)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_GPS_STATUS: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaGpsStatus)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SCALED_IMU: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaScaledImu)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_RAW_IMU: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaRawImu)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_RAW_PRESSURE: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaRawPressure)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SCALED_PRESSURE: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaScaledPressure)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_ATTITUDE: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaAttitude)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_ATTITUDE_QUATERNION: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaAttitudeQuaternion)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_LOCAL_POSITION_NED: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaLocalPositionNed)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_GLOBAL_POSITION_INT: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaGlobalPositionInt)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_RC_CHANNELS_SCALED: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaRcChannelsScaled)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_RC_CHANNELS_RAW: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaRcChannelsRaw)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SERVO_OUTPUT_RAW: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaServoOutputRaw)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_MISSION_REQUEST_PARTIAL_LIST: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaMissionRequestPartialList)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_MISSION_WRITE_PARTIAL_LIST: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaMissionWritePartialList)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_MISSION_ITEM: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaMissionItem)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_MISSION_REQUEST: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaMissionRequest)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_MISSION_SET_CURRENT: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaMissionSetCurrent)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_MISSION_CURRENT: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaMissionCurrent)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_MISSION_REQUEST_LIST: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaMissionRequestList)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_MISSION_COUNT: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaMissionCount)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_MISSION_CLEAR_ALL: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaMissionClearAll)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_MISSION_ITEM_REACHED: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaMissionItemReached)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_MISSION_ACK: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaMissionAck)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SET_GPS_GLOBAL_ORIGIN: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaSetGpsGlobalOrigin)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_GPS_GLOBAL_ORIGIN: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaGpsGlobalOrigin)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_PARAM_MAP_RC: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaParamMapRc)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_MISSION_REQUEST_INT: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaMissionRequestInt)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_MISSION_CHANGED: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaMissionChanged)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SAFETY_SET_ALLOWED_AREA: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaSafetySetAllowedArea)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SAFETY_ALLOWED_AREA: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaSafetyAllowedArea)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_ATTITUDE_QUATERNION_COV: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaAttitudeQuaternionCov)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_NAV_CONTROLLER_OUTPUT: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaNavControllerOutput)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_GLOBAL_POSITION_INT_COV: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaGlobalPositionIntCov)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_LOCAL_POSITION_NED_COV: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaLocalPositionNedCov)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_RC_CHANNELS: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaRcChannels)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_REQUEST_DATA_STREAM: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaRequestDataStream)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_DATA_STREAM: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaDataStream)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_MANUAL_CONTROL: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaManualControl)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_RC_CHANNELS_OVERRIDE: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaRcChannelsOverride)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_MISSION_ITEM_INT: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaMissionItemInt)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_VFR_HUD: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaVfrHud)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_COMMAND_INT: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaCommandInt)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_COMMAND_LONG: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaCommandLong)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_COMMAND_ACK: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaCommandAck)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_COMMAND_CANCEL: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaCommandCancel)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_MANUAL_SETPOINT: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaManualSetpoint)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SET_ATTITUDE_TARGET: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaSetAttitudeTarget)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_ATTITUDE_TARGET: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaAttitudeTarget)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SET_POSITION_TARGET_LOCAL_NED: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaSetPositionTargetLocalNed)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_POSITION_TARGET_LOCAL_NED: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaPositionTargetLocalNed)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SET_POSITION_TARGET_GLOBAL_INT: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaSetPositionTargetGlobalInt)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_POSITION_TARGET_GLOBAL_INT: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaPositionTargetGlobalInt)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaLocalPositionNedSystemGlobalOffset)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_HIL_STATE: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaHilState)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_HIL_CONTROLS: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaHilControls)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_HIL_RC_INPUTS_RAW: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaHilRcInputsRaw)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_HIL_ACTUATOR_CONTROLS: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaHilActuatorControls)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_OPTICAL_FLOW: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaOpticalFlow)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_GLOBAL_VISION_POSITION_ESTIMATE: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaGlobalVisionPositionEstimate)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_VISION_POSITION_ESTIMATE: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaVisionPositionEstimate)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_VISION_SPEED_ESTIMATE: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaVisionSpeedEstimate)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_VICON_POSITION_ESTIMATE: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaViconPositionEstimate)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_HIGHRES_IMU: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaHighresImu)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_OPTICAL_FLOW_RAD: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaOpticalFlowRad)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_HIL_SENSOR: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaHilSensor)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SIM_STATE: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaSimState)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_RADIO_STATUS: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaRadioStatus)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_FILE_TRANSFER_PROTOCOL: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaFileTransferProtocol)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_TIMESYNC: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaTimesync)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_CAMERA_TRIGGER: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaCameraTrigger)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_HIL_GPS: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaHilGps)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_HIL_OPTICAL_FLOW: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaHilOpticalFlow)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_HIL_STATE_QUATERNION: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaHilStateQuaternion)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SCALED_IMU2: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaScaledImu2)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_LOG_REQUEST_LIST: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaLogRequestList)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_LOG_ENTRY: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaLogEntry)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_LOG_REQUEST_DATA: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaLogRequestData)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_LOG_DATA: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaLogData)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_LOG_ERASE: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaLogErase)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_LOG_REQUEST_END: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaLogRequestEnd)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_GPS_INJECT_DATA: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaGpsInjectData)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_GPS2_RAW: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaGps2Raw)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_POWER_STATUS: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaPowerStatus)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SERIAL_CONTROL: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaSerialControl)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_GPS_RTK: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaGpsRtk)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_GPS2_RTK: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaGps2Rtk)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SCALED_IMU3: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaScaledImu3)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_DATA_TRANSMISSION_HANDSHAKE: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaDataTransmissionHandshake)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_ENCAPSULATED_DATA: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaEncapsulatedData)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_DISTANCE_SENSOR: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaDistanceSensor)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_TERRAIN_REQUEST: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaTerrainRequest)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_TERRAIN_DATA: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaTerrainData)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_TERRAIN_CHECK: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaTerrainCheck)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_TERRAIN_REPORT: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaTerrainReport)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SCALED_PRESSURE2: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaScaledPressure2)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_ATT_POS_MOCAP: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaAttPosMocap)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SET_ACTUATOR_CONTROL_TARGET: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaSetActuatorControlTarget)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_ACTUATOR_CONTROL_TARGET: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaActuatorControlTarget)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_ALTITUDE: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaAltitude)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_RESOURCE_REQUEST: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaResourceRequest)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SCALED_PRESSURE3: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaScaledPressure3)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_FOLLOW_TARGET: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaFollowTarget)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_CONTROL_SYSTEM_STATE: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaControlSystemState)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_BATTERY_STATUS: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaBatteryStatus)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_AUTOPILOT_VERSION: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaAutopilotVersion)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_LANDING_TARGET: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaLandingTarget)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_FENCE_STATUS: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaFenceStatus)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_MAG_CAL_REPORT: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaMagCalReport)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_EFI_STATUS: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaEfiStatus)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_ESTIMATOR_STATUS: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaEstimatorStatus)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_WIND_COV: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaWindCov)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_GPS_INPUT: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaGpsInput)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_GPS_RTCM_DATA: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaGpsRtcmData)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_HIGH_LATENCY: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaHighLatency)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_HIGH_LATENCY2: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaHighLatency2)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_VIBRATION: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaVibration)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_HOME_POSITION: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaHomePosition)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SET_HOME_POSITION: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaSetHomePosition)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_MESSAGE_INTERVAL: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaMessageInterval)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_EXTENDED_SYS_STATE: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaExtendedSysState)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_ADSB_VEHICLE: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaAdsbVehicle)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_COLLISION: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaCollision)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_V2_EXTENSION: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaV2Extension)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_MEMORY_VECT: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaMemoryVect)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_DEBUG_VECT: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaDebugVect)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_NAMED_VALUE_FLOAT: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaNamedValueFloat)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_NAMED_VALUE_INT: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaNamedValueInt)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_STATUSTEXT: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaStatustext)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_DEBUG: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaDebug)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_HEARTBEAT: func(pkt *Packet) Message {
			msg := new(ArdupilotmegaHeartbeat)
			msg.Unpack(pkt)
			return msg
		},
	},
}

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

// MsgName (generated function)
func (m *ArdupilotmegaSensorOffsets) MsgName() string {
	return "SensorOffsets"
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
func (m *ArdupilotmegaSensorOffsets) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 42 {
		payload = append(payload, zeroTail[:42-len(p.Payload)]...)
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

// MsgName (generated function)
func (m *ArdupilotmegaSetMagOffsets) MsgName() string {
	return "SetMagOffsets"
}

// Pack (generated function)
func (m *ArdupilotmegaSetMagOffsets) Pack(p *Packet) error {
	payload := make([]byte, 8)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.MagOfsX))
	binary.LittleEndian.PutUint16(payload[2:], uint16(m.MagOfsY))
	binary.LittleEndian.PutUint16(payload[4:], uint16(m.MagOfsZ))
	payload[6] = byte(m.TargetSystem)
	payload[7] = byte(m.TargetComponent)
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
func (m *ArdupilotmegaSetMagOffsets) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 8 {
		payload = append(payload, zeroTail[:8-len(p.Payload)]...)
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

// MsgName (generated function)
func (m *ArdupilotmegaMeminfo) MsgName() string {
	return "Meminfo"
}

// Pack (generated function)
func (m *ArdupilotmegaMeminfo) Pack(p *Packet) error {
	payload := make([]byte, 4)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.Brkval))
	binary.LittleEndian.PutUint16(payload[2:], uint16(m.Freemem))
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
func (m *ArdupilotmegaMeminfo) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 4 {
		payload = append(payload, zeroTail[:4-len(p.Payload)]...)
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

// MsgName (generated function)
func (m *ArdupilotmegaApAdc) MsgName() string {
	return "ApAdc"
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
func (m *ArdupilotmegaApAdc) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 12 {
		payload = append(payload, zeroTail[:12-len(p.Payload)]...)
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

// MsgName (generated function)
func (m *ArdupilotmegaDigicamConfigure) MsgName() string {
	return "DigicamConfigure"
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
func (m *ArdupilotmegaDigicamConfigure) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 15 {
		payload = append(payload, zeroTail[:15-len(p.Payload)]...)
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

// MsgName (generated function)
func (m *ArdupilotmegaDigicamControl) MsgName() string {
	return "DigicamControl"
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
func (m *ArdupilotmegaDigicamControl) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 13 {
		payload = append(payload, zeroTail[:13-len(p.Payload)]...)
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

// MsgName (generated function)
func (m *ArdupilotmegaMountConfigure) MsgName() string {
	return "MountConfigure"
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
func (m *ArdupilotmegaMountConfigure) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 6 {
		payload = append(payload, zeroTail[:6-len(p.Payload)]...)
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

// MsgName (generated function)
func (m *ArdupilotmegaMountControl) MsgName() string {
	return "MountControl"
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
func (m *ArdupilotmegaMountControl) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 15 {
		payload = append(payload, zeroTail[:15-len(p.Payload)]...)
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

// MsgName (generated function)
func (m *ArdupilotmegaMountStatus) MsgName() string {
	return "MountStatus"
}

// Pack (generated function)
func (m *ArdupilotmegaMountStatus) Pack(p *Packet) error {
	payload := make([]byte, 14)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.PointingA))
	binary.LittleEndian.PutUint32(payload[4:], uint32(m.PointingB))
	binary.LittleEndian.PutUint32(payload[8:], uint32(m.PointingC))
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
func (m *ArdupilotmegaMountStatus) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 14 {
		payload = append(payload, zeroTail[:14-len(p.Payload)]...)
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

// MsgName (generated function)
func (m *ArdupilotmegaFencePoint) MsgName() string {
	return "FencePoint"
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
func (m *ArdupilotmegaFencePoint) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 12 {
		payload = append(payload, zeroTail[:12-len(p.Payload)]...)
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

// MsgName (generated function)
func (m *ArdupilotmegaFenceFetchPoint) MsgName() string {
	return "FenceFetchPoint"
}

// Pack (generated function)
func (m *ArdupilotmegaFenceFetchPoint) Pack(p *Packet) error {
	payload := make([]byte, 3)
	payload[0] = byte(m.TargetSystem)
	payload[1] = byte(m.TargetComponent)
	payload[2] = byte(m.Idx)
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
func (m *ArdupilotmegaFenceFetchPoint) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 3 {
		payload = append(payload, zeroTail[:3-len(p.Payload)]...)
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

// MsgName (generated function)
func (m *ArdupilotmegaAhrs) MsgName() string {
	return "Ahrs"
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
func (m *ArdupilotmegaAhrs) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 28 {
		payload = append(payload, zeroTail[:28-len(p.Payload)]...)
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

// MsgName (generated function)
func (m *ArdupilotmegaSimstate) MsgName() string {
	return "Simstate"
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
func (m *ArdupilotmegaSimstate) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 44 {
		payload = append(payload, zeroTail[:44-len(p.Payload)]...)
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

// MsgName (generated function)
func (m *ArdupilotmegaHwstatus) MsgName() string {
	return "Hwstatus"
}

// Pack (generated function)
func (m *ArdupilotmegaHwstatus) Pack(p *Packet) error {
	payload := make([]byte, 3)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.Vcc))
	payload[2] = byte(m.I2cerr)
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
func (m *ArdupilotmegaHwstatus) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 3 {
		payload = append(payload, zeroTail[:3-len(p.Payload)]...)
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

// MsgName (generated function)
func (m *ArdupilotmegaRadio) MsgName() string {
	return "Radio"
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
func (m *ArdupilotmegaRadio) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 9 {
		payload = append(payload, zeroTail[:9-len(p.Payload)]...)
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

// MsgName (generated function)
func (m *ArdupilotmegaLimitsStatus) MsgName() string {
	return "LimitsStatus"
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
func (m *ArdupilotmegaLimitsStatus) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 22 {
		payload = append(payload, zeroTail[:22-len(p.Payload)]...)
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

// MsgName (generated function)
func (m *ArdupilotmegaWind) MsgName() string {
	return "Wind"
}

// Pack (generated function)
func (m *ArdupilotmegaWind) Pack(p *Packet) error {
	payload := make([]byte, 12)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(m.Direction))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(m.Speed))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.SpeedZ))
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
func (m *ArdupilotmegaWind) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 12 {
		payload = append(payload, zeroTail[:12-len(p.Payload)]...)
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

// MsgName (generated function)
func (m *ArdupilotmegaData16) MsgName() string {
	return "Data16"
}

// Pack (generated function)
func (m *ArdupilotmegaData16) Pack(p *Packet) error {
	payload := make([]byte, 18)
	payload[0] = byte(m.Type)
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
func (m *ArdupilotmegaData16) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 18 {
		payload = append(payload, zeroTail[:18-len(p.Payload)]...)
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

// MsgName (generated function)
func (m *ArdupilotmegaData32) MsgName() string {
	return "Data32"
}

// Pack (generated function)
func (m *ArdupilotmegaData32) Pack(p *Packet) error {
	payload := make([]byte, 34)
	payload[0] = byte(m.Type)
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
func (m *ArdupilotmegaData32) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 34 {
		payload = append(payload, zeroTail[:34-len(p.Payload)]...)
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

// MsgName (generated function)
func (m *ArdupilotmegaData64) MsgName() string {
	return "Data64"
}

// Pack (generated function)
func (m *ArdupilotmegaData64) Pack(p *Packet) error {
	payload := make([]byte, 66)
	payload[0] = byte(m.Type)
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
func (m *ArdupilotmegaData64) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 66 {
		payload = append(payload, zeroTail[:66-len(p.Payload)]...)
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

// MsgName (generated function)
func (m *ArdupilotmegaData96) MsgName() string {
	return "Data96"
}

// Pack (generated function)
func (m *ArdupilotmegaData96) Pack(p *Packet) error {
	payload := make([]byte, 98)
	payload[0] = byte(m.Type)
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
func (m *ArdupilotmegaData96) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 98 {
		payload = append(payload, zeroTail[:98-len(p.Payload)]...)
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

// MsgName (generated function)
func (m *ArdupilotmegaRangefinder) MsgName() string {
	return "Rangefinder"
}

// Pack (generated function)
func (m *ArdupilotmegaRangefinder) Pack(p *Packet) error {
	payload := make([]byte, 8)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(m.Distance))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(m.Voltage))
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
func (m *ArdupilotmegaRangefinder) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 8 {
		payload = append(payload, zeroTail[:8-len(p.Payload)]...)
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

// MsgName (generated function)
func (m *ArdupilotmegaAirspeedAutocal) MsgName() string {
	return "AirspeedAutocal"
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
func (m *ArdupilotmegaAirspeedAutocal) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 48 {
		payload = append(payload, zeroTail[:48-len(p.Payload)]...)
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

// MsgName (generated function)
func (m *ArdupilotmegaRallyPoint) MsgName() string {
	return "RallyPoint"
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
func (m *ArdupilotmegaRallyPoint) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 19 {
		payload = append(payload, zeroTail[:19-len(p.Payload)]...)
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

// MsgName (generated function)
func (m *ArdupilotmegaRallyFetchPoint) MsgName() string {
	return "RallyFetchPoint"
}

// Pack (generated function)
func (m *ArdupilotmegaRallyFetchPoint) Pack(p *Packet) error {
	payload := make([]byte, 3)
	payload[0] = byte(m.TargetSystem)
	payload[1] = byte(m.TargetComponent)
	payload[2] = byte(m.Idx)
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
func (m *ArdupilotmegaRallyFetchPoint) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 3 {
		payload = append(payload, zeroTail[:3-len(p.Payload)]...)
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

// MsgName (generated function)
func (m *ArdupilotmegaCompassmotStatus) MsgName() string {
	return "CompassmotStatus"
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
func (m *ArdupilotmegaCompassmotStatus) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 20 {
		payload = append(payload, zeroTail[:20-len(p.Payload)]...)
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

// MsgName (generated function)
func (m *ArdupilotmegaAhrs2) MsgName() string {
	return "Ahrs2"
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
func (m *ArdupilotmegaAhrs2) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 24 {
		payload = append(payload, zeroTail[:24-len(p.Payload)]...)
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

// MsgName (generated function)
func (m *ArdupilotmegaCameraStatus) MsgName() string {
	return "CameraStatus"
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
func (m *ArdupilotmegaCameraStatus) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 29 {
		payload = append(payload, zeroTail[:29-len(p.Payload)]...)
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

// MsgName (generated function)
func (m *ArdupilotmegaCameraFeedback) MsgName() string {
	return "CameraFeedback"
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
func (m *ArdupilotmegaCameraFeedback) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 45 {
		payload = append(payload, zeroTail[:45-len(p.Payload)]...)
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

// MsgName (generated function)
func (m *ArdupilotmegaBattery2) MsgName() string {
	return "Battery2"
}

// Pack (generated function)
func (m *ArdupilotmegaBattery2) Pack(p *Packet) error {
	payload := make([]byte, 4)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.Voltage))
	binary.LittleEndian.PutUint16(payload[2:], uint16(m.CurrentBattery))
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
func (m *ArdupilotmegaBattery2) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 4 {
		payload = append(payload, zeroTail[:4-len(p.Payload)]...)
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

// MsgName (generated function)
func (m *ArdupilotmegaAhrs3) MsgName() string {
	return "Ahrs3"
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
func (m *ArdupilotmegaAhrs3) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 40 {
		payload = append(payload, zeroTail[:40-len(p.Payload)]...)
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

// MsgName (generated function)
func (m *ArdupilotmegaAutopilotVersionRequest) MsgName() string {
	return "AutopilotVersionRequest"
}

// Pack (generated function)
func (m *ArdupilotmegaAutopilotVersionRequest) Pack(p *Packet) error {
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
func (m *ArdupilotmegaAutopilotVersionRequest) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 2 {
		payload = append(payload, zeroTail[:2-len(p.Payload)]...)
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

// MsgName (generated function)
func (m *ArdupilotmegaRemoteLogDataBlock) MsgName() string {
	return "RemoteLogDataBlock"
}

// Pack (generated function)
func (m *ArdupilotmegaRemoteLogDataBlock) Pack(p *Packet) error {
	payload := make([]byte, 206)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.Seqno))
	payload[4] = byte(m.TargetSystem)
	payload[5] = byte(m.TargetComponent)
	copy(payload[6:], m.Data[:])
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
func (m *ArdupilotmegaRemoteLogDataBlock) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 206 {
		payload = append(payload, zeroTail[:206-len(p.Payload)]...)
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

// MsgName (generated function)
func (m *ArdupilotmegaRemoteLogBlockStatus) MsgName() string {
	return "RemoteLogBlockStatus"
}

// Pack (generated function)
func (m *ArdupilotmegaRemoteLogBlockStatus) Pack(p *Packet) error {
	payload := make([]byte, 7)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.Seqno))
	payload[4] = byte(m.TargetSystem)
	payload[5] = byte(m.TargetComponent)
	payload[6] = byte(m.Status)
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
func (m *ArdupilotmegaRemoteLogBlockStatus) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 7 {
		payload = append(payload, zeroTail[:7-len(p.Payload)]...)
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

// MsgName (generated function)
func (m *ArdupilotmegaLedControl) MsgName() string {
	return "LedControl"
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
func (m *ArdupilotmegaLedControl) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 29 {
		payload = append(payload, zeroTail[:29-len(p.Payload)]...)
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

// MsgName (generated function)
func (m *ArdupilotmegaMagCalProgress) MsgName() string {
	return "MagCalProgress"
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
func (m *ArdupilotmegaMagCalProgress) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 27 {
		payload = append(payload, zeroTail[:27-len(p.Payload)]...)
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

// MsgName (generated function)
func (m *ArdupilotmegaEkfStatusReport) MsgName() string {
	return "EkfStatusReport"
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
func (m *ArdupilotmegaEkfStatusReport) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 22 {
		payload = append(payload, zeroTail[:22-len(p.Payload)]...)
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

// MsgName (generated function)
func (m *ArdupilotmegaPidTuning) MsgName() string {
	return "PidTuning"
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
func (m *ArdupilotmegaPidTuning) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 25 {
		payload = append(payload, zeroTail[:25-len(p.Payload)]...)
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

// MsgName (generated function)
func (m *ArdupilotmegaDeepstall) MsgName() string {
	return "Deepstall"
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
func (m *ArdupilotmegaDeepstall) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 37 {
		payload = append(payload, zeroTail[:37-len(p.Payload)]...)
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

// MsgName (generated function)
func (m *ArdupilotmegaGimbalReport) MsgName() string {
	return "GimbalReport"
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
func (m *ArdupilotmegaGimbalReport) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 42 {
		payload = append(payload, zeroTail[:42-len(p.Payload)]...)
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

// MsgName (generated function)
func (m *ArdupilotmegaGimbalControl) MsgName() string {
	return "GimbalControl"
}

// Pack (generated function)
func (m *ArdupilotmegaGimbalControl) Pack(p *Packet) error {
	payload := make([]byte, 14)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(m.DemandedRateX))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(m.DemandedRateY))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.DemandedRateZ))
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
func (m *ArdupilotmegaGimbalControl) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 14 {
		payload = append(payload, zeroTail[:14-len(p.Payload)]...)
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

// MsgName (generated function)
func (m *ArdupilotmegaGimbalTorqueCmdReport) MsgName() string {
	return "GimbalTorqueCmdReport"
}

// Pack (generated function)
func (m *ArdupilotmegaGimbalTorqueCmdReport) Pack(p *Packet) error {
	payload := make([]byte, 8)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.RlTorqueCmd))
	binary.LittleEndian.PutUint16(payload[2:], uint16(m.ElTorqueCmd))
	binary.LittleEndian.PutUint16(payload[4:], uint16(m.AzTorqueCmd))
	payload[6] = byte(m.TargetSystem)
	payload[7] = byte(m.TargetComponent)
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
func (m *ArdupilotmegaGimbalTorqueCmdReport) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 8 {
		payload = append(payload, zeroTail[:8-len(p.Payload)]...)
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

// MsgName (generated function)
func (m *ArdupilotmegaGoproHeartbeat) MsgName() string {
	return "GoproHeartbeat"
}

// Pack (generated function)
func (m *ArdupilotmegaGoproHeartbeat) Pack(p *Packet) error {
	payload := make([]byte, 3)
	payload[0] = byte(m.Status)
	payload[1] = byte(m.CaptureMode)
	payload[2] = byte(m.Flags)
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
func (m *ArdupilotmegaGoproHeartbeat) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 3 {
		payload = append(payload, zeroTail[:3-len(p.Payload)]...)
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

// MsgName (generated function)
func (m *ArdupilotmegaGoproGetRequest) MsgName() string {
	return "GoproGetRequest"
}

// Pack (generated function)
func (m *ArdupilotmegaGoproGetRequest) Pack(p *Packet) error {
	payload := make([]byte, 3)
	payload[0] = byte(m.TargetSystem)
	payload[1] = byte(m.TargetComponent)
	payload[2] = byte(m.CmdID)
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
func (m *ArdupilotmegaGoproGetRequest) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 3 {
		payload = append(payload, zeroTail[:3-len(p.Payload)]...)
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

// MsgName (generated function)
func (m *ArdupilotmegaGoproGetResponse) MsgName() string {
	return "GoproGetResponse"
}

// Pack (generated function)
func (m *ArdupilotmegaGoproGetResponse) Pack(p *Packet) error {
	payload := make([]byte, 6)
	payload[0] = byte(m.CmdID)
	payload[1] = byte(m.Status)
	copy(payload[2:], m.Value[:])
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
func (m *ArdupilotmegaGoproGetResponse) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 6 {
		payload = append(payload, zeroTail[:6-len(p.Payload)]...)
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

// MsgName (generated function)
func (m *ArdupilotmegaGoproSetRequest) MsgName() string {
	return "GoproSetRequest"
}

// Pack (generated function)
func (m *ArdupilotmegaGoproSetRequest) Pack(p *Packet) error {
	payload := make([]byte, 7)
	payload[0] = byte(m.TargetSystem)
	payload[1] = byte(m.TargetComponent)
	payload[2] = byte(m.CmdID)
	copy(payload[3:], m.Value[:])
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
func (m *ArdupilotmegaGoproSetRequest) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 7 {
		payload = append(payload, zeroTail[:7-len(p.Payload)]...)
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

// MsgName (generated function)
func (m *ArdupilotmegaGoproSetResponse) MsgName() string {
	return "GoproSetResponse"
}

// Pack (generated function)
func (m *ArdupilotmegaGoproSetResponse) Pack(p *Packet) error {
	payload := make([]byte, 2)
	payload[0] = byte(m.CmdID)
	payload[1] = byte(m.Status)
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
func (m *ArdupilotmegaGoproSetResponse) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 2 {
		payload = append(payload, zeroTail[:2-len(p.Payload)]...)
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

// MsgName (generated function)
func (m *ArdupilotmegaRpm) MsgName() string {
	return "Rpm"
}

// Pack (generated function)
func (m *ArdupilotmegaRpm) Pack(p *Packet) error {
	payload := make([]byte, 8)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(m.Rpm1))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(m.Rpm2))
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
func (m *ArdupilotmegaRpm) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 8 {
		payload = append(payload, zeroTail[:8-len(p.Payload)]...)
	}
	m.Rpm1 = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.Rpm2 = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	return nil
}

// Message IDs
const (
	MSG_ID_SENSOR_OFFSETS            MessageID = 150
	MSG_ID_SET_MAG_OFFSETS           MessageID = 151
	MSG_ID_MEMINFO                   MessageID = 152
	MSG_ID_AP_ADC                    MessageID = 153
	MSG_ID_DIGICAM_CONFIGURE         MessageID = 154
	MSG_ID_DIGICAM_CONTROL           MessageID = 155
	MSG_ID_MOUNT_CONFIGURE           MessageID = 156
	MSG_ID_MOUNT_CONTROL             MessageID = 157
	MSG_ID_MOUNT_STATUS              MessageID = 158
	MSG_ID_FENCE_POINT               MessageID = 160
	MSG_ID_FENCE_FETCH_POINT         MessageID = 161
	MSG_ID_AHRS                      MessageID = 163
	MSG_ID_SIMSTATE                  MessageID = 164
	MSG_ID_HWSTATUS                  MessageID = 165
	MSG_ID_RADIO                     MessageID = 166
	MSG_ID_LIMITS_STATUS             MessageID = 167
	MSG_ID_WIND                      MessageID = 168
	MSG_ID_DATA16                    MessageID = 169
	MSG_ID_DATA32                    MessageID = 170
	MSG_ID_DATA64                    MessageID = 171
	MSG_ID_DATA96                    MessageID = 172
	MSG_ID_RANGEFINDER               MessageID = 173
	MSG_ID_AIRSPEED_AUTOCAL          MessageID = 174
	MSG_ID_RALLY_POINT               MessageID = 175
	MSG_ID_RALLY_FETCH_POINT         MessageID = 176
	MSG_ID_COMPASSMOT_STATUS         MessageID = 177
	MSG_ID_AHRS2                     MessageID = 178
	MSG_ID_CAMERA_STATUS             MessageID = 179
	MSG_ID_CAMERA_FEEDBACK           MessageID = 180
	MSG_ID_BATTERY2                  MessageID = 181
	MSG_ID_AHRS3                     MessageID = 182
	MSG_ID_AUTOPILOT_VERSION_REQUEST MessageID = 183
	MSG_ID_REMOTE_LOG_DATA_BLOCK     MessageID = 184
	MSG_ID_REMOTE_LOG_BLOCK_STATUS   MessageID = 185
	MSG_ID_LED_CONTROL               MessageID = 186
	MSG_ID_MAG_CAL_PROGRESS          MessageID = 191
	MSG_ID_EKF_STATUS_REPORT         MessageID = 193
	MSG_ID_PID_TUNING                MessageID = 194
	MSG_ID_DEEPSTALL                 MessageID = 195
	MSG_ID_GIMBAL_REPORT             MessageID = 200
	MSG_ID_GIMBAL_CONTROL            MessageID = 201
	MSG_ID_GIMBAL_TORQUE_CMD_REPORT  MessageID = 214
	MSG_ID_GOPRO_HEARTBEAT           MessageID = 215
	MSG_ID_GOPRO_GET_REQUEST         MessageID = 216
	MSG_ID_GOPRO_GET_RESPONSE        MessageID = 217
	MSG_ID_GOPRO_SET_REQUEST         MessageID = 218
	MSG_ID_GOPRO_SET_RESPONSE        MessageID = 219
	MSG_ID_RPM                       MessageID = 226
)

// DialectArdupilotmega is the dialect represented by ardupilotmega.xml
var DialectArdupilotmega = &Dialect{
	Name: "ardupilotmega",
	crcExtras: map[MessageID]uint8{
		MSG_ID_SENSOR_OFFSETS:            134,
		MSG_ID_SET_MAG_OFFSETS:           219,
		MSG_ID_MEMINFO:                   208,
		MSG_ID_AP_ADC:                    188,
		MSG_ID_DIGICAM_CONFIGURE:         84,
		MSG_ID_DIGICAM_CONTROL:           22,
		MSG_ID_MOUNT_CONFIGURE:           19,
		MSG_ID_MOUNT_CONTROL:             21,
		MSG_ID_MOUNT_STATUS:              134,
		MSG_ID_FENCE_POINT:               78,
		MSG_ID_FENCE_FETCH_POINT:         68,
		MSG_ID_AHRS:                      127,
		MSG_ID_SIMSTATE:                  154,
		MSG_ID_HWSTATUS:                  21,
		MSG_ID_RADIO:                     21,
		MSG_ID_LIMITS_STATUS:             144,
		MSG_ID_WIND:                      1,
		MSG_ID_DATA16:                    234,
		MSG_ID_DATA32:                    73,
		MSG_ID_DATA64:                    181,
		MSG_ID_DATA96:                    22,
		MSG_ID_RANGEFINDER:               83,
		MSG_ID_AIRSPEED_AUTOCAL:          167,
		MSG_ID_RALLY_POINT:               138,
		MSG_ID_RALLY_FETCH_POINT:         234,
		MSG_ID_COMPASSMOT_STATUS:         240,
		MSG_ID_AHRS2:                     47,
		MSG_ID_CAMERA_STATUS:             189,
		MSG_ID_CAMERA_FEEDBACK:           52,
		MSG_ID_BATTERY2:                  174,
		MSG_ID_AHRS3:                     229,
		MSG_ID_AUTOPILOT_VERSION_REQUEST: 85,
		MSG_ID_REMOTE_LOG_DATA_BLOCK:     159,
		MSG_ID_REMOTE_LOG_BLOCK_STATUS:   186,
		MSG_ID_LED_CONTROL:               72,
		MSG_ID_MAG_CAL_PROGRESS:          92,
		MSG_ID_EKF_STATUS_REPORT:         71,
		MSG_ID_PID_TUNING:                98,
		MSG_ID_DEEPSTALL:                 120,
		MSG_ID_GIMBAL_REPORT:             134,
		MSG_ID_GIMBAL_CONTROL:            205,
		MSG_ID_GIMBAL_TORQUE_CMD_REPORT:  69,
		MSG_ID_GOPRO_HEARTBEAT:           101,
		MSG_ID_GOPRO_GET_REQUEST:         50,
		MSG_ID_GOPRO_GET_RESPONSE:        202,
		MSG_ID_GOPRO_SET_REQUEST:         17,
		MSG_ID_GOPRO_SET_RESPONSE:        162,
		MSG_ID_RPM:                       207,
	},
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
	},
}

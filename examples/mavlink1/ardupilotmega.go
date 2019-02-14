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

// AccelcalVehiclePos (generated enum)
//
const (
	ACCELCAL_VEHICLE_POS_LEVEL    = 1 //
	ACCELCAL_VEHICLE_POS_LEFT     = 2 //
	ACCELCAL_VEHICLE_POS_RIGHT    = 3 //
	ACCELCAL_VEHICLE_POS_NOSEDOWN = 4 //
	ACCELCAL_VEHICLE_POS_NOSEUP   = 5 //
	ACCELCAL_VEHICLE_POS_BACK     = 6 //
)

// MavCmd (generated enum)
//
const (
	MAV_CMD_DO_GRIPPER                      = 211   // Mission command to operate EPM gripper
	MAV_CMD_DO_AUTOTUNE_ENABLE              = 212   // Enable/disable autotune
	MAV_CMD_NAV_ALTITUDE_WAIT               = 83    // Mission command to wait for an altitude or downwards vertical speed. This is meant for high altitude balloon launches, allowing the aircraft to be idle until either an altitude is reached or a negative vertical speed is reached (indicating early balloon burst). The wiggle time is how often to wiggle the control surfaces to prevent them seizing up.
	MAV_CMD_POWER_OFF_INITIATED             = 42000 // A system wide power-off event has been initiated.
	MAV_CMD_SOLO_BTN_FLY_CLICK              = 42001 // FLY button has been clicked.
	MAV_CMD_SOLO_BTN_FLY_HOLD               = 42002 // FLY button has been held for 1.5 seconds.
	MAV_CMD_SOLO_BTN_PAUSE_CLICK            = 42003 // PAUSE button has been clicked.
	MAV_CMD_DO_START_MAG_CAL                = 42424 // Initiate a magnetometer calibration
	MAV_CMD_DO_ACCEPT_MAG_CAL               = 42425 // Initiate a magnetometer calibration
	MAV_CMD_DO_CANCEL_MAG_CAL               = 42426 // Cancel a running magnetometer calibration
	MAV_CMD_ACCELCAL_VEHICLE_POS            = 42429 // Used when doing accelerometer calibration. When sent to the GCS tells it what position to put the vehicle in. When sent to the vehicle says what position the vehicle is in.
	MAV_CMD_DO_SEND_BANNER                  = 42428 // Reply with the version banner
	MAV_CMD_GIMBAL_RESET                    = 42501 // Causes the gimbal to reset and boot as if it was just powered on
	MAV_CMD_SET_FACTORY_TEST_MODE           = 42427 // Command autopilot to get into factory test/diagnostic mode
	MAV_CMD_GIMBAL_AXIS_CALIBRATION_STATUS  = 42502 // Reports progress and success or failure of gimbal axis calibration procedure
	MAV_CMD_GIMBAL_REQUEST_AXIS_CALIBRATION = 42503 // Starts commutation calibration on the gimbal
	MAV_CMD_GIMBAL_FULL_RESET               = 42505 // Erases gimbal application and parameters
)

// LimitsState (generated enum)
//
const (
	LIMITS_INIT       = 0 // pre-initialization
	LIMITS_DISABLED   = 1 // disabled
	LIMITS_ENABLED    = 2 // checking limits
	LIMITS_TRIGGERED  = 3 // a limit has been breached
	LIMITS_RECOVERING = 4 // taking action eg. RTL
	LIMITS_RECOVERED  = 5 // we're no longer in breach of a limit
)

// LimitModule (generated enum)
//
const (
	LIMIT_GPSLOCK  = 1 // pre-initialization
	LIMIT_GEOFENCE = 2 // disabled
	LIMIT_ALTITUDE = 4 // checking limits
)

// RallyFlags (generated enum)
// Flags in RALLY_POINT message
const (
	FAVORABLE_WIND   = 1 // Flag set when requiring favorable winds for landing.
	LAND_IMMEDIATELY = 2 // Flag set when plane is to immediately descend to break altitude and land without GCS intervention. Flag not set when plane is to loiter at Rally point until commanded to land.
)

// ParachuteAction (generated enum)
//
const (
	PARACHUTE_DISABLE = 0 // Disable parachute release
	PARACHUTE_ENABLE  = 1 // Enable parachute release
	PARACHUTE_RELEASE = 2 // Release parachute
)

// GripperActions (generated enum)
// Gripper actions.
const (
	GRIPPER_ACTION_RELEASE = 0 // gripper release of cargo
	GRIPPER_ACTION_GRAB    = 1 // gripper grabs onto cargo
)

// CameraStatusTypes (generated enum)
//
const (
	CAMERA_STATUS_TYPE_HEARTBEAT  = 0 // Camera heartbeat, announce camera component ID at 1hz
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
	GOPRO_CAPTURE_MODE_BURST      = 2   // Burst mode, hero 3+ only
	GOPRO_CAPTURE_MODE_TIME_LAPSE = 3   // Time lapse mode, hero 3+ only
	GOPRO_CAPTURE_MODE_MULTI_SHOT = 4   // Multi shot mode, hero 4 only
	GOPRO_CAPTURE_MODE_PLAYBACK   = 5   // Playback mode, hero 4 only, silver only except when LCD or HDMI is connected to black
	GOPRO_CAPTURE_MODE_SETUP      = 6   // Playback mode, hero 4 only
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
// Flags in EKF_STATUS message
const (
	EKF_ATTITUDE           = 1   // set if EKF's attitude estimate is good
	EKF_VELOCITY_HORIZ     = 2   // set if EKF's horizontal velocity estimate is good
	EKF_VELOCITY_VERT      = 4   // set if EKF's vertical velocity estimate is good
	EKF_POS_HORIZ_REL      = 8   // set if EKF's horizontal position (relative) estimate is good
	EKF_POS_HORIZ_ABS      = 16  // set if EKF's horizontal position (absolute) estimate is good
	EKF_POS_VERT_ABS       = 32  // set if EKF's vertical position (absolute) estimate is good
	EKF_POS_VERT_AGL       = 64  // set if EKF's vertical position (above ground) estimate is good
	EKF_CONST_POS_MODE     = 128 // EKF is in constant position mode and does not know it's absolute or relative position
	EKF_PRED_POS_HORIZ_REL = 256 // set if EKF's predicted horizontal position (relative) estimate is good
	EKF_PRED_POS_HORIZ_ABS = 512 // set if EKF's predicted horizontal position (absolute) estimate is good
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

// MagCalStatus (generated enum)
//
const (
	MAG_CAL_NOT_STARTED      = 0 //
	MAG_CAL_WAITING_TO_START = 1 //
	MAG_CAL_RUNNING_STEP_ONE = 2 //
	MAG_CAL_RUNNING_STEP_TWO = 3 //
	MAG_CAL_SUCCESS          = 4 //
	MAG_CAL_FAILED           = 5 //
)

// MavRemoteLogDataBlockCommands (generated enum)
// Special ACK block numbers control activation of dataflash log streaming
const (
	MAV_REMOTE_LOG_DATA_BLOCK_STOP  = 2147483645 // UAV to stop sending DataFlash blocks
	MAV_REMOTE_LOG_DATA_BLOCK_START = 2147483646 // UAV to start sending DataFlash blocks
)

// MavRemoteLogDataBlockStatuses (generated enum)
// Possible remote log data block statuses
const (
	MAV_REMOTE_LOG_DATA_BLOCK_NACK = 0 // This block has NOT been received
	MAV_REMOTE_LOG_DATA_BLOCK_ACK  = 1 // This block has been received
)

// DeviceOpBustype (generated enum)
// Bus types for device operations
const (
	DEVICE_OP_BUSTYPE_I2C = 0 // I2C Device operation
	DEVICE_OP_BUSTYPE_SPI = 1 // SPI Device operation
)

// SensorOffsets struct (generated typeinfo)
// Offsets and calibrations values for hardware sensors. This makes it easier to debug the calibration process.
type SensorOffsets struct {
	MagDeclination float32 // magnetic declination (radians)
	RawPress       int32   // raw pressure from barometer
	RawTemp        int32   // raw temperature from barometer
	GyroCalX       float32 // gyro X calibration
	GyroCalY       float32 // gyro Y calibration
	GyroCalZ       float32 // gyro Z calibration
	AccelCalX      float32 // accel X calibration
	AccelCalY      float32 // accel Y calibration
	AccelCalZ      float32 // accel Z calibration
	MagOfsX        int16   // magnetometer X offset
	MagOfsY        int16   // magnetometer Y offset
	MagOfsZ        int16   // magnetometer Z offset
}

// MsgID (generated function)
func (m *SensorOffsets) MsgID() MessageID {
	return MSG_ID_SENSOR_OFFSETS
}

// MsgName (generated function)
func (m *SensorOffsets) MsgName() string {
	return "SensorOffsets"
}

// Pack (generated function)
func (m *SensorOffsets) Pack(p *Packet) error {
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
func (m *SensorOffsets) Unpack(p *Packet) error {
	if len(p.Payload) < 42 {
		return fmt.Errorf("payload too small")
	}
	m.MagDeclination = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.RawPress = int32(binary.LittleEndian.Uint32(p.Payload[4:]))
	m.RawTemp = int32(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.GyroCalX = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	m.GyroCalY = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	m.GyroCalZ = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	m.AccelCalX = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	m.AccelCalY = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	m.AccelCalZ = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[32:]))
	m.MagOfsX = int16(binary.LittleEndian.Uint16(p.Payload[36:]))
	m.MagOfsY = int16(binary.LittleEndian.Uint16(p.Payload[38:]))
	m.MagOfsZ = int16(binary.LittleEndian.Uint16(p.Payload[40:]))
	return nil
}

// SetMagOffsets struct (generated typeinfo)
// Deprecated. Use MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS instead. Set the magnetometer offsets
type SetMagOffsets struct {
	MagOfsX         int16 // magnetometer X offset
	MagOfsY         int16 // magnetometer Y offset
	MagOfsZ         int16 // magnetometer Z offset
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
}

// MsgID (generated function)
func (m *SetMagOffsets) MsgID() MessageID {
	return MSG_ID_SET_MAG_OFFSETS
}

// MsgName (generated function)
func (m *SetMagOffsets) MsgName() string {
	return "SetMagOffsets"
}

// Pack (generated function)
func (m *SetMagOffsets) Pack(p *Packet) error {
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
func (m *SetMagOffsets) Unpack(p *Packet) error {
	if len(p.Payload) < 8 {
		return fmt.Errorf("payload too small")
	}
	m.MagOfsX = int16(binary.LittleEndian.Uint16(p.Payload[0:]))
	m.MagOfsY = int16(binary.LittleEndian.Uint16(p.Payload[2:]))
	m.MagOfsZ = int16(binary.LittleEndian.Uint16(p.Payload[4:]))
	m.TargetSystem = uint8(p.Payload[6])
	m.TargetComponent = uint8(p.Payload[7])
	return nil
}

// Meminfo struct (generated typeinfo)
// state of APM memory
type Meminfo struct {
	Brkval  uint16 // heap top
	Freemem uint16 // free memory
}

// MsgID (generated function)
func (m *Meminfo) MsgID() MessageID {
	return MSG_ID_MEMINFO
}

// MsgName (generated function)
func (m *Meminfo) MsgName() string {
	return "Meminfo"
}

// Pack (generated function)
func (m *Meminfo) Pack(p *Packet) error {
	payload := make([]byte, 4)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.Brkval))
	binary.LittleEndian.PutUint16(payload[2:], uint16(m.Freemem))

	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *Meminfo) Unpack(p *Packet) error {
	if len(p.Payload) < 4 {
		return fmt.Errorf("payload too small")
	}
	m.Brkval = uint16(binary.LittleEndian.Uint16(p.Payload[0:]))
	m.Freemem = uint16(binary.LittleEndian.Uint16(p.Payload[2:]))
	return nil
}

// ApAdc struct (generated typeinfo)
// raw ADC output
type ApAdc struct {
	Adc1 uint16 // ADC output 1
	Adc2 uint16 // ADC output 2
	Adc3 uint16 // ADC output 3
	Adc4 uint16 // ADC output 4
	Adc5 uint16 // ADC output 5
	Adc6 uint16 // ADC output 6
}

// MsgID (generated function)
func (m *ApAdc) MsgID() MessageID {
	return MSG_ID_AP_ADC
}

// MsgName (generated function)
func (m *ApAdc) MsgName() string {
	return "ApAdc"
}

// Pack (generated function)
func (m *ApAdc) Pack(p *Packet) error {
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
func (m *ApAdc) Unpack(p *Packet) error {
	if len(p.Payload) < 12 {
		return fmt.Errorf("payload too small")
	}
	m.Adc1 = uint16(binary.LittleEndian.Uint16(p.Payload[0:]))
	m.Adc2 = uint16(binary.LittleEndian.Uint16(p.Payload[2:]))
	m.Adc3 = uint16(binary.LittleEndian.Uint16(p.Payload[4:]))
	m.Adc4 = uint16(binary.LittleEndian.Uint16(p.Payload[6:]))
	m.Adc5 = uint16(binary.LittleEndian.Uint16(p.Payload[8:]))
	m.Adc6 = uint16(binary.LittleEndian.Uint16(p.Payload[10:]))
	return nil
}

// DigicamConfigure struct (generated typeinfo)
// Configure on-board Camera Control System.
type DigicamConfigure struct {
	ExtraValue      float32 // Correspondent value to given extra_param
	ShutterSpeed    uint16  // Divisor number //e.g. 1000 means 1/1000 (0 means ignore)
	TargetSystem    uint8   // System ID
	TargetComponent uint8   // Component ID
	Mode            uint8   // Mode enumeration from 1 to N //P, TV, AV, M, Etc (0 means ignore)
	Aperture        uint8   // F stop number x 10 //e.g. 28 means 2.8 (0 means ignore)
	Iso             uint8   // ISO enumeration from 1 to N //e.g. 80, 100, 200, Etc (0 means ignore)
	ExposureType    uint8   // Exposure type enumeration from 1 to N (0 means ignore)
	CommandID       uint8   // Command Identity (incremental loop: 0 to 255)//A command sent multiple times will be executed or pooled just once
	EngineCutOff    uint8   // Main engine cut-off time before camera trigger in seconds/10 (0 means no cut-off)
	ExtraParam      uint8   // Extra parameters enumeration (0 means ignore)
}

// MsgID (generated function)
func (m *DigicamConfigure) MsgID() MessageID {
	return MSG_ID_DIGICAM_CONFIGURE
}

// MsgName (generated function)
func (m *DigicamConfigure) MsgName() string {
	return "DigicamConfigure"
}

// Pack (generated function)
func (m *DigicamConfigure) Pack(p *Packet) error {
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
func (m *DigicamConfigure) Unpack(p *Packet) error {
	if len(p.Payload) < 15 {
		return fmt.Errorf("payload too small")
	}
	m.ExtraValue = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.ShutterSpeed = uint16(binary.LittleEndian.Uint16(p.Payload[4:]))
	m.TargetSystem = uint8(p.Payload[6])
	m.TargetComponent = uint8(p.Payload[7])
	m.Mode = uint8(p.Payload[8])
	m.Aperture = uint8(p.Payload[9])
	m.Iso = uint8(p.Payload[10])
	m.ExposureType = uint8(p.Payload[11])
	m.CommandID = uint8(p.Payload[12])
	m.EngineCutOff = uint8(p.Payload[13])
	m.ExtraParam = uint8(p.Payload[14])
	return nil
}

// DigicamControl struct (generated typeinfo)
// Control on-board Camera Control System to take shots.
type DigicamControl struct {
	ExtraValue      float32 // Correspondent value to given extra_param
	TargetSystem    uint8   // System ID
	TargetComponent uint8   // Component ID
	Session         uint8   // 0: stop, 1: start or keep it up //Session control e.g. show/hide lens
	ZoomPos         uint8   // 1 to N //Zoom's absolute position (0 means ignore)
	ZoomStep        int8    // -100 to 100 //Zooming step value to offset zoom from the current position
	FocusLock       uint8   // 0: unlock focus or keep unlocked, 1: lock focus or keep locked, 3: re-lock focus
	Shot            uint8   // 0: ignore, 1: shot or start filming
	CommandID       uint8   // Command Identity (incremental loop: 0 to 255)//A command sent multiple times will be executed or pooled just once
	ExtraParam      uint8   // Extra parameters enumeration (0 means ignore)
}

// MsgID (generated function)
func (m *DigicamControl) MsgID() MessageID {
	return MSG_ID_DIGICAM_CONTROL
}

// MsgName (generated function)
func (m *DigicamControl) MsgName() string {
	return "DigicamControl"
}

// Pack (generated function)
func (m *DigicamControl) Pack(p *Packet) error {
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
func (m *DigicamControl) Unpack(p *Packet) error {
	if len(p.Payload) < 13 {
		return fmt.Errorf("payload too small")
	}
	m.ExtraValue = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.TargetSystem = uint8(p.Payload[4])
	m.TargetComponent = uint8(p.Payload[5])
	m.Session = uint8(p.Payload[6])
	m.ZoomPos = uint8(p.Payload[7])
	m.ZoomStep = int8(p.Payload[8])
	m.FocusLock = uint8(p.Payload[9])
	m.Shot = uint8(p.Payload[10])
	m.CommandID = uint8(p.Payload[11])
	m.ExtraParam = uint8(p.Payload[12])
	return nil
}

// MountConfigure struct (generated typeinfo)
// Message to configure a camera mount, directional antenna, etc.
type MountConfigure struct {
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
	MountMode       uint8 // mount operating mode (see MAV_MOUNT_MODE enum)
	StabRoll        uint8 // (1 = yes, 0 = no)
	StabPitch       uint8 // (1 = yes, 0 = no)
	StabYaw         uint8 // (1 = yes, 0 = no)
}

// MsgID (generated function)
func (m *MountConfigure) MsgID() MessageID {
	return MSG_ID_MOUNT_CONFIGURE
}

// MsgName (generated function)
func (m *MountConfigure) MsgName() string {
	return "MountConfigure"
}

// Pack (generated function)
func (m *MountConfigure) Pack(p *Packet) error {
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
func (m *MountConfigure) Unpack(p *Packet) error {
	if len(p.Payload) < 6 {
		return fmt.Errorf("payload too small")
	}
	m.TargetSystem = uint8(p.Payload[0])
	m.TargetComponent = uint8(p.Payload[1])
	m.MountMode = uint8(p.Payload[2])
	m.StabRoll = uint8(p.Payload[3])
	m.StabPitch = uint8(p.Payload[4])
	m.StabYaw = uint8(p.Payload[5])
	return nil
}

// MountControl struct (generated typeinfo)
// Message to control a camera mount, directional antenna, etc.
type MountControl struct {
	InputA          int32 // pitch(deg*100) or lat, depending on mount mode
	InputB          int32 // roll(deg*100) or lon depending on mount mode
	InputC          int32 // yaw(deg*100) or alt (in cm) depending on mount mode
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
	SavePosition    uint8 // if "1" it will save current trimmed position on EEPROM (just valid for NEUTRAL and LANDING)
}

// MsgID (generated function)
func (m *MountControl) MsgID() MessageID {
	return MSG_ID_MOUNT_CONTROL
}

// MsgName (generated function)
func (m *MountControl) MsgName() string {
	return "MountControl"
}

// Pack (generated function)
func (m *MountControl) Pack(p *Packet) error {
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
func (m *MountControl) Unpack(p *Packet) error {
	if len(p.Payload) < 15 {
		return fmt.Errorf("payload too small")
	}
	m.InputA = int32(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.InputB = int32(binary.LittleEndian.Uint32(p.Payload[4:]))
	m.InputC = int32(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.TargetSystem = uint8(p.Payload[12])
	m.TargetComponent = uint8(p.Payload[13])
	m.SavePosition = uint8(p.Payload[14])
	return nil
}

// MountStatus struct (generated typeinfo)
// Message with some status from APM to GCS about camera or antenna mount
type MountStatus struct {
	PointingA       int32 // pitch(deg*100)
	PointingB       int32 // roll(deg*100)
	PointingC       int32 // yaw(deg*100)
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
}

// MsgID (generated function)
func (m *MountStatus) MsgID() MessageID {
	return MSG_ID_MOUNT_STATUS
}

// MsgName (generated function)
func (m *MountStatus) MsgName() string {
	return "MountStatus"
}

// Pack (generated function)
func (m *MountStatus) Pack(p *Packet) error {
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
func (m *MountStatus) Unpack(p *Packet) error {
	if len(p.Payload) < 14 {
		return fmt.Errorf("payload too small")
	}
	m.PointingA = int32(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.PointingB = int32(binary.LittleEndian.Uint32(p.Payload[4:]))
	m.PointingC = int32(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.TargetSystem = uint8(p.Payload[12])
	m.TargetComponent = uint8(p.Payload[13])
	return nil
}

// FencePoint struct (generated typeinfo)
// A fence point. Used to set a point when from GCS -> MAV. Also used to return a point from MAV -> GCS
type FencePoint struct {
	Lat             float32 // Latitude of point
	Lng             float32 // Longitude of point
	TargetSystem    uint8   // System ID
	TargetComponent uint8   // Component ID
	Idx             uint8   // point index (first point is 1, 0 is for return point)
	Count           uint8   // total number of points (for sanity checking)
}

// MsgID (generated function)
func (m *FencePoint) MsgID() MessageID {
	return MSG_ID_FENCE_POINT
}

// MsgName (generated function)
func (m *FencePoint) MsgName() string {
	return "FencePoint"
}

// Pack (generated function)
func (m *FencePoint) Pack(p *Packet) error {
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
func (m *FencePoint) Unpack(p *Packet) error {
	if len(p.Payload) < 12 {
		return fmt.Errorf("payload too small")
	}
	m.Lat = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.Lng = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	m.TargetSystem = uint8(p.Payload[8])
	m.TargetComponent = uint8(p.Payload[9])
	m.Idx = uint8(p.Payload[10])
	m.Count = uint8(p.Payload[11])
	return nil
}

// FenceFetchPoint struct (generated typeinfo)
// Request a current fence point from MAV
type FenceFetchPoint struct {
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
	Idx             uint8 // point index (first point is 1, 0 is for return point)
}

// MsgID (generated function)
func (m *FenceFetchPoint) MsgID() MessageID {
	return MSG_ID_FENCE_FETCH_POINT
}

// MsgName (generated function)
func (m *FenceFetchPoint) MsgName() string {
	return "FenceFetchPoint"
}

// Pack (generated function)
func (m *FenceFetchPoint) Pack(p *Packet) error {
	payload := make([]byte, 3)
	payload[0] = byte(m.TargetSystem)
	payload[1] = byte(m.TargetComponent)
	payload[2] = byte(m.Idx)

	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *FenceFetchPoint) Unpack(p *Packet) error {
	if len(p.Payload) < 3 {
		return fmt.Errorf("payload too small")
	}
	m.TargetSystem = uint8(p.Payload[0])
	m.TargetComponent = uint8(p.Payload[1])
	m.Idx = uint8(p.Payload[2])
	return nil
}

// FenceStatus struct (generated typeinfo)
// Status of geo-fencing. Sent in extended status stream when fencing enabled
type FenceStatus struct {
	BreachTime   uint32 // time of last breach in milliseconds since boot
	BreachCount  uint16 // number of fence breaches
	BreachStatus uint8  // 0 if currently inside fence, 1 if outside
	BreachType   uint8  // last breach type (see FENCE_BREACH_* enum)
}

// MsgID (generated function)
func (m *FenceStatus) MsgID() MessageID {
	return MSG_ID_FENCE_STATUS
}

// MsgName (generated function)
func (m *FenceStatus) MsgName() string {
	return "FenceStatus"
}

// Pack (generated function)
func (m *FenceStatus) Pack(p *Packet) error {
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
func (m *FenceStatus) Unpack(p *Packet) error {
	if len(p.Payload) < 8 {
		return fmt.Errorf("payload too small")
	}
	m.BreachTime = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.BreachCount = uint16(binary.LittleEndian.Uint16(p.Payload[4:]))
	m.BreachStatus = uint8(p.Payload[6])
	m.BreachType = uint8(p.Payload[7])
	return nil
}

// Ahrs struct (generated typeinfo)
// Status of DCM attitude estimator
type Ahrs struct {
	Omegaix     float32 // X gyro drift estimate rad/s
	Omegaiy     float32 // Y gyro drift estimate rad/s
	Omegaiz     float32 // Z gyro drift estimate rad/s
	AccelWeight float32 // average accel_weight
	RenormVal   float32 // average renormalisation value
	ErrorRp     float32 // average error_roll_pitch value
	ErrorYaw    float32 // average error_yaw value
}

// MsgID (generated function)
func (m *Ahrs) MsgID() MessageID {
	return MSG_ID_AHRS
}

// MsgName (generated function)
func (m *Ahrs) MsgName() string {
	return "Ahrs"
}

// Pack (generated function)
func (m *Ahrs) Pack(p *Packet) error {
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
func (m *Ahrs) Unpack(p *Packet) error {
	if len(p.Payload) < 28 {
		return fmt.Errorf("payload too small")
	}
	m.Omegaix = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.Omegaiy = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	m.Omegaiz = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.AccelWeight = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	m.RenormVal = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	m.ErrorRp = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	m.ErrorYaw = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	return nil
}

// Simstate struct (generated typeinfo)
// Status of simulation environment, if used
type Simstate struct {
	Roll  float32 // Roll angle (rad)
	Pitch float32 // Pitch angle (rad)
	Yaw   float32 // Yaw angle (rad)
	Xacc  float32 // X acceleration m/s/s
	Yacc  float32 // Y acceleration m/s/s
	Zacc  float32 // Z acceleration m/s/s
	Xgyro float32 // Angular speed around X axis rad/s
	Ygyro float32 // Angular speed around Y axis rad/s
	Zgyro float32 // Angular speed around Z axis rad/s
	Lat   int32   // Latitude in degrees * 1E7
	Lng   int32   // Longitude in degrees * 1E7
}

// MsgID (generated function)
func (m *Simstate) MsgID() MessageID {
	return MSG_ID_SIMSTATE
}

// MsgName (generated function)
func (m *Simstate) MsgName() string {
	return "Simstate"
}

// Pack (generated function)
func (m *Simstate) Pack(p *Packet) error {
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
func (m *Simstate) Unpack(p *Packet) error {
	if len(p.Payload) < 44 {
		return fmt.Errorf("payload too small")
	}
	m.Roll = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.Pitch = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	m.Yaw = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.Xacc = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	m.Yacc = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	m.Zacc = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	m.Xgyro = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	m.Ygyro = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	m.Zgyro = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[32:]))
	m.Lat = int32(binary.LittleEndian.Uint32(p.Payload[36:]))
	m.Lng = int32(binary.LittleEndian.Uint32(p.Payload[40:]))
	return nil
}

// Hwstatus struct (generated typeinfo)
// Status of key hardware
type Hwstatus struct {
	Vcc    uint16 // board voltage (mV)
	I2cerr uint8  // I2C error count
}

// MsgID (generated function)
func (m *Hwstatus) MsgID() MessageID {
	return MSG_ID_HWSTATUS
}

// MsgName (generated function)
func (m *Hwstatus) MsgName() string {
	return "Hwstatus"
}

// Pack (generated function)
func (m *Hwstatus) Pack(p *Packet) error {
	payload := make([]byte, 3)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.Vcc))
	payload[2] = byte(m.I2cerr)

	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *Hwstatus) Unpack(p *Packet) error {
	if len(p.Payload) < 3 {
		return fmt.Errorf("payload too small")
	}
	m.Vcc = uint16(binary.LittleEndian.Uint16(p.Payload[0:]))
	m.I2cerr = uint8(p.Payload[2])
	return nil
}

// Radio struct (generated typeinfo)
// Status generated by radio
type Radio struct {
	Rxerrors uint16 // receive errors
	Fixed    uint16 // count of error corrected packets
	Rssi     uint8  // local signal strength
	Remrssi  uint8  // remote signal strength
	Txbuf    uint8  // how full the tx buffer is as a percentage
	Noise    uint8  // background noise level
	Remnoise uint8  // remote background noise level
}

// MsgID (generated function)
func (m *Radio) MsgID() MessageID {
	return MSG_ID_RADIO
}

// MsgName (generated function)
func (m *Radio) MsgName() string {
	return "Radio"
}

// Pack (generated function)
func (m *Radio) Pack(p *Packet) error {
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
func (m *Radio) Unpack(p *Packet) error {
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

// LimitsStatus struct (generated typeinfo)
// Status of AP_Limits. Sent in extended status stream when AP_Limits is enabled
type LimitsStatus struct {
	LastTrigger   uint32 // time of last breach in milliseconds since boot
	LastAction    uint32 // time of last recovery action in milliseconds since boot
	LastRecovery  uint32 // time of last successful recovery in milliseconds since boot
	LastClear     uint32 // time of last all-clear in milliseconds since boot
	BreachCount   uint16 // number of fence breaches
	LimitsState   uint8  // state of AP_Limits, (see enum LimitState, LIMITS_STATE)
	ModsEnabled   uint8  // AP_Limit_Module bitfield of enabled modules, (see enum moduleid or LIMIT_MODULE)
	ModsRequired  uint8  // AP_Limit_Module bitfield of required modules, (see enum moduleid or LIMIT_MODULE)
	ModsTriggered uint8  // AP_Limit_Module bitfield of triggered modules, (see enum moduleid or LIMIT_MODULE)
}

// MsgID (generated function)
func (m *LimitsStatus) MsgID() MessageID {
	return MSG_ID_LIMITS_STATUS
}

// MsgName (generated function)
func (m *LimitsStatus) MsgName() string {
	return "LimitsStatus"
}

// Pack (generated function)
func (m *LimitsStatus) Pack(p *Packet) error {
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
func (m *LimitsStatus) Unpack(p *Packet) error {
	if len(p.Payload) < 22 {
		return fmt.Errorf("payload too small")
	}
	m.LastTrigger = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.LastAction = uint32(binary.LittleEndian.Uint32(p.Payload[4:]))
	m.LastRecovery = uint32(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.LastClear = uint32(binary.LittleEndian.Uint32(p.Payload[12:]))
	m.BreachCount = uint16(binary.LittleEndian.Uint16(p.Payload[16:]))
	m.LimitsState = uint8(p.Payload[18])
	m.ModsEnabled = uint8(p.Payload[19])
	m.ModsRequired = uint8(p.Payload[20])
	m.ModsTriggered = uint8(p.Payload[21])
	return nil
}

// Wind struct (generated typeinfo)
// Wind estimation
type Wind struct {
	Direction float32 // wind direction that wind is coming from (degrees)
	Speed     float32 // wind speed in ground plane (m/s)
	SpeedZ    float32 // vertical wind speed (m/s)
}

// MsgID (generated function)
func (m *Wind) MsgID() MessageID {
	return MSG_ID_WIND
}

// MsgName (generated function)
func (m *Wind) MsgName() string {
	return "Wind"
}

// Pack (generated function)
func (m *Wind) Pack(p *Packet) error {
	payload := make([]byte, 12)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(m.Direction))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(m.Speed))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.SpeedZ))

	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *Wind) Unpack(p *Packet) error {
	if len(p.Payload) < 12 {
		return fmt.Errorf("payload too small")
	}
	m.Direction = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.Speed = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	m.SpeedZ = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	return nil
}

// Data16 struct (generated typeinfo)
// Data packet, size 16
type Data16 struct {
	Type uint8     // data type
	Len  uint8     // data length
	Data [16]uint8 // raw data
}

// MsgID (generated function)
func (m *Data16) MsgID() MessageID {
	return MSG_ID_DATA16
}

// MsgName (generated function)
func (m *Data16) MsgName() string {
	return "Data16"
}

// Pack (generated function)
func (m *Data16) Pack(p *Packet) error {
	payload := make([]byte, 18)
	payload[0] = byte(m.Type)
	payload[1] = byte(m.Len)
	copy(payload[2:], m.Data[:])

	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *Data16) Unpack(p *Packet) error {
	if len(p.Payload) < 18 {
		return fmt.Errorf("payload too small")
	}
	m.Type = uint8(p.Payload[0])
	m.Len = uint8(p.Payload[1])
	copy(m.Data[:], p.Payload[2:18])
	return nil
}

// Data32 struct (generated typeinfo)
// Data packet, size 32
type Data32 struct {
	Type uint8     // data type
	Len  uint8     // data length
	Data [32]uint8 // raw data
}

// MsgID (generated function)
func (m *Data32) MsgID() MessageID {
	return MSG_ID_DATA32
}

// MsgName (generated function)
func (m *Data32) MsgName() string {
	return "Data32"
}

// Pack (generated function)
func (m *Data32) Pack(p *Packet) error {
	payload := make([]byte, 34)
	payload[0] = byte(m.Type)
	payload[1] = byte(m.Len)
	copy(payload[2:], m.Data[:])

	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *Data32) Unpack(p *Packet) error {
	if len(p.Payload) < 34 {
		return fmt.Errorf("payload too small")
	}
	m.Type = uint8(p.Payload[0])
	m.Len = uint8(p.Payload[1])
	copy(m.Data[:], p.Payload[2:34])
	return nil
}

// Data64 struct (generated typeinfo)
// Data packet, size 64
type Data64 struct {
	Type uint8     // data type
	Len  uint8     // data length
	Data [64]uint8 // raw data
}

// MsgID (generated function)
func (m *Data64) MsgID() MessageID {
	return MSG_ID_DATA64
}

// MsgName (generated function)
func (m *Data64) MsgName() string {
	return "Data64"
}

// Pack (generated function)
func (m *Data64) Pack(p *Packet) error {
	payload := make([]byte, 66)
	payload[0] = byte(m.Type)
	payload[1] = byte(m.Len)
	copy(payload[2:], m.Data[:])

	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *Data64) Unpack(p *Packet) error {
	if len(p.Payload) < 66 {
		return fmt.Errorf("payload too small")
	}
	m.Type = uint8(p.Payload[0])
	m.Len = uint8(p.Payload[1])
	copy(m.Data[:], p.Payload[2:66])
	return nil
}

// Data96 struct (generated typeinfo)
// Data packet, size 96
type Data96 struct {
	Type uint8     // data type
	Len  uint8     // data length
	Data [96]uint8 // raw data
}

// MsgID (generated function)
func (m *Data96) MsgID() MessageID {
	return MSG_ID_DATA96
}

// MsgName (generated function)
func (m *Data96) MsgName() string {
	return "Data96"
}

// Pack (generated function)
func (m *Data96) Pack(p *Packet) error {
	payload := make([]byte, 98)
	payload[0] = byte(m.Type)
	payload[1] = byte(m.Len)
	copy(payload[2:], m.Data[:])

	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *Data96) Unpack(p *Packet) error {
	if len(p.Payload) < 98 {
		return fmt.Errorf("payload too small")
	}
	m.Type = uint8(p.Payload[0])
	m.Len = uint8(p.Payload[1])
	copy(m.Data[:], p.Payload[2:98])
	return nil
}

// Rangefinder struct (generated typeinfo)
// Rangefinder reporting
type Rangefinder struct {
	Distance float32 // distance in meters
	Voltage  float32 // raw voltage if available, zero otherwise
}

// MsgID (generated function)
func (m *Rangefinder) MsgID() MessageID {
	return MSG_ID_RANGEFINDER
}

// MsgName (generated function)
func (m *Rangefinder) MsgName() string {
	return "Rangefinder"
}

// Pack (generated function)
func (m *Rangefinder) Pack(p *Packet) error {
	payload := make([]byte, 8)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(m.Distance))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(m.Voltage))

	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *Rangefinder) Unpack(p *Packet) error {
	if len(p.Payload) < 8 {
		return fmt.Errorf("payload too small")
	}
	m.Distance = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.Voltage = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	return nil
}

// AirspeedAutocal struct (generated typeinfo)
// Airspeed auto-calibration
type AirspeedAutocal struct {
	Vx           float32 // GPS velocity north m/s
	Vy           float32 // GPS velocity east m/s
	Vz           float32 // GPS velocity down m/s
	DiffPressure float32 // Differential pressure pascals
	Eas2tas      float32 // Estimated to true airspeed ratio
	Ratio        float32 // Airspeed ratio
	StateX       float32 // EKF state x
	StateY       float32 // EKF state y
	StateZ       float32 // EKF state z
	Pax          float32 // EKF Pax
	Pby          float32 // EKF Pby
	Pcz          float32 // EKF Pcz
}

// MsgID (generated function)
func (m *AirspeedAutocal) MsgID() MessageID {
	return MSG_ID_AIRSPEED_AUTOCAL
}

// MsgName (generated function)
func (m *AirspeedAutocal) MsgName() string {
	return "AirspeedAutocal"
}

// Pack (generated function)
func (m *AirspeedAutocal) Pack(p *Packet) error {
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
func (m *AirspeedAutocal) Unpack(p *Packet) error {
	if len(p.Payload) < 48 {
		return fmt.Errorf("payload too small")
	}
	m.Vx = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.Vy = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	m.Vz = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.DiffPressure = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	m.Eas2tas = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	m.Ratio = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	m.StateX = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	m.StateY = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	m.StateZ = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[32:]))
	m.Pax = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[36:]))
	m.Pby = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[40:]))
	m.Pcz = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[44:]))
	return nil
}

// RallyPoint struct (generated typeinfo)
// A rally point. Used to set a point when from GCS -> MAV. Also used to return a point from MAV -> GCS
type RallyPoint struct {
	Lat             int32  // Latitude of point in degrees * 1E7
	Lng             int32  // Longitude of point in degrees * 1E7
	Alt             int16  // Transit / loiter altitude in meters relative to home
	BreakAlt        int16  // Break altitude in meters relative to home
	LandDir         uint16 // Heading to aim for when landing. In centi-degrees.
	TargetSystem    uint8  // System ID
	TargetComponent uint8  // Component ID
	Idx             uint8  // point index (first point is 0)
	Count           uint8  // total number of points (for sanity checking)
	Flags           uint8  // See RALLY_FLAGS enum for definition of the bitmask.
}

// MsgID (generated function)
func (m *RallyPoint) MsgID() MessageID {
	return MSG_ID_RALLY_POINT
}

// MsgName (generated function)
func (m *RallyPoint) MsgName() string {
	return "RallyPoint"
}

// Pack (generated function)
func (m *RallyPoint) Pack(p *Packet) error {
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
func (m *RallyPoint) Unpack(p *Packet) error {
	if len(p.Payload) < 19 {
		return fmt.Errorf("payload too small")
	}
	m.Lat = int32(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.Lng = int32(binary.LittleEndian.Uint32(p.Payload[4:]))
	m.Alt = int16(binary.LittleEndian.Uint16(p.Payload[8:]))
	m.BreakAlt = int16(binary.LittleEndian.Uint16(p.Payload[10:]))
	m.LandDir = uint16(binary.LittleEndian.Uint16(p.Payload[12:]))
	m.TargetSystem = uint8(p.Payload[14])
	m.TargetComponent = uint8(p.Payload[15])
	m.Idx = uint8(p.Payload[16])
	m.Count = uint8(p.Payload[17])
	m.Flags = uint8(p.Payload[18])
	return nil
}

// RallyFetchPoint struct (generated typeinfo)
// Request a current rally point from MAV. MAV should respond with a RALLY_POINT message. MAV should not respond if the request is invalid.
type RallyFetchPoint struct {
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
	Idx             uint8 // point index (first point is 0)
}

// MsgID (generated function)
func (m *RallyFetchPoint) MsgID() MessageID {
	return MSG_ID_RALLY_FETCH_POINT
}

// MsgName (generated function)
func (m *RallyFetchPoint) MsgName() string {
	return "RallyFetchPoint"
}

// Pack (generated function)
func (m *RallyFetchPoint) Pack(p *Packet) error {
	payload := make([]byte, 3)
	payload[0] = byte(m.TargetSystem)
	payload[1] = byte(m.TargetComponent)
	payload[2] = byte(m.Idx)

	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *RallyFetchPoint) Unpack(p *Packet) error {
	if len(p.Payload) < 3 {
		return fmt.Errorf("payload too small")
	}
	m.TargetSystem = uint8(p.Payload[0])
	m.TargetComponent = uint8(p.Payload[1])
	m.Idx = uint8(p.Payload[2])
	return nil
}

// CompassmotStatus struct (generated typeinfo)
// Status of compassmot calibration
type CompassmotStatus struct {
	Current       float32 // current (Ampere)
	Compensationx float32 // Motor Compensation X
	Compensationy float32 // Motor Compensation Y
	Compensationz float32 // Motor Compensation Z
	Throttle      uint16  // throttle (percent*10)
	Interference  uint16  // interference (percent)
}

// MsgID (generated function)
func (m *CompassmotStatus) MsgID() MessageID {
	return MSG_ID_COMPASSMOT_STATUS
}

// MsgName (generated function)
func (m *CompassmotStatus) MsgName() string {
	return "CompassmotStatus"
}

// Pack (generated function)
func (m *CompassmotStatus) Pack(p *Packet) error {
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
func (m *CompassmotStatus) Unpack(p *Packet) error {
	if len(p.Payload) < 20 {
		return fmt.Errorf("payload too small")
	}
	m.Current = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.Compensationx = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	m.Compensationy = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.Compensationz = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	m.Throttle = uint16(binary.LittleEndian.Uint16(p.Payload[16:]))
	m.Interference = uint16(binary.LittleEndian.Uint16(p.Payload[18:]))
	return nil
}

// Ahrs2 struct (generated typeinfo)
// Status of secondary AHRS filter if available
type Ahrs2 struct {
	Roll     float32 // Roll angle (rad)
	Pitch    float32 // Pitch angle (rad)
	Yaw      float32 // Yaw angle (rad)
	Altitude float32 // Altitude (MSL)
	Lat      int32   // Latitude in degrees * 1E7
	Lng      int32   // Longitude in degrees * 1E7
}

// MsgID (generated function)
func (m *Ahrs2) MsgID() MessageID {
	return MSG_ID_AHRS2
}

// MsgName (generated function)
func (m *Ahrs2) MsgName() string {
	return "Ahrs2"
}

// Pack (generated function)
func (m *Ahrs2) Pack(p *Packet) error {
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
func (m *Ahrs2) Unpack(p *Packet) error {
	if len(p.Payload) < 24 {
		return fmt.Errorf("payload too small")
	}
	m.Roll = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.Pitch = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	m.Yaw = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.Altitude = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	m.Lat = int32(binary.LittleEndian.Uint32(p.Payload[16:]))
	m.Lng = int32(binary.LittleEndian.Uint32(p.Payload[20:]))
	return nil
}

// CameraStatus struct (generated typeinfo)
// Camera Event
type CameraStatus struct {
	TimeUsec     uint64  // Image timestamp (microseconds since UNIX epoch, according to camera clock)
	P1           float32 // Parameter 1 (meaning depends on event, see CAMERA_STATUS_TYPES enum)
	P2           float32 // Parameter 2 (meaning depends on event, see CAMERA_STATUS_TYPES enum)
	P3           float32 // Parameter 3 (meaning depends on event, see CAMERA_STATUS_TYPES enum)
	P4           float32 // Parameter 4 (meaning depends on event, see CAMERA_STATUS_TYPES enum)
	ImgIdx       uint16  // Image index
	TargetSystem uint8   // System ID
	CamIdx       uint8   // Camera ID
	EventID      uint8   // See CAMERA_STATUS_TYPES enum for definition of the bitmask
}

// MsgID (generated function)
func (m *CameraStatus) MsgID() MessageID {
	return MSG_ID_CAMERA_STATUS
}

// MsgName (generated function)
func (m *CameraStatus) MsgName() string {
	return "CameraStatus"
}

// Pack (generated function)
func (m *CameraStatus) Pack(p *Packet) error {
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
func (m *CameraStatus) Unpack(p *Packet) error {
	if len(p.Payload) < 29 {
		return fmt.Errorf("payload too small")
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	m.P1 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.P2 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	m.P3 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	m.P4 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	m.ImgIdx = uint16(binary.LittleEndian.Uint16(p.Payload[24:]))
	m.TargetSystem = uint8(p.Payload[26])
	m.CamIdx = uint8(p.Payload[27])
	m.EventID = uint8(p.Payload[28])
	return nil
}

// CameraFeedback struct (generated typeinfo)
// Camera Capture Feedback
type CameraFeedback struct {
	TimeUsec     uint64  // Image timestamp (microseconds since UNIX epoch), as passed in by CAMERA_STATUS message (or autopilot if no CCB)
	Lat          int32   // Latitude in (deg * 1E7)
	Lng          int32   // Longitude in (deg * 1E7)
	AltMsl       float32 // Altitude Absolute (meters AMSL)
	AltRel       float32 // Altitude Relative (meters above HOME location)
	Roll         float32 // Camera Roll angle (earth frame, degrees, +-180)
	Pitch        float32 // Camera Pitch angle (earth frame, degrees, +-180)
	Yaw          float32 // Camera Yaw (earth frame, degrees, 0-360, true)
	FocLen       float32 // Focal Length (mm)
	ImgIdx       uint16  // Image index
	TargetSystem uint8   // System ID
	CamIdx       uint8   // Camera ID
	Flags        uint8   // See CAMERA_FEEDBACK_FLAGS enum for definition of the bitmask
}

// MsgID (generated function)
func (m *CameraFeedback) MsgID() MessageID {
	return MSG_ID_CAMERA_FEEDBACK
}

// MsgName (generated function)
func (m *CameraFeedback) MsgName() string {
	return "CameraFeedback"
}

// Pack (generated function)
func (m *CameraFeedback) Pack(p *Packet) error {
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
func (m *CameraFeedback) Unpack(p *Packet) error {
	if len(p.Payload) < 45 {
		return fmt.Errorf("payload too small")
	}
	m.TimeUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	m.Lat = int32(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.Lng = int32(binary.LittleEndian.Uint32(p.Payload[12:]))
	m.AltMsl = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	m.AltRel = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	m.Roll = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	m.Pitch = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	m.Yaw = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[32:]))
	m.FocLen = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[36:]))
	m.ImgIdx = uint16(binary.LittleEndian.Uint16(p.Payload[40:]))
	m.TargetSystem = uint8(p.Payload[42])
	m.CamIdx = uint8(p.Payload[43])
	m.Flags = uint8(p.Payload[44])
	return nil
}

// Battery2 struct (generated typeinfo)
// 2nd Battery status
type Battery2 struct {
	Voltage        uint16 // voltage in millivolts
	CurrentBattery int16  // Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
}

// MsgID (generated function)
func (m *Battery2) MsgID() MessageID {
	return MSG_ID_BATTERY2
}

// MsgName (generated function)
func (m *Battery2) MsgName() string {
	return "Battery2"
}

// Pack (generated function)
func (m *Battery2) Pack(p *Packet) error {
	payload := make([]byte, 4)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.Voltage))
	binary.LittleEndian.PutUint16(payload[2:], uint16(m.CurrentBattery))

	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *Battery2) Unpack(p *Packet) error {
	if len(p.Payload) < 4 {
		return fmt.Errorf("payload too small")
	}
	m.Voltage = uint16(binary.LittleEndian.Uint16(p.Payload[0:]))
	m.CurrentBattery = int16(binary.LittleEndian.Uint16(p.Payload[2:]))
	return nil
}

// Ahrs3 struct (generated typeinfo)
// Status of third AHRS filter if available. This is for ANU research group (Ali and Sean)
type Ahrs3 struct {
	Roll     float32 // Roll angle (rad)
	Pitch    float32 // Pitch angle (rad)
	Yaw      float32 // Yaw angle (rad)
	Altitude float32 // Altitude (MSL)
	Lat      int32   // Latitude in degrees * 1E7
	Lng      int32   // Longitude in degrees * 1E7
	V1       float32 // test variable1
	V2       float32 // test variable2
	V3       float32 // test variable3
	V4       float32 // test variable4
}

// MsgID (generated function)
func (m *Ahrs3) MsgID() MessageID {
	return MSG_ID_AHRS3
}

// MsgName (generated function)
func (m *Ahrs3) MsgName() string {
	return "Ahrs3"
}

// Pack (generated function)
func (m *Ahrs3) Pack(p *Packet) error {
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
func (m *Ahrs3) Unpack(p *Packet) error {
	if len(p.Payload) < 40 {
		return fmt.Errorf("payload too small")
	}
	m.Roll = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.Pitch = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	m.Yaw = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.Altitude = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	m.Lat = int32(binary.LittleEndian.Uint32(p.Payload[16:]))
	m.Lng = int32(binary.LittleEndian.Uint32(p.Payload[20:]))
	m.V1 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	m.V2 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	m.V3 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[32:]))
	m.V4 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[36:]))
	return nil
}

// AutopilotVersionRequest struct (generated typeinfo)
// Request the autopilot version from the system/component.
type AutopilotVersionRequest struct {
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
}

// MsgID (generated function)
func (m *AutopilotVersionRequest) MsgID() MessageID {
	return MSG_ID_AUTOPILOT_VERSION_REQUEST
}

// MsgName (generated function)
func (m *AutopilotVersionRequest) MsgName() string {
	return "AutopilotVersionRequest"
}

// Pack (generated function)
func (m *AutopilotVersionRequest) Pack(p *Packet) error {
	payload := make([]byte, 2)
	payload[0] = byte(m.TargetSystem)
	payload[1] = byte(m.TargetComponent)

	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *AutopilotVersionRequest) Unpack(p *Packet) error {
	if len(p.Payload) < 2 {
		return fmt.Errorf("payload too small")
	}
	m.TargetSystem = uint8(p.Payload[0])
	m.TargetComponent = uint8(p.Payload[1])
	return nil
}

// RemoteLogDataBlock struct (generated typeinfo)
// Send a block of log data to remote location
type RemoteLogDataBlock struct {
	Seqno           uint32     // log data block sequence number
	TargetSystem    uint8      // System ID
	TargetComponent uint8      // Component ID
	Data            [200]uint8 // log data block
}

// MsgID (generated function)
func (m *RemoteLogDataBlock) MsgID() MessageID {
	return MSG_ID_REMOTE_LOG_DATA_BLOCK
}

// MsgName (generated function)
func (m *RemoteLogDataBlock) MsgName() string {
	return "RemoteLogDataBlock"
}

// Pack (generated function)
func (m *RemoteLogDataBlock) Pack(p *Packet) error {
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
func (m *RemoteLogDataBlock) Unpack(p *Packet) error {
	if len(p.Payload) < 206 {
		return fmt.Errorf("payload too small")
	}
	m.Seqno = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.TargetSystem = uint8(p.Payload[4])
	m.TargetComponent = uint8(p.Payload[5])
	copy(m.Data[:], p.Payload[6:206])
	return nil
}

// RemoteLogBlockStatus struct (generated typeinfo)
// Send Status of each log block that autopilot board might have sent
type RemoteLogBlockStatus struct {
	Seqno           uint32 // log data block sequence number
	TargetSystem    uint8  // System ID
	TargetComponent uint8  // Component ID
	Status          uint8  // log data block status
}

// MsgID (generated function)
func (m *RemoteLogBlockStatus) MsgID() MessageID {
	return MSG_ID_REMOTE_LOG_BLOCK_STATUS
}

// MsgName (generated function)
func (m *RemoteLogBlockStatus) MsgName() string {
	return "RemoteLogBlockStatus"
}

// Pack (generated function)
func (m *RemoteLogBlockStatus) Pack(p *Packet) error {
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
func (m *RemoteLogBlockStatus) Unpack(p *Packet) error {
	if len(p.Payload) < 7 {
		return fmt.Errorf("payload too small")
	}
	m.Seqno = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.TargetSystem = uint8(p.Payload[4])
	m.TargetComponent = uint8(p.Payload[5])
	m.Status = uint8(p.Payload[6])
	return nil
}

// LedControl struct (generated typeinfo)
// Control vehicle LEDs
type LedControl struct {
	TargetSystem    uint8     // System ID
	TargetComponent uint8     // Component ID
	Instance        uint8     // Instance (LED instance to control or 255 for all LEDs)
	Pattern         uint8     // Pattern (see LED_PATTERN_ENUM)
	CustomLen       uint8     // Custom Byte Length
	CustomBytes     [24]uint8 // Custom Bytes
}

// MsgID (generated function)
func (m *LedControl) MsgID() MessageID {
	return MSG_ID_LED_CONTROL
}

// MsgName (generated function)
func (m *LedControl) MsgName() string {
	return "LedControl"
}

// Pack (generated function)
func (m *LedControl) Pack(p *Packet) error {
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
func (m *LedControl) Unpack(p *Packet) error {
	if len(p.Payload) < 29 {
		return fmt.Errorf("payload too small")
	}
	m.TargetSystem = uint8(p.Payload[0])
	m.TargetComponent = uint8(p.Payload[1])
	m.Instance = uint8(p.Payload[2])
	m.Pattern = uint8(p.Payload[3])
	m.CustomLen = uint8(p.Payload[4])
	copy(m.CustomBytes[:], p.Payload[5:29])
	return nil
}

// MagCalProgress struct (generated typeinfo)
// Reports progress of compass calibration.
type MagCalProgress struct {
	DirectionX     float32   // Body frame direction vector for display
	DirectionY     float32   // Body frame direction vector for display
	DirectionZ     float32   // Body frame direction vector for display
	CompassID      uint8     // Compass being calibrated
	CalMask        uint8     // Bitmask of compasses being calibrated
	CalStatus      uint8     // Status (see MAG_CAL_STATUS enum)
	Attempt        uint8     // Attempt number
	CompletionPct  uint8     // Completion percentage
	CompletionMask [10]uint8 // Bitmask of sphere sections (see http://en.wikipedia.org/wiki/Geodesic_grid)
}

// MsgID (generated function)
func (m *MagCalProgress) MsgID() MessageID {
	return MSG_ID_MAG_CAL_PROGRESS
}

// MsgName (generated function)
func (m *MagCalProgress) MsgName() string {
	return "MagCalProgress"
}

// Pack (generated function)
func (m *MagCalProgress) Pack(p *Packet) error {
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
func (m *MagCalProgress) Unpack(p *Packet) error {
	if len(p.Payload) < 27 {
		return fmt.Errorf("payload too small")
	}
	m.DirectionX = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.DirectionY = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	m.DirectionZ = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.CompassID = uint8(p.Payload[12])
	m.CalMask = uint8(p.Payload[13])
	m.CalStatus = uint8(p.Payload[14])
	m.Attempt = uint8(p.Payload[15])
	m.CompletionPct = uint8(p.Payload[16])
	copy(m.CompletionMask[:], p.Payload[17:27])
	return nil
}

// MagCalReport struct (generated typeinfo)
// Reports results of completed compass calibration. Sent until MAG_CAL_ACK received.
type MagCalReport struct {
	Fitness   float32 // RMS milligauss residuals
	OfsX      float32 // X offset
	OfsY      float32 // Y offset
	OfsZ      float32 // Z offset
	DiagX     float32 // X diagonal (matrix 11)
	DiagY     float32 // Y diagonal (matrix 22)
	DiagZ     float32 // Z diagonal (matrix 33)
	OffdiagX  float32 // X off-diagonal (matrix 12 and 21)
	OffdiagY  float32 // Y off-diagonal (matrix 13 and 31)
	OffdiagZ  float32 // Z off-diagonal (matrix 32 and 23)
	CompassID uint8   // Compass being calibrated
	CalMask   uint8   // Bitmask of compasses being calibrated
	CalStatus uint8   // Status (see MAG_CAL_STATUS enum)
	Autosaved uint8   // 0=requires a MAV_CMD_DO_ACCEPT_MAG_CAL, 1=saved to parameters
}

// MsgID (generated function)
func (m *MagCalReport) MsgID() MessageID {
	return MSG_ID_MAG_CAL_REPORT
}

// MsgName (generated function)
func (m *MagCalReport) MsgName() string {
	return "MagCalReport"
}

// Pack (generated function)
func (m *MagCalReport) Pack(p *Packet) error {
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
func (m *MagCalReport) Unpack(p *Packet) error {
	if len(p.Payload) < 44 {
		return fmt.Errorf("payload too small")
	}
	m.Fitness = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.OfsX = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	m.OfsY = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.OfsZ = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	m.DiagX = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	m.DiagY = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	m.DiagZ = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	m.OffdiagX = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	m.OffdiagY = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[32:]))
	m.OffdiagZ = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[36:]))
	m.CompassID = uint8(p.Payload[40])
	m.CalMask = uint8(p.Payload[41])
	m.CalStatus = uint8(p.Payload[42])
	m.Autosaved = uint8(p.Payload[43])
	return nil
}

// EkfStatusReport struct (generated typeinfo)
// EKF Status message including flags and variances
type EkfStatusReport struct {
	VelocityVariance   float32 // Velocity variance
	PosHorizVariance   float32 // Horizontal Position variance
	PosVertVariance    float32 // Vertical Position variance
	CompassVariance    float32 // Compass variance
	TerrainAltVariance float32 // Terrain Altitude variance
	Flags              uint16  // Flags
}

// MsgID (generated function)
func (m *EkfStatusReport) MsgID() MessageID {
	return MSG_ID_EKF_STATUS_REPORT
}

// MsgName (generated function)
func (m *EkfStatusReport) MsgName() string {
	return "EkfStatusReport"
}

// Pack (generated function)
func (m *EkfStatusReport) Pack(p *Packet) error {
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
func (m *EkfStatusReport) Unpack(p *Packet) error {
	if len(p.Payload) < 22 {
		return fmt.Errorf("payload too small")
	}
	m.VelocityVariance = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.PosHorizVariance = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	m.PosVertVariance = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.CompassVariance = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	m.TerrainAltVariance = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	m.Flags = uint16(binary.LittleEndian.Uint16(p.Payload[20:]))
	return nil
}

// PidTuning struct (generated typeinfo)
// PID tuning information
type PidTuning struct {
	Desired  float32 // desired rate (degrees/s)
	Achieved float32 // achieved rate (degrees/s)
	Ff       float32 // FF component
	P        float32 // P component
	I        float32 // I component
	D        float32 // D component
	Axis     uint8   // axis
}

// MsgID (generated function)
func (m *PidTuning) MsgID() MessageID {
	return MSG_ID_PID_TUNING
}

// MsgName (generated function)
func (m *PidTuning) MsgName() string {
	return "PidTuning"
}

// Pack (generated function)
func (m *PidTuning) Pack(p *Packet) error {
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
func (m *PidTuning) Unpack(p *Packet) error {
	if len(p.Payload) < 25 {
		return fmt.Errorf("payload too small")
	}
	m.Desired = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.Achieved = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	m.Ff = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.P = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	m.I = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	m.D = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	m.Axis = uint8(p.Payload[24])
	return nil
}

// GimbalReport struct (generated typeinfo)
// 3 axis gimbal mesuraments
type GimbalReport struct {
	DeltaTime       float32 // Time since last update (seconds)
	DeltaAngleX     float32 // Delta angle X (radians)
	DeltaAngleY     float32 // Delta angle Y (radians)
	DeltaAngleZ     float32 // Delta angle X (radians)
	DeltaVelocityX  float32 // Delta velocity X (m/s)
	DeltaVelocityY  float32 // Delta velocity Y (m/s)
	DeltaVelocityZ  float32 // Delta velocity Z (m/s)
	JointRoll       float32 // Joint ROLL (radians)
	JointEl         float32 // Joint EL (radians)
	JointAz         float32 // Joint AZ (radians)
	TargetSystem    uint8   // System ID
	TargetComponent uint8   // Component ID
}

// MsgID (generated function)
func (m *GimbalReport) MsgID() MessageID {
	return MSG_ID_GIMBAL_REPORT
}

// MsgName (generated function)
func (m *GimbalReport) MsgName() string {
	return "GimbalReport"
}

// Pack (generated function)
func (m *GimbalReport) Pack(p *Packet) error {
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
func (m *GimbalReport) Unpack(p *Packet) error {
	if len(p.Payload) < 42 {
		return fmt.Errorf("payload too small")
	}
	m.DeltaTime = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.DeltaAngleX = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	m.DeltaAngleY = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.DeltaAngleZ = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	m.DeltaVelocityX = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	m.DeltaVelocityY = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	m.DeltaVelocityZ = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	m.JointRoll = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	m.JointEl = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[32:]))
	m.JointAz = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[36:]))
	m.TargetSystem = uint8(p.Payload[40])
	m.TargetComponent = uint8(p.Payload[41])
	return nil
}

// GimbalControl struct (generated typeinfo)
// Control message for rate gimbal
type GimbalControl struct {
	DemandedRateX   float32 // Demanded angular rate X (rad/s)
	DemandedRateY   float32 // Demanded angular rate Y (rad/s)
	DemandedRateZ   float32 // Demanded angular rate Z (rad/s)
	TargetSystem    uint8   // System ID
	TargetComponent uint8   // Component ID
}

// MsgID (generated function)
func (m *GimbalControl) MsgID() MessageID {
	return MSG_ID_GIMBAL_CONTROL
}

// MsgName (generated function)
func (m *GimbalControl) MsgName() string {
	return "GimbalControl"
}

// Pack (generated function)
func (m *GimbalControl) Pack(p *Packet) error {
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
func (m *GimbalControl) Unpack(p *Packet) error {
	if len(p.Payload) < 14 {
		return fmt.Errorf("payload too small")
	}
	m.DemandedRateX = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.DemandedRateY = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	m.DemandedRateZ = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.TargetSystem = uint8(p.Payload[12])
	m.TargetComponent = uint8(p.Payload[13])
	return nil
}

// GimbalTorqueCmdReport struct (generated typeinfo)
// 100 Hz gimbal torque command telemetry
type GimbalTorqueCmdReport struct {
	RlTorqueCmd     int16 // Roll Torque Command
	ElTorqueCmd     int16 // Elevation Torque Command
	AzTorqueCmd     int16 // Azimuth Torque Command
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
}

// MsgID (generated function)
func (m *GimbalTorqueCmdReport) MsgID() MessageID {
	return MSG_ID_GIMBAL_TORQUE_CMD_REPORT
}

// MsgName (generated function)
func (m *GimbalTorqueCmdReport) MsgName() string {
	return "GimbalTorqueCmdReport"
}

// Pack (generated function)
func (m *GimbalTorqueCmdReport) Pack(p *Packet) error {
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
func (m *GimbalTorqueCmdReport) Unpack(p *Packet) error {
	if len(p.Payload) < 8 {
		return fmt.Errorf("payload too small")
	}
	m.RlTorqueCmd = int16(binary.LittleEndian.Uint16(p.Payload[0:]))
	m.ElTorqueCmd = int16(binary.LittleEndian.Uint16(p.Payload[2:]))
	m.AzTorqueCmd = int16(binary.LittleEndian.Uint16(p.Payload[4:]))
	m.TargetSystem = uint8(p.Payload[6])
	m.TargetComponent = uint8(p.Payload[7])
	return nil
}

// GoproHeartbeat struct (generated typeinfo)
// Heartbeat from a HeroBus attached GoPro
type GoproHeartbeat struct {
	Status      uint8 // Status
	CaptureMode uint8 // Current capture mode
	Flags       uint8 // additional status bits
}

// MsgID (generated function)
func (m *GoproHeartbeat) MsgID() MessageID {
	return MSG_ID_GOPRO_HEARTBEAT
}

// MsgName (generated function)
func (m *GoproHeartbeat) MsgName() string {
	return "GoproHeartbeat"
}

// Pack (generated function)
func (m *GoproHeartbeat) Pack(p *Packet) error {
	payload := make([]byte, 3)
	payload[0] = byte(m.Status)
	payload[1] = byte(m.CaptureMode)
	payload[2] = byte(m.Flags)

	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *GoproHeartbeat) Unpack(p *Packet) error {
	if len(p.Payload) < 3 {
		return fmt.Errorf("payload too small")
	}
	m.Status = uint8(p.Payload[0])
	m.CaptureMode = uint8(p.Payload[1])
	m.Flags = uint8(p.Payload[2])
	return nil
}

// GoproGetRequest struct (generated typeinfo)
// Request a GOPRO_COMMAND response from the GoPro
type GoproGetRequest struct {
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
	CmdID           uint8 // Command ID
}

// MsgID (generated function)
func (m *GoproGetRequest) MsgID() MessageID {
	return MSG_ID_GOPRO_GET_REQUEST
}

// MsgName (generated function)
func (m *GoproGetRequest) MsgName() string {
	return "GoproGetRequest"
}

// Pack (generated function)
func (m *GoproGetRequest) Pack(p *Packet) error {
	payload := make([]byte, 3)
	payload[0] = byte(m.TargetSystem)
	payload[1] = byte(m.TargetComponent)
	payload[2] = byte(m.CmdID)

	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *GoproGetRequest) Unpack(p *Packet) error {
	if len(p.Payload) < 3 {
		return fmt.Errorf("payload too small")
	}
	m.TargetSystem = uint8(p.Payload[0])
	m.TargetComponent = uint8(p.Payload[1])
	m.CmdID = uint8(p.Payload[2])
	return nil
}

// GoproGetResponse struct (generated typeinfo)
// Response from a GOPRO_COMMAND get request
type GoproGetResponse struct {
	CmdID  uint8    // Command ID
	Status uint8    // Status
	Value  [4]uint8 // Value
}

// MsgID (generated function)
func (m *GoproGetResponse) MsgID() MessageID {
	return MSG_ID_GOPRO_GET_RESPONSE
}

// MsgName (generated function)
func (m *GoproGetResponse) MsgName() string {
	return "GoproGetResponse"
}

// Pack (generated function)
func (m *GoproGetResponse) Pack(p *Packet) error {
	payload := make([]byte, 6)
	payload[0] = byte(m.CmdID)
	payload[1] = byte(m.Status)
	copy(payload[2:], m.Value[:])

	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *GoproGetResponse) Unpack(p *Packet) error {
	if len(p.Payload) < 6 {
		return fmt.Errorf("payload too small")
	}
	m.CmdID = uint8(p.Payload[0])
	m.Status = uint8(p.Payload[1])
	copy(m.Value[:], p.Payload[2:6])
	return nil
}

// GoproSetRequest struct (generated typeinfo)
// Request to set a GOPRO_COMMAND with a desired
type GoproSetRequest struct {
	TargetSystem    uint8    // System ID
	TargetComponent uint8    // Component ID
	CmdID           uint8    // Command ID
	Value           [4]uint8 // Value
}

// MsgID (generated function)
func (m *GoproSetRequest) MsgID() MessageID {
	return MSG_ID_GOPRO_SET_REQUEST
}

// MsgName (generated function)
func (m *GoproSetRequest) MsgName() string {
	return "GoproSetRequest"
}

// Pack (generated function)
func (m *GoproSetRequest) Pack(p *Packet) error {
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
func (m *GoproSetRequest) Unpack(p *Packet) error {
	if len(p.Payload) < 7 {
		return fmt.Errorf("payload too small")
	}
	m.TargetSystem = uint8(p.Payload[0])
	m.TargetComponent = uint8(p.Payload[1])
	m.CmdID = uint8(p.Payload[2])
	copy(m.Value[:], p.Payload[3:7])
	return nil
}

// GoproSetResponse struct (generated typeinfo)
// Response from a GOPRO_COMMAND set request
type GoproSetResponse struct {
	CmdID  uint8 // Command ID
	Status uint8 // Status
}

// MsgID (generated function)
func (m *GoproSetResponse) MsgID() MessageID {
	return MSG_ID_GOPRO_SET_RESPONSE
}

// MsgName (generated function)
func (m *GoproSetResponse) MsgName() string {
	return "GoproSetResponse"
}

// Pack (generated function)
func (m *GoproSetResponse) Pack(p *Packet) error {
	payload := make([]byte, 2)
	payload[0] = byte(m.CmdID)
	payload[1] = byte(m.Status)

	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *GoproSetResponse) Unpack(p *Packet) error {
	if len(p.Payload) < 2 {
		return fmt.Errorf("payload too small")
	}
	m.CmdID = uint8(p.Payload[0])
	m.Status = uint8(p.Payload[1])
	return nil
}

// Rpm struct (generated typeinfo)
// RPM sensor output
type Rpm struct {
	Rpm1 float32 // RPM Sensor1
	Rpm2 float32 // RPM Sensor2
}

// MsgID (generated function)
func (m *Rpm) MsgID() MessageID {
	return MSG_ID_RPM
}

// MsgName (generated function)
func (m *Rpm) MsgName() string {
	return "Rpm"
}

// Pack (generated function)
func (m *Rpm) Pack(p *Packet) error {
	payload := make([]byte, 8)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(m.Rpm1))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(m.Rpm2))

	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *Rpm) Unpack(p *Packet) error {
	if len(p.Payload) < 8 {
		return fmt.Errorf("payload too small")
	}
	m.Rpm1 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	m.Rpm2 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
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
	MSG_ID_FENCE_STATUS              MessageID = 162
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
	MSG_ID_MAG_CAL_REPORT            MessageID = 192
	MSG_ID_EKF_STATUS_REPORT         MessageID = 193
	MSG_ID_PID_TUNING                MessageID = 194
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
		MSG_ID_FENCE_STATUS:              189,
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
		MSG_ID_MAG_CAL_REPORT:            36,
		MSG_ID_EKF_STATUS_REPORT:         71,
		MSG_ID_PID_TUNING:                98,
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
			msg := new(SensorOffsets)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SET_MAG_OFFSETS: func(pkt *Packet) Message {
			msg := new(SetMagOffsets)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_MEMINFO: func(pkt *Packet) Message {
			msg := new(Meminfo)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_AP_ADC: func(pkt *Packet) Message {
			msg := new(ApAdc)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_DIGICAM_CONFIGURE: func(pkt *Packet) Message {
			msg := new(DigicamConfigure)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_DIGICAM_CONTROL: func(pkt *Packet) Message {
			msg := new(DigicamControl)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_MOUNT_CONFIGURE: func(pkt *Packet) Message {
			msg := new(MountConfigure)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_MOUNT_CONTROL: func(pkt *Packet) Message {
			msg := new(MountControl)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_MOUNT_STATUS: func(pkt *Packet) Message {
			msg := new(MountStatus)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_FENCE_POINT: func(pkt *Packet) Message {
			msg := new(FencePoint)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_FENCE_FETCH_POINT: func(pkt *Packet) Message {
			msg := new(FenceFetchPoint)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_FENCE_STATUS: func(pkt *Packet) Message {
			msg := new(FenceStatus)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_AHRS: func(pkt *Packet) Message {
			msg := new(Ahrs)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SIMSTATE: func(pkt *Packet) Message {
			msg := new(Simstate)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_HWSTATUS: func(pkt *Packet) Message {
			msg := new(Hwstatus)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_RADIO: func(pkt *Packet) Message {
			msg := new(Radio)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_LIMITS_STATUS: func(pkt *Packet) Message {
			msg := new(LimitsStatus)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_WIND: func(pkt *Packet) Message {
			msg := new(Wind)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_DATA16: func(pkt *Packet) Message {
			msg := new(Data16)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_DATA32: func(pkt *Packet) Message {
			msg := new(Data32)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_DATA64: func(pkt *Packet) Message {
			msg := new(Data64)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_DATA96: func(pkt *Packet) Message {
			msg := new(Data96)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_RANGEFINDER: func(pkt *Packet) Message {
			msg := new(Rangefinder)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_AIRSPEED_AUTOCAL: func(pkt *Packet) Message {
			msg := new(AirspeedAutocal)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_RALLY_POINT: func(pkt *Packet) Message {
			msg := new(RallyPoint)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_RALLY_FETCH_POINT: func(pkt *Packet) Message {
			msg := new(RallyFetchPoint)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_COMPASSMOT_STATUS: func(pkt *Packet) Message {
			msg := new(CompassmotStatus)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_AHRS2: func(pkt *Packet) Message {
			msg := new(Ahrs2)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_CAMERA_STATUS: func(pkt *Packet) Message {
			msg := new(CameraStatus)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_CAMERA_FEEDBACK: func(pkt *Packet) Message {
			msg := new(CameraFeedback)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_BATTERY2: func(pkt *Packet) Message {
			msg := new(Battery2)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_AHRS3: func(pkt *Packet) Message {
			msg := new(Ahrs3)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_AUTOPILOT_VERSION_REQUEST: func(pkt *Packet) Message {
			msg := new(AutopilotVersionRequest)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_REMOTE_LOG_DATA_BLOCK: func(pkt *Packet) Message {
			msg := new(RemoteLogDataBlock)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_REMOTE_LOG_BLOCK_STATUS: func(pkt *Packet) Message {
			msg := new(RemoteLogBlockStatus)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_LED_CONTROL: func(pkt *Packet) Message {
			msg := new(LedControl)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_MAG_CAL_PROGRESS: func(pkt *Packet) Message {
			msg := new(MagCalProgress)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_MAG_CAL_REPORT: func(pkt *Packet) Message {
			msg := new(MagCalReport)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_EKF_STATUS_REPORT: func(pkt *Packet) Message {
			msg := new(EkfStatusReport)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_PID_TUNING: func(pkt *Packet) Message {
			msg := new(PidTuning)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_GIMBAL_REPORT: func(pkt *Packet) Message {
			msg := new(GimbalReport)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_GIMBAL_CONTROL: func(pkt *Packet) Message {
			msg := new(GimbalControl)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_GIMBAL_TORQUE_CMD_REPORT: func(pkt *Packet) Message {
			msg := new(GimbalTorqueCmdReport)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_GOPRO_HEARTBEAT: func(pkt *Packet) Message {
			msg := new(GoproHeartbeat)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_GOPRO_GET_REQUEST: func(pkt *Packet) Message {
			msg := new(GoproGetRequest)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_GOPRO_GET_RESPONSE: func(pkt *Packet) Message {
			msg := new(GoproGetResponse)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_GOPRO_SET_REQUEST: func(pkt *Packet) Message {
			msg := new(GoproSetRequest)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_GOPRO_SET_RESPONSE: func(pkt *Packet) Message {
			msg := new(GoproSetResponse)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_RPM: func(pkt *Packet) Message {
			msg := new(Rpm)
			msg.Unpack(pkt)
			return msg
		},
	},
}

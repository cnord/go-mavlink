//////////////////////////////////////////////////
//
// NOTE: do not edit,
// this file created automatically by mavgen.go
//
//////////////////////////////////////////////////

package asluav

import (
	mavlink ".."
	"encoding/binary"
	"fmt"
	"math"
)

// MavCmd (generated enum)
//
const (
	MAV_CMD_RESET_MPPT      = 40001 // Mission command to reset Maximum Power Point Tracker (MPPT). Params: 1) MPPT number; 2) Empty; 3) Empty; 4) Empty; 5) Empty; 6) Empty; 7) Empty;
	MAV_CMD_PAYLOAD_CONTROL = 40002 // Mission command to perform a power cycle on payload. Params: 1) Complete power cycle; 2) VISensor power cycle; 3) Empty; 4) Empty; 5) Empty; 6) Empty; 7) Empty;
)

// GsmLinkType (generated enum)
//
const (
	GSM_LINK_TYPE_NONE    = 0 // no service
	GSM_LINK_TYPE_UNKNOWN = 1 // link type unknown
	GSM_LINK_TYPE_2G      = 2 // 2G (GSM/GRPS/EDGE) link
	GSM_LINK_TYPE_3G      = 3 // 3G link (WCDMA/HSDPA/HSPA)
	GSM_LINK_TYPE_4G      = 4 // 4G link (LTE)
)

// GsmModemType (generated enum)
//
const (
	GSM_MODEM_TYPE_UNKNOWN      = 0 // not specified
	GSM_MODEM_TYPE_HUAWEI_E3372 = 1 // HUAWEI LTE USB Stick E3372
)

// CommandIntStamped struct (generated typeinfo)
// Message encoding a command with parameters as scaled integers and additional metadata. Scaling depends on the actual command value.
type CommandIntStamped struct {
	VehicleTimestamp uint64  // Microseconds elapsed since vehicle boot
	UtcTime          uint32  // UTC time, seconds elapsed since 01.01.1970
	Param1           float32 // PARAM1, see MAV_CMD enum
	Param2           float32 // PARAM2, see MAV_CMD enum
	Param3           float32 // PARAM3, see MAV_CMD enum
	Param4           float32 // PARAM4, see MAV_CMD enum
	X                int32   // PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7
	Y                int32   // PARAM6 / local: y position in meters * 1e4, global: longitude in degrees * 10^7
	Z                float32 // PARAM7 / z position: global: altitude in meters (MSL, WGS84, AGL or relative to home - depending on frame).
	Command          uint16  // The scheduled action for the mission item, as defined by MAV_CMD enum
	TargetSystem     uint8   // System ID
	TargetComponent  uint8   // Component ID
	Frame            uint8   // The coordinate system of the COMMAND, as defined by MAV_FRAME enum
	Current          uint8   // false:0, true:1
	Autocontinue     uint8   // autocontinue to next wp
}

// MsgID (generated function)
func (m *CommandIntStamped) MsgID() mavlink.MessageID {
	return MSG_ID_COMMAND_INT_STAMPED
}

// String (generated function)
func (m *CommandIntStamped) String() string {
	return fmt.Sprintf(
		"&ASLUAV.CommandIntStamped{ VehicleTimestamp: %+v, UtcTime: %+v, Param1: %+v, Param2: %+v, Param3: %+v, Param4: %+v, X: %+v, Y: %+v, Z: %+v, Command: %+v, TargetSystem: %+v, TargetComponent: %+v, Frame: %+v, Current: %+v, Autocontinue: %+v }",
		m.VehicleTimestamp,
		m.UtcTime,
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
func (m *CommandIntStamped) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 47)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.VehicleTimestamp))
	binary.LittleEndian.PutUint32(payload[8:], uint32(m.UtcTime))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.Param1))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(m.Param2))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(m.Param3))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(m.Param4))
	binary.LittleEndian.PutUint32(payload[28:], uint32(m.X))
	binary.LittleEndian.PutUint32(payload[32:], uint32(m.Y))
	binary.LittleEndian.PutUint32(payload[36:], math.Float32bits(m.Z))
	binary.LittleEndian.PutUint16(payload[40:], uint16(m.Command))
	payload[42] = byte(m.TargetSystem)
	payload[43] = byte(m.TargetComponent)
	payload[44] = byte(m.Frame)
	payload[45] = byte(m.Current)
	payload[46] = byte(m.Autocontinue)
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
func (m *CommandIntStamped) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 47 {
		payload = append(payload, mavlink.ZeroTail[:47-len(p.Payload)]...)
	}
	m.VehicleTimestamp = uint64(binary.LittleEndian.Uint64(payload[0:]))
	m.UtcTime = uint32(binary.LittleEndian.Uint32(payload[8:]))
	m.Param1 = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.Param2 = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	m.Param3 = math.Float32frombits(binary.LittleEndian.Uint32(payload[20:]))
	m.Param4 = math.Float32frombits(binary.LittleEndian.Uint32(payload[24:]))
	m.X = int32(binary.LittleEndian.Uint32(payload[28:]))
	m.Y = int32(binary.LittleEndian.Uint32(payload[32:]))
	m.Z = math.Float32frombits(binary.LittleEndian.Uint32(payload[36:]))
	m.Command = uint16(binary.LittleEndian.Uint16(payload[40:]))
	m.TargetSystem = uint8(payload[42])
	m.TargetComponent = uint8(payload[43])
	m.Frame = uint8(payload[44])
	m.Current = uint8(payload[45])
	m.Autocontinue = uint8(payload[46])
	return nil
}

// CommandLongStamped struct (generated typeinfo)
// Send a command with up to seven parameters to the MAV and additional metadata
type CommandLongStamped struct {
	VehicleTimestamp uint64  // Microseconds elapsed since vehicle boot
	UtcTime          uint32  // UTC time, seconds elapsed since 01.01.1970
	Param1           float32 // Parameter 1, as defined by MAV_CMD enum.
	Param2           float32 // Parameter 2, as defined by MAV_CMD enum.
	Param3           float32 // Parameter 3, as defined by MAV_CMD enum.
	Param4           float32 // Parameter 4, as defined by MAV_CMD enum.
	Param5           float32 // Parameter 5, as defined by MAV_CMD enum.
	Param6           float32 // Parameter 6, as defined by MAV_CMD enum.
	Param7           float32 // Parameter 7, as defined by MAV_CMD enum.
	Command          uint16  // Command ID, as defined by MAV_CMD enum.
	TargetSystem     uint8   // System which should execute the command
	TargetComponent  uint8   // Component which should execute the command, 0 for all components
	Confirmation     uint8   // 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
}

// MsgID (generated function)
func (m *CommandLongStamped) MsgID() mavlink.MessageID {
	return MSG_ID_COMMAND_LONG_STAMPED
}

// String (generated function)
func (m *CommandLongStamped) String() string {
	return fmt.Sprintf(
		"&ASLUAV.CommandLongStamped{ VehicleTimestamp: %+v, UtcTime: %+v, Param1: %+v, Param2: %+v, Param3: %+v, Param4: %+v, Param5: %+v, Param6: %+v, Param7: %+v, Command: %+v, TargetSystem: %+v, TargetComponent: %+v, Confirmation: %+v }",
		m.VehicleTimestamp,
		m.UtcTime,
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
func (m *CommandLongStamped) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 45)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.VehicleTimestamp))
	binary.LittleEndian.PutUint32(payload[8:], uint32(m.UtcTime))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.Param1))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(m.Param2))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(m.Param3))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(m.Param4))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(m.Param5))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(m.Param6))
	binary.LittleEndian.PutUint32(payload[36:], math.Float32bits(m.Param7))
	binary.LittleEndian.PutUint16(payload[40:], uint16(m.Command))
	payload[42] = byte(m.TargetSystem)
	payload[43] = byte(m.TargetComponent)
	payload[44] = byte(m.Confirmation)
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
func (m *CommandLongStamped) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 45 {
		payload = append(payload, mavlink.ZeroTail[:45-len(p.Payload)]...)
	}
	m.VehicleTimestamp = uint64(binary.LittleEndian.Uint64(payload[0:]))
	m.UtcTime = uint32(binary.LittleEndian.Uint32(payload[8:]))
	m.Param1 = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.Param2 = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	m.Param3 = math.Float32frombits(binary.LittleEndian.Uint32(payload[20:]))
	m.Param4 = math.Float32frombits(binary.LittleEndian.Uint32(payload[24:]))
	m.Param5 = math.Float32frombits(binary.LittleEndian.Uint32(payload[28:]))
	m.Param6 = math.Float32frombits(binary.LittleEndian.Uint32(payload[32:]))
	m.Param7 = math.Float32frombits(binary.LittleEndian.Uint32(payload[36:]))
	m.Command = uint16(binary.LittleEndian.Uint16(payload[40:]))
	m.TargetSystem = uint8(payload[42])
	m.TargetComponent = uint8(payload[43])
	m.Confirmation = uint8(payload[44])
	return nil
}

// SensPower struct (generated typeinfo)
// Voltage and current sensor data
type SensPower struct {
	Adc121VspbVolt float32 //  Power board voltage sensor reading
	Adc121CspbAmp  float32 //  Power board current sensor reading
	Adc121Cs1Amp   float32 //  Board current sensor 1 reading
	Adc121Cs2Amp   float32 //  Board current sensor 2 reading
}

// MsgID (generated function)
func (m *SensPower) MsgID() mavlink.MessageID {
	return MSG_ID_SENS_POWER
}

// String (generated function)
func (m *SensPower) String() string {
	return fmt.Sprintf(
		"&ASLUAV.SensPower{ Adc121VspbVolt: %+v, Adc121CspbAmp: %+v, Adc121Cs1Amp: %+v, Adc121Cs2Amp: %+v }",
		m.Adc121VspbVolt,
		m.Adc121CspbAmp,
		m.Adc121Cs1Amp,
		m.Adc121Cs2Amp,
	)
}

// Pack (generated function)
func (m *SensPower) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 16)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(m.Adc121VspbVolt))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(m.Adc121CspbAmp))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.Adc121Cs1Amp))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.Adc121Cs2Amp))
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
func (m *SensPower) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 16 {
		payload = append(payload, mavlink.ZeroTail[:16-len(p.Payload)]...)
	}
	m.Adc121VspbVolt = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.Adc121CspbAmp = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	m.Adc121Cs1Amp = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.Adc121Cs2Amp = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	return nil
}

// SensMppt struct (generated typeinfo)
// Maximum Power Point Tracker (MPPT) sensor data for solar module power performance tracking
type SensMppt struct {
	MpptTimestamp uint64  //  MPPT last timestamp
	Mppt1Volt     float32 //  MPPT1 voltage
	Mppt1Amp      float32 //  MPPT1 current
	Mppt2Volt     float32 //  MPPT2 voltage
	Mppt2Amp      float32 //  MPPT2 current
	Mppt3Volt     float32 // MPPT3 voltage
	Mppt3Amp      float32 //  MPPT3 current
	Mppt1Pwm      uint16  //  MPPT1 pwm
	Mppt2Pwm      uint16  //  MPPT2 pwm
	Mppt3Pwm      uint16  //  MPPT3 pwm
	Mppt1Status   uint8   //  MPPT1 status
	Mppt2Status   uint8   //  MPPT2 status
	Mppt3Status   uint8   //  MPPT3 status
}

// MsgID (generated function)
func (m *SensMppt) MsgID() mavlink.MessageID {
	return MSG_ID_SENS_MPPT
}

// String (generated function)
func (m *SensMppt) String() string {
	return fmt.Sprintf(
		"&ASLUAV.SensMppt{ MpptTimestamp: %+v, Mppt1Volt: %+v, Mppt1Amp: %+v, Mppt2Volt: %+v, Mppt2Amp: %+v, Mppt3Volt: %+v, Mppt3Amp: %+v, Mppt1Pwm: %+v, Mppt2Pwm: %+v, Mppt3Pwm: %+v, Mppt1Status: %+v, Mppt2Status: %+v, Mppt3Status: %+v }",
		m.MpptTimestamp,
		m.Mppt1Volt,
		m.Mppt1Amp,
		m.Mppt2Volt,
		m.Mppt2Amp,
		m.Mppt3Volt,
		m.Mppt3Amp,
		m.Mppt1Pwm,
		m.Mppt2Pwm,
		m.Mppt3Pwm,
		m.Mppt1Status,
		m.Mppt2Status,
		m.Mppt3Status,
	)
}

// Pack (generated function)
func (m *SensMppt) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 41)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.MpptTimestamp))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.Mppt1Volt))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.Mppt1Amp))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(m.Mppt2Volt))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(m.Mppt2Amp))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(m.Mppt3Volt))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(m.Mppt3Amp))
	binary.LittleEndian.PutUint16(payload[32:], uint16(m.Mppt1Pwm))
	binary.LittleEndian.PutUint16(payload[34:], uint16(m.Mppt2Pwm))
	binary.LittleEndian.PutUint16(payload[36:], uint16(m.Mppt3Pwm))
	payload[38] = byte(m.Mppt1Status)
	payload[39] = byte(m.Mppt2Status)
	payload[40] = byte(m.Mppt3Status)
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
func (m *SensMppt) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 41 {
		payload = append(payload, mavlink.ZeroTail[:41-len(p.Payload)]...)
	}
	m.MpptTimestamp = uint64(binary.LittleEndian.Uint64(payload[0:]))
	m.Mppt1Volt = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.Mppt1Amp = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.Mppt2Volt = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	m.Mppt2Amp = math.Float32frombits(binary.LittleEndian.Uint32(payload[20:]))
	m.Mppt3Volt = math.Float32frombits(binary.LittleEndian.Uint32(payload[24:]))
	m.Mppt3Amp = math.Float32frombits(binary.LittleEndian.Uint32(payload[28:]))
	m.Mppt1Pwm = uint16(binary.LittleEndian.Uint16(payload[32:]))
	m.Mppt2Pwm = uint16(binary.LittleEndian.Uint16(payload[34:]))
	m.Mppt3Pwm = uint16(binary.LittleEndian.Uint16(payload[36:]))
	m.Mppt1Status = uint8(payload[38])
	m.Mppt2Status = uint8(payload[39])
	m.Mppt3Status = uint8(payload[40])
	return nil
}

// AslctrlData struct (generated typeinfo)
// ASL-fixed-wing controller data
type AslctrlData struct {
	Timestamp       uint64  //  Timestamp
	H               float32 //  See sourcecode for a description of these values...
	Href            float32 //
	HrefT           float32 //
	Pitchangle      float32 // Pitch angle
	Pitchangleref   float32 // Pitch angle reference
	Q               float32 //
	Qref            float32 //
	Uelev           float32 //
	Uthrot          float32 //
	Uthrot2         float32 //
	Nz              float32 //
	Airspeedref     float32 // Airspeed reference
	Yawangle        float32 // Yaw angle
	Yawangleref     float32 // Yaw angle reference
	Rollangle       float32 // Roll angle
	Rollangleref    float32 // Roll angle reference
	P               float32 //
	Pref            float32 //
	R               float32 //
	Rref            float32 //
	Uail            float32 //
	Urud            float32 //
	AslctrlMode     uint8   //  ASLCTRL control-mode (manual, stabilized, auto, etc...)
	Spoilersengaged uint8   //
}

// MsgID (generated function)
func (m *AslctrlData) MsgID() mavlink.MessageID {
	return MSG_ID_ASLCTRL_DATA
}

// String (generated function)
func (m *AslctrlData) String() string {
	return fmt.Sprintf(
		"&ASLUAV.AslctrlData{ Timestamp: %+v, H: %+v, Href: %+v, HrefT: %+v, Pitchangle: %+v, Pitchangleref: %+v, Q: %+v, Qref: %+v, Uelev: %+v, Uthrot: %+v, Uthrot2: %+v, Nz: %+v, Airspeedref: %+v, Yawangle: %+v, Yawangleref: %+v, Rollangle: %+v, Rollangleref: %+v, P: %+v, Pref: %+v, R: %+v, Rref: %+v, Uail: %+v, Urud: %+v, AslctrlMode: %+v, Spoilersengaged: %+v }",
		m.Timestamp,
		m.H,
		m.Href,
		m.HrefT,
		m.Pitchangle,
		m.Pitchangleref,
		m.Q,
		m.Qref,
		m.Uelev,
		m.Uthrot,
		m.Uthrot2,
		m.Nz,
		m.Airspeedref,
		m.Yawangle,
		m.Yawangleref,
		m.Rollangle,
		m.Rollangleref,
		m.P,
		m.Pref,
		m.R,
		m.Rref,
		m.Uail,
		m.Urud,
		m.AslctrlMode,
		m.Spoilersengaged,
	)
}

// Pack (generated function)
func (m *AslctrlData) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 98)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.Timestamp))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.H))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.Href))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(m.HrefT))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(m.Pitchangle))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(m.Pitchangleref))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(m.Q))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(m.Qref))
	binary.LittleEndian.PutUint32(payload[36:], math.Float32bits(m.Uelev))
	binary.LittleEndian.PutUint32(payload[40:], math.Float32bits(m.Uthrot))
	binary.LittleEndian.PutUint32(payload[44:], math.Float32bits(m.Uthrot2))
	binary.LittleEndian.PutUint32(payload[48:], math.Float32bits(m.Nz))
	binary.LittleEndian.PutUint32(payload[52:], math.Float32bits(m.Airspeedref))
	binary.LittleEndian.PutUint32(payload[56:], math.Float32bits(m.Yawangle))
	binary.LittleEndian.PutUint32(payload[60:], math.Float32bits(m.Yawangleref))
	binary.LittleEndian.PutUint32(payload[64:], math.Float32bits(m.Rollangle))
	binary.LittleEndian.PutUint32(payload[68:], math.Float32bits(m.Rollangleref))
	binary.LittleEndian.PutUint32(payload[72:], math.Float32bits(m.P))
	binary.LittleEndian.PutUint32(payload[76:], math.Float32bits(m.Pref))
	binary.LittleEndian.PutUint32(payload[80:], math.Float32bits(m.R))
	binary.LittleEndian.PutUint32(payload[84:], math.Float32bits(m.Rref))
	binary.LittleEndian.PutUint32(payload[88:], math.Float32bits(m.Uail))
	binary.LittleEndian.PutUint32(payload[92:], math.Float32bits(m.Urud))
	payload[96] = byte(m.AslctrlMode)
	payload[97] = byte(m.Spoilersengaged)
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
func (m *AslctrlData) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 98 {
		payload = append(payload, mavlink.ZeroTail[:98-len(p.Payload)]...)
	}
	m.Timestamp = uint64(binary.LittleEndian.Uint64(payload[0:]))
	m.H = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.Href = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.HrefT = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	m.Pitchangle = math.Float32frombits(binary.LittleEndian.Uint32(payload[20:]))
	m.Pitchangleref = math.Float32frombits(binary.LittleEndian.Uint32(payload[24:]))
	m.Q = math.Float32frombits(binary.LittleEndian.Uint32(payload[28:]))
	m.Qref = math.Float32frombits(binary.LittleEndian.Uint32(payload[32:]))
	m.Uelev = math.Float32frombits(binary.LittleEndian.Uint32(payload[36:]))
	m.Uthrot = math.Float32frombits(binary.LittleEndian.Uint32(payload[40:]))
	m.Uthrot2 = math.Float32frombits(binary.LittleEndian.Uint32(payload[44:]))
	m.Nz = math.Float32frombits(binary.LittleEndian.Uint32(payload[48:]))
	m.Airspeedref = math.Float32frombits(binary.LittleEndian.Uint32(payload[52:]))
	m.Yawangle = math.Float32frombits(binary.LittleEndian.Uint32(payload[56:]))
	m.Yawangleref = math.Float32frombits(binary.LittleEndian.Uint32(payload[60:]))
	m.Rollangle = math.Float32frombits(binary.LittleEndian.Uint32(payload[64:]))
	m.Rollangleref = math.Float32frombits(binary.LittleEndian.Uint32(payload[68:]))
	m.P = math.Float32frombits(binary.LittleEndian.Uint32(payload[72:]))
	m.Pref = math.Float32frombits(binary.LittleEndian.Uint32(payload[76:]))
	m.R = math.Float32frombits(binary.LittleEndian.Uint32(payload[80:]))
	m.Rref = math.Float32frombits(binary.LittleEndian.Uint32(payload[84:]))
	m.Uail = math.Float32frombits(binary.LittleEndian.Uint32(payload[88:]))
	m.Urud = math.Float32frombits(binary.LittleEndian.Uint32(payload[92:]))
	m.AslctrlMode = uint8(payload[96])
	m.Spoilersengaged = uint8(payload[97])
	return nil
}

// AslctrlDebug struct (generated typeinfo)
// ASL-fixed-wing controller debug data
type AslctrlDebug struct {
	I321 uint32  //  Debug data
	F1   float32 //  Debug data
	F2   float32 //  Debug data
	F3   float32 //  Debug data
	F4   float32 //  Debug data
	F5   float32 //  Debug data
	F6   float32 //  Debug data
	F7   float32 //  Debug data
	F8   float32 //  Debug data
	I81  uint8   //  Debug data
	I82  uint8   //  Debug data
}

// MsgID (generated function)
func (m *AslctrlDebug) MsgID() mavlink.MessageID {
	return MSG_ID_ASLCTRL_DEBUG
}

// String (generated function)
func (m *AslctrlDebug) String() string {
	return fmt.Sprintf(
		"&ASLUAV.AslctrlDebug{ I321: %+v, F1: %+v, F2: %+v, F3: %+v, F4: %+v, F5: %+v, F6: %+v, F7: %+v, F8: %+v, I81: %+v, I82: %+v }",
		m.I321,
		m.F1,
		m.F2,
		m.F3,
		m.F4,
		m.F5,
		m.F6,
		m.F7,
		m.F8,
		m.I81,
		m.I82,
	)
}

// Pack (generated function)
func (m *AslctrlDebug) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 38)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.I321))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(m.F1))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.F2))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.F3))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(m.F4))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(m.F5))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(m.F6))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(m.F7))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(m.F8))
	payload[36] = byte(m.I81)
	payload[37] = byte(m.I82)
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
func (m *AslctrlDebug) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 38 {
		payload = append(payload, mavlink.ZeroTail[:38-len(p.Payload)]...)
	}
	m.I321 = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.F1 = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	m.F2 = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.F3 = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.F4 = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	m.F5 = math.Float32frombits(binary.LittleEndian.Uint32(payload[20:]))
	m.F6 = math.Float32frombits(binary.LittleEndian.Uint32(payload[24:]))
	m.F7 = math.Float32frombits(binary.LittleEndian.Uint32(payload[28:]))
	m.F8 = math.Float32frombits(binary.LittleEndian.Uint32(payload[32:]))
	m.I81 = uint8(payload[36])
	m.I82 = uint8(payload[37])
	return nil
}

// AsluavStatus struct (generated typeinfo)
// Extended state information for ASLUAVs
type AsluavStatus struct {
	MotorRpm     float32  //  Motor RPM
	LedStatus    uint8    //  Status of the position-indicator LEDs
	SatcomStatus uint8    //  Status of the IRIDIUM satellite communication system
	ServoStatus  [8]uint8 //  Status vector for up to 8 servos
}

// MsgID (generated function)
func (m *AsluavStatus) MsgID() mavlink.MessageID {
	return MSG_ID_ASLUAV_STATUS
}

// String (generated function)
func (m *AsluavStatus) String() string {
	return fmt.Sprintf(
		"&ASLUAV.AsluavStatus{ MotorRpm: %+v, LedStatus: %+v, SatcomStatus: %+v, ServoStatus: %0b (\"%s\") }",
		m.MotorRpm,
		m.LedStatus,
		m.SatcomStatus,
		m.ServoStatus, string(m.ServoStatus[:]),
	)
}

// Pack (generated function)
func (m *AsluavStatus) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 14)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(m.MotorRpm))
	payload[4] = byte(m.LedStatus)
	payload[5] = byte(m.SatcomStatus)
	copy(payload[6:], m.ServoStatus[:])
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
func (m *AsluavStatus) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 14 {
		payload = append(payload, mavlink.ZeroTail[:14-len(p.Payload)]...)
	}
	m.MotorRpm = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.LedStatus = uint8(payload[4])
	m.SatcomStatus = uint8(payload[5])
	copy(m.ServoStatus[:], payload[6:14])
	return nil
}

// EkfExt struct (generated typeinfo)
// Extended EKF state estimates for ASLUAVs
type EkfExt struct {
	Timestamp uint64  //  Time since system start
	Windspeed float32 //  Magnitude of wind velocity (in lateral inertial plane)
	Winddir   float32 //  Wind heading angle from North
	Windz     float32 //  Z (Down) component of inertial wind velocity
	Airspeed  float32 //  Magnitude of air velocity
	Beta      float32 //  Sideslip angle
	Alpha     float32 //  Angle of attack
}

// MsgID (generated function)
func (m *EkfExt) MsgID() mavlink.MessageID {
	return MSG_ID_EKF_EXT
}

// String (generated function)
func (m *EkfExt) String() string {
	return fmt.Sprintf(
		"&ASLUAV.EkfExt{ Timestamp: %+v, Windspeed: %+v, Winddir: %+v, Windz: %+v, Airspeed: %+v, Beta: %+v, Alpha: %+v }",
		m.Timestamp,
		m.Windspeed,
		m.Winddir,
		m.Windz,
		m.Airspeed,
		m.Beta,
		m.Alpha,
	)
}

// Pack (generated function)
func (m *EkfExt) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 32)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.Timestamp))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.Windspeed))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.Winddir))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(m.Windz))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(m.Airspeed))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(m.Beta))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(m.Alpha))
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
func (m *EkfExt) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 32 {
		payload = append(payload, mavlink.ZeroTail[:32-len(p.Payload)]...)
	}
	m.Timestamp = uint64(binary.LittleEndian.Uint64(payload[0:]))
	m.Windspeed = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.Winddir = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.Windz = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	m.Airspeed = math.Float32frombits(binary.LittleEndian.Uint32(payload[20:]))
	m.Beta = math.Float32frombits(binary.LittleEndian.Uint32(payload[24:]))
	m.Alpha = math.Float32frombits(binary.LittleEndian.Uint32(payload[28:]))
	return nil
}

// AslObctrl struct (generated typeinfo)
// Off-board controls/commands for ASLUAVs
type AslObctrl struct {
	Timestamp    uint64  //  Time since system start
	Uelev        float32 //  Elevator command [~]
	Uthrot       float32 //  Throttle command [~]
	Uthrot2      float32 //  Throttle 2 command [~]
	Uaill        float32 //  Left aileron command [~]
	Uailr        float32 //  Right aileron command [~]
	Urud         float32 //  Rudder command [~]
	ObctrlStatus uint8   //  Off-board computer status
}

// MsgID (generated function)
func (m *AslObctrl) MsgID() mavlink.MessageID {
	return MSG_ID_ASL_OBCTRL
}

// String (generated function)
func (m *AslObctrl) String() string {
	return fmt.Sprintf(
		"&ASLUAV.AslObctrl{ Timestamp: %+v, Uelev: %+v, Uthrot: %+v, Uthrot2: %+v, Uaill: %+v, Uailr: %+v, Urud: %+v, ObctrlStatus: %+v }",
		m.Timestamp,
		m.Uelev,
		m.Uthrot,
		m.Uthrot2,
		m.Uaill,
		m.Uailr,
		m.Urud,
		m.ObctrlStatus,
	)
}

// Pack (generated function)
func (m *AslObctrl) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 33)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.Timestamp))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.Uelev))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.Uthrot))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(m.Uthrot2))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(m.Uaill))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(m.Uailr))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(m.Urud))
	payload[32] = byte(m.ObctrlStatus)
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
func (m *AslObctrl) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 33 {
		payload = append(payload, mavlink.ZeroTail[:33-len(p.Payload)]...)
	}
	m.Timestamp = uint64(binary.LittleEndian.Uint64(payload[0:]))
	m.Uelev = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.Uthrot = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.Uthrot2 = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	m.Uaill = math.Float32frombits(binary.LittleEndian.Uint32(payload[20:]))
	m.Uailr = math.Float32frombits(binary.LittleEndian.Uint32(payload[24:]))
	m.Urud = math.Float32frombits(binary.LittleEndian.Uint32(payload[28:]))
	m.ObctrlStatus = uint8(payload[32])
	return nil
}

// SensAtmos struct (generated typeinfo)
// Atmospheric sensors (temperature, humidity, ...)
type SensAtmos struct {
	Timestamp   uint64  // Time since system boot
	Tempambient float32 //  Ambient temperature
	Humidity    float32 //  Relative humidity
}

// MsgID (generated function)
func (m *SensAtmos) MsgID() mavlink.MessageID {
	return MSG_ID_SENS_ATMOS
}

// String (generated function)
func (m *SensAtmos) String() string {
	return fmt.Sprintf(
		"&ASLUAV.SensAtmos{ Timestamp: %+v, Tempambient: %+v, Humidity: %+v }",
		m.Timestamp,
		m.Tempambient,
		m.Humidity,
	)
}

// Pack (generated function)
func (m *SensAtmos) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 16)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.Timestamp))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.Tempambient))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.Humidity))
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
func (m *SensAtmos) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 16 {
		payload = append(payload, mavlink.ZeroTail[:16-len(p.Payload)]...)
	}
	m.Timestamp = uint64(binary.LittleEndian.Uint64(payload[0:]))
	m.Tempambient = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.Humidity = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	return nil
}

// SensBatmon struct (generated typeinfo)
// Battery pack monitoring data for Li-Ion batteries
type SensBatmon struct {
	BatmonTimestamp uint64  // Time since system start
	Temperature     float32 // Battery pack temperature
	Safetystatus    uint32  // Battery monitor safetystatus report bits in Hex
	Operationstatus uint32  // Battery monitor operation status report bits in Hex
	Voltage         uint16  // Battery pack voltage
	Current         int16   // Battery pack current
	Batterystatus   uint16  // Battery monitor status report bits in Hex
	Serialnumber    uint16  // Battery monitor serial number in Hex
	Cellvoltage1    uint16  // Battery pack cell 1 voltage
	Cellvoltage2    uint16  // Battery pack cell 2 voltage
	Cellvoltage3    uint16  // Battery pack cell 3 voltage
	Cellvoltage4    uint16  // Battery pack cell 4 voltage
	Cellvoltage5    uint16  // Battery pack cell 5 voltage
	Cellvoltage6    uint16  // Battery pack cell 6 voltage
	Soc             uint8   // Battery pack state-of-charge
}

// MsgID (generated function)
func (m *SensBatmon) MsgID() mavlink.MessageID {
	return MSG_ID_SENS_BATMON
}

// String (generated function)
func (m *SensBatmon) String() string {
	return fmt.Sprintf(
		"&ASLUAV.SensBatmon{ BatmonTimestamp: %+v, Temperature: %+v, Safetystatus: %+v, Operationstatus: %+v, Voltage: %+v, Current: %+v, Batterystatus: %+v, Serialnumber: %+v, Cellvoltage1: %+v, Cellvoltage2: %+v, Cellvoltage3: %+v, Cellvoltage4: %+v, Cellvoltage5: %+v, Cellvoltage6: %+v, Soc: %+v }",
		m.BatmonTimestamp,
		m.Temperature,
		m.Safetystatus,
		m.Operationstatus,
		m.Voltage,
		m.Current,
		m.Batterystatus,
		m.Serialnumber,
		m.Cellvoltage1,
		m.Cellvoltage2,
		m.Cellvoltage3,
		m.Cellvoltage4,
		m.Cellvoltage5,
		m.Cellvoltage6,
		m.Soc,
	)
}

// Pack (generated function)
func (m *SensBatmon) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 41)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.BatmonTimestamp))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.Temperature))
	binary.LittleEndian.PutUint32(payload[12:], uint32(m.Safetystatus))
	binary.LittleEndian.PutUint32(payload[16:], uint32(m.Operationstatus))
	binary.LittleEndian.PutUint16(payload[20:], uint16(m.Voltage))
	binary.LittleEndian.PutUint16(payload[22:], uint16(m.Current))
	binary.LittleEndian.PutUint16(payload[24:], uint16(m.Batterystatus))
	binary.LittleEndian.PutUint16(payload[26:], uint16(m.Serialnumber))
	binary.LittleEndian.PutUint16(payload[28:], uint16(m.Cellvoltage1))
	binary.LittleEndian.PutUint16(payload[30:], uint16(m.Cellvoltage2))
	binary.LittleEndian.PutUint16(payload[32:], uint16(m.Cellvoltage3))
	binary.LittleEndian.PutUint16(payload[34:], uint16(m.Cellvoltage4))
	binary.LittleEndian.PutUint16(payload[36:], uint16(m.Cellvoltage5))
	binary.LittleEndian.PutUint16(payload[38:], uint16(m.Cellvoltage6))
	payload[40] = byte(m.Soc)
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
func (m *SensBatmon) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 41 {
		payload = append(payload, mavlink.ZeroTail[:41-len(p.Payload)]...)
	}
	m.BatmonTimestamp = uint64(binary.LittleEndian.Uint64(payload[0:]))
	m.Temperature = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.Safetystatus = uint32(binary.LittleEndian.Uint32(payload[12:]))
	m.Operationstatus = uint32(binary.LittleEndian.Uint32(payload[16:]))
	m.Voltage = uint16(binary.LittleEndian.Uint16(payload[20:]))
	m.Current = int16(binary.LittleEndian.Uint16(payload[22:]))
	m.Batterystatus = uint16(binary.LittleEndian.Uint16(payload[24:]))
	m.Serialnumber = uint16(binary.LittleEndian.Uint16(payload[26:]))
	m.Cellvoltage1 = uint16(binary.LittleEndian.Uint16(payload[28:]))
	m.Cellvoltage2 = uint16(binary.LittleEndian.Uint16(payload[30:]))
	m.Cellvoltage3 = uint16(binary.LittleEndian.Uint16(payload[32:]))
	m.Cellvoltage4 = uint16(binary.LittleEndian.Uint16(payload[34:]))
	m.Cellvoltage5 = uint16(binary.LittleEndian.Uint16(payload[36:]))
	m.Cellvoltage6 = uint16(binary.LittleEndian.Uint16(payload[38:]))
	m.Soc = uint8(payload[40])
	return nil
}

// FwSoaringData struct (generated typeinfo)
// Fixed-wing soaring (i.e. thermal seeking) data
type FwSoaringData struct {
	Timestamp            uint64  // Timestamp
	Timestampmodechanged uint64  // Timestamp since last mode change
	Xw                   float32 // Thermal core updraft strength
	Xr                   float32 // Thermal radius
	Xlat                 float32 // Thermal center latitude
	Xlon                 float32 // Thermal center longitude
	Varw                 float32 // Variance W
	Varr                 float32 // Variance R
	Varlat               float32 // Variance Lat
	Varlon               float32 // Variance Lon
	Loiterradius         float32 // Suggested loiter radius
	Loiterdirection      float32 // Suggested loiter direction
	Disttosoarpoint      float32 // Distance to soar point
	Vsinkexp             float32 // Expected sink rate at current airspeed, roll and throttle
	Z1Localupdraftspeed  float32 // Measurement / updraft speed at current/local airplane position
	Z2Deltaroll          float32 // Measurement / roll angle tracking error
	Z1Exp                float32 // Expected measurement 1
	Z2Exp                float32 // Expected measurement 2
	Thermalgsnorth       float32 // Thermal drift (from estimator prediction step only)
	Thermalgseast        float32 // Thermal drift (from estimator prediction step only)
	TseDot               float32 //  Total specific energy change (filtered)
	Debugvar1            float32 //  Debug variable 1
	Debugvar2            float32 //  Debug variable 2
	Controlmode          uint8   // Control Mode [-]
	Valid                uint8   // Data valid [-]
}

// MsgID (generated function)
func (m *FwSoaringData) MsgID() mavlink.MessageID {
	return MSG_ID_FW_SOARING_DATA
}

// String (generated function)
func (m *FwSoaringData) String() string {
	return fmt.Sprintf(
		"&ASLUAV.FwSoaringData{ Timestamp: %+v, Timestampmodechanged: %+v, Xw: %+v, Xr: %+v, Xlat: %+v, Xlon: %+v, Varw: %+v, Varr: %+v, Varlat: %+v, Varlon: %+v, Loiterradius: %+v, Loiterdirection: %+v, Disttosoarpoint: %+v, Vsinkexp: %+v, Z1Localupdraftspeed: %+v, Z2Deltaroll: %+v, Z1Exp: %+v, Z2Exp: %+v, Thermalgsnorth: %+v, Thermalgseast: %+v, TseDot: %+v, Debugvar1: %+v, Debugvar2: %+v, Controlmode: %+v, Valid: %+v }",
		m.Timestamp,
		m.Timestampmodechanged,
		m.Xw,
		m.Xr,
		m.Xlat,
		m.Xlon,
		m.Varw,
		m.Varr,
		m.Varlat,
		m.Varlon,
		m.Loiterradius,
		m.Loiterdirection,
		m.Disttosoarpoint,
		m.Vsinkexp,
		m.Z1Localupdraftspeed,
		m.Z2Deltaroll,
		m.Z1Exp,
		m.Z2Exp,
		m.Thermalgsnorth,
		m.Thermalgseast,
		m.TseDot,
		m.Debugvar1,
		m.Debugvar2,
		m.Controlmode,
		m.Valid,
	)
}

// Pack (generated function)
func (m *FwSoaringData) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 102)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.Timestamp))
	binary.LittleEndian.PutUint64(payload[8:], uint64(m.Timestampmodechanged))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(m.Xw))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(m.Xr))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(m.Xlat))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(m.Xlon))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(m.Varw))
	binary.LittleEndian.PutUint32(payload[36:], math.Float32bits(m.Varr))
	binary.LittleEndian.PutUint32(payload[40:], math.Float32bits(m.Varlat))
	binary.LittleEndian.PutUint32(payload[44:], math.Float32bits(m.Varlon))
	binary.LittleEndian.PutUint32(payload[48:], math.Float32bits(m.Loiterradius))
	binary.LittleEndian.PutUint32(payload[52:], math.Float32bits(m.Loiterdirection))
	binary.LittleEndian.PutUint32(payload[56:], math.Float32bits(m.Disttosoarpoint))
	binary.LittleEndian.PutUint32(payload[60:], math.Float32bits(m.Vsinkexp))
	binary.LittleEndian.PutUint32(payload[64:], math.Float32bits(m.Z1Localupdraftspeed))
	binary.LittleEndian.PutUint32(payload[68:], math.Float32bits(m.Z2Deltaroll))
	binary.LittleEndian.PutUint32(payload[72:], math.Float32bits(m.Z1Exp))
	binary.LittleEndian.PutUint32(payload[76:], math.Float32bits(m.Z2Exp))
	binary.LittleEndian.PutUint32(payload[80:], math.Float32bits(m.Thermalgsnorth))
	binary.LittleEndian.PutUint32(payload[84:], math.Float32bits(m.Thermalgseast))
	binary.LittleEndian.PutUint32(payload[88:], math.Float32bits(m.TseDot))
	binary.LittleEndian.PutUint32(payload[92:], math.Float32bits(m.Debugvar1))
	binary.LittleEndian.PutUint32(payload[96:], math.Float32bits(m.Debugvar2))
	payload[100] = byte(m.Controlmode)
	payload[101] = byte(m.Valid)
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
func (m *FwSoaringData) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 102 {
		payload = append(payload, mavlink.ZeroTail[:102-len(p.Payload)]...)
	}
	m.Timestamp = uint64(binary.LittleEndian.Uint64(payload[0:]))
	m.Timestampmodechanged = uint64(binary.LittleEndian.Uint64(payload[8:]))
	m.Xw = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	m.Xr = math.Float32frombits(binary.LittleEndian.Uint32(payload[20:]))
	m.Xlat = math.Float32frombits(binary.LittleEndian.Uint32(payload[24:]))
	m.Xlon = math.Float32frombits(binary.LittleEndian.Uint32(payload[28:]))
	m.Varw = math.Float32frombits(binary.LittleEndian.Uint32(payload[32:]))
	m.Varr = math.Float32frombits(binary.LittleEndian.Uint32(payload[36:]))
	m.Varlat = math.Float32frombits(binary.LittleEndian.Uint32(payload[40:]))
	m.Varlon = math.Float32frombits(binary.LittleEndian.Uint32(payload[44:]))
	m.Loiterradius = math.Float32frombits(binary.LittleEndian.Uint32(payload[48:]))
	m.Loiterdirection = math.Float32frombits(binary.LittleEndian.Uint32(payload[52:]))
	m.Disttosoarpoint = math.Float32frombits(binary.LittleEndian.Uint32(payload[56:]))
	m.Vsinkexp = math.Float32frombits(binary.LittleEndian.Uint32(payload[60:]))
	m.Z1Localupdraftspeed = math.Float32frombits(binary.LittleEndian.Uint32(payload[64:]))
	m.Z2Deltaroll = math.Float32frombits(binary.LittleEndian.Uint32(payload[68:]))
	m.Z1Exp = math.Float32frombits(binary.LittleEndian.Uint32(payload[72:]))
	m.Z2Exp = math.Float32frombits(binary.LittleEndian.Uint32(payload[76:]))
	m.Thermalgsnorth = math.Float32frombits(binary.LittleEndian.Uint32(payload[80:]))
	m.Thermalgseast = math.Float32frombits(binary.LittleEndian.Uint32(payload[84:]))
	m.TseDot = math.Float32frombits(binary.LittleEndian.Uint32(payload[88:]))
	m.Debugvar1 = math.Float32frombits(binary.LittleEndian.Uint32(payload[92:]))
	m.Debugvar2 = math.Float32frombits(binary.LittleEndian.Uint32(payload[96:]))
	m.Controlmode = uint8(payload[100])
	m.Valid = uint8(payload[101])
	return nil
}

// SensorpodStatus struct (generated typeinfo)
// Monitoring of sensorpod status
type SensorpodStatus struct {
	Timestamp           uint64 // Timestamp in linuxtime (since 1.1.1970)
	FreeSpace           uint16 // Free space available in recordings directory in [Gb] * 1e2
	VisensorRate1       uint8  // Rate of ROS topic 1
	VisensorRate2       uint8  // Rate of ROS topic 2
	VisensorRate3       uint8  // Rate of ROS topic 3
	VisensorRate4       uint8  // Rate of ROS topic 4
	RecordingNodesCount uint8  // Number of recording nodes
	CPUTemp             uint8  // Temperature of sensorpod CPU in
}

// MsgID (generated function)
func (m *SensorpodStatus) MsgID() mavlink.MessageID {
	return MSG_ID_SENSORPOD_STATUS
}

// String (generated function)
func (m *SensorpodStatus) String() string {
	return fmt.Sprintf(
		"&ASLUAV.SensorpodStatus{ Timestamp: %+v, FreeSpace: %+v, VisensorRate1: %+v, VisensorRate2: %+v, VisensorRate3: %+v, VisensorRate4: %+v, RecordingNodesCount: %+v, CPUTemp: %+v }",
		m.Timestamp,
		m.FreeSpace,
		m.VisensorRate1,
		m.VisensorRate2,
		m.VisensorRate3,
		m.VisensorRate4,
		m.RecordingNodesCount,
		m.CPUTemp,
	)
}

// Pack (generated function)
func (m *SensorpodStatus) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 16)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.Timestamp))
	binary.LittleEndian.PutUint16(payload[8:], uint16(m.FreeSpace))
	payload[10] = byte(m.VisensorRate1)
	payload[11] = byte(m.VisensorRate2)
	payload[12] = byte(m.VisensorRate3)
	payload[13] = byte(m.VisensorRate4)
	payload[14] = byte(m.RecordingNodesCount)
	payload[15] = byte(m.CPUTemp)
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
func (m *SensorpodStatus) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 16 {
		payload = append(payload, mavlink.ZeroTail[:16-len(p.Payload)]...)
	}
	m.Timestamp = uint64(binary.LittleEndian.Uint64(payload[0:]))
	m.FreeSpace = uint16(binary.LittleEndian.Uint16(payload[8:]))
	m.VisensorRate1 = uint8(payload[10])
	m.VisensorRate2 = uint8(payload[11])
	m.VisensorRate3 = uint8(payload[12])
	m.VisensorRate4 = uint8(payload[13])
	m.RecordingNodesCount = uint8(payload[14])
	m.CPUTemp = uint8(payload[15])
	return nil
}

// SensPowerBoard struct (generated typeinfo)
// Monitoring of power board status
type SensPowerBoard struct {
	Timestamp         uint64  // Timestamp
	PwrBrdSystemVolt  float32 // Power board system voltage
	PwrBrdServoVolt   float32 // Power board servo voltage
	PwrBrdDigitalVolt float32 // Power board digital voltage
	PwrBrdMotLAmp     float32 // Power board left motor current sensor
	PwrBrdMotRAmp     float32 // Power board right motor current sensor
	PwrBrdAnalogAmp   float32 // Power board analog current sensor
	PwrBrdDigitalAmp  float32 // Power board digital current sensor
	PwrBrdExtAmp      float32 // Power board extension current sensor
	PwrBrdAuxAmp      float32 // Power board aux current sensor
	PwrBrdStatus      uint8   // Power board status register
	PwrBrdLedStatus   uint8   // Power board leds status
}

// MsgID (generated function)
func (m *SensPowerBoard) MsgID() mavlink.MessageID {
	return MSG_ID_SENS_POWER_BOARD
}

// String (generated function)
func (m *SensPowerBoard) String() string {
	return fmt.Sprintf(
		"&ASLUAV.SensPowerBoard{ Timestamp: %+v, PwrBrdSystemVolt: %+v, PwrBrdServoVolt: %+v, PwrBrdDigitalVolt: %+v, PwrBrdMotLAmp: %+v, PwrBrdMotRAmp: %+v, PwrBrdAnalogAmp: %+v, PwrBrdDigitalAmp: %+v, PwrBrdExtAmp: %+v, PwrBrdAuxAmp: %+v, PwrBrdStatus: %+v, PwrBrdLedStatus: %+v }",
		m.Timestamp,
		m.PwrBrdSystemVolt,
		m.PwrBrdServoVolt,
		m.PwrBrdDigitalVolt,
		m.PwrBrdMotLAmp,
		m.PwrBrdMotRAmp,
		m.PwrBrdAnalogAmp,
		m.PwrBrdDigitalAmp,
		m.PwrBrdExtAmp,
		m.PwrBrdAuxAmp,
		m.PwrBrdStatus,
		m.PwrBrdLedStatus,
	)
}

// Pack (generated function)
func (m *SensPowerBoard) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 46)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.Timestamp))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.PwrBrdSystemVolt))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.PwrBrdServoVolt))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(m.PwrBrdDigitalVolt))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(m.PwrBrdMotLAmp))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(m.PwrBrdMotRAmp))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(m.PwrBrdAnalogAmp))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(m.PwrBrdDigitalAmp))
	binary.LittleEndian.PutUint32(payload[36:], math.Float32bits(m.PwrBrdExtAmp))
	binary.LittleEndian.PutUint32(payload[40:], math.Float32bits(m.PwrBrdAuxAmp))
	payload[44] = byte(m.PwrBrdStatus)
	payload[45] = byte(m.PwrBrdLedStatus)
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
func (m *SensPowerBoard) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 46 {
		payload = append(payload, mavlink.ZeroTail[:46-len(p.Payload)]...)
	}
	m.Timestamp = uint64(binary.LittleEndian.Uint64(payload[0:]))
	m.PwrBrdSystemVolt = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.PwrBrdServoVolt = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.PwrBrdDigitalVolt = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	m.PwrBrdMotLAmp = math.Float32frombits(binary.LittleEndian.Uint32(payload[20:]))
	m.PwrBrdMotRAmp = math.Float32frombits(binary.LittleEndian.Uint32(payload[24:]))
	m.PwrBrdAnalogAmp = math.Float32frombits(binary.LittleEndian.Uint32(payload[28:]))
	m.PwrBrdDigitalAmp = math.Float32frombits(binary.LittleEndian.Uint32(payload[32:]))
	m.PwrBrdExtAmp = math.Float32frombits(binary.LittleEndian.Uint32(payload[36:]))
	m.PwrBrdAuxAmp = math.Float32frombits(binary.LittleEndian.Uint32(payload[40:]))
	m.PwrBrdStatus = uint8(payload[44])
	m.PwrBrdLedStatus = uint8(payload[45])
	return nil
}

// GsmLinkStatus struct (generated typeinfo)
// Status of GSM modem (connected to onboard computer)
type GsmLinkStatus struct {
	Timestamp    uint64 // Timestamp (of OBC)
	GsmModemType uint8  // GSM modem used
	GsmLinkType  uint8  // GSM link type
	Rssi         uint8  // RSSI as reported by modem (unconverted)
	RsrpRscp     uint8  // RSRP (LTE) or RSCP (WCDMA) as reported by modem (unconverted)
	SinrEcio     uint8  // SINR (LTE) or ECIO (WCDMA) as reported by modem (unconverted)
	Rsrq         uint8  // RSRQ (LTE only) as reported by modem (unconverted)
}

// MsgID (generated function)
func (m *GsmLinkStatus) MsgID() mavlink.MessageID {
	return MSG_ID_GSM_LINK_STATUS
}

// String (generated function)
func (m *GsmLinkStatus) String() string {
	return fmt.Sprintf(
		"&ASLUAV.GsmLinkStatus{ Timestamp: %+v, GsmModemType: %+v, GsmLinkType: %+v, Rssi: %+v, RsrpRscp: %+v, SinrEcio: %+v, Rsrq: %+v }",
		m.Timestamp,
		m.GsmModemType,
		m.GsmLinkType,
		m.Rssi,
		m.RsrpRscp,
		m.SinrEcio,
		m.Rsrq,
	)
}

// Pack (generated function)
func (m *GsmLinkStatus) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 14)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.Timestamp))
	payload[8] = byte(m.GsmModemType)
	payload[9] = byte(m.GsmLinkType)
	payload[10] = byte(m.Rssi)
	payload[11] = byte(m.RsrpRscp)
	payload[12] = byte(m.SinrEcio)
	payload[13] = byte(m.Rsrq)
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
func (m *GsmLinkStatus) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 14 {
		payload = append(payload, mavlink.ZeroTail[:14-len(p.Payload)]...)
	}
	m.Timestamp = uint64(binary.LittleEndian.Uint64(payload[0:]))
	m.GsmModemType = uint8(payload[8])
	m.GsmLinkType = uint8(payload[9])
	m.Rssi = uint8(payload[10])
	m.RsrpRscp = uint8(payload[11])
	m.SinrEcio = uint8(payload[12])
	m.Rsrq = uint8(payload[13])
	return nil
}

// SatcomLinkStatus struct (generated typeinfo)
// Status of the SatCom link
type SatcomLinkStatus struct {
	Timestamp          uint64 // Timestamp
	LastHeartbeat      uint64 // Timestamp of the last successful sbd session
	FailedSessions     uint16 // Number of failed sessions
	SuccessfulSessions uint16 // Number of successful sessions
	SignalQuality      uint8  // Signal quality
	RingPending        uint8  // Ring call pending
	TxSessionPending   uint8  // Transmission session pending
	RxSessionPending   uint8  // Receiving session pending
}

// MsgID (generated function)
func (m *SatcomLinkStatus) MsgID() mavlink.MessageID {
	return MSG_ID_SATCOM_LINK_STATUS
}

// String (generated function)
func (m *SatcomLinkStatus) String() string {
	return fmt.Sprintf(
		"&ASLUAV.SatcomLinkStatus{ Timestamp: %+v, LastHeartbeat: %+v, FailedSessions: %+v, SuccessfulSessions: %+v, SignalQuality: %+v, RingPending: %+v, TxSessionPending: %+v, RxSessionPending: %+v }",
		m.Timestamp,
		m.LastHeartbeat,
		m.FailedSessions,
		m.SuccessfulSessions,
		m.SignalQuality,
		m.RingPending,
		m.TxSessionPending,
		m.RxSessionPending,
	)
}

// Pack (generated function)
func (m *SatcomLinkStatus) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 24)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.Timestamp))
	binary.LittleEndian.PutUint64(payload[8:], uint64(m.LastHeartbeat))
	binary.LittleEndian.PutUint16(payload[16:], uint16(m.FailedSessions))
	binary.LittleEndian.PutUint16(payload[18:], uint16(m.SuccessfulSessions))
	payload[20] = byte(m.SignalQuality)
	payload[21] = byte(m.RingPending)
	payload[22] = byte(m.TxSessionPending)
	payload[23] = byte(m.RxSessionPending)
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
func (m *SatcomLinkStatus) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 24 {
		payload = append(payload, mavlink.ZeroTail[:24-len(p.Payload)]...)
	}
	m.Timestamp = uint64(binary.LittleEndian.Uint64(payload[0:]))
	m.LastHeartbeat = uint64(binary.LittleEndian.Uint64(payload[8:]))
	m.FailedSessions = uint16(binary.LittleEndian.Uint16(payload[16:]))
	m.SuccessfulSessions = uint16(binary.LittleEndian.Uint16(payload[18:]))
	m.SignalQuality = uint8(payload[20])
	m.RingPending = uint8(payload[21])
	m.TxSessionPending = uint8(payload[22])
	m.RxSessionPending = uint8(payload[23])
	return nil
}

// SensorAirflowAngles struct (generated typeinfo)
// Calibrated airflow angle measurements
type SensorAirflowAngles struct {
	Timestamp          uint64  // Timestamp
	Angleofattack      float32 // Angle of attack
	Sideslip           float32 // Sideslip angle
	AngleofattackValid uint8   // Angle of attack measurement valid
	SideslipValid      uint8   // Sideslip angle measurement valid
}

// MsgID (generated function)
func (m *SensorAirflowAngles) MsgID() mavlink.MessageID {
	return MSG_ID_SENSOR_AIRFLOW_ANGLES
}

// String (generated function)
func (m *SensorAirflowAngles) String() string {
	return fmt.Sprintf(
		"&ASLUAV.SensorAirflowAngles{ Timestamp: %+v, Angleofattack: %+v, Sideslip: %+v, AngleofattackValid: %+v, SideslipValid: %+v }",
		m.Timestamp,
		m.Angleofattack,
		m.Sideslip,
		m.AngleofattackValid,
		m.SideslipValid,
	)
}

// Pack (generated function)
func (m *SensorAirflowAngles) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 18)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.Timestamp))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.Angleofattack))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.Sideslip))
	payload[16] = byte(m.AngleofattackValid)
	payload[17] = byte(m.SideslipValid)
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
func (m *SensorAirflowAngles) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 18 {
		payload = append(payload, mavlink.ZeroTail[:18-len(p.Payload)]...)
	}
	m.Timestamp = uint64(binary.LittleEndian.Uint64(payload[0:]))
	m.Angleofattack = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.Sideslip = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.AngleofattackValid = uint8(payload[16])
	m.SideslipValid = uint8(payload[17])
	return nil
}

// Message IDs
const (
	MSG_ID_COMMAND_INT_STAMPED   mavlink.MessageID = 78
	MSG_ID_COMMAND_LONG_STAMPED  mavlink.MessageID = 79
	MSG_ID_SENS_POWER            mavlink.MessageID = 201
	MSG_ID_SENS_MPPT             mavlink.MessageID = 202
	MSG_ID_ASLCTRL_DATA          mavlink.MessageID = 203
	MSG_ID_ASLCTRL_DEBUG         mavlink.MessageID = 204
	MSG_ID_ASLUAV_STATUS         mavlink.MessageID = 205
	MSG_ID_EKF_EXT               mavlink.MessageID = 206
	MSG_ID_ASL_OBCTRL            mavlink.MessageID = 207
	MSG_ID_SENS_ATMOS            mavlink.MessageID = 208
	MSG_ID_SENS_BATMON           mavlink.MessageID = 209
	MSG_ID_FW_SOARING_DATA       mavlink.MessageID = 210
	MSG_ID_SENSORPOD_STATUS      mavlink.MessageID = 211
	MSG_ID_SENS_POWER_BOARD      mavlink.MessageID = 212
	MSG_ID_GSM_LINK_STATUS       mavlink.MessageID = 213
	MSG_ID_SATCOM_LINK_STATUS    mavlink.MessageID = 214
	MSG_ID_SENSOR_AIRFLOW_ANGLES mavlink.MessageID = 215
)

// init Asluav dialect
func init() {
	mavlink.Register(MSG_ID_COMMAND_INT_STAMPED, "MSG_ID_COMMAND_INT_STAMPED", 119, func(p *mavlink.Packet) mavlink.Message {
		msg := new(CommandIntStamped)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_COMMAND_LONG_STAMPED, "MSG_ID_COMMAND_LONG_STAMPED", 102, func(p *mavlink.Packet) mavlink.Message {
		msg := new(CommandLongStamped)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_SENS_POWER, "MSG_ID_SENS_POWER", 218, func(p *mavlink.Packet) mavlink.Message {
		msg := new(SensPower)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_SENS_MPPT, "MSG_ID_SENS_MPPT", 231, func(p *mavlink.Packet) mavlink.Message {
		msg := new(SensMppt)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_ASLCTRL_DATA, "MSG_ID_ASLCTRL_DATA", 172, func(p *mavlink.Packet) mavlink.Message {
		msg := new(AslctrlData)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_ASLCTRL_DEBUG, "MSG_ID_ASLCTRL_DEBUG", 251, func(p *mavlink.Packet) mavlink.Message {
		msg := new(AslctrlDebug)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_ASLUAV_STATUS, "MSG_ID_ASLUAV_STATUS", 97, func(p *mavlink.Packet) mavlink.Message {
		msg := new(AsluavStatus)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_EKF_EXT, "MSG_ID_EKF_EXT", 64, func(p *mavlink.Packet) mavlink.Message {
		msg := new(EkfExt)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_ASL_OBCTRL, "MSG_ID_ASL_OBCTRL", 234, func(p *mavlink.Packet) mavlink.Message {
		msg := new(AslObctrl)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_SENS_ATMOS, "MSG_ID_SENS_ATMOS", 144, func(p *mavlink.Packet) mavlink.Message {
		msg := new(SensAtmos)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_SENS_BATMON, "MSG_ID_SENS_BATMON", 155, func(p *mavlink.Packet) mavlink.Message {
		msg := new(SensBatmon)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_FW_SOARING_DATA, "MSG_ID_FW_SOARING_DATA", 20, func(p *mavlink.Packet) mavlink.Message {
		msg := new(FwSoaringData)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_SENSORPOD_STATUS, "MSG_ID_SENSORPOD_STATUS", 54, func(p *mavlink.Packet) mavlink.Message {
		msg := new(SensorpodStatus)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_SENS_POWER_BOARD, "MSG_ID_SENS_POWER_BOARD", 222, func(p *mavlink.Packet) mavlink.Message {
		msg := new(SensPowerBoard)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_GSM_LINK_STATUS, "MSG_ID_GSM_LINK_STATUS", 200, func(p *mavlink.Packet) mavlink.Message {
		msg := new(GsmLinkStatus)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_SATCOM_LINK_STATUS, "MSG_ID_SATCOM_LINK_STATUS", 23, func(p *mavlink.Packet) mavlink.Message {
		msg := new(SatcomLinkStatus)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_SENSOR_AIRFLOW_ANGLES, "MSG_ID_SENSOR_AIRFLOW_ANGLES", 149, func(p *mavlink.Packet) mavlink.Message {
		msg := new(SensorAirflowAngles)
		msg.Unpack(p)
		return msg
	})
}

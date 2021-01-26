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

// AsluavCommandIntStamped struct (generated typeinfo)
// Message encoding a command with parameters as scaled integers and additional metadata. Scaling depends on the actual command value.
type AsluavCommandIntStamped struct {
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

// Dialect (generated function)
func (m *AsluavCommandIntStamped) Dialect() *Dialect {
	return DialectAsluav
}

// MsgID (generated function)
func (m *AsluavCommandIntStamped) MsgID() MessageID {
	return MSG_ID_COMMAND_INT_STAMPED
}

// MsgName (generated function)
func (m *AsluavCommandIntStamped) MsgName() string {
	return "CommandIntStamped"
}

// Pack (generated function)
func (m *AsluavCommandIntStamped) Pack(p *Packet) error {
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *AsluavCommandIntStamped) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 47 {
		return errPayloadTooSmall
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

// AsluavCommandLongStamped struct (generated typeinfo)
// Send a command with up to seven parameters to the MAV and additional metadata
type AsluavCommandLongStamped struct {
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

// Dialect (generated function)
func (m *AsluavCommandLongStamped) Dialect() *Dialect {
	return DialectAsluav
}

// MsgID (generated function)
func (m *AsluavCommandLongStamped) MsgID() MessageID {
	return MSG_ID_COMMAND_LONG_STAMPED
}

// MsgName (generated function)
func (m *AsluavCommandLongStamped) MsgName() string {
	return "CommandLongStamped"
}

// Pack (generated function)
func (m *AsluavCommandLongStamped) Pack(p *Packet) error {
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *AsluavCommandLongStamped) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 45 {
		return errPayloadTooSmall
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

// AsluavSensPower struct (generated typeinfo)
// Voltage and current sensor data
type AsluavSensPower struct {
	Adc121VspbVolt float32 //  Power board voltage sensor reading
	Adc121CspbAmp  float32 //  Power board current sensor reading
	Adc121Cs1Amp   float32 //  Board current sensor 1 reading
	Adc121Cs2Amp   float32 //  Board current sensor 2 reading
}

// Dialect (generated function)
func (m *AsluavSensPower) Dialect() *Dialect {
	return DialectAsluav
}

// MsgID (generated function)
func (m *AsluavSensPower) MsgID() MessageID {
	return MSG_ID_SENS_POWER
}

// MsgName (generated function)
func (m *AsluavSensPower) MsgName() string {
	return "SensPower"
}

// Pack (generated function)
func (m *AsluavSensPower) Pack(p *Packet) error {
	payload := make([]byte, 16)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(m.Adc121VspbVolt))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(m.Adc121CspbAmp))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.Adc121Cs1Amp))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.Adc121Cs2Amp))
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *AsluavSensPower) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 16 {
		return errPayloadTooSmall
	}
	m.Adc121VspbVolt = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.Adc121CspbAmp = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	m.Adc121Cs1Amp = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.Adc121Cs2Amp = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	return nil
}

// AsluavSensMppt struct (generated typeinfo)
// Maximum Power Point Tracker (MPPT) sensor data for solar module power performance tracking
type AsluavSensMppt struct {
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

// Dialect (generated function)
func (m *AsluavSensMppt) Dialect() *Dialect {
	return DialectAsluav
}

// MsgID (generated function)
func (m *AsluavSensMppt) MsgID() MessageID {
	return MSG_ID_SENS_MPPT
}

// MsgName (generated function)
func (m *AsluavSensMppt) MsgName() string {
	return "SensMppt"
}

// Pack (generated function)
func (m *AsluavSensMppt) Pack(p *Packet) error {
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *AsluavSensMppt) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 41 {
		return errPayloadTooSmall
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

// AsluavAslctrlData struct (generated typeinfo)
// ASL-fixed-wing controller data
type AsluavAslctrlData struct {
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

// Dialect (generated function)
func (m *AsluavAslctrlData) Dialect() *Dialect {
	return DialectAsluav
}

// MsgID (generated function)
func (m *AsluavAslctrlData) MsgID() MessageID {
	return MSG_ID_ASLCTRL_DATA
}

// MsgName (generated function)
func (m *AsluavAslctrlData) MsgName() string {
	return "AslctrlData"
}

// Pack (generated function)
func (m *AsluavAslctrlData) Pack(p *Packet) error {
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *AsluavAslctrlData) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 98 {
		return errPayloadTooSmall
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

// AsluavAslctrlDebug struct (generated typeinfo)
// ASL-fixed-wing controller debug data
type AsluavAslctrlDebug struct {
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

// Dialect (generated function)
func (m *AsluavAslctrlDebug) Dialect() *Dialect {
	return DialectAsluav
}

// MsgID (generated function)
func (m *AsluavAslctrlDebug) MsgID() MessageID {
	return MSG_ID_ASLCTRL_DEBUG
}

// MsgName (generated function)
func (m *AsluavAslctrlDebug) MsgName() string {
	return "AslctrlDebug"
}

// Pack (generated function)
func (m *AsluavAslctrlDebug) Pack(p *Packet) error {
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *AsluavAslctrlDebug) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 38 {
		return errPayloadTooSmall
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

// AsluavAsluavStatus struct (generated typeinfo)
// Extended state information for ASLUAVs
type AsluavAsluavStatus struct {
	MotorRpm     float32  //  Motor RPM
	LedStatus    uint8    //  Status of the position-indicator LEDs
	SatcomStatus uint8    //  Status of the IRIDIUM satellite communication system
	ServoStatus  [8]uint8 //  Status vector for up to 8 servos
}

// Dialect (generated function)
func (m *AsluavAsluavStatus) Dialect() *Dialect {
	return DialectAsluav
}

// MsgID (generated function)
func (m *AsluavAsluavStatus) MsgID() MessageID {
	return MSG_ID_ASLUAV_STATUS
}

// MsgName (generated function)
func (m *AsluavAsluavStatus) MsgName() string {
	return "AsluavStatus"
}

// Pack (generated function)
func (m *AsluavAsluavStatus) Pack(p *Packet) error {
	payload := make([]byte, 14)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(m.MotorRpm))
	payload[4] = byte(m.LedStatus)
	payload[5] = byte(m.SatcomStatus)
	copy(payload[6:], m.ServoStatus[:])
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *AsluavAsluavStatus) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 14 {
		return errPayloadTooSmall
	}
	m.MotorRpm = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.LedStatus = uint8(payload[4])
	m.SatcomStatus = uint8(payload[5])
	copy(m.ServoStatus[:], payload[6:14])
	return nil
}

// AsluavEkfExt struct (generated typeinfo)
// Extended EKF state estimates for ASLUAVs
type AsluavEkfExt struct {
	Timestamp uint64  //  Time since system start
	Windspeed float32 //  Magnitude of wind velocity (in lateral inertial plane)
	Winddir   float32 //  Wind heading angle from North
	Windz     float32 //  Z (Down) component of inertial wind velocity
	Airspeed  float32 //  Magnitude of air velocity
	Beta      float32 //  Sideslip angle
	Alpha     float32 //  Angle of attack
}

// Dialect (generated function)
func (m *AsluavEkfExt) Dialect() *Dialect {
	return DialectAsluav
}

// MsgID (generated function)
func (m *AsluavEkfExt) MsgID() MessageID {
	return MSG_ID_EKF_EXT
}

// MsgName (generated function)
func (m *AsluavEkfExt) MsgName() string {
	return "EkfExt"
}

// Pack (generated function)
func (m *AsluavEkfExt) Pack(p *Packet) error {
	payload := make([]byte, 32)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.Timestamp))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.Windspeed))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.Winddir))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(m.Windz))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(m.Airspeed))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(m.Beta))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(m.Alpha))
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *AsluavEkfExt) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 32 {
		return errPayloadTooSmall
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

// AsluavAslObctrl struct (generated typeinfo)
// Off-board controls/commands for ASLUAVs
type AsluavAslObctrl struct {
	Timestamp    uint64  //  Time since system start
	Uelev        float32 //  Elevator command [~]
	Uthrot       float32 //  Throttle command [~]
	Uthrot2      float32 //  Throttle 2 command [~]
	Uaill        float32 //  Left aileron command [~]
	Uailr        float32 //  Right aileron command [~]
	Urud         float32 //  Rudder command [~]
	ObctrlStatus uint8   //  Off-board computer status
}

// Dialect (generated function)
func (m *AsluavAslObctrl) Dialect() *Dialect {
	return DialectAsluav
}

// MsgID (generated function)
func (m *AsluavAslObctrl) MsgID() MessageID {
	return MSG_ID_ASL_OBCTRL
}

// MsgName (generated function)
func (m *AsluavAslObctrl) MsgName() string {
	return "AslObctrl"
}

// Pack (generated function)
func (m *AsluavAslObctrl) Pack(p *Packet) error {
	payload := make([]byte, 33)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.Timestamp))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.Uelev))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.Uthrot))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(m.Uthrot2))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(m.Uaill))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(m.Uailr))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(m.Urud))
	payload[32] = byte(m.ObctrlStatus)
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *AsluavAslObctrl) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 33 {
		return errPayloadTooSmall
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

// AsluavSensAtmos struct (generated typeinfo)
// Atmospheric sensors (temperature, humidity, ...)
type AsluavSensAtmos struct {
	Timestamp   uint64  // Time since system boot
	Tempambient float32 //  Ambient temperature
	Humidity    float32 //  Relative humidity
}

// Dialect (generated function)
func (m *AsluavSensAtmos) Dialect() *Dialect {
	return DialectAsluav
}

// MsgID (generated function)
func (m *AsluavSensAtmos) MsgID() MessageID {
	return MSG_ID_SENS_ATMOS
}

// MsgName (generated function)
func (m *AsluavSensAtmos) MsgName() string {
	return "SensAtmos"
}

// Pack (generated function)
func (m *AsluavSensAtmos) Pack(p *Packet) error {
	payload := make([]byte, 16)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.Timestamp))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.Tempambient))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.Humidity))
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *AsluavSensAtmos) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 16 {
		return errPayloadTooSmall
	}
	m.Timestamp = uint64(binary.LittleEndian.Uint64(payload[0:]))
	m.Tempambient = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.Humidity = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	return nil
}

// AsluavSensBatmon struct (generated typeinfo)
// Battery pack monitoring data for Li-Ion batteries
type AsluavSensBatmon struct {
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

// Dialect (generated function)
func (m *AsluavSensBatmon) Dialect() *Dialect {
	return DialectAsluav
}

// MsgID (generated function)
func (m *AsluavSensBatmon) MsgID() MessageID {
	return MSG_ID_SENS_BATMON
}

// MsgName (generated function)
func (m *AsluavSensBatmon) MsgName() string {
	return "SensBatmon"
}

// Pack (generated function)
func (m *AsluavSensBatmon) Pack(p *Packet) error {
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *AsluavSensBatmon) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 41 {
		return errPayloadTooSmall
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

// AsluavFwSoaringData struct (generated typeinfo)
// Fixed-wing soaring (i.e. thermal seeking) data
type AsluavFwSoaringData struct {
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

// Dialect (generated function)
func (m *AsluavFwSoaringData) Dialect() *Dialect {
	return DialectAsluav
}

// MsgID (generated function)
func (m *AsluavFwSoaringData) MsgID() MessageID {
	return MSG_ID_FW_SOARING_DATA
}

// MsgName (generated function)
func (m *AsluavFwSoaringData) MsgName() string {
	return "FwSoaringData"
}

// Pack (generated function)
func (m *AsluavFwSoaringData) Pack(p *Packet) error {
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *AsluavFwSoaringData) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 102 {
		return errPayloadTooSmall
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

// AsluavSensorpodStatus struct (generated typeinfo)
// Monitoring of sensorpod status
type AsluavSensorpodStatus struct {
	Timestamp           uint64 // Timestamp in linuxtime (since 1.1.1970)
	FreeSpace           uint16 // Free space available in recordings directory in [Gb] * 1e2
	VisensorRate1       uint8  // Rate of ROS topic 1
	VisensorRate2       uint8  // Rate of ROS topic 2
	VisensorRate3       uint8  // Rate of ROS topic 3
	VisensorRate4       uint8  // Rate of ROS topic 4
	RecordingNodesCount uint8  // Number of recording nodes
	CPUTemp             uint8  // Temperature of sensorpod CPU in
}

// Dialect (generated function)
func (m *AsluavSensorpodStatus) Dialect() *Dialect {
	return DialectAsluav
}

// MsgID (generated function)
func (m *AsluavSensorpodStatus) MsgID() MessageID {
	return MSG_ID_SENSORPOD_STATUS
}

// MsgName (generated function)
func (m *AsluavSensorpodStatus) MsgName() string {
	return "SensorpodStatus"
}

// Pack (generated function)
func (m *AsluavSensorpodStatus) Pack(p *Packet) error {
	payload := make([]byte, 16)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.Timestamp))
	binary.LittleEndian.PutUint16(payload[8:], uint16(m.FreeSpace))
	payload[10] = byte(m.VisensorRate1)
	payload[11] = byte(m.VisensorRate2)
	payload[12] = byte(m.VisensorRate3)
	payload[13] = byte(m.VisensorRate4)
	payload[14] = byte(m.RecordingNodesCount)
	payload[15] = byte(m.CPUTemp)
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *AsluavSensorpodStatus) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 16 {
		return errPayloadTooSmall
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

// AsluavSensPowerBoard struct (generated typeinfo)
// Monitoring of power board status
type AsluavSensPowerBoard struct {
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

// Dialect (generated function)
func (m *AsluavSensPowerBoard) Dialect() *Dialect {
	return DialectAsluav
}

// MsgID (generated function)
func (m *AsluavSensPowerBoard) MsgID() MessageID {
	return MSG_ID_SENS_POWER_BOARD
}

// MsgName (generated function)
func (m *AsluavSensPowerBoard) MsgName() string {
	return "SensPowerBoard"
}

// Pack (generated function)
func (m *AsluavSensPowerBoard) Pack(p *Packet) error {
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *AsluavSensPowerBoard) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 46 {
		return errPayloadTooSmall
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

// AsluavGsmLinkStatus struct (generated typeinfo)
// Status of GSM modem (connected to onboard computer)
type AsluavGsmLinkStatus struct {
	Timestamp    uint64 // Timestamp (of OBC)
	GsmModemType uint8  // GSM modem used
	GsmLinkType  uint8  // GSM link type
	Rssi         uint8  // RSSI as reported by modem (unconverted)
	RsrpRscp     uint8  // RSRP (LTE) or RSCP (WCDMA) as reported by modem (unconverted)
	SinrEcio     uint8  // SINR (LTE) or ECIO (WCDMA) as reported by modem (unconverted)
	Rsrq         uint8  // RSRQ (LTE only) as reported by modem (unconverted)
}

// Dialect (generated function)
func (m *AsluavGsmLinkStatus) Dialect() *Dialect {
	return DialectAsluav
}

// MsgID (generated function)
func (m *AsluavGsmLinkStatus) MsgID() MessageID {
	return MSG_ID_GSM_LINK_STATUS
}

// MsgName (generated function)
func (m *AsluavGsmLinkStatus) MsgName() string {
	return "GsmLinkStatus"
}

// Pack (generated function)
func (m *AsluavGsmLinkStatus) Pack(p *Packet) error {
	payload := make([]byte, 14)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.Timestamp))
	payload[8] = byte(m.GsmModemType)
	payload[9] = byte(m.GsmLinkType)
	payload[10] = byte(m.Rssi)
	payload[11] = byte(m.RsrpRscp)
	payload[12] = byte(m.SinrEcio)
	payload[13] = byte(m.Rsrq)
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *AsluavGsmLinkStatus) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 14 {
		return errPayloadTooSmall
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

// Message IDs
const (
	MSG_ID_COMMAND_INT_STAMPED  MessageID = 78
	MSG_ID_COMMAND_LONG_STAMPED MessageID = 79
	MSG_ID_SENS_POWER           MessageID = 201
	MSG_ID_SENS_MPPT            MessageID = 202
	MSG_ID_ASLCTRL_DATA         MessageID = 203
	MSG_ID_ASLCTRL_DEBUG        MessageID = 204
	MSG_ID_ASLUAV_STATUS        MessageID = 205
	MSG_ID_EKF_EXT              MessageID = 206
	MSG_ID_ASL_OBCTRL           MessageID = 207
	MSG_ID_SENS_ATMOS           MessageID = 208
	MSG_ID_SENS_BATMON          MessageID = 209
	MSG_ID_FW_SOARING_DATA      MessageID = 210
	MSG_ID_SENSORPOD_STATUS     MessageID = 211
	MSG_ID_SENS_POWER_BOARD     MessageID = 212
	MSG_ID_GSM_LINK_STATUS      MessageID = 213
)

// DialectAsluav is the dialect represented by ASLUAV.xml
var DialectAsluav = &Dialect{
	Name: "ASLUAV",
	crcExtras: map[MessageID]uint8{
		MSG_ID_COMMAND_INT_STAMPED:  119,
		MSG_ID_COMMAND_LONG_STAMPED: 102,
		MSG_ID_SENS_POWER:           218,
		MSG_ID_SENS_MPPT:            231,
		MSG_ID_ASLCTRL_DATA:         172,
		MSG_ID_ASLCTRL_DEBUG:        251,
		MSG_ID_ASLUAV_STATUS:        97,
		MSG_ID_EKF_EXT:              64,
		MSG_ID_ASL_OBCTRL:           234,
		MSG_ID_SENS_ATMOS:           144,
		MSG_ID_SENS_BATMON:          155,
		MSG_ID_FW_SOARING_DATA:      20,
		MSG_ID_SENSORPOD_STATUS:     54,
		MSG_ID_SENS_POWER_BOARD:     222,
		MSG_ID_GSM_LINK_STATUS:      200,
	},
	messageConstructorByMsgID: map[MessageID]func(*Packet) Message{
		MSG_ID_COMMAND_INT_STAMPED: func(pkt *Packet) Message {
			msg := new(AsluavCommandIntStamped)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_COMMAND_LONG_STAMPED: func(pkt *Packet) Message {
			msg := new(AsluavCommandLongStamped)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SENS_POWER: func(pkt *Packet) Message {
			msg := new(AsluavSensPower)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SENS_MPPT: func(pkt *Packet) Message {
			msg := new(AsluavSensMppt)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_ASLCTRL_DATA: func(pkt *Packet) Message {
			msg := new(AsluavAslctrlData)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_ASLCTRL_DEBUG: func(pkt *Packet) Message {
			msg := new(AsluavAslctrlDebug)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_ASLUAV_STATUS: func(pkt *Packet) Message {
			msg := new(AsluavAsluavStatus)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_EKF_EXT: func(pkt *Packet) Message {
			msg := new(AsluavEkfExt)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_ASL_OBCTRL: func(pkt *Packet) Message {
			msg := new(AsluavAslObctrl)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SENS_ATMOS: func(pkt *Packet) Message {
			msg := new(AsluavSensAtmos)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SENS_BATMON: func(pkt *Packet) Message {
			msg := new(AsluavSensBatmon)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_FW_SOARING_DATA: func(pkt *Packet) Message {
			msg := new(AsluavFwSoaringData)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SENSORPOD_STATUS: func(pkt *Packet) Message {
			msg := new(AsluavSensorpodStatus)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SENS_POWER_BOARD: func(pkt *Packet) Message {
			msg := new(AsluavSensPowerBoard)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_GSM_LINK_STATUS: func(pkt *Packet) Message {
			msg := new(AsluavGsmLinkStatus)
			msg.Unpack(pkt)
			return msg
		},
	},
}

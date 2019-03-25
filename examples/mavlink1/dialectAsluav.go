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
	MAV_CMD_RESET_MPPT      = 40001 // Mission command to reset Maximum Power Point Tracker (MPPT)
	MAV_CMD_PAYLOAD_CONTROL = 40002 // Mission command to perform a power cycle on payload
)

// AsluavSensPower struct (generated typeinfo)
// Voltage and current sensor data
type AsluavSensPower struct {
	Adc121VspbVolt float32 //  Power board voltage sensor reading in volts
	Adc121CspbAmp  float32 //  Power board current sensor reading in amps
	Adc121Cs1Amp   float32 //  Board current sensor 1 reading in amps
	Adc121Cs2Amp   float32 //  Board current sensor 2 reading in amps
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
	if MavlinkVersion > 1 {
		payloadLen := len(payload)
		for payloadLen > 1 && payload[payloadLen-1] == 0 {
			payloadLen--
		}
		payload = payload[:payloadLen]
	}
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *AsluavSensPower) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 16 {
		if MavlinkVersion == 1 {
			return errPayloadTooSmall
		}
		payload = append(payload, zeroTail[:16-len(p.Payload)]...)
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
	Mppt3Volt     float32 //  MPPT3 voltage
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
	if MavlinkVersion > 1 {
		payloadLen := len(payload)
		for payloadLen > 1 && payload[payloadLen-1] == 0 {
			payloadLen--
		}
		payload = payload[:payloadLen]
	}
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *AsluavSensMppt) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 41 {
		if MavlinkVersion == 1 {
			return errPayloadTooSmall
		}
		payload = append(payload, zeroTail[:41-len(p.Payload)]...)
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
	Pitchangle      float32 // Pitch angle [deg]
	Pitchangleref   float32 // Pitch angle reference[deg]
	Q               float32 //
	Qref            float32 //
	Uelev           float32 //
	Uthrot          float32 //
	Uthrot2         float32 //
	Nz              float32 //
	Airspeedref     float32 // Airspeed reference [m/s]
	Yawangle        float32 // Yaw angle [deg]
	Yawangleref     float32 // Yaw angle reference[deg]
	Rollangle       float32 // Roll angle [deg]
	Rollangleref    float32 // Roll angle reference[deg]
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
	if MavlinkVersion > 1 {
		payloadLen := len(payload)
		for payloadLen > 1 && payload[payloadLen-1] == 0 {
			payloadLen--
		}
		payload = payload[:payloadLen]
	}
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *AsluavAslctrlData) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 98 {
		if MavlinkVersion == 1 {
			return errPayloadTooSmall
		}
		payload = append(payload, zeroTail[:98-len(p.Payload)]...)
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
	if MavlinkVersion > 1 {
		payloadLen := len(payload)
		for payloadLen > 1 && payload[payloadLen-1] == 0 {
			payloadLen--
		}
		payload = payload[:payloadLen]
	}
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *AsluavAslctrlDebug) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 38 {
		if MavlinkVersion == 1 {
			return errPayloadTooSmall
		}
		payload = append(payload, zeroTail[:38-len(p.Payload)]...)
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
	if MavlinkVersion > 1 {
		payloadLen := len(payload)
		for payloadLen > 1 && payload[payloadLen-1] == 0 {
			payloadLen--
		}
		payload = payload[:payloadLen]
	}
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *AsluavAsluavStatus) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 14 {
		if MavlinkVersion == 1 {
			return errPayloadTooSmall
		}
		payload = append(payload, zeroTail[:14-len(p.Payload)]...)
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
	Timestamp uint64  //  Time since system start [us]
	Windspeed float32 //  Magnitude of wind velocity (in lateral inertial plane) [m/s]
	Winddir   float32 //  Wind heading angle from North [rad]
	Windz     float32 //  Z (Down) component of inertial wind velocity [m/s]
	Airspeed  float32 //  Magnitude of air velocity [m/s]
	Beta      float32 //  Sideslip angle [rad]
	Alpha     float32 //  Angle of attack [rad]
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
	if MavlinkVersion > 1 {
		payloadLen := len(payload)
		for payloadLen > 1 && payload[payloadLen-1] == 0 {
			payloadLen--
		}
		payload = payload[:payloadLen]
	}
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *AsluavEkfExt) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 32 {
		if MavlinkVersion == 1 {
			return errPayloadTooSmall
		}
		payload = append(payload, zeroTail[:32-len(p.Payload)]...)
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
	Timestamp    uint64  //  Time since system start [us]
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
	if MavlinkVersion > 1 {
		payloadLen := len(payload)
		for payloadLen > 1 && payload[payloadLen-1] == 0 {
			payloadLen--
		}
		payload = payload[:payloadLen]
	}
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *AsluavAslObctrl) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 33 {
		if MavlinkVersion == 1 {
			return errPayloadTooSmall
		}
		payload = append(payload, zeroTail[:33-len(p.Payload)]...)
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
	Tempambient float32 //  Ambient temperature [degrees Celsius]
	Humidity    float32 //  Relative humidity [%]
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
	payload := make([]byte, 8)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(m.Tempambient))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(m.Humidity))
	if MavlinkVersion > 1 {
		payloadLen := len(payload)
		for payloadLen > 1 && payload[payloadLen-1] == 0 {
			payloadLen--
		}
		payload = payload[:payloadLen]
	}
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *AsluavSensAtmos) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 8 {
		if MavlinkVersion == 1 {
			return errPayloadTooSmall
		}
		payload = append(payload, zeroTail[:8-len(p.Payload)]...)
	}
	m.Tempambient = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.Humidity = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	return nil
}

// AsluavSensBatmon struct (generated typeinfo)
// Battery pack monitoring data for Li-Ion batteries
type AsluavSensBatmon struct {
	Temperature    float32 // Battery pack temperature in [deg C]
	Voltage        uint16  // Battery pack voltage in [mV]
	Current        int16   // Battery pack current in [mA]
	Batterystatus  uint16  // Battery monitor status report bits in Hex
	Serialnumber   uint16  // Battery monitor serial number in Hex
	Hostfetcontrol uint16  // Battery monitor sensor host FET control in Hex
	Cellvoltage1   uint16  // Battery pack cell 1 voltage in [mV]
	Cellvoltage2   uint16  // Battery pack cell 2 voltage in [mV]
	Cellvoltage3   uint16  // Battery pack cell 3 voltage in [mV]
	Cellvoltage4   uint16  // Battery pack cell 4 voltage in [mV]
	Cellvoltage5   uint16  // Battery pack cell 5 voltage in [mV]
	Cellvoltage6   uint16  // Battery pack cell 6 voltage in [mV]
	Soc            uint8   // Battery pack state-of-charge
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
	payload := make([]byte, 27)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(m.Temperature))
	binary.LittleEndian.PutUint16(payload[4:], uint16(m.Voltage))
	binary.LittleEndian.PutUint16(payload[6:], uint16(m.Current))
	binary.LittleEndian.PutUint16(payload[8:], uint16(m.Batterystatus))
	binary.LittleEndian.PutUint16(payload[10:], uint16(m.Serialnumber))
	binary.LittleEndian.PutUint16(payload[12:], uint16(m.Hostfetcontrol))
	binary.LittleEndian.PutUint16(payload[14:], uint16(m.Cellvoltage1))
	binary.LittleEndian.PutUint16(payload[16:], uint16(m.Cellvoltage2))
	binary.LittleEndian.PutUint16(payload[18:], uint16(m.Cellvoltage3))
	binary.LittleEndian.PutUint16(payload[20:], uint16(m.Cellvoltage4))
	binary.LittleEndian.PutUint16(payload[22:], uint16(m.Cellvoltage5))
	binary.LittleEndian.PutUint16(payload[24:], uint16(m.Cellvoltage6))
	payload[26] = byte(m.Soc)
	if MavlinkVersion > 1 {
		payloadLen := len(payload)
		for payloadLen > 1 && payload[payloadLen-1] == 0 {
			payloadLen--
		}
		payload = payload[:payloadLen]
	}
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *AsluavSensBatmon) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 27 {
		if MavlinkVersion == 1 {
			return errPayloadTooSmall
		}
		payload = append(payload, zeroTail[:27-len(p.Payload)]...)
	}
	m.Temperature = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.Voltage = uint16(binary.LittleEndian.Uint16(payload[4:]))
	m.Current = int16(binary.LittleEndian.Uint16(payload[6:]))
	m.Batterystatus = uint16(binary.LittleEndian.Uint16(payload[8:]))
	m.Serialnumber = uint16(binary.LittleEndian.Uint16(payload[10:]))
	m.Hostfetcontrol = uint16(binary.LittleEndian.Uint16(payload[12:]))
	m.Cellvoltage1 = uint16(binary.LittleEndian.Uint16(payload[14:]))
	m.Cellvoltage2 = uint16(binary.LittleEndian.Uint16(payload[16:]))
	m.Cellvoltage3 = uint16(binary.LittleEndian.Uint16(payload[18:]))
	m.Cellvoltage4 = uint16(binary.LittleEndian.Uint16(payload[20:]))
	m.Cellvoltage5 = uint16(binary.LittleEndian.Uint16(payload[22:]))
	m.Cellvoltage6 = uint16(binary.LittleEndian.Uint16(payload[24:]))
	m.Soc = uint8(payload[26])
	return nil
}

// AsluavFwSoaringData struct (generated typeinfo)
// Fixed-wing soaring (i.e. thermal seeking) data
type AsluavFwSoaringData struct {
	Timestamp            uint64  // Timestamp [ms]
	Timestampmodechanged uint64  // Timestamp since last mode change[ms]
	Xw                   float32 // Thermal core updraft strength [m/s]
	Xr                   float32 // Thermal radius [m]
	Xlat                 float32 // Thermal center latitude [deg]
	Xlon                 float32 // Thermal center longitude [deg]
	Varw                 float32 // Variance W
	Varr                 float32 // Variance R
	Varlat               float32 // Variance Lat
	Varlon               float32 // Variance Lon
	Loiterradius         float32 // Suggested loiter radius [m]
	Loiterdirection      float32 // Suggested loiter direction
	Disttosoarpoint      float32 // Distance to soar point [m]
	Vsinkexp             float32 // Expected sink rate at current airspeed, roll and throttle [m/s]
	Z1Localupdraftspeed  float32 // Measurement / updraft speed at current/local airplane position [m/s]
	Z2Deltaroll          float32 // Measurement / roll angle tracking error [deg]
	Z1Exp                float32 // Expected measurement 1
	Z2Exp                float32 // Expected measurement 2
	Thermalgsnorth       float32 // Thermal drift (from estimator prediction step only) [m/s]
	Thermalgseast        float32 // Thermal drift (from estimator prediction step only) [m/s]
	TseDot               float32 //  Total specific energy change (filtered) [m/s]
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
	if MavlinkVersion > 1 {
		payloadLen := len(payload)
		for payloadLen > 1 && payload[payloadLen-1] == 0 {
			payloadLen--
		}
		payload = payload[:payloadLen]
	}
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *AsluavFwSoaringData) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 102 {
		if MavlinkVersion == 1 {
			return errPayloadTooSmall
		}
		payload = append(payload, zeroTail[:102-len(p.Payload)]...)
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
	Timestamp           uint64 // Timestamp in linuxtime [ms] (since 1.1.1970)
	FreeSpace           uint16 // Free space available in recordings directory in [Gb] * 1e2
	VisensorRate1       uint8  // Rate of ROS topic 1
	VisensorRate2       uint8  // Rate of ROS topic 2
	VisensorRate3       uint8  // Rate of ROS topic 3
	VisensorRate4       uint8  // Rate of ROS topic 4
	RecordingNodesCount uint8  // Number of recording nodes
	CPUTemp             uint8  // Temperature of sensorpod CPU in [deg C]
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
	if MavlinkVersion > 1 {
		payloadLen := len(payload)
		for payloadLen > 1 && payload[payloadLen-1] == 0 {
			payloadLen--
		}
		payload = payload[:payloadLen]
	}
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *AsluavSensorpodStatus) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 16 {
		if MavlinkVersion == 1 {
			return errPayloadTooSmall
		}
		payload = append(payload, zeroTail[:16-len(p.Payload)]...)
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
	Timestamp        uint64  // Timestamp
	PwrBrdSystemVolt float32 // Power board system voltage
	PwrBrdServoVolt  float32 // Power board servo voltage
	PwrBrdMotLAmp    float32 // Power board left motor current sensor
	PwrBrdMotRAmp    float32 // Power board right motor current sensor
	PwrBrdServo1Amp  float32 // Power board servo1 current sensor
	PwrBrdServo2Amp  float32 // Power board servo1 current sensor
	PwrBrdServo3Amp  float32 // Power board servo1 current sensor
	PwrBrdServo4Amp  float32 // Power board servo1 current sensor
	PwrBrdAuxAmp     float32 // Power board aux current sensor
	PwrBrdStatus     uint8   // Power board status register
	PwrBrdLedStatus  uint8   // Power board leds status
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
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(m.PwrBrdMotLAmp))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(m.PwrBrdMotRAmp))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(m.PwrBrdServo1Amp))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(m.PwrBrdServo2Amp))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(m.PwrBrdServo3Amp))
	binary.LittleEndian.PutUint32(payload[36:], math.Float32bits(m.PwrBrdServo4Amp))
	binary.LittleEndian.PutUint32(payload[40:], math.Float32bits(m.PwrBrdAuxAmp))
	payload[44] = byte(m.PwrBrdStatus)
	payload[45] = byte(m.PwrBrdLedStatus)
	if MavlinkVersion > 1 {
		payloadLen := len(payload)
		for payloadLen > 1 && payload[payloadLen-1] == 0 {
			payloadLen--
		}
		payload = payload[:payloadLen]
	}
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *AsluavSensPowerBoard) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 46 {
		if MavlinkVersion == 1 {
			return errPayloadTooSmall
		}
		payload = append(payload, zeroTail[:46-len(p.Payload)]...)
	}
	m.Timestamp = uint64(binary.LittleEndian.Uint64(payload[0:]))
	m.PwrBrdSystemVolt = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.PwrBrdServoVolt = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.PwrBrdMotLAmp = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	m.PwrBrdMotRAmp = math.Float32frombits(binary.LittleEndian.Uint32(payload[20:]))
	m.PwrBrdServo1Amp = math.Float32frombits(binary.LittleEndian.Uint32(payload[24:]))
	m.PwrBrdServo2Amp = math.Float32frombits(binary.LittleEndian.Uint32(payload[28:]))
	m.PwrBrdServo3Amp = math.Float32frombits(binary.LittleEndian.Uint32(payload[32:]))
	m.PwrBrdServo4Amp = math.Float32frombits(binary.LittleEndian.Uint32(payload[36:]))
	m.PwrBrdAuxAmp = math.Float32frombits(binary.LittleEndian.Uint32(payload[40:]))
	m.PwrBrdStatus = uint8(payload[44])
	m.PwrBrdLedStatus = uint8(payload[45])
	return nil
}

// Message IDs
const (
	MSG_ID_SENS_POWER       MessageID = 201
	MSG_ID_SENS_MPPT        MessageID = 202
	MSG_ID_ASLCTRL_DATA     MessageID = 203
	MSG_ID_ASLCTRL_DEBUG    MessageID = 204
	MSG_ID_ASLUAV_STATUS    MessageID = 205
	MSG_ID_EKF_EXT          MessageID = 206
	MSG_ID_ASL_OBCTRL       MessageID = 207
	MSG_ID_SENS_ATMOS       MessageID = 208
	MSG_ID_SENS_BATMON      MessageID = 209
	MSG_ID_FW_SOARING_DATA  MessageID = 210
	MSG_ID_SENSORPOD_STATUS MessageID = 211
	MSG_ID_SENS_POWER_BOARD MessageID = 212
)

// DialectAsluav is the dialect represented by ASLUAV.xml
var DialectAsluav = &Dialect{
	Name: "ASLUAV",
	crcExtras: map[MessageID]uint8{
		MSG_ID_SENS_POWER:       218,
		MSG_ID_SENS_MPPT:        231,
		MSG_ID_ASLCTRL_DATA:     172,
		MSG_ID_ASLCTRL_DEBUG:    251,
		MSG_ID_ASLUAV_STATUS:    97,
		MSG_ID_EKF_EXT:          64,
		MSG_ID_ASL_OBCTRL:       234,
		MSG_ID_SENS_ATMOS:       175,
		MSG_ID_SENS_BATMON:      62,
		MSG_ID_FW_SOARING_DATA:  20,
		MSG_ID_SENSORPOD_STATUS: 54,
		MSG_ID_SENS_POWER_BOARD: 242,
	},
	messageConstructorByMsgID: map[MessageID]func(*Packet) Message{
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
	},
}

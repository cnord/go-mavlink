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

// UalbertaAutopilotMode (generated enum)
// Available autopilot modes for ualberta uav
const (
	MODE_MANUAL_DIRECT = 0 // Raw input pulse widts sent to output
	MODE_MANUAL_SCALED = 1 // Inputs are normalized using calibration, the converted back to raw pulse widths for output
	MODE_AUTO_PID_ATT  = 2 //  dfsdfs
	MODE_AUTO_PID_VEL  = 3 //  dfsfds
	MODE_AUTO_PID_POS  = 4 //  dfsdfsdfs
)

// UalbertaNavMode (generated enum)
// Navigation filter mode
const (
	NAV_AHRS_INIT    = 0 //
	NAV_AHRS         = 1 // AHRS mode
	NAV_INS_GPS_INIT = 2 // INS/GPS initialization mode
	NAV_INS_GPS      = 3 // INS/GPS mode
)

// UalbertaPilotMode (generated enum)
// Mode currently commanded by pilot
const (
	PILOT_MANUAL = 0 //  sdf
	PILOT_AUTO   = 1 //  dfs
	PILOT_ROTO   = 2 //  Rotomotion mode
)

// UalbertaNavFilterBias struct (generated typeinfo)
// Accelerometer and Gyro biases from the navigation filter
type UalbertaNavFilterBias struct {
	Usec   uint64  // Timestamp (microseconds)
	Accel0 float32 // b_f[0]
	Accel1 float32 // b_f[1]
	Accel2 float32 // b_f[2]
	Gyro0  float32 // b_f[0]
	Gyro1  float32 // b_f[1]
	Gyro2  float32 // b_f[2]
}

// Dialect (generated function)
func (m *UalbertaNavFilterBias) Dialect() *Dialect {
	return DialectUalberta
}

// MsgID (generated function)
func (m *UalbertaNavFilterBias) MsgID() MessageID {
	return MSG_ID_NAV_FILTER_BIAS
}

// MsgName (generated function)
func (m *UalbertaNavFilterBias) MsgName() string {
	return "NavFilterBias"
}

// Pack (generated function)
func (m *UalbertaNavFilterBias) Pack(p *Packet) error {
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
func (m *UalbertaNavFilterBias) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 32 {
		payload = append(payload, zeroTail[:32-len(p.Payload)]...)
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

// UalbertaRadioCalibration struct (generated typeinfo)
// Complete set of calibration parameters for the radio
type UalbertaRadioCalibration struct {
	Aileron  [3]uint16 // Aileron setpoints: left, center, right
	Elevator [3]uint16 // Elevator setpoints: nose down, center, nose up
	Rudder   [3]uint16 // Rudder setpoints: nose left, center, nose right
	Gyro     [2]uint16 // Tail gyro mode/gain setpoints: heading hold, rate mode
	Pitch    [5]uint16 // Pitch curve setpoints (every 25%)
	Throttle [5]uint16 // Throttle curve setpoints (every 25%)
}

// Dialect (generated function)
func (m *UalbertaRadioCalibration) Dialect() *Dialect {
	return DialectUalberta
}

// MsgID (generated function)
func (m *UalbertaRadioCalibration) MsgID() MessageID {
	return MSG_ID_RADIO_CALIBRATION
}

// MsgName (generated function)
func (m *UalbertaRadioCalibration) MsgName() string {
	return "RadioCalibration"
}

// Pack (generated function)
func (m *UalbertaRadioCalibration) Pack(p *Packet) error {
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
func (m *UalbertaRadioCalibration) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 42 {
		payload = append(payload, zeroTail[:42-len(p.Payload)]...)
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

// UalbertaUalbertaSysStatus struct (generated typeinfo)
// System status specific to ualberta uav
type UalbertaUalbertaSysStatus struct {
	Mode    uint8 // System mode, see UALBERTA_AUTOPILOT_MODE ENUM
	NavMode uint8 // Navigation mode, see UALBERTA_NAV_MODE ENUM
	Pilot   uint8 // Pilot mode, see UALBERTA_PILOT_MODE
}

// Dialect (generated function)
func (m *UalbertaUalbertaSysStatus) Dialect() *Dialect {
	return DialectUalberta
}

// MsgID (generated function)
func (m *UalbertaUalbertaSysStatus) MsgID() MessageID {
	return MSG_ID_UALBERTA_SYS_STATUS
}

// MsgName (generated function)
func (m *UalbertaUalbertaSysStatus) MsgName() string {
	return "UalbertaSysStatus"
}

// Pack (generated function)
func (m *UalbertaUalbertaSysStatus) Pack(p *Packet) error {
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
func (m *UalbertaUalbertaSysStatus) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 3 {
		payload = append(payload, zeroTail[:3-len(p.Payload)]...)
	}
	m.Mode = uint8(payload[0])
	m.NavMode = uint8(payload[1])
	m.Pilot = uint8(payload[2])
	return nil
}

// Message IDs
const (
	MSG_ID_NAV_FILTER_BIAS     MessageID = 220
	MSG_ID_RADIO_CALIBRATION   MessageID = 221
	MSG_ID_UALBERTA_SYS_STATUS MessageID = 222
)

// DialectUalberta is the dialect represented by ualberta.xml
var DialectUalberta = &Dialect{
	Name: "ualberta",
	crcExtras: map[MessageID]uint8{
		MSG_ID_NAV_FILTER_BIAS:     34,
		MSG_ID_RADIO_CALIBRATION:   71,
		MSG_ID_UALBERTA_SYS_STATUS: 15,
	},
	messageConstructorByMsgID: map[MessageID]func(*Packet) Message{
		MSG_ID_NAV_FILTER_BIAS: func(pkt *Packet) Message {
			msg := new(UalbertaNavFilterBias)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_RADIO_CALIBRATION: func(pkt *Packet) Message {
			msg := new(UalbertaRadioCalibration)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_UALBERTA_SYS_STATUS: func(pkt *Packet) Message {
			msg := new(UalbertaUalbertaSysStatus)
			msg.Unpack(pkt)
			return msg
		},
	},
}

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

// UalbertaAutopilotMode: Available autopilot modes for ualberta uav
const (
	MODE_MANUAL_DIRECT = 0 // Raw input pulse widts sent to output
	MODE_MANUAL_SCALED = 1 // Inputs are normalized using calibration, the converted back to raw pulse widths for output
	MODE_AUTO_PID_ATT  = 2 //  dfsdfs
	MODE_AUTO_PID_VEL  = 3 //  dfsfds
	MODE_AUTO_PID_POS  = 4 //  dfsdfsdfs
)

// UalbertaNavMode: Navigation filter mode
const (
	NAV_AHRS_INIT    = 0 //
	NAV_AHRS         = 1 // AHRS mode
	NAV_INS_GPS_INIT = 2 // INS/GPS initialization mode
	NAV_INS_GPS      = 3 // INS/GPS mode
)

// UalbertaPilotMode: Mode currently commanded by pilot
const (
	PILOT_MANUAL = 0 //  sdf
	PILOT_AUTO   = 1 //  dfs
	PILOT_ROTO   = 2 //  Rotomotion mode
)

// NavFilterBias struct (generated typeinfo)
// Accelerometer and Gyro biases from the navigation filter
type NavFilterBias struct {
	Usec   uint64  // Timestamp (microseconds)
	Accel0 float32 // b_f[0]
	Accel1 float32 // b_f[1]
	Accel2 float32 // b_f[2]
	Gyro0  float32 // b_f[0]
	Gyro1  float32 // b_f[1]
	Gyro2  float32 // b_f[2]
}

// MsgID (generated function)
func (m *NavFilterBias) MsgID() MessageID {
	return MSG_ID_NAV_FILTER_BIAS
}

// MsgName (generated function)
func (m *NavFilterBias) MsgName() string {
	return "NavFilterBias"
}

// Pack (generated function)
func (m *NavFilterBias) Pack(p *Packet) error {
	payload := make([]byte, 32)
	binary.LittleEndian.PutUint64(payload[0:], uint64(m.Usec))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.Accel0))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.Accel1))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(m.Accel2))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(m.Gyro0))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(m.Gyro1))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(m.Gyro2))

	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *NavFilterBias) Unpack(p *Packet) error {
	if len(p.Payload) < 32 {
		return fmt.Errorf("payload too small")
	}
	m.Usec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	m.Accel0 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	m.Accel1 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	m.Accel2 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	m.Gyro0 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	m.Gyro1 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	m.Gyro2 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	return nil
}

// RadioCalibration struct (generated typeinfo)
// Complete set of calibration parameters for the radio
type RadioCalibration struct {
	Aileron  [3]uint16 // Aileron setpoints: left, center, right
	Elevator [3]uint16 // Elevator setpoints: nose down, center, nose up
	Rudder   [3]uint16 // Rudder setpoints: nose left, center, nose right
	Gyro     [2]uint16 // Tail gyro mode/gain setpoints: heading hold, rate mode
	Pitch    [5]uint16 // Pitch curve setpoints (every 25%)
	Throttle [5]uint16 // Throttle curve setpoints (every 25%)
}

// MsgID (generated function)
func (m *RadioCalibration) MsgID() MessageID {
	return MSG_ID_RADIO_CALIBRATION
}

// MsgName (generated function)
func (m *RadioCalibration) MsgName() string {
	return "RadioCalibration"
}

// Pack (generated function)
func (m *RadioCalibration) Pack(p *Packet) error {
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

	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *RadioCalibration) Unpack(p *Packet) error {
	if len(p.Payload) < 42 {
		return fmt.Errorf("payload too small")
	}
	for i := 0; i < len(m.Aileron); i++ {
		m.Aileron[i] = uint16(binary.LittleEndian.Uint16(p.Payload[0+i*2:]))
	}
	for i := 0; i < len(m.Elevator); i++ {
		m.Elevator[i] = uint16(binary.LittleEndian.Uint16(p.Payload[6+i*2:]))
	}
	for i := 0; i < len(m.Rudder); i++ {
		m.Rudder[i] = uint16(binary.LittleEndian.Uint16(p.Payload[12+i*2:]))
	}
	for i := 0; i < len(m.Gyro); i++ {
		m.Gyro[i] = uint16(binary.LittleEndian.Uint16(p.Payload[18+i*2:]))
	}
	for i := 0; i < len(m.Pitch); i++ {
		m.Pitch[i] = uint16(binary.LittleEndian.Uint16(p.Payload[22+i*2:]))
	}
	for i := 0; i < len(m.Throttle); i++ {
		m.Throttle[i] = uint16(binary.LittleEndian.Uint16(p.Payload[32+i*2:]))
	}
	return nil
}

// UalbertaSysStatus struct (generated typeinfo)
// System status specific to ualberta uav
type UalbertaSysStatus struct {
	Mode    uint8 // System mode, see UALBERTA_AUTOPILOT_MODE ENUM
	NavMode uint8 // Navigation mode, see UALBERTA_NAV_MODE ENUM
	Pilot   uint8 // Pilot mode, see UALBERTA_PILOT_MODE
}

// MsgID (generated function)
func (m *UalbertaSysStatus) MsgID() MessageID {
	return MSG_ID_UALBERTA_SYS_STATUS
}

// MsgName (generated function)
func (m *UalbertaSysStatus) MsgName() string {
	return "UalbertaSysStatus"
}

// Pack (generated function)
func (m *UalbertaSysStatus) Pack(p *Packet) error {
	payload := make([]byte, 3)
	payload[0] = byte(m.Mode)
	payload[1] = byte(m.NavMode)
	payload[2] = byte(m.Pilot)

	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *UalbertaSysStatus) Unpack(p *Packet) error {
	if len(p.Payload) < 3 {
		return fmt.Errorf("payload too small")
	}
	m.Mode = uint8(p.Payload[0])
	m.NavMode = uint8(p.Payload[1])
	m.Pilot = uint8(p.Payload[2])
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
			msg := new(NavFilterBias)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_RADIO_CALIBRATION: func(pkt *Packet) Message {
			msg := new(RadioCalibration)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_UALBERTA_SYS_STATUS: func(pkt *Packet) Message {
			msg := new(UalbertaSysStatus)
			msg.Unpack(pkt)
			return msg
		},
	},
}

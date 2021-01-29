//////////////////////////////////////////////////
//
// NOTE: do not edit,
// this file created automatically by mavgen.go
//
//////////////////////////////////////////////////

package ualberta

import (
	mavlink ".."
	"encoding/binary"
	"fmt"
	"math"
)

// UalbertaAutopilotMode (generated enum)
// Available autopilot modes for ualberta uav
const (
	MODE_MANUAL_DIRECT = 1 // Raw input pulse widts sent to output
	MODE_MANUAL_SCALED = 2 // Inputs are normalized using calibration, the converted back to raw pulse widths for output
	MODE_AUTO_PID_ATT  = 3 // dfsdfs
	MODE_AUTO_PID_VEL  = 4 // dfsfds
	MODE_AUTO_PID_POS  = 5 // dfsdfsdfs
)

// UalbertaNavMode (generated enum)
// Navigation filter mode
const (
	NAV_AHRS_INIT    = 1 //
	NAV_AHRS         = 2 // AHRS mode
	NAV_INS_GPS_INIT = 3 // INS/GPS initialization mode
	NAV_INS_GPS      = 4 // INS/GPS mode
)

// UalbertaPilotMode (generated enum)
// Mode currently commanded by pilot
const (
	PILOT_MANUAL = 1 // sdf
	PILOT_AUTO   = 2 // dfs
	PILOT_ROTO   = 3 // Rotomotion mode
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
func (m *NavFilterBias) MsgID() mavlink.MessageID {
	return MSG_ID_NAV_FILTER_BIAS
}

// String (generated function)
func (m *NavFilterBias) String() string {
	return fmt.Sprintf(
		"&ualberta.NavFilterBias{ Usec: %+v, Accel0: %+v, Accel1: %+v, Accel2: %+v, Gyro0: %+v, Gyro1: %+v, Gyro2: %+v }",
		m.Usec,
		m.Accel0,
		m.Accel1,
		m.Accel2,
		m.Gyro0,
		m.Gyro1,
		m.Gyro2,
	)
}

// Pack (generated function)
func (m *NavFilterBias) Pack(p *mavlink.Packet) error {
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
func (m *NavFilterBias) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 32 {
		return mavlink.ErrPayloadTooSmall
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
func (m *RadioCalibration) MsgID() mavlink.MessageID {
	return MSG_ID_RADIO_CALIBRATION
}

// String (generated function)
func (m *RadioCalibration) String() string {
	return fmt.Sprintf(
		"&ualberta.RadioCalibration{ Aileron: %+v, Elevator: %+v, Rudder: %+v, Gyro: %+v, Pitch: %+v, Throttle: %+v }",
		m.Aileron,
		m.Elevator,
		m.Rudder,
		m.Gyro,
		m.Pitch,
		m.Throttle,
	)
}

// Pack (generated function)
func (m *RadioCalibration) Pack(p *mavlink.Packet) error {
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
func (m *RadioCalibration) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 42 {
		return mavlink.ErrPayloadTooSmall
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

// UalbertaSysStatus struct (generated typeinfo)
// System status specific to ualberta uav
type UalbertaSysStatus struct {
	Mode    uint8 // System mode, see UALBERTA_AUTOPILOT_MODE ENUM
	NavMode uint8 // Navigation mode, see UALBERTA_NAV_MODE ENUM
	Pilot   uint8 // Pilot mode, see UALBERTA_PILOT_MODE
}

// MsgID (generated function)
func (m *UalbertaSysStatus) MsgID() mavlink.MessageID {
	return MSG_ID_UALBERTA_SYS_STATUS
}

// String (generated function)
func (m *UalbertaSysStatus) String() string {
	return fmt.Sprintf(
		"&ualberta.UalbertaSysStatus{ Mode: %+v, NavMode: %+v, Pilot: %+v }",
		m.Mode,
		m.NavMode,
		m.Pilot,
	)
}

// Pack (generated function)
func (m *UalbertaSysStatus) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 3)
	payload[0] = byte(m.Mode)
	payload[1] = byte(m.NavMode)
	payload[2] = byte(m.Pilot)
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *UalbertaSysStatus) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 3 {
		return mavlink.ErrPayloadTooSmall
	}
	m.Mode = uint8(payload[0])
	m.NavMode = uint8(payload[1])
	m.Pilot = uint8(payload[2])
	return nil
}

// Message IDs
const (
	MSG_ID_NAV_FILTER_BIAS     mavlink.MessageID = 220
	MSG_ID_RADIO_CALIBRATION   mavlink.MessageID = 221
	MSG_ID_UALBERTA_SYS_STATUS mavlink.MessageID = 222
)

// init Ualberta dialect
func init() {
	mavlink.Register(MSG_ID_NAV_FILTER_BIAS, "MSG_ID_NAV_FILTER_BIAS", 34, func(p *mavlink.Packet) mavlink.Message {
		msg := new(NavFilterBias)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_RADIO_CALIBRATION, "MSG_ID_RADIO_CALIBRATION", 71, func(p *mavlink.Packet) mavlink.Message {
		msg := new(RadioCalibration)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_UALBERTA_SYS_STATUS, "MSG_ID_UALBERTA_SYS_STATUS", 15, func(p *mavlink.Packet) mavlink.Message {
		msg := new(UalbertaSysStatus)
		msg.Unpack(p)
		return msg
	})
}

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

// MavPreflightStorageAction (generated enum)
// Action required when performing CMD_PREFLIGHT_STORAGE
const (
	MAV_PFS_CMD_READ_ALL       = 0 // Read all parameters from storage
	MAV_PFS_CMD_WRITE_ALL      = 1 // Write all parameters to storage
	MAV_PFS_CMD_CLEAR_ALL      = 2 // Clear all  parameters in storage
	MAV_PFS_CMD_READ_SPECIFIC  = 3 // Read specific parameters from storage
	MAV_PFS_CMD_WRITE_SPECIFIC = 4 // Write specific parameters to storage
	MAV_PFS_CMD_CLEAR_SPECIFIC = 5 // Clear specific parameters in storage
	MAV_PFS_CMD_DO_NOTHING     = 6 // do nothing
)

// MavCmd (generated enum)
//
const (
	MAV_CMD_PREFLIGHT_STORAGE_ADVANCED = 0 // Request storage of different parameter values and logs. This command will be only accepted if in pre-flight mode.
)

// MatrixpilotFlexifunctionSet struct (generated typeinfo)
// Depreciated but used as a compiler flag.  Do not remove
type MatrixpilotFlexifunctionSet struct {
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
}

// Dialect (generated function)
func (m *MatrixpilotFlexifunctionSet) Dialect() *Dialect {
	return DialectMatrixpilot
}

// MsgID (generated function)
func (m *MatrixpilotFlexifunctionSet) MsgID() MessageID {
	return MSG_ID_FLEXIFUNCTION_SET
}

// MsgName (generated function)
func (m *MatrixpilotFlexifunctionSet) MsgName() string {
	return "FlexifunctionSet"
}

// Pack (generated function)
func (m *MatrixpilotFlexifunctionSet) Pack(p *Packet) error {
	payload := make([]byte, 2)
	payload[0] = byte(m.TargetSystem)
	payload[1] = byte(m.TargetComponent)
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
func (m *MatrixpilotFlexifunctionSet) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 2 {
		if MavlinkVersion == 1 {
			return errPayloadTooSmall
		}
		payload = append(payload, zeroTail[:2-len(p.Payload)]...)
	}
	m.TargetSystem = uint8(payload[0])
	m.TargetComponent = uint8(payload[1])
	return nil
}

// MatrixpilotFlexifunctionReadReq struct (generated typeinfo)
// Reqest reading of flexifunction data
type MatrixpilotFlexifunctionReadReq struct {
	ReadReqType     int16 // Type of flexifunction data requested
	DataIndex       int16 // index into data where needed
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
}

// Dialect (generated function)
func (m *MatrixpilotFlexifunctionReadReq) Dialect() *Dialect {
	return DialectMatrixpilot
}

// MsgID (generated function)
func (m *MatrixpilotFlexifunctionReadReq) MsgID() MessageID {
	return MSG_ID_FLEXIFUNCTION_READ_REQ
}

// MsgName (generated function)
func (m *MatrixpilotFlexifunctionReadReq) MsgName() string {
	return "FlexifunctionReadReq"
}

// Pack (generated function)
func (m *MatrixpilotFlexifunctionReadReq) Pack(p *Packet) error {
	payload := make([]byte, 6)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.ReadReqType))
	binary.LittleEndian.PutUint16(payload[2:], uint16(m.DataIndex))
	payload[4] = byte(m.TargetSystem)
	payload[5] = byte(m.TargetComponent)
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
func (m *MatrixpilotFlexifunctionReadReq) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 6 {
		if MavlinkVersion == 1 {
			return errPayloadTooSmall
		}
		payload = append(payload, zeroTail[:6-len(p.Payload)]...)
	}
	m.ReadReqType = int16(binary.LittleEndian.Uint16(payload[0:]))
	m.DataIndex = int16(binary.LittleEndian.Uint16(payload[2:]))
	m.TargetSystem = uint8(payload[4])
	m.TargetComponent = uint8(payload[5])
	return nil
}

// MatrixpilotFlexifunctionBufferFunction struct (generated typeinfo)
// Flexifunction type and parameters for component at function index from buffer
type MatrixpilotFlexifunctionBufferFunction struct {
	FuncIndex       uint16   // Function index
	FuncCount       uint16   // Total count of functions
	DataAddress     uint16   // Address in the flexifunction data, Set to 0xFFFF to use address in target memory
	DataSize        uint16   // Size of the
	TargetSystem    uint8    // System ID
	TargetComponent uint8    // Component ID
	Data            [48]int8 // Settings data
}

// Dialect (generated function)
func (m *MatrixpilotFlexifunctionBufferFunction) Dialect() *Dialect {
	return DialectMatrixpilot
}

// MsgID (generated function)
func (m *MatrixpilotFlexifunctionBufferFunction) MsgID() MessageID {
	return MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION
}

// MsgName (generated function)
func (m *MatrixpilotFlexifunctionBufferFunction) MsgName() string {
	return "FlexifunctionBufferFunction"
}

// Pack (generated function)
func (m *MatrixpilotFlexifunctionBufferFunction) Pack(p *Packet) error {
	payload := make([]byte, 58)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.FuncIndex))
	binary.LittleEndian.PutUint16(payload[2:], uint16(m.FuncCount))
	binary.LittleEndian.PutUint16(payload[4:], uint16(m.DataAddress))
	binary.LittleEndian.PutUint16(payload[6:], uint16(m.DataSize))
	payload[8] = byte(m.TargetSystem)
	payload[9] = byte(m.TargetComponent)
	for i, v := range m.Data {
		payload[10+i*1] = byte(v)
	}
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
func (m *MatrixpilotFlexifunctionBufferFunction) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 58 {
		if MavlinkVersion == 1 {
			return errPayloadTooSmall
		}
		payload = append(payload, zeroTail[:58-len(p.Payload)]...)
	}
	m.FuncIndex = uint16(binary.LittleEndian.Uint16(payload[0:]))
	m.FuncCount = uint16(binary.LittleEndian.Uint16(payload[2:]))
	m.DataAddress = uint16(binary.LittleEndian.Uint16(payload[4:]))
	m.DataSize = uint16(binary.LittleEndian.Uint16(payload[6:]))
	m.TargetSystem = uint8(payload[8])
	m.TargetComponent = uint8(payload[9])
	for i := 0; i < len(m.Data); i++ {
		m.Data[i] = int8(payload[10+i*1])
	}
	return nil
}

// MatrixpilotFlexifunctionBufferFunctionAck struct (generated typeinfo)
// Flexifunction type and parameters for component at function index from buffer
type MatrixpilotFlexifunctionBufferFunctionAck struct {
	FuncIndex       uint16 // Function index
	Result          uint16 // result of acknowledge, 0=fail, 1=good
	TargetSystem    uint8  // System ID
	TargetComponent uint8  // Component ID
}

// Dialect (generated function)
func (m *MatrixpilotFlexifunctionBufferFunctionAck) Dialect() *Dialect {
	return DialectMatrixpilot
}

// MsgID (generated function)
func (m *MatrixpilotFlexifunctionBufferFunctionAck) MsgID() MessageID {
	return MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_ACK
}

// MsgName (generated function)
func (m *MatrixpilotFlexifunctionBufferFunctionAck) MsgName() string {
	return "FlexifunctionBufferFunctionAck"
}

// Pack (generated function)
func (m *MatrixpilotFlexifunctionBufferFunctionAck) Pack(p *Packet) error {
	payload := make([]byte, 6)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.FuncIndex))
	binary.LittleEndian.PutUint16(payload[2:], uint16(m.Result))
	payload[4] = byte(m.TargetSystem)
	payload[5] = byte(m.TargetComponent)
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
func (m *MatrixpilotFlexifunctionBufferFunctionAck) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 6 {
		if MavlinkVersion == 1 {
			return errPayloadTooSmall
		}
		payload = append(payload, zeroTail[:6-len(p.Payload)]...)
	}
	m.FuncIndex = uint16(binary.LittleEndian.Uint16(payload[0:]))
	m.Result = uint16(binary.LittleEndian.Uint16(payload[2:]))
	m.TargetSystem = uint8(payload[4])
	m.TargetComponent = uint8(payload[5])
	return nil
}

// MatrixpilotFlexifunctionDirectory struct (generated typeinfo)
// Acknowldge sucess or failure of a flexifunction command
type MatrixpilotFlexifunctionDirectory struct {
	TargetSystem    uint8    // System ID
	TargetComponent uint8    // Component ID
	DirectoryType   uint8    // 0=inputs, 1=outputs
	StartIndex      uint8    // index of first directory entry to write
	Count           uint8    // count of directory entries to write
	DirectoryData   [48]int8 // Settings data
}

// Dialect (generated function)
func (m *MatrixpilotFlexifunctionDirectory) Dialect() *Dialect {
	return DialectMatrixpilot
}

// MsgID (generated function)
func (m *MatrixpilotFlexifunctionDirectory) MsgID() MessageID {
	return MSG_ID_FLEXIFUNCTION_DIRECTORY
}

// MsgName (generated function)
func (m *MatrixpilotFlexifunctionDirectory) MsgName() string {
	return "FlexifunctionDirectory"
}

// Pack (generated function)
func (m *MatrixpilotFlexifunctionDirectory) Pack(p *Packet) error {
	payload := make([]byte, 53)
	payload[0] = byte(m.TargetSystem)
	payload[1] = byte(m.TargetComponent)
	payload[2] = byte(m.DirectoryType)
	payload[3] = byte(m.StartIndex)
	payload[4] = byte(m.Count)
	for i, v := range m.DirectoryData {
		payload[5+i*1] = byte(v)
	}
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
func (m *MatrixpilotFlexifunctionDirectory) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 53 {
		if MavlinkVersion == 1 {
			return errPayloadTooSmall
		}
		payload = append(payload, zeroTail[:53-len(p.Payload)]...)
	}
	m.TargetSystem = uint8(payload[0])
	m.TargetComponent = uint8(payload[1])
	m.DirectoryType = uint8(payload[2])
	m.StartIndex = uint8(payload[3])
	m.Count = uint8(payload[4])
	for i := 0; i < len(m.DirectoryData); i++ {
		m.DirectoryData[i] = int8(payload[5+i*1])
	}
	return nil
}

// MatrixpilotFlexifunctionDirectoryAck struct (generated typeinfo)
// Acknowldge sucess or failure of a flexifunction command
type MatrixpilotFlexifunctionDirectoryAck struct {
	Result          uint16 // result of acknowledge, 0=fail, 1=good
	TargetSystem    uint8  // System ID
	TargetComponent uint8  // Component ID
	DirectoryType   uint8  // 0=inputs, 1=outputs
	StartIndex      uint8  // index of first directory entry to write
	Count           uint8  // count of directory entries to write
}

// Dialect (generated function)
func (m *MatrixpilotFlexifunctionDirectoryAck) Dialect() *Dialect {
	return DialectMatrixpilot
}

// MsgID (generated function)
func (m *MatrixpilotFlexifunctionDirectoryAck) MsgID() MessageID {
	return MSG_ID_FLEXIFUNCTION_DIRECTORY_ACK
}

// MsgName (generated function)
func (m *MatrixpilotFlexifunctionDirectoryAck) MsgName() string {
	return "FlexifunctionDirectoryAck"
}

// Pack (generated function)
func (m *MatrixpilotFlexifunctionDirectoryAck) Pack(p *Packet) error {
	payload := make([]byte, 7)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.Result))
	payload[2] = byte(m.TargetSystem)
	payload[3] = byte(m.TargetComponent)
	payload[4] = byte(m.DirectoryType)
	payload[5] = byte(m.StartIndex)
	payload[6] = byte(m.Count)
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
func (m *MatrixpilotFlexifunctionDirectoryAck) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 7 {
		if MavlinkVersion == 1 {
			return errPayloadTooSmall
		}
		payload = append(payload, zeroTail[:7-len(p.Payload)]...)
	}
	m.Result = uint16(binary.LittleEndian.Uint16(payload[0:]))
	m.TargetSystem = uint8(payload[2])
	m.TargetComponent = uint8(payload[3])
	m.DirectoryType = uint8(payload[4])
	m.StartIndex = uint8(payload[5])
	m.Count = uint8(payload[6])
	return nil
}

// MatrixpilotFlexifunctionCommand struct (generated typeinfo)
// Acknowldge sucess or failure of a flexifunction command
type MatrixpilotFlexifunctionCommand struct {
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
	CommandType     uint8 // Flexifunction command type
}

// Dialect (generated function)
func (m *MatrixpilotFlexifunctionCommand) Dialect() *Dialect {
	return DialectMatrixpilot
}

// MsgID (generated function)
func (m *MatrixpilotFlexifunctionCommand) MsgID() MessageID {
	return MSG_ID_FLEXIFUNCTION_COMMAND
}

// MsgName (generated function)
func (m *MatrixpilotFlexifunctionCommand) MsgName() string {
	return "FlexifunctionCommand"
}

// Pack (generated function)
func (m *MatrixpilotFlexifunctionCommand) Pack(p *Packet) error {
	payload := make([]byte, 3)
	payload[0] = byte(m.TargetSystem)
	payload[1] = byte(m.TargetComponent)
	payload[2] = byte(m.CommandType)
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
func (m *MatrixpilotFlexifunctionCommand) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 3 {
		if MavlinkVersion == 1 {
			return errPayloadTooSmall
		}
		payload = append(payload, zeroTail[:3-len(p.Payload)]...)
	}
	m.TargetSystem = uint8(payload[0])
	m.TargetComponent = uint8(payload[1])
	m.CommandType = uint8(payload[2])
	return nil
}

// MatrixpilotFlexifunctionCommandAck struct (generated typeinfo)
// Acknowldge sucess or failure of a flexifunction command
type MatrixpilotFlexifunctionCommandAck struct {
	CommandType uint16 // Command acknowledged
	Result      uint16 // result of acknowledge
}

// Dialect (generated function)
func (m *MatrixpilotFlexifunctionCommandAck) Dialect() *Dialect {
	return DialectMatrixpilot
}

// MsgID (generated function)
func (m *MatrixpilotFlexifunctionCommandAck) MsgID() MessageID {
	return MSG_ID_FLEXIFUNCTION_COMMAND_ACK
}

// MsgName (generated function)
func (m *MatrixpilotFlexifunctionCommandAck) MsgName() string {
	return "FlexifunctionCommandAck"
}

// Pack (generated function)
func (m *MatrixpilotFlexifunctionCommandAck) Pack(p *Packet) error {
	payload := make([]byte, 4)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.CommandType))
	binary.LittleEndian.PutUint16(payload[2:], uint16(m.Result))
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
func (m *MatrixpilotFlexifunctionCommandAck) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 4 {
		if MavlinkVersion == 1 {
			return errPayloadTooSmall
		}
		payload = append(payload, zeroTail[:4-len(p.Payload)]...)
	}
	m.CommandType = uint16(binary.LittleEndian.Uint16(payload[0:]))
	m.Result = uint16(binary.LittleEndian.Uint16(payload[2:]))
	return nil
}

// MatrixpilotSerialUdbExtraF2A struct (generated typeinfo)
// Backwards compatible MAVLink version of SERIAL_UDB_EXTRA - F2: Format Part A
type MatrixpilotSerialUdbExtraF2A struct {
	SueTime           uint32 // Serial UDB Extra Time
	SueLatitude       int32  // Serial UDB Extra Latitude
	SueLongitude      int32  // Serial UDB Extra Longitude
	SueAltitude       int32  // Serial UDB Extra Altitude
	SueWaypointIndex  uint16 // Serial UDB Extra Waypoint Index
	SueRmat0          int16  // Serial UDB Extra Rmat 0
	SueRmat1          int16  // Serial UDB Extra Rmat 1
	SueRmat2          int16  // Serial UDB Extra Rmat 2
	SueRmat3          int16  // Serial UDB Extra Rmat 3
	SueRmat4          int16  // Serial UDB Extra Rmat 4
	SueRmat5          int16  // Serial UDB Extra Rmat 5
	SueRmat6          int16  // Serial UDB Extra Rmat 6
	SueRmat7          int16  // Serial UDB Extra Rmat 7
	SueRmat8          int16  // Serial UDB Extra Rmat 8
	SueCog            uint16 // Serial UDB Extra GPS Course Over Ground
	SueSog            int16  // Serial UDB Extra Speed Over Ground
	SueCPULoad        uint16 // Serial UDB Extra CPU Load
	SueAirSpeed3dimu  uint16 // Serial UDB Extra 3D IMU Air Speed
	SueEstimatedWind0 int16  // Serial UDB Extra Estimated Wind 0
	SueEstimatedWind1 int16  // Serial UDB Extra Estimated Wind 1
	SueEstimatedWind2 int16  // Serial UDB Extra Estimated Wind 2
	SueMagfieldearth0 int16  // Serial UDB Extra Magnetic Field Earth 0
	SueMagfieldearth1 int16  // Serial UDB Extra Magnetic Field Earth 1
	SueMagfieldearth2 int16  // Serial UDB Extra Magnetic Field Earth 2
	SueSvs            int16  // Serial UDB Extra Number of Sattelites in View
	SueHdop           int16  // Serial UDB Extra GPS Horizontal Dilution of Precision
	SueStatus         uint8  // Serial UDB Extra Status
}

// Dialect (generated function)
func (m *MatrixpilotSerialUdbExtraF2A) Dialect() *Dialect {
	return DialectMatrixpilot
}

// MsgID (generated function)
func (m *MatrixpilotSerialUdbExtraF2A) MsgID() MessageID {
	return MSG_ID_SERIAL_UDB_EXTRA_F2_A
}

// MsgName (generated function)
func (m *MatrixpilotSerialUdbExtraF2A) MsgName() string {
	return "SerialUdbExtraF2A"
}

// Pack (generated function)
func (m *MatrixpilotSerialUdbExtraF2A) Pack(p *Packet) error {
	payload := make([]byte, 61)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.SueTime))
	binary.LittleEndian.PutUint32(payload[4:], uint32(m.SueLatitude))
	binary.LittleEndian.PutUint32(payload[8:], uint32(m.SueLongitude))
	binary.LittleEndian.PutUint32(payload[12:], uint32(m.SueAltitude))
	binary.LittleEndian.PutUint16(payload[16:], uint16(m.SueWaypointIndex))
	binary.LittleEndian.PutUint16(payload[18:], uint16(m.SueRmat0))
	binary.LittleEndian.PutUint16(payload[20:], uint16(m.SueRmat1))
	binary.LittleEndian.PutUint16(payload[22:], uint16(m.SueRmat2))
	binary.LittleEndian.PutUint16(payload[24:], uint16(m.SueRmat3))
	binary.LittleEndian.PutUint16(payload[26:], uint16(m.SueRmat4))
	binary.LittleEndian.PutUint16(payload[28:], uint16(m.SueRmat5))
	binary.LittleEndian.PutUint16(payload[30:], uint16(m.SueRmat6))
	binary.LittleEndian.PutUint16(payload[32:], uint16(m.SueRmat7))
	binary.LittleEndian.PutUint16(payload[34:], uint16(m.SueRmat8))
	binary.LittleEndian.PutUint16(payload[36:], uint16(m.SueCog))
	binary.LittleEndian.PutUint16(payload[38:], uint16(m.SueSog))
	binary.LittleEndian.PutUint16(payload[40:], uint16(m.SueCPULoad))
	binary.LittleEndian.PutUint16(payload[42:], uint16(m.SueAirSpeed3dimu))
	binary.LittleEndian.PutUint16(payload[44:], uint16(m.SueEstimatedWind0))
	binary.LittleEndian.PutUint16(payload[46:], uint16(m.SueEstimatedWind1))
	binary.LittleEndian.PutUint16(payload[48:], uint16(m.SueEstimatedWind2))
	binary.LittleEndian.PutUint16(payload[50:], uint16(m.SueMagfieldearth0))
	binary.LittleEndian.PutUint16(payload[52:], uint16(m.SueMagfieldearth1))
	binary.LittleEndian.PutUint16(payload[54:], uint16(m.SueMagfieldearth2))
	binary.LittleEndian.PutUint16(payload[56:], uint16(m.SueSvs))
	binary.LittleEndian.PutUint16(payload[58:], uint16(m.SueHdop))
	payload[60] = byte(m.SueStatus)
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
func (m *MatrixpilotSerialUdbExtraF2A) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 61 {
		if MavlinkVersion == 1 {
			return errPayloadTooSmall
		}
		payload = append(payload, zeroTail[:61-len(p.Payload)]...)
	}
	m.SueTime = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.SueLatitude = int32(binary.LittleEndian.Uint32(payload[4:]))
	m.SueLongitude = int32(binary.LittleEndian.Uint32(payload[8:]))
	m.SueAltitude = int32(binary.LittleEndian.Uint32(payload[12:]))
	m.SueWaypointIndex = uint16(binary.LittleEndian.Uint16(payload[16:]))
	m.SueRmat0 = int16(binary.LittleEndian.Uint16(payload[18:]))
	m.SueRmat1 = int16(binary.LittleEndian.Uint16(payload[20:]))
	m.SueRmat2 = int16(binary.LittleEndian.Uint16(payload[22:]))
	m.SueRmat3 = int16(binary.LittleEndian.Uint16(payload[24:]))
	m.SueRmat4 = int16(binary.LittleEndian.Uint16(payload[26:]))
	m.SueRmat5 = int16(binary.LittleEndian.Uint16(payload[28:]))
	m.SueRmat6 = int16(binary.LittleEndian.Uint16(payload[30:]))
	m.SueRmat7 = int16(binary.LittleEndian.Uint16(payload[32:]))
	m.SueRmat8 = int16(binary.LittleEndian.Uint16(payload[34:]))
	m.SueCog = uint16(binary.LittleEndian.Uint16(payload[36:]))
	m.SueSog = int16(binary.LittleEndian.Uint16(payload[38:]))
	m.SueCPULoad = uint16(binary.LittleEndian.Uint16(payload[40:]))
	m.SueAirSpeed3dimu = uint16(binary.LittleEndian.Uint16(payload[42:]))
	m.SueEstimatedWind0 = int16(binary.LittleEndian.Uint16(payload[44:]))
	m.SueEstimatedWind1 = int16(binary.LittleEndian.Uint16(payload[46:]))
	m.SueEstimatedWind2 = int16(binary.LittleEndian.Uint16(payload[48:]))
	m.SueMagfieldearth0 = int16(binary.LittleEndian.Uint16(payload[50:]))
	m.SueMagfieldearth1 = int16(binary.LittleEndian.Uint16(payload[52:]))
	m.SueMagfieldearth2 = int16(binary.LittleEndian.Uint16(payload[54:]))
	m.SueSvs = int16(binary.LittleEndian.Uint16(payload[56:]))
	m.SueHdop = int16(binary.LittleEndian.Uint16(payload[58:]))
	m.SueStatus = uint8(payload[60])
	return nil
}

// MatrixpilotSerialUdbExtraF2B struct (generated typeinfo)
// Backwards compatible version of SERIAL_UDB_EXTRA - F2: Part B
type MatrixpilotSerialUdbExtraF2B struct {
	SueTime                uint32 // Serial UDB Extra Time
	SueFlags               uint32 // Serial UDB Extra Status Flags
	SueBaromPress          int32  // SUE barometer pressure
	SueBaromAlt            int32  // SUE barometer altitude
	SuePwmInput1           int16  // Serial UDB Extra PWM Input Channel 1
	SuePwmInput2           int16  // Serial UDB Extra PWM Input Channel 2
	SuePwmInput3           int16  // Serial UDB Extra PWM Input Channel 3
	SuePwmInput4           int16  // Serial UDB Extra PWM Input Channel 4
	SuePwmInput5           int16  // Serial UDB Extra PWM Input Channel 5
	SuePwmInput6           int16  // Serial UDB Extra PWM Input Channel 6
	SuePwmInput7           int16  // Serial UDB Extra PWM Input Channel 7
	SuePwmInput8           int16  // Serial UDB Extra PWM Input Channel 8
	SuePwmInput9           int16  // Serial UDB Extra PWM Input Channel 9
	SuePwmInput10          int16  // Serial UDB Extra PWM Input Channel 10
	SuePwmInput11          int16  // Serial UDB Extra PWM Input Channel 11
	SuePwmInput12          int16  // Serial UDB Extra PWM Input Channel 12
	SuePwmOutput1          int16  // Serial UDB Extra PWM Output Channel 1
	SuePwmOutput2          int16  // Serial UDB Extra PWM Output Channel 2
	SuePwmOutput3          int16  // Serial UDB Extra PWM Output Channel 3
	SuePwmOutput4          int16  // Serial UDB Extra PWM Output Channel 4
	SuePwmOutput5          int16  // Serial UDB Extra PWM Output Channel 5
	SuePwmOutput6          int16  // Serial UDB Extra PWM Output Channel 6
	SuePwmOutput7          int16  // Serial UDB Extra PWM Output Channel 7
	SuePwmOutput8          int16  // Serial UDB Extra PWM Output Channel 8
	SuePwmOutput9          int16  // Serial UDB Extra PWM Output Channel 9
	SuePwmOutput10         int16  // Serial UDB Extra PWM Output Channel 10
	SuePwmOutput11         int16  // Serial UDB Extra PWM Output Channel 11
	SuePwmOutput12         int16  // Serial UDB Extra PWM Output Channel 12
	SueImuLocationX        int16  // Serial UDB Extra IMU Location X
	SueImuLocationY        int16  // Serial UDB Extra IMU Location Y
	SueImuLocationZ        int16  // Serial UDB Extra IMU Location Z
	SueLocationErrorEarthX int16  // Serial UDB Location Error Earth X
	SueLocationErrorEarthY int16  // Serial UDB Location Error Earth Y
	SueLocationErrorEarthZ int16  // Serial UDB Location Error Earth Z
	SueOscFails            int16  // Serial UDB Extra Oscillator Failure Count
	SueImuVelocityX        int16  // Serial UDB Extra IMU Velocity X
	SueImuVelocityY        int16  // Serial UDB Extra IMU Velocity Y
	SueImuVelocityZ        int16  // Serial UDB Extra IMU Velocity Z
	SueWaypointGoalX       int16  // Serial UDB Extra Current Waypoint Goal X
	SueWaypointGoalY       int16  // Serial UDB Extra Current Waypoint Goal Y
	SueWaypointGoalZ       int16  // Serial UDB Extra Current Waypoint Goal Z
	SueAeroX               int16  // Aeroforce in UDB X Axis
	SueAeroY               int16  // Aeroforce in UDB Y Axis
	SueAeroZ               int16  // Aeroforce in UDB Z axis
	SueBaromTemp           int16  // SUE barometer temperature
	SueBatVolt             int16  // SUE battery voltage
	SueBatAmp              int16  // SUE battery current
	SueBatAmpHours         int16  // SUE battery milli amp hours used
	SueDesiredHeight       int16  // Sue autopilot desired height
	SueMemoryStackFree     int16  // Serial UDB Extra Stack Memory Free
}

// Dialect (generated function)
func (m *MatrixpilotSerialUdbExtraF2B) Dialect() *Dialect {
	return DialectMatrixpilot
}

// MsgID (generated function)
func (m *MatrixpilotSerialUdbExtraF2B) MsgID() MessageID {
	return MSG_ID_SERIAL_UDB_EXTRA_F2_B
}

// MsgName (generated function)
func (m *MatrixpilotSerialUdbExtraF2B) MsgName() string {
	return "SerialUdbExtraF2B"
}

// Pack (generated function)
func (m *MatrixpilotSerialUdbExtraF2B) Pack(p *Packet) error {
	payload := make([]byte, 108)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.SueTime))
	binary.LittleEndian.PutUint32(payload[4:], uint32(m.SueFlags))
	binary.LittleEndian.PutUint32(payload[8:], uint32(m.SueBaromPress))
	binary.LittleEndian.PutUint32(payload[12:], uint32(m.SueBaromAlt))
	binary.LittleEndian.PutUint16(payload[16:], uint16(m.SuePwmInput1))
	binary.LittleEndian.PutUint16(payload[18:], uint16(m.SuePwmInput2))
	binary.LittleEndian.PutUint16(payload[20:], uint16(m.SuePwmInput3))
	binary.LittleEndian.PutUint16(payload[22:], uint16(m.SuePwmInput4))
	binary.LittleEndian.PutUint16(payload[24:], uint16(m.SuePwmInput5))
	binary.LittleEndian.PutUint16(payload[26:], uint16(m.SuePwmInput6))
	binary.LittleEndian.PutUint16(payload[28:], uint16(m.SuePwmInput7))
	binary.LittleEndian.PutUint16(payload[30:], uint16(m.SuePwmInput8))
	binary.LittleEndian.PutUint16(payload[32:], uint16(m.SuePwmInput9))
	binary.LittleEndian.PutUint16(payload[34:], uint16(m.SuePwmInput10))
	binary.LittleEndian.PutUint16(payload[36:], uint16(m.SuePwmInput11))
	binary.LittleEndian.PutUint16(payload[38:], uint16(m.SuePwmInput12))
	binary.LittleEndian.PutUint16(payload[40:], uint16(m.SuePwmOutput1))
	binary.LittleEndian.PutUint16(payload[42:], uint16(m.SuePwmOutput2))
	binary.LittleEndian.PutUint16(payload[44:], uint16(m.SuePwmOutput3))
	binary.LittleEndian.PutUint16(payload[46:], uint16(m.SuePwmOutput4))
	binary.LittleEndian.PutUint16(payload[48:], uint16(m.SuePwmOutput5))
	binary.LittleEndian.PutUint16(payload[50:], uint16(m.SuePwmOutput6))
	binary.LittleEndian.PutUint16(payload[52:], uint16(m.SuePwmOutput7))
	binary.LittleEndian.PutUint16(payload[54:], uint16(m.SuePwmOutput8))
	binary.LittleEndian.PutUint16(payload[56:], uint16(m.SuePwmOutput9))
	binary.LittleEndian.PutUint16(payload[58:], uint16(m.SuePwmOutput10))
	binary.LittleEndian.PutUint16(payload[60:], uint16(m.SuePwmOutput11))
	binary.LittleEndian.PutUint16(payload[62:], uint16(m.SuePwmOutput12))
	binary.LittleEndian.PutUint16(payload[64:], uint16(m.SueImuLocationX))
	binary.LittleEndian.PutUint16(payload[66:], uint16(m.SueImuLocationY))
	binary.LittleEndian.PutUint16(payload[68:], uint16(m.SueImuLocationZ))
	binary.LittleEndian.PutUint16(payload[70:], uint16(m.SueLocationErrorEarthX))
	binary.LittleEndian.PutUint16(payload[72:], uint16(m.SueLocationErrorEarthY))
	binary.LittleEndian.PutUint16(payload[74:], uint16(m.SueLocationErrorEarthZ))
	binary.LittleEndian.PutUint16(payload[76:], uint16(m.SueOscFails))
	binary.LittleEndian.PutUint16(payload[78:], uint16(m.SueImuVelocityX))
	binary.LittleEndian.PutUint16(payload[80:], uint16(m.SueImuVelocityY))
	binary.LittleEndian.PutUint16(payload[82:], uint16(m.SueImuVelocityZ))
	binary.LittleEndian.PutUint16(payload[84:], uint16(m.SueWaypointGoalX))
	binary.LittleEndian.PutUint16(payload[86:], uint16(m.SueWaypointGoalY))
	binary.LittleEndian.PutUint16(payload[88:], uint16(m.SueWaypointGoalZ))
	binary.LittleEndian.PutUint16(payload[90:], uint16(m.SueAeroX))
	binary.LittleEndian.PutUint16(payload[92:], uint16(m.SueAeroY))
	binary.LittleEndian.PutUint16(payload[94:], uint16(m.SueAeroZ))
	binary.LittleEndian.PutUint16(payload[96:], uint16(m.SueBaromTemp))
	binary.LittleEndian.PutUint16(payload[98:], uint16(m.SueBatVolt))
	binary.LittleEndian.PutUint16(payload[100:], uint16(m.SueBatAmp))
	binary.LittleEndian.PutUint16(payload[102:], uint16(m.SueBatAmpHours))
	binary.LittleEndian.PutUint16(payload[104:], uint16(m.SueDesiredHeight))
	binary.LittleEndian.PutUint16(payload[106:], uint16(m.SueMemoryStackFree))
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
func (m *MatrixpilotSerialUdbExtraF2B) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 108 {
		if MavlinkVersion == 1 {
			return errPayloadTooSmall
		}
		payload = append(payload, zeroTail[:108-len(p.Payload)]...)
	}
	m.SueTime = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.SueFlags = uint32(binary.LittleEndian.Uint32(payload[4:]))
	m.SueBaromPress = int32(binary.LittleEndian.Uint32(payload[8:]))
	m.SueBaromAlt = int32(binary.LittleEndian.Uint32(payload[12:]))
	m.SuePwmInput1 = int16(binary.LittleEndian.Uint16(payload[16:]))
	m.SuePwmInput2 = int16(binary.LittleEndian.Uint16(payload[18:]))
	m.SuePwmInput3 = int16(binary.LittleEndian.Uint16(payload[20:]))
	m.SuePwmInput4 = int16(binary.LittleEndian.Uint16(payload[22:]))
	m.SuePwmInput5 = int16(binary.LittleEndian.Uint16(payload[24:]))
	m.SuePwmInput6 = int16(binary.LittleEndian.Uint16(payload[26:]))
	m.SuePwmInput7 = int16(binary.LittleEndian.Uint16(payload[28:]))
	m.SuePwmInput8 = int16(binary.LittleEndian.Uint16(payload[30:]))
	m.SuePwmInput9 = int16(binary.LittleEndian.Uint16(payload[32:]))
	m.SuePwmInput10 = int16(binary.LittleEndian.Uint16(payload[34:]))
	m.SuePwmInput11 = int16(binary.LittleEndian.Uint16(payload[36:]))
	m.SuePwmInput12 = int16(binary.LittleEndian.Uint16(payload[38:]))
	m.SuePwmOutput1 = int16(binary.LittleEndian.Uint16(payload[40:]))
	m.SuePwmOutput2 = int16(binary.LittleEndian.Uint16(payload[42:]))
	m.SuePwmOutput3 = int16(binary.LittleEndian.Uint16(payload[44:]))
	m.SuePwmOutput4 = int16(binary.LittleEndian.Uint16(payload[46:]))
	m.SuePwmOutput5 = int16(binary.LittleEndian.Uint16(payload[48:]))
	m.SuePwmOutput6 = int16(binary.LittleEndian.Uint16(payload[50:]))
	m.SuePwmOutput7 = int16(binary.LittleEndian.Uint16(payload[52:]))
	m.SuePwmOutput8 = int16(binary.LittleEndian.Uint16(payload[54:]))
	m.SuePwmOutput9 = int16(binary.LittleEndian.Uint16(payload[56:]))
	m.SuePwmOutput10 = int16(binary.LittleEndian.Uint16(payload[58:]))
	m.SuePwmOutput11 = int16(binary.LittleEndian.Uint16(payload[60:]))
	m.SuePwmOutput12 = int16(binary.LittleEndian.Uint16(payload[62:]))
	m.SueImuLocationX = int16(binary.LittleEndian.Uint16(payload[64:]))
	m.SueImuLocationY = int16(binary.LittleEndian.Uint16(payload[66:]))
	m.SueImuLocationZ = int16(binary.LittleEndian.Uint16(payload[68:]))
	m.SueLocationErrorEarthX = int16(binary.LittleEndian.Uint16(payload[70:]))
	m.SueLocationErrorEarthY = int16(binary.LittleEndian.Uint16(payload[72:]))
	m.SueLocationErrorEarthZ = int16(binary.LittleEndian.Uint16(payload[74:]))
	m.SueOscFails = int16(binary.LittleEndian.Uint16(payload[76:]))
	m.SueImuVelocityX = int16(binary.LittleEndian.Uint16(payload[78:]))
	m.SueImuVelocityY = int16(binary.LittleEndian.Uint16(payload[80:]))
	m.SueImuVelocityZ = int16(binary.LittleEndian.Uint16(payload[82:]))
	m.SueWaypointGoalX = int16(binary.LittleEndian.Uint16(payload[84:]))
	m.SueWaypointGoalY = int16(binary.LittleEndian.Uint16(payload[86:]))
	m.SueWaypointGoalZ = int16(binary.LittleEndian.Uint16(payload[88:]))
	m.SueAeroX = int16(binary.LittleEndian.Uint16(payload[90:]))
	m.SueAeroY = int16(binary.LittleEndian.Uint16(payload[92:]))
	m.SueAeroZ = int16(binary.LittleEndian.Uint16(payload[94:]))
	m.SueBaromTemp = int16(binary.LittleEndian.Uint16(payload[96:]))
	m.SueBatVolt = int16(binary.LittleEndian.Uint16(payload[98:]))
	m.SueBatAmp = int16(binary.LittleEndian.Uint16(payload[100:]))
	m.SueBatAmpHours = int16(binary.LittleEndian.Uint16(payload[102:]))
	m.SueDesiredHeight = int16(binary.LittleEndian.Uint16(payload[104:]))
	m.SueMemoryStackFree = int16(binary.LittleEndian.Uint16(payload[106:]))
	return nil
}

// MatrixpilotSerialUdbExtraF4 struct (generated typeinfo)
// Backwards compatible version of SERIAL_UDB_EXTRA F4: format
type MatrixpilotSerialUdbExtraF4 struct {
	SueRollStabilizationAilerons uint8 // Serial UDB Extra Roll Stabilization with Ailerons Enabled
	SueRollStabilizationRudder   uint8 // Serial UDB Extra Roll Stabilization with Rudder Enabled
	SuePitchStabilization        uint8 // Serial UDB Extra Pitch Stabilization Enabled
	SueYawStabilizationRudder    uint8 // Serial UDB Extra Yaw Stabilization using Rudder Enabled
	SueYawStabilizationAileron   uint8 // Serial UDB Extra Yaw Stabilization using Ailerons Enabled
	SueAileronNavigation         uint8 // Serial UDB Extra Navigation with Ailerons Enabled
	SueRudderNavigation          uint8 // Serial UDB Extra Navigation with Rudder Enabled
	SueAltitudeholdStabilized    uint8 // Serial UDB Extra Type of Alitude Hold when in Stabilized Mode
	SueAltitudeholdWaypoint      uint8 // Serial UDB Extra Type of Alitude Hold when in Waypoint Mode
	SueRacingMode                uint8 // Serial UDB Extra Firmware racing mode enabled
}

// Dialect (generated function)
func (m *MatrixpilotSerialUdbExtraF4) Dialect() *Dialect {
	return DialectMatrixpilot
}

// MsgID (generated function)
func (m *MatrixpilotSerialUdbExtraF4) MsgID() MessageID {
	return MSG_ID_SERIAL_UDB_EXTRA_F4
}

// MsgName (generated function)
func (m *MatrixpilotSerialUdbExtraF4) MsgName() string {
	return "SerialUdbExtraF4"
}

// Pack (generated function)
func (m *MatrixpilotSerialUdbExtraF4) Pack(p *Packet) error {
	payload := make([]byte, 10)
	payload[0] = byte(m.SueRollStabilizationAilerons)
	payload[1] = byte(m.SueRollStabilizationRudder)
	payload[2] = byte(m.SuePitchStabilization)
	payload[3] = byte(m.SueYawStabilizationRudder)
	payload[4] = byte(m.SueYawStabilizationAileron)
	payload[5] = byte(m.SueAileronNavigation)
	payload[6] = byte(m.SueRudderNavigation)
	payload[7] = byte(m.SueAltitudeholdStabilized)
	payload[8] = byte(m.SueAltitudeholdWaypoint)
	payload[9] = byte(m.SueRacingMode)
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
func (m *MatrixpilotSerialUdbExtraF4) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 10 {
		if MavlinkVersion == 1 {
			return errPayloadTooSmall
		}
		payload = append(payload, zeroTail[:10-len(p.Payload)]...)
	}
	m.SueRollStabilizationAilerons = uint8(payload[0])
	m.SueRollStabilizationRudder = uint8(payload[1])
	m.SuePitchStabilization = uint8(payload[2])
	m.SueYawStabilizationRudder = uint8(payload[3])
	m.SueYawStabilizationAileron = uint8(payload[4])
	m.SueAileronNavigation = uint8(payload[5])
	m.SueRudderNavigation = uint8(payload[6])
	m.SueAltitudeholdStabilized = uint8(payload[7])
	m.SueAltitudeholdWaypoint = uint8(payload[8])
	m.SueRacingMode = uint8(payload[9])
	return nil
}

// MatrixpilotSerialUdbExtraF5 struct (generated typeinfo)
// Backwards compatible version of SERIAL_UDB_EXTRA F5: format
type MatrixpilotSerialUdbExtraF5 struct {
	SueYawkpAileron float32 // Serial UDB YAWKP_AILERON Gain for Proporional control of navigation
	SueYawkdAileron float32 // Serial UDB YAWKD_AILERON Gain for Rate control of navigation
	SueRollkp       float32 // Serial UDB Extra ROLLKP Gain for Proportional control of roll stabilization
	SueRollkd       float32 // Serial UDB Extra ROLLKD Gain for Rate control of roll stabilization
}

// Dialect (generated function)
func (m *MatrixpilotSerialUdbExtraF5) Dialect() *Dialect {
	return DialectMatrixpilot
}

// MsgID (generated function)
func (m *MatrixpilotSerialUdbExtraF5) MsgID() MessageID {
	return MSG_ID_SERIAL_UDB_EXTRA_F5
}

// MsgName (generated function)
func (m *MatrixpilotSerialUdbExtraF5) MsgName() string {
	return "SerialUdbExtraF5"
}

// Pack (generated function)
func (m *MatrixpilotSerialUdbExtraF5) Pack(p *Packet) error {
	payload := make([]byte, 16)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(m.SueYawkpAileron))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(m.SueYawkdAileron))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.SueRollkp))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.SueRollkd))
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
func (m *MatrixpilotSerialUdbExtraF5) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 16 {
		if MavlinkVersion == 1 {
			return errPayloadTooSmall
		}
		payload = append(payload, zeroTail[:16-len(p.Payload)]...)
	}
	m.SueYawkpAileron = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.SueYawkdAileron = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	m.SueRollkp = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.SueRollkd = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	return nil
}

// MatrixpilotSerialUdbExtraF6 struct (generated typeinfo)
// Backwards compatible version of SERIAL_UDB_EXTRA F6: format
type MatrixpilotSerialUdbExtraF6 struct {
	SuePitchgain     float32 // Serial UDB Extra PITCHGAIN Proportional Control
	SuePitchkd       float32 // Serial UDB Extra Pitch Rate Control
	SueRudderElevMix float32 // Serial UDB Extra Rudder to Elevator Mix
	SueRollElevMix   float32 // Serial UDB Extra Roll to Elevator Mix
	SueElevatorBoost float32 // Gain For Boosting Manual Elevator control When Plane Stabilized
}

// Dialect (generated function)
func (m *MatrixpilotSerialUdbExtraF6) Dialect() *Dialect {
	return DialectMatrixpilot
}

// MsgID (generated function)
func (m *MatrixpilotSerialUdbExtraF6) MsgID() MessageID {
	return MSG_ID_SERIAL_UDB_EXTRA_F6
}

// MsgName (generated function)
func (m *MatrixpilotSerialUdbExtraF6) MsgName() string {
	return "SerialUdbExtraF6"
}

// Pack (generated function)
func (m *MatrixpilotSerialUdbExtraF6) Pack(p *Packet) error {
	payload := make([]byte, 20)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(m.SuePitchgain))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(m.SuePitchkd))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.SueRudderElevMix))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.SueRollElevMix))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(m.SueElevatorBoost))
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
func (m *MatrixpilotSerialUdbExtraF6) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 20 {
		if MavlinkVersion == 1 {
			return errPayloadTooSmall
		}
		payload = append(payload, zeroTail[:20-len(p.Payload)]...)
	}
	m.SuePitchgain = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.SuePitchkd = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	m.SueRudderElevMix = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.SueRollElevMix = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.SueElevatorBoost = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	return nil
}

// MatrixpilotSerialUdbExtraF7 struct (generated typeinfo)
// Backwards compatible version of SERIAL_UDB_EXTRA F7: format
type MatrixpilotSerialUdbExtraF7 struct {
	SueYawkpRudder  float32 // Serial UDB YAWKP_RUDDER Gain for Proporional control of navigation
	SueYawkdRudder  float32 // Serial UDB YAWKD_RUDDER Gain for Rate control of navigation
	SueRollkpRudder float32 // Serial UDB Extra ROLLKP_RUDDER Gain for Proportional control of roll stabilization
	SueRollkdRudder float32 // Serial UDB Extra ROLLKD_RUDDER Gain for Rate control of roll stabilization
	SueRudderBoost  float32 // SERIAL UDB EXTRA Rudder Boost Gain to Manual Control when stabilized
	SueRtlPitchDown float32 // Serial UDB Extra Return To Landing - Angle to Pitch Plane Down
}

// Dialect (generated function)
func (m *MatrixpilotSerialUdbExtraF7) Dialect() *Dialect {
	return DialectMatrixpilot
}

// MsgID (generated function)
func (m *MatrixpilotSerialUdbExtraF7) MsgID() MessageID {
	return MSG_ID_SERIAL_UDB_EXTRA_F7
}

// MsgName (generated function)
func (m *MatrixpilotSerialUdbExtraF7) MsgName() string {
	return "SerialUdbExtraF7"
}

// Pack (generated function)
func (m *MatrixpilotSerialUdbExtraF7) Pack(p *Packet) error {
	payload := make([]byte, 24)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(m.SueYawkpRudder))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(m.SueYawkdRudder))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.SueRollkpRudder))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.SueRollkdRudder))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(m.SueRudderBoost))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(m.SueRtlPitchDown))
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
func (m *MatrixpilotSerialUdbExtraF7) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 24 {
		if MavlinkVersion == 1 {
			return errPayloadTooSmall
		}
		payload = append(payload, zeroTail[:24-len(p.Payload)]...)
	}
	m.SueYawkpRudder = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.SueYawkdRudder = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	m.SueRollkpRudder = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.SueRollkdRudder = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.SueRudderBoost = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	m.SueRtlPitchDown = math.Float32frombits(binary.LittleEndian.Uint32(payload[20:]))
	return nil
}

// MatrixpilotSerialUdbExtraF8 struct (generated typeinfo)
// Backwards compatible version of SERIAL_UDB_EXTRA F8: format
type MatrixpilotSerialUdbExtraF8 struct {
	SueHeightTargetMax    float32 // Serial UDB Extra HEIGHT_TARGET_MAX
	SueHeightTargetMin    float32 // Serial UDB Extra HEIGHT_TARGET_MIN
	SueAltHoldThrottleMin float32 // Serial UDB Extra ALT_HOLD_THROTTLE_MIN
	SueAltHoldThrottleMax float32 // Serial UDB Extra ALT_HOLD_THROTTLE_MAX
	SueAltHoldPitchMin    float32 // Serial UDB Extra ALT_HOLD_PITCH_MIN
	SueAltHoldPitchMax    float32 // Serial UDB Extra ALT_HOLD_PITCH_MAX
	SueAltHoldPitchHigh   float32 // Serial UDB Extra ALT_HOLD_PITCH_HIGH
}

// Dialect (generated function)
func (m *MatrixpilotSerialUdbExtraF8) Dialect() *Dialect {
	return DialectMatrixpilot
}

// MsgID (generated function)
func (m *MatrixpilotSerialUdbExtraF8) MsgID() MessageID {
	return MSG_ID_SERIAL_UDB_EXTRA_F8
}

// MsgName (generated function)
func (m *MatrixpilotSerialUdbExtraF8) MsgName() string {
	return "SerialUdbExtraF8"
}

// Pack (generated function)
func (m *MatrixpilotSerialUdbExtraF8) Pack(p *Packet) error {
	payload := make([]byte, 28)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(m.SueHeightTargetMax))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(m.SueHeightTargetMin))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.SueAltHoldThrottleMin))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.SueAltHoldThrottleMax))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(m.SueAltHoldPitchMin))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(m.SueAltHoldPitchMax))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(m.SueAltHoldPitchHigh))
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
func (m *MatrixpilotSerialUdbExtraF8) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 28 {
		if MavlinkVersion == 1 {
			return errPayloadTooSmall
		}
		payload = append(payload, zeroTail[:28-len(p.Payload)]...)
	}
	m.SueHeightTargetMax = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.SueHeightTargetMin = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	m.SueAltHoldThrottleMin = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.SueAltHoldThrottleMax = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.SueAltHoldPitchMin = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	m.SueAltHoldPitchMax = math.Float32frombits(binary.LittleEndian.Uint32(payload[20:]))
	m.SueAltHoldPitchHigh = math.Float32frombits(binary.LittleEndian.Uint32(payload[24:]))
	return nil
}

// MatrixpilotSerialUdbExtraF13 struct (generated typeinfo)
// Backwards compatible version of SERIAL_UDB_EXTRA F13: format
type MatrixpilotSerialUdbExtraF13 struct {
	SueLatOrigin int32 // Serial UDB Extra MP Origin Latitude
	SueLonOrigin int32 // Serial UDB Extra MP Origin Longitude
	SueAltOrigin int32 // Serial UDB Extra MP Origin Altitude Above Sea Level
	SueWeekNo    int16 // Serial UDB Extra GPS Week Number
}

// Dialect (generated function)
func (m *MatrixpilotSerialUdbExtraF13) Dialect() *Dialect {
	return DialectMatrixpilot
}

// MsgID (generated function)
func (m *MatrixpilotSerialUdbExtraF13) MsgID() MessageID {
	return MSG_ID_SERIAL_UDB_EXTRA_F13
}

// MsgName (generated function)
func (m *MatrixpilotSerialUdbExtraF13) MsgName() string {
	return "SerialUdbExtraF13"
}

// Pack (generated function)
func (m *MatrixpilotSerialUdbExtraF13) Pack(p *Packet) error {
	payload := make([]byte, 14)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.SueLatOrigin))
	binary.LittleEndian.PutUint32(payload[4:], uint32(m.SueLonOrigin))
	binary.LittleEndian.PutUint32(payload[8:], uint32(m.SueAltOrigin))
	binary.LittleEndian.PutUint16(payload[12:], uint16(m.SueWeekNo))
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
func (m *MatrixpilotSerialUdbExtraF13) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 14 {
		if MavlinkVersion == 1 {
			return errPayloadTooSmall
		}
		payload = append(payload, zeroTail[:14-len(p.Payload)]...)
	}
	m.SueLatOrigin = int32(binary.LittleEndian.Uint32(payload[0:]))
	m.SueLonOrigin = int32(binary.LittleEndian.Uint32(payload[4:]))
	m.SueAltOrigin = int32(binary.LittleEndian.Uint32(payload[8:]))
	m.SueWeekNo = int16(binary.LittleEndian.Uint16(payload[12:]))
	return nil
}

// MatrixpilotSerialUdbExtraF14 struct (generated typeinfo)
// Backwards compatible version of SERIAL_UDB_EXTRA F14: format
type MatrixpilotSerialUdbExtraF14 struct {
	SueTrapSource     uint32 // Serial UDB Extra Type Program Address of Last Trap
	SueRcon           int16  // Serial UDB Extra Reboot Register of DSPIC
	SueTrapFlags      int16  // Serial UDB Extra  Last dspic Trap Flags
	SueOscFailCount   int16  // Serial UDB Extra Number of Ocillator Failures
	SueWindEstimation uint8  // Serial UDB Extra Wind Estimation Enabled
	SueGpsType        uint8  // Serial UDB Extra Type of GPS Unit
	SueDr             uint8  // Serial UDB Extra Dead Reckoning Enabled
	SueBoardType      uint8  // Serial UDB Extra Type of UDB Hardware
	SueAirframe       uint8  // Serial UDB Extra Type of Airframe
	SueClockConfig    uint8  // Serial UDB Extra UDB Internal Clock Configuration
	SueFlightPlanType uint8  // Serial UDB Extra Type of Flight Plan
}

// Dialect (generated function)
func (m *MatrixpilotSerialUdbExtraF14) Dialect() *Dialect {
	return DialectMatrixpilot
}

// MsgID (generated function)
func (m *MatrixpilotSerialUdbExtraF14) MsgID() MessageID {
	return MSG_ID_SERIAL_UDB_EXTRA_F14
}

// MsgName (generated function)
func (m *MatrixpilotSerialUdbExtraF14) MsgName() string {
	return "SerialUdbExtraF14"
}

// Pack (generated function)
func (m *MatrixpilotSerialUdbExtraF14) Pack(p *Packet) error {
	payload := make([]byte, 17)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.SueTrapSource))
	binary.LittleEndian.PutUint16(payload[4:], uint16(m.SueRcon))
	binary.LittleEndian.PutUint16(payload[6:], uint16(m.SueTrapFlags))
	binary.LittleEndian.PutUint16(payload[8:], uint16(m.SueOscFailCount))
	payload[10] = byte(m.SueWindEstimation)
	payload[11] = byte(m.SueGpsType)
	payload[12] = byte(m.SueDr)
	payload[13] = byte(m.SueBoardType)
	payload[14] = byte(m.SueAirframe)
	payload[15] = byte(m.SueClockConfig)
	payload[16] = byte(m.SueFlightPlanType)
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
func (m *MatrixpilotSerialUdbExtraF14) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 17 {
		if MavlinkVersion == 1 {
			return errPayloadTooSmall
		}
		payload = append(payload, zeroTail[:17-len(p.Payload)]...)
	}
	m.SueTrapSource = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.SueRcon = int16(binary.LittleEndian.Uint16(payload[4:]))
	m.SueTrapFlags = int16(binary.LittleEndian.Uint16(payload[6:]))
	m.SueOscFailCount = int16(binary.LittleEndian.Uint16(payload[8:]))
	m.SueWindEstimation = uint8(payload[10])
	m.SueGpsType = uint8(payload[11])
	m.SueDr = uint8(payload[12])
	m.SueBoardType = uint8(payload[13])
	m.SueAirframe = uint8(payload[14])
	m.SueClockConfig = uint8(payload[15])
	m.SueFlightPlanType = uint8(payload[16])
	return nil
}

// MatrixpilotSerialUdbExtraF15 struct (generated typeinfo)
// Backwards compatible version of SERIAL_UDB_EXTRA F15 format
type MatrixpilotSerialUdbExtraF15 struct {
	SueIDVehicleModelName    [40]uint8 // Serial UDB Extra Model Name Of Vehicle
	SueIDVehicleRegistration [20]uint8 // Serial UDB Extra Registraton Number of Vehicle
}

// Dialect (generated function)
func (m *MatrixpilotSerialUdbExtraF15) Dialect() *Dialect {
	return DialectMatrixpilot
}

// MsgID (generated function)
func (m *MatrixpilotSerialUdbExtraF15) MsgID() MessageID {
	return MSG_ID_SERIAL_UDB_EXTRA_F15
}

// MsgName (generated function)
func (m *MatrixpilotSerialUdbExtraF15) MsgName() string {
	return "SerialUdbExtraF15"
}

// Pack (generated function)
func (m *MatrixpilotSerialUdbExtraF15) Pack(p *Packet) error {
	payload := make([]byte, 60)
	copy(payload[0:], m.SueIDVehicleModelName[:])
	copy(payload[40:], m.SueIDVehicleRegistration[:])
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
func (m *MatrixpilotSerialUdbExtraF15) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 60 {
		if MavlinkVersion == 1 {
			return errPayloadTooSmall
		}
		payload = append(payload, zeroTail[:60-len(p.Payload)]...)
	}
	copy(m.SueIDVehicleModelName[:], payload[0:40])
	copy(m.SueIDVehicleRegistration[:], payload[40:60])
	return nil
}

// MatrixpilotSerialUdbExtraF16 struct (generated typeinfo)
// Backwards compatible version of SERIAL_UDB_EXTRA F16 format
type MatrixpilotSerialUdbExtraF16 struct {
	SueIDLeadPilot    [40]uint8 // Serial UDB Extra Name of Expected Lead Pilot
	SueIDDiyDronesURL [70]uint8 // Serial UDB Extra URL of Lead Pilot or Team
}

// Dialect (generated function)
func (m *MatrixpilotSerialUdbExtraF16) Dialect() *Dialect {
	return DialectMatrixpilot
}

// MsgID (generated function)
func (m *MatrixpilotSerialUdbExtraF16) MsgID() MessageID {
	return MSG_ID_SERIAL_UDB_EXTRA_F16
}

// MsgName (generated function)
func (m *MatrixpilotSerialUdbExtraF16) MsgName() string {
	return "SerialUdbExtraF16"
}

// Pack (generated function)
func (m *MatrixpilotSerialUdbExtraF16) Pack(p *Packet) error {
	payload := make([]byte, 110)
	copy(payload[0:], m.SueIDLeadPilot[:])
	copy(payload[40:], m.SueIDDiyDronesURL[:])
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
func (m *MatrixpilotSerialUdbExtraF16) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 110 {
		if MavlinkVersion == 1 {
			return errPayloadTooSmall
		}
		payload = append(payload, zeroTail[:110-len(p.Payload)]...)
	}
	copy(m.SueIDLeadPilot[:], payload[0:40])
	copy(m.SueIDDiyDronesURL[:], payload[40:110])
	return nil
}

// MatrixpilotAltitudes struct (generated typeinfo)
// The altitude measured by sensors and IMU
type MatrixpilotAltitudes struct {
	TimeBootMs     uint32 // Timestamp (milliseconds since system boot)
	AltGps         int32  // GPS altitude in meters, expressed as * 1000 (millimeters), above MSL
	AltImu         int32  // IMU altitude above ground in meters, expressed as * 1000 (millimeters)
	AltBarometric  int32  // barometeric altitude above ground in meters, expressed as * 1000 (millimeters)
	AltOpticalFlow int32  // Optical flow altitude above ground in meters, expressed as * 1000 (millimeters)
	AltRangeFinder int32  // Rangefinder Altitude above ground in meters, expressed as * 1000 (millimeters)
	AltExtra       int32  // Extra altitude above ground in meters, expressed as * 1000 (millimeters)
}

// Dialect (generated function)
func (m *MatrixpilotAltitudes) Dialect() *Dialect {
	return DialectMatrixpilot
}

// MsgID (generated function)
func (m *MatrixpilotAltitudes) MsgID() MessageID {
	return MSG_ID_ALTITUDES
}

// MsgName (generated function)
func (m *MatrixpilotAltitudes) MsgName() string {
	return "Altitudes"
}

// Pack (generated function)
func (m *MatrixpilotAltitudes) Pack(p *Packet) error {
	payload := make([]byte, 28)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.TimeBootMs))
	binary.LittleEndian.PutUint32(payload[4:], uint32(m.AltGps))
	binary.LittleEndian.PutUint32(payload[8:], uint32(m.AltImu))
	binary.LittleEndian.PutUint32(payload[12:], uint32(m.AltBarometric))
	binary.LittleEndian.PutUint32(payload[16:], uint32(m.AltOpticalFlow))
	binary.LittleEndian.PutUint32(payload[20:], uint32(m.AltRangeFinder))
	binary.LittleEndian.PutUint32(payload[24:], uint32(m.AltExtra))
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
func (m *MatrixpilotAltitudes) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 28 {
		if MavlinkVersion == 1 {
			return errPayloadTooSmall
		}
		payload = append(payload, zeroTail[:28-len(p.Payload)]...)
	}
	m.TimeBootMs = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.AltGps = int32(binary.LittleEndian.Uint32(payload[4:]))
	m.AltImu = int32(binary.LittleEndian.Uint32(payload[8:]))
	m.AltBarometric = int32(binary.LittleEndian.Uint32(payload[12:]))
	m.AltOpticalFlow = int32(binary.LittleEndian.Uint32(payload[16:]))
	m.AltRangeFinder = int32(binary.LittleEndian.Uint32(payload[20:]))
	m.AltExtra = int32(binary.LittleEndian.Uint32(payload[24:]))
	return nil
}

// MatrixpilotAirspeeds struct (generated typeinfo)
// The airspeed measured by sensors and IMU
type MatrixpilotAirspeeds struct {
	TimeBootMs         uint32 // Timestamp (milliseconds since system boot)
	AirspeedImu        int16  // Airspeed estimate from IMU, cm/s
	AirspeedPitot      int16  // Pitot measured forward airpseed, cm/s
	AirspeedHotWire    int16  // Hot wire anenometer measured airspeed, cm/s
	AirspeedUltrasonic int16  // Ultrasonic measured airspeed, cm/s
	Aoa                int16  // Angle of attack sensor, degrees * 10
	Aoy                int16  // Yaw angle sensor, degrees * 10
}

// Dialect (generated function)
func (m *MatrixpilotAirspeeds) Dialect() *Dialect {
	return DialectMatrixpilot
}

// MsgID (generated function)
func (m *MatrixpilotAirspeeds) MsgID() MessageID {
	return MSG_ID_AIRSPEEDS
}

// MsgName (generated function)
func (m *MatrixpilotAirspeeds) MsgName() string {
	return "Airspeeds"
}

// Pack (generated function)
func (m *MatrixpilotAirspeeds) Pack(p *Packet) error {
	payload := make([]byte, 16)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.TimeBootMs))
	binary.LittleEndian.PutUint16(payload[4:], uint16(m.AirspeedImu))
	binary.LittleEndian.PutUint16(payload[6:], uint16(m.AirspeedPitot))
	binary.LittleEndian.PutUint16(payload[8:], uint16(m.AirspeedHotWire))
	binary.LittleEndian.PutUint16(payload[10:], uint16(m.AirspeedUltrasonic))
	binary.LittleEndian.PutUint16(payload[12:], uint16(m.Aoa))
	binary.LittleEndian.PutUint16(payload[14:], uint16(m.Aoy))
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
func (m *MatrixpilotAirspeeds) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 16 {
		if MavlinkVersion == 1 {
			return errPayloadTooSmall
		}
		payload = append(payload, zeroTail[:16-len(p.Payload)]...)
	}
	m.TimeBootMs = uint32(binary.LittleEndian.Uint32(payload[0:]))
	m.AirspeedImu = int16(binary.LittleEndian.Uint16(payload[4:]))
	m.AirspeedPitot = int16(binary.LittleEndian.Uint16(payload[6:]))
	m.AirspeedHotWire = int16(binary.LittleEndian.Uint16(payload[8:]))
	m.AirspeedUltrasonic = int16(binary.LittleEndian.Uint16(payload[10:]))
	m.Aoa = int16(binary.LittleEndian.Uint16(payload[12:]))
	m.Aoy = int16(binary.LittleEndian.Uint16(payload[14:]))
	return nil
}

// MatrixpilotSerialUdbExtraF17 struct (generated typeinfo)
// Backwards compatible version of SERIAL_UDB_EXTRA F17 format
type MatrixpilotSerialUdbExtraF17 struct {
	SueFeedForward float32 // SUE Feed Forward Gain
	SueTurnRateNav float32 // SUE Max Turn Rate when Navigating
	SueTurnRateFbw float32 // SUE Max Turn Rate in Fly By Wire Mode
}

// Dialect (generated function)
func (m *MatrixpilotSerialUdbExtraF17) Dialect() *Dialect {
	return DialectMatrixpilot
}

// MsgID (generated function)
func (m *MatrixpilotSerialUdbExtraF17) MsgID() MessageID {
	return MSG_ID_SERIAL_UDB_EXTRA_F17
}

// MsgName (generated function)
func (m *MatrixpilotSerialUdbExtraF17) MsgName() string {
	return "SerialUdbExtraF17"
}

// Pack (generated function)
func (m *MatrixpilotSerialUdbExtraF17) Pack(p *Packet) error {
	payload := make([]byte, 12)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(m.SueFeedForward))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(m.SueTurnRateNav))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.SueTurnRateFbw))
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
func (m *MatrixpilotSerialUdbExtraF17) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 12 {
		if MavlinkVersion == 1 {
			return errPayloadTooSmall
		}
		payload = append(payload, zeroTail[:12-len(p.Payload)]...)
	}
	m.SueFeedForward = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.SueTurnRateNav = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	m.SueTurnRateFbw = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	return nil
}

// MatrixpilotSerialUdbExtraF18 struct (generated typeinfo)
// Backwards compatible version of SERIAL_UDB_EXTRA F18 format
type MatrixpilotSerialUdbExtraF18 struct {
	AngleOfAttackNormal   float32 // SUE Angle of Attack Normal
	AngleOfAttackInverted float32 // SUE Angle of Attack Inverted
	ElevatorTrimNormal    float32 // SUE Elevator Trim Normal
	ElevatorTrimInverted  float32 // SUE Elevator Trim Inverted
	ReferenceSpeed        float32 // SUE reference_speed
}

// Dialect (generated function)
func (m *MatrixpilotSerialUdbExtraF18) Dialect() *Dialect {
	return DialectMatrixpilot
}

// MsgID (generated function)
func (m *MatrixpilotSerialUdbExtraF18) MsgID() MessageID {
	return MSG_ID_SERIAL_UDB_EXTRA_F18
}

// MsgName (generated function)
func (m *MatrixpilotSerialUdbExtraF18) MsgName() string {
	return "SerialUdbExtraF18"
}

// Pack (generated function)
func (m *MatrixpilotSerialUdbExtraF18) Pack(p *Packet) error {
	payload := make([]byte, 20)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(m.AngleOfAttackNormal))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(m.AngleOfAttackInverted))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.ElevatorTrimNormal))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.ElevatorTrimInverted))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(m.ReferenceSpeed))
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
func (m *MatrixpilotSerialUdbExtraF18) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 20 {
		if MavlinkVersion == 1 {
			return errPayloadTooSmall
		}
		payload = append(payload, zeroTail[:20-len(p.Payload)]...)
	}
	m.AngleOfAttackNormal = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.AngleOfAttackInverted = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	m.ElevatorTrimNormal = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.ElevatorTrimInverted = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.ReferenceSpeed = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	return nil
}

// MatrixpilotSerialUdbExtraF19 struct (generated typeinfo)
// Backwards compatible version of SERIAL_UDB_EXTRA F19 format
type MatrixpilotSerialUdbExtraF19 struct {
	SueAileronOutputChannel  uint8 // SUE aileron output channel
	SueAileronReversed       uint8 // SUE aileron reversed
	SueElevatorOutputChannel uint8 // SUE elevator output channel
	SueElevatorReversed      uint8 // SUE elevator reversed
	SueThrottleOutputChannel uint8 // SUE throttle output channel
	SueThrottleReversed      uint8 // SUE throttle reversed
	SueRudderOutputChannel   uint8 // SUE rudder output channel
	SueRudderReversed        uint8 // SUE rudder reversed
}

// Dialect (generated function)
func (m *MatrixpilotSerialUdbExtraF19) Dialect() *Dialect {
	return DialectMatrixpilot
}

// MsgID (generated function)
func (m *MatrixpilotSerialUdbExtraF19) MsgID() MessageID {
	return MSG_ID_SERIAL_UDB_EXTRA_F19
}

// MsgName (generated function)
func (m *MatrixpilotSerialUdbExtraF19) MsgName() string {
	return "SerialUdbExtraF19"
}

// Pack (generated function)
func (m *MatrixpilotSerialUdbExtraF19) Pack(p *Packet) error {
	payload := make([]byte, 8)
	payload[0] = byte(m.SueAileronOutputChannel)
	payload[1] = byte(m.SueAileronReversed)
	payload[2] = byte(m.SueElevatorOutputChannel)
	payload[3] = byte(m.SueElevatorReversed)
	payload[4] = byte(m.SueThrottleOutputChannel)
	payload[5] = byte(m.SueThrottleReversed)
	payload[6] = byte(m.SueRudderOutputChannel)
	payload[7] = byte(m.SueRudderReversed)
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
func (m *MatrixpilotSerialUdbExtraF19) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 8 {
		if MavlinkVersion == 1 {
			return errPayloadTooSmall
		}
		payload = append(payload, zeroTail[:8-len(p.Payload)]...)
	}
	m.SueAileronOutputChannel = uint8(payload[0])
	m.SueAileronReversed = uint8(payload[1])
	m.SueElevatorOutputChannel = uint8(payload[2])
	m.SueElevatorReversed = uint8(payload[3])
	m.SueThrottleOutputChannel = uint8(payload[4])
	m.SueThrottleReversed = uint8(payload[5])
	m.SueRudderOutputChannel = uint8(payload[6])
	m.SueRudderReversed = uint8(payload[7])
	return nil
}

// MatrixpilotSerialUdbExtraF20 struct (generated typeinfo)
// Backwards compatible version of SERIAL_UDB_EXTRA F20 format
type MatrixpilotSerialUdbExtraF20 struct {
	SueTrimValueInput1  int16 // SUE UDB PWM Trim Value on Input 1
	SueTrimValueInput2  int16 // SUE UDB PWM Trim Value on Input 2
	SueTrimValueInput3  int16 // SUE UDB PWM Trim Value on Input 3
	SueTrimValueInput4  int16 // SUE UDB PWM Trim Value on Input 4
	SueTrimValueInput5  int16 // SUE UDB PWM Trim Value on Input 5
	SueTrimValueInput6  int16 // SUE UDB PWM Trim Value on Input 6
	SueTrimValueInput7  int16 // SUE UDB PWM Trim Value on Input 7
	SueTrimValueInput8  int16 // SUE UDB PWM Trim Value on Input 8
	SueTrimValueInput9  int16 // SUE UDB PWM Trim Value on Input 9
	SueTrimValueInput10 int16 // SUE UDB PWM Trim Value on Input 10
	SueTrimValueInput11 int16 // SUE UDB PWM Trim Value on Input 11
	SueTrimValueInput12 int16 // SUE UDB PWM Trim Value on Input 12
	SueNumberOfInputs   uint8 // SUE Number of Input Channels
}

// Dialect (generated function)
func (m *MatrixpilotSerialUdbExtraF20) Dialect() *Dialect {
	return DialectMatrixpilot
}

// MsgID (generated function)
func (m *MatrixpilotSerialUdbExtraF20) MsgID() MessageID {
	return MSG_ID_SERIAL_UDB_EXTRA_F20
}

// MsgName (generated function)
func (m *MatrixpilotSerialUdbExtraF20) MsgName() string {
	return "SerialUdbExtraF20"
}

// Pack (generated function)
func (m *MatrixpilotSerialUdbExtraF20) Pack(p *Packet) error {
	payload := make([]byte, 25)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.SueTrimValueInput1))
	binary.LittleEndian.PutUint16(payload[2:], uint16(m.SueTrimValueInput2))
	binary.LittleEndian.PutUint16(payload[4:], uint16(m.SueTrimValueInput3))
	binary.LittleEndian.PutUint16(payload[6:], uint16(m.SueTrimValueInput4))
	binary.LittleEndian.PutUint16(payload[8:], uint16(m.SueTrimValueInput5))
	binary.LittleEndian.PutUint16(payload[10:], uint16(m.SueTrimValueInput6))
	binary.LittleEndian.PutUint16(payload[12:], uint16(m.SueTrimValueInput7))
	binary.LittleEndian.PutUint16(payload[14:], uint16(m.SueTrimValueInput8))
	binary.LittleEndian.PutUint16(payload[16:], uint16(m.SueTrimValueInput9))
	binary.LittleEndian.PutUint16(payload[18:], uint16(m.SueTrimValueInput10))
	binary.LittleEndian.PutUint16(payload[20:], uint16(m.SueTrimValueInput11))
	binary.LittleEndian.PutUint16(payload[22:], uint16(m.SueTrimValueInput12))
	payload[24] = byte(m.SueNumberOfInputs)
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
func (m *MatrixpilotSerialUdbExtraF20) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 25 {
		if MavlinkVersion == 1 {
			return errPayloadTooSmall
		}
		payload = append(payload, zeroTail[:25-len(p.Payload)]...)
	}
	m.SueTrimValueInput1 = int16(binary.LittleEndian.Uint16(payload[0:]))
	m.SueTrimValueInput2 = int16(binary.LittleEndian.Uint16(payload[2:]))
	m.SueTrimValueInput3 = int16(binary.LittleEndian.Uint16(payload[4:]))
	m.SueTrimValueInput4 = int16(binary.LittleEndian.Uint16(payload[6:]))
	m.SueTrimValueInput5 = int16(binary.LittleEndian.Uint16(payload[8:]))
	m.SueTrimValueInput6 = int16(binary.LittleEndian.Uint16(payload[10:]))
	m.SueTrimValueInput7 = int16(binary.LittleEndian.Uint16(payload[12:]))
	m.SueTrimValueInput8 = int16(binary.LittleEndian.Uint16(payload[14:]))
	m.SueTrimValueInput9 = int16(binary.LittleEndian.Uint16(payload[16:]))
	m.SueTrimValueInput10 = int16(binary.LittleEndian.Uint16(payload[18:]))
	m.SueTrimValueInput11 = int16(binary.LittleEndian.Uint16(payload[20:]))
	m.SueTrimValueInput12 = int16(binary.LittleEndian.Uint16(payload[22:]))
	m.SueNumberOfInputs = uint8(payload[24])
	return nil
}

// MatrixpilotSerialUdbExtraF21 struct (generated typeinfo)
// Backwards compatible version of SERIAL_UDB_EXTRA F21 format
type MatrixpilotSerialUdbExtraF21 struct {
	SueAccelXOffset int16 // SUE X accelerometer offset
	SueAccelYOffset int16 // SUE Y accelerometer offset
	SueAccelZOffset int16 // SUE Z accelerometer offset
	SueGyroXOffset  int16 // SUE X gyro offset
	SueGyroYOffset  int16 // SUE Y gyro offset
	SueGyroZOffset  int16 // SUE Z gyro offset
}

// Dialect (generated function)
func (m *MatrixpilotSerialUdbExtraF21) Dialect() *Dialect {
	return DialectMatrixpilot
}

// MsgID (generated function)
func (m *MatrixpilotSerialUdbExtraF21) MsgID() MessageID {
	return MSG_ID_SERIAL_UDB_EXTRA_F21
}

// MsgName (generated function)
func (m *MatrixpilotSerialUdbExtraF21) MsgName() string {
	return "SerialUdbExtraF21"
}

// Pack (generated function)
func (m *MatrixpilotSerialUdbExtraF21) Pack(p *Packet) error {
	payload := make([]byte, 12)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.SueAccelXOffset))
	binary.LittleEndian.PutUint16(payload[2:], uint16(m.SueAccelYOffset))
	binary.LittleEndian.PutUint16(payload[4:], uint16(m.SueAccelZOffset))
	binary.LittleEndian.PutUint16(payload[6:], uint16(m.SueGyroXOffset))
	binary.LittleEndian.PutUint16(payload[8:], uint16(m.SueGyroYOffset))
	binary.LittleEndian.PutUint16(payload[10:], uint16(m.SueGyroZOffset))
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
func (m *MatrixpilotSerialUdbExtraF21) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 12 {
		if MavlinkVersion == 1 {
			return errPayloadTooSmall
		}
		payload = append(payload, zeroTail[:12-len(p.Payload)]...)
	}
	m.SueAccelXOffset = int16(binary.LittleEndian.Uint16(payload[0:]))
	m.SueAccelYOffset = int16(binary.LittleEndian.Uint16(payload[2:]))
	m.SueAccelZOffset = int16(binary.LittleEndian.Uint16(payload[4:]))
	m.SueGyroXOffset = int16(binary.LittleEndian.Uint16(payload[6:]))
	m.SueGyroYOffset = int16(binary.LittleEndian.Uint16(payload[8:]))
	m.SueGyroZOffset = int16(binary.LittleEndian.Uint16(payload[10:]))
	return nil
}

// MatrixpilotSerialUdbExtraF22 struct (generated typeinfo)
// Backwards compatible version of SERIAL_UDB_EXTRA F22 format
type MatrixpilotSerialUdbExtraF22 struct {
	SueAccelXAtCalibration int16 // SUE X accelerometer at calibration time
	SueAccelYAtCalibration int16 // SUE Y accelerometer at calibration time
	SueAccelZAtCalibration int16 // SUE Z accelerometer at calibration time
	SueGyroXAtCalibration  int16 // SUE X gyro at calibration time
	SueGyroYAtCalibration  int16 // SUE Y gyro at calibration time
	SueGyroZAtCalibration  int16 // SUE Z gyro at calibration time
}

// Dialect (generated function)
func (m *MatrixpilotSerialUdbExtraF22) Dialect() *Dialect {
	return DialectMatrixpilot
}

// MsgID (generated function)
func (m *MatrixpilotSerialUdbExtraF22) MsgID() MessageID {
	return MSG_ID_SERIAL_UDB_EXTRA_F22
}

// MsgName (generated function)
func (m *MatrixpilotSerialUdbExtraF22) MsgName() string {
	return "SerialUdbExtraF22"
}

// Pack (generated function)
func (m *MatrixpilotSerialUdbExtraF22) Pack(p *Packet) error {
	payload := make([]byte, 12)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.SueAccelXAtCalibration))
	binary.LittleEndian.PutUint16(payload[2:], uint16(m.SueAccelYAtCalibration))
	binary.LittleEndian.PutUint16(payload[4:], uint16(m.SueAccelZAtCalibration))
	binary.LittleEndian.PutUint16(payload[6:], uint16(m.SueGyroXAtCalibration))
	binary.LittleEndian.PutUint16(payload[8:], uint16(m.SueGyroYAtCalibration))
	binary.LittleEndian.PutUint16(payload[10:], uint16(m.SueGyroZAtCalibration))
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
func (m *MatrixpilotSerialUdbExtraF22) Unpack(p *Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 12 {
		if MavlinkVersion == 1 {
			return errPayloadTooSmall
		}
		payload = append(payload, zeroTail[:12-len(p.Payload)]...)
	}
	m.SueAccelXAtCalibration = int16(binary.LittleEndian.Uint16(payload[0:]))
	m.SueAccelYAtCalibration = int16(binary.LittleEndian.Uint16(payload[2:]))
	m.SueAccelZAtCalibration = int16(binary.LittleEndian.Uint16(payload[4:]))
	m.SueGyroXAtCalibration = int16(binary.LittleEndian.Uint16(payload[6:]))
	m.SueGyroYAtCalibration = int16(binary.LittleEndian.Uint16(payload[8:]))
	m.SueGyroZAtCalibration = int16(binary.LittleEndian.Uint16(payload[10:]))
	return nil
}

// Message IDs
const (
	MSG_ID_FLEXIFUNCTION_SET                 MessageID = 150
	MSG_ID_FLEXIFUNCTION_READ_REQ            MessageID = 151
	MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION     MessageID = 152
	MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_ACK MessageID = 153
	MSG_ID_FLEXIFUNCTION_DIRECTORY           MessageID = 155
	MSG_ID_FLEXIFUNCTION_DIRECTORY_ACK       MessageID = 156
	MSG_ID_FLEXIFUNCTION_COMMAND             MessageID = 157
	MSG_ID_FLEXIFUNCTION_COMMAND_ACK         MessageID = 158
	MSG_ID_SERIAL_UDB_EXTRA_F2_A             MessageID = 170
	MSG_ID_SERIAL_UDB_EXTRA_F2_B             MessageID = 171
	MSG_ID_SERIAL_UDB_EXTRA_F4               MessageID = 172
	MSG_ID_SERIAL_UDB_EXTRA_F5               MessageID = 173
	MSG_ID_SERIAL_UDB_EXTRA_F6               MessageID = 174
	MSG_ID_SERIAL_UDB_EXTRA_F7               MessageID = 175
	MSG_ID_SERIAL_UDB_EXTRA_F8               MessageID = 176
	MSG_ID_SERIAL_UDB_EXTRA_F13              MessageID = 177
	MSG_ID_SERIAL_UDB_EXTRA_F14              MessageID = 178
	MSG_ID_SERIAL_UDB_EXTRA_F15              MessageID = 179
	MSG_ID_SERIAL_UDB_EXTRA_F16              MessageID = 180
	MSG_ID_ALTITUDES                         MessageID = 181
	MSG_ID_AIRSPEEDS                         MessageID = 182
	MSG_ID_SERIAL_UDB_EXTRA_F17              MessageID = 183
	MSG_ID_SERIAL_UDB_EXTRA_F18              MessageID = 184
	MSG_ID_SERIAL_UDB_EXTRA_F19              MessageID = 185
	MSG_ID_SERIAL_UDB_EXTRA_F20              MessageID = 186
	MSG_ID_SERIAL_UDB_EXTRA_F21              MessageID = 187
	MSG_ID_SERIAL_UDB_EXTRA_F22              MessageID = 188
)

// DialectMatrixpilot is the dialect represented by matrixpilot.xml
var DialectMatrixpilot = &Dialect{
	Name: "matrixpilot",
	crcExtras: map[MessageID]uint8{
		MSG_ID_FLEXIFUNCTION_SET:                 181,
		MSG_ID_FLEXIFUNCTION_READ_REQ:            26,
		MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION:     101,
		MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_ACK: 109,
		MSG_ID_FLEXIFUNCTION_DIRECTORY:           12,
		MSG_ID_FLEXIFUNCTION_DIRECTORY_ACK:       218,
		MSG_ID_FLEXIFUNCTION_COMMAND:             133,
		MSG_ID_FLEXIFUNCTION_COMMAND_ACK:         208,
		MSG_ID_SERIAL_UDB_EXTRA_F2_A:             103,
		MSG_ID_SERIAL_UDB_EXTRA_F2_B:             245,
		MSG_ID_SERIAL_UDB_EXTRA_F4:               191,
		MSG_ID_SERIAL_UDB_EXTRA_F5:               54,
		MSG_ID_SERIAL_UDB_EXTRA_F6:               54,
		MSG_ID_SERIAL_UDB_EXTRA_F7:               171,
		MSG_ID_SERIAL_UDB_EXTRA_F8:               142,
		MSG_ID_SERIAL_UDB_EXTRA_F13:              249,
		MSG_ID_SERIAL_UDB_EXTRA_F14:              123,
		MSG_ID_SERIAL_UDB_EXTRA_F15:              7,
		MSG_ID_SERIAL_UDB_EXTRA_F16:              222,
		MSG_ID_ALTITUDES:                         55,
		MSG_ID_AIRSPEEDS:                         154,
		MSG_ID_SERIAL_UDB_EXTRA_F17:              175,
		MSG_ID_SERIAL_UDB_EXTRA_F18:              41,
		MSG_ID_SERIAL_UDB_EXTRA_F19:              87,
		MSG_ID_SERIAL_UDB_EXTRA_F20:              144,
		MSG_ID_SERIAL_UDB_EXTRA_F21:              134,
		MSG_ID_SERIAL_UDB_EXTRA_F22:              91,
	},
	messageConstructorByMsgID: map[MessageID]func(*Packet) Message{
		MSG_ID_FLEXIFUNCTION_SET: func(pkt *Packet) Message {
			msg := new(MatrixpilotFlexifunctionSet)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_FLEXIFUNCTION_READ_REQ: func(pkt *Packet) Message {
			msg := new(MatrixpilotFlexifunctionReadReq)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION: func(pkt *Packet) Message {
			msg := new(MatrixpilotFlexifunctionBufferFunction)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_ACK: func(pkt *Packet) Message {
			msg := new(MatrixpilotFlexifunctionBufferFunctionAck)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_FLEXIFUNCTION_DIRECTORY: func(pkt *Packet) Message {
			msg := new(MatrixpilotFlexifunctionDirectory)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_FLEXIFUNCTION_DIRECTORY_ACK: func(pkt *Packet) Message {
			msg := new(MatrixpilotFlexifunctionDirectoryAck)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_FLEXIFUNCTION_COMMAND: func(pkt *Packet) Message {
			msg := new(MatrixpilotFlexifunctionCommand)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_FLEXIFUNCTION_COMMAND_ACK: func(pkt *Packet) Message {
			msg := new(MatrixpilotFlexifunctionCommandAck)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SERIAL_UDB_EXTRA_F2_A: func(pkt *Packet) Message {
			msg := new(MatrixpilotSerialUdbExtraF2A)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SERIAL_UDB_EXTRA_F2_B: func(pkt *Packet) Message {
			msg := new(MatrixpilotSerialUdbExtraF2B)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SERIAL_UDB_EXTRA_F4: func(pkt *Packet) Message {
			msg := new(MatrixpilotSerialUdbExtraF4)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SERIAL_UDB_EXTRA_F5: func(pkt *Packet) Message {
			msg := new(MatrixpilotSerialUdbExtraF5)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SERIAL_UDB_EXTRA_F6: func(pkt *Packet) Message {
			msg := new(MatrixpilotSerialUdbExtraF6)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SERIAL_UDB_EXTRA_F7: func(pkt *Packet) Message {
			msg := new(MatrixpilotSerialUdbExtraF7)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SERIAL_UDB_EXTRA_F8: func(pkt *Packet) Message {
			msg := new(MatrixpilotSerialUdbExtraF8)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SERIAL_UDB_EXTRA_F13: func(pkt *Packet) Message {
			msg := new(MatrixpilotSerialUdbExtraF13)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SERIAL_UDB_EXTRA_F14: func(pkt *Packet) Message {
			msg := new(MatrixpilotSerialUdbExtraF14)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SERIAL_UDB_EXTRA_F15: func(pkt *Packet) Message {
			msg := new(MatrixpilotSerialUdbExtraF15)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SERIAL_UDB_EXTRA_F16: func(pkt *Packet) Message {
			msg := new(MatrixpilotSerialUdbExtraF16)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_ALTITUDES: func(pkt *Packet) Message {
			msg := new(MatrixpilotAltitudes)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_AIRSPEEDS: func(pkt *Packet) Message {
			msg := new(MatrixpilotAirspeeds)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SERIAL_UDB_EXTRA_F17: func(pkt *Packet) Message {
			msg := new(MatrixpilotSerialUdbExtraF17)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SERIAL_UDB_EXTRA_F18: func(pkt *Packet) Message {
			msg := new(MatrixpilotSerialUdbExtraF18)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SERIAL_UDB_EXTRA_F19: func(pkt *Packet) Message {
			msg := new(MatrixpilotSerialUdbExtraF19)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SERIAL_UDB_EXTRA_F20: func(pkt *Packet) Message {
			msg := new(MatrixpilotSerialUdbExtraF20)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SERIAL_UDB_EXTRA_F21: func(pkt *Packet) Message {
			msg := new(MatrixpilotSerialUdbExtraF21)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SERIAL_UDB_EXTRA_F22: func(pkt *Packet) Message {
			msg := new(MatrixpilotSerialUdbExtraF22)
			msg.Unpack(pkt)
			return msg
		},
	},
}

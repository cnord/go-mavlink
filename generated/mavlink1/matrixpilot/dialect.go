//////////////////////////////////////////////////
//
// NOTE: do not edit,
// this file created automatically by mavgen.go
//
//////////////////////////////////////////////////

package matrixpilot

import (
	mavlink ".."
	"encoding/binary"
	"fmt"
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
	MAV_CMD_PREFLIGHT_STORAGE_ADVANCED = 0 // Request storage of different parameter values and logs. This command will be only accepted if in pre-flight mode. Params: 1) Storage action: Action defined by MAV_PREFLIGHT_STORAGE_ACTION_ADVANCED; 2) Storage area as defined by parameter database; 3) Storage flags as defined by parameter database; 4) Empty; 5) Empty; 6) Empty; 7) Empty;
)

// FlexifunctionSet struct (generated typeinfo)
// Depreciated but used as a compiler flag.  Do not remove
type FlexifunctionSet struct {
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
}

// MsgID (generated function)
func (m *FlexifunctionSet) MsgID() mavlink.MessageID {
	return MSG_ID_FLEXIFUNCTION_SET
}

// String (generated function)
func (m *FlexifunctionSet) String() string {
	return fmt.Sprintf(
		"&matrixpilot.FlexifunctionSet{ TargetSystem: %+v, TargetComponent: %+v }",
		m.TargetSystem,
		m.TargetComponent,
	)
}

// Pack (generated function)
func (m *FlexifunctionSet) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 2)
	payload[0] = byte(m.TargetSystem)
	payload[1] = byte(m.TargetComponent)
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *FlexifunctionSet) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 2 {
		return mavlink.ErrPayloadTooSmall
	}
	m.TargetSystem = uint8(payload[0])
	m.TargetComponent = uint8(payload[1])
	return nil
}

// FlexifunctionReadReq struct (generated typeinfo)
// Reqest reading of flexifunction data
type FlexifunctionReadReq struct {
	ReadReqType     int16 // Type of flexifunction data requested
	DataIndex       int16 // index into data where needed
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
}

// MsgID (generated function)
func (m *FlexifunctionReadReq) MsgID() mavlink.MessageID {
	return MSG_ID_FLEXIFUNCTION_READ_REQ
}

// String (generated function)
func (m *FlexifunctionReadReq) String() string {
	return fmt.Sprintf(
		"&matrixpilot.FlexifunctionReadReq{ ReadReqType: %+v, DataIndex: %+v, TargetSystem: %+v, TargetComponent: %+v }",
		m.ReadReqType,
		m.DataIndex,
		m.TargetSystem,
		m.TargetComponent,
	)
}

// Pack (generated function)
func (m *FlexifunctionReadReq) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 6)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.ReadReqType))
	binary.LittleEndian.PutUint16(payload[2:], uint16(m.DataIndex))
	payload[4] = byte(m.TargetSystem)
	payload[5] = byte(m.TargetComponent)
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *FlexifunctionReadReq) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 6 {
		return mavlink.ErrPayloadTooSmall
	}
	m.ReadReqType = int16(binary.LittleEndian.Uint16(payload[0:]))
	m.DataIndex = int16(binary.LittleEndian.Uint16(payload[2:]))
	m.TargetSystem = uint8(payload[4])
	m.TargetComponent = uint8(payload[5])
	return nil
}

// FlexifunctionBufferFunction struct (generated typeinfo)
// Flexifunction type and parameters for component at function index from buffer
type FlexifunctionBufferFunction struct {
	FuncIndex       uint16   // Function index
	FuncCount       uint16   // Total count of functions
	DataAddress     uint16   // Address in the flexifunction data, Set to 0xFFFF to use address in target memory
	DataSize        uint16   // Size of the
	TargetSystem    uint8    // System ID
	TargetComponent uint8    // Component ID
	Data            [48]int8 // Settings data
}

// MsgID (generated function)
func (m *FlexifunctionBufferFunction) MsgID() mavlink.MessageID {
	return MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION
}

// String (generated function)
func (m *FlexifunctionBufferFunction) String() string {
	return fmt.Sprintf(
		"&matrixpilot.FlexifunctionBufferFunction{ FuncIndex: %+v, FuncCount: %+v, DataAddress: %+v, DataSize: %+v, TargetSystem: %+v, TargetComponent: %+v, Data: %+v }",
		m.FuncIndex,
		m.FuncCount,
		m.DataAddress,
		m.DataSize,
		m.TargetSystem,
		m.TargetComponent,
		m.Data,
	)
}

// Pack (generated function)
func (m *FlexifunctionBufferFunction) Pack(p *mavlink.Packet) error {
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *FlexifunctionBufferFunction) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 58 {
		return mavlink.ErrPayloadTooSmall
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

// FlexifunctionBufferFunctionAck struct (generated typeinfo)
// Flexifunction type and parameters for component at function index from buffer
type FlexifunctionBufferFunctionAck struct {
	FuncIndex       uint16 // Function index
	Result          uint16 // result of acknowledge, 0=fail, 1=good
	TargetSystem    uint8  // System ID
	TargetComponent uint8  // Component ID
}

// MsgID (generated function)
func (m *FlexifunctionBufferFunctionAck) MsgID() mavlink.MessageID {
	return MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_ACK
}

// String (generated function)
func (m *FlexifunctionBufferFunctionAck) String() string {
	return fmt.Sprintf(
		"&matrixpilot.FlexifunctionBufferFunctionAck{ FuncIndex: %+v, Result: %+v, TargetSystem: %+v, TargetComponent: %+v }",
		m.FuncIndex,
		m.Result,
		m.TargetSystem,
		m.TargetComponent,
	)
}

// Pack (generated function)
func (m *FlexifunctionBufferFunctionAck) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 6)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.FuncIndex))
	binary.LittleEndian.PutUint16(payload[2:], uint16(m.Result))
	payload[4] = byte(m.TargetSystem)
	payload[5] = byte(m.TargetComponent)
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *FlexifunctionBufferFunctionAck) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 6 {
		return mavlink.ErrPayloadTooSmall
	}
	m.FuncIndex = uint16(binary.LittleEndian.Uint16(payload[0:]))
	m.Result = uint16(binary.LittleEndian.Uint16(payload[2:]))
	m.TargetSystem = uint8(payload[4])
	m.TargetComponent = uint8(payload[5])
	return nil
}

// FlexifunctionDirectory struct (generated typeinfo)
// Acknowldge sucess or failure of a flexifunction command
type FlexifunctionDirectory struct {
	TargetSystem    uint8    // System ID
	TargetComponent uint8    // Component ID
	DirectoryType   uint8    // 0=inputs, 1=outputs
	StartIndex      uint8    // index of first directory entry to write
	Count           uint8    // count of directory entries to write
	DirectoryData   [48]int8 // Settings data
}

// MsgID (generated function)
func (m *FlexifunctionDirectory) MsgID() mavlink.MessageID {
	return MSG_ID_FLEXIFUNCTION_DIRECTORY
}

// String (generated function)
func (m *FlexifunctionDirectory) String() string {
	return fmt.Sprintf(
		"&matrixpilot.FlexifunctionDirectory{ TargetSystem: %+v, TargetComponent: %+v, DirectoryType: %+v, StartIndex: %+v, Count: %+v, DirectoryData: %+v }",
		m.TargetSystem,
		m.TargetComponent,
		m.DirectoryType,
		m.StartIndex,
		m.Count,
		m.DirectoryData,
	)
}

// Pack (generated function)
func (m *FlexifunctionDirectory) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 53)
	payload[0] = byte(m.TargetSystem)
	payload[1] = byte(m.TargetComponent)
	payload[2] = byte(m.DirectoryType)
	payload[3] = byte(m.StartIndex)
	payload[4] = byte(m.Count)
	for i, v := range m.DirectoryData {
		payload[5+i*1] = byte(v)
	}
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *FlexifunctionDirectory) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 53 {
		return mavlink.ErrPayloadTooSmall
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

// FlexifunctionDirectoryAck struct (generated typeinfo)
// Acknowldge sucess or failure of a flexifunction command
type FlexifunctionDirectoryAck struct {
	Result          uint16 // result of acknowledge, 0=fail, 1=good
	TargetSystem    uint8  // System ID
	TargetComponent uint8  // Component ID
	DirectoryType   uint8  // 0=inputs, 1=outputs
	StartIndex      uint8  // index of first directory entry to write
	Count           uint8  // count of directory entries to write
}

// MsgID (generated function)
func (m *FlexifunctionDirectoryAck) MsgID() mavlink.MessageID {
	return MSG_ID_FLEXIFUNCTION_DIRECTORY_ACK
}

// String (generated function)
func (m *FlexifunctionDirectoryAck) String() string {
	return fmt.Sprintf(
		"&matrixpilot.FlexifunctionDirectoryAck{ Result: %+v, TargetSystem: %+v, TargetComponent: %+v, DirectoryType: %+v, StartIndex: %+v, Count: %+v }",
		m.Result,
		m.TargetSystem,
		m.TargetComponent,
		m.DirectoryType,
		m.StartIndex,
		m.Count,
	)
}

// Pack (generated function)
func (m *FlexifunctionDirectoryAck) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 7)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.Result))
	payload[2] = byte(m.TargetSystem)
	payload[3] = byte(m.TargetComponent)
	payload[4] = byte(m.DirectoryType)
	payload[5] = byte(m.StartIndex)
	payload[6] = byte(m.Count)
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *FlexifunctionDirectoryAck) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 7 {
		return mavlink.ErrPayloadTooSmall
	}
	m.Result = uint16(binary.LittleEndian.Uint16(payload[0:]))
	m.TargetSystem = uint8(payload[2])
	m.TargetComponent = uint8(payload[3])
	m.DirectoryType = uint8(payload[4])
	m.StartIndex = uint8(payload[5])
	m.Count = uint8(payload[6])
	return nil
}

// FlexifunctionCommand struct (generated typeinfo)
// Acknowldge sucess or failure of a flexifunction command
type FlexifunctionCommand struct {
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
	CommandType     uint8 // Flexifunction command type
}

// MsgID (generated function)
func (m *FlexifunctionCommand) MsgID() mavlink.MessageID {
	return MSG_ID_FLEXIFUNCTION_COMMAND
}

// String (generated function)
func (m *FlexifunctionCommand) String() string {
	return fmt.Sprintf(
		"&matrixpilot.FlexifunctionCommand{ TargetSystem: %+v, TargetComponent: %+v, CommandType: %+v }",
		m.TargetSystem,
		m.TargetComponent,
		m.CommandType,
	)
}

// Pack (generated function)
func (m *FlexifunctionCommand) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 3)
	payload[0] = byte(m.TargetSystem)
	payload[1] = byte(m.TargetComponent)
	payload[2] = byte(m.CommandType)
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *FlexifunctionCommand) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 3 {
		return mavlink.ErrPayloadTooSmall
	}
	m.TargetSystem = uint8(payload[0])
	m.TargetComponent = uint8(payload[1])
	m.CommandType = uint8(payload[2])
	return nil
}

// FlexifunctionCommandAck struct (generated typeinfo)
// Acknowldge sucess or failure of a flexifunction command
type FlexifunctionCommandAck struct {
	CommandType uint16 // Command acknowledged
	Result      uint16 // result of acknowledge
}

// MsgID (generated function)
func (m *FlexifunctionCommandAck) MsgID() mavlink.MessageID {
	return MSG_ID_FLEXIFUNCTION_COMMAND_ACK
}

// String (generated function)
func (m *FlexifunctionCommandAck) String() string {
	return fmt.Sprintf(
		"&matrixpilot.FlexifunctionCommandAck{ CommandType: %+v, Result: %+v }",
		m.CommandType,
		m.Result,
	)
}

// Pack (generated function)
func (m *FlexifunctionCommandAck) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 4)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.CommandType))
	binary.LittleEndian.PutUint16(payload[2:], uint16(m.Result))
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *FlexifunctionCommandAck) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 4 {
		return mavlink.ErrPayloadTooSmall
	}
	m.CommandType = uint16(binary.LittleEndian.Uint16(payload[0:]))
	m.Result = uint16(binary.LittleEndian.Uint16(payload[2:]))
	return nil
}

// SerialUdbExtraF2A struct (generated typeinfo)
// Backwards compatible MAVLink version of SERIAL_UDB_EXTRA - F2: Format Part A
type SerialUdbExtraF2A struct {
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

// MsgID (generated function)
func (m *SerialUdbExtraF2A) MsgID() mavlink.MessageID {
	return MSG_ID_SERIAL_UDB_EXTRA_F2_A
}

// String (generated function)
func (m *SerialUdbExtraF2A) String() string {
	return fmt.Sprintf(
		"&matrixpilot.SerialUdbExtraF2A{ SueTime: %+v, SueLatitude: %+v, SueLongitude: %+v, SueAltitude: %+v, SueWaypointIndex: %+v, SueRmat0: %+v, SueRmat1: %+v, SueRmat2: %+v, SueRmat3: %+v, SueRmat4: %+v, SueRmat5: %+v, SueRmat6: %+v, SueRmat7: %+v, SueRmat8: %+v, SueCog: %+v, SueSog: %+v, SueCPULoad: %+v, SueAirSpeed3dimu: %+v, SueEstimatedWind0: %+v, SueEstimatedWind1: %+v, SueEstimatedWind2: %+v, SueMagfieldearth0: %+v, SueMagfieldearth1: %+v, SueMagfieldearth2: %+v, SueSvs: %+v, SueHdop: %+v, SueStatus: %+v }",
		m.SueTime,
		m.SueLatitude,
		m.SueLongitude,
		m.SueAltitude,
		m.SueWaypointIndex,
		m.SueRmat0,
		m.SueRmat1,
		m.SueRmat2,
		m.SueRmat3,
		m.SueRmat4,
		m.SueRmat5,
		m.SueRmat6,
		m.SueRmat7,
		m.SueRmat8,
		m.SueCog,
		m.SueSog,
		m.SueCPULoad,
		m.SueAirSpeed3dimu,
		m.SueEstimatedWind0,
		m.SueEstimatedWind1,
		m.SueEstimatedWind2,
		m.SueMagfieldearth0,
		m.SueMagfieldearth1,
		m.SueMagfieldearth2,
		m.SueSvs,
		m.SueHdop,
		m.SueStatus,
	)
}

// Pack (generated function)
func (m *SerialUdbExtraF2A) Pack(p *mavlink.Packet) error {
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *SerialUdbExtraF2A) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 61 {
		return mavlink.ErrPayloadTooSmall
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

// SerialUdbExtraF2B struct (generated typeinfo)
// Backwards compatible version of SERIAL_UDB_EXTRA - F2: Part B
type SerialUdbExtraF2B struct {
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

// MsgID (generated function)
func (m *SerialUdbExtraF2B) MsgID() mavlink.MessageID {
	return MSG_ID_SERIAL_UDB_EXTRA_F2_B
}

// String (generated function)
func (m *SerialUdbExtraF2B) String() string {
	return fmt.Sprintf(
		"&matrixpilot.SerialUdbExtraF2B{ SueTime: %+v, SueFlags: %+v, SueBaromPress: %+v, SueBaromAlt: %+v, SuePwmInput1: %+v, SuePwmInput2: %+v, SuePwmInput3: %+v, SuePwmInput4: %+v, SuePwmInput5: %+v, SuePwmInput6: %+v, SuePwmInput7: %+v, SuePwmInput8: %+v, SuePwmInput9: %+v, SuePwmInput10: %+v, SuePwmInput11: %+v, SuePwmInput12: %+v, SuePwmOutput1: %+v, SuePwmOutput2: %+v, SuePwmOutput3: %+v, SuePwmOutput4: %+v, SuePwmOutput5: %+v, SuePwmOutput6: %+v, SuePwmOutput7: %+v, SuePwmOutput8: %+v, SuePwmOutput9: %+v, SuePwmOutput10: %+v, SuePwmOutput11: %+v, SuePwmOutput12: %+v, SueImuLocationX: %+v, SueImuLocationY: %+v, SueImuLocationZ: %+v, SueLocationErrorEarthX: %+v, SueLocationErrorEarthY: %+v, SueLocationErrorEarthZ: %+v, SueOscFails: %+v, SueImuVelocityX: %+v, SueImuVelocityY: %+v, SueImuVelocityZ: %+v, SueWaypointGoalX: %+v, SueWaypointGoalY: %+v, SueWaypointGoalZ: %+v, SueAeroX: %+v, SueAeroY: %+v, SueAeroZ: %+v, SueBaromTemp: %+v, SueBatVolt: %+v, SueBatAmp: %+v, SueBatAmpHours: %+v, SueDesiredHeight: %+v, SueMemoryStackFree: %+v }",
		m.SueTime,
		m.SueFlags,
		m.SueBaromPress,
		m.SueBaromAlt,
		m.SuePwmInput1,
		m.SuePwmInput2,
		m.SuePwmInput3,
		m.SuePwmInput4,
		m.SuePwmInput5,
		m.SuePwmInput6,
		m.SuePwmInput7,
		m.SuePwmInput8,
		m.SuePwmInput9,
		m.SuePwmInput10,
		m.SuePwmInput11,
		m.SuePwmInput12,
		m.SuePwmOutput1,
		m.SuePwmOutput2,
		m.SuePwmOutput3,
		m.SuePwmOutput4,
		m.SuePwmOutput5,
		m.SuePwmOutput6,
		m.SuePwmOutput7,
		m.SuePwmOutput8,
		m.SuePwmOutput9,
		m.SuePwmOutput10,
		m.SuePwmOutput11,
		m.SuePwmOutput12,
		m.SueImuLocationX,
		m.SueImuLocationY,
		m.SueImuLocationZ,
		m.SueLocationErrorEarthX,
		m.SueLocationErrorEarthY,
		m.SueLocationErrorEarthZ,
		m.SueOscFails,
		m.SueImuVelocityX,
		m.SueImuVelocityY,
		m.SueImuVelocityZ,
		m.SueWaypointGoalX,
		m.SueWaypointGoalY,
		m.SueWaypointGoalZ,
		m.SueAeroX,
		m.SueAeroY,
		m.SueAeroZ,
		m.SueBaromTemp,
		m.SueBatVolt,
		m.SueBatAmp,
		m.SueBatAmpHours,
		m.SueDesiredHeight,
		m.SueMemoryStackFree,
	)
}

// Pack (generated function)
func (m *SerialUdbExtraF2B) Pack(p *mavlink.Packet) error {
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *SerialUdbExtraF2B) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 108 {
		return mavlink.ErrPayloadTooSmall
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

// SerialUdbExtraF4 struct (generated typeinfo)
// Backwards compatible version of SERIAL_UDB_EXTRA F4: format
type SerialUdbExtraF4 struct {
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

// MsgID (generated function)
func (m *SerialUdbExtraF4) MsgID() mavlink.MessageID {
	return MSG_ID_SERIAL_UDB_EXTRA_F4
}

// String (generated function)
func (m *SerialUdbExtraF4) String() string {
	return fmt.Sprintf(
		"&matrixpilot.SerialUdbExtraF4{ SueRollStabilizationAilerons: %+v, SueRollStabilizationRudder: %+v, SuePitchStabilization: %+v, SueYawStabilizationRudder: %+v, SueYawStabilizationAileron: %+v, SueAileronNavigation: %+v, SueRudderNavigation: %+v, SueAltitudeholdStabilized: %+v, SueAltitudeholdWaypoint: %+v, SueRacingMode: %+v }",
		m.SueRollStabilizationAilerons,
		m.SueRollStabilizationRudder,
		m.SuePitchStabilization,
		m.SueYawStabilizationRudder,
		m.SueYawStabilizationAileron,
		m.SueAileronNavigation,
		m.SueRudderNavigation,
		m.SueAltitudeholdStabilized,
		m.SueAltitudeholdWaypoint,
		m.SueRacingMode,
	)
}

// Pack (generated function)
func (m *SerialUdbExtraF4) Pack(p *mavlink.Packet) error {
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *SerialUdbExtraF4) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 10 {
		return mavlink.ErrPayloadTooSmall
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

// SerialUdbExtraF5 struct (generated typeinfo)
// Backwards compatible version of SERIAL_UDB_EXTRA F5: format
type SerialUdbExtraF5 struct {
	SueYawkpAileron float32 // Serial UDB YAWKP_AILERON Gain for Proporional control of navigation
	SueYawkdAileron float32 // Serial UDB YAWKD_AILERON Gain for Rate control of navigation
	SueRollkp       float32 // Serial UDB Extra ROLLKP Gain for Proportional control of roll stabilization
	SueRollkd       float32 // Serial UDB Extra ROLLKD Gain for Rate control of roll stabilization
}

// MsgID (generated function)
func (m *SerialUdbExtraF5) MsgID() mavlink.MessageID {
	return MSG_ID_SERIAL_UDB_EXTRA_F5
}

// String (generated function)
func (m *SerialUdbExtraF5) String() string {
	return fmt.Sprintf(
		"&matrixpilot.SerialUdbExtraF5{ SueYawkpAileron: %+v, SueYawkdAileron: %+v, SueRollkp: %+v, SueRollkd: %+v }",
		m.SueYawkpAileron,
		m.SueYawkdAileron,
		m.SueRollkp,
		m.SueRollkd,
	)
}

// Pack (generated function)
func (m *SerialUdbExtraF5) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 16)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(m.SueYawkpAileron))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(m.SueYawkdAileron))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.SueRollkp))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.SueRollkd))
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *SerialUdbExtraF5) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 16 {
		return mavlink.ErrPayloadTooSmall
	}
	m.SueYawkpAileron = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.SueYawkdAileron = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	m.SueRollkp = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.SueRollkd = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	return nil
}

// SerialUdbExtraF6 struct (generated typeinfo)
// Backwards compatible version of SERIAL_UDB_EXTRA F6: format
type SerialUdbExtraF6 struct {
	SuePitchgain     float32 // Serial UDB Extra PITCHGAIN Proportional Control
	SuePitchkd       float32 // Serial UDB Extra Pitch Rate Control
	SueRudderElevMix float32 // Serial UDB Extra Rudder to Elevator Mix
	SueRollElevMix   float32 // Serial UDB Extra Roll to Elevator Mix
	SueElevatorBoost float32 // Gain For Boosting Manual Elevator control When Plane Stabilized
}

// MsgID (generated function)
func (m *SerialUdbExtraF6) MsgID() mavlink.MessageID {
	return MSG_ID_SERIAL_UDB_EXTRA_F6
}

// String (generated function)
func (m *SerialUdbExtraF6) String() string {
	return fmt.Sprintf(
		"&matrixpilot.SerialUdbExtraF6{ SuePitchgain: %+v, SuePitchkd: %+v, SueRudderElevMix: %+v, SueRollElevMix: %+v, SueElevatorBoost: %+v }",
		m.SuePitchgain,
		m.SuePitchkd,
		m.SueRudderElevMix,
		m.SueRollElevMix,
		m.SueElevatorBoost,
	)
}

// Pack (generated function)
func (m *SerialUdbExtraF6) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 20)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(m.SuePitchgain))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(m.SuePitchkd))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.SueRudderElevMix))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.SueRollElevMix))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(m.SueElevatorBoost))
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *SerialUdbExtraF6) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 20 {
		return mavlink.ErrPayloadTooSmall
	}
	m.SuePitchgain = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.SuePitchkd = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	m.SueRudderElevMix = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.SueRollElevMix = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.SueElevatorBoost = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	return nil
}

// SerialUdbExtraF7 struct (generated typeinfo)
// Backwards compatible version of SERIAL_UDB_EXTRA F7: format
type SerialUdbExtraF7 struct {
	SueYawkpRudder  float32 // Serial UDB YAWKP_RUDDER Gain for Proporional control of navigation
	SueYawkdRudder  float32 // Serial UDB YAWKD_RUDDER Gain for Rate control of navigation
	SueRollkpRudder float32 // Serial UDB Extra ROLLKP_RUDDER Gain for Proportional control of roll stabilization
	SueRollkdRudder float32 // Serial UDB Extra ROLLKD_RUDDER Gain for Rate control of roll stabilization
	SueRudderBoost  float32 // SERIAL UDB EXTRA Rudder Boost Gain to Manual Control when stabilized
	SueRtlPitchDown float32 // Serial UDB Extra Return To Landing - Angle to Pitch Plane Down
}

// MsgID (generated function)
func (m *SerialUdbExtraF7) MsgID() mavlink.MessageID {
	return MSG_ID_SERIAL_UDB_EXTRA_F7
}

// String (generated function)
func (m *SerialUdbExtraF7) String() string {
	return fmt.Sprintf(
		"&matrixpilot.SerialUdbExtraF7{ SueYawkpRudder: %+v, SueYawkdRudder: %+v, SueRollkpRudder: %+v, SueRollkdRudder: %+v, SueRudderBoost: %+v, SueRtlPitchDown: %+v }",
		m.SueYawkpRudder,
		m.SueYawkdRudder,
		m.SueRollkpRudder,
		m.SueRollkdRudder,
		m.SueRudderBoost,
		m.SueRtlPitchDown,
	)
}

// Pack (generated function)
func (m *SerialUdbExtraF7) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 24)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(m.SueYawkpRudder))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(m.SueYawkdRudder))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.SueRollkpRudder))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.SueRollkdRudder))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(m.SueRudderBoost))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(m.SueRtlPitchDown))
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *SerialUdbExtraF7) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 24 {
		return mavlink.ErrPayloadTooSmall
	}
	m.SueYawkpRudder = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.SueYawkdRudder = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	m.SueRollkpRudder = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.SueRollkdRudder = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.SueRudderBoost = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	m.SueRtlPitchDown = math.Float32frombits(binary.LittleEndian.Uint32(payload[20:]))
	return nil
}

// SerialUdbExtraF8 struct (generated typeinfo)
// Backwards compatible version of SERIAL_UDB_EXTRA F8: format
type SerialUdbExtraF8 struct {
	SueHeightTargetMax    float32 // Serial UDB Extra HEIGHT_TARGET_MAX
	SueHeightTargetMin    float32 // Serial UDB Extra HEIGHT_TARGET_MIN
	SueAltHoldThrottleMin float32 // Serial UDB Extra ALT_HOLD_THROTTLE_MIN
	SueAltHoldThrottleMax float32 // Serial UDB Extra ALT_HOLD_THROTTLE_MAX
	SueAltHoldPitchMin    float32 // Serial UDB Extra ALT_HOLD_PITCH_MIN
	SueAltHoldPitchMax    float32 // Serial UDB Extra ALT_HOLD_PITCH_MAX
	SueAltHoldPitchHigh   float32 // Serial UDB Extra ALT_HOLD_PITCH_HIGH
}

// MsgID (generated function)
func (m *SerialUdbExtraF8) MsgID() mavlink.MessageID {
	return MSG_ID_SERIAL_UDB_EXTRA_F8
}

// String (generated function)
func (m *SerialUdbExtraF8) String() string {
	return fmt.Sprintf(
		"&matrixpilot.SerialUdbExtraF8{ SueHeightTargetMax: %+v, SueHeightTargetMin: %+v, SueAltHoldThrottleMin: %+v, SueAltHoldThrottleMax: %+v, SueAltHoldPitchMin: %+v, SueAltHoldPitchMax: %+v, SueAltHoldPitchHigh: %+v }",
		m.SueHeightTargetMax,
		m.SueHeightTargetMin,
		m.SueAltHoldThrottleMin,
		m.SueAltHoldThrottleMax,
		m.SueAltHoldPitchMin,
		m.SueAltHoldPitchMax,
		m.SueAltHoldPitchHigh,
	)
}

// Pack (generated function)
func (m *SerialUdbExtraF8) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 28)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(m.SueHeightTargetMax))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(m.SueHeightTargetMin))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.SueAltHoldThrottleMin))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.SueAltHoldThrottleMax))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(m.SueAltHoldPitchMin))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(m.SueAltHoldPitchMax))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(m.SueAltHoldPitchHigh))
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *SerialUdbExtraF8) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 28 {
		return mavlink.ErrPayloadTooSmall
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

// SerialUdbExtraF13 struct (generated typeinfo)
// Backwards compatible version of SERIAL_UDB_EXTRA F13: format
type SerialUdbExtraF13 struct {
	SueLatOrigin int32 // Serial UDB Extra MP Origin Latitude
	SueLonOrigin int32 // Serial UDB Extra MP Origin Longitude
	SueAltOrigin int32 // Serial UDB Extra MP Origin Altitude Above Sea Level
	SueWeekNo    int16 // Serial UDB Extra GPS Week Number
}

// MsgID (generated function)
func (m *SerialUdbExtraF13) MsgID() mavlink.MessageID {
	return MSG_ID_SERIAL_UDB_EXTRA_F13
}

// String (generated function)
func (m *SerialUdbExtraF13) String() string {
	return fmt.Sprintf(
		"&matrixpilot.SerialUdbExtraF13{ SueLatOrigin: %+v, SueLonOrigin: %+v, SueAltOrigin: %+v, SueWeekNo: %+v }",
		m.SueLatOrigin,
		m.SueLonOrigin,
		m.SueAltOrigin,
		m.SueWeekNo,
	)
}

// Pack (generated function)
func (m *SerialUdbExtraF13) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 14)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.SueLatOrigin))
	binary.LittleEndian.PutUint32(payload[4:], uint32(m.SueLonOrigin))
	binary.LittleEndian.PutUint32(payload[8:], uint32(m.SueAltOrigin))
	binary.LittleEndian.PutUint16(payload[12:], uint16(m.SueWeekNo))
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *SerialUdbExtraF13) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 14 {
		return mavlink.ErrPayloadTooSmall
	}
	m.SueLatOrigin = int32(binary.LittleEndian.Uint32(payload[0:]))
	m.SueLonOrigin = int32(binary.LittleEndian.Uint32(payload[4:]))
	m.SueAltOrigin = int32(binary.LittleEndian.Uint32(payload[8:]))
	m.SueWeekNo = int16(binary.LittleEndian.Uint16(payload[12:]))
	return nil
}

// SerialUdbExtraF14 struct (generated typeinfo)
// Backwards compatible version of SERIAL_UDB_EXTRA F14: format
type SerialUdbExtraF14 struct {
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

// MsgID (generated function)
func (m *SerialUdbExtraF14) MsgID() mavlink.MessageID {
	return MSG_ID_SERIAL_UDB_EXTRA_F14
}

// String (generated function)
func (m *SerialUdbExtraF14) String() string {
	return fmt.Sprintf(
		"&matrixpilot.SerialUdbExtraF14{ SueTrapSource: %+v, SueRcon: %+v, SueTrapFlags: %+v, SueOscFailCount: %+v, SueWindEstimation: %+v, SueGpsType: %+v, SueDr: %+v, SueBoardType: %+v, SueAirframe: %+v, SueClockConfig: %+v, SueFlightPlanType: %+v }",
		m.SueTrapSource,
		m.SueRcon,
		m.SueTrapFlags,
		m.SueOscFailCount,
		m.SueWindEstimation,
		m.SueGpsType,
		m.SueDr,
		m.SueBoardType,
		m.SueAirframe,
		m.SueClockConfig,
		m.SueFlightPlanType,
	)
}

// Pack (generated function)
func (m *SerialUdbExtraF14) Pack(p *mavlink.Packet) error {
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *SerialUdbExtraF14) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 17 {
		return mavlink.ErrPayloadTooSmall
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

// SerialUdbExtraF15 struct (generated typeinfo)
// Backwards compatible version of SERIAL_UDB_EXTRA F15 format
type SerialUdbExtraF15 struct {
	SueIDVehicleModelName    [40]uint8 // Serial UDB Extra Model Name Of Vehicle
	SueIDVehicleRegistration [20]uint8 // Serial UDB Extra Registraton Number of Vehicle
}

// MsgID (generated function)
func (m *SerialUdbExtraF15) MsgID() mavlink.MessageID {
	return MSG_ID_SERIAL_UDB_EXTRA_F15
}

// String (generated function)
func (m *SerialUdbExtraF15) String() string {
	return fmt.Sprintf(
		"&matrixpilot.SerialUdbExtraF15{ SueIDVehicleModelName: %+v, SueIDVehicleRegistration: %+v }",
		m.SueIDVehicleModelName,
		m.SueIDVehicleRegistration,
	)
}

// Pack (generated function)
func (m *SerialUdbExtraF15) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 60)
	copy(payload[0:], m.SueIDVehicleModelName[:])
	copy(payload[40:], m.SueIDVehicleRegistration[:])
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *SerialUdbExtraF15) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 60 {
		return mavlink.ErrPayloadTooSmall
	}
	copy(m.SueIDVehicleModelName[:], payload[0:40])
	copy(m.SueIDVehicleRegistration[:], payload[40:60])
	return nil
}

// SerialUdbExtraF16 struct (generated typeinfo)
// Backwards compatible version of SERIAL_UDB_EXTRA F16 format
type SerialUdbExtraF16 struct {
	SueIDLeadPilot    [40]uint8 // Serial UDB Extra Name of Expected Lead Pilot
	SueIDDiyDronesURL [70]uint8 // Serial UDB Extra URL of Lead Pilot or Team
}

// MsgID (generated function)
func (m *SerialUdbExtraF16) MsgID() mavlink.MessageID {
	return MSG_ID_SERIAL_UDB_EXTRA_F16
}

// String (generated function)
func (m *SerialUdbExtraF16) String() string {
	return fmt.Sprintf(
		"&matrixpilot.SerialUdbExtraF16{ SueIDLeadPilot: %+v, SueIDDiyDronesURL: %+v }",
		m.SueIDLeadPilot,
		m.SueIDDiyDronesURL,
	)
}

// Pack (generated function)
func (m *SerialUdbExtraF16) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 110)
	copy(payload[0:], m.SueIDLeadPilot[:])
	copy(payload[40:], m.SueIDDiyDronesURL[:])
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *SerialUdbExtraF16) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 110 {
		return mavlink.ErrPayloadTooSmall
	}
	copy(m.SueIDLeadPilot[:], payload[0:40])
	copy(m.SueIDDiyDronesURL[:], payload[40:110])
	return nil
}

// Altitudes struct (generated typeinfo)
// The altitude measured by sensors and IMU
type Altitudes struct {
	TimeBootMs     uint32 // Timestamp (milliseconds since system boot)
	AltGps         int32  // GPS altitude (MSL) in meters, expressed as * 1000 (millimeters)
	AltImu         int32  // IMU altitude above ground in meters, expressed as * 1000 (millimeters)
	AltBarometric  int32  // barometeric altitude above ground in meters, expressed as * 1000 (millimeters)
	AltOpticalFlow int32  // Optical flow altitude above ground in meters, expressed as * 1000 (millimeters)
	AltRangeFinder int32  // Rangefinder Altitude above ground in meters, expressed as * 1000 (millimeters)
	AltExtra       int32  // Extra altitude above ground in meters, expressed as * 1000 (millimeters)
}

// MsgID (generated function)
func (m *Altitudes) MsgID() mavlink.MessageID {
	return MSG_ID_ALTITUDES
}

// String (generated function)
func (m *Altitudes) String() string {
	return fmt.Sprintf(
		"&matrixpilot.Altitudes{ TimeBootMs: %+v, AltGps: %+v, AltImu: %+v, AltBarometric: %+v, AltOpticalFlow: %+v, AltRangeFinder: %+v, AltExtra: %+v }",
		m.TimeBootMs,
		m.AltGps,
		m.AltImu,
		m.AltBarometric,
		m.AltOpticalFlow,
		m.AltRangeFinder,
		m.AltExtra,
	)
}

// Pack (generated function)
func (m *Altitudes) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 28)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.TimeBootMs))
	binary.LittleEndian.PutUint32(payload[4:], uint32(m.AltGps))
	binary.LittleEndian.PutUint32(payload[8:], uint32(m.AltImu))
	binary.LittleEndian.PutUint32(payload[12:], uint32(m.AltBarometric))
	binary.LittleEndian.PutUint32(payload[16:], uint32(m.AltOpticalFlow))
	binary.LittleEndian.PutUint32(payload[20:], uint32(m.AltRangeFinder))
	binary.LittleEndian.PutUint32(payload[24:], uint32(m.AltExtra))
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *Altitudes) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 28 {
		return mavlink.ErrPayloadTooSmall
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

// Airspeeds struct (generated typeinfo)
// The airspeed measured by sensors and IMU
type Airspeeds struct {
	TimeBootMs         uint32 // Timestamp (milliseconds since system boot)
	AirspeedImu        int16  // Airspeed estimate from IMU, cm/s
	AirspeedPitot      int16  // Pitot measured forward airpseed, cm/s
	AirspeedHotWire    int16  // Hot wire anenometer measured airspeed, cm/s
	AirspeedUltrasonic int16  // Ultrasonic measured airspeed, cm/s
	Aoa                int16  // Angle of attack sensor, degrees * 10
	Aoy                int16  // Yaw angle sensor, degrees * 10
}

// MsgID (generated function)
func (m *Airspeeds) MsgID() mavlink.MessageID {
	return MSG_ID_AIRSPEEDS
}

// String (generated function)
func (m *Airspeeds) String() string {
	return fmt.Sprintf(
		"&matrixpilot.Airspeeds{ TimeBootMs: %+v, AirspeedImu: %+v, AirspeedPitot: %+v, AirspeedHotWire: %+v, AirspeedUltrasonic: %+v, Aoa: %+v, Aoy: %+v }",
		m.TimeBootMs,
		m.AirspeedImu,
		m.AirspeedPitot,
		m.AirspeedHotWire,
		m.AirspeedUltrasonic,
		m.Aoa,
		m.Aoy,
	)
}

// Pack (generated function)
func (m *Airspeeds) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 16)
	binary.LittleEndian.PutUint32(payload[0:], uint32(m.TimeBootMs))
	binary.LittleEndian.PutUint16(payload[4:], uint16(m.AirspeedImu))
	binary.LittleEndian.PutUint16(payload[6:], uint16(m.AirspeedPitot))
	binary.LittleEndian.PutUint16(payload[8:], uint16(m.AirspeedHotWire))
	binary.LittleEndian.PutUint16(payload[10:], uint16(m.AirspeedUltrasonic))
	binary.LittleEndian.PutUint16(payload[12:], uint16(m.Aoa))
	binary.LittleEndian.PutUint16(payload[14:], uint16(m.Aoy))
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *Airspeeds) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 16 {
		return mavlink.ErrPayloadTooSmall
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

// SerialUdbExtraF17 struct (generated typeinfo)
// Backwards compatible version of SERIAL_UDB_EXTRA F17 format
type SerialUdbExtraF17 struct {
	SueFeedForward float32 // SUE Feed Forward Gain
	SueTurnRateNav float32 // SUE Max Turn Rate when Navigating
	SueTurnRateFbw float32 // SUE Max Turn Rate in Fly By Wire Mode
}

// MsgID (generated function)
func (m *SerialUdbExtraF17) MsgID() mavlink.MessageID {
	return MSG_ID_SERIAL_UDB_EXTRA_F17
}

// String (generated function)
func (m *SerialUdbExtraF17) String() string {
	return fmt.Sprintf(
		"&matrixpilot.SerialUdbExtraF17{ SueFeedForward: %+v, SueTurnRateNav: %+v, SueTurnRateFbw: %+v }",
		m.SueFeedForward,
		m.SueTurnRateNav,
		m.SueTurnRateFbw,
	)
}

// Pack (generated function)
func (m *SerialUdbExtraF17) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 12)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(m.SueFeedForward))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(m.SueTurnRateNav))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.SueTurnRateFbw))
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *SerialUdbExtraF17) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 12 {
		return mavlink.ErrPayloadTooSmall
	}
	m.SueFeedForward = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.SueTurnRateNav = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	m.SueTurnRateFbw = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	return nil
}

// SerialUdbExtraF18 struct (generated typeinfo)
// Backwards compatible version of SERIAL_UDB_EXTRA F18 format
type SerialUdbExtraF18 struct {
	AngleOfAttackNormal   float32 // SUE Angle of Attack Normal
	AngleOfAttackInverted float32 // SUE Angle of Attack Inverted
	ElevatorTrimNormal    float32 // SUE Elevator Trim Normal
	ElevatorTrimInverted  float32 // SUE Elevator Trim Inverted
	ReferenceSpeed        float32 // SUE reference_speed
}

// MsgID (generated function)
func (m *SerialUdbExtraF18) MsgID() mavlink.MessageID {
	return MSG_ID_SERIAL_UDB_EXTRA_F18
}

// String (generated function)
func (m *SerialUdbExtraF18) String() string {
	return fmt.Sprintf(
		"&matrixpilot.SerialUdbExtraF18{ AngleOfAttackNormal: %+v, AngleOfAttackInverted: %+v, ElevatorTrimNormal: %+v, ElevatorTrimInverted: %+v, ReferenceSpeed: %+v }",
		m.AngleOfAttackNormal,
		m.AngleOfAttackInverted,
		m.ElevatorTrimNormal,
		m.ElevatorTrimInverted,
		m.ReferenceSpeed,
	)
}

// Pack (generated function)
func (m *SerialUdbExtraF18) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 20)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(m.AngleOfAttackNormal))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(m.AngleOfAttackInverted))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(m.ElevatorTrimNormal))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(m.ElevatorTrimInverted))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(m.ReferenceSpeed))
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *SerialUdbExtraF18) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 20 {
		return mavlink.ErrPayloadTooSmall
	}
	m.AngleOfAttackNormal = math.Float32frombits(binary.LittleEndian.Uint32(payload[0:]))
	m.AngleOfAttackInverted = math.Float32frombits(binary.LittleEndian.Uint32(payload[4:]))
	m.ElevatorTrimNormal = math.Float32frombits(binary.LittleEndian.Uint32(payload[8:]))
	m.ElevatorTrimInverted = math.Float32frombits(binary.LittleEndian.Uint32(payload[12:]))
	m.ReferenceSpeed = math.Float32frombits(binary.LittleEndian.Uint32(payload[16:]))
	return nil
}

// SerialUdbExtraF19 struct (generated typeinfo)
// Backwards compatible version of SERIAL_UDB_EXTRA F19 format
type SerialUdbExtraF19 struct {
	SueAileronOutputChannel  uint8 // SUE aileron output channel
	SueAileronReversed       uint8 // SUE aileron reversed
	SueElevatorOutputChannel uint8 // SUE elevator output channel
	SueElevatorReversed      uint8 // SUE elevator reversed
	SueThrottleOutputChannel uint8 // SUE throttle output channel
	SueThrottleReversed      uint8 // SUE throttle reversed
	SueRudderOutputChannel   uint8 // SUE rudder output channel
	SueRudderReversed        uint8 // SUE rudder reversed
}

// MsgID (generated function)
func (m *SerialUdbExtraF19) MsgID() mavlink.MessageID {
	return MSG_ID_SERIAL_UDB_EXTRA_F19
}

// String (generated function)
func (m *SerialUdbExtraF19) String() string {
	return fmt.Sprintf(
		"&matrixpilot.SerialUdbExtraF19{ SueAileronOutputChannel: %+v, SueAileronReversed: %+v, SueElevatorOutputChannel: %+v, SueElevatorReversed: %+v, SueThrottleOutputChannel: %+v, SueThrottleReversed: %+v, SueRudderOutputChannel: %+v, SueRudderReversed: %+v }",
		m.SueAileronOutputChannel,
		m.SueAileronReversed,
		m.SueElevatorOutputChannel,
		m.SueElevatorReversed,
		m.SueThrottleOutputChannel,
		m.SueThrottleReversed,
		m.SueRudderOutputChannel,
		m.SueRudderReversed,
	)
}

// Pack (generated function)
func (m *SerialUdbExtraF19) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 8)
	payload[0] = byte(m.SueAileronOutputChannel)
	payload[1] = byte(m.SueAileronReversed)
	payload[2] = byte(m.SueElevatorOutputChannel)
	payload[3] = byte(m.SueElevatorReversed)
	payload[4] = byte(m.SueThrottleOutputChannel)
	payload[5] = byte(m.SueThrottleReversed)
	payload[6] = byte(m.SueRudderOutputChannel)
	payload[7] = byte(m.SueRudderReversed)
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *SerialUdbExtraF19) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 8 {
		return mavlink.ErrPayloadTooSmall
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

// SerialUdbExtraF20 struct (generated typeinfo)
// Backwards compatible version of SERIAL_UDB_EXTRA F20 format
type SerialUdbExtraF20 struct {
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

// MsgID (generated function)
func (m *SerialUdbExtraF20) MsgID() mavlink.MessageID {
	return MSG_ID_SERIAL_UDB_EXTRA_F20
}

// String (generated function)
func (m *SerialUdbExtraF20) String() string {
	return fmt.Sprintf(
		"&matrixpilot.SerialUdbExtraF20{ SueTrimValueInput1: %+v, SueTrimValueInput2: %+v, SueTrimValueInput3: %+v, SueTrimValueInput4: %+v, SueTrimValueInput5: %+v, SueTrimValueInput6: %+v, SueTrimValueInput7: %+v, SueTrimValueInput8: %+v, SueTrimValueInput9: %+v, SueTrimValueInput10: %+v, SueTrimValueInput11: %+v, SueTrimValueInput12: %+v, SueNumberOfInputs: %+v }",
		m.SueTrimValueInput1,
		m.SueTrimValueInput2,
		m.SueTrimValueInput3,
		m.SueTrimValueInput4,
		m.SueTrimValueInput5,
		m.SueTrimValueInput6,
		m.SueTrimValueInput7,
		m.SueTrimValueInput8,
		m.SueTrimValueInput9,
		m.SueTrimValueInput10,
		m.SueTrimValueInput11,
		m.SueTrimValueInput12,
		m.SueNumberOfInputs,
	)
}

// Pack (generated function)
func (m *SerialUdbExtraF20) Pack(p *mavlink.Packet) error {
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
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *SerialUdbExtraF20) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 25 {
		return mavlink.ErrPayloadTooSmall
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

// SerialUdbExtraF21 struct (generated typeinfo)
// Backwards compatible version of SERIAL_UDB_EXTRA F21 format
type SerialUdbExtraF21 struct {
	SueAccelXOffset int16 // SUE X accelerometer offset
	SueAccelYOffset int16 // SUE Y accelerometer offset
	SueAccelZOffset int16 // SUE Z accelerometer offset
	SueGyroXOffset  int16 // SUE X gyro offset
	SueGyroYOffset  int16 // SUE Y gyro offset
	SueGyroZOffset  int16 // SUE Z gyro offset
}

// MsgID (generated function)
func (m *SerialUdbExtraF21) MsgID() mavlink.MessageID {
	return MSG_ID_SERIAL_UDB_EXTRA_F21
}

// String (generated function)
func (m *SerialUdbExtraF21) String() string {
	return fmt.Sprintf(
		"&matrixpilot.SerialUdbExtraF21{ SueAccelXOffset: %+v, SueAccelYOffset: %+v, SueAccelZOffset: %+v, SueGyroXOffset: %+v, SueGyroYOffset: %+v, SueGyroZOffset: %+v }",
		m.SueAccelXOffset,
		m.SueAccelYOffset,
		m.SueAccelZOffset,
		m.SueGyroXOffset,
		m.SueGyroYOffset,
		m.SueGyroZOffset,
	)
}

// Pack (generated function)
func (m *SerialUdbExtraF21) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 12)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.SueAccelXOffset))
	binary.LittleEndian.PutUint16(payload[2:], uint16(m.SueAccelYOffset))
	binary.LittleEndian.PutUint16(payload[4:], uint16(m.SueAccelZOffset))
	binary.LittleEndian.PutUint16(payload[6:], uint16(m.SueGyroXOffset))
	binary.LittleEndian.PutUint16(payload[8:], uint16(m.SueGyroYOffset))
	binary.LittleEndian.PutUint16(payload[10:], uint16(m.SueGyroZOffset))
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *SerialUdbExtraF21) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 12 {
		return mavlink.ErrPayloadTooSmall
	}
	m.SueAccelXOffset = int16(binary.LittleEndian.Uint16(payload[0:]))
	m.SueAccelYOffset = int16(binary.LittleEndian.Uint16(payload[2:]))
	m.SueAccelZOffset = int16(binary.LittleEndian.Uint16(payload[4:]))
	m.SueGyroXOffset = int16(binary.LittleEndian.Uint16(payload[6:]))
	m.SueGyroYOffset = int16(binary.LittleEndian.Uint16(payload[8:]))
	m.SueGyroZOffset = int16(binary.LittleEndian.Uint16(payload[10:]))
	return nil
}

// SerialUdbExtraF22 struct (generated typeinfo)
// Backwards compatible version of SERIAL_UDB_EXTRA F22 format
type SerialUdbExtraF22 struct {
	SueAccelXAtCalibration int16 // SUE X accelerometer at calibration time
	SueAccelYAtCalibration int16 // SUE Y accelerometer at calibration time
	SueAccelZAtCalibration int16 // SUE Z accelerometer at calibration time
	SueGyroXAtCalibration  int16 // SUE X gyro at calibration time
	SueGyroYAtCalibration  int16 // SUE Y gyro at calibration time
	SueGyroZAtCalibration  int16 // SUE Z gyro at calibration time
}

// MsgID (generated function)
func (m *SerialUdbExtraF22) MsgID() mavlink.MessageID {
	return MSG_ID_SERIAL_UDB_EXTRA_F22
}

// String (generated function)
func (m *SerialUdbExtraF22) String() string {
	return fmt.Sprintf(
		"&matrixpilot.SerialUdbExtraF22{ SueAccelXAtCalibration: %+v, SueAccelYAtCalibration: %+v, SueAccelZAtCalibration: %+v, SueGyroXAtCalibration: %+v, SueGyroYAtCalibration: %+v, SueGyroZAtCalibration: %+v }",
		m.SueAccelXAtCalibration,
		m.SueAccelYAtCalibration,
		m.SueAccelZAtCalibration,
		m.SueGyroXAtCalibration,
		m.SueGyroYAtCalibration,
		m.SueGyroZAtCalibration,
	)
}

// Pack (generated function)
func (m *SerialUdbExtraF22) Pack(p *mavlink.Packet) error {
	payload := make([]byte, 12)
	binary.LittleEndian.PutUint16(payload[0:], uint16(m.SueAccelXAtCalibration))
	binary.LittleEndian.PutUint16(payload[2:], uint16(m.SueAccelYAtCalibration))
	binary.LittleEndian.PutUint16(payload[4:], uint16(m.SueAccelZAtCalibration))
	binary.LittleEndian.PutUint16(payload[6:], uint16(m.SueGyroXAtCalibration))
	binary.LittleEndian.PutUint16(payload[8:], uint16(m.SueGyroYAtCalibration))
	binary.LittleEndian.PutUint16(payload[10:], uint16(m.SueGyroZAtCalibration))
	p.MsgID = m.MsgID()
	p.Payload = payload
	return nil
}

// Unpack (generated function)
func (m *SerialUdbExtraF22) Unpack(p *mavlink.Packet) error {
	payload := p.Payload[:]
	if len(p.Payload) < 12 {
		return mavlink.ErrPayloadTooSmall
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
	MSG_ID_FLEXIFUNCTION_SET                 mavlink.MessageID = 150
	MSG_ID_FLEXIFUNCTION_READ_REQ            mavlink.MessageID = 151
	MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION     mavlink.MessageID = 152
	MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_ACK mavlink.MessageID = 153
	MSG_ID_FLEXIFUNCTION_DIRECTORY           mavlink.MessageID = 155
	MSG_ID_FLEXIFUNCTION_DIRECTORY_ACK       mavlink.MessageID = 156
	MSG_ID_FLEXIFUNCTION_COMMAND             mavlink.MessageID = 157
	MSG_ID_FLEXIFUNCTION_COMMAND_ACK         mavlink.MessageID = 158
	MSG_ID_SERIAL_UDB_EXTRA_F2_A             mavlink.MessageID = 170
	MSG_ID_SERIAL_UDB_EXTRA_F2_B             mavlink.MessageID = 171
	MSG_ID_SERIAL_UDB_EXTRA_F4               mavlink.MessageID = 172
	MSG_ID_SERIAL_UDB_EXTRA_F5               mavlink.MessageID = 173
	MSG_ID_SERIAL_UDB_EXTRA_F6               mavlink.MessageID = 174
	MSG_ID_SERIAL_UDB_EXTRA_F7               mavlink.MessageID = 175
	MSG_ID_SERIAL_UDB_EXTRA_F8               mavlink.MessageID = 176
	MSG_ID_SERIAL_UDB_EXTRA_F13              mavlink.MessageID = 177
	MSG_ID_SERIAL_UDB_EXTRA_F14              mavlink.MessageID = 178
	MSG_ID_SERIAL_UDB_EXTRA_F15              mavlink.MessageID = 179
	MSG_ID_SERIAL_UDB_EXTRA_F16              mavlink.MessageID = 180
	MSG_ID_ALTITUDES                         mavlink.MessageID = 181
	MSG_ID_AIRSPEEDS                         mavlink.MessageID = 182
	MSG_ID_SERIAL_UDB_EXTRA_F17              mavlink.MessageID = 183
	MSG_ID_SERIAL_UDB_EXTRA_F18              mavlink.MessageID = 184
	MSG_ID_SERIAL_UDB_EXTRA_F19              mavlink.MessageID = 185
	MSG_ID_SERIAL_UDB_EXTRA_F20              mavlink.MessageID = 186
	MSG_ID_SERIAL_UDB_EXTRA_F21              mavlink.MessageID = 187
	MSG_ID_SERIAL_UDB_EXTRA_F22              mavlink.MessageID = 188
)

// init Matrixpilot dialect
func init() {
	mavlink.Register(MSG_ID_FLEXIFUNCTION_SET, "MSG_ID_FLEXIFUNCTION_SET", 181, func(p *mavlink.Packet) mavlink.Message {
		msg := new(FlexifunctionSet)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_FLEXIFUNCTION_READ_REQ, "MSG_ID_FLEXIFUNCTION_READ_REQ", 26, func(p *mavlink.Packet) mavlink.Message {
		msg := new(FlexifunctionReadReq)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION, "MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION", 101, func(p *mavlink.Packet) mavlink.Message {
		msg := new(FlexifunctionBufferFunction)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_ACK, "MSG_ID_FLEXIFUNCTION_BUFFER_FUNCTION_ACK", 109, func(p *mavlink.Packet) mavlink.Message {
		msg := new(FlexifunctionBufferFunctionAck)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_FLEXIFUNCTION_DIRECTORY, "MSG_ID_FLEXIFUNCTION_DIRECTORY", 12, func(p *mavlink.Packet) mavlink.Message {
		msg := new(FlexifunctionDirectory)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_FLEXIFUNCTION_DIRECTORY_ACK, "MSG_ID_FLEXIFUNCTION_DIRECTORY_ACK", 218, func(p *mavlink.Packet) mavlink.Message {
		msg := new(FlexifunctionDirectoryAck)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_FLEXIFUNCTION_COMMAND, "MSG_ID_FLEXIFUNCTION_COMMAND", 133, func(p *mavlink.Packet) mavlink.Message {
		msg := new(FlexifunctionCommand)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_FLEXIFUNCTION_COMMAND_ACK, "MSG_ID_FLEXIFUNCTION_COMMAND_ACK", 208, func(p *mavlink.Packet) mavlink.Message {
		msg := new(FlexifunctionCommandAck)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_SERIAL_UDB_EXTRA_F2_A, "MSG_ID_SERIAL_UDB_EXTRA_F2_A", 103, func(p *mavlink.Packet) mavlink.Message {
		msg := new(SerialUdbExtraF2A)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_SERIAL_UDB_EXTRA_F2_B, "MSG_ID_SERIAL_UDB_EXTRA_F2_B", 245, func(p *mavlink.Packet) mavlink.Message {
		msg := new(SerialUdbExtraF2B)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_SERIAL_UDB_EXTRA_F4, "MSG_ID_SERIAL_UDB_EXTRA_F4", 191, func(p *mavlink.Packet) mavlink.Message {
		msg := new(SerialUdbExtraF4)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_SERIAL_UDB_EXTRA_F5, "MSG_ID_SERIAL_UDB_EXTRA_F5", 54, func(p *mavlink.Packet) mavlink.Message {
		msg := new(SerialUdbExtraF5)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_SERIAL_UDB_EXTRA_F6, "MSG_ID_SERIAL_UDB_EXTRA_F6", 54, func(p *mavlink.Packet) mavlink.Message {
		msg := new(SerialUdbExtraF6)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_SERIAL_UDB_EXTRA_F7, "MSG_ID_SERIAL_UDB_EXTRA_F7", 171, func(p *mavlink.Packet) mavlink.Message {
		msg := new(SerialUdbExtraF7)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_SERIAL_UDB_EXTRA_F8, "MSG_ID_SERIAL_UDB_EXTRA_F8", 142, func(p *mavlink.Packet) mavlink.Message {
		msg := new(SerialUdbExtraF8)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_SERIAL_UDB_EXTRA_F13, "MSG_ID_SERIAL_UDB_EXTRA_F13", 249, func(p *mavlink.Packet) mavlink.Message {
		msg := new(SerialUdbExtraF13)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_SERIAL_UDB_EXTRA_F14, "MSG_ID_SERIAL_UDB_EXTRA_F14", 123, func(p *mavlink.Packet) mavlink.Message {
		msg := new(SerialUdbExtraF14)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_SERIAL_UDB_EXTRA_F15, "MSG_ID_SERIAL_UDB_EXTRA_F15", 7, func(p *mavlink.Packet) mavlink.Message {
		msg := new(SerialUdbExtraF15)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_SERIAL_UDB_EXTRA_F16, "MSG_ID_SERIAL_UDB_EXTRA_F16", 222, func(p *mavlink.Packet) mavlink.Message {
		msg := new(SerialUdbExtraF16)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_ALTITUDES, "MSG_ID_ALTITUDES", 55, func(p *mavlink.Packet) mavlink.Message {
		msg := new(Altitudes)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_AIRSPEEDS, "MSG_ID_AIRSPEEDS", 154, func(p *mavlink.Packet) mavlink.Message {
		msg := new(Airspeeds)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_SERIAL_UDB_EXTRA_F17, "MSG_ID_SERIAL_UDB_EXTRA_F17", 175, func(p *mavlink.Packet) mavlink.Message {
		msg := new(SerialUdbExtraF17)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_SERIAL_UDB_EXTRA_F18, "MSG_ID_SERIAL_UDB_EXTRA_F18", 41, func(p *mavlink.Packet) mavlink.Message {
		msg := new(SerialUdbExtraF18)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_SERIAL_UDB_EXTRA_F19, "MSG_ID_SERIAL_UDB_EXTRA_F19", 87, func(p *mavlink.Packet) mavlink.Message {
		msg := new(SerialUdbExtraF19)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_SERIAL_UDB_EXTRA_F20, "MSG_ID_SERIAL_UDB_EXTRA_F20", 144, func(p *mavlink.Packet) mavlink.Message {
		msg := new(SerialUdbExtraF20)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_SERIAL_UDB_EXTRA_F21, "MSG_ID_SERIAL_UDB_EXTRA_F21", 134, func(p *mavlink.Packet) mavlink.Message {
		msg := new(SerialUdbExtraF21)
		msg.Unpack(p)
		return msg
	})
	mavlink.Register(MSG_ID_SERIAL_UDB_EXTRA_F22, "MSG_ID_SERIAL_UDB_EXTRA_F22", 91, func(p *mavlink.Packet) mavlink.Message {
		msg := new(SerialUdbExtraF22)
		msg.Unpack(p)
		return msg
	})
}

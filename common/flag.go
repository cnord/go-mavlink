package common

import "flag"

var (
	// Mavlink1 common flag
	Mavlink1 = flag.Bool("1", false, "mavlink v1")
	// Mavlink2 common flag
	Mavlink2 = flag.Bool("2", false, "mavlink v2")
)

// Mavlink versions
const (
	MAVLINK_V1 = 1 // v1
	MAVLINK_V2 = 2 // v2
)

// MavlinkVersion process user flags and return mavlink version
func MavlinkVersion() int {
	return mavlinkVersion(*Mavlink1, *Mavlink2)
}

func mavlinkVersion(mavlink1 bool, mavlink2 bool) int {
	mavlink := MAVLINK_V1 | MAVLINK_V2
	if !mavlink1 && mavlink2 {
		mavlink &= ^MAVLINK_V1
	}
	if !mavlink2 && mavlink1 {
		mavlink &= ^MAVLINK_V2
	}
	return mavlink
}

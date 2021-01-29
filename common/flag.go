package common

import "flag"

var (
	// Mavlink1 common flag
	Mavlink1 = flag.Bool("1", false, "mavlink v1")
	// Mavlink2 common flag
	Mavlink2 = flag.Bool("2", false, "mavlink v2")
)

// MavlinkVersion process user flags and return mavlink version
func MavlinkVersion() int {
	if *Mavlink1 {
		return 1
	}
	if *Mavlink2 {
		return 2
	}
	return -1
}

package common

import "flag"

var (
	Mavlink1 = flag.Bool("1", false, "mavlink v1")
	Mavlink2 = flag.Bool("2", false, "mavlink v2")
)

func MavlinkVersion() int {
	if *Mavlink1 {
		return 1
	}
	if *Mavlink2 {
		return 2
	}
	return -1
}

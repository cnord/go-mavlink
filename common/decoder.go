package common

import (
	mavlink1 "../generated/mavlink1"
	_ "../generated/mavlink1/ardupilotmega"
	_ "../generated/mavlink1/common"
	_ "../generated/mavlink1/icarous"
	_ "../generated/mavlink1/minimal"
	_ "../generated/mavlink1/uAvionix"
	mavlink2 "../generated/mavlink2"
	_ "../generated/mavlink2/ardupilotmega"
	_ "../generated/mavlink2/common"
	_ "../generated/mavlink2/icarous"
	_ "../generated/mavlink2/minimal"
	_ "../generated/mavlink2/uAvionix"
	"io"
	"log"
)

// Decoder struct provide top-level Decoder API over mavlink1/mavlink2 protocols
type Decoder interface {
	Decode() (interface{}, error)
}

type mavlink1Decoder struct {
	dec *mavlink1.Decoder
}

// Decode trying to return decoded message from input stream
func (d *mavlink1Decoder) Decode() (interface{}, error) {
	var packet mavlink1.Packet
	if err := d.dec.Decode(&packet); err != nil {
		return nil, err
	}
	return packet.Message()
}

type mavlink2Decoder struct {
	dec *mavlink2.Decoder
}

// Decode trying to return decoded message from input stream
func (d *mavlink2Decoder) Decode() (interface{}, error) {
	var packet mavlink2.Packet
	if err := d.dec.Decode(&packet); err != nil {
		return nil, err
	}
	return packet.Message()
}

// NewDecoder return decoder depended from mavlink version
func NewDecoder(reader io.Reader) Decoder {
	switch MavlinkVersion() {
	case 1:
		return &mavlink1Decoder{
			dec: mavlink1.NewDecoder(reader),
		}
	case 2:
		return &mavlink2Decoder{
			dec: mavlink2.NewDecoder(reader),
		}
	default:
		log.Fatalf("undefined version (%d) of mavlink decoder\n", MavlinkVersion())
		return nil
	}
}

package mavlinkSwitcher

import (
	mavlink1 "../mavlink1"
	mavlink2 "../mavlink2"
	"io"
	"log"
)

type Decoder interface {
	Decode() (interface{}, error)
}

type mavlink1Decoder struct {
	dec *mavlink1.Decoder
}

func (d *mavlink1Decoder) Decode() (interface{}, error) {
	var packet mavlink1.Packet
	return &packet, d.dec.Decode(&packet)
}

type mavlink2Decoder struct {
	dec *mavlink2.Decoder
}

func (d *mavlink2Decoder) Decode() (interface{}, error) {
	var packet mavlink2.Packet
	return &packet, d.dec.Decode(&packet)
}

func Init(reader io.Reader, version int) Decoder {
	switch version {
	case 1:
		return &mavlink1Decoder{
			dec: mavlink1.NewDecoder(reader),
		}
	case 2:
		return &mavlink2Decoder{
			dec: mavlink2.NewDecoder(reader),
		}
	default:
		log.Fatalf("undefined version (%d) of mavlink decoder\n", version)
		return nil
	}
}

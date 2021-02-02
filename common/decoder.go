package common

import (
	"bufio"
	"fmt"
	mavlink1 "github.com/asmyasnikov/go-mavlink/generated/mavlink1"
	mavlink2 "github.com/asmyasnikov/go-mavlink/generated/mavlink2"
	"github.com/pkg/errors"
	"io"
	"log"
)

// Decoder struct provide top-level Decoder API over mavlink1/mavlink2 protocols
type Decoder interface {
	Decode() (fmt.Stringer, error)
	Name() string
}

type mavlink1Decoder struct {
	dec *mavlink1.Decoder
}

// Decode trying to return decoded message from input stream
func (d *mavlink1Decoder) Decode() (fmt.Stringer, error) {
	var packet mavlink1.Packet
	if err := d.dec.Decode(&packet); err != nil {
		return nil, err
	}
	return &packet, nil
}

func (d *mavlink1Decoder) Name() string {
	return "Mavlink1 decoder"
}

type mavlink2Decoder struct {
	dec *mavlink2.Decoder
}

// Decode trying to return decoded message from input stream
func (d *mavlink2Decoder) Decode() (fmt.Stringer, error) {
	var packet mavlink2.Packet
	if err := d.dec.Decode(&packet); err != nil {
		return nil, err
	}
	return &packet, nil
}

func (d *mavlink2Decoder) Name() string {
	return "Mavlink2 decoder"
}

type Buffer struct {
	in chan byte
}

func (b Buffer) Read(p []byte) (n int, err error) {
	for c := range b.in {
		p[0] = c
		return 1, nil
	}
	return 0, errors.New("EOF")
}

// NewDecoder return decoder depended from mavlink version
func Decoders(reader io.Reader) []Decoder {
	decoders := make([]Decoder, 0)
	buffers := make([]*Buffer, 0)
	mavlinkVersion := MavlinkVersion()
	if mavlinkVersion&MAVLINK_V1 > 0 {
		buffer := &Buffer{
			in: make(chan byte),
		}
		buffers = append(buffers, buffer)
		decoders = append(decoders, &mavlink1Decoder{
			dec: mavlink1.NewDecoder(buffer),
		})
	}
	if mavlinkVersion&MAVLINK_V2 > 0 {
		buffer := &Buffer{
			in: make(chan byte),
		}
		buffers = append(buffers, buffer)
		decoders = append(decoders, &mavlink2Decoder{
			dec: mavlink2.NewDecoder(buffer),
		})
	}
	go func() {
		in := bufio.NewReader(reader)
		for {
			c, err := in.ReadByte()
			if err != nil {
				log.Fatal("Multicast reader error: ", err)
				for _, buffer := range buffers {
					close(buffer.in)
				}
				return
			}
			for i := range buffers {
				buffers[i].in <- c
			}
		}
	}()
	return decoders
}

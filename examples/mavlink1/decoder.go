/*
 * CODE GENERATED AUTOMATICALLY WITH
 *    github.com/asmyasnikov/go-mavlink/mavgen
 * THIS FILE SHOULD NOT BE EDITED BY HAND
 */

package mavlink

import (
	"sort"
	"time"
)

// Decoder struct provide decoding processor
type Decoder struct {
	data    chan []byte
	decoded chan *Packet
}

func (d *Decoder) PushData(data []byte) {
	data = append(data[:0:0], data...)
	d.data <- data
}

func (d *Decoder) NextPacket(duration time.Duration) *Packet {
	select {
	case packet, ok := <-d.decoded:
		if ok {
			return packet
		}
		return nil
	case <-time.After(duration):
		return nil
	}
}

// Stop make safely stop of decoder
func (d *Decoder) Stop() {
	close(d.data)
}

// NewChannelDecoder function create decoder instance with default dialect
func NewChannelDecoder() *Decoder {
	d := &Decoder{
		data:    make(chan []byte, 256),
		decoded: make(chan *Packet, 256),
	}
	go func() {
		var parsers []*Parser
		for {
			buffer, ok := <-d.data
			if !ok {
				close(d.decoded)
				return
			}
			for _, c := range buffer {
				if c == magicNumber {
					parsers = append(parsers, &Parser{})
				}

				indexesToDelete := map[int]bool{}
				for i, parser := range parsers {
					packet, err := parser.parseChar(c)
					if err != nil {
						indexesToDelete[i] = true
						continue
					}
					if packet != nil {
						d.decoded <- packet
						for j := i; j >= 0; j-- {
							indexesToDelete[i] = true
						}
						continue
					}
				}

				if len(indexesToDelete) != 0 {
					var indexes []int
					for index := range indexesToDelete {
						indexes = append(indexes, index)
					}
					sort.Ints(indexes)
					for i := len(indexes) - 1; i >= 0; i-- {
						index := indexes[i]
						copy(parsers[index:], parsers[index+1:])
						parsers[len(parsers)-1] = nil
						parsers = parsers[:len(parsers)-1]
					}
				}
			}
		}
	}()
	return d
}

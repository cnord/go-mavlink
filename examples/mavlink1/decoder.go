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
		uniqueIndexesToDelete := map[int]*Parser{}
		defer func() {
			for index := range uniqueIndexesToDelete {
				parsersPool.Put(parsers[index])
			}
		}()
		var indexesToDelete []int
		for {
			buffer, ok := <-d.data
			if !ok {
				close(d.decoded)
				return
			}
			for _, c := range buffer {
				if c == magicNumber {
					parsers = append(parsers, parsersPool.Get().(*Parser))
				}

				for i, parser := range parsers {
					packet, err := parser.parseChar(c)
					if err != nil {
						uniqueIndexesToDelete[i] = parser
						continue
					}
					if packet != nil {
						d.decoded <- packet
						for j := i; j >= 0; j-- {
							uniqueIndexesToDelete[i] = parser
						}
						continue
					}
				}

				if len(uniqueIndexesToDelete) != 0 {
					for index := range uniqueIndexesToDelete {
						indexesToDelete = append(indexesToDelete, index)
						delete(uniqueIndexesToDelete, index)
					}
					if len(indexesToDelete) > 1 {
						sort.Ints(indexesToDelete)
					}
					for i := len(indexesToDelete) - 1; i >= 0; i-- {
						index := indexesToDelete[i]
						parsersPool.Put(parsers[index])
						copy(parsers[index:], parsers[index+1:])
						parsers[len(parsers)-1] = nil
						parsers = parsers[:len(parsers)-1]
					}
					indexesToDelete = indexesToDelete[:0]
				}
			}
		}
	}()
	return d
}

/*
 * CODE GENERATED AUTOMATICALLY WITH
 *    github.com/asmyasnikov/go-mavlink/mavgen
 * THIS FILE SHOULD NOT BE EDITED BY HAND
 */

package mavlink

import (
	"bufio"
	"fmt"
	"io"
)

// Decoder struct provide decoding processor
type Decoder struct {
	reader  io.ByteReader
	parsers []*Parser
}

func (d *Decoder) clearParser(parser *Parser) {
	parser.Reset()
	parsersPool.Put(parser)
}

func (d *Decoder) clearParsers() {
	for _, parser := range d.parsers {
		d.clearParser(parser)
	}
	d.parsers = d.parsers[:0]
}

// Decode decode input stream to packet. Method return error or nil
func (d *Decoder) Decode(v interface{}) error {
	packet, ok := v.(*Packet)
	if !ok {
		return fmt.Errorf("cast interface '%+v' to Packet fail", v)
	}
	for {
		c, err := d.reader.ReadByte()
		if err != nil {
			return err
		}
		if c == magicNumber {
			d.parsers = append(d.parsers, parsersPool.Get().(*Parser))
		}
		parsers := make([]*Parser, 0, len(d.parsers))
		for _, parser := range d.parsers {
			if p, err := parser.parseChar(c); err != nil {
				d.clearParser(parser)
			} else if p != nil {
				packet.InCompatFlags = p.InCompatFlags
				packet.CompatFlags = p.CompatFlags
				packet.SeqID = p.SeqID
				packet.SysID = p.SysID
				packet.CompID = p.CompID
				packet.MsgID = p.MsgID
				packet.Checksum = p.Checksum
				packet.Payload = append(packet.Payload[:0], p.Payload...)
				d.clearParsers()
				return nil
			} else {
				parsers = append(parsers, parser)
			}
		}
		d.parsers = parsers
	}
}

func byteReader(r io.Reader) io.ByteReader {
	if rb, ok := r.(io.ByteReader); ok {
		return rb
	}
	return bufio.NewReader(r)
}

// NewDecoder function create decoder instance
func NewDecoder(r io.Reader) *Decoder {
	return &Decoder{
		reader:  byteReader(r),
		parsers: make([]*Parser, 0),
	}
}

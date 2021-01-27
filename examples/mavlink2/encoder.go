/*
 * CODE GENERATED AUTOMATICALLY WITH
 *    github.com/asmyasnikov/go-mavlink/mavgen
 * THIS FILE SHOULD NOT BE EDITED BY HAND
 */

/*
 * CODE GENERATED AUTOMATICALLY WITH
 *    github.com/asmyasnikov/go-mavlink/mavgen
 * THIS FILE SHOULD NOT BE EDITED BY HAND
 */

package mavlink

import (
	"fmt"
	"io"
)

// Decoder struct provide decoding processor
type Encoder struct {
	writer io.Writer
}

// Decode decode input stream to packet. Method return error or nil
func (e *Encoder) Encode(v interface{}) error {
	packet, ok := v.(*Packet)
	if !ok {
		return fmt.Errorf("cast interface '%+v' to Packet fail", v)
	}
	b := packet.Bytes()
	n, err := e.writer.Write(b)
	if len(b) != n {
		return fmt.Errorf("writed %d bytes, but need to write %d bytes", n, len(b))
	}
	return err
}

// NewDecoder function create decoder instance with default dialect
func NewEncoder(w io.Writer) *Encoder {
	return &Encoder{
		writer: w,
	}
}

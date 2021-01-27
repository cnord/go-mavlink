/*
 * CODE GENERATED AUTOMATICALLY WITH
 *    github.com/asmyasnikov/go-mavlink/mavgen
 * THIS FILE SHOULD NOT BE EDITED BY HAND
 */

package mavlink

// zeroTail is a cache of zero slice for auto append tail to
// payload in Mavlink2 messages with trimmed payload (variable length)
var (
	zeroTail = make([]byte, 256)
)

// Message is a basic type for encoding/decoding mavlink messages.
// use the Pack() and Unpack() routines on specific message
// types to convert them to/from the Packet type.
type Message interface {
	Dialect() *Dialect
	Pack(*Packet) error
	Unpack(*Packet) error
	MsgID() MessageID
	MsgName() string
	CRCExtra() uint8
}

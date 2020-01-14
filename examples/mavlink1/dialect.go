/*
 * CODE GENERATED AUTOMATICALLY WITH
 *    github.com/asmyasnikov/go-mavlink/mavgen
 * THIS FILE SHOULD NOT BE EDITED BY HAND
 */

package mavlink

// Dialect represents a set of message definitions.
// Some dialects have conflicting definitions for given message IDs,
// so a list of dialects must be provided to an Encoder/Decoder in
// order to specify which packets to use for the conflicting IDs.
//
type Dialect struct {
	Name                      string
	crcExtras                 map[MessageID]uint8
	messageConstructorByMsgID map[MessageID]func(*Packet) Message
}

// GetMessage function produce message by packet
func (d *Dialect) GetMessage(pkt *Packet) (msg Message, ok bool) {
	constructor, ok := d.messageConstructorByMsgID[pkt.MsgID]
	if !ok {
		return nil, false
	}
	return constructor(pkt), true
}

// AddDialect append dialect to internal store
func AddDialect(d *Dialect) {
	dialects.Add(d)
}

// RemoveDialect remove dialect from internal store
func RemoveDialect(d *Dialect) {
	dialects.Remove(d)
}
